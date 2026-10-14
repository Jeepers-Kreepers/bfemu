"""
Транспорты виртуального FC и рабочая сессия.

Поддерживаются:
  * TCP  (--tcp 0.0.0.0:5761)   — Configurator: ручное подключение, порт "tcp://127.0.0.1:5761";
  * PTY  (--pty /tmp/ttyBFEMU)  — Linux/macOS: Configurator видит это как обычный COM/tty;
  * STDIO (--stdio)             — мост для socat / com0com / pipe.

Сессия повторяет поведение прошивки:
  '$'            — запрос MSP (v1/v2), отвечаем тем же протоколом;
  '#'            — вход в интерактивный CLI (эхо, приглашение "\\r\\n# ");
  0x02 (STX)     — вход в неинтерактивный CLI (так работает MSP.send_cli_command),
                   вывод обрамляется STX ... ETX;
  ребут          — после 'save'/'exit'/'defaults'/MSP_SET_REBOOT линк рвётся и поднимается снова.
"""
from __future__ import annotations

import argparse
import os
import socket
import sys
import tempfile
import threading
import time
from html import escape
from pathlib import Path

from .fc import BUILD_OPTIONS, MSC_SECONDS, VirtualFC
from .tx_page import TX_PAGE as _TX_PAGE
from . import __build__ as _BUILD, __tx_page__ as _TX_PAGE_VERSION
from . import pages as _pages
from .failsafe_page import failsafe_page as _failsafe_page
from .msp import FROM_FC, MspParser, encode_v1, encode_v2

STX = 0x02
ETX = 0x03

_LOG_STREAM = None

# Имена портов по serialPortIdentifier_e (как в Configurator: composables/ports/portNames.js)
_PORT_NAMES = {
    0: "UART1", 1: "UART2", 2: "UART3", 3: "UART4", 4: "UART5", 5: "UART6",
    6: "UART7", 7: "UART8", 8: "UART9", 9: "UART10",
    20: "VCP", 30: "SOFTSERIAL1", 31: "SOFTSERIAL2",
}


def set_log_file(path):
    """Дублирует вывод эмулятора в файл — удобно, когда окно консоли закрывается."""
    global _LOG_STREAM
    try:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        _LOG_STREAM = open(path, "a", encoding="utf-8", buffering=1)
    except OSError:
        _LOG_STREAM = None

# Имена портов по serialPortIdentifier_e (как в Configurator: composables/ports/portNames.js)
_PORT_NAMES = {
    0: "UART1", 1: "UART2", 2: "UART3", 3: "UART4", 4: "UART5", 5: "UART6",
    6: "UART7", 7: "UART8", 8: "UART9", 9: "UART10",
    20: "VCP", 30: "SOFTSERIAL1", 31: "SOFTSERIAL2",
}


def _clock(ts: float) -> str:
    """Время события для страницы («19:05:31») — а не секунды с 1970 года."""
    try:
        return time.strftime("%H:%M:%S", time.localtime(ts))
    except (OverflowError, OSError, ValueError):
        return "—"


def _fatal(code: int):
    """Останавливает процесс целиком: серверы работают в отдельных потоках."""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.flush()
        except (OSError, ValueError):
            pass
    if _LOG_STREAM:
        try:
            _LOG_STREAM.flush()
        except OSError:
            pass
    os._exit(code)


def emit(msg: str):
    try:
        print(msg, flush=True)
    except (OSError, ValueError):  # консоль могла исчезнуть
        pass
    if _LOG_STREAM:
        try:
            _LOG_STREAM.write(str(msg) + "\n")
        except OSError:
            pass


class Session:
    """Обработка байтов одного соединения."""

    def __init__(self, fc: VirtualFC, verbose: bool = True, log=print):
        self.fc = fc
        self.verbose = verbose
        self.log = log
        self.parser = MspParser()
        self.terminal = fc.terminal()
        self.reboot_requested = False

    # ------------------------------------------------------------------ вход
    def process(self, data: bytes) -> bytes:
        out = bytearray()
        for i in range(len(data)):
            ch = data[i]
            if self.terminal.mode is not None:
                out += self.terminal.feed(bytes([ch]))
                if self.fc.pending_reboot and not self.reboot_requested:
                    self.reboot_requested = True
                    self.log("[FC] перезагрузка по CLI-команде: линк будет разорван")
                continue
            if self._parser_busy():
                # идёт незавершённый MSP-кадр: отдаём байт парсеру как есть
                self.parser.feed(bytes([ch]))
                for req in self.parser:
                    out += self._dispatch(req)
                continue
            if ch == 0x23:  # '#' — интерактивный CLI
                if self.verbose:
                    self.log("[CLI] вход в интерактивный режим ('#')")
                out += self.terminal.enter_interactive()
                continue
            if ch == STX:  # 0x02 — неинтерактивный CLI (flow control)
                if self.verbose:
                    self.log("[CLI] вход в неинтерактивный режим (STX)")
                out += self.terminal.enter_noninteractive()
                continue
            self.parser.feed(bytes([ch]))
            for req in self.parser:
                out += self._dispatch(req)
        return bytes(out)

    def _parser_busy(self) -> bool:
        return len(self.parser._buf) > 0

    def _dispatch(self, req) -> bytes:
        try:
            payload = self.fc.handle(req.code, req.payload, req.version)
        except Exception:
            # Ошибка в обработчике раньше молчала: Configurator получал таймаут и непонятное
            # поведение в интерфейсе. Теперь всё видно в логе (и в мониторе состояния).
            import traceback

            name = _code_name(req.code)
            self.log(f"[X] ошибка в обработчике 0x{req.code:04x} {name} ({len(req.payload)} байт):")
            self.log(traceback.format_exc())
            self.log("    Ответ отдан пустым, чтобы Configurator не ждал таймаут.")
            payload = b""
        if payload is None:
            return b""
        if req.version == 2:
            resp = encode_v2(req.code, payload)
        else:
            resp = encode_v1(req.code & 0xFF, payload, direction=FROM_FC)
        self.fc.note_msp(req.code, _code_name(req.code), len(req.payload), len(payload))
        if self.verbose and req.code not in (101, 150, 110, 130, 105, 102, 108, 109, 104, 106, 107):
            name = _code_name(req.code)
            self.log(f"[MSP] 0x{req.code:04x} {name} ({len(req.payload)} -> {len(payload)} байт)")
        if self.fc.pending_reboot and not self.reboot_requested:
            self.reboot_requested = True
            self.log("[FC] перезагрузка: линк будет разорван")
        return resp

    def on_reboot(self):
        """Вызывается после отправки последнего ответа: имитируем перезагрузку."""
        self.fc.reload_after_reboot()  # прошивка стартует с конфигурацией из «флеша»
        self.fc.pending_reboot = False
        self.fc.pending_bootloader = False
        self.fc.boot_time = time.time()
        self.fc.cli_active = False
        self.terminal.mode = None
        self.parser = MspParser()
        self.reboot_requested = False


def _gps_state(fc) -> dict:
    """Состояние виртуального GPS: то, что уходит вкладке «GPS» и что видно на мониторе."""
    solution = fc.gps_solution()
    _used, rows = fc.gps_satellites()
    return {
        "provider": fc._setting("gps_provider", 1),
        "sbas_mode": fc._setting("gps_sbas_mode", 0),
        "auto_config": fc._setting("gps_auto_config", 1),
        "auto_baud": fc._setting("gps_auto_baud", 0),
        "home_point_once": fc._setting("gps_set_home_point_once", 0),
        "use_galileo": fc._setting("gps_ublox_use_galileo", 0),
        "use_gps_build_option": "USE_GPS" in {name for name, _ in BUILD_OPTIONS},
        "feature_gps": "GPS" in fc.features,
        "fix": solution["fix"],
        "sats": solution["sats"],
        "lat": round(solution["lat"], 7),
        "lon": round(solution["lon"], 7),
        "alt_m": solution["alt_m"],
        "speed_cms": solution["speed_cms"],
        "course_deg10": solution["course_deg10"],
        "pdop": solution["pdop"],
        "home_dist_m": solution["home_dist_m"],
        "home_dir_deg": solution["home_dir_deg"],
        "satellites": [
            {"gnss": gnss, "svid": svid, "quality": quality, "used": bool(quality & 0x08),
             "cno": cno}
            for gnss, svid, quality, cno in rows
        ],
    }


def _code_name(code: int) -> str:
    names = {v: k for k, v in vars(__import__("bfemu.fc", fromlist=["x"])).items() if k.startswith("MSP") and isinstance(v, int)}
    return names.get(code, "?")


# --------------------------------------------------------------------------- TCP
class TcpServer:
    def __init__(self, fc: VirtualFC, host: str, port: int, boot_delay: float = 1.5, verbose: bool = True):
        self.fc = fc
        self.host = host
        self.port = port
        self.boot_delay = boot_delay
        self.verbose = verbose
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._sock = None

    def log(self, msg):
        emit(msg)

    def serve_forever(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind((self.host, self.port))
        except OSError as exc:
            emit(f"[X] не удалось занять {self.host}:{self.port} — {exc}")
            emit("    Порт уже занят? Запустите на другом порту, например:")
            emit("        python -m bfemu --tcp 127.0.0.1:6761 --http 8088")
            emit("    и укажите в Configurator tcp://127.0.0.1:6761")
            _fatal(3)
        srv.listen(1)
        srv.settimeout(0.5)
        self._sock = srv
        self.log(f"[TCP] виртуальный FC слушает {self.host}:{self.port} "
                 f"(в Configurator: manual connection, порт tcp://{self.host}:{self.port})")
        while not self._stop.is_set():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self.log(f"[TCP] подключение от {addr[0]}:{addr[1]}")
            threading.Thread(target=self._serve_conn_thread, args=(conn,), daemon=True).start()
        try:
            srv.close()
        except OSError:
            pass

    def _serve_conn_thread(self, conn: socket.socket):
        try:
            self._serve_conn(conn)
        except Exception as exc:  # не роняем сервер из-за одного клиента
            self.log(f"[TCP] ошибка сессии: {type(exc).__name__}: {exc}")
        finally:
            self.log("[TCP] соединение закрыто")

    def _serve_conn(self, conn: socket.socket):
        if self.fc.msc_active():
            # Как настоящий борт в режиме запоминающего устройства: MSP-порт молчит,
            # «флеш» доступен файлами на странице /msc.
            self.log("[MSC] борт в режиме запоминающего устройства — MSP не отвечает "
                     "(файлы: /msc; выйти: кнопка на странице или перезапуск эмулятора)")
            time.sleep(0.2)
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            conn.close()
            return
        session = Session(self.fc, verbose=self.verbose, log=self.log)
        conn.settimeout(0.2)
        try:
            while not self._stop.is_set():
                try:
                    data = conn.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not data:
                    break
                with self._lock:
                    resp = session.process(data)
                if resp:
                    conn.sendall(resp)
                if session.reboot_requested:
                    time.sleep(0.2)  # даём ответу уйти
                    try:
                        conn.shutdown(socket.SHUT_RDWR)
                    except OSError:
                        pass
                    break
        finally:
            try:
                conn.close()
            except OSError:
                pass
        if session.reboot_requested:
            session.on_reboot()
            if self.fc.msc_active():
                # не «перезагрузка», а вход в режим ЗУ: MSP остаётся молчащим до выхода
                self.log(f"[MSC] борт ушёл в режим запоминающего устройства на {MSC_SECONDS // 60} мин")
                return
            self.log(f"[FC] перезагрузка завершена через {self.boot_delay}s")
            time.sleep(self.boot_delay)

    def stop(self):
        self._stop.set()
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass


# --------------------------------------------------------------------------- PTY
class PtyServer:
    """Linux/macOS: создаёт псевдотерминал, который виден как обычный последовательный порт."""

    def __init__(self, fc: VirtualFC, link_path: str, verbose: bool = True):
        self.fc = fc
        self.link_path = Path(link_path)
        self.verbose = verbose
        self._stop = threading.Event()

    def log(self, msg):
        emit(msg)

    def serve_forever(self):
        if os.name != "posix":
            raise RuntimeError("PTY доступен только на Linux/macOS; используйте --tcp или --stdio")
        master, slave = os.openpty()
        slave_name = os.ttyname(slave)
        if self.link_path.exists() or self.link_path.is_symlink():
            self.link_path.unlink()
        os.symlink(slave_name, self.link_path)
        self.log(f"[PTY] tty: {slave_name} -> символическая ссылка {self.link_path}")
        self.log("[PTY] в Configurator выберите этот порт (или 'socat' для COM на Windows)")
        session = Session(self.fc, verbose=self.verbose, log=self.log)
        while not self._stop.is_set():
            try:
                data = os.read(master, 4096)
            except OSError:
                break
            if not data:
                continue
            resp = session.process(data)
            if resp:
                os.write(master, resp)
            if session.reboot_requested:
                session.on_reboot()
                time.sleep(1.5)

    def stop(self):
        self._stop.set()


# -------------------------------------------------------------------------- stdio
class StdioServer:
    """Байтовый мост stdin/stdout: удобно подключать через socat/pipe/com0com."""

    def __init__(self, fc: VirtualFC, verbose: bool = True):
        self.fc = fc
        self.verbose = verbose

    def log(self, msg):
        emit(msg)

    def serve_forever(self):
        session = Session(self.fc, verbose=self.verbose, log=self.log)
        self.log("[STDIO] работаю через stdin/stdout")
        stdin = sys.stdin.buffer
        stdout = sys.stdout.buffer
        while True:
            data = stdin.read(1)
            if not data:
                break
            resp = session.process(data)
            if resp:
                stdout.write(resp)
                stdout.flush()
        return


# ------------------------------------------------------------------------ HTTP
class HttpMonitor:
    """Небольшая веб-страница состояния — удобно смотреть, что делает Configurator."""

    def __init__(self, fc: VirtualFC, port: int, strict_port: bool = False, host: str = "127.0.0.1"):
        self.fc = fc
        self.port = port
        # По умолчанию монитор виден только с этой машины: на странице /tx можно двигать стики
        # и переключать режимы, а в общей сети это значит «любой сосед управляет вашим FC».
        # Нужен доступ с планшета или другой машины — запускайте с --http-host 0.0.0.0.
        self.host = host
        # strict_port — для проверок: если порт занят, лучше сразу остановиться, чем
        # «тихо» переехать и потом опрашивать чужой монитор по исходному адресу.
        self.strict_port = strict_port
        self._httpd = None
        self._prepared = False

    def prepare(self):
        """Занимает порт сразу, до запуска потоков.

        Иначе получается гонка: TCP-порт уже принимает подключения, а веб-монитор
        ещё не слушает — первый запрос страницы получал «соединение отклонено».
        """
        if self._prepared:
            return
        from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

        fc = self.fc

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *args):
                pass

            def _foreign(self) -> bool:
                """Запрос инициирован чужим сайтом?

                Кнопки монитора — обычные ссылки и POST'ы, поэтому страница на другом домене
                может дёрнуть их у всех, кто открыл монитор (`<img src="http://.../failsafe/on">`).
                Браузеры присылают Sec-Fetch-Site; `cross-site`/`same-site` означает «не наша
                страница». Консольные клиенты (curl, urllib, tools/tx.py) заголовок не шлют —
                для них ничего не меняется.
                """
                site = (self.headers.get("Sec-Fetch-Site") or "").lower()
                if site in ("cross-site", "same-site"):
                    return True
                origin = self.headers.get("Origin")
                if origin and origin.split("//")[-1] != (self.headers.get("Host") or ""):
                    return True
                return False

            def _deny_foreign(self) -> bool:
                if not self._foreign():
                    return False
                self._send("запрос с чужой страницы отклонён".encode(),
                           "text/plain; charset=utf-8", 403)
                return True

            def do_GET(self):
                import json as _json
                path = self.path.split("?")[0].rstrip("/") or "/"
                # Меняющие состояние адреса (кнопки страниц) защищаем от межсайтовых вызовов.
                if (path.startswith(("/failsafe/", "/sim_flight/")) or path == "/msc/exit") \
                        and self._deny_foreign():
                    return
                if path == "/api/rc":
                    body = _json.dumps(fc.rc_state(), ensure_ascii=False).encode()
                    self._send(body, "application/json; charset=utf-8")
                    return
                if path == "/tx":
                    body = _TX_PAGE.replace("__HTTP_PORT__", str(self.server.server_address[1])) \
                                    .replace("__TCP_PORT__", _tcp_port()) \
                                    .replace("__BUILD__", _BUILD) \
                                    .replace("__TX_PAGE__", _TX_PAGE_VERSION) \
                                    .replace("__NAV_STYLE__", _pages.NAV_STYLE) \
                                    .replace("__NAV__", _pages.nav_html("tx", _BUILD)).encode()
                    self._send(body, "text/html; charset=utf-8")
                    return
                if path in ("/nav", "/pages"):
                    body = _pages.nav_page(_pages.nav_html("nav", _BUILD), _pages.NAV_STYLE,
                                           _BUILD, self.server.server_address[1], _tcp_port()).encode()
                    self._send(body, "text/html; charset=utf-8")
                    return
                if path in ("/failsafe", "/fs"):
                    st = fc.sim_failsafe_state()
                    body = _failsafe_page(_pages.nav_html("failsafe", _BUILD), _pages.NAV_STYLE,
                                          _BUILD, self.server.server_address[1], _tcp_port(),
                                          st["scale"], st["procedure"], st["scenario"]).encode()
                    self._send(body, "text/html; charset=utf-8")
                    return
                if path == "/api/failsafe":
                    body = _json.dumps(_failsafe_api(fc), ensure_ascii=False).encode()
                    self._send(body, "application/json; charset=utf-8")
                    return
                if path.startswith("/failsafe/"):
                    action = path.split("/", 2)[2]
                    if action == "on":
                        fc.set_sim_failsafe(True)
                    elif action == "off":
                        fc.set_sim_failsafe(False)
                    elif action == "loss":
                        fc.sim_failsafe_loss()
                    elif action == "restore":
                        fc.sim_failsafe_restore()
                    elif action == "reset":
                        fc.sim_failsafe_reset()
                    elif action == "arm_air":
                        fc.set_sim_failsafe(True)
                        fc.sim_failsafe_arm(True, throttle=1500)
                    elif action == "arm_ground":
                        fc.set_sim_failsafe(True)
                        fc.sim_failsafe_arm(True, throttle=1000)
                    elif action == "disarm":
                        fc.sim_failsafe_arm(False)
                    elif action == "box_on":
                        fc.set_sim_failsafe_box(True)
                    elif action == "box_off":
                        fc.set_sim_failsafe_box(False)
                    elif action.startswith("switch/"):
                        fc.set_sim_failsafe_switch(action.split("/", 1)[1])
                    elif action.startswith("scale/"):
                        fc.set_sim_failsafe_scale(action.split("/", 1)[1])
                    elif action.startswith("procedure/"):
                        fc._store_setting("failsafe_procedure", int(action.split("/", 1)[1]))
                        fc._log("[SIM] процедура Этапа 2: "
                                f"{fc.FS_PROCEDURE_NAMES[fc._fs_config()['procedure']]}")
                    elif action.startswith("scenario/"):
                        fc.set_sim_failsafe_scenario(action.split("/", 1)[1])
                    self.send_response(303)
                    self.send_header("Location", "/failsafe")
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                    return
                if path in ("/aux", "/modes"):
                    body = _pages.aux_page(fc, _pages.nav_html("aux", _BUILD), _pages.NAV_STYLE,
                                           _BUILD, self.server.server_address[1], _tcp_port()).encode()
                    self._send(body, "text/html; charset=utf-8")
                    return
                if path == "/msc":
                    page = self._msc_page() \
                        .replace("__NAV_STYLE__", _pages.NAV_STYLE) \
                        .replace("__NAV__", _pages.nav_html("msc", _BUILD))
                    self._send(page.encode(), "text/html; charset=utf-8")
                    return
                if path in ("/sim_flight/on", "/sim_flight/off", "/sim_flight/reset"):
                    action = path.rsplit("/", 1)[1]
                    if action == "on":
                        fc.set_sim_flight(True)
                    elif action == "off":
                        fc.set_sim_flight(False)
                    else:
                        fc.mah_drawn = 0.0
                        fc.motor_mah = [0.0] * len(fc.motor_mah)
                        fc._log("[SIM] счётчик мА·ч сброшен кнопкой на мониторе")
                    self.send_response(303)
                    self.send_header("Location", "/")
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                    return
                if path == "/msc/exit":
                    fc.exit_msc("кнопка на странице")
                    self.send_response(303)
                    self.send_header("Location", "/")
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                    return
                if path.startswith("/msc/"):
                    self._send_file(path.split("/", 2)[2])
                    return
                if path.startswith("/api/state"):
                    body = _json.dumps({
                        "fc": fc.banner(),
                        "rc": fc.rc_state(),
                        "settings": len(fc.settings_list),
                        "uptime": round(time.time() - fc.boot_time, 1),
                        "build": _BUILD,
                        "features": sorted(fc.features),
                        "ports": [
                            {
                                "identifier": port["identifier"],
                                "name": _PORT_NAMES.get(port["identifier"], str(port["identifier"])),
                                "functions": list(port["functions"]),
                            }
                            for port in fc.serial_ports
                        ],
                        "blackbox": fc.blackbox.summary(),
                        "gps": _gps_state(fc),
                        "motors": fc.motors_state(),
                        "power": fc.power_state(),
                        "modes": fc.modes_state(),
                        "failsafe": fc.sim_failsafe_state(),
                        "dshot_commands": fc.dshot_commands_log[-5:],
                        "cli": fc.recent_cli[-10:],
                        "msp": fc.recent_msp[-15:],
                        "counters": {f"0x{k:04x}": v for k, v in sorted(fc.stats.items())},
                    }, ensure_ascii=False).encode()
                    ctype = "application/json; charset=utf-8"
                else:
                    body = self._page() \
                        .replace("__NAV_STYLE__", _pages.NAV_STYLE) \
                        .replace("__NAV__", _pages.nav_html("monitor", _BUILD)).encode()
                    ctype = "text/html; charset=utf-8"
                self._send(body, ctype)

            def _send(self, body: bytes, ctype: str, status: int = 200):
                self.send_response(status)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):
                """Виртуальная аппаратура: POST /api/rc — каналы из страницы /tx или скрипта."""
                import json as _json
                if self._deny_foreign():
                    return
                path = self.path.split("?")[0].rstrip("/") or "/"
                if path != "/api/rc":
                    self._send(b'{"error":"only POST /api/rc is supported"}',
                               "application/json; charset=utf-8", 404)
                    return
                try:
                    length = int(self.headers.get("Content-Length") or 0)
                    data = _json.loads(self.rfile.read(length) or b"{}")
                    if not isinstance(data, dict):
                        raise ValueError("нужен JSON-объект")
                except (ValueError, _json.JSONDecodeError) as exc:
                    self._send(_json.dumps({"error": f"плохой JSON: {exc}"}, ensure_ascii=False).encode(),
                               "application/json; charset=utf-8", 400)
                    return

                # «Один хозяин» у каналов: писать может только та сторона, за которой управление.
                # Смотрим на управление ПОСЛЕ этого запроса: если в нём есть control, значит
                # хозяин меняется этим же запросом (так делает, например, выход из tools/tx.py).
                target_control = str(data["control"]) if "control" in data else fc.rc_control
                payload = data["channels"] if "channels" in data else {
                    k: v for k, v in data.items()
                    if k not in ("reset", "connected", "source", "channels", "bind",
                                 "bind_seconds", "control", "physical", "rc_map",
                                 "layout_note", "aux_map")}
                if data.get("physical") and target_control != "physical":
                    # «Отключить (выдернуть USB)» на странице /tx отсоединяет настоящую
                    # аппаратуру: её кадры больше не принимаются, пока не нажмут
                    # «Подключить аппаратуру». Иначе два источника (пульт и ползунки)
                    # писали бы каналы одновременно — это и давало «срывы».
                    self._send(_json.dumps(
                        {"error": "управление каналами сейчас у виртуальной аппаратуры "
                                  "(страница /tx): нажмите «Подключить аппаратуру»",
                         "refused": True, "state": fc.rc_state()}, ensure_ascii=False).encode(),
                        "application/json; charset=utf-8", 409)
                    return
                if payload and not data.get("physical") and target_control == "physical":
                    # обратный случай: каналами владеет настоящая аппаратура — страница и
                    # скрипты не пишут, иначе ползунки дёргали бы стики пульта
                    self._send(_json.dumps(
                        {"error": "управление каналами сейчас у настоящей аппаратуры "
                                  "(стики пульта/приёмника): на странице /tx нажмите "
                                  "«Отключить (выдернуть USB)», чтобы вернуть управление странице",
                         "refused": True, "state": fc.rc_state()}, ensure_ascii=False).encode(),
                        "application/json; charset=utf-8", 409)
                    return

                applied = 0
                if data.get("reset"):
                    fc.reset_rc_channels()
                    applied += 1
                if data.get("source"):
                    # источник — тоже изменение состояния (страница показывает, кто двигал стики);
                    # заодно эмулятор считает, не пишут ли каналы сразу несколько инструментов
                    fc.rc_source = str(data["source"])[:96]     # с запасом: в подписи есть PID и название пульта
                    fc.note_rc_source(fc.rc_source)
                    applied += 1
                if "connected" in data:
                    # Виртуальная аппаратура «в эфире» / «выключена»: при выключенной
                    # прошивка отдаёт значения «Этапа 1», как при потере сигнала.
                    fc.set_rx_link(bool(data["connected"]))
                    applied += 1
                if "rc_map" in data:
                    fc.set_rc_map_info(data["rc_map"])
                    applied += 1
                if "layout_note" in data:
                    # заметка «раскладка осей не та» — её показывает страница /tx
                    fc.set_rc_layout_note(data["layout_note"])
                    applied += 1
                if "aux_map" in data:
                    # раскладка тумблеров и крутилок (её находит калибровка --detect-aux)
                    fc.set_rc_aux_map(data["aux_map"])
                    applied += 1
                if "control" in data:
                    # «Подключить аппаратуру» (control: "physical") — управление у настоящей
                    # аппаратуры, кнопки страницы гаснут. «Отключить (выдернуть USB)»
                    # (control: "virtual") — управление возвращается странице /tx.
                    fc.set_rc_control(str(data["control"]))
                    applied += 1
                if "bind" in data:
                    # bind: true — «связать» (по умолчанию 2 с поиска, bind_seconds: 0 — сразу),
                    # bind: false — «развязать». Пока идёт связывание, кадров в эфире нет.
                    if data["bind"]:
                        fc.start_bind(data.get("bind_seconds"),
                                      connected=bool(data.get("connected", True)))
                    else:
                        fc.unbind()
                    applied += 1
                if payload:
                    # сначала проверяем имена/типы, чтобы «повторное» значение не выглядело ошибкой
                    unknown = [k for k in payload if fc._rc_channel_index(k) is None] \
                        if isinstance(payload, dict) else []
                    if not unknown and not isinstance(payload, (dict, list, tuple)):
                        unknown = ["?"]
                    if unknown:
                        self._send(_json.dumps(
                            {"error": "каналы не распознаны: " + ", ".join(map(str, unknown)) +
                                      " (нужны roll/pitch/yaw/throttle/aux1…aux12 или номера 0…15)",
                             "state": fc.rc_state()}, ensure_ascii=False).encode(),
                            "application/json; charset=utf-8", 400)
                        return
                    before = list(fc.rc_channels)
                    fc.set_rc_channels(payload)
                    applied += max(1, sum(1 for a, b in zip(before, fc.rc_channels) if a != b))
                if payload or data.get("reset"):
                    fc.note_rc_source(fc.rc_source)      # каналы точно менялись: отметим писателя
                if not applied:
                    self._send(_json.dumps(
                        {"error": "пустой запрос: нужны каналы, reset, connected, control или bind",
                         "state": fc.rc_state()}, ensure_ascii=False).encode(),
                        "application/json; charset=utf-8", 400)
                    return
                body = _json.dumps({"applied": applied, **fc.rc_state()}, ensure_ascii=False).encode()
                self._send(body, "application/json; charset=utf-8")

            def _msc_files(self):
                """Содержимое «флеша» так, как его увидел бы компьютер: файлами по пролётам."""
                return fc.blackbox.files()

            def _msc_page(self):
                f = fc
                rows = "".join(
                    f"<tr><td><a href=\"/msc/{esc(item['name'])}\">{esc(item['name'])}</a></td>"
                    f"<td>{item['title']}</td><td>{item['size']:,} Б</td>"
                    f"<td>{item['start'].replace('T', ' ') if item['start'] else '—'}</td>"
                    f"<td><a class=dl href=\"/msc/{esc(item['name'])}\">скачать</a></td></tr>".replace(",", " ")
                    for item in self._msc_files()
                ) or "<tr><td colspan=5>накопитель пуст</td></tr>"
                return f"""<!doctype html><html lang=ru><meta charset=utf-8>
<title>bfemu — режим запоминающего устройства</title>
<style>
 body{{font:14px/1.5 -apple-system,Segoe UI,Roboto,sans-serif;margin:0;background:#101418;color:#e6edf3}}
 header{{padding:18px 24px;background:#161b22;border-bottom:1px solid #30363d}}
 h1{{margin:0 0 4px;font-size:18px}} h2{{font-size:14px;color:#58a6ff;text-transform:uppercase;letter-spacing:.04em;margin:0 0 10px}}
 .sub{{color:#8b949e;font-size:13px}} main{{padding:18px 24px;display:grid;gap:18px}}
 .card{{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:14px 16px}}
 table{{border-collapse:collapse;width:100%;font-size:13px}} td,th{{text-align:left;padding:5px 8px;border-bottom:1px solid #21262d}}
 a{{color:#58a6ff}} code{{color:#7ee787}} .btn{{display:inline-block;margin-top:8px;background:#21262d;
 border:1px solid #30363d;border-radius:8px;padding:6px 14px;color:#e6edf3;text-decoration:none}}
 .warn{{color:#e3b341}}
__NAV_STYLE__
</style>
__NAV__
<header>
 <h1>Борт в режиме запоминающего устройства (MSC)</h1>
 <div class=sub>Так же, как у настоящего полётного контроллера: MSP не отвечает, а содержимое
 «флеша» видно файлами. {f.blackbox.used:,} Б занято, {len(self._msc_files()) - 1} пролётов.</div>
</header>
<main>
 <div class=card><h2>Содержимое «флеша»</h2>
  <table><tr><th>файл</th><th>что это</th><th>размер</th><th>начало</th><th></th></tr>{rows}</table>
  <p class=sub><code>LOG0000N.BBL</code> — отдельный пролёт (каждый читается просмотрщиком
  и декодером blackbox_decode). <code>BLACKBOX_ALL.BBL</code> — то же, что вкладка пишет
  кнопкой «Сохранено в файл…»: все пролёты в одном файле, переключение — в списке «Log»
  просмотрщика.</p>
  <p class=warn>У настоящего борта режим ЗУ выключается «передёргиванием питания».
  Здесь: кнопка ниже (или перезапуск эмулятора).</p>
  <a class=btn href="/msc/exit">Выйти из режима запоминающего устройства</a>
 </div></main></html>"""

            def _send_file(self, name: str):
                item = fc.blackbox.file(name)
                if item is None:
                    self._send(b"\xd1\x84\xd0\xb0\xd0\xb9\xd0\xbb \xd0\xbd\xd0\xb5 \xd0\xbd\xd0\xb0\xd0\xb9\xd0\xb4\xd0\xb5\xd0\xbd",
                               "text/plain; charset=utf-8", 404)
                    return
                body = item["data"]
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(len(body)))
                safe_name = "".join(ch for ch in name if ch.isalnum() or ch in "._-")[:64]
                self.send_header("Content-Disposition", f'attachment; filename="{safe_name}"')
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def _page(self):
                f = fc
                rows = "".join(
                    f"<tr><td>0x{c:04x}</td><td>{esc(n)}</td><td>{rq}</td><td>{rs}</td>"
                    f"<td>{_clock(t)}</td></tr>"
                    for t, c, n, rq, rs in reversed(f.recent_msp[-15:])
                )
                cli_rows = "".join(
                    f"<tr><td>{_clock(t)}</td><td><code>{esc(cmd)}</code></td>"
                    f"<td><pre>{esc(out.strip()[:400])}</pre></td></tr>"
                    for t, cmd, out in reversed(f.recent_cli[-10:])
                )
                counters = "".join(
                    f"<span class=chip>0x{c:04x} {_code_name(c)}: <b>{v}</b></span>"
                    for c, v in sorted(f.stats.items(), key=lambda kv: -kv[1])[:24]
                )
                bb = f.blackbox.summary()
                bb_logs = "".join(
                    f"<tr><td>{entry['index']}</td><td>{entry['title']}</td><td>{entry['size']} Б</td>"
                    f"<td>{entry['duration']:.1f} с</td><td>{entry['rate_hz']} Гц</td>"
                    f"<td>{entry['start']}</td></tr>"
                    for entry in bb["logs"]
                ) or "<tr><td colspan=6>логов нет — в CLI: <code>blackbox refill</code></td></tr>"
                gps = _gps_state(f)
                sats_used = sum(1 for row in gps["satellites"] if row["used"])
                provider = {0: "NMEA", 1: "UBLOX", 2: "MSP"}.get(gps["provider"], str(gps["provider"]))
                gps_card = (
                    '<div class=card><h2>GPS (приёмник на UART1)</h2><table>'
                    f'<tr><td>протокол</td><td>{provider}</td>'
                    f'<td>фикс</td><td>{"есть" if gps["fix"] else "нет"}</td>'
                    f'<td>спутников в решении</td><td>{sats_used} из {len(gps["satellites"])}</td></tr>'
                    f'<tr><td>широта</td><td>{gps["lat"]:.6f}</td>'
                    f'<td>долгота</td><td>{gps["lon"]:.6f}</td>'
                    f'<td>высота</td><td>{gps["alt_m"]} м</td></tr>'
                    f'<tr><td>скорость</td><td>{gps["speed_cms"]} см/с</td>'
                    f'<td>курс</td><td>{gps["course_deg10"] / 10:.0f}°</td>'
                    f'<td>до дома</td><td>{gps["home_dist_m"]} м / {gps["home_dir_deg"]}°</td></tr>'
                    '</table>'
                    f'<div class=sub style="margin-top:8px">build-опция <code>USE_GPS</code> (16412) — '
                    'по ней Configurator показывает вкладку «GPS»; фича '
                    f'<code>GPS</code> {"включена" if "GPS" in f.features else "выключена"}, '
                    f'auto_config={gps["auto_config"]}, galileo={gps["use_galileo"]}</div></div>'
                )
                motors = f.motors_state()
                protocol_names = {0: "PWM", 1: "ONESHOT125", 2: "ONESHOT42", 3: "MULTISHOT",
                                  4: "BRUSHED", 5: "DSHOT150", 6: "DSHOT300", 7: "DSHOT600",
                                  8: "PROSHOT1000", 9: "DISABLED"}
                motor_rows = "".join(
                    f"<tr><td>M{m['index']}</td><td><b>{m['output']}</b></td><td>{m['rpm']}</td>"
                    f"<td>{m['temp_c']} °C</td><td>{m['current_a']:.2f} А</td><td>{m['mah']:.1f} мА·ч</td></tr>"
                    for m in motors["motors"]
                )
                motor_state = ("идёт тест со слайдеров" if motors["test_active"]
                               else "моторы стоят (mincommand)")
                motors_card = (
                    f"<div class=card><h2>Моторы (вкладка «Моторы»: тест со слайдеров)</h2>"
                    f"<div class=sub>{motor_state} · протокол "
                    f"<b>{protocol_names.get(motors['protocol'], motors['protocol'])}</b> · "
                    f"DShot-телеметрия {'вкл' if motors['dshot_telemetry'] else 'выкл'} · "
                    f"ESC_SENSOR {'вкл' if motors['esc_sensor'] else 'выкл'} · "
                    f"напряжение на ESC <b>{motors['voltage_v']:.2f} В</b>"
                    f"{' (просадка под нагрузкой)' if motors['test_active'] else ''}</div>"
                    f"<table><tr><th>мотор</th><th>выход</th><th>обороты</th><th>ESC</th>"
                    f"<th>ток</th><th>расход</th></tr>{motor_rows}</table>"
                    f"<div class=sub style=\"margin-top:8px\">Слайдер на вкладке «Моторы» → "
                    f"<code>MSP_SET_MOTOR</code> (214) пишет <code>motor_disarmed[]</code>, а "
                    f"<code>MSP_MOTOR</code> (104) и <code>MSP_MOTOR_TELEMETRY</code> (139) "
                    f"отдают живые выходы, обороты, температуру, ток и напряжение — "
                    f"как на железной плате с ESC-телеметрией.</div></div>"
                )
                fs = f.sim_failsafe_state()
                fs_buttons = (
                    '<a class=chip href="/failsafe/off" style="color:#e3b341">&#9632; выключить демо</a>'
                    if fs["demo"] else
                    '<a class=chip href="/failsafe/on" style="color:#7ee787">&#9654; включить демо</a>')
                fs_buttons += ('<a class=chip href="/failsafe/loss" style="color:#f85149">'
                               'потерять сигнал</a>'
                               '<a class=chip href="/failsafe/restore" style="color:#58a6ff">'
                               'вернуть сигнал</a>'
                               '<a class=chip href="/failsafe/reset">сброс демо</a>'
                               '<a class=chip href="/failsafe" style="color:#7ee787">'
                               'страница /failsafe</a>')
                fs_plate = ("ОТКАТ НЕВОЗМОЖЕН" if fs["rollback"] and not fs["rollback"]["possible"]
                            else "ОТКАТ ВОЗМОЖЕН")
                fs_card = (
                    f"<div class=card><h2>Failsafe — демонстрация (failsafe.c 4.5.4)</h2>"
                    f"<div class=sub>вкладка «Failsafe» настраивает, а здесь видно, что борт делает: "
                    f"Этап 1 → <b>точка отката</b> → Этап 2 → возврат сигнала · "
                    f"масштаб показа ×{fs['scale']:g}</div>"
                    f"<table><tr><td>фаза</td><td><b>{fs['phase_text']}</b> "
                    f"<span class=sub>({fs['phase_en']})</span></td>"
                    f"<td>откат</td><td class={'ok' if fs['rollback'] and fs['rollback']['possible'] else 'no'}>"
                    f"<b>{fs_plate}</b> <span class=sub>{fs['rollback']['text'] if fs['rollback'] else ''}</span></td></tr>"
                    f"<tr><td>связь</td><td>{'кадры идут' if fs['link'] else 'кадров нет'}</td>"
                    f"<td>процедура</td><td><b>{fs['procedure_name']}</b> "
                    f"<span class=sub>{fs['procedure_text']}</span></td></tr>"
                    f"<tr><td>моторы</td><td>{' '.join(str(int(round(v))) for v in f.motor_disarmed)}</td>"
                    f"<td>взвод</td><td>{'разрешён' if not f.arming_disable_flags else 'запрещён'} "
                    f"<span class=sub>0x{f.arming_disable_flags:08X}</span></td></tr>"
                    f"<tr><td>каналы (MSP_RC)</td><td colspan=3>"
                    f"{' '.join(str(v) for v in fs['channels'][:8])} …</td></tr>"
                    f"</table>"
                    f"<div class=sub style=\"margin-top:8px\">{fs_buttons}</div>"
                    f"<div class=sub>Живой показ для аудитории — "
                    f"<a href=\"/failsafe\" style=\"color:#58a6ff\">страница /failsafe</a>; "
                    f"из CLI то же самое: <code>sim_failsafe on</code> / "
                    f"<code>sim_failsafe loss</code> / <code>sim_failsafe status</code>.</div></div>"
                )
                power = f.power_state()
                sim = power["sim"]
                sim_phase = {"idle": "выключен", "flight": "полёт",
                             "landing": "посадка", "swap": "замена батареи"}.get(
                                 sim["phase"], sim["phase"])
                sim_buttons = (
                    '<a class=chip href="/sim_flight/off" style="color:#e3b341">'
                    '&#9632; остановить полёт</a>'
                    if sim["on"] else
                    '<a class=chip href="/sim_flight/on" style="color:#7ee787">'
                    '&#9654; включить виртуальный полёт</a>'
                )
                sim_buttons += ('<a class=chip href="/sim_flight/reset" style="color:#58a6ff">'
                                'сбросить расход</a>')
                power_rows = (
                    f"<tr><td>напряжение</td><td><b>{power['voltage']:.2f} В</b> "
                    f"<span class=sub>(покой {power['rest_voltage']:.2f} В, "
                    f"{power['cells']}S)</span></td>"
                    f"<td>ток</td><td><b>{power['current']:.2f} А</b> "
                    f"<span class=sub>(электроника + моторы)</span></td>"
                    f"<td>расход</td><td><b>{power['mah']} мА·ч</b> "
                    f"<span class=sub>из {power['capacity']}</span></td>"
                    f"<td>статус</td><td class={'ok' if power['state'] == 0 else ''}>"
                    f"<b>{power['state_name']}</b></td></tr>"
                )
                power_card = (
                    f"<div class=card><h2>Питание и батарея (вкладка «Питание и батарея»)</h2>"
                    f"<div class=sub>те же цифры, что читает Configurator: "
                    f"<code>MSP_BATTERY_STATE</code> (130) и <code>MSP_ANALOG</code> (110) · "
                    f"виртуальный полёт: <b>{sim_phase}</b>"
                    f"{(', ' + str(sim['seconds']) + ' с') if sim['on'] else ''}"
                    f"{(' · батарей заменено: ' + str(sim['packs'])) if sim['packs'] else ''}"
                    f"</div>"
                    f"<table>{power_rows}</table>"
                    f"<div class=sub style=\"margin-top:8px\">{sim_buttons} "
                    f"&#183; то же самое из CLI: <code>sim_flight on</code> / "
                    f"<code>sim_flight off</code> / <code>sim_flight status</code></div>"
                    f"<div class=sub>В покое горит ток электроники (≈0.55 А) и медленно капает "
                    f"расход. В полёте газ идёт по сценарию: взлёт, висение с подруливанием, "
                    f"«газ в пол» каждые 25 с — ток 3…27 А, напряжение проседает и падает по "
                    f"мере разряда, счётчик мА·ч растёт в {sim['time_scale']:.0f} раз быстрее "
                    f"стенных часов. Когда расход доходит до ёмкости, борт садится и получает "
                    f"новую батарею: счётчик обнуляется, напряжение снова 4.20 В на банку."
                    f"{(' Последнее событие: ' + sim['note'] + '.') if sim['note'] else ''}</div>"
                    f"</div>"
                )
                modes = f.modes_state()
                used_slots = [s for s in modes["slots"] if s["configured"]]
                if used_slots:
                    mode_rows = "".join(
                        f"<tr><td>{s['index']}</td><td><b>{s['box']}</b></td>"
                        f"<td>{'AUX' + str(s['aux'] + 1) if not s['linked'] else 'связь с ' + s['linked']}</td>"
                        f"<td>{'' if s['linked'] else str(s['start']) + '…' + str(s['end'])}</td>"
                        f"<td>{s['logic']}</td>"
                        f"<td class={'ok' if s['active'] else ''}>"
                        f"{'включён' if s['active'] else 'выключен'}</td></tr>"
                        for s in used_slots
                    )
                else:
                    mode_rows = ('<tr><td colspan=6>слотов нет — как на свежем борде после '
                                 '<code>defaults</code>: вкладка «Режимы» пустая</td></tr>')
                active_modes = ", ".join(modes["active"]) or "нет"
                # Справочник «режим (permanentId)» — то, что нужно для CLI 'aux': в прошивке
                # номера не сквозные (3, 9-11, 14, 16, 18, 21, 22, 38 — удалённые режимы).
                modes_ref = " · ".join(f"<b>{n}</b>&nbsp;({f.box_permanent_id(n)})"
                                       for n in f.active_boxes)
                modes_card = (
                    f"<div class=card><h2>Режимы (вкладка «Режимы»: MSP_MODE_RANGES)</h2>"
                    f"<div class=sub>занято слотов <b>{modes['slots_used']}</b> из "
                    f"{modes['slots_total']} · включены сейчас: <b>{active_modes}</b> · "
                    f"маска режимов в MSP_STATUS: <code>0x{f.rc_mode_mask():08x}</code></div>"
                    f"<table><tr><th>слот</th><th>режим</th><th>канал</th><th>диапазон</th>"
                    f"<th>логика</th><th>состояние</th></tr>{mode_rows}</table>"
                    f"<div class=sub style=\"margin-top:8px\">Диапазоны вкладки → "
                    f"<code>MSP_SET_MODE_RANGE</code> (35), чтение — <code>MSP_MODE_RANGES</code> (34) "
                    f"и <code>MSP_MODE_RANGES_EXTRA</code> (238, логика и связи). Состояние "
                    f"считается по положению тумблеров аппаратуры: канал попадает в диапазон — "
                    f"«включён», и это видно точкой на вкладке и в MSP_STATUS.</div>"
                    f"<div class=sub style=\"margin-top:8px\">Режимы этой платы и их "
                    f"<code>permanentId</code> для CLI <code>aux &lt;слот&gt; &lt;permanentId&gt; …</code>: "
                    f"{modes_ref}. Номера не сквозные: 3, 9–11, 14, 16, 18, 21, 22, 38 — "
                    f"удалённые режимы (BARO, CAMTRIG, GPS HOME, GPS HOLD, …), их использовать "
                    f"нельзя.</div></div>"
                )
                bb_card = (
                    f"<div class=card><h2>Чёрный ящик (dataflash)</h2>"
                    f"<div class=sub>занято <b>{bb['used']}</b> из {bb['size']} байт, "
                    f"секторов {bb['sectors']}, свободно {bb['free']} · запись "
                    f"{(('1/' + str(bb['p_interval'])) if bb['p_interval'] else '1') + ' цикла'}"
                    f"{(' = ' + str(bb['rate_hz']) + ' Гц') if bb['rate_hz'] else ''}</div>"
                    f"<table><tr><th>#</th><th>пролёт</th><th>размер</th><th>длительность</th>"
                    f"<th>частота</th><th>начало</th></tr>{bb_logs}</table></div>"
                )
                msc_banner = ""
                if f.msc_active():
                    msc_banner = (
                        "<div class=card style=\"border-color:#e3b341\"><h2 style=\"color:#e3b341\">"
                        "Борт в режиме запоминающего устройства</h2>"
                        "<div class=sub>MSP не отвечает — так же ведёт себя настоящий полётный "
                        "контроллер, ушедший в режим ЗУ. Файлы «флеша»: "
                        "<a href=\"/msc\" style=\"color:#58a6ff\">/msc</a> · "
                        "выйти: <a href=\"/msc/exit\" style=\"color:#58a6ff\">/msc/exit</a> "
                        "или перезапустить эмулятор.</div></div>"
                    )
                return f"""<!doctype html><html lang=ru><meta charset=utf-8>
<title>bfemu — виртуальный полётный контроллер</title>
<meta http-equiv=refresh content=2>
<style>
 body{{font:14px/1.5 -apple-system,Segoe UI,Roboto,sans-serif;margin:0;background:#101418;color:#e6edf3}}
 header{{padding:18px 24px;background:#161b22;border-bottom:1px solid #30363d}}
 h1{{margin:0 0 4px;font-size:18px}} .sub{{color:#8b949e;font-size:13px}}
 main{{padding:18px 24px;display:grid;gap:18px}}
 .card{{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:14px 16px}}
 .card h2{{margin:0 0 10px;font-size:14px;color:#58a6ff;text-transform:uppercase;letter-spacing:.04em}}
 .chip{{display:inline-block;background:#21262d;border:1px solid #30363d;border-radius:999px;padding:2px 10px;margin:2px 4px 2px 0;font-size:12px}}
 table{{border-collapse:collapse;width:100%;font-size:13px}} td,th{{text-align:left;padding:4px 8px;border-bottom:1px solid #21262d}}
 code{{color:#7ee787}} pre{{margin:0;white-space:pre-wrap;color:#c9d1d9;font-size:12px}}
 .ok{{color:#3fb950}}
__NAV_STYLE__
</style>
__NAV__
<header>
 <h1>Виртуальный полётный контроллер <span class=ok>работает</span></h1>
 <div class=sub>{esc(f.banner())} · uptime {time.time()-f.boot_time:.0f}s · настроек в базе: {len(f.settings_list)}</div>
 <div class=sub>Подключение из Betaflight Configurator: ручное подключение, порт <code>tcp://127.0.0.1:{_tcp_port()}</code></div>
 <div class=sub>Виртуальная аппаратура (двигать стики): <a href="/tx" style="color:#58a6ff">http://127.0.0.1:{self.server.server_address[1]}/tx</a></div>
 <div class=sub>сборка <code>{_BUILD}</code> · страница аппаратуры v<code>{_TX_PAGE_VERSION}</code></div>
</header>
<main>
 <div class=card><h2>Активные фичи</h2>{' '.join(f'<span class=chip>{x}</span>' for x in sorted(f.features))}</div>
 {gps_card}
 {fs_card}
 {power_card}
 {motors_card}
 {modes_card}
 {msc_banner}
 {bb_card}
 <div class=card><h2>Последние MSP-запросы</h2>
  <div class=sub>обновляется само, раз в 2 секунды</div>
  <table><tr><th>код</th><th>сообщение</th><th>запрос</th><th>ответ</th><th>когда</th></tr>
  <tbody id=msp-body>{rows}</tbody></table></div>
 <div class=card><h2>CLI-команды</h2>
  <div class=sub>команды, введённые и в Configurator (вкладка CLI), и через MSP2_CLI_COMMAND;
  обновляется само, раз в 2 секунды</div>
  <table><tr><th>когда</th><th>команда</th><th>вывод</th></tr>
  <tbody id=cli-body>{cli_rows}</tbody></table></div>
 <div class=card><h2>Счётчики команд</h2>{counters}</div>
</main>
<script>
'use strict';
/* Раздел «CLI-команды» и «Последние MSP-запросы» обновляются сами: команда, введённая
   в CLI Configurator, появляется здесь в пределах пары секунд — страницу перезагружать
   не нужно. Перерисовываем только при изменении данных, чтобы не мешать выделять текст. */
function esc(v) {{
  return String(v).replace(/[&<>]/g, function (c) {{ return {{'&':'&amp;','<':'&lt;','>':'&gt;'}}[c]; }});
}}
function clock(seconds) {{
  const d = new Date(seconds * 1000);
  return isNaN(d) ? '—' : d.toTimeString().slice(0, 8);
}}
function paint(id, html) {{
  const body = document.getElementById(id);
  if (body && body.dataset.stamp !== html) {{
    body.dataset.stamp = html;
    body.innerHTML = html || '<tr><td colspan=9>—</td></tr>';
  }}
}}
async function tick() {{
  let s;
  try {{ s = await (await fetch('/api/state', {{cache: 'no-store'}})).json(); }} catch (e) {{ return; }}
  paint('cli-body', (s.cli || []).slice().reverse().map(function (row) {{
    return '<tr><td>' + clock(row[0]) + '</td><td><code>' + esc(row[1]) + '</code></td>' +
           '<td><pre>' + esc(String(row[2]).trim().slice(0, 400)) + '</pre></td></tr>';
  }}).join(''));
  paint('msp-body', (s.msp || []).slice().reverse().map(function (row) {{
    return '<tr><td>0x' + row[1].toString(16).padStart(4, '0') + '</td><td>' + esc(row[2]) +
           '</td><td>' + row[3] + '</td><td>' + row[4] + '</td><td>' + clock(row[0]) + '</td></tr>';
  }}).join(''));
}}
tick();
setInterval(tick, 2000);
</script>
</html>"""

        # Порт веб-страниц — для человека, поэтому занятый порт не повод падать: ищем
        # следующий свободный и честно печатаем, куда открывать браузер. (Порт MSP так
        # себя вести не может: Configurator подключается строго к указанному адресу.)
        wanted = self.port
        if self.strict_port:
            try:
                self._httpd = ThreadingHTTPServer((self.host, wanted), Handler)
            except OSError as exc:
                raise RuntimeError(f"порт веб-монитора {wanted} занят ({exc}) — проверка остановлена") from exc
            self._prepared = True
            return
        for candidate in range(wanted, wanted + HTTP_PORT_SCAN):
            try:
                self._httpd = ThreadingHTTPServer((self.host, candidate), Handler)
                break
            except OSError:
                continue
        if self._httpd is None:
            self.port = None
            emit(f"[X] порты {wanted}…{wanted + HTTP_PORT_SCAN - 1} заняты — веб-монитор и /tx не подняты")
            emit("    Освободите порт или запустите с другим: python -m bfemu --tcp 127.0.0.1:5761 --http 8188")
        else:
            self.port = candidate
            if candidate != wanted:
                emit(f"[FC] порт {wanted} занят — веб-монитор поднят на {candidate}")
        self._prepared = True

    def serve_forever(self):
        self.prepare()
        if self._httpd:
            self._httpd.serve_forever()

    def stop(self):
        if self._httpd:
            self._httpd.shutdown()


HTTP_PORT_SCAN = 20        # сколько портов подряд пробовать для веб-монитора, если занято

_PORT_FOR_PAGE = {"tcp": 5761}


# --------------------------------------------------------------------------- демо failsafe
# Пояснения «параметр → что делает сейчас»: те же слова, что в failsafe.c/rx.c/gps_rescue.c,
# чтобы студент видел не числа, а роль каждого значения.
_FS_PARAM_HINTS = (
    ("failsafe_delay", "страж-время Этапа 1: сколько ждём перед Этапом 2 (rxDataFailurePeriod)"),
    ("failsafe_off_delay", "AUTO-LAND: за это время газ убирается до нуля"),
    ("failsafe_throttle", "газ в Этапе 2 (после посадки или при падении)"),
    ("failsafe_switch_mode", "что делает тумблер FAILSAFE из вкладки «Режимы»"),
    ("failsafe_throttle_low_delay", "JustDisarm: если газ был ниже min_check дольше — моторы выключаются без процедуры"),
    ("failsafe_procedure", "что именно делает Этап 2"),
    ("failsafe_recovery_delay", "сколько валидных кадров ждём после возврата связи (rxDataRecoveryPeriod)"),
    ("failsafe_stick_threshold", "двинутые стики прерывают GPS-спасение (areSticksActive)"),
)
_FS_RESCUE_HINTS = (
    ("min_sats", "минимум спутников, иначе спасение не считается доступным и срабатывает RESCUE_LOWSATS"),
    ("sanity", "что делать при неудачной проверке: OFF — 20 с спуска, ON — сразу выключить моторы, FS_ONLY — так же, но только при потерянной связи"),
    ("allow_no_fix", "разрешить взвод без GPS-фикса (в проверках это отдельный случай)"),
    ("throttle_hover", "газ висения: середина диапазона газа спасения"),
    ("initial_climb", "сколько метров набрать перед полётом домой, если старт ниже высоты возврата"),
    ("ascend_rate", "скорость подъёма на высоту возврата"),
    ("descend_rate", "скорость снижения перед посадкой"),
    ("return_alt", "на какой высоте идти домой"),
    ("ground_speed", "скорость полёта домой"),
    ("min_start_dist", "ближе этого расстояния до дома спасение сначала отлетает и набирает высоту"),
    ("disarm_threshold", "порог ударного ускорения для выключения моторов при посадке"),
    ("landing_alt", "ниже этой высоты начинается посадка"),
    ("descent_dist", "с какого расстояния до дома начинается снижение"),
    ("alt_mode", "0 MAX_ALT (не ниже высоты возврата), 1 FIXED_ALT, 2 CURRENT_ALT (текущая + подъём)"),
    ("max_angle", "предел наклона на полёте домой"),
    ("use_mag", "использовать магнитометр для курса"),
)


_RC_NAMES = ("крен (roll)", "тангаж (pitch)", "рыск (yaw)", "газ (throttle)")


def _failsafe_api(fc) -> dict:
    """Снимок демо для страницы /failsafe: состояние машины фаз + подписи параметров."""
    fc._motors_step()                      # страница опрашивает эмулятор — двигаем модель
    st = fc.sim_failsafe_state()
    cfg = st["config"]
    rescue = st["rescue_config"]
    flags = fc.arming_disable_flags
    notes = []
    if flags & (1 << 1):
        notes.append("FAILSAFE")
    if flags & (1 << 2):
        notes.append("RX_FAILSAFE")
    if flags & (1 << 17):
        notes.append("MSP")
    if flags & (1 << 25):
        notes.append("ARM_SWITCH")
    switch_names = ("STAGE1 — как потеря сигнала", "KILL — сразу выключить моторы",
                    "STAGE2 — сразу Этап 2")
    values = {
        "failsafe_delay": f"{cfg['delay']:.1f} с ({int(cfg['delay'] * 10)} в прошивке)",
        "failsafe_off_delay": f"{cfg['off_delay']:.1f} с ({int(cfg['off_delay'] * 10)})",
        "failsafe_throttle": f"{cfg['throttle']} мкс",
        "failsafe_switch_mode": switch_names[st["switch_mode"]],
        "failsafe_throttle_low_delay": f"{cfg['throttle_low_delay']:.0f} с "
                                       f"({int(cfg['throttle_low_delay'] * 10)})",
        "failsafe_procedure": st["procedure_name"],
        "failsafe_recovery_delay": f"{cfg['recovery_delay']:.1f} с "
                                   f"({int(cfg['recovery_delay'] * 10)})",
        "failsafe_stick_threshold": f"{cfg['stick_threshold']} %",
    }
    param_rows = [[name, values.get(name, ""), hint] for name, hint in _FS_PARAM_HINTS]
    param_rows.append(["rx.c: MAX_INVALID_PULSE_TIME_MS", "300 мс",
                       "столько борт держит последние значения каналов, пока кадров нет"])
    rescue_values = {
        "min_sats": str(rescue["min_sats"]),
        "sanity": ("OFF", "ON", "FS_ONLY")[rescue["sanity"]],
        "allow_no_fix": "да" if rescue["allow_no_fix"] else "нет",
        "throttle_hover": f"{rescue['throttle_hover']} мкс",
        "initial_climb": f"{rescue['initial_climb']} м",
        "ascend_rate": f"{rescue['ascend_rate']:.1f} м/с",
        "descend_rate": f"{rescue['descend_rate']:.1f} м/с",
        "return_alt": f"{rescue['return_alt']} м",
        "ground_speed": f"{rescue['ground_speed']:.1f} м/с",
        "min_start_dist": f"{rescue['min_start_dist']} м",
        "disarm_threshold": f"{rescue['disarm_threshold']:.1f} м/с²",
        "landing_alt": f"{rescue['landing_alt']} м",
        "descent_dist": f"{rescue['descent_dist']} м",
        "alt_mode": ("MAX_ALT", "FIXED_ALT", "CURRENT_ALT")[rescue["alt_mode"]],
        "max_angle": f"{rescue['max_angle']}°",
        "use_mag": "да" if rescue["use_mag"] else "нет",
    }
    solution = fc.gps_solution()
    st.update({
        "motors": [int(round(v)) for v in fc.motor_disarmed],
        "arming_flags": flags,
        "arming_notes": notes,
        "sats": int(solution["sats"]),
        "channel_names": list(_RC_NAMES) + ["AUX %d" % (i + 1) for i in range(12)],
        "param_rows": param_rows,
        "rescue_param_rows": [[name, rescue_values[name], hint]
                              for name, hint in _FS_RESCUE_HINTS],
    })
    if not isinstance(st.get("rescue"), dict):
        # демо выключено: показываем «спасение в покое», чтобы страница не оставалась пустой
        st["rescue"] = {"phase": 0, "phase_name": "IDLE", "phase_text": "спасение не запущено",
                        "alt_m": 0.0, "dist_m": 0.0, "throttle": st["config"]["throttle"],
                        "note": "", "checks": [], "failure": 0,
                        "failure_name": "RESCUE_HEALTHY", "failure_text": "проверки пройдены"}
    st["rescue"]["min_sats"] = rescue["min_sats"]
    return st


def _tcp_port():
    """Порт MSP, который показываем на страницах. Строкой: подставляется в HTML через replace()."""
    return str(_PORT_FOR_PAGE["tcp"])



# --------------------------------------------------------------------------- COM/последовательный порт
class ComServer:
    r"""Отдаёт виртуальный FC на реальный последовательный порт.

    Windows: пара com0com COM10 <-> COM11, Configurator подключается к COM10,
    эмулятор занимает COM11. Если установлен pyserial — используем его,
    иначе открываем порт как файл (`\\.\COM11`), чего достаточно для байтового моста.

    Linux/macOS: тем же кодом можно занять любой tty (например, /dev/ttyUSB0
    от USB-UART или pty), полезно для отладки.
    """

    def __init__(self, fc: VirtualFC, port_name: str, baudrate: int = 115200, verbose: bool = True):
        self.fc = fc
        self.port_name = port_name
        self.baudrate = baudrate
        self.verbose = verbose
        self._stop = threading.Event()

    def log(self, msg):
        emit(msg)

    def serve_forever(self):
        session = Session(self.fc, verbose=self.verbose, log=self.log)
        try:
            import serial  # pyserial, если есть
        except ImportError:
            serial = None

        if serial is not None:
            with serial.Serial(self.port_name, self.baudrate, timeout=0.2) as port:
                self.log(f"[COM] порт {self.port_name} открыт через pyserial @ {self.baudrate}")
                self._pump(port.read, port.write, session)
            return

        path = self.port_name
        if os.name == "nt" and not path.startswith("\\."):
            path = "\\." + path
        fd = os.open(path, os.O_RDWR | (os.O_NOCTTY if os.name == "posix" else 0))
        self.log(f"[COM] порт {path} открыт напрямую (pyserial не установлен)")
        try:
            if os.name == "posix":
                try:  # без raw-режима терминал будет эхоить и портить кадры
                    import tty
                    tty.setraw(fd)
                except Exception:
                    pass
                import select
                while not self._stop.is_set():
                    ready, _, _ = select.select([fd], [], [], 0.2)
                    if not ready:
                        continue
                    data = os.read(fd, 4096)
                    if not data:
                        continue
                    resp = session.process(data)
                    if resp:
                        os.write(fd, resp)
                    if session.reboot_requested:
                        self._reboot(session)
            else:
                while not self._stop.is_set():
                    data = os.read(fd, 4096)
                    if not data:
                        time.sleep(0.01)
                        continue
                    resp = session.process(data)
                    if resp:
                        os.write(fd, resp)
                    if session.reboot_requested:
                        self._reboot(session)
        finally:
            os.close(fd)

    def _pump(self, read, write, session):
        while not self._stop.is_set():
            try:
                data = read(4096)
            except Exception as exc:  # порт мог закрыться (перезагрузка USB)
                self.log(f"[COM] чтение прервано: {exc}")
                break
            if not data:
                continue
            resp = session.process(data)
            if resp:
                write(resp)
            if session.reboot_requested:
                self._reboot(session)

    def _reboot(self, session):
        session.on_reboot()
        self.log(f"[FC] перезагрузка: пауза {1.5}s (прошивка «загружается»)")
        time.sleep(1.5)

    def stop(self):
        self._stop.set()


# --------------------------------------------------------------------------- CLI
def build_arg_parser():
    p = argparse.ArgumentParser(description="Виртуальный полётный контроллер Betaflight (MSP + CLI) для Betaflight Configurator")
    p.add_argument("--tcp", default="0.0.0.0:5761", help="адрес прослушивания TCP (пусто — не поднимать)")
    p.add_argument("--pty", default=None, help="создать псевдотерминал (Linux/macOS), напр. /tmp/ttyBFEMU")
    p.add_argument("--stdio", action="store_true", help="работать через stdin/stdout")
    p.add_argument("--fc-version", default="4.5.4", help="версия прошивки, которую показываем")
    p.add_argument("--api-version", default="1.46.0", help="версия MSP API")
    p.add_argument("--data", default=None, help="путь к bf_data.json (сгенерирован tools/gen_data.py)")
    p.add_argument("--quiet", action="store_true", help="не логировать MSP-запросы")
    p.add_argument("--boot-delay", type=float, default=1.5, help="сколько секунд «прошивка загружается» после ребута")
    p.add_argument("--http-host", default="127.0.0.1",
                   help="адрес веб-монитора (по умолчанию 127.0.0.1 — только эта машина; "
                        "0.0.0.0 — видно всей сети)")
    p.add_argument("--http", default=None,
                   help="порт веб-страниц: монитор состояния и виртуальная аппаратура /tx, напр. 8088")
    p.add_argument("--com", default=None,
                   help="занять последовательный порт (Windows: COM11 из пары com0com; Linux: /dev/ttyUSB0 или pty)")
    p.add_argument("--com-baud", type=int, default=115200, help="скорость порта для pyserial (по умолчанию 115200)")
    p.add_argument("--log", default=None, help=r"дублировать вывод в файл (напр. logs\bfemu.log)")
    p.add_argument("--eeprom", default=None,
                   help="файл «флеша» (по умолчанию eeprom.json рядом с проектом; 'none' — не сохранять)")
    p.add_argument("--wipe", action="store_true", help="стереть «флеш» перед запуском (полный сброс к заводским)")
    return p


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    if args.log:
        set_log_file(args.log)
    try:
        return _run(args)
    except SystemExit:
        raise
    except Exception:
        import traceback
        emit("[X] Внутренняя ошибка эмулятора:")
        emit(traceback.format_exc())
        emit("    Пришлите этот текст — по нему видно, что дописать.")
        if args.log:
            emit(f"    Лог сохранён: {args.log}")
        return 1


def esc(value) -> str:
    """Экранирование для HTML-страниц монитора.

    В страницы попадают строки, которыми управляет пользователь: имя дрона, вывод CLI,
    имена файлов «флеша». Без экранирования команда вида
    ``set craft_name = <img src=x onerror=...>`` превращается в исполняемую разметку
    у всех, кто открыл монитор.
    """
    return escape(str(value), quote=True)


def _default_eeprom_path() -> Path:
    """Файл «флеша» по умолчанию.

    Запуск из исходников (рядом лежат ``pyproject.toml`` / ``run_windows.bat``) — кладём в корень
    проекта: так привыкли командные файлы, и «флеш» переживает перезапуск на том же месте.
    Установленный пакет (``pip install bfemu``) пишет в текущий каталог: в ``site-packages``
    лезть нельзя — он бывает только для чтения и общий для всех проектов пользователя.
    """
    project_root = Path(__file__).resolve().parent.parent
    from_checkout = (project_root / "pyproject.toml").exists() or (project_root / "run_windows.bat").exists()

    candidates = []
    if from_checkout:
        candidates.append(project_root / "eeprom.json")
    candidates.append(Path.cwd() / "eeprom.json")
    candidates.append(Path(os.environ.get("TEMP") or tempfile.gettempdir()) / "bfemu-eeprom.json")

    for candidate in candidates:
        try:
            candidate.parent.mkdir(parents=True, exist_ok=True)
            probe = candidate.with_suffix(".probe")
            probe.write_text("", encoding="utf-8")
            probe.unlink()
            return candidate
        except OSError:
            continue
    fallback = Path(tempfile.gettempdir()) / "bfemu-eeprom.json"
    emit(f"[FC] каталоги проекта и запуска недоступны для записи — «флеш» будет в {fallback}")
    return fallback


def _run(args):
    # на Windows при перенаправленном выводе русские строки не должны ронять процесс
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except (AttributeError, ValueError):
            pass

    eeprom = None
    if args.eeprom is None:
        eeprom = _default_eeprom_path()
    elif args.eeprom.lower() not in ("none", "off", ""):
        eeprom = Path(args.eeprom).expanduser()

    fc = VirtualFC(
        data_path=args.data,
        fc_version=args.fc_version,
        api_version=args.api_version,
        eeprom_path=eeprom,
        log=emit,
    )
    if args.wipe:
        fc.wipe_eeprom()
        emit("[FC] «флеш» стёрт: стартуем с заводских настроек")
    verbose = not args.quiet
    emit(f"[FC] Betaflight {args.fc_version} / MSP API {args.api_version} / настроек в базе: {len(fc.settings_list)}")
    if eeprom:
        emit(f"[FC] «флеш»: {eeprom} ({'конфигурация загружена' if fc.eeprom_loaded else 'пусто, используются заводские настройки'})")
    else:
        emit("[FC] «флеш» отключён: изменения живут только до выхода")

    servers = []
    if args.tcp:
        host, _, port = args.tcp.rpartition(":")
        tcp = TcpServer(fc, host or "0.0.0.0", int(port), boot_delay=args.boot_delay, verbose=verbose)
        servers.append(tcp)
    if args.pty:
        servers.append(PtyServer(fc, args.pty, verbose=verbose))
    if args.com:
        servers.append(ComServer(fc, args.com, baudrate=args.com_baud, verbose=verbose))
    if args.stdio:
        servers.append(StdioServer(fc, verbose=verbose))
    if args.http:
        _PORT_FOR_PAGE["tcp"] = (args.tcp.rpartition(":")[2] or "5761")
        http = HttpMonitor(fc, int(args.http), host=args.http_host)
        http.prepare()          # порт занимаем до старта потоков — без гонки на первом запросе
        servers.append(http)
        if http.port:
            emit(f"[FC] веб-монитор: http://127.0.0.1:{http.port}/  ·  виртуальная аппаратура "
                 f"(двигать стики): http://127.0.0.1:{http.port}/tx")
            emit(f"[FC] сборка {_BUILD}, страница аппаратуры v{_TX_PAGE_VERSION} "
                 f"(кнопки «Связать (Bind)» / «Развязать» — на /tx)")
    else:
        emit("[FC] веб-монитор и /tx выключены (страницы не поднимаются): добавьте --http 8088,")
        emit("     как это делает run_windows.bat, — тогда работают http://127.0.0.1:8088/ и /tx")
    if not servers:
        emit("нечего запускать: укажите --tcp, --pty, --com или --stdio")
        return 2

    threads = [threading.Thread(target=s.serve_forever, daemon=True) for s in servers]
    for t in threads:
        t.start()
    try:
        while any(t.is_alive() for t in threads):
            time.sleep(0.2)
    except KeyboardInterrupt:
        emit("\n[FC] остановка")
        for s in servers:
            s.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

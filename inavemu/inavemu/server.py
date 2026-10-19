"""
TCP-сервер MSP (как последовательный порт FC) + минимальный веб-монитор.

Configurator подключается по tcp://127.0.0.1:5761 (настольное приложение,
Manual connection). Веб-монитор — просто страница состояния для отладки.
"""
from __future__ import annotations

import json
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from . import msp
from .fc import EQ_KEYS, MSPError, InavFC


class FCServer:
    def __init__(self, fc: InavFC, host: str = "127.0.0.1", port: int = 5761):
        self.fc = fc
        self.host = host
        self.port = port
        self._sock = None
        self._thread = None
        self._stop = threading.Event()
        self.clients = 0
        self.requests = 0

    # ------------------------------------------------------------------
    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.listen(4)
        self._sock.settimeout(0.5)
        self._thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._thread.start()
        print(f"[TCP] виртуальный FC INAV слушает {self.host}:{self.port}")

    def stop(self):
        self._stop.set()
        if self._sock:
            self._sock.close()

    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                conn, addr = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self.clients += 1
            threading.Thread(target=self._client_loop,
                             args=(conn, addr), daemon=True).start()

    @staticmethod
    def _drop_link(conn: socket.socket):
        """Грациозный разрыв: сначала FIN (SHUT_WR), дочитываем входящие
        и только потом close(). Немедленный close() при непрочитанном буфере
        даёт RST — и ответ, ещё не прочитанный клиентом, теряется
        (конфигуратор висит в ожидании колбэка)."""
        try:
            conn.shutdown(socket.SHUT_WR)
        except OSError:
            pass
        conn.settimeout(0.25)
        try:
            while conn.recv(4096):
                pass
        except OSError:
            pass
        try:
            conn.close()
        except OSError:
            pass

    def _client_loop(self, conn: socket.socket, addr):
        print(f"[TCP] подключился {addr[0]}:{addr[1]}")
        from .cli import CliSession
        parser = msp.Parser()
        cli = CliSession(self.fc)
        in_cli = False
        dropped = False
        conn.settimeout(1.0)
        try:
            while not self._stop.is_set():
                try:
                    data = conn.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    break
                if in_cli:
                    out = cli.feed(data)
                    if out:
                        conn.sendall(out)
                    if cli.reboot_requested:
                        self.fc.save_eeprom()
                        self._drop_link(conn)
                        dropped = True
                        print("[TCP] линк разорван (перезагрузка FC)")
                        return
                    continue
                for i, byte in enumerate(data):
                    if byte == 0x23 and not parser._buf:  # '#' — вход в CLI
                        in_cli = True
                        self.fc._log("CLI: режим командной строки")
                        conn.sendall(cli.enter())
                        rest = data[i + 1:]
                        if rest:
                            out = cli.feed(rest)
                            if out:
                                conn.sendall(out)
                            if cli.reboot_requested:
                                self.fc.save_eeprom()
                                self._drop_link(conn)
                                dropped = True
                                print("[TCP] линк разорван (перезагрузка FC)")
                                return
                        break
                    for req in parser.feed(bytes([byte])):
                        self.requests += 1
                        reply = self._dispatch(req)
                        if reply is not None:
                            conn.sendall(reply)
                    if self.fc.reboot_requested:
                        # как настоящий FC: после перезагрузки линк рвётся
                        self.fc.reboot_requested = False
                        self._drop_link(conn)
                        dropped = True
                        print("[TCP] линк разорван (перезагрузка FC)")
                        return
        except OSError:
            pass
        finally:
            if not dropped:
                try:
                    conn.close()
                except OSError:
                    pass
        print(f"[TCP] {addr[0]}:{addr[1]} отключился")

    # опрашиваются конфигуратором фоново — в логе не нужны:
    # MSP_RAW_GPS(106), MSP_ATTITUDE(108), MSP_ACTIVEBOXES(113), MSP_SENSOR_STATUS(151),
    # MSPV2_INAV_STATUS(0x2000), MSPV2_INAV_ANALOG(0x2002)
    _POLL_CODES = {0x2000, 0x2002, 106, 108, 113, 151}

    def _dispatch(self, req: msp.Request) -> bytes | None:
        try:
            payload = self.fc.handle(req)
        except MSPError as exc:
            self.fc._log(f"MSP 0x{req.code:04X}: {exc}")
            if req.code not in self._POLL_CODES:
                print(f"[MSP] 0x{req.code:04X} -> ошибка: {exc}", flush=True)
            return msp.encode(req, b"", error=True)
        except Exception as exc:  # noqa: BLE001 — не ронять линк из-за бага обработчика
            self.fc._log(f"MSP 0x{req.code:04X}: внутренняя ошибка {exc!r}")
            print(f"[MSP] 0x{req.code:04X} -> ВНУТРЕННЯЯ ОШИБКА {exc!r}", flush=True)
            return msp.encode(req, b"", error=True)
        if req.code not in self._POLL_CODES:
            print(f"[MSP] 0x{req.code:04X} ok ({len(payload)} байт)", flush=True)
        return msp.encode(req, payload)

    # ------------------------------------------------------------------
    def state_json(self) -> dict:
        fc = self.fc
        roll, pitch, yaw = fc.attitude()
        return {
            "fc_version": fc.fc_version,
            "api_version": f"{fc.api_major}.{fc.api_minor}.0",
            "board": fc.board,
            "target": fc.target,
            "craft_name": fc.craft_name,
            "features": fc.features,
            "attitude": {"roll": round(roll, 2), "pitch": round(pitch, 2), "yaw": round(yaw, 2)},
            "altitude_cm": round(fc.altitude_cm(), 1),
            "gps": fc.gps,
            "clients": self.clients,
            "requests": self.requests,
            "settings": len(fc.store.settings),
            "equipment": fc.get_equipment(),
            "outputs": {
                "motors": fc.motors_connected,
                "servos": fc.servos_connected,
                "motor_values": list(fc.motors[:8]),
                "servo_values": fc.servo_values(),
            },
            "log": fc.log[-30:],
        }


def make_http_handler(server: FCServer):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):  # не спамить в stderr
            pass

        def _send(self, body: bytes, ctype: str, code: int = 200):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_json(self, obj, code: int = 200):
            self._send(json.dumps(obj, ensure_ascii=False).encode(),
                       "application/json", code)

        def _page(self, kind: str):
            from . import pages
            nav = pages.nav_html(kind, tcp=f"{server.host}:{server.port}")
            style = pages.NAV_STYLE
            page = {"monitor": pages.monitor_page,
                    "equipment": pages.equipment_page,
                    "outputs": pages.outputs_page}[kind]
            self._send(page(nav, style).encode(), "text/html; charset=utf-8")

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/api/state":
                self._send_json(server.state_json())
            elif path == "/api/equipment":
                self._send_json({"equipment": server.fc.get_equipment()})
            elif path in ("/", "/index.html"):
                self._page("monitor")
            elif path == "/equipment":
                self._page("equipment")
            elif path == "/outputs":
                self._page("outputs")
            else:
                self._send(b"not found", "text/plain", 404)

        def do_POST(self):
            path = self.path.split("?", 1)[0]
            if path == "/api/equipment":
                self._post_equipment()
            elif path == "/api/outputs":
                self._post_outputs()
            else:
                self._send(b"not found", "text/plain", 404)

        def _read_json(self):
            length = int(self.headers.get("Content-Length", "0"))
            payload = json.loads(self.rfile.read(length) or b"{}")
            if not isinstance(payload, dict) or not payload:
                raise ValueError("ожидался JSON-объект")
            return payload

        def _post_outputs(self):
            try:
                payload = self._read_json()
                for name, val in payload.items():
                    if name not in ("motors", "servos"):
                        raise ValueError(f"неизвестное поле {name!r}")
                    if isinstance(val, bool) or not isinstance(val, int):
                        raise ValueError(f"{name}: ожидается целое число")
                for name, val in payload.items():
                    try:
                        if name == "motors":
                            server.fc.set_motors_connected(val)
                        else:
                            server.fc.set_servos_connected(val)
                    except MSPError as exc:
                        raise ValueError(str(exc))
            except (ValueError, json.JSONDecodeError) as exc:
                self._send_json({"error": str(exc)}, 400)
                return
            self._send_json({"outputs": server.state_json()["outputs"]})

        def _post_equipment(self):
            try:
                payload = self._read_json()
                for name, on in payload.items():
                    if name not in EQ_KEYS:
                        raise ValueError(f"неизвестное оборудование {name!r}")
                    if not isinstance(on, bool):
                        raise ValueError(f"{name}: ожидается true/false")
                for name, on in payload.items():
                    server.fc.set_equipment(name, on)
            except (ValueError, json.JSONDecodeError) as exc:
                self._send_json({"error": str(exc)}, 400)
                return
            self._send_json({"equipment": server.fc.get_equipment()})

    return Handler


class MonitorServer:
    def __init__(self, fc_server: FCServer, host: str = "127.0.0.1", port: int = 8088):
        self.httpd = ThreadingHTTPServer((host, port), make_http_handler(fc_server))
        self.host = host
        self.port = self.httpd.server_address[1]

    def start(self):
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        print(f"[HTTP] веб-монитор: http://{self.host}:{self.port}/")

    def stop(self):
        self.httpd.shutdown()

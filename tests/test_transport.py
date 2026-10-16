"""Транспорт TCP: окно загрузки после ребута и буфер вывода CLI у каждого соединения.

Повторное ревью, часть 4 нашло здесь две вещи: `--boot-delay` только писал строку в журнал
(новое подключение получало MSP-ответ через 0,001 с вместо паузы), а вывод CLI лежал в одном
поле FC — вторая вкладка CLI подменяла страницы первой. Здесь обе проверяются «по проводу»:
живой TcpServer, настоящие MSP-кадры.
"""
from __future__ import annotations

import contextlib
import socket
import struct
import threading
import time

from bfemu.fc import MSP_REBOOT, MSP_STATUS, MSP2_CLI_COMMAND, MSP2_CLI_SETTING, MSP2_CLI_SETTING_INFO
from bfemu.msp import MspParser, encode_v1, encode_v2
from bfemu.server import TcpServer

CLI_WINDOW = 480  # столько байт отдаёт MSP2_CLI_COMMAND за одну страницу (fc_identity.py)


def free_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


@contextlib.contextmanager
def running_server(fc, boot_delay: float, slack: float | None = None):
    """TcpServer в отдельном потоке: тесты не пишут в консоль и всегда его останавливают."""
    port = free_port()
    server = TcpServer(fc, "127.0.0.1", port, boot_delay=boot_delay, verbose=False)
    if slack is not None:
        server.boot_wait_slack = slack
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    for _ in range(200):                       # ждём, пока порт начнёт приниматься
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.5):
                break
        except OSError:
            time.sleep(0.02)
    else:
        raise AssertionError("TcpServer не поднялся за 4 с")
    try:
        yield server, port
    finally:
        server.stop()
        thread.join(timeout=5)


def recv_frames(conn: socket.socket, code: int, timeout: float = 5.0, want: int = 1):
    """Прочитать с соединения кадры MSP и вернуть payload'ы с нужным кодом.

    Ждём ровно `want` кадров: иначе измерение «сколько ждал клиент» растягивалось бы
    на весь таймаут ожидания лишних кадров.
    """
    parser = MspParser()
    out = []
    deadline = time.monotonic() + timeout
    while len(out) < want and time.monotonic() < deadline:
        conn.settimeout(max(0.05, deadline - time.monotonic()))
        try:
            data = conn.recv(65536)
        except TimeoutError:
            break
        if not data:
            break
        parser.feed(data)
        for request in parser:
            if request.code == code:
                out.append(request.payload)
    return out


def one_frame(conn: socket.socket, code: int, timeout: float = 5.0):
    """Один ответ MSP с нужным кодом: (третий байт кадра, payload).

    Третий байт — «>» для обычного ответа и «!» для отказа (MSP_RESULT_ERROR,
    msp_serial.c). Хелпер `recv_frames` этот байт теряет, потому что им пользуются
    тесты, где отказов быть не должно.
    """
    parser = MspParser()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        conn.settimeout(max(0.05, deadline - time.monotonic()))
        try:
            data = conn.recv(65536)
        except TimeoutError:
            break
        if not data:
            break
        parser.feed(data)
        for request in parser:
            if request.code == code:
                return chr(request.direction), request.payload
    raise AssertionError(f"нет ответа на MSP {code:#06x}")


def cli_page(conn: socket.socket, payload: bytes):
    """Запрос MSP2_CLI_COMMAND: (полная длина вывода, окно страницы)."""
    conn.sendall(encode_v2(MSP2_CLI_COMMAND, payload))
    frames = recv_frames(conn, MSP2_CLI_COMMAND)
    assert frames, "нет ответа на MSP2_CLI_COMMAND"
    page = frames[-1]
    total = struct.unpack_from("<H", page, 0)[0]
    assert page[2] == 0, "за ответом CLI идёт байт-заглушка"
    return total, page[3:]


def test_connection_without_reboot_is_answered_immediately(fc):
    """Пауза загрузки не должна тормозить обычные подключения: она относится только к ребуту.

    Так выглядела бы ошибка, если бы окно загрузки выставлялось при каждом старте сервера
    или ждалось на любом соединении: Configurator «подключается» на секунды дольше.
    """
    with running_server(fc, boot_delay=1.0) as (_, port):
        with socket.create_connection(("127.0.0.1", port), timeout=5) as conn:
            started = time.monotonic()
            conn.sendall(encode_v1(MSP_STATUS, b""))
            assert recv_frames(conn, MSP_STATUS), "MSP_STATUS без ответа"
            waited = time.monotonic() - started
    assert waited < 0.4, f"подключение без ребута ждало {waited:.3f} с"


def test_connection_after_reboot_waits_out_boot_pause(fc):
    """`--boot-delay 3` раньше не соблюдался на TCP: ответ приходил за 0,001 с.

    Теперь линк закрывается после ответа на MSP_SET_REBOOT, а следующее подключение ждёт
    окончания «загрузки» — как настоящий борт, который на это время отключает USB.
    """
    delay = 0.6
    with running_server(fc, boot_delay=delay) as (_, port):
        with socket.create_connection(("127.0.0.1", port), timeout=5) as first:
            first.sendall(encode_v1(MSP_REBOOT, b"\x00"))
            assert recv_frames(first, MSP_REBOOT) == [b"\x00"]
            first.settimeout(5)
            while first.recv(4096):            # прошивка разрывает линк на перезагрузку
                pass
        with socket.create_connection(("127.0.0.1", port), timeout=5) as second:
            started = time.monotonic()
            second.sendall(encode_v1(MSP_STATUS, b""))
            payloads = recv_frames(second, MSP_STATUS)
            waited = time.monotonic() - started
    assert payloads and len(payloads[0]) == 13, "после загрузки MSP_STATUS отвечает как обычно"
    assert waited > delay * 0.5, f"новое подключение не ждало загрузку: {waited:.3f} с"


def test_stale_reboot_flag_does_not_hang_the_connection(fc):
    """Если перезагрузку никто не завершит (упавший поток), подключение обязано ожить.

    Иначе `pending_reboot` оставлял бы эмулятор «мёртвым» навсегда: клиент ждёт ответа,
    а сервер — события, которого уже не будет.
    """
    fc.pending_reboot = True                   # «ребут» без потока, который его добьёт
    with running_server(fc, boot_delay=0.05, slack=0.2) as (_, port):
        with socket.create_connection(("127.0.0.1", port), timeout=5) as conn:
            started = time.monotonic()
            conn.sendall(encode_v1(MSP_STATUS, b""))
            payloads = recv_frames(conn, MSP_STATUS, timeout=3)
            waited = time.monotonic() - started
    assert payloads, "соединение осталось без ответа"
    assert waited < 1.0, f"ожидание затянулось на {waited:.3f} с вместо держёжки"


def test_cli_paging_buffer_is_per_connection(fc):
    """Постраничное чтение CLI не должно путать соединения (был общий буфер `fc._cli_out`).

    Соединение A читает `dump` двумя страницами, соединение B в это время выполняет свою
    команду. Вторая страница A обязана быть продолжением ЕЁ вывода, а не чужого.
    """
    full = fc.cli_execute("dump").encode("latin-1")
    assert len(full) > 3 * CLI_WINDOW, "для проверки пейджинга вывод должен быть длинным"
    with running_server(fc, boot_delay=0.0) as (_, port):
        with socket.create_connection(("127.0.0.1", port), timeout=5) as page_reader, \
                socket.create_connection(("127.0.0.1", port), timeout=5) as other:
            total, first = cli_page(page_reader, b"dump")
            assert total == len(full) and first == full[:CLI_WINDOW]
            other_total, _ = cli_page(other, b"get craft_name")   # короткий чужой вывод
            assert other_total != total
            same_total, second = cli_page(page_reader, b"dump\x00" + struct.pack("<H", CLI_WINDOW))
    assert same_total == total, "номер страницы относится к буферу своего соединения"
    assert second == full[CLI_WINDOW : 2 * CLI_WINDOW]
    assert fc.cli_output() == "", "общего буфера вывода на FC больше нет"


def test_refused_setting_write_is_an_error_frame(fc):
    """MSP2_CLI_SETTING отвечает отказом кадром «!» — как MSP_RESULT_ERROR в msp.c мастера.

    Раньше эмулятор на отказ отдавал пустой успешный ответ, а Configurator считает пустое
    эхо удачной записью: недопустимое значение выглядело применённым (useMspSetting.ts
    бросает исключение только на `unsupported`, то есть на кадр-отказ).
    """
    with running_server(fc, boot_delay=0.0) as (_, port):
        with socket.create_connection(("127.0.0.1", port), timeout=5) as conn:
            conn.sendall(encode_v2(MSP2_CLI_SETTING, b"gyro_lpf1_static_hz = 320"))
            assert one_frame(conn, MSP2_CLI_SETTING) == (">", b"gyro_lpf1_static_hz = 320")

            for request, note in [
                (b"gyro_lpf1_static_hz = 999999", "значение вне диапазона"),
                (b"gyro_lpf1_type = 1", "номер вместо имени списка"),
                (b"no_such_setting = 1", "неизвестная настройка"),
                (b"no_such_setting", "чтение неизвестной настройки"),
            ]:
                conn.sendall(encode_v2(MSP2_CLI_SETTING, request))
                assert one_frame(conn, MSP2_CLI_SETTING) == ("!", b""), note

            conn.sendall(encode_v2(MSP2_CLI_SETTING_INFO, b"gyro_lpf1_static_hz\x00\x00\x00"))
            direction, payload = one_frame(conn, MSP2_CLI_SETTING_INFO)
            assert direction == ">" and b"max=1000" in payload
            conn.sendall(encode_v2(MSP2_CLI_SETTING_INFO, b"no_such_setting"))
            assert one_frame(conn, MSP2_CLI_SETTING_INFO) == ("!", b"")
    assert fc.settings["gyro_lpf1_static_hz"] == 320, "отказы не меняют значение"

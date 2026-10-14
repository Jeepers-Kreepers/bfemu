#!/usr/bin/env python3
"""Строковые параметры CLI: 'set craft_name=…', 'set pilot_name=…', профили, «флеш».

Проверяем ровно то, что делает человек в командной строке:
  * строка принимается и видна в 'get' и в 'dump' ('# name: …');
  * длиннее 16 символов и пустая — отказ, как в прошивке;
  * '-' стирает имя (в CLI показывается как '-');
  * Configurator (MSP_SET_NAME / MSP2_SET_TEXT) и CLI видят одно и то же поле;
  * 'save' → ребут → имя на месте (лежит в «флеше» рядом с остальными настройками);
  * 'defaults' возвращает заводское имя.

Запуск:  python3 tools/check_craft_name.py
"""
from __future__ import annotations

import json
import shutil
import socket
import struct
import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bfemu.fc import VirtualFC, MSP2_GET_TEXT, MSP2_SET_TEXT, MSP_SET_NAME
from bfemu.msp import MspParser, encode_v1, encode_v2
from bfemu.server import TcpServer

PORT = 15791

ok_count = 0
fail_count = 0


def check(cond, msg):
    global ok_count, fail_count
    if cond:
        ok_count += 1
        print(f"   [OK ] {msg}")
    else:
        fail_count += 1
        print(f"   [FAIL] {msg}")


class Client:
    """Мини-клиент MSP: то же, что делает Configurator по TCP."""

    def __init__(self, port=PORT, timeout=3.0):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=timeout)
        self.sock.settimeout(timeout)
        self.parser = MspParser()

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass

    def send_raw(self, data: bytes):
        self.sock.sendall(data)

    def request(self, code: int, payload: bytes = b"", version: int = 1) -> bytes:
        frame = encode_v1(code, payload) if version == 1 else encode_v2(code, payload)
        self.sock.sendall(frame)
        deadline = time.time() + 3
        while time.time() < deadline:
            try:
                data = self.sock.recv(65536)
            except TimeoutError:
                break
            if not data:
                break
            self.parser.feed(data)
            for req in self.parser:
                if req.code == code:
                    return req.payload
        raise TimeoutError(f"нет ответа на MSP {code}")

    def command(self, text: str) -> str:
        """Строка в интерактивный CLI (окно Configurator): ждём приглашение '#'."""
        self.sock.sendall(text.encode() + b"\r\n")
        out = b""
        self.sock.settimeout(1.5)
        for _ in range(3):
            try:
                data = self.sock.recv(65536)
            except TimeoutError:
                break
            if not data:
                break
            out += data
            if out.rstrip().endswith(b"#"):
                break
        return out.decode("latin-1", "replace")


def cli_over_msp(cli: Client, command: str) -> str:
    """Аналог MSP.send_cli_command — этим путём Configurator гоняет CLI."""
    out = b""
    offset = 0
    while True:
        payload = command.encode() if offset == 0 else command.encode() + b"\x00" + struct.pack("<H", offset)
        resp = cli.request(0x3012, payload, version=2)
        total = int.from_bytes(resp[0:2], "little")
        out += resp[3:]
        offset += len(resp) - 3
        if offset >= total or len(resp) - 3 == 0:
            break
    return out.decode("latin-1", "replace")


def get_text(data: bytes, pos: int = 0):
    size = data[pos]
    pos += 1
    return data[pos:pos + size].decode("latin-1"), pos + size


def main() -> int:
    workdir = Path(tempfile.mkdtemp(prefix="bfemu-name-"))
    eeprom = workdir / "eeprom.json"
    fc = VirtualFC(eeprom_path=eeprom)
    server = TcpServer(fc, "127.0.0.1", PORT, boot_delay=0.3, verbose=False)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    time.sleep(0.6)
    try:
        msp = Client()          # канал MSP: так работает вкладка «Конфигурация»
        cli = Client()          # окно CLI Configurator: интерактивный режим
        cli.send_raw(b"#")
        time.sleep(0.5)
        cli.sock.recv(65536)

        default_dump = cli_over_msp(msp, "dump")
        check("# name: EMU-QUAD" in default_dump, "заводской дамп: '# name: EMU-QUAD'")

        print("\n1. Строка принимается (тот самый случай «set craft_name=qqq»)")
        answer = cli.command("set craft_name=qqq")
        check("###ERROR" not in answer, "set craft_name=qqq прошёл без '###ERROR IN set'")
        answer = cli.command("get craft_name")
        check("craft_name = qqq" in answer, "get craft_name показывает qqq")
        check("String length: 1 - 16" in answer, "get подсказывает длину строки: 1 - 16 символов")

        print("\n2. То же видно в dump и в MSP (Configurator и CLI — одно поле)")
        dump = cli_over_msp(msp, "dump")
        check("# name: qqq" in dump, "dump: строка '# name: qqq'")
        check("set craft_name = qqq" in dump, "dump: 'set craft_name = qqq' (уедет в файл конфигурации)")
        name, _ = get_text(msp.request(MSP2_GET_TEXT, bytes([2]), version=2), 1)
        check(name == "qqq", f"MSP2_GET_TEXT (вкладка «Конфигурация») отдаёт {name!r}")

        print("\n3. Границы как в прошивке")
        answer = cli.command("set craft_name=0123456789ABCDEFG")          # 17 символов
        check("STRING MUST BE 1-16 CHARACTERS OR '-' FOR EMPTY" in answer,
              "17 символов: 'STRING MUST BE 1-16 CHARACTERS OR '-' FOR EMPTY'")
        answer = cli.command("set craft_name=")
        check("STRING MUST BE 1-16 CHARACTERS" in answer, "пустое значение: тот же отказ")
        answer = cli.command("set craft_name=0123456789ABCDEF")           # ровно 16
        check("###ERROR" not in answer and "craft_name = 0123456789ABCDEF" in cli.command("get craft_name"),
              "16 символов принимается (ровно предел)")
        cli.command('set craft_name="qq"')      # кавычки — обычные символы, прошивка их не снимает
        check('craft_name = "qq"' in cli.command("get craft_name"),
              "кавычки не снимаются: значение сохраняется как есть (как cliSet)")
        answer = cli.command("set craft_name=-")
        check("###ERROR" not in answer and "craft_name = -" in cli.command("get craft_name"),
              "'-' стирает имя (в CLI показывается как '-')")
        cli.command("set craft_name=qqq")

        print("\n4. Другие строки: пилот и имена профилей")
        cli.command("set pilot_name=vasya")
        check("pilot_name = vasya" in cli.command("get pilot_name"), "pilot_name принимается")
        answer = cli.command("set profile_name=abcdefghij")               # предел 8
        check("STRING MUST BE 1-8 CHARACTERS" in answer, "profile_name длиннее 8: свой предел (8 символов)")
        cli.command("set profile_name=race")
        check("profile_name = race" in cli.command("get profile_name"), "profile_name=race принимается")

        print("\n5. Configurator пишет имя — CLI видит то же")
        msp.request(MSP2_SET_TEXT, bytes([2, 4]) + b"tab1", version=2)     # TEXT_CRAFT_NAME = 2
        check("craft_name = tab1" in cli.command("get craft_name"),
              "MSP2_SET_TEXT: имя из вкладки появилось в 'get craft_name'")
        msp.request(MSP_SET_NAME, b"tab2")                                # устаревший, но вкладки ещё пишут
        check("craft_name = tab2" in cli.command("get craft_name"),
              "MSP_SET_NAME: тоже пишет в то же поле")
        cli.command("set craft_name=qqq")

        print("\n6. 'save' → ребут: имя переживает перезапуск")
        msp.close()
        cli.send_raw(b"save\r\n")
        time.sleep(3.0)
        cli.close()
        check(eeprom.exists(), "конфигурация записана в «флеш» (eeprom.json)")
        saved = json.loads(eeprom.read_text(encoding="utf-8"))
        check(saved["settings"]["craft_name"] == "qqq", "в «флеше» лежит craft_name = qqq")
        fc2 = VirtualFC(eeprom_path=eeprom)
        check(fc2.settings["craft_name"] == "qqq", "после перезапуска настройка на месте")
        check(fc2.craft_name == "qqq", "после перезапуска MSP-имя тоже qqq (поля не разошлись)")
        check(fc2.pilot_name == "vasya", "имя пилота тоже пережило перезапуск")

        print("\n7. 'defaults' возвращает заводское имя")
        fc2.reset_to_defaults()
        check(fc2.settings["craft_name"] == "EMU-QUAD" and fc2.craft_name == "EMU-QUAD",
              "после 'defaults': craft_name = EMU-QUAD")
        check(fc2.settings["pilot_name"] == "VIRTUAL PILOT", "после 'defaults': pilot_name = VIRTUAL PILOT")
        check(fc2.format_setting(fc2.settings_by_name["craft_name"]) == "EMU-QUAD",
              "печать настройки идёт строкой, а не числом")

        print()
        if fail_count:
            print(f"ПРОВАЛЕНО проверок: {fail_count}")
            return 1
        print(f"СТРОКОВЫЕ ПАРАМЕТРЫ CLI РАБОТАЮТ: {ok_count} проверок, 0 провалов")
        return 0
    finally:
        server.stop()
        shutil.rmtree(workdir, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())

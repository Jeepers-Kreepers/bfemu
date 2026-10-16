#!/usr/bin/env python3
"""Проверка вкладки «Режимы» (диапазоны AUX): строки на вкладке, живое состояние, CLI, «флеш».

Скрипт повторяет то, что делает вкладка Configurator 2026.6.2 (AuxiliaryTab.vue +
js/utils/modeRanges.js):

  loadData(): MSP_BOXNAMES(116) → MSP_MODE_RANGES(34) → MSP_MODE_RANGES_EXTRA(238) →
              MSP_BOXIDS(119) → MSP_RSSI_CONFIG(50) → MSP_RC(105) → MSP2_COMMON_SERIAL_CONFIG;
  «Добавить диапазон» → MSP_SET_MODE_RANGE(35): index, permanentId, auxChannelIndex,
              startStep, endStep, modeLogic, linkedTo (семь байт — sendModeRanges);
  Save:       MSP_SET_MODE_RANGE на каждый слот → MSP_EEPROM_WRITE(250) → MSP_SET_REBOOT(68).

Правила — из исходников прошивки 4.5.4:

  * MSP_MODE_RANGES отвечает ВСЕМИ 20 слотами (msp.c: цикл по
    MAX_MODE_ACTIVATION_CONDITION_COUNT = 20), 4 байта на слот: permanentId, AUX,
    startStep, endStep. Пустой слот — «0 0 0 0» (режим ARM, 900/900);
  * MSP_MODE_RANGES_EXTRA — count = 20 и по 3 байта: permanentId, modeLogic, linkedTo.
    Без него вкладка не знает логику И/ИЛИ и связи режимов;
  * диапазон живёт шагами по 25 мкс: MODE_STEP_TO_CHANNEL_VALUE(step) = 900 + 25 * step,
    IS_RANGE_USABLE(range) = startStep < endStep (rc_modes.h);
  * строку вкладка рисует, только если start < end (modeRanges.js: entriesFromModeRanges):
    слот «900/900» — заполнитель, а не строка. Поэтому свежий борт выглядит пустым;
  * CLI печатает 20 строк «aux i permId aux MIN MAX logic linkedTo» (printAux в cli.c);
  * режим включён, когда канал попадает в диапазон: 900 + 25 * startStep <= канал <
    900 + 25 * endStep, канал берётся из rcData[auxChannelIndex + 4]; логика И/ИЛИ,
    связи и «залипающий» PARALYZE — updateActivatedModes() из rc_modes.c.

Запуск:  python3 tools/check_modes_tab.py      (Windows: python tools\\check_modes_tab.py)
"""
from __future__ import annotations

import json
import socket
import sys
import tempfile
import threading
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bfemu.fc import VirtualFC
from bfemu.msp import MspParser, encode_v1, encode_v2
from bfemu.server import HttpMonitor, TcpServer

PORT = 15821
PORT2 = 15822
HTTP_PORT = 8094

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
    def __init__(self, port=PORT, timeout=3.0):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=timeout)
        self.sock.settimeout(timeout)
        self.parser = MspParser()

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass

    def request(self, code: int, payload: bytes = b"", version: int = 1) -> bytes:
        frame = encode_v1(code, payload) if version == 1 else encode_v2(code, payload)
        self.sock.sendall(frame)
        deadline = time.time() + 3
        while time.time() < deadline:
            try:
                chunk = self.sock.recv(65536)
            except TimeoutError:
                continue
            if not chunk:
                break
            self.parser.feed(chunk)
            for reply in list(self.parser):
                if reply.code == code:
                    return reply.payload
        return b""

    def cli(self, command: str) -> str:
        """CLI через MSP2_CLI_COMMAND (0x3012) — тем же путём, что шлёт Configurator."""
        out, offset = b"", 0
        while True:
            payload = command.encode() if offset == 0 else command.encode() + b"\x00" + offset.to_bytes(2, "little")
            resp = self.request(0x3012, payload, version=2)
            if len(resp) < 3:
                break
            out += resp[3:]
            offset += len(resp) - 3
            if offset >= int.from_bytes(resp[0:2], "little") or len(resp) == 3:
                break
        return out.decode("latin-1", "replace").replace("\r\n", "\n")


# ------------------------------------------------------------------ разбор ответов вкладки
def read_ranges(cli: Client) -> list:
    """MSP_MODE_RANGES глазами вкладки (modeRanges.js): id, AUX, start/end в микросекундах."""
    data = cli.request(34)
    ranges = []
    for i in range(len(data) // 4):
        perm, aux, start_step, end_step = data[i * 4 : i * 4 + 4]
        ranges.append({"perm": perm, "aux": aux,
                       "start": 900 + start_step * 25, "end": 900 + end_step * 25,
                       "has_range": start_step < end_step})
    return ranges


def read_extra(cli: Client) -> list:
    data = cli.request(238)
    if not data:
        return []
    count = data[0]
    return [{"perm": data[1 + i * 3], "logic": data[2 + i * 3], "linked": data[3 + i * 3]}
            for i in range(count)]


def visible_rows(cli: Client, ranges=None, extra=None) -> list:
    """Строки, которые нарисует вкладка: режим есть в BOXNAMES и записан диапазон или связь.

    Вкладка перебирает список режимов из MSP_BOXNAMES/BOXIDS и раскладывает по ним записи
    из MSP_MODE_RANGES, поэтому строка для режима, которого в списке нет (например LEDLOW
    при выключенной фиче), не появится — как на железной плате.
    """
    ranges = ranges if ranges is not None else read_ranges(cli)
    extra = extra if extra is not None else read_extra(cli)
    listed = set(cli.request(119))
    rows = []
    for index, rng in enumerate(ranges):
        if rng["perm"] not in listed:
            continue
        info = extra[index] if index < len(extra) else {"logic": 0, "linked": 0}
        if info.get("linked"):
            rows.append({"index": index, "perm": rng["perm"], "kind": "link",
                         "linked": info["linked"], "logic": info["logic"]})
        elif rng["has_range"]:
            rows.append({"index": index, "perm": rng["perm"], "kind": "range", "aux": rng["aux"],
                         "start": rng["start"], "end": rng["end"], "logic": info.get("logic", 0)})
    return rows


def set_slot(cli: Client, index, perm, aux, start, end, logic=0, linked=0):
    """sendModeRanges(): ровно такой кадр шлёт вкладка при «Сохранить»."""
    cli.request(35, bytes([index, perm, aux, (start - 900) // 25, (end - 900) // 25, logic, linked]))


def mode_mask(cli: Client) -> int:
    """MSP_STATUS → CONFIG.mode: по этим битам вкладка рисует «включён / выключен»."""
    return int.from_bytes(cli.request(101)[6:10], "little")


def aux_lines(text: str) -> list:
    return [line.strip() for line in text.splitlines() if line.strip().startswith("aux ")]


def start_emu(fc, port, http_port):
    server = TcpServer(fc, "127.0.0.1", port, boot_delay=0.3, verbose=False)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    monitor = HttpMonitor(fc, http_port, strict_port=True)
    monitor.prepare()
    threading.Thread(target=monitor.serve_forever, daemon=True).start()
    return server, monitor


def main() -> int:
    workdir = Path(tempfile.mkdtemp(prefix="bfemu-modes-"))
    eeprom = workdir / "eeprom.json"
    fc = VirtualFC(eeprom_path=eeprom)
    start_emu(fc, PORT, HTTP_PORT)
    time.sleep(0.6)
    cli = Client()
    arm, angle, horizon = (fc.box_permanent_id(n) for n in ("ARM", "ANGLE", "HORIZON"))

    try:
        print("1. Загрузка вкладки (порядок и раскладки как в AuxiliaryTab.vue)")
        names = [n for n in cli.request(116).decode("latin-1").split(";") if n]
        ids = list(cli.request(119))
        ranges = read_ranges(cli)
        extra = read_extra(cli)
        rc = cli.request(105)
        check(len(names) == len(ids) == len(fc.active_boxes),
              f"MSP_BOXNAMES: {len(names)} режимов + MSP_BOXIDS: {len(ids)} — столько строк вкладка и строит")
        check(names[0] == "ARM" and ids[0] == 0, f"первый режим {names[0]!r}, permanent id {ids[0]}")
        check(ids == [fc.box_permanent_id(n) for n in names] and names == fc.active_boxes,
              "порядок режимов и их permanentId совпадают с initActiveBoxIds(): "
              + ", ".join(f"{n}={i}" for n, i in list(zip(names, ids, strict=False))[:4]) + ", …")
        check(all(m in names for m in ("BEEPER", "TELEMETRY", "OSD DISABLE", "VTX PIT MODE", "PREARM"))
              and not any(m in names for m in ("MAG", "LEDLOW", "AIR MODE", "PID AUDIO")),
              "список собран по фичам и датчикам: BEEPER/TELEMETRY/OSD DISABLE есть, "
              "MAG/LEDLOW/AIR MODE (выключены) — нет")
        check(len(ranges) == 20 and len(cli.request(34)) == 80,
              f"MSP_MODE_RANGES: {len(ranges)} слотов × 4 байта = {len(cli.request(34))} байт (прошивка шлёт все 20)")
        check(len(extra) == 20 and len(cli.request(238)) == 61,
              f"MSP_MODE_RANGES_EXTRA: {len(cli.request(238))} байт — count 20 и по 3 байта на слот")
        check(all(not r["has_range"] for r in ranges) and not visible_rows(cli, ranges, extra),
              "свежий борт: ни одного диапазона — вкладка «Режимы» пустая, как после defaults")
        check(all(r["perm"] == 0 for r in ranges),
              "пустые слоты отдаются как режим ARM (0 0 0 0) — нулевая modeActivationCondition_t")
        check(len(rc) // 2 == 16, f"MSP_RC: {len(rc) // 2} каналов → вкладка берёт 12 AUX (активные минус 4)")
        check(len(cli.request(50)) == 1, "MSP_RSSI_CONFIG: 1 байт (вкладка читает канал RSSI)")

        print("\n2. «Добавить диапазон»: ARM на AUX1, 1300…1700 (кадр вкладки — семь байт)")
        set_slot(cli, 0, arm, 0, 1300, 1700)
        rows = visible_rows(cli)
        check(len(rows) == 1 and rows[0]["kind"] == "range" and rows[0]["index"] == 0,
              f"вкладка видит строку: слот 0, {rows[0]['start']}…{rows[0]['end']}" if rows else "строки нет")
        check(rows and rows[0]["aux"] == 0, "канал AUX1 (индекс 0) — как выбрала вкладка")
        check(read_ranges(cli)[1]["has_range"] is False,
              "остальные 19 слотов остались заполнителями — вкладка рисует только одну строку")

        print("\n3. Второй режим и связь: ANGLE на AUX1, HORIZON — «Связь» с ANGLE")
        set_slot(cli, 1, angle, 0, 1300, 1700)
        set_slot(cli, 2, horizon, 0, 900, 900, logic=0, linked=angle)
        rows = visible_rows(cli)
        check(len(rows) == 3 and [r["kind"] for r in rows] == ["range", "range", "link"],
              f"на вкладке три строки: {[(r['index'], r['kind']) for r in rows]}")
        check(rows[2]["linked"] == angle,
              f"«Связь» хранит linkedTo = ANGLE (permanent id {angle}) и читается из 238")

        print("\n4. Логика И / ИЛИ (вкладка показывает выпадающий список у каждой строки)")
        set_slot(cli, 0, arm, 0, 1300, 1700, logic=0)
        set_slot(cli, 1, angle, 1, 1300, 1700, logic=1)
        extra = read_extra(cli)
        check(extra[0]["logic"] == 0 and extra[1]["logic"] == 1,
              "ARM — «ИЛИ», ANGLE — «И»: вкладка читает логику из MSP_MODE_RANGES_EXTRA")

        print("\n5. Живое состояние: тумблеры двигают режимами (rc_modes.c, isRangeActive)")
        fc.set_rc_channel("aux1", 1000)
        fc.set_rc_channel("aux2", 1500)
        check("ARM" not in fc.active_mode_names(),
              "AUX1 = 1000 → ARM выключен (вне 1300…1700), точка на вкладке серая")
        fc.set_rc_channel("aux1", 1500)
        check("ARM" in fc.active_mode_names(),
              f"AUX1 = 1500 → ARM включился (тумблер внутри диапазона): {sorted(fc.active_mode_names())}")
        check(fc.active_mode_names() == {"ARM", "ANGLE", "HORIZON"},
              f"AUX2 = 1500 → к ARM добавились ANGLE (логика И) и HORIZON (связь за ANGLE): "
              f"{sorted(fc.active_mode_names())}")
        fc.set_rc_channel("aux2", 1000)
        check(fc.active_mode_names() == {"ARM"},
              "AUX2 обратно вниз → ANGLE и HORIZON выключились, ARM остался")

        print("\n6. То же состояние доходит до вкладки: MSP_STATUS → CONFIG.mode")
        fc.set_rc_channel("aux1", 1500)
        fc.set_rc_channel("aux2", 1500)
        mask = mode_mask(cli)
        check(mask == fc.rc_mode_mask() != 0,
              f"маска режимов в MSP_STATUS: 0x{mask:08x} (биты — позиции режимов в списке вкладки)")
        check(mask & 0b1 and mask & 0b10 and mask & 0b100,
              f"подняты биты ARM, ANGLE и HORIZON (0x{mask:08x})")
        fc.set_rc_channel("aux1", 1800)   # тумблер переведён за границу диапазона
        check(mode_mask(cli) & 0b1 == 0,
              f"AUX1 = 1800 (вне 1300…1700) → точка ARM на вкладке гаснет (0x{mode_mask(cli):08x})")
        fc.set_rc_channel("aux1", 1000)
        fc.set_rc_channel("aux2", 1000)
        check(mode_mask(cli) == 0, "оба тумблера вниз → вкладка показывает все режимы выключенными")

        print("\n7. Тумблеры аппаратуры: /api/rc (страница /tx) двигают режимами")
        def tx_post(payload):
            body = json.dumps(payload).encode()
            request = urllib.request.Request(f"http://127.0.0.1:{HTTP_PORT}/api/rc", data=body,
                                            headers={"Content-Type": "application/json"}, method="POST")
            with urllib.request.urlopen(request, timeout=5) as answer:
                return json.load(answer)

        reply = tx_post({"aux1": 1500, "aux2": 1800})
        check(reply.get("applied") == 2, f"страница аппаратуры приняла тумблеры: применено {reply.get('applied')} канала")
        check(fc.active_mode_names() == {"ARM"},
              f"AUX1 внутри диапазона, AUX2 снаружи → «включён» только ARM: {sorted(fc.active_mode_names())}")
        tx_post({"aux2": 1500})
        check(fc.active_mode_names() == {"ARM", "ANGLE", "HORIZON"},
              f"AUX2 = 1500 → три режима (маска 0x{mode_mask(cli):08x})")
        tx_post({"aux1": 1000, "aux2": 1000})
        check(mode_mask(cli) == 0, "тумблеры вниз → вкладка «Режимы» снова показывает всё выключенным")
        fc.set_rc_channel("aux1", 1500)
        fc.set_rc_channel("aux2", 1500)

        print("\n8. Кли сматривает то же: 'aux' печатает 20 строк по семь чисел (printAux)")
        lines = aux_lines(cli.cli("aux"))
        check(len(lines) == 20, f"'aux' вернул {len(lines)} строк — по одной на слот")
        check(lines[0] == "aux 0 0 0 1300 1700 0 0",
              f"настроенный слот: {lines[0]!r} (index permId AUX MIN MAX logic linkedTo)")
        check(lines[3] == "aux 3 0 0 900 900 0 0", f"пустой слот: {lines[3]!r} — как на свежем борде")
        check("aux 2 2 0 900 900 0 1" in lines, "связь HORIZON → ANGLE видна в CLI как linkedTo = 1")
        diff_lines = aux_lines(cli.cli("diff"))
        check(len(diff_lines) == 3 and all(not line.startswith(("aux 3 ", "aux 4 ")) for line in diff_lines),
              f"'diff' печатает только настроенные слоты ({len(diff_lines)}), как cliDumpPrintLinef")

        print("\n9. Правка из CLI видна на вкладке и наоборот — состояние одно")
        cli.cli("aux 5 1 4 1500 2000")
        row = next((r for r in visible_rows(cli) if r["index"] == 5), None)
        check(row and row["aux"] == 4 and (row["start"], row["end"]) == (1500, 2000),
              f"CLI 'aux 5 1 4 1500 2000' → на вкладке строка AUX5, 1500…2000: {row}")
        cli.cli("aux 5 0 0 900 900")
        check(all(r["index"] != 5 for r in visible_rows(cli)),
              "CLI 'aux 5 0 0 900 900' убирает строку — диапазон 900/900 снова пустой слот")
        cli.cli("aux 5 1 4 1500 2000")
        set_slot(cli, 6, horizon, 2, 1000, 1200)
        check("aux 6 2 2 1000 1200 0 0" in aux_lines(cli.cli("aux")),
              "правка вкладки (слот 6) сразу видна в CLI")

        print("\n10. «Сохранить»: MSP_EEPROM_WRITE + перезагрузка — диапазоны переживают ребут")
        fc.save_to_eeprom()
        fc2 = VirtualFC(eeprom_path=eeprom)
        start_emu(fc2, PORT2, HTTP_PORT + 1)
        time.sleep(0.5)
        cli2 = Client(PORT2)
        ranges2 = read_ranges(cli2)
        rows2 = visible_rows(cli2)
        check((ranges2[0]["aux"], ranges2[0]["start"], ranges2[0]["end"]) == (0, 1300, 1700),
              f"слот 0 после перезагрузки: AUX{ranges2[0]['aux'] + 1} {ranges2[0]['start']}…{ranges2[0]['end']}")
        check(read_extra(cli2)[1]["logic"] == 1, "логика И у ANGLE тоже сохранилась")
        check([r["index"] for r in rows2] == [0, 1, 2, 5, 6],
              f"вкладка видит прежние строки: {[r['index'] for r in rows2]}")
        check(cli2.cli("aux").count("aux ") == 20, "CLI после ребута по-прежнему печатает 20 строк")

        print("\n11. 'defaults' возвращает свежий борт: все 20 слотов пустые")
        cli2.cli("defaults nosave")
        check(not visible_rows(cli2), "после defaults вкладка «Режимы» снова пустая (как на новом борде)")
        check(aux_lines(cli2.cli("aux"))[0] == "aux 0 0 0 900 900 0 0",
              "CLI печатает пустой слот: 'aux 0 0 0 900 900 0 0'")
        check(mode_mask(cli2) == 0, "маска режимов нулевая — ни один режим не «включён»")

        print("\n12. Монитор эмулятора: карточка «Режимы» и /api/state")
        set_slot(cli2, 0, fc2.box_permanent_id("ARM"), 0, 1300, 1700)
        fc2.set_rc_channel("aux1", 1500)
        with urllib.request.urlopen(f"http://127.0.0.1:{HTTP_PORT + 1}/api/state", timeout=5) as answer:
            state = json.load(answer)
        modes = state.get("modes", {})
        check(modes.get("slots_total") == 20 and modes.get("slots_used") == 1,
              f"api/state: занято слотов {modes.get('slots_used')} из {modes.get('slots_total')}")
        check("ARM" in (modes.get("active") or []), f"api/state: включён режим {modes.get('active')}")
        page = urllib.request.urlopen(f"http://127.0.0.1:{HTTP_PORT + 1}/", timeout=5).read().decode()
        check("Режимы (вкладка «Режимы»" in page and "MSP_MODE_RANGES" in page,
              "страница монитора: карточка «Режимы» с протоколом и живым состоянием")
        check("0x00000001" in page, "в карточке видна маска режимов из MSP_STATUS (0x00000001)")

        print("\n13. permanentId: почему «не прописываются» строки (правила cliAux)")
        bad = cli2.cli("aux 7 11 1 1300 1700").strip()
        check(bad == "aux 7 0 0 900 900 0 0",
              f"id 11 в таблице режимов нет (это удалённый GPS HOLD) — разобрано 3 значения, "
              f"слот обнуляется: {bad!r}")
        set_slot(cli2, 7, 13, 2, 1700, 2100)
        bad6 = cli2.cli("aux 7 11 1 1300 1700 0 0").strip()
        check(bad6 == "aux 7 13 1 1300 1700 0 0",
              f"шесть чисел: канал и диапазон записались, а режим остался прежним (BEEPER): {bad6!r}")
        check(cli2.cli("aux 7 13 2 1310 1707 0 0").strip() == "aux 7 13 2 1300 1700 0 0",
              "значения вне сетки округляются вниз до шага 25 мкс (CHANNEL_VALUE_TO_STEP)")
        fc2.set_rc_channel("aux1", 1000)
        fc2.set_rc_channel("aux3", 1500)          # слот 7 после проверки округления: BEEPER, AUX3, 1300…1700
        position = fc2.active_boxes.index("BEEPER")
        check(mode_mask(cli2) == 1 << position,
              f"BEEPER есть в BOXNAMES: тумблер AUX3 внутри 1300…1700 → бит {position} "
              f"(0x{mode_mask(cli2):08x})")
        fc2.set_rc_channel("aux3", 1000)
        cli2.cli("aux 9 15 1 1700 2100 0 0")
        check(cli2.cli("aux 9 15 1 1700 2100 0 0").strip() == "aux 9 15 1 1700 2100 0 0"
              and not any(r["index"] == 9 for r in visible_rows(cli2)),
              "режим вне BOXNAMES (LEDLOW при выключенной LED_STRIP): CLI строку пишет, "
              "вкладка её не показывает — как на железе")

        print("\n14. Страницы эмулятора: /nav, /aux и кнопки перехода на /, /tx и /msc")
        base = f"http://127.0.0.1:{HTTP_PORT + 1}"

        def get(path):
            return urllib.request.urlopen(base + path, timeout=5).read().decode()

        nav_page = get("/nav")
        check(all(f'href="{href}"' in nav_page for href in ("/tx", "/msc", "/aux", "/")),
              "страница «Переходы» (/nav) ведёт кнопками на /tx, /msc, /aux и монитор")
        check("Режимы (AUX)" in nav_page and "Чёрный ящик (MSC)" in nav_page and "Аппаратура (TX)" in nav_page,
              "на кнопках написано, что за страница: «Аппаратура (TX)», «Режимы (AUX)», «Чёрный ящик (MSC)»")
        check(all("class=nav" in get(p) for p in ("/", "/tx", "/aux", "/nav")),
              "полоска кнопок есть на всех страницах: /, /tx, /aux и /nav")
        check(all(0 <= page.find(".nav{") < page.find("</style>")
                  for page in (get("/"), get("/tx"), get("/aux"), get("/nav"))),
              "стили полоски лежат внутри <style> — на странице не видно текста CSS")
        tx_page = get("/tx")
        check('href="/aux"' in tx_page and 'href="/msc"' in tx_page,
              "со страницы аппаратуры /tx можно уйти на «Режимы» и «Чёрный ящик» одним нажатием")

        fc2.set_rc_channel("aux1", 1500)          # тумблер внутри диапазона слота 0: ARM включён
        aux_page = get("/aux")
        check('href="/tx"' in aux_page and "включён" in aux_page and "выключен" in aux_page,
              "/aux отдаётся: таблица слотов с живым состоянием и ссылкой на аппаратуру")
        check("aux 0 0 0 1300 1700 0 0" in aux_page and aux_page.count("aux 19 0 0 900 900 0 0") == 1,
              "/aux показывает те же 20 строк CLI, что печатает команда 'aux'")
        check("ARM <b>0</b>" in aux_page and "LAP TIMER RESET <b>54</b>" in aux_page,
              "на /aux есть справочник режимов с permanentId — его видно перед вводом команд 'aux'")
        check('id="rows"' in aux_page and "/api/state" in aux_page and "setInterval" in aux_page,
              "страница обновляется сама: тумблеры на /tx двигают состояние на /aux без перезагрузки")
        first_lines = "".join(aux_lines(cli2.cli("aux"))[:2])
        check(first_lines in aux_page.replace("\n", ""),
              "сервер уже отрисовал строки — таблица не пустая даже до первого запроса страницы")
        state_perm = json.load(urllib.request.urlopen(base + "/api/state", timeout=5))["modes"]
        check(state_perm["mask"] == 1 and state_perm["slots"][0]["perm"] == 0
              and state_perm["slots"][0]["active"],
              f"/api/state отдаёт маску и permanentId каждого слота (0x{state_perm['mask']:08x})")

        # Раздел «CLI-команды» на мониторе: команда, введённая в «сырой» CLI-порт (так
        # работает вкладка CLI в Configurator), должна попадать в журнал и в карточку.
        probe = "get pid_process_denom"
        raw = socket.create_connection(("127.0.0.1", PORT2), timeout=3)
        raw.settimeout(0.7)
        raw.sendall(b"#")
        time.sleep(0.3)
        raw.recv(4096)
        raw.sendall(probe.encode() + b"\r")
        time.sleep(0.4)
        try:
            raw.recv(8192)
        except TimeoutError:
            pass
        raw.close()
        time.sleep(0.3)
        journal = json.load(urllib.request.urlopen(base + "/api/state", timeout=5))["cli"]
        check(any(entry[1] == probe for entry in journal),
              f"команда из CLI Configurator попала в журнал монитора: "
              f"{[e[1] for e in journal[-3:]]}")
        check("cli-body" in get("/") and probe in get("/"),
              "карточка «CLI-команды» показывает её сразу, без перезагрузки страницы")

        # --- команды CLI прошивки 4.5.4: раньше на них отвечало «UNKNOWN COMMAND».
        # Проверяем имена, форматы строк и то, что правки живут в тех же таблицах FC.
        print("\n19. Команды CLI прошивки 4.5.4: профили, rxfail, rxrange, adjrange, подпись")
        check(cli2.cli("profile").strip() == "profile 0"
              and cli2.cli("profile 3").strip() == "profile 3"
              and "PROFILE OUTSIDE OF [0..3]" in cli2.cli("profile 4"),
              "CLI: 'profile' печатает и переключает профиль PID (0…3), вне диапазона — ошибка")
        check(cli2.cli("rateprofile 2").strip() == "rateprofile 2"
              and "RATE PROFILE OUTSIDE OF [0..3]" in cli2.cli("rateprofile 9"),
              "CLI: 'rateprofile' переключает профиль рейтов и ругается вне 0…3")
        rxfail_dump = cli2.cli("rxfail").strip().split("\n")
        check(len(rxfail_dump) == 18 and rxfail_dump[0] == "rxfail 0 a"
              and cli2.cli("rxfail 16 h").strip() == "rxfail 16 h",
              "CLI: 'rxfail' печатает 18 каналов и правит 17-й (каналы 16–17 видны только из CLI)")
        check(cli2.cli("rxrange 2 1200 1800").strip() == "rxrange 2 1200 1800"
              and cli2.cli("rxrange").strip().split("\n")[2] == "rxrange 2 1200 1800"
              and cli2.cli("rxrange reset").strip() == "",
              "CLI: 'rxrange' правит диапазон канала и сбрасывает его ('reset')")
        check(cli2.cli("adjrange 4 0 3 1500 1700 5 1").strip() == "adjrange 4 0 3 1500 1700 5 1 0 0"
              and len(cli2.cli("adjrange").strip().split("\n")) == 30,
              "CLI: 'adjrange' пишет слот корректировки и печатает все 30 (шаги 25 мкс → микросекунды)")
        check(cli2.cli("board_name").strip() == "board_name VIRTUALFC"
              and cli2.cli("sd_info").strip() == "SD card: Not configured"
              and len(cli2.cli("signature").strip().split()[1]) == 64,
              "CLI: 'board_name', 'sd_info' и 'signature' отвечают как прошивка 4.5.4")
        # cliHelp() ищет подстроку и в имени, и в описании (strcasestr по name И description),
        # поэтому 'help profile' печатает ТРИ записи: battery_profile, profile и rateprofile —
        # в описании battery_profile тоже есть слово «profile».
        check(cli2.cli("help profile").strip() ==
              "battery_profile - change battery profile\n\t[<index>]\n"
              "profile - change profile\n\t[<index>]\n"
              "rateprofile - change rate profile\n\t[<index>]",
              "CLI: 'help' печатает имя, описание и аргументы — как cliHelp() в cli.c")
        check(cli2.cli("battery_profile").strip() == "battery_profile 0"
              and cli2.cli("battery_profile 2").strip() == "battery_profile 2"
              and cli2.cli("battery_profile 9").strip()
              == "###ERROR IN battery_profile: BATTERY PROFILE OUTSIDE OF [0..2]###"
              and cli2.cli("battery_profile").strip() == "battery_profile 2",
              "CLI: 'battery_profile' печатает активный профиль, переключает и ругается на границу (мастер BF)")

        cli2.close()
        cli.close()
    finally:
        pass

    print()
    print("=" * 78)
    if fail_count:
        print(f"ПРОВАЛЕНО проверок: {fail_count}")
        return 1
    print(f"ВКЛАДКА «РЕЖИМЫ» РАБОТАЕТ: диапазоны видны и правятся, состояние живое, "
          f"CLI и «флеш» совпадают — {ok_count} проверок, 0 провалов")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

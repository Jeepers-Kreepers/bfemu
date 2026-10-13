#!/usr/bin/env python3
"""Вкладка «Моторы»: реальный отклик моторов на слайдеры.

Здесь проверяется ровно то, что делает MotorsTab.vue (Betaflight Configurator 2026.6.2)
и прошивка 4.5.4:

  * загрузка вкладки — MSP_PID_ADVANCED → MSP_FEATURE_CONFIG → MSP_MIXER_CONFIG →
    MSP_MOTOR_CONFIG → MSP_MOTOR_TELEMETRY → MSP_MOTOR_3D_CONFIG → MSP2_MOTOR_OUTPUT_REORDERING
    → MSP_ADVANCED_CONFIG → MSP_FILTER_CONFIG → MSP_ARMING_CONFIG; все ответы должны
    разбираться вкладкой без ошибок (у каждого — своя раскладка байт);
  * «Я понимаю риски» → MSP_SET_ARMING_DISABLED(1) + MSP2_SEND_DSHOT_COMMAND(13):
    взвод запрещается (в MSP_STATUS появляется ARMING_DISABLED_MSP), ESC включают
    расширенную телеметрию;
  * слайдер → MSP_SET_MOTOR (214) пишет motor_disarmed[], а MSP_MOTOR (104) отдаёт
    восемь u16 — как msp.c 4.5.4;
  * отклик «как на железе»: обороты раскручиваются постепенно (есть промежуточные
    значения), ток и температура растут, напряжение проседает тем сильнее, чем больше
    суммарный ток; после «Стоп» мотор выбегает, а не глохнет мгновенно;
  * тест не сбрасывается по таймауту (на железной плате мотор держит газ, пока
    Configurator не пришлёт нули);
  * «Сохранить и перезагрузить» во вкладке: настройки моторов живут во «флеше»,
    а сам тест после перезагрузки, конечно, прекращается;
  * монитор эмулятора и /api/state показывают те же цифры.

Запуск:  python3 tools/check_motors_tab.py
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
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bfemu.fc import VirtualFC
from bfemu.msp import MspParser, encode_v1, encode_v2
from bfemu.server import HttpMonitor, TcpServer

PORT = 15811
HTTP_PORT = 8090

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
                data = self.sock.recv(65536)
            except socket.timeout:
                break
            if not data:
                break
            self.parser.feed(data)
            for req in self.parser:
                if req.code == code:
                    return req.payload
        raise TimeoutError(f"нет ответа на MSP {code}")


def u16(data: bytes, offset: int = 0) -> int:
    return int.from_bytes(data[offset:offset + 2], "little")


def motor_values(cli: Client) -> list:
    """MSP_MOTOR (104) — как его разбирает MSPHelper.js: по u16 на мотор."""
    payload = cli.request(104)
    return [u16(payload, i * 2) for i in range(len(payload) // 2)]


def telemetry(cli: Client) -> list:
    """MSP_MOTOR_TELEMETRY (139) — по 13 байт на мотор, как в MSPHelper.js."""
    payload = cli.request(139)
    rows = []
    for i in range(payload[0]):
        base = 1 + i * 13
        rows.append({
            "rpm": int.from_bytes(payload[base:base + 4], "little"),
            "invalid": u16(payload, base + 4),
            "temp_c": payload[base + 6],
            "voltage_v": u16(payload, base + 7) / 100,
            "current_a": u16(payload, base + 9) / 100,
            "mah": u16(payload, base + 11),
        })
    return rows


def send_motors(cli: Client, values: list) -> None:
    """Вкладка всегда шлёт восемь u16 (useMotorTesting.js: sendMotorCommand)."""
    payload = b"".join(struct.pack("<H", v) for v in (values + [1000] * 8)[:8])
    cli.request(214, payload)


def main() -> int:
    workdir = Path(tempfile.mkdtemp(prefix="bfemu-motors-"))
    eeprom = workdir / "eeprom.json"
    fc = VirtualFC(eeprom_path=eeprom)
    port, http_port = PORT, HTTP_PORT
    server = TcpServer(fc, "127.0.0.1", port, boot_delay=0.3, verbose=False)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    monitor = HttpMonitor(fc, http_port, strict_port=True)
    monitor.prepare()
    threading.Thread(target=monitor.serve_forever, daemon=True).start()
    time.sleep(0.7)
    try:
        cli = Client()

        print("\n1. Загрузка вкладки (порядок и раскладки как в MotorsTab.vue)")
        pid_adv = cli.request(94)
        check(len(pid_adv) == 61, f"MSP_PID_ADVANCED: {len(pid_adv)} байт (вкладка читает поля RPM-фильтра)")
        feats = cli.request(36)
        check(len(feats) == 4, "MSP_FEATURE_CONFIG: 4 байта (MOTOR_STOP, ESC_SENSOR, 3D)")
        mixer = cli.request(42)
        check(len(mixer) == 2, f"MSP_MIXER_CONFIG: 2 байта (миксер {mixer[0]})")
        cfg = cli.request(131)
        check(len(cfg) == 10, "MSP_MOTOR_CONFIG: 10 байт (min/max/mincommand, моторы, полюса, телеметрия)")
        check((u16(cfg, 0), u16(cfg, 2), u16(cfg, 4)) == (1070, 2000, 1000),
              f"газ: minthrottle {u16(cfg, 0)}, maxthrottle {u16(cfg, 2)}, mincommand {u16(cfg, 4)}")
        check(cfg[6] == 4 and cfg[7] == 14,
              f"моторов {cfg[6]}, полюсов {cfg[7]} (моторы 2207 — 7 пар полюсов)")
        check(cfg[8] == 1, "DShot-телеметрия включена (dshot_bidir) — вкладка будет опрашивать ESC")
        tele = telemetry(cli)
        check(len(tele) == 4, f"MSP_MOTOR_TELEMETRY: {len(tele)} моторов × 13 байт")
        check(all(t["rpm"] == 0 and t["current_a"] == 0 for t in tele),
              "в покое ESC молчат: 0 оборотов, 0.00 А")
        check(20 <= tele[0]["temp_c"] <= 30, f"температура ESC на столе: {tele[0]['temp_c']} °C")
        check(tele[0]["voltage_v"] > 12, f"напряжение на ESC: {tele[0]['voltage_v']:.2f} В (батарея подключена)")
        m3d = cli.request(124)
        check(len(m3d) == 6 and u16(m3d, 0) == 1406 and u16(m3d, 4) == 1460,
              "MSP_MOTOR_3D_CONFIG: 6 байт (deadband 1406/1514, neutral3d 1460)")
        order = cli.request(0x3001, version=2)
        check(len(order) == 9 and order[0] == 8 and list(order[1:]) == list(range(8)),
              "MSP2_MOTOR_OUTPUT_REORDERING: 8 моторов, заводской порядок 0…7")
        adv = cli.request(90)
        check(len(adv) >= 18 and adv[3] == 7, f"MSP_ADVANCED_CONFIG: протокол {adv[3]} = DSHOT600")
        check(u16(adv, 6) == 550, f"digitalIdleOffsetValue {u16(adv, 6)} (dshot_idle_value)")
        filters = cli.request(92)
        check(len(filters) >= 25, f"MSP_FILTER_CONFIG: {len(filters)} байт (вкладка читает RPM-фильтр)")
        arming = cli.request(61)
        check(len(arming) == 3, "MSP_ARMING_CONFIG: 3 байта")

        print("\n2. «Я понимаю риски» — вкладка включает тест моторов")
        cli.request(99, bytes([1, 1]))
        flags = struct.unpack_from("<I", cli.request(150), 17)[0]     # MSP_STATUS_EX: armingDisableFlags
        check(bool(flags & (1 << 17)),
              f"MSP_SET_ARMING_DISABLED(1): взвод запрещён, в флагах ARMING_DISABLED_MSP (0x{flags:08x})")
        cli.request(0x3003, bytes([0, 255, 1, 13]), version=2)         # DSHOT_CMD_TYPE_BLOCKING, всем, EDT on
        check(fc.dshot_commands_log and fc.dshot_commands_log[-1]["commands"] == [13],
              "MSP2_SEND_DSHOT_COMMAND: ESC получили команду 13 (расширенная телеметрия)")

        print("\n3. Слайдер мастера: MSP_SET_MOTOR → выходы (как msp.c 4.5.4)")
        idle = motor_values(cli)
        check(idle[:4] == [1000] * 4 and idle[4:] == [0] * 4,
              f"до теста MSP_MOTOR: {idle[:4]} (у несуществующих моторов нули)")
        send_motors(cli, [1600, 1600, 1600, 1600])
        outputs = motor_values(cli)
        check(len(outputs) == 8, "MSP_MOTOR отдаёт ВОСЕМЬ значений (даже когда моторов четыре)")
        check(outputs[:4] == [1600] * 4 and outputs[4:] == [0] * 4,
              f"выходы моторов: {outputs[:4]}, у несуществующих моторов — нули {outputs[4:]}")

        print("\n4. Живой отклик: обороты, ток, температура, напряжение")
        samples = []
        for _ in range(6):
            time.sleep(0.05)
            samples.append(telemetry(cli)[0])
        rpms = [s["rpm"] for s in samples]
        check(rpms[0] < rpms[-1], f"обороты раскручиваются постепенно: {rpms[0]} → {rpms[-1]} (промежуточные видны)")
        check(rpms[-1] > 12000,
              f"на 1600 мкс мотор даёт {rpms[-1]} об/мин (газ 60 % от 24000)")
        check(all(s["current_a"] > 3 for s in samples[-2:]),
              f"ток ESC вырос до {samples[-1]['current_a']:.2f} А")
        check(samples[-1]["temp_c"] >= samples[0]["temp_c"],
              f"температура ESC {samples[0]['temp_c']} → {samples[-1]['temp_c']} °C")
        check(samples[-1]["voltage_v"] < 16.8,
              f"напряжение просело до {samples[-1]['voltage_v']:.2f} В под нагрузкой")
        check(samples[-1]["mah"] >= samples[0]["mah"], f"расход тикает: {samples[-1]['mah']} мА·ч")
        check(samples[-1]["invalid"] < 100, f"доля битых кадров телеметрии: {samples[-1]['invalid'] / 100:.2f} %")
        # напряжение в MSP_ANALOG — то же самое, что видят ESC (читаем подряд, без пауз)
        esc_voltage = telemetry(cli)[0]["voltage_v"]
        analog = cli.request(110)
        check(abs(u16(analog, 7) / 100 - esc_voltage) <= 0.03,
              f"MSP_ANALOG показывает ту же просадку: {u16(analog, 7) / 100:.2f} В "
              f"(вкладка «Питание и батарея» и OSD сойдутся)")

        print("\n5. Моторы держат газ, пока Configurator не пришлёт нули (таймаута нет)")
        time.sleep(1.2)
        held = telemetry(cli)[0]
        check(held["rpm"] > 12000 and held["current_a"] > 3,
              f"через 1.2 с без команд мотор всё ещё крутится: {held['rpm']} об/мин, {held['current_a']:.2f} А")
        check(motor_values(cli)[:4] == [1600] * 4, "MSP_MOTOR по-прежнему отдаёт заданные 1600")

        print("\n6. Один мотор из четырёх (слайдеры независимы)")
        send_motors(cli, [1000, 1800, 1000, 1000])
        time.sleep(0.1)
        slowing = telemetry(cli)
        time.sleep(1.2)
        rows = telemetry(cli)
        check(rows[1]["rpm"] > 15000 and rows[0]["rpm"] < 100 and rows[2]["rpm"] < 100,
              f"крутится только M2: обороты M1…M4 = {[r['rpm'] for r in rows]}")
        check(slowing[0]["rpm"] > rows[0]["rpm"],
              f"остальные моторы при этом выбегают: M1 {slowing[0]['rpm']} → {rows[0]['rpm']} об/мин")
        check(motor_values(cli)[:4] == [1000, 1800, 1000, 1000], "выходы: 1000 / 1800 / 1000 / 1000")

        print("\n7. «Стоп»: мотор выбегает, а не глохнет мгновенно")
        send_motors(cli, [1000] * 4)
        first_after_stop = telemetry(cli)[1]["rpm"]
        check(first_after_stop > 1000, f"сразу после команды стоп ещё {first_after_stop} об/мин (выбег)")
        time.sleep(0.8)
        rows = telemetry(cli)
        check(max(r["rpm"] for r in rows) < 1500,
              f"через 0.8 с обороты почти нулевые (выбег): {[r['rpm'] for r in rows]}")
        time.sleep(0.7)
        rows = telemetry(cli)
        check(max(r["rpm"] for r in rows) < 100,
              f"через 1.5 с моторы практически стоят: {[r['rpm'] for r in rows]}")
        time.sleep(0.6)
        rows = telemetry(cli)
        check(all(r["rpm"] == 0 for r in rows), f"дальше — ровный ноль: {[r['rpm'] for r in rows]}")
        check(all(r["current_a"] == 0 for r in rows), "ток ESC вернулся к нулю")
        check(motor_values(cli) == [1000] * 4 + [0] * 4, "MSP_MOTOR: выходы на «стопе» (1000), у мнимых моторов нули")

        print("\n8. «Сохранить и перезагрузить»: настройки моторов во «флеше»")
        cli.request(37, feats)                                          # MSP_SET_FEATURE_CONFIG
        cli.request(43, b"")                                            # MSP_SET_MIXER_CONFIG
        cli.request(222, struct.pack("<HHHBBBB", 1050, 1990, 1005, 4, 12, 1, 0))
        cli.request(217, struct.pack("<HHH", 1420, 1520, 1470))
        cli.request(91, bytes([0, 1, 0, 6]) + struct.pack("<HH", 480, 600) + bytes([0, 0, 0, 0, 48, 125, 0, 0, 0, 2, 0, 7]))
        cli.request(62, bytes([1, 5, 25]))
        cli.request(93, b"\x00" * 25)
        cli.request(250)                                                # MSP_EEPROM_WRITE
        cli.close()

        fc2 = VirtualFC(eeprom_path=eeprom)
        server2 = TcpServer(fc2, "127.0.0.1", port + 1, boot_delay=0.3, verbose=False)
        threading.Thread(target=server2.serve_forever, daemon=True).start()
        time.sleep(0.5)
        cli2 = Client(port=port + 1)
        saved = cli2.request(131)
        check((u16(saved, 0), u16(saved, 2), u16(saved, 4)) == (1050, 1990, 1005),
              f"после перезапуска газ: {u16(saved, 0)}/{u16(saved, 2)}/{u16(saved, 4)}")
        check(saved[7] == 12, f"полюсов {saved[7]} (правка вкладки сохранилась)")
        m3d2 = cli2.request(124)
        check((u16(m3d2, 0), u16(m3d2, 2), u16(m3d2, 4)) == (1420, 1520, 1470),
              "3D-настройки пережили перезагрузку")
        adv2 = cli2.request(90)
        check(adv2[3] == 6 and u16(adv2, 6) == 600,
              f"протокол {adv2[3]} = DSHOT300, idle {u16(adv2, 6)} — как сохранила вкладка")
        rows = telemetry(cli2)
        check(all(r["rpm"] == 0 and r["current_a"] == 0 for r in rows),
              "после перезагрузки тест моторов прекращён (моторы не «убегают»)")
        check(motor_values(cli2)[:4] == [1000] * 4, "выходы после ребута — 1000 (motor_disarmed[] в нуле, как на железе)")

        print("\n9. Монитор эмулятора и /api/state")
        monitor2 = HttpMonitor(fc2, http_port + 1, strict_port=True)
        monitor2.prepare()
        threading.Thread(target=monitor2.serve_forever, daemon=True).start()
        time.sleep(0.3)
        send_motors(cli2, [1700, 1700, 1700, 1700])
        time.sleep(0.4)
        with urllib.request.urlopen(f"http://127.0.0.1:{http_port + 1}/api/state", timeout=5) as answer:
            state = json.loads(answer.read())
        motors = state.get("motors") or {}
        check(motors.get("count") == 4 and motors.get("test_active"),
              f"api/state: {motors.get('count')} мотора, тест {'идёт' if motors.get('test_active') else 'нет'}")
        check((motors.get("motors") or [{}])[0].get("rpm", 0) > 12000,
              f"api/state: обороты M1 {motors['motors'][0]['rpm']}, ток {motors['motors'][0]['current_a']} А")
        page = urllib.request.urlopen(f"http://127.0.0.1:{http_port + 1}/", timeout=5).read().decode()
        card_ok = "Моторы (вкладка «Моторы»: тест со слайдеров)" in page and "DSHOT" in page
        check(card_ok, "страница монитора: карточка «Моторы» с протоколом и живыми цифрами")
        if not card_ok:
            import re as _re
            snippet = _re.search(r"<h2>Мотор[^<]*</h2>.{0,220}", page)
            print("        страница:", (snippet.group(0) if snippet else "карточки нет")[:240])
        check("MSP_MOTOR_TELEMETRY" in page, "карточка объясняет, откуда берутся цифры")
        cli2.close()
        server2.stop()

        print()
        if fail_count:
            print(f"ПРОВАЛЕНО проверок: {fail_count}")
            return 1
        print(f"ВКЛАДКА «МОТОРЫ» ОТВЕЧАЕТ ЖИВЬЁМ: слайдеры → выходы, обороты, ток, температура, "
              f"просадка — {ok_count} проверок, 0 провалов")
        return 0
    finally:
        server.stop()
        shutil.rmtree(workdir, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())

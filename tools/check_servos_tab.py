#!/usr/bin/env python3
"""Проверка вкладки «Сервоприводы»: build-опции прошивки, конфигурация серво, сохранение.

Главная причина, по которой вкладки не было: Configurator 2026.x показывает её только если
прошивка сообщает build-опцию USE_SERVOS (или USE_WING) — список опций приходит в ответе
MSP_BUILD_INFO (код 5) для API >= 1.46. Наш эмулятор эти опции не отдавал.

Скрипт:

  1. разбирает MSP_BUILD_INFO так же, как MSPHelper.js: дата 11 Б, время 8 Б, git-ревизия 7 Б,
     затем u16-коды build options до нуля; переводит коды в имена таблицей FIRMWARE_BUILD_OPTIONS
     из src/js/fc.js;
  2. повторяет логику Configurator: GUI.allowedTabs (serial_backend.js finishOpen) и
     видимость пункта меню «Сервоприводы» (sidebar_items.js: buildOptions USE_SERVOS/USE_WING),
     а также фильтр функций вкладки «Конфигурация» (Features.js: dependsOn);
  3. проверяет обмен вкладки: MSP_SERVO_CONFIGURATIONS (120), MSP_SERVO_MIX_RULES (241),
     MSP_SERVO (103), MSP_RC (105), BOXNES (116), запись MSP_SET_SERVO_CONFIGURATION (212);
  4. сохраняет правки, перезагружает, перезапускает эмулятор и проверяет результат + CLI.

Запуск:  python3 tools/check_servos_tab.py      (Windows: python tools\\check_servos_tab.py)
"""
from __future__ import annotations

import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from bfemu.msp import encode_v1, encode_v2  # noqa: E402

PORT = 16081
FAILS: list[str] = []

# FIRMWARE_BUILD_OPTIONS из src/js/fc.js (Configurator 2026.6.2) — нужные нам коды.
BUILD_OPTION_NAMES = {
    4097: "USE_SERIALRX_CRSF", 4102: "USE_RX_PPM", 4103: "USE_SERIALRX_SBUS",
    8230: "USE_BRUSHED", 8231: "USE_DSHOT", 8232: "USE_MULTISHOT", 8233: "USE_ONESHOT",
    8234: "USE_PROSHOT", 8235: "USE_PWM_OUTPUT",
    16408: "USE_DASHBOARD", 16412: "USE_GPS", 16413: "USE_LED_STRIP", 16415: "USE_MAG",
    16416: "USE_OSD_SD", 16417: "USE_OSD_HD", 16420: "USE_SERVOS", 16421: "USE_VTX",
    16423: "USE_SOFTSERIAL", 16424: "USE_WING", 16427: "USE_FLIGHT_PLAN",
    16429: "USE_RANGEFINDER",
}

# GUI.defaultAllowedTabs + defaultCloudBuildTabOptions из src/js/gui.js
DEFAULT_ALLOWED_TABS = [
    "setup", "failsafe", "power", "adjustments", "auxiliary", "presets", "cli", "configuration",
    "logging", "onboard_logging", "modes", "motors", "pid_tuning", "autotune", "ports",
    "receiver", "sensors", "blackbox_viewer",
]
CLOUD_BUILD_TABS = {"gps": "USE_GPS", "led_strip": "USE_LED_STRIP", "osd": "USE_OSD_SD",
                    "servos": "USE_SERVOS", "vtx": "USE_VTX", "flight_plan": "USE_FLIGHT_PLAN"}

# sidebar_items.js: пункты меню, у которых есть условие по build-опциям
SIDEBAR_BUILD_GATES = {"servos": ("USE_SERVOS", "USE_WING"), "gps": ("USE_GPS",),
                       "led_strip": ("USE_LED_STRIP",)}

# Features.js: функции вкладки «Конфигурация» и их dependsOn (порядок как в исходнике 2026.6.2)
FEATURES = [
    ("RX_PPM", "RX_PPM"), ("INFLIGHT_ACC_CAL", None), ("RX_SERIAL", None), ("MOTOR_STOP", None),
    ("SERVO_TILT", "SERVOS"), ("SOFTSERIAL", None), ("GPS", "GPS"), ("SONAR", "RANGEFINDER"),
    ("TELEMETRY", "TELEMETRY"), ("3D", None), ("RX_PARALLEL_PWM", None), ("RX_MSP", None),
    ("RSSI_ADC", None), ("LED_STRIP", "LED_STRIP"), ("DISPLAY", "DASHBOARD"), ("OSD", "OSD"),
    ("CHANNEL_FORWARDING", "SERVOS"), ("TRANSPONDER", "TRANSPONDER"), ("AIRMODE", None),
    ("RX_SPI", None), ("ESC_SENSOR", None), ("ANTI_GRAVITY", None),
]
# функции без dependsOn, которые видны всегда (у SOFTSERIAL dependsOn появляется только с API 1.47)
FEATURES_WITHOUT_DEPENDS_ON = [name for name, dep in FEATURES if dep is None]


def _frame(buf: bytes):
    for i in range(len(buf) - 5):
        if buf[i] != 0x24:
            continue
        if buf[i + 1] == 0x4D and buf[i + 2] == 0x3E:
            size = buf[i + 3]
            if i + 5 + size <= len(buf):
                return buf[i + 5 : i + 5 + size]
        if buf[i + 1] == 0x58 and buf[i + 2] == 0x3E:
            size = buf[i + 6] | (buf[i + 7] << 8)
            if i + 8 + size <= len(buf):
                return buf[i + 8 : i + 8 + size]
    return None


class Fc:
    def __init__(self, port: int):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=3)
        self.sock.settimeout(0.4)

    def call(self, code: int, payload: bytes = b"", v2: bool = False) -> bytes:
        self.sock.sendall(encode_v2(code, payload) if v2 else encode_v1(code, payload))
        buf, deadline = b"", time.time() + 2.5
        while time.time() < deadline:
            try:
                buf += self.sock.recv(65536)
            except TimeoutError:
                pass
            frame = _frame(buf)
            if frame is not None:
                return frame
            time.sleep(0.02)
        return b""

    def cli(self, command: str, window: float = 2.5) -> str:
        out, offset, deadline = b"", 0, time.time() + window
        while time.time() < deadline:
            payload = command.encode() if offset == 0 else command.encode() + b"\x00" + offset.to_bytes(2, "little")
            resp = self.call(0x3012, payload, v2=True)
            if len(resp) < 3:
                break
            out += resp[3:]
            offset += len(resp) - 3
            if offset >= int.from_bytes(resp[0:2], "little") or len(resp) == 3:
                break
        return out.decode("latin-1", "replace").replace("\r\n", "\n")

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def start(port: int, eeprom: Path):
    proc = subprocess.Popen(
        [sys.executable, "-m", "bfemu", "--tcp", f"127.0.0.1:{port}", "--eeprom", str(eeprom), "--quiet"],
        cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
    )
    for _ in range(60):
        try:
            return proc, Fc(port)
        except OSError:
            time.sleep(0.25)
    raise RuntimeError("эмулятор не поднялся")


def stop(proc):
    proc.terminate()
    proc.wait(timeout=10)


def check(ok: bool, what: str):
    print(f"   {'[OK ]' if ok else '[НЕТ]'} {what}")
    if not ok:
        FAILS.append(what)


def u16(b, off=0):
    return int.from_bytes(b[off : off + 2], "little")


def parse_build_info(data: bytes) -> dict:
    """Разбор MSP_BUILD_INFO так, как это делает MSPHelper.js."""
    info = {
        "buildInfo": data[0:11].decode("latin-1") + " " + data[11:19].decode("latin-1"),
        "gitRevision": data[19:26].decode("latin-1"),
        "options": [],
    }
    pos = 26
    while pos + 1 < len(data):
        code = u16(data, pos)
        pos += 2
        if code == 0:
            break
        info["options"].append(BUILD_OPTION_NAMES.get(code, f"0x{code:04X}"))
    return info


def allowed_tabs(options: list) -> set:
    """GUI.allowedTabs из serial_backend.js finishOpen()."""
    if not options:
        return set(DEFAULT_ALLOWED_TABS) | set(CLOUD_BUILD_TABS)
    tabs = set(DEFAULT_ALLOWED_TABS)
    for tab, option in CLOUD_BUILD_TABS.items():
        if option in options:
            tabs.add(tab)
    if "USE_WING" in options:
        tabs.add("servos")
    return tabs


def visible_features(options: list) -> set:
    """Features.js (API 1.46): фильтр по build options + возврат TELEMETRY для CRSF/GHST/FPORT/JETI."""
    visible = {name for name, dep in FEATURES
               if dep is None or any(dep in option for option in options)}
    if any(key in option for option in options
           for key in ("CRSF", "GHST", "FPORT", "JETI")):
        visible.add("TELEMETRY")            # Configurator добавляет эту функцию обратно
    return visible


def read_servos(fc: Fc) -> list:
    data = fc.call(120)
    return [
        {
            "min": u16(data, i * 12),
            "max": u16(data, i * 12 + 2),
            "middle": u16(data, i * 12 + 4),
            "rate": data[i * 12 + 6],
            "forwardFromChannel": data[i * 12 + 7],
            "reversedSources": int.from_bytes(data[i * 12 + 8 : i * 12 + 12], "little"),
        }
        for i in range(len(data) // 12)
    ]


def main() -> int:
    eeprom = Path(tempfile.mkdtemp()) / "eeprom.json"
    proc, fc = start(PORT, eeprom)

    print("=" * 78)
    print("ПОЧЕМУ ВКЛАДКИ НЕ БЫЛО: build options в MSP_BUILD_INFO")
    print("=" * 78)
    build = parse_build_info(fc.call(5))
    print(f"   Сборка: {build['buildInfo']} | git-ревизия: {build['gitRevision']!r}")
    print(f"   Build options ({len(build['options'])}): {', '.join(build['options'])}")

    check("USE_SERVOS" in build["options"], "прошивка сообщает USE_SERVOS — вкладка «Сервоприводы» разрешена")
    check(build["buildInfo"].startswith("Sep 24 2026 12:00:00"),
          f"дата и время сборки разбираются без сдвига: {build['buildInfo']!r}")

    tabs = allowed_tabs(build["options"])
    print(f"   Разрешённые вкладки: {', '.join(sorted(t for t in tabs if t in CLOUD_BUILD_TABS))}")
    check("servos" in tabs, "GUI.allowedTabs содержит «servos» (sidebar_items.js: USE_SERVOS)")
    check("gps" in tabs, "GUI.allowedTabs содержит «gps» — в сборке есть USE_GPS (16412)")
    for tab in ("osd", "vtx", "led_strip"):
        check(tab in tabs, f"вкладка «{tab}» по-прежнему разрешена (не потерялась из-за build options)")
    for tab in ("setup", "sensors", "ports", "configuration", "power", "failsafe", "motors",
                "receiver", "auxiliary", "presets", "pid_tuning"):
        check(tab in tabs, f"вкладка «{tab}» по-прежнему разрешена")

    features = visible_features(build["options"])
    for feature in ("SERVO_TILT", "CHANNEL_FORWARDING", "LED_STRIP", "DISPLAY", "OSD", "SONAR",
                    "TELEMETRY", "AIRMODE", "SOFTSERIAL"):
        check(feature in features, f"функция {feature} остаётся на вкладке «Конфигурация»")
    check("TELEMETRY" in features and "CRSF" in " ".join(build["options"]),
          "TELEMETRY на месте: Configurator возвращает её для CRSF, хотя build-опции USE_TELEMETRY нет")
    check("GPS" in features,
          "функция GPS видна: Configurator показывает её при build-опции USE_GPS (Features.js: dependsOn GPS)")
    for feature in ("RX_PPM", "TRANSPONDER"):
        check(feature not in features,
              f"функции {feature} нет — как у платы, в прошивке которой нет соответствующей опции")

    print("\n" + "=" * 78)
    print("ВКЛАДКА «СЕРВОПРИВОДЫ»: что отдаёт эмулятор")
    print("=" * 78)
    servos = read_servos(fc)
    mix_rules = fc.call(241)
    live = fc.call(103)
    rc = fc.call(105)
    box_names = fc.call(116).decode("latin-1")

    print(f"   Сервоприводов: {len(servos)}")
    for index, servo in enumerate(servos[:3]):
        print(f"      Servo {index + 1}: min {servo['min']}, середина {servo['middle']}, max {servo['max']}, "
              f"rate {servo['rate']}, перенаправление "
              f"{'выключено' if servo['forwardFromChannel'] == 0xFF else servo['forwardFromChannel'] + 1}")
    print(f"   Правила микшера: {len(mix_rules) // 7} по 7 байт")
    print(f"   Живые значения (103): {[u16(live, i * 2) for i in range(len(live) // 2)]}")
    print(f"   Каналов RC (105): {len(rc) // 2} | режимов (BOXNES): {len([n for n in box_names.split(';') if n])}")

    check(len(servos) == 8, f"MSP_SERVO_CONFIGURATIONS: {len(servos)} записей по 12 байт ({len(fc.call(120))} Б всего)")
    check(all(s["min"] == 1000 and s["middle"] == 1500 and s["max"] == 2000 for s in servos),
          "заводские значения как в pgResetFn_servoParams(): 1000/1500/2000")
    check(all(s["rate"] == 100 and s["forwardFromChannel"] == 0xFF for s in servos),
          "rate 100 и «не перенаправлять» (0xFF) — как в прошивке")
    check(len(mix_rules) == 112, f"MSP_SERVO_MIX_RULES: {len(mix_rules)} байт = 16 правил")
    check(len(live) == 16, f"MSP_SERVO: {len(live) // 2} значений (MAX_SUPPORTED_SERVOS)")
    check(len(rc) == 32 and len(box_names) > 0, "MSP_RC и BOXNES приходят (нужны вкладке для списка каналов)")

    print("\n" + "=" * 78)
    print("ПРАВКА СЕРВОПРИВОДОВ (как вкладка перед «Сохранить»)")
    print("=" * 78)
    # Servo 3: 900/1500/2100, rate 50, перенаправление AUX 1, реверс канала AUX 1
    fc.call(212, bytes([2]) + (900).to_bytes(2, "little") + (2100).to_bytes(2, "little")
            + (1500).to_bytes(2, "little") + bytes([50, 4]) + (1 << 4).to_bytes(4, "little"))
    edited = read_servos(fc)[2]
    print(f"   Servo 3: min {edited['min']}, max {edited['max']}, rate {edited['rate']}, "
          f"перенаправление {edited['forwardFromChannel'] + 1} (AUX 1), реверс 0x{edited['reversedSources']:X}")
    check((edited["min"], edited["middle"], edited["max"]) == (900, 1500, 2100),
          "границы и середина записаны")
    check(edited["rate"] == 50, f"скорость (rate) записана: {edited['rate']}")
    check(edited["forwardFromChannel"] == 4, "перенаправление с AUX 1 записано")
    check(edited["reversedSources"] == 1 << 4, "реверс источника сохранён (бит 4 = RC roll)")

    live2 = [u16(fc.call(103), i * 2) for i in range(8)]
    print(f"   Живые значения: {live2} (Servo 3 отдаёт уровень AUX 1)")
    check(live2[2] == 1000, f"Servo 3 с перенаправлением отдаёт уровень AUX 1 (1000 мкс — как в MSP_RC), а не середину: {live2[2]}")
    check(all(v == 1500 for i, v in enumerate(live2) if i != 2),
          "остальные серво без перенаправления отдают середину (1500) — как servo.c")

    fc.call(212, bytes([9]) + bytes(12))          # неверный индекс
    check(len(read_servos(fc)) == 8, "запись с индексом >= 8 отклоняется (как MSP_RESULT_ERROR в msp.c)")

    print("\n-- «Сохранить» + «Сохранить и перезагрузить» (212 × 8 -> EEPROM_WRITE -> ребут) --")
    fc.call(250)
    fc.call(68, b"\x00")
    time.sleep(1.0)
    fc.close()
    stop(proc)

    print("\n" + "=" * 78)
    print("ПОЛНЫЙ ПЕРЕЗАПУСК ЭМУЛЯТОРА")
    print("=" * 78)
    proc, fc2 = start(PORT, eeprom)
    saved = read_servos(fc2)[2]
    check((saved["min"], saved["max"], saved["rate"], saved["forwardFromChannel"]) == (900, 2100, 50, 4),
          f"Servo 3 пережил перезапуск: {saved['min']}/{saved['max']}, rate {saved['rate']}, "
          f"перенаправление {saved['forwardFromChannel'] + 1}")
    check(read_servos(fc2)[0]["min"] == 1000, "остальные сервоприводы остались заводскими")
    check("USE_SERVOS" in parse_build_info(fc2.call(5))["options"], "build options на месте после перезапуска")

    print("\n-- 'defaults': сервоприводы возвращаются к заводским --")
    fc2.cli("defaults")
    time.sleep(0.7)
    fc2.close()
    stop(proc)
    proc, fc3 = start(PORT, eeprom)
    reset = read_servos(fc3)
    check(all(s["min"] == 1000 and s["middle"] == 1500 and s["max"] == 2000 and s["rate"] == 100
              and s["forwardFromChannel"] == 0xFF for s in reset),
          "после 'defaults' — заводские 1000/1500/2000, rate 100, перенаправление выключено")
    fc3.close()
    stop(proc)

    print()
    print("=" * 78)
    if FAILS:
        print(f"ПРОВАЛЕНО проверок: {len(FAILS)}")
        for item in FAILS:
            print("  -", item)
        return 1
    print("ВКЛАДКА «СЕРВОПРИВОДЫ» РАБОТАЕТ: вкладка разрешена, серво читаются, пишутся и сохраняются")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

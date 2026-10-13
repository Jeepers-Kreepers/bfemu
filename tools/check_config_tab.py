#!/usr/bin/env python3
"""Проверка вкладки «Конфигурация»: все параметры, которые она читает и пишет.

Для каждого запроса:
  * длина ответа сверяется с раскладкой MSPHelper.js (то, что читает Configurator);
  * значения разбираются по тем же офсетам и печатаются как в интерфейсе;
  * запись (MSP_SET_*) → Save → Save and Reboot → **перезапуск процесса эмулятора** → чтение:
    параметр должен вернуться тем же.

Запуск:  python3 tools/check_config_tab.py        (Linux/macOS)
         python tools\\check_config_tab.py          (Windows: эмулятор запускается сам)
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

PORT = 16031
HTTP = 16032
FAILS: list[str] = []


# ----------------------------------------------------------------- разбор ответов FC
def _frame(buf: bytes):
    for i in range(len(buf) - 5):
        if buf[i] != 0x24:
            continue
        if buf[i + 1] == 0x4D and buf[i + 2] == 0x3E:            # $M> (v1)
            size = buf[i + 3]
            if i + 5 + size <= len(buf):
                return buf[i + 5 : i + 5 + size]
        if buf[i + 1] == 0x58 and buf[i + 2] == 0x3E:            # $X> (v2)
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
        buf = b""
        deadline = time.time() + 2.5
        while time.time() < deadline:
            try:
                buf += self.sock.recv(65536)
            except socket.timeout:
                break
            frame = _frame(buf)
            if frame is not None:
                return frame
            time.sleep(0.03)
        return b""

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def u8(b, off, default=-1):
    return b[off] if off < len(b) else default


def u16(b, off, signed=False):
    return int.from_bytes(b[off : off + 2], "little", signed=signed) if off + 2 <= len(b) else -1


def u32(b, off):
    return int.from_bytes(b[off : off + 4], "little") if off + 4 <= len(b) else -1


def text(b, off):
    """self.getText(): длина + символы."""
    size = u8(b, off, 0)
    return b[off + 1 : off + 1 + size].decode("latin-1") if size and off + 1 + size <= len(b) else ""


# ----------------------------------------------------------------- запуск эмулятора
def start(port: int, eeprom: Path):
    proc = subprocess.Popen(
        [sys.executable, "-m", "bfemu", "--tcp", f"127.0.0.1:{port}", "--eeprom", str(eeprom), "--quiet"],
        cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
    )
    for _ in range(60):
        try:
            client = Fc(port)
        except OSError:
            if proc.poll() is not None:      # процесс уже умер: чаще всего порт занят другой копией
                raise RuntimeError(
                    f"эмулятор не поднялся: порт {port} занят. Остановите другую копию "
                    f"(Ctrl+C в её окне) и запустите проверку снова"
                )
            time.sleep(0.25)
            continue
        return proc, client
    raise RuntimeError("эмулятор не поднялся")


def stop(proc):
    proc.terminate()
    proc.wait(timeout=10)


def check(ok: bool, what: str):
    print(f"   {'[OK ]' if ok else '[НЕТ]'} {what}")
    if not ok:
        FAILS.append(what)


# ----------------------------------------------------------------- вкладка «Конфигурация»
def read_all(fc: Fc) -> dict:
    """Читает всё, что читает вкладка, и разбирает по офсетам MSPHelper.js."""
    out = {}
    api = fc.call(1)
    board = fc.call(4)
    out["api"] = f"{u8(api,0)}.{u8(api,1)}.{u8(api,2)}"
    out["board"] = board[:4].decode() + "/" + text(board, 8) + "/" + text(board, 8 + 1 + len("STM32F405") + 1) \
        if board else "?"
    out["features"] = u32(fc.call(36), 0)
    out["arming"] = fc.call(61)
    out["beeper"] = fc.call(184)
    out["advanced"] = fc.call(90)
    out["rx"] = fc.call(44)
    out["sensor"] = fc.call(96)
    out["name"] = fc.call(10).decode("latin-1")
    out["craft"] = text(fc.call(0x3006, bytes([2]), v2=True), 1)
    out["pilot"] = text(fc.call(0x3006, bytes([1]), v2=True), 1)
    return out


def main() -> int:
    eeprom = Path(tempfile.mkdtemp()) / "eeprom.json"
    proc, fc = start(PORT, eeprom)

    print("=" * 78)
    print("ВКЛАДКА «КОНФИГУРАЦИЯ»: что эмулятор отдаёт (разбор как в MSPHelper.js)")
    print("=" * 78)
    a = read_all(fc)
    print(f"Версия API        : {a['api']}")
    print(f"Плата             : {a['board']}")
    print(f"Имя дрона         : {a['craft']!r}   (MSP_NAME: {a['name']!r})")
    print(f"Имя пилота        : {a['pilot']!r}")

    print("\n-- Активные настройки (MSP_ADVANCED_CONFIG) --")
    adv = a["advanced"]
    check(len(adv) == 20, f"длина ответа {len(adv)} байт (нужно 20)")
    print(f"   Частота цикла PID : pid_process_denom = {u8(adv,1)} → {8000 // max(u8(adv,1),1)} кГц")
    print(f"   Протокол моторов  : {u8(adv,3)} (0=PWM…7=DSHOT600, 9=DISABLED)")
    print(f"   Частота ШИМ       : {u16(adv,4)} Гц")
    print(f"   Idle (digital)    : {u16(adv,6)} = {u16(adv,6)/10:.1f} %")
    print(f"   Инверсия мотора   : {u8(adv,9)} | гиро: to_use={u8(adv,10)}, high_fsr={u8(adv,11)}, "
          f"noise={u8(adv,12)}, calib={u16(adv,13)} мс, overflow={u8(adv,17)}")
    print(f"   Debug mode        : {u8(adv,18)} из {u8(adv,19)}")

    print("\n-- Прочие функции (MSP_FEATURE_CONFIG) --")
    feats = a["features"]
    check(feats != 0xFFFFFFFF and feats != -1, f"маска фич получена: 0x{feats & 0xFFFFFFFF:08x}")
    print(f"   SOFTSERIAL (бит 6) включён: {bool(feats & (1 << 6))}")
    print(f"   OSD (бит 18) включён      : {bool(feats & (1 << 18))}")
    print(f"   TELEMETRY (бит 10)        : {bool(feats & (1 << 10))}")

    print("\n-- Arming (MSP_ARMING_CONFIG) --")
    arm = a["arming"]
    check(len(arm) == 3, f"длина ответа {len(arm)} байт (нужно 3)")
    print(f"   auto_disarm_delay = {u8(arm,0)} (0.1 с) | small_angle = {u8(arm,2)}°")

    print("\n-- Звуковые сигналы (MSP_BEEPER_CONFIG) --")
    bp = a["beeper"]
    check(len(bp) == 9, f"длина ответа {len(bp)} байт (нужно 9)")
    print(f"   OFF-флаги = 0x{u32(bp,0):08x}, тон DShot = {u8(bp,4)}, условия = 0x{u32(bp,5):08x}")

    print("\n-- Приёмник/RC (MSP_RX_CONFIG) --")
    rx = a["rx"]
    check(len(rx) == 38, f"длина ответа {len(rx)} байт (нужно 38)")
    print(f"   provider = {u8(rx,0)} (9=CRSF) | max_check={u16(rx,1)} mid_rc={u16(rx,3)} min_check={u16(rx,5)}")
    print(f"   rx_min_usec={u16(rx,8)} rx_max_usec={u16(rx,10)} | airmode={max(0,(u16(rx,14)-1000))//10} %")
    print(f"   RC smoothing: setpoint={u8(rx,25)} feedforward={u8(rx,26)} "
          f"auto={u8(rx,30)} mode={u8(rx,31)} | USB HID={u8(rx,29)}")

    print("\n-- Датчики (MSP_SENSOR_CONFIG) --")
    sc = a["sensor"]
    check(len(sc) == 4, f"длина ответа {len(sc)} байт (нужно 4)")
    print(f"   acc={u8(sc,0)} baro={u8(sc,1)} mag={u8(sc,2)} sonar={u8(sc,3)}  (0=AUTO, 1=NONE)")

    # ------------------------------------------------------------------ запись и перезапуск
    print("\n" + "=" * 78)
    print("МЕНЯЕМ КАЖДЫЙ ПАРАМЕТР → SAVE → REBOOT → ПЕРЕЗАПУСК ПРОЦЕССА → ПРОВЕРКА")
    print("=" * 78)

    # 1. Advanced: pid_process_denom = 2 (4 кГц), протокол = DSHOT300 (6), idle = 700 (7.0%)
    adv_req = bytearray(adv)
    adv_req[1] = 2
    adv_req[3] = 6
    adv_req[6:8] = (700).to_bytes(2, "little")
    adv_req[13:15] = (200).to_bytes(2, "little")     # gyro_calib_duration
    adv_req[18] = 3                                   # debug_mode
    fc.call(91, bytes(adv_req[:19]))                  # интерфейс шлёт 19 байт

    # 2. RX: provider SPEK2048 (1), min/max check, airmode 25 %, smoothing
    rx_req = bytearray(rx)
    rx_req[0] = 1
    rx_req[1:3] = (1900).to_bytes(2, "little")
    rx_req[5:7] = (1100).to_bytes(2, "little")
    rx_req[14:16] = (25 * 10 + 1000).to_bytes(2, "little")
    rx_req[25], rx_req[26], rx_req[30] = 30, 40, 50
    fc.call(45, bytes(rx_req))

    # 3. Arming: auto_disarm_delay = 20 (2 c), small_angle = 45
    fc.call(62, bytes([20, 0, 45]))

    # 4. Фичи: включить SOFTSERIAL (бит 6) и LED_STRIP (16)
    feats_new = feats | (1 << 6) | (1 << 16)
    fc.call(37, (feats_new & 0xFFFFFFFF).to_bytes(4, "little"))

    # 5. Имена: дрон и пилот (MSP2_SET_TEXT)
    craft, pilot = b"CFG-TEST", b"TEST PILOT"
    fc.call(0x3007, bytes([2, len(craft)]) + craft, v2=True)
    fc.call(0x3007, bytes([1, len(pilot)]) + pilot, v2=True)

    # 6. Датчики: выключить акселерометр
    fc.call(97, bytes([1, 0, 1, 0]))

    # 7. Beeper: тон DShot = 5, условия = 0x02
    fc.call(185, u32(bp, 0).to_bytes(4, "little") + bytes([5]) + (2).to_bytes(4, "little"))

    fc.call(250)                       # MSP_EEPROM_WRITE (кнопка Save)
    fc.call(68, bytes([0]))            # MSP_SET_REBOOT
    time.sleep(1.5)
    fc.close()
    stop(proc)

    proc, fc2 = start(PORT, eeprom)
    b = read_all(fc2)
    print()
    check(u8(b["advanced"], 1) == 2, f"PID-цикл: pid_process_denom = {u8(b['advanced'],1)} (ждали 2)")
    check(u8(b["advanced"], 3) == 6, f"протокол моторов = {u8(b['advanced'],3)} (ждали DSHOT300 = 6)")
    check(u16(b["advanced"], 6) == 700, f"digital idle = {u16(b['advanced'],6)} (ждали 700)")
    check(u16(b["advanced"], 13) == 200, f"gyro_calib_duration = {u16(b['advanced'],13)} (ждали 200)")
    check(u8(b["advanced"], 18) == 3, f"debug_mode = {u8(b['advanced'],18)} (ждали 3)")
    check(u8(b["rx"], 0) == 1, f"serialrx_provider = {u8(b['rx'],0)} (ждали 1 = SPEK2048)")
    check(u16(b["rx"], 1) == 1900 and u16(b["rx"], 5) == 1100,
          f"max_check={u16(b['rx'],1)}, min_check={u16(b['rx'],5)} (ждали 1900/1100)")
    check(max(0, (u16(b["rx"], 14) - 1000)) // 10 == 25,
          f"airmode порог = {max(0,(u16(b['rx'],14)-1000))//10} % (ждали 25)")
    check((u8(b["rx"], 25), u8(b["rx"], 26), u8(b["rx"], 30)) == (30, 40, 50),
          f"RC smoothing = {(u8(b['rx'],25), u8(b['rx'],26), u8(b['rx'],30))} (ждали 30/40/50)")
    check(u8(b["arming"], 0) == 20 and u8(b["arming"], 2) == 45,
          f"arming: auto_disarm={u8(b['arming'],0)}, small_angle={u8(b['arming'],2)}")
    check(b["features"] & (1 << 6) and b["features"] & (1 << 16),
          f"фичи: SOFTSERIAL и LED_STRIP сохранены (0x{b['features'] & 0xFFFFFFFF:08x})")
    check(b["craft"] == "CFG-TEST", f"имя дрона = {b['craft']!r}")
    check(b["pilot"] == "TEST PILOT", f"имя пилота = {b['pilot']!r}")
    check(u8(b["sensor"], 0) == 1, f"датчики: acc = {u8(b['sensor'],0)} (1 = выключен)")
    check(u8(b["beeper"], 4) == 5 and u32(b["beeper"], 5) == 2,
          f"beeper: тон = {u8(b['beeper'],4)}, условия = {u32(b['beeper'],5)}")

    # CLI должен показывать те же значения, что интерфейс
    cli = Fc(PORT)
    for cmd, expected in (
        ("get pid_process_denom", "pid_process_denom = 2"),
        ("get small_angle", "small_angle = 45"),
        ("get serialrx_provider", "serialrx_provider = SPEK2048"),
        ("get motor_pwm_protocol", "motor_pwm_protocol = DSHOT300"),
        ("get dshot_idle_value", "dshot_idle_value = 700"),
        ("get gyro_calib_duration", "gyro_calib_duration = 200"),
    ):
        out = cli.call(0x3012, cmd.encode(), v2=True)          # MSP2_CLI_COMMAND
        text_out = out[3:].decode("latin-1") if len(out) > 3 else ""
        check(expected in text_out, f"CLI '{cmd}' → {'найдено' if expected in text_out else 'НЕТ: ' + text_out[:60]!r}")
    cli.close()

    fc2.close()
    stop(proc)

    # ------------------------------------------------ «старый флеш» с устаревшим эхом
    print("\n" + "=" * 78)
    print("СТАРЫЙ «ФЛЕШ»: ЭХО ПРЕЖНИХ СБОРОК НЕ ДОЛЖНО ГЛУШИТЬ ЖИВЫЕ ЗНАЧЕНИЯ")
    print("=" * 78)

    import json as _json

    old_eeprom = eeprom.with_name("eeprom_stale.json")
    state = _json.loads(eeprom.read_text())
    # Прежние сборки писали в «флеш» эхо записанных payload'ов. Подкладываем такие эхо-значения
    # для блоков, которые теперь строятся из состояния FC (фичи, RX_CONFIG, arming, сенсоры,
    # сервы, слайдеры, имена), и убеждаемся, что вкладка видит настоящие значения.
    stale = {
        "36": (0x00000000).to_bytes(4, "little").hex(),                 # фичи: «всё выключено»
        "44": (bytes([9]) + (1000).to_bytes(2, "little") + (1500).to_bytes(2, "little")
               + (1000).to_bytes(2, "little") + bytes(1) + (885).to_bytes(2, "little")
               + (2115).to_bytes(2, "little") + bytes(26)).hex(),       # RX_CONFIG: «заводской»
        "61": bytes([0, 0, 20]).hex(),                                  # arming: small_angle 20
        "120": bytes(96).hex(),                                         # серво: всё по нулям
        "140": bytes(53).hex(),                                         # слайдеры: нули
        "0x3006": "0000",                                              # имена: пусто
    }
    state["config_payloads"] = dict(stale)
    state["features"] = ["RX_SERIAL"]
    state.pop("servo_params", None)
    old_eeprom.write_text(_json.dumps(state))
    print(f"   «флеш» с эхом: {', '.join(sorted(stale))} (фичи — только RX_SERIAL)")

    proc, fc3 = start(PORT, old_eeprom)
    feats = u32(fc3.call(36), 0)
    check(feats & (1 << 6) == 0 and feats & (1 << 3) != 0,
          f"фичи читаются из состояния, а не из эха: 0x{feats:08X} (RX_SERIAL включён, SOFTSERIAL выключен)")
    check(u8(fc3.call(61), 2) == 45, f"arming: small_angle = {u8(fc3.call(61), 2)} — из состояния, а не 20 из эха")
    expected_provider = int(state.get("settings", {}).get("serialrx_provider", -1))
    check(u8(fc3.call(44), 0) == expected_provider,
          f"RX_CONFIG: serialrx_provider = {u8(fc3.call(44), 0)} — как в состоянии FC "
          f"({expected_provider} = SPEK2048, записан ранее тестом), а не 9 из эха")

    # вкладка «Конфигурация»: включаем SERVO_TILT (бит 5) и SOFTSERIAL (бит 6) и сохраняем
    fc3.call(37, (feats | (1 << 6) | (1 << 5)).to_bytes(4, "little"))
    after = u32(fc3.call(36), 0)
    check(after & (1 << 6) and after & (1 << 5),
          f"после «Сохранить» фичи видны сразу: 0x{after:08X} (SOFTSERIAL + SERVO_TILT) — эхо больше не глушит")
    ports = fc3.call(4105, v2=True)
    ids = [ports[1 + i * 9] for i in range(ports[0])]
    check(30 in ids and 31 in ids, f"SOFTSERIAL появился во вкладке «Порты»: {ids}")

    fc3.call(250)                 # MSP_EEPROM_WRITE
    fc3.call(68, bytes([0]))      # MSP_SET_REBOOT
    time.sleep(1.0)
    fc3.close()
    stop(proc)

    proc, fc4 = start(PORT, old_eeprom)
    feats_after = u32(fc4.call(36), 0)
    ports = fc4.call(4105, v2=True)
    ids = [ports[1 + i * 9] for i in range(ports[0])]
    check(feats_after & (1 << 6) and feats_after & (1 << 5),
          f"SERVO_TILT и SOFTSERIAL пережили перезапуск: 0x{feats_after:08X}")
    check(30 in ids and 31 in ids, f"порты SOFTSERIAL1/2 на месте после перезапуска: {ids}")
    feature_out = fc4.call(0x3012, b"feature", v2=True)
    feature_text = feature_out[3:].decode("latin-1") if len(feature_out) > 3 else ""
    check("SERVO_TILT" in feature_text and "SOFTSERIAL" in feature_text,
          "CLI 'feature': те же фичи, что на вкладке («vtx»/«led»-строки выводятся из состояния)")

    saved = _json.loads(old_eeprom.read_text())
    check("36" not in (saved.get("config_payloads") or {}),
          f"устаревшее эхо фич удалено из «флеша» при сохранении (остались ключи: "
          f"{sorted(saved.get('config_payloads') or {})})")
    # сквозные блоки по-прежнему работают через эхо (их эмулятор не моделирует)
    fc4.call(185, (1).to_bytes(4, "little") + bytes([7]) + (0x02).to_bytes(4, "little"))
    fc4.call(250)
    fc4.close()
    stop(proc)
    proc, fc5 = start(PORT, old_eeprom)
    beeper = fc5.call(184)
    check(len(beeper) > 4 and u8(beeper, 4) == 7,
          f"сквозная запись (beeper, эхо-путь) работает как раньше: тон = {u8(beeper, 4)}")
    fc5.close()
    stop(proc)
    old_eeprom.unlink(missing_ok=True)

    print()
    print("=" * 78)
    if FAILS:
        print(f"ПРОВАЛЕНО проверок: {len(FAILS)}")
        for f in FAILS:
            print("  -", f)
        return 1
    print("ВСЕ ПАРАМЕТРЫ ВКЛАДКИ «КОНФИГУРАЦИЯ» ПРОЙДЕНЫ (чтение, запись, сохранение после перезапуска)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

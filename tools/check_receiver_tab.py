#!/usr/bin/env python3
"""Проверка вкладки «Приёмник»: каналы, «Предпросмотр» модели, карта каналов, сохранение.

Скрипт повторяет то, что делает вкладка Configurator 2026.6.2 (ReceiverTab.vue):

  loadConfig(): MSP_FEATURE_CONFIG(36), MSP_RC(105), MSP_RSSI_CONFIG(50),
                MSP_RC_TUNING(111), MSP_RX_MAP(64), MSP_RC_DEADBAND(125),
                MSP_RX_CONFIG(44), MSP_MIXER_CONFIG(42);
  Save:         MSP_SET_RX_MAP(65) -> MSP_SET_RSSI_CONFIG(51) ->
                MSP_SET_RC_DEADBAND(218) -> MSP_SET_RX_CONFIG(45) ->
                MSP_EEPROM_WRITE(250) -> MSP_SET_REBOOT(68).

Отдельно проверяется «живая» аппаратура (ближе к концу скрипта):
  POST /api/rc {"bind": false}      — «развязать»: кадров нет, «Этап 1», флаг RX_FAILSAFE;
  POST /api/rc {"bind": true}       — «связать»: 2 с поиска, потом каналы снова живые;
  SBUS/CRSF-кадры (tools/tx.py)     — то, что читает `tools/tx.py wire --port COM5` из настоящего
                                      приёмника в USB-UART переходнике (или `joystick` — USB-пульт).

Ключевое, что проверяет «Предпросмотр»:
  Configurator вращает модель по каналам 0…2 (Крен/Тангаж/Рыск) через
  RateCurve.rcCommandRawToDegreesPerSecond(). Прошивка хранит rcData[] в порядке
  `rcChannelLetters = "AERT"` (rx.c), то есть индекс 2 — рыск, индекс 3 — газ.
  На столе стики стоят в центре (1500), газ внизу (1000) — модель стоит.
  Раньше эмулятор отдавал Рыск=1000 / Газ=1500, и модель крутилась на ~2000 °/с.

Ожидаемые значения — из прошивки Betaflight 4.5.4:
  serialrx_provider CRSF (9), диапазон стиков 1000/1500/2000, rx_min_usec 885,
  rx_max_usec 2115, airmode_start_throttle_percent 30, rc_smoothing ON (mode AUTО),
  auto_factor 30, карта AETR1234 -> [0, 1, 3, 2, 4, 5, 6, 7], дедбенды 0/0/50,
  рейты ACTUAL: rc_rate 7 (70), srate 67 (670), thr_mid 50, лимит 1998°/с,
  каналов 16 (CRSF_MAX_CHANNEL), микшер QUADX (2).

Запуск:  python3 tools/check_receiver_tab.py      (Windows: python tools\\check_receiver_tab.py)
"""
from __future__ import annotations

import json as _json
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from bfemu.msp import encode_v1, encode_v2  # noqa: E402

PORT = 16101
FAILS: list[str] = []
STICK_NAMES = ["Крен", "Тангаж", "Рыск", "Газ"]
PROVIDERS = {0: "NONE", 2: "SBUS", 9: "CRSF", 11: "SRXL", 12: "CUSTOM", 13: "FPORT", 14: "SRXL2"}
MID_RC, MAX_RC = 1500, 2000
SMOOTHING = {0: "ВЫКЛ", 1: "ВКЛ (Авто)"}


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


def start(port: int, eeprom: Path, http: int | None = None):
    command = [sys.executable, "-m", "bfemu", "--tcp", f"127.0.0.1:{port}", "--eeprom", str(eeprom), "--quiet"]
    if http:
        command += ["--http", str(http)]
    proc = subprocess.Popen(
        command,
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


# ----------------------------------------------------------- математика предпросмотра
def rc_command(rc_data: int, rc_rate: int, deadband: int) -> float:
    """RateCurve.rcCommand() из Configurator: снимает дедбенд и нормирует стик."""
    tmp = min(max(abs(rc_data - MID_RC) - deadband, 0), 500)
    result = tmp * rc_rate
    return -result if rc_data < MID_RC else result


def rc_command_raw_to_degrees_per_second(rc_data: int, rate: float, rc_rate: float,
                                         rc_expo: float, deadband: int, limit: float) -> float:
    """RateCurve.rcCommandRawToDegreesPerSecond() для ACTUAL-рейтов (rates_type = 3)."""
    command_f = rc_command(rc_data, 1, deadband) / (500 - deadband)
    command_abs = abs(command_f)
    # getActualRates(): expof = |f| * (f^5 * expo + f * (1 - expo))
    expof = command_abs * (command_f**5 * rc_expo + command_f * (1 - rc_expo))
    angular_vel = command_f * rc_rate + max(0.0, rate - rc_rate) * expof
    return max(-limit, min(limit, angular_vel))


def spin_rates(channels: list, rc_tuning: bytes, deadband: int) -> list:
    """Угловые скорости (°/с), с которыми предпросмотр крутит модель: крен, тангаж, рыск."""
    # getCurrentRates() для ACTUAL: rate и rc_rate умножаются на 1000
    roll_rate, pitch_rate, yaw_rate = rc_tuning[2] * 1000, rc_tuning[3] * 1000, rc_tuning[4] * 1000
    rc_rate, rc_pitch_rate = rc_tuning[0] * 1000, rc_tuning[12] * 1000
    rc_expo, rc_pitch_expo = rc_tuning[1] / 100, rc_tuning[11] / 100
    rc_yaw_expo = rc_tuning[10] / 100
    limit_roll, limit_pitch, limit_yaw = u16(rc_tuning, 16), u16(rc_tuning, 18), u16(rc_tuning, 20)
    return [
        rc_command_raw_to_degrees_per_second(channels[0], roll_rate, rc_rate, rc_expo, deadband, limit_roll),
        rc_command_raw_to_degrees_per_second(channels[1], pitch_rate, rc_pitch_rate, rc_pitch_expo, deadband, limit_pitch),
        rc_command_raw_to_degrees_per_second(channels[2], yaw_rate, rc_rate, rc_yaw_expo, deadband, limit_yaw),
    ]


def read_channels(fc: Fc) -> list:
    data = fc.call(105)
    return [u16(data, i * 2) for i in range(len(data) // 2)]


def read_rx_config(fc: Fc) -> dict:
    d = fc.call(44)
    return {
        "raw": d,
        "provider": d[0],
        "stick_max": u16(d, 1),
        "stick_center": u16(d, 3),
        "stick_min": u16(d, 5),
        "rx_min_usec": u16(d, 8),
        "rx_max_usec": u16(d, 10),
        "airmode": (u16(d, 14) - 1000) // 10,
        "setpoint_cutoff": d[25],
        "feedforward_cutoff": d[26],
        "auto_factor": d[30],
        "rc_smoothing": d[31],
    }


def read_rc_tuning(fc: Fc) -> bytes:
    return fc.call(111)


def read_rx_map(fc: Fc) -> list:
    return list(fc.call(64))


def read_deadband(fc: Fc) -> list:
    return list(fc.call(125))


def main():
    eeprom = Path(tempfile.gettempdir()) / "bfemu_receiver_check.json"
    if eeprom.exists():
        eeprom.unlink()

    print("=" * 78)
    print("ВКЛАДКА «ПРИЁМНИК»: ЧТО ЗАГРУЖАЕТ ВКЛАДКА (MSP_FEATURE_CONFIG → MSP_MIXER_CONFIG)")
    print("=" * 78)
    proc, fc = start(PORT, eeprom)

    features = fc.call(36)
    print(f"   MSP_FEATURE_CONFIG: 0x{int.from_bytes(features, 'little'):08X}")
    check(len(features) == 4, "MSP_FEATURE_CONFIG: 4 байта (режим приёмника и RSSI_ADC берутся отсюда)")

    channels = read_channels(fc)
    check(len(channels) == 16, f"MSP_RC: {len(channels)} каналов (CRSF_MAX_CHANNEL)")
    check(channels[:3] == [MID_RC] * 3, f"стики в центре: {channels[:3]} мкс (Крен/Тангаж/Рыск)")
    check(channels[3] == 1000, f"газ внизу: {channels[3]} мкс (в «Предпросмотре» газ не крутит модель)")
    check(all(950 <= value <= 1050 for value in channels[4:]), "AUX 1…12 выключены (тумблеры на столе)")

    rc_tuning = read_rc_tuning(fc)
    deadband = read_deadband(fc)
    rates = spin_rates(channels, rc_tuning, deadband[0])
    check(all(abs(rate) < 0.5 for rate in rates),
          f"«Предпросмотр»: угловые скорости {[round(r, 1) for r in rates]} °/с — модель стоит")

    swapped = list(channels)
    swapped[2], swapped[3] = channels[3], channels[2]
    old_rates = spin_rates(swapped, rc_tuning, deadband[0])
    print(f"   (для сравнения: с прежними данными Рыск=1000/Газ=1500 было бы "
          f"{[round(r) for r in old_rates]} °/с — полный разворот за {360 / abs(old_rates[2]):.2f} с)")
    check(abs(old_rates[2]) > 1000, f"контроль: перепутанные Рыск/Газ дали бы {abs(old_rates[2]):.0f} °/с")

    rssi = fc.call(50)
    check(len(rssi) == 1 and rssi[0] == 0, f"MSP_RSSI_CONFIG: 1 байт = {list(rssi)} (аналоговый RSSI выключен)")

    check(len(rc_tuning) == 23, f"MSP_RC_TUNING: {len(rc_tuning)} байта")
    check(rc_tuning[0] == 7 and rc_tuning[2] == 67 and rc_tuning[6] == 50 and rc_tuning[22] == 3,
          f"рейты ACTUAL: rc_rate {rc_tuning[0]} (70), srate {rc_tuning[2]} (670), thr_mid {rc_tuning[6]} (50%)")
    check(u16(rc_tuning, 16) == 1998 and u16(rc_tuning, 20) == 1998,
          f"лимиты скорости {u16(rc_tuning, 16)}°/с — «Предпросмотр» ограничен ими")

    rx_map = read_rx_map(fc)
    check(rx_map == [0, 1, 3, 2, 4, 5, 6, 7],
          f"MSP_RX_MAP: {rx_map} = AETR1234 (первые четыре канала: A→0, E→1, R→3, T→2)")

    check(deadband == [0, 0, 40, 50, 0],
          f"MSP_RC_DEADBAND: RC {deadband[0]} / Yaw {deadband[1]} / 3D-газ {u16(bytes(deadband), 3)} мкс")

    rx = read_rx_config(fc)
    check(len(rx["raw"]) == 38, f"MSP_RX_CONFIG: {len(rx['raw'])} байт (раскладка MSPHelper.js для API 1.46)")
    check(rx["provider"] == 9, f"Serial RX: {PROVIDERS.get(rx['provider'], rx['provider'])} (provider {rx['provider']})")
    check((rx["stick_min"], rx["stick_center"], rx["stick_max"]) == (1050, 1500, 2000),
          f"«Диапазон стиков»: {rx['stick_min']} / {rx['stick_center']} / {rx['stick_max']} мкс")
    check((rx["rx_min_usec"], rx["rx_max_usec"]) == (885, 2115),
          f"диапазон импульсов приёмника: {rx['rx_min_usec']}…{rx['rx_max_usec']} мкс")
    check(rx["airmode"] == 30, f"«Порог включения Airmode»: {rx['airmode']}%")
    check(rx["rc_smoothing"] == 1 and rx["auto_factor"] == 30
          and rx["setpoint_cutoff"] == 0 and rx["feedforward_cutoff"] == 0,
          f"RC-сглаживание: {SMOOTHING[rx['rc_smoothing']]} / множитель {rx['auto_factor']} / "
          f"срезы {rx['setpoint_cutoff']} и {rx['feedforward_cutoff']} (0 = Авто)")

    mixer = fc.call(42)
    check(len(mixer) == 2 and mixer[0] == 2, f"MSP_MIXER_CONFIG: {list(mixer)} = QUADX (микшер коптера)")


    # ------------------------------------------------------------------ правки
    print("\n" + "=" * 78)
    print("ПРАВИМ ВСЁ, ЧТО ЕСТЬ НА ВКЛАДКЕ (как перед «Сохранить»)")
    print("=" * 78)

    fc.call(65, bytes([3, 0, 1, 2, 4, 5, 6, 7]))                 # карта TAER1234
    check(read_rx_map(fc) == [3, 0, 1, 2, 4, 5, 6, 7], f"карта каналов сохранена: {read_rx_map(fc)} (TAER1234)")

    fc.call(51, bytes([7]))                                     # аналоговый RSSI: канал 7
    check(fc.call(50) == b"\x07", f"MSP_SET_RSSI_CONFIG (51): «Канал RSSI» = {fc.call(50)[0]}")

    fc.call(218, bytes([4, 8, 40]) + (60).to_bytes(2, "little"))
    db2 = read_deadband(fc)
    check((db2[0], db2[1], u16(bytes(db2), 3)) == (4, 8, 60),
          f"дедбенды сохранены: RC {db2[0]} / Yaw {db2[1]} / 3D-газ {u16(bytes(db2), 3)}")

    edit = bytearray(rx["raw"])
    edit[0] = 2                                                  # SBUS
    edit[1:3] = (2000).to_bytes(2, "little")                     # «Диапазон стиков»: макс (max_check)
    edit[3:5] = (1500).to_bytes(2, "little")                     # центр (mid_rc)
    edit[5:7] = (1000).to_bytes(2, "little")                     # мин (min_check)
    edit[14:16] = (1250).to_bytes(2, "little")                    # Airmode 25%
    edit[25] = 30                                                 # срез сетпоинта 30 Гц
    fc.call(45, bytes(edit))
    rx2 = read_rx_config(fc)
    check(rx2["provider"] == 2 and (rx2["stick_min"], rx2["stick_center"], rx2["stick_max"]) == (1000, 1500, 2000),
          f"MSP_SET_RX_CONFIG (45): {PROVIDERS[rx2['provider']]}, стики "
          f"{rx2['stick_min']}/{rx2['stick_center']}/{rx2['stick_max']}")
    check((rx2["airmode"], rx2["setpoint_cutoff"]) == (25, 30),
          f"порог Airmode {rx2['airmode']}%, срез сетпоинта {rx2['setpoint_cutoff']} Гц (не «Авто»)")

    print("\n-- «Сохранить» + «Сохранить и перезагрузить» (SET_* -> EEPROM_WRITE -> ребут) --")
    fc.call(250)
    fc.call(68, b"\x00")
    time.sleep(1.0)
    fc.close()
    stop(proc)

    print("\n" + "=" * 78)
    print("ПОЛНЫЙ ПЕРЕЗАПУСК ЭМУЛЯТОРА")
    print("=" * 78)
    proc, fc2 = start(PORT, eeprom)
    rx3 = read_rx_config(fc2)
    check(read_rx_map(fc2) == [3, 0, 1, 2, 4, 5, 6, 7], f"карта каналов пережила перезапуск: {read_rx_map(fc2)}")
    check(fc2.call(50) == b"\x07", "«Канал RSSI» пережил перезапуск: 7")
    db3 = read_deadband(fc2)
    check((db3[0], db3[1], u16(bytes(db3), 3)) == (4, 8, 60), "дедбенды пережили перезапуск: 4 / 8 / 60")
    check(rx3["provider"] == 2 and rx3["airmode"] == 25 and rx3["setpoint_cutoff"] == 30,
          f"RX_CONFIG пережил перезапуск: {PROVIDERS[rx3['provider']]}, Airmode {rx3['airmode']}%, срез {rx3['setpoint_cutoff']} Гц")

    channels3 = read_channels(fc2)
    rates3 = spin_rates(channels3, read_rc_tuning(fc2), read_deadband(fc2)[0])
    check(channels3[:4] == [1500, 1500, 1500, 1000] and all(abs(r) < 0.5 for r in rates3),
          f"каналы после перезапуска: {channels3[:4]} — «Предпросмотр» по-прежнему стоит")

    for command, expected in (
        ("get serialrx_provider", "serialrx_provider = SBUS"),
        ("get rssi_channel", "rssi_channel = 7"),
        ("get deadband", "deadband = 4"),
        ("get yaw_deadband", "yaw_deadband = 8"),
        ("get 3d_deadband_throttle", "3d_deadband_throttle = 60"),
        ("get rx_min_usec", "rx_min_usec = 885"),
        ("get max_check", "max_check = 2000"),
        ("get mid_rc", "mid_rc = 1500"),
        ("get min_check", "min_check = 1000"),
    ):
        out = fc2.cli(command)
        check(expected in out, f"CLI '{command}' → {'найдено' if expected in out else 'НЕТ: ' + out.strip()[:45]!r}")

    out = fc2.cli("map")
    check("map TAER1234" in out, f"CLI 'map' → {'map TAER1234' if 'map TAER1234' in out else out.strip()[:45]!r}")
    out = fc2.cli("dump")
    check("map TAER1234" in out and "set rssi_channel = 7" in out, "CLI 'dump' показывает карту и RSSI-канал")

    print("\n-- 'defaults': заводские значения возвращаются --")
    fc2.cli("defaults")
    time.sleep(0.7)
    fc2.close()
    stop(proc)
    proc, fc3 = start(PORT, eeprom)
    check(read_rx_map(fc3) == [0, 1, 3, 2, 4, 5, 6, 7], f"карта снова AETR1234: {read_rx_map(fc3)}")
    check(fc3.call(50) == b"\x00", "«Канал RSSI» снова 0")
    check(read_deadband(fc3) == [0, 0, 40, 50, 0], f"дедбенды снова заводские: {read_deadband(fc3)}")
    rx4 = read_rx_config(fc3)
    check(rx4["provider"] == 9 and (rx4["stick_min"], rx4["stick_max"]) == (1050, 2000),
          f"RX_CONFIG снова заводской: {PROVIDERS[rx4['provider']]}, стики {rx4['stick_min']}…{rx4['stick_max']}")
    channels4 = read_channels(fc3)
    check(channels4[:4] == [1500, 1500, 1500, 1000], f"каналы снова на столе: {channels4[:4]}")
    fc3.close()
    stop(proc)

    # ------------------------------------------------- виртуальная аппаратура («пульт»)
    print("\n" + "=" * 78)
    print("ВИРТУАЛЬНАЯ АППАРАТУРА: СТИКИ ДВИГАЮТСЯ, КАНАЛЫ РАБОТАЮТ")
    print("=" * 78)
    http_port = PORT + 1
    tx_proc, tx_fc = start(PORT, eeprom, http=http_port)

    def tx_post(payload):
        request = urllib.request.Request(
            f"http://127.0.0.1:{http_port}/api/rc",
            data=_json.dumps(payload).encode(), headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(request, timeout=4) as response:
                return response.status, _json.loads(response.read())
        except urllib.error.HTTPError as exc:
            return exc.code, _json.loads(exc.read())

    def channels_of(client: Fc) -> list:
        data = client.call(105)
        return [u16(data, i * 2) for i in range(len(data) // 2)]

    state = _json.loads(urllib.request.urlopen(f"http://127.0.0.1:{http_port}/api/rc", timeout=4).read())
    check(state["channels"] == [1500, 1500, 1500, 1000] + [1000] * 12,
          f"аппаратура подключена, стики стоят: {state['channels'][:4]}")

    status, body = tx_post({"roll": 1600, "yaw": 1400, "throttle": 1200, "aux1": 2000})
    check(status == 200 and body["applied"] == 4, f"POST /api/rc: применено каналов — {body.get('applied')}")
    moved = channels_of(tx_fc)
    check(moved[:4] == [1600, 1500, 1400, 1200],
          f"MSP_RC отдаёт движения стиков: крен {moved[0]}, тангаж {moved[1]}, рыск {moved[2]}, газ {moved[3]}")
    check(abs(moved[4] - 2000) <= 2, f"тумблер AUX 1 включён: {moved[4]} мкс — на вкладке «Режимы» сработают режимы")

    rates = spin_rates(moved, read_rc_tuning(tx_fc), read_deadband(tx_fc)[0])
    check(rates[0] != 0 and rates[1] == 0 and rates[2] != 0,
          f"«Предпросмотр» реагирует на стики: крен {rates[0]:.0f} °/с, тангаж {rates[1]:.0f} °/с, рыск {rates[2]:.0f} °/с")

    page = urllib.request.urlopen(f"http://127.0.0.1:{http_port}/tx", timeout=4).read().decode("utf-8")
    check('id="roll"' in page and "Стики в центр" in page and "/api/rc" in page,
          "страница аппаратуры /tx отдаётся (четыре стика, тумблеры AUX 1…12, кнопки)")

    status, body = tx_post({"bogus": 1500})
    check(status == 400 and "не распознаны" in body["error"], f"неизвестный канал отклонён: HTTP {status}")

    # «Выдернуть USB» у аппаратуры: прошивка отдаёт значения «Этапа 1» + флаг RX_FAILSAFE
    status, body = tx_post({"connected": False, "source": "check_receiver_tab"})
    check(status == 200 and body["failsafe"], "аппаратура отключена: прошивка ушла в failsafe")
    stage1 = channels_of(tx_fc)
    check(stage1[:4] == [1500, 1500, 1500, 900],
          f"значения «Этапа 1» (rx.c): стики в центре, газ {stage1[3]} мкс — как в вкладке Failsafe")
    flags = int.from_bytes(tx_fc.call(150)[17:21], "little")
    check(flags & (1 << 2) != 0,
          f"флаг ARMING_DISABLED_RX_FAILSAFE поднят (0x{flags:08X}) — вкладка покажет «потеря связи», полосы жёлтые")

    rates_failsafe = spin_rates(stage1, read_rc_tuning(tx_fc), read_deadband(tx_fc)[0])
    check(all(abs(r) < 0.5 for r in rates_failsafe),
          f"модель в «Предпросмотре» стоит при потере связи: {[round(r, 1) for r in rates_failsafe]} °/с")

    status, body = tx_post({"connected": True})
    check(status == 200 and not body["failsafe"], "аппаратура подключена обратно — связь восстановлена")

    # Вкладка «Порты»: без функции Serial RX приёмник не поднимается и каналов нет
    RX_SERIAL_MASK = 0x40
    ports_payload = tx_fc.call(4105, v2=True)
    port_count = ports_payload[0]
    stripped = bytearray(ports_payload)
    for i in range(port_count):
        if stripped[1 + i * 9] == 1:                       # UART2 — порт приёмника
            mask = int.from_bytes(stripped[1 + i * 9 + 1 : 1 + i * 9 + 5], "little") & ~RX_SERIAL_MASK
            stripped[1 + i * 9 + 1 : 1 + i * 9 + 5] = mask.to_bytes(4, "little")
    tx_fc.call(4106, bytes(stripped), v2=True)
    check(tx_fc.call(105) == b"", "Serial RX выключен во вкладке «Порты»: MSP_RC пуст — полосы каналов пустые")
    state = _json.loads(urllib.request.urlopen(f"http://127.0.0.1:{http_port}/api/rc", timeout=4).read())
    check(state["serial_ok"] is False and state["active_channels"] == 0,
          "API сообщает: приёмник не настроен (serial_ok=false, активных каналов 0)")
    page = urllib.request.urlopen(f"http://127.0.0.1:{http_port}/tx", timeout=4).read().decode("utf-8")
    check("btn_disconnect" in page and "port_hint" in page, "на странице /tx есть кнопки «Подключить/Отключить» и подсказка про «Порты»")

    tx_fc.call(4106, ports_payload, v2=True)
    check(len(tx_fc.call(105)) == 32, "Serial RX включён обратно — 16 каналов вернулись")

    tx_post({"reset": True})
    check(channels_of(tx_fc)[:4] == [1500, 1500, 1500, 1000], "«стики в центр»: каналы вернулись в исходное")

    # --- биндинг: как кнопка Bind на настоящем приёмнике («аппаратура нашла приёмник») ---
    status, body = tx_post({"bind": False, "source": "check_receiver_tab"})
    check(status == 200 and body["bind"]["state"] == "none" and not body["connected"],
          f"«Развязать»: приёмник кадров не отдаёт — {body['bind']['text']}")
    check(channels_of(tx_fc)[:4] == [1500, 1500, 1500, 900],
          "без бинда на вкладке «Приёмник» «Этап 1» — как при потере связи")

    status, body = tx_post({"bind": True, "bind_seconds": 1.2})
    check(status == 200 and body["bind"]["state"] == "binding",
          f"«Связать (Bind)»: идёт поиск приёмника ({body['bind']['seconds_left']} с)")
    check(channels_of(tx_fc)[:4] == [1500, 1500, 1500, 900],
          "пока идёт биндинг, каналов в эфире нет — полосы стоят")
    time.sleep(1.5)
    state = _json.loads(urllib.request.urlopen(f"http://127.0.0.1:{http_port}/api/rc", timeout=4).read())
    check(state["bind"]["state"] == "bound" and state["connected"],
          f"биндинг завершился сам: {state['bind']['text']}")
    tx_post({"roll": 1800})
    check(channels_of(tx_fc)[0] == 1800, "после бинда стики снова управляют каналами (крен 1800 мкс)")

    page = urllib.request.urlopen(f"http://127.0.0.1:{http_port}/tx", timeout=4).read().decode("utf-8")
    check("Связать (Bind)" in page and "Развязать" in page and "USB Joystick" in page,
          "страница /tx: кнопки «Связать (Bind)»/«Развязать» и инструкция про реальный пульт (Zorro)")

    # --- приёмник в COM-порту: SBUS/CRSF-кадры (проверяется кодек, без железа) ---
    sys.path.insert(0, str(ROOT / "tools"))
    import tx as tx_tool
    sbus = tx_tool.build_sbus_frame([988, 1500, 2012, 1000] + [1500] * 12)
    decoded = tx_tool.decode_sbus_frame(sbus)
    check(len(sbus) == 25 and tx_tool.decode_sbus_frame(sbus[:-1] + b"\x01") is None,
          f"SBUS: кадр {len(sbus)} Б (0x0F … 0x00), битый отбрасывается")
    check(decoded is not None and decoded[0][:3] == [988, 1500, 2012],
          f"SBUS-кодек 172..1811 -> 988..2012 мкс: {decoded[0][:3] if decoded else '—'}")
    decoder = tx_tool.WireDecoder("auto")
    feed = decoder.feed(b"\x00" + tx_tool.build_crsf_channels_frame([1100, 1500, 1900, 1200] + [1500] * 12))
    check(feed and feed[0]["protocol"] == "crsf" and feed[0]["channels"][:4] == [1100, 1500, 1900, 1200],
          "поток из COM-порта: кадр CRSF распознан (то же делает `tools/tx.py wire --port COM5`)")
    decoder = tx_tool.WireDecoder("auto")
    feed = decoder.feed(tx_tool.build_sbus_frame([1500] * 16, failsafe=True))
    check(feed and feed[0]["failsafe"], "SBUS-кадр с флагом failsafe -> «Этап 1» (аппаратуру выключили)")

    tx_post({"bind": True, "bind_seconds": 0})

    # --- у кого управление: физическая аппаратура или страница /tx, и разные органы AUX ---
    state = _json.loads(urllib.request.urlopen(f"http://127.0.0.1:{http_port}/api/rc", timeout=4).read())
    check(state["control"]["virtual"] and state["aux_types"]["aux3"] == "switch3"
          and state["aux_types"]["aux7"] == "knob",
          "управление по умолчанию у страницы /tx; AUX 3/4 — трёхпозиционные, AUX 7/10 — крутилки")

    status, body = tx_post({"control": "physical", "source": "check_receiver_tab"})
    check(status == 200 and body["control"]["physical"] and not body["connected"]
          and body["bind"]["state"] == "none",
          "«Подключить аппаратуру»: управление у физической аппаратуры — до биндинга связи нет")
    check(channels_of(tx_fc)[:4] == [1500, 1500, 1500, 900],
          "пока аппаратура не связана, на вкладке «Приёмник» «Этап 1» (полосы стоят)")
    tx_post({"bind": True, "bind_seconds": 0})
    check(channels_of(tx_fc)[:4] == [1800, 1500, 1500, 1000],
          f"«Связать (Bind)» активировал каналы: {channels_of(tx_fc)[:4]} — стик, сдвинутый ранее, "
          f"снова управляет (газ 1000 мкс, не «Этап 1»)")

    tx_post({"control": "virtual", "source": "check_receiver_tab"})
    check(channels_of(tx_fc)[:4] == [1500, 1500, 1500, 900],
          "«Отключить (выдернуть USB)»: управление вернулось странице, каналы снова «Этап 1»")
    tx_post({"bind": True, "bind_seconds": 0})
    check(channels_of(tx_fc)[:4] == [1800, 1500, 1500, 1000],
          f"повторный «Связать» — каналы снова живые: {channels_of(tx_fc)[:4]}")

    tx_post({"aux3": 1500, "aux7": 1700})
    check(channels_of(tx_fc)[6] == 1500 and channels_of(tx_fc)[10] == 1700,
          f"трёхпозиционный AUX 3 = {channels_of(tx_fc)[6]} (середина), крутилка AUX 7 = "
          f"{channels_of(tx_fc)[10]} мкс")
    # «чей ход — того и каналы»: данные физической аппаратуры отклоняются, пока управление у /tx
    tx_post({"control": "physical", "bind": True, "bind_seconds": 0})
    status, body = tx_post({"channels": {"roll": 1900}, "physical": True, "source": "пульт"})
    check(status == 200 and body["channels"][0] == 1900,
          "физическая аппаратура пишет каналы, когда управление у неё")
    tx_post({"layout_note": "канал ГАЗ назначен на ось 2 (Z), а в покое эта ось в ЦЕНТРЕ — "
                            "раскладка осей не та", "source": "пульт"})
    note_state = _json.loads(urllib.request.urlopen(f"http://127.0.0.1:{http_port}/api/rc", timeout=4).read())
    check("ГАЗ назначен на ось 2" in (note_state.get("layout_note") or ""),
          "инструмент аппаратуры передаёт на страницу предупреждение о неверной раскладке осей")

    status, body = tx_post({"channels": {"roll": 1300}, "source": "страница /tx"})
    check(status == 409 and body["state"]["channels"][0] == 1900,
          f"пока каналами владеет аппаратура, страница и скрипты не пишут (HTTP {status}), "
          f"крен остался {body['state']['channels'][0]}")
    tx_post({"control": "virtual"})
    status, body = tx_post({"channels": {"roll": 1000}, "physical": True, "source": "пульт"})
    check(status == 409 and body["state"]["channels"][0] == 1900,
          f"после «Отключить (выдернуть USB)» данные пульта отклоняются (HTTP {status}) — "
          f"один хозяин у каналов, без «срывов»")
    tx_post({"control": "physical", "bind": True, "bind_seconds": 0,
             "rc_map": {"roll": 0, "pitch": 1, "throttle": 2, "yaw": 4, "invert": ["throttle"]}})
    state = _json.loads(urllib.request.urlopen(f"http://127.0.0.1:{http_port}/api/rc", timeout=4).read())
    check(state["map"].get("yaw") == 4 and state["map"].get("invert") == ["throttle"],
          f"раскладка осей пульта видна в /api/rc: {state['map']}")
    tx_post({"control": "virtual", "bind": True, "bind_seconds": 0})
    tx_post({"reset": True})

    page = urllib.request.urlopen(f"http://127.0.0.1:{http_port}/tx", timeout=4).read().decode("utf-8")
    check("Подключить аппаратуру" in page and "Отключить (выдернуть USB)" in page
          and "buildAux" in page and "switch3" in page and "крутилка" in page,
          "страница /tx: кнопки подключения/отключения и три вида органов AUX на месте")

    tx_fc.close()
    stop(tx_proc)

    print()
    print("=" * 78)
    if FAILS:
        print(f"ПРОВАЛЕНО проверок: {len(FAILS)}")
        for item in FAILS:
            print("  -", item)
        return 1
    print("ВКЛАДКА «ПРИЁМНИК» РАБОТАЕТ: каналы как у прошивки, модель в «Предпросмотре» стоит, "
          "настройки сохраняются и переживают перезапуск")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

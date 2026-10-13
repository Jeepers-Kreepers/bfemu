#!/usr/bin/env python3
"""Проверка вкладки «Датчики»: поза, живые данные, триммирование, выравнивание, калибровка.

Скрипт повторяет то, что делает вкладка Configurator 2026.6.2 (SensorsTab.vue):

  1. загрузка: MSP_RAW_IMU (живые данные), MSP_ATTITUDE (поза для 3D-модели и приборов),
     MSP_SENSOR_CONFIG + MSP2_SENSOR_CONFIG_ACTIVE (какие датчики есть),
     MSP_SENSOR_ALIGNMENT (выравнивание гироскопа/акселерометра/магнитометра),
     MSP_BOARD_ALIGNMENT, MSP_ACC_TRIM (триммирование), MSP_COMPASS_CONFIG (склонение),
     MSP_MIXER_CONFIG;
  2. правка: трим по Roll/Pitch, выравнивание гироскопа и магнитометра, склонение,
     калибровка акселерометра (MSP_ACC_CALIBRATION);
  3. «Сохранить» → MSP_EEPROM_WRITE, «Сохранить и перезагрузить» → ребут;
  4. полный перезапуск процесса эмулятора: значения должны вернуться, а флаг
     «акселерометр не откалиброван» — остаться снятым;
  5. CLI показывает те же значения (get acc_trim_roll, get gyro_1_sensor_align, get mag_declination).

Запуск:  python3 tools/check_sensors_tab.py      (Windows: python tools\\check_sensors_tab.py)
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

PORT = 16051
FAILS: list[str] = []

ALIGN_NAMES = {0: "DEFAULT", 1: "CW0", 2: "CW90", 3: "CW180", 4: "CW270",
               5: "CW0FLIP", 6: "CW90FLIP", 7: "CW180FLIP", 8: "CW270FLIP", 9: "CUSTOM"}


def _frame(buf: bytes):
    for i in range(len(buf) - 5):
        if buf[i] != 0x24:
            continue
        if buf[i + 1] == 0x4D and buf[i + 2] == 0x3E:                 # $M> (v1)
            size = buf[i + 3]
            if i + 5 + size <= len(buf):
                return buf[i + 5 : i + 5 + size]
        if buf[i + 1] == 0x58 and buf[i + 2] == 0x3E:                 # $X> (v2)
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
            except socket.timeout:
                break
            frame = _frame(buf)
            if frame is not None:
                return frame
            time.sleep(0.03)
        return b""

    def cli(self, command: str, window: int = 2.0) -> str:
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


def i16(b, off):
    return int.from_bytes(b[off : off + 2], "little", signed=True)


def u16(b, off):
    return int.from_bytes(b[off : off + 2], "little")


def read_imu(fc: Fc):
    data = fc.call(102)
    return {"acc": [i16(data, 2 * i) for i in range(3)],
            "gyro": [i16(data, 6 + 2 * i) for i in range(3)],
            "mag": [i16(data, 12 + 2 * i) for i in range(3)], "raw": data}


def read_all(fc: Fc) -> dict:
    att = fc.call(108)
    imu = read_imu(fc)
    return {
        "attitude": {"roll": i16(att, 0) / 10, "pitch": i16(att, 2) / 10, "heading": i16(att, 4)},
        "attitude_len": len(att), "imu": imu,
        "sensor_config": list(fc.call(96)),
        "sensor_active": list(fc.call(0x300A, v2=True)),
        "alignment": list(fc.call(126)),
        "board_alignment": [i16(fc.call(38), 0), i16(fc.call(38), 2), i16(fc.call(38), 4)],
        "acc_trim": [i16(fc.call(240), 0), i16(fc.call(240), 2)],
        "compass": u16(fc.call(133), 0),
        "mixer": list(fc.call(42)),
        "board_info": fc.call(4),
        "status": fc.call(101),
    }


def problems(fc: Fc) -> int:
    data = fc.call(4)
    return int.from_bytes(data[-4:], "little")


def main() -> int:
    eeprom = Path(tempfile.mkdtemp()) / "eeprom.json"
    proc, fc = start(PORT, eeprom)

    print("=" * 78)
    print("ВКЛАДКА «ДАТЧИКИ»: что эмулятор отдаёт (дрон лежит на столе)")
    print("=" * 78)
    state = read_all(fc)
    att = state["attitude"]
    imu = state["imu"]
    print(f"   Поза     : крен {att['roll']}°, тангаж {att['pitch']}°, курс {att['heading']}°")
    print(f"   Живые данные: акселерометр {imu['acc']} (1 g ≈ 2048), "
          f"гироскоп {imu['gyro']}, магнитометр {imu['mag']}")
    print(f"   Обнаружены датчики (0x300A): gyro={state['sensor_active'][0]}, acc={state['sensor_active'][1]}, "
          f"baro={state['sensor_active'][2]}, mag={state['sensor_active'][3]}, sonar={state['sensor_active'][4]}")
    print(f"   Настройки датчиков (96): acc={state['sensor_config'][0]}, baro={state['sensor_config'][1]}, "
          f"mag={state['sensor_config'][2]}, sonar={state['sensor_config'][3]} (0 = AUTO, 1 = NONE)")
    align = state["alignment"]
    print(f"   Выравнивание: гироскоп {ALIGN_NAMES.get(align[0])}, акселерометр {ALIGN_NAMES.get(align[1])}, "
          f"магнитометр {ALIGN_NAMES.get(align[2])}, gyro_to_use={align[4]}")
    print(f"   Триммирование: pitch {state['acc_trim'][0]}, roll {state['acc_trim'][1]}")
    print(f"   Склонение магнитометра: {state['compass'] / 10:.1f}°")
    print(f"   Микшер   : тип {state['mixer'][0]}, моторы наоборот: {state['mixer'][1]}")

    check(state["attitude_len"] == 6, f"MSP_ATTITUDE: {state['attitude_len']} байт (нужно 6)")
    check(att["roll"] == 0.0 and att["pitch"] == 0.0, "поза неподвижна: крен и тангаж 0.0")
    check(0 <= att["heading"] < 360, f"курс в градусах: {att['heading']} (Configurator читает курс как градусы)")
    check(len(imu["raw"]) == 18, f"MSP_RAW_IMU: {len(imu['raw'])} байт (нужно 18 — 9 значений)")
    check(2040 <= imu["acc"][2] <= 2056, f"по Z ровно 1 g: {imu['acc'][2]} (2048 = 1.0 g)")
    check(abs(imu["acc"][0]) < 32 and abs(imu["acc"][1]) < 32, "по X/Y акселерометр в покое (только шум)")
    check(abs(imu["gyro"][0]) < 40 and abs(imu["gyro"][1]) < 40, f"гироскоп в покое: {imu['gyro']}")
    check(imu["mag"] == [0, 0, 0], "магнитометр выключен — нули (в настройках NONE)")
    check(len(state["alignment"]) == 7, f"MSP_SENSOR_ALIGNMENT: {len(state['alignment'])} байт (нужно 7)")
    check(len(fc.call(240)) == 4, "MSP_ACC_TRIM: 4 байта (pitch, roll)")
    check(len(fc.call(133)) == 2, "MSP_COMPASS_CONFIG: 2 байта (склонение)")
    check(state["sensor_active"][0] != 0 and state["sensor_active"][1] != 0,
          f"гироскоп и акселерометр «обнаружены»: id {state['sensor_active'][0]}/{state['sensor_active'][1]}")

    # --------------------------------------------------------------- настройка
    print("\n" + "=" * 78)
    print("ПРАВИМ ТРИММИРОВАНИЕ, ВЫРАВНИВАНИЕ, СКЛОНЕНИЕ И КАЛИБРУЕМ АКСЕЛЕРОМЕТР")
    print("=" * 78)
    fc.call(239, (15).to_bytes(2, "little", signed=True) + (-9).to_bytes(2, "little", signed=True))
    trim = fc.call(240)
    check((i16(trim, 0), i16(trim, 2)) == (15, -9), f"трим прочитан обратно: {i16(trim, 0)}/{i16(trim, 2)}")

    fc.call(220, bytes([2, 2, 3, 0, 2, 0]))          # гироскоп CW90, магнит CW180
    align2 = fc.call(126)
    check((align2[0], align2[2]) == (2, 3), f"выравнивание сохранено: гироскоп {ALIGN_NAMES[align2[0]]}, "
                                            f"магнитометр {ALIGN_NAMES[align2[2]]}")

    fc.call(224, (125).to_bytes(2, "little"))        # 12.5°
    check(u16(fc.call(133), 0) == 125, f"склонение сохранено: {u16(fc.call(133), 0) / 10:.1f}°")

    check(problems(fc) & 1 == 1, "до калибровки: флаг «акселерометр не откалиброван» выставлен")
    fc.call(205)                                     # кнопка «Калибровать»
    time.sleep(1.4)
    check(problems(fc) & 1 == 0, "после MSP_ACC_CALIBRATION флаг снят (вкладка видит завершение)")

    fc.call(250)                                     # «Сохранить»
    fc.call(68, bytes([0]))                          # «Сохранить и перезагрузить»
    time.sleep(1.2)
    fc.close()
    stop(proc)

    # --------------------------------------------------------------- перезапуск
    print("\n" + "=" * 78)
    print("ПОЛНЫЙ ПЕРЕЗАПУСК ЭМУЛЯТОРА")
    print("=" * 78)
    proc, fc2 = start(PORT, eeprom)
    state2 = read_all(fc2)
    check(state2["acc_trim"] == [15, -9], f"триммирование пережило перезапуск: {state2['acc_trim']}")
    check(state2["alignment"][:3] == [2, 2, 3], f"выравнивание: {state2['alignment'][:3]}")
    check(state2["compass"] == 125, f"склонение: {state2['compass'] / 10:.1f}°")
    check(problems(fc2) & 1 == 0, "калибровка акселерометра осталась зачтённой")

    for command, expected in (
        ("get acc_trim_pitch", "acc_trim_pitch = 15"),
        ("get acc_trim_roll", "acc_trim_roll = -9"),
        ("get gyro_1_sensor_align", "gyro_1_sensor_align = CW90"),
        ("get align_mag", "align_mag = CW180"),
        ("get mag_declination", "mag_declination = 125"),
    ):
        out = fc2.cli(command)
        check(expected in out, f"CLI '{command}' → {'найдено' if expected in out else 'НЕТ: ' + out.strip()[:50]!r}")

    # --------------------------------------------------------------- «defaults»
    print("\n-- Сброс прошивки: 'defaults' возвращает нули и снова просит калибровку --")
    fc2.cli("defaults")
    time.sleep(0.7)
    fc2.close()
    stop(proc)
    proc, fc3 = start(PORT, eeprom)
    state3 = read_all(fc3)
    check(state3["acc_trim"] == [0, 0], f"триммирование обнулилось: {state3['acc_trim']}")
    check(state3["compass"] == 0, "склонение обнулилось")
    check(problems(fc3) & 1 == 1, "флаг «не откалиброван» снова выставлен")
    fc3.close()
    stop(proc)

    print()
    print("=" * 78)
    if FAILS:
        print(f"ПРОВАЛЕНО проверок: {len(FAILS)}")
        for item in FAILS:
            print("  -", item)
        return 1
    print("ВКЛАДКА «ДАТЧИКИ» РАБОТАЕТ: поза стоит, данные живые, трим и калибровка сохраняются")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

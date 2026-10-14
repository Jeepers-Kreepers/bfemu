#!/usr/bin/env python3
"""Проверка вкладки «PID настройки»: таблица PID, фильтры, слайдеры, профили, сохранение.

Вкладка была пустой, потому что Configurator бросает исключение при отсутствии ответа
на первый же запрос (MSP_PIDNAMES, 117) — дальше цепочка loadData не идёт. Кроме 117 нужны
MSP_PID_ADVANCED (94), MSP_RC_DEADBAND (125), MSP_MOTOR_CONFIG (131), MSP_SIMPLIFIED_TUNING (140),
а для профилей — MSP2_GET_TEXT (0x3006) и MSP_SELECT_SETTING (210).

Скрипт:

  1. повторяет loadData() из PidTuningTab.vue запрос за запросом и проверяет раскладки:
     117, 112, 94, 111, 92, 125, 131, имена профилей (0x3006), MSP_SIMPLIFIED_TUNING,
     MSP_ADVANCED_CONFIG (90), MSP_MIXER_CONFIG (42);
  2. сверяет значения с заводскими 4.5.4 (resetPidProfile / pgResetFn_controlRateProfiles);
  3. повторяет save(): MSP_SET_PID (202), MSP_SET_PID_ADVANCED (95), MSP_SET_RC_TUNING (204),
     MSP_SET_FILTER_CONFIG (93), MSP_SET_SIMPLIFIED_TUNING (141), имена профилей (0x3007),
     MSP_EEPROM_WRITE (250);
  4. проверяет переключение профилей (210), копирование (183) и сброс профиля (219);
  5. проверяет, что правки переживают перезапуск эмулятора, и что CLI показывает те же значения.

Запуск:  python3 tools/check_pid_tab.py      (Windows: python tools\\check_pid_tab.py)
"""
from __future__ import annotations

import socket
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from bfemu.msp import encode_v1, encode_v2  # noqa: E402

PORT = 16091
FAILS: list[str] = []

# Заводские значения resetPidProfile() и pgResetFn_controlRateProfiles() прошивки 4.5.4.
FACTORY_PIDS = {                       # строка таблицы PID: P, I, D (MSP_PID, 112)
    "ROLL": (45, 80, 40), "PITCH": (47, 84, 46), "YAW": (45, 80, 0),
    "LEVEL": (50, 75, 75), "MAG": (40, 0, 0),
}
FACTORY_ADVANCED = {                   # MSP_PID_ADVANCED, 94 — смещения как в msp.c
    "feedforward_transition": (8, 1, 0),
    "acc_limit": (13, 2, 0),
    "acc_limit_yaw": (15, 2, 0),
    "angle_limit": (17, 1, 60),
    "anti_gravity_gain": (21, 2, 80),
    "iterm_rotation": (25, 1, 0),
    "iterm_relax": (27, 1, 1),
    "iterm_relax_type": (28, 1, 1),
    "abs_control_gain": (29, 1, 0),
    "throttle_boost": (30, 1, 5),
    "acro_trainer_angle_limit": (31, 1, 20),
    "feedforward_roll": (32, 2, 120),
    "feedforward_pitch": (34, 2, 125),
    "feedforward_yaw": (36, 2, 120),
    "d_max_roll": (39, 1, 30),
    "d_max_pitch": (40, 1, 34),
    "d_max_yaw": (41, 1, 0),
    "d_max_gain": (42, 1, 37),
    "d_max_advance": (43, 1, 20),
    "use_integrated_yaw": (44, 1, 0),
    "integrated_yaw_relax": (45, 1, 200),
    "iterm_relax_cutoff": (46, 1, 15),
    "motor_output_limit": (47, 1, 100),
    "auto_profile_cell_count": (48, 1, 0),
    "dyn_idle_min_rpm": (49, 1, 0),
    "feedforward_averaging": (50, 1, 0),
    "feedforward_smooth_factor": (51, 1, 25),
    "feedforward_boost": (52, 1, 15),
    "feedforward_max_rate_limit": (53, 1, 90),
    "feedforward_jitter_factor": (54, 1, 7),
    "vbat_sag_compensation": (55, 1, 0),
    "thrust_linear": (56, 1, 0),
    "tpa_mode": (57, 1, 1),
    "tpa_rate": (58, 1, 65),
    "tpa_breakpoint": (59, 2, 1350),
}
FACTORY_RC_TUNING = {                  # MSP_RC_TUNING, 111 — профиль рейтов
    "roll_rc_rate": (0, 1, 7), "roll_expo": (1, 1, 0),
    "roll_srate": (2, 1, 67), "pitch_srate": (3, 1, 67), "yaw_srate": (4, 1, 67),
    "thr_mid": (6, 1, 50), "thr_expo": (7, 1, 0),
    "yaw_expo": (10, 1, 0), "yaw_rc_rate": (11, 1, 7),
    "pitch_rc_rate": (12, 1, 7), "pitch_expo": (13, 1, 0),
    "throttle_limit_type": (14, 1, 0), "throttle_limit_percent": (15, 1, 100),
    "roll_rate_limit": (16, 2, 1998), "pitch_rate_limit": (18, 2, 1998), "yaw_rate_limit": (20, 2, 1998),
    "rates_type": (22, 1, 3),
}
FACTORY_FILTERS = {                    # MSP_FILTER_CONFIG, 92 — 49 байт, порядок из msp.c 4.5.4
    "gyro_lpf1_static_hz": (20, 2, 250), "gyro_lpf2_static_hz": (22, 2, 500),
    "dterm_lpf1_static_hz": (1, 2, 75), "dterm_lpf2_static_hz": (26, 2, 150),
    "dterm_lpf1_dyn_min_hz": (33, 2, 75), "dterm_lpf1_dyn_max_hz": (35, 2, 150),
    "gyro_lpf1_dyn_min_hz": (29, 2, 250), "gyro_lpf1_dyn_max_hz": (31, 2, 500),
    "yaw_lowpass_hz": (3, 2, 100), "dyn_notch_q": (39, 2, 300),
    "dyn_notch_min_hz": (41, 2, 100), "dyn_notch_max_hz": (45, 2, 600),
    "gyro_rpm_notch_harmonics": (43, 1, 3), "gyro_rpm_notch_min_hz": (44, 1, 100),
    "dyn_lpf_curve_expo": (47, 1, 5), "dyn_notch_count": (48, 1, 3),
}


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


def field(data: bytes, offset: int, size: int) -> int:
    return int.from_bytes(data[offset : offset + size], "little")


def check_layout(data: bytes, table: dict, title: str):
    for name, (offset, size, expected) in table.items():
        value = field(data, offset, size)
        check(value == expected, f"{title}: {name} = {value} (заводское {expected})")


def main() -> int:
    import tempfile

    eeprom = Path(tempfile.gettempdir()) / "bfemu_pid_tab_eeprom.json"
    if eeprom.exists():
        eeprom.unlink()

    print("1) обмен вкладки «PID настройки» — как loadData() в PidTuningTab.vue")
    proc, fc = start(PORT, eeprom)

    names = fc.call(117)
    check(names.decode("latin-1") == "ROLL;PITCH;YAW;LEVEL;MAG;", f"MSP_PIDNAMES (117): {names.decode('latin-1')!r}")
    check(names.endswith(b";"), "строка имён осей заканчивается ';' — Configurator режет именно по нему")

    pids = fc.call(112)
    check(len(pids) == 15, f"MSP_PID (112): 15 байт = 5 осей × 3 ({len(pids)})")
    for index, axis in enumerate(("ROLL", "PITCH", "YAW", "LEVEL", "MAG")):
        values = tuple(pids[index * 3 : index * 3 + 3])
        check(values == FACTORY_PIDS[axis], f"таблица PID, строка {axis}: {values[0]}/{values[1]}/{values[2]}")

    advanced = fc.call(94)
    check(len(advanced) == 61, f"MSP_PID_ADVANCED (94): 61 байт ({len(advanced)})")
    check_layout(advanced, FACTORY_ADVANCED, "PID Advanced")

    rc = fc.call(111)
    check(len(rc) == 23, f"MSP_RC_TUNING (111): 23 байта ({len(rc)})")
    check_layout(rc, FACTORY_RC_TUNING, "Rates")

    filters = fc.call(92)
    check(len(filters) == 49, f"MSP_FILTER_CONFIG (92): 49 байт ({len(filters)})")
    check_layout(filters, FACTORY_FILTERS, "Filters")

    deadband = fc.call(125)
    check(len(deadband) == 5, f"MSP_RC_DEADBAND (125): 5 байт ({len(deadband)})")
    check(list(deadband) == [0, 0, 40, 50, 0],
          f"дедбенды: RC {deadband[0]}, yaw {deadband[1]}, alt-hold {deadband[2]}, 3D {field(deadband, 3, 2)}")

    motor = fc.call(131)
    check(len(motor) == 10, f"MSP_MOTOR_CONFIG (131): 10 байт ({len(motor)})")
    check(field(motor, 0, 2) == 1070 and field(motor, 2, 2) == 2000 and field(motor, 4, 2) == 1000,
          f"газ: min {field(motor, 0, 2)}, max {field(motor, 2, 2)}, min_command {field(motor, 4, 2)}")
    check(motor[8] == 1, "bidirectional DShot включён — иначе «Динамический Idle» на вкладке недоступен")

    pid_name = fc.call(0x3006, bytes([3]), v2=True)
    rate_name = fc.call(0x3006, bytes([4]), v2=True)
    check(pid_name[0] == 3 and pid_name[2:].decode("latin-1") == "Profile 1",
          f"имя PID-профиля: {pid_name[2:].decode('latin-1')!r}")
    check(rate_name[0] == 4 and rate_name[2:].decode("latin-1") == "Actual",
          f"имя профиля рейтов: {rate_name[2:].decode('latin-1')!r}")

    sliders = fc.call(140)
    check(len(sliders) == 53, f"MSP_SIMPLIFIED_TUNING (140): 53 байта = 17 + 18 + 18 ({len(sliders)})")
    check(list(sliders[0:9]) == [2, 100, 100, 100, 100, 100, 100, 100, 100],
          f"ползунки: режим {sliders[0]} (RPY), все множители по 100")
    check(field(sliders, 19, 2) == 75 and field(sliders, 21, 2) == 150,
          f"D-term фильтр: LPF1 {field(sliders, 19, 2)} / LPF2 {field(sliders, 21, 2)}")
    check(field(sliders, 37, 2) == 250 and field(sliders, 39, 2) == 500,
          f"фильтр гироскопа: LPF1 {field(sliders, 37, 2)} / LPF2 {field(sliders, 39, 2)}")

    advanced_config = fc.call(90)
    mixer = fc.call(42)
    check(len(advanced_config) == 20 and len(mixer) == 2,
          f"остальные запросы вкладки: MSP_ADVANCED_CONFIG {len(advanced_config)} байт, MSP_MIXER_CONFIG {len(mixer)}")

    valid = fc.call(145)
    check(list(valid) == [1, 1, 1], f"MSP_VALIDATE_SIMPLIFIED_TUNING (145): {list(valid)} — ползунки совпадают с PID")

    print("2) правки и сохранение (Save во вкладке: 202 → 95 → 204 → 93 → 141 → имена → EEPROM)")
    edit_pids = bytes([55, 90, 45, 57, 94, 51, 55, 90, 5, 50, 75, 75, 40, 0, 0])
    check(fc.call(202, edit_pids) == b"", "MSP_SET_PID (202) принят (пустой ACK)")
    saved = fc.call(112)
    check(tuple(saved[0:3]) == (55, 90, 45), f"новые PID roll: {tuple(saved[0:3])}")

    edit_adv = bytearray(advanced)
    edit_adv[21:23] = (100).to_bytes(2, "little")      # anti_gravity_gain
    edit_adv[17] = 65                                  # angle_limit
    edit_adv[42] = 45                                  # d_max_gain
    edit_adv[58] = 70                                  # tpa_rate
    edit_adv[59:61] = (1500).to_bytes(2, "little")     # tpa_breakpoint
    check(fc.call(95, bytes(edit_adv)) == b"", "MSP_SET_PID_ADVANCED (95) принят")
    advanced = fc.call(94)
    check(field(advanced, 21, 2) == 100 and advanced[17] == 65 and advanced[42] == 45,
          f"записано: антигравитация {field(advanced, 21, 2)}, угол {advanced[17]}, D Max gain {advanced[42]}")
    check(advanced[58] == 70 and field(advanced, 59, 2) == 1500,
          f"TPA: {advanced[57]}/{advanced[58]}% @ {field(advanced, 59, 2)}")

    edit_rc = bytearray(rc)
    edit_rc[2] = 80                                    # roll_srate
    edit_rc[6] = 55                                    # thr_mid
    edit_rc[16:18] = (1900).to_bytes(2, "little")      # roll_rate_limit
    check(fc.call(204, bytes(edit_rc)) == b"", "MSP_SET_RC_TUNING (204) принят")
    rc = fc.call(111)
    check(rc[2] == 80 and rc[6] == 55 and field(rc, 16, 2) == 1900,
          f"рейты сохранены: roll_srate {rc[2]}, thr_mid {rc[6]}, roll_rate_limit {field(rc, 16, 2)}")

    edit_filters = bytearray(filters)
    edit_filters[1:3] = (90).to_bytes(2, "little")     # dterm_lpf1_static_hz
    edit_filters[5:7] = (180).to_bytes(2, "little")    # gyro_notch1_hz
    edit_filters[7:9] = (90).to_bytes(2, "little")     # gyro_notch1_cutoff
    check(fc.call(93, bytes(edit_filters)) == b"", "MSP_SET_FILTER_CONFIG (93) принят")
    filters = fc.call(92)
    check(field(filters, 1, 2) == 90 and field(filters, 5, 2) == 180 and field(filters, 7, 2) == 90,
          f"фильтры сохранены: D-term LPF1 {field(filters, 1, 2)}, гиро-нотч {field(filters, 5, 2)}/{field(filters, 7, 2)}")

    # Ползунки: 35 байт от Configurator (17 + 18), меняем master_multiplier
    edit_sliders = bytearray(sliders[:35])
    edit_sliders[1] = 120                              # simplified_master_multiplier
    check(fc.call(141, bytes(edit_sliders)) == b"", "MSP_SET_SIMPLIFIED_TUNING (141) принят (35 байт)")
    pids = fc.call(112)
    check(tuple(pids[0:3]) == (54, 96, 48), f"ползунок 120% пересчитал PID roll: {tuple(pids[0:3])} (45/80/40 × 1.2)")
    check(tuple(pids[3:6]) == (56, 100, 55), f"и pitch: {tuple(pids[3:6])} (47/84/46 × 1.2)")
    sliders = fc.call(140)
    check(sliders[1] == 120, f"множитель сохранён в профиле: {sliders[1]}")

    check(fc.call(0x3007, b"\x03\x09MyProfile", v2=True) == b"", "MSP2_SET_TEXT (0x3007): имя PID-профиля сохранено")
    pid_name = fc.call(0x3006, bytes([3]), v2=True)
    check(pid_name[2:].decode("latin-1") == "MyProfile", f"имя профиля читается: {pid_name[2:].decode('latin-1')!r}")

    check(fc.call(250) == b"", "MSP_EEPROM_WRITE (250): настройки записаны во «флеш»")

    print("3) профили: переключение (210), копирование (183), сброс (219)")
    fc.call(210, bytes([1]))                           # профиль PID 2
    status_ex = fc.call(150)
    check(status_ex[13] == 4, f"STATUS_EX: количество PID-профилей {status_ex[13]} (в 4.5.4 их 4)")
    check(status_ex[10] == 1, f"активный профиль после переключения: {status_ex[10]}")
    factory = fc.call(112)
    check(tuple(factory[0:3]) == (45, 80, 40), f"профиль 2 отдаёт заводские PID: {tuple(factory[0:3])}")

    check(fc.call(183, bytes([0, 1, 0])) == b"", "MSP_COPY_PROFILE (183): профиль 1 скопирован в профиль 2")
    copied = fc.call(112)
    check(tuple(copied[0:3]) == (54, 96, 48), f"в профиле 2 значения профиля 1: {tuple(copied[0:3])}")

    check(fc.call(219) == b"", "MSP_SET_RESET_CURR_PID (219): сброс текущего профиля")
    reset = fc.call(112)
    check(tuple(reset[0:3]) == (45, 80, 40), f"после сброса заводские: {tuple(reset[0:3])}")

    fc.call(210, bytes([0]))                           # вернуться на профиль 1
    check(tuple(fc.call(112)[0:3]) == (54, 96, 48), "профиль 1 сохранил свои правки (значения переносятся между профилями)")
    fc.call(210, bytes([0x80 | 1]))                    # профиль рейтов 2
    rc2 = fc.call(111)
    check(rc2[2] == 67 and rc2[6] == 50, f"профиль рейтов 2 — заводские: srate {rc2[2]}, thr_mid {rc2[6]}")
    fc.call(210, bytes([0x80 | 0]))

    print("4) CLI видит те же значения")
    cli = fc.cli("get p_roll")
    check("p_roll = 54" in cli, f"CLI 'get p_roll' → {cli.strip().splitlines()[0] if cli.strip() else 'пусто'}")
    cli = fc.cli("get angle_p_gain")
    check("50" in cli, f"CLI 'get angle_p_gain' → {cli.strip().splitlines()[0] if cli.strip() else 'пусто'}")
    cli = fc.cli("get d_min_roll")
    check("= 36" in cli, f"CLI 'get d_min_roll' → {cli.strip().splitlines()[0] if cli.strip() else 'пусто'} (30 × 1.2 = 36)")
    cli = fc.cli("get min_throttle")
    check("1070" in cli, f"CLI 'get min_throttle' → {cli.strip().splitlines()[0] if cli.strip() else 'пусто'}")
    cli = fc.cli("dump", window=8.0)
    check("p_roll" in cli and "angle_p_gain" in cli and "d_max_gain" in cli and "roll_srate" in cli,
          f"CLI 'dump' содержит PID-, Advanced- и rate-настройки ({len(cli)} символов)")

    print("5) перезапуск эмулятора: правки должны остаться")
    fc.close()
    stop(proc)
    proc, fc = start(PORT, eeprom)

    pids = fc.call(112)
    check(tuple(pids[0:3]) == (54, 96, 48), f"PID после перезапуска: {tuple(pids[0:3])}")
    advanced = fc.call(94)
    check(field(advanced, 21, 2) == 100 and advanced[17] == 65 and field(advanced, 59, 2) == 1500,
          f"Advanced после перезапуска: антигравитация {field(advanced, 21, 2)}, угол {advanced[17]}, TPA @ {field(advanced, 59, 2)}")
    rc = fc.call(111)
    check(rc[2] == 80 and field(rc, 16, 2) == 1900, f"рейты после перезапуска: srate {rc[2]}, rate_limit {field(rc, 16, 2)}")
    filters = fc.call(92)
    check(field(filters, 5, 2) == 180, f"фильтры после перезапуска: гиро-нотч {field(filters, 5, 2)}")
    sliders = fc.call(140)
    check(sliders[1] == 120, f"ползунок после перезапуска: {sliders[1]}")
    pid_name = fc.call(0x3006, bytes([3]), v2=True)
    check(pid_name[2:].decode("latin-1") == "MyProfile", f"имя профиля после перезапуска: {pid_name[2:].decode('latin-1')!r}")
    check(len(fc.call(117)) == 25, "MSP_PIDNAMES отвечает и после перезапуска")

    print("6) 'defaults' возвращает заводские значения 4.5.4")
    cli = fc.cli("defaults")
    check("reset" in cli.lower() or "defaults" in cli.lower(), f"CLI 'defaults' выполнен: {cli.strip()[:60]!r}")
    time.sleep(0.8)                        # прошивка после defaults перезагружается
    fc.close()
    for _ in range(40):                    # ждём, пока эмулятор снова начнёт отвечать
        try:
            fc = Fc(PORT)
            if fc.call(112):
                break
        except OSError:
            time.sleep(0.25)
    pids = fc.call(112)
    check(tuple(pids[0:3]) == (45, 80, 40), f"после defaults заводские PID: {tuple(pids[0:3])}")
    sliders = fc.call(140)
    check(list(sliders[0:9]) == [2, 100, 100, 100, 100, 100, 100, 100, 100], "ползунки снова 100%")

    fc.close()
    stop(proc)

    print("\n" + "=" * 78)
    if FAILS:
        print(f"ПРОВАЛЕНО ПРОВЕРОК: {len(FAILS)}")
        for item in FAILS:
            print(f"  - {item}")
        return 1
    print("ВКЛАДКА «PID НАСТРОЙКИ» РАБОТАЕТ: таблица, фильтры, слайдеры, профили и сохранение")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

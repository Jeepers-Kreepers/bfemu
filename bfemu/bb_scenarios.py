"""Эмулятор наполнения «чёрного ящика»: пробные пролёты разных типов.

Каждый сценарий — короткая модель квада: стики → задатчики → гироскоп → ПИД → моторы →
обороты/ток/напряжение → акселерометр. Плюс своя «изюминка»: отказ двигателя, резонанс
луча, развороты по квадрату, бочки и сальто. Полёты кодируются тем же форматом, что
пишет прошивка (bfemu.blackbox), и складываются в «флеш» — ровно то, что Configurator
читает на вкладке «Чёрный ящик» (MSP_DATAFLASH_SUMMARY / MSP_DATAFLASH_READ) и открывает
в просмотрщике логов.

Сценарии (они же подписи в списке логов):

* «Взлёт и посадка» — набор высоты, висение, снижение, касание;
* «Спокойный полёт по прямой» — ровный горизонт, медленный ход вперёд;
* «Полёт по квадрату» — четыре прямые и развороты на 90°;
* «Проблема с двигателем» — мотор 3 теряет тягу: обороты падают, квад уводит;
* «Плохая жёсткость луча» — резонанс 120 Гц по крену, растёт с газом;
* «Резкие манёвры: бочки и сальто» — быстрые вращения, броски газа и тока.
"""
from __future__ import annotations

import json
import math
import random
import shutil
import threading
from datetime import datetime, timedelta
from pathlib import Path

from .blackbox import (ACC_1G, LOOPTIME_US, MAXTHROTTLE, MINTHROTTLE, MOTOR_IDLE,
                       MOTOR_POLES, BlackboxFlash, encode_flight, flight_mode_event,
                       scan_logs)

RATE_HZ = 1_000_000 // LOOPTIME_US          # 1000 кадров в секунду (цикл PID 1 кГц)
MAX_RATE_DEG_S = 660.0                      # предел задатчика крена/тангажа
MAX_YAW_RATE_DEG_S = 400.0
CRAFT_NAME = "BFEMU QUAD"
ARM_FLAG = 1

DEFAULT_FIRST_START_US = 6_000_000          # лог начинается через 6 с после включения
DEFAULT_GAP_S = 3.0


# ------------------------------------------------------------------ вспомогательное
def _stick(t: float, points) -> float:
    """Положение стика (мкс) по ломаной «время → значение»."""
    if t <= points[0][0]:
        return float(points[0][1])
    for (t0, v0), (t1, v1) in zip(points, points[1:]):
        if t <= t1:
            return v0 + (v1 - v0) * (t - t0) / (t1 - t0)
    return float(points[-1][1])


def _smooth(t: float, start: float, end: float) -> float:
    """Плавный переход 0…1 (косинус)."""
    if t <= start:
        return 0.0
    if t >= end:
        return 1.0
    return 0.5 - 0.5 * math.cos(math.pi * (t - start) / (end - start))


def sysinfo(log_every: int = 2, i_interval: int = 32) -> dict:
    """Что прошивка пишет в шапку лога (настройки нашего «борта»).

    ``log_every`` — «P interval»: во «флеш» попадает каждый log_every-й цикл PID;
    ``i_interval`` — период кадров I (32 цикла ≈ 32 мс при looptime 1000).
    """
    return {
        "log_every": int(log_every),
        "i_interval": int(i_interval),
        "p_ratio": max(1, int(i_interval) // max(1, int(log_every))),
        "firmware": "4.5.4",
        "build": "bfemu",
        "board": "BFRVIRTUALFC",
        "manufacturer": "BFR",
        "build_date": "2026-09-27 12:00:00",
        "vbatref": 1680,
        "vbat_min_cell": 330,
        "vbat_warn_cell": 350,
        "vbat_max_cell": 430,
        "vbat_scale": 110,
        "minthrottle": MINTHROTTLE,
        "maxthrottle": MAXTHROTTLE,
        "acc_1g": ACC_1G,
        "motor_poles": MOTOR_POLES,
        "current_offset": 0,
        "current_scale": 400,
        "pid_roll": "45,80,40",
        "pid_pitch": "47,84,46",
        "pid_yaw": "45,80,0",
        "rc_rates": "70,70,70",
        "deadband": "0",
        "yaw_deadband": "0",
    }


class Flight:
    """Один пролёт: кадры модели, подпись и кодирование в байты лога."""

    def __init__(self, key: str, title: str, note: str, start: datetime, craft_name: str,
                 samples: list, sysinfo: dict):
        self.key = key
        self.title = title
        self.note = note
        self.start = start
        self.craft_name = craft_name
        self.samples = samples
        self.sysinfo = sysinfo
        self.encoded: bytes | None = None

    @property
    def duration(self) -> float:
        """Длительность пролёта: время модели, а не число кадров в «флеше»."""
        return len(self.samples) * LOOPTIME_US / 1_000_000

    def encode(self) -> bytes:
        if self.encoded is None:
            self.encoded = encode_flight(self, self.sysinfo)
        return self.encoded

    def __repr__(self) -> str:
        return (f"<Пролёт {self.key}: {self.title}, {self.duration:.1f} с, "
                f"{len(self.samples)} кадров>")


# ------------------------------------------------------------------ модель полёта
class Sim:
    """Простая модель квадра: стики → задатчики → гироскоп → моторы → питание."""

    RESPONSE = 0.32                     # как быстро квад догоняет задатчик (1 порядок)
    KP, KI, KD, KF = 1.15, 0.45, 0.26, 0.12
    NOISE = (1.6, 1.6, 1.1)             # шум гироскопа, °/с
    RESPONSE_S = 0.045                  # задержка отклика, с
    ROLL_MIX = (-1.0, 1.0, 1.0, -1.0)   # мотор 0 — передний правый, далее по кругу
    PITCH_MIX = (-1.0, -1.0, 1.0, 1.0)
    YAW_MIX = (-1.0, 1.0, -1.0, 1.0)
    INTERNAL_R = 0.075                  # сопротивление батареи, Ом (4S)

    def __init__(self, seed: int = 20260927, start_us: int = DEFAULT_FIRST_START_US):
        self.rng = random.Random(seed)
        self.start_us = start_us

    def fly(self, script, duration: float, craft_name: str = CRAFT_NAME) -> list:
        """Прогон сценария: на выходе кадры (по одному на итерацию цикла)."""
        rng = self.rng
        samples = []
        steps = int(round(duration * RATE_HZ))
        dt = LOOPTIME_US / 1_000_000
        gyro = [0.0, 0.0, 0.0]
        integral = [0.0, 0.0, 0.0]
        previous_error = [0.0, 0.0, 0.0]
        previous_setpoint = [0.0, 0.0, 0.0]
        attitude = [0.0, 0.0, 0.0]                    # крен, тангаж, рыскание (град)
        armed = False
        throttle_smooth = 1000.0
        vbat = 16.80
        mah = 0.0
        time_us = self.start_us

        for index in range(steps):
            t = index / RATE_HZ
            cmd = script(t)
            events = []
            want_armed = bool(cmd.get("armed", True))
            if want_armed != armed:
                flags, last = (ARM_FLAG, 0) if want_armed else (0, ARM_FLAG)
                events.append(flight_mode_event(flags, last))
                armed = want_armed
            throttle_stick = float(cmd.get("throttle", 1000.0)) if armed else 1000.0
            roll_stick = float(cmd.get("roll", 1500.0)) if armed else 1500.0
            pitch_stick = float(cmd.get("pitch", 1500.0)) if armed else 1500.0
            yaw_stick = float(cmd.get("yaw", 1500.0)) if armed else 1500.0
            setpoint = [
                (roll_stick - 1500.0) / 500.0 * MAX_RATE_DEG_S,
                (pitch_stick - 1500.0) / 500.0 * MAX_RATE_DEG_S,
                (yaw_stick - 1500.0) / 500.0 * MAX_YAW_RATE_DEG_S,
            ]
            # «увод по рысканию» — внешнее возмущение (сценарий с отказом двигателя).
            # Это добавка к ЗАДАТЧИКУ: квад честно крутится вокруг оси со скоростью
            # возмущения, а не «залипает» в бесконечном вращении. Если добавить её
            # прямо в гироскоп, модель уходила бы в равновесие ~45× значение.
            setpoint[2] += float(cmd.get("yaw_bias", 0.0))
            # отклик квада: гироскоп плавно догоняет задатчик
            alpha = 1.0 - math.exp(-dt / self.RESPONSE_S)
            for axis in range(3):
                gyro[axis] += (setpoint[axis] - gyro[axis]) * alpha
                gyro[axis] += rng.gauss(0.0, self.NOISE[axis])
            for axis, (freq, amp) in cmd.get("osc", {}).items():
                gyro[axis] += amp * math.sin(2 * math.pi * freq * t)
            for axis in range(3):
                attitude[axis] += gyro[axis] * dt
            # ПИД-регулятор (величины правдоподобные, как в логах настоящего квада)
            p_term, i_term, d_term, f_term = [0.0] * 3, [0.0] * 3, [0.0] * 3, [0.0] * 3
            for axis in range(3):
                error = setpoint[axis] - gyro[axis]
                p_term[axis] = self.KP * error
                integral[axis] += self.KI * error * dt
                i_term[axis] = integral[axis]
                d_term[axis] = self.KD * (error - previous_error[axis]) / dt / 10.0
                f_term[axis] = self.KF * (setpoint[axis] - previous_setpoint[axis]) / dt / 10.0
                previous_error[axis] = error
                previous_setpoint[axis] = setpoint[axis]
            for axis, (freq, amp) in cmd.get("pid_noise", {}).items():
                d_term[axis] += amp * math.cos(2 * math.pi * freq * t)
            # моторы: газ + микширование по осям
            throttle_smooth += (throttle_stick - throttle_smooth) * 0.25
            motors = []
            for motor in range(4):
                value = (throttle_smooth
                         + self.ROLL_MIX[motor] * p_term[0]
                         + self.PITCH_MIX[motor] * p_term[1]
                         + self.YAW_MIX[motor] * p_term[2] * 0.6
                         + 40.0 * self.ROLL_MIX[motor])
                motors.append(max(MINTHROTTLE, min(MAXTHROTTLE, value)))
            for motor, factor in enumerate(cmd.get("motor_scale", [1.0] * 4)):
                if factor != 1.0:
                    span = MAXTHROTTLE - MINTHROTTLE
                    motors[motor] = max(MINTHROTTLE, min(MAXTHROTTLE,
                                                        MINTHROTTLE + (motors[motor] - MINTHROTTLE) * factor))
            for motor, offset in enumerate(cmd.get("motor_offset", [0.0] * 4)):
                motors[motor] = max(MINTHROTTLE, min(MAXTHROTTLE, motors[motor] + offset))
            # обороты: 24 000 об/мин на полном газе; у неисправного мотора — меньше
            rpm_scale = cmd.get("rpm_scale", [1.0] * 4)
            rpm = []
            for motor in range(4):
                load = (motors[motor] - MOTOR_IDLE) / (MAXTHROTTLE - MOTOR_IDLE)
                rpm.append(max(0.0, load * 24_000.0 * rpm_scale[motor]))
            erpm = [int(round(value * MOTOR_POLES / 200.0)) for value in rpm]
            # питание: просадка под током, разряд и шум
            amps = max(0.8, (sum(motors) / 4.0 - 1000.0) / 1000.0 * 26.0) + cmd.get("extra_amps", 0.0)
            amps = max(0.0, amps + rng.gauss(0.0, 0.25) * (1.0 if armed else 0.2))
            mah += amps * dt * 1000.0 / 3600.0
            if armed:
                vbat = 16.80 - self.INTERNAL_R * amps - 0.0009 * (t * 1000.0) / 60.0 * 6.0
            vbat = max(13.2, vbat + rng.gauss(0.0, 0.004))
            # акселерометр: сила тяжести по крену/тангажу + вибрации
            roll_rad, pitch_rad = math.radians(attitude[0]), math.radians(attitude[1])
            acc = [
                int(round(-math.sin(pitch_rad) * ACC_1G + rng.gauss(0, 9))),
                int(round(math.sin(roll_rad) * math.cos(pitch_rad) * ACC_1G + rng.gauss(0, 9))),
                int(round(math.cos(roll_rad) * math.cos(pitch_rad) * ACC_1G + rng.gauss(0, 9))),
            ]
            for axis, (freq, amp) in cmd.get("acc_osc", {}).items():
                acc[axis] += int(round(amp * ACC_1G * math.sin(2 * math.pi * freq * t)))
            samples.append({
                "loop_iteration": index,
                "time": time_us,
                "axis_p": [int(round(p_term[0])), int(round(p_term[1])), int(round(p_term[2]))],
                "axis_i": [int(round(i_term[0])), int(round(i_term[1])), int(round(i_term[2]))],
                "axis_d": [int(round(d_term[0])), int(round(d_term[1])), int(round(d_term[2]))],
                "axis_f": [int(round(f_term[0])), int(round(f_term[1])), int(round(f_term[2]))],
                "rc_command": [int(round(roll_stick - 1500)), int(round(pitch_stick - 1500)),
                               int(round(yaw_stick - 1500)), int(round(throttle_stick))],
                "vbat": int(round(vbat * 100)),
                "amperage": int(round(amps * 100)),
                "rssi": min(1023, max(0, int(round(940 + rng.gauss(0, 6))))),
                "gyro_adc": [int(round(gyro[0])), int(round(gyro[1])), int(round(gyro[2]))],
                "gyro_unfilt": [int(round(gyro[0] + rng.gauss(0, 3))),
                                int(round(gyro[1] + rng.gauss(0, 3))),
                                int(round(gyro[2] + rng.gauss(0, 2)))],
                "acc_smooth": acc,
                "motor": [int(round(value)) for value in motors],
                "erpm": erpm,
                "flight_mode_flags": ARM_FLAG if armed else 0,
                # stateFlags — это НЕ «armed»: в прошивке 4.5 туда идёт flightState
                # (бит0 = GPS_FIX_HOME). Без GPS он честно нулевой, а факт взлёта
                # виден по flightModeFlags (бит ARM) и событиям ARM/DISARM.
                "state_flags": 0,
                "failsafe_phase": 0,
                "rx_signal_received": 1,
                "rx_channels_valid": 1,
                "slow_changed": bool(events),
                "events": events,
            })
            time_us += LOOPTIME_US
        return samples

# ------------------------------------------------------------------ сценарии
def _script_takeoff_landing():
    def script(t):
        throttle = _stick(t, [(0, 1000), (1.0, 1000), (1.8, 1310), (3.0, 1560), (4.0, 1530),
                              (7.0, 1545), (9.0, 1580), (10.5, 1330), (11.6, 1150), (12.2, 1000)])
        return {
            "armed": 0.9 <= t < 12.4,
            "throttle": throttle,
            "roll": _stick(t, [(0, 1500), (3.0, 1500), (4.5, 1522), (6.0, 1481), (7.5, 1506), (9.0, 1500)]),
            "pitch": _stick(t, [(0, 1500), (5.0, 1500), (6.5, 1528), (8.0, 1477), (9.5, 1500)]),
            "yaw": _stick(t, [(0, 1500), (4.0, 1512), (7.0, 1490), (9.0, 1500)]),
        }
    return script


def _script_straight_cruise():
    def script(t):
        throttle = _stick(t, [(0, 1000), (1.0, 1000), (2.0, 1490), (3.0, 1520), (6.0, 1512),
                              (10.0, 1505), (13.0, 1518), (15.0, 1300), (16.0, 1050)])
        return {
            "armed": 1.0 <= t < 16.4,
            "throttle": throttle,
            "pitch": _stick(t, [(0, 1500), (2.6, 1500), (3.4, 1552), (12.0, 1556), (14.0, 1500)]),
            "roll": _stick(t, [(0, 1500), (5.0, 1504), (8.0, 1494), (11.0, 1501), (14.0, 1500)]),
            "yaw": _stick(t, [(0, 1500), (6.0, 1500), (10.0, 1496), (14.0, 1500)]),
        }
    return script


def _script_square():
    corners = [(4.5, 90.0), (8.4, 90.0), (12.3, 90.0), (16.2, 90.0)]

    def yaw_at(t):
        value = 1500.0
        for start, _ in corners:
            value += _smooth(t, start, start + 1.0) * 190.0 - _smooth(t, start + 1.0, start + 2.0) * 190.0
        return value

    def script(t):
        throttle = _stick(t, [(0, 1000), (1.0, 1000), (2.2, 1520), (19.0, 1510), (20.0, 1250), (20.8, 1050)])
        leg = int(t // 4.0) % 4                     # сторона квадрата
        pitch_stick = 1556.0 if leg in (0, 2) else 1444.0
        roll_stick = 1556.0 if leg in (1, 3) else 1444.0
        blend = _smooth(t, 3.4, 4.2)
        return {
            "armed": 1.0 <= t < 21.2,
            "throttle": throttle,
            "pitch": _stick(t, [(0, 1500), (3.4, 1500), (4.2, pitch_stick), (20.0, 1500)]),
            "roll": _stick(t, [(0, 1500), (3.4, 1500), (4.2, roll_stick), (20.0, 1500)]),
            "yaw": yaw_at(t) if t > 3.0 else 1500.0,
        }
    return script


def _script_motor_fault():
    def script(t):
        fault = _smooth(t, 6.0, 6.4)               # мотор 3 (индекс 2) начинает терять тягу
        return {
            "armed": 0.9 <= t < 15.4,
            "throttle": _stick(t, [(0, 1000), (1.6, 1500), (5.0, 1540), (13.5, 1520), (15.0, 1100)]),
            "roll": _stick(t, [(0, 1500), (3.0, 1500), (7.0, 1508), (10.0, 1496), (13.0, 1502)]),
            "pitch": _stick(t, [(0, 1500), (8.0, 1504), (12.0, 1497), (14.0, 1500)]),
            "yaw": _stick(t, [(0, 1500), (6.0, 1500), (8.0, 1506), (11.0, 1492), (14.0, 1500)]),
            "motor_scale": [1.0, 1.0, 1.0 - 0.28 * fault, 1.0],
            "rpm_scale": [1.0, 1.0, 1.0 - 0.45 * fault, 1.0],
            "yaw_bias": 26.0 * fault,               # квад медленно уводит по рысканию
            "extra_amps": 1.6 * fault,
            "pid_noise": {0: (7.5, 6.0 * fault), 1: (7.5, 5.0 * fault)},
        }
    return script


def _script_arm_resonance():
    def script(t):
        throttle = _stick(t, [(0, 1000), (1.6, 1500), (4.0, 1520), (9.5, 1620), (11.5, 1500), (13.0, 1050)])
        load = max(0.0, (throttle - 1200.0) / 800.0)          # вибрация растёт с газом
        amp = 2.0 + 34.0 * load
        return {
            "armed": 0.9 <= t < 13.6,
            "throttle": throttle,
            "roll": _stick(t, [(0, 1500), (3.0, 1500), (6.0, 1503), (9.0, 1497), (12.0, 1500)]),
            "pitch": _stick(t, [(0, 1500), (7.0, 1502), (11.0, 1498), (13.0, 1500)]),
            "yaw": _stick(t, [(0, 1500), (9.0, 1500), (12.0, 1499)]),
            "osc": {0: (120.0, amp * 0.55), 1: (120.0, amp * 0.25)},
            "acc_osc": {2: (120.0, 0.05 + 0.35 * load), 0: (120.0, 0.03 + 0.16 * load)},
            "pid_noise": {0: (120.0, 55.0 * load), 1: (120.0, 24.0 * load)},
        }
    return script


def _script_acro():
    def script(t):
        roll = 1500.0
        if 2.6 <= t < 3.6:                          # бочка
            roll = 1500.0 + 430.0 * math.sin(2 * math.pi * (t - 2.6) / 1.0)
        elif 5.0 <= t < 6.0:                        # вторая бочка
            roll = 1500.0 - 430.0 * math.sin(2 * math.pi * (t - 5.0) / 1.0)
        pitch = 1500.0
        if 7.6 <= t < 8.6:                          # сальто
            pitch = 1500.0 + 470.0 * math.sin(2 * math.pi * (t - 7.6) / 1.0)
        yaw = _stick(t, [(0, 1500), (9.0, 1500), (10.5, 1760), (12.0, 1500)])
        return {
            "armed": 0.8 <= t < 12.6,
            "throttle": _stick(t, [(0, 1000), (1.4, 1250), (2.2, 1700), (4.0, 1620), (6.6, 1780),
                                   (9.0, 1600), (11.0, 1200), (12.0, 1050)]),
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "extra_amps": 4.0 * _smooth(t, 2.2, 2.8),
        }
    return script


SCENARIOS = [
    ("takeoff_landing", "Взлёт и посадка",
     "набор высоты, висение, снижение и касание: видно поведение газа и тока",
     12.6, _script_takeoff_landing),
    ("straight_cruise", "Спокойный полёт по прямой",
     "ровный горизонт и медленный ход вперёд: чистые графики без вибраций",
     16.6, _script_straight_cruise),
    ("square", "Полёт по квадрату",
     "четыре прямые и повороты на 90°: всплески по крену и рысканию",
     21.4, _script_square),
    ("motor_fault", "Проблема с двигателем",
     "мотор 3 теряет тягу: обороты проваливаются, квад уводит по рысканию",
     15.6, _script_motor_fault),
    ("arm_resonance", "Плохая жёсткость луча",
     "резонанс 120 Гц по крену, усиливается с газом: дрожание гироскопа и D-терма",
     13.8, _script_arm_resonance),
    ("acro", "Резкие манёвры: бочки и сальто",
     "быстрые вращения и броски газа: гироскоп уходит за 700 °/с",
     12.8, _script_acro),
]


def scenario_catalog() -> list:
    """Список сценариев для интерфейса: ключ, название, пояснение, длительность."""
    return [{"key": key, "title": title, "note": note, "duration": duration}
            for key, title, note, duration, _ in SCENARIOS]


def build_flights(first_start_us: int = DEFAULT_FIRST_START_US, gap_s: float = DEFAULT_GAP_S,
                  base_datetime: datetime | None = None, log_every: int = 2,
                  i_interval: int = 32) -> list:
    """Все сценарии подряд — как шесть полётов, записанных в «флеш» за один вылет на поле."""
    info = sysinfo(log_every=log_every, i_interval=i_interval)
    base = base_datetime or datetime(2026, 9, 27, 12, 5, 0)
    flights = []
    time_us = first_start_us
    wall = base
    for index, (key, title, note, duration, factory) in enumerate(SCENARIOS):
        sim = Sim(seed=0x5EED + index * 977, start_us=time_us)
        samples = sim.fly(factory(), duration)
        flights.append(Flight(key=key, title=title, note=note, start=wall,
                              craft_name=CRAFT_NAME, samples=samples, sysinfo=info))
        time_us += int((duration + gap_s) * 1_000_000)
        wall = wall + timedelta(seconds=duration + gap_s)
    return flights


def build_flash_data(**kwargs) -> bytes:
    """Содержимое «флеша»: кодированные логи подряд, как их пишет прошивка."""
    out = bytearray()
    for flight in build_flights(**kwargs):
        out.extend(flight.encode())
    return bytes(out)


def build_model() -> list:
    """Модель пролётов без привязки к частоте записи (её можно менять при кодировании)."""
    return build_flights(log_every=1, i_interval=32)


def recode(flights: list, log_every: int = 2, i_interval: int = 32) -> list:
    """Пересобирает байты логов под другую частоту записи (модель остаётся той же)."""
    info = sysinfo(log_every=log_every, i_interval=i_interval)
    return [Flight(key=flight.key, title=flight.title, note=flight.note, start=flight.start,
                   craft_name=flight.craft_name, samples=flight.samples, sysinfo=info)
            for flight in flights]


def log_every_for_sample_rate(sample_rate: int, i_interval: int = 32) -> int:
    """«P interval» из настройки sample_rate (blackboxCfg.sample_rate): 1<<rate, не больше I."""
    interval = 1 << max(0, int(sample_rate))
    return 0 if interval > i_interval else interval


def log_catalog(data: bytes, flights: list | None = None) -> list:
    """Описание логов внутри «флеша» — для вкладки «Чёрный ящик» и консоли.

    Если рядом есть собранные пролёты, подставляем их названия; иначе остаётся то,
    что есть в самом логе (дата, борт, интервалы).
    """
    titles = {index: flight for index, flight in enumerate(flights or [])}
    catalog = []
    for index, entry in enumerate(scan_logs(data)):
        flight = titles.get(index)
        catalog.append({
            **entry,
            "title": flight.title if flight else "",
            "key": flight.key if flight else "",
            "note": flight.note if flight else "",
            "duration": round(flight.duration, 1) if flight else entry["duration"],
            "rate_hz": (1_000_000 // (int(entry["looptime"] or 1000) * max(1, int(entry["p_interval"] or 1)))),
        })
    return catalog


def load_or_build(path: Path | str | None) -> BlackboxFlash:
    """«Флеш» накопителя: читаем сохранённый файл, а если его нет — наполняем заново."""
    if path is None:
        return BlackboxFlash(build_flash_data())
    path = Path(path)
    if path.exists():
        return BlackboxFlash.load(path)
    flash = BlackboxFlash(build_flash_data())
    path.parent.mkdir(parents=True, exist_ok=True)
    flash.save(path)
    return flash


def regenerate(path: Path | str | None) -> BlackboxFlash:
    """Пересобрать «флеш» заново (например, после команды CLI blackbox_refill)."""
    flash = BlackboxFlash(build_flash_data())
    if path is not None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        flash.save(path)
    return flash


# ------------------------------------------------------------------ «флеш» чёрного ящика
# Биты маски blackboxConfig.fields_disabled_mask (FLIGHT_LOG_FIELD_SELECT_*) —
# порядок совпадает с настройками blackbox_disable_* в CLI-таблице прошивки.
FIELD_SELECT_SETTINGS = [
    "blackbox_disable_pids", "blackbox_disable_rc", "blackbox_disable_setpoint",
    "blackbox_disable_bat", "blackbox_disable_mag", "blackbox_disable_alt",
    "blackbox_disable_rssi", "blackbox_disable_gyro", "blackbox_disable_gyrounfilt",
    "blackbox_disable_acc", "blackbox_disable_debug", "blackbox_disable_motors",
    "blackbox_disable_rpm", "blackbox_disable_gps",
]

# «Флеш» W25Q128 у нашего борта: 4096 секторов по 4 КиБ = 16 МиБ.
FLASH_CAPACITY = BlackboxFlash.CAPACITY
FLASH_SECTORS = BlackboxFlash.CAPACITY // BlackboxFlash.SECTOR_SIZE


def mask_from_settings(settings) -> int:
    """Маска выключенных полей из настроек blackbox_disable_*."""
    mask = 0
    for bit, name in enumerate(FIELD_SELECT_SETTINGS):
        if int(settings.get(name, 0) or 0):
            mask |= 1 << bit
    return mask


def apply_mask_to_settings(settings, mask: int) -> None:
    """Раскладывает маску MSP_SET_BLACKBOX_CONFIG обратно по настройкам blackbox_disable_*."""
    for bit, name in enumerate(FIELD_SELECT_SETTINGS):
        if name in settings:
            settings[name] = 1 if (int(mask) >> bit) & 1 else 0


class BlackboxStore:
    """«Флеш» чёрного ящика: тестовые пролёты, чтение/очистка, кэш на диске.

    Вкладка «Чёрный ящик» читает накопитель через MSP_DATAFLASH_SUMMARY (70) и
    MSP_DATAFLASH_READ (71); здесь лежит то, что отдаётся в ответ. Данные собираются
    моделью пролётов (bb_scenarios.build_model) и кодируются форматом прошивки;
    готовый «флеш» кладётся рядом с «флешем» настроек, поэтому следующие запуски
    эмулятора стартуют мгновенно.
    """

    def __init__(self, path: Path | str | None = None, log=None):
        self.path = Path(path) if path else None
        self.index_path = self.path.with_suffix(".index.json") if self.path else None
        self._log = log or (lambda _msg: None)
        self._lock = threading.RLock()
        self._model_lock = threading.Lock()      # сборка модели пролётов (дорогая, ~секунды)
        self._event = threading.Event()
        self._flights: list | None = None       # модель (потоки данных без кодирования)
        self._flash = BlackboxFlash()
        self._p_interval: int | None = None
        self._building = False
        self._index_source: Path | None = None    # индекс рядом с загруженным «флешем»
        self._catalog_cache: tuple | None = None   # (данные, список логов) — чтобы не сканировать на каждый запрос

    # --- состояние (для MSP_DATAFLASH_SUMMARY)
    @property
    def data(self) -> bytes:
        return self._flash.data

    @property
    def used(self) -> int:
        return self._flash.used

    @property
    def size(self) -> int:
        return FLASH_CAPACITY

    @property
    def sectors(self) -> int:
        return FLASH_SECTORS

    @property
    def flags(self) -> int:
        """MSP_DATAFLASH_SUMMARY: бит0 «готов», бит1 «поддерживается» (msp.c)."""
        return 0b11

    def read(self, address: int, length: int) -> bytes:
        return self._flash.read(address, length)

    def store_ready(self) -> bool:
        """Готов ли накопитель к «режиму запоминающего устройства» (сборка идёт или данные есть)."""
        with self._lock:
            return bool(self._flash.used) or self._building

    def files(self) -> list:
        """Содержимое накопителя «как на флешке»: по файлу на пролёт + один общий дамп.

        Настоящий борт в режиме запоминающего устройства показывает компьютер как обычную
        флешку, поэтому и здесь содержимое раскладывается по файлам: пролёты получают имена
        вида ``LOG00001.BBL``, как их пишет прошивка при выгрузке в MSC.
        """
        data = self.data
        index = self._load_index()
        if not index:
            entries = self.catalog()
            index = {entry["index"]: entry for entry in entries}
        files = []
        for entry in self.catalog():
            meta = index.get(entry["index"], {})
            files.append({
                "name": f"LOG{entry['index']:05d}.BBL",
                "title": meta.get("title") or entry.get("title") or f"лог {entry['index']}",
                "key": meta.get("key") or entry.get("key") or "",
                "size": entry["size"],
                "start": meta.get("start") or entry.get("start_datetime") or "",
                "data": data[entry["offset"]:entry["offset"] + entry["size"]],
            })
        if files:
            files.append({
                "name": "BLACKBOX_ALL.BBL",
                "title": ("все пролёты одним файлом — то, что вкладка пишет кнопкой "
                          "«Сохранено в файл…»"),
                "key": "all",
                "size": len(data),
                "start": "",
                "data": data,
            })
        return files

    def file(self, name: str):
        """Файл по имени из :meth:`files` (или None, если такого нет)."""
        for entry in self.files():
            if entry["name"] == name:
                return entry
        return None

    # --- подготовка данных
    def start(self, p_interval: int) -> None:
        """Готовит «флеш» к работе: читает готовый файл, иначе собирает в фоне.

        Готовый «флеш» может лежать по пути настроек (blackbox_flash.bin рядом с eeprom.json)
        или в поставке эмулятора (``bfemu/data/blackbox_flash.bin``) — во втором случае он
        просто копируется на место, чтобы первый запуск не ждал сборки. Вместе с данными
        копируется и индекс: без него вкладка и консоль показали бы «лог 1» вместо названий.
        """
        p_interval = int(p_interval)
        flash = self._load_matching(p_interval)
        if flash is not None:
            self._flash = flash
            self._p_interval = p_interval
            self._event.set()
            if self.path:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                if not self.path.exists():
                    flash.save(self.path)
                self._copy_index()
            names = [entry["title"] or f"лог {entry['index']}" for entry in self.catalog()]
            self._log(f"[ЧЯ] «флеш» чёрного ящика готов: {flash.used} Б, {len(names)} логов "
                      f"({', '.join(names)})")
            return
        self._spawn(p_interval)

    def ensure(self, p_interval: int, timeout: float = 5.0) -> bool:
        """Данные под нужную частоту: ждём фоновую сборку или собираем прямо сейчас."""
        p_interval = int(p_interval)
        if self._p_interval == p_interval:
            return True
        with self._lock:
            building = self._building
            event = self._event
        if building:
            event.wait(timeout)
            if self._p_interval == p_interval:
                return True
        return self._build_now(p_interval)

    def ensure_async(self, p_interval: int) -> bool:
        """Как ensure(), но НЕ ждёт: нужен другой «P interval» — наполняем в фоне.

        Вызывать из обработчиков MSP: вкладка не должна зависать на несколько секунд.
        """
        p_interval = int(p_interval)
        with self._lock:
            if self._p_interval == p_interval or self._building:
                return False
        self._spawn(p_interval)
        return True

    def _build_now(self, p_interval: int) -> bool:
        with self._lock:
            if self._building:
                return self._p_interval == p_interval
            self._building = True
            self._event.clear()
        try:
            self.rebuild(p_interval)
        finally:
            with self._lock:
                self._building = False
            self._event.set()
        return self._p_interval == p_interval

    def _spawn(self, p_interval: int) -> None:
        """Сборка в фоне: эмулятор отвечает на MSP, пока наполняется «флеш»."""
        with self._lock:
            if self._building:
                return
            self._building = True
            self._event.clear()
        def worker():
            try:
                self.rebuild(p_interval)
            except Exception as exc:                       # сборка не должна ронять эмулятор
                self._log(f"[ЧЯ] не удалось наполнить чёрный ящик: {exc}")
            finally:
                with self._lock:
                    self._building = False
                self._event.set()
        threading.Thread(target=worker, daemon=True, name="blackbox-fill").start()

    def _candidates(self) -> list:
        """Где искать готовый «флеш»: рядом с настройками, затем в поставке эмулятора."""
        candidates = []
        if self.path:
            candidates.append(self.path)
        shipped = Path(__file__).resolve().parent / "data" / "blackbox_flash.bin"
        if shipped not in candidates:
            candidates.append(shipped)
        return candidates

    @staticmethod
    def _index_path_for(flash_path: Path) -> Path:
        return Path(flash_path).with_suffix(".index.json")

    def _copy_index(self) -> None:
        """Индекс (названия пролётов) рядом с рабочим «флешем».

        Файл данных мог быть просто скопирован из поставки — тогда индекса рядом нет.
        Без него вкладка и CLI показали бы «лог 1», поэтому переносим индекс от того
        файла, из которого взяли данные.
        """
        if not (self.path and self.index_path):
            return
        source = Path(getattr(self, "_index_source", None) or self._index_path_for(self.path))
        if source == self.index_path or not source.exists() or self.index_path.exists():
            return
        try:
            self.index_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, self.index_path)
        except OSError:
            pass
        else:
            self._index_source = self.index_path

    def _load_matching(self, p_interval: int):
        """Подходящий готовый «флеш»: сначала из каталога настроек, потом из поставки.

        Если рядом с одним из файлов нет индекса, а рядом с другим есть — берём тот,
        у которого индекс на месте: названия пролётов важнее того, откуда файл.
        """
        matches = []
        for candidate in self._candidates():
            if not candidate.exists():
                continue
            flash = BlackboxFlash.load(candidate)
            if not flash.used or self._header_p_interval(flash.data) != p_interval:
                continue
            matches.append((candidate, flash))
        if not matches:
            return None
        matches.sort(key=lambda item: not self._index_path_for(item[0]).exists())
        candidate, flash = matches[0]
        # Индекс (названия пролётов) берём от того же файла, что и данные: иначе
        # у «флеша» из поставки не будет подписей, и вкладка покажет «лог 1».
        self._index_source = self._index_path_for(candidate)
        return flash

    def rebuild(self, p_interval: int) -> None:
        """Собирает «флеш» заново: модель пролётов + кодирование под заданную частоту.

        Сборка занимает несколько секунд, поэтому идёт ВНЕ замка: пока «флеш» пишется,
        эмулятор продолжает отвечать на MSP_DATAFLASH_SUMMARY/READ старыми данными —
        иначе вкладка получала бы тайм-аут вместо ответа. Готовый «флеш» подменяется
        целиком, одним присваиванием, так что читатели видят либо старый, либо новый.
        """
        p_interval = int(p_interval)
        log_every, i_interval = self._intervals(p_interval)
        if self._flights is None:
            with self._model_lock:
                if self._flights is None:
                    self._log("[ЧЯ] наполняю чёрный ящик: 6 тестовых пролётов "
                              "(взлёт/посадка, прямая, квадрат, отказ двигателя, резонанс луча, акро)")
                    self._flights = build_model()
        flights = recode(self._flights, log_every=log_every, i_interval=i_interval)
        data = bytearray()
        for flight in flights:
            data.extend(flight.encode())
        flash = BlackboxFlash(bytes(data))
        index_text = json.dumps({
            "p_interval": p_interval,
            "logs": [{"key": flight.key, "title": flight.title, "note": flight.note,
                      "start": flight.start.isoformat(timespec="seconds"),
                      "duration": round(flight.duration, 1)} for flight in flights],
        }, ensure_ascii=False, indent=1)
        with self._lock:
            self._catalog_cache = None
            self._flash = flash
            self._p_interval = p_interval
            if self.path:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                flash.save(self.path)
                self._index_source = self.path
            if self.index_path:
                self.index_path.write_text(index_text, encoding="utf-8")
            self._log(f"[ЧЯ] чёрный ящик наполнен: {flash.used} Б, "
                      f"{len(flights)} логов, запись 1/{p_interval} цикла "
                      f"({1_000_000 // (p_interval * LOOPTIME_US)} Гц)"
                      + (f", файл {self.path.name}" if self.path else ""))
        self._event.set()

    def erase(self) -> None:
        """MSP_DATAFLASH_ERASE (72): стираем накопитель (как blackboxEraseAll на борту)."""
        with self._lock:
            self._catalog_cache = None
            self._flash.erase()
            if self.path:
                self._flash.save(self.path)
            self._log("[ЧЯ] «флеш» чёрного ящика очищен (MSP_DATAFLASH_ERASE)")

    def force_refill(self, p_interval: int | None = None) -> None:
        """Наполнить заново в фоне (CLI blackbox refill): не держим вызывающий поток."""
        interval = int(p_interval if p_interval is not None else (self._p_interval or 2))
        self._p_interval = None
        self._spawn(interval)

    def refill(self, p_interval: int | None = None) -> int:
        """Наполнить заново (команда CLI blackbox_refill). Возвращает размер данных."""
        interval = int(p_interval if p_interval is not None else (self._p_interval or 2))
        self.rebuild(interval)
        return self.used

    # --- описание логов
    def catalog(self) -> list:
        """Список логов в накопителе: что показывает вкладка и консоль эмулятора.

        Результат кэшируется: страница эмулятора обновляется каждые 2 с, а разбор
        1,7 МБ «флеша» на чистом Python занимает заметное время.
        """
        cached = self._catalog_cache
        if cached is not None and cached[0] is self._flash.data:
            return [dict(entry) for entry in cached[1]]
        index = self._load_index()
        entries = scan_logs(self.data)
        if not index and len(entries) == len(SCENARIOS):
            # индекс потерялся (например, файл перенесли вручную) — подписываем пролёты
            # стандартным набором эмулятора: порядок логов совпадает с порядком сценариев
            index = {number: {"key": key, "title": title, "note": note}
                     for number, (key, title, note, _duration, _script)
                     in enumerate(SCENARIOS, 1)}
        for entry in entries:
            meta = index.get(entry["index"]) if index else None
            entry["title"] = (meta or {}).get("title", "")
            entry["key"] = (meta or {}).get("key", "")
            entry["note"] = (meta or {}).get("note", "")
            if meta:
                entry["duration"] = meta.get("duration", entry["duration"])
                entry["start_datetime"] = meta.get("start", entry["start_datetime"])
            entry["rate_hz"] = 1_000_000 // (int(entry["looptime"] or LOOPTIME_US)
                                             * max(1, int(entry["p_interval"] or 1)))
        self._catalog_cache = (self._flash.data, entries)
        return [dict(entry) for entry in entries]

    def summary(self) -> dict:
        """Краткая сводка для страницы эмулятора и проверок."""
        return {
            "size": self.size,
            "used": self.used,
            "sectors": self.sectors,
            "free": self.size - self.used,
            "p_interval": self._p_interval,
            "rate_hz": (1_000_000 // (self._p_interval * LOOPTIME_US)
                        if self._p_interval else 0),
            "logs": [{"index": entry["index"], "title": entry["title"] or f"лог {entry['index']}",
                      "size": entry["size"], "duration": entry["duration"],
                      "rate_hz": entry["rate_hz"], "start": entry["start_datetime"]}
                     for entry in self.catalog()],
        }

    # --- служебное
    def _intervals(self, p_interval: int):
        """Частота записи → (log_every, i_interval): как blackboxInit() в прошивке."""
        i_interval = 32
        p_interval = int(p_interval)
        if p_interval < 1:
            p_interval = 1                       # «только I-кадры» в эмуляторе не используется
        return min(p_interval, i_interval), i_interval

    @staticmethod
    def _header_p_interval(data: bytes) -> int:
        """«P interval» из первого лога — чтобы понять, подходит ли готовый файл."""
        entries = scan_logs(data)
        if not entries:
            return 0
        try:
            return int(entries[0]["p_interval"])
        except (TypeError, ValueError):
            return 0

    def _load_index(self) -> dict:
        path = getattr(self, "_index_source", None) or self.index_path
        if not path or not path.exists():
            return {}
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {}
        return {index + 1: entry for index, entry in enumerate(payload.get("logs") or [])}

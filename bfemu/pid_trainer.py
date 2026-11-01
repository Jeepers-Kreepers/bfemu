"""Учебная модель одной оси. Это не код PID Betaflight и не модель реального FC."""
from __future__ import annotations

AXES = ("roll", "pitch", "yaw")
SCENARIOS = ("step", "disturbance", "reverse")


def profile_state(fc, axis):
    return {"profile": fc.current_profile, "axis": axis,
            "values": {term: int(fc.settings[f"{term}_{axis}"]) for term in "pid"}}


def simulate(values, scenario="step"):
    """Детерминированный замкнутый контур, 1 кГц, показания каждые 10 мс.

    P/I/D нормированы относительно заводских значений roll; D действует на гироскоп,
    мотор — инерционное звено, управление и интеграл ограничены. Не включает BF-фильтры,
    feedforward, микшер и реальные рейты. Все величины намеренно учебные.
    """
    dt = 0.001
    gyro = motor = integral = d_filtered = 0.0
    p_gain = 0.0045 * values["p"] / 45
    i_gain = 0.015 * values["i"] / 80
    d_gain = 0.00020 * values["d"] / 40
    previous_gyro = 0.0
    points = []
    for index in range(3001):
        t = index * dt
        target = 200.0 if t >= 0.4 else 0.0
        if scenario == "reverse" and t >= 1.5:
            target = -200.0
        disturbance = -350.0 if scenario == "disturbance" and 1.5 <= t < 2.0 else 0.0
        error = target - gyro
        p = p_gain * error
        integral = max(-0.8, min(0.8, integral + i_gain * error * dt))
        d_filtered += ((gyro - previous_gyro) / dt - d_filtered) * dt / 0.015
        d = -d_gain * d_filtered
        previous_gyro = gyro
        command = max(-1.0, min(1.0, p + integral + d))
        if index % 10 == 0:
            points.append([round(t, 2), round(target, 2), round(gyro, 2),
                           round(p, 3), round(integral, 3), round(d, 3), round(command, 3)])
        motor += (command - motor) * dt / 0.045
        gyro += (2200 * motor - 3 * gyro + disturbance) * dt

    # Оцениваем первую ступень до следующего события; удержание в полосе ±5% до конца окна.
    end = 1.5 if scenario != "step" else 3.0
    window = [row for row in points if 0.4 <= row[0] < end]
    overshoot = max(0.0, (max(row[2] for row in window) - 200) / 2)
    outside = [row[0] for row in window if abs(row[2] - 200) > 10]
    settling = None if outside and outside[-1] >= window[-1][0] else (
        round((outside[-1] + 0.01 if outside else 0.4) - 0.4, 2))
    return {"points": points, "metrics": {"overshoot_pct": round(overshoot, 1),
            "settling_s": settling, "end_error": round(200 - window[-1][2], 1)}}

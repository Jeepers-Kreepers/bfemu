"""Учебная модель одной оси. Это не код PID Betaflight и не модель реального FC."""
from __future__ import annotations

import math

AXES = ("roll", "pitch", "yaw")
SCENARIOS = ("step", "disturbance", "reverse", "throttle")
REFERENCE_TIME_CONSTANT_S = 0.12


def reference_response(points):
    """Учебный отклик без перерегулирования и возмущений, не настройка реального FC.

    Эталон следует тому же заданию, что и сценарий, с постоянной времени 0,12 с.
    На интервале между отсчётами действует задание с предыдущего отсчёта.
    """
    if not points:
        return []
    response = 0.0
    previous_time, previous_target = points[0][:2]
    result = []
    for sample in points:
        time, target = sample[:2]
        alpha = 1.0 - math.exp(-(time - previous_time) / REFERENCE_TIME_CONSTANT_S)
        response += (previous_target - response) * alpha
        result.append([time, round(response, 2)])
        previous_time, previous_target = time, target
    return result


def profile_state(fc, axis):
    return {"profile": fc.current_profile, "axis": axis,
            "values": {term: int(fc.settings[f"{term}_{axis}"]) for term in "pid"}}


def simulate(values, scenario="step", settings=None, axis="roll"):
    """Детерминированный замкнутый контур, 1 кГц, показания каждые 10 мс.

    Коэффициенты нормированы относительно заводских значений roll; D действует на гироскоп,
    мотор — инерционное звено, управление и интеграл ограничены. D Max, Feedforward,
    I-term Relax и TPA — приблизительные демонстрации, не формулы Betaflight.
    """
    dt = 0.001
    gyro = motor = integral = d_filtered = ff_filtered = stick_speed = 0.0
    last_target = 0.0
    settings = settings or {}
    p_gain = 0.0045 * values["p"] / 45
    i_gain = 0.015 * values["i"] / 80
    previous_gyro = 0.0
    points = []
    for index in range(3001):
        t = index * dt
        target = 200.0 if t >= 0.4 else 0.0
        if scenario == "reverse" and t >= 1.5:
            target = -200.0
        if scenario == "throttle" and 1.7 <= t < 2.3:
            target = 300.0
        disturbance = -350.0 if scenario == "disturbance" and 1.5 <= t < 2.0 else 0.0
        # Сценарий газа предназначен для изучения TPA; газ — мкс, не реальные моторы.
        throttle = 1800.0 if scenario == "throttle" and t >= 1.5 else 1300.0
        breakpoint = settings.get("tpa_breakpoint", 2000)
        tpa = 1.0 - settings.get("tpa_rate", 0) / 100 * max(
            0.0, min(1.0, (throttle - breakpoint) / max(1, 2000 - breakpoint)))
        error = target - gyro
        p = p_gain * error * (tpa if settings.get("tpa_mode", 1) == 0 else 1.0)
        velocity = (target - last_target) / dt
        last_target = target
        stick_speed += (abs(velocity) - stick_speed) * dt / 0.05
        relax_mode = settings.get("iterm_relax", 0)
        relax_axis = relax_mode in (2, 4) or (axis != "yaw" and relax_mode in (1, 3))
        relax = 1.0 / (1.0 + stick_speed / max(1, settings.get("iterm_relax_cutoff", 15) * 150)) if relax_axis else 1.0
        integral = max(-0.8, min(0.8, integral + i_gain * error * relax * dt))
        d_filtered += ((gyro - previous_gyro) / dt - d_filtered) * dt / 0.015
        # Учебная аппроксимация D Max и Feedforward, не формулы прошивки.
        minimum_d = min(values["d"], settings.get(f"d_min_{axis}", values["d"]))
        dynamic = min(1.0, abs(d_filtered) / 1200 * settings.get("d_max_gain", 0) / 37
                      + stick_speed / 5000 * settings.get("d_max_advance", 0) / 20)
        effective_d = minimum_d + (values["d"] - minimum_d) * dynamic
        d = -(0.00020 * effective_d / 40) * d_filtered * tpa
        ff_filtered += (velocity - ff_filtered) * dt / 0.03
        feedforward = 0.00008 * settings.get(f"f_{axis}", 0) / 120 * ff_filtered
        previous_gyro = gyro
        limit = settings.get("motor_output_limit", 100) / 100
        command = max(-limit, min(limit, p + integral + d + feedforward))
        if index % 10 == 0:
            points.append([round(t, 2), round(target, 2), round(gyro, 2),
                           round(p, 3), round(integral, 3), round(d, 3), round(command, 3),
                           round(feedforward, 3), round(throttle, 1)])
        motor += (command - motor) * dt / 0.045
        gyro += (2200 * motor - 3 * gyro + disturbance) * dt

    # Оцениваем первую ступень до следующего события; удержание в полосе ±5% до конца окна.
    end = 1.5 if scenario != "step" else 3.0
    window = [row for row in points if 0.4 <= row[0] < end]
    overshoot = max(0.0, (max(row[2] for row in window) - 200) / 2)
    outside = [row[0] for row in window if abs(row[2] - 200) > 10]
    settling = None if outside and outside[-1] >= window[-1][0] else (
        round((outside[-1] + 0.01 if outside else 0.4) - 0.4, 2))
    return {"points": points, "reference_points": reference_response(points),
            "metrics": {"overshoot_pct": round(overshoot, 1),
            "settling_s": settling, "end_error": round(200 - window[-1][2], 1)}}

# Схема только для полей видимой вкладки PID Configurator 4.5.4. Границы и
# варианты читаются из общей базы CLI, а не дублируются в браузере.
FIELD_GROUPS = (
    ("Ползунки PID", (
        ("simplified_pids_mode", "Режим", "Выключено, Roll/Pitch или все три оси."),
        ("simplified_d_gain", "Демпфирование", "Множитель D."),
        ("simplified_pi_gain", "Трекинг", "Множитель P и I."),
        ("simplified_feedforward_gain", "Отзывчивость стиков", "Множитель Feedforward."),
        ("simplified_dmax_gain", "Динамическое демпфирование", "Соотношение D Max и D."),
        ("simplified_i_gain", "Дрейф / колебания", "Дополнительный множитель I."),
        ("simplified_pitch_d_gain", "Демпфирование по тангажу", "Множитель D на Pitch."),
        ("simplified_pitch_pi_gain", "Трекинг по тангажу", "Множитель P, I и Feedforward на Pitch."),
        ("simplified_master_multiplier", "Мастер-множитель", "Общий множитель PID и Feedforward."),
    )),
    ("Таблица PID", tuple(
        (f"{prefix}_{axis}", f"{axis.upper()} · {title}", note)
        for axis in AXES
        for prefix, title, note in (
            ("p", "P", "Пропорциональная реакция на ошибку."),
            ("i", "I", "Интегральная реакция на длительную ошибку."),
            ("d", "D Max", "Верхнее значение динамического D."),
            ("d_min", "Derivative / D", "Базовое значение D (в CLI — d_min)."),
            ("f", "Feedforward", "Реакция на изменение задания."),
        )
    )),
    ("Angle / Horizon", (
        ("angle_p_gain", "Угол", "Сила выравнивания Angle."),
        ("horizon_level_strength", "Горизонт", "Сила выравнивания Horizon."),
        ("horizon_limit_sticks", "Переход", "Переход режима Horizon по стику."),
        ("angle_limit", "Максимальный угол", "Ограничение угла наклона."),
    )),
    ("Feedforward", (
        ("feedforward_jitter_factor", "Подавление дрожания", "Подавление мелких изменений команды."),
        ("feedforward_smooth_factor", "Сглаживание", "Сглаживание Feedforward."),
        ("feedforward_averaging", "Усреднение", "Усреднение команд приёмника."),
        ("feedforward_boost", "Усиление", "Усиление быстрых изменений команды."),
        ("feedforward_max_rate_limit", "Предел максимального значения", "Ограничение Feedforward у максимального рейта."),
        ("feedforward_transition", "Переход", "Ослабление Feedforward вблизи центра стика."),
    )),
    ("TPA и I-term", (
        ("tpa_mode", "Режим TPA", "Какие компоненты ослабляются на высоком газе."),
        ("tpa_rate", "Коэффициент TPA, %", "Величина ослабления."),
        ("tpa_breakpoint", "Порог TPA, мкс", "Газ, начиная с которого действует TPA."),
        ("iterm_relax", "Ослабление I: оси", "Режим I-term Relax."),
        ("iterm_relax_type", "Ослабление I: тип", "Источник для I-term Relax."),
        ("iterm_relax_cutoff", "Ослабление I: отсечка", "Порог/частота I-term Relax."),
        ("anti_gravity_gain", "Антигравитация ×10", "Усиление I при изменении газа; 80 означает 8.0."),
        ("iterm_rotation", "Вращение I", "Компенсация вращения интегрального вектора."),
    )),
    ("Динамическое демпфирование", (
        ("d_max_gain", "Коэффициент усиления D Max", "Реакция динамического D на вращение."),
        ("d_max_advance", "Коэффициент опережения D Max", "Реакция динамического D на команду."),
    )),
    ("Газ и моторы", (
        ("throttle_boost", "Ускорение газа", "Усиление быстрых изменений газа."),
        ("motor_output_limit", "Ограничение выхода на двигатель, %", "Максимальный выход мотора."),
        ("dyn_idle_min_rpm", "Динамический холостой ход, ×100 об/мин", "Минимальные обороты динамического idle."),
        ("vbat_sag_compensation", "Компенсация просадки аккумулятора", "Компенсация падения напряжения."),
        ("thrust_linear", "Линеаризация тяги", "Коррекция нелинейности тяги."),
    )),
    ("Другие настройки", (
        ("auto_profile_cell_count", "Банки для автопереключения профиля", "Специальные значения хранятся как целые числа CLI."),
        ("acro_trainer_angle_limit", "Угол Acro Trainer", "Ограничение угла режима Acro Trainer."),
        ("use_integrated_yaw", "Интегрированный Yaw", "Включает интегрированный режим рыскания."),
        ("abs_control_gain", "Абсолютный контроль", "Усиление Absolute Control."),
    )),
)
SLIDER_NAMES = frozenset(name for name, _, _ in FIELD_GROUPS[0][1])
FIELD_NAMES = frozenset(name for _, fields in FIELD_GROUPS for name, _, _ in fields)
MODELED_NAMES = frozenset(
    [f"{term}_{axis}" for axis in AXES for term in ("p", "i", "d", "d_min", "f")]
    + ["d_max_gain", "d_max_advance", "tpa_mode", "tpa_rate", "tpa_breakpoint",
       "motor_output_limit", "iterm_relax", "iterm_relax_cutoff"]
)


def all_settings(fc):
    return {name: int(fc.settings[name]) for name in FIELD_NAMES}


def field_definitions(fc):
    groups = []
    for title, fields in FIELD_GROUPS:
        entries = []
        for name, label, note in fields:
            info = fc.settings_by_name[name]
            options = info.get("values") if info["mode"] == "MODE_LOOKUP" else None
            entries.append({"name": name, "label": label, "note": note,
                            "min": 0 if options else info["min"],
                            "max": len(options) - 1 if options else info["max"],
                            "options": options, "modeled": name in MODELED_NAMES,
                            "slider": name in SLIDER_NAMES})
        groups.append({"title": title, "fields": entries})
    return groups

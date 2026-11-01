"""Объяснения изменений PID-профиля и сравнение учебных моделей.

Снимки не являются журналом прошивки: между двумя опросами может быть несколько действий.
"""
from __future__ import annotations

from .pid_trainer import AXES, FIELD_GROUPS, MODELED_NAMES, SLIDER_NAMES, simulate


# Для каждой настройки формулируем самостоятельное объяснение, а не сокращённую подпись из формы.
_EFFECTS = {
    "simplified_pids_mode": "Этот режим определяет, применяются ли ползунки к осям крена и тангажа, ко всем трём осям или не применяются вовсе.",
    "simplified_d_gain": "Настройка меняет базовый коэффициент D и верхнюю границу D Max, влияющие на подавление колебаний и отскока после манёвра.",
    "simplified_pi_gain": "Настройка меняет коэффициенты P и I, которые влияют на следование команде и сопротивление длительным возмущениям.",
    "simplified_feedforward_gain": "Настройка меняет коэффициент Feedforward, который влияет на реакцию аппарата на движение стика.",
    "simplified_dmax_gain": "Настройка меняет верхнюю границу D Max относительно базового коэффициента D и влияет на демпфирование при манёврах.",
    "simplified_i_gain": "Настройка меняет интегральный коэффициент I, который помогает исправлять длительную ошибку угловой скорости.",
    "simplified_pitch_d_gain": "Настройка меняет демпфирующие коэффициенты D и D Max для оси тангажа.",
    "simplified_pitch_pi_gain": "Настройка меняет коэффициенты P, I и Feedforward для оси тангажа.",
    "simplified_master_multiplier": "Настройка меняет общий множитель коэффициентов PID и Feedforward одновременно.",
    "angle_p_gain": "Эта настройка задаёт силу стремления аппарата вернуться к заданному углу наклона в режиме Angle.",
    "horizon_level_strength": "Эта настройка задаёт силу самовыравнивания аппарата в режиме Horizon.",
    "horizon_limit_sticks": "Эта настройка влияет на переход между самовыравниванием и акробатическим управлением при отклонении стика в режиме Horizon.",
    "angle_limit": "Эта настройка задаёт максимальный угол наклона, запрашиваемый в режиме Angle.",
    "feedforward_jitter_factor": "Эта настройка задаёт подавление мелких колебаний входной команды при расчёте упреждающего управления Feedforward.",
    "feedforward_smooth_factor": "Эта настройка задаёт степень сглаживания сигнала упреждающего управления Feedforward.",
    "feedforward_averaging": "Эта настройка определяет, используется ли усреднение входных команд при расчёте упреждающего управления Feedforward.",
    "feedforward_boost": "Эта настройка влияет на дополнительное усиление быстрых изменений входной команды при расчёте Feedforward.",
    "feedforward_max_rate_limit": "Эта настройка ограничивает действие упреждающего управления Feedforward при приближении к максимальной угловой скорости.",
    "feedforward_transition": "Эта настройка влияет на ослабление упреждающего управления Feedforward вблизи центрального положения стика.",
    "tpa_mode": "Эта настройка определяет, какие составляющие регулятора ослабляются функцией TPA при высоком положении газа. Эта функция снижает усиление регулятора на высоком газе.",
    "tpa_rate": "Эта настройка задаёт величину ослабления коэффициентов регулятора функцией TPA при высоком положении газа. Эта функция снижает усиление регулятора на высоком газе.",
    "tpa_breakpoint": "Эта настройка задаёт положение газа в микросекундах, выше которого начинает действовать функция TPA.",
    "iterm_relax": "Эта настройка определяет режим ослабления интегральной составляющей I при манёврах и оси, для которых оно применяется.",
    "iterm_relax_type": "Эта настройка определяет, используется ли команда управления или сигнал гироскопа для ослабления интегральной составляющей I.",
    "iterm_relax_cutoff": "Эта настройка задаёт частоту отсечки, влияющую на реакцию механизма ослабления интегральной составляющей I.",
    "anti_gravity_gain": "Эта настройка задаёт усиление механизма антигравитации, который поддерживает стабилизацию при изменениях газа. Значение хранится в десятых долях: 80 соответствует 8,0.",
    "iterm_rotation": "Эта настройка определяет, учитывается ли вращение аппарата при преобразовании накопленной интегральной составляющей.",
    "d_max_gain": "Эта настройка влияет на увеличение демпфирующей составляющей D от базового значения до значения D Max при изменении вращения.",
    "d_max_advance": "Эта настройка влияет на упреждающее увеличение демпфирующей составляющей D в ответ на изменение команды управления.",
    "throttle_boost": "Эта настройка задаёт дополнительное усиление при быстром изменении команды газа.",
    "motor_output_limit": "Эта настройка ограничивает максимальный выход, подаваемый на двигатель, и тем самым влияет на доступный запас управления.",
    "dyn_idle_min_rpm": "Эта настройка задаёт минимальные обороты для динамического холостого хода. Значение задаётся в сотнях оборотов в минуту.",
    "vbat_sag_compensation": "Эта настройка влияет на компенсацию снижения напряжения аккумулятора под нагрузкой.",
    "thrust_linear": "Эта настройка влияет на линеаризацию зависимости тяги от команды, подаваемой на двигатели.",
    "auto_profile_cell_count": "Эта настройка определяет количество элементов аккумулятора, которое используется при автоматическом выборе профиля. Специальные значения имеют отдельное значение в прошивке.",
    "acro_trainer_angle_limit": "Эта настройка задаёт предельный угол наклона, используемый режимом Acro Trainer.",
    "use_integrated_yaw": "Эта настройка включает или выключает интегрированный режим управления по оси рыскания.",
    "abs_control_gain": "Эта настройка задаёт усиление механизма абсолютного контроля, который корректирует накопленное отклонение ориентации.",
}
_AXIS_NAMES = {"roll": "крена", "pitch": "тангажа", "yaw": "рыскания"}
_AXIS_EFFECTS = {
    "p": "Пропорциональный коэффициент P определяет силу реакции контроллера на текущую ошибку угловой скорости по оси {axis}.",
    "i": "Интегральный коэффициент I определяет силу коррекции длительно сохраняющейся ошибки угловой скорости по оси {axis}.",
    "d": "Значение D Max задаёт верхнюю границу динамической демпфирующей составляющей D по оси {axis}.",
    "d_min": "Значение D задаёт базовую демпфирующую составляющую по оси {axis}; в настройках командной строки оно называется d_min.",
    "f": "Коэффициент Feedforward задаёт силу упреждающей реакции на изменение команды стика по оси {axis}.",
}


def _effect(name: str, axis: str | None) -> str:
    if axis:
        prefix = name.removesuffix("_" + axis)
        if prefix in _AXIS_EFFECTS:
            return _AXIS_EFFECTS[prefix].format(axis=_AXIS_NAMES[axis])
    return _EFFECTS[name]


# Коэффициенты, которые данный ползунок может пересчитать. Это не доказательство
# причинной связи: снимок может содержать и независимые изменения из Configurator.
_SLIDER_TARGETS = {
    "simplified_pids_mode": ("p", "i", "d", "d_min", "f"),
    "simplified_master_multiplier": ("p", "i", "d", "d_min", "f"),
    "simplified_d_gain": ("d", "d_min"),
    "simplified_pi_gain": ("p", "i"),
    "simplified_feedforward_gain": ("f",),
    "simplified_dmax_gain": ("d",),
    "simplified_i_gain": ("i",),
    "simplified_pitch_d_gain": ("d", "d_min"),
    "simplified_pitch_pi_gain": ("p", "i", "f"),
}
_SLIDER_GUIDANCE = {
    "simplified_d_gain": (
        "Повышение демпфирования может уменьшить отскок после манёвра, но чрезмерный D усиливает реакцию на шум гироскопа и может нагревать двигатели.",
        "Снижение демпфирования может уменьшить нагрев двигателей, но недостаточный D способен усилить отскок и колебания после манёвра."),
    "simplified_pi_gain": (
        "Повышение коэффициентов P и I может улучшить следование команде и сопротивление возмущениям, но увеличить колебания и отскок.",
        "Снижение коэффициентов P и I может уменьшить колебания и отскок, но ухудшить точность следования команде и удержание при возмущениях."),
    "simplified_feedforward_gain": (
        "Повышение Feedforward может сделать реакцию на движение стика быстрее, но чрезмерное значение способно вызвать резкий отклик и перерегулирование.",
        "Снижение Feedforward может смягчить реакцию на движение стика, но уменьшить отзывчивость управления."),
    "simplified_dmax_gain": (
        "Повышение D Max может усилить демпфирование при манёврах, но способно увеличить реакцию на шум и нагрев двигателей.",
        "Снижение D Max может уменьшить реакцию на шум, но ослабить демпфирование при резких манёврах."),
    "simplified_i_gain": (
        "Повышение I может улучшить удержание при длительных возмущениях, но чрезмерное накопление интегральной составляющей способно вызвать отскок.",
        "Снижение I может уменьшить отскок из-за накопленной ошибки, но увеличить дрейф при длительных возмущениях."),
    "simplified_pitch_d_gain": (
        "Повышение демпфирования по тангажу может уменьшить отскок на этой оси, но чрезмерный D способен усилить шум и нагрев двигателей.",
        "Снижение демпфирования по тангажу может уменьшить реакцию на шум, но усилить отскок и колебания на этой оси."),
    "simplified_pitch_pi_gain": (
        "Повышение коэффициентов P, I и Feedforward по тангажу может улучшить отклик на этой оси, но увеличить колебания и отскок.",
        "Снижение коэффициентов P, I и Feedforward по тангажу может уменьшить колебания, но ухудшить отклик и удержание на этой оси."),
    "simplified_master_multiplier": (
        "Повышение общего множителя может усилить реакцию на команды и возмущения, но одновременно повысить риск колебаний и нагрева двигателей.",
        "Снижение общего множителя может уменьшить колебания и нагрузку на двигатели, но ослабить реакцию на команды и возмущения."),
}


def _slider_context(name: str, after: dict, axis: str, changed: set) -> str:
    mode = after["simplified_pids_mode"]
    if mode == 0:
        return "В активном профиле упрощённая настройка выключена, поэтому применение ползунков не пересчитывает коэффициенты PID."
    if name in ("simplified_pitch_d_gain", "simplified_pitch_pi_gain") and axis != "pitch":
        return "Эта настройка относится к оси тангажа. Для оценки её влияния выберите ось тангажа."
    if axis == "yaw" and mode == 1:
        return "В текущем режиме ползунки применяются только к крену и тангажу. Значения для оси рыскания не пересчитываются."
    if name != "simplified_pids_mode" and not any(f"{prefix}_{axis}" in changed for prefix in _SLIDER_TARGETS[name]):
        return ("При последнем опросе связанные коэффициенты выбранной оси остались прежними. "
                "По этому снимку нельзя подтвердить влияние изменения; проверьте сохранение настроек и ограничения коэффициентов.")
    return ""


_GROUP_GUIDANCE = {
    "Ползунки PID": "Ползунки могут пересчитать несколько коэффициентов. Проверьте таблицу всех осей и выбранный режим ползунков: выключено, крен и тангаж или все три оси.",
    "Angle / Horizon": "Оценивать нужно в соответствующем режиме и в безопасных условиях; модель тренажёра работает по угловой скорости и эти режимы не воспроизводит.",
    "Feedforward": "Сравните ощущение отклика и шум входа; чрезмерная отзывчивость может сделать управление резким. Дополнительные параметры Feedforward в учебной модели не рассчитываются.",
    "TPA и I-term": "Эффект зависит от газа, команды и оси. Проверяйте подходящий сценарий, а не только реакцию на первую ступень.",
    "Динамическое демпфирование": "Проверьте отклик при манёврах и шум гироскопа; чрезмерный D может нагревать моторы, а недостаточный — оставлять колебания.",
    "Газ и моторы": "Действие этих настроек зависит от тяги, питания и электронных регуляторов оборотов двигателей. Проверяйте запас управления и моторы; модель не воспроизводит силовую установку.",
    "Другие настройки": "Действие зависит от режима полёта и оборудования; модель не проверяет это поведение.",
}


def _guidance(name: str, group: str, before: int, after: int) -> str:
    prefix = name.rsplit("_", 1)[0]
    increased = after > before
    if name in _SLIDER_GUIDANCE:
        return _SLIDER_GUIDANCE[name][0 if increased else 1]
    if name == "simplified_pids_mode":
        if after == 0:
            return "Выключение упрощённой настройки прекращает пересчёт коэффициентов ползунками. Ранее установленные коэффициенты остаются в профиле."
        if after == 1:
            return "В этом режиме пересчитываются коэффициенты крена и тангажа. Коэффициенты рыскания сохраняются без пересчёта."
        return "В этом режиме упрощённая настройка распространяется на крен, тангаж и рыскание. Проверьте полученные коэффициенты всех трёх осей."
    if prefix == "p":
        return ("Повышение P может ускорить коррекцию, но увеличить риск колебаний и нагрузку на моторы."
                if increased else "Снижение P может уменьшить колебания, но ухудшить следование заданию.")
    if prefix == "i":
        return ("Повышение I может лучше удерживать заданную скорость при возмущениях, но вызвать накопление интеграла и отскок."
                if increased else "Снижение I может уменьшить накопление интеграла, но увеличить дрейф под нагрузкой.")
    if prefix in ("d", "d_min"):
        return ("Повышение D может улучшить демпфирование, но усилить шум и нагрев моторов; модель не оценивает нагрев."
                if increased else "Снижение D может уменьшить реакцию на шум, но ослабить демпфирование и усилить отскок.")
    if prefix == "f":
        return ("Повышение Feedforward может ускорить реакцию на стик, но сделать её резкой или с перерегулированием."
                if increased else "Снижение Feedforward может смягчить реакцию, но замедлить отклик на стик.")
    if name == "tpa_rate":
        return ("Большее TPA сильнее ослабляет выбранные компоненты на высоком газе; это может уменьшить колебания, но ослабить управление."
                if increased else "Меньшее TPA сохраняет больше усиления на высоком газе, но может повысить склонность к колебаниям.")
    if name == "tpa_breakpoint":
        return ("Более высокий порог откладывает применение TPA до большего газа."
                if increased else "Более низкий порог включает TPA раньше; проверьте устойчивость и отзывчивость.")
    if name == "motor_output_limit":
        return ("Более высокий предел увеличивает доступный выход, но требует проверки силовой установки."
                if increased else "Более низкий предел уменьшает доступный выход и может ограничить управляющий запас.")
    return _GROUP_GUIDANCE.get(group, "Проверьте результат на подходящем сценарии; изменение не гарантирует улучшения.")


_OPTION_EXPLANATIONS = {
    "OFF": "выключено", "ON": "включено", "RP": "крен и тангаж", "RPY": "крен, тангаж и рыскание",
    "RP_INC": "крен и тангаж, только при увеличении интегральной составляющей",
    "RPY_INC": "все три оси, только при увеличении интегральной составляющей",
    "GYRO": "сигнал гироскопа", "SETPOINT": "заданная угловая скорость",
    "PD": "составляющие P и D", "D": "составляющая D",
    "2_POINT": "усреднение по двум точкам", "3_POINT": "усреднение по трём точкам",
    "4_POINT": "усреднение по четырём точкам",
}


def _display(value, options):
    if options and 0 <= value < len(options):
        label = options[value]
        explanation = _OPTION_EXPLANATIONS.get(label)
        return f"{label} ({explanation})" if explanation else label
    return str(value)


def analyze_change(fc, before: dict, after: dict, axis: str, scenario: str) -> dict:
    """Проверить разницу снимков и сравнить один и тот же сценарий до/после."""
    changes = []
    changed = {name for name in before if before[name] != after[name]}
    slider_changes = changed & SLIDER_NAMES
    related_coefficients = {
        f"{prefix}_{item_axis}"
        for slider in slider_changes
        for prefix in _SLIDER_TARGETS[slider]
        for item_axis in AXES
        if not slider.startswith("simplified_pitch_") or item_axis == "pitch"
    }
    for group, fields in FIELD_GROUPS:
        for name, label, _note in fields:
            if before[name] == after[name]:
                continue
            options = fc.settings_by_name[name].get("values")
            field_axis = next((item for item in AXES if name.endswith("_" + item)), None)
            is_modeled = name in MODELED_NAMES and (field_axis is None or field_axis == axis)
            if name in SLIDER_NAMES:
                applicability = _slider_context(name, after, axis, changed)
            elif is_modeled:
                applicability = ""
            elif field_axis and name in MODELED_NAMES:
                applicability = (f"Эта настройка относится к оси {_AXIS_NAMES[field_axis]}. "
                                 f"Её влияние не отображается на выбранном графике оси {_AXIS_NAMES[axis]}.")
            else:
                applicability = ("Учебная модель не рассчитывает влияние этой настройки. "
                                 "По текущим графикам нельзя оценить последствия её изменения.")
            changes.append({"name": name, "label": label, "before": _display(before[name], options),
                            "after": _display(after[name], options), "effect": _effect(name, field_axis),
                            "guidance": _guidance(name, group, before[name], after[name]),
                            "model": applicability, "related": name in related_coefficients})

    def run(settings):
        values = {term: settings[f"{term}_{axis}"] for term in "pid"}
        return simulate(values, scenario, settings, axis)

    previous, current = run(before), run(after)
    max_difference = max(abs(a[2] - b[2]) for a, b in zip(previous["points"], current["points"], strict=True))
    warnings = []
    if ({f"d_min_{axis}", f"d_{axis}"} & changed and
            after[f"d_min_{axis}"] > after[f"d_{axis}"]):
        warnings.append("Базовый D выше D Max на выбранной оси. Модель ограничивает его значением D Max; проверьте профиль.")
    if f"p_{axis}" in changed and after[f"p_{axis}"] == 0:
        warnings.append("P выбранной оси равен нулю: пропорциональная коррекция ошибки отключена.")
    if f"i_{axis}" in changed and after[f"i_{axis}"] == 0:
        warnings.append("I выбранной оси равен нулю: интегральная коррекция постоянного возмущения отключена.")
    if scenario != "throttle" and any(name.startswith("tpa_") for name in (c["name"] for c in changes)):
        warnings.append("Для оценки TPA попробуйте сценарий «Высокий газ»: в других сценариях газа может не хватать для TPA.")
    before_metrics, after_metrics = previous["metrics"], current["metrics"]
    observations = []
    # Ползунки сами не входят в модель; реакцию сравниваем только если изменился
    # хотя бы один учитываемый коэффициент выбранной оси или общая настройка модели.
    modeled_changes = [item for item in changes if item["name"] in MODELED_NAMES and
                       (not any(item["name"].endswith("_" + other) for other in AXES if other != axis))]
    if modeled_changes:
        old_overshoot, new_overshoot = before_metrics["overshoot_pct"], after_metrics["overshoot_pct"]
        if old_overshoot != new_overshoot:
            direction = "выросло" if new_overshoot > old_overshoot else "снизилось"
            observations.append(f"В учебной модели перерегулирование {direction} с {old_overshoot} до {new_overshoot} процентов.")
        old_time, new_time = before_metrics["settling_s"], after_metrics["settling_s"]
        if old_time is None and new_time is not None:
            observations.append(f"Отклик начал удерживаться в пределах пяти процентов от заданной скорости за {new_time} секунды.")
        elif old_time is not None and new_time is None:
            observations.append("Отклик больше не удерживается в пределах пяти процентов от заданной скорости до конца рассматриваемого интервала.")
        elif old_time is not None and new_time is not None and abs(new_time - old_time) >= 0.009:
            direction = "увеличилось" if new_time > old_time else "уменьшилось"
            observations.append(f"Время установления в учебной модели {direction} с {old_time} до {new_time} секунды.")
        old_error, new_error = abs(before_metrics["end_error"]), abs(after_metrics["end_error"])
        if abs(new_error - old_error) >= 0.1:
            direction = "выросла" if new_error > old_error else "снизилась"
            observations.append(f"Абсолютная ошибка угловой скорости в конце интервала {direction} с {old_error} до {new_error} градусов в секунду.")
        if not observations:
            if max_difference >= 0.1:
                observations.append(f"Показатели первой ступени не изменились. Максимальное различие угловой скорости за весь сценарий составило {max_difference:.1f} градуса в секунду.")
            else:
                observations.append("Расчётная реакция выбранной оси в этом сценарии не изменилась. Это не доказывает отсутствие эффекта в других условиях.")
    return {"axis": axis, "scenario": scenario, "changes": changes, "warnings": warnings,
            "observations": observations, "before_metrics": before_metrics, "after_metrics": after_metrics,
            "max_gyro_delta": round(max_difference, 1)}

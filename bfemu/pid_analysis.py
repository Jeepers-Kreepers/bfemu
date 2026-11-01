"""Объяснения изменений PID-профиля и сравнение учебных моделей.

Снимки не являются журналом прошивки: между двумя опросами может быть несколько действий.
"""
from __future__ import annotations

from .pid_trainer import AXES, FIELD_GROUPS, MODELED_NAMES, SLIDER_NAMES, simulate


_GROUP_GUIDANCE = {
    "Ползунки PID": "Ползунки могут пересчитать несколько коэффициентов; проверьте таблицу всех осей и режим OFF/RP/RPY.",
    "Angle / Horizon": "Оценивать нужно в соответствующем режиме и в безопасных условиях; модель тренажёра работает по угловой скорости и эти режимы не воспроизводит.",
    "Feedforward": "Сравните ощущение отклика и шум входа; чрезмерная отзывчивость может сделать управление резким. Дополнительные параметры FF в модели не рассчитываются.",
    "TPA и I-term": "Эффект зависит от газа, команды и оси. Проверяйте подходящий сценарий, а не только реакцию на первую ступень.",
    "Динамическое демпфирование": "Проверьте отклик при манёврах и шум гироскопа; чрезмерный D может нагревать моторы, а недостаточный — оставлять колебания.",
    "Газ и моторы": "Параметры зависят от тяги, питания и ESC. Проверяйте запас управления и моторы; модель не воспроизводит силовую установку.",
    "Другие настройки": "Действие зависит от режима полёта и оборудования; модель не проверяет это поведение.",
}


def _guidance(name: str, group: str, before: int, after: int) -> str:
    prefix = name.rsplit("_", 1)[0]
    increased = after > before
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


def _display(value, options):
    return options[value] if options and 0 <= value < len(options) else str(value)


def analyze_change(fc, before: dict, after: dict, axis: str, scenario: str) -> dict:
    """Проверить разницу снимков и сравнить один и тот же сценарий до/после."""
    changes = []
    for group, fields in FIELD_GROUPS:
        for name, label, note in fields:
            if before[name] == after[name]:
                continue
            options = fc.settings_by_name[name].get("values")
            field_axis = next((item for item in AXES if name.endswith("_" + item)), None)
            is_modeled = name in MODELED_NAMES and (field_axis is None or field_axis == axis)
            if name in SLIDER_NAMES:
                applicability = "возможно косвенно через пересчитанные PID/FF, если режим включён"
            elif is_modeled:
                applicability = "учтено приближённо"
            elif field_axis and name in MODELED_NAMES:
                applicability = "другая ось; не влияет на выбранный график"
            else:
                applicability = "не моделируется"
            changes.append({"name": name, "label": label, "before": _display(before[name], options),
                            "after": _display(after[name], options), "effect": note,
                            "guidance": _guidance(name, group, before[name], after[name]),
                            "model": applicability})

    def run(settings):
        values = {term: settings[f"{term}_{axis}"] for term in "pid"}
        return simulate(values, scenario, settings, axis)

    previous, current = run(before), run(after)
    max_difference = max(abs(a[2] - b[2]) for a, b in zip(previous["points"], current["points"], strict=True))
    warnings = []
    if after[f"d_min_{axis}"] > after[f"d_{axis}"]:
        warnings.append("Базовый D выше D Max на выбранной оси. Модель ограничивает его значением D Max; проверьте профиль.")
    if after[f"p_{axis}"] == 0:
        warnings.append("P выбранной оси равен нулю: пропорциональная коррекция ошибки отключена.")
    if after[f"i_{axis}"] == 0:
        warnings.append("I выбранной оси равен нулю: интегральная коррекция постоянного возмущения отключена.")
    if scenario != "throttle" and any(name.startswith("tpa_") for name in (c["name"] for c in changes)):
        warnings.append("Для оценки TPA попробуйте сценарий «Высокий газ»: в других сценариях газа может не хватать для TPA.")
    before_metrics, after_metrics = previous["metrics"], current["metrics"]
    observations = []
    overshoot_diff = round(after_metrics["overshoot_pct"] - before_metrics["overshoot_pct"], 1)
    if overshoot_diff:
        direction = "выросло" if overshoot_diff > 0 else "снизилось"
        observations.append(f"В модели перерегулирование {direction} на {abs(overshoot_diff)} п.п.; это не оценка реального полёта.")
    old_time, new_time = before_metrics["settling_s"], after_metrics["settling_s"]
    if old_time is None and new_time is not None:
        observations.append("В модели отклик начал укладываться в полосу ±5% до конца окна.")
    elif old_time is not None and new_time is None:
        observations.append("В модели отклик перестал укладываться в полосу ±5% до конца окна.")
    elif old_time is not None and new_time is not None and abs(new_time - old_time) >= 0.01:
        direction = "увеличилось" if new_time > old_time else "уменьшилось"
        observations.append(f"Время установления в модели {direction} на {abs(new_time - old_time):.2f} с.")
    old_error, new_error = abs(before_metrics["end_error"]), abs(after_metrics["end_error"])
    if abs(new_error - old_error) >= 0.1:
        direction = "выросла" if new_error > old_error else "снизилась"
        observations.append(f"Абсолютная ошибка в конце окна {direction} на {abs(new_error - old_error):.1f} °/с.")
    if not observations:
        observations.append("Округлённые метрики первой ступени не изменились; это не доказывает отсутствие эффекта или безопасность.")
    return {"axis": axis, "scenario": scenario, "changes": changes, "warnings": warnings,
            "observations": observations, "before_metrics": before_metrics, "after_metrics": after_metrics,
            "max_gyro_delta": round(max_difference, 1)}

"""Анализ сообщает наблюдаемые изменения и полезные риски без лишних объяснений."""
from bfemu.pid_analysis import analyze_change
from bfemu.pid_trainer import all_settings


def test_analysis_direction_axis_and_model_limits(fc):
    before = all_settings(fc)
    after = {**before, "p_roll": 65, "d_pitch": 70, "anti_gravity_gain": 100}
    result = analyze_change(fc, before, after, "roll", "step")
    changes = {item["name"]: item for item in result["changes"]}
    assert set(changes) == {"p_roll", "d_pitch", "anti_gravity_gain"}
    assert changes["p_roll"]["before"] == "45" and changes["p_roll"]["after"] == "65"
    assert "колебан" in changes["p_roll"]["guidance"]
    assert "относится к оси тангажа" in changes["d_pitch"]["model"]
    assert "не рассчитывает влияние этой настройки" in changes["anti_gravity_gain"]["model"]
    assert changes["p_roll"]["model"] == ""  # Не повторяем устройство модели для каждой настройки.
    assert result["max_gyro_delta"] > 0
    assert result["before_metrics"] != result["after_metrics"]
    assert result["observations"]


def test_damping_slider_gives_actionable_risk_without_demagogy(fc):
    before = all_settings(fc)
    fc.settings["simplified_d_gain"] = 125
    after = {**before, "simplified_d_gain": 125, **fc._calculate_simplified_pids()}
    report = analyze_change(fc, before, after, "pitch", "step")
    changes = {item["name"]: item for item in report["changes"]}
    slider = changes["simplified_d_gain"]
    assert (slider["before"], slider["after"]) == ("100", "125")
    assert "отскок" in slider["guidance"] and "нагревать двигатели" in slider["guidance"]
    assert slider["model"] == ""
    assert changes["d_pitch"]["related"] and changes["d_min_pitch"]["related"]
    assert changes["d_pitch"]["before"] == "46" and changes["d_pitch"]["after"] == "57"
    assert report["observations"]
    assert "не используется в расчёте графика напрямую" not in str(report)
    assert "а не на положение ползунка" not in str(report)


def test_damping_slider_without_coefficient_change_is_not_misrepresented(fc):
    before = all_settings(fc)
    after = {**before, "simplified_d_gain": 125}
    report = analyze_change(fc, before, after, "pitch", "step")
    assert "коэффициенты выбранной оси остались прежними" in report["changes"][0]["model"]
    assert "подтвердить влияние" in report["changes"][0]["model"]
    assert report["observations"] == []
    disabled = {**after, "simplified_pids_mode": 0}
    report = analyze_change(fc, {**before, "simplified_pids_mode": 0}, disabled, "pitch", "step")
    assert "упрощённая настройка выключена" in report["changes"][0]["model"]
    assert report["observations"] == []


def test_damping_slider_direction_and_selected_axis(fc):
    before = all_settings(fc)
    change = analyze_change(fc, before, {**before, "simplified_d_gain": 80}, "roll", "step")["changes"][0]
    assert "Снижение демпфирования" in change["guidance"] and "отскок" in change["guidance"]
    change = analyze_change(fc, before, {**before, "simplified_pitch_d_gain": 120}, "roll", "step")["changes"][0]
    assert "Для оценки её влияния выберите ось тангажа" in change["model"]
    before["simplified_pids_mode"] = 1
    change = analyze_change(fc, before, {**before, "simplified_d_gain": 120}, "yaw", "step")["changes"][0]
    assert "только к крену и тангажу" in change["model"]


def test_analysis_different_context_and_warnings(fc):
    before = all_settings(fc)
    after = {**before, "i_roll": 0, "d_min_roll": 60, "tpa_rate": 90}
    report = analyze_change(fc, before, after, "roll", "step")
    assert any("Базовый D выше" in warning for warning in report["warnings"])
    assert any("интегральная" in warning for warning in report["warnings"])
    assert any("Высокий газ" in warning for warning in report["warnings"])
    assert not any("Высокий газ" in warning for warning in analyze_change(fc, before, after, "roll", "throttle")["warnings"])
    mode = analyze_change(fc, before, {**before, "simplified_pids_mode": 0}, "roll", "step")["changes"][0]
    assert mode["before"] == "RPY (крен, тангаж и рыскание)" and mode["after"] == "OFF (выключено)"
    assert "упрощённая настройка выключена" in mode["model"]


def test_unmodeled_field_does_not_claim_model_observation(fc):
    before = all_settings(fc)
    result = analyze_change(fc, before, {**before, "angle_limit": 45}, "roll", "reverse")
    assert result["max_gyro_delta"] == 0
    assert result["before_metrics"] == result["after_metrics"]
    assert result["observations"] == []
    assert "не рассчитывает влияние этой настройки" in result["changes"][0]["model"]


def test_all_parameters_have_complete_relevant_explanations(fc):
    from bfemu.pid_trainer import AXES, FIELD_NAMES

    before = all_settings(fc)
    after = {name: value + 1 for name, value in before.items()}
    for axis in AXES:
        report = analyze_change(fc, before, after, axis, "step")
        assert {change["name"] for change in report["changes"]} == FIELD_NAMES
        for change in report["changes"]:
            assert change["effect"].endswith(".") and change["guidance"].endswith(".")
            assert not change["model"] or change["model"].endswith(".")
        for observation in report["observations"]:
            assert "п.п." not in observation and "°/с" not in observation

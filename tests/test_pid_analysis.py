"""Анализ двух снимков не выдаёт учебную модель за реальную прошивку."""
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
    assert changes["d_pitch"]["model"].startswith("другая ось")
    assert changes["anti_gravity_gain"]["model"] == "не моделируется"
    assert result["max_gyro_delta"] > 0
    assert result["before_metrics"] != result["after_metrics"]
    assert result["observations"] and all("реального полёта" in text or "модел" in text or
                                          "ошибк" in text for text in result["observations"])


def test_analysis_different_context_and_warnings(fc):
    before = all_settings(fc)
    after = {**before, "i_roll": 0, "d_min_roll": 60, "tpa_rate": 90}
    report = analyze_change(fc, before, after, "roll", "step")
    assert any("Базовый D выше" in warning for warning in report["warnings"])
    assert any("интегральная" in warning for warning in report["warnings"])
    assert any("Высокий газ" in warning for warning in report["warnings"])
    assert not any("Высокий газ" in warning for warning in analyze_change(fc, before, after, "roll", "throttle")["warnings"])
    mode = analyze_change(fc, before, {**before, "simplified_pids_mode": 0}, "roll", "step")["changes"][0]
    assert mode["before"] == "RPY" and mode["after"] == "OFF"
    assert "косвенно" in mode["model"]


def test_unmodeled_field_does_not_change_model(fc):
    before = all_settings(fc)
    result = analyze_change(fc, before, {**before, "angle_limit": 45}, "roll", "reverse")
    assert result["max_gyro_delta"] == 0
    assert result["before_metrics"] == result["after_metrics"]
    assert "не доказывает" in result["observations"][0]
    assert result["changes"][0]["model"] == "не моделируется"

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
    assert "относится к оси PITCH" in changes["d_pitch"]["model"]
    assert "но его влияние не рассчитывается" in changes["anti_gravity_gain"]["model"]
    assert "разнице между заданной и текущей угловой скоростью" in changes["p_roll"]["model"]
    assert "поведение реальной прошивки Betaflight" in changes["p_roll"]["model"]
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
    assert mode["before"] == "RPY (крен, тангаж и рыскание)" and mode["after"] == "OFF (выключено)"
    assert "не используется в расчёте графика напрямую" in mode["model"]
    assert "пересчитать коэффициенты PID и Feedforward" in mode["model"]


def test_unmodeled_field_does_not_change_model(fc):
    before = all_settings(fc)
    result = analyze_change(fc, before, {**before, "angle_limit": 45}, "roll", "reverse")
    assert result["max_gyro_delta"] == 0
    assert result["before_metrics"] == result["after_metrics"]
    assert "не доказывает" in result["observations"][0]
    assert "его влияние не рассчитывается учебной моделью" in result["changes"][0]["model"]


def test_all_parameters_have_complete_explanations(fc):
    from bfemu.pid_trainer import AXES, FIELD_NAMES, MODELED_NAMES

    before = all_settings(fc)
    # Меняем все поля в снимке, чтобы проверить полное текстовое покрытие без записи в FC.
    after = {name: value + 1 for name, value in before.items()}
    for axis in AXES:
        report = analyze_change(fc, before, after, axis, "step")
        assert {change["name"] for change in report["changes"]} == FIELD_NAMES
        for change in report["changes"]:
            assert change["effect"].endswith(".") and len(change["effect"]) > 50
            assert change["guidance"].endswith(".") and len(change["guidance"]) > 50
            assert change["model"].endswith(".") and len(change["model"]) > 100
            assert change["label"] in change["model"]
            if change["name"] in MODELED_NAMES and (
                not change["name"].endswith(tuple("_" + other for other in AXES if other != axis))
            ):
                assert "В учебной модели" in change["model"]
        for observation in report["observations"]:
            assert "п.п." not in observation and "°/с" not in observation

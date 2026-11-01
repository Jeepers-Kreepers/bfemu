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
    assert "влияние этой настройки на график не рассчитывается" in changes["anti_gravity_gain"]["model"]
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
    assert "влияние этой настройки на график не рассчитывается" in result["changes"][0]["model"]


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


def test_integral_explanation_identifies_rate_error_and_gain_consequences(fc):
    before = all_settings(fc)
    for name, axis in (("i_roll", "roll"), ("i_pitch", "pitch"), ("i_yaw", "yaw"),
                       ("simplified_i_gain", "roll")):
        for difference in (-1, 1):
            result = analyze_change(fc, before, {**before, name: before[name] + difference}, axis, "step")
            change = next(item for item in result["changes"] if item["name"] == name)
            assert "ошибку угловой скорости" in change["effect"]
            assert "заданной скоростью вращения" in change["effect"]
            assert "измеренной гироскопом" in change["effect"]
            assert "длительной ошибки угловой скорости" in change["guidance"]
            assert "накопление интеграла" not in change["guidance"]
            assert "накопленной ошибки" not in change["guidance"]
            assert ("ослабляет" if difference < 0 else "усиливает") in change["guidance"]


def test_every_field_has_individual_actionable_guidance_in_both_directions(fc):
    from bfemu.pid_trainer import FIELD_NAMES

    before = all_settings(fc)
    for name in FIELD_NAMES:
        # Этот тест проверяет пояснения каждого направления, даже если граница
        # реального диапазона не допускает одно из них для заводского значения.
        changes = []
        for delta in (-1, 1):
            old = {**before, name: 1 if name == "auto_profile_cell_count" else before[name]}
            new = {**old, name: old[name] + delta}
            change = analyze_change(fc, old, new, "roll", "step")["changes"][0]
            assert change["effect"].endswith(".")
            assert change["guidance"].endswith(".")
            assert "Проверьте результат на подходящем сценарии" not in change["guidance"]
            assert "Эффект зависит от газа, команды и оси" not in change["guidance"]
            changes.append(change["guidance"])
        # Переключатели и выбор осей могут иметь особые значения вне диапазона;
        # валидные варианты отдельно проверяются ниже.
        if name not in {"use_integrated_yaw", "iterm_rotation", "iterm_relax_type", "tpa_mode"}:
            assert changes[0] != changes[1], name


def test_unmodeled_options_and_direction_explain_real_effects(fc):
    before = all_settings(fc)

    def change(name, old, new, **context):
        snapshot = {**before, **context, name: old}
        return analyze_change(fc, snapshot, {**snapshot, name: new}, "roll", "step")["changes"][0]

    higher = change("feedforward_smooth_factor", 25, 35)
    lower = change("feedforward_smooth_factor", 25, 15)
    assert "задержку" in higher["guidance"] and "отзывчивее" in lower["guidance"]
    assert "реальному Betaflight" in higher["model"] and "график не рассчитывается" in higher["model"]
    for old, new, phrase in [(0, 20, "включён"), (20, 0, "выключен"), (20, 30, "Большее"), (30, 20, "Меньшее")]:
        guidance = change("feedforward_max_rate_limit", old, new)["guidance"]
        assert phrase in guidance
        assert "крен" in guidance and "тангаж" in guidance
    assert "SETPOINT" in change("iterm_relax_type", 0, 1)["guidance"]
    assert "GYRO" in change("iterm_relax_type", 1, 0)["guidance"]
    assert "не действует" in change("iterm_relax_type", 1, 0, iterm_relax=0)["guidance"]
    assert "не влияет" in change("iterm_relax_cutoff", 15, 20, iterm_relax=0)["guidance"]
    assert "не действует" in change("abs_control_gain", 0, 1, iterm_relax=0)["guidance"]
    assert "данные об оборотах" in change("dyn_idle_min_rpm", 0, 30)["guidance"]
    assert "не жёсткая гарантия" in change("acro_trainer_angle_limit", 20, 30)["guidance"]
    assert "ограничение команды" in change("angle_limit", 30, 40)["guidance"]
    assert "свежем аккумуляторе" in change("vbat_sag_compensation", 0, 10)["guidance"]


def test_modes_and_profile_special_values_have_specific_meaning(fc):
    before = all_settings(fc)

    def change(name, old, new):
        snapshot = {**before, name: old}
        return analyze_change(fc, snapshot, {**snapshot, name: new}, "yaw", "step")["changes"][0]

    assert "выключен" in change("iterm_relax", 1, 0)["guidance"]
    assert "рыскания" in change("iterm_relax", 1, 2)["guidance"]
    inc = change("iterm_relax", 1, 3)
    assert "только увеличение" in inc["guidance"]
    assert "INC отдельно не воспроизводится" in inc["model"]
    assert "только D" in change("tpa_mode", 0, 1)["guidance"]
    assert "P и D" in change("tpa_mode", 1, 0)["guidance"]
    assert "накопленной I-коррекции" in change("iterm_rotation", 0, 1)["guidance"]
    assert "обычный прямой выход" in change("use_integrated_yaw", 1, 0)["guidance"]
    assert "оставаться на профиле" in change("auto_profile_cell_count", -1, 0)["after"]
    assert "переключаться на подходящий профиль" in change("auto_profile_cell_count", 0, -1)["after"]
    assert "с 4 элементами" in change("auto_profile_cell_count", 0, 4)["guidance"]
    assert "вне диапазона" in change("auto_profile_cell_count", 0, -2)["guidance"]


def test_cell_count_annotation_explains_indirect_effect_and_emulator_limit(fc):
    before = all_settings(fc)
    after = {**before, "auto_profile_cell_count": 6}
    report = analyze_change(fc, before, after, "roll", "step")
    assert report["max_gyro_delta"] == 0
    assert report["before_metrics"] == report["after_metrics"]
    assert report["observations"] == []
    change = report["changes"][0]
    assert "не меняет PID-коэффициенты и график" in change["model"]
    assert "ручном переключении" in change["model"]
    assert "график может измениться" in change["model"]
    assert "при подключении аккумулятора" in change["model"]
    assert "пока не реализовано" in change["model"]


def test_observation_classification_uses_each_metric_not_the_setting_direction(fc, monkeypatch):
    import bfemu.pid_analysis as analysis

    before = all_settings(fc)
    after = {**before, 'p_roll': before['p_roll'] + 1}
    old = {'overshoot_pct': 20, 'settling_s': 1.2, 'end_error': -12}
    new = {'overshoot_pct': 10, 'settling_s': .8, 'end_error': -4}
    curve_delta = [1]

    def simulate(values, scenario, settings, axis):
        is_before = settings['p_roll'] == before['p_roll']
        return {'metrics': old if is_before else new, 'points': [[0, 0, 0 if is_before else curve_delta[0]]]}

    monkeypatch.setattr(analysis, 'simulate', simulate)

    def outcomes():
        report = analysis.analyze_change(fc, before, after, 'roll', 'step')
        assert report['observations'] == [item['text'] for item in report['observation_details']]
        return [item['outcome'] for item in report['observation_details']]

    assert outcomes() == ['positive', 'positive', 'positive']
    # Результат может быть смешанным: не объявляем всё изменение настройки хорошим.
    new.update(overshoot_pct=30, settling_s=.8, end_error=15)
    assert outcomes() == ['negative', 'positive', 'negative']
    # Знак ошибки не важен, важна её абсолютная величина.
    new.update(overshoot_pct=20, settling_s=1.2, end_error=3)
    assert outcomes() == ['positive']
    new.update(end_error=-30)
    assert outcomes() == ['negative']
    # Отсутствие установления в окне и появление установления оцениваются отдельно.
    old.update(settling_s=None, end_error=0)
    new.update(overshoot_pct=20, settling_s=.8, end_error=0)
    assert outcomes() == ['positive']
    old.update(settling_s=.8)
    new.update(settling_s=None)
    assert outcomes() == ['negative']
    old.update(settling_s=.8)
    new.update(settling_s=1.1)
    assert outcomes() == ['negative']
    # Разница кривых сама по себе не говорит о том, лучше или хуже результат.
    new.update(settling_s=.8)
    assert outcomes() == ['neutral']
    curve_delta[0] = 0
    assert outcomes() == ['neutral']
    assert 'не изменилась' in analysis.analyze_change(fc, before, after, 'roll', 'step')['observations'][0]


def test_no_modeled_observation_is_not_given_a_sentiment(fc):
    before = all_settings(fc)
    report = analyze_change(fc, before, {**before, 'angle_limit': 45}, 'roll', 'step')
    assert report['observations'] == [] and report['observation_details'] == []

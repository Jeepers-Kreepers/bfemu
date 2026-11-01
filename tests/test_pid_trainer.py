"""Проверки учебной модели PID; это не тесты реальной прошивки."""
from itertools import pairwise

from bfemu.pid_trainer import simulate


def test_model_closed_loop_and_damping():
    zero = simulate({"p": 0, "i": 0, "d": 0})
    normal = simulate({"p": 45, "i": 80, "d": 40})
    assert zero["metrics"]["settling_s"] is None
    assert zero["metrics"]["end_error"] == 200
    assert abs(normal["metrics"]["end_error"]) < 1
    assert normal["metrics"]["settling_s"] is not None
    no_d = simulate({"p": 100, "i": 80, "d": 0})
    with_d = simulate({"p": 100, "i": 80, "d": 100})
    assert with_d["metrics"]["overshoot_pct"] < no_d["metrics"]["overshoot_pct"]
    assert all(-1 <= row[6] <= 1 for row in with_d["points"])


def test_scenarios_and_determinism():
    values = {"p": 45, "i": 80, "d": 40}
    step = simulate(values)
    assert simulate(values) == step
    reverse = simulate(values, "reverse")
    disturbance = simulate(values, "disturbance")
    assert reverse["points"][200][1] == -200
    assert disturbance["points"][180][2] != step["points"][180][2]


def test_extended_model_parameters_are_effective():
    values = {"p": 45, "i": 80, "d": 40}
    zero_ff = {"f_roll": 0, "d_min_roll": 20, "d_max_gain": 37, "d_max_advance": 20}
    with_ff = {**zero_ff, "f_roll": 300}
    assert simulate(values, "reverse", zero_ff)["points"] != simulate(values, "reverse", with_ff)["points"]
    no_tpa = {**zero_ff, "tpa_mode": 0, "tpa_rate": 0, "tpa_breakpoint": 1350}
    tpa = {**no_tpa, "tpa_rate": 90}
    assert simulate(values, "throttle", no_tpa)["points"] != simulate(values, "throttle", tpa)["points"]
    limited = {**zero_ff, "motor_output_limit": 35}
    assert all(abs(point[6]) <= .35 for point in simulate(values, settings=limited)["points"])
    assert simulate(values, settings={**zero_ff, "d_min_roll": 0})["points"] != simulate(
        values, settings=zero_ff)["points"]


def test_reference_is_fixed_monotonic_and_independent_of_pid():
    step = simulate({"p": 45, "i": 80, "d": 40}, "step")
    bad_pid = simulate({"p": 0, "i": 0, "d": 0}, "step")
    reference = step["reference_points"]
    assert reference == bad_pid["reference_points"]
    assert len(reference) == len(step["points"]) == 301
    assert reference[40] == [0.4, 0.0]
    assert reference[52][1] == 126.42
    assert reference[76][1] == 190.04  # 95% через 0,36 секунды после ступени.
    assert all(0 <= a[1] <= b[1] <= 200 for a, b in pairwise(reference))
    assert simulate({"p": 45, "i": 80, "d": 40}, "disturbance")["reference_points"] == reference


def test_reference_tracks_each_scenario_without_overshoot():
    values = {"p": 45, "i": 80, "d": 40}
    reverse = simulate(values, "reverse")["reference_points"]
    assert reverse[150][1] > 199 and reverse[200][1] < -190
    assert all(-200 <= row[1] <= 200 for row in reverse)
    throttle = simulate(values, "throttle")["reference_points"]
    assert throttle[170][1] == 200
    assert 290 < throttle[200][1] < 300
    assert 200 < throttle[250][1] < 220


def test_first_zero_error_is_sampled_after_step_and_can_be_transient():
    from bfemu.pid_trainer import first_zero_error_time

    points = [
        [0.0, 0.0, 0.0],        # Совпадение до ступени не считается.
        [0.4, 200.0, 0.0],
        [0.5, 200.0, 199.99],   # Близко к нулю, но не равно нулю.
        [0.6, 200.0, 200.0],    # Первое совпадение.
        [0.7, 200.0, 205.0],    # После совпадения возможен отскок.
        [1.5, 200.0, 200.0],
    ]
    assert first_zero_error_time(points, 0.4, 1.5) == 0.2
    assert first_zero_error_time(points[:3], 0.4, 1.5) is None
    assert first_zero_error_time(points[4:], 0.4, 1.5) is None  # Граница следующего события исключается.
    # Пересечение между отсчётами не является совпадением на отсчёте.
    assert first_zero_error_time([[0.4, 200.0, 199.0], [0.41, 200.0, 201.0]], 0.4, 1.5) is None


def test_first_zero_error_metric_in_real_scenarios(fc):
    values = {term: fc.settings[f"{term}_roll"] for term in "pid"}
    step = simulate(values, "step", fc.settings, "roll")
    assert step["metrics"]["first_zero_error_s"] == 2.16
    for scenario in ("reverse", "disturbance", "throttle"):
        result = simulate(values, scenario, fc.settings, "roll")
        assert result["metrics"]["first_zero_error_s"] is None
    stopped = simulate({"p": 0, "i": 0, "d": 0})
    assert stopped["metrics"]["first_zero_error_s"] is None


def test_resultant_signal_matches_sum_after_output_limit():
    """Отдельный график показывает ограниченную сумму, а не отдельную составляющую."""
    values = {"p": 110, "i": 80, "d": 45}
    limit = 0.35
    settings = {"motor_output_limit": 35, "f_roll": 120, "d_min_roll": 25,
                "d_max_gain": 37, "d_max_advance": 20}
    points = simulate(values, "reverse", settings, "roll")["points"]
    assert any(abs(row[6]) == limit for row in points)
    for row in points:
        rounded_sum = row[3] + row[4] + row[5] + row[7]
        expected = max(-limit, min(limit, rounded_sum))
        # В кадре каждая составляющая и сумма округлены отдельно до 0,001.
        assert abs(row[6] - expected) <= 0.0025

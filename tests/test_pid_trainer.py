"""Проверки учебной модели PID; это не тесты реальной прошивки."""
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

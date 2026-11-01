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

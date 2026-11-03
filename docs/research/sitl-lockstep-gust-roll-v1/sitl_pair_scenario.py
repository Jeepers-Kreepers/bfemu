"""Явные условия сценария «Возмущение — порыв ветра» для парного опыта."""
import math

AXES = {'roll': 0, 'pitch': 1, 'yaw': 2}
GUST_TORQUE_NM = 0.05
GUST_FRAMES = 50  # 0,2 с при шаге 4 мс
GUST_RECOVERY_BAND_DPS = 5.0


def validate_gust(moment_nm, duration_frames):
    if (not isinstance(moment_nm, (int, float)) or isinstance(moment_nm, bool) or
            not math.isfinite(moment_nm) or not 0 < abs(moment_nm) <= 0.2):
        raise ValueError('Момент порыва должен быть ненулевым и не больше 0,2 Н·м по модулю.')
    if type(duration_frames) is not int or not 5 <= duration_frames <= 200:
        raise ValueError('Длительность порыва должна составлять от 20 до 800 мс с шагом 4 мс.')


def gust_torque(axis, moment_nm=GUST_TORQUE_NM):
    """Постоянный момент в осях тела FRD; выбранное воздействие, не аэродинамика."""
    if axis not in AXES:
        raise ValueError('Неизвестная ось возмущения.')
    values = [0., 0., 0.]
    values[AXES[axis]] = moment_nm
    return tuple(values)

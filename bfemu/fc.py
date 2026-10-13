"""
Виртуальный полётный контроллер: состояние + обработчики MSP + CLI.

Идентичность по умолчанию: Betaflight 4.5.4, MSP API 1.46, MCU F405, variant BTFL.
Такой набор выбран не случайно:
  * API 1.46 >= минимально поддерживаемого Configurator'ом (1.44);
  * версия прошивки 4.5.4 включает CLI-поверх-MSP (MSP2_CLI_COMMAND);
  * версия < 10 в MSP_FC_VERSION передаётся тремя байтами (старый формат),
    что совпадает с реальной прошивкой 4.5.x.
"""
from __future__ import annotations

import json
import math
import os
import random
import threading
import time
from pathlib import Path

from . import bb_scenarios
from . import msp as mspmod
from .cli import CliEngine
from .msp import Reader, Writer

DATA_PATH = Path(__file__).resolve().parent / "data" / "bf_data.json"

# ---------------------------------------------------------------- MSP-коды
# Значения взяты из src/main/msp/msp_protocol*.h прошивки (Betaflight 4.5.4).
MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_BOARD_INFO = 4
MSP_BUILD_INFO = 5
MSP_NAME = 10
MSP_SET_NAME = 11
MSP_BATTERY_CONFIG = 32
MSP_SET_BATTERY_CONFIG = 33
MSP_MODE_RANGES = 34
MSP_SET_MODE_RANGE = 35
MSP_MODE_RANGES_EXTRA = 238
MSP_FEATURE_CONFIG = 36
MSP_SET_FEATURE_CONFIG = 37
MSP_BOARD_ALIGNMENT_CONFIG = 38
MSP_SET_BOARD_ALIGNMENT_CONFIG = 39
MSP_CURRENT_METER_CONFIG = 40
MSP_SET_CURRENT_METER_CONFIG = 41
MSP_MIXER_CONFIG = 42
MSP_SET_MIXER_CONFIG = 43
MSP_RX_CONFIG = 44
MSP_SET_RX_CONFIG = 45
MSP_RSSI_CONFIG = 50
MSP_SET_RSSI_CONFIG = 51
MSP_CF_SERIAL_CONFIG = 54
MSP_SET_CF_SERIAL_CONFIG = 55
MSP_VOLTAGE_METER_CONFIG = 56
MSP_SET_VOLTAGE_METER_CONFIG = 57
MSP_PID_CONTROLLER = 59
MSP_ARMING_CONFIG = 61
MSP_SET_ARMING_CONFIG = 62
MSP_RX_MAP = 64
MSP_SET_RX_MAP = 65
MSP_REBOOT = 68
# MSP_REBOOT_* из msp/msp.c 4.5.4: 0 — обычная перезагрузка, 1 — загрузчик в ПЗУ,
# 2/3 — режим запоминающего устройства (MSC / MSC с UTC), 4 — загрузчик во «флеше».
MSP_REBOOT_FIRMWARE = 0
MSP_REBOOT_BOOTLOADER_ROM = 1
MSP_REBOOT_MSC = 2
MSP_REBOOT_MSC_UTC = 3
MSP_REBOOT_BOOTLOADER_FLASH = 4
MSP_REBOOT_COUNT = 5
MSC_SECONDS = 600                  # сколько эмулятор остаётся в режиме ЗУ без вмешательства
MSP_DATAFLASH_SUMMARY = 70
MSP_DATAFLASH_BUFFER_SIZE = 4096   # MSP_PORT_DATAFLASH_BUFFER_SIZE из msp_serial.c
MSP_DATAFLASH_READ = 71
MSP_DATAFLASH_ERASE = 72
MSP_LOOP_TIME = 73
MSP_FAILSAFE_CONFIG = 75
MSP_SET_FAILSAFE_CONFIG = 76
MSP_RXFAIL_CONFIG = 77
MSP_SET_RXFAIL_CONFIG = 78
MSP_SDCARD_SUMMARY = 79
MSP_BLACKBOX_CONFIG = 80
MSP_SET_BLACKBOX_CONFIG = 81
MSP_TRANSPONDER_CONFIG = 82
MSP_OSD_CONFIG = 84
MSP_SET_OSD_CONFIG = 85
MSP_OSD_CHAR_READ = 86
MSP_OSD_CHAR_WRITE = 87
MSP_OSD_CANVAS = 189               # размер «полотна» OSD (столбцы × строки)
MSP_SET_OSD_CANVAS = 190
MSP_VTX_CONFIG = 88
MSP_SET_VTX_CONFIG = 89
MSP_VTXTABLE_BAND = 137
MSP_VTXTABLE_POWERLEVEL = 138
MSP_SET_VTXTABLE_BAND = 227
MSP_SET_VTXTABLE_POWERLEVEL = 228
MSP_ADVANCED_CONFIG = 90
MSP_SET_ADVANCED_CONFIG = 91
MSP_FILTER_CONFIG = 92
MSP_SET_FILTER_CONFIG = 93
MSP_PID_ADVANCED = 94
MSP_SET_PID_ADVANCED = 95
MSP_SENSOR_CONFIG = 96
MSP_SET_SENSOR_CONFIG = 97
MSP_CAMERA_CONTROL = 98
MSP_SET_ARMING_DISABLED = 99
MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_SERVO = 103
MSP_SERVO_CONFIGURATIONS = 120
MSP_SET_SERVO_CONFIGURATION = 212
MSP_SERVO_MIX_RULES = 241
MSP_SET_SERVO_MIX_RULE = 242
MSP_MOTOR = 104
MSP_RC = 105
MSP_RAW_GPS = 106
MSP_COMP_GPS = 107
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
MSP_RC_TUNING = 111
MSP_PID = 112
MSP_MISC = 114
MSP_BOXNAMES = 116
MSP_PIDNAMES = 117
MSP_BOXIDS = 119
# --- «Корректирование в полёте» (fc/rc_adjustments.h 4.5.4)
MSP_ADJUSTMENT_RANGES = 52
MSP_SET_ADJUSTMENT_RANGE = 53
ADJUSTMENT_RANGE_COUNT = 30        # MAX_ADJUSTMENT_RANGE_COUNT
ADJUSTMENT_FUNCTION_COUNT = 34     # ADJUSTMENT_FUNCTION_COUNT - 1: значения 1…34
# --- «Питание и батарея» (voltage_ids.h / current_ids.h / voltage.h / current.h)
VOLTAGE_SENSOR_ADC_COUNT = 1                     # MAX_VOLTAGE_SENSOR_ADC: один ADC-вход (батарея)
CURRENT_SENSOR_ADC_COUNT = 2                     # ADC + виртуальный датчик (USE_VIRTUAL_CURRENT_METER)
VOLTAGE_METER_CONFIG_SUBFRAME = 5                # len(id, type, scale, divider, multiplier)
CURRENT_METER_CONFIG_SUBFRAME = 6                # len(id, type, scale u16, offset u16)
VOLTAGE_METER_ID_BATTERY_1 = 10                  # 10-19 — метры батареи
CURRENT_METER_ID_BATTERY_1 = 10
CURRENT_METER_ID_VIRTUAL_1 = 80                  # 80-89 — виртуальные метры
VOLTAGE_SENSOR_ADC_RESISTOR_DIVIDER = 0          # VOLTAGE_SENSOR_TYPE_ADC_RESISTOR_DIVIDER
CURRENT_SENSOR_VIRTUAL = 0                       # currentSensorType_e
CURRENT_SENSOR_ADC = 1
BATTERY_OK = 0                                   # batteryState_e
BATTERY_WARNING = 1
BATTERY_CRITICAL = 2
BATTERY_NOT_PRESENT = 3

# --- Приёмник и failsafe (rx/rx.h, rx/crsf.h, target/common_defaults_post.h)
RC_CHANNEL_COUNT = 16                    # CRSF_MAX_CHANNEL: CRSF всегда отдаёт 16 каналов
NON_AUX_CHANNEL_COUNT = 4                # крен/тангаж/рыск/газ
# --- Режимы (rc_modes.c / msp_box.c 4.5.4) -----------------------------------------------
# Слотов «aux» столько же, сколько modeActivationConditions[] в прошивке: 20. Пустой слот —
# это не «дырка в списке», а запись с режимом ARM и диапазоном 900/900 (startStep = endStep),
# ровно как нулевая modeActivationCondition_t на свежем борде. Именно поэтому CLI печатает
# 20 строк «aux …», а вкладка «Режимы» показывает только те, у которых start < end.
MAX_MODE_ACTIVATION_CONDITION_COUNT = 20
CHANNEL_RANGE_MIN = 900                  # CHANNEL_RANGE_MIN (rc_modes.h)
CHANNEL_RANGE_MAX = 2100                 # CHANNEL_RANGE_MAX
MODE_RANGE_STEP = 25                     # MODE_STEP_TO_CHANNEL_VALUE = 900 + 25 * step
MODELOGIC_OR = 0                         # MODELOGIC_OR / MODELOGIC_AND (modeLogic_e)
MODELOGIC_AND = 1
MAX_AUX_CHANNEL_COUNT = 14               # rx.h: MAX_SUPPORTED_RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT
RC_MAPPABLE_CHANNEL_COUNT = 8            # RX_MAPPABLE_CHANNEL_COUNT: AETR1234
PWM_PULSE_MIN = 750                      # PWM_PULSE_MIN в rx.h (те же границы у min_check/RX-диапазонов)
PWM_PULSE_MAX = 2250                     # PWM_PULSE_MAX в rx.h
MAX_RXFAIL_RANGE_STEP = (PWM_PULSE_MAX - PWM_PULSE_MIN) // 25   # MAX_RXFAIL_RANGE_STEP = 60
DSHOT_MIN_THROTTLE = 48                  # dshot.h: команды ESC — значения 1…47, дальше газ
DSHOT_CMD_ESC_INFO = 6                   # DSHOT_CMD_ESC_INFO: команда «расскажи о себе»
RX_MIN_USEC = 885
RX_LOW_USEC = 1000                       # «стик внизу» / тумблер выключен — так видит приёмник
TX_MID_USEC = 1500                       # центр стика (midrc)
TX_SWITCH_ON_USEC = 2000                 # тумблер аппаратуры во включённом положении
TX_MIN_USEC, TX_MAX_USEC = 1000, 2000    # пределы, которые отдаёт «аппаратура»
RX_SERIAL_FUNCTION = "RX_SERIAL"         # функция порта, при которой прошивка поднимает приёмник
ARMING_DISABLED_FAILSAFE = 1 << 1        # runtime_config.h: ARMING_DISABLED_FAILSAFE
ARMING_DISABLED_RX_FAILSAFE = 1 << 2     # runtime_config.h: ARMING_DISABLED_RX_FAILSAFE
RC_STICK_NAMES = ("roll", "pitch", "yaw", "throttle")
RC_AUX_NAMES = tuple(f"aux{i}" for i in range(1, RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT + 1))
# На аппаратуре каналы разной природы, и на странице /tx они показаны по-разному:
#   "switch"  — двухпозиционный тумблер (1000/2000 мкс) — по умолчанию;
#   "switch3" — трёхпозиционный тумблер (1000/1500/2000 мкс);
#   "knob"    — крутилка (потенциометр), значение плавно 1000…2000 мкс.
AUX_THREE_POSITION = ("aux3", "aux4")
AUX_KNOBS = ("aux7", "aux10")
AUX_SWITCH3_VALUES = (RX_LOW_USEC, TX_MID_USEC, TX_SWITCH_ON_USEC)
RX_MID_USEC = 1500
MAX_SUPPORTED_RC_CHANNEL_COUNT = 18      # столько каналов принимает MSP_SET_RXFAIL_CONFIG
RX_FAILSAFE_MODE_AUTO = 0                # RX_FAILSAFE_MODE_AUTO/HOLD/SET
RX_FAILSAFE_MODE_HOLD = 1
RX_FAILSAFE_MODE_SET = 2
MAX_AUTO_DETECT_CELL_COUNT = 8             # battery.h: автодетект не более 8 банок

# --- Сервоприводы (flight/servos.h, target/common_defaults_post.h)
MAX_SUPPORTED_SERVOS = 8                 # столько сервоприводов отдаёт MSP_SERVO_CONFIGURATIONS
MAX_SERVO_RULES = 16                     # MAX_SERVO_RULES = 2 * MAX_SUPPORTED_SERVOS
DEFAULT_SERVO_MIN = 1000
DEFAULT_SERVO_MAX = 2000
DEFAULT_SERVO_MIDDLE = 1500
CHANNEL_FORWARDING_DISABLED = 0xFF       # «не перенаправлять» — как в flight/servos.h
INPUT_SOURCE_COUNT = 14                  # INPUT_STABILIZED_* (4) + INPUT_RC_* (8) + INPUT_GIMBAL_* (2)
SERVO_PWM_MIN = 500                      # PWM_SERVO_MIN: минимум, который принимает CLI-команда servo
SERVO_PWM_MAX = 2500                     # PWM_SERVO_MAX
MAX_SERVO_SPEED = 255                    # MAX_SERVO_SPEED = UINT8_MAX
# Микшеры «самолётного» вида: PASSTHRU включается только на них, SERVO1..3 — на CUSTOMAIRPLANE
# (msp_box.c: MIXER_FLYING_WING/MIXER_AIRPLANE/MIXER_CUSTOM_AIRPLANE).
AIRPLANE_MIXERS = ("FLYING_WING", "AIRPLANE", "CUSTOMAIRPLANE")
MAX_SERVO_BOXES = 3                      # MAX_SERVO_BOXES

# --- Светодиодная лента (io/ledstrip.h): размеры таблиц, которые печатают `led`, `color`
# и `mode_color`. Сами таблицы в эмуляторе пустые: лента не настроена, как у квада «из коробки».
LED_STRIP_MAX_LENGTH = 32                # LED_STRIP_MAX_LENGTH: строк конфигурации ленты
LED_CONFIGURABLE_COLOR_COUNT = 16        # LED_CONFIGURABLE_COLOR_COUNT: цветов в таблице
LED_MODE_COUNT = 6                       # LED_MODE_COUNT (ORIENTATION…BARO + LED_SPECIAL/AUX)
LED_DIRECTION_COUNT = 6                  # LED_DIRECTION_COUNT (N, E, S, W, U, D)
LED_SPECIAL_COLOR_COUNT = 11             # LED_SPECIAL_COLOR_COUNT
LED_SPECIAL = LED_MODE_COUNT             # ledModeIndex_e: строка спец-цветов в mode_color
LED_AUX_CHANNEL = LED_MODE_COUNT + 1     # ledModeIndex_e: строка AUX-канала ленты
LED_COLOR_COUNT = 14                     # ARRAYLEN(hsv): цветов в заводской таблице ledstrip.c

# --- PID, регуляторы, рейты (flight/pid.c, fc/controlrate_profile.c, config/simplified_tuning.h)
# Строка «Level» в таблице PID — это те же поля pidProfile_t, что и настройки CLI
# (msp.c: PID_LEVEL.P/I/D = angle_p_gain / horizon_level_strength / horizon_limit_sticks).
PID_PROFILE_ALIASES = {
    "p_level": "angle_p_gain", "i_level": "horizon_level_strength", "d_level": "horizon_limit_sticks",
}
PID_PROFILE_COUNT = 4                    # PID_PROFILE_COUNT (target/common_pre.h)
CONTROL_RATE_PROFILE_COUNT = 4           # CONTROL_RATE_PROFILE_COUNT
PID_ITEM_COUNT = 5                       # PID_ROLL, PID_PITCH, PID_YAW, PID_LEVEL, PID_MAG
FLIGHT_DYNAMICS_INDEX_COUNT = 3          # XYZ_AXIS_COUNT: только R/P/Y участвуют в микшере
PID_GAIN_MAX = 250                       # PID_GAIN_MAX
F_GAIN_MAX = 1000                        # F_GAIN_MAX
SIMPLIFIED_TUNING_DEFAULT = 100          # SIMPLIFIED_TUNING_DEFAULT
SIMPLIFIED_TUNING_D_DEFAULT = 100        # SIMPLIFIED_TUNING_D_DEFAULT
SIMPLIFIED_TUNING_OFF = 0                # PID_SIMPLIFIED_TUNING_OFF
SIMPLIFIED_TUNING_RP = 1                 # PID_SIMPLIFIED_TUNING_RP
SIMPLIFIED_TUNING_RPY = 2                # PID_SIMPLIFIED_TUNING_RPY
PID_NAMES = "ROLL;PITCH;YAW;LEVEL;MAG;"  # flight/pid.c: const char pidNames[]
RATEPROFILE_MASK = 0x80                  # fc/rc_controls.h
# Профили батареи — появились ПОСЛЕ 4.5.4 (мастер BF, будущая 4.6): в 4.5.4 их нет ни в CLI,
# ни в MSP. Значения ниже — из мастер-исходников: количество профилей — target/common_pre.h
# (BATTERY_PROFILE_COUNT 3), бит профиля в MSP_SELECT_SETTING — msp/msp.c (BATTERYPROFILE_MASK
# 1 << 6), длина имени — sensors/battery.h, заводские пороги — pgResetFn_batteryProfiles()
# в sensors/battery.c.
BATTERY_PROFILE_COUNT = 3                # common_pre.h: BATTERY_PROFILE_COUNT
BATTERYPROFILE_MASK = 1 << 6             # msp.c: BATTERYPROFILE_MASK (0x40)
MAX_BATTERY_PROFILE_NAME_LENGTH = 8      # sensors/battery.h: MAX_BATTERY_PROFILE_NAME_LENGTH
VBAT_PROFILE_DEFAULTS = dict(            # pgResetFn_batteryProfiles()
    vbatmaxcellvoltage=430,              # VBAT_CELL_VOLTAGE_DEFAULT_MAX
    vbatmincellvoltage=330,              # VBAT_CELL_VOLTAGE_DEFAULT_MIN
    vbatwarningcellvoltage=350,
    vbatfullcellvoltage=410,
    batteryCapacity=0,
    forceBatteryCellCount=0,
    consumptionWarningPercentage=10,
)
AUTO_PROFILE_CELL_COUNT_STAY = 0         # sensors/battery.h: AUTO_PROFILE_CELL_COUNT_STAY
AUTO_PROFILE_CELL_COUNT_CHANGE = -1      # sensors/battery.h: AUTO_PROFILE_CELL_COUNT_CHANGE (в MSP уходит как 255)
DTERM_LPF1_DYN_MIN_HZ_DEFAULT = 75       # flight/pid.h
DTERM_LPF1_DYN_MAX_HZ_DEFAULT = 150
DTERM_LPF2_HZ_DEFAULT = 150
GYRO_LPF1_DYN_MIN_HZ_DEFAULT = 250       # sensors/gyro.h
GYRO_LPF1_DYN_MAX_HZ_DEFAULT = 500
GYRO_LPF2_HZ_DEFAULT = 500
DYN_LPF_MAX_HZ = 1000                    # sensors/gyro.h: DYN_LPF_MAX_HZ
LPF_MAX_HZ = 1000
RATES_TYPE_ACTUAL = 3                    # fc/controlrate_profile.h: ratesType_e
CONTROL_RATE_CONFIG_RATE_LIMIT_MAX = 1998
# Заводские значения PID-профиля (resetPidProfile в flight/pid.c, версия 4.5.4).
# Ключи — имена CLI-настроек; LEVEL/MAG и accel-лимиты в 4.5.4 доступны только через MSP.
PID_PROFILE_DEFAULTS = {
    "p_roll": 45, "i_roll": 80, "d_roll": 40, "f_roll": 120,
    "p_pitch": 47, "i_pitch": 84, "d_pitch": 46, "f_pitch": 125,
    "p_yaw": 45, "i_yaw": 80, "d_yaw": 0, "f_yaw": 120,
    "p_level": 50, "i_level": 75, "d_level": 75, "f_level": 50,
    "p_mag": 40, "i_mag": 0, "d_mag": 0, "f_mag": 0,
    "d_min_roll": 30, "d_min_pitch": 34, "d_min_yaw": 0,
    "d_max_gain": 37, "d_max_advance": 20,
    "anti_gravity_gain": 80, "iterm_rotation": 0,
    "iterm_relax": 1, "iterm_relax_type": 1, "iterm_relax_cutoff": 15,
    "abs_control_gain": 0, "throttle_boost": 5, "acro_trainer_angle_limit": 20,
    "angle_limit": 60, "feedforward_transition": 0,
    "feedforward_averaging": 0, "feedforward_smooth_factor": 25, "feedforward_boost": 15,
    "feedforward_max_rate_limit": 90, "feedforward_jitter_factor": 7,
    "vbat_sag_compensation": 0, "thrust_linear": 0,
    "tpa_mode": 1, "tpa_rate": 65, "tpa_breakpoint": 1350,
    "motor_output_limit": 100, "auto_profile_cell_count": AUTO_PROFILE_CELL_COUNT_STAY,
    "dyn_idle_min_rpm": 0, "use_integrated_yaw": 0, "integrated_yaw_relax": 200,
    "acc_limit": 0, "acc_limit_yaw": 0,          # в CLI 4.5.4 rateAccelLimit = acc_limit
    "simplified_pids_mode": SIMPLIFIED_TUNING_RPY,       # в 4.5.4 слайдеры включены по умолчанию
    "simplified_master_multiplier": SIMPLIFIED_TUNING_DEFAULT,
    "simplified_pitch_d_gain": SIMPLIFIED_TUNING_DEFAULT,   # это и есть simplified_roll_pitch_ratio
    "simplified_i_gain": SIMPLIFIED_TUNING_DEFAULT,
    "simplified_d_gain": SIMPLIFIED_TUNING_D_DEFAULT,
    "simplified_pi_gain": SIMPLIFIED_TUNING_DEFAULT,
    "simplified_dmax_gain": SIMPLIFIED_TUNING_D_DEFAULT,    # это и есть simplified_dmin_ratio
    "simplified_feedforward_gain": SIMPLIFIED_TUNING_DEFAULT,
    "simplified_pitch_pi_gain": SIMPLIFIED_TUNING_DEFAULT,
    "simplified_dterm_filter": 1, "simplified_dterm_filter_multiplier": SIMPLIFIED_TUNING_DEFAULT,
    "dterm_lpf1_dyn_min_hz": DTERM_LPF1_DYN_MIN_HZ_DEFAULT,
    "dterm_lpf1_dyn_max_hz": DTERM_LPF1_DYN_MAX_HZ_DEFAULT,
    "dterm_lpf1_dyn_expo": 5,
    "dterm_lpf1_type": 0, "dterm_lpf2_type": 0,
    "dterm_lpf1_static_hz": DTERM_LPF1_DYN_MIN_HZ_DEFAULT,
    "dterm_lpf2_static_hz": DTERM_LPF2_HZ_DEFAULT,
    "dterm_notch_hz": 0, "dterm_notch_cutoff": 0,
    "yaw_lowpass_hz": 100,
}
# Заводские значения профиля рейтов (pgResetFn_controlRateProfiles в fc/controlrate_profile.c)
RATE_PROFILE_DEFAULTS = {
    "thr_mid": 50, "thr_expo": 0, "rates_type": RATES_TYPE_ACTUAL,
    "roll_rc_rate": 7, "pitch_rc_rate": 7, "yaw_rc_rate": 7,
    "roll_expo": 0, "pitch_expo": 0, "yaw_expo": 0,
    "roll_srate": 67, "pitch_srate": 67, "yaw_srate": 67,
    "throttle_limit_type": 0, "throttle_limit_percent": 100,
    "roll_rate_limit": CONTROL_RATE_CONFIG_RATE_LIMIT_MAX,
    "pitch_rate_limit": CONTROL_RATE_CONFIG_RATE_LIMIT_MAX,
    "yaw_rate_limit": CONTROL_RATE_CONFIG_RATE_LIMIT_MAX,
    "quickrates_rc_expo": 0,
}

# --- Build-опции прошивки (msp/msp_build_info.h + FIRMWARE_BUILD_OPTIONS в Configurator)
# Configurator показывает вкладку «Сервоприводы» только если прошивка собрана с USE_SERVOS
# (или USE_WING), а вкладку «GPS» — только с USE_GPS (src/components/sidebar/sidebar_items.js).
# Список подобран так, чтобы все уже видимые вкладки и функции остались на месте.
BUILD_OPTIONS = (
    ("USE_SERIALRX_CRSF", 4097),     # приёмник CRSF (serialrx_provider = CRSF)
    ("USE_DSHOT", 8231),             # протокол моторов DSHOT600
    ("USE_DASHBOARD", 16408),        # функция DISPLAY
    ("USE_GPS", 16412),              # вкладка «GPS» (sidebar_items.js: buildOptions: ["USE_GPS"])
    ("USE_LED_STRIP", 16413),        # функция и вкладка «Светодиодная лента»
    ("USE_OSD_SD", 16416),           # функция OSD и вкладка OSD
    ("USE_SERVOS", 16420),           # SERVO_TILT / CHANNEL_FORWARDING и вкладка «Сервоприводы»
    ("USE_VTX", 16421),              # вкладка «Видеопередатчик»
    ("USE_SOFTSERIAL", 16423),       # функция SOFTSERIAL
    ("USE_RANGEFINDER", 16429),      # функция SONAR
)

MSP_VOLTAGE_METERS = 128
MSP_CURRENT_METERS = 129
MSP_BATTERY_STATE = 130
MSP_MOTOR_CONFIG = 131
MSP_GPS_CONFIG = 132
MSP_GPS_RESCUE = 135
MSP_MOTOR_TELEMETRY = 139
MSP_MOTOR_3D_CONFIG = 124
MSP_SET_MOTOR_3D_CONFIG = 217
MSP_STATUS_EX = 150
MSP_UID = 160
MSP_BEEPER_CONFIG = 184
MSP_SET_BEEPER_CONFIG = 185
MSP_SET_RAW_RC = 200
MSP_SET_PID = 202
MSP_SET_RC_TUNING = 204
MSP_RC_DEADBAND = 125
MSP_SET_RC_DEADBAND = 218
MSP_SIMPLIFIED_TUNING = 140
MSP_SET_SIMPLIFIED_TUNING = 141
MSP_CALCULATE_SIMPLIFIED_PID = 142
MSP_CALCULATE_SIMPLIFIED_GYRO = 143
MSP_CALCULATE_SIMPLIFIED_DTERM = 144
MSP_VALIDATE_SIMPLIFIED_TUNING = 145
MSP_COPY_PROFILE = 183
MSP_SELECT_SETTING = 210
MSP_SET_RESET_CURR_PID = 219
MSP_ACC_CALIBRATION = 205
MSP_MAG_CALIBRATION = 206
MSP_SET_MISC = 207
MSP_RESET_CONF = 208
MSP_SET_MOTOR = 214
MSP_SET_MOTOR_CONFIG = 222
MSP_SET_GPS_CONFIG = 223
MSP_SET_GPS_RESCUE = 225
MSP_SENSOR_ALIGNMENT = 126        # MSP_SENSOR_ALIGNMENT
MSP_COMPASS_CONFIG = 133          # MSP_COMPASS_CONFIG
MSP_SET_SENSOR_ALIGNMENT = 220    # MSP_SET_SENSOR_ALIGNMENT
MSP_SET_COMPASS_CONFIG = 224      # MSP_SET_COMPASS_CONFIG
MSP_SET_ACC_TRIM = 239
MSP_ACC_TRIM = 240
MSP_SET_PASSTHROUGH = 245
MSP_SET_RTC = 246
MSP_EEPROM_WRITE = 250
MSP_DEBUG = 254
MSP2_COMMON_SERIAL_CONFIG = 4105
MSP2_COMMON_SET_SERIAL_CONFIG = 4106
MSP2_MOTOR_OUTPUT_REORDERING = 12289
MSP2_SET_MOTOR_OUTPUT_REORDERING = 12290
MSP2_SEND_DSHOT_COMMAND = 12291
MSP2_GET_OSD_WARNINGS = 12293
MSP2_GET_TEXT = 12294
MSP2_SET_TEXT = 12295
MSP2_SENSOR_CONFIG_ACTIVE = 12298

# устаревшие/нестандартные коды, которые Configurator всё ещё может запрашивать
MSP_LOOP_TIME = 73          # DEPRECATED в 4.5, но запрашивается старыми сборками
MSP_SENSOR_STATUS = 151     # не входит в msp_protocol.h, но встречается в UI

MSP2_BATTERY_PROFILE = 12302
MSP2_SET_BATTERY_PROFILE = 12303
MSP2_GET_VTX_DEVICE_STATUS = 12292  # 0x3004
MSP2_CLI_COMMAND = 12306
MSP2_CLI_SETTING = 12304
MSP2_CLI_SETTING_INFO = 12305
MSP2_GYRO_SENSOR_ACTIVE = 12301
MSP2_MCU_INFO = 12300
MSP_GPS_SV_INFO = 164

TEXT_PILOT_NAME = 1
TEXT_CRAFT_NAME = 2
TEXT_PID_PROFILE_NAME = 3
TEXT_RATE_PROFILE_NAME = 4
TEXT_BUILDKEY = 5
TEXT_BATTERY_PROFILE_NAME = 11

# серийные функции — биты должны совпадать с serialPortFunction_e прошивки
SERIAL_FUNCTIONS = {
    "MSP": 1 << 0,
    "GPS": 1 << 1,
    "TELEMETRY_FRSKY": 1 << 2,
    "TELEMETRY_HOTT": 1 << 3,
    "TELEMETRY_LTM": 1 << 4,
    "TELEMETRY_SMARTPORT": 1 << 5,
    "RX_SERIAL": 1 << 6,
    "BLACKBOX": 1 << 7,
    "TELEMETRY_MAVLINK": 1 << 9,
    "ESC_SENSOR": 1 << 10,
    "TBS_SMARTAUDIO": 1 << 11,
    "TELEMETRY_IBUS": 1 << 12,
    "IRC_TRAMP": 1 << 13,
    "RUNCAM_DEVICE_CONTROL": 1 << 14,
    "LIDAR_TF": 1 << 15,
    "FRSKY_OSD": 1 << 16,
    "VTX_MSP": 1 << 17,
}
BAUD_RATES = ["AUTO", "9600", "19200", "38400", "57600", "115200", "230400", "250000", "400000", "460800",
              "500000", "921600", "1000000", "1500000", "2000000", "2470000"]

# секции dump() — как в прошивке
SECTION_ORDER = ["HARDWARE_VALUE", "MASTER_VALUE", "PROFILE_VALUE", "PROFILE_RATE_VALUE", "PROFILE_BATTERY_VALUE"]

# Биты активных сенсоров в MSP_STATUS/STATUS_EX. Порядок именно такой —
# прошивка упаковывает их в msp.c:
#   sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 |
#   sensors(SENSOR_GPS) << 3 | sensors(SENSOR_RANGEFINDER) << 4 | sensors(SENSOR_GYRO) << 5
# (а не тот, что в sensors.h) — иначе Configurator не увидит GPS/mag и не опросит их.
SENSOR_ACC = 1 << 0
SENSOR_BARO = 1 << 1
SENSOR_MAG = 1 << 2
SENSOR_GPS = 1 << 3
SENSOR_RANGEFINDER = 1 << 4
SENSOR_GYRO = 1 << 5

# ARMING_DISABLED_ARM_SWITCH — старший бит (1 << 25) в runtime_config.h;
# ARMING_DISABLE_FLAGS_COUNT = LOG2(ARM_SWITCH) + 1 = 26
ARMING_DISABLED_ARM_SWITCH = 1 << 25
# ARMING_DISABLED_MSP (runtime_config.h) — вкладка «Моторы» поднимает его на время теста,
# отправляя MSP_SET_ARMING_DISABLED (99): пока флаг стоит, борт взвести нельзя.
ARMING_DISABLED_MSP = 1 << 17

# motor.h: MAX_SUPPORTED_MOTORS — MSP_MOTOR всегда отдаёт ровно столько значений.
MAX_SUPPORTED_MOTORS = 8
ARMING_DISABLE_FLAGS_COUNT = 26

# Значения железа датчиков: 0 = AUTO, 1 = NONE (acceleration.h / barometer.h / compass.h).
# У дальномера инверсия: 0 = NONE, 1 = AUTO (rangefinder.h).
SENSOR_AUTO = 0

# Длины строковых параметров (MODE_STRING, cli/settings.c 4.5.4): min/max в символах.
MAX_NAME_LENGTH = 16                 # config.h: имена дрона и пилота
MAX_RATE_PROFILE_NAME_LENGTH = 8     # controlrate_profile.h
MAX_PROFILE_NAME_LENGTH = 8           # flight/pid.h
OSD_PROFILE_NAME_LENGTH = 16          # osd/osd.h
MAX_BOX_USER_NAME_LENGTH = 8          # pg/boxnames.h
NMEA_CUSTOM_COMMANDS_MAX_LENGTH = 64  # pg/gps.h
# заводские строки: то, что прошивка кладёт в PG_RESET_TEMPLATE (pilot.c: "EMUF-QUAD"/"EMUF-PILOT")
STRING_DEFAULTS = {
    "craft_name": "EMU-QUAD",
    "pilot_name": "VIRTUAL PILOT",
}
STRING_LENGTHS = {
    "craft_name": (1, MAX_NAME_LENGTH),
    "pilot_name": (1, MAX_NAME_LENGTH),
    "profile_name": (1, MAX_PROFILE_NAME_LENGTH),
    "rateprofile_name": (1, MAX_RATE_PROFILE_NAME_LENGTH),
    "osd_profile_1_name": (1, OSD_PROFILE_NAME_LENGTH),
    "osd_profile_2_name": (1, OSD_PROFILE_NAME_LENGTH),
    "osd_profile_3_name": (1, OSD_PROFILE_NAME_LENGTH),
    "box_user_1_name": (1, MAX_BOX_USER_NAME_LENGTH),
    "box_user_2_name": (1, MAX_BOX_USER_NAME_LENGTH),
    "box_user_3_name": (1, MAX_BOX_USER_NAME_LENGTH),
    "box_user_4_name": (1, MAX_BOX_USER_NAME_LENGTH),
    "gps_nmea_custom_commands": (1, NMEA_CUSTOM_COMMANDS_MAX_LENGTH),
}
SENSOR_NONE = 1
RANGEFINDER_NONE = 0
RANGEFINDER_AUTO = 1

# Пары «запись -> чтение» из MSPHelper.js: вкладки шлют MSP_SET_*, а читают MSP_*.
# Формат payload у таких пар совпадает, поэтому конфигурацию можно хранить сквозным образом.
# Коды MSP_SET_*, которые разбираются «по смыслу» (пишут в настройки/профили), а не отдаются
# эхом записанного payload'а. Список используется и в handle(), и при чистке старых «флешей».
SEMANTIC_SET_CODES = (11, 33, 35, 45, 51, 53, 62, 65, 76, 85, 91, 93, 95, 202, 204, 218, 220, 222,
                      223, 224, 225, 190, 185)

# Пары «запись → чтение», которых нет в WRITE_TO_READ, но ответ на чтение строится из состояния FC.
# Если в старом «флеше» (или в этой же сессии) лежало эхо записанного payload'а, оно глушило
# живые значения: вкладка «Конфигурация» показывала прежние фичи, и правки «не сохранялись».
# Пример из жизни: SERIAL_TILT/SOFTSERIAL включались, но MSP_FEATURE_CONFIG отдавал старое эхо.
STATE_PAIRS_EXTRA = {
    37: 36,           # MSP_SET_FEATURE_CONFIG -> MSP_FEATURE_CONFIG
    78: 77,           # MSP_SET_RXFAIL_CONFIG -> MSP_RXFAIL_CONFIG
    97: 96,           # MSP_SET_SENSOR_CONFIG -> MSP_SENSOR_CONFIG
    41: 40,           # MSP_SET_CURRENT_METER_CONFIG -> MSP_CURRENT_METER_CONFIG
    57: 56,           # MSP_SET_VOLTAGE_METER_CONFIG -> MSP_VOLTAGE_METER_CONFIG
    55: 54,           # MSP_SET_CF_SERIAL_CONFIG -> MSP_CF_SERIAL_CONFIG
    89: 88,           # MSP_SET_VTX_CONFIG -> MSP_VTX_CONFIG
    141: 140,         # MSP_SET_SIMPLIFIED_TUNING -> MSP_SIMPLIFIED_TUNING
    212: 120,         # MSP_SET_SERVO_CONFIGURATION -> MSP_SERVO_CONFIGURATIONS
    242: 241,         # MSP_SET_SERVO_MIX_RULE -> MSP_SERVO_MIX_RULES
    239: 240,         # MSP_SET_ACC_TRIM -> MSP_ACC_TRIM
    12295: 0x3006,    # MSP2_SET_TEXT -> MSP2_GET_TEXT (имена дрона, пилота, профилей)
    4106: 4105,       # MSP2_COMMON_SET_SERIAL_CONFIG -> MSP2_COMMON_SERIAL_CONFIG
}


def state_based_read_codes() -> set:
    """Коды чтения, ответ на которые FC строит сам из своего состояния.

    Для них эхо записанного payload'а игнорируется (иначе правки «не видны» до перезапуска),
    а сами записи идут через смысловые обработчики.
    """
    codes = {WRITE_TO_READ[code] for code in SEMANTIC_SET_CODES if code in WRITE_TO_READ}
    codes.update(STATE_PAIRS_EXTRA.values())
    return codes


WRITE_TO_READ = {
    11: 10,    # MSP_SET_NAME -> MSP_NAME (обрабатывается отдельно)
    33: 32,    # MSP_SET_BATTERY_CONFIG
    35: 34,    # MSP_SET_MODE_RANGE -> MSP_MODE_RANGES
    39: 38,    # MSP_SET_BOARD_ALIGNMENT_CONFIG
    43: 42,    # MSP_SET_MIXER_CONFIG
    45: 44,    # MSP_SET_RX_CONFIG
    51: 50,    # MSP_SET_RSSI_CONFIG
    53: 52,    # MSP_SET_ADJUSTMENT_RANGE -> MSP_ADJUSTMENT_RANGES
    62: 61,    # MSP_SET_ARMING_CONFIG
    65: 64,    # MSP_SET_RX_MAP (обрабатывается отдельно)
    76: 75,    # MSP_SET_FAILSAFE_CONFIG
    85: 84,    # MSP_SET_OSD_CONFIG
    91: 90,    # MSP_SET_ADVANCED_CONFIG
    93: 92,    # MSP_SET_FILTER_CONFIG
    95: 94,    # MSP_SET_PID_ADVANCED
    185: 184,  # MSP_SET_BEEPER_CONFIG
    190: 189,  # MSP_SET_OSD_CANVAS -> MSP_OSD_CANVAS
    202: 112,  # MSP_SET_PID
    204: 111,  # MSP_SET_RC_TUNING
    217: 124,  # MSP_SET_MOTOR_3D_CONFIG
    218: 125,  # MSP_SET_RC_DEADBAND
    220: 126,  # MSP_SET_SENSOR_ALIGNMENT
    222: 131,  # MSP_SET_MOTOR_CONFIG
    223: 132,  # MSP_SET_GPS_CONFIG
    224: 133,  # MSP_SET_COMPASS_CONFIG
    225: 135,  # MSP_SET_GPS_RESCUE
}

# --------------------------------------------------------------- VTX (вкладка «ВИДЕОПЕРЕДАТЧИК»)
# Типы устройств (drivers/vtx_common.h) и флаги состояния.
VTXDEV_RTC6705 = 1
VTXDEV_SMARTAUDIO = 3
VTXDEV_TRAMP = 4
VTXDEV_MSP = 5
VTXDEV_UNKNOWN = 0xFF
VTX_STATUS_PIT_MODE = 1 << 0
VTX_STATUS_LOCKED = 1 << 1
# Размеры таблицы (drivers/vtx_table.h)
VTX_TABLE_MAX_BANDS = 8
VTX_TABLE_MAX_CHANNELS = 8
VTX_TABLE_MAX_POWER_LEVELS = 8
VTX_TABLE_BAND_NAME_LENGTH = 8
VTX_TABLE_POWER_LABEL_LENGTH = 3
# Значения самого устройства SmartAudio (io/vtx_smartaudio.c)
SA_VERSION = 3            # версия 2.1
SA_MODE_UNLOCKED = 16     # mode & 16 → передатчик не заблокирован
VTX_COMMON_MSP_BANDCHAN_CHKVAL = (7 << 3) + 7   # 63

# ------------------------------------------------------------ OSD (вкладка «OSD»)
# Счётчики строго как в прошивке 4.5.4 (src/main/osd/osd.h): именно их ждёт Configurator —
# он читает их из ответа и сверяет со своим списком полей.
OSD_ITEM_COUNT = 80               # OSD_ITEM_COUNT (osd_items_e)
OSD_STAT_COUNT = 31               # OSD_STAT_COUNT (osd_stats_e)
OSD_TIMER_COUNT = 2               # OSD_TIMER_COUNT
OSD_WARNING_COUNT = 19            # OSD_WARNING_COUNT (osdWarningsFlags_e)
OSD_PROFILE_COUNT = 3             # USE_OSD_PROFILES
OSD_VISIBLE = 0x0800              # VISIBLE: бит 11 = профиль 1, 12 = профиль 2, 13 = профиль 3
OSD_VARIANTS = 0xC000             # VARIANTS (два старших бита)
OSD_CANVAS_COLS = 30              # OSD_SD_COLS = VIDEO_COLUMNS_SD
OSD_CANVAS_ROWS = 16              # OSD_SD_ROWS = VIDEO_LINES_PAL
# Флаги состояния OSD (msp.c: MSP_OSD_CONFIG)
OSD_FLAG_FEATURE = 1 << 0         # OSD_FLAGS_OSD_FEATURE
OSD_FLAG_SLAVE = 1 << 1           # OSD_FLAGS_OSD_SLAVE
OSD_FLAG_FRSKYOSD = 1 << 3        # OSD_FLAGS_OSD_HARDWARE_FRSKYOSD
OSD_FLAG_MAX7456 = 1 << 4         # OSD_FLAGS_OSD_HARDWARE_MAX_7456
OSD_FLAG_DETECTED = 1 << 5        # OSD_FLAGS_OSD_DEVICE_DETECTED
OSD_FLAG_MSP_DEVICE = 1 << 6      # OSD_FLAGS_OSD_MSP_DEVICE
# Порядок элементов — как в osd_items_e. Нужен, чтобы раскладка по умолчанию («что видно
# на экране") ссылалась на правильные номера, как это делает прошивка.
OSD_ITEMS = (
    "OSD_RSSI_VALUE", "OSD_MAIN_BATT_VOLTAGE", "OSD_CROSSHAIRS", "OSD_ARTIFICIAL_HORIZON",
    "OSD_HORIZON_SIDEBARS", "OSD_ITEM_TIMER_1", "OSD_ITEM_TIMER_2", "OSD_FLYMODE",
    "OSD_CRAFT_NAME", "OSD_THROTTLE_POS", "OSD_VTX_CHANNEL", "OSD_CURRENT_DRAW",
    "OSD_MAH_DRAWN", "OSD_GPS_SPEED", "OSD_GPS_SATS", "OSD_ALTITUDE",
    "OSD_ROLL_PIDS", "OSD_PITCH_PIDS", "OSD_YAW_PIDS", "OSD_POWER",
    "OSD_PIDRATE_PROFILE", "OSD_WARNINGS", "OSD_AVG_CELL_VOLTAGE", "OSD_GPS_LON",
    "OSD_GPS_LAT", "OSD_DEBUG", "OSD_PITCH_ANGLE", "OSD_ROLL_ANGLE",
    "OSD_MAIN_BATT_USAGE", "OSD_DISARMED", "OSD_HOME_DIR", "OSD_HOME_DIST",
    "OSD_NUMERICAL_HEADING", "OSD_NUMERICAL_VARIO", "OSD_COMPASS_BAR", "OSD_ESC_TMP",
    "OSD_ESC_RPM", "OSD_REMAINING_TIME_ESTIMATE", "OSD_RTC_DATETIME", "OSD_ADJUSTMENT_RANGE",
    "OSD_CORE_TEMPERATURE", "OSD_ANTI_GRAVITY", "OSD_G_FORCE", "OSD_MOTOR_DIAG",
    "OSD_LOG_STATUS", "OSD_FLIP_ARROW", "OSD_LINK_QUALITY", "OSD_FLIGHT_DIST",
    "OSD_STICK_OVERLAY_LEFT", "OSD_STICK_OVERLAY_RIGHT", "OSD_PILOT_NAME", "OSD_ESC_RPM_FREQ",
    "OSD_RATE_PROFILE_NAME", "OSD_PID_PROFILE_NAME", "OSD_PROFILE_NAME", "OSD_RSSI_DBM_VALUE",
    "OSD_RC_CHANNELS", "OSD_CAMERA_FRAME", "OSD_EFFICIENCY", "OSD_TOTAL_FLIGHTS",
    "OSD_UP_DOWN_REFERENCE", "OSD_TX_UPLINK_POWER", "OSD_WATT_HOURS_DRAWN", "OSD_AUX_VALUE",
    "OSD_READY_MODE", "OSD_RSNR_VALUE", "OSD_SYS_GOGGLE_VOLTAGE", "OSD_SYS_VTX_VOLTAGE",
    "OSD_SYS_BITRATE", "OSD_SYS_DELAY", "OSD_SYS_DISTANCE", "OSD_SYS_LQ",
    "OSD_SYS_GOGGLE_DVR", "OSD_SYS_VTX_DVR", "OSD_SYS_WARNINGS", "OSD_SYS_VTX_TEMP",
    "OSD_SYS_FAN_SPEED", "OSD_GPS_LAP_TIME_CURRENT", "OSD_GPS_LAP_TIME_PREVIOUS", "OSD_GPS_LAP_TIME_BEST3",
)
# Статистика «после полёта», включённая с завода (pgResetFn_osdConfig без USE_RC_STATS).
OSD_STATS_DEFAULT = (2, 3, 5, 8, 9, 10, 12, 13)
# Предупреждения: включены все, кроме RSSI, LQ, RSSI dBm, over-cap и RSNR (так в прошивке).
OSD_WARNINGS_OFF = (13, 14, 15, 16, 17)
# Раскладка по умолчанию: что видно на экране у настроенного квада (x, y), профиль 1.
OSD_DEFAULT_LAYOUT = {
    "OSD_MAIN_BATT_VOLTAGE": (1, 1), "OSD_WARNINGS": (13, 1), "OSD_ITEM_TIMER_1": (25, 1),
    "OSD_THROTTLE_POS": (1, 8), "OSD_ALTITUDE": (1, 14), "OSD_GPS_SATS": (25, 14),
    "OSD_CRAFT_NAME": (13, 15), "OSD_RSSI_VALUE": (1, 15),
}

# Заводские фичи: борт с GPS-модулем на UART1 (см. serial_ports и маску датчиков в MSP_STATUS),
# поэтому GPS включён — вкладка «GPS» показывает живой фикс, как на настроенном кваде.
FEATURE_DEFAULTS = {"OSD", "AIRMODE", "RX_SERIAL", "TELEMETRY", "ANTI_GRAVITY", "GPS"}

BEEPER_NAMES = [
    "GYRO_CALIBRATED", "RX_LOST", "RX_LOST_LANDING", "DISARMING", "ARMING", "ARMING_GPS_FIX",
    "BAT_CRIT_LOW", "BAT_LOW", "GPS_STATUS", "RX_SET", "ACC_CALIBRATION", "ACC_CALIBRATION_FAIL",
    "READY_BEEP", "MULTI_BEEPS", "DISARM_REPEAT", "ARMED", "SYSTEM_INIT", "USB", "BLACKBOX_ERASE",
    "CRASH_FLIP", "CAM_CONNECTION_OPEN", "CAM_CONNECTION_CLOSE", "RC_SMOOTHING_INIT_FAIL",
]


class VirtualFC:
    # ------------------------------------------------------------------ init
    def __init__(
        self,
        data_path: Path | str | None = None,
        fc_version: str = "4.5.4",
        api_version: str = "1.46.0",
        eeprom_path: Path | str | None = None,
        log=None,
    ):
        path = Path(data_path) if data_path else DATA_PATH
        self.data = json.loads(path.read_text())
        self.fc_version = fc_version
        self.api_version = api_version
        self.msp_protocol_version = 0
        self._log = log or (lambda _msg: None)
        # «флеш» эмулятора: обычный JSON-файл. None — живём только в памяти.
        self.eeprom_path = Path(eeprom_path) if eeprom_path else None
        self.eeprom_loaded = False
        # «Чёрный ящик»: накопитель с тестовыми пролётами (MSP_DATAFLASH_SUMMARY/READ).
        # Данные лежат рядом с «флешем» настроек и наполняются эмулятором пролётов.
        flash_path = (self.eeprom_path.with_name("blackbox_flash.bin")
                      if self.eeprom_path else None)
        self.blackbox = bb_scenarios.BlackboxStore(flash_path, log=self._log)

        self.settings_list = []
        self.settings_by_name = {}
        self.settings = {}
        self._load_settings()

        # идентичность платы
        self.board_identifier = "S405"
        self.target_name = "STM32F405"
        self.board_name = "VIRTUALFC"
        self.manufacturer_id = "BFR"
        self.mcu_type_id = 1  # F40X
        self.mcu = "F40X"
        self.signature = b"\x00" * 32
        # Возможности платы (MSP_BOARD_INFO): прошивка выставляет их из msp.c —
        #   bit 0 TARGET_HAS_VCP        (USE_VCP)
        #   bit 1 TARGET_HAS_SOFTSERIAL (USE_SOFTSERIAL)
        # Configurator по биту SOFTSERIAL понимает, что сборка вообще умеет софт-серийные
        # порты (fc.js: boardHasSoftSerial -> unreportedSoftSerialIdentifiers).
        self.target_capabilities = 0x01 | 0x02
        self.configuration_state = 1  # CONFIGURED
        self.configuration_state_name = "CONFIGURED"
        self.config_rev = "0"
        self.eeprom_size = 4166
        self.git_revision = "emu0bf"
        self.build_date = "Sep 24 2026"
        self.build_time = "12:00:00"

        # изменяемое состояние
        self.features = set(FEATURE_DEFAULTS)
        # Биты фич: из bf_data.json ("feature_bits" — сгенерировано из featureNames[] прошивки).
        # ВАЖНО: нумеровать по порядку непустых имён нельзя — в прошивке биты идут с пропусками,
        # и Configurator (Features.js) ждёт именно «дырявую» нумерацию: SOFTSERIAL = 1 << 6.
        bits = self.data.get("feature_bits") or {}
        self.feature_bits_by_name = {name: (1 << bit) for name, bit in bits.items() if isinstance(bit, int)}
        self.feature_bits_by_name = {name: bit for name, bit in self.feature_bits_by_name.items() if bit}
        if not self.feature_bits_by_name:  # страховка, если данные не сгенерированы
            self.feature_bits_by_name = {
                "RX_PPM": 1 << 0, "INFLIGHT_ACC_CAL": 1 << 2, "RX_SERIAL": 1 << 3, "MOTOR_STOP": 1 << 4,
                "SERVO_TILT": 1 << 5, "SOFTSERIAL": 1 << 6, "GPS": 1 << 7, "RANGEFINDER": 1 << 9,
                "TELEMETRY": 1 << 10, "3D": 1 << 12, "RX_PARALLEL_PWM": 1 << 13, "RX_MSP": 1 << 14,
                "RSSI_ADC": 1 << 15, "LED_STRIP": 1 << 16, "DASHBOARD": 1 << 17, "OSD": 1 << 18,
                "CHANNEL_FORWARDING": 1 << 20, "TRANSPONDER": 1 << 21, "AIRMODE": 1 << 22,
                "RX_SPI": 1 << 25, "ESC_SENSOR": 1 << 27, "ANTI_GRAVITY": 1 << 28,
            }
        self.beepers_disabled: set[str] = set()   # beeper_off_flags: поля нет в reset-шаблоне
        # (pg/beeper.c 4.5.4) → по умолчанию 0, то есть выключенных сигналов нет
        self.beeper_names = [b for b in BEEPER_NAMES]
        # Идентификаторы — как в serialPortIdentifier_e прошивки:
        # UART1..UART5 = 0..4, USB VCP = 20, SOFTSERIAL1/2 = 30/31. Именно их ждёт
        # Configurator (composables/ports/usePortsState.js) при отрисовке вкладки «Порты».
        # Порядок строк — тоже как в прошивке: массив serialPortIdentifiers[]
        # (io/serial.c 4.5.4) начинается с USB VCP, и этот же порядок уходит в
        # MSP_SERIAL_CONFIG / MSP2_COMMON_SERIAL_CONFIG и в CLI-дамп `serial`
        # («serial 20 1 115200 ...» перед «serial 0 ...»). Configurator строки не
        # сортирует — рисует как пришли, поэтому USB VCP в таблице идёт первым.
        # Скорости — заводские, как pgResetFn_serialConfig() в io/serial.c 4.5.4:
        # MSP 115200, датчики/GPS 57600, телеметрия AUTO, «чёрный ящик» 115200
        # (baudRate_e: AUTO=0, 9600=1, ..., 57600=4, 115200=5).
        def _port(identifier, functions):
            return {"identifier": identifier, "functions": functions, "msp_baudrate": 5,
                    "gps_baudrate": 4, "telemetry_baudrate": 0, "blackbox_baudrate": 5}

        self.serial_ports = [
            _port(20, ["MSP"]),              # USB VCP
            _port(0, ["GPS"]),               # UART1 — GPS-модуль
            _port(1, ["RX_SERIAL"]),         # UART2 — приёмник
            _port(2, ["ESC_SENSOR"]),        # UART3 — ESC-телеметрия
            _port(3, ["TBS_SMARTAUDIO"]),    # UART4 — VTX
            _port(4, []),                    # UART5 — свободен
        ]
        self.boxes = self.data.get("boxes") or []
        if not self.boxes:
            self.boxes = [
                {"box_id": "BOXARM", "name": "ARM", "permanent_id": 0},
                {"box_id": "BOXANGLE", "name": "ANGLE", "permanent_id": 1},
                {"box_id": "BOXHORIZON", "name": "HORIZON", "permanent_id": 2},
                {"box_id": "BOXANTIGRAVITY", "name": "ANTI GRAVITY", "permanent_id": 4},
                {"box_id": "BOXMAG", "name": "MAG", "permanent_id": 5},
                {"box_id": "BOXHEADFREE", "name": "HEADFREE", "permanent_id": 6},
                {"box_id": "BOXHEADADJ", "name": "HEADADJ", "permanent_id": 7},
                {"box_id": "BOXCAMSTAB", "name": "CAMSTAB", "permanent_id": 8},
                {"box_id": "BOXGPSHOME", "name": "GPS HOME", "permanent_id": 11},
                {"box_id": "BOXGPSHOLD", "name": "GPS HOLD", "permanent_id": 12},
                {"box_id": "BOXFAILSAFE", "name": "FAILSAFE", "permanent_id": 15},
                {"box_id": "BOXAIRMODE", "name": "AIRMODE", "permanent_id": 17},
                {"box_id": "BOX3DDISABLE", "name": "3D DISABLE", "permanent_id": 22},
                {"box_id": "BOXFPVANGLEMIX", "name": "FPV ANGLE MIX", "permanent_id": 23},
                {"box_id": "BOXBLACKBOXERASE", "name": "BLACKBOX ERASE", "permanent_id": 24},
                {"box_id": "BOXCAMERA1", "name": "CAMERA CONTROL 1", "permanent_id": 25},
                {"box_id": "BOXFLIPOVERAFTERCRASH", "name": "FLIP OVER AFTER CRASH", "permanent_id": 26},
                {"box_id": "BOXPREARM", "name": "PREARM", "permanent_id": 27},
                {"box_id": "BOXPARALYZE", "name": "PARALYZE", "permanent_id": 28},
                {"box_id": "BOXGPSRESCUE", "name": "GPS RESCUE", "permanent_id": 29},
            ]
        self.box_name_by_permanent_id = {b["permanent_id"]: b["name"] for b in self.boxes}
        # «стандартные» диапазоны AUX, как на свежей прошивке после дефолтов: все 20 слотов
        # пустые (режим ARM, диапазон 900/900) — вкладка «Режимы» показывает их как «нет строк»,
        # а CLI печатает 20 строк «aux i 0 0 900 900 0 0».
        self.mode_ranges = self._default_mode_ranges()
        self.rx_map = "AETR1234"
        # Виртуальная аппаратура: 16 каналов в порядке A/E/R/T, как rcData[] в прошивке.
        # Стики в центре, газ внизу, тумблеры AUX выключены. Это состояние НЕ пишется во
        # «флеш»: настоящая аппаратура тоже не хранится в FC — после перезагрузки стики
        # снова в центре (см. tools/tx.py и страницу http://127.0.0.1:8088/tx).
        self.rc_channels = self._default_rc_channels()
        # «В эфире» ли аппаратура (виртуальный USB-пульт). Как и настоящий пульт, это не
        # конфигурация FC: во «флеш» не пишется и после ребута сбрасывается.
        self.rx_link_connected = True
        self.rc_source = "—"           # кто последним двигал стики (страница /tx, tools/tx.py)
        # Бинд («связана ли аппаратура с приёмником»). Как и каналы — это состояние «в эфире»,
        # а не конфигурация FC: во «флеш» не пишется, после ребута сбрасывается в «связано»
        # (виртуальный FC «включён вместе с уже связанной аппаратурой»).
        #   "bound"   — связь установлена (кадры идут);
        #   "binding" — идёт поиск/связывание, через rx_bind_seconds станет "bound";
        #   "none"    — не связана: приёмник кадров не отдаёт, на вкладке «Приёмник» «Этап 1».
        # У кого управление: "virtual" — страница /tx (ползунки, тумблеры, крутилки),
        # "physical" — настоящая аппаратура (tools/tx.py joystick|wire). Кнопки страницы
        # переключают это состояние: «Подключить аппаратуру» — управление физической,
        # «Отключить (выдернуть USB)» — обратно виртуальной. Смена источника = другая
        # аппаратура: связь сбрасывается, каналы активирует «Связать (Bind)».
        self.rc_control = "virtual"
        # Какая ось пульта каким стиком управляет — это знает только инструмент, читающий
        # пульт (`tools/tx.py joystick`). Он сообщает раскладку сюда, чтобы её было видно
        # на странице /tx: если стики двигают не свои каналы, это сразу заметно.
        self.rc_map_info = {}
        self.rc_layout_note = ""            # «раскладка осей не та» — считает tools/tx.py
        self.rc_aux_map = []                # «AUX1 ← тумблер (2 положения) — ось 3» — из tools/tx.py
        self.rc_sources = []                # [(время, источник)] — кто писал каналы
        self.rc_channels_updated = 0.0      # когда каналы обновлялись в последний раз
        self.rx_bind_state = "bound"
        # «RX rate» модели виртуальной аппаратуры — то же число, что печатает status
        # («RX rate: 33»): команда rc_smoothing_info берёт частоту приёма отсюда.
        self.RX_RATE_HZ = 33
        self.rx_bind_deadline = 0.0
        self.rx_bind_seconds = VirtualFC.RX_BIND_SECONDS
        self.mixer_name = "QUADX"
        self.mixer_names = ["TRI", "QUADP", "QUADX", "BI", "GIMBAL", "Y6", "HEX6", "FLYING_WING", "Y4",
                            "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX", "AIRPLANE", "HELI_120_CCPM",
                            "HELI_90_DEG", "VTAIL4", "HEX6H", "PPM_TO_SERVO", "DUALCOPTER", "SINGLECOPTER",
                            "ATAIL4", "CUSTOM", "CUSTOMAIRPLANE", "CUSTOMTRI", "QUADX1234"]
        self.detected_sensors = ["GYRO=MPU6000, ACC=MPU6000, BARO=BMP280 (2)"]
        self.battery_cells = 4
        self.battery_cell_voltage = 4.20   # 4S, каждая банка ровно 4.20 В (стол)
        self.mah_drawn = 0.0               # счётчик израсходованного: тикает только при работе моторов
        self._mah_t = None
        # Моторы: motor_disarmed[] из mixer.c — то, что пишет MSP_SET_MOTOR (214) со слайдеров
        # вкладки «Моторы». Прошивка НЕ сбрасывает эти значения по таймауту: пока борт не взведён,
        # mixer.c отдаёт выходы напрямую (`motor[i] = motor_disarmed[i]`), и мотор продолжает
        # крутиться на выставленном газе, пока Configurator не пришлёт нули.
        self.motor_disarmed = [1000.0] * MAX_SUPPORTED_MOTORS   # motor_disarmed[] в нуле → внешние 1000
        self.motor_output = list(self.motor_disarmed)     # MSP_MOTOR (104): внешние значения 1000…2000
        self.motor_rpm = [0.0] * MAX_SUPPORTED_MOTORS     # MSP_MOTOR_TELEMETRY (139): обороты
        self.motor_temp_c = [self.MOTOR_AMBIENT_C] * MAX_SUPPORTED_MOTORS
        self.motor_current_a = [0.0] * MAX_SUPPORTED_MOTORS
        self.motor_mah = [0.0] * MAX_SUPPORTED_MOTORS
        self.motor_voltage_v = 0.0
        self._motors_t = time.time()
        self.motor_test_arming_block = False               # флаг ARMING_DISABLED_MSP
        # «Виртуальный полёт» (SIM_* выше): сценарий газа, чтобы вкладка «Питание и батарея»
        # была живой — ток, расход мА·ч и напряжение меняются сами. Пока выключен, эмулятор
        # ведёт себя как стол: электроника ест ≈0.55 А, моторы стоят (тест со слайдеров).
        self.sim_flight = False
        self.sim_flight_phase = "idle"      # idle | flight | landing | swap
        self.sim_flight_since = 0.0         # время начала текущей фазы
        self.sim_flight_note = ""           # последнее событие: посадка, замена батареи…
        self.sim_pack_events = 0            # сколько раз батарея «кончилась» за сеанс
        self.sim_time_scale = self.SIM_FLIGHT_TIME_SCALE
        self._fs_init_state()               # «демо failsafe»: фазы, таймеры, журнал
        self.motor_output_reordering = list(range(MAX_SUPPORTED_MOTORS))
        self.dshot_commands_log = []
        # Таблицы, к которым в прошивке 4.5.4 обращаются только команды CLI (rxrange, led,
        # color, mode_color, mmix, beacon). Живут во «флеше» рядом с остальной конфигурацией,
        # чтобы Save/Reboot их не терял — как PG-таблицы прошивки.
        # Ресурсы платы (команда `resource`): назначения пинов — это конфигурация,
        # поэтому они живут во «флеше» и сбрасываются `defaults`, как PG-таблицы прошивки.
        self.resources = self._default_resources()
        self.rx_ranges = self._default_rx_ranges()               # rxChannelRangeConfigs[]
        self.led_configs = self._default_led_configs()            # ledStripStatusModeConfig.ledConfigs[]
        self.led_colors = self._default_led_colors()              # ...colors[]
        self.mode_colors = self._default_mode_colors()            # ...modeColors[]/specialColors
        self.custom_motor_mix = self._default_custom_motor_mix()  # customMotorMixer[]
        # beeperConfig.dshotBeaconOffFlags: DEFAULT_DSHOT_BEACON_OFF_FLAGS =
        # DSHOT_BEACON_ALLOWED_MODES (io/beeper.h, без USE_RACE_PRO) → RX_LOST и RX_SET выключены
        self.dshot_beacon_disabled: set[str] = {"RX_LOST", "RX_SET"}
        self.dshot_beacon_tone = 1            # dshotBeaconTone из reset-шаблона pg/beeper.c
        # flash_read/flash_write (USE_FLASH_TOOLS): «флеш» эмулятора — файл чёрного ящика.
        # Запись держим в памяти: прошивка пишет прямо в накопитель, но нам нельзя портить
        # журнал пролётов, поэтому наложение живёт до перезапуска (и сбрасывается flash_erase).
        self.flash_overlay: dict[int, bytes] = {}
        # rxFailsafeChannelConfigs: «Этап 1» вкладки Failsafe. Живёт во «флеше»,
        # заводские значения — как в pgResetFn_rxFailsafeChannelConfigs().
        self.rxfail = self._default_rxfail()
        # Сервоприводы (PG_SERVO_PARAMS / PG_SERVO_MIXER): живут во «флеше».
        self.servo_params = self._default_servo_params()
        self.servo_mix_rules = self._default_servo_mix_rules()
        self.arming_disable_names = "ARMING_DISABLED_ARM_SWITCH"
        self.uids = (0x1A2B3C4D, 0x5E6F7081, 0x92A3B4C5)
        self.cycle_time = 125
        self.cpu_load = 12
        self.boot_time = time.time()
        self.cli_active = False
        self.batch_mode = False
        self.pending_reboot = False
        self.pending_bootloader = False
        # Режим запоминающего устройства (MSP_SET_REBOOT с кодом 2/3): как настоящий борт,
        # эмулятор «отключает» MSP и отдаёт содержимое «флеша» файлами — только не по USB,
        # а на странице веб-монитора (см. /msc). Живёт ограниченное время, чтобы не застрять.
        self.msc_mode = False
        self.msc_deadline = 0.0
        self.craft_name = STRING_DEFAULTS["craft_name"]
        self.pilot_name = STRING_DEFAULTS["pilot_name"]
        self.pid_profile_names = {0: "Profile 1", 1: "Profile 2", 2: "Profile 3", 3: "Profile 4"}
        self.rate_profile_names = {0: "Actual", 1: "Actual", 2: "Actual", 3: "Actual"}
        # Профили PID и рейтов (PID_PROFILE_COUNT / CONTROL_RATE_PROFILE_COUNT).
        # Активный профиль хранится в self.settings — так CLI и MSP показывают одно и то же,
        # а переключение профиля (MSP_SELECT_SETTING) переносит значения между ними.
        self.pid_profiles = [dict(PID_PROFILE_DEFAULTS) for _ in range(PID_PROFILE_COUNT)]
        self.rate_profiles = [dict(RATE_PROFILE_DEFAULTS) for _ in range(CONTROL_RATE_PROFILE_COUNT)]
        # Профили батареи (BATTERY_PROFILE_COUNT) и активный индекс — как systemConfig
        # мастера. В 4.5.4 таких профилей нет: таблица живёт только ради CLI/MSP новой
        # прошивки, настройки vbat_* из 4.5.4 при этом не трогаются.
        self.battery_profiles = self._default_battery_profiles()
        self.active_battery_profile = 0
        self.build_key = "0" * 32
        self.current_profile = 0
        self.current_rate_profile = 0
        self.writes_log: list[str] = []
        self.stats: dict[int, int] = {}
        self.recent_msp: list[tuple[float, int, str, int, int]] = []  # (время, код, имя, запрос, ответ)
        self.recent_cli: list[tuple[float, str, str]] = []  # (время, команда, ответ)
        self._echo_store: dict[int, bytes] = {}
        # Конфигурация, записанная вкладками через MSP_SET_* (код GET -> payload)
        self.config_payloads: dict[int, bytes] = {}
        # Выбор железа датчиков во вкладке «Датчики»: 0 = AUTO, 1 = NONE
        self.sensor_config = {
            "acc_hardware": SENSOR_AUTO,
            "baro_hardware": SENSOR_AUTO,
            "mag_hardware": SENSOR_NONE,   # магнитометра нет
            "sonar_hardware": RANGEFINDER_NONE,
        }
        # Настройки OSD (вкладка «OSD»): элементы, статистика, таймеры, предупреждения.
        self.osd_config = self._default_osd_config()
        # Слоты «Корректирования в полёте» (вкладка «Корректирование в полёте»): пара
        # «диапазон AUX-канала → функция корректировки». Заполнены как у настроенного квада:
        # с пустой таблицей вкладка выглядит пустой (она показывает только включённые слоты).
        self.adjustment_ranges = self._stock_adjustment_ranges()
        # Таблица VTX (вкладка «ВИДЕОПЕРЕДАТЧИК») и состояние самого передатчика.
        # Заполненная (как у настроенного квада), чтобы Configurator не показывал
        # «таблица VTX не настроена» на вкладке «Порты».
        self.vtx_table = self._stock_vtx_table()
        self.vtx_pit_mode = False
        self.vtx_device_type = VTXDEV_UNKNOWN
        # vtxChannelActivationConditions[10] (vtx_control.h): переключение каналов VTX
        # по AUX-каналу; команда CLI `vtx`. Как во «флеше» с завода — все слоты нулевые.
        self.vtx_activation = self._default_vtx_activation()
        # Калибровка акселерометра (PG_ACCELEROMETER_CONFIG в прошивке): после «defaults»
        # флаг снят, кнопка «Калибровать» во вкладке «Датчики» его выставляет.
        self.acc_calibrated = False
        self.acc_calibration_until = 0.0
        self._cli = CliEngine(self)
        self._cli_log: list[str] = []
        # «Флеш» читаем последним: к этому моменту созданы все поля, которые он восстанавливает
        # (фичи, порты, имена, профили, UID), иначе значения перетирались бы инициализацией.
        self.load_eeprom()
        # «Чёрный ящик»: наполняем накопитель тестовыми пролётами. Если «флеш» уже собран
        # (файл рядом с настройками), он просто читается с диска — старт остаётся быстрым.
        self.blackbox.start(self.blackbox_p_interval())
        self._select_default_vtx_channel()
        self.check_feature_bits()
        # Передатчик ищется после чтения «флеша»: тип зависит от функций портов.
        self.init_vtx_device()

    # ------------------------------------- «Корректирование в полёте» (adjustments)
    @classmethod
    def _default_adjustment_ranges(cls) -> list:
        """Заводские слоты корректировок — как pgResetFn_adjustmentRangeConfig(): все пустые.

        Слот = «когда AUX-канал в диапазоне → применить функцию к каналу-переключателю».
        Пустой слот: диапазон 900…900 (начало равно концу), функция 0 («без изменений») —
        прошивка такой слот не выполняет, и вкладка считает его выключенным.
        """
        return [{"aux": 0, "start": 0, "end": 0, "function": 0, "switch": 0}
                for _ in range(ADJUSTMENT_RANGE_COUNT)]

    @classmethod
    def _stock_adjustment_ranges(cls) -> list:
        """Слоты «настроенного квада»: несколько рабочих корректировок.

        Пустая таблица — валидное заводское состояние, но вкладка «Корректирование в полёте»
        по умолчанию показывает только включённые слоты, поэтому с пустой таблицей страница
        выглядит пустой. Здесь — обычный набор: тумблеры подстраивают RC Rate и RC Expo,
        ещё два выбирают профиль рейтов и силу режима Horizon.
        """
        ranges = cls._default_adjustment_ranges()
        demo = [
            # шаг = (импульс − 900) / 25 — в таком виде значение и уходит в MSP
            {"aux": 0, "start": 16, "end": 32, "function": 1, "switch": 0},   # AUX1 1300…1700 → RC Rate
            {"aux": 1, "start": 16, "end": 32, "function": 2, "switch": 1},   # AUX2 1300…1700 → RC Expo
            {"aux": 2, "start": 32, "end": 48, "function": 12, "switch": 2},  # AUX3 1700…2100 → профиль рейтов
            {"aux": 3, "start": 32, "end": 48, "function": 24, "switch": 3},  # AUX4 1700…2100 → сила Horizon
        ]
        for slot, values in enumerate(demo):
            ranges[slot] = values
        return ranges

    @classmethod
    def _adjustment_ranges_from_json(cls, data) -> list:
        """Слоты корректировок из «флеша»: битое и лишнее отбрасываем, слоты добираем."""
        ranges = cls._default_adjustment_ranges()
        if not isinstance(data, list):
            return ranges
        for index, item in enumerate(data[:ADJUSTMENT_RANGE_COUNT]):
            if not isinstance(item, dict):
                continue
            slot = dict(ranges[index])
            for key, maximum in (("aux", 15), ("start", 48), ("end", 48),
                                 ("function", ADJUSTMENT_FUNCTION_COUNT), ("switch", 15)):
                try:
                    slot[key] = max(0, min(maximum, int(item[key])))
                except (KeyError, TypeError, ValueError):
                    pass
            ranges[index] = slot
        return ranges

    def adjustment_slots_active(self) -> int:
        """Сколько слотов прошивка считает включёнными (начало диапазона не равно концу)."""
        return sum(1 for slot in self.adjustment_ranges if slot["start"] != slot["end"])

    # --------------------- таблицы команд CLI 4.5.4 во «флеше» (rxrange / led / color / mmix)
    def _load_cli_table(self, name: str, data):
        """Читает таблицы `rxrange`, `led`, `color`, `mode_color` и `mmix` из «флеша».

        Прошивка держит их в PG-таблицах (rxChannelRangeConfigs, ledStripStatusModeConfig,
        customMotorMixer), поэтому эмулятор хранит их рядом с остальной конфигурацией: битое
        и лишнее отбрасываем, недостающие записи добираем заводскими.
        """
        if name == "rx_ranges":
            ranges = self._default_rx_ranges()
            if isinstance(data, list):
                for index, item in enumerate(data[:len(ranges)]):
                    if isinstance(item, dict):
                        ranges[index] = {
                            "min": self._clamp_int(item.get("min"), PWM_PULSE_MIN, PWM_PULSE_MAX,
                                                   ranges[index]["min"]),
                            "max": self._clamp_int(item.get("max"), PWM_PULSE_MIN, PWM_PULSE_MAX,
                                                   ranges[index]["max"]),
                        }
            self.rx_ranges = ranges
        elif name == "led_configs":
            configs = self._default_led_configs()
            if isinstance(data, list):
                for index, item in enumerate(data[:len(configs)]):
                    if isinstance(item, str) and item:      # строка формата generateLedConfig()
                        configs[index] = item[:40]
            self.led_configs = configs
        elif name == "led_colors":
            colors = self._default_led_colors()
            if isinstance(data, list):
                for index, item in enumerate(data[:len(colors)]):
                    if isinstance(item, list) and len(item) == 3:
                        h = min(359, max(0, self._clamp_int(item[0], 0, 359, 0)))
                        colors[index] = [h, self._clamp_int(item[1], 0, 255, 0),
                                         self._clamp_int(item[2], 0, 255, 0)]
            self.led_colors = colors
        elif name == "mode_colors":
            colors = self._default_mode_colors()
            if isinstance(data, dict):
                mode = data.get("mode")
                if isinstance(mode, list):
                    for index, row in enumerate(mode[:LED_MODE_COUNT]):
                        if isinstance(row, list) and len(row) == LED_DIRECTION_COUNT:
                            colors["mode"][index] = [self._clamp_int(v, 0, LED_CONFIGURABLE_COLOR_COUNT - 1, 0)
                                                     for v in row]
                special = data.get("special")
                if isinstance(special, list) and len(special) >= LED_SPECIAL_COLOR_COUNT:
                    colors["special"] = [self._clamp_int(v, 0, LED_CONFIGURABLE_COLOR_COUNT - 1, 0)
                                        for v in special[:LED_SPECIAL_COLOR_COUNT]]
                colors["aux_channel"] = self._clamp_int(data.get("aux_channel"), 0, MAX_AUX_CHANNEL_COUNT,
                                                        colors["aux_channel"])
            self.mode_colors = colors
        elif name == "custom_motor_mix":
            mix = []
            if isinstance(data, list):
                for item in data[:MAX_SUPPORTED_MOTORS]:
                    if not isinstance(item, dict):
                        continue
                    entry = {}
                    for field in ("throttle", "roll", "pitch", "yaw"):
                        try:
                            entry[field] = float(item[field])
                        except (KeyError, TypeError, ValueError):
                            entry[field] = 0.0
                    if entry["throttle"] == 0.0:      # printMotorMix() останавливается здесь
                        break
                    mix.append(entry)
            self.custom_motor_mix = mix

    @staticmethod
    def _clamp_int(value, low: int, high: int, fallback: int) -> int:
        """Значение из «флеша» с ограничением: не число — берём заводское."""
        try:
            return max(low, min(high, int(value)))
        except (TypeError, ValueError):
            return fallback

    # ------------------------------------------------------------------ OSD
    @classmethod
    def _osd_config_from_json(cls, data) -> dict:
        """Настройки OSD из «флеша»: неизвестное и битое заменяется заводскими значениями."""
        cfg = cls._default_osd_config()
        if not isinstance(data, dict):
            return cfg
        simple = ("flags", "video_system", "units", "rssi_alarm", "cap_alarm", "alt_alarm",
                  "link_quality_alarm", "enabled_warnings", "enabled_stats", "osd_profile_index",
                  "overlay_radio_mode", "camera_frame_width", "camera_frame_height",
                  "canvas_cols", "canvas_rows")
        for key in simple:
            try:
                cfg[key] = int(data[key])
            except (KeyError, TypeError, ValueError):
                pass
        positions = data.get("item_pos")
        if isinstance(positions, list) and len(positions) == OSD_ITEM_COUNT:
            cfg["item_pos"] = [int(value) & 0xFFFF for value in positions]
        timers = data.get("timers")
        if isinstance(timers, list) and len(timers) == OSD_TIMER_COUNT:
            cfg["timers"] = [int(value) & 0xFFFF for value in timers]
        if not 1 <= cfg["osd_profile_index"] <= OSD_PROFILE_COUNT:
            cfg["osd_profile_index"] = 1
        return cfg

    @classmethod
    def _default_osd_config(cls) -> dict:
        """Заводские настройки OSD — pgResetFn_osdConfig() из 4.5.4.

        Счётчики (80 элементов, 31 статистика, 19 предупреждений, 2 таймера, 3 профиля) —
        ровно те, что ждёт Configurator по своей версии API. Раскладка по умолчанию повторяет
        привычный экран настроенного квада: напряжение, предупреждения, таймер, газ, высота,
        спутники, имя и RSSI.
        """
        warnings = 0
        for index in range(OSD_WARNING_COUNT):
            if index not in OSD_WARNINGS_OFF:
                warnings |= 1 << index
        stats = 0
        for index in OSD_STATS_DEFAULT:
            stats |= 1 << index
        positions = [0] * OSD_ITEM_COUNT
        for name, (x, y) in cls._osd_default_layout().items():
            index = OSD_ITEMS.index(name)
            if 0 <= index < OSD_ITEM_COUNT:
                positions[index] = cls._osd_position(x, y, 1)   # видно в профиле 1
        return {
            "flags": OSD_FLAG_FEATURE | OSD_FLAG_MAX7456 | OSD_FLAG_DETECTED,
            "video_system": 0,                       # VIDEO_SYSTEM_AUTO — как в pgResetFn_vcdProfile
            "units": 1,                              # UNIT_METRIC
            "rssi_alarm": 20,
            "cap_alarm": 2200,
            "alt_alarm": 100,
            "link_quality_alarm": 80,
            "timers": [cls._osd_timer(0, 0, 10), cls._osd_timer(1, 0, 10)],
            "enabled_warnings": warnings,
            "enabled_stats": stats,
            "osd_profile_index": 1,
            "overlay_radio_mode": 2,
            "camera_frame_width": 24,
            "camera_frame_height": 11,
            "canvas_cols": OSD_CANVAS_COLS,
            "canvas_rows": OSD_CANVAS_ROWS,
            "item_pos": positions,
        }

    @classmethod
    def _osd_default_layout(cls) -> dict:
        return dict(OSD_DEFAULT_LAYOUT)

    @staticmethod
    def _osd_position(x: int, y: int, profile: int = 0) -> int:
        """Упаковать место элемента: биты 0…4 — столбец, 5…9 — строка, 11+номер профиля — «видно».

        Точно как OSD_POSITION/VISIBLE в osd.h: Configurator читает x = bits & 0x1f,
        y = (bits >> 5) & 0x1f, видимость — (bits & (0x0800 << профиль)).
        """
        value = (int(x) & 0x1F) | ((int(y) & 0x1F) << 5)
        if profile:
            value |= OSD_VISIBLE << (int(profile) - 1)
        return value

    @staticmethod
    def _osd_timer(source: int, precision: int, alarm: int) -> int:
        """OSD_TIMER(src, prec, alarm) из osd.h: src — биты 0…3, prec — 4…7, alarm — 8…15."""
        return (int(source) & 0x0F) | ((int(precision) & 0x0F) << 4) | ((int(alarm) & 0xFF) << 8)

    @staticmethod
    def _osd_unpack_position(value: int) -> tuple:
        """Разложить место элемента обратно в (столбец, строка) — для страниц и логов."""
        return (int(value) & 0x1F, (int(value) >> 5) & 0x1F)

    def osd_item_positions_info(self) -> list:
        """Что стоит на экране: [(индекс, имя, x, y, видно ли), …] — для монитора и тестов."""
        rows = []
        for index, value in enumerate(self.osd_config["item_pos"]):
            if not value:
                continue
            x, y = self._osd_unpack_position(value)
            visible = [bool(value & (OSD_VISIBLE << profile))
                       for profile in range(OSD_PROFILE_COUNT)]
            rows.append({"index": index, "name": OSD_ITEMS[index] if index < len(OSD_ITEMS) else "?",
                         "x": x, "y": y, "visible": visible})
        return rows

    # ------------------------------------------------------------ настройки
    # Найденное «железо» виртуальной платы (id из таблиц прошивки).
    DETECTED_GYRO = 15        # ICM42688P
    DETECTED_ACC = 16         # ICM42688P
    DETECTED_BARO = 4         # BMP280
    DETECTED_MAG = 5          # QMC5883
    DETECTED_RANGEFINDER = 2  # TFMINI
    GYRO_1_MASK = 1 << 0      # gyroDetectionFlags: найден гироскоп 1

    # ------------------------------------------------------------------ модель моторов
    # Вкладка «Моторы» — единственное место, где эмулятор должен быть «физическим»: слайдеры
    # задают газ, а MSP_MOTOR (104) и MSP_MOTOR_TELEMETRY (139) обязаны на это живо откликаться.
    # Обороты/температура/ток/напряжение считаются по простой модели мотора и ESC:
    #   • обороты догоняют задание с постоянной времени раскрутки (и медленнее — при остановке);
    #   • температура греется от газа (≈1.15 °C/с на полном) и остывает к температуре воздуха;
    #   • ток ESC = 0.35 А + gas² · 12 А, напряжение — напряжение батареи минус просадка от
    #     суммарного тока (0.010 Ом), расход интегрируется в мА·ч.
    MOTOR_MAX_RPM = 24000.0          # 2207 ~2500 KV на 4S, пары полюсов 7 → eRPM ≈ 168 тыс.
    MOTOR_SPOOL_TAU_UP = 0.10        # с — раскрутка оборотов
    MOTOR_SPOOL_TAU_DOWN = 0.25      # с — выбег (винт тормозит медленнее, чем ESC разгоняет)
    MOTOR_AMBIENT_C = 24             # ESC на столе
    MOTOR_HEAT_C_PER_S = 1.8         # нагрев на полном газу, °C/с
    MOTOR_COOL_TAU_S = 25.0          # постоянная остывания → 24 + 45 ≈ 69 °C на полном газу
    MOTOR_CURRENT_IDLE_A = 0.35      # холостой ток ESC
    MOTOR_CURRENT_FULL_A = 12.0      # ток мотора на полном газу
    MOTOR_PACK_RESISTANCE_OHM = 0.010
    MOTOR_INVALID_PCT_NO_TELEMETRY = 10000   # msp.c: 10000 = «100.00 %» — телеметрии нет

    # ------------------------------------------------------------ ресурсы (команда resource)
    # Пины виртуальной платы BFR/VIRTUALFC (STM32F405) — как resourceTable[] в cli.c 4.5.4:
    # имя владельца, число слотов и назначенные пины. Порядок записей — порядок прошивки
    # (BEEPER, MOTOR, LED_STRIP, SERIAL_TX/RX, SOFTSERIAL_TX/RX, I2C, LED, SPI, ADC,
    # FLASH_CS, OSD_CS, GYRO_*, USB_DETECT); в нём же печатают `resource` и `resource show`.
    # Пины согласованы с тем, что эмулятор уже показывает во вкладке «Порты»: UART1…UART5,
    # GPS на UART1, приёмник на UART2, ESC_SENSOR на UART3, VTX (TBS SmartAudio) на UART4.
    RESOURCE_OWNERS = (
        ("BEEPER",        ["C13"]),
        ("MOTOR",         ["A00", "A01", "A02", "A03", None, None, None, None]),   # QUADX: 4 из 8
        ("LED_STRIP",     ["B08"]),
        ("SERIAL_TX",     ["A09", "D05", "D08", "C10", "C12"]),
        ("SERIAL_RX",     ["A10", "D06", "D09", "C11", "D02"]),
        ("SOFTSERIAL_TX", ["A08", "B00"]),
        ("SOFTSERIAL_RX", ["A11", "B11"]),
        ("I2C_SCL",       ["B06", None]),
        ("I2C_SDA",       ["B07", None]),
        ("LED",           ["C14"]),
        ("SPI_SCK",       ["A05", "B13"]),
        ("SPI_SDI",       ["A06", "B14"]),
        ("SPI_SDO",       ["A07", "B15"]),
        ("ADC_BATT",      ["C00"]),
        ("ADC_RSSI",      ["C02"]),
        ("ADC_CURR",      ["C01"]),
        ("FLASH_CS",      ["C15"]),
        ("OSD_CS",        ["B12"]),
        ("GYRO_EXTI",     ["C04"]),
        ("GYRO_CS",       ["A04"]),
        ("USB_DETECT",    ["C05"]),
    )
    # Пины с таймерами — как timerHardware[] в target-файле: (таймер, канал, AF).
    # Нужны для `timer` и для раздела таймеров в `resource show all`.
    RESOURCE_PIN_TIMERS = {
        "A00": ("TIM2", 1, 1), "A01": ("TIM2", 2, 1),
        "A02": ("TIM2", 3, 1), "A03": ("TIM2", 4, 1),
        "B08": ("TIM4", 3, 2),
    }
    RESOURCE_TIMERS = ("TIM1", "TIM2", "TIM3", "TIM4", "TIM5", "TIM8")   # таймеры F405 с каналами
    # Таблица заданий DMA — dmaoptEntryTable 4.5.4 (cli.c:5321) для виртуальной платы:
    # SPI_SDO/SPI_SDI и алиасы SPI_TX/SPI_RX (та же настройка SPI, совместимость
    # с конфигами 4.2.x), ADC 1…3 (ADCDEV_COUNT для F405), UART_TX/UART_RX 1…11
    # (UARTDEV_CONFIG_MAX = 11). Элемент: (устройство, индекс для CLI, поток или None);
    # None — DMA_OPT_UNUSED, такой слот прошивка печатает как «dma <устройство> <индекс> NONE».
    # SDIO в таблице прошивки тоже есть, но у него maxIndex = 0 — строк он не даёт.
    RESOURCE_DMA_OPT = (
        ("SPI_SDO", 1, (2, 3, 3)),
        ("SPI_SDI", 1, (2, 0, 3)),
        ("SPI_TX", 1, (2, 3, 3)),
        ("SPI_RX", 1, (2, 0, 3)),
        ("ADC", 1, (2, 4, 0)),
        ("ADC", 2, None),
        ("ADC", 3, None),
    ) + tuple(("UART_TX", i, {1: (2, 7, 4), 2: (1, 6, 4)}.get(i)) for i in range(1, 12)) \
      + tuple(("UART_RX", i, {1: (2, 5, 4), 2: (1, 5, 4)}.get(i)) for i in range(1, 12))
    # Сколько индексов у устройства в dmaoptEntryTable (для `dma <устройство> <индекс>`).
    RESOURCE_DMA_DEVICES = {
        "SPI_SDO": 1, "SPI_SDI": 1, "SPI_TX": 1, "SPI_RX": 1,
        "ADC": 3, "SDIO": 0, "UART_TX": 11, "UART_RX": 11,
    }
    # Занятые DMA-потоки F405 (два контроллера по 8 потоков) — что печатает `dma show`.
    RESOURCE_DMA = {
        ("DMA1", 5): ("SERIAL_RX", 2), ("DMA1", 6): ("SERIAL_TX", 2),
        ("DMA2", 0): ("SPI_SDI", 1), ("DMA2", 3): ("SPI_SDO", 1),
        ("DMA2", 4): ("ADC_BATT", 1), ("DMA2", 5): ("SERIAL_RX", 1),
        ("DMA2", 7): ("SERIAL_TX", 1),
    }

    # ------------------------------------------------- «виртуальный полёт» (живое питание)
    # На столе вкладка «Питание и батарея» мертва: моторы стоят, ток нулевой, счётчик
    # мА·ч не растёт. Чтобы состояние питания было живым, есть «виртуальный полёт»:
    # кнопка на веб-мониторе (/sim_flight/on) или команда CLI `sim_flight on`.
    # В полёте эмулятор ведёт себя как летящий борт: газ по сценарию, ток 3…27 А,
    # напряжение проседает под нагрузкой и постепенно падает по мере разряда,
    # счётчик мА·ч растёт. Время в полёте идёт быстрее стенных часов (×10), чтобы
    # цифры на вкладке были видны глазом, а не менялись раз в минуту.
    SIM_FLIGHT_TIME_SCALE = 10.0      # во сколько раз быстрее «летит» время (можно менять)
    SIM_IDLE_CURRENT_A = 0.55         # электроника борта: FC, приёмник, VTX, LED — покой
    SIM_IDLE_RIPPLE_A = 0.06          # «дыхание» тока покоя: 0.49…0.61 А, как у живого датчика
    SIM_HOVER_GAS = 0.32              # газ висения → ≈6 А на четырёх моторах
    SIM_PUNCH_GAS = 0.42              # надбавка «газ в пол» → ≈27 А
    SIM_TAKEOFF_S = 3.0               # с — раскрутка на взлёте
    SIM_PUNCH_PERIOD_S = 25.0         # с — как часто «даём газу»
    SIM_PUNCH_S = 3.0                 # с — длительность «газа в пол»
    SIM_LAND_S = 2.5                  # с — посадка при разряженной батарее
    SIM_SWAP_S = 3.0                  # с — пауза «замена батареи»
    SIM_PACK_FULL_V = 4.20            # В на банку у заряженной батареи
    SIM_PACK_MIN_V = 3.50             # В на банку в конце полёта (уровень предупреждения)

    FEATURE_BITS_EXPECTED = {"SOFTSERIAL": 1 << 6, "GPS": 1 << 7, "OSD": 1 << 18, "LED_STRIP": 1 << 16}

    def check_feature_bits(self):
        """Сверка с feature.h/Features.js: SOFTSERIAL = бит 6, GPS = 7, LED_STRIP = 16, OSD = 18."""
        wrong = [
            name for name, bit in self.FEATURE_BITS_EXPECTED.items()
            if self.feature_bits_by_name.get(name) != bit
        ]
        if wrong:
            self._log(
                "[FC] ВНИМАНИЕ: биты фич не совпадают с прошивкой: "
                + ", ".join(f"{n}=0x{self.feature_bits_by_name.get(n, 0):08x}" for n in wrong)
            )
        return not wrong

    # ------------------------------------------------------------ VTX
    @staticmethod
    def _pad(text: str, length: int) -> str:
        """vtxTableStrncpyWithPad(): обрезать и добрать пробелами до нужной длины."""
        return text[:length].ljust(length)

    @staticmethod
    def _default_vtx_activation() -> list:
        """vtxChannelActivationConditions[10] как с завода: все слоты нулевые
        (MAX_CHANNEL_ACTIVATION_CONDITION_COUNT = 10, vtx_control.h 4.5.4)."""
        return [{"aux": 0, "band": 0, "channel": 0, "power": 0,
                 "start_step": 0, "end_step": 0}
                for _ in range(10)]

    def _vtx_activation_from_json(self, value) -> list:
        """Слоты из старого/нового «флеша»: битое отбрасываем, недостающие добираем нулями."""
        out = []
        for item in (value or [])[:10] if isinstance(value, list) else []:
            if isinstance(item, dict):
                out.append({
                    "aux": int(item.get("aux", 0)), "band": int(item.get("band", 0)),
                    "channel": int(item.get("channel", 0)), "power": int(item.get("power", 0)),
                    "start_step": int(item.get("start_step", 0)), "end_step": int(item.get("end_step", 0)),
                })
        while len(out) < 10:
            out.append({"aux": 0, "band": 0, "channel": 0, "power": 0,
                        "start_step": 0, "end_step": 0})
        return out

    @classmethod
    def _empty_vtx_table(cls) -> dict:
        """Пустая таблица — ровно то, что оставляет pgResetFn_vtxTableConfig().

        Заглушки как в прошивке: имена BAND1..BAND8, буквы 1..8, подписи LV0..LV7,
        частоты 0. Числа bands/channels/powerlevels при этом нулевые, и вкладка
        показывает «таблица не настроена» — с этого и начинается живой FC.
        """
        return {
            "bands": 0,
            "channels": 0,
            "powerlevels": 0,
            "band_names": [cls._pad(f"BAND{i + 1}", VTX_TABLE_BAND_NAME_LENGTH)
                           for i in range(VTX_TABLE_MAX_BANDS)],
            "band_letters": [str(i + 1) for i in range(VTX_TABLE_MAX_BANDS)],
            "is_factory": [False] * VTX_TABLE_MAX_BANDS,
            "frequencies": [[0] * VTX_TABLE_MAX_CHANNELS for _ in range(VTX_TABLE_MAX_BANDS)],
            "power_values": [0] * VTX_TABLE_MAX_POWER_LEVELS,
            "power_labels": [cls._pad(f"LV{i}", VTX_TABLE_POWER_LABEL_LENGTH)
                             for i in range(VTX_TABLE_MAX_POWER_LEVELS)],
        }

    # Стандартная таблица 5.8 ГГц: полосы из файла «Betaflight VTX Config», который
    # производители (BNF/RTF) и Configurator грузят во вкладке «ВИДЕОПЕРЕДАТЧИК».
    # Тем она и отличается от pgResetFn_vtxTableConfig(): у «голой» прошивки таблица пуста,
    # а у готового квада — заполнена. С ней жёлтая плашка «таблица VTX не настроена»
    # на вкладке «Порты» не появляется.
    STOCK_BANDS = (
        ("BOSCAM A", "A", [5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725]),
        ("BOSCAM B", "B", [5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866]),
        ("BOSCAM E", "E", [5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945]),
        ("FATSHARK", "F", [5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880]),
        ("RACEBAND", "R", [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917]),
    )
    STOCK_POWERLEVELS = ((25, "25"), (100, "100"), (400, "400"), (800, "800"))

    @classmethod
    def _stock_vtx_table(cls) -> dict:
        """Заполненная таблица передатчика — то, что стоит в «уже настроенном» кваде."""
        table = cls._empty_vtx_table()
        table["bands"] = len(cls.STOCK_BANDS)
        table["channels"] = len(cls.STOCK_BANDS[0][2])
        table["powerlevels"] = len(cls.STOCK_POWERLEVELS)
        for index, (name, letter, frequencies) in enumerate(cls.STOCK_BANDS):
            table["band_names"][index] = cls._pad(name, VTX_TABLE_BAND_NAME_LENGTH)
            table["band_letters"][index] = letter
            table["is_factory"][index] = True
            for channel, frequency in enumerate(frequencies):
                table["frequencies"][index][channel] = frequency
        for index, (value, label) in enumerate(cls.STOCK_POWERLEVELS):
            table["power_values"][index] = value
            table["power_labels"][index] = cls._pad(label, VTX_TABLE_POWER_LABEL_LENGTH)
        return table

    def _select_default_vtx_channel(self) -> None:
        """Выбрать полосу/канал/мощность по таблице — как настроенный передатчик на кваде."""
        table = self.vtx_table
        if not table["bands"] or not table["channels"] or not table["powerlevels"]:
            return
        if self._setting("vtx_band", 0) or self._setting("vtx_channel", 0):
            return                     # человек уже что-то выбрал — не трогаем
        self._store_setting("vtx_band", 1)
        self._store_setting("vtx_channel", 1)
        self._store_setting("vtx_power", 1)
        self._store_setting("vtx_freq", int(table["frequencies"][0][0]))

    def _vtx_device_type(self) -> int:
        """Тип найденного передатчика (VTXDEV_UNKNOWN, если на портах нет функции VTX)."""
        return self.vtx_device_type

    def init_vtx_device(self) -> int:
        """Определение передатчика при загрузке — как vtxInit()/vtxSmartAudioInit().

        Прошивка ищет устройство один раз на старте — по порту с функцией VTX. Поэтому
        включение функции на вкладке «Порты» подхватывается после перезагрузки.
        """
        detected = VTXDEV_UNKNOWN
        for port in self.serial_ports:
            functions = port.get("functions") or []
            if "TBS_SMARTAUDIO" in functions:
                detected = VTXDEV_SMARTAUDIO
                break
            if "IRC_TRAMP" in functions:
                detected = VTXDEV_TRAMP
                break
        names = {VTXDEV_SMARTAUDIO: "TBS SmartAudio", VTXDEV_TRAMP: "IRC Tramp", VTXDEV_UNKNOWN: "не найден"}
        self._log(f"[FC] VTX: {names.get(detected)} (тип {detected})")
        self.vtx_device_type = detected
        return detected

    def _vtx_lookup_frequency(self, band: int, channel: int) -> int:
        """vtxCommonLookupFrequency(): частота из таблицы по банду и каналу."""
        table = self.vtx_table
        if 1 <= band <= table["bands"] and 1 <= channel <= table["channels"]:
            return int(table["frequencies"][band - 1][channel - 1])
        return 0

    def _vtx_clear_table(self) -> None:
        """vtxTableConfigClearBand/ClearPowerValues/ClearPowerLabels — как при флаге clear."""
        self.vtx_table = self._empty_vtx_table()
        self.vtx_activation = self._default_vtx_activation()

    def _load_settings(self):
        for s in self.data["settings"]:
            entry = dict(s)
            if entry.get("mode") == "MODE_STRING":
                # MODE_STRING: у прошивки заданы min/max длины (cli/settings.c) — их в
                # таблице формата нет, дописываем по именам параметров.
                entry["string_min"], entry["string_max"] = STRING_LENGTHS.get(
                    entry["name"], (1, MAX_NAME_LENGTH))
                if entry.get("default") is None:
                    entry["default"] = self._string_default(entry["name"])
            elif entry.get("default") is None:
                entry["default"] = self._heuristic_default(entry)
            self.settings_list.append(entry)
            self.settings_by_name[entry["name"]] = entry
            self.settings[entry["name"]] = entry["default"]
        self._apply_overrides()
        self._apply_profile_defaults()
        # _defaults — заводские настройки прошивки (их же показывает 'diff all' и 'defaults'),
        # self.settings — активная конфигурация, которая читается из «флеша» при старте.
        self._defaults = dict(self.settings)

    def _apply_profile_defaults(self):
        """Заводские значения PID-профиля и профиля рейтов (resetPidProfile / pgResetFn_controlRateProfiles)."""
        for table in (PID_PROFILE_DEFAULTS, RATE_PROFILE_DEFAULTS):
            for name, value in table.items():
                if name in self.settings_by_name:
                    self.settings[name] = value

    def _apply_overrides(self):
        """Небольшая таблица «хорошо известных» заводских значений.

        Большая часть defaults извлекается автоматически из pgResetFn_*/PG_RESET_TEMPLATE,
        но некоторые значения задаются выражениями, которые не разбираются генератором.
        Значения ниже соответствуют стоковой прошивке Betaflight 4.5.x.
        """
        override_path = Path(__file__).resolve().parent / "data" / "fc_overrides.json"
        if not override_path.exists():
            return
        for name, value in json.loads(override_path.read_text()).items():
            entry = self.settings_by_name.get(name)
            if entry is None:
                continue
            if isinstance(value, str) and entry["mode"] == "MODE_LOOKUP" and entry.get("values"):
                value = self._parse_lookup(entry, value)
                if value is None:
                    continue
            self.settings[name] = value
            entry["default"] = value

    @staticmethod
    def _heuristic_default(entry):
        if entry["mode"] == "MODE_LOOKUP":
            return 0
        return entry["min"] if entry["min"] > 0 else 0

    @staticmethod
    def _string_default(name: str) -> str:
        """Заводская строка параметра (пустая — «не задано», показывается как '-')."""
        return STRING_DEFAULTS.get(name, "")

    def reset_to_defaults(self):
        """'defaults': заводские настройки прошивки (из bf_data + fc_overrides).

        Таблица VTX тоже возвращается к заводской (у эмулятора она заполнена — как у квада,
        которому производитель залил таблицу своего передатчика).
        """
        self.settings = dict(self._defaults)
        self.vtx_table = self._stock_vtx_table()
        self.vtx_pit_mode = False
        self._select_default_vtx_channel()
        self.osd_config = self._default_osd_config()
        # pgResetFn_adjustmentRangeConfig(): заводская таблица корректировок пустая —
        # 'defaults' убирает все слоты (вкладка покажет «Слотов активно 0/30»).
        self.adjustment_ranges = self._default_adjustment_ranges()
        # Как pgResetFn_accelerometerConfig(): калибровка акселерометра теряется,
        # и вкладка «Датчики» снова показывает «акселерометр не откалиброван».
        self.acc_calibrated = False
        self.acc_calibration_until = 0.0
        self.rxfail = self._default_rxfail()   # pgResetFn_rxFailsafeChannelConfigs()
        # Таблицы команд CLI — как их сбрасывают PG-функции прошивки: rxrange снова
        # 1000…2000, лента и цвета не настроены, свой микшер пуст, DShot-пищалка включена.
        self.resources = self._default_resources()             # pgResetFn_* пинов платы
        self.rx_ranges = self._default_rx_ranges()
        self.led_configs = self._default_led_configs()
        self.led_colors = self._default_led_colors()
        self.mode_colors = self._default_mode_colors()
        self.custom_motor_mix = self._default_custom_motor_mix()
        self.dshot_beacon_disabled = {"RX_LOST", "RX_SET"}   # DEFAULT_DSHOT_BEACON_OFF_FLAGS
        self.dshot_beacon_tone = 1
        self.battery_profiles = self._default_battery_profiles()
        self.active_battery_profile = 0
        self.signature = b"\x00" * len(self.signature)
        # modeActivationConditions[] регистрируется без шаблона сброса: 'defaults' обнуляет
        # все 20 слотов (режим ARM, 900/900) — вкладка «Режимы» снова пустая.
        self.mode_ranges = self._default_mode_ranges()
        self.servo_params = self._default_servo_params()      # pgResetFn_servoParams()
        self.servo_mix_rules = self._default_servo_mix_rules()
        # resetPidProfile() + pgResetFn_controlRateProfiles(): все профили — заводские
        self.pid_profiles = [dict(PID_PROFILE_DEFAULTS) for _ in range(PID_PROFILE_COUNT)]
        self.rate_profiles = [dict(RATE_PROFILE_DEFAULTS) for _ in range(CONTROL_RATE_PROFILE_COUNT)]
        self.pid_profile_names = {i: f"Profile {i + 1}" for i in range(PID_PROFILE_COUNT)}
        self.rate_profile_names = {i: "Actual" for i in range(CONTROL_RATE_PROFILE_COUNT)}
        self.current_profile = 0
        self.current_rate_profile = 0
        # Состояние виртуальной аппаратуры 'defaults' не трогает: стики — не конфиг FC.
        # pgResetFn_rxConfig(): карта каналов — часть конфигурации приёмника,
        # поэтому 'defaults' возвращает заводскую AETR1234 (иначе вкладка «Приёмник»
        # показывала бы прежнюю карту после сброса).
        self.rx_map = "AETR1234"
        # pgResetFn_pilotConfig(): имена дрона и пилота — тоже часть настроек,
        # 'defaults' возвращает заводские (и CLI, и MSP видят одно и то же).
        for name in ("craft_name", "pilot_name"):
            if isinstance(self.settings.get(name), str):
                self._mirror_name_setting(name, self.settings[name])

    # ---------------------------------------------------- виртуальная аппаратура (TX)
    def state_based_read_codes(self) -> set:
        """Коды чтения, ответы на которые строятся из состояния FC (см. state_based_read_codes())."""
        cached = getattr(self, "_state_read_codes", None)
        if cached is None:
            cached = state_based_read_codes()
            self._state_read_codes = cached
        return cached

    @staticmethod
    def _default_rc_channels() -> list:
        """Стики в центре, газ внизу, тумблеры AUX выключены — «аппаратура включена, но стоит»."""
        return ([TX_MID_USEC, TX_MID_USEC, TX_MID_USEC, RX_LOW_USEC]
                + [RX_LOW_USEC] * (RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT))

    def reset_rc_channels(self) -> list:
        """Вернуть стики в исходное положение (кнопка «Сброс» на странице аппаратуры)."""
        self.rc_channels = self._default_rc_channels()
        return list(self.rc_channels)

    def set_rc_channel(self, channel, value) -> bool:
        """Один канал: номер (0…15) или имя (`roll`, `pitch`, `yaw`, `throttle`, `aux1`…`aux12`)."""
        index = self._rc_channel_index(channel)
        if index is None:
            return False
        self.rc_channels[index] = max(TX_MIN_USEC, min(TX_MAX_USEC, int(value)))
        return True

    def set_rc_channels(self, values) -> list:
        """Несколько каналов сразу: список из 16 значений или словарь {имя/номер: значение}."""
        if isinstance(values, dict):
            for channel, value in values.items():
                self.set_rc_channel(channel, value)
        else:
            for index, value in enumerate(list(values)[:RC_CHANNEL_COUNT]):
                if value is not None:
                    self.set_rc_channel(index, value)
        return list(self.rc_channels)

    @staticmethod
    def _rc_channel_index(channel):
        if isinstance(channel, int) or (isinstance(channel, str) and channel.lstrip("-").isdigit()):
            index = int(channel)
            return index if 0 <= index < RC_CHANNEL_COUNT else None
        name = str(channel).strip().lower().replace("_", "").replace(" ", "")
        name = {"a": "roll", "e": "pitch", "r": "yaw", "t": "throttle"}.get(name, name)
        names = list(RC_STICK_NAMES) + list(RC_AUX_NAMES)
        return names.index(name) if name in names else None

    def rc_state(self) -> dict:
        """Снимок каналов для веб-страницы аппаратуры и `/api/rc`."""
        self._resolve_bind()
        values = list(self.rc_channels)
        link = self.rx_link_state()
        return {
            "channels": values,
            "connected": link["connected"],
            "serial_ok": link["serial_ok"],
            "serial_port": link["serial_port"],
            "serial_functions": link["serial_functions"],
            "failsafe": link["failsafe"],
            "bind": link["bind"],
            "control": self.control_state(),
            "map": self.rc_map_info,
            "layout_note": self.rc_layout_note,
            "aux_map": list(self.rc_aux_map),
            **self.rc_source_state(),
            "aux_types": self.aux_types(),
            "switch3_values": list(AUX_SWITCH3_VALUES),
            "source": self.rc_source,
            "active_channels": RC_CHANNEL_COUNT if link["serial_ok"] else 0,
            "names": ["roll", "pitch", "yaw", "throttle", "aux1", "aux2", "aux3", "aux4", "aux5",
                      "aux6", "aux7", "aux8", "aux9", "aux10", "aux11", "aux12"],
            "labels": ["Крен", "Тангаж", "Рыск", "Газ"] + [f"AUX {i}" for i in range(1, 13)],
            "order": "A/E/R/T — как rcData[] в прошивке",
            "switches": {"off": RX_LOW_USEC, "on": TX_SWITCH_ON_USEC},
        }

    def set_setting(self, name: str, raw: str):
        entry = self.settings_by_name[name]
        if entry["mode"] == "MODE_STRING":
            # cliSet(): пустая строка и '-' (emptyName) стирают значение, длиннее max —
            # «STRING MUST BE 1-N CHARACTERS», а строка без завершающего нуля вообще
            # не принимается (границы в прошивке заданы как «до следующего параметра»).
            lengths = self.setting_lengths(entry)
            if raw == "":
                return False, f"STRING MUST BE {lengths[0]}-{lengths[1]} CHARACTERS OR '-' FOR EMPTY"
            if raw == "-":
                self.settings[name] = ""
                self._mirror_name_setting(name, "")
                return True, None
            if len(raw) > lengths[1]:
                return False, f"STRING MUST BE {lengths[0]}-{lengths[1]} CHARACTERS OR '-' FOR EMPTY"
            self.settings[name] = raw
            self._mirror_name_setting(name, raw)
            return True, None
        if entry["mode"] == "MODE_LOOKUP" and entry.get("values"):
            value = self._parse_lookup(entry, raw)
            if value is None:
                return False, f"INVALID NAME: {name}"
            self.settings[name] = value
            return True, None
        try:
            value = int(raw, 0)
        except ValueError:
            return False, f"INVALID NAME: {name}"
        lo, hi = entry["min"], entry["max"]
        if not (lo <= value <= hi):
            return False, f"{name} NOT BETWEEN {lo} AND {hi}"
        self.settings[name] = value
        return True, None

    def _mirror_name_setting(self, name: str, value: str) -> None:
        """Имя дрона/пилота живёт и в таблице настроек, и отдельным полем («MSP2_GET_TEXT»).

        В прошивке это одно поле pilotConfig_t: 'set craft_name=…' меняет то же,
        что вкладка пишет через MSP_SET_TEXT.
        """
        if name == "craft_name":
            self.craft_name = value
        elif name == "pilot_name":
            self.pilot_name = value

    @staticmethod
    def setting_lengths(entry) -> tuple:
        """Границы длины строкового параметра: из таблицы, либо по имени."""
        return (int(entry.get("string_min", 1)),
                int(entry.get("string_max", STRING_LENGTHS.get(entry["name"], (1, MAX_NAME_LENGTH))[1])))

    @staticmethod
    def _parse_lookup(entry, raw: str):
        raw = raw.strip().upper()
        for i, v in enumerate(entry["values"]):
            if v.upper() == raw:
                return i
        try:
            n = int(raw)
        except ValueError:
            return None
        return n if 0 <= n < len(entry["values"]) else None

    def format_setting(self, entry):
        value = self.settings[entry["name"]]
        if entry["mode"] == "MODE_STRING":
            return value if value else "-"
        if entry["mode"] == "MODE_LOOKUP" and entry.get("values"):
            if 0 <= value < len(entry["values"]):
                return entry["values"][value]
            return str(value)
        return str(value)

    def default_display(self, entry):
        dv = entry.get("default")
        if dv is None:
            return None
        if entry["mode"] == "MODE_LOOKUP" and entry.get("values") and 0 <= dv < len(entry["values"]):
            return entry["values"][dv]
        if entry["mode"] == "MODE_STRING":
            return dv if dv else "-"
        return str(dv)

    def setting_lines(self, section: str):
        out = []
        for entry in self.settings_list:
            if entry["section"] != section:
                continue
            if entry["mode"] in ("MODE_ARRAY", "MODE_BITSET"):  # упрощение: массивы не печатаем
                continue
            out.append(f"set {entry['name']} = {self.format_setting(entry)}")
        return out

    def changed_setting_lines(self):
        out = []
        for entry in self.settings_list:
            if self.settings[entry["name"]] != self._defaults.get(entry["name"]):
                out.append(f"set {entry['name']} = {self.format_setting(entry)}")
        return out

    # --------------------------------------------------------- dump-помощники
    def banner(self):
        api = self.api_version
        if api.endswith(".0"):
            api = api[:-2]
        return (f"Betaflight / {self.target_name} ({self.board_identifier}) {self.fc_version} "
                f"{self.build_date} / {self.build_time} ({self.git_revision}) MSP API: {api}")

    def features_items(self):
        for name, bit in sorted(self.feature_bits_by_name.items(), key=lambda kv: kv[1]):
            yield name, name in self.features

    @property
    def feature_mask(self):
        mask = 0
        for name in self.features:
            mask |= self.feature_bits_by_name.get(name, 0)
        return mask

    def feature_lines(self, only_diff=False):
        out = []
        for name, bit in sorted(self.feature_bits_by_name.items(), key=lambda kv: kv[1]):
            enabled = name in self.features
            default_enabled = name in FEATURE_DEFAULTS
            if only_diff and enabled == default_enabled:
                continue
            out.append(("feature " if enabled else "feature -") + name)
        return out

    @staticmethod
    def _mask_to_functions(mask):
        return [name for name, bit in SERIAL_FUNCTIONS.items() if mask & bit]

    def _functions_to_mask(self, functions):
        mask = 0
        for name in functions:
            mask |= SERIAL_FUNCTIONS.get(name, 0)
        return mask

    def serial_lines(self):
        """Как printSerial() в cli.c: идентификатор, маска функций и реальные скорости (не индексы)."""
        def baud(idx):
            return BAUD_RATES[idx] if 0 <= idx < len(BAUD_RATES) else "AUTO"

        out = []
        for p in self.serial_ports:
            out.append(f"serial {p['identifier']} {self._functions_to_mask(p['functions'])} "
                       f"{baud(p['msp_baudrate'])} {baud(p['gps_baudrate'])} "
                       f"{baud(p['telemetry_baudrate'])} {baud(p['blackbox_baudrate'])}")
        return out

    def beeper_lines(self, only_diff=False):
        out = []
        for name in self.beeper_names:
            disabled = name in self.beepers_disabled
            out.append(("beeper -" if disabled else "beeper ") + name)
        return out

    # ------------------------------------------------------ режимы (вкладка «РЕЖИМЫ»)
    @staticmethod
    def _empty_mode_range(index: int) -> dict:
        """Пустой слот modeActivationConditions[]: ARM, 900/900, логика ИЛИ, без связи."""
        return {"index": index, "box": "ARM", "aux": 0, "start": CHANNEL_RANGE_MIN,
                "end": CHANNEL_RANGE_MIN, "logic": MODELOGIC_OR, "linked": 0}

    def _default_mode_ranges(self) -> list:
        return [self._empty_mode_range(i) for i in range(MAX_MODE_ACTIVATION_CONDITION_COUNT)]

    def _normalize_mode_ranges(self, raw) -> list:
        """Приводим список слотов к 20 записям с полями logic/linked (старый «флеш» их не знал)."""
        slots = self._default_mode_ranges()
        for item in raw or []:
            try:
                index = int(item.get("index"))
            except (AttributeError, TypeError, ValueError):
                continue
            if not 0 <= index < MAX_MODE_ACTIVATION_CONDITION_COUNT:
                continue
            name = item.get("box") or "ARM"
            if name not in {b["name"] for b in self.boxes}:
                name = "ARM"
            slots[index] = {
                "index": index,
                "box": name,
                "aux": max(0, min(MAX_AUX_CHANNEL_COUNT - 1, int(item.get("aux", 0)))),
                "start": max(CHANNEL_RANGE_MIN, min(CHANNEL_RANGE_MAX, int(item.get("start", CHANNEL_RANGE_MIN)))),
                "end": max(CHANNEL_RANGE_MIN, min(CHANNEL_RANGE_MAX, int(item.get("end", CHANNEL_RANGE_MIN)))),
                "logic": MODELOGIC_AND if int(item.get("logic", MODELOGIC_OR)) else MODELOGIC_OR,
                "linked": int(item.get("linked", 0) or 0),
            }
        return slots

    @staticmethod
    def _range_usable(slot: dict) -> bool:
        """IS_RANGE_USABLE(): startStep < endStep — иначе слот считается пустым."""
        return slot["start"] < slot["end"]

    @staticmethod
    def _range_configured(slot: dict) -> bool:
        return bool(slot["linked"]) or slot["start"] < slot["end"]

    @property
    def active_boxes(self):
        """Режимы, которые уходят в MSP_BOXNAMES/MSP_BOXIDS — initActiveBoxIds() из msp_box.c.

        Прошивка собирает этот список на старте: он зависит от фич, датчиков, микшера,
        протокола моторов и того, что вообще собрано в прошивке (USE_*). Именно по нему
        Configurator строит строки вкладки «Режимы», и по нему же packFlightModeFlags()
        нумерует биты маски в MSP_STATUS. Считаем на лету: включили фичу в CLI — список
        сразу поменялся, как после перезагрузки на железной плате.
        """
        return self._compute_active_boxes()

    def _compute_active_boxes(self):
        names = {b["name"] for b in self.boxes}
        feats = self.features
        sensors = self._active_sensor_mask()
        mixer = getattr(self, "mixer_name", "QUADX")
        enabled = set()

        def add(*items):
            enabled.update(n for n in items if n in names)

        add("ARM", "PREARM")
        if "AIRMODE" not in feats:                 # BOX нужен, только если фичи нет
            add("AIR MODE")
        if self._setting("anti_gravity_gain", 0) and "ANTI_GRAVITY" not in feats:
            add("ANTI GRAVITY")
        if sensors & SENSOR_ACC:
            add("ANGLE", "HORIZON", "HEADFREE", "HEADADJ", "FPV ANGLE MIX", "ACRO TRAINER")
            if "INFLIGHT_ACC_CAL" in feats:
                add("CALIB")
        if sensors & SENSOR_MAG:
            add("MAG")
        if "GPS" in feats:
            if "3D" not in feats and mixer not in AIRPLANE_MIXERS:
                add("GPS RESCUE")
            add("GPS BEEP SATELLITE COUNT")
        add("FAILSAFE")
        if mixer in AIRPLANE_MIXERS:
            add("PASSTHRU")
        add("BEEPER", "BEEPER MUTE")
        if "LED_STRIP" in feats:
            add("LEDLOW")
        add("BLACKBOX", "BLACKBOX ERASE")           # USE_BLACKBOX + USE_FLASHFS
        if "3D" in feats:
            add("3D DISABLE / SWITCH")
        if self._setting("motor_pwm_protocol", 7) in (5, 6, 7, 8):   # DShot → FLIP OVER AFTER CRASH
            add("FLIP OVER AFTER CRASH")
        if "SERVO_TILT" in feats:
            add("CAMSTAB")
        add("OSD DISABLE")
        if "TELEMETRY" in feats:
            add("TELEMETRY")
        if mixer == "CUSTOMAIRPLANE":
            add("SERVO1", "SERVO2", "SERVO3")
        add("CAMERA CONTROL 1", "CAMERA CONTROL 2", "CAMERA CONTROL 3")   # USE_RCDEVICE
        add("VTX PIT MODE", "VTX CONTROL DISABLE")                        # USE_VTX_SMARTAUDIO
        add("PARALYZE")
        # USER1..USER4 попадают в список, только если их слушает pinio (у нас pinio не настроен),
        # а PID AUDIO — только в сборках с USE_PID_AUDIO (в этой сборке его нет).
        add("LAUNCH CONTROL")                       # USE_LAUNCH_CONTROL
        if self._setting("msp_override_channels_mask", 0):                # USE_RX_MSP_OVERRIDE
            add("MSP OVERRIDE")
        add("STICK COMMANDS DISABLE", "READY")
        add("LAP TIMER RESET")                      # USE_GPS_LAP_TIMER
        return [b["name"] for b in sorted(self.boxes, key=lambda b: b["permanent_id"])
                if b["name"] in enabled]

    def box_permanent_id(self, name: str) -> int:
        box = next((b for b in self.boxes if b["name"] == name), None)
        return box["permanent_id"] if box else 0

    def set_mode_range(self, index, perm_id, aux, start, end, logic=MODELOGIC_OR, linked=0) -> bool:
        """Запись слота так, как это делает MSP_SET_MODE_RANGE / CLI 'aux'."""
        if not 0 <= index < MAX_MODE_ACTIVATION_CONDITION_COUNT:
            return False
        name = self.box_name_by_permanent_id.get(perm_id)
        if name is None:
            return False
        if not 0 <= aux < MAX_AUX_CHANNEL_COUNT:
            return False
        self.mode_ranges[index] = {
            "index": index,
            "box": name,
            "aux": aux,
            "start": max(CHANNEL_RANGE_MIN, min(CHANNEL_RANGE_MAX, int(start))),
            "end": max(CHANNEL_RANGE_MIN, min(CHANNEL_RANGE_MAX, int(end))),
            "logic": MODELOGIC_AND if int(logic) else MODELOGIC_OR,
            "linked": int(linked or 0),
        }
        return True

    def clear_mode_range(self, index) -> bool:
        if not 0 <= index < MAX_MODE_ACTIVATION_CONDITION_COUNT:
            return False
        self.mode_ranges[index] = self._empty_mode_range(index)
        return True

    def _mode_range_step(self, value: int) -> int:
        return max(0, min((CHANNEL_RANGE_MAX - CHANNEL_RANGE_MIN) // MODE_RANGE_STEP,
                          (int(value) - CHANNEL_RANGE_MIN) // MODE_RANGE_STEP))

    def _is_range_active(self, aux: int, start: int, end: int) -> bool:
        """isRangeActive() из rc_modes.c: канал AUX попадает в [start, end)."""
        if start >= end:
            return False
        channels = self._rc_channel_values()
        index = NON_AUX_CHANNEL_COUNT + int(aux)
        if not 0 <= index < len(channels):
            return False
        value = max(CHANNEL_RANGE_MIN, min(CHANNEL_RANGE_MAX - 1, int(channels[index])))
        return start <= value < end

    def active_mode_names(self) -> set:
        """updateActivatedModes() из rc_modes.c: какие режимы сейчас включены тумблерами.

        Логика И/ИЛИ, связи (linkedTo) и «залипающий» PARALYZE — как в прошивке.
        """
        and_mask, new_mask = set(), set()

        def update(mode: str, active: bool, logic: int) -> None:
            if mode in and_mask or mode not in new_mask:
                if logic != MODELOGIC_AND:            # OR-условие
                    if active:
                        and_mask.discard(mode)
                        new_mask.add(mode)
                else:                                 # AND-условие
                    and_mask.add(mode)
                    if not active:
                        new_mask.add(mode)

        for slot in sorted(self.mode_ranges, key=lambda s: s["index"]):
            if slot["linked"]:
                continue
            if not self._range_configured(slot):
                continue
            update(slot["box"], self._is_range_active(slot["aux"], slot["start"], slot["end"]),
                   slot["logic"])
        for slot in sorted(self.mode_ranges, key=lambda s: s["index"]):
            if not slot["linked"]:
                continue
            linked_name = self.box_name_by_permanent_id.get(slot["linked"], "ARM")
            active = (linked_name in and_mask) != (linked_name in new_mask)
            update(slot["box"], active, slot["logic"])
        return and_mask ^ new_mask

    def rc_mode_mask(self) -> int:
        """Маска режимов для MSP_STATUS (CONFIG.mode).

        Бит — позиция режима в списке MSP_BOXNAMES: именно так его читает Configurator
        (`bit_check(config.mode, mode.index)`, index — порядок в AUX_CONFIG), поэтому
        подсветка «включён / выключен» на вкладке «Режимы» совпадает с тумблерами.
        """
        names = self.active_mode_names()
        mask = 0
        for position, name in enumerate(self.active_boxes):
            if name in names:
                mask |= 1 << position
        return mask

    def modes_state(self) -> dict:
        """Снимок режимов для монитора эмулятора (/api/state)."""
        active = self.active_mode_names()
        slots = []
        for slot in sorted(self.mode_ranges, key=lambda s: s["index"]):
            linked_name = self.box_name_by_permanent_id.get(slot["linked"], "") if slot["linked"] else ""
            slots.append({
                "index": slot["index"],
                "box": slot["box"],
                "aux": slot["aux"],
                "start": slot["start"],
                "end": slot["end"],
                "logic": "И" if slot["logic"] == MODELOGIC_AND else "ИЛИ",
                "logic_value": int(slot["logic"]),          # как в протоколе: 0 ИЛИ / 1 И
                "linked": linked_name,
                "linked_perm": int(slot["linked"] or 0),    # permanentId связанного режима
                "perm": self.box_permanent_id(slot["box"]),  # permanentId режима слота
                "configured": self._range_configured(slot),
                "listed": slot["box"] in self.active_boxes,
                "active": slot["box"] in active and slot["box"] in self.active_boxes,
            })
        return {
            "slots_total": MAX_MODE_ACTIVATION_CONDITION_COUNT,
            "slots_used": sum(1 for s in slots if s["configured"]),
            "slots": slots,
            "active": sorted(active),
            "mask": self.rc_mode_mask(),
        }

    def aux_lines(self, only_diff: bool = False):
        """Строки 'aux i permId aux start end logic linkedPermanentId' — printAux() из cli.c.

        Прошивка печатает ВСЕ 20 слотов (в том числе пустые: 'aux 3 0 0 900 900 0 0'),
        а в diff-режиме — только настроенные (cliDumpPrintLinef с DO_DIFF).
        """
        out = []
        for r in sorted(self.mode_ranges, key=lambda r: r["index"]):
            if only_diff and not (self._range_configured(r) or r["box"] != "ARM"):
                continue
            out.append(f"aux {r['index']} {self.box_permanent_id(r['box'])} {r['aux']} "
                       f"{r['start']} {r['end']} {r['logic']} {r['linked']}")
        return out

    def servo_lines(self, only_diff=False):
        """Строки 'servo i min max middle rate forward' — printServo() из cli.c.

        Формат ровно как в прошивке (4.5.4): ``servo %u %d %d %d %d %d``.
        """
        defaults = self._default_servo_params()
        out = []
        for index, params in enumerate(self.servo_params):
            if only_diff and params == defaults[index]:
                continue
            out.append(f"servo {index} {params['min']} {params['max']} {params['middle']} "
                       f"{params['rate']} {params['forwardFromChannel']}")
        return out

    def smix_lines(self, only_diff=False):
        """Строки 'smix ...' — printServoMix() из cli.c.

        Как и в прошивке, список обрывается на первом правиле с rate == 0:
        заводские правила (нули) поэтому не печатаются вовсе.
        """
        out = []
        for index, rule in enumerate(self.servo_mix_rules):
            if rule["rate"] == 0:
                break
            if only_diff and self._default_servo_mix_rules()[index] == rule:
                continue
            out.append(f"smix {index} {rule['targetChannel']} {rule['inputSource']} {rule['rate']} "
                       f"{rule['speed']} {rule['min']} {rule['max']} {rule['box']}")
        return out

    def smix_reverse_lines(self, only_diff=False):
        """Строки 'smix reverse <servo> <source> r' — направления сервоприводов."""
        out = []
        for index, params in enumerate(self.servo_params):
            for source in range(INPUT_SOURCE_COUNT):
                if params["reversedSources"] & (1 << source):
                    out.append(f"smix reverse {index} {source} r")
        return out

    # ------------------------------------------------------------------ ресурсы платы
    def _default_resources(self) -> dict:
        """Заводские назначения пинов — pgResetFn_* / target-файл платы."""
        return {name: list(pins) for name, pins in self.RESOURCE_OWNERS}

    def _resources_from_json(self, data) -> dict:
        """Ресурсы из «флеша»: битые строки отбрасываем, длины выравниваем по таблице."""
        table = self._default_resources()
        if not isinstance(data, dict):
            return table
        for name, pins in table.items():
            saved = data.get(name)
            if not isinstance(saved, list):
                continue
            for i in range(min(len(pins), len(saved))):
                value = saved[i]
                if value is None:
                    pins[i] = None
                elif isinstance(value, str):
                    kind, pin = self.parse_pin(value)
                    pins[i] = pin if kind == "pin" else None
        return table

    @staticmethod
    def parse_pin(text) -> tuple:
        """strToPin() из cli.c: 'NONE' — освобождение, 'A00'…'D15' — пин.

        Возвращает («none», None) | («pin», 'A04') | (None, None), если разобрать не удалось.
        Порты платы — A…D (у STM32F405 четыре порта), пины 0…15: 'E01' прошивка тоже
        принимает по формату, но такого IO на плате нет — получается PARSING FAILED.
        """
        if text is None:
            return (None, None)
        word = str(text).strip().upper()
        if word == "NONE":
            return ("none", None)
        if len(word) >= 2 and word[0] in "ABCD" and word[1:].isdigit():
            number = int(word[1:])
            if 0 <= number <= 15:
                return ("pin", f"{word[0]}{number:02d}")
        return (None, None)

    def resource_owner_names(self) -> list:
        return [name for name, _pins in self.RESOURCE_OWNERS]

    def resource_slots(self, owner: str) -> int:
        """Число слотов владельца (RESOURCE_VALUE_MAX_INDEX): у одиночных — 1."""
        for name, pins in self.RESOURCE_OWNERS:
            if name == owner:
                return max(1, len(pins))
        return 0

    def resource_dump_lines(self, hide_unused=True, only_diff=False) -> list:
        """printResource(): дамп таблицы ресурсов в порядке таблицы прошивки.

        `resource` без аргументов зовёт printResource(DUMP_MASTER | HIDE_UNUSED) — только
        назначенные пины; `dump` — без HIDE_UNUSED, поэтому свободные слоты идут строками
        «resource MOTOR 5 NONE». В diff печатаются только слоты, отличающиеся от заводских.
        """
        defaults = self._default_resources()
        lines = []
        for name, pins in self.RESOURCE_OWNERS:
            for i, pin in enumerate(self.resources[name]):
                if only_diff and pin == defaults[name][i]:
                    continue
                if pin:
                    lines.append(f"resource {name} {i + 1} {pin}")
                elif not hide_unused:
                    lines.append(f"resource {name} {i + 1} NONE")
        return lines

    def resource_show_lines(self) -> list:
        """`resource show`: «Currently active IO resource assignments» — как ioRecs[].

        Индекс печатается у многоместных владельцев (SERIAL_TX 1, MOTOR 2), у одиночных
        его нет — ровно как printResourceOwner()/IOInit() считают RESOURCE_INDEX.
        """
        lines = ["Currently active IO resource assignments:", "(reboot to update)", "-" * 20]
        for name, pins in self.RESOURCE_OWNERS:
            for i, pin in enumerate(self.resources[name]):
                if pin:
                    index = f" {i + 1}" if len(pins) > 1 else ""
                    lines.append(f"{pin}: {name}{index}")
        return lines

    def resource_note_lines(self, owner: str, index: int, pin: str) -> list:
        """resourceCheck(): пин уже занят — печатаем NOTE и освобождаем свой же дубль.

        Как в прошивке: дубль внутри того же владельца освобождается («… disabled»),
        занятость чужим владельцем только помечается — прошивка пин не отбирает.
        """
        notes = []
        if not pin:
            return notes
        for name, pins in self.RESOURCE_OWNERS:
            for i, current in enumerate(self.resources[name]):
                if current != pin:
                    continue
                cleared = False
                if name == owner:
                    if i == index:
                        continue
                    self.resources[name][i] = None
                    cleared = True
                number = f" {i + 1}" if len(pins) > 1 else ""
                # Прошивка печатает это через cliPrintf("\r\nNOTE: …") — с пустой строкой
                # перед предупреждением.
                note = f"\r\nNOTE: {pin} already assigned to {name}{number}"
                if cleared:
                    note += f". {name}{number} disabled"
                note += "."
                notes.append(note)
        return notes

    def timer_dump_lines(self) -> list:
        """`timer` без аргументов и секция «# timer» в dump: printTimer(DUMP_MASTER, NULL).

        На каждый пин две строки — задание и расшифровка канала, как printTimerDetails():
            timer A00 AF1
            # pin A00: TIM2 CH1 (AF1)
        """
        lines = []
        for pin in sorted(self.RESOURCE_PIN_TIMERS):
            lines.extend(self.timer_opt_lines(pin))
        return lines

    def timer_opt_lines(self, pin: str) -> list:
        """printTimerDetails() для одного пина: у пина без таймера — «timer <пин> NONE»."""
        timer = self.RESOURCE_PIN_TIMERS.get(pin)
        if not timer:
            return [f"timer {pin} NONE"]
        name, channel, af = timer
        return [f"timer {pin} AF{af}", f"# pin {pin}: {name} CH{channel} (AF{af})"]

    def dma_dump_lines(self, hide_unused=True) -> list:
        """printDmaopt(): `dma` без аргументов — DUMP_MASTER | HIDE_UNUSED, в `dump` — без него.

        Задание с DMA печатается парой строк (задание и комментарий с потоком и каналом
        из dmaGetChannelSpecByPeripheral), пустой слот — «dma ADC 2 NONE»; в конце идут
        задания DMA для пинов таймеров — «dma pin A00 NONE» (printTimerDmaoptDetails).
        """
        lines = []
        for device, index, spec in self.RESOURCE_DMA_OPT:
            if spec is None:
                if not hide_unused:
                    lines.append(f"dma {device} {index} NONE")
                continue
            controller, stream, channel = spec
            lines.append(f"dma {device} {index} 0")
            lines.append(f"# {device} {index}: DMA{controller} Stream {stream} Channel {channel}")
        if not hide_unused:
            for pin in sorted(self.RESOURCE_PIN_TIMERS):
                lines.append(f"dma pin {pin} NONE")
        return lines

    def dma_pin_lines(self, pin: str) -> list:
        """Задание DMA для пина таймера — printTimerDmaoptDetails(): у платы его нет."""
        if pin in self.RESOURCE_PIN_TIMERS:
            return [f"dma pin {pin} NONE"]
        return []

    def dma_opt_lines(self, device: str, index: int) -> list:
        """Строки текущего задания DMA — printPeripheralDmaoptDetails() для `dma <устр> <инд>`."""
        for name, number, spec in self.RESOURCE_DMA_OPT:
            if name.upper() != device.upper() or number != index:
                continue
            if spec is None:
                return [f"dma {name} {number} NONE"]
            controller, stream, channel = spec
            return [f"dma {name} {number} 0",
                    f"# {name} {number}: DMA{controller} Stream {stream} Channel {channel}"]
        return []

    def timer_show_lines(self) -> list:
        """`timer show`: showTimers() — активные таймеры, каналы и их владельцы."""
        owners = {}
        for name, pins in self.RESOURCE_OWNERS:
            for i, pin in enumerate(self.resources[name]):
                timer = self.RESOURCE_PIN_TIMERS.get(pin) if pin else None
                if timer:
                    owners[(timer[0], timer[1])] = (name, i + 1 if len(pins) > 1 else 0)
        lines = ["", "Currently active Timers:", "-" * 23]
        for timer in self.RESOURCE_TIMERS:
            used = sorted((channel, owner) for (name, channel), owner in owners.items() if name == timer)
            if not used:
                lines.append(f"{timer}: FREE")
                continue
            lines.append(f"{timer}:")
            for channel, (name, index) in used:
                suffix = f" {index}" if index else ""
                lines.append(f"    CH{channel} : {name}{suffix}")
        return lines

    def dma_show_lines(self) -> list:
        """`dma show`: showDma() — «DMA<контроллер> Stream <поток>: <владелец> [индекс]»."""
        lines = ["", "Currently active DMA:", "-" * 20]
        for controller in (1, 2):
            for stream in range(8):
                owner = self.RESOURCE_DMA.get((f"DMA{controller}", stream))
                if owner:
                    lines.append(f"DMA{controller} Stream {stream}: {owner[0]} {owner[1]}")
                else:
                    lines.append(f"DMA{controller} Stream {stream}: FREE")
        return lines

    @property
    def uid_hex(self):
        return "".join(f"{u:08x}" for u in self.uids)

    # ------------------------------------------------------------ «флеш» (EEPROM)
    _EEPROM_SECTIONS = (
        "sensor_config", "config_payloads",
        "features", "beepers_disabled", "serial_ports", "mode_ranges", "rx_map", "rxfail",
        "servo_params", "servo_mix_rules", "mixer_name",
        "pid_profiles", "rate_profiles",
        "craft_name", "pilot_name", "board_name", "manufacturer_id", "pid_profile_names",
        "rate_profile_names", "current_profile", "current_rate_profile",
        "battery_profiles", "active_battery_profile",
        "uids", "signature",
        "vtx_table", "vtx_pit_mode", "vtx_activation", "acc_calibrated",
        "osd_config", "adjustment_ranges",
        # таблицы команд CLI 4.5.4: rxrange / led / color / mode_color / mmix / beacon
        "rx_ranges", "led_configs", "led_colors", "mode_colors", "custom_motor_mix",
        "dshot_beacon_disabled", "dshot_beacon_tone", "resources",
    )

    def eeprom_state(self) -> dict:
        state = {"version": 1, "settings": dict(self.settings)}
        for name in self._EEPROM_SECTIONS:
            value = getattr(self, name, None)
            if isinstance(value, set):
                value = sorted(value)
            if isinstance(value, tuple):
                value = list(value)
            if isinstance(value, (bytes, bytearray)):
                value = bytes(value).hex()          # signature: во «флеше» — hex-строка
            if isinstance(value, dict):
                value = {
                    str(k): (v.hex() if isinstance(v, (bytes, bytearray)) else v) for k, v in value.items()
                }
            state[name] = value
        return state

    def load_eeprom(self):
        """Читает конфигурацию из файла при старте — как прошивка из флеша."""
        if not self.eeprom_path or not self.eeprom_path.exists():
            return False
        try:
            state = json.loads(self.eeprom_path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            self._log(f"[FC] «флеш» {self.eeprom_path} повреждён ({exc}) — стартую с заводскими настройками")
            return False
        for name, value in (state.get("settings") or {}).items():
            entry = self.settings_by_name.get(name)
            if entry is None:
                continue
            if entry["mode"] == "MODE_STRING" and isinstance(value, str):
                self.settings[name] = value            # строки (имена) — как есть
            elif isinstance(value, (int, float)):
                self.settings[name] = int(value)
        for name in self._EEPROM_SECTIONS:
            if name not in state or state[name] is None:
                continue
            if name == "serial_ports":
                # Записи из старого «флеша» могли не иметь скоростей — добираем значения
                # как в serialConfig_t по умолчанию, чтобы ответ вкладке «Порты» не падал.
                self.serial_ports = [
                    {
                        "identifier": int(port.get("identifier", 0)),
                        "functions": list(port.get("functions") or []),
                        "msp_baudrate": int(port.get("msp_baudrate", 0)),
                        "gps_baudrate": int(port.get("gps_baudrate", 4)),
                        "telemetry_baudrate": int(port.get("telemetry_baudrate", 0)),
                        "blackbox_baudrate": int(port.get("blackbox_baudrate", 5)),
                    }
                    for port in state[name]
                ]
                continue
            if name == "vtx_table":
                # Таблицу VTX восстанавливаем явно: в ней лежат строки и вложенные списки,
                # а общий разбор «флеша» умеет только числа и hex-строки.
                self.vtx_table = self._vtx_table_from_json(state[name])
                continue
            if name == "osd_config":
                # Настройки OSD — тоже со списками (места элементов, таймеры): разбираем явно и
                # добираем недостающее заводскими значениями, чтобы старый «флеш» не ломал вкладку.
                self.osd_config = self._osd_config_from_json(state[name])
                continue
            if name == "adjustment_ranges":
                # Слоты корректировок — список словарей: общий разбор умеет только числа,
                # поэтому читаем сами и добираем недостающие слоты.
                self.adjustment_ranges = self._adjustment_ranges_from_json(state[name])
                continue
            if name == "mode_ranges":
                # Слоты режимов: старый «флеш» знал только index/box/aux/start/end — добираем
                # остальные слоты и поля (logic/linked), иначе вкладка «Режимы» увидела бы
                # «дырки» в массиве из 20 записей.
                self.mode_ranges = self._normalize_mode_ranges(state[name])
                continue
            if name == "battery_profiles":
                # Таблица профилей батареи — список словарей: разбираем сами, битое
                # отбрасываем, недостающие профили добираем заводскими.
                self.battery_profiles = self._battery_profiles_from_json(state[name])
                continue
            if name == "resources":
                # Назначения пинов: битые строки отбрасываем, длины выравниваем по таблице,
                # чтобы старый «флеш» не ломал `resource`.
                self.resources = self._resources_from_json(state[name])
                continue
            if name == "vtx_activation":
                # Слоты `vtx`: битое отбрасываем, недостающие добираем нулевыми,
                # чтобы старый «флеш» не ломал команду.
                self.vtx_activation = self._vtx_activation_from_json(state[name])
                continue
            if name in ("rx_ranges", "led_configs", "led_colors", "mode_colors", "custom_motor_mix"):
                # Таблицы команд CLI (rxrange/led/color/mmix): числа лежат вложенными списками,
                # поэтому разбираем их сами — битое отбрасываем, недостающее добираем заводским.
                self._load_cli_table(name, state[name])
                continue
            value = state[name]
            current = getattr(self, name, None)
            if isinstance(current, (bytes, bytearray)) and isinstance(value, str):
                value = (bytes.fromhex(value)
                         if len(value) % 2 == 0 and all(c in "0123456789abcdef" for c in value.lower())
                         else current)          # подпись платы: hex во «флеше», байты в поле
            elif isinstance(current, set):
                value = set(value)
            elif isinstance(current, tuple):
                value = tuple(value)
            elif isinstance(current, dict):
                value = {
                    int(k) if k.lstrip("-").isdigit() else k: (bytes.fromhex(v) if isinstance(v, str) and len(v) % 2 == 0 and all(c in "0123456789abcdef" for c in v.lower()) else v)
                    for k, v in value.items()
                }
            setattr(self, name, value)
        # Пищалка из старых «флешей»: сборки до 2026-10-08-b шли с «заводским» набором
        # выключенных сигналов (RX_LOST, USB, CAM_CONNECTION_CLOSE, RC_SMOOTHING_INIT_FAIL),
        # хотя в прошивке beeper_off_flags = 0 (pg/beeper.c), а DShot-бикон выключен по
        # RX_LOST/RX_SET (DEFAULT_DSHOT_BEACON_OFF_FLAGS, io/beeper.h). Если во «флеше» лежит
        # ровно старый заводской набор — подменяем на прошивочный: пользователь его не выбирал.
        if self.beepers_disabled == {"RX_LOST", "USB", "CAM_CONNECTION_CLOSE", "RC_SMOOTHING_INIT_FAIL"}:
            self.beepers_disabled = set()
        if not self.dshot_beacon_disabled:
            self.dshot_beacon_disabled = {"RX_LOST", "RX_SET"}
        # В старых «флешах» для этих кодов лежало эхо записанного payload; теперь ответы
        # строятся из настроек, поэтому эхо не должно подменять их. Чистим сразу все такие
        # коды — иначе, например, включённый во вкладке «Конфигурация» SOFTSERIAL/SERVO_TILT
        # «не сохранялся»: чтение отдавало старое эхо.
        stale = [code for code in list(self.config_payloads) if code in self.state_based_read_codes()]
        for code in stale:
            self.config_payloads.pop(code, None)
        if stale:
            self._log("[FC] «флеш»: устаревшее эхо для кодов " +
                      ", ".join(f"0x{c:04X}" for c in sorted(stale)) + " отброшено")
        self.sync_soft_serial_ports()
        self.check_feature_bits()
        # Имена дрона и пилота лежат и в таблице настроек, и отдельными полями
        # (pilotConfig_t): приводим второе к первому, чтобы CLI и MSP не расходились.
        for name in ("craft_name", "pilot_name"):
            if isinstance(self.settings.get(name), str):
                self._mirror_name_setting(name, self.settings[name])
        self.eeprom_loaded = True
        self._log(f"[FC] конфигурация загружена из «флеша»: {self.eeprom_path}")
        return True

    @classmethod
    def _vtx_table_from_json(cls, data: dict) -> dict:
        """Восстановление таблицы VTX из «флеша» с проверкой размеров."""
        table = cls._empty_vtx_table()
        table["bands"] = max(0, min(VTX_TABLE_MAX_BANDS, int(data.get("bands", 0))))
        table["channels"] = max(0, min(VTX_TABLE_MAX_CHANNELS, int(data.get("channels", 0))))
        table["powerlevels"] = max(0, min(VTX_TABLE_MAX_POWER_LEVELS, int(data.get("powerlevels", 0))))
        names = data.get("band_names") or []
        letters = data.get("band_letters") or []
        factory = data.get("is_factory") or []
        freqs = data.get("frequencies") or []
        values = data.get("power_values") or []
        labels = data.get("power_labels") or []
        for i in range(min(len(names), VTX_TABLE_MAX_BANDS)):
            table["band_names"][i] = cls._pad(str(names[i]), VTX_TABLE_BAND_NAME_LENGTH)
        for i in range(min(len(letters), VTX_TABLE_MAX_BANDS)):
            table["band_letters"][i] = str(letters[i])[:1]
        for i in range(min(len(factory), VTX_TABLE_MAX_BANDS)):
            table["is_factory"][i] = bool(factory[i])
        for i, row in enumerate(freqs[:VTX_TABLE_MAX_BANDS]):
            for j, freq in enumerate(list(row)[:VTX_TABLE_MAX_CHANNELS]):
                table["frequencies"][i][j] = max(0, min(5999, int(freq)))
        for i in range(min(len(values), VTX_TABLE_MAX_POWER_LEVELS)):
            table["power_values"][i] = max(0, min(65535, int(values[i])))
        for i in range(min(len(labels), VTX_TABLE_MAX_POWER_LEVELS)):
            table["power_labels"][i] = cls._pad(str(labels[i]).upper(), VTX_TABLE_POWER_LABEL_LENGTH)
        return table

    def save_to_eeprom(self):
        """EEPROM_WRITE / 'save': пишем активную конфигурацию в «флеш».

        Как и у настоящего FC, пишется всё состояние сразу, а при следующем старте
        прошивка поднимает его из флеша.
        """
        if not self.eeprom_path:
            self._log("[FC] EEPROM_WRITE: файл «флеша» не задан, конфигурация живёт только в памяти")
            return False
        try:
            self.eeprom_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.eeprom_path.with_suffix(self.eeprom_path.suffix + ".tmp")
            tmp.write_text(json.dumps(self.eeprom_state(), indent=1, ensure_ascii=False), encoding="utf-8")
            tmp.replace(self.eeprom_path)  # атомарная замена: «запись во флеш» не может побиться
        except OSError as exc:
            self._log(f"[FC] не удалось записать «флеш» {self.eeprom_path}: {exc}")
            return False
        self._log(f"[FC] конфигурация записана в «флеш»: {self.eeprom_path}")
        return True

    def reload_after_reboot(self):
        """Ребут FC: конфигурация поднимается из «флеша», несохранённое теряется.

        Если «флеш» отключён (--eeprom none), состояние остаётся в памяти процесса —
        так эмулятор ведёт себя как «FC без флеша».
        """
        if not self.eeprom_path:
            return
        self.reset_to_defaults()
        if not self.load_eeprom():
            self._log("[FC] «флеш» пуст: после перезагрузки — заводские настройки")
        self.init_vtx_device()
        self._battery_volatile_reset()

    def wipe_eeprom(self):
        """Полный сброс: как «Erase» флеша перед первым запуском."""
        if self.eeprom_path and self.eeprom_path.exists():
            try:
                self.eeprom_path.unlink()
            except OSError:
                pass
        self._defaults_restore()
        self.init_vtx_device()

    def _defaults_restore(self):
        self.reset_to_defaults()
        self.sync_soft_serial_ports()
        self.config_payloads = {}
        self.sensor_config = {
            "acc_hardware": SENSOR_AUTO,
            "baro_hardware": SENSOR_AUTO,
            "mag_hardware": SENSOR_NONE,
            "sonar_hardware": RANGEFINDER_NONE,
        }
        self.acc_calibrated = False   # после сброса прошивка снова просит калибровку
        self.acc_calibration_until = 0.0
        self.features = set(FEATURE_DEFAULTS)
        self.beepers_disabled = set()          # как при старте: beeper_off_flags = 0
        self.craft_name = STRING_DEFAULTS["craft_name"]
        self.pilot_name = STRING_DEFAULTS["pilot_name"]

    def _apply_overrides(self):
        """Небольшая таблица «хорошо известных» заводских значений.

        Большая часть defaults извлекается автоматически из pgResetFn_*/PG_RESET_TEMPLATE,
        но некоторые значения задаются выражениями, которые не разбираются генератором.
        Значения ниже соответствуют стоковой прошивке Betaflight 4.5.x.
        """
        override_path = Path(__file__).resolve().parent / "data" / "fc_overrides.json"
        if not override_path.exists():
            return
        for name, value in json.loads(override_path.read_text()).items():
            entry = self.settings_by_name.get(name)
            if entry is None:
                continue
            if isinstance(value, str) and entry["mode"] == "MODE_LOOKUP" and entry.get("values"):
                value = self._parse_lookup(entry, value)
                if value is None:
                    continue
            self.settings[name] = value
            entry["default"] = value

    def _battery_volatile_reset(self):
        """Счётчик мА·ч и тест моторов живут только до перезагрузки — как на железе."""
        self._fs_init_state()               # демо failsafe после ребута выключено: борт на столе
        self.mah_drawn = 0.0
        self._mah_t = None
        self.motor_disarmed = [1000.0] * MAX_SUPPORTED_MOTORS   # motor_disarmed[] в нуле → внешние 1000
        self.motor_output = list(self.motor_disarmed)
        self.motor_rpm = [0.0] * MAX_SUPPORTED_MOTORS
        self.motor_temp_c = [self.MOTOR_AMBIENT_C] * MAX_SUPPORTED_MOTORS
        self.motor_current_a = [0.0] * MAX_SUPPORTED_MOTORS
        self.motor_mah = [0.0] * MAX_SUPPORTED_MOTORS
        self.motor_voltage_v = 0.0
        self._motors_t = time.time()
        self.motor_test_arming_block = False
        # Полёт — не конфигурация: после перезагрузки борт снова «на столе».
        self.sim_flight = False
        self.sim_flight_phase = "idle"
        self.sim_flight_since = 0.0
        self.sim_flight_note = ""
        self.sim_pack_events = 0

    def reboot(self, bootloader=False):
        self.pending_reboot = True
        self.pending_bootloader = bootloader

    # ---------------------------------------------------------- режим ЗУ (MSC)
    def enter_msc(self, mode: int = MSP_REBOOT_MSC) -> None:
        """Перезагрузка «в режим запоминающего устройства» — кнопка во вкладке «Чёрный ящик».

        Настоящий борт после этого перестаёт отвечать по MSP и появляется на компьютере
        как флешка. У эмулятора флешки нет, поэтому содержимое «флеша» отдаёт страница
        /msc (файлы по пролётам + один общий дамп), а MSP перестаёт отвечать — как у
        настоящего борта, который «ушёл» в режим ЗУ.
        """
        self.msc_mode = True
        self.msc_deadline = time.time() + MSC_SECONDS
        self.reboot()
        self._log(f"[MSC] режим запоминающего устройства включён (перезагрузка {mode}): "
                  f"файлы «флеша» — на странице /msc, MSP не отвечает; "
                  f"выйти — кнопка на странице или перезапуск эмулятора")

    def exit_msc(self, reason: str = "") -> None:
        """Выход из режима ЗУ: как «передёрнуть питание» у настоящего борта."""
        if not self.msc_mode:
            return
        self.msc_mode = False
        self.msc_deadline = 0.0
        self._log("[MSC] режим запоминающего устройства выключен"
                  + (f" ({reason})" if reason else "") + ": MSP снова отвечает")

    def msc_active(self) -> bool:
        """Активен ли режим ЗУ (с автовыходом по времени, чтобы эмулятор не «залипал»)."""
        if self.msc_mode and self.msc_deadline and time.time() > self.msc_deadline:
            self.exit_msc("истекло время ожидания")
        return self.msc_mode

    # ----------------------------------------------------------------- CLI
    def note_msp(self, code: int, name: str, req_len: int, resp_len: int):
        self.stats[code] = self.stats.get(code, 0) + 1
        self.recent_msp.append((time.time(), code, name, req_len, resp_len))
        del self.recent_msp[:-40]

    def cli_execute(self, line: str) -> str:
        """CLI-строка через MSP2_CLI_COMMAND: текст с \\r\\n, без приглашения."""
        text = self._cli.execute_line(line)
        self._cli_log.extend(text.split("\r\n"))
        del self._cli_log[:-400]              # держим не больше 400 строк вывода
        self.recent_cli.append((time.time(), line, text))
        del self.recent_cli[:-40]
        if self.pending_reboot:
            # 'save'/'exit'/'defaults' перезагружают FC: связь оборвётся
            pass
        return text

    def terminal(self):
        """Терминал «сырого» CLI-порта для текущего соединения."""
        from .cli import CliTerminal
        return CliTerminal(self)

    # -------------------------------------------------------------- MSP API
    def _handle_write(self, code: int, payload: bytes, version: int) -> bytes:
        """Сквозная запись конфигурации из вкладок.

        payload запоминается под кодом «чтения» и отдаётся на GET, если размер совпадает
        с генерируемым ответом — страховка от расхождения раскладок между версиями API.
        """
        read_code = WRITE_TO_READ[code]
        self.config_payloads[read_code] = bytes(payload)
        self._log(f"[MSP] 0x{code:04X}: конфигурация сохранена ({len(payload)} байт -> 0x{read_code:04X})")
        return b""

    def handle(self, code: int, payload: bytes, version: int):
        """Возвращает payload ответа (bytes) или None, если ответа нет."""
        if self.acc_calibration_until and time.time() >= self.acc_calibration_until:
            self.acc_calibration_until = 0.0
            self.acc_calibrated = True
            self._log("[FC] калибровка акселерометра завершена")
        # 1) Запись конфигурации из вкладки: любой MSP_SET_* из таблицы пар сохраняется.
        # 45 (RX_CONFIG) и 91 (ADVANCED_CONFIG) обрабатываются по смыслу — см. ниже
        # 220 (SENSOR_ALIGNMENT) и 224 (COMPASS_CONFIG) разбираются по смыслу ниже,
        # 45 (RX_CONFIG) и 91 (ADVANCED_CONFIG) — тоже.
        # 76 (FAILSAFE), 223 (GPS_CONFIG) и 225 (GPS_RESCUE) тоже разбираются по смыслу — ниже:
# вкладка «GPS» пишет провайдера/SBAS/Galileo, и CLI должен видеть то же самое (get gps_*).
        # 93/95/202/204/218/222 (фильтры, PID, PID Advanced, Rates, дедбенды, параметры моторов)
        # разбираются по смыслу: значения уходят в те же настройки, что видит CLI, — иначе вкладка
        # «PID настройки» показывала бы одно, а CLI и «Моторы» другое.
        if code in WRITE_TO_READ and code not in SEMANTIC_SET_CODES:
            return self._handle_write(code, payload, version)

        # Смысловая запись: ответ на чтение этого блока строится из состояния, поэтому старое
        # эхо (из «флеша» прежней сборки или из прежней записи) убираем — иначе вкладка
        # увидела бы прежние значения и «изменения не сохранялись».
        read_after_write = WRITE_TO_READ.get(code) or STATE_PAIRS_EXTRA.get(code)
        if read_after_write is not None and read_after_write in self.config_payloads:
            self.config_payloads.pop(read_after_write, None)
            self._log(f"[MSP] 0x{code:04X}: старое эхо 0x{read_after_write:04X} удалено — "
                      f"ответ строится из состояния FC")

        handler = self._handlers().get(code)
        if handler is None:
            stored = self.config_payloads.get(code)
            if stored is not None:  # конфигурация, записанная вкладкой, — отдаём её
                return stored
            self.writes_log.append(f"неизвестный код 0x{code:04x} (MSP{'2' if code > 255 else '1'})")
            return b""  # отвечаем пустым payload, чтобы Configurator не ждал таймаут
        w = Writer()
        handler(w, Reader(payload))
        response = w.bytes()

        # 2) Чтение: если вкладка раньше что-то записала, отдаём записанное — но только для
        # блоков, которые эмулятор не моделирует. Для «состоянийных» блоков (фичи, сенсоры,
        # failsafe, PID, порты, сервы, VTX…) приоритет у сгенерированного ответа.
        stored = None if code in self.state_based_read_codes() else self.config_payloads.get(code)
        if stored is not None:
            # Эхо записанного значения допустимо только при совпадении длины: иначе ответ
            # вкладке разошёлся бы с прошивкой (пример: BEEPER_CONFIG — раскладка 9 байт,
            # а записать могли более короткий payload). Тогда отдаём канонический ответ.
            if stored and response and len(stored) == len(response):
                return stored
            self._log(
                f"[MSP] 0x{code:04X}: записанное значение не подходит по размеру "
                f"({len(stored)} vs {len(response)}) — отдаю сгенерированное"
            )
        return response

    def _handlers(self):
        if getattr(self, "_handler_cache", None) is None:
            self._handler_cache = {
                MSP_API_VERSION: self._h_api_version,
                MSP_FC_VARIANT: self._h_fc_variant,
                MSP_FC_VERSION: self._h_fc_version,
                MSP_BUILD_INFO: self._h_build_info,
                MSP_BOARD_INFO: self._h_board_info,
                MSP_UID: self._h_uid,
                MSP_STATUS: self._h_status,
                MSP_STATUS_EX: self._h_status_ex,
                MSP_RAW_IMU: self._h_raw_imu,
                MSP_ATTITUDE: self._h_attitude,
                MSP_ALTITUDE: self._h_altitude,
                MSP_ANALOG: self._h_analog,
                MSP_VOLTAGE_METERS: self._h_voltage_meters,
                MSP_CURRENT_METERS: self._h_current_meters,
                MSP_BATTERY_STATE: self._h_battery_state,
                MSP_RC: self._h_rc,
                MSP_MOTOR: self._h_motor,
                MSP_SERVO_CONFIGURATIONS: self._h_servo_configurations,
                MSP_SERVO_MIX_RULES: self._h_servo_mix_rules,
                MSP_SERVO: self._h_servo,
                MSP_MOTOR_TELEMETRY: self._h_motor_telemetry,
                MSP_MOTOR_3D_CONFIG: self._h_motor_3d_config,
                MSP_SET_MOTOR_3D_CONFIG: self._h_set_motor_3d_config,
                MSP2_MOTOR_OUTPUT_REORDERING: self._h_motor_output_reordering,
                MSP2_SET_MOTOR_OUTPUT_REORDERING: self._h_set_motor_output_reordering,
                MSP2_SEND_DSHOT_COMMAND: self._h_send_dshot_command,
                MSP_RAW_GPS: self._h_raw_gps,
                MSP_COMP_GPS: self._h_comp_gps,
                MSP_MIXER_CONFIG: self._h_mixer_config,
                MSP_FEATURE_CONFIG: self._h_feature_config,
                MSP_BEEPER_CONFIG: self._h_beeper_config,
                MSP_BOARD_ALIGNMENT_CONFIG: self._h_board_alignment,
                MSP_RC_TUNING: self._h_rc_tuning,
                MSP_PID: self._h_pid,
                MSP_ARMING_CONFIG: self._h_arming_config,
                MSP_SET_ARMING_CONFIG: self._h_set_arming_config,
                MSP_LOOP_TIME: self._h_loop_time,
                MSP_MISC: self._h_misc,
                MSP_MOTOR_CONFIG: self._h_motor_config,
                MSP_BOXNAMES: self._h_boxnames,
                MSP_BOXIDS: self._h_boxids,
                MSP_MODE_RANGES: self._h_mode_ranges,
                MSP_MODE_RANGES_EXTRA: self._h_mode_ranges_extra,
                MSP_CF_SERIAL_CONFIG: self._h_serial_config,
                MSP_FAILSAFE_CONFIG: self._h_failsafe_config,
                MSP_GPS_CONFIG: self._h_gps_config,
                MSP_SENSOR_CONFIG: self._h_sensor_config_read,
                MSP_SENSOR_ALIGNMENT: self._h_sensor_alignment,
                MSP_COMPASS_CONFIG: self._h_compass_config,
                MSP_ACC_TRIM: self._h_acc_trim,
                MSP2_SENSOR_CONFIG_ACTIVE: self._h_sensor_config_active,
                MSP2_COMMON_SERIAL_CONFIG: self._h_common_serial_config,
                MSP_RX_CONFIG: self._h_rx_config,
                MSP_RSSI_CONFIG: self._h_rssi_config,
                MSP_NAME: self._h_name,
                MSP2_GET_TEXT: self._h_get_text,
                MSP_ADVANCED_CONFIG: self._h_advanced_config,
                MSP_FILTER_CONFIG: self._h_filter_config,
                MSP_ADJUSTMENT_RANGES: self._h_adjustment_ranges,
                MSP_OSD_CONFIG: self._h_osd_config,
                MSP_VTX_CONFIG: self._h_vtx_config,
                MSP_VTXTABLE_BAND: self._h_vtx_table_band,
                MSP_VTXTABLE_POWERLEVEL: self._h_vtx_table_powerlevel,
                MSP2_GET_VTX_DEVICE_STATUS: self._h_vtx_device_status,
                MSP_BATTERY_CONFIG: self._h_battery_config,
                MSP_VOLTAGE_METER_CONFIG: self._h_voltage_meter_config,
                MSP_CURRENT_METER_CONFIG: self._h_current_meter_config,
                MSP_TRANSPONDER_CONFIG: self._h_transponder_config,
                MSP_BLACKBOX_CONFIG: self._h_blackbox_config,
                MSP_DATAFLASH_READ: self._h_dataflash_read,
                MSP_DATAFLASH_SUMMARY: self._h_dataflash_summary,
                MSP_SDCARD_SUMMARY: self._h_sdcard_summary,
                MSP_RXFAIL_CONFIG: self._h_rxfail_config,
                MSP_RX_MAP: self._h_rx_map,
                MSP_PID_CONTROLLER: self._h_pid_controller,
                MSP_GPS_RESCUE: self._h_gps_rescue,
                MSP_DEBUG: self._h_debug,
                MSP_SENSOR_STATUS: self._h_sensor_status,
                MSP_GPS_SV_INFO: self._h_gps_sv_info,
                # запись
                MSP_SET_FEATURE_CONFIG: self._h_set_feature_config,
                MSP_SET_SENSOR_CONFIG: self._h_set_sensor_config,
                MSP_SET_SENSOR_ALIGNMENT: self._h_set_sensor_alignment,
                MSP_SET_COMPASS_CONFIG: self._h_set_compass_config,
                MSP_SET_ACC_TRIM: self._h_set_acc_trim,
                MSP_SET_BEEPER_CONFIG: self._h_set_beeper_config,
                MSP_SET_MODE_RANGE: self._h_set_mode_range,
                MSP_SET_NAME: self._h_set_name,
                MSP_SET_BOARD_ALIGNMENT_CONFIG: self._h_set_echo,
                MSP_SET_MIXER_CONFIG: self._h_set_echo,
                MSP_SET_RX_CONFIG: self._h_set_rx_config,
                MSP_SET_RSSI_CONFIG: self._h_set_rssi_config,
                MSP_SET_FAILSAFE_CONFIG: self._h_set_failsafe_config,
                MSP_SET_RC_TUNING: self._h_set_echo,
                MSP_SET_PID: self._h_set_pid,
                MSP_PIDNAMES: self._h_pidnames,
                MSP_PID_ADVANCED: self._h_pid_advanced,
                MSP_SET_PID_ADVANCED: self._h_set_pid_advanced,
                MSP_SET_RC_TUNING: self._h_set_rc_tuning,
                MSP_RC_DEADBAND: self._h_rc_deadband,
                MSP_SET_RC_DEADBAND: self._h_set_rc_deadband,
                MSP_SET_FILTER_CONFIG: self._h_set_filter_config,
                MSP_SET_MOTOR_CONFIG: self._h_set_motor_config,
                MSP_SIMPLIFIED_TUNING: self._h_simplified_tuning,
                MSP_SET_SIMPLIFIED_TUNING: self._h_set_simplified_tuning,
                MSP_CALCULATE_SIMPLIFIED_PID: self._h_calculate_simplified_pid,
                MSP_CALCULATE_SIMPLIFIED_GYRO: self._h_calculate_simplified_gyro,
                MSP_CALCULATE_SIMPLIFIED_DTERM: self._h_calculate_simplified_dterm,
                MSP_VALIDATE_SIMPLIFIED_TUNING: self._h_validate_simplified_tuning,
                MSP_SELECT_SETTING: self._h_select_setting,
                MSP_COPY_PROFILE: self._h_copy_profile,
                MSP_SET_RESET_CURR_PID: self._h_set_reset_curr_pid,
                MSP_SET_MISC: self._h_set_echo,
                MSP_SET_GPS_CONFIG: self._h_set_gps_config,
                MSP_SET_ADVANCED_CONFIG: self._h_set_advanced_config,
                MSP_SET_ADJUSTMENT_RANGE: self._h_set_adjustment_range,
                MSP_SET_OSD_CONFIG: self._h_set_osd_config,
                MSP_OSD_CANVAS: self._h_osd_canvas,
                MSP_SET_OSD_CANVAS: self._h_set_osd_canvas,
                MSP_SET_VTX_CONFIG: self._h_set_vtx_config,
                MSP_SET_VTXTABLE_BAND: self._h_set_vtx_table_band,
                MSP_SET_VTXTABLE_POWERLEVEL: self._h_set_vtx_table_powerlevel,
                MSP_SET_BATTERY_CONFIG: self._h_set_battery_config,
                MSP_SET_VOLTAGE_METER_CONFIG: self._h_set_voltage_meter_config,
                MSP_SET_CURRENT_METER_CONFIG: self._h_set_current_meter_config,
                MSP_SET_BLACKBOX_CONFIG: self._h_set_blackbox_config,
                MSP_SET_RXFAIL_CONFIG: self._h_set_rxfail_config,
                MSP_SET_RX_MAP: self._h_set_rx_map,
                MSP_SET_GPS_RESCUE: self._h_set_gps_rescue,
                MSP_SET_CF_SERIAL_CONFIG: self._h_set_serial_config,
                MSP2_COMMON_SET_SERIAL_CONFIG: self._h_set_common_serial_config,
                MSP_SET_ARMING_DISABLED: self._h_set_arming_disabled,
                MSP_SET_MOTOR: self._h_set_motor,
                MSP_SET_SERVO_CONFIGURATION: self._h_set_servo_configuration,
                MSP_SET_SERVO_MIX_RULE: self._h_set_servo_mix_rule,
                MSP_SET_RAW_RC: self._h_ack,
                MSP_ACC_CALIBRATION: self._h_acc_calibration,
                MSP_MAG_CALIBRATION: self._h_ack,
                MSP_EEPROM_WRITE: self._h_eeprom_write,
                MSP_RESET_CONF: self._h_reset_conf,
                MSP_REBOOT: self._h_reboot,
                MSP_SET_RTC: self._h_ack,
                MSP_DATAFLASH_ERASE: self._h_dataflash_erase,
                MSP_SET_PASSTHROUGH: self._h_ack,
                MSP_CAMERA_CONTROL: self._h_ack,
                MSP2_SET_TEXT: self._h_set_text,
                MSP2_GET_OSD_WARNINGS: self._h_osd_warnings,
                MSP2_MCU_INFO: self._h_mcu_info,
                MSP2_GYRO_SENSOR_ACTIVE: self._h_gyro_sensor_active,
                MSP2_SEND_DSHOT_COMMAND: self._h_send_dshot_command,
                MSP2_SET_MOTOR_OUTPUT_REORDERING: self._h_set_motor_output_reordering,
                MSP2_BATTERY_PROFILE: self._h_battery_profile,
                MSP2_SET_BATTERY_PROFILE: self._h_set_battery_profile,
                MSP2_CLI_SETTING: self._h_cli_setting,
                MSP2_CLI_SETTING_INFO: self._h_cli_setting_info,
                MSP2_CLI_COMMAND: self._h_cli_command,
            }
        return self._handler_cache

    # --- идентификация
    def _h_api_version(self, w, r):
        w.u8(self.msp_protocol_version)
        major, minor, _ = (int(x) for x in self.api_version.split("."))
        w.u8(major).u8(minor)

    def _h_fc_variant(self, w, r):
        w.raw(b"BTFL")

    def _h_fc_version(self, w, r):
        major, minor, patch = (int(x) for x in self.fc_version.split("."))
        w.u8(major).u8(minor).u8(patch)

    def _h_build_info(self, w, r):
        """MSP_BUILD_INFO (5): дата 11 Б, время 8 Б, git-ревизия 7 Б, затем build options.

        После даты раньше писался лишний пробел: он сдвигал разбор, из-за чего
        git-ревизия в консоли Configurator выглядела мусором, а build options
        вообще не читались (API 1.46 и новее берёт их именно отсюда).

        По build options Configurator решает, какие вкладки и функции показывать:
        вкладка «Сервоприводы» есть только при USE_SERVOS или USE_WING.
        """
        w.raw(self.build_date.ljust(11).encode()[:11])
        w.raw(self.build_time.ljust(8).encode()[:8])
        w.raw(self.git_revision.ljust(7).encode()[:7])
        for _name, code in BUILD_OPTIONS:
            w.u16(code)
        w.u16(0)  # терминатор списка

    def _h_acc_calibration(self, w, r):
        """MSP_ACC_CALIBRATION: прошивка минуту держит MCU в калибровке — у нас это ~1 с.

        Configurator ждёт, что флаг «акселерометр не откалиброван» в MSP_BOARD_INFO снимется.
        """
        if not self.acc_calibrated:
            self.acc_calibration_until = time.time() + 1.0
            self._log("[FC] калибровка акселерометра запущена (≈1 с, как на железе)")
        return

    def _h_board_info(self, w, r):
        w.raw(self.board_identifier.encode()[:4])
        w.u16(0)  # board version
        w.u8(0)  # board type
        w.u8(self.target_capabilities)
        # msp.c: sbufWriteU8(strlen(name)); sbufWriteData(name, strlen(name))
        w.ltext(self.target_name)
        w.ltext(self.board_name)
        w.ltext(self.manufacturer_id)
        w.raw(self.signature)
        w.u8(self.mcu_type_id)
        w.u8(self.configuration_state)
        w.u16(self.GYRO_SAMPLE_RATE_HZ)  # sample rate Hz (API 1.43+)
        w.u32(self._configuration_problems())  # configuration problems (API 1.43+)

    GYRO_SAMPLE_RATE_HZ = 8000
    PROBLEM_ACC_NEEDS_CALIBRATION = 1 << 0
    PROBLEM_MOTOR_PROTOCOL_DISABLED = 1 << 1

    def _configuration_problems(self) -> int:
        """Как configurationProblems в msp.c: некалиброванный акселерометр и выключенный протокол."""
        problems = 0
        if not self.acc_calibrated:
            problems |= self.PROBLEM_ACC_NEEDS_CALIBRATION
        if self._setting("motor_pwm_protocol", 0) == 9:  # DISABLED
            problems |= self.PROBLEM_MOTOR_PROTOCOL_DISABLED
        return problems

    def _h_uid(self, w, r):
        for u in self.uids:
            w.u32(u)

    # --- телеметрия/статус
    def _t(self):
        return time.time() - self.boot_time

    def _h_sensor_alignment(self, w, r):
        """MSP_SENSOR_ALIGNMENT (126) — 7 байт, как в msp.c для API < 1.47.

        Раскладка: align_gyro, align_acc (в 4.x равен гироскопу), align_mag,
        флаги найденных гироскопов, gyro_to_use, gyro_1_align, gyro_2_align.
        """
        gyro_align = self._setting("gyro_1_sensor_align", 0)
        w.u8(gyro_align)
        w.u8(gyro_align)                       # в прошивке крен/тангаж акселерометра = гироскопу
        w.u8(self._setting("align_mag", 0))
        w.u8(self.GYRO_1_MASK)                 # найден один гироскоп (gyroDetectionFlags)
        w.u8(self._setting("gyro_to_use", 0))
        w.u8(gyro_align)
        w.u8(self._setting("gyro_2_sensor_align", 0))

    def _h_set_sensor_alignment(self, w, r):
        """MSP_SET_SENSOR_ALIGNMENT (220): раскладка та же, что у чтения."""
        data = bytes(r.rest())
        if len(data) < 3:
            return
        self._store_setting("gyro_1_sensor_align", data[0])
        self._store_setting("align_mag", data[2])          # data[1] — устаревшее align_acc
        if len(data) >= 6:
            self._store_setting("gyro_to_use", data[3])
            self._store_setting("gyro_1_sensor_align", data[4])
            self._store_setting("gyro_2_sensor_align", data[5])
        names = {0: "DEFAULT", 1: "CW0", 2: "CW90", 3: "CW180", 4: "CW270", 5: "CW0FLIP",
                 6: "CW90FLIP", 7: "CW180FLIP", 8: "CW270FLIP", 9: "CUSTOM"}
        self._log(
            "[MSP] SENSOR_ALIGNMENT сохранён: гироскоп={}, магнитометр={}, gyro_to_use={}".format(
                names.get(self._setting("gyro_1_sensor_align"), self._setting("gyro_1_sensor_align")),
                names.get(self._setting("align_mag"), self._setting("align_mag")),
                self._setting("gyro_to_use"),
            )
        )

    def _h_compass_config(self, w, r):
        """MSP_COMPASS_CONFIG (133): склонение магнитометра в десятых долях градуса."""
        w.u16(self._setting("mag_declination", 0))

    def _h_set_compass_config(self, w, r):
        """MSP_SET_COMPASS_CONFIG (224)."""
        data = bytes(r.rest())
        if len(data) < 2:
            return
        self._store_setting("mag_declination", int.from_bytes(data[0:2], "little"))
        self._log(f"[MSP] COMPASS_CONFIG сохранён: склонение {self._setting('mag_declination') / 10:.1f}°")

    def _h_status(self, w, r):
        w.u16(self.cycle_time).u16(0).u16(self._active_sensor_mask())
        w.u32(self._mode_flags())
        w.u8(self.current_profile)
        w.u16(0)  # gyro cycle time

    def _h_status_ex(self, w, r):
        w.u16(self.cycle_time).u16(0).u16(self._active_sensor_mask())
        w.u32(self._mode_flags())
        w.u8(self.current_profile)
        w.u16(self.cpu_load)
        w.u8(PID_PROFILE_COUNT)  # количество профилей PID
        w.u8(self.current_rate_profile)
        w.u8(0)  # byteCount: дополнительных байт флагов режимов нет
        w.u8(ARMING_DISABLE_FLAGS_COUNT)
        w.u32(self.arming_disable_flags)
        w.u8(0)  # config state flags: перезагрузка не требуется
        w.u16(42)  # температура ядра, °C (API 1.46+)

    def _active_sensor_mask(self):
        """Что FC видит «живым». Учитываем выбор во вкладке «Датчики»: NONE = датчик выключен."""
        cfg = self.sensor_config
        mask = SENSOR_GYRO | SENSOR_GPS
        if cfg["acc_hardware"] != SENSOR_NONE:
            mask |= SENSOR_ACC
        if cfg["baro_hardware"] != SENSOR_NONE:
            mask |= SENSOR_BARO
        if cfg["mag_hardware"] != SENSOR_NONE:
            mask |= SENSOR_MAG
        if cfg["sonar_hardware"] != RANGEFINDER_NONE:
            mask |= SENSOR_RANGEFINDER
        return mask

    @property
    def arming_disable_flags(self):
        # На свежем FC арм-переключатель выключен — это и есть базовый флаг.
        flags = ARMING_DISABLED_ARM_SWITCH
        if self.motor_test_arming_block:
            # Вкладка «Моторы» на время теста шлёт MSP_SET_ARMING_DISABLED(1): борт взвести нельзя,
            # в Setup появляется причина «MSP» — как на железной плате.
            flags |= ARMING_DISABLED_MSP
        if self.rx_serial_ready() and not self.rx_link_connected:
            # rx.c: при потере сигнала прошивка поднимает ARMING_DISABLED_RX_FAILSAFE.
            # Configurator по этому биту показывает на вкладке «Приёмник» предупреждение
            # о потерянной связи и подсвечивает полосы каналов жёлтым.
            flags |= ARMING_DISABLED_RX_FAILSAFE
        if self.fs_demo and self.fs_armed:
            flags &= ~ARMING_DISABLED_ARM_SWITCH      # борт взведён: ARM-переключатель поднят
        if self.fs_demo and self.fs_phase in (self.FS_LANDED, self.FS_RX_LOSS_MONITORING):
            # failsafe.c: FAILSAFE_LANDED — setArmingDisabled(ARMING_DISABLED_FAILSAFE),
            # снимается только в FAILSAFE_RX_LOSS_RECOVERED. Пока флаг стоит, Configurator
            # в Setup показывает «Взвод запрещён: failsafe».
            flags |= ARMING_DISABLED_FAILSAFE
        return flags

    def _mode_flags(self):
        """Маска включённых режимов для MSP_STATUS — по положению тумблеров.

        В демо failsafe борт «в воздухе» и взведён, поэтому к маске добавляется позиция
        режима ARM: Configurator подсвечивает «ARM», а вкладка «Режимы» показывает борт
        взведённым — как на реальном борде под газом.
        """
        mask = self.rc_mode_mask()
        if self.fs_demo and self.fs_armed and "ARM" in self.active_boxes:
            mask |= 1 << self.active_boxes.index("ARM")
        return mask

    def _h_raw_imu(self, w, r):
        """MSP_RAW_IMU: 9 знаковых 16-битных значений — акселерометр, гироскоп, магнитометр.

        Дрон стоит на столе: по Z ровно 1 g (в единицах FC это 2048), по остальным осям —
        только шум младших разрядов. Configurator делит акселерометр на 2048 (1.0 g),
        гироскоп переводит в °/с. Раньше ответ был 24 байта вместо 18 — разбор сдвигался.
        """
        t = self._t()
        w.i16(int(math.sin(t * 31) * 4))                     # X: покой, только шум
        w.i16(int(math.sin(t * 31 + 1) * 4))                 # Y: покой, только шум
        w.i16(2048 + int(math.sin(t * 31 + 2) * 4))          # Z: 1 g (2048) + шум
        for axis in range(3):
            w.i16(int(math.sin(t * 47 + axis) * 12))         # гироскоп: ±1 °/с шума
        if self.sensor_config["mag_hardware"] != SENSOR_NONE:
            fields = (300, -120, 410)                         # ровное магнитное поле стенда
            for axis in range(3):
                w.i16(fields[axis] + int(math.sin(t * 29 + axis) * 2))
        else:
            for _ in range(3):
                w.i16(0)                                      # магнитометра нет

    def _h_attitude(self, w, r):
        """MSP_ATTITUDE: дрон неподвижен — крен, тангаж и курс 0.

        Roll/pitch прошивка пишет в десятых долях градуса, а курс — уже в градусах
        (sbufWriteU16(DECIDEGREES_TO_DEGREES(attitude.values.yaw))), и Configurator
        делит на 10 только первые два. Раньше курс уходил в десятых долях (в интерфейсе
        появлялось «3412.0»), а значения ещё и менялись во времени — фигурка коптера
        вращалась сама по себе.
        """
        w.i16(int(self._setting("acc_trim_roll", 0)) * 0)  # крен, десятые доли градуса
        w.i16(0)  # тангаж
        w.i16(0)  # курс, градусы

    def _h_acc_trim(self, w, r):
        """MSP_ACC_TRIM (240): смещения акселерометра — как accelerometerTrims в прошивке."""
        w.i16(self._setting("acc_trim_pitch", 0)).i16(self._setting("acc_trim_roll", 0))

    def _h_set_acc_trim(self, w, r):
        """MSP_SET_ACC_TRIM (239): как MSP_SET_ACC_TRIM в msp.c — pitch, затем roll."""
        data = bytes(r.rest())
        if len(data) < 4:
            return
        self._store_setting("acc_trim_pitch", int.from_bytes(data[0:2], "little", signed=True))
        self._store_setting("acc_trim_roll", int.from_bytes(data[2:4], "little", signed=True))
        self._log(
            f"[MSP] ACC_TRIM сохранён: pitch={self._setting('acc_trim_pitch')}, "
            f"roll={self._setting('acc_trim_roll')}"
        )
        return b""   # пустой ACK: вкладка «Датчики» ждёт ответа на запись

    def _h_altitude(self, w, r):
        t = self._t()
        w.i32(int((math.sin(t / 5) * 2) * 100))  # см
        w.i16(0)

    # ---------------------------------------------------------------- батарея
    @staticmethod
    def _default_rxfail() -> list:
        """Заводские значения «Этапа 1» (pgResetFn_rxFailsafeChannelConfigs в rx.c).

        Первые четыре канала — AUTO (0): у газа значение RX_MIN_USEC, у остальных
        среднее; AUX-каналы — HOLD (1) со средним значением. Значения идут по сетке
        25 мкс: прошивка хранит «шаг» и переводит его обратно через
        CHANNEL_VALUE_TO_RXFAIL_STEP()/RXFAIL_STEP_TO_CHANNEL_VALUE().
        """
        def snap(value):
            step = (max(PWM_PULSE_MIN, min(2250, value)) - PWM_PULSE_MIN) // 25
            return PWM_PULSE_MIN + 25 * step

        channels = []
        for index in range(RC_CHANNEL_COUNT):
            if index < NON_AUX_CHANNEL_COUNT:
                mode = RX_FAILSAFE_MODE_AUTO
                value = snap(RX_MIN_USEC) if index == 3 else snap(RX_MID_USEC)
            else:
                mode = RX_FAILSAFE_MODE_HOLD
                value = snap(RX_MID_USEC)
            channels.append({"mode": mode, "value": value})
        return channels

    # ------------------- таблицы «только для CLI» (rxrange / led / color / mode_color / mmix)
    @staticmethod
    def _default_rx_ranges() -> list:
        """Заводские диапазоны каналов приёмника (pgResetFn_rxChannelRangeConfigs).

        Прошивка печатает их командой `rxrange`: строка на канал 0…3, заводские 1000…2000.
        Границы правки — PWM_PULSE_MIN/PWM_PULSE_MAX (750…2250) в rx.h, как в cliRxRange().
        """
        return [{"min": 1000, "max": 2000} for _ in range(NON_AUX_CHANNEL_COUNT)]

    @staticmethod
    def _default_led_configs() -> list:
        """Строки ленты (ledConfigs[]): у свежего борта пусто — «0,0::C:0» на каждую позицию.

        Формат строки — generateLedConfig() из ledstrip.c: «x,y:направления:функции:цвет»
        (базовые функции «C F A L S G R», направления «N E S W U D», наложения «T Y O B V I W»).
        """
        return ["0,0::C:0"] * LED_STRIP_MAX_LENGTH

    @staticmethod
    def _default_led_colors() -> list:
        """Таблица цветов ленты: первые LED_COLOR_COUNT — hsv[] из ledstrip.c, остальные нули.

        pgResetFn_ledStripStatusModeConfig() копирует в colors[] ровно ARRAYLEN(hsv) записей.
        """
        colors = [[0, 0, 0] for _ in range(LED_CONFIGURABLE_COLOR_COUNT)]
        for index, (hue, saturation, value) in enumerate((
            (0, 0, 0),      # COLOR_BLACK
            (0, 255, 255),  # COLOR_WHITE
            (0, 0, 255),    # COLOR_RED
            (30, 0, 255),   # COLOR_ORANGE
            (60, 0, 255),   # COLOR_YELLOW
            (90, 0, 255),   # COLOR_LIME_GREEN
            (120, 0, 255),  # COLOR_GREEN
            (150, 0, 255),  # COLOR_MINT_GREEN
            (180, 0, 255),  # COLOR_CYAN
            (210, 0, 255),  # COLOR_LIGHT_BLUE
            (240, 0, 255),  # COLOR_BLUE
            (270, 0, 255),  # COLOR_DARK_VIOLET
            (300, 0, 255),  # COLOR_MAGENTA
            (330, 0, 255),  # COLOR_DEEP_PINK
        )):
            colors[index] = [hue, saturation, value]
        return colors

    @staticmethod
    def _default_mode_colors() -> dict:
        """Цвета режимов и спец-цвета ленты — defaultModeColors/defaultSpecialColors (ledstrip.c).

        Направления идут как LED_DIRECTION_COUNT в ledstrip.h: N, E, S, W, U, D. Для режима
        LED_MODE_BARO значений в заводской таблице нет (копируется только 5 строк), поэтому он
        нулевой; AUX-канал ленты по умолчанию THROTTLE (rc_alias_e: ROLL 0, PITCH 1, YAW 2,
        THROTTLE 3) — в эмуляторе газ тоже третий канал (порядок A/E/R/T).
        """
        return {
            "mode": [
                [1, 11, 2, 13, 10, 3],    # ORIENTATION: WHITE, DARK_VIOLET, RED, DEEP_PINK, BLUE, ORANGE
                [5, 11, 3, 13, 10, 3],    # HEADFREE: LIME_GREEN …
                [10, 11, 4, 13, 10, 3],   # HORIZON: BLUE, DARK_VIOLET, YELLOW …
                [8, 11, 4, 13, 10, 3],    # ANGLE: CYAN …
                [7, 11, 3, 13, 10, 3],    # MAG: MINT_GREEN …
                [0, 0, 0, 0, 0, 0],       # BARO: в заводской таблице пусто
            ],
            "special": [6, 10, 1, 0, 0, 2, 3, 6, 0, 0, 0],   # DISARMED…GPSLOCKED, дальше нули
            "aux_channel": 3,         # THROTTLE
        }

    @staticmethod
    def _default_custom_motor_mix() -> list:
        """customMotorMixer[] пуст: printMotorMix() останавливается на нулевом газе — строк нет."""
        return []

    @staticmethod
    def _default_battery_profiles() -> list:
        """batteryProfiles[] — pgResetFn_batteryProfiles(): три профиля с одинаковыми порогами.

        Профили батареи появились после 4.5.4 (мастер BF): каждый хранит свои пороги ячеек,
        ёмкость, принудительное число банок, процент предупреждения по расходу и короткое
        имя (до 8 символов).
        """
        return [dict(VBAT_PROFILE_DEFAULTS, name="") for _ in range(BATTERY_PROFILE_COUNT)]

    def _battery_profiles_from_json(self, data) -> list:
        """Профили батареи из «флеша»: битое отбрасываем, недостающее добираем заводским."""
        profiles = self._default_battery_profiles()
        if not isinstance(data, list):
            return profiles
        for index, item in enumerate(data[:BATTERY_PROFILE_COUNT]):
            if not isinstance(item, dict):
                continue
            profile = dict(profiles[index])
            for field in ("vbatmaxcellvoltage", "vbatmincellvoltage", "vbatwarningcellvoltage",
                          "vbatfullcellvoltage"):
                profile[field] = self._clamp_int(item.get(field), 100, 500, profile[field])
            profile["batteryCapacity"] = self._clamp_int(item.get("batteryCapacity"), 0, 20000,
                                                         profile["batteryCapacity"])
            profile["forceBatteryCellCount"] = self._clamp_int(item.get("forceBatteryCellCount"), 0, 24,
                                                               profile["forceBatteryCellCount"])
            profile["consumptionWarningPercentage"] = self._clamp_int(
                item.get("consumptionWarningPercentage"), 0, 100,
                profile["consumptionWarningPercentage"])
            name = item.get("name")
            if isinstance(name, str):
                profile["name"] = name[:MAX_BATTERY_PROFILE_NAME_LENGTH]
            profiles[index] = profile
        return profiles

    # --- «флеш» для команд flash_read/flash_write (USE_FLASH_TOOLS)
    def flash_read_bytes(self, address: int, length: int) -> bytes:
        """flashfsReadAbs(): содержимое «флеша» эмулятора плюс то, что записал flash_write.

        «Флеш» здесь — накопитель чёрного ящика (store.data): адреса за его пределами
        прошивка тоже читает нулями, поэтому отдаём столько байт, сколько поместилось.
        """
        if address < 0 or length <= 0:
            return b""
        data = self.blackbox.data
        size = min(len(data), address + length)
        out = bytearray(data[address:size]) if size > address else bytearray()
        for start, chunk in self.flash_overlay.items():
            end = start + len(chunk)
            if end <= address or start >= address + length:
                continue
            need = min(address + length, end) - address
            if len(out) < need:
                out.extend(b"\x00" * (need - len(out)))     # записанное читается и на пустом накопителе
            for offset in range(max(start, address), min(end, address + length)):
                out[offset - address] = chunk[offset - start]
        return bytes(out)

    def flash_write_bytes(self, address: int, text: bytes):
        """flashfsWrite(): запись в память эмулятора — файл пролётов остаётся целым."""
        if address < 0:
            return
        for start in [start for start in self.flash_overlay if start < address < start + len(self.flash_overlay[start])]:
            # запись поверх уже записанного куска: хвост оставляем как есть
            prefix = self.flash_overlay[start][:address - start]
            rest = self.flash_overlay[start][address - start + len(text):]
            del self.flash_overlay[start]
            if prefix:
                self.flash_overlay[start] = prefix
            if rest:
                self.flash_overlay[address + len(text)] = rest
        self.flash_overlay[address] = bytes(text)

    @staticmethod
    def _default_servo_params() -> list:
        """Заводские параметры сервоприводов (pgResetFn_servoParams в flight/servos.c)."""
        return [
            {
                "min": DEFAULT_SERVO_MIN,
                "max": DEFAULT_SERVO_MAX,
                "middle": DEFAULT_SERVO_MIDDLE,
                "rate": 100,
                "forwardFromChannel": CHANNEL_FORWARDING_DISABLED,
                "reversedSources": 0,
            }
            for _ in range(MAX_SUPPORTED_SERVOS)
        ]

    @staticmethod
    def _default_servo_mix_rules() -> list:
        """Правила микшера сервоприводов: по умолчанию всё в нулях (customServoMixers)."""
        return [
            {"targetChannel": 0, "inputSource": 0, "rate": 0, "speed": 0, "min": 0, "max": 0, "box": 0}
            for _ in range(MAX_SERVO_RULES)
        ]

    def _battery_setting(self, name, lo=None, hi=None):
        """Значение настройки батареи с ограничением, как у VAR_UINT16 в прошивке."""
        entry = self.settings_by_name.get(name)
        default = entry["default"] if entry else 0
        value = int(self.settings.get(name, default) or 0)
        if lo is None or hi is None:
            return value
        return max(lo, min(hi, value))

    def _battery_meter_source(self):
        """voltageMeterSource: 0 NONE, 1 ADC, 2 ESC (battery_meter)."""
        return int(self.settings.get("battery_meter", 0) or 0)

    def _current_meter_source(self):
        """currentMeterSource: 0 NONE, 1 ADC, 2 VIRTUAL, 3 ESC, 4 MSP (current_meter)."""
        return int(self.settings.get("current_meter", 0) or 0)

    def _pack_rest_voltage(self) -> float:
        """Напряжение «покоя» батареи — с учётом разряда.

        Заряженная батарея — 4.20 В на банку; по мере расхода (mah_drawn против
        ёмкости bat_capacity) напряжение падает до SIM_PACK_MIN_V (3.50 В — уровень
        предупреждения, на нём эмулятор садится и «меняет батарею»). Ёмкость 0 —
        «ёмкость неизвестна», тогда напряжение не падает.
        """
        if self._battery_meter_source() == 0:
            return 0.0
        per_cell = self.SIM_PACK_FULL_V
        capacity = self._battery_setting("bat_capacity", 0, 20000)
        if capacity > 0:
            spent = min(1.0, self.mah_drawn / capacity)
            per_cell = self.SIM_PACK_FULL_V - (self.SIM_PACK_FULL_V - self.SIM_PACK_MIN_V) * spent
        return self.battery_cells * per_cell

    def _battery_voltage(self) -> float:
        """Напряжение батареи: 0, пока не выбран датчик напряжения (как на столе без батареи).

        Под нагрузкой напряжение проседает — считаем это здесь, в одном месте
        (rest - I · R), поэтому вкладки «Моторы», «Питание и батарея» и OSD показывают
        одну и ту же просадку. Напряжение «покоя» берётся у _pack_rest_voltage():
        разряженная батарея проседает сильнее, чем заряженная.
        """
        if self._battery_meter_source() == 0:
            return 0.0
        # Сначала добираем расход: «покой» зависит от счётчика мА·ч, и если посчитать
        # напряжение по прошлому значению, напряжение и расход в одном ответе MSP
        # разойдутся на один опрос (заметно при редком опросе вкладки).
        self._mah_update()
        rest = self._pack_rest_voltage()
        current = self._battery_current()
        return max(0.0, rest - current * self.MOTOR_PACK_RESISTANCE_OHM)

    def _battery_current(self) -> float:
        """Ток батареи: ток моторов + ток электроники борта.

        В покое — не ноль: FC, приёмник, VTX и LED потребляют около 0.55 А, и на вкладке
        «Питание и батарея» это видно («Ток потребления» 0.49…0.61 А). Ток чуть «дышит»,
        как настоящий датчик. Датчик тока не выбран — 0.
        """
        if self._current_meter_source() == 0:
            return 0.0
        idle = self.SIM_IDLE_CURRENT_A + self.SIM_IDLE_RIPPLE_A * math.sin(time.time() * 0.7)
        return round(max(0.0, idle) + sum(self.motor_current_a), 2)

    def _sim_scaled_dt(self, dt: float) -> float:
        """Время «виртуального полёта» в стенных секундах: ×SIM_FLIGHT_TIME_SCALE.

        Ускорение нужно только счётчикам расхода: напряжение, обороты и нагрев
        считаются по настоящему времени, поэтому картинка на вкладках остаётся
        физичной, а «Израсходовано mAh» бежит заметно для глаза.
        """
        return dt * self.sim_time_scale

    def _mah_update(self):
        """Интегрируем ток в мА·ч (getMAhDrawn в прошивке).

        Расход растёт и в покое (электроника), и в полёте; время масштабируется
        множителем _sim_scaled_dt().
        """
        now = time.time()
        if self._mah_t is None:
            self._mah_t = now
            return
        dt = max(0.0, now - self._mah_t)
        self._mah_t = now
        current = self._battery_current()
        if current > 0:
            self.mah_drawn += current * self._sim_scaled_dt(dt) / 3.6   # А·с -> мА·ч

    def _mah_drawn(self) -> int:
        self._mah_update()
        return int(self.mah_drawn)

    def _battery_filtered_voltage(self) -> int:
        """«Отфильтрованное» напряжение в 0.01 В — как voltageMeter.displayFiltered.

        Оно всегда на шаг ниже «живого»: фильтр в прошивке никогда не выходит точно
        на измеренное значение. Без этого ровно 16.80 В при vbat_max_cell_voltage = 420
        дало бы 5 банок вместо 4 (в batteryUpdatePresence() деление целочисленное).
        """
        millivolts = int(round(self._battery_voltage() * 100))
        return max(0, millivolts - 1)

    def _battery_cell_count(self) -> int:
        """getBatteryCellCount(): 0 — батарея не обнаружена, иначе число банок."""
        if self._battery_meter_source() == 0:
            return 0
        forced = int(self.settings.get("force_battery_cell_count", 0) or 0)
        if forced:
            return forced
        filtered = self._battery_filtered_voltage()
        not_present = self._battery_setting("vbat_detect_cell_voltage")
        per_cell_max = self._battery_setting("vbat_max_cell_voltage")
        # isVoltageFromBattery(): выше «порога отсутствия» и не выше напряжения 1S,
        # либо заведомо выше двойного порога (напряжение USB-питания).
        from_battery = (not_present <= filtered <= per_cell_max) or filtered > not_present * 2
        if not from_battery:
            return 0
        cells = filtered // per_cell_max + 1             # как в batteryUpdatePresence()
        return min(cells, MAX_AUTO_DETECT_CELL_COUNT)

    def _battery_state_code(self) -> int:
        """batteryState_e: 0 OK, 1 предупреждение, 2 критично, 3 батарея не подключена."""
        cells = self._battery_cell_count()
        if cells == 0:
            return BATTERY_NOT_PRESENT
        voltage = self._battery_voltage()
        if voltage < cells * self._battery_setting("vbat_min_cell_voltage") / 100:
            return BATTERY_CRITICAL
        if voltage < cells * self._battery_setting("vbat_warning_cell_voltage") / 100:
            return BATTERY_WARNING
        return BATTERY_OK

    # ------------------------------------------------------------------ виртуальный полёт
    def set_sim_flight(self, on: bool) -> None:
        """Включить/выключить «виртуальный полёт» (кнопка монитора, команда sim_flight).

        Выключение — как «стоп» на вкладке «Моторы»: выходы падают на 1000, моторы
        выбегают, ток возвращается к току электроники.
        """
        on = bool(on)
        if on and not self.sim_flight:
            self.sim_flight = True
            self.sim_flight_phase = "flight"
            self.sim_flight_since = time.time()
            self.sim_flight_note = "Полёт начат: газ по сценарию, расход мА·ч пошёл"
            self._log("[SIM] виртуальный полёт включён (газ по сценарию, расход мА·ч учитывается)")
        elif not on and self.sim_flight:
            self.sim_flight = False
            self.sim_flight_phase = "idle"
            self.motor_disarmed = [1000.0] * MAX_SUPPORTED_MOTORS
            self.sim_flight_note = "Полёт остановлен: моторы на 1000, ток — только электроника"
            self._log("[SIM] виртуальный полёт выключен (моторы на 1000)")

    def sim_flight_state(self) -> dict:
        """Снимок «виртуального полёта» для монитора, CLI и /api/state."""
        self._motors_step()
        self._mah_update()
        phase_names = {
            "idle": "выключен",
            "flight": "полёт",
            "landing": "посадка",
            "swap": "замена батареи",
        }
        return {
            "on": self.sim_flight,
            "phase": self.sim_flight_phase,
            "phase_text": phase_names.get(self.sim_flight_phase, self.sim_flight_phase),
            "note": self.sim_flight_note,
            "time_scale": self.sim_time_scale,
            "packs": self.sim_pack_events,
            "seconds": round(time.time() - self.sim_flight_since, 1) if self.sim_flight else 0.0,
        }

    def power_state(self) -> dict:
        """Состояние питания для веб-монитора: то же, что читает вкладка «Питание и батарея»."""
        self._motors_step()
        self._mah_update()
        voltage = self._battery_voltage()
        return {
            "cells": self._battery_cell_count(),
            "voltage": round(voltage, 2),
            "rest_voltage": round(self._pack_rest_voltage(), 2),
            "current": self._battery_current(),
            "mah": int(self.mah_drawn),
            "capacity": self._battery_setting("bat_capacity", 0, 20000),
            "state": self._battery_state_code(),
            "state_name": ("OK", "ПРЕДУПРЕЖДЕНИЕ", "КРИТИЧНО", "БАТАРЕЯ НЕ ПОДКЛЮЧЕНА")[
                self._battery_state_code()],
            "motors": [int(round(v)) for v in self.motor_output[:self._motor_count()]],
            "sim": self.sim_flight_state(),
        }

    def _sim_flight_outputs(self, now: float) -> list:
        """Выходы моторов (1000…2000) по сценарию «виртуального полёта».

        Сценарий: раскрутка на взлёте, висение с подруливанием и «газ в пол» каждые
        SIM_PUNCH_PERIOD_S секунд; у каждого мотора своя мелочь, поэтому токи ESC
        не одинаковые. При посадке газ линейно убирается до нуля.
        """
        t = now - self.sim_flight_since
        if self.sim_flight_phase == "landing":
            gas = self.SIM_HOVER_GAS * max(0.0, 1.0 - t / self.SIM_LAND_S)
        elif self.sim_flight_phase == "swap":
            gas = 0.0
        elif t < self.SIM_TAKEOFF_S:
            gas = self.SIM_HOVER_GAS * (t / self.SIM_TAKEOFF_S)          # взлёт
        else:
            gas = self.SIM_HOVER_GAS \
                + 0.05 * math.sin(t * 2 * math.pi / 9.0) \
                + 0.02 * math.sin(t * 2 * math.pi / 2.7)                  # висение с подруливанием
            punch = t % self.SIM_PUNCH_PERIOD_S
            if punch < self.SIM_PUNCH_S:                                  # «газ в пол»
                gas += self.SIM_PUNCH_GAS * (1.0 - punch / self.SIM_PUNCH_S)
        gas = max(0.0, min(1.0, gas))
        count = self._motor_count()                    # QUADX — четыре мотора, остальные стоят
        outputs = []
        for i in range(MAX_SUPPORTED_MOTORS):
            if i >= count:
                outputs.append(1000.0)
                continue
            wobble = 0.012 * math.sin(now * (9.0 + i) + i * 1.7)          # моторы неидеальны
            outputs.append(1000.0 + 1000.0 * max(0.0, min(1.0, gas + wobble)))
        return outputs

    def _sim_flight_step(self, now: float) -> None:
        """Шаг сценария: фазы полёта, посадка при разряде, «замена батареи».

        Разрядилась (расход дошёл до ёмкости) — садимся, ждём SIM_SWAP_S и ставим
        новую батарею: счётчик мА·ч обнуляется, напряжение снова 4.20 В на банку.
        Так вкладка «Питание и батарея» проходит весь цикл: OK → предупреждение →
        посадка → новая батарея.
        """
        if not self.sim_flight:
            return
        phase = self.sim_flight_phase
        if phase not in ("flight", "landing", "swap"):
            phase = self.sim_flight_phase = "flight"
            self.sim_flight_since = now
        t = now - self.sim_flight_since
        capacity = self._battery_setting("bat_capacity", 0, 20000)
        if phase == "flight" and capacity > 0 and self.mah_drawn >= capacity:
            self.sim_flight_phase = "landing"
            self.sim_flight_since = now
            self.sim_flight_note = ("Батарея разряжена (расход дошёл до ёмкости) — посадка, "
                                    f"затем новая батарея")
            self._log("[SIM] батарея разряжена: посадка")
        elif phase == "landing" and t >= self.SIM_LAND_S:
            self.sim_flight_phase = "swap"
            self.sim_flight_since = now
        elif phase == "swap" and t >= self.SIM_SWAP_S:
            self.mah_drawn = 0.0
            self.motor_mah = [0.0] * MAX_SUPPORTED_MOTORS
            self.sim_pack_events += 1
            self.sim_flight_phase = "flight"
            self.sim_flight_since = now
            self.sim_flight_note = (f"Поставлена новая батарея (#{self.sim_pack_events}): "
                                    f"{self.battery_cells * self.SIM_PACK_FULL_V:.2f} В, 0 мА·ч")
            self._log(f"[SIM] новая батарея #{self.sim_pack_events}, расход сброшен")

    def _h_analog(self, w, r):
        """MSP_ANALOG (110) — устаревшая телеметрия в 0.1 В: то же, что MSP_BATTERY_STATE."""
        self._motors_step()   # просадка напряжения считается по току моторов
        voltage = self._battery_voltage()
        w.u8(int(voltage * 10) & 0xFF)
        w.u16(self._mah_drawn())
        w.u16(1023)                                        # RSSI 100 %
        w.i16(int(round(self._battery_current() * 100)))
        w.u16(int(round(voltage * 100)))

    def _h_battery_state(self, w, r):
        """MSP_BATTERY_STATE (130): 11 байт — банки, ёмкость, напряжение, мА·ч, ток, статус."""
        self._motors_step()   # просадка напряжения считается по току моторов
        voltage = self._battery_voltage()
        self._mah_update()
        w.u8(self._battery_cell_count())
        w.u16(self._battery_setting("bat_capacity", 0, 20000))
        w.u8(int(voltage * 10) & 0xFF)
        w.u16(int(self.mah_drawn))
        w.i16(int(round(self._battery_current() * 100)))
        w.u8(self._battery_state_code())
        w.u16(int(round(voltage * 100)))

    def _h_voltage_meters(self, w, r):
        """MSP_VOLTAGE_METERS (128): id метра и напряжение (после фильтра, в 0.1 В)."""
        voltage = self._battery_voltage()
        w.u8(VOLTAGE_METER_ID_BATTERY_1)
        w.u8(int(round(voltage * 10)))

    def _h_current_meters(self, w, r):
        """MSP_CURRENT_METERS (129): для каждого метра id, мА·ч и ток в мА (0.001 А)."""
        self._mah_update()
        milliamps = int(round(self._battery_current() * 1000))
        drawn = int(self.mah_drawn)
        w.u8(CURRENT_METER_ID_BATTERY_1).u16(drawn).u16(milliamps)
        if CURRENT_SENSOR_ADC_COUNT > 1:
            w.u8(CURRENT_METER_ID_VIRTUAL_1).u16(drawn).u16(milliamps)

    def _h_rc(self, w, r):
        """MSP_RC (105): 16 каналов в порядке A/E/R/T — как rcData[] в прошивке.

        Прошивка хранит rcData[] в порядке `rcChannelLetters = "AERT…"` (rx.c), то есть
        индекс 2 — это РЫСК, а индекс 3 — ГАЗ. Configurator подписывает первые четыре полосы
        вкладки жёстко (крен/тангаж/рыск/газ) и вращает модель предпросмотра по каналам 0…2,
        поэтому перепутанные рыск с газом превращаются в «коптер постоянно вращается».

        На столе: стики ровно в центре, газ внизу, тумблеры выключены. Стики отдаём без
        микрошума намеренно: предпросмотр интегрирует поворот от каналов 0…2, и даже ±2 мкс
        шума при ACTUAL-рейтах (rc_rate 7000) дают ~30 °/с — модель уезжает. «Живость» видно
        по AUX 1: там ±2 мкс ни на что не влияют (режимы включаются от 1300 мкс).
        """
        if not self.rx_serial_ready():
            # Во вкладке «Порты» нет ни одного UART с функцией Serial RX: прошивка не
            # поднимает приёмник, channelCount = 0 — MSP_RC отдаёт пустой ответ.
            return

        self._resolve_bind()          # биндинг закончился по времени — кадры пошли
        # rx.c: кадры есть — rcData из эфира; кадров нет — «Этап 1»/«Этап 2» (см. _rx_link_values)
        values = self._rx_link_values(self.rc_channels)

        t = self._t()
        for index in range(RC_CHANNEL_COUNT):
            value = values[index]
            if index == NON_AUX_CHANNEL_COUNT and self.rx_link_connected:
                value += int(math.sin(t * 1.7) * 2)   # AUX 1 — «живой» тумблер
            w.u16(max(TX_MIN_USEC - 100, min(TX_MAX_USEC + 100, int(value))))

    # ------------------------------------------------------------------ моторы: модель и MSP
    def motor_telemetry_active(self) -> bool:
        """Есть ли данные ESC: бидирекшн DShot или датчик ESC — то же условие, что в msp.c."""
        return bool(self._setting("dshot_bidir", 0)) or "ESC_SENSOR" in self.features

    def motor_invalid_percent(self, index: int) -> int:
        """Доля «битых» кадров ESC в сотых долях процента (как её отдаёт MSP_MOTOR_TELEMETRY).

        Модель та же, что и у вкладки «Моторы»: у вращающегося мотора ошибок чуть больше.
        """
        rpm = int(round(self.motor_rpm[index]))
        invalid = 3 + (5 if rpm > 500 else 0) + int(2 * abs(math.sin(time.time() * 3.1 + index)))
        return min(invalid, 99)


    # ═════════════════════════════════════════════════════ «демо failsafe» (страница /failsafe)
    # Источники поведения — 4.5.4 (копии в docs/bf454-failsafe/):
    #   flight/failsafe.c — failsafeUpdateState(): страж-время (failsafe_delay), JustDisarm,
    #       «Этап 2» по процедуре, KILL/STAGE2 тумблер, восстановление (failsafe_recovery_delay);
    #   rx/rx.c — значения каналов: 0…300 мс держим последние, затем «Этап 1» (rxfail),
    #       в «Этапе 2» стики в центр, газ — failsafe_throttle, AUX — по rxfail;
    #   fc/rc_controls.c — calculateThrottleStatus(): газ ниже min_check = «низкий газ»;
    #   flight/gps_rescue.c — фазы спасения и проверки (спутники, sanity checks, удар).
    # Демо ничего не выдумывает: времена и порядок переходов прошивочные. Масштаб времени
    # (fs_scale) только растягивает показ: ×0.25 — 1.5 с страж-времени идут 6 секунд.
    FS_IDLE = 0
    FS_RX_LOSS_DETECTED = 1
    FS_LANDING = 2
    FS_LANDED = 3
    FS_RX_LOSS_MONITORING = 4
    FS_RX_LOSS_RECOVERED = 5
    FS_GPS_RESCUE = 6
    FS_PHASE_EN = ("IDLE", "RX_LOSS_DETECTED", "LANDING", "LANDED",
                   "RX_LOSS_MONITORING", "RX_LOSS_RECOVERED", "GPS_RESCUE")
    FS_PHASE_RU = ("дежурство (связь есть)",
                   "потеря сигнала: страж-время (Этап 1)",
                   "Этап 2: посадка (газ убирается)",
                   "Этап 2: моторы выключены",
                   "ждём восстановления связи",
                   "сигнал вернулся — выходим из failsafe",
                   "Этап 2: GPS-спасение")
    FS_SCALES = (1.0, 0.5, 0.25)
    FS_HOLD_MS = 300              # rx.c: MAX_INVALID_PULSE_TIME_MS — держим последние значения
    FS_PROCEDURE_NAMES = ("AUTO-LAND", "DROP", "GPS-RESCUE")
    FS_PROCEDURE_RU = ("посадка на месте", "сразу выключить моторы (падение)",
                       "GPS-спасение: вернуться домой")
    FS_SWITCH_MODE_NAMES = ("STAGE1", "KILL", "STAGE2")
    FS_SWITCH_MODE_EN = ("acts as signal loss (stage 1)",
                         "KILL - motors off at once",
                         "stage 2 at once, no guard time")
    FS_SWITCH_MODE_RU = ("как потеря сигнала (Этап 1)",
                         "сразу выключить моторы",
                         "сразу Этап 2, не дожидаясь страж-времени")
    FS_RESCUE_PHASES = ("IDLE", "INITIALIZE", "ATTAIN_ALT", "ROTATE", "FLY_HOME",
                        "DESCENT", "LANDING", "DO_NOTHING", "ABORT", "COMPLETE")
    FS_RESCUE_RU = ("не идёт", "инициализация", "набор высоты", "разворот домой",
                    "полёт домой", "снижение", "посадка", "неуправляемый спуск (20 с)",
                    "прервано: моторы выключены", "завершено")
    FS_RESCUE_FAILURES = ("HEALTHY", "FLYAWAY", "GPSLOST", "LOWSATS", "CRASH_FLIP",
                          "STALLED", "TOO_CLOSE", "NO_HOME_POINT")
    FS_RESCUE_FAILURES_EN = ("checks passed", "flyaway - drifting away from home",
                             "GPS fix lost", "too few satellites", "flip / crash detected",
                             "rescue is not gaining altitude", "too close to home",
                             "no home point")
    FS_RESCUE_FAILURES_RU = ("проверки пройдены", "уносит от дома (flyaway)",
                             "потерян GPS-фикс", "мало спутников", "переворот (краш)",
                             "спасение не набирает высоту", "слишком близко к дому",
                             "нет точки дома")

    def _fs_init_state(self) -> None:
        """Начальное состояние демо — как при включении борта (борт на столе, связь есть)."""
        self.fs_demo = False               # демо включено
        self.fs_scale = 0.25               # масштаб времени показа (×1 … ×0.25)
        self.fs_ms = 0.0                   # «прошивочные» миллисекунды демо
        self.fs_phase = self.FS_IDLE
        self.fs_active = False             # failsafeIsActive(): идёт «Этап 2»
        self.fs_events = 0
        self.fs_armed = False
        self.fs_was_ever_armed = False
        self.fs_valid_failed_at = None     # время, с которого сигнал «плохой» (мс демо)
        self.fs_recovery_started = None    # сигнал вернулся: отсчёт failsafe_recovery_delay
        self.fs_receiving_period = 0.0     # до какого времени связь ещё «не считается поднятой»
        self.fs_landing_finish_at = 0.0
        self.fs_throttle_low_period = 0.0
        self.fs_box_was_on = False
        self.fs_rollback_ms = None         # ТОЧКА ОТКАТА: момент перехода к «Этапу 2»
        self.fs_stage2_ms = None           # время начала «Этапа 2» (для таймлайна)
        self.fs_log = []                   # [(мс, текст)] — журнал событий
        self.fs_beeper = ""                # что «пищит» борт (BEEPER_RX_LOST и т.п.)
        self.fs_disarm_reason = ""
        self.fs_switch_on_ms = None        # когда включён тумблер FAILSAFE
        self.fs_sticks_active = False
        self.fs_receiving_period_preset = 100.0   # мс: задержка восстановления (failsafe.c)
        self.fs_receiving = True                  # снимок failsafeIsReceivingRxData() на шаге
        self.fs_rescue = {"phase": 0, "alt_m": 0.0, "dist_m": 0.0, "failure": 0,
                          "throttle": 1000, "note": "", "checks": []}
        self.fs_gps_north = None           # сценарий спасения двигает точку домой
        self.fs_gps_east = None
        self.fs_gps_alt = None
        self.fs_gps_prev = None
        self.fs_exit_hint = ""
        self.fs_scenario = "none"          # сценарий проверок: none | low_sats | hover_low

    # ---------------------------------------------------------------- конфигурация прошивки
    def _fs_config(self) -> dict:
        """Значения вкладки Failsafe так, как их читает прошивка (failsafeConfig_t)."""
        return {
            "delay_ms": max(100, self._setting("failsafe_delay", 15) * 100),
            "off_delay_ms": int(self._setting("failsafe_off_delay", 10)) * 100,
            "throttle": int(self._setting("failsafe_throttle", 1000)),
            "switch_mode": int(self._setting("failsafe_switch_mode", 0)),
            "throttle_low_ms": int(self._setting("failsafe_throttle_low_delay", 100)) * 100,
            "procedure": int(self._setting("failsafe_procedure", 1)),
            "recovery_ms": max(100, self._setting("failsafe_recovery_delay", 5) * 100),
            "stick_threshold": int(self._setting("failsafe_stick_threshold", 30)),
        }

    def _fs_rescue_config(self) -> dict:
        """Параметры GPS-спасения (вкладка Failsafe, блок «Этап 2 — Процедура Failsafe»)."""
        return {
            "min_sats": int(self._setting("gps_rescue_min_sats", 8)),
            "sanity": int(self._setting("gps_rescue_sanity_checks", 2)),
            "allow_no_fix": int(self._setting("gps_rescue_allow_arming_without_fix", 0)),
            "throttle_hover": int(self._setting("gps_rescue_throttle_hover", 1275)),
            "initial_climb": int(self._setting("gps_rescue_initial_climb", 10)),
            "ascend_rate": int(self._setting("gps_rescue_ascend_rate", 750)) / 100.0,   # м/с
            "descend_rate": int(self._setting("gps_rescue_descend_rate", 150)) / 100.0,  # м/с
            "return_alt": int(self._setting("gps_rescue_return_alt", 30)),
            "ground_speed": int(self._setting("gps_rescue_ground_speed", 750)) / 100.0,  # м/с
            "min_start_dist": int(self._setting("gps_rescue_min_start_dist", 15)),
            "disarm_threshold": int(self._setting("gps_rescue_disarm_threshold", 20)) / 10.0,
            "landing_alt": int(self._setting("gps_rescue_landing_alt", 4)),
            "descent_dist": int(self._setting("gps_rescue_descent_dist", 20)),
            "alt_mode": int(self._setting("gps_rescue_alt_mode", 0)),
            "max_angle": int(self._setting("gps_rescue_max_angle", 45)),
            "use_mag": int(self._setting("gps_rescue_use_mag", 1)),
        }

    # ---------------------------------------------------------------- управление демо
    def set_sim_failsafe(self, on: bool) -> dict:
        """Включить/выключить демо failsafe (кнопка страницы /failsafe, команда CLI).

        Включение: борт «в воздухе и взведён», связь есть, все таймеры с нуля; если был
        включён «виртуальный полёт» — выключаем его (моторы пишет сценарий failsafe).
        Выключение: борт снова на столе — моторы на 1000, фазы сброшены.
        """
        on = bool(on)
        if on:
            if not self.fs_demo:
                if self.sim_flight:
                    self.set_sim_flight(False)
                self._fs_init_state()
                self.fs_demo = True
                self.fs_armed = True
                self.fs_was_ever_armed = True
                self.fs_throttle_low_period = 0.0
                self.fs_log = []
                self._fs_hold_airborne()
                self._fs_log("демо включено: борт в воздухе (газ висения), связь есть, борт взведён",
                             "demo ON: craft airborne (hover throttle), link OK, armed")
                self._log("[SIM] демо failsafe включено (борт взведён, связь есть)")
        elif self.fs_demo:
            self.fs_demo = False
            self.fs_armed = False
            self.motor_disarmed = [1000.0] * MAX_SUPPORTED_MOTORS
            self.fs_gps_north = self.fs_gps_east = self.fs_gps_alt = None
            self.fs_scenario = getattr(self, "fs_scenario", "none")
            self._log("[SIM] демо failsafe выключено (борт на столе, моторы на 1000)")
        return self.sim_failsafe_state()

    def sim_failsafe_loss(self) -> dict:
        """«Потерять сигнал»: выдёргиваем аппаратуру — дальше всё решает прошивка.

        Как «Отключить (выдернуть USB)» на странице аппаратуры: `rx_link_connected = False`.
        Дальше идут страж-время, «Этап 1», «Этап 2» — по таймерам вкладки Failsafe.
        """
        if not self.fs_demo:
            self.set_sim_failsafe(True)
        self.set_rx_link(False)
        return self.sim_failsafe_state()

    def sim_failsafe_restore(self) -> dict:
        """«Вернуть сигнал»: аппаратура снова в эфире.

        До «точки отката» борт возвращается в строй; после неё процедура уже выполняется —
        видно, что сигнал вернулся поздно (или что управление вернётся только после
        failsafe_recovery_delay и фазы восстановления).
        """
        self.set_rx_link(True)
        return self.sim_failsafe_state()

    def sim_failsafe_reset(self) -> dict:
        """«Сброс демо»: связь есть, борт в воздухе и взведён, журнал очищен.

        Масштаб показа (×1/×0.5/×0.25) не сбиваем: это настройка показа для аудитории,
        а не часть сценария — преподаватель выбрал её один раз на всю демонстрацию.
        """
        scale = getattr(self, "fs_scale", 0.25)
        self._fs_init_state()      # журнал, фазы и таймеры — с нуля
        self.fs_scale = scale
        self.fs_demo = True        # «Сброс демо» оставляет демонстрацию включённой
        self.fs_armed = True
        self.fs_was_ever_armed = True
        self.set_rx_link(True)
        self.fs_bind_state = "bound"
        self._fs_hold_airborne()
        self._fs_log("демо сброшено: связь есть, борт в воздухе (газ висения), борт взведён",
                     "demo reset: link OK, craft airborne, armed")
        return self.sim_failsafe_state()

    def _fs_hold_airborne(self) -> None:
        """Борт «в воздухе»: газ висения на виртуальной аппаратуре (1500 мкс).

        Демо показывает полёт, поэтому газ поднят — иначе сработал бы JustDisarm
        (газ ниже min_check дольше failsafe_throttle_low_delay), как на столе.
        """
        self.set_rc_channels({"throttle": RX_MID_USEC})

    def set_sim_failsafe_box(self, on=True) -> dict:
        """Щёлкнуть тумблером FAILSAFE (режим во вкладке «Режимы») — как это делает пилот.

        Если режим FAILSAFE ни на чём не назначен, назначаем его на AUX1 (1300…2100) —
        ровно так его настраивают во вкладке «Режимы»; это видно в журнале демо.
        Тумблер действует и без потери связи: каналы становятся недействительными (rx.c:688),
        а дальше всё решает failsafe_switch_mode: STAGE1 — страж-время, KILL — моторы сразу,
        STAGE2 — Этап 2 немедленно.
        """
        on = bool(on)
        slot = next((s for s in sorted(self.mode_ranges, key=lambda s: s["index"])
                     if s["box"] == "FAILSAFE" and self._range_configured(s)), None)
        if slot is None:
            index = next((s["index"] for s in sorted(self.mode_ranges, key=lambda s: s["index"])
                          if not self._range_configured(s)), 0)
            self.set_mode_range(index, self.box_permanent_id("FAILSAFE"), 0, 1300, 2100)
            slot = self.mode_ranges[index]
            self._fs_log("режим FAILSAFE назначен на AUX1 (1300–2100) — как во вкладке «Режимы»",
                         "FAILSAFE mode assigned to AUX1 (1300-2100) as in the Modes tab")
        # внутри диапазона строго: isRangeActive() проверяет start <= value < end
        value = slot["end"] - 25 if on else max(CHANNEL_RANGE_MIN, slot["start"] - 200)
        self.set_rc_channel(NON_AUX_CHANNEL_COUNT + slot["aux"], value)
        self.fs_box_was_on = on
        mode_text = self.FS_SWITCH_MODE_RU[self._fs_config()["switch_mode"]]
        self._fs_log(f"тумблер FAILSAFE {'включён' if on else 'выключен'} "
                     f"(AUX{slot['aux'] + 1} = {value} мкс, режим {mode_text})",
                     f"FAILSAFE box switch {'ON' if on else 'off'} "
                     f"(AUX{slot['aux'] + 1} = {value} us, {mode_text})")
        return self.sim_failsafe_state()

    def sim_failsafe_arm(self, armed=True, throttle=None) -> dict:
        """Взвести борт: «в воздухе» (газ 1500), «на земле» (газ низкий) или выключить моторы.

        Кнопки страницы /failsafe: «Взлететь», «Стоять на земле (взведён)», «Выключить моторы».
        """
        armed = bool(armed)
        if throttle is not None:
            self.set_rc_channels({"throttle": int(throttle)})
        if armed and not self.fs_armed:
            self.fs_armed = True
            self.fs_was_ever_armed = True
            self.fs_disarm_reason = ""
            if not self._fs_throttle_low():
                self.fs_throttle_low_period = self.fs_ms + self._fs_config()["throttle_low_ms"]
            self._fs_log(f"борт взведён, газ {int(self.rc_channels[3])} мкс",
                         f"armed, throttle {int(self.rc_channels[3])} us")
        elif not armed and self.fs_armed:
            self.fs_armed = False
            self.fs_disarm_reason = "вручную"
            self._fs_log("моторы выключены (disarm вручную)",
                         "motors stopped (disarmed by hand)")
        return self.sim_failsafe_state()

    def set_sim_failsafe_switch(self, mode) -> dict:
        """Режим тумблера FAILSAFE (вкладка «Режимы»): 0 STAGE1, 1 KILL, 2 STAGE2.

        Это та же настройка failsafe_switch_mode, что правит вкладка Failsafe (MSP 75/77) и
        CLI: кнопки страницы /failsafe пишут её через _store_setting(), и машина фаз сразу
        читает новое значение — STAGE1 идёт через страж-время, KILL гасит моторы немедленно,
        STAGE2 входит в «Этап 2» без ожидания (failsafe.c, FAILSAFE_IDLE).
        """
        try:
            value = int(mode)
        except (TypeError, ValueError):
            value = 0
        value = min(2, max(0, value))
        self._store_setting("failsafe_switch_mode", value)
        self._fs_log(f"режим тумблера FAILSAFE: {self.FS_SWITCH_MODE_NAMES[value]} — "
                     f"{self.FS_SWITCH_MODE_RU[value]}",
                     f"FAILSAFE switch mode: {self.FS_SWITCH_MODE_NAMES[value]} - "
                     f"{self.FS_SWITCH_MODE_EN[value]}")
        return self.sim_failsafe_state()

    def set_sim_failsafe_scale(self, value) -> dict:
        """Масштаб времени показа: ×1 (честные тайминги), ×0.5, ×0.25 (для аудитории)."""
        try:
            scale = float(value)
        except (TypeError, ValueError):
            scale = self.fs_scale
        self.fs_scale = scale if scale in self.FS_SCALES else min(self.FS_SCALES,
                                                                 key=lambda s: abs(s - scale))
        self._fs_log(f"масштаб времени показа ×{self.fs_scale:g}",
                     f"show speed x{self.fs_scale:g} (1 = real firmware timing)")
        return self.sim_failsafe_state()

    FS_SCENARIOS = ("none", "low_sats", "hover_low")
    FS_SCENARIO_NAMES = ("обычный полёт домой", "мало спутников",
                         "газа зависания не хватает (не набирает высоту)")

    def set_sim_failsafe_scenario(self, name) -> dict:
        """Сценарий проверок GPS-спасения: обычный / мало спутников / газа зависания не хватает.

        Это единственное, что демо «подстраивает» под сценарий: дальше срабатывают
        прошивочные проверки gps_rescue.c (RESCUE_LOWSATS, переход в RESCUE_LANDING,
        если борт 10 с не набирает/не теряет высоту как задумано).
        """
        key = str(name or "").strip().lower()
        aliases = {"none": "none", "обычный": "none", "ok": "none",
                   "low_sats": "low_sats", "спутники": "low_sats", "sats": "low_sats",
                   "hover_low": "hover_low", "газ": "hover_low", "hover": "hover_low"}
        chosen = aliases.get(key, key if key in self.FS_SCENARIOS else "none")
        self.fs_scenario = chosen
        text = self.FS_SCENARIO_NAMES[self.FS_SCENARIOS.index(chosen)]
        self._fs_log(f"сценарий проверок GPS-спасения: {text}",
                     f"GPS rescue check scenario: {chosen}")
        return self.sim_failsafe_state()

    def _fs_log(self, text: str, en: str = "") -> None:
        """Событие в журнал демо: «1.5 с — сработал failsafe, Этап 1».

        Русский текст — для страницы /failsafe и монитора, английский — для CLI: в консоли
        Configurator'а кириллица приходит мусором, поэтому журнал ведём на двух языках.
        """
        self.fs_log.append((round(self.fs_ms / 1000.0, 2), text, en or text))
        del self.fs_log[:-40]

    # ---------------------------------------------------------------- связь и каналы
    def _fs_box_on(self) -> bool:
        """IS_RC_MODE_ACTIVE(BOXFAILSAFE): включён ли режим FAILSAFE тумблером (вкладка «Режимы»)."""
        return "FAILSAFE" in self.active_mode_names()

    def _fs_signal_bad(self) -> bool:
        """rx.c: rxFlightChannelsValid = rxSignalReceived && !BOXFAILSAFE (rx.c:688).

        Тумблер FAILSAFE делает каналы недействительными при любой настройке (STAGE1/KILL/STAGE2) —
        отличается только то, что дальше делает машина фаз.
        """
        return (not self.rx_link_connected) or self._fs_box_on()

    def _fs_receiving(self) -> bool:
        """failsafeIsReceivingRxData(): rxLinkState == UP.

        Становится false по истечении страж-времени (failsafe_delay), становится true
        сразу после возврата тумблера или через failsafe_recovery_delay после возврата связи.
        """
        cfg = self._fs_config()
        if self._fs_signal_bad():
            self.fs_recovery_started = None
            if self.fs_valid_failed_at is None:
                self.fs_valid_failed_at = self.fs_ms
                self._fs_log(f"сигнала нет: пошло страж-время {cfg['delay_ms'] / 1000:.1f} с",
                             "no signal: stage 1 guard time "
                             f"{cfg['delay_ms'] / 1000:.1f} s started")
            return (self.fs_ms - self.fs_valid_failed_at) < cfg["delay_ms"]
        if self.fs_valid_failed_at is not None:
            self._fs_log("сигнал вернулся до точки отката: управление вернулось, Этап 1 прерван",
                         "signal is back before the rollback point: control returned, stage 1 aborted")
        self.fs_valid_failed_at = None
        if self.fs_phase != self.FS_IDLE or self.fs_active or self.fs_recovery_started is not None:
            if self.fs_recovery_started is None:
                self.fs_recovery_started = self.fs_ms
                if self.fs_receiving_period_preset <= 0:
                    self._fs_log("сигнал вернулся: тумблерный failsafe выходит сразу "
                                 "(receivingRxDataPeriodPreset = 0)",
                                 "signal is back: switch-induced failsafe recovers at once "
                                 "(receivingRxDataPeriodPreset = 0)")
                else:
                    self._fs_log(f"сигнал вернулся: ждём {self.fs_receiving_period_preset / 1000:.1f} с "
                                 f"валидных данных (failsafe_recovery_delay)",
                                 f"signal is back: waiting "
                                 f"{self.fs_receiving_period_preset / 1000:.1f} s of valid data "
                                 "(failsafe_recovery_delay)")
            return (self.fs_ms - self.fs_recovery_started) >= self.fs_receiving_period_preset
        return True

    def _fs_throttle_low(self) -> bool:
        """rc_controls.c: THROTTLE_LOW — газ ниже min_check (вкладка «Приёмник», rxConfig)."""
        channels = self._rx_link_values(self.rc_channels)
        return channels[3] < self._setting("min_check", 1050)

    def _fs_stick_deflection(self) -> int:
        """Насколько стики ушли от центра (в процентах от половины хода) — для выхода из спасения."""
        channels = self._rx_link_values(self.rc_channels)
        deflection = max(abs(channels[0] - RX_MID_USEC), abs(channels[1] - RX_MID_USEC),
                         abs(channels[2] - RX_MID_USEC))
        return int(round(deflection / 5.0))     # 500 мкс = 100 %

    def _rxfail_values(self, channels: list) -> list:
        """«Этап 1» из rx.c: AUTO — центр (газ — rx_min_usec), HOLD — последнее, SET — заданное."""
        values = []
        for index, channel in enumerate(self._rxfail_channels()):
            mode = channel["mode"]
            if mode == RX_FAILSAFE_MODE_HOLD:
                values.append(channels[index])
            elif mode == RX_FAILSAFE_MODE_SET:
                values.append(channel["value"])
            elif index == 3:                     # THROTTLE: AUTO → rx_min_usec
                values.append(RX_MIN_USEC)
            else:                                # ROLL/PITCH/YAW и AUX: AUTO → midrc
                values.append(RX_MID_USEC)
        return values

    def _rx_link_values(self, channels: list) -> list:
        """Значения каналов, которые прошивка отдаёт приложению (rx.c: rcData[]).

        Кадров нет: первые FS_HOLD_MS держим последние значения (rx.c HOLD), затем идут
        значения «Этапа 1» (rxfail), а в «Этапе 2» — стики в центр, газ failsafe_throttle,
        у AUX-каналов значения «Этапа 1» (rx.c, ветка failsafeIsActive()).
        """
        if self.rx_link_connected and not self._fs_signal_bad():
            return list(channels)
        values = self._rxfail_values(channels)
        if not self.fs_demo:
            return values                        # без демо — как раньше: сразу «Этап 1»
        if self.fs_active:
            values[0] = values[1] = values[2] = RX_MID_USEC
            values[3] = self._fs_config()["throttle"]
            return values
        elapsed = self.fs_ms - self.fs_valid_failed_at if self.fs_valid_failed_at is not None \
            else self.fs_ms - (self.fs_switch_on_ms or self.fs_ms)
        if elapsed < self.FS_HOLD_MS:
            return list(channels)                # rx.c: MAX_INVALID_PULSE_TIME_MS
        return values

    # ---------------------------------------------------------------- машина фаз
    def _fs_step(self, dt: float) -> None:
        """Шаг демо: часы (с масштабом показа), затем failsafeUpdateState() и сценарий спасения."""
        if not self.fs_demo:
            return
        # Часы демо идут по-настоящему: шаг — сколько времени реально прошло, но не больше
        # секунды (если эмулятор долго не опрашивали, показ не «проскакивает» целые фазы).
        step = min(dt, 1.0)
        self.fs_ms += step * 1000.0 * self.fs_scale
        box_on = self._fs_box_on()
        if box_on and self.fs_switch_on_ms is None:
            self.fs_switch_on_ms = self.fs_ms
            self._fs_log("тумблер FAILSAFE включён (вкладка «Режимы»)",
                         "FAILSAFE box switch is ON (Modes tab)")
        elif not box_on and self.fs_switch_on_ms is not None:
            self.fs_switch_on_ms = None
            self._fs_log("тумблер FAILSAFE выключен",
                         "FAILSAFE box switch is off")
        self.fs_sticks_active = self._fs_stick_deflection() >= self._fs_config()["stick_threshold"]
        self.fs_receiving = self._fs_receiving()
        self._fs_update_state()
        if self.fs_phase == self.FS_GPS_RESCUE:
            self._fs_rescue_step(step)      # тот же шаг, что у часов демо

    def _fs_update_state(self) -> None:
        """Порт failsafeUpdateState() из failsafe.c 4.5.4 (с циклом reprocessState)."""
        cfg = self._fs_config()
        now = self.fs_ms
        armed = self.fs_armed
        box_on = self._fs_box_on()
        beeper = ""
        reprocess = True
        guard = 0
        while reprocess:
            reprocess = False
            guard += 1
            if guard > 64:
                # страховка демо: любое зацикливание машины фаз останавливаем,
                # а не «вешаем» эмулятор (иначе страница и монитор перестают отвечать)
                self._fs_log("машина фаз: слишком много переходов за шаг — переходы остановлены "
                             "(защита демо)", "failsafe state machine: too many transitions "
                             "in one step - stopped (demo guard)")
                break
            receiving = self._fs_receiving()
            if box_on and cfg["switch_mode"] == 2:
                receiving = False              # STAGE2: «как будто сигнала нет» сразу
            if not receiving and (armed or self.fs_was_ever_armed):
                beeper = "RX_LOST"

            if self.fs_phase == self.FS_IDLE:
                self.fs_box_was_on = box_on
                if armed:
                    if not self._fs_throttle_low():
                        self.fs_throttle_low_period = now + cfg["throttle_low_ms"]
                    if self.fs_box_was_on and cfg["switch_mode"] == 1:
                        self._fs_log("тумблер FAILSAFE = KILL: моторы выключены сразу",
                                     "FAILSAFE switch mode KILL: motors off at once")
                        self.fs_active = True
                        self.fs_events += 1
                        self.fs_rollback_ms = now
                        self.fs_stage2_ms = now
                        self.fs_phase = self.FS_LANDED
                        # failsafe.c: receivingRxDataPeriodPreset = rxDataRecoveryPeriod —
                        # тумблерный KILL восстанавливается по failsafe_recovery_delay
                        self.fs_receiving_period_preset = cfg["recovery_ms"]
                        reprocess = True
                    elif not receiving:
                        if now > self.fs_throttle_low_period and cfg["procedure"] != 2:
                            self.fs_active = True
                            self.fs_events += 1
                            self.fs_rollback_ms = now
                            self.fs_stage2_ms = now
                            self.fs_phase = self.FS_LANDED
                            self.fs_receiving_period_preset = cfg["recovery_ms"]
                            self._fs_log("JustDisarm: газ ниже min_check дольше "
                                         f"failsafe_throttle_low_delay ({cfg['throttle_low_ms'] / 1000:.0f} с) — "
                                         f"моторы выключены без процедуры (failsafe.c)",
                                         "JustDisarm: throttle below min_check for longer than "
                                         f"failsafe_throttle_low_delay ({cfg['throttle_low_ms'] / 1000:.0f} s) "
                                         "- motors off without a procedure (failsafe.c)")
                            reprocess = True
                        else:
                            self.fs_phase = self.FS_RX_LOSS_DETECTED
                            self._fs_log(f"страж-время {cfg['delay_ms'] / 1000:.1f} с истекло: "
                                         f"сработал failsafe, процедура "
                                         f"{self.FS_PROCEDURE_NAMES[cfg['procedure']]}",
                                         f"guard time {cfg['delay_ms'] / 1000:.1f} s expired: failsafe "
                                         f"fired, procedure {self.FS_PROCEDURE_NAMES[cfg['procedure']]} - "
                                         "this is the ROLLBACK POINT")
                            reprocess = True
                else:
                    if self.fs_box_was_on:
                        beeper = beeper or ""

            elif self.fs_phase == self.FS_RX_LOSS_DETECTED:
                if receiving:
                    self.fs_phase = self.FS_RX_LOSS_RECOVERED
                else:
                    self.fs_active = True
                    self.fs_events += 1
                    self.fs_rollback_ms = now
                    self.fs_stage2_ms = now
                    procedure = cfg["procedure"]
                    if procedure == 0:
                        self.fs_phase = self.FS_LANDING
                        self.fs_landing_finish_at = now + cfg["off_delay_ms"]
                        self._fs_log(f"Этап 2: посадка, газ убирается за "
                                     f"{cfg['off_delay_ms'] / 1000:.1f} с, затем моторы в 0",
                                     "stage 2: auto landing, throttle fades out over "
                                     f"{cfg['off_delay_ms'] / 1000:.1f} s, then motors to 0 "
                                     "(failsafe_off_delay)")
                    elif procedure == 1:
                        self.fs_phase = self.FS_LANDED
                        self._fs_log("Этап 2: DROP — моторы выключены сразу, этапа посадки нет",
                                     "stage 2: DROP - motors off at once, no landing phase")
                    else:
                        self.fs_phase = self.FS_GPS_RESCUE
                        self._fs_rescue_start(self._fs_rescue_config())
                    # тумблерный failsafe восстанавливается сразу, потеря связи — по
                    # failsafe_recovery_delay (failsafe.c: receivingRxDataPeriodPreset)
                    self.fs_receiving_period_preset = 0 if self.fs_box_was_on else cfg["recovery_ms"]
                reprocess = True

            elif self.fs_phase == self.FS_LANDING:
                if receiving:
                    self.fs_phase = self.FS_RX_LOSS_RECOVERED
                    self._fs_log("сигнал вернулся во время посадки: управление возвращается, "
                                 "но борт уже снижался (failsafe.c: FAILSAFE_LANDING → RECOVERED)",
                                 "signal back during landing: control returns, but the craft was "
                                 "already descending (failsafe.c: FAILSAFE_LANDING -> RECOVERED)")
                    reprocess = True
                else:
                    if armed:
                        beeper = "RX_LOST_LANDING"
                    if now > self.fs_landing_finish_at or not armed:
                        self.fs_phase = self.FS_LANDED
                        reprocess = True

            elif self.fs_phase == self.FS_GPS_RESCUE:
                if receiving:
                    if self.fs_sticks_active or self.fs_box_was_on:
                        self.fs_phase = self.FS_RX_LOSS_RECOVERED
                        self._fs_log("спасение прервано: стики ушли дальше "
                                     f"{cfg['stick_threshold']} % (failsafe.c: areSticksActive)",
                                     "rescue stopped: sticks moved beyond "
                                     f"{cfg['stick_threshold']} % (failsafe.c: areSticksActive)")
                        reprocess = True
                else:
                    if armed:
                        beeper = "RX_LOST_LANDING"
                    elif self.fs_phase == self.FS_GPS_RESCUE:
                        self.fs_phase = self.FS_LANDED
                        reprocess = True

            elif self.fs_phase == self.FS_LANDED:
                if self.fs_armed:
                    self.fs_armed = False
                    self.fs_disarm_reason = "FAILSAFE"
                    self._fs_log("Этап 2: борт снят с взвода (disarm), "
                                 "взвод заблокирован до восстановления связи",
                                 "stage 2: disarmed, arming blocked until the link is back")
                self.fs_receiving_period = now + self.fs_receiving_period_preset
                self.fs_phase = self.FS_RX_LOSS_MONITORING
                reprocess = True

            elif self.fs_phase == self.FS_RX_LOSS_MONITORING:
                if receiving:
                    if now >= self.fs_receiving_period:
                        self.fs_phase = self.FS_RX_LOSS_RECOVERED
                        reprocess = True
                else:
                    self.fs_receiving_period = now + self.fs_receiving_period_preset

            elif self.fs_phase == self.FS_RX_LOSS_RECOVERED:
                self.fs_throttle_low_period = now + cfg["throttle_low_ms"]
                self.fs_phase = self.FS_IDLE
                self.fs_active = False
                self.fs_rollback_ms = None
                self.fs_stage2_ms = None
                self.fs_recovery_started = None
                self.fs_valid_failed_at = None
                self._fs_log("failsafe завершён: борт снова в строю (IDLE), взвод разрешён",
                             "failsafe finished: craft back in service (IDLE), arming allowed")
                reprocess = True

        if beeper and beeper in self.beepers_disabled:
            beeper = ""            # beeper() (io/beeper.c 4.5.4): выключенный сигнал не пищит
        self.fs_beeper = beeper

    # ---------------------------------------------------------------- GPS-спасение
    def _fs_rescue_start(self, cfg: dict) -> None:
        """Начало спасения: запоминаем точку дома и высоту (gps_rescue.c, RESCUE_INITIALIZE).

        cfg — параметры GPS-спасения вкладки Failsafe (gps_rescue_*), не failsafeConfig_t.
        """
        solution = self.gps_solution()
        # координаты «сейчас» берём из живого GPS (в метрах от дома) и дальше ведём их сами
        dist = float(solution["home_dist_m"])
        angle = math.radians(solution["home_dir_deg"])
        self.fs_gps_north = dist * math.cos(angle)
        self.fs_gps_east = dist * math.sin(angle)
        self.fs_gps_alt = max(30.0, float(solution["alt_m"]))
        self.fs_gps_prev = (self.fs_gps_north, self.fs_gps_east, self.fs_ms)
        self.fs_rescue = {
            "phase": 1,                        # INITIALIZE
            "prev_dist": None,
            "dist_m": round(math.hypot(self.fs_gps_north, self.fs_gps_east), 1),
            "alt_m": round(self.fs_gps_alt, 1),
            "target_alt": 0.0, "target_landing_alt": 0.0,
            "failure": 0, "throttle": cfg["throttle_hover"], "note": "",
            "checks": [], "started_ms": self.fs_ms, "do_nothing_ms": None,
            "prev_alt": round(self.fs_gps_alt, 2), "progress_at": None, "seconds_failing": 0,
        }
        self._fs_log(f"GPS-спасение: борт в {self.fs_rescue['dist_m']:.0f} м от дома, "
                     f"высота {self.fs_rescue['alt_m']:.0f} м",
                     f"GPS rescue: craft {self.fs_rescue['dist_m']:.0f} m from home, "
                     f"altitude {self.fs_rescue['alt_m']:.0f} m")

    def _fs_rescue_step(self, dt: float) -> None:
        """Сценарий спасения по параметрам вкладки Failsafe (flight/gps_rescue.c 4.5.4)."""
        cfg = self._fs_rescue_config()
        rescue = self.fs_rescue
        solution = self.gps_solution()
        sats = solution["sats"]
        if self.fs_scenario == "low_sats":
            sats = max(0, cfg["min_sats"] - 2)     # сценарий: спутников меньше min_sats
        seconds = dt * self.fs_scale
        # заведомо малый газ зависания: в модели борт не держит высоту (порог модели 1050 мкс)
        hover_low = (self.fs_scenario == "hover_low") or cfg["throttle_hover"] <= 1050
        checks = []
        checks.append(f"точка дома: есть (GPS-фикс, {sats} спутников)")
        if sats < cfg["min_sats"]:
            checks.append(f"спутников {sats} < {cfg['min_sats']} (gps_rescue_min_sats) — "
                          f"спасение не считается доступным")
            rescue["failure"] = 3              # RESCUE_LOWSATS
        if rescue["phase"] not in (7, 8, 9):   # не DO_NOTHING/ABORT/COMPLETE
            reason = rescue["failure"]
            if reason:
                sanity = cfg["sanity"]
                hard = not self.rx_link_connected
                if sanity == 1 or (sanity == 2 and hard):
                    rescue["phase"] = 8        # ABORT
                    self._fs_log(f"проверки спасения не пройдены ({self.FS_RESCUE_FAILURES_RU[reason]}): "
                                 f"sanity checks = {('OFF', 'ON', 'FS_ONLY')[sanity]} → моторы выключены",
                                 f"rescue checks failed ({self.FS_RESCUE_FAILURES_EN[reason]}): "
                                 f"sanity checks = {('OFF', 'ON', 'FS_ONLY')[sanity]} - motors off "
                                 "(RESCUE_ABORT)")
                else:
                    rescue["phase"] = 7        # DO_NOTHING
                    rescue["do_nothing_ms"] = self.fs_ms
                    self._fs_log(f"проверки спасения не пройдены ({self.FS_RESCUE_FAILURES_RU[reason]}): "
                                 f"sanity checks = {('OFF', 'ON', 'FS_ONLY')[sanity]} → "
                                 f"20 с неуправляемого спуска с выключением по удару",
                                 f"rescue checks failed ({self.FS_RESCUE_FAILURES_EN[reason]}): "
                                 f"sanity checks = {('OFF', 'ON', 'FS_ONLY')[sanity]} - "
                                 "20 s of uncontrolled descent, motors off on impact (RESCUE_DO_NOTHING)")
        rescue["checks"] = checks

        west, south = self.fs_gps_north, self.fs_gps_east
        dist = math.hypot(west, south)
        alt = self.fs_gps_alt
        phase = rescue["phase"]
        if phase == 1:                          # INITIALIZE
            rescue["target_landing_alt"] = float(cfg["landing_alt"])
            if dist < 5.0 and alt < cfg["landing_alt"]:
                rescue["phase"] = 8
                self._fs_log("спасение у самого дома и на земле: мгновенный disarm (RESCUE_ABORT)",
                             "rescue started at home and on the ground: immediate disarm "
                             "(RESCUE_ABORT)")
            else:
                if dist < cfg["min_start_dist"]:
                    rescue["target_alt"] = max(10.0, alt + cfg["initial_climb"])
                    rescue["note"] = (f"борт ближе {cfg['min_start_dist']} м от дома — сначала "
                                      f"отлетаем, набор высоты как минимум 10 м")
                else:
                    if cfg["alt_mode"] == 0:    # MAX_ALT
                        rescue["target_alt"] = max(alt, float(cfg["return_alt"]))
                    elif cfg["alt_mode"] == 1:  # FIXED_ALT
                        rescue["target_alt"] = float(cfg["return_alt"])
                    else:                       # CURRENT_ALT
                        rescue["target_alt"] = max(alt, alt + cfg["initial_climb"])
                rescue["phase"] = 2
                self._fs_log(f"высота возврата {rescue['target_alt']:.0f} м "
                             f"(gps_rescue_return_alt = {cfg['return_alt']} м, режим "
                             f"{('MAX_ALT', 'FIXED_ALT', 'CURRENT_ALT')[cfg['alt_mode']]})",
                             f"return altitude {rescue['target_alt']:.0f} m "
                             f"(gps_rescue_return_alt = {cfg['return_alt']} m, "
                             f"{('MAX_ALT', 'FIXED_ALT', 'CURRENT_ALT')[cfg['alt_mode']]})")
        elif phase == 2:                        # ATTAIN_ALT — набор высоты
            if hover_low:
                # газа зависания не хватает: борт «висит» на месте, высота не растёт
                rescue["note"] = (f"газ зависания {cfg['throttle_hover']} мкс не держит борт — "
                                   f"высота не растёт")
                if not rescue.get("hover_note") and self.fs_scenario != "hover_low":
                    rescue["hover_note"] = True
                    self._fs_log(f"газ зависания {cfg['throttle_hover']} мкс (gps_rescue_throttle_hover) "
                                 f"слишком мал — борт не держит высоту",
                                 f"hover throttle {cfg['throttle_hover']} us "
                                 "(gps_rescue_throttle_hover) is too low - the craft cannot hold "
                                 "altitude")
            elif alt < rescue["target_alt"]:
                alt = min(rescue["target_alt"], alt + cfg["ascend_rate"] * seconds)
            elif alt > rescue["target_alt"]:
                alt = max(rescue["target_alt"], alt - cfg["descend_rate"] * seconds)
            else:
                rescue["phase"] = 3
                self._fs_log(f"высота {alt:.0f} м набрана (скорость подъёма "
                             f"{cfg['ascend_rate']:.1f} м/с) — разворот домой",
                             f"altitude {alt:.0f} m reached (ascend rate "
                             f"{cfg['ascend_rate']:.1f} m/s) - rotating to head home")
        elif phase == 3:                        # ROTATE — разворот (1 с)
            if not rescue.get("rotate_until"):
                rescue["rotate_until"] = self.fs_ms + 1000.0
            if self.fs_ms >= rescue["rotate_until"]:
                rescue["phase"] = 4
                self._fs_log("разворот домой выполнен (gps_rescue_yaw_p) — полёт домой",
                             "heading home (gps_rescue_yaw_p) - flying home at "
                             f"{cfg['ground_speed']:.1f} m/s")
        elif phase == 4:                        # FLY_HOME
            step = cfg["ground_speed"] * seconds
            if dist <= cfg["descent_dist"]:
                rescue["phase"] = 5
                self._fs_log(f"осталось {dist:.0f} м (gps_rescue_descent_dist = "
                             f"{cfg['descent_dist']} м) — снижение",
                             f"{dist:.0f} m to home (gps_rescue_descent_dist = "
                             f"{cfg['descent_dist']} m) - starting to descend")
            elif step >= dist:
                rescue["phase"] = 5
            else:
                ratio = (dist - step) / dist if dist else 0.0
                west *= ratio
                south *= ratio
                previous = rescue.get("prev_dist")
                if previous is not None and dist >= previous and dist > cfg["min_start_dist"]:
                    rescue["flyaway_s"] = rescue.get("flyaway_s", 0.0) + seconds
                    if rescue["flyaway_s"] > 10.0:
                        rescue["failure"] = 1
                else:
                    rescue["flyaway_s"] = 0.0
                rescue["prev_dist"] = dist
        elif phase == 5:                        # DESCENT — снижается и продолжает идти домой
            alt = max(0.0, alt - cfg["descend_rate"] * seconds)
            if dist > 3.0:
                # gps_rescue.c: descend() гасит скорость; в демо борт идёт домой, пока не окажется
                # над точкой дома, и оттуда снижается вниз
                ratio = max(0.0, (dist - cfg["ground_speed"] * seconds)) / dist
                west *= ratio
                south *= ratio
            if alt < max(1.0, float(cfg["landing_alt"])):
                rescue["phase"] = 6
                self._fs_log(f"высота ниже gps_rescue_landing_alt = {cfg['landing_alt']} м — посадка",
                             f"altitude below gps_rescue_landing_alt = {cfg['landing_alt']} m - landing")
        elif phase == 6:                        # LANDING
            alt = 0.0
            rescue["throttle"] = max(1000, int(cfg["throttle_hover"] * 0.4))
            rescue["phase"] = 9
            self._fs_log(f"посадка завершена: борт сел в {dist:.0f} м от дома, "
                         f"disarm по gps_rescue_disarm_threshold = {cfg['disarm_threshold']:.1f} м/с²",
                         f"landed {dist:.0f} m from home, disarm on impact per "
                         f"gps_rescue_disarm_threshold = {cfg['disarm_threshold']:.1f} m/s2")
        elif phase == 7:                        # DO_NOTHING — 20 с спуска и выключение
            elapsed = (self.fs_ms - rescue["do_nothing_ms"]) / 1000.0
            alt = max(0.0, alt - 2.0 * seconds)
            rescue["throttle"] = max(1000, int(cfg["throttle_hover"] * 0.5))
            if elapsed >= 20.0 or alt <= 0.5:
                rescue["phase"] = 9
                self._fs_log("20 с неуправляемого спуска истекли: борт на земле, моторы выключены",
                             "20 s of uncontrolled descent expired: craft on the ground, motors off")

        # gps_rescue.c (1 Гц): если борт 10 с не меняет высоту как задумано (ratio <= 0.5) —
        # спасение переходит в посадку на месте (RESCUE_ATTAIN_ALT/DESCENT → RESCUE_LANDING)
        if rescue["phase"] in (2, 5):
            now_s = self.fs_ms / 1000.0
            if rescue["progress_at"] is None:
                rescue["progress_at"] = now_s
                rescue["prev_alt"] = alt
            elif now_s - rescue["progress_at"] >= 1.0:
                seconds = now_s - rescue["progress_at"]
                expected = ((cfg["ascend_rate"] if rescue["phase"] == 2 else cfg["descend_rate"])
                            * seconds)
                actual = abs(alt - float(rescue["prev_alt"]))
                ratio = (actual / expected) if expected > 1e-6 else 1.0
                rescue["seconds_failing"] = max(0, min(10, rescue["seconds_failing"]
                                                      + (-1 if ratio > 0.5 else 1)))
                rescue["prev_alt"], rescue["progress_at"] = alt, now_s
                if rescue["seconds_failing"] >= 10:
                    rescue["seconds_failing"] = 0
                    rescue["phase"] = 6
                    self._fs_log("высота не меняется как задумано 10 с (проверка gps_rescue.c) — "
                                 "посадка на месте, домой борт не вернулся",
                                 "altitude has not changed as intended for 10 s (gps_rescue.c check) "
                                 "- landing in place, the craft did not come home")

        self.fs_gps_alt = alt
        self.fs_gps_north, self.fs_gps_east = west, south
        rescue["alt_m"] = round(alt, 1)
        rescue["dist_m"] = round(math.hypot(west, south), 1)
        if rescue["phase"] in (8, 9):
            # RESCUE_ABORT: disarm(DISARM_REASON_FAILSAFE); RESCUE_COMPLETE: rescueStop() + disarm
            rescue["throttle"] = 1000
            if self.fs_armed:
                self.fs_armed = False
                self.fs_disarm_reason = "GPS-RESCUE"

    # ---------------------------------------------------------------- выходы моторов
    def _fs_outputs(self, now: float) -> list:
        """Выходы моторов в демо: висение, посадка (газ вниз), спасение, выключено."""
        count = self._motor_count()
        outputs = [1000.0] * MAX_SUPPORTED_MOTORS
        if not self.fs_armed:
            return outputs
        cfg = self._fs_config()
        phase = self.fs_phase
        if phase in (self.FS_IDLE, self.FS_RX_LOSS_DETECTED, self.FS_RX_LOSS_MONITORING,
                     self.FS_RX_LOSS_RECOVERED):
            gas = self.SIM_HOVER_GAS + 0.02 * math.sin(now * 2 * math.pi / 9.0)
        elif phase == self.FS_LANDING:
            total = max(0.001, cfg["off_delay_ms"] / 1000.0)
            left = max(0.0, (self.fs_landing_finish_at - self.fs_ms) / 1000.0)
            gas = self.SIM_HOVER_GAS * (left / total)
        elif phase == self.FS_GPS_RESCUE:
            rescue_phase = self.fs_rescue["phase"]
            hover = (self.fs_rescue["throttle"] - 1000.0) / 1000.0
            if rescue_phase in (8, 9):
                # RESCUE_ABORT / RESCUE_COMPLETE: gps_rescue.c делает disarm — моторы в 0
                gas = 0.0
            elif rescue_phase in (6, 7):
                gas = max(0.0, hover * 0.4)      # посадка / неуправляемый спуск
            else:
                gas = hover
            if rescue_phase == 2:
                gas = min(1.0, hover + 0.05)          # набор высоты
        else:                                          # LANDED
            gas = 0.0
        for i in range(count):
            wobble = 0.008 * math.sin(now * (9.0 + i) + i * 1.7)
            outputs[i] = 1000.0 + 1000.0 * max(0.0, min(1.0, gas + wobble))
        return outputs

    # ---------------------------------------------------------------- снимок для страницы
    def sim_failsafe_state(self) -> dict:
        """Состояние демо для страницы /failsafe, монитора и CLI."""
        cfg = self._fs_config()
        rescue_cfg = self._fs_rescue_config()

        def wire(value):
            """Как значение уходит в MSP_RC: та же отсечка по краям, что в _h_rc.
            Поэтому «Этап 1» газа виден в Configurator как 900 (rx.c: 885 мкс, отсечка)."""
            return max(TX_MIN_USEC - 100, min(TX_MAX_USEC + 100, int(value)))

        channels = [wire(v) for v in self._rx_link_values(self.rc_channels)]
        stage1 = [wire(v) for v in self._rxfail_values(self.rc_channels)]
        now = self.fs_ms
        # снимок ничего не меняет: значение обновляет шаг демо (_fs_step → _fs_receiving)
        receiving = self.fs_receiving if self.fs_demo else self.rx_link_connected
        timeline = []
        if self.fs_demo:
            timeline = [
                {"name": "связь есть", "done": bool(self.fs_valid_failed_at), "ms": None,
                 "active": self.fs_valid_failed_at is None},
                {"name": (f"Этап 1: страж-время {cfg['delay_ms'] / 1000:.1f} с, "
                          f"значения каналов — по «Этапу 1» (rxfail)"),
                 "done": bool(self.fs_stage2_ms),
                 "left_ms": max(0.0, cfg["delay_ms"] - (now - self.fs_valid_failed_at))
                 if self.fs_valid_failed_at is not None else cfg["delay_ms"],
                 "active": self.fs_valid_failed_at is not None and not self.fs_stage2_ms},
                {"name": "ТОЧКА ОТКАТА: начало Этапа 2", "done": bool(self.fs_stage2_ms),
                 "left_ms": max(0.0, cfg["delay_ms"] - (now - self.fs_valid_failed_at))
                 if self.fs_valid_failed_at is not None and not self.fs_stage2_ms else None,
                 "active": False, "marker": True},
                {"name": {"0": "посадка + моторы в 0", "1": "моторы сразу в 0",
                          "2": "GPS-спасение"}[str(cfg["procedure"])],
                 "done": self.fs_phase in (self.FS_LANDED, self.FS_RX_LOSS_MONITORING,
                                           self.FS_RX_LOSS_RECOVERED)
                 or (self.fs_phase == self.FS_GPS_RESCUE and self.fs_rescue["phase"] == 9),
                 "active": self.fs_phase in (self.FS_LANDING, self.FS_GPS_RESCUE)
                 or (self.fs_phase == self.FS_LANDED), "left_ms": None},
                {"name": "связь восстановлена → IDLE", "done": self.fs_phase == self.FS_IDLE and bool(self.fs_events),
                 "active": self.fs_phase in (self.FS_RX_LOSS_MONITORING, self.FS_RX_LOSS_RECOVERED),
                 "left_ms": None},
            ]
        # английские названия шагов — для CLI (кириллица в консоли Configurator'а не читается)
        for entry, en in zip(timeline, ("link OK",
                                        "stage 1: guard time and stage 1 channel values (rxfail)",
                                        "ROLLBACK POINT: stage 2 starts",
                                        f"stage 2: {self.FS_PROCEDURE_NAMES[cfg['procedure']]}",
                                        "link back, leaving failsafe -> IDLE")):
            entry["en"] = en
        rollback = None
        if self.fs_demo:
            if self.fs_stage2_ms:
                rollback = {"possible": False, "value": self.fs_stage2_ms, "left_s": None,
                            "text": "откат невозможен: Этап 2 идёт — процедура уже выполняется"}
            elif self.fs_valid_failed_at is not None:
                left = max(0.0, (cfg["delay_ms"] - (now - self.fs_valid_failed_at)) / 1000.0)
                rollback = {"possible": True, "value": self.fs_stage2_ms, "left_s": round(left, 1),
                            "text": f"откат возможен: сигнал вернёт управление, осталось {left:.1f} с "
                                    f"(страж-время)"}
            else:
                rollback = {"possible": True, "value": None, "left_s": None,
                            "text": "связь есть — борт в строю"}
        return {
            "demo": self.fs_demo,
            "scale": self.fs_scale,
            "millis": round(self.fs_ms),
            "phase": self.fs_phase,
            "phase_en": self.FS_PHASE_EN[self.fs_phase],
            "phase_text": (f"Этап 1: страж-время {cfg['delay_ms'] / 1000:.1f} с идёт, "
                           f"каналы — по «Этапу 1» (rxfail)"
                           if self.fs_phase == self.FS_IDLE and self.fs_valid_failed_at is not None
                           and not self.fs_stage2_ms else self.FS_PHASE_RU[self.fs_phase]),
            "active": self.fs_active,
            "events": self.fs_events,
            "armed": self.fs_armed,
            "throttle": int(round(self.rc_channels[3])),
            "was_ever_armed": self.fs_was_ever_armed,
            "link": bool(self.rx_link_connected),
            "receiving": bool(receiving),
            "beeper": self.fs_beeper,
            "disarm_reason": self.fs_disarm_reason,
            "box_failsafe": self._fs_box_on(),
            "sticks_active": self.fs_sticks_active,
            "procedure": cfg["procedure"],
            "procedure_name": self.FS_PROCEDURE_NAMES[cfg["procedure"]],
            "procedure_text": self.FS_PROCEDURE_RU[cfg["procedure"]],
            "switch_mode": cfg["switch_mode"],
            "switch_mode_text": self.FS_SWITCH_MODE_RU[cfg["switch_mode"]],
            "timeline": timeline,
            "rollback": rollback,
            "log": [(ms, ru) for ms, ru, _en in self.fs_log],
            "log_en": [(ms, en) for ms, _ru, en in self.fs_log],
            "channels": [int(round(v)) for v in channels],
            "stage1": [int(round(v)) for v in stage1],
            "config": {
                "delay": cfg["delay_ms"] / 1000.0,
                "off_delay": cfg["off_delay_ms"] / 1000.0,
                "throttle": cfg["throttle"],
                "throttle_low_delay": cfg["throttle_low_ms"] / 1000.0,
                "recovery_delay": cfg["recovery_ms"] / 1000.0,
                "stick_threshold": cfg["stick_threshold"],
            },
            "scenario": self.fs_scenario,
            "scenario_text": self.FS_SCENARIO_NAMES[self.FS_SCENARIOS.index(self.fs_scenario)],
            "rescue": dict(self.fs_rescue, phase_name=self.FS_RESCUE_PHASES[self.fs_rescue["phase"]],
                           phase_text=self.FS_RESCUE_RU[self.fs_rescue["phase"]],
                           failure_name=self.FS_RESCUE_FAILURES[self.fs_rescue["failure"]],
                           failure_text=self.FS_RESCUE_FAILURES_RU[self.fs_rescue["failure"]]) if self.fs_demo else None,
            "rescue_config": rescue_cfg,
            "exit_hint": self.fs_exit_hint,
        }

    def _motor_count(self) -> int:
        """Число моторов борта (getMotorCount()): из настройки motor_count, 4 по умолчанию."""
        return max(1, min(MAX_SUPPORTED_MOTORS, self._setting("motor_count", 4)))

    def _motors_step(self) -> None:
        """Один шаг «физики» моторов: обороты, температура, ток, напряжение, расход.

        Вызывается перед каждым ответом MSP_MOTOR/MSP_MOTOR_TELEMETRY — вкладка опрашивает
        их каждые 50 мс, поэтому шаг получается ровно такой, как нужно для живых цифр.
        """
        now = time.time()
        dt = max(0.0, min(2.0, now - self._motors_t))   # до 2 с «простоя» учитываем целиком
        self._motors_t = now
        if dt <= 0.0:
            return
        if self.fs_demo:
            # «Демо failsafe»: часы демо (с масштабом показа) и машина фаз прошивки —
            # страж-время, «Этап 1»/«Этап 2», посадка или спасение. Выходы моторов берём
            # из сценария failsafe и пишем в motor_disarmed — как mixer.c на взведённом борде.
            self._fs_step(dt)
            self.motor_disarmed = self._fs_outputs(now)
        elif self.sim_flight:
            # «Виртуальный полёт»: сначала шаг сценария (фазы, посадка, замена батареи),
            # затем выходы моторов — пишем их в motor_disarmed, ровно как mixer.c на
            # взведённом борде. MSP_MOTOR (104) и вкладка «Моторы» показывают те же числа.
            self._sim_flight_step(now)
            self.motor_disarmed = self._sim_flight_outputs(now)
        # Интегрируем мелкими шагами по 5 мс: если между опросами прошло много времени
        # (вкладка была закрыта), мотор успевает раскрутиться физически правильно, но
        # обороты не «прыгают» через промежуточные значения.
        steps = max(1, min(400, int(dt / 0.005) + 1))
        for _ in range(steps):
            self._motors_integrate(dt / steps)

    def _motors_integrate(self, dt: float) -> None:
        """Шаг модели моторов на dt секунд (вызывается из _motors_step)."""
        # Напряжение «покоя» — без просадки: её считаем здесь, в одном месте
        # (иначе просадка вычиталась бы дважды — ещё и в _battery_voltage()).
        # «Покой» — с учётом разряда: у разряженной батареи он ниже (см. _pack_rest_voltage).
        rest_v = self._pack_rest_voltage()
        total_current = 0.0
        for i in range(MAX_SUPPORTED_MOTORS):
            output = max(1000.0, min(2000.0, self.motor_disarmed[i]))
            self.motor_output[i] = output
            gas = max(0.0, min(1.0, (output - 1000.0) / 1000.0))

            target_rpm = gas * self.MOTOR_MAX_RPM
            tau = self.MOTOR_SPOOL_TAU_UP if target_rpm >= self.motor_rpm[i] else self.MOTOR_SPOOL_TAU_DOWN
            self.motor_rpm[i] += (target_rpm - self.motor_rpm[i]) * min(1.0, dt / tau)
            if self.motor_rpm[i] < 20.0:
                self.motor_rpm[i] = 0.0

            # Ток и нагрев считаем по ФАКТИЧЕСКИМ оборотам, а не по заданию: раскрученный
            # винт продолжает грузить ESC и на выбеге — ток спадает вместе с оборотами.
            gas_eff = min(1.0, self.motor_rpm[i] / self.MOTOR_MAX_RPM)
            heat = self.MOTOR_HEAT_C_PER_S * gas_eff * gas_eff
            cool = (self.motor_temp_c[i] - self.MOTOR_AMBIENT_C) / self.MOTOR_COOL_TAU_S
            self.motor_temp_c[i] = max(self.MOTOR_AMBIENT_C, self.motor_temp_c[i] + (heat - cool) * dt)

            current = (self.MOTOR_CURRENT_IDLE_A + self.MOTOR_CURRENT_FULL_A * gas_eff * gas_eff) if gas_eff > 0.005 else 0.0
            self.motor_current_a[i] = current
            total_current += current
            self.motor_mah[i] += current * self._sim_scaled_dt(dt) / 3.6   # А·с → мА·ч

        # Напряжение на ESC — напряжение батареи минус просадка от общего тока
        self.motor_voltage_v = max(0.0, rest_v - total_current * self.MOTOR_PACK_RESISTANCE_OHM) if rest_v else 0.0

    def motor_test_active(self) -> bool:
        """Идёт ли тест моторов: выход хотя бы одного мотора выше «стопа» (1000)."""
        return any(value > 1001.0 for value in self.motor_disarmed)

    def motors_state(self) -> dict:
        """Снимок моторов для монитора эмулятора (/api/state)."""
        self._motors_step()
        count = self._motor_count()
        motors = []
        for i in range(count):
            motors.append({
                "index": i + 1,
                "output": int(round(self.motor_output[i])),
                "rpm": int(round(self.motor_rpm[i])),
                "temp_c": int(round(self.motor_temp_c[i])),
                "current_a": round(self.motor_current_a[i], 2),
                "mah": round(self.motor_mah[i], 1),
            })
        return {
            "count": count,
            "test_active": self.motor_test_active(),
            "sim_flight": self.sim_flight,
            "voltage_v": round(self.motor_voltage_v, 2),
            "dshot_telemetry": bool(self._setting("dshot_bidir", 0)),
            "esc_sensor": "ESC_SENSOR" in self.features,
            "protocol": self._setting("motor_pwm_protocol", 7),
            "motors": motors,
        }

    def _h_motor(self, w, r):
        """MSP_MOTOR (104): msp.c всегда пишет ВОСЕМЬ u16 — для выключенных моторов нули,
        для остальных текущий выход `motor[]` (когда борт не взведён — motor_disarmed[])."""
        self._motors_step()
        count = self._motor_count()
        for i in range(MAX_SUPPORTED_MOTORS):
            w.u16(int(round(self.motor_output[i])) if i < count else 0)

    def _h_servo(self, w, r):
        """MSP_SERVO (103): 8 текущих значений сервоприводов.

        Как servoMixer() в прошивке: если у сервопривода задано перенаправление
        канала, отдаём значение этого RC-канала, иначе — «середину».
        """
        channels = self._rc_channel_values()
        for index in range(MAX_SUPPORTED_SERVOS):
            params = self.servo_params[index]
            forward = params["forwardFromChannel"]
            if forward != CHANNEL_FORWARDING_DISABLED and forward < len(channels):
                w.u16(channels[forward])
            else:
                w.u16(params["middle"])

    def _rc_channel_values(self) -> list:
        """Значения RC-каналов (на столе: стики по центру, газ внизу, AUX выключены)."""
        # Порядок A/E/R/T, как в MSP_RC: значения берём у виртуальной аппаратуры, чтобы
        # тумблеры на вкладке «Приёмник» и таблица «Серво» показывали одно и то же.
        return list(self.rc_channels)

    def _h_servo_configurations(self, w, r):
        """MSP_SERVO_CONFIGURATIONS (120): 8 × 12 байт — min, max, middle, rate, forward, reversed.

        Вкладка «Сервоприводы» считает набор непустым только если длина кратна 12.
        """
        for params in self.servo_params:
            w.u16(params["min"])
            w.u16(params["max"])
            w.u16(params["middle"])
            w.u8(params["rate"])
            w.u8(params["forwardFromChannel"])
            w.u32(params["reversedSources"])

    def _h_set_servo_configuration(self, w, r):
        """MSP_SET_SERVO_CONFIGURATION (212): index + те же 12 байт (msp.c требует ровно 13)."""
        data = bytes(r.rest())
        if len(data) < 13:
            return b""
        index = data[0]
        if index >= MAX_SUPPORTED_SERVOS:
            self._log(f"[MSP] SERVO: неверный индекс {index}, запись отклонена")
            return b""
        self.servo_params[index] = {
            "min": int.from_bytes(data[1:3], "little"),
            "max": int.from_bytes(data[3:5], "little"),
            "middle": int.from_bytes(data[5:7], "little"),
            "rate": data[7],
            "forwardFromChannel": data[8],
            "reversedSources": int.from_bytes(data[9:13], "little"),
        }
        params = self.servo_params[index]
        forward = params["forwardFromChannel"]
        self._log(
            f"[MSP] SERVO {index + 1}: {params['min']}/{params['middle']}/{params['max']}, "
            f"rate {params['rate']}, перенаправление "
            f"{'выключено' if forward == CHANNEL_FORWARDING_DISABLED else forward + 1}"
        )
        return b""   # пустой ACK: вкладка «Сервоприводы» ждёт ответа на каждую запись

    def _h_servo_mix_rules(self, w, r):
        """MSP_SERVO_MIX_RULES (241): 16 правил по 7 байт."""
        for rule in self.servo_mix_rules:
            w.u8(rule["targetChannel"]).u8(rule["inputSource"]).u8(rule["rate"]).u8(rule["speed"])
            w.u8(rule["min"]).u8(rule["max"]).u8(rule["box"])

    def _h_set_servo_mix_rule(self, w, r):
        """MSP_SET_SERVO_MIX_RULE (242): index + правило из 7 байт."""
        data = bytes(r.rest())
        if len(data) < 8:
            return b""
        index = data[0]
        if index >= MAX_SERVO_RULES:
            return b""
        self.servo_mix_rules[index] = {
            "targetChannel": data[1], "inputSource": data[2], "rate": data[3], "speed": data[4],
            "min": data[5], "max": data[6], "box": data[7],
        }
        return b""

    def _h_motor_telemetry(self, w, r):
        """MSP_MOTOR_TELEMETRY (139): число моторов, затем по 13 байт на мотор —
        обороты (u32), доля ошибок (u16, 10000 = 100.00 %), температура (u8, °C),
        напряжение (u16, 0.01 В), ток (u16, 0.01 А), расход (u16, мА·ч).

        Раскладка ровно как в msp.c 4.5.4 и как её читает MSPHelper.js. Данные живые:
        слайдер на вкладке «Моторы» → растут обороты, ток и температура, а напряжение
        проседает тем сильнее, чем больше суммарный ток четырёх ESC.
        """
        self._motors_step()
        count = self._motor_count()
        w.u8(count)
        for i in range(count):
            rpm = int(round(self.motor_rpm[i]))
            if self.motor_telemetry_active():
                invalid = self.motor_invalid_percent(i)     # 0.03…0.10 %
            else:
                invalid = self.MOTOR_INVALID_PCT_NO_TELEMETRY
            w.u32(rpm)
            w.u16(invalid)
            w.u8(int(round(self.motor_temp_c[i])))
            w.u16(int(round(self.motor_voltage_v * 100)))
            w.u16(int(round(self.motor_current_a[i] * 100)))
            w.u16(int(self.motor_mah[i]) & 0xFFFF)

    # ------------------------------------------------------------ GPS (виртуальный приёмник)
    # На виртуальной плате GPS сидит на UART1 (см. serial_ports), модуль — UBLOX.
    # Координаты медленно «ходят» вокруг точки взлёта: вкладка «GPS» и OSD показывают живые
    # данные, а не замороженные константы. Курс — в 0.1°, как gpsSol.groundCourse в прошивке.
    GPS_BASE_LAT = 48.137154          # база (тот же Мюнхен, что в примерах Betaflight)
    GPS_BASE_LON = 11.575382
    GPS_ALT_BASE_M = 12
    GPS_TRACK_M = 25.0                # радиус «прогулки» вокруг базы, м
    # Спутники: (система, номер, C/N0). Системы — как в прошивке (gnssId): 0 GPS, 1 SBAS,
    # 2 Galileo, 3 BeiDou, 6 Glonass; порядок — как отдаёт настоящий модуль (по системам).
    # Список длиннее 16 спутников, и это принципиально: Configurator (GpsTab.vue) включает
    # подробную таблицу сигналов — имена систем в колонке Gnss ID, зелёные метки
    # «используется», подписи качества — ТОЛЬКО когда спутников больше 16. При 16 и меньше
    # он рисует упрощённый список: прочерки в Gnss ID, числовое качество, все метки красные.
    GPS_SATELLITES = (
        (0, 2, 44), (0, 5, 47), (0, 10, 28), (0, 12, 46), (0, 15, 42),
        (0, 18, 38), (0, 24, 35), (0, 26, 26), (0, 29, 33),
        (1, 123, 37), (1, 136, 30),
        (2, 1, 45), (2, 11, 39), (2, 15, 34), (2, 19, 41), (2, 36, 27),
        (3, 7, 31), (3, 12, 25),
        (6, 3, 43), (6, 4, 28), (6, 9, 37), (6, 10, 29), (6, 17, 32), (6, 21, 24),
    )

    def gps_solution(self) -> dict:
        """Текущее решение GPS: координаты, скорость, курс, высота, спутники, «дом»."""
        t = self._t()
        north = self.GPS_TRACK_M * math.sin(t / 37.0)          # м от базы
        east = self.GPS_TRACK_M * math.cos(t / 53.0)
        north_v = self.GPS_TRACK_M / 37.0 * math.cos(t / 37.0)  # м/с
        east_v = -self.GPS_TRACK_M / 53.0 * math.sin(t / 53.0)
        lat = self.GPS_BASE_LAT + north / 111_320.0
        lon = self.GPS_BASE_LON + east / (111_320.0 * math.cos(math.radians(self.GPS_BASE_LAT)))
        used, sats = self.gps_satellites(t)
        if self.fs_gps_north is not None:
            # Демо GPS-спасения: борт реально летит домой — координаты, высота, скорость
            # и курс берём из сценария, чтобы вкладка «GPS» показывала возврат.
            north, east, alt = self.fs_gps_north, self.fs_gps_east, self.fs_gps_alt
            previous = self.fs_gps_prev
            if previous and self.fs_ms > previous[2]:
                dt_s = (self.fs_ms - previous[2]) / 1000.0
                north_v = (north - previous[0]) / dt_s
                east_v = (east - previous[1]) / dt_s
            else:
                north_v = east_v = 0.0
            self.fs_gps_prev = (north, east, self.fs_ms)
            return {
                "fix": 2,
                "sats": used,
                "lat": self.GPS_BASE_LAT + north / 111_320.0,
                "lon": self.GPS_BASE_LON + east / (111_320.0 * math.cos(math.radians(self.GPS_BASE_LAT))),
                "alt_m": int(round(alt)),
                "speed_cms": int(round(math.hypot(north_v, east_v) * 100)),
                "course_deg10": int(round((math.degrees(math.atan2(east_v, north_v)) % 360.0) * 10)),
                "pdop": 105,
                "home_dist_m": int(round(math.hypot(north, east))),
                "home_dir_deg": int(round(math.degrees(math.atan2(east, north)) % 360.0)),
                "satellites": sats,
            }
        return {
            "fix": 2,                                   # STATE(GPS_FIX) = 1 << 1 (runtime_config.h)
            "sats": used,
            "lat": lat,
            "lon": lon,
            "alt_m": int(round(self.GPS_ALT_BASE_M + 1.5 * math.sin(t / 23.0))),
            "speed_cms": int(round(math.hypot(north_v, east_v) * 100)),
            "course_deg10": int(round((math.degrees(math.atan2(east_v, north_v)) % 360.0) * 10)),
            "pdop": 105 + int(round(15 * math.sin(t / 31.0))),   # pdop = 0.01 на единицу
            "home_dist_m": int(round(math.hypot(north, east))),
            "home_dir_deg": int(round(math.degrees(math.atan2(east, north)) % 360.0)),
            "satellites": sats,
        }

    def gps_satellites(self, t: float | None = None) -> tuple:
        """Спутники для MSP_GPSSVINFO: (число спутников в решении, [(система, svid, quality, cno)]).

        В решении — самые сильные по сигналу (как отбирает сам приёмник), остальные видны,
        но «не используются». quality: младшие 3 бита — качество (0 нет сигнала, 1 поиск,
        2/4 отслеживается, 5…7 полностью отслеживается), бит 3 — «используется в решении»
        (Configurator читает: (quality & 0x8) >> 3 и quality & 0x7).
        """
        if t is None:
            t = self._t()
        used = 9 + int(round(math.sin(t / 29.0)))     # 8…10 спутников идут в решение
        entries = []
        for i, (gnss, svid, base_cno) in enumerate(self.GPS_SATELLITES):
            cno = max(10, min(52, base_cno + int(round(2 * math.sin(t / 7.0 + i)))))
            entries.append((gnss, svid, cno))
        strongest = {index for index, _ in
                     sorted(enumerate(entries), key=lambda pair: -pair[1][2])[:used]}
        rows = []
        for index, (gnss, svid, cno) in enumerate(entries):
            if cno >= 40:
                quality_value = 7                       # полностью отслеживается
            elif cno >= 37:
                quality_value = 6
            elif cno >= 34:
                quality_value = 5
            elif cno >= 31:
                quality_value = 4                       # «отслеживается» (жёлтая метка)
            elif cno >= 28:
                quality_value = 2                       # слабый — «приобретён» (серая метка)
            else:
                quality_value = 1                       # едва виден — «поиск»
            quality = quality_value | (0x08 if index in strongest else 0x00)
            rows.append((gnss, svid, quality, cno))
        return used, rows

    def _h_raw_gps(self, w, r):
        """MSP_RAW_GPS (106) — раскладка msp.c 4.5.4: fix, спутники, координаты, высота,
        скорость, курс (0.1°) и pdop (0.01)."""
        g = self.gps_solution()
        w.u8(g["fix"]).u8(g["sats"])
        w.i32(int(round(g["lat"] * 1e7))).i32(int(round(g["lon"] * 1e7)))
        w.u16(g["alt_m"]).u16(g["speed_cms"]).u16(g["course_deg10"]).u16(g["pdop"])

    def _h_comp_gps(self, w, r):
        """MSP_COMP_GPS (107): дистанция до дома (м), направление (град) и признак обновления."""
        g = self.gps_solution()
        w.u16(g["home_dist_m"]).u16(g["home_dir_deg"])
        w.u8(1 if int(self._t() * 10) % 2 else 0)     # GPS_update & 1 — мигает, как на железе

    def _h_sensor_status(self, w, r):
        w.u8(1).u8(1).u8(1 if "MAG" in self.active_boxes else 0).u8(1)

    def _h_gps_sv_info(self, w, r):
        """MSP_GPSSVINFO (164): число спутников, затем по 4 байта на спутник —
        система (gnss), номер (svid), качество, уровень сигнала (C/N0, дБ)."""
        _used, rows = self.gps_satellites()
        w.u8(len(rows))
        for gnss, svid, quality, cno in rows:
            w.u8(gnss).u8(svid).u8(quality).u8(cno)

    # --- конфигурация (только чтение/эхо)
    def _h_mixer_config(self, w, r):
        idx = max(0, min(len(self.mixer_names) - 1, self._mixer_index()))
        w.u8(idx).u8(0)

    def _mixer_index(self):
        try:
            return self.mixer_names.index(self.mixer_name)
        except ValueError:
            return 0

    # Идентификаторы SOFTSERIAL1/2 из serialPortIdentifier_e (Configurator: portNames.js)
    SOFT_SERIAL_IDS = (30, 31)

    # На нашей виртуальной плате пины SOFTSERIAL выведены (в прошивке это
    # serialPortUsageList + softSerialPinConfig); если бы пинов не было, порты
    # не появлялись бы даже при включённой фиче.
    SOFT_SERIAL_PINS_ASSIGNED = True

    SERIAL_PORT_NAMES = {0: "UART1", 1: "UART2", 2: "UART3", 3: "UART4", 4: "UART5",
                         20: "USB VCP", 30: "SOFTSERIAL1", 31: "SOFTSERIAL2"}

    # -------------------------------------- приёмник: «в эфире» и связь с аппаратурой
    def rx_serial_port(self):
        """Порт, на котором включён Serial RX (в прошивке это проверяет serialRxInit())."""
        for port in self.serial_ports:
            if RX_SERIAL_FUNCTION in port["functions"]:
                return port
        return None

    def rx_serial_ready(self) -> bool:
        """Есть ли во вкладке «Порты» конфигурация приёмника: UART с функцией Serial RX.

        Если такого порта нет, прошивка не поднимает приёмник: `rxRuntimeState.channelCount`
        остаётся нулевым, и MSP_RC отдаёт пустой ответ — Configurator показывает полосы
        приглушёнными, без значений.
        """
        return self.rx_serial_port() is not None

    def set_rx_link(self, connected: bool):
        """Подключить/отключить виртуальную аппаратуру (USB-«пульт» к приёмнику).

        Включённая аппаратура означает, что она уже связана с приёмником; выключенная —
        это потеря связи (не «развязка»): состояние бинда при этом не меняется.
        """
        self.rx_link_connected = bool(connected)
        if self.rx_link_connected:
            self.rx_bind_state = "bound"
            self.rx_bind_deadline = 0.0
            self.rx_bind_seconds = 0.0
        return self.rx_link_connected

    RX_BIND_SECONDS = 2.0          # сколько «ищет приёмник» кнопка «Связать»

    @classmethod
    def aux_type(cls, channel) -> str:
        """Тип органа управления для AUX-канала: `switch`, `switch3` или `knob`."""
        name = str(channel).strip().lower()
        if name.isdigit():
            # номер канала 0…15, как в MSP_RC: AUX начинаются с NON_AUX_CHANNEL_COUNT
            index = int(name)
            if index < NON_AUX_CHANNEL_COUNT:
                return "stick"
            name = f"aux{index - NON_AUX_CHANNEL_COUNT + 1}"
        if name in AUX_KNOBS:
            return "knob"
        if name in AUX_THREE_POSITION:
            return "switch3"
        return "switch"

    def aux_types(self) -> dict:
        """Типы всех AUX-каналов — страница /tx рисует по ним тумблеры, трёхпозиционники и крутилки."""
        return {name: self.aux_type(name) for name in RC_AUX_NAMES}

    def set_rc_control(self, name: str) -> str:
        """Передать управление виртуальной аппаратуре (`virtual`) или физической (`physical`).

        Как в жизни: сменилась аппаратура — связь теряется, и её нужно «связать» заново
        (кнопка «Связать (Bind)» или автоматически, когда пойдут кадры от физической аппаратуры).
        """
        wanted = "physical" if str(name).strip().lower().startswith(("phys", "физ", "реал")) else "virtual"
        if wanted != self.rc_control:
            self.rc_control = wanted
            self.rx_link_connected = False
            self.rx_bind_state = "none"
            self.rx_bind_deadline = 0.0
            self.rc_source = ("физическая аппаратура — ждём кадры "
                              "(run_tx_joystick.bat / run_tx_wire.bat)"
                              if wanted == "physical" else "виртуальная аппаратура (страница /tx)")
        return self.rc_control

    RC_SOURCE_MEMORY = 10.0             # сколько секунд помним прежних писателей каналов
    RC_SOURCE_WINDOW = 5.0              # окно, в котором ищем «пишут несколько источников»

    def note_rc_source(self, source: str) -> None:
        """Запомнить, кто писал каналы: по чередованию источников видно два окна инструмента.

        Симптом: старый запуск `tools/tx.py` (с прежней раскладкой осей) продолжает работать в
        другом окне, а рядом запущен новый. Каналы «прыгают», на вкладке «Приёмник» рыск уезжает
        в AUX, а коптер крутится. Сам по себе инструмент этого не видит — а эмулятор видит.
        """
        now = self._t()
        text = str(source or "")[:96]
        self.rc_channels_updated = now
        if not text:
            return
        if not self.rc_sources or self.rc_sources[-1][1] != text:
            self.rc_sources.append((now, text))
        else:
            self.rc_sources[-1] = (now, text)
        self.rc_sources = [(t, name) for t, name in self.rc_sources
                           if now - t <= self.RC_SOURCE_MEMORY][-12:]

    def rc_source_state(self) -> dict:
        """Кто пишет каналы: последний источник, сколько секунд назад и нет ли чередования."""
        now = self._t()
        fresh = [(t, name) for t, name in self.rc_sources if now - t <= self.RC_SOURCE_WINDOW]
        names = []
        for _t, name in fresh:
            if name not in names:
                names.append(name)
        # Один переход (страница отдала управление инструменту) — это норма. «Пишут двое» — это
        # когда источники ЧЕРЕДУЮТСЯ: A, B, A, B… Тогда каналы и «прыгают».
        changes = sum(1 for (_, a), (_, b) in zip(fresh, fresh[1:]) if a != b)
        return {
            "source": fresh[-1][1] if fresh else "",
            "updated_ago": round(now - self.rc_channels_updated, 2) if self.rc_channels_updated else None,
            "sources": names,
            "source_changes": changes,
            "multi_source": len(names) > 1 and changes >= 3,
        }

    def set_rc_layout_note(self, text) -> str:
        """Заметка от инструмента аппаратуры: «раскладка осей не та» (её показывает страница /tx).

        Инструмент (`tools/tx.py`) считает это по настоящим положениям осей пульта — страница
        просто показывает текст, чтобы предупреждение было видно и без консоли.
        """
        self.rc_layout_note = str(text or "")[:300]
        return self.rc_layout_note

    def set_rc_aux_map(self, items) -> list:
        """Раскладка AUX от инструмента: какие оси ведут тумблеры и крутилки (показывает /tx).

        Инструмент (`tools/tx.py`) находит их калибровкой `--detect-aux` и присылает готовые
        строки вида «AUX1 ← тумблер (2 положения) — ось 3» — страница просто их показывает.
        """
        if isinstance(items, (str, bytes)):
            items = [items]
        self.rc_aux_map = [str(item)[:80] for item in (items or []) if str(item).strip()][:16]
        return self.rc_aux_map

    def set_rc_map_info(self, info) -> dict:
        """Запомнить раскладку осей физической аппаратуры (для страницы /tx)."""
        if not isinstance(info, dict):
            return self.rc_map_info
        clean = {}
        for name in RC_STICK_NAMES:
            value = info.get(name)
            if value is None:
                continue
            try:
                clean[name] = int(value)
            except (TypeError, ValueError):
                continue
        invert = [str(x) for x in (info.get("invert") or []) if str(x) in RC_STICK_NAMES]
        if clean:
            clean["invert"] = invert
            if info.get("device"):
                clean["device"] = str(info["device"])[:40]
            self.rc_map_info = clean
        return self.rc_map_info

    def control_state(self) -> dict:
        """У кого управление и что это значит для страницы /tx."""
        physical = self.rc_control == "physical"
        return {
            "mode": self.rc_control,
            "physical": physical,
            "virtual": not physical,
            "text": ("управление у физической аппаратуры (пульт по USB или приёмник в COM-порту)"
                     if physical else "управление у виртуальной аппаратуры (эта страница)"),
        }

    def rx_provider_name(self) -> str:
        """Имя протокола приёмника из `serialrx_provider` (CRSF, SBUS, SPEKTRUM2048…)."""
        entry = self.settings_by_name.get("serialrx_provider") or {}
        values = entry.get("values") or []
        index = int(self.settings.get("serialrx_provider", 0))
        if 0 <= index < len(values):
            return str(values[index])
        return str(index)

    def start_bind(self, seconds=None, connected: bool = True):
        """«Связать»: аппаратура ищет приёмник и через несколько секунд связывается.

        Это эмуляция того, что делает кнопка Bind на настоящем приёмнике: пока идёт поиск,
        каналы в эфир не идут (как и при живом бинде — полосы стоят), через `seconds`
        (по умолчанию 2 с) состояние становится «связано» и каналы возвращаются.
        """
        self.rx_bind_seconds = max(0.0, float(self.RX_BIND_SECONDS if seconds is None else seconds))
        self.rx_bind_deadline = time.time() + self.rx_bind_seconds
        if self.rx_bind_seconds <= 0:
            self.rx_bind_state = "bound"
            self.rx_link_connected = bool(connected)
        else:
            # пока аппаратура ищет приёмник, кадров в эфире нет — как при настоящем бинде
            self.rx_bind_state = "binding"
            self.rx_link_connected = False
        return self.bind_state()

    def _resolve_bind(self):
        """Завершить биндинг по времени: связь установлена, кадры пошли."""
        if self.rx_bind_state == "binding" and time.time() >= self.rx_bind_deadline:
            self.rx_bind_state = "bound"
            self.rx_link_connected = True

    def unbind(self):
        """«Развязать»: аппаратура больше не связана с приёмником — кадров нет, «Этап 1»."""
        self.rx_bind_state = "none"
        self.rx_bind_deadline = 0.0
        self.rx_link_connected = False
        return self.bind_state()

    def bind_state(self) -> dict:
        """Состояние бинда для страницы /tx, `tools/tx.py` и вкладки «Приёмник»."""
        self._resolve_bind()
        left = max(0.0, self.rx_bind_deadline - time.time()) if self.rx_bind_state == "binding" else 0.0
        port = self.rx_serial_port()
        if port is None:
            text = "приёмник не поднят: во вкладке «Порты» нет UART с Serial RX"
        elif self.rx_bind_state == "binding":
            text = f"идёт связывание ({self.rx_provider_name()})… осталось {left:.1f} с"
        elif self.rx_bind_state == "bound":
            text = f"связана ({self.rx_provider_name()} на {self.SERIAL_PORT_NAMES.get(port['identifier'], port['identifier'])})"
        elif self.rc_control == "physical":
            text = ("физическая аппаратура не отдаёт кадры — запустите run_tx_joystick.bat (пульт) "
                    "или run_tx_wire.bat (приёмник), затем нажмите «Связать»")
        else:
            text = "НЕ связана: нажмите «Связать» — каналы заработают"
        return {
            "state": self.rx_bind_state,
            "text": text,
            "protocol": self.rx_provider_name(),
            "seconds_left": round(left, 1),
            "seconds": round(self.rx_bind_seconds, 2),
            "ready": port is not None and self.rx_bind_state == "bound",
        }

    def _rx_link_hold_values(self, channels: list) -> list:
        """Значения «Этапа 1» из rx.c: AUTO — центр (газ — rx_min_usec), HOLD — последнее, SET — заданное."""
        values = []
        for index, channel in enumerate(self._rxfail_channels()):
            mode = channel["mode"]
            if mode == RX_FAILSAFE_MODE_HOLD:
                values.append(channels[index])
            elif mode == RX_FAILSAFE_MODE_SET:
                values.append(channel["value"])
            elif index == 3:                     # THROTTLE: AUTO → rx_min_usec
                values.append(RX_MIN_USEC)
            else:                                # ROLL/PITCH/YAW и AUX: AUTO → midrc
                values.append(RX_MID_USEC)
        return values

    def rx_link_state(self) -> dict:
        """Состояние приёмника для вкладки «Приёмник» и страницы аппаратуры."""
        self._resolve_bind()
        port = self.rx_serial_port()
        return {
            "connected": self.rx_link_connected,
            "serial_ok": port is not None,
            "serial_port": self.SERIAL_PORT_NAMES.get(port["identifier"], str(port["identifier"])) if port else None,
            "serial_functions": list(port["functions"]) if port else [],
            "protocol": (self.settings_by_name.get("serialrx_provider") or {}).get("values") or [],
            "provider": int(self.settings.get("serialrx_provider", 0)),
            "failsafe": (not self.rx_link_connected) and port is not None,
            "bind": self.bind_state(),
            "control": self.control_state(),
            "stage1": [{"mode": c["mode"], "value": c["value"]} for c in self._rxfail_channels()],
        }

    def sync_soft_serial_ports(self):
        """Фича SOFTSERIAL включает порты SOFTSERIAL1/2 — как на настоящем FC.

        Прошивка собрана с USE_SOFTSERIAL, поэтому в списке портов они появляются
        тогда, когда фича включена: во вкладке «Порты» видны строки SOFTSERIAL1/2.
        """
        has_soft = {port["identifier"] for port in self.serial_ports} & set(self.SOFT_SERIAL_IDS)
        wanted = "SOFTSERIAL" in self.features and self.SOFT_SERIAL_PINS_ASSIGNED
        if wanted and not has_soft:
            for ident in self.SOFT_SERIAL_IDS:
                self.serial_ports.append({
                    "identifier": ident, "functions": [],
                    "msp_baudrate": 5, "gps_baudrate": 4,
                    "telemetry_baudrate": 0, "blackbox_baudrate": 5,
                })
            self._log("[FC] фича SOFTSERIAL: добавлены порты SOFTSERIAL1/2")
        elif not wanted and has_soft:
            self.serial_ports = [
                port for port in self.serial_ports if port["identifier"] not in self.SOFT_SERIAL_IDS
            ]
            self._log("[FC] фича SOFTSERIAL выключена: порты SOFTSERIAL1/2 убраны")

    def _h_set_feature_config(self, w, r):
        """MSP_SET_FEATURE_CONFIG: u32 маска фич — вкладка «Конфигурация» и порты."""
        mask = r.u32()
        self.features = {name for name, bit in self.feature_bits_by_name.items() if mask & bit}
        self.sync_soft_serial_ports()
        self._log(f"[MSP] фичи сохранены: {', '.join(sorted(self.features)) or '(нет)'}")

    def _h_set_name(self, w, r):
        """MSP_SET_NAME (устаревший, но вкладки ещё пишут через него): символы имени дрона."""
        name = bytes(r.rest()).decode("latin-1", "replace").strip("\x00")[:MAX_NAME_LENGTH]
        self._mirror_name_setting("craft_name", name)
        self.settings["craft_name"] = name
        self._log(f"[MSP] имя дрона: {name!r}")

    def _h_set_rx_map(self, w, r):
        """MSP_SET_RX_MAP (65): RX_MAPPABLE_CHANNEL_COUNT букв карты каналов."""
        names = {0: "A", 1: "E", 2: "R", 3: "T", 4: "1", 5: "2", 6: "3", 7: "4"}
        data = bytes(r.rest())[:RC_MAPPABLE_CHANNEL_COUNT]
        self.rx_map = "".join(names.get(byte, "1") for byte in data)
        self._log(f"[MSP] карта каналов: {self.rx_map}")
        return b""

    def _h_feature_config(self, w, r):
        w.u32(self.feature_mask)

    def _h_beeper_config(self, w, r):
        """MSP_BEEPER_CONFIG (184): флаги выключенных сигналов и флаги DShot-пищалки.

        Второе поле — dshotBeaconOffFlags: их правит команда CLI `beacon`, поэтому вкладка
        «Конфигурация» показывает то же состояние, что и консоль (beeperConfig в прошивке).
        """
        mask = 0
        for i, name in enumerate(self.beeper_names):
            if name in self.beepers_disabled:
                mask |= 1 << i
        beacon = 0
        for i, name in enumerate(self.beeper_names):
            if name in self.dshot_beacon_disabled:
                beacon |= 1 << i
        w.u32(mask).u8(self.dshot_beacon_tone).u32(beacon)

    def _h_set_beeper_config(self, w, r):
        """MSP_SET_BEEPER_CONFIG (185): та же раскладка, что у GET (msp.c 4.5.4):
        u32 off-флаги, u8 тон DShot, u32 off-флаги DShot-пищалки. Пишем в те же поля,
        что трогают CLI `beeper`/`beacon`, — вкладка «Конфигурация» и консоль не расходятся.
        """
        if len(r.data) - r.pos >= 4:
            mask = r.u32()
            self.beepers_disabled = {n for i, n in enumerate(self.beeper_names) if mask & (1 << i)}
        if len(r.data) - r.pos >= 1:
            self.dshot_beacon_tone = r.u8()
        if len(r.data) - r.pos >= 4:
            beacon = r.u32()
            self.dshot_beacon_disabled = {n for i, n in enumerate(self.beeper_names) if beacon & (1 << i)}

    def _h_board_alignment(self, w, r):
        w.i16(self.settings.get("board_align_roll", 0))
        w.i16(self.settings.get("board_align_pitch", 0))
        w.i16(self.settings.get("board_align_yaw", 0))

    def _h_rc_tuning(self, w, r):
        """MSP_RC_TUNING (111): 23 байта — раскладка sbufWrite* в msp.c 4.5.4.

        Значения — из настроек профиля рейтов (`roll_rc_rate`, `roll_expo`, `roll_srate`, …),
        то есть ровно те, что видны в CLI через 'get roll_rc_rate'.
        """
        s = lambda n, d=0: int(self.settings.get(n, d))
        w.u8(s("roll_rc_rate", 7)).u8(s("roll_expo", 0))
        w.u8(s("roll_srate", 67)).u8(s("pitch_srate", 67)).u8(s("yaw_srate", 67))
        w.u8(0)  # был tpa_rate (переехал в PID-профиль)
        w.u8(s("thr_mid", 50)).u8(s("thr_expo", 0))
        w.u16(0)  # был tpa_breakpoint (переехал в PID-профиль)
        w.u8(s("yaw_expo", 0)).u8(s("yaw_rc_rate", 7)).u8(s("pitch_rc_rate", 7)).u8(s("pitch_expo", 0))
        w.u8(s("throttle_limit_type", 0)).u8(s("throttle_limit_percent", 100))
        w.u16(s("roll_rate_limit", 1998)).u16(s("pitch_rate_limit", 1998)).u16(s("yaw_rate_limit", 1998))
        w.u8(s("rates_type", RATES_TYPE_ACTUAL))

    def _h_set_rc_tuning(self, w, r):
        """MSP_SET_RC_TUNING (204): вкладка «Rates» пишет те же 23 байта."""
        d = bytes(r.rest())
        if len(d) < 23:
            return b""
        # Смещения — как в MSP_RC_TUNING из msp.c 4.5.4 (8-9 и 16-21 — двухбайтовые поля).
        self.settings["roll_rc_rate"], self.settings["roll_expo"] = d[0], d[1]
        self.settings["roll_srate"], self.settings["pitch_srate"], self.settings["yaw_srate"] = d[2], d[3], d[4]
        self.settings["thr_mid"], self.settings["thr_expo"] = d[6], d[7]
        self.settings["yaw_expo"] = d[10]
        self.settings["yaw_rc_rate"], self.settings["pitch_rc_rate"], self.settings["pitch_expo"] = d[11], d[12], d[13]
        self.settings["throttle_limit_type"], self.settings["throttle_limit_percent"] = d[14], d[15]
        for index, name in enumerate(("roll_rate_limit", "pitch_rate_limit", "yaw_rate_limit")):
            self.settings[name] = int.from_bytes(d[16 + index * 2:18 + index * 2], "little")
        self.settings["rates_type"] = d[22]
        self._save_active_rate_profile()
        self._log(f"[MSP] RC_TUNING сохранён: rates_type={self.settings['rates_type']}, "
                  f"roll_rc_rate={self.settings['roll_rc_rate']}")
        return b""

    def _h_pid_advanced(self, w, r):
        """MSP_PID_ADVANCED (94): 61 байт — ровно как в msp.c 4.5.4 (порядок полей фиксирован).

        Configurator читает их в том же порядке (MSPHelper.js), поэтому раскладку нельзя менять:
        TPA, «Ослабление I», антигравитация, вращение I, Feedforward, D Max, «Другие настройки».
        """
        s = lambda n, d=0: int(self.settings.get(n, PID_PROFILE_DEFAULTS.get(n, d)))
        w.u16(0).u16(0).u16(0)                        # устаревшие rollPitch/yaw iterm ignore, yaw_p_limit
        w.u8(0).u8(0)                                 # зарезервировано, был vbatPidCompensation
        w.u8(s("feedforward_transition"))
        w.u8(0).u8(0).u8(0).u8(0)                     # зарезервировано (был dtermSetpointWeight)
        w.u16(s("acc_limit")).u16(s("acc_limit_yaw"))
        w.u8(s("angle_limit")).u8(0)                  # «Макс. угол наклона» (была levelSensitivity)
        w.u16(0)                                      # был itermThrottleThreshold
        w.u16(s("anti_gravity_gain"))
        w.u16(0)                                      # был dtermSetpointWeight
        w.u8(s("iterm_rotation")).u8(0)               # был smart_feedforward
        w.u8(s("iterm_relax")).u8(s("iterm_relax_type"))
        w.u8(s("abs_control_gain")).u8(s("throttle_boost")).u8(s("acro_trainer_angle_limit"))
        w.u16(s("f_roll")).u16(s("f_pitch")).u16(s("f_yaw"))   # Feedforward по осям
        w.u8(0)                                       # был antiGravityMode
        w.u8(s("d_min_roll")).u8(s("d_min_pitch")).u8(s("d_min_yaw"))
        w.u8(s("d_max_gain")).u8(s("d_max_advance"))
        w.u8(s("use_integrated_yaw")).u8(s("integrated_yaw_relax"))
        w.u8(s("iterm_relax_cutoff"))
        w.u8(s("motor_output_limit")).u8(s("auto_profile_cell_count") & 0xFF)
        w.u8(s("dyn_idle_min_rpm"))
        w.u8(s("feedforward_averaging")).u8(s("feedforward_smooth_factor")).u8(s("feedforward_boost"))
        w.u8(s("feedforward_max_rate_limit")).u8(s("feedforward_jitter_factor"))
        w.u8(s("vbat_sag_compensation")).u8(s("thrust_linear"))
        w.u8(s("tpa_mode")).u8(s("tpa_rate")).u16(s("tpa_breakpoint"))

    def _h_set_pid_advanced(self, w, r):
        """MSP_SET_PID_ADVANCED (95) — разбор тех же 61 байта (msp.c читает поля по порядку)."""
        d = bytes(r.rest())
        if len(d) < 61:
            return b""
        def u16(off):
            return int.from_bytes(d[off:off + 2], "little")
        # Смещения — ровно те, что собирает crunch(MSP_SET_PID_ADVANCED) в MSPHelper.js:
        # 0-5 три устаревших u16, 6 deltaMethod, 7 vbatPidCompensation, 8 feedforward_transition,
        # 9 старый dtermSetpointWeight (байт), 10-12 прочие устаревшие байты, далее — как ниже.
        self.settings["feedforward_transition"] = d[8]
        self.settings["acc_limit"], self.settings["acc_limit_yaw"] = u16(13), u16(15)
        self.settings["angle_limit"] = d[17]
        self.settings["anti_gravity_gain"] = u16(21)
        self.settings["iterm_rotation"] = d[25]
        self.settings["iterm_relax"] = d[27]
        self.settings["iterm_relax_type"] = d[28]
        self.settings["abs_control_gain"] = d[29]
        self.settings["throttle_boost"] = d[30]
        self.settings["acro_trainer_angle_limit"] = d[31]
        self.settings["f_roll"], self.settings["f_pitch"], self.settings["f_yaw"] = u16(32), u16(34), u16(36)
        self.settings["d_min_roll"], self.settings["d_min_pitch"], self.settings["d_min_yaw"] = d[39], d[40], d[41]
        self.settings["d_max_gain"], self.settings["d_max_advance"] = d[42], d[43]
        self.settings["use_integrated_yaw"], self.settings["integrated_yaw_relax"] = d[44], d[45]
        self.settings["iterm_relax_cutoff"] = d[46]
        self.settings["motor_output_limit"] = d[47]
        self.settings["auto_profile_cell_count"] = d[48] - 256 if d[48] > 127 else d[48]
        self.settings["dyn_idle_min_rpm"] = d[49]
        self.settings["feedforward_averaging"] = d[50]
        self.settings["feedforward_smooth_factor"] = d[51]
        self.settings["feedforward_boost"] = d[52]
        self.settings["feedforward_max_rate_limit"] = d[53]
        self.settings["feedforward_jitter_factor"] = d[54]
        self.settings["vbat_sag_compensation"] = d[55]
        self.settings["thrust_linear"] = d[56]
        self.settings["tpa_mode"] = d[57]
        self.settings["tpa_rate"] = d[58]
        self.settings["tpa_breakpoint"] = u16(59)
        self._save_active_pid_profile()
        self._log(
            f"[MSP] PID_ADVANCED сохранён: TPA {self.settings['tpa_mode']}/{self.settings['tpa_rate']}% "
            f"@ {self.settings['tpa_breakpoint']}, угол {self.settings['angle_limit']}, "
            f"feedforward boost {self.settings['feedforward_boost']}"
        )
        return b""

    def _h_pid(self, w, r):
        """MSP_PID (112): PID_ITEM_COUNT × 3 байта — P, I, D по осям ROLL/PITCH/YAW/LEVEL/MAG.

        Значения берём из тех же настроек, что видит CLI (`p_roll`, `i_roll`, `d_roll`, …):
        MspHelper.js раскладывает ответ по строкам таблицы «PID-настройки».
        """
        for axis in ("roll", "pitch", "yaw", "level", "mag"):
            w.u8(self._pid_value(f"p_{axis}")).u8(self._pid_value(f"i_{axis}")).u8(self._pid_value(f"d_{axis}"))

    def _h_set_pid(self, w, r):
        """MSP_SET_PID (202): 15 байт P/I/D по осям — та же раскладка, что у MSP_PID."""
        data = bytes(r.rest())
        if len(data) < PID_ITEM_COUNT * 3:
            return b""
        axes = ("roll", "pitch", "yaw", "level", "mag")
        for index, axis in enumerate(axes):
            base = index * 3
            for prefix, offset in (("p", 0), ("i", 1), ("d", 2)):
                name = f"{prefix}_{axis}"
                key = PID_PROFILE_ALIASES.get(name, name)
                self.settings[key] = data[base + offset]
        self._save_active_pid_profile()
        self._log(
            "[MSP] PID сохранены: roll {}/{}/{}, pitch {}/{}/{}, yaw {}/{}/{}".format(
                *[self.settings[f"{p}_roll"] for p in "pid"],
                *[self.settings[f"{p}_pitch"] for p in "pid"],
                *[self.settings[f"{p}_yaw"] for p in "pid"],
            )
        )
        return b""   # пустой ACK: вкладка ждёт ответ на каждую запись

    def _h_pidnames(self, w, r):
        """MSP_PIDNAMES (117): имена PID-осей строкой, как pidNames[] в flight/pid.c."""
        w.raw(PID_NAMES.encode())

    def _pid_value(self, name):
        """Значение PID-параметра.

        ROLL/PITCH/YAW — обычные настройки профиля (`p_roll` и т.д.). Строка LEVEL — те же поля,
        что и `angle_p_gain`/`horizon_level_strength`/`horizon_limit_sticks`. MAG в 4.5.4 не имеет
        настроек CLI, поэтому берём значение профиля (40/0/0 по умолчанию).
        """
        alias = PID_PROFILE_ALIASES.get(name)
        if alias and alias in self.settings:
            return int(self.settings[alias])
        default = PID_PROFILE_DEFAULTS.get(name, 0)
        if name in self.settings:
            return int(self.settings[name])
        return int(self.pid_profiles[self.current_profile].get(name, default))

    def _save_active_pid_profile(self):
        """Переносит значения активного профиля из настроек в хранилище профилей."""
        profile = self.pid_profiles[self.current_profile]
        for name in PID_PROFILE_DEFAULTS:
            key = PID_PROFILE_ALIASES.get(name, name)
            if key in self.settings:
                profile[name] = int(self.settings[key])

    def _h_arming_config(self, w, r):
        """MSP_ARMING_CONFIG: auto_disarm_delay, (бывший kill switch), small_angle — 3 байта."""
        w.u8(int(self.settings.get("auto_disarm_delay", 5))).u8(0)
        w.u8(int(self.settings.get("small_angle", 25)))

    def _h_set_arming_config(self, w, r):
        """MSP_SET_ARMING_CONFIG: те же значения, что настройки auto_disarm_delay/small_angle."""
        self.settings["auto_disarm_delay"] = r.u8()
        r.u8()  # устаревший kill switch
        self.settings["small_angle"] = r.u8()
        self._log(
            f"[MSP] arming: auto_disarm_delay={self.settings['auto_disarm_delay']}, "
            f"small_angle={self.settings['small_angle']}"
        )

    def _h_loop_time(self, w, r):
        w.u16(125)

    def _h_misc(self, w, r):
        w.u16(int(self.settings.get("midrc", 1500)))
        w.u16(int(self.settings.get("min_throttle", 1070)))
        w.u16(int(self.settings.get("max_throttle", 2000)))
        w.u16(int(self.settings.get("min_command", 1000)))
        w.u16(int(self.settings.get("failsafe_throttle", 1000)))
        w.u8(int(self.settings.get("gps_provider", 1)))
        w.u8(int(self.settings.get("gps_baudrate", 0)))
        w.u8(int(self.settings.get("gps_ublox_use_galileo", 0)))
        w.u8(0).u8(int(self.settings.get("rssi_channel", 0))).u8(0)
        w.u16(0)
        w.u8(int(self.settings.get("vbat_scale", 110)))
        w.u8(self._battery_setting("vbat_min_cell_voltage") // 10)
        w.u8(self._battery_setting("vbat_max_cell_voltage") // 10)
        w.u8(self._battery_setting("vbat_warning_cell_voltage") // 10)

    def _h_motor_config(self, w, r):
        """MSP_MOTOR_CONFIG (131): 10 байт — как pgResetFn_motorConfig + getMotorCount().

        Значения берутся из CLI-настроек min_throttle/max_throttle/min_command, поэтому
        вкладка «PID» («Параметры газа и моторов»), вкладка «Моторы» и CLI не расходятся.
        """
        s = lambda n, d=0: int(self.settings.get(n, d))
        w.u16(s("min_throttle", 1070)).u16(s("max_throttle", 2000)).u16(s("min_command", 1000))
        w.u8(int(s("motor_count", 4)))
        w.u8(s("motor_poles", 14))
        # useDshotTelemetry = DEFAULT_DSHOT_TELEMETRY: у 4.5.4 по умолчанию выключено
        # (настройка dshot_bidir), поэтому «Динамический Idle» во вкладке выключен — как на железе.
        w.u8(1 if s("dshot_bidir", 0) else 0)
        w.u8(1 if "ESC_SENSOR" in self.features else 0)   # esc_sensor = featureIsEnabled(FEATURE_ESC_SENSOR)

    def _h_set_motor_config(self, w, r):
        """MSP_SET_MOTOR_CONFIG (222): 10 байт — раскладка как у ответа 131."""
        d = bytes(r.rest())
        if len(d) < 10:
            return b""
        u16 = lambda o: int.from_bytes(d[o:o + 2], "little")
        self.settings["min_throttle"], self.settings["max_throttle"] = u16(0), u16(2)
        self.settings["min_command"] = u16(4)
        self.settings["motor_poles"] = d[7]
        self.settings["dshot_bidir"] = 1 if d[8] else 0
        self._log(f"[MSP] MOTOR_CONFIG сохранён: min_throttle={self.settings['min_throttle']}, "
                  f"max_throttle={self.settings['max_throttle']}, min_command={self.settings['min_command']}")
        return b""

    # ------------------------------------------------- слайдеры (упрощённая настройка)
    def _slider(self, name):
        return int(self.settings.get(name, PID_PROFILE_DEFAULTS.get(name, 0)))

    def _simplified_pids_bytes(self):
        """17 байт «Ползунков настройки PID»: 9 значений + 8 зарезервированных (writeSimplifiedPids)."""
        keep = lambda v: max(0, min(255, int(v)))
        return bytes([
            keep(self._slider("simplified_pids_mode")),
            keep(self._slider("simplified_master_multiplier")),
            keep(self._slider("simplified_pitch_d_gain")),      # simplified_roll_pitch_ratio
            keep(self._slider("simplified_i_gain")),
            keep(self._slider("simplified_d_gain")),
            keep(self._slider("simplified_pi_gain")),
            keep(self._slider("simplified_dmax_gain")),         # simplified_dmin_ratio
            keep(self._slider("simplified_feedforward_gain")),
            keep(self._slider("simplified_pitch_pi_gain")),
        ] + [0] * 8)                                     # 8 байт зарезервировано, как в msp.c

    def _simplified_dterm_bytes(self):
        """18 байт упрощённого D-term фильтра (writeSimplifiedDtermFilters)."""
        s = lambda n, d=0: int(self.settings.get(n, d))
        return (bytes([1 if s("simplified_dterm_filter", 1) else 0,
                       max(0, min(255, s("simplified_dterm_filter_multiplier", 100)))])
                + s("dterm_lpf1_static_hz", 75).to_bytes(2, "little")
                + s("dterm_lpf2_static_hz", 150).to_bytes(2, "little")
                + s("dterm_lpf1_dyn_min_hz", 75).to_bytes(2, "little")
                + s("dterm_lpf1_dyn_max_hz", 150).to_bytes(2, "little")
                + bytes(8))                                   # зарезервировано

    def _simplified_gyro_bytes(self):
        """18 байт упрощённого фильтра гироскопа (writeSimplifiedGyroFilters)."""
        s = lambda n, d=0: int(self.settings.get(n, d))
        return (bytes([1 if s("simplified_gyro_filter", 1) else 0,
                       max(0, min(255, s("simplified_gyro_filter_multiplier", 100)))])
                + s("gyro_lpf1_static_hz", 250).to_bytes(2, "little")
                + s("gyro_lpf2_static_hz", 500).to_bytes(2, "little")
                + s("gyro_lpf1_dyn_min_hz", 250).to_bytes(2, "little")
                + s("gyro_lpf1_dyn_max_hz", 500).to_bytes(2, "little")
                + bytes(8))                                   # зарезервировано

    def _h_simplified_tuning(self, w, r):
        """MSP_SIMPLIFIED_TUNING (140): ползунки PID + фильтры D-term и гироскопа (53 байта)."""
        w.raw(self._simplified_pids_bytes() + self._simplified_dterm_bytes() + self._simplified_gyro_bytes())

    def _h_set_simplified_tuning(self, w, r):
        """MSP_SET_SIMPLIFIED_TUNING (141): сохранить ползунки и применить их (applySimplifiedTuning)."""
        d = bytes(r.rest())
        # Configurator шлёт 35 байт (17 ползунков PID + 18 D-term); часть фильтра гироскопа
        # (ещё 18 байт) добавляется только при сохранении из вкладки «Фильтры».
        if len(d) < 35:
            return b""
        names = ["simplified_pids_mode", "simplified_master_multiplier", "simplified_pitch_d_gain",
                 "simplified_i_gain", "simplified_d_gain", "simplified_pi_gain", "simplified_dmax_gain",
                 "simplified_feedforward_gain", "simplified_pitch_pi_gain"]
        for index, name in enumerate(names):
            if name in self.settings:
                self.settings[name] = d[index]
        self.settings["simplified_dterm_filter"] = 1 if d[17] else 0
        self.settings["simplified_dterm_filter_multiplier"] = d[18]
        if len(d) >= 53:                     # полный набор: PID (17) + D-term (18) + гироскоп (18)
            self.settings["simplified_gyro_filter"] = 1 if d[35] else 0
            self.settings["simplified_gyro_filter_multiplier"] = d[36]
        self._apply_simplified_tuning()
        self._save_active_pid_profile()
        self._log("[MSP] SIMPLIFIED_TUNING сохранены: режим "
                  f"{self.settings.get('simplified_pids_mode')}, "
                  f"master {self.settings.get('simplified_master_multiplier')}")
        return b""

    def _h_calculate_simplified_pid(self, w, r):
        """MSP_CALCULATE_SIMPLIFIED_PID (142): посчитать PID по ползункам во временном профиле."""
        d = bytes(r.rest())
        if len(d) >= 9:
            names = ["simplified_pids_mode", "simplified_master_multiplier", "simplified_pitch_d_gain",
                     "simplified_i_gain", "simplified_d_gain", "simplified_pi_gain", "simplified_dmax_gain",
                     "simplified_feedforward_gain", "simplified_pitch_pi_gain"]
            saved = {name: self.settings.get(name) for name in names}
            for index, name in enumerate(names):
                self.settings[name] = d[index]
            result = self._calculate_simplified_pids()
            for name, value in saved.items():
                self.settings[name] = value
        else:
            result = self._calculate_simplified_pids()
        for axis in ("roll", "pitch", "yaw"):
            w.u8(result[f"p_{axis}"]).u8(result[f"i_{axis}"]).u8(result[f"d_{axis}"])
            w.u8(result[f"d_min_{axis}"])
            w.u16(result[f"f_{axis}"])

    def _h_calculate_simplified_gyro(self, w, r):
        """MSP_CALCULATE_SIMPLIFIED_GYRO (143): пересчитать фильтры гироскопа по мультипликатору."""
        d = bytes(r.rest())
        saved = {n: self.settings.get(n) for n in ("simplified_gyro_filter", "simplified_gyro_filter_multiplier")}
        if len(d) >= 18:
            self.settings["simplified_gyro_filter"] = 1 if d[0] else 0
            self.settings["simplified_gyro_filter_multiplier"] = d[1]
        self._calculate_simplified_gyro_filters()
        w.raw(self._simplified_gyro_bytes() if len(d) >= 18 else self._simplified_gyro_bytes())
        for name, value in saved.items():
            self.settings[name] = value

    def _h_calculate_simplified_dterm(self, w, r):
        """MSP_CALCULATE_SIMPLIFIED_DTERM (144): пересчитать D-term фильтры по мультипликатору."""
        d = bytes(r.rest())
        saved = {n: self.settings.get(n) for n in ("simplified_dterm_filter", "simplified_dterm_filter_multiplier")}
        if len(d) >= 18:
            self.settings["simplified_dterm_filter"] = 1 if d[0] else 0
            self.settings["simplified_dterm_filter_multiplier"] = d[1]
        self._calculate_simplified_dterm_filters()
        w.raw(self._simplified_dterm_bytes())
        for name, value in saved.items():
            self.settings[name] = value

    def _h_validate_simplified_tuning(self, w, r):
        """MSP_VALIDATE_SIMPLIFIED_TUNING (145): 3 байта — совпадают ли PID/фильтры с ползунками.

        Configurator по этим флагам выключает ползунки, если значения разошлись с профилем.
        """
        pids = self._calculate_simplified_pids()
        pids_ok = all(int(self.settings.get(f"{p}_{axis}", 0)) == pids[f"{p}_{axis}"]
                      for axis in ("roll", "pitch", "yaw") for p in ("p", "i", "d"))
        pids_ok = pids_ok and all(int(self.settings.get(f"d_min_{axis}", 0)) == pids[f"d_min_{axis}"]
                                  for axis in ("roll", "pitch", "yaw"))
        pids_ok = pids_ok and all(int(self.settings.get(f"f_{axis}", 0)) == pids[f"f_{axis}"]
                                  for axis in ("roll", "pitch", "yaw"))
        gyro = self._calculate_simplified_gyro_filters(apply=False)
        gyro_ok = all(int(self.settings.get(name, 0)) == value for name, value in gyro.items())
        dterm = self._calculate_simplified_dterm_filters(apply=False)
        dterm_ok = all(int(self.settings.get(name, 0)) == value for name, value in dterm.items())
        w.u8(1 if pids_ok else 0).u8(1 if gyro_ok else 0).u8(1 if dterm_ok else 0)

    # --- арифметика ползунков: порт config/simplified_tuning.c прошивки 4.5.4
    _PID_DEFAULTS_FOR_SLIDERS = {
        "roll": (45, 80, 40, 120), "pitch": (47, 84, 46, 125), "yaw": (45, 80, 0, 120),
    }
    _D_MIN_DEFAULTS_FOR_SLIDERS = {"roll": 30, "pitch": 34, "yaw": 0}

    def _calculate_simplified_pids(self) -> dict:
        """calculateNewPidValues(): PID = заводские × множители ползунков (без записи в настройки)."""
        mode = self._slider("simplified_pids_mode")
        master = self._slider("simplified_master_multiplier") / 100.0
        pi = self._slider("simplified_pi_gain") / 100.0
        d_gain = self._slider("simplified_d_gain") / 100.0
        ff = self._slider("simplified_feedforward_gain") / 100.0
        i_gain = self._slider("simplified_i_gain") / 100.0
        ratio = self._slider("simplified_pitch_d_gain") / 100.0
        pitch_pi = self._slider("simplified_pitch_pi_gain") / 100.0
        dmin_ratio = self._slider("simplified_dmax_gain") / 100.0
        clamp = lambda v, hi: max(0, min(hi, int(v)))
        result = {}
        for index, axis in enumerate(("roll", "pitch", "yaw")):
            if index > mode:                          # mode 0 = выключено, 1 = RP, 2 = RPY
                result[f"p_{axis}"] = int(self.settings.get(f"p_{axis}", 0))
                result[f"i_{axis}"] = int(self.settings.get(f"i_{axis}", 0))
                result[f"d_{axis}"] = int(self.settings.get(f"d_{axis}", 0))
                result[f"d_min_{axis}"] = int(self.settings.get(f"d_min_{axis}", 0))
                result[f"f_{axis}"] = int(self.settings.get(f"f_{axis}", 0))
                continue
            pitch_d = ratio if axis == "pitch" else 1.0
            pitch_pi_factor = pitch_pi if axis == "pitch" else 1.0
            p_default, i_default, d_default, f_default = self._PID_DEFAULTS_FOR_SLIDERS[axis]
            d_min_default = self._D_MIN_DEFAULTS_FOR_SLIDERS[axis]
            result[f"p_{axis}"] = clamp(p_default * master * pi * pitch_pi_factor, PID_GAIN_MAX)
            result[f"i_{axis}"] = clamp(i_default * master * pi * i_gain * pitch_pi_factor, PID_GAIN_MAX)
            if d_min_default > 0:
                dmin_k = 1.0 + ((d_default - d_min_default) / d_min_default) * dmin_ratio
            else:
                dmin_k = 1.0
            result[f"d_{axis}"] = clamp(d_min_default * master * d_gain * pitch_d * dmin_k, PID_GAIN_MAX)
            result[f"d_min_{axis}"] = clamp(d_min_default * master * d_gain * pitch_d, PID_GAIN_MAX)
            result[f"f_{axis}"] = clamp(f_default * master * pitch_pi_factor * ff, F_GAIN_MAX)
        return result

    def _calculate_simplified_dterm_filters(self, apply=True) -> dict:
        """calculateNewDTermFilterValues(): фильтры D-term от мультипликатора ползунка."""
        multiplier = self._slider("simplified_dterm_filter_multiplier") / 100.0
        out = {}
        if int(self.settings.get("simplified_dterm_filter", 1)) == 0:
            return out
        if int(self.settings.get("dterm_lpf1_dyn_min_hz", 0)):
            out["dterm_lpf1_dyn_min_hz"] = min(DYN_LPF_MAX_HZ, int(DTERM_LPF1_DYN_MIN_HZ_DEFAULT * multiplier))
            out["dterm_lpf1_dyn_max_hz"] = min(DYN_LPF_MAX_HZ, int(DTERM_LPF1_DYN_MAX_HZ_DEFAULT * multiplier))
        if int(self.settings.get("dterm_lpf1_static_hz", 0)):
            out["dterm_lpf1_static_hz"] = min(DYN_LPF_MAX_HZ, int(DTERM_LPF1_DYN_MIN_HZ_DEFAULT * multiplier))
        if int(self.settings.get("dterm_lpf2_static_hz", 0)):
            out["dterm_lpf2_static_hz"] = min(LPF_MAX_HZ, int(DTERM_LPF2_HZ_DEFAULT * multiplier))
        if apply:
            self.settings.update(out)
        return out

    def _calculate_simplified_gyro_filters(self, apply=True) -> dict:
        """calculateNewGyroFilterValues(): фильтры гироскопа от мультипликатора ползунка."""
        multiplier = self._slider("simplified_gyro_filter_multiplier") / 100.0
        out = {}
        if int(self.settings.get("simplified_gyro_filter", 1)) == 0:
            return out
        if int(self.settings.get("gyro_lpf1_dyn_min_hz", 0)):
            out["gyro_lpf1_dyn_min_hz"] = min(DYN_LPF_MAX_HZ, int(GYRO_LPF1_DYN_MIN_HZ_DEFAULT * multiplier))
            out["gyro_lpf1_dyn_max_hz"] = min(DYN_LPF_MAX_HZ, int(GYRO_LPF1_DYN_MAX_HZ_DEFAULT * multiplier))
        if int(self.settings.get("gyro_lpf1_static_hz", 0)):
            out["gyro_lpf1_static_hz"] = min(DYN_LPF_MAX_HZ, int(GYRO_LPF1_DYN_MIN_HZ_DEFAULT * multiplier))
        if int(self.settings.get("gyro_lpf2_static_hz", 0)):
            out["gyro_lpf2_static_hz"] = min(LPF_MAX_HZ, int(GYRO_LPF2_HZ_DEFAULT * multiplier))
        if apply:
            self.settings.update(out)
        return out

    def _apply_simplified_tuning(self):
        """applySimplifiedTuning(): пересчитать и записать PID и фильтры по ползункам."""
        if self._slider("simplified_pids_mode") != SIMPLIFIED_TUNING_OFF:
            self.settings.update(self._calculate_simplified_pids())
        self._calculate_simplified_dterm_filters()
        self._calculate_simplified_gyro_filters()

    # --------------------------------------------------- профили PID и профили рейтов
    def _save_active_rate_profile(self):
        """Переносит значения активного профиля рейтов из настроек в хранилище профилей."""
        profile = self.rate_profiles[self.current_rate_profile]
        for name in RATE_PROFILE_DEFAULTS:
            if name in self.settings:
                profile[name] = int(self.settings[name])

    def get_battery_profile_index(self) -> int:
        """getCurrentBatteryProfileIndex() мастера: активный профиль батареи (systemConfig)."""
        return self.active_battery_profile

    def change_battery_profile(self, index: int):
        """changeBatteryProfile() мастера: вне 0…BATTERY_PROFILE_COUNT-1 индекс не меняется."""
        if 0 <= index < BATTERY_PROFILE_COUNT:
            self.active_battery_profile = index
        self._log(f"[FC] профиль батареи выбран: {self.active_battery_profile}")

    def select_profile(self, value: int):
        """MSP_SELECT_SETTING (210): профиль PID, рейтов (бит 0x80) или батареи (бит 0x40).

        Разбор и ограничения — как в msp.c мастера: при батарейном бите значение вне
        диапазона приводится к 0, остальные ветки — как раньше.
        """
        if value & BATTERYPROFILE_MASK:
            index = value & ~BATTERYPROFILE_MASK
            if index >= BATTERY_PROFILE_COUNT:
                index = 0
            self.change_battery_profile(index)
            return
        if value & RATEPROFILE_MASK:
            index = value & ~RATEPROFILE_MASK
            if index >= CONTROL_RATE_PROFILE_COUNT:
                index = 0
            self._save_active_rate_profile()
            self.current_rate_profile = index
            for name, default in self.rate_profiles[index].items():
                self.settings[name] = default
        else:
            index = value if value < PID_PROFILE_COUNT else 0
            self._save_active_pid_profile()
            self.current_profile = index
            for name, default in self.pid_profiles[index].items():
                key = PID_PROFILE_ALIASES.get(name, name)
                self.settings[key if key in self.settings else name] = default
        self._log(f"[FC] профиль выбран: PID {self.current_profile}, рейты {self.current_rate_profile}")

    def _h_select_setting(self, w, r):
        value = r.u8() if len(r.data) else 0
        self.select_profile(value)
        return b""

    def _h_copy_profile(self, w, r):
        """MSP_COPY_PROFILE (183): 3 байта — тип (0 = PID, 1 = рейты), приёмник, источник."""
        d = bytes(r.rest())
        if len(d) < 3:
            return b""
        kind, dst, src = d[0], d[1], d[2]
        if kind == 0 and src < PID_PROFILE_COUNT and dst < PID_PROFILE_COUNT:
            self._save_active_pid_profile()
            self.pid_profiles[dst] = dict(self.pid_profiles[src])
            if dst == self.current_profile:
                for name, value in self.pid_profiles[dst].items():
                    self.settings[name] = value
            self._log(f"[FC] профиль PID {src + 1} скопирован в {dst + 1}")
        elif kind == 1 and src < CONTROL_RATE_PROFILE_COUNT and dst < CONTROL_RATE_PROFILE_COUNT:
            self._save_active_rate_profile()
            self.rate_profiles[dst] = dict(self.rate_profiles[src])
            if dst == self.current_rate_profile:
                for name, value in self.rate_profiles[dst].items():
                    self.settings[name] = value
            self._log(f"[FC] профиль рейтов {src + 1} скопирован в {dst + 1}")
        return b""

    def _h_set_reset_curr_pid(self, w, r):
        """MSP_SET_RESET_CURR_PID (219): resetPidProfile() — только для текущего профиля."""
        profile = dict(PID_PROFILE_DEFAULTS)
        self.pid_profiles[self.current_profile] = dict(profile)
        for name, value in profile.items():
            key = PID_PROFILE_ALIASES.get(name, name)
            if key in self.settings:
                self.settings[key] = value
            elif name in self.settings:
                self.settings[name] = value
        self._log(f"[FC] профиль PID {self.current_profile + 1} сброшен к заводским")
        return b""

    def _h_boxnames(self, w, r):
        page = r.u8() if len(r.data) else 0
        for name in self.active_boxes[page * 32 : (page + 1) * 32]:
            w.raw(name.encode() + b";")

    def _h_boxids(self, w, r):
        page = r.u8() if len(r.data) else 0
        active = [b for b in self.boxes if b["name"] in self.active_boxes]
        for b in active[page * 32 : (page + 1) * 32]:
            w.u8(b["permanent_id"])

    def _h_mode_ranges(self, w, r):
        """MSP_MODE_RANGES (34): ВСЕ 20 слотов по 4 байта — permanentId, AUX, startStep, endStep.

        Так же, как msp.c 4.5.4: цикл идёт по всему modeActivationConditions[], поэтому ответ
        всегда 80 байт. Пустые слоты уходят как «0 0 0 0» (режим ARM, 900/900) — вкладка
        «Режимы» считает их заполнителями и строк не рисует (см. modeRanges.js).
        """
        for slot in sorted(self.mode_ranges, key=lambda s: s["index"]):
            w.u8(self.box_permanent_id(slot["box"]))
            w.u8(slot["aux"])
            w.u8(self._mode_range_step(slot["start"]))
            w.u8(self._mode_range_step(slot["end"]))

    def _h_mode_ranges_extra(self, w, r):
        """MSP_MODE_RANGES_EXTRA (238): count + по 3 байта на слот — permanentId, logic, linkedTo.

        Без этого ответа вкладка не знает логику И/ИЛИ и связи режимов («Добавить связь»):
        Configurator читает его сразу после MSP_MODE_RANGES.
        """
        slots = sorted(self.mode_ranges, key=lambda s: s["index"])
        w.u8(len(slots))
        for slot in slots:
            w.u8(self.box_permanent_id(slot["box"]))
            w.u8(MODELOGIC_AND if slot["logic"] == MODELOGIC_AND else MODELOGIC_OR)
            w.u8(int(slot["linked"] or 0))

    def _h_serial_config(self, w, r):
        for p in self.serial_ports:
            w.u8(p["identifier"])
            w.u16(self._functions_to_mask(p["functions"]))
            w.u8(p.get("msp_baudrate", 5)).u8(p.get("gps_baudrate", 4)) \
             .u8(p.get("telemetry_baudrate", 0)).u8(p.get("blackbox_baudrate", 5))

    def _h_failsafe_config(self, w, r):
        """MSP_FAILSAFE_CONFIG (75): как pgResetFn_failsafeConfig() в failsafe.c 4.5.4.

        Заводские значения: задержка 15 (1.5 с), время посадки 10 (1.0 с), газ 1000,
        тумблер «Этап 1» (0), задержка низкого газа 100 (10 с), процедура DROP (1).
        """
        w.u8(int(self.settings.get("failsafe_delay", 15)))
        w.u8(int(self.settings.get("failsafe_off_delay", 10)))
        w.u16(int(self.settings.get("failsafe_throttle", 1000)))
        w.u8(int(self.settings.get("failsafe_switch_mode", 0)))
        w.u16(int(self.settings.get("failsafe_throttle_low_delay", 100)))
        w.u8(int(self.settings.get("failsafe_procedure", 1)))

    def _h_set_failsafe_config(self, w, r):
        """MSP_SET_FAILSAFE_CONFIG (76): пишем в те же настройки, что видны в CLI."""
        data = bytes(r.rest())
        if len(data) < 8:
            return b""
        self._store_setting("failsafe_delay", data[0])
        self._store_setting("failsafe_off_delay", data[1])
        self._store_setting("failsafe_throttle", int.from_bytes(data[2:4], "little"))
        self._store_setting("failsafe_switch_mode", data[4])
        self._store_setting("failsafe_throttle_low_delay", int.from_bytes(data[5:7], "little"))
        self._store_setting("failsafe_procedure", data[7])
        self._log(
            f"[MSP] FAILSAFE сохранён: задержка {self.settings['failsafe_delay']} (0.1 с), "
            f"посадка {self.settings['failsafe_off_delay']} (0.1 с), газ {self.settings['failsafe_throttle']}, "
            f"тумблер {self.settings['failsafe_switch_mode']}, низкий газ "
            f"{self.settings['failsafe_throttle_low_delay']} (0.1 с), процедура "
            f"{self.settings['failsafe_procedure']}"
        )
        return b""   # пустой ACK: вкладка Failsafe ждёт ответа на запись

    def _h_gps_config(self, w, r):
        """MSP_GPS_CONFIG (132): provider, ublox_sbas, auto_config, auto_baud, home_point_once,
        ublox_use_galileo — ровно в том порядке, в каком их читает MSPHelper.js."""
        w.u8(self._setting("gps_provider", 1))
        w.u8(self._setting("gps_sbas_mode", 0))
        w.u8(self._setting("gps_auto_config", 1))
        w.u8(self._setting("gps_auto_baud", 0))
        w.u8(self._setting("gps_set_home_point_once", 0))
        w.u8(self._setting("gps_ublox_use_galileo", 0))

    def _h_set_gps_config(self, w, r):
        """MSP_SET_GPS_CONFIG (223) — то, что пишет кнопка «Сохранить» во вкладке «GPS»:
        те же шесть полей в том же порядке, что и чтение."""
        names = ("gps_provider", "gps_sbas_mode", "gps_auto_config", "gps_auto_baud",
                 "gps_set_home_point_once", "gps_ublox_use_galileo")
        values = list(r.data[r.pos : r.pos + len(names)])
        for name, value in zip(names, values):
            self._store_setting(name, value)
        self._log("[MSP] GPS_CONFIG сохранён: " + ", ".join(f"{n}={v}" for n, v in zip(names, values)))
        return b""

    def _h_set_sensor_config(self, w, r):
        """MSP_SET_SENSOR_CONFIG: acc, baro, mag, sonar (API 1.46 — 4 байта).

        Так вкладка «Датчики» выключает датчики: 1 = NONE, 0 = AUTO.
        """
        keys = ("acc_hardware", "baro_hardware", "mag_hardware", "sonar_hardware")
        for key in keys:
            if r.pos < len(r.data):
                self.sensor_config[key] = r.u8()
        self._log(
            "[MSP] конфигурация датчиков: acc={acc_hardware} baro={baro_hardware} "
            "mag={mag_hardware} sonar={sonar_hardware}".format(**self.sensor_config)
        )

    def _h_sensor_config_read(self, w, r):
        """MSP_SENSOR_CONFIG (96): acc, baro, mag, sonar — что выбрано во вкладке «Датчики»."""
        cfg = self.sensor_config
        w.u8(cfg["acc_hardware"]).u8(cfg["baro_hardware"])
        w.u8(cfg["mag_hardware"]).u8(cfg["sonar_hardware"])

    def _h_sensor_config_active(self, w, r):
        """MSP2_SENSOR_CONFIG_ACTIVE (0x300A): какие датчики реально найдены (detectedSensors[]).

        id — из таблиц прошивки (gyro.h, acceleration.h, barometer.h, compass.h,
        rangefinder.h); Configurator показывает по ним название чипа. 0 — «не найден».
        """
        cfg = self.sensor_config
        mag = self.DETECTED_MAG if cfg["mag_hardware"] != SENSOR_NONE else 0
        sonar = self.DETECTED_RANGEFINDER if cfg["sonar_hardware"] != RANGEFINDER_NONE else 0
        w.u8(self.DETECTED_GYRO).u8(self.DETECTED_ACC).u8(self.DETECTED_BARO).u8(mag).u8(sonar)

    def _setting(self, name, default=0):
        """Значение настройки (если она есть в прошивке), иначе значение по умолчанию."""
        value = self.settings.get(name)
        return default if value is None else int(value)

    def _store_setting(self, name, value):
        """Запись значения так, как это сделал бы CLI 'set': с ограничением по диапазону."""
        entry = self.settings_by_name.get(name)
        if entry is None:
            return False
        if entry["mode"] == "MODE_LOOKUP":
            lo, hi = 0, max(len(entry.get("values") or []) - 1, 0)
        else:
            lo, hi = entry["min"], entry["max"]
        self.settings[name] = max(lo, min(hi, int(value)))
        return True

    def _h_rx_config(self, w, r):
        """MSP_RX_CONFIG — ровно как sbufWrite* в msp.c (API 1.46, 38 байт).

        Раскладка: provider, maxcheck, midrc, mincheck, sat_bind, rx_min_usec, rx_max_usec,
        два устаревших байта, порог airmode (значение*10+1000), RX-SPI (3 поля),
        угол камеры, два устаревших байта, срезы RC-smoothing, два устаревших байта,
        USB CDC HID, авто-фактор и режим RC-smoothing, UID ExpressLRS (6 байт).
        """
        airmode_wire = self._setting("airmode_start_throttle_percent", 30) * 10 + 1000
        w.u8(self._setting("serialrx_provider", 0))
        w.u16(self._setting("max_check", 2000))
        w.u16(self._setting("mid_rc", 1500))
        w.u16(self._setting("min_check", 1050))
        w.u8(self._setting("spektrum_sat_bind", 0))
        w.u16(self._setting("rx_min_usec", 885))
        w.u16(self._setting("rx_max_usec", 2115))
        w.u8(0)  # устарело: rcInterpolation
        w.u8(0)  # устарело: rcInterpolationInterval
        w.u16(airmode_wire)
        w.u8(0)  # RX-SPI протокол (на плате не используется)
        w.u32(0)  # RX-SPI id
        w.u8(0)  # RX-SPI rf channel count
        w.u8(self._setting("fpv_mix_degrees", 0))
        w.u8(0)  # устарело: rcInterpolationChannels
        w.u8(0)  # устарело: rcSmoothingType
        w.u8(self._setting("rc_smoothing_setpoint_cutoff", 0))
        w.u8(self._setting("rc_smoothing_feedforward_cutoff", 0))
        w.u8(0)  # устарело: rcSmoothingInputType
        w.u8(0)  # устарело: rcSmoothingDerivativeType
        w.u8(self._setting("usb_hid_cdc", 0))
        w.u8(self._setting("rc_smoothing_auto_factor", 30))
        w.u8(self._setting("rc_smoothing", 0))
        w.raw(bytes(6))  # UID ExpressLRS (пусто)

    def _h_set_rx_config(self, w, r):
        """MSP_SET_RX_CONFIG: пишем в те же CLI-настройки, что видны командой get."""
        data = bytes(r.rest())
        if len(data) < 32:
            return
        self._store_setting("serialrx_provider", data[0])
        self._store_setting("max_check", int.from_bytes(data[1:3], "little"))
        self._store_setting("mid_rc", int.from_bytes(data[3:5], "little"))
        self._store_setting("min_check", int.from_bytes(data[5:7], "little"))
        self._store_setting("spektrum_sat_bind", data[7])
        self._store_setting("rx_min_usec", int.from_bytes(data[8:10], "little"))
        self._store_setting("rx_max_usec", int.from_bytes(data[10:12], "little"))
        airmode_wire = int.from_bytes(data[14:16], "little")
        self._store_setting("airmode_start_throttle_percent", max(0, (airmode_wire - 1000)) // 10)
        self._store_setting("fpv_mix_degrees", data[22])
        self._store_setting("rc_smoothing_setpoint_cutoff", data[25])
        self._store_setting("rc_smoothing_feedforward_cutoff", data[26])
        self._store_setting("usb_hid_cdc", data[29])
        self._store_setting("rc_smoothing_auto_factor", data[30])
        self._store_setting("rc_smoothing", data[31])
        self._log(
            "[MSP] RX_CONFIG сохранён: provider={serialrx_provider}, rx_min_usec={rx_min_usec}, "
            "rx_max_usec={rx_max_usec}, airmode={airmode_start_throttle_percent}%, "
            "rc_smoothing={rc_smoothing}".format(**self.settings)
        )

    def _h_rssi_config(self, w, r):
        w.u8(int(self.settings.get("rssi_channel", 0)))

    def _h_set_rssi_config(self, w, r):
        """MSP_SET_RSSI_CONFIG (51): номер канала RSSI — в ту же настройку, что видит CLI.

        Раньше код шёл «сквозной записью» и подменял только ответ вкладке: «Приёмник»
        показывал выбранный канал, а `get rssi_channel` возвращал 0.
        """
        data = bytes(r.rest())
        if data and self._store_setting("rssi_channel", data[0]):
            self._log(f"[MSP] RSSI-канал: {self.settings['rssi_channel']} (rssi_channel)")
        return b""

    def _h_name(self, w, r):
        # msp.c (MSP_NAME): символы без длины и без завершающего нуля
        w.raw(self.craft_name.encode("latin-1", "replace"))

    def _h_get_text(self, w, r):
        text_type = r.u8()
        mapping = {
            TEXT_PILOT_NAME: self.pilot_name,
            TEXT_CRAFT_NAME: self.craft_name,
            TEXT_PID_PROFILE_NAME: self.pid_profile_names.get(self.current_profile, "Profile 1"),
            TEXT_RATE_PROFILE_NAME: self.rate_profile_names.get(self.current_rate_profile, "Actual"),
            TEXT_BUILDKEY: self.build_key,
            TEXT_BATTERY_PROFILE_NAME: self.battery_profiles[self.active_battery_profile]["name"],
        }
        w.u8(text_type).ltext(mapping.get(text_type, ""))

    # Протоколы моторов, для которых шим не синхронизирован (analog), — как motorPwmProtocol_e.
    UNSYNCED_PROTOCOLS = {0, 1, 2, 3, 4}  # PWM, ONESHOT125, ONESHOT42, MULTISHOT, BRUSHED
    DEBUG_COUNT = 90  # DEBUG_COUNT из build/debug.h (4.5.4)

    def _h_advanced_config(self, w, r):
        """MSP_ADVANCED_CONFIG — 20 байт, как sbufWrite* в msp.c.

        Второй байт (pid_process_denom) задаёт частоту цикла PID в интерфейсе, поэтому
        берём его из настройки: get pid_process_denom показывает то же значение.
        """
        protocol = self._setting("motor_pwm_protocol", 7)
        w.u8(1)  # устаревшее gyro_sync_denom (прошивка всегда пишет 1)
        w.u8(self._setting("pid_process_denom", 1))
        w.u8(1 if protocol in self.UNSYNCED_PROTOCOLS else 0)
        w.u8(protocol)
        w.u16(self._setting("motor_pwm_rate", 480))
        w.u16(self._setting("dshot_idle_value", 550))  # digitalIdleOffsetValue
        w.u8(0)  # устаревшее gyro_use_32kHz
        w.u8(self._setting("motor_pwm_inversion", 0))
        w.u8(self._setting("gyro_to_use", 0))
        w.u8(self._setting("gyro_high_range", 0))          # gyro_high_fsr
        w.u8(self._setting("gyro_calib_noise_limit", 48))  # gyroMovementCalibrationThreshold
        w.u16(self._setting("gyro_calib_duration", 125))
        w.u16(self._setting("gyro_offset_yaw", 0) & 0xFFFF)
        w.u8(self._setting("gyro_overflow_detect", 1))
        w.u8(self._setting("debug_mode", 0))
        w.u8(self.DEBUG_COUNT)

    def _h_set_advanced_config(self, w, r):
        """MSP_SET_ADVANCED_CONFIG (19 байт из интерфейса): пишем то, что есть в CLI-настройках."""
        data = bytes(r.rest())
        if len(data) < 19:
            return
        self._store_setting("pid_process_denom", data[1])
        self._store_setting("motor_pwm_protocol", data[3])
        self._store_setting("motor_pwm_rate", int.from_bytes(data[4:6], "little"))
        self._store_setting("dshot_idle_value", int.from_bytes(data[6:8], "little"))
        self._store_setting("motor_pwm_inversion", data[9])
        self._store_setting("gyro_to_use", data[10])
        self._store_setting("gyro_high_range", data[11])
        self._store_setting("gyro_calib_noise_limit", data[12])
        self._store_setting("gyro_calib_duration", int.from_bytes(data[13:15], "little"))
        self._store_setting("gyro_offset_yaw", int.from_bytes(data[15:17], "little", signed=True))
        self._store_setting("gyro_overflow_detect", data[17])
        self._store_setting("debug_mode", data[18])
        self._log(
            "[MSP] ADVANCED_CONFIG сохранён: pid_process_denom={pid_process_denom}, "
            "motor_pwm_protocol={motor_pwm_protocol}, motor_pwm_rate={motor_pwm_rate}, "
            "dshot_idle_value={dshot_idle_value}, debug_mode={debug_mode}".format(**self.settings)
        )

    def _h_rc_deadband(self, w, r):
        """MSP_RC_DEADBAND (125): 5 байт — deadband, yaw_deadband, alt_hold_deadband, 3D-дедбенд.

        Заводские значения rcControlsConfig: deadband 0, yaw_deadband 0, alt_hold_deadband 40;
        flight3DConfig: deadband3d_throttle 50 (fc/rc_controls.c в 4.5.4).
        """
        s = lambda n, d=0: int(self.settings.get(n, d))
        w.u8(s("deadband", 0)).u8(s("yaw_deadband", 0)).u8(s("alt_hold_deadband", 40))
        w.u16(s("3d_deadband_throttle", 50))

    def _h_set_rc_deadband(self, w, r):
        """MSP_SET_RC_DEADBAND (218): 5 байт — та же раскладка, что у ответа 125."""
        d = bytes(r.rest())
        if len(d) < 5:
            return b""
        self.settings["deadband"], self.settings["yaw_deadband"] = d[0], d[1]
        self.settings["alt_hold_deadband"] = d[2]
        self._store_setting("3d_deadband_throttle", int.from_bytes(d[3:5], "little"))
        self._log(f"[MSP] RC_DEADBAND сохранены: deadband={self.settings['deadband']}, "
                  f"yaw_deadband={self.settings['yaw_deadband']}, 3D={self.settings['3d_deadband_throttle']}")
        return b""

    def _h_filter_config(self, w, r):
        """MSP_FILTER_CONFIG (92): 49 байт строго в порядке sbufWrite* из msp.c 4.5.4.

        Порядок фиксирован Configurator'ом (MSPHelper.js читает поля подряд), поэтому
        «улучшать» раскладку нельзя: гироскоп и D-term идут вперемешку.
        """
        s = lambda n, d=0: int(self.settings.get(n, d))
        w.u8(s("gyro_lpf1_static_hz", 250) & 0xFF)            # устаревший байт gyro_lpf1_static_hz
        w.u16(s("dterm_lpf1_static_hz", 75))
        w.u16(s("yaw_lowpass_hz", 100))
        w.u16(s("gyro_notch1_hz", 0)).u16(s("gyro_notch1_cutoff", 0))
        w.u16(s("dterm_notch_hz", 0)).u16(s("dterm_notch_cutoff", 0))
        w.u16(s("gyro_notch2_hz", 0)).u16(s("gyro_notch2_cutoff", 0))
        w.u8(s("dterm_lpf1_type", 0))
        w.u8(s("gyro_hardware_lpf", 0))
        w.u8(0)                                               # устаревший gyro_32khz_hardware_lpf
        w.u16(s("gyro_lpf1_static_hz", 250)).u16(s("gyro_lpf2_static_hz", 500))
        w.u8(s("gyro_lpf1_type", 0)).u8(s("gyro_lpf2_type", 0))
        w.u16(s("dterm_lpf2_static_hz", 150))
        w.u8(s("dterm_lpf2_type", 0))
        w.u16(s("gyro_lpf1_dyn_min_hz", 250)).u16(s("gyro_lpf1_dyn_max_hz", 500))
        w.u16(s("dterm_lpf1_dyn_min_hz", 75)).u16(s("dterm_lpf1_dyn_max_hz", 150))
        w.u8(0).u8(0)                                         # устаревшие dyn_notch_range/width
        w.u16(s("dyn_notch_q", 300)).u16(s("dyn_notch_min_hz", 100))
        w.u8(self._rpm_setting("rpm_filter_harmonics", 3))
        w.u8(self._rpm_setting("rpm_filter_min_hz", 100))
        w.u16(s("dyn_notch_max_hz", 600))
        w.u8(s("dterm_lpf1_dyn_expo", 5))
        w.u8(s("dyn_notch_count", 3))

    def _rpm_setting(self, name, default):
        """Настройки RPM-фильтра: в bf_data.json они носят имена gyro_rpm_notch_* (из msp.c)."""
        alias = {"rpm_filter_harmonics": "gyro_rpm_notch_harmonics",
                 "rpm_filter_min_hz": "gyro_rpm_notch_min_hz"}
        for key in (name, alias.get(name, name)):
            if key in self.settings:
                return int(self.settings[key])
        return default

    def _h_set_filter_config(self, w, r):
        """MSP_SET_FILTER_CONFIG (93): разбор той же раскладки (вкладки «Фильтры» и «PID»)."""
        d = bytes(r.rest())
        if len(d) < 49:
            return b""
        u16 = lambda o: int.from_bytes(d[o:o + 2], "little")
        self.settings["dterm_lpf1_static_hz"] = u16(1)
        self.settings["yaw_lowpass_hz"] = u16(3)
        self.settings["gyro_notch1_hz"], self.settings["gyro_notch1_cutoff"] = u16(5), u16(7)
        self.settings["dterm_notch_hz"], self.settings["dterm_notch_cutoff"] = u16(9), u16(11)
        self.settings["gyro_notch2_hz"], self.settings["gyro_notch2_cutoff"] = u16(13), u16(15)
        self.settings["dterm_lpf1_type"] = d[17]
        self.settings["gyro_hardware_lpf"] = d[18]
        self.settings["gyro_lpf1_static_hz"], self.settings["gyro_lpf2_static_hz"] = u16(20), u16(22)
        self.settings["gyro_lpf1_type"], self.settings["gyro_lpf2_type"] = d[24], d[25]
        self.settings["dterm_lpf2_static_hz"] = u16(26)
        self.settings["dterm_lpf2_type"] = d[28]
        self.settings["gyro_lpf1_dyn_min_hz"], self.settings["gyro_lpf1_dyn_max_hz"] = u16(29), u16(31)
        self.settings["dterm_lpf1_dyn_min_hz"], self.settings["dterm_lpf1_dyn_max_hz"] = u16(33), u16(35)
        self.settings["dyn_notch_q"], self.settings["dyn_notch_min_hz"] = u16(39), u16(41)
        self.settings["gyro_rpm_notch_harmonics"], self.settings["gyro_rpm_notch_min_hz"] = d[43], d[44]
        self.settings["dyn_notch_max_hz"] = u16(45)
        self.settings["dterm_lpf1_dyn_expo"] = d[47]
        self.settings["dyn_notch_count"] = d[48]
        self._save_active_pid_profile()
        self._log(f"[MSP] FILTER_CONFIG сохранён: dterm_lpf1={self.settings['dterm_lpf1_static_hz']}, "
                  f"gyro_notch1={self.settings['gyro_notch1_hz']}")
        return b""

    def _h_adjustment_ranges(self, w, r):
        """MSP_ADJUSTMENT_RANGES (52): все 30 слотов подряд по 6 байт — как в msp.c 4.5.4.

        Байта-счётчика в ответе нет: Configurator считает слоты как «длина ответа / 6»
        (MSPHelper.ts; при API ≥ 1.48 на слот приходится 10 байт — центр и масштаб). Первый
        байт прошивка пишет нулём («was adjRange->adjustmentIndex»), следом идут канал,
        начало, конец, функция и канал-переключатель.
        """
        for slot in self.adjustment_ranges:
            w.u8(0).u8(slot["aux"]).u8(slot["start"]).u8(slot["end"])
            w.u8(slot["function"]).u8(slot["switch"])

    def _h_set_adjustment_range(self, w, r):
        """MSP_SET_ADJUSTMENT_RANGE (53): запись одного слота (его индекс — первым байтом)."""
        index = r.u8()
        r.u8()                    # slotIndex: прошивка его игнорирует («was adjustmentIndex»)
        if index >= ADJUSTMENT_RANGE_COUNT:
            self.writes_log.append(
                f"слот корректировки {index}: вне таблицы (0…{ADJUSTMENT_RANGE_COUNT - 1})"
            )
            return
        slot = self.adjustment_ranges[index]
        slot["aux"] = r.u8() & 0x0F
        slot["start"] = min(48, r.u8())
        slot["end"] = min(48, r.u8())
        slot["function"] = min(ADJUSTMENT_FUNCTION_COUNT, r.u8())
        slot["switch"] = r.u8() & 0x0F
        self._log(f"[MSP] слот корректировки {index + 1}: AUX{slot['aux'] + 1} "
                  f"{900 + slot['start'] * 25}…{900 + slot['end'] * 25} импульсов → "
                  f"функция {slot['function']} через AUX{slot['switch'] + 1}")

    def _h_osd_config(self, w, r):
        """MSP_OSD_CONFIG (84) — ровно тот порядок полей, что в msp.c 4.5.4.

        Configurator читает ответ по этой раскладке (osd.js: OSD.msp.decode): сначала флаги,
        потом видеосистему, единицы, тревоги, количество элементов и сами места элементов,
        статистику, таймеры, включённые предупреждения, профили, кадр камеры и порог LQ.
        Если флаги нулевые, вкладка «OSD» остаётся пустой — поэтому флаги обязательно непустые.
        """
        cfg = self.osd_config
        w.u8(cfg["flags"])
        w.u8(cfg["video_system"])
        w.u8(cfg["units"])
        w.u8(cfg["rssi_alarm"])
        w.u16(cfg["cap_alarm"])
        w.u8(0)                       # устаревшая «тревога таймера» — теперь тут номер, ниже
        w.u8(OSD_ITEM_COUNT)
        w.u16(cfg["alt_alarm"])
        for value in cfg["item_pos"]:
            w.u16(int(value))
        w.u8(OSD_STAT_COUNT)
        for index in range(OSD_STAT_COUNT):
            w.u8(1 if cfg["enabled_stats"] & (1 << index) else 0)
        w.u8(OSD_TIMER_COUNT)
        for value in cfg["timers"]:
            w.u16(int(value))
        w.u16(cfg["enabled_warnings"] & 0xFFFF)     # младшее слово — для старых версий
        w.u8(OSD_WARNING_COUNT)
        w.u32(cfg["enabled_warnings"])
        w.u8(OSD_PROFILE_COUNT)
        w.u8(cfg["osd_profile_index"])
        w.u8(cfg["overlay_radio_mode"])
        w.u8(cfg["camera_frame_width"])
        w.u8(cfg["camera_frame_height"])
        w.u16(cfg["link_quality_alarm"])            # API >= 1.46

    def _h_set_osd_config(self, w, r):
        """MSP_SET_OSD_CONFIG (85) — как mspProcessInCommand: адрес -1 = общее, -2 = таймер.

        Иначе адрес — номер элемента (значение u16 и байт «экрана»: 0 — статистика после полёта,
        1 и выше — элементы на экране). Так Configurator сохраняет перетаскивание элементов,
        галочки статистики, тревоги и пороги.
        """
        data = bytes(r.rest())
        if not data:
            return
        addr = data[0]
        pos = 1
        unsigned_addr = addr if addr < 128 else addr - 256      # (int8_t)addr, как в прошивке
        cfg = self.osd_config
        if unsigned_addr == -1:                                  # общие настройки
            def take(count):
                nonlocal pos
                chunk = data[pos:pos + count]
                pos += count
                return chunk
            if pos < len(data):
                cfg["video_system"] = take(1)[0]
            if pos < len(data):
                cfg["units"] = take(1)[0]
            if pos < len(data):
                cfg["rssi_alarm"] = take(1)[0]
            if pos + 2 <= len(data):
                cfg["cap_alarm"] = int.from_bytes(take(2), "little")
            if pos + 2 <= len(data):                             # устаревший таймер — пропускаем
                take(2)
            if pos + 2 <= len(data):
                cfg["alt_alarm"] = int.from_bytes(take(2), "little")
            if pos + 2 <= len(data):
                cfg["enabled_warnings"] |= int.from_bytes(take(2), "little")
            if pos + 4 <= len(data):
                cfg["enabled_warnings"] = int.from_bytes(take(4), "little")
            if pos < len(data):
                index = take(1)[0]
                cfg["osd_profile_index"] = index if 1 <= index <= OSD_PROFILE_COUNT else 1
            if pos < len(data):
                cfg["overlay_radio_mode"] = take(1)[0]
            if pos + 2 <= len(data):
                cfg["camera_frame_width"] = take(1)[0]
                cfg["camera_frame_height"] = take(1)[0]
            if pos + 2 <= len(data):
                cfg["link_quality_alarm"] = int.from_bytes(take(2), "little")
            self._log("[MSP] SET_OSD_CONFIG: общие настройки — видео "
                      f"{cfg['video_system']}, единицы {cfg['units']}, предупреждения "
                      f"0x{cfg['enabled_warnings']:x}, профиль {cfg['osd_profile_index']}")
            return
        if unsigned_addr == -2:                                  # таймер
            if pos < len(data):
                index = data[pos]
                pos += 1
                if index < OSD_TIMER_COUNT and pos + 2 <= len(data):
                    cfg["timers"][index] = int.from_bytes(data[pos:pos + 2], "little")
                    self._log(f"[MSP] SET_OSD_CONFIG: таймер {index + 1} = {cfg['timers'][index]}")
            return
        # элемент: u16 значение, затем байт экрана (0 — статистика)
        if pos + 2 > len(data):
            return
        value = int.from_bytes(data[pos:pos + 2], "little")
        pos += 2
        screen = data[pos] if pos < len(data) else 1
        if screen == 0:
            if addr < OSD_STAT_COUNT:
                if value:
                    cfg["enabled_stats"] |= 1 << addr
                else:
                    cfg["enabled_stats"] &= ~(1 << addr)
        elif addr < OSD_ITEM_COUNT:
            cfg["item_pos"][addr] = value
        else:
            return
        self._log(f"[MSP] SET_OSD_CONFIG: экран {screen}, адрес {addr} = {value}")

    def _h_osd_canvas(self, w, r):
        """MSP_OSD_CANVAS (189) — размер «полотна» OSD: столбцы и строки."""
        w.u8(self.osd_config["canvas_cols"])
        w.u8(self.osd_config["canvas_rows"])

    def _h_set_osd_canvas(self, w, r):
        """MSP_SET_OSD_CANVAS (190) — установка размера полотна (как в msp.c)."""
        data = bytes(r.rest())
        if len(data) >= 2:
            self.osd_config["canvas_cols"] = data[0]
            self.osd_config["canvas_rows"] = data[1]
            self._log(f"[MSP] SET_OSD_CANVAS: {data[0]}×{data[1]}")

    def _h_vtx_config(self, w, r):
        """MSP_VTX_CONFIG — 15 байт, как sbufWrite* в msp.c (API 1.42).

        Раскладка: type, band, channel, power, pit_mode, freq u16, device_ready,
        low_power_disarm, pit_mode_freq u16, table_available, bands, channels, powerlevels.
        """
        device_type = self._vtx_device_type()
        table = self.vtx_table
        w.u8(device_type)
        w.u8(self._setting("vtx_band", 0))
        w.u8(self._setting("vtx_channel", 0))
        w.u8(self._setting("vtx_power", 0))
        w.u8(1 if self.vtx_pit_mode else 0)
        w.u16(self._setting("vtx_freq", 0))
        w.u8(1 if device_type != VTXDEV_UNKNOWN else 0)  # deviceIsReady
        w.u8(self._setting("vtx_low_power_disarm", 0))
        w.u16(self._setting("vtx_pit_mode_freq", 0))
        w.u8(1)  # таблица доступна (сборка с USE_VTX_TABLE)
        w.u8(table["bands"]).u8(table["channels"]).u8(table["powerlevels"])

    def _h_vtx_table_band(self, w, r):
        """MSP_VTXTABLE_BAND: в запросе номер банда, в ответ — имя, буква, флаг и частоты."""
        band = r.u8() if r.pos < len(r.data) else 0
        if not 1 <= band <= VTX_TABLE_MAX_BANDS:
            self._log(f"[MSP] VTXTABLE_BAND: банду {band} нет — как на железе, ошибка")
            return
        table = self.vtx_table
        w.u8(band)
        w.u8(VTX_TABLE_BAND_NAME_LENGTH)
        for char in self._pad(table["band_names"][band - 1], VTX_TABLE_BAND_NAME_LENGTH):
            w.u8(ord(char))
        letter = table["band_letters"][band - 1] or " "
        w.u8(ord(letter[0]))
        w.u8(1 if table["is_factory"][band - 1] else 0)
        w.u8(table["channels"])
        for channel in range(table["channels"]):
            w.u16(int(table["frequencies"][band - 1][channel]))

    def _h_vtx_table_powerlevel(self, w, r):
        """MSP_VTXTABLE_POWERLEVEL: значение в мВт и подпись уровня."""
        level = r.u8() if r.pos < len(r.data) else 0
        if not 1 <= level <= VTX_TABLE_MAX_POWER_LEVELS:
            self._log(f"[MSP] VTXTABLE_POWERLEVEL: уровня {level} нет — как на железе, ошибка")
            return
        table = self.vtx_table
        w.u8(level)
        w.u16(int(table["power_values"][level - 1]))
        w.u8(VTX_TABLE_POWER_LABEL_LENGTH)
        for char in self._pad(table["power_labels"][level - 1], VTX_TABLE_POWER_LABEL_LENGTH):
            w.u8(ord(char))

    def _h_vtx_device_status(self, w, r):
        """MSP2_GET_VTX_DEVICE_STATUS (0x3004) — vtxCommonSerializeDeviceStatus().

        Без устройства прошивка не пишет ничего; со SmartAudio — тип, готовность,
        банд/канал, индекс мощности, частоту, флаги состояния, список уровней (dBm и мВт)
        и «паспорт» передатчика: размер, версия, режим, частота pit-режима, признак загрузки в pit.
        """
        device_type = self._vtx_device_type()
        if device_type == VTXDEV_UNKNOWN:
            return
        table = self.vtx_table
        band = self._setting("vtx_band", 0)
        channel = self._setting("vtx_channel", 0)
        w.u8(device_type).u8(1)                  # type, deviceIsReady
        w.u8(1).u8(band).u8(channel)             # bandAndChannelAvailable
        w.u8(1).u8(self._setting("vtx_power", 0))  # powerIndexAvailable
        w.u8(1).u16(self._setting("vtx_freq", 0))  # frequencyAvailable
        status = VTX_STATUS_PIT_MODE if self.vtx_pit_mode else 0
        w.u8(1).u32(status)                      # vtxStatusAvailable
        w.u8(table["powerlevels"])
        for index in range(table["powerlevels"]):
            power_mw = int(table["power_values"][index])
            dbm = round(10 * math.log10(power_mw)) if power_mw > 0 else 0
            w.u16(dbm).u16(power_mw)
        w.u8(5)                                   # VTX_CUSTOM_DEVICE_STATUS_SIZE
        w.u8(SA_VERSION).u8(SA_MODE_UNLOCKED)
        w.u16(self._setting("vtx_pit_mode_freq", 0))
        w.u8(0)                                   # willBootIntoPitMode

    def _h_set_vtx_config(self, w, r):
        """MSP_SET_VTX_CONFIG — та же логика, что в msp.c: частота/банд-канал и условные поля."""
        data = bytes(r.rest())
        if len(data) < 2:
            return
        new_frequency = int.from_bytes(data[0:2], "little")
        if new_frequency <= VTX_COMMON_MSP_BANDCHAN_CHKVAL:      # упакованные банд и канал
            band = new_frequency // 8 + 1
            channel = new_frequency % 8 + 1
            self._store_setting("vtx_band", band)
            self._store_setting("vtx_channel", channel)
            self._store_setting("vtx_freq", self._vtx_lookup_frequency(band, channel))
        elif new_frequency <= 5999:                              # частота в МГц
            self._store_setting("vtx_band", 0)
            self._store_setting("vtx_freq", new_frequency)

        if len(data) >= 4:
            self._store_setting("vtx_power", data[2])
            self.vtx_pit_mode = bool(data[3])                    # делегируется передатчику

        if len(data) >= 5:
            self._store_setting("vtx_low_power_disarm", data[4])

        if len(data) >= 7:
            self._store_setting("vtx_pit_mode_freq", int.from_bytes(data[5:7], "little"))

        if len(data) >= 11:                                      # банд/канал/частота отдельными полями
            band, channel = data[7], data[8]
            frequency = int.from_bytes(data[9:11], "little")
            if band:
                frequency = self._vtx_lookup_frequency(band, channel)
            self._store_setting("vtx_band", band)
            self._store_setting("vtx_channel", channel)
            self._store_setting("vtx_freq", frequency)

        if len(data) >= 15:                                      # размеры таблицы и её очистка
            bands, channels, powerlevels, clear = data[11], data[12], data[13], data[14]
            if (bands > VTX_TABLE_MAX_BANDS or channels > VTX_TABLE_MAX_CHANNELS
                    or powerlevels > VTX_TABLE_MAX_POWER_LEVELS):
                self._log("[MSP] SET_VTX_CONFIG: размеры таблицы больше допустимых — как на железе, ошибка")
                return
            self.vtx_table["bands"] = bands
            self.vtx_table["channels"] = channels
            self.vtx_table["powerlevels"] = powerlevels
            if clear:
                self._vtx_clear_table()
                self.vtx_table["bands"] = bands
                self.vtx_table["channels"] = channels
                self.vtx_table["powerlevels"] = powerlevels
        self._log(
            "[MSP] VTX_CONFIG сохранён: band={vtx_band}, channel={vtx_channel}, "
            "power={vtx_power}, freq={vtx_freq}, pit_mode={}".format(
                "on" if self.vtx_pit_mode else "off", **self.settings
            )
        )

    def _h_set_vtx_table_band(self, w, r):
        """MSP_SET_VTXTABLE_BAND: имя и буква в верхний регистр, частоты по числу каналов."""
        data = bytes(r.rest())
        if len(data) < 4:
            return
        band = data[0]
        name_length = data[1]
        name = "".join(chr(c) for c in data[2:2 + name_length]).upper()
        pos = 2 + name_length
        if pos + 2 > len(data):
            return
        letter = chr(data[pos]).upper()
        is_factory = bool(data[pos + 1])
        channel_count = data[pos + 2] if pos + 2 < len(data) else 0
        frequencies = [
            int.from_bytes(data[pos + 3 + 2 * i:pos + 5 + 2 * i], "little")
            for i in range(channel_count)
            if pos + 5 + 2 * i <= len(data)
        ]
        table = self.vtx_table
        if not 1 <= band <= table["bands"]:
            self._log(f"[MSP] SET_VTXTABLE_BAND: банду {band} нет в таблице — ошибка")
            return
        table["band_names"][band - 1] = self._pad(name, VTX_TABLE_BAND_NAME_LENGTH)
        table["band_letters"][band - 1] = letter or " "
        table["is_factory"][band - 1] = is_factory
        for channel in range(table["channels"]):
            table["frequencies"][band - 1][channel] = (
                frequencies[channel] if channel < len(frequencies) else 0
            )
        if band == self._setting("vtx_band", 0):                 # это выбранный банд — частота меняется
            current_channel = self._setting("vtx_channel", 0)
            if 1 <= current_channel <= table["channels"]:
                self._store_setting("vtx_freq", table["frequencies"][band - 1][current_channel - 1])
        self._log(f"[MSP] VTXTABLE_BAND {band}: имя {name!r}, каналов {table['channels']}")

    def _h_set_vtx_table_powerlevel(self, w, r):
        """MSP_SET_VTXTABLE_POWERLEVEL: значение в мВт и подпись уровня."""
        data = bytes(r.rest())
        if len(data) < 4:
            return
        level = data[0]
        value = int.from_bytes(data[1:3], "little")
        label_length = data[3]
        label = "".join(chr(c) for c in data[4:4 + label_length]).upper()
        table = self.vtx_table
        if not 1 <= level <= table["powerlevels"]:
            self._log(f"[MSP] SET_VTXTABLE_POWERLEVEL: уровня {level} нет в таблице — ошибка")
            return
        table["power_values"][level - 1] = value
        table["power_labels"][level - 1] = self._pad(label, VTX_TABLE_POWER_LABEL_LENGTH)
        self._log(f"[MSP] VTXTABLE_POWERLEVEL {level}: {value} мВт, подпись {label!r}")

    def _h_battery_config(self, w, r):
        """MSP_BATTERY_CONFIG (32): 13 байт, как в msp.c 4.5.4.

        Сначала три «округлённых до 0.1 В» байта (для старых GUI), затем ёмкость,
        источники датчиков и, в конце, точные напряжения в 0.01 В — Configurator
        читает их последними и они перекрывают округлённые.
        """
        w.u8((self._battery_setting("vbat_min_cell_voltage") + 5) // 10)
        w.u8((self._battery_setting("vbat_max_cell_voltage") + 5) // 10)
        w.u8((self._battery_setting("vbat_warning_cell_voltage") + 5) // 10)
        w.u16(self._battery_setting("bat_capacity", 0, 20000))
        w.u8(int(self.settings.get("battery_meter", 0)))
        w.u8(int(self.settings.get("current_meter", 0)))
        w.u16(self._battery_setting("vbat_min_cell_voltage"))
        w.u16(self._battery_setting("vbat_max_cell_voltage"))
        w.u16(self._battery_setting("vbat_warning_cell_voltage"))

    def _h_set_battery_config(self, w, r):
        """MSP_SET_BATTERY_CONFIG (33): 13 байт — три u8 (округлённые), ёмкость u16,
        два источника датчиков и точные напряжения u16 в 0.01 В (как в msp.c 4.5.4)."""
        data = bytes(r.rest())
        if len(data) < 13:
            return
        self._store_setting("bat_capacity", int.from_bytes(data[3:5], "little"))
        self._store_setting("battery_meter", data[5])
        self._store_setting("current_meter", data[6])
        self._store_setting("vbat_min_cell_voltage", int.from_bytes(data[7:9], "little"))
        self._store_setting("vbat_max_cell_voltage", int.from_bytes(data[9:11], "little"))
        self._store_setting("vbat_warning_cell_voltage", int.from_bytes(data[11:13], "little"))
        self._log(
            f"[MSP] BATTERY_CONFIG сохранён: {self.settings['vbat_min_cell_voltage']}/"
            f"{self.settings['vbat_max_cell_voltage']}/{self.settings['vbat_warning_cell_voltage']} (0.01 В), "
            f"ёмкость {self.settings['bat_capacity']} мАч, источники {self.settings['battery_meter']}/"
            f"{self.settings['current_meter']}"
        )
        return b""   # пустой ACK: вкладка «Питание и батарея» ждёт ответа на запись

    def _h_set_voltage_meter_config(self, w, r):
        """MSP_SET_VOLTAGE_METER_CONFIG (57): id, vbatscale, vbatresdivval, vbatresdivmultiplier."""
        data = bytes(r.rest())
        if len(data) < 4:
            return
        if data[0] != VOLTAGE_METER_ID_BATTERY_1:
            return
        self._store_setting("vbat_scale", max(0, min(255, data[1])))
        self._store_setting("vbat_divider", max(1, min(255, data[2])))
        self._store_setting("vbat_multiplier", max(1, min(255, data[3])))
        self._log(
            f"[MSP] VOLTAGE_METER_CONFIG: vbat_scale={self.settings['vbat_scale']}, "
            f"divider={self.settings['vbat_divider']}, multiplier={self.settings['vbat_multiplier']}"
        )
        return b""

    def _h_set_current_meter_config(self, w, r):
        """MSP_SET_CURRENT_METER_CONFIG (41): id, scale u16, offset u16 (offset — знаковый)."""
        data = bytes(r.rest())
        if len(data) < 5:
            return
        scale = int.from_bytes(data[1:3], "little", signed=True)
        offset = int.from_bytes(data[3:5], "little", signed=True)
        if data[0] == CURRENT_METER_ID_BATTERY_1:
            self._store_setting("ibata_scale", scale)
            self._store_setting("ibata_offset", offset)
        elif data[0] == CURRENT_METER_ID_VIRTUAL_1:
            self._store_setting("ibatv_scale", scale)
            self._store_setting("ibatv_offset", max(0, offset))
        else:
            return
        self._log(
            f"[MSP] CURRENT_METER_CONFIG (id {data[0]}): scale={scale}, offset={offset}"
        )
        return b""

    def _h_voltage_meter_config(self, w, r):
        """MSP_VOLTAGE_METER_CONFIG (56): count + подкадр [len=5, id, тип, scale, divider, multiplier].

        Раньше длина подкадра была 0: Configurator пропускал такой подкадр и оставлял
        FC.VOLTAGE_METER_CONFIGS пустым — вкладка не показывала калибровку, а «Сохранить»
        вообще ничего не отправлял по напряжению.
        """
        w.u8(VOLTAGE_SENSOR_ADC_COUNT)
        w.u8(VOLTAGE_METER_CONFIG_SUBFRAME)   # 5
        w.u8(VOLTAGE_METER_ID_BATTERY_1)      # 10
        w.u8(VOLTAGE_SENSOR_ADC_RESISTOR_DIVIDER)
        w.u8(int(self.settings.get("vbat_scale", 110)))
        w.u8(int(self.settings.get("vbat_divider", 10)))
        w.u8(int(self.settings.get("vbat_multiplier", 1)))

    def _h_current_meter_config(self, w, r):
        """MSP_CURRENT_METER_CONFIG (40): count + подкадр [len=6, id, тип, scale u16, offset u16]."""
        w.u8(CURRENT_SENSOR_ADC_COUNT)
        w.u8(CURRENT_METER_CONFIG_SUBFRAME)   # 6
        w.u8(CURRENT_METER_ID_BATTERY_1)      # 10
        w.u8(CURRENT_SENSOR_ADC)
        w.i16(int(self.settings.get("ibata_scale", 400)))
        w.i16(int(self.settings.get("ibata_offset", 0)))
        w.u8(CURRENT_METER_CONFIG_SUBFRAME)   # виртуальный датчик (индекс 1 = источник «VIRTUAL»)
        w.u8(CURRENT_METER_ID_VIRTUAL_1)      # 80
        w.u8(CURRENT_SENSOR_VIRTUAL)
        w.i16(int(self.settings.get("ibatv_scale", 0)))
        w.i16(int(self.settings.get("ibatv_offset", 0)))

    def _h_transponder_config(self, w, r):
        w.u8(0).u8(0).u8(0)

    # --- чёрный ящик: накопитель, чтение логов, настройки записи -----------------
    def blackbox_sample_rate(self) -> int:
        """blackboxConfig()->sample_rate: 0…4 → «P interval» = 1 << rate."""
        return max(0, min(4, int(self.settings.get("blackbox_sample_rate", 1) or 0)))

    def blackbox_p_interval(self) -> int:
        """blackboxPInterval из blackboxInit(): период кадров P в циклах PID."""
        return bb_scenarios.log_every_for_sample_rate(self.blackbox_sample_rate())

    def _h_blackbox_config(self, w, r):
        """MSP_BLACKBOX_CONFIG (80): настройки записи — как sbufWrite в msp.c 4.5.4."""
        p_interval = self.blackbox_p_interval() or 1
        w.u8(1)                                        # Blackbox supported
        w.u8(int(self.settings.get("blackbox_device", 1) or 0))
        w.u8(1)                                        # Rate numerator (не используется)
        w.u8(p_interval)                               # blackboxGetRateDenom()
        w.u16(max(1, 32 // p_interval))                # blackboxGetPRatio() = I / P
        w.u8(self.blackbox_sample_rate())              # sample_rate (API 1.44+)
        w.u32(bb_scenarios.mask_from_settings(self.settings))   # fields_disabled_mask (API 1.45+)

    def _h_set_blackbox_config(self, w, r):
        """MSP_SET_BLACKBOX_CONFIG (81): device, rateNum, rateDenom, [pRatio], [sample_rate], [mask].

        Прошивка применяет настройки, только если blackboxMayEditConfig() — у эмулятора
        запись не идёт никогда, поэтому принимаем всегда. Частота записи определяет
        «P interval» в заголовках логов, поэтому при её смене «флеш» пересобирается.
        """
        device = r.u8()
        r.u8()                       # rateNum — устаревшее поле
        rate_denom = r.u8()
        p_ratio = None
        if r.remaining >= 2:
            p_ratio = r.u16()
        sample_rate = None
        if r.remaining >= 1:
            sample_rate = r.u8()
        mask = None
        if r.remaining >= 4:
            mask = r.u32()
        if sample_rate is None:
            # sample_rate в MSP не пришёл — считаем из «P ratio» (blackboxCalculateSampleRate)
            p_ratio = p_ratio or (32 * max(1, rate_denom) // max(1, rate_denom))
            sample_rate = max(0, min(4, (max(1, p_ratio)).bit_length() - 1))
        sample_rate = max(0, min(4, int(sample_rate)))
        old = self.blackbox_sample_rate()
        self.settings["blackbox_device"] = device
        self.settings["blackbox_sample_rate"] = sample_rate
        if mask is not None:
            bb_scenarios.apply_mask_to_settings(self.settings, mask)
        self._log(f"[ЧЯ] настройки записи от вкладки: устройство {device}, "
                  f"частота 1/{1 << sample_rate} (P interval {1 << sample_rate})"
                  + (f", маска полей 0x{mask:08x}" if mask is not None else ""))
        if sample_rate != old:
            # «Флеш» пересобирается под новую частоту записи — в фоне, не держа вкладку
            self.blackbox.ensure_async(self.blackbox_p_interval())

    def _h_dataflash_summary(self, w, r):
        """MSP_DATAFLASH_SUMMARY (70): ready/supported, число секторов, размер и занятое.

        Обработчик ничего не ждёт: если «флеш» собран под другую частоту записи,
        наполнение уходит в фон, а вкладка получает то, что уже лежит в накопителе
        (пустой накопитель — ноль занятых байт, обычный ответ прошивки).
        """
        store = self.blackbox
        store.ensure_async(self.blackbox_p_interval())
        w.u8(store.flags)
        w.u32(store.sectors)
        w.u32(store.size)
        w.u32(store.used)

    def _h_dataflash_read(self, w, r):
        """MSP_DATAFLASH_READ (71): запрос [addr u32, len u16, allowCompression u8] → кусок лога.

        Ответ: [addr u32, readLen u16, compression u8 (0 = без сжатия), data].
        Запрос короче 6 байт — устаревший формат: 128 байт без заголовка сжатия,
        хвост добивается нулями (serializeDataflashReadReply в msp.c).
        """
        store = self.blackbox
        store.ensure_async(self.blackbox_p_interval())
        data_size = r.remaining
        address = r.u32()
        if data_size >= 6:
            request_len = r.u16()
            if r.remaining:
                r.u8()                                     # allowCompression: сжатие не шлём
            legacy = False
        else:
            request_len = 128
            legacy = True
        read_len = min(int(request_len), MSP_DATAFLASH_BUFFER_SIZE)
        chunk = store.read(address, read_len)
        w.u32(address)
        if legacy:
            w.raw(chunk + b"\x00" * (read_len - len(chunk)))
        else:
            w.u16(len(chunk))
            w.u8(0)                                        # NO_COMPRESSION
            w.raw(chunk)

    def _h_dataflash_erase(self, w, r):
        """MSP_DATAFLASH_ERASE (72): blackboxEraseAll() — стираем «флеш» чёрного ящика."""
        self.blackbox.erase()

    def _h_sdcard_summary(self, w, r):
        w.u8(0).u8(0).u32(0).u32(0)

    def _h_rxfail_config(self, w, r):
        """MSP_RXFAIL_CONFIG (77): по 3 байта на канал (mode, value).

        Раньше отдавалось 18 байт подряд, и вкладка Failsafe собирала из них
        6 «каналов» с мусорными значениями вместо 16 настоящих, а у всех каналов
        сразу стоял режим HOLD — в том числе у стиков, где должен быть AUTO.
        """
        for channel in self._rxfail_channels():
            w.u8(channel["mode"])
            w.u16(channel["value"])

    def _rxfail_channels(self) -> list:
        """Список каналов «Этапа 1» ровно длиной RC_CHANNEL_COUNT."""
        channels = [dict(ch) for ch in (self.rxfail or [])]
        channels += self._default_rxfail()[len(channels):]
        return channels[:RC_CHANNEL_COUNT]

    def _rxfail_channels_all(self) -> list:
        """Все каналы «Этапа 1» (MAX_SUPPORTED_RC_CHANNEL_COUNT = 18), как их читает CLI.

        Команда `rxfail` печатает и правит 18 каналов, а MSP-ответ вкладке Failsafe — 16:
        каналы 16 и 17 (AUX13, AUX14) живут во «флеше» и видны только из CLI.
        """
        channels = [dict(ch) for ch in (self.rxfail or [])]
        defaults = self._default_rxfail()
        while len(channels) < MAX_SUPPORTED_RC_CHANNEL_COUNT:
            # каналы 16 и 17 (AUX13/AUX14) — заводские значения AUX-каналов (HOLD, середина)
            channels.append(dict(defaults[-1]))
        return channels[:MAX_SUPPORTED_RC_CHANNEL_COUNT]

    def _h_set_rxfail_config(self, w, r):
        """MSP_SET_RXFAIL_CONFIG (78): index, mode, value — как в msp.c.

        Прошивка переводит значение в «шаг» и обратно, то есть округляет его
        до сетки 25 мкс (CHANNEL_VALUE_TO_RXFAIL_STEP), и отвергает каналы
        с индексом >= MAX_SUPPORTED_RC_CHANNEL_COUNT.
        """
        data = bytes(r.rest())
        if len(data) < 4:
            return b""
        index = data[0]
        if index >= MAX_SUPPORTED_RC_CHANNEL_COUNT:
            return b""
        mode = data[1]
        value = int.from_bytes(data[2:4], "little")
        step = (max(PWM_PULSE_MIN, min(2250, value)) - PWM_PULSE_MIN) // 25
        channel = {"mode": mode, "value": PWM_PULSE_MIN + 25 * step}
        channels = self._rxfail_channels_all()
        channels[index] = channel
        self.rxfail = channels
        self._log(f"[MSP] RXFAIL канал {index + 1}: режим {mode}, значение {channel['value']} мкс")
        return b""   # пустой ACK: вкладка Failsafe ждёт ответа на каждую запись

    def _h_rx_map(self, w, r):
        """MSP_RX_MAP (64): ровно RX_MAPPABLE_CHANNEL_COUNT байт — как sbufWriteData в msp.c.

        Лишние два байта «маски» сбивали разбор: Configurator клал их в FC.RC_MAP
        и подписывал каналы приёмника мусорными буквами.
        """
        letters = {"A": 0, "E": 1, "R": 2, "T": 3, "1": 4, "2": 5, "3": 6, "4": 7}
        for ch in self.rx_map[:RC_MAPPABLE_CHANNEL_COUNT].ljust(RC_MAPPABLE_CHANNEL_COUNT, "1"):
            w.u8(letters.get(ch.upper(), 0))

    def _h_pid_controller(self, w, r):
        w.u8(2)  # PID_CONTROLLER_BETAFLIGHT

    def _h_gps_rescue(self, w, r):
        """MSP_GPS_RESCUE (API 1.46) — 26 байт, ровно как sbufWrite* в msp.c.

        Раскладка: 7 значений u16 (угол, высоты, скорости, газ), 2 байта проверок,
        скорости подъёма/спуска u16, 2 байта режима, дистанция старта u16,
        высота начального набора u16 (добавлена в 1.46).
        """
        w.u16(self._gps_rescue_setting("gps_rescue_max_angle"))       # maxRescueAngle, °
        w.u16(self._gps_rescue_setting("gps_rescue_return_alt"))      # returnAltitudeM
        w.u16(self._gps_rescue_setting("gps_rescue_descent_dist"))    # descentDistanceM
        w.u16(self._gps_rescue_setting("gps_rescue_ground_speed"))    # groundSpeedCmS
        w.u16(self._gps_rescue_setting("gps_rescue_throttle_min"))    # throttleMin
        w.u16(self._gps_rescue_setting("gps_rescue_throttle_max"))    # throttleMax
        w.u16(self._gps_rescue_setting("gps_rescue_throttle_hover"))  # throttleHover
        w.u8(self._gps_rescue_setting("gps_rescue_sanity_checks"))    # sanityChecks
        w.u8(self._gps_rescue_setting("gps_rescue_min_sats"))         # minSats
        w.u16(self._gps_rescue_setting("gps_rescue_ascend_rate"))     # ascendRate, см/с
        w.u16(self._gps_rescue_setting("gps_rescue_descend_rate"))    # descendRate, см/с
        w.u8(self._gps_rescue_setting("gps_rescue_allow_arming_without_fix"))
        w.u8(self._gps_rescue_setting("gps_rescue_alt_mode"))
        w.u16(self._gps_rescue_setting("gps_rescue_min_start_dist"))  # minStartDistM
        w.u16(self._gps_rescue_setting("gps_rescue_initial_climb"))   # initialClimbM (API 1.46+)

    def _gps_rescue_setting(self, name) -> int:
        entry = self.settings_by_name.get(name)
        default = entry["default"] if entry else 0
        return int(self.settings.get(name, default) or 0)

    def _h_set_gps_rescue(self, w, r):
        """MSP_SET_GPS_RESCUE (225): те же поля, что отдаёт чтение (msp.c 4.5.4)."""
        data = bytes(r.rest())
        if len(data) < 16:
            return b""
        pairs = [
            ("gps_rescue_max_angle", int.from_bytes(data[0:2], "little")),
            ("gps_rescue_return_alt", int.from_bytes(data[2:4], "little")),
            ("gps_rescue_descent_dist", int.from_bytes(data[4:6], "little")),
            ("gps_rescue_ground_speed", int.from_bytes(data[6:8], "little")),
            ("gps_rescue_throttle_min", int.from_bytes(data[8:10], "little")),
            ("gps_rescue_throttle_max", int.from_bytes(data[10:12], "little")),
            ("gps_rescue_throttle_hover", int.from_bytes(data[12:14], "little")),
            ("gps_rescue_sanity_checks", data[14]),
            ("gps_rescue_min_sats", data[15]),
        ]
        # Поля добавлялись по мере роста API: 1.43 — скорости и режим, 1.44 — дистанция старта,
        # 1.46 — высота начального набора. Payload короче — просто не трогаем эти настройки.
        if len(data) >= 20:
            pairs += [
                ("gps_rescue_ascend_rate", int.from_bytes(data[16:18], "little")),
                ("gps_rescue_descend_rate", int.from_bytes(data[18:20], "little")),
            ]
        if len(data) >= 22:
            pairs += [
                ("gps_rescue_allow_arming_without_fix", data[20]),
                ("gps_rescue_alt_mode", data[21]),
            ]
        if len(data) >= 24:
            pairs.append(("gps_rescue_min_start_dist", int.from_bytes(data[22:24], "little")))
        if len(data) >= 26:
            pairs.append(("gps_rescue_initial_climb", int.from_bytes(data[24:26], "little")))
        for name, value in pairs:
            self._store_setting(name, value)
        self._log(
            "[MSP] GPS_RESCUE сохранён: возврат "
            f"{self._gps_rescue_setting('gps_rescue_return_alt')} м, газ "
            f"{self._gps_rescue_setting('gps_rescue_throttle_min')}…"
            f"{self._gps_rescue_setting('gps_rescue_throttle_max')}"
        )
        return b""

    def _h_debug(self, w, r):
        for _ in range(4):
            w.i16(0)

    # --- запись
    def _h_ack(self, w, r):
        self.writes_log.append("ack")

    def _h_set_echo(self, w, r):
        """Сохраняем payload записи и отдаём его же в соответствующем GET."""
        return None

    def _h_set_mode_range(self, w, r):
        """MSP_SET_MODE_RANGE (35): index, permanentId, AUX, startStep, endStep [, logic, linkedTo].

        Configurator шлёт семь байт (sendModeRanges), но прошивка принимает и пять —
        logic/linkedTo читаются, только если байты остались (sbufBytesRemaining). Пишем ровно
        в тот же слот массива: пустой слот — это диапазон с startStep = endStep.
        """
        index = r.u8()
        box_id = r.u8()
        aux = r.u8()
        start = CHANNEL_RANGE_MIN + r.u8() * MODE_RANGE_STEP
        end = CHANNEL_RANGE_MIN + r.u8() * MODE_RANGE_STEP
        logic = MODELOGIC_OR
        linked = 0
        if len(r.data) - r.pos >= 1:
            logic = r.u8()
        if len(r.data) - r.pos >= 1:
            linked = r.u8()
        if not self.set_mode_range(index, box_id, aux, start, end, logic, linked):
            self.writes_log.append(f"MSP_SET_MODE_RANGE: слот {index} не изменён "
                                   f"(режим {box_id}, AUX {aux})")

    def _h_common_serial_config(self, w, r):
        """MSP2_COMMON_SERIAL_CONFIG: count + (identifier u8, functions u32, 4 скорости u8).

        Именно так порты читает вкладка «Порты» в Configurator 2026.x
        (composables/ports/useFeaturePort.js -> MSP.promise(MSP2_COMMON_SERIAL_CONFIG)).
        """
        w.u8(len(self.serial_ports))
        for p in self.serial_ports:
            w.u8(p["identifier"])
            w.u32(self._functions_to_mask(p["functions"]))
            w.u8(p.get("msp_baudrate", 5)).u8(p.get("gps_baudrate", 4))
            w.u8(p.get("telemetry_baudrate", 0)).u8(p.get("blackbox_baudrate", 5))

    def _h_set_serial_config(self, w, r):
        """MSP_SET_CF_SERIAL_CONFIG (v1): 7 байт на порт (identifier u8, mask u16, 4 скорости)."""
        ports = []
        while r.pos + 7 <= len(r.data):
            ident = r.u8()
            mask = r.u16()
            ports.append({
                "identifier": ident,
                "functions": self._mask_to_functions(mask),
                "msp_baudrate": r.u8(), "gps_baudrate": r.u8(),
                "telemetry_baudrate": r.u8(), "blackbox_baudrate": r.u8(),
            })
        if ports:
            self.serial_ports = ports

    def _h_set_common_serial_config(self, w, r):
        """MSP2_COMMON_SET_SERIAL_CONFIG: count + 9 байт на порт (identifier u8, mask u32, 4 скорости)."""
        count = r.u8()
        ports = []
        for _ in range(count):
            ident = r.u8()
            mask = r.u32()
            ports.append({
                "identifier": ident,
                "functions": self._mask_to_functions(mask),
                "msp_baudrate": r.u8(), "gps_baudrate": r.u8(),
                "telemetry_baudrate": r.u8(), "blackbox_baudrate": r.u8(),
            })
        if ports:
            self.serial_ports = ports

    def _h_set_arming_disabled(self, w, r):
        """MSP_SET_ARMING_DISABLED (99): msp.c читает u8 «команда» и необязательный u8
        «разрешить обход проверки на взлёт». Команда 1 — запретить взвод (ставим
        ARMING_DISABLED_MSP и, если борт уже взведён, расснаряжаем), 0 — разрешить снова.
        Именно это вкладка «Моторы» шлёт при включении и выключении теста.
        """
        data = bytes(r.rest())
        command = data[0] if data else 0
        if command:
            self.motor_test_arming_block = True
            self._log("[MSP] тест моторов: взвод запрещён (ARMING_DISABLED_MSP)")
        else:
            self.motor_test_arming_block = False
            self._log("[MSP] тест моторов выключен: взвод снова разрешён")
        self.writes_log.append(f"arming disabled set ({'on' if command else 'off'})")
        return b""

    def _h_motor_3d_config(self, w, r):
        """MSP_MOTOR_3D_CONFIG (124): 6 байт — deadband3d_low/high и neutral3d
        (msp.c: flight3DConfig(); заводские значения из PG_MOTOR_3D_CONFIG)."""
        w.u16(self._setting("3d_deadband_low", 1406))
        w.u16(self._setting("3d_deadband_high", 1514))
        w.u16(self._setting("3d_neutral", 1460))

    def _h_set_motor_3d_config(self, w, r):
        """MSP_SET_MOTOR_3D_CONFIG (217): пишем те же три настройки в «флеш»."""
        data = bytes(r.rest())
        if len(data) >= 6:
            self._store_setting("3d_deadband_low", int.from_bytes(data[0:2], "little"))
            self._store_setting("3d_deadband_high", int.from_bytes(data[2:4], "little"))
            self._store_setting("3d_neutral", int.from_bytes(data[4:6], "little"))
            self._log("[MSP] MOTOR_3D_CONFIG сохранён: deadband "
                      f"{self.settings.get('3d_deadband_low')}/{self.settings.get('3d_deadband_high')}, "
                      f"neutral {self.settings.get('3d_neutral')}")
        return b""

    def _h_motor_output_reordering(self, w, r):
        """MSP2_MOTOR_OUTPUT_REORDERING (0x3001): u8 count + по u8 на мотор.

        msp.c пишет MAX_SUPPORTED_MOTORS (8) и массив motorOutputReordering[],
        заводское значение — 0,1,2,…,7 (pg/motor.c: reordering[i] = i).
        """
        w.u8(MAX_SUPPORTED_MOTORS)
        for i in range(MAX_SUPPORTED_MOTORS):
            w.u8(self.motor_output_reordering[i] & 0xFF)

    def _h_set_motor_output_reordering(self, w, r):
        """MSP2_SET_MOTOR_OUTPUT_REORDERING (0x3002): сохраняем порядок моторов."""
        data = bytes(r.rest())
        if len(data) >= 1 + MAX_SUPPORTED_MOTORS:
            self.motor_output_reordering = list(data[1:1 + MAX_SUPPORTED_MOTORS])
            self._log(f"[MSP] порядок выходов моторов сохранён: {self.motor_output_reordering}")
        return b""

    def _h_send_dshot_command(self, w, r):
        """MSP2_SEND_DSHOT_COMMAND (0x3003): команда ESC. Вкладка «Моторы» шлёт так
        «включить расширенную телеметрию» (команда 13) при входе в тест и «стоп»
        (команда 12) при выходе. Прошивка отвечает пустым ответом — логируем для монитора.
        """
        data = bytes(r.rest())
        if len(data) >= 3:
            command_type, motor, count = data[0], data[1], data[2]
            commands = list(data[3:3 + count])
            self.dshot_commands_log.append({
                "type": command_type, "motor": motor, "commands": commands,
                "time": time.strftime("%H:%M:%S"),
            })
            self.dshot_commands_log = self.dshot_commands_log[-10:]
            self._log(f"[MSP] DSHOT-команды ESC (мотор {motor}): {commands}")
        return b""

    def _h_set_motor(self, w, r):
        """MSP_SET_MOTOR (214) — тест моторов со слайдеров вкладки «Моторы».

        msp.c 4.5.4: `motor_disarmed[i] = motorConvertFromExternal(...)` для getMotorCount()
        моторов, лишние байты игнорируются. Значения НЕ сбрасываются по таймауту: на железной
        плате мотор продолжает крутиться на выставленном газе, пока Configurator не пришлёт
        нули (кнопка «Стоп» / отпущенный ползунок мастера кладёт 1000). Останавливает их
        только MSP_SET_MOTOR с mincommand, выключение теста или перезагрузка платы.
        """
        count = self._motor_count()
        data = bytes(r.rest())
        values = []
        for i in range(count):
            offset = i * 2
            if offset + 2 > len(data):
                break
            values.append(max(1000, min(2000, int.from_bytes(data[offset:offset + 2], "little"))))
        if self.sim_flight:
            # Идёт виртуальный полёт: газ держит сценарий (как mixer.c на взведённом борде),
            # поэтому слайдеры вкладки «Моторы» сейчас ничего не меняют — иначе моторы
            # «дёргались» бы между сценарием и ползунком.
            self.writes_log.append(f"motors: {values} (игнор: виртуальный полёт)")
            return b""
        if values:
            for i, value in enumerate(values):
                self.motor_disarmed[i] = float(value)
        self.writes_log.append(f"motors: {values}")
        return b""

    def _h_eeprom_write(self, w, r):
        self.save_to_eeprom()

    def _h_reset_conf(self, w, r):
        """MSP_RESET_CONF.

        В прошивке это resetEEPROM() = resetConfig() + writeUnmodifiedConfigToEEPROM()
        (config.c), то есть сброс к заводским НАСТРОЙКАМ С ЗАПИСЬЮ ВО ФЛЕШ, затем ребут.
        """
        self._defaults_restore()
        self.save_to_eeprom()
        self.reboot()

    def _h_reboot(self, w, r):
        """MSP_SET_REBOOT (68): перезагрузка с указанием режима — как handleMspReboot в msp.c.

        Ответ прошивки: [режим] и, для режима запоминающего устройства, ещё байт готовности
        (1 — «флеш готов», 0 — «не готово»; тогда Configurator показывает ошибку).
        """
        mode = r.u8() if r.remaining else MSP_REBOOT_FIRMWARE
        if mode >= MSP_REBOOT_COUNT:
            self._log(f"[FC] MSP_SET_REBOOT: режим {mode} не поддерживается")
            return
        if mode in (MSP_REBOOT_MSC, MSP_REBOOT_MSC_UTC):
            busy = self.blackbox.store_ready()
            w.u8(mode).u8(1 if self.blackbox.used or busy else 0)
            if not self.blackbox.used and not busy:
                self._log("[MSC] «флеш» чёрного ящика ещё пуст — Configurator покажет «не готово»")
                return
            self.enter_msc(mode)
            return
        w.u8(mode)
        self.reboot(bootloader=mode in (MSP_REBOOT_BOOTLOADER_ROM, MSP_REBOOT_BOOTLOADER_FLASH))

    def _h_set_text(self, w, r):
        text_type = r.u8()
        length = r.u8()  # формат запроса: type, длина, символы
        raw = r.data[r.pos : r.pos + length]
        text = raw.decode("latin-1", "replace")
        if text_type == TEXT_CRAFT_NAME:
            text = text[:MAX_NAME_LENGTH]
            self._mirror_name_setting("craft_name", text)
            self.settings["craft_name"] = text
        elif text_type == TEXT_PILOT_NAME:
            text = text[:MAX_NAME_LENGTH]
            self._mirror_name_setting("pilot_name", text)
            self.settings["pilot_name"] = text
        elif text_type == TEXT_PID_PROFILE_NAME:
            self.pid_profile_names[self.current_profile] = text[:16]   # MAX_PROFILE_NAME_LENGTH
        elif text_type == TEXT_RATE_PROFILE_NAME:
            self.rate_profile_names[self.current_rate_profile] = text[:16]
        elif text_type == TEXT_BATTERY_PROFILE_NAME:
            # мастера: textSpace = MAX_BATTERY_PROFILE_NAME_LENGTH (8) у активного профиля
            self.battery_profiles[self.active_battery_profile]["name"] = \
                text[:MAX_BATTERY_PROFILE_NAME_LENGTH]

    def _h_osd_warnings(self, w, r):
        pass

    def _h_mcu_info(self, w, r):
        w.u8(self.mcu_type_id).u16(8192).u32(0).u32(0)

    def _h_gyro_sensor_active(self, w, r):
        w.u8(0)

    def _h_battery_profile(self, w, r):
        """MSP2_BATTERY_PROFILE (0x300E): раскладка мастера BF — 1 + 5·2 + 2 = 13 байт.

        Без аргумента отдаётся активный профиль (msp.c: `sbufBytesRemaining(src) ?
        sbufReadU8(src) : systemConfig()->activeBatteryProfile`), индекс вне диапазона —
        ошибка: отвечаем пустым payload, как прошивка на MSP_RESULT_ERROR.
        """
        index = r.u8() if len(r.data) else self.active_battery_profile
        if index >= BATTERY_PROFILE_COUNT:
            return b""
        profile = self.battery_profiles[index]
        w.u8(index)
        w.u16(profile["vbatmincellvoltage"]).u16(profile["vbatmaxcellvoltage"])
        w.u16(profile["vbatwarningcellvoltage"]).u16(profile["vbatfullcellvoltage"])
        w.u16(profile["batteryCapacity"])
        w.u8(profile["forceBatteryCellCount"]).u8(profile["consumptionWarningPercentage"])

    def _h_set_battery_profile(self, w, r):
        """MSP2_SET_BATTERY_PROFILE (0x300F): запись профиля — проверки как в msp.c мастера.

        Прошивка требует минимум 13 байт, проверяет порядок порогов (min ≤ warning ≤ full ≤
        max), число банок ≤ 24 и процент ≤ 100 — иначе MSP_RESULT_ERROR (у эмулятора —
        пустой ответ без изменения таблицы).
        """
        data = bytes(r.rest())
        if len(data) < 13:
            return b""
        index = data[0]
        if index >= BATTERY_PROFILE_COUNT:
            return b""
        values = [int.from_bytes(data[1 + 2 * i:3 + 2 * i], "little") for i in range(5)]
        cells, percent = data[11], data[12]
        vbat_min, vbat_max, vbat_warn, vbat_full, capacity = values
        if not (vbat_min <= vbat_warn <= vbat_full <= vbat_max):
            return b""
        if cells > 24 or percent > 100:
            return b""
        self.battery_profiles[index] = {
            "vbatmincellvoltage": vbat_min, "vbatmaxcellvoltage": vbat_max,
            "vbatwarningcellvoltage": vbat_warn, "vbatfullcellvoltage": vbat_full,
            "batteryCapacity": capacity, "forceBatteryCellCount": cells,
            "consumptionWarningPercentage": percent,
            "name": self.battery_profiles[index].get("name", ""),
        }
        self._log(f"[MSP] профиль батареи {index + 1}: {vbat_min}…{vbat_max} мВ/ячейка, {capacity} мА·ч")
        return b""

    # --- CLI поверх MSP
    def _h_cli_setting(self, w, r):
        request = r.rest().decode("latin-1", "replace").split("\x00")[0].strip()
        if not request:
            return
        name, _, value = request.partition("=")
        name = name.strip()
        if value.strip():
            ok, _err = self.set_setting(name, value.strip())
            if not ok:
                # прошивка возвращает пустой ответ => Configurator считает, что FC отказал
                return
        entry = self.settings_by_name.get(name)
        if entry is None:
            return
        w.raw(f"{name} = {self.format_setting(entry)}".encode())

    def _h_cli_setting_info(self, w, r):
        raw = r.data
        if b"\x00" in raw:
            name, rest = raw.split(b"\x00", 1)
            offset = int.from_bytes(rest[:2], "little") if len(rest) >= 2 else 0
        else:
            name, offset = raw, 0
        entry = self.settings_by_name.get(name.decode("latin-1", "replace").strip())
        if entry is None:
            return
        lines = [f"pgn={entry['pg']}", f"type={entry['type']}"]
        if entry["mode"] == "MODE_LOOKUP" and entry.get("values"):
            lines.append("values=" + ",".join(entry["values"]))
        else:
            lines.append(f"min={entry['min']}")
            lines.append(f"max={entry['max']}")
        block = ("\n".join(lines)).encode()
        window = block[offset : offset + 480]
        w.u16(len(block)).raw(window)

    def _h_cli_command(self, w, r):
        raw = r.data
        if b"\x00" in raw:
            cmdline, rest = raw.split(b"\x00", 1)
            offset = int.from_bytes(rest[:2], "little")
        else:
            cmdline, offset = raw, 0
        line = cmdline.decode("latin-1", "replace")
        if offset == 0:
            self._cli_out = self.cli_execute(line)
        out = getattr(self, "_cli_out", "")
        block = out.encode("latin-1", "replace")
        window = block[offset : offset + 480]
        w.u16(len(block)).u8(0).raw(window)

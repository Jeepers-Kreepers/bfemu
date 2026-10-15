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

from pathlib import Path

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

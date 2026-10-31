"""Сквозной сценарий «Значения по умолчанию» (по скриншотам пользователя):

  1. диалог пресетов → «Mini Quad with 7" propellers» (настройки + mixer 3),
  2. мастер Receiver (порт + SPEK1024),
  3. мастер GPS (порт + UBLOX + pitot VIRTUAL),
  4. saveToEeprom + MSP_SET_REBOOT (модалка «Перезагружается»),
  5. applied_defaults = 5 — при реконнекте диалог не показывается.

Каждый обмен повторяет то, что реально шлёт inav-configurator 9.1.1
(defaults_dialog.js → chain, wizard_save_framework.js, mspHelper).
"""
import struct

import pytest

from inavemu import msp
from inavemu.fc import (
    MSP2_CF_SERIAL_CONFIG, MSP2_COMMON_MOTOR_MIXER, MSP2_COMMON_SET_MOTOR_MIXER,
    MSP2_COMMON_SET_SETTING, MSP2_COMMON_SETTING, MSP2_COMMON_SETTING_INFO,
    MSP2_INAV_MIXER, MSP2_INAV_SELECT_BATTERY_PROFILE, MSP2_INAV_SET_MIXER, MSP2_INAV_SET_SERVO_MIXER,
    MSP2_SET_CF_SERIAL_CONFIG, MSP_EEPROM_WRITE, MSP_FEATURE, MSP_SELECT_SETTING,
    MSP_SET_FEATURE, MSP_SET_REBOOT, MSPError, InavFC,
)
from inavemu.settings import TYPE_PACK


@pytest.fixture()
def fc():
    return InavFC()


def ask(fc, code, payload=b""):
    return fc.handle(msp.Request(version=2, code=code, payload=payload))


def js_encode_setting(fc, name, value):
    """Кодировка значения как у Configurator (encodeSetting): lookup → индекс."""
    s = fc.store.by_name[name]
    typ = s["type"]
    if "values" in s and isinstance(value, str):
        raw = s["values"].index(value)
    else:
        raw = value
    if typ == "string":
        size = s.get("max", 0)
        blob = str(raw).encode()[:size]
        return blob + b"\x00" * (size - len(blob))
    if typ == "float":
        return struct.pack(TYPE_PACK[typ], float(raw))
    return struct.pack(TYPE_PACK[typ], int(raw))


def js_set_setting(fc, name, value):
    """MSPHelper.setSetting: SETTING_INFO + SET_SETTING (ref 0x00+u16, без
    байта ширины — как _encodeSettingReference + encodeSetting в Configurator)."""
    assert ask(fc, MSP2_COMMON_SETTING_INFO, name.encode() + b"\x00"), name
    ref = bytes([0]) + struct.pack("<H", fc.store.by_name[name]["index"])
    ask(fc, MSP2_COMMON_SET_SETTING, ref + js_encode_setting(fc, name, value))


def js_get_setting(fc, name):
    return ask(fc, MSP2_COMMON_SETTING, name.encode() + b"\x00")


# Пресет "Mini Quad with 7" propellers" (id=5): defaults_dialog_entries.js:256
PRESET_MISC = [  # не per-profile — пишутся один раз
    ("model_preview_type", 3),
    ("gyro_hardware_lpf", "256HZ"),   # фантомное имя — ошибку пропускаем
    ("motor_pwm_protocol", "DSHOT300"),
    ("ez_enabled", "ON"),
    ("ez_filter_hz", 90),
    ("ez_axis_ratio", 110),
    ("ez_response", 101),
    ("ez_damping", 115),
    ("ez_stability", 100),
    ("ez_aggressiveness", 100),
    ("ez_rate", 134),
    ("ez_expo", 118),
    ("airmode_type", "THROTTLE_THRESHOLD"),
    ("airmode_throttle_threshold", 1150),
    ("platform_type", "MULTIROTOR"),
    ("applied_defaults", 5),
    ("failsafe_procedure", "DROP"),
]
PRESET_CONTROL = [  # getControlProfileParameters() — пишутся в 3 профиля
    ("mc_iterm_relax", "RPY"),
    ("d_boost_min", 0.8),
    ("d_boost_max", 1.2),
    ("antigravity_gain", 2),
    ("antigravity_accelerator", 5),
    ("tpa_rate", 20),
    ("tpa_breakpoint", 1200),
]
PHANTOM = {"gyro_hardware_lpf"}  # есть и в прошивке? нет — падает везде


def test_preset_mini_quad_7_full_flow(fc):
    # ── подключение: defaults_dialog.init читает applied_defaults ──
    assert js_get_setting(fc, "applied_defaults") == b"\x00"

    # ── onPresetClick: loadFeatures → setSettings ──
    features = ask(fc, MSP_FEATURE)
    assert len(features) == 4  # битовая маска 32 бита
    gps_bit = next(f["bit"] for f in fc.store.features if f["name"] == "GPS")

    errors = []
    for name, value in PRESET_MISC:
        try:
            js_set_setting(fc, name, value)
        except MSPError as exc:
            errors.append(name)
            assert "нет такой настройки" in str(exc), name
    assert errors == ["gyro_hardware_lpf"], "фантом — единственная ошибка"

    for profile in range(3):  # как chain: SELECT_SETTING + control ×3
        ask(fc, MSP_SELECT_SETTING, bytes([profile]))
        for name, value in PRESET_CONTROL:
            js_set_setting(fc, name, value)
        ask(fc, MSP2_INAV_SELECT_BATTERY_PROFILE, bytes([profile]))
    ask(fc, MSP_SELECT_SETTING, bytes([0]))   # финал: current profile
    ask(fc, MSP2_INAV_SELECT_BATTERY_PROFILE, bytes([0]))

    # ── mixer: saveMixerConfig + sendServoMixer + sendMotorMixer ──
    mixer = ask(fc, MSP2_INAV_MIXER)
    assert mixer[7] == 12 and mixer[8] == 18, "ёмкости 12/18 (как в прошивке)"
    # SET_MIXER: mixerToApply=3 (Mini Quad), платформа MULTIROTOR и т.п.
    ask(fc, MSP2_INAV_SET_MIXER, bytes([3, 0, 1, 0, 0, 0, 0, 0, 0]))
    for i in range(36):  # sendServoMixer: всегда MAX_SERVO_RULES
        ask(fc, MSP2_INAV_SET_SERVO_MIXER,
            bytes([i, 3, 1]) + struct.pack("<h", 100 if i == 0 else 0) +
            bytes([0, 0]))
    for i in range(12):  # sendMotorMixer: всегда MAX_MOTORS
        ask(fc, MSP2_COMMON_SET_MOTOR_MIXER,
            bytes([i]) + struct.pack("<hhhh", 1000, 0, 0, 0))
    assert ask(fc, MSP2_COMMON_MOTOR_MIXER)[:8] == \
        struct.pack("<HHHH", 1000, 0, 0, 0), "motor mixer сохранился"

    # значения пресета применились (кроме фантома)
    assert fc.store.get("model_preview_type") == 3
    assert fc.store.get("motor_pwm_protocol") == \
        fc.store.by_name["motor_pwm_protocol"]["values"].index("DSHOT300")
    assert fc.store.get("ez_enabled") == 1
    assert fc.store.get("ez_rate") == 134
    assert fc.store.get("airmode_throttle_threshold") == 1150
    assert fc.store.get("platform_type") == 0
    assert abs(fc.store.get("d_boost_min") - 0.8) < 1e-6, "float сохранился"
    assert abs(fc.store.get("d_boost_max") - 1.2) < 1e-6
    assert fc.store.get("tpa_rate") == 20
    assert fc.store.get("tpa_breakpoint") == 1200
    assert fc.store.get("failsafe_procedure") == \
        fc.store.by_name["failsafe_procedure"]["values"].index("DROP")
    assert "gyro_hardware_lpf" not in fc.store.values

    # ── мастер: Receiver → configureInputs (data-setting serialrx_provider) ──
    js_set_setting(fc, "serialrx_provider", "SPEK1024")
    # wizardSaveFramework: receiverPort → serialPortHelper.set + saveSerialPorts
    # crunch шлёт ВСЕ порты (как FC.SERIAL_CONFIG.ports после загрузки)
    raw = ask(fc, MSP2_CF_SERIAL_CONFIG)
    ports = [bytearray(raw[i:i + 9]) for i in range(0, len(raw), 9)]
    for rec in ports:
        rec[1:5] = struct.pack("<I", 0)          # сброс функций
    next(r for r in ports if r[0] == 1)[1:5] = struct.pack("<I", 1 << 6)  # UART2=RX_SERIAL
    ask(fc, MSP2_SET_CF_SERIAL_CONFIG, b"".join(bytes(r) for r in ports))
    assert len(ask(fc, MSP2_CF_SERIAL_CONFIG)) == 11 * 9

    # ── мастер: GPS → gpsPort (features GPS + serial + pitot) + gpsProtocol ──
    feat_mask = struct.unpack("<I", features)[0] | (1 << gps_bit)
    next(r for r in ports if r[0] == 2)[1:5] = struct.pack("<I", 1 << 1)  # UART3=GPS
    ask(fc, MSP2_SET_CF_SERIAL_CONFIG, b"".join(bytes(r) for r in ports))
    ask(fc, MSP_SET_FEATURE, struct.pack("<I", feat_mask))
    js_set_setting(fc, "pitot_hardware", "VIRTUAL")
    js_set_setting(fc, "gps_provider", "UBLOX")

    # ── сохранение и перезагрузка (модалка «Перезагружается») ──
    assert ask(fc, MSP_EEPROM_WRITE) == b""
    assert ask(fc, MSP_SET_REBOOT) == b""
    assert fc.reboot_requested, "сервер должен оборвать линк (handleReconnect)"
    assert fc.store.get("applied_defaults") == 5, \
        "при реконнекте диалог «Значения по умолчанию» не показывается"

    # ── после «перезагрузки» всё на месте ──
    fc.reboot_requested = False
    assert js_get_setting(fc, "serialrx_provider") == b"\x00"  # SPEK1024 = 0
    assert js_get_setting(fc, "gps_provider") == b"\x00"       # UBLOX = 0
    assert js_get_setting(fc, "pitot_hardware") == b"\x04"     # VIRTUAL = 4
    assert js_get_setting(fc, "applied_defaults") == b"\x05"


def test_wizard_settings_exist(fc):
    """Имена мастера обязаны быть в базе — configureInputs читает их."""
    for name in ("serialrx_provider", "gps_provider", "pitot_hardware"):
        assert name in fc.store.by_name
        assert js_get_setting(fc, name) is not None

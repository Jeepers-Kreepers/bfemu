"""«Флеш» (EEPROM) и сброс `defaults` — проверки по находкам повторного ревью.

Закрепляется три дефекта, которые ловились только руками:

1. имя профиля вида «BEEF» превращалось после перезапуска в байты `b"\\xbe\\xef"`
   и ломало чтение MSP (MSP2_GET_TEXT отдавал пустой ответ);
2. `defaults` сбрасывал не всю конфигурацию: фичи, пищалка, порты, микшер,
   слоты `vtx` и «эхо» записей вкладок переживали сброс;
3. валидный JSON с неверной структурой (список вместо объекта, строка вместо
   списка) ронял запуск — эмулятор не поднимался вовсе.
"""
from __future__ import annotations

import json

import pytest

from bfemu.fc import VirtualFC
from bfemu.fc_constants import MSP2_GET_TEXT, MSP2_SET_TEXT, TEXT_PID_PROFILE_NAME
from bfemu.msp import Reader

SETTINGS_COUNT = 683                     # столько настроек в таблице 4.5.4
FACTORY_UIDS = (0x1A2B3C4D, 0x5E6F7081, 0x92A3B4C5)


def open_fc(eeprom) -> VirtualFC:
    """Эмулятор с указанным «флешем»; журнал глушим — тест проверяет состояние."""
    return VirtualFC(eeprom_path=eeprom, log=lambda *a, **k: None)


def test_hex_like_profile_name_survives_restart(tmp_path):
    """«BEEF» — это имя, а не hex-строка: раньше оно приезжало из «флеша» байтами."""
    eeprom = tmp_path / "eeprom.json"
    fc = open_fc(eeprom)
    fc.handle(MSP2_SET_TEXT, bytes([TEXT_PID_PROFILE_NAME, 4]) + b"BEEF", 2)
    assert fc.pid_profile_names[0] == "BEEF"
    fc.save_to_eeprom()
    assert json.loads(eeprom.read_text())["pid_profile_names"]["0"] == "BEEF"

    restarted = open_fc(eeprom)
    assert restarted.pid_profile_names[0] == "BEEF", "имя профиля должно остаться строкой"
    payload = restarted.handle(MSP2_GET_TEXT, bytes([TEXT_PID_PROFILE_NAME]), 2)
    reader = Reader(payload)
    assert reader.u8() == TEXT_PID_PROFILE_NAME
    length = reader.u8()                     # msp.c: sbufWriteU8(strlen) + символы
    assert payload[reader.pos : reader.pos + length] == b"BEEF", "MSP2_GET_TEXT отдаёт имя профиля"


def test_hex_payloads_still_round_trip(tmp_path):
    """Обратная сторона той же правки: там, где лежат байты, hex по-прежнему разбирается."""
    eeprom = tmp_path / "eeprom.json"
    fc = open_fc(eeprom)
    fc.config_payloads[0x1234] = b"\x01\x02\x03"
    fc.save_to_eeprom()
    assert json.loads(eeprom.read_text())["config_payloads"] == {"4660": "010203"}

    restarted = open_fc(eeprom)
    assert restarted.config_payloads == {0x1234: b"\x01\x02\x03"}
    assert restarted.handle(0x1234, b"", 2) == b"\x01\x02\x03"


def test_defaults_resets_the_whole_configuration(fc, tmp_path):
    """`defaults` = resetConfig() прошивки: сбрасывается вся PG-конфигурация, а не только настройки."""
    fc.cli_execute("feature -GPS")
    fc.cli_execute("feature SOFTSERIAL")
    fc.cli_execute("beeper -USB")
    fc.cli_execute("serial 4 1 115200 57600 0 115200")     # UART5: функция MSP
    fc.cli_execute("mixer TRI")
    fc.cli_execute("vtx 0 3 0 0 1 900 1200")
    fc.config_payloads[0x1234] = b"\x01"
    fc.sensor_config["baro_hardware"] = 1
    changed = {
        "features": sorted(fc.features),
        "beepers": sorted(fc.beepers_disabled),
        "ports": [port["functions"] for port in fc.serial_ports],
        "mixer": fc.mixer_name,
        "vtx": fc.vtx_activation[0],
        "profile_name": fc.pid_profile_names[0],
    }
    assert changed["features"] != sorted(open_fc(tmp_path / "factory.json").features)

    fc.cli_execute("defaults nosave")
    factory = open_fc(tmp_path / "factory.json")

    assert sorted(fc.features) == sorted(factory.features), "фичи возвращаются к заводским"
    assert fc.beepers_disabled == factory.beepers_disabled, "выключенные сигналы пищалки сбрасываются"
    assert [port["functions"] for port in fc.serial_ports] == \
        [port["functions"] for port in factory.serial_ports], "функции на портах сбрасываются"
    assert [port["identifier"] for port in fc.serial_ports] == \
        [port["identifier"] for port in factory.serial_ports], "порты SOFTSERIAL1/2 уходят вместе с фичей"
    assert fc.mixer_name == factory.mixer_name == "QUADX", "микшер — заводской QUADX"
    assert fc.vtx_activation == factory.vtx_activation, "слоты `vtx` обнуляются"
    assert fc.sensor_config == factory.sensor_config, "выбор датчиков возвращается к AUTO/NONE"
    assert fc.config_payloads == {}, "«эхо» записей вкладок больше не отдаётся на чтение"


def test_defaults_nosave_keeps_working_over_cli(fc):
    """`defaults nosave` (вкладка «Режимы» и CLI) не перезагружает FC и сбрасывает состояние."""
    fc.cli_execute("feature SOFTSERIAL")
    fc.cli_execute("defaults nosave")
    assert "SOFTSERIAL" not in fc.features
    assert fc.pending_reboot is False, "без `save` перезагрузки нет"


def test_reset_conf_via_msp_also_resets_everything(fc, tmp_path):
    """Кнопка «Сбросить настройки» Configurator (MSP_RESET_CONF = 0x0A) идёт тем же путём."""
    from bfemu.fc_constants import MSP_RESET_CONF

    fc.cli_execute("feature SOFTSERIAL")
    fc.cli_execute("mixer Y6")
    fc.config_payloads[0x1234] = b"\x01"
    fc.handle(MSP_RESET_CONF, b"", 1)
    assert "SOFTSERIAL" not in fc.features
    assert fc.mixer_name == "QUADX"
    assert fc.config_payloads == {}
    assert fc.pending_reboot is True, "MSP_RESET_CONF перезагружает FC, как прошивка"


@pytest.mark.parametrize(
    "content",
    [
        [1, 2, 3],
        "строка вместо объекта",
        {"version": 1, "settings": "нет"},
        {"version": 1, "features": 5},
        {"version": 1, "serial_ports": [1, 2]},
        {"version": 1, "mode_names": 7},
        {"version": 1, "pid_profile_names": 7},
        {"version": 1, "uids": "abc"},
        {"version": 1, "mode_ranges": "нет"},
        {"version": 1, "config_payloads": {"4660": "не hex"}},
        {"version": 1, "vtx_activation": "нет"},
        {"version": 1, "signature": 12345, "dshot_beacon_tone": "громко"},
    ],
    ids=lambda value: type(value).__name__ if not isinstance(value, dict)
    else ",".join(list(value)[1:]) or "объект",
)
def test_wrong_structure_does_not_break_startup(tmp_path, content):
    """Чужой «флеш» — не повод не запуститься: разделы разбираются поштучно, битое — заводское."""
    eeprom = tmp_path / "eeprom.json"
    eeprom.write_text(json.dumps(content, ensure_ascii=False), encoding="utf-8")
    fc = open_fc(eeprom)                     # главное: не бросает

    assert len(fc.settings_list) == SETTINGS_COUNT
    assert fc.uids == FACTORY_UIDS, "UID платы не портится чужими данными"
    assert fc.rx_map == "AETR1234"
    assert fc.mixer_name == "QUADX"
    assert fc.uid_hex and len(fc.uid_hex) == 24
    assert len(fc.serial_ports) == 6, "таблица портов остаётся рабочей"


def test_broken_section_does_not_block_the_good_ones(tmp_path):
    """Валидная часть «флеша» применяется, битая — отбрасывается (а не роняет запуск)."""
    eeprom = tmp_path / "eeprom.json"
    eeprom.write_text(json.dumps({
        "version": 1,
        "settings": {"craft_name": "ЧАСТИЧНО"},
        "features": ["GPS", "LED_STRIP", 5, None],
        "uids": "abc",
        "config_payloads": {"4660": "aabb", "не код": "cc"},
    }), encoding="utf-8")
    fc = open_fc(eeprom)

    assert fc.craft_name == "ЧАСТИЧНО", "настройки из валидной части читаются"
    assert {"GPS", "LED_STRIP"} <= fc.features, "фичи читаются"
    assert all(isinstance(name, str) for name in fc.features), "мусор в списке фич отброшен"
    assert fc.uids == FACTORY_UIDS
    assert fc.config_payloads == {0x1234: b"\xaa\xbb"}, "код-не-число и «не hex» отброшены"

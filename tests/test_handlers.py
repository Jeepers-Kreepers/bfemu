"""Карта MSP-обработчиков и формат ключевых ответов.

Проверяется то, на чём Configurator спотыкается молча: размеры ответов, порядок бит
и отсутствие «затёртых» обработчиков (повтор ключа в словаре).
"""
from __future__ import annotations

import ast
import struct

from bfemu.fc import (MSP2_SEND_DSHOT_COMMAND, MSP2_SET_MOTOR_OUTPUT_REORDERING, MSP_ANALOG,
                      MSP_BOARD_INFO, MSP_SET_RC_TUNING, MSP_STATUS, MSP_STATUS_EX, MSP_VTX_CONFIG)


def test_handler_map_has_no_repeated_keys(repo_root):
    """Повтор ключа в литерале словаря молча затирает первый обработчик.

    Так уже случалось: `MSP_SET_RC_TUNING` стоял дважды, и первой записью был `_h_set_echo`
    (вкладка «PID настройки» сохраняла бы рейты в никуда, поменяйся порядок строк).
    """
    tree = ast.parse((repo_root / "bfemu" / "fc.py").read_text(encoding="utf-8"))
    repeated = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "_handlers":
            for dict_node in ast.walk(node):
                if isinstance(dict_node, ast.Dict):
                    keys = [ast.unparse(k) for k in dict_node.keys]
                    repeated = {k: keys.count(k) for k in set(keys) if keys.count(k) > 1}
                    break
    assert repeated == {}, f"повторяющиеся ключи в карте обработчиков: {repeated}"


def test_all_handlers_are_callable(fc):
    handlers = fc._handlers()
    assert len(handlers) > 140
    assert all(callable(h) for h in handlers.values())
    assert handlers[MSP_SET_RC_TUNING].__name__ == "_h_set_rc_tuning"
    assert handlers[MSP2_SEND_DSHOT_COMMAND].__name__ == "_h_send_dshot_command"
    assert handlers[MSP2_SET_MOTOR_OUTPUT_REORDERING].__name__ == "_h_set_motor_output_reordering"


def test_unknown_code_answers_empty_instead_of_timeout(fc):
    """Неизвестный код не игнорируется: Configurator получает пустой ответ, а не таймаут."""
    assert fc.handle(0x7FFE, b"", 1) == b""


def test_status_ex_layout(fc):
    """MSP_STATUS_EX для API 1.46 — ровно 24 байта (иначе вкладка Setup «моргает»)."""
    payload = fc.handle(MSP_STATUS_EX, b"", 1)
    assert len(payload) == 24
    # ... u8 rateProfile(14), u8 byteCount(15), u8 ARMING_DISABLE_FLAGS_COUNT, u32 флаги, u8 состояние, u16 температура
    byte_count = payload[15]
    assert byte_count == 0, "дополнительных байт флагов режимов нет"
    assert payload[16] == 26, "ARMING_DISABLE_FLAGS_COUNT прошивки 4.5.4"
    assert struct.unpack_from("<H", payload, 22)[0] == 42, "температура ядра (API 1.46+)"


def test_sensor_bits_use_msp_packing(fc):
    """Маска сенсоров пакуется как в msp.c: acc|baro|mag|gps|rangefinder|gyro, а не как в sensors.h."""
    sensors = struct.unpack_from("<H", fc.handle(MSP_STATUS, b"", 1), 4)[0]
    assert sensors & (1 << 0), "нет бита acc"
    assert sensors & (1 << 3), "нет бита GPS — Configurator не запросит MSP_RAW_GPS"
    assert sensors & (1 << 5), "нет бита gyro — в Setup появится NO_GYRO"


def test_board_info_strings_are_length_prefixed(fc):
    """Строки идут с префиксом длины (getText в Configurator), а не как C-строки."""
    payload = fc.handle(MSP_BOARD_INFO, b"", 1)
    offset = 8
    names = []
    for _ in range(3):
        length = payload[offset]
        names.append(payload[offset + 1 : offset + 1 + length].decode())
        offset += 1 + length
    assert names == ["STM32F405", "VIRTUALFC", "BFR"]
    assert payload[offset : offset + 32] == bytes(32), "подпись без USE_SIGNATURE — 32 нуля"


def test_vtx_config_is_15_bytes(fc):
    """Вкладка «Порты» ждёт полный ответ MSP_VTX_CONFIG, иначе не строит таблицу портов."""
    assert len(fc.handle(MSP_VTX_CONFIG, b"", 1)) == 15


def test_analog_is_9_bytes_and_alive(fc):
    payload = fc.handle(MSP_ANALOG, b"", 1)
    assert len(payload) == 9
    voltage = struct.unpack_from("<H", payload, 7)[0] / 100
    assert 10.0 < voltage < 26.0, f"напряжение вне разумного: {voltage} В"


def test_handler_exception_does_not_break_fc(fc):
    """Кривые данные не должны валить обработчик: ответ пустой, FC продолжает работать."""
    fc.handle(0x3012, b"\xff" * 3, 2)
    assert len(fc.handle(MSP_STATUS, b"", 1)) == 13

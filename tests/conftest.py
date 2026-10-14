"""Общая обвязка для быстрых тестов (pytest).

Здесь нет ни сокетов, ни подпроцессов: виртуальный FC создаётся прямо в памяти, «флеш»
кладётся во временный каталог. Такой тест выполняется десятки миллисекунд — его можно
гонять на каждое сохранение файла, в отличие от сквозного `tests/smoke_test.py` (~100 с).
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:                      # запуск без установки пакета
    sys.path.insert(0, str(ROOT))

from bfemu.fc import VirtualFC                     # noqa: E402  (после правки sys.path)


@pytest.fixture
def fc(tmp_path) -> VirtualFC:
    """Чистый виртуальный FC: заводские настройки, «флеш» во временном каталоге."""
    return VirtualFC(eeprom_path=tmp_path / "eeprom.json", log=lambda *a, **k: None)


@pytest.fixture
def repo_root() -> Path:
    return ROOT

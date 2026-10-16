"""
База настроек INAV (settings.yaml → inav_data.json) и протокол настроек MSP:
  MSP2_COMMON_SETTING       (0x1003) — значение настройки;
  MSP2_COMMON_SET_SETTING   (0x1004) — запись значения;
  MSP2_COMMON_SETTING_INFO  (0x1007) — тип, режим, min/max, lookup-таблица, индекс.

Формат ответов — как в fc_msp.c (mspSettingCommand / mspSettingInfoCommand).
"""
from __future__ import annotations

import json
import struct
from pathlib import Path

DATA_PATH = Path(__file__).resolve().parent / "data" / "inav_data.json"

# коды типов (settings.h: VAR_UINT8 = 0 << 0, ...)
TYPE_CODE = {"uint8": 0, "int8": 1, "uint16": 2, "int16": 3, "uint32": 4, "float": 5, "string": 6}
TYPE_SIZE = {"uint8": 1, "int8": 1, "uint16": 2, "int16": 2, "uint32": 4, "float": 4, "string": 0}
TYPE_PACK = {"uint8": "<B", "int8": "<b", "uint16": "<H", "int16": "<h",
             "uint32": "<I", "float": "<f"}

MODE_LOOKUP = 1 << 6  # settings.h

# секции (settings.h): сколько профилей и какой текущий
SECTION_PROFILES = {
    "MASTER_VALUE": 0,
    "PROFILE_VALUE": 3,        # MAX_PROFILE_COUNT
    "CONTROL_VALUE": 3,        # MAX_CONTROL_PROFILE_COUNT
    "EZ_TUNE_VALUE": 3,
    "BATTERY_CONFIG_VALUE": 3,  # MAX_BATTERY_PROFILE_COUNT
    "MIXER_CONFIG_VALUE": 2,   # MAX_MIXER_PROFILE_COUNT
}


class SettingsStore:
    def __init__(self, path: Path | None = None):
        data = json.loads((path or DATA_PATH).read_text(encoding="utf-8"))
        self.source = data.get("source", {})
        self.tables = data.get("lookup_tables", {})
        self.features = data.get("features", [])
        self.boxes = data.get("boxes", [])
        self.serial = data.get("serial", {})
        self.osd_layout_defaults = data.get("osd_layout_defaults", [])
        self.settings = data["settings"]
        self.by_name = {s["name"]: s for s in self.settings}
        self.by_index = {s["index"]: s for s in self.settings} if "index" in self.settings[0] \
            else {}
        # абсолютные индексы — позиции в списке (как в прошивке)
        for i, s in enumerate(self.settings):
            s.setdefault("index", i)
            self.by_index[i] = s
        # текущие значения: стартуют с дефолтов
        self.values = {}
        self.reset_defaults()

    def reset_defaults(self):
        for s in self.settings:
            self.values[s["name"]] = s.get("default", 0)

    # --- доступ по имени ---

    def get(self, name: str):
        return self.values.get(name)

    def set(self, name: str, value) -> bool:
        s = self.by_name.get(name)
        if s is None:
            return False
        self.values[name] = value
        return True

    def encode_value(self, s: dict) -> bytes:
        """Текущее значение в байтах (как шлёт прошивка)."""
        typ = s["type"]
        val = self.values.get(s["name"], 0)
        if typ == "string":
            text = val or ""
            size = s.get("max", 0)  # settingGetValueSize для строк = max
            raw = text.encode("utf-8", "ignore")[:size]
            return raw + b"\x00" * (size - len(raw)) if size else b""
        try:
            return struct.pack(TYPE_PACK[typ], int(val) if typ != "float" else float(val))
        except (struct.error, ValueError, TypeError):
            return struct.pack(TYPE_PACK[typ], 0)

    def decode_value(self, s: dict, payload: bytes):
        typ = s["type"]
        if typ == "string":
            return payload.split(b"\x00", 1)[0].decode("utf-8", "ignore")
        size = TYPE_SIZE[typ]
        if len(payload) < size:
            return None
        return struct.unpack_from(TYPE_PACK[typ], payload)[0]

    # --- протокол MSP ---

    def find(self, payload: bytes):
        """Разбирает ссылку на настройку: имя + NUL, либо 0x00 + u16 индекс."""
        if len(payload) >= 3 and payload[0] == 0:
            idx = int.from_bytes(payload[1:3], "little")
            return self.by_index.get(idx)
        name = payload.split(b"\x00", 1)[0].decode("utf-8", "ignore")
        return self.by_name.get(name)

    def setting_info(self, s: dict) -> bytes:
        """Ответ MSP2_COMMON_SETTING_INFO (mspSettingInfoCommand)."""
        out = bytearray()
        out += s["name"].encode() + b"\x00"
        out += struct.pack("<H", s.get("pg", 0) & 0xFFFF)
        out += bytes([TYPE_CODE[s["type"]]])
        out += bytes([_section_code(s.get("section", "MASTER_VALUE"))])
        out += bytes([s.get("mode", 0)])
        out += struct.pack("<i", int(s.get("min", 0)))
        out += struct.pack("<I", int(s.get("max", 0)) & 0xFFFFFFFF)
        out += struct.pack("<H", s["index"] & 0xFFFF)
        section = s.get("section", "MASTER_VALUE")
        count = SECTION_PROFILES.get(section, 0)
        if count:
            out += bytes([0, count])  # текущий профиль 0 и их число
        else:
            out += bytes([0, 0])
        if s.get("mode") == MODE_LOOKUP:
            for name in s.get("values", []):
                out += name.encode() + b"\x00"
        out += self.encode_value(s)
        return bytes(out)


def _section_code(section: str) -> int:
    return {
        "MASTER_VALUE": 0,
        "PROFILE_VALUE": 1,
        "CONTROL_VALUE": 2,
        "BATTERY_CONFIG_VALUE": 3,
        "MIXER_CONFIG_VALUE": 4,
        "EZ_TUNE_VALUE": 5,
    }.get(section, 0)

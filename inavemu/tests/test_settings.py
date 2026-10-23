"""Протокол настроек MSP2_COMMON_SETTING(_INFO) поверх inav_data.json."""
import struct

from inavemu.settings import MODE_LOOKUP, SettingsStore


def test_data_loaded():
    store = SettingsStore()
    assert len(store.settings) > 600  # у INAV 9.1 — 677
    assert store.source.get("api_version", "").startswith("2.5")
    assert store.boxes and store.boxes[0]["name"] == "ARM"
    assert any(f["name"] == "GPS" for f in store.features)


def test_find_by_name_and_index():
    store = SettingsStore()
    s = store.find(b"looptime\x00")
    assert s is not None and s["name"] == "looptime"
    s2 = store.find(bytes([0]) + struct.pack("<H", s["index"]))
    assert s2 is not None and s2["name"] == "looptime"
    assert store.find(b"no_such_setting\x00") is None


def test_setting_info_layout():
    store = SettingsStore()
    s = store.by_name["gyro_filter_mode"]
    info = store.setting_info(s)
    # name + NUL
    assert info.startswith(b"gyro_filter_mode\x00")
    off = 17
    _pg = struct.unpack_from("<H", info, off)[0]
    off += 2
    typ, section, mode = info[off], info[off + 1], info[off + 2]
    off += 3
    mn = struct.unpack_from("<i", info, off)[0]
    mx = struct.unpack_from("<I", info, off + 4)[0]
    off += 8
    idx = struct.unpack_from("<H", info, off)[0]
    off += 2
    _prof_cur, _prof_cnt = info[off], info[off + 1]
    off += 2
    assert mode == MODE_LOOKUP
    assert (mn, mx) == (0, len(s["values"]) - 1)
    assert typ == 0 and section == 0  # VAR_UINT8, MASTER
    assert idx == s["index"]
    # lookup-таблица строк с NUL
    rest = info[off:]
    names = rest.split(b"\x00")
    assert [n.decode() for n in names[:len(s["values"])]] == s["values"]


def test_value_roundtrip():
    store = SettingsStore()
    assert store.get("looptime") == 500
    store.set("looptime", 2000)
    s = store.by_name["looptime"]
    assert store.encode_value(s) == struct.pack("<H", 2000)
    assert store.decode_value(s, struct.pack("<H", 2000)) == 2000


def test_string_value():
    store = SettingsStore()
    s = store.by_name["name"]
    store.set("name", "ABC")
    raw = store.encode_value(s)
    assert len(raw) == s["max"]  # settingGetValueSize = max
    assert raw.startswith(b"ABC")
    assert store.decode_value(s, b"XYZ\x00") == "XYZ"


def test_defaults_are_sane():
    store = SettingsStore()
    # дефолты — числа/строки, lookup — индексы в диапазоне
    for s in store.settings:
        v = store.values[s["name"]]
        if s["type"] == "string":
            assert isinstance(v, str)
        else:
            assert isinstance(v, (int, float))
            if s.get("mode") == MODE_LOOKUP:
                assert s["min"] <= v <= s["max"]

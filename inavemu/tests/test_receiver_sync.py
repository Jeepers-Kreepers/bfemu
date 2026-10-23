"""Синхронизация вкладки Receiver с базой настроек.

Вкладка Receiver (tabs/receiver.js 9.1.1) читает и пишет:
  * RSSI channel — FC.MISC.rssi_channel (MSP2_INAV_MISC/SET_MISC);
  * Deadband / Yaw deadband — FC.RC_deadband (MSP_RC_DEADBAND/SET);
  * Throttle MID/EXPO — FC.RC_tuning (MSP2_INAV_RATE_PROFILE/SET);
  * receiver_type, serialrx_* — data-setting (протокол настроек).
Как и в прошивке, всё это — поля PG_RX_CONFIG / rcControls / controlProfile,
то есть те же CLI-настройки: `set rssi_channel`, `set deadband`,
`set serialrx_provider`, `set thr_mid` и т.д. обязаны переключать вкладку,
а Save вкладки — менять значения CLI.
"""
import struct

import pytest

from inavemu.cli import CliSession
from inavemu.fc import (
    MSP2_INAV_MISC, MSP2_INAV_RATE_PROFILE, MSP2_INAV_SET_MISC,
    MSP2_INAV_SET_RATE_PROFILE, MSP_RC_DEADBAND, MSP_RC_TUNING,
    MSP_RX_CONFIG, MSP_SET_RC_DEADBAND, MSP_SET_RC_TUNING,
    MSP_SET_RX_CONFIG, InavFC,
)
from inavemu import msp

# смещения: MSP2_INAV_MISC (41 байт)
MIS_RSSI = 13
# MSP_RX_CONFIG (24 байта)
RX = {"provider": 0, "maxcheck": 1, "mincheck": 5, "bind": 7,
      "rx_min": 8, "rx_max": 10, "receiver_type": 23}
# MSP_RC_DEADBAND (5 байт)
DB = {"deadband": 0, "yaw": 1, "alt_hold": 2, "throttle": 3}
# MSP2_INAV_RATE_PROFILE (15 байт)
RP = {"thr_mid": 0, "thr_expo": 1, "dyn_pid": 2, "pa_breakpoint": 3,
      "rc_expo": 5, "yaw_expo": 6}


@pytest.fixture()
def fc():
    return InavFC()


def ask(fc, code, payload=b"", version=1):
    return fc.handle(msp.Request(version=version, code=code, payload=payload))


# ---------------- дефолты: вкладка и CLI показывают одно и то же ----------------

def test_defaults_receiver_blocks_match_store(fc):
    st = fc.store
    db = ask(fc, MSP_RC_DEADBAND)
    assert db[DB["deadband"]] == st.get("deadband") == 2
    assert db[DB["yaw"]] == st.get("yaw_deadband") == 2
    assert db[DB["alt_hold"]] == st.get("alt_hold_deadband") == 50
    assert struct.unpack_from("<H", db, DB["throttle"])[0] == 50
    rx = ask(fc, MSP_RX_CONFIG)
    assert rx[RX["provider"]] == st.get("serialrx_provider") == 0  # SPEK1024
    assert struct.unpack_from("<H", rx, RX["mincheck"])[0] == 1100
    assert struct.unpack_from("<H", rx, RX["maxcheck"])[0] == 1900
    assert rx[RX["receiver_type"]] == st.get("receiver_type") == 0
    rp = ask(fc, MSP2_INAV_RATE_PROFILE, version=2)
    assert rp[RP["thr_mid"]] == st.get("thr_mid") == 50
    assert rp[RP["rc_expo"]] == st.get("rc_expo") == 70


# ---------------- CLI set → вкладка ----------------

def test_cli_set_rssi_channel_switches_misc(fc):
    cli = CliSession(fc)
    cli.enter()
    cli.feed(b"set rssi_channel = 5\r\n")
    assert ask(fc, MSP2_INAV_MISC, version=2)[MIS_RSSI] == 5


def test_cli_set_deadband_switches_rc_deadband(fc):
    cli = CliSession(fc)
    cli.enter()
    cli.feed(b"set deadband = 7\r\n")
    cli.feed(b"set yaw_deadband = 9\r\n")
    cli.feed(b"set alt_hold_deadband = 60\r\n")
    cli.feed(b"set 3d_deadband_throttle = 70\r\n")
    db = ask(fc, MSP_RC_DEADBAND)
    assert db[DB["deadband"]] == 7
    assert db[DB["yaw"]] == 9
    assert db[DB["alt_hold"]] == 60
    assert struct.unpack_from("<H", db, DB["throttle"])[0] == 70


def test_cli_set_serialrx_provider_switches_rx_config(fc):
    cli = CliSession(fc)
    cli.enter()
    out = cli.feed(b"set serialrx_provider = CRSF\r\n").decode()
    assert "serialrx_provider set to CRSF" in out
    rx = ask(fc, MSP_RX_CONFIG)
    assert rx[RX["provider"]] == 6  # CRSF
    cli.feed(b"set min_check = 1050\r\n")
    cli.feed(b"set rx_max_usec = 2050\r\n")
    rx = ask(fc, MSP_RX_CONFIG)
    assert struct.unpack_from("<H", rx, RX["mincheck"])[0] == 1050
    assert struct.unpack_from("<H", rx, RX["rx_max"])[0] == 2050


def test_cli_set_thr_mid_switches_rate_profile(fc):
    cli = CliSession(fc)
    cli.enter()
    cli.feed(b"set thr_mid = 40\r\n")
    cli.feed(b"set thr_expo = 15\r\n")
    cli.feed(b"set rc_expo = 55\r\n")
    rp = ask(fc, MSP2_INAV_RATE_PROFILE, version=2)
    assert rp[RP["thr_mid"]] == 40
    assert rp[RP["thr_expo"]] == 15
    assert rp[RP["rc_expo"]] == 55
    # легаси MSP_RC_TUNING читает те же значения
    tuning = ask(fc, MSP_RC_TUNING)
    assert tuning[1] == 55
    assert tuning[6] == 40 and tuning[7] == 15


# ---------------- Save вкладки → CLI ----------------

def test_tab_save_rssi_channel_updates_cli(fc):
    # вкладка Receiver: Select Save шлёт MSP2_INAV_SET_MISC с rssi_channel
    misc = bytearray(ask(fc, MSP2_INAV_MISC, version=2))
    misc[MIS_RSSI] = 6
    assert ask(fc, MSP2_INAV_SET_MISC, bytes(misc), version=2) == b""
    assert fc.store.get("rssi_channel") == 6
    cli = CliSession(fc)
    cli.enter()
    assert "rssi_channel = 6" in cli.feed(b"get rssi_channel\r\n").decode()


def test_tab_save_deadband_updates_cli(fc):
    db = struct.pack("<BBBH", 11, 12, 70, 90)
    assert ask(fc, MSP_SET_RC_DEADBAND, db) == b""
    assert fc.store.get("deadband") == 11
    assert fc.store.get("yaw_deadband") == 12
    assert fc.store.get("alt_hold_deadband") == 70
    assert fc.store.get("3d_deadband_throttle") == 90
    cli = CliSession(fc)
    cli.enter()
    assert "deadband = 11" in cli.feed(b"get deadband\r\n").decode()


def test_tab_save_rx_config_updates_cli(fc):
    raw = bytearray(ask(fc, MSP_RX_CONFIG))
    struct.pack_into("<B", raw, RX["provider"], 2)  # SBUS
    struct.pack_into("<H", raw, RX["mincheck"], 1080)
    assert ask(fc, MSP_SET_RX_CONFIG, bytes(raw)) == b""
    assert fc.store.get("serialrx_provider") == 2
    assert fc.store.get("min_check") == 1080
    cli = CliSession(fc)
    cli.enter()
    assert "serialrx_provider = SBUS" in cli.feed(b"get serialrx_provider\r\n").decode()


def test_tab_save_rate_profile_updates_cli(fc):
    rp = bytearray(ask(fc, MSP2_INAV_RATE_PROFILE, version=2))
    struct.pack_into("<B", rp, RP["thr_mid"], 45)
    struct.pack_into("<B", rp, RP["thr_expo"], 20)
    assert ask(fc, MSP2_INAV_SET_RATE_PROFILE, bytes(rp), version=2) == b""
    assert fc.store.get("thr_mid") == 45
    assert fc.store.get("thr_expo") == 20
    # легаси SET_RC_TUNING (10 байт) пишет туда же
    tuning = bytes([100, 30, 3, 3, 3, 0, 48, 0, 0xB8, 0x0B])
    assert ask(fc, MSP_SET_RC_TUNING, tuning) == b""
    assert fc.store.get("rc_expo") == 30
    assert fc.store.get("roll_rate") == 30
    assert fc.store.get("thr_mid") == 48
    assert fc.store.get("tpa_breakpoint") == 3000

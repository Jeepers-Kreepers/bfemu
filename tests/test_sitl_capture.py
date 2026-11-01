"""Запись только фактических MSP-ответов, без подстановки модели полёта."""
from __future__ import annotations

import io
import json
import math
import os
import socket
import struct
import subprocess
import time

import pytest

from bfemu.sitl_capture import (
    MSP_ATTITUDE,
    MSP_MOTOR,
    MSP_RAW_IMU,
    MSP_RC,
    CaptureError,
    SitlMspClient,
    capture,
    decode_payload,
    main,
)
from tools import betaflight_sitl as sitl


def test_decodes_only_actual_protocol_fields_and_preserves_signed_values():
    imu = struct.pack("<9h", -1, 2, -3, 12, -25, 150, -4, 5, -6)
    assert decode_payload(MSP_RAW_IMU, imu) == {"acc_adc": [-1, 2, -3], "gyro_dps": [12, -25, 150],
                                                 "mag_adc": [-4, 5, -6]}
    assert decode_payload(MSP_ATTITUDE, struct.pack("<hhh", -125, 90, -170)) == {
        "roll_decideg": -125, "pitch_decideg": 90, "yaw_deg": -170}
    assert decode_payload(MSP_MOTOR, struct.pack("<8H", *range(1000, 1008)))["motor_output_msp"][0] == 1000
    assert decode_payload(MSP_RC, struct.pack("<4H", 1000, 1500, 1600, 2000))["channels_msp"] == [1000, 1500, 1600, 2000]
    assert decode_payload(94, b"\xff\x00") == {}
    for code, raw in ((MSP_RAW_IMU, b"x"), (MSP_ATTITUDE, b"x"), (MSP_MOTOR, b"x"), (MSP_RC, b"x")):
        with pytest.raises(CaptureError):
            decode_payload(code, raw)


class _FixtureClient:
    def identify(self):
        return {"variant": "BTFL", "version": "4.5.4", "board": "SITL"}

    def request(self, code):
        # Только тестовые фикстуры, не используются в записи реального SITL.
        return {MSP_RAW_IMU: bytes(18), MSP_MOTOR: bytes(16), MSP_RC: bytes(8), MSP_ATTITUDE: bytes(6)}.get(code, b"")


def test_capture_labels_separate_requests_and_keeps_raw_response():
    stream = io.StringIO()
    count = capture(_FixtureClient(), stream, 0.06, 0.02)
    rows = [json.loads(line) for line in stream.getvalue().splitlines()]
    assert rows[0]["type"] == "identity" and "не одновременно" in rows[0]["note"]
    assert len(rows) == count + 1
    assert [row["command"] for row in rows[1:3]] == ["MSP_PID", "MSP_PID_ADVANCED"]
    imu = next(row for row in rows[1:] if row["command"] == "MSP_RAW_IMU")
    assert imu["payload_hex"] == "00" * 18
    assert imu["decoded"]["gyro_dps"] == [0, 0, 0]
    times = [row["received_s"] for row in rows[1:]]
    assert times == sorted(times)


def test_capture_validates_time_and_will_not_overwrite(tmp_path):
    for duration, interval in ((0, 0.1), (math.nan, 0.1), (1, math.inf), (1, 0.001), (3601, 0.1)):
        with pytest.raises(CaptureError):
            capture(_FixtureClient(), io.StringIO(), duration, interval)
    bad_output = tmp_path / "bad.jsonl"
    assert main(["--output", str(bad_output), "--duration", "0"]) == 1
    assert not bad_output.exists()
    output = tmp_path / "record.jsonl"
    output.write_text("важные данные")
    assert main(["--output", str(output)]) == 1
    assert output.read_text() == "важные данные"


def test_identify_rejects_other_controller():
    class Fake:
        def request(self, code):
            return {2: b"BTFL", 3: bytes([4, 5, 4]), 4: b"S405"}[code]

    with pytest.raises(CaptureError, match="SITL"):
        SitlMspClient.identify(Fake())


@pytest.mark.skipif(not os.getenv("BFEMU_TEST_REAL_SITL"), reason="Требуется заранее собранный Betaflight SITL")
def test_capture_reads_real_upstream_sitl(tmp_path):
    assert sitl.built(sitl.DEFAULT_CACHE)
    with socket.socket() as probe:
        try:
            probe.bind(("127.0.0.1", 5761))
        except OSError:
            pytest.skip("Порт 5761 уже используется")
    process = subprocess.Popen([str(sitl.binary_path(sitl.DEFAULT_CACHE))], cwd=tmp_path,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    client = None
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            try:
                client = SitlMspClient(timeout=1)
                break
            except OSError:
                if process.poll() is not None:
                    pytest.fail("SITL завершился до открытия MSP")
                time.sleep(0.1)
        assert client is not None
        output = io.StringIO()
        count = capture(client, output, duration=0.5, interval=0.1)
        rows = [json.loads(line) for line in output.getvalue().splitlines()]
        assert rows[0]["board"] == "SITL" and count > 5
        names = {row["command"] for row in rows[1:]}
        assert {"MSP_PID", "MSP_RAW_IMU", "MSP_MOTOR", "MSP_ATTITUDE"} <= names
        for row in rows[1:]:
            code = {"MSP_RAW_IMU": MSP_RAW_IMU, "MSP_MOTOR": MSP_MOTOR, "MSP_RC": MSP_RC,
                    "MSP_ATTITUDE": MSP_ATTITUDE}.get(row["command"])
            if code is not None:
                assert row["decoded"] == decode_payload(code, bytes.fromhex(row["payload_hex"]))
    finally:
        if client:
            client.close()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)

"""Сборка штатного Betaflight SITL: источник, изоляция сети и запуск."""
from __future__ import annotations

import hashlib
import io
import json
import os
import shutil
import socket
import subprocess
import tarfile
import time

import pytest

from bfemu.msp import FROM_FC, MspParser, encode_v1
from tools import betaflight_sitl as sitl


def test_extract_checks_digest_and_rejects_traversal_and_links(tmp_path, monkeypatch):
    archive = tmp_path / "source.tgz"

    def make_archive(name, *, link=False):
        with tarfile.open(archive, "w:gz") as tar:
            info = tarfile.TarInfo(name)
            if link:
                info.type = tarfile.SYMTYPE
                info.linkname = "/etc/passwd"
                tar.addfile(info)
            else:
                content = b"safe"
                info.size = len(content)
                tar.addfile(info, io.BytesIO(content))
        monkeypatch.setattr(sitl, "ARCHIVE_SHA256", hashlib.sha256(archive.read_bytes()).hexdigest())

    make_archive(f"{sitl.ARCHIVE_ROOT}/../escape")
    with pytest.raises(sitl.SitlError, match="Недопустимый элемент"):
        sitl.extract_verified(archive, tmp_path / "output")
    make_archive(f"{sitl.ARCHIVE_ROOT}/link", link=True)
    with pytest.raises(sitl.SitlError, match="Недопустимый элемент"):
        sitl.extract_verified(archive, tmp_path / "output")
    make_archive(f"{sitl.ARCHIVE_ROOT}/file")
    destination = tmp_path / "output"
    destination.mkdir()
    source = sitl.extract_verified(archive, destination)
    assert (source / "file").read_bytes() == b"safe"
    archive.write_bytes(archive.read_bytes() + b"unexpected")
    with pytest.raises(sitl.SitlError, match="Контрольная сумма"):
        sitl.extract_verified(archive, destination)


def test_network_patch_restricts_tcp_and_udp_without_modifying_controller(tmp_path):
    tcp = tmp_path / "src/main/drivers/serial_tcp.c"
    udp = tmp_path / "src/main/target/SITL/sitl.c"
    tcp.parent.mkdir(parents=True)
    udp.parent.mkdir(parents=True)
    tcp.write_text("dyad_listenEx(s->serv, NULL, BASE_PORT + id + 1, 10)")
    udp.write_text("udpInit(&stateLink, NULL, PORT_STATE, true)\n"
                   "udpInit(&rcLink, NULL, PORT_RC, true)\n")
    sitl.restrict_network(tmp_path)
    assert 'dyad_listenEx(s->serv, "127.0.0.1", BASE_PORT + id + 1, 10)' in tcp.read_text()
    assert udp.read_text().count('"127.0.0.1"') == 2
    with pytest.raises(sitl.SitlError, match="Неожиданный исходный код"):
        sitl.restrict_network(tmp_path)


def test_build_state_validation_and_missing_binary(tmp_path):
    assert not sitl.built(tmp_path)
    source = tmp_path / sitl.REVISION
    source.mkdir()
    (source / "bfemu-sitl.json").write_text(json.dumps(sitl.STAMP))
    assert not sitl.built(tmp_path)
    binary = sitl.binary_path(tmp_path)
    binary.parent.mkdir(parents=True)
    binary.write_bytes(b"not a real executable")
    assert sitl.built(tmp_path)
    (source / "bfemu-sitl.json").write_text('{}')
    assert not sitl.built(tmp_path)
    assert sitl.main(["--cache-dir", str(tmp_path), "run"]) == 1


def test_refuses_to_overwrite_existing_incomplete_build(tmp_path):
    if not shutil.which("gcc") or not shutil.which("make"):
        pytest.skip("Компилятор недоступен")
    target = tmp_path / sitl.REVISION
    target.mkdir()
    (target / "important").write_text("данные")
    with pytest.raises(sitl.SitlError, match="существует"):
        sitl.build(tmp_path, 1)
    assert (target / "important").read_text() == "данные"


def test_occupied_port_is_reported():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        port = listener.getsockname()[1]
        with pytest.raises(sitl.SitlError, match=f"TCP {port} занят"):
            sitl.ensure_ports_free(((socket.SOCK_STREAM, port),))


@pytest.mark.skipif(not os.getenv("BFEMU_TEST_REAL_SITL"), reason="Требуется заранее собранный Betaflight SITL")
def test_real_sitl_msp_identity_and_pid_roundtrip(tmp_path):
    """Опциональная проверка прошивки, без эмулятора и без сетевых загрузок в pytest."""
    cache = sitl.DEFAULT_CACHE
    assert sitl.built(cache), "Сначала запустите tools/betaflight_sitl.py build"
    with socket.socket() as probe:
        try:
            probe.bind(("127.0.0.1", 5761))
        except OSError:
            pytest.skip("Порт 5761 уже используется")
    process = subprocess.Popen([str(sitl.binary_path(cache))], cwd=tmp_path,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    client = None
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            try:
                client = socket.create_connection(("127.0.0.1", 5761), timeout=1)
                break
            except OSError:
                if process.poll() is not None:
                    pytest.fail("SITL завершился до открытия MSP")
                time.sleep(0.1)
        assert client is not None
        client.settimeout(3)
        parser = MspParser()

        def exchange(code, payload=b""):
            client.sendall(encode_v1(code, payload))
            while True:
                chunk = client.recv(4096)
                assert chunk, "SITL закрыл соединение"
                parser.feed(chunk)
                for reply in parser:
                    assert reply.code == code and reply.direction == FROM_FC
                    return reply.payload

        assert exchange(2) == b"BTFL"
        assert exchange(3) == bytes([4, 5, 4])
        assert exchange(4)[:4] == b"SITL"
        original = exchange(112)
        assert len(original) >= 15
        adjusted = bytes([min(250, original[0] + 1)]) + original[1:]
        assert adjusted != original
        exchange(202, adjusted)
        assert exchange(112) == adjusted
        exchange(202, original)
        assert exchange(112) == original
    finally:
        if client:
            client.close()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)

#!/usr/bin/env python3
"""Сборка и запуск официальной прошивки Betaflight 4.5.4 для SITL на Linux.

Это отдельный процесс; он пока не подключён к веб-тренажёру bfemu.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import shutil
import socket
import subprocess
import sys
import tarfile
import tempfile
from pathlib import Path, PurePosixPath
from urllib.request import urlopen

REVISION = "25356b59a96dc975d777844f896b7c7855ea90e6"  # Betaflight 4.5.4
ARCHIVE_SHA256 = "0613fcc69675ae82891d97ee8aaa91545b73ba43f7e9120a8e1799e897dde4e3"
ARCHIVE_URL = f"https://codeload.github.com/betaflight/betaflight/tar.gz/{REVISION}"
ARCHIVE_ROOT = f"betaflight-{REVISION}"
DEFAULT_CACHE = Path.home() / ".cache" / "bfemu" / "betaflight-sitl"
STAMP = {"revision": REVISION, "archive_sha256": ARCHIVE_SHA256, "network": "loopback-v1"}
MAX_ARCHIVE_BYTES = 50 * 1024 * 1024
MAX_EXTRACTED_BYTES = 300 * 1024 * 1024


class SitlError(Exception):
    """Сборка или запуск штатного SITL невозможны."""


def extract_verified(archive: Path, destination: Path) -> Path:
    """Извлечь только проверенный архив без ссылок и выхода за каталог."""
    if archive.stat().st_size > MAX_ARCHIVE_BYTES:
        raise SitlError("Архив исходников слишком велик.")
    with archive.open("rb") as f:
        digest = hashlib.sha256()
        for block in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(block)
    if digest.hexdigest() != ARCHIVE_SHA256:
        raise SitlError("Контрольная сумма архива Betaflight не совпадает.")
    with tarfile.open(archive, "r:gz") as tar:
        members = tar.getmembers()
        size = 0
        for member in members:
            path = PurePosixPath(member.name)
            if (not path.parts or path.parts[0] != ARCHIVE_ROOT or path.is_absolute()
                    or ".." in path.parts or not (member.isfile() or member.isdir())):
                raise SitlError(f"Недопустимый элемент архива: {member.name}")
            size += member.size
            if size > MAX_EXTRACTED_BYTES:
                raise SitlError("Распакованные исходники слишком велики.")
        # Архив уже проверен по SHA-256; дополнительно запрещены ссылки и путь за пределами destination.
        tar.extractall(destination, members=members)
    return destination / ARCHIVE_ROOT


def restrict_network(source: Path) -> None:
    """Изолировать стандартные TCP/UDP-сокеты SITL на localhost, не меняя алгоритмы FC."""
    edits = {
        "src/main/drivers/serial_tcp.c": (
            ("dyad_listenEx(s->serv, NULL, BASE_PORT + id + 1, 10)",
             'dyad_listenEx(s->serv, "127.0.0.1", BASE_PORT + id + 1, 10)'),
        ),
        "src/main/target/SITL/sitl.c": (
            ("udpInit(&stateLink, NULL, PORT_STATE, true)",
             'udpInit(&stateLink, "127.0.0.1", PORT_STATE, true)'),
            ("udpInit(&rcLink, NULL, PORT_RC, true)",
             'udpInit(&rcLink, "127.0.0.1", PORT_RC, true)'),
        ),
    }
    for relative, replacements in edits.items():
        path = source / relative
        text = path.read_text(encoding="utf-8")
        for old, new in replacements:
            if text.count(old) != 1:
                raise SitlError(f"Неожиданный исходный код сетевого интерфейса: {relative}")
            text = text.replace(old, new)
        path.write_text(text, encoding="utf-8")


def binary_path(cache: Path) -> Path:
    return cache / REVISION / "obj" / "main" / "betaflight_SITL.elf"


def built(cache: Path) -> bool:
    try:
        marker = json.loads((cache / REVISION / "bfemu-sitl.json").read_text(encoding="utf-8"))
        return marker == STAMP and binary_path(cache).is_file()
    except (OSError, ValueError):
        return False


def build(cache: Path, jobs: int) -> None:
    if platform.system() != "Linux":
        raise SitlError("Сборка проверена только на Linux; для неё нужны make и gcc.")
    if not all(shutil.which(name) for name in ("make", "gcc")):
        raise SitlError("Для сборки нужны make и gcc в PATH.")
    if not 1 <= jobs <= 32:
        raise SitlError("Число параллельных задач должно быть от 1 до 32.")
    cache.mkdir(parents=True, exist_ok=True)
    if built(cache):
        print(f"SITL уже собран: {binary_path(cache)}")
        return
    target = cache / REVISION
    if target.exists():
        raise SitlError(f"Каталог {target} существует, но не является готовой сборкой; выберите другой каталог кэша.")
    with tempfile.TemporaryDirectory(prefix="build-", dir=cache) as tmp:
        temporary = Path(tmp)
        archive = temporary / "source.tar.gz"
        print(f"Загрузка Betaflight 4.5.4, ревизия {REVISION}...", flush=True)
        with urlopen(ARCHIVE_URL, timeout=60) as response, archive.open("wb") as output:
            copied = 0
            while block := response.read(1024 * 1024):
                copied += len(block)
                if copied > MAX_ARCHIVE_BYTES:
                    raise SitlError("Архив исходников слишком велик.")
                output.write(block)
        source = extract_verified(archive, temporary)
        restrict_network(source)
        # upstream tools.mk требует ARM SDK даже для SITL. Указание существующего
        # каталога обходит эту проверку; mk/mcu/SITL.mk затем выбирает системный gcc.
        sdk_guard = temporary / "sitl-sdk-guard"
        sdk_guard.mkdir()
        print("Сборка нативного SITL; это может занять несколько минут...", flush=True)
        subprocess.run(["make", "TARGET=SITL", f"ARM_SDK_DIR={sdk_guard}", f"-j{jobs}"],
                       cwd=source, check=True)
        if not (source / "obj" / "main" / "betaflight_SITL.elf").is_file():
            raise SitlError("Сборка завершилась без исполняемого файла SITL.")
        (source / "bfemu-sitl.json").write_text(json.dumps(STAMP) + "\n", encoding="utf-8")
        os.rename(source, target)
    print(f"Сборка готова: {binary_path(cache)}")


def ensure_ports_free(ports=((socket.SOCK_STREAM, 5761), (socket.SOCK_DGRAM, 9003), (socket.SOCK_DGRAM, 9004))) -> None:
    # Проверка не устраняет гонку с другими процессами после закрытия сокета.
    for kind, port in ports:
        with socket.socket(socket.AF_INET, kind) as probe:
            try:
                probe.bind(("127.0.0.1", port))
            except OSError as exc:
                protocol = "TCP" if kind == socket.SOCK_STREAM else "UDP"
                raise SitlError(f"Порт {protocol} {port} занят; остановите конфликтующий процесс.") from exc


def run(cache: Path, workdir: Path) -> int:
    if not built(cache):
        raise SitlError("Сначала выполните команду build для этой ревизии.")
    ensure_ports_free()
    workdir.mkdir(parents=True, exist_ok=True)
    print("Запуск Betaflight SITL 4.5.4. MSP: tcp://127.0.0.1:5761; UDP: 9001–9004.", flush=True)
    print(f"EEPROM SITL: {workdir / 'eeprom.bin'}. Это отдельный контроллер, не веб-тренажёр bfemu.", flush=True)
    print("Нужна внешняя модель полёта; без неё графики и динамика дрона не рассчитываются.", flush=True)
    return subprocess.run([str(binary_path(cache))], cwd=workdir, check=False).returncode


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache-dir", type=Path, default=DEFAULT_CACHE)
    commands = parser.add_subparsers(dest="command", required=True)
    build_parser = commands.add_parser("build", help="загрузить проверенные исходники и собрать прошивку")
    build_parser.add_argument("--jobs", type=int, default=min(os.cpu_count() or 1, 4))
    run_parser = commands.add_parser("run", help="запустить отдельный процесс прошивки")
    run_parser.add_argument("--work-dir", type=Path, help="каталог для EEPROM SITL")
    args = parser.parse_args(argv)
    cache = args.cache_dir.expanduser().resolve()
    try:
        if args.command == "build":
            build(cache, args.jobs)
            return 0
        return run(cache, (args.work_dir or cache / "state").expanduser().resolve())
    except (SitlError, OSError, subprocess.CalledProcessError, tarfile.TarError) as exc:
        print(f"SITL: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())

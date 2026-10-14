#!/usr/bin/env python3
"""Все проверки проекта одной командой — и локально, и в CI.

    python ci/checks.py lint          # ruff
    python ci/checks.py unit          # быстрые тесты (pytest)
    python ci/checks.py bat unit      # несколько проверок подряд
    python ci/checks.py all           # всё, в порядке возрастания времени

Зачем отдельный файл, а не шаги в `.github/workflows/ci.yml`: файлы рабочих процессов
GitHub разрешает менять только токеном с правом `workflows`, поэтому рабочий процесс
сведён к обёртке «склонировать — поставить Python — вызвать эту программу», а вся
логика живёт в репозитории и меняется обычным коммитом. Побочная польза: ровно те же
проверки запускаются на своей машине одной строкой, без установки GitHub-раннера.

Зависимостей нет: только стандартная библиотека. Недостающие инструменты (ruff, pytest,
build) программа ставит сама через `python -m pip`.
"""
from __future__ import annotations

import argparse
import os
import platform
import shutil
import socket
import subprocess
import sys
import time
import venv
from collections.abc import Callable, Sequence
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
IN_ACTIONS = bool(os.environ.get("GITHUB_ACTIONS"))

# Вывод у нас русский, а Python на Windows пишет stdout в системной кодовой странице:
# без этой строки первое же русское слово роняет программу с UnicodeEncodeError.
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")


# --------------------------------------------------------------------------------------
# мелкая обвязка


def say(text: str) -> None:
    print(text, flush=True)


def group(title: str) -> None:
    """Заголовок раздела: в логе GitHub Actions — сворачиваемый блок."""
    say(f"::group::{title}" if IN_ACTIONS else f"\n=== {title} ===")


def endgroup() -> None:
    if IN_ACTIONS:
        say("::endgroup::")


def run(command: Sequence[str], cwd: Path | None = None, timeout: int | None = None) -> int:
    """Запустить команду, показав её целиком. Возвращает код возврата."""
    say("$ " + " ".join(str(part) for part in command))
    try:
        return subprocess.call(list(command), cwd=str(cwd or ROOT), timeout=timeout)
    except subprocess.TimeoutExpired:
        say(f"[!] команда не уложилась в {timeout} с")
        return 124
    except FileNotFoundError:
        say(f"[!] не найдена программа: {command[0]}")
        return 127


def ensure(module: str, package: str | None = None) -> bool:
    """Поставить инструмент, если его ещё нет (в CI окружение всегда чистое).

    Возвращает False, если поставить не удалось: на домашней машине системный Python
    обычно защищён от установки пакетов (PEP 668), и честное объяснение полезнее
    невнятного «No module named ruff».
    """
    if _importable(module):
        return True
    say(f"[i] ставлю {package or module}")
    if run([sys.executable, "-m", "pip", "install", "--quiet", package or module]) == 0:
        return True
    say(f"[!] не удалось поставить {package or module} в {sys.executable}.")
    say('    Создайте окружение: python -m venv .venv && .venv/bin/pip install -e ".[dev]"')
    say("    и запускайте проверки его интерпретатором: .venv/bin/python ci/checks.py all")
    return False


def _importable(module: str) -> bool:
    import importlib.util

    try:
        return importlib.util.find_spec(module) is not None
    except (ImportError, ValueError):
        return False


# --------------------------------------------------------------------------------------
# сами проверки


def check_lint() -> int:
    """Линтер: правила E/W/F/B/UP/RUF, настройки — в `pyproject.toml`."""
    if not ensure("ruff"):
        return 127
    return run([sys.executable, "-m", "ruff", "check", "."])


def check_unit() -> int:
    """Быстрые тесты: виртуальный FC в памяти, без сокетов и подпроцессов (~10 с)."""
    if not ensure("pytest"):
        return 127
    return run([sys.executable, "-m", "pytest", "tests", "-q"])


def check_smoke() -> int:
    """Сквозной прогон: настоящий сервер, настоящие MSP-кадры, 617 проверок (~100 с)."""
    return run([sys.executable, "tests/smoke_test.py"], timeout=900)


def check_bat() -> int:
    """Командные файлы Windows: только ASCII и переводы строк CRLF.

    Файл с LF запускается, выводит мусор и закрывает окно быстрее, чем его прочтут, —
    именно из-за этого проект падал сразу после клонирования. Проверка обязана идти
    на `windows-latest`: концы строк появляются при выгрузке из git, а не в репозитории.
    """
    beaten = []
    for path in sorted(ROOT.glob("*.bat")):
        data = path.read_bytes()
        lines, with_cr = data.count(b"\n"), data.count(b"\r\n")
        cyrillic = sum(1 for byte in data if byte > 127)
        if cyrillic:
            beaten.append(f"{path.name}: {cyrillic} не-ASCII байт")
        if lines != with_cr:
            beaten.append(f"{path.name}: строки без CR — cmd.exe закроет окно")
        say(f"  {path.name}: строк {lines}, из них CRLF {with_cr}, не-ASCII {cyrillic}")
    for complaint in beaten:
        say(f"[!] {complaint}")
    return 1 if beaten else 0


def check_package() -> int:
    """Пакет собирается, ставится в чистое окружение и отвечает по MSP.

    Здесь ловятся опечатки в `pyproject.toml`, забытые файлы данных (`bf_data.json`
    не попадёт в колесо, если его не перечислить) и сломанная точка входа `bfemu`.
    """
    if not ensure("build"):
        return 127
    dist = ROOT / "dist"
    if dist.exists():
        shutil.rmtree(dist)
    if (code := run([sys.executable, "-m", "build"])) != 0:
        return code

    wheels = sorted(dist.glob("*.whl"))
    if not wheels:
        say("[!] колесо не собралось")
        return 1

    env_dir = ROOT / "build" / "ci-venv"
    if env_dir.exists():
        shutil.rmtree(env_dir)
    venv.create(env_dir, with_pip=True, clear=True)
    scripts = env_dir / ("Scripts" if platform.system() == "Windows" else "bin")
    suffix = ".exe" if platform.system() == "Windows" else ""
    pip, emulator = scripts / f"pip{suffix}", scripts / f"bfemu{suffix}"

    if (code := run([str(pip), "install", "--quiet", str(wheels[0])])) != 0:
        return code
    if (code := run([str(emulator), "--help"])) != 0:
        say("[!] установленная команда bfemu не запускается")
        return code
    return _installed_emulator_answers(emulator)


def _installed_emulator_answers(emulator: Path, port: int = 15761) -> int:
    """Поднять установленный пакет и получить от него настоящий ответ MSP_FC_VARIANT."""
    log_path = ROOT / "build" / "ci-emulator.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    say(f"$ {emulator} --tcp 127.0.0.1:{port} --quiet   (в фоне, вывод в {log_path.name})")
    # Вывод обязательно в файл: шаг CI, унаследовавший поток фонового процесса,
    # висит до таймаута, потому что раннер ждёт закрытия этого потока.
    with log_path.open("wb") as log:
        process = subprocess.Popen(
            [str(emulator), "--tcp", f"127.0.0.1:{port}", "--quiet"],
            stdout=log, stderr=subprocess.STDOUT, cwd=str(log_path.parent),
        )
    try:
        answer = _ask_fc_variant(port)
        if answer != b"BTFL":
            say(f"[!] ожидали BTFL, получили {answer!r}")
            say(log_path.read_text(encoding="utf-8", errors="replace")[-2000:])
            return 1
        say("  установленный bfemu отвечает: BTFL")
        return 0
    finally:
        process.terminate()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()


def _ask_fc_variant(port: int, attempts: int = 30) -> bytes:
    """Дождаться порта и спросить MSP_FC_VARIANT (команда 2) вручную собранным кадром."""
    body = bytes([0, 2])
    checksum = 0
    for byte in body:
        checksum ^= byte
    for attempt in range(attempts):
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=2) as sock:
                sock.sendall(b"$M<" + body + bytes([checksum]))
                sock.settimeout(5)
                return sock.recv(64)[5:9]
        except OSError:
            if attempt == attempts - 1:
                raise
            time.sleep(1)
    return b""


CHECKS: dict[str, Callable[[], int]] = {
    "lint": check_lint,
    "unit": check_unit,
    "bat": check_bat,
    "smoke": check_smoke,
    "package": check_package,
}
ORDER = ["lint", "bat", "unit", "package", "smoke"]      # от быстрых к медленным


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Проверки проекта bfemu (те же, что в CI).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="\n".join(f"  {name:8} {CHECKS[name].__doc__.splitlines()[0]}" for name in ORDER),
    )
    parser.add_argument("checks", nargs="+", metavar="ПРОВЕРКА",
                        choices=[*CHECKS, "all"], help="имена проверок или all")
    names = parser.parse_args(argv).checks
    if "all" in names:
        names = ORDER

    results: dict[str, int] = {}
    for name in names:
        group(f"{name}: {CHECKS[name].__doc__.splitlines()[0]}")
        started = time.monotonic()
        results[name] = CHECKS[name]()
        say(f"[{'ok' if results[name] == 0 else 'ОШИБКА'}] {name}, {time.monotonic() - started:.0f} с")
        endgroup()

    if len(results) > 1:
        say("\nИтог:")
        for name, code in results.items():
            say(f"  {'✔' if code == 0 else '✘'} {name}")
    return 0 if all(code == 0 for code in results.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())

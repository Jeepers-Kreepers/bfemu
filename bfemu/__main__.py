"""Точка входа ``python -m bfemu``.

Дублирует защиту на случай запуска не через .bat: ошибка печатается, окно не «схлопывается»,
а при интерактивном запуске ещё и ждёт нажатия Enter.
"""
from __future__ import annotations

import sys
import traceback

from .server import main


def run() -> int:
    try:
        return main()
    except KeyboardInterrupt:
        print("\n[FC] остановка")
        return 0
    except SystemExit as exc:  # argparse / явные выходы
        code = exc.code
        return code if isinstance(code, int) else (0 if code is None else 1)
    except Exception:
        print("[X] Непредвиденная ошибка:", file=sys.stderr)
        traceback.print_exc()
        return 1


def _pause_if_interactive():
    if sys.stdin is None or not getattr(sys.stdin, "isatty", lambda: False)():
        return
    try:
        input("\nНажмите Enter, чтобы закрыть окно...")
    except (EOFError, KeyboardInterrupt, OSError):
        pass


if __name__ == "__main__":
    code = run()
    if code not in (0, 2):  # 2 = «нечего запускать» (это осознанный выход с подсказкой)
        _pause_if_interactive()
    raise SystemExit(code)

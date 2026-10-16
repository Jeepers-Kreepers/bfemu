"""Эксклюзивная блокировка: запуск двух и более виртуальных FC запрещён.

Bfemu-совместимая идея: lock-файл в системном temp-каталоге, который
удерживается ядром (flock на POSIX, msvcrt.locking на Windows) и снимается
автоматически при завершении процесса — «протухших» блокировок не бывает,
убитый процесс не блокирует следующий запуск.
"""
from __future__ import annotations

import os
import tempfile
from pathlib import Path

_LOCK_SUFFIX = ".lock"


class InstanceLock:
    """Глобальная (на машину) блокировка экземпляра виртуального контроллера."""

    def __init__(self, name: str = "inavemu", path: Path | None = None):
        self.path = Path(path) if path else \
            Path(tempfile.gettempdir()) / (name + _LOCK_SUFFIX)
        self._fh = None

    def acquire(self) -> bool:
        """Пытается занять блокировку. False — виртуальный FC уже запущен."""
        if self._fh is not None:
            return True
        fh = open(self.path, "w", encoding="utf-8")
        try:
            if os.name == "nt":  # pragma: no cover - проверяется на Windows
                import msvcrt

                msvcrt.locking(fh.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl

                fcntl.flock(fh.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            fh.close()
            return False
        try:
            fh.write(str(os.getpid()))
            fh.flush()
        except OSError:
            pass
        self._fh = fh
        return True

    def release(self) -> None:
        if self._fh is None:
            return
        try:
            if os.name == "nt":  # pragma: no cover
                import msvcrt

                msvcrt.locking(self._fh.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                import fcntl

                fcntl.flock(self._fh.fileno(), fcntl.LOCK_UN)
        except OSError:
            pass
        self._fh.close()
        self._fh = None

    def __enter__(self):
        if not self.acquire():
            raise RuntimeError(f"виртуальный FC уже запущен (lock: {self.path})")
        return self

    def __exit__(self, *exc):
        self.release()
        return False

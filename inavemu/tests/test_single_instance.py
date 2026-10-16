"""Блокировка: запуск двух и более виртуальных контроллеров запрещён."""
import os

import pytest

from inavemu.single_instance import InstanceLock


def test_second_instance_rejected(tmp_path):
    lock_path = tmp_path / "inavemu-test.lock"
    a = InstanceLock(path=lock_path)
    b = InstanceLock(path=lock_path)

    assert a.acquire() is True
    assert b.acquire() is False, "второй экземпляр должен быть отклонён"

    a.release()
    assert b.acquire() is True, "после остановки первого запуск должен работать"
    b.release()


def test_lock_pid_recorded(tmp_path):
    lock = InstanceLock(path=tmp_path / "inavemu-test2.lock")
    assert lock.acquire()
    try:
        assert lock.path.read_text() == str(os.getpid())
    finally:
        lock.release()


def test_context_manager(tmp_path):
    lock = InstanceLock(path=tmp_path / "inavemu-test3.lock")
    with lock:
        again = InstanceLock(path=lock.path)
        assert again.acquire() is False
        with pytest.raises(RuntimeError):
            with again:
                pass
    with lock:
        pass  # повторный захват после release работает

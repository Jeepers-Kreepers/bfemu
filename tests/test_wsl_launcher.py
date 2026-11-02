"""Запуск Linux-части Windows/WSL без перезаписи Windows-конфигурации."""
from __future__ import annotations

import os
from pathlib import Path
import socket
import subprocess
import sys
import time
from urllib.request import urlopen

import pytest

from bfemu import aircraft_sitl
from bfemu.server import build_arg_parser

ROOT = Path(__file__).resolve().parent.parent


def test_windows_sitl_explains_wsl_without_starting_builder(monkeypatch):
    with monkeypatch.context() as patch:
        patch.setattr(aircraft_sitl.sys, 'platform', 'win32')
        status = aircraft_sitl.start_sitl_build()
    assert status['state'] == 'error'
    assert 'WSL' in status['error'] and 'run_sitl_wsl.bat' in status['error']


def test_strict_http_option_rejects_busy_port():
    assert build_arg_parser().parse_args(['--http', '9999', '--http-strict']).http_strict
    with socket.socket() as holder:
        holder.bind(('127.0.0.1', 0))
        holder.listen()
        port = holder.getsockname()[1]
        result = subprocess.run([sys.executable, '-m', 'bfemu', '--tcp', '', '--http', str(port),
                                 '--http-strict', '--eeprom', 'none'], cwd=ROOT, capture_output=True,
                                text=True, timeout=15, check=False)
    assert result.returncode != 0
    assert f'порт веб-монитора {port} занят' in result.stdout


def test_linux_side_of_wsl_launcher_serves_aircraft_without_touching_project_eeprom(tmp_path):
    if sys.platform != 'linux':
        pytest.skip('Linux-часть WSL требует Linux')
    ports = []
    try:
        for port in (18788, 15777):
            probe = socket.socket()
            probe.bind(('127.0.0.1', port))
            ports.append(probe)
    except OSError:
        pytest.skip('Тестовые порты WSL заняты')
    finally:
        for probe in ports:
            probe.close()
    env = dict(os.environ, XDG_STATE_HOME=str(tmp_path / 'state'))
    process = subprocess.Popen(['sh', str(ROOT / 'tools/run_wsl_sitl.sh')], cwd=ROOT, env=env,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.monotonic() + 15
        page = None
        while time.monotonic() < deadline:
            if process.poll() is not None:
                pytest.fail('Linux-часть запускателя завершилась: ' + process.stdout.read())
            try:
                with urlopen('http://127.0.0.1:18788/aircraft', timeout=.5) as response:
                    page = response.read().decode()
                break
            except OSError:
                time.sleep(.1)
        assert page is not None and 'Запустить готовый пример 7″' in page
        assert (tmp_path / 'state' / 'bfemu-wsl').is_dir()
    finally:
        process.terminate()
        process.wait(timeout=5)

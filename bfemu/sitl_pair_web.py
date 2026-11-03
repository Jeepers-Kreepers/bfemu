"""Изолированный парный опыт для веб-интерфейса, без изменения профиля FC."""
from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile

from . import aircraft_sitl
from .flight_actuators import output_mapping, read_curves
from .flight_physics import Aircraft, PhysicsError
from .sitl_pair import analyze

ROOT = Path(__file__).resolve().parent.parent


def experiment(axis, before, after, profile):
    if axis not in ('roll', 'pitch', 'yaw'):
        raise PhysicsError('Неизвестная ось.')
    for values in (before, after):
        if (not isinstance(values, dict) or set(values) != {'p', 'i', 'd'} or
                any(type(values[k]) is not int or not 0 <= values[k] <= 250 for k in ('p', 'i', 'd'))):
            raise PhysicsError('P, I и D Max должны быть целыми числами от 0 до 250.')
    if sum(before[k] != after[k] for k in ('p', 'i', 'd')) != 1:
        raise PhysicsError('Измените ровно один коэффициент.')
    if sys.platform != 'linux':
        raise PhysicsError('Для опыта необходим Linux или Ubuntu в WSL.')
    script = ROOT/'tools/sitl_lockstep_probe.py'
    if not script.is_file():
        raise PhysicsError('Нужна полная копия проекта с исследовательскими инструментами.')
    Aircraft.from_profile(profile)
    output_mapping(profile)
    read_curves(aircraft_sitl.theoretical_curves(profile))
    if not aircraft_sitl.sitl_available():
        raise PhysicsError('Сначала подготовьте Betaflight SITL.')
    if not aircraft_sitl._LOCK.acquire(blocking=False):
        raise PhysicsError('Другой эксперимент SITL уже выполняется.')
    try:
        with tempfile.TemporaryDirectory(prefix='bfemu-pair-') as temporary:
            root = Path(temporary)
            profile_path, out = root/'profile.json', root/'results'
            profile_path.write_text(json.dumps(profile, ensure_ascii=False, allow_nan=False), encoding='utf-8')
            command = [sys.executable, str(script), '--output-dir', str(out), '--profile', str(profile_path),
                       '--scenario', 'flight', '--prearm-bench', '--axis', axis, '--repeats', '2',
                       '--pid', *(str(before[k]) for k in ('p', 'i', 'd')),
                       '--compare-pid', *(str(after[k]) for k in ('p', 'i', 'd'))]
            with (root/'run.log').open('w', encoding='utf-8') as log:
                process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                           start_new_session=True)
                try:
                    process.wait(timeout=600)
                except subprocess.TimeoutExpired as exc:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                    raise PhysicsError('Результат не воспроизводится: эксперимент превысил 10 минут и остановлен.') from exc
            if not (out/'summary.jsonl').is_file():
                detail = (root/'run.log').read_text(encoding='utf-8', errors='replace').strip().splitlines()
                message = detail[-1][-500:] if detail else 'Нет журнала сборки или запуска.'
                raise PhysicsError('Результат не воспроизводится: эксперимент не завершён. '+message)
            result = analyze(out)
            if process.returncode and result['status'] == 'reproducible':
                raise PhysicsError('Результат не воспроизводится: процесс завершился с ошибкой.')
            from .sitl_pair_report import render, technical_content
            return {'status': result['status'], 'reason': result['reason'],
                    'html': render(result, include_technical=False), 'technical_html': technical_content(result)}
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise PhysicsError(f'Результат не воспроизводится: {exc}') from exc
    finally:
        aircraft_sitl._LOCK.release()

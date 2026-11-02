"""Численное повторение расчёта физики по записанным выходам SITL и шагам времени.

Повторяется только внешняя модель; регулятор Betaflight повторно не исполняется.
Равенство вывода при одинаковом входе не устанавливает причину различий между запусками.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from bfemu import aircraft_sitl
from bfemu.flight_actuators import Actuators, read_curves
from bfemu.flight_physics import Aircraft, PhysicsError, State, step
from tools.sitl_pid_cycle_probe import digest


def replay(text):
    """Восстановить физику без интерполяции, проверяя исходные шаги и задержку команды."""
    try:
        rows = [json.loads(line) for line in text.splitlines() if line.strip()]
        if len(rows) < 2 or rows[0].get('type') != 'header' or rows[0].get('time_basis') != 'host_monotonic':
            raise ValueError('Нужен полный журнал физики с заголовком host_monotonic.')
        header = rows[0]
        aircraft = Aircraft.from_profile(header['profile'])
        actuators = Actuators(read_curves(header['actuator_model']))
        state = State()
        expected_commands = (0., 0., 0., 0.)
        previous_time = 0.
        mismatch_count = 0
        first_mismatch = None
        worst_absolute = 0.
        for index, row in enumerate(rows[1:], 1):
            if row.get('type') != 'sample':
                raise ValueError('Неверный тип записи физики.')
            dt, elapsed = row['dt_s'], row['elapsed_s']
            if (not isinstance(dt, (int, float)) or isinstance(dt, bool) or not math.isfinite(dt)
                    or not 0 < dt < .02 or not isinstance(elapsed, (int, float)) or isinstance(elapsed, bool)
                    or not math.isfinite(elapsed) or elapsed <= previous_time
                    or abs((elapsed-previous_time)-dt) > 1e-9):
                raise ValueError('Неверный или непоследовательный временной шаг.')
            previous_time = elapsed
            commands = tuple(row['applied_commands'])
            if commands != expected_commands:
                raise ValueError('Команда не соответствует предыдущему отсчёту моторов.')
            next_commands = row['next_commands']
            if next_commands is not None:
                if (len(next_commands) != 4 or any(isinstance(x, bool) or not isinstance(x, (int, float))
                        or not math.isfinite(x) or not 0 <= x <= 1 for x in next_commands)):
                    raise ValueError('Недопустимая следующая команда моторов.')
                expected_commands = tuple(next_commands)
            avg_thrust, avg_reaction = actuators.advance(commands, dt)
            state = step(aircraft, state, avg_thrust, avg_reaction, dt)
            observed = row['state']
            fields = [(name, values, observed[name]) for name, values in asdict(state).items()]
            fields += [('thrust_n', actuators.thrust_n, row['thrust_n']),
                       ('reaction_nm', actuators.reaction_nm, row['reaction_nm'])]
            for name, actual, recorded in fields:
                if len(actual) != len(recorded) or any(isinstance(v, bool) or not isinstance(v, (int, float))
                                                       or not math.isfinite(v) for v in recorded):
                    raise ValueError('Некорректное записанное состояние физики.')
                for component, (value, original) in enumerate(zip(actual, recorded, strict=True)):
                    delta = abs(value-original)
                    worst_absolute = max(worst_absolute, delta)
                    if value != original:
                        mismatch_count += 1
                        if first_mismatch is None:
                            first_mismatch = {'sample': index, 'field': name, 'component': component,
                                              'recorded': original, 'replayed': value}
        return {'samples': len(rows)-1, 'mismatched_values': mismatch_count,
                'first_mismatch': first_mismatch, 'max_abs_difference': worst_absolute}
    except (KeyError, TypeError, IndexError) as exc:
        raise ValueError('Журнал физики неполон или имеет неверную структуру.') from exc


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    record = commands.add_parser('record', help='выполнить один реальный SITL и сохранить полные входы физики')
    record.add_argument('--output-dir', type=Path, required=True)
    record.add_argument('--axis', choices=('roll', 'pitch', 'yaw'), default='pitch')
    record.add_argument('--pid', nargs=3, type=int, metavar=('P', 'I', 'D_MAX'), default=[49, 84, 46])
    record.add_argument('--period-ms', choices=(2, 4, 8), type=int, default=4)
    verify = commands.add_parser('verify', help='проверить сохранённый журнал')
    verify.add_argument('file', type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == 'verify':
            report = replay(args.file.read_text(encoding='utf-8'))
            print(json.dumps(report, ensure_ascii=False, allow_nan=False))
            return int(report['mismatched_values'] != 0)
        if any(p not in range(251) for p in args.pid):
            parser.error('Значения PID должны быть от 0 до 250.')
        if os.environ.get('LD_PRELOAD'):
            raise PhysicsError('Отключите внешнее вмешательство LD_PRELOAD.')
        if not aircraft_sitl.sitl_available():
            raise PhysicsError('Сначала соберите штатный Betaflight SITL.')
        profile = json.loads((Path(aircraft_sitl.__file__).parent/'data/aircraft_build03.json').read_text(encoding='utf-8'))
        aircraft_sitl.theoretical_curves(profile)
        pid = dict(zip(('p', 'i', 'd'), args.pid, strict=True))
        target = args.output_dir.resolve()
        target.mkdir(parents=True, exist_ok=False)
        raw, started = [], time.monotonic()
        normal_loop = aircraft_sitl.run_loop
        def capture_loop(*a, **kw):
            count = normal_loop(*a, **kw)
            raw.append(a[4].getvalue())
            return count
        try:
            aircraft_sitl.run_loop = capture_loop
            result = aircraft_sitl.experiment(profile, axis=args.axis, pid=pid, sensor_period_ms=args.period_ms,
                                              motor_mode='poll')
            if len(raw) != 1:
                raise PhysicsError('Не получен полный журнал физики.')
            report = replay(raw[0])
            with (target/'physics.jsonl').open('x', encoding='utf-8') as f:
                f.write(raw[0])
            summary = {'type': 'run', 'ok': True, 'revision': aircraft_sitl.REVISION,
                       'elf_sha256': digest(aircraft_sitl.BINARY), 'axis': args.axis, 'pid': pid,
                       'verified_pid': result['verified_pid'], 'period_ms': args.period_ms,
                       'motor_mode': 'poll', 'diagnostics': result['diagnostics'],
                       'command_window_s': result['command_window_s'], 'replay': report,
                       'wall_seconds': round(time.monotonic()-started, 3)}
            with (target/'summary.json').open('x', encoding='utf-8') as f:
                json.dump(summary, f, ensure_ascii=False, allow_nan=False, indent=2)
            print(json.dumps(summary, ensure_ascii=False, allow_nan=False))
            return int(report['mismatched_values'] != 0)
        except (PhysicsError, OSError, ValueError) as exc:
            failure = {'type': 'run', 'ok': False, 'axis': args.axis, 'pid': pid, 'period_ms': args.period_ms,
                       'error': str(exc), 'wall_seconds': round(time.monotonic()-started, 3)}
            with (target/'summary.json').open('x', encoding='utf-8') as f:
                json.dump(failure, f, ensure_ascii=False, allow_nan=False, indent=2)
            print(json.dumps(failure, ensure_ascii=False, allow_nan=False), file=sys.stderr)
            return 1
        finally:
            aircraft_sitl.run_loop = normal_loop
    except (OSError, ValueError) as exc:
        print(f'Повтор физики: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('Повтор физики прерван; файлы могут быть неполными.', file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())

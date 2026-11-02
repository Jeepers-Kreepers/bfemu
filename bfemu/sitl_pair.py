"""Строгое сравнение записей парного lockstep-эксперимента, не запуск FC.

Все метрики относятся к выбранной модели, а не к настоящему летательному аппарату.
"""
from __future__ import annotations

import gzip
import itertools
import json
import math
from pathlib import Path

FIELDS = ('setpoint_dps', 'gyro_dps', 'p', 'i', 'd', 'f', 'sum')
AXES = {'roll': 0, 'pitch': 1, 'yaw': 2}
# Это фиксированные допуски численной повторяемости, не пределы пригодности полёта.
TOLERANCES = {'gyro_dps': 0.01, 'setpoint_dps': 0.01, 'pid': 0.001,
              'motor': 0.00001, 'position_m': 0.000001, 'velocity_m_s': 0.000001,
              'orientation': 0.00000001, 'angular_velocity_rad_s': 0.0000001,
              'thrust_n': 0.000001, 'reaction_nm': 0.00000001}


def _lines(path):
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rt', encoding='utf-8') as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def _finite(values):
    if not all(isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v) for v in values):
        raise ValueError('Некорректное числовое значение в трассе.')


def _read_run(directory, item, header):
    filename = Path(item['file'])
    if str(filename) != filename.name:
        raise ValueError('Некорректное имя трассы.')
    axis = AXES[header['axis']]
    events = item['events']
    start, end = events['command_start'], events['command_end']
    if end-start != 300 or events['bench_release'] != events['armed_at']+1:
        raise ValueError('Неизвестный сценарий или границы команды.')
    frames, initial, previous_clock, count = [], None, None, 0
    path = directory/filename
    if not path.exists():
        path = directory/(filename.name+'.gz')
    for count, row in enumerate(_lines(path), 1):
        if (row['sequence'] != count or row['read_sequence'] != count or row['mask'] != 15 or
                row['pid_cycles'] != 32 or len(row['pid_samples']) != 32 or
                abs(row['input_time_s']-count*.004) > 1e-9 or abs(row['fdm_time_s']-count*.004) > 1e-9 or
                (previous_clock is not None and row['sim_us']-previous_clock != 4000)):
            raise ValueError('Нарушена последовательность или время кадров.')
        previous_clock = row['sim_us']
        if count == start-1:
            initial = {key: row[key] for key in ('state', 'motors', 'thrust_n', 'reaction_nm', 'applied_commands')}
            initial['pid_axes'] = row['pid_samples'][-1]['axes']
        if start <= count < end:
            if not row['armed'] or row['bench_held'] or len(row['motors']) != 4:
                raise ValueError('Некорректное состояние в окне команды.')
            _finite(row['motors'])
            samples = []
            last_time = None
            for sample_index, sample in enumerate(row['pid_samples']):
                offset = (row['sim_us'] - sample['time_us']) % (1 << 32)
                if offset != 125*(31-sample_index) or (last_time is not None and (sample['time_us']-last_time) % (1 << 32) != 125):
                    raise ValueError('Нарушено время PID-цикла.')
                last_time = sample['time_us']
                if len(sample['axes']) != 3:
                    raise ValueError('В PID-цикле отсутствует ось.')
                axes = [tuple(a[k] for k in FIELDS) for a in sample['axes']]
                _finite(v for a in axes for v in a)
                samples.append((offset, axes))
            frames.append({'channels': row['channels'], 'motors': row['motors'],
                           'state': row['state'], 'samples': samples})
    if count != item['frames'] or initial is None or len(frames) != 300:
        raise ValueError('Трасса неполная.')
    return {'condition': item['condition'], 'cycle': item['cycle'], 'initial': initial,
            'frames': frames, 'axis': axis}


def _initial_matches(a, b):
    for field in ('position_m', 'velocity_m_s', 'orientation', 'angular_velocity_rad_s'):
        x, y = a['state'][field], b['state'][field]
        _finite([*x, *y])
        if len(x) != len(y) or any(abs(u-v) > TOLERANCES[field] for u, v in zip(x, y, strict=True)):
            return False
    for field, tolerance in (('motors', 'motor'), ('applied_commands', 'motor'),
                             ('thrust_n', 'thrust_n'), ('reaction_nm', 'reaction_nm')):
        x, y = a[field], b[field]
        _finite([*x, *y])
        if len(x) != len(y) or any(abs(u-v) > TOLERANCES[tolerance] for u, v in zip(x, y, strict=True)):
            return False
    for aa, bb in zip(a['pid_axes'], b['pid_axes'], strict=True):
        for field in FIELDS:
            _finite([aa[field], bb[field]])
            tolerance = TOLERANCES[field] if field in ('setpoint_dps', 'gyro_dps') else TOLERANCES['pid']
            if abs(aa[field]-bb[field]) > tolerance:
                return False
    return True


def _spread(a, b):
    maximum = {key: 0. for key in ('gyro_dps', 'setpoint_dps', 'pid', 'motor',
                                  'position_m', 'velocity_m_s', 'orientation', 'angular_velocity_rad_s')}
    squared, n = 0., 0
    for x, y in zip(a['frames'], b['frames'], strict=True):
        if x['channels'] != y['channels']:
            raise ValueError('Команды в повторах отличаются.')
        maximum['motor'] = max(maximum['motor'], *(abs(u-v) for u, v in zip(x['motors'], y['motors'], strict=True)))
        for field in ('position_m', 'velocity_m_s', 'orientation', 'angular_velocity_rad_s'):
            xx, yy = x['state'][field], y['state'][field]
            _finite([*xx, *yy])
            maximum[field] = max(maximum[field], *(abs(u-v) for u, v in zip(xx, yy, strict=True)))
        for (tx, sx), (ty, sy) in zip(x['samples'], y['samples'], strict=True):
            if tx != ty:
                raise ValueError('PID-циклы по фазе внутри кадра не совпадают.')
            for xx, yy in zip(sx, sy, strict=True):
                maximum['setpoint_dps'] = max(maximum['setpoint_dps'], abs(xx[0]-yy[0]))
                maximum['gyro_dps'] = max(maximum['gyro_dps'], abs(xx[1]-yy[1]))
                maximum['pid'] = max(maximum['pid'], *(abs(u-v) for u, v in zip(xx[2:], yy[2:], strict=True)))
            d = sx[a['axis']][1] - sy[a['axis']][1]
            squared += d*d
            n += 1
    maximum['rms_selected_gyro_dps'] = math.sqrt(squared/n)
    return maximum


def _series(run):
    values = []
    for frame_index, frame in enumerate(run['frames']):
        for offset, axes in frame['samples']:
            t = frame_index * .004 + (4000-offset)/1e6
            values.append((t, *axes[run['axis']]))
    return values


def _metrics(run):
    series = _series(run)
    target = sum(row[1] for row in series[-800:])/800  # последние 100 мс команды
    errors = [row[1]-row[2] for row in series]
    integral = sum(abs(x)*.000125 for x in errors)
    peak = None
    if abs(target) < 1e-9:
        overshoot, settling = None, None
    else:
        sign = 1 if target > 0 else -1
        peak = max(series, key=lambda row: sign*row[2])
        overshoot = max(0., (sign*peak[2]-abs(target))/abs(target)*100)
        tolerance = .05*abs(target)
        outside = [i for i, row in enumerate(series) if abs(row[2]-target) > tolerance]
        settling = (series[outside[-1]+1][0] if outside and outside[-1]+1 < len(series)
                    else (series[0][0] if not outside else None))
    saturated = sum(any(m <= 0 or m >= 1 for m in frame['motors']) for frame in run['frames'])
    return {'target_dps': target, 'overshoot_percent': overshoot, 'settling_s': settling,
            'integral_abs_error_deg': integral, 'saturated_frames': saturated,
            'saturated_time_s': saturated*.004, 'peak_sample': peak}


def analyze(directory):
    directory = Path(directory)
    summary = list(_lines(directory/'summary.jsonl'))
    if len(summary) < 2 or summary[0].get('type') != 'header' or summary[-1].get('type') != 'summary':
        raise ValueError('Нет полного журнала эксперимента.')
    header = summary[0]
    if (header.get('scenario') != 'flight' or not header.get('prearm_bench') or header.get('axis') not in AXES or
            header.get('repeats', 0) < 2 or not isinstance(header.get('pid'), dict) or
            not isinstance(header.get('compare_pid'), dict) or
            set(header['pid']) != {'p', 'i', 'd'} or set(header['compare_pid']) != {'p', 'i', 'd'} or
            sum(header['pid'][k] != header['compare_pid'][k] for k in ('p', 'i', 'd')) != 1):
        raise ValueError('Журнал не описывает допустимый парный опыт.')
    rows = [r for r in summary if r.get('type') == 'run']
    result = {'status': 'not_reproducible', 'reason': '', 'header': header, 'tolerances': TOLERANCES,
              'failures': [r for r in rows if not r.get('ok')]}
    if len(rows) != header['repeats']*2 or summary[-1].get('not_completed') != 0 or result['failures']:
        result['reason'] = 'Не все запуски завершились; сравнение настройки недоступно.'
        return result
    runs = {'before': [], 'after': []}
    for row in rows:
        condition = row.get('condition')
        if condition not in runs or row.get('verified_pid') != {'axis': header['axis'], 'values': header['pid' if condition == 'before' else 'compare_pid']}:
            raise ValueError('Не подтверждены заданные настройки PID.')
        try:
            runs[condition].append(_read_run(directory, row, header))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            result['reason'] = f'Трасса не прошла проверку: {exc}'
            return result
    for condition in runs:
        if sorted(r['cycle'] for r in runs[condition]) != list(range(1, header['repeats']+1)):
            raise ValueError('Отсутствуют или повторяются номера опытов.')
    if not all(_initial_matches(runs['before'][0]['initial'], run['initial'])
               for run in runs['before']+runs['after']):
        result['reason'] = 'Начальное состояние перед командой отличается; сравнение настройки недоступно.'
        return result
    # Команды должны совпадать не только внутри повторов, но и между настройками.
    if any(x['channels'] != y['channels'] for x, y in zip(runs['before'][0]['frames'], runs['after'][0]['frames'], strict=True)):
        result['reason'] = 'Сценарии команд отличаются; сравнение настройки недоступно.'
        return result
    if any([u[0] for u in x['samples']] != [u[0] for u in y['samples']]
           for x, y in zip(runs['before'][0]['frames'], runs['after'][0]['frames'], strict=True)):
        result['reason'] = 'PID-циклы двух настроек имеют разную фазу; сравнение недоступно.'
        return result
    spreads = {}
    for condition, group in runs.items():
        try:
            pairs = [_spread(a, b) for a, b in itertools.combinations(group, 2)]
        except (ValueError, KeyError, TypeError) as exc:
            result['reason'] = f'Повторы не сопоставимы: {exc}'
            return result
        spreads[condition] = {key: max(p[key] for p in pairs) for key in pairs[0]}
    result['spread'] = spreads
    if any(spread[key] > TOLERANCES[key] for spread in spreads.values() for key in ('gyro_dps', 'setpoint_dps', 'pid', 'motor', 'position_m', 'velocity_m_s', 'orientation', 'angular_velocity_rad_s')):
        result['reason'] = 'Разброс повторов превышает установленный допуск; эффект настройки не определяется.'
        return result
    result['status'], result['reason'] = 'reproducible', 'Повторы в пределах установленных допусков.'
    result['metrics'] = {name: _metrics(group[0]) for name, group in runs.items()}
    result['curves'] = {name: _series(group[0]) for name, group in runs.items()}
    return result

#!/usr/bin/env python3
"""Сравнение полных кадров исследовательского SITL, без воспроизведения трассы."""
import argparse
import gzip
import itertools
import json
import math
from pathlib import Path

AXES = {'roll': 0, 'pitch': 1, 'yaw': 2}


def read_jsonl(path):
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rt', encoding='utf-8') as source:
        return [json.loads(line) for line in source if line.strip()]


def load_series(directory):
    directory = Path(directory)
    summary = read_jsonl(directory/'summary.jsonl')
    if not summary or summary[0]['type'] != 'header':
        raise ValueError('Нет заголовка эксперимента.')
    header = summary[0]
    if header['scenario'] != 'flight':
        raise ValueError('Для сравнения требуется сценарий flight.')
    runs, failures = [], []
    for item in summary[1:]:
        if item['type'] != 'run':
            continue
        if not item['ok']:
            failures.append({'cycle': item['cycle'], 'error': item['error'],
                             'error_stage': item.get('error_stage')})
            continue
        filename = Path(item['file'])
        if filename.name != str(filename):
            raise ValueError('Некорректное имя файла трассы.')
        path = directory/filename
        if not path.exists():
            path = directory/(filename.name+'.gz')
        rows = read_jsonl(path)
        if len(rows) != item['frames']:
            raise ValueError('Трасса неполная.')
        for index, row in enumerate(rows, 1):
            if (row['sequence'] != index or row['read_sequence'] != index or
                    row['mask'] != 15 or row['pid_cycles'] != 32 or len(row['motors']) != 4):
                raise ValueError('Нарушен протокол кадров.')
            if index > 1 and row['sim_us']-rows[index-2]['sim_us'] != 4000:
                raise ValueError('Нарушен шаг моделируемого времени.')
        axis = header.get('axis', item['verified_pid']['axis'])
        if axis not in AXES or item['verified_pid']['axis'] != axis:
            raise ValueError('Ось эксперимента не совпадает с подтверждёнными PID.')
        events = item['events']
        start, end = events['command_start']-1, events['disarm_request']-1
        if not 0 <= start < end < len(rows):
            raise ValueError('Отсутствует полный интервал команды.')
        segment = rows[start:end]
        if not all(r['armed'] for r in segment):
            raise ValueError('Дизарм до окончания интервала сравнения.')
        runs.append({'cycle': item['cycle'], 'pid': item['verified_pid'], 'events': events, 'axis': axis,
                     'segment': segment})
    return header, runs, failures


def snapshot(row, axis, field):
    # Ранние архивы содержали только снимки PITCH; новые содержат все оси.
    vector = 'rc_axes' if field == 'rc' else 'setpoint_axes_dps'
    if vector in row:
        if len(row[vector]) != 3:
            raise ValueError('Неверный размер снимка осей.')
        return row[vector][AXES[axis]]
    if axis != 'pitch':
        raise ValueError('В архиве нет снимка выбранной оси.')
    return row['rc_pitch' if field == 'rc' else 'setpoint_pitch_dps']


def compare(first, second):
    axis = first.get('axis', 'pitch')
    if axis not in AXES or second.get('axis', 'pitch') != axis:
        raise ValueError('Нельзя сравнивать разные оси.')
    a, b = first['segment'], second['segment']
    if not a or len(a) != len(b) or [r['channels'] for r in a] != [r['channels'] for r in b]:
        raise ValueError('Интервалы или команды не совпадают.')
    differences = [math.degrees(x['state']['angular_velocity_rad_s'][AXES[axis]] -
                                y['state']['angular_velocity_rad_s'][AXES[axis]]) for x, y in zip(a, b, strict=True)]
    if not all(math.isfinite(v) for v in differences):
        raise ValueError('Неконечное значение угловой скорости.')
    result = {'samples': len(differences),
            'rms_dps': math.sqrt(sum(v*v for v in differences)/len(differences)),
            'max_abs_dps': max(map(abs, differences)),
            'motor_frames_differ': sum(x['motors'] != y['motors'] for x, y in zip(a, b, strict=True)),
            'max_motor_difference': max(abs(u-v) for x, y in zip(a, b, strict=True)
                                        for u, v in zip(x['motors'], y['motors'], strict=True)),
            'rc_frames_differ': sum(snapshot(x, axis, 'rc') != snapshot(y, axis, 'rc')
                                    for x, y in zip(a, b, strict=True)),
            'setpoint_frames_differ': sum(snapshot(x, axis, 'setpoint') != snapshot(y, axis, 'setpoint')
                                          for x, y in zip(a, b, strict=True))}
    if all('pid_axes' in r and 'gyro_axes_dps' in r for r in (*a, *b)):
        index = AXES[axis]
        result['gyro_max_abs_dps'] = max(abs(x['gyro_axes_dps'][index] - y['gyro_axes_dps'][index])
                                         for x, y in zip(a, b, strict=True))
        result['pid_term_max_abs_difference'] = {
            term: max(abs(x['pid_axes'][index][term] - y['pid_axes'][index][term])
                      for x, y in zip(a, b, strict=True)) for term in ('p', 'i', 'd', 'f', 'sum')}
    return result


def analyze(directories):
    all_runs, groups = [], []
    for directory in directories:
        header, runs, failures = load_series(directory)
        results = []
        for run in runs:
            segment, events = run['segment'], run['events']
            command_count = events['command_end']-events['command_start']
            if not 0 < command_count <= len(segment):
                raise ValueError('Неверная продолжительность команды.')
            results.append({'cycle': run['cycle'], 'verified_pid': run['pid'],
                            'armed_at': events['armed_at'], 'axis': run['axis'],
                            'peak_command_dps': max(abs(math.degrees(r['state']['angular_velocity_rad_s'][AXES[run['axis']]]))
                                                    for r in segment[:command_count]),
                            'motor_at_upper_limit_frames': sum(any(m >= 1 for m in r['motors'])
                                                               for r in segment[:command_count]),
                            'motor_at_lower_limit_frames': sum(any(m <= 0 for m in r['motors'])
                                                               for r in segment[:command_count])})
            all_runs.append((str(directory), run))
        groups.append({'directory': str(directory), 'header_pid': header.get('pid'),
                       'results': results, 'failures': failures})
    pairs = []
    for (directory_a, a), (directory_b, b) in itertools.combinations(all_runs, 2):
        if a['axis'] == b['axis']:
            pairs.append({'a': [directory_a, a['cycle']], 'b': [directory_b, b['cycle']],
                          'axis': a['axis'], **compare(a, b)})
    return {'groups': groups, 'pairs': pairs}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', nargs='+', help='Каталоги с summary.jsonl и кадровыми трассами')
    args = parser.parse_args()
    print(json.dumps(analyze(args.directories), ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()

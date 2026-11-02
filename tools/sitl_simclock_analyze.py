#!/usr/bin/env python3
"""Сравнение полных кадров исследовательского SITL, без воспроизведения трассы."""
import argparse
import gzip
import itertools
import json
import math
from pathlib import Path


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
            failures.append({'cycle': item['cycle'], 'error': item['error']})
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
        events = item['events']
        start, end = events['command_start']-1, events['disarm_request']-1
        if not 0 <= start < end < len(rows):
            raise ValueError('Отсутствует полный интервал команды.')
        segment = rows[start:end]
        if not all(r['armed'] for r in segment):
            raise ValueError('Дизарм до окончания интервала сравнения.')
        runs.append({'cycle': item['cycle'], 'pid': item['verified_pid'], 'events': events,
                     'segment': segment})
    return header, runs, failures


def compare(first, second):
    a, b = first['segment'], second['segment']
    if len(a) != len(b) or [r['channels'] for r in a] != [r['channels'] for r in b]:
        raise ValueError('Интервалы или команды не совпадают.')
    differences = [math.degrees(x['state']['angular_velocity_rad_s'][1] -
                                y['state']['angular_velocity_rad_s'][1]) for x, y in zip(a, b, strict=True)]
    if not all(math.isfinite(v) for v in differences):
        raise ValueError('Неконечное значение угловой скорости.')
    return {'samples': len(differences),
            'rms_dps': math.sqrt(sum(v*v for v in differences)/len(differences)),
            'max_abs_dps': max(map(abs, differences)),
            'motor_frames_differ': sum(x['motors'] != y['motors'] for x, y in zip(a, b, strict=True)),
            'rc_frames_differ': sum(x['rc_pitch'] != y['rc_pitch'] for x, y in zip(a, b, strict=True)),
            'setpoint_frames_differ': sum(x['setpoint_pitch_dps'] != y['setpoint_pitch_dps']
                                           for x, y in zip(a, b, strict=True))}


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
                            'armed_at': events['armed_at'],
                            'peak_command_dps': max(abs(math.degrees(r['state']['angular_velocity_rad_s'][1]))
                                                    for r in segment[:command_count]),
                            'motor_at_upper_limit_frames': sum(any(m >= 1 for m in r['motors'])
                                                               for r in segment[:command_count])})
            all_runs.append((str(directory), run))
        groups.append({'directory': str(directory), 'header_pid': header.get('pid'),
                       'results': results, 'failures': failures})
    pairs = []
    for (directory_a, a), (directory_b, b) in itertools.combinations(all_runs, 2):
        pairs.append({'a': [directory_a, a['cycle']], 'b': [directory_b, b['cycle']], **compare(a, b)})
    return {'groups': groups, 'pairs': pairs}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', nargs='+', help='Каталоги с summary.jsonl и кадровыми трассами')
    args = parser.parse_args()
    print(json.dumps(analyze(args.directories), ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()

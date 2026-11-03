"""Тесты проверяют протокол и запрет интерпретации недостоверного сравнения."""
import json
import math

import pytest

from bfemu import sitl_pair
from tools import sitl_lockstep_probe as probe
from bfemu.sitl_pair_report import render


def message(seq=3, clock=12000, count=32, input_time=.012, sample_time=None, bad_value=None):
    header = probe.OUTPUT.pack(seq, seq, 15, count, 1, 0, clock, .1, .2, .3, .4,
                               1, 125, 11000, 12000, 1500, 1500, 1500, 0, 75, 0,
                               *([0.] * 18), 125, 125, 125, .000125, 1, 2, 3, input_time)
    result = []
    for i in range(32):
        values = [0.] * 21
        if i == 2 and bad_value is not None:
            values[3] = bad_value
        t = (clock-125*(31-i)) if i != 2 or sample_time is None else sample_time
        result.append(probe.PID_SAMPLE.pack(t, *values))
    return header + b''.join(result)


def test_lockstep_ack_validates_time_sequence_and_all_pid_samples():
    data = message()
    record = probe.decode_response(data, 3, (3, 0, 1, 2), 8000)
    assert len(record['pid_samples']) == 32
    assert record['pid_samples'][0]['time_us'] == 8125
    assert record['pid_samples'][-1]['axes'][1] == dict.fromkeys(sitl_pair.FIELDS, 0.)
    for invalid in (message(seq=2), message(count=31), message(input_time=.013),
                    message(sample_time=12), message(bad_value=math.nan), data[:-1]):
        with pytest.raises(probe.PhysicsError):
            probe.decode_response(invalid, 3, (3, 0, 1, 2), 8000)


def test_pair_cli_requires_one_change_two_repeats_and_bench():
    base = ['--output-dir', 'out', '--scenario', 'flight', '--prearm-bench', '--pid', '49', '84', '46']
    assert probe.parse_args([*base, '--compare-pid', '55', '84', '46']).compare_pid == [55, 84, 46]
    for extra in (['--compare-pid', '49', '84', '46'], ['--compare-pid', '55', '85', '46'],
                  ['--compare-pid', '55', '84', '46', '--repeats', '1']):
        with pytest.raises(SystemExit):
            probe.parse_args(base + extra)
    with pytest.raises(SystemExit):
        probe.parse_args(['--output-dir', 'out', '--scenario', 'flight', '--compare-pid', '55', '84', '46'])


def artificial_run(condition='before', cycle=1, gyro=75):
    axis = dict.fromkeys(sitl_pair.FIELDS, 0.)
    initial = {'state': {'position_m': [0, 0, 0], 'velocity_m_s': [0, 0, 0],
                         'orientation': [1, 0, 0, 0], 'angular_velocity_rad_s': [0, 0, 0]},
               'motors': [0]*4, 'applied_commands': [0]*4, 'thrust_n': [0]*4, 'reaction_nm': [0]*4,
               'pid_axes': [axis.copy() for _ in range(3)]}
    values = (75., gyro, 1., 2., 3., 4., 10.)
    samples = [(125*(31-i), [values, values, values]) for i in range(32)]
    frame = {'channels': [1500]*4, 'motors': [.5]*4, 'state': initial['state'], 'samples': samples}
    return {'condition': condition, 'cycle': cycle, 'initial': initial, 'frames': [frame]*300, 'axis': 1}


def pair_summary(tmp_path):
    h = {'type': 'header', 'scenario': 'flight', 'prearm_bench': True, 'axis': 'pitch', 'repeats': 2,
         'pid': {'p': 49, 'i': 84, 'd': 46}, 'compare_pid': {'p': 55, 'i': 84, 'd': 46},
         'revision': 'rev', 'simclock_elf_sha256': 'hash'}
    rows = [{'type': 'run', 'cycle': i, 'condition': c, 'ok': True,
             'verified_pid': {'axis': 'pitch', 'values': h['pid' if c == 'before' else 'compare_pid']}}
            for i in (1, 2) for c in ('before', 'after')]
    p = tmp_path/'summary.jsonl'
    def write():
        p.write_text('\n'.join(json.dumps(x) for x in [h, *rows, {'type': 'summary', 'not_completed': 0}]))
    write()
    return h, rows, write


def test_repeatability_gate_and_metrics(tmp_path, monkeypatch):
    pair_summary(tmp_path)
    monkeypatch.setattr(sitl_pair, '_read_run', lambda d, r, h: artificial_run(r['condition'], r['cycle']))
    report = sitl_pair.analyze(tmp_path)
    assert report['status'] == 'reproducible'
    assert report['metrics']['before']['overshoot_percent'] == 0
    assert report['metrics']['before']['integral_abs_error_deg'] == 0
    assert len(report['curves']['before']) == 9600
    assert 'Betaflight' in render(report)


def test_failed_runs_and_large_spread_never_produce_effect(tmp_path, monkeypatch):
    h, rows, write = pair_summary(tmp_path)
    rows[0].update(ok=False, error='timeout')
    write()
    report = sitl_pair.analyze(tmp_path)
    assert report['status'] == 'not_reproducible' and 'metrics' not in report
    assert 'Результат не воспроизводится' in render(report)
    rows[0]['ok'] = True
    write()
    monkeypatch.setattr(sitl_pair, '_read_run', lambda d, r, h: artificial_run(r['condition'], r['cycle'],
                                      76 if r['condition'] == 'before' and r['cycle'] == 2 else 75))
    report = sitl_pair.analyze(tmp_path)
    assert report['status'] == 'not_reproducible' and 'metrics' not in report
    assert report['spread']['before']['gyro_dps'] == 1
    h['compare_pid']['i'] += 1
    write()
    with pytest.raises(ValueError):
        sitl_pair.analyze(tmp_path)


def test_initial_condition_mismatch_blocks_comparison(tmp_path, monkeypatch):
    pair_summary(tmp_path)
    def load(d, row, header):
        run = artificial_run(row['condition'], row['cycle'])
        if row['condition'] == 'after':
            run['initial']['state']['position_m'] = [0, 0, 1]
        return run
    monkeypatch.setattr(sitl_pair, '_read_run', load)
    report = sitl_pair.analyze(tmp_path)
    assert report['status'] == 'not_reproducible' and 'Начальное состояние' in report['reason']
    assert 'metrics' not in report


def test_web_pair_validates_before_build_and_respects_existing_lock():
    from bfemu import aircraft_sitl, sitl_pair_web
    profile = json.loads((sitl_pair_web.ROOT/'bfemu/data/aircraft_build03.json').read_text())
    before = {'p': 49, 'i': 84, 'd': 46}
    with pytest.raises(sitl_pair_web.PhysicsError, match='ровно один'):
        sitl_pair_web.experiment('pitch', before, before, profile)
    with pytest.raises(sitl_pair_web.PhysicsError, match='ровно один'):
        sitl_pair_web.experiment('pitch', before, {'p': 55, 'i': 85, 'd': 46}, profile)
    with pytest.raises(sitl_pair_web.PhysicsError, match='целыми'):
        sitl_pair_web.experiment('pitch', before, {'p': True, 'i': 84, 'd': 46}, profile)
    if aircraft_sitl.sitl_available():
        assert aircraft_sitl._LOCK.acquire(blocking=False)
        try:
            with pytest.raises(sitl_pair_web.PhysicsError, match='уже выполняется'):
                sitl_pair_web.experiment('pitch', before, {'p': 55, 'i': 84, 'd': 46}, profile)
        finally:
            aircraft_sitl._LOCK.release()


def test_archived_pair_uses_real_recorded_pid_cycles_and_verified_sources():
    import hashlib
    from pathlib import Path

    directory = Path(__file__).resolve().parents[1]/'docs/research/sitl-lockstep-pair-p49-p55-v2'
    header = json.loads((directory/'summary.jsonl').read_text().splitlines()[0])
    assert header['driver_sha256'] == hashlib.sha256((directory/'sitl_lockstep_probe.py').read_bytes()).hexdigest()
    assert header['profile_sha256'] == hashlib.sha256((directory/'input-profile.json').read_bytes()).hexdigest()
    for name, digest in header['includes_sha256'].items():
        assert hashlib.sha256((directory/name).read_bytes()).hexdigest() == digest
    report = sitl_pair.analyze(directory)
    assert report['status'] == 'reproducible'
    assert report['metrics']['before']['target_dps'] == pytest.approx(75)
    assert report['metrics']['before']['overshoot_percent'] == pytest.approx(25.894795735677086)
    assert report['metrics']['after']['integral_abs_error_deg'] == pytest.approx(2.8394511329313885)
    assert all(spread['gyro_dps'] == 0 and spread['pid'] == 0 for spread in report['spread'].values())


@pytest.mark.parametrize(('name', 'before', 'after'), [
    ('roll-d40-d50-v1', 27.277882893880207, 26.463948567708336),
    ('yaw-i80-i40-v1', 100.11378987630208, 73.99082438151041),
])
def test_additional_axis_archives_repeat_and_change_pid(name, before, after):
    import hashlib
    from pathlib import Path

    directory = Path(__file__).resolve().parents[1]/f'docs/research/sitl-lockstep-pair-{name}'
    header = json.loads((directory/'summary.jsonl').read_text().splitlines()[0])
    assert hashlib.sha256((directory/'input-profile.json').read_bytes()).hexdigest() == header['profile_sha256']
    assert hashlib.sha256((directory/'sitl_lockstep_probe.py').read_bytes()).hexdigest() == header['driver_sha256']
    for filename, digest in header['includes_sha256'].items():
        assert hashlib.sha256((directory/filename).read_bytes()).hexdigest() == digest
    report = sitl_pair.analyze(directory)
    assert report['status'] == 'reproducible'
    assert report['metrics']['before']['overshoot_percent'] == pytest.approx(before)
    assert report['metrics']['after']['overshoot_percent'] == pytest.approx(after)


def test_technical_details_are_at_the_end_of_download_and_separate_from_embedded_report(tmp_path, monkeypatch):
    from bfemu.sitl_pair_report import technical_content

    pair_summary(tmp_path)
    monkeypatch.setattr(sitl_pair, '_read_run', lambda d, r, h: artificial_run(r['condition'], r['cycle']))
    report = sitl_pair.analyze(tmp_path)
    embedded, complete, technical = render(report, include_technical=False), render(report), technical_content(report)
    phrase = 'Это не MSP-опрос и не данные Blackbox.'
    assert phrase not in embedded and phrase in technical and phrase in complete
    assert 'Каждая колонка показывает различия между повторами одной настройки' in embedded
    assert 'Технические сведения' in complete and complete.index('Технические сведения') > complete.index('Телеметрия регулятора')
    assert technical in complete and 'hash' in technical and 'rev' in technical
    assert 'PID-составляющие' not in embedded or 'внутренних единицах FC' in embedded


def test_web_experiment_returns_report_and_separate_technical_details(tmp_path, monkeypatch):
    from bfemu import aircraft_sitl, sitl_pair_web

    pair_summary(tmp_path)
    monkeypatch.setattr(sitl_pair, '_read_run', lambda d, r, h: artificial_run(r['condition'], r['cycle']))
    result = sitl_pair.analyze(tmp_path)
    result['header']['revision'] = '<script>bad</script>'
    monkeypatch.setattr(sitl_pair_web, 'analyze', lambda path: result)
    monkeypatch.setattr(aircraft_sitl, 'sitl_available', lambda: True)
    class FakeProcess:
        returncode = 0
        def __init__(self, command, **kwargs):
            from pathlib import Path
            output = Path(command[command.index('--output-dir')+1])
            output.mkdir()
            (output/'summary.jsonl').write_text('{}')
        def wait(self, timeout):
            return 0
    monkeypatch.setattr(sitl_pair_web.subprocess, 'Popen', FakeProcess)
    profile = json.loads((sitl_pair_web.ROOT/'bfemu/data/aircraft_build03.json').read_text())
    response = sitl_pair_web.experiment('pitch', {'p': 49, 'i': 84, 'd': 46},
                                        {'p': 55, 'i': 84, 'd': 46}, profile)
    assert response['status'] == 'reproducible'
    assert 'Это не MSP-опрос и не данные Blackbox.' not in response['html']
    assert 'Это не MSP-опрос и не данные Blackbox.' in response['technical_html']
    assert '<script>bad</script>' not in response['technical_html']
    assert '&lt;script&gt;bad&lt;/script&gt;' in response['technical_html']


def test_peak_markers_match_described_times_on_every_telemetry_graph(tmp_path, monkeypatch):
    import re
    from xml.etree import ElementTree

    pair_summary(tmp_path)
    monkeypatch.setattr(sitl_pair, '_read_run', lambda d, r, h: artificial_run(r['condition'], r['cycle']))
    report = sitl_pair.analyze(tmp_path)
    for name, time_s in (('before', .072), ('after', .064)):
        peak = report['metrics'][name]['peak_sample']
        report['metrics'][name]['peak_sample'] = (time_s, *peak[1:])
    rendered = render(report, include_technical=False)
    assert 'До: через 0.0720 с' in rendered
    assert 'После: через 0.0640 с' in rendered
    graphs = re.findall(r'<svg\b.*?</svg>', rendered, flags=re.DOTALL)
    assert len(graphs) == 5
    for svg in graphs:
        root = ElementTree.fromstring(svg)
        lines = root.findall(".//line[@class='peak-marker']")
        assert len(lines) == 2
        for line, name, time_s, offset in zip(lines, ('до', 'после'), (.072, .064), ('0', '5'), strict=True):
            assert line.attrib['data-condition'] == name
            assert float(line.attrib['x1']) == pytest.approx(75+910*time_s/1.2, abs=.01)
            assert line.attrib['x1'] == line.attrib['x2']
            assert line.attrib['stroke-dasharray'] == '5 5'
            assert line.attrib['stroke-dashoffset'] == offset
    assert rendered.count('Пик до · 0.0720 с') == 5
    assert rendered.count('Пик после · 0.0640 с') == 5


def test_peak_markers_handle_overlapping_and_undefined_peaks(tmp_path, monkeypatch):
    import re
    from xml.etree import ElementTree

    pair_summary(tmp_path)
    monkeypatch.setattr(sitl_pair, '_read_run', lambda d, r, h: artificial_run(r['condition'], r['cycle']))
    report = sitl_pair.analyze(tmp_path)
    rendered = render(report, include_technical=False)
    lines = ElementTree.fromstring(re.search(r'<svg\b.*?</svg>', rendered, re.DOTALL).group()).findall(".//line[@class='peak-marker']")
    assert len(lines) == 2 and lines[0].attrib['x1'] == lines[1].attrib['x1']
    assert lines[0].attrib['stroke'] != lines[1].attrib['stroke']
    assert [line.attrib['stroke-dashoffset'] for line in lines] == ['0', '5']
    report['metrics']['after']['peak_sample'] = None
    rendered = render(report, include_technical=False)
    assert 'После: задание близко к нулю' in rendered
    assert rendered.count('class="peak-marker"') == 5
    assert 'Пик после ·' not in rendered
    report['metrics']['before']['peak_sample'] = None
    rendered = render(report, include_technical=False)
    assert 'class="peak-marker"' not in rendered


@pytest.mark.parametrize('name', ['p49-p55-v2', 'yaw-i80-i40-v1'])
def test_peak_markers_use_archived_pid_sample_timestamps(name):
    import re
    from pathlib import Path
    from xml.etree import ElementTree

    directory = Path(__file__).resolve().parents[1]/f'docs/research/sitl-lockstep-pair-{name}'
    report = sitl_pair.analyze(directory)
    html = render(report)
    graphs = re.findall(r'<svg\b.*?</svg>', html, flags=re.DOTALL)
    assert len(graphs) == 5
    for svg in graphs:
        lines = ElementTree.fromstring(svg).findall(".//line[@class='peak-marker']")
        assert len(lines) == 2
        for line, condition in zip(lines, ('before', 'after'), strict=True):
            peak_time = report['metrics'][condition]['peak_sample'][0]
            end_time = report['curves'][condition][-1][0]
            assert float(line.attrib['x1']) == pytest.approx(75+910*peak_time/end_time, abs=.01)
            assert f'{peak_time:.4f}' in line.find('title').text

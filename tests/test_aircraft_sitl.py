"""Теоретический 7-дюймовый привод и изолированный эксперимент."""
import os

import pytest

from bfemu.aircraft_sitl import experiment, theoretical_curves
from bfemu.flight_actuators import read_curves
from bfemu.flight_physics import PhysicsError
from tests.test_flight_actuators import aircraft_profile


def build03():
    profile = aircraft_profile()
    v = profile['values']
    v.update({'mass_g': '931', 'inertia_xx': '0.00711', 'inertia_yy': '0.00754', 'inertia_zz': '0.01362'})
    for i, (x, y, direction) in enumerate(((-143, 143, 'cw'), (143, 143, 'ccw'),
                                            (-143, -143, 'ccw'), (143, -143, 'cw')), 1):
        for name, value in {'x': x, 'y': y, 'z': -17.4, 'direction': direction, 'model': '2807SL 1300KV',
                            'prop': '7040 × 3', 'max_thrust': 19.6, 'rise_ms': 20, 'fall_ms': 35}.items():
            v[f'motor_{i}_{name}'] = str(value)
    return profile


def test_curves_use_saved_model_parameters_and_direction():
    profile = build03()
    model = theoretical_curves(profile)
    curves = read_curves(model)
    assert curves[0].target(.5) == pytest.approx((4.9, -.0784))
    assert curves[1].target(.5) == pytest.approx((4.9, .0784))
    assert curves[0].time_up_s == .02
    assert curves[0].time_down_s == .035
    assert 'скорость пропорциональна команде' in model['provenance']
    profile['values']['motor_1_max_thrust'] = '10'
    profile['values']['motor_1_rise_ms'] = '40'
    modified = read_curves(theoretical_curves(profile))
    assert modified[0].target(.5)[0] == pytest.approx(2.5)
    assert modified[0].time_up_s == .04
    assert modified[1] == curves[1]


def test_refuses_unknown_hardware_or_missing_physical_parameters():
    profile = build03()
    profile['values']['motor_1_prop'] = 'другой винт'
    with pytest.raises(PhysicsError, match='только 2807SL'):
        theoretical_curves(profile)
    profile = build03()
    profile['values']['motor_1_rise_ms'] = 'Неизвестно'
    with pytest.raises(PhysicsError, match='числовые'):
        theoretical_curves(profile)
    profile = build03()
    profile['values']['motor_1_direction'] = ''
    with pytest.raises(PhysicsError, match='направление'):
        theoretical_curves(profile)
    profile = build03()
    profile['values']['mass_g'] = 'Неизвестно'
    with pytest.raises(PhysicsError, match='mass_g'):
        theoretical_curves(profile)


@pytest.mark.skipif(not os.getenv('BFEMU_TEST_REAL_SITL'), reason='Нужен собранный Betaflight SITL и свободные порты')
def test_real_isolated_sitl_runs_theoretical_profile():
    result = experiment(build03())
    assert result['firmware'] == 'Betaflight 4.5.4 SITL'
    assert len(result['samples']) > 100
    assert max(abs(row['rates'][0]) for row in result['samples']) > 5
    assert result['samples'][-1]['t'] > 10
    assert 0 < result['command_window_s'][0] < result['command_window_s'][1] < result['samples'][-1]['t']


def test_listening_port_must_belong_to_child():
    import socket
    import subprocess
    import sys

    from bfemu.aircraft_sitl import _owns_listening_port
    with socket.socket() as own:
        own.bind(('127.0.0.1', 0))
        own.listen()
        port = own.getsockname()[1]
        process = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(10)'])
        try:
            assert not _owns_listening_port(process, port)
            assert _owns_listening_port(type('P', (), {'pid': os.getpid()})(), port)
        finally:
            process.terminate()
            process.wait(timeout=2)


def test_sitl_page_allows_example_without_overwriting_and_explains_errors():
    import shutil
    import subprocess

    from bfemu.aircraft_page import aircraft_page
    node = shutil.which('node')
    if node is None:
        pytest.skip('Node.js не установлен')
    script = aircraft_page().rsplit('<script>', 1)[1].split('</script>', 1)[0]
    harness = r'''
const vm=require('vm'),assert=require('assert');
let handlers={}, sent, stored={schema:1,values:{mass_g:''}}, form={mass_g:''},fail=false,prep='ready',polls=0,timingErrors=0,tries=0;
const ctx={clearRect(){},fillRect(){},beginPath(){},moveTo(){},lineTo(){},stroke(){},fillText(){}};
function button(id){return {disabled:false,addEventListener(_,fn){handlers[id]=fn}}}
const saved=button('saved'),example=button('example'),status={},summary={},canvas={width:1000,height:300,getContext(){return ctx}};
const nodes={'run-sitl':saved,'run-sitl-example':example,'sitl-status':status,'sitl-summary':summary,'sitl-graph':canvas};
const document={getElementById(id){return nodes[id]}},STORAGE_KEY='test',PRESET={mass_g:'931',profile_name:'7'};
const localStorage={getItem(){return stored&&JSON.stringify(stored)}};
const current=()=>form,checked=profile=>profile.values;
const fetch=async (url,opts)=>{if(url==='/api/aircraft-sitl/prepare'){if(opts.method==='POST')return {ok:true,json:async()=>({state:prep})};polls++;return {ok:true,json:async()=>({state:'ready'})}}sent={url,body:JSON.parse(opts.body)};tries++;if(timingErrors>0){timingErrors--;return {ok:false,json:async()=>({error:'Нарушен темп расчёта: задержка'})}}return fail?{ok:false,json:async()=>({error:'Для физики необходимо числовое значение mass_g; «» не подходит.'})}:{ok:true,json:async()=>({samples:[{t:1,rates:[10,2,1]}],command_window_s:[.2,.8],firmware:'SITL',profile:'7',note:'модель'})}};
vm.runInNewContext(process.argv[1],{document,localStorage,fetch,JSON,Math,Error,Date,Promise,setTimeout:fn=>fn(),STORAGE_KEY,PRESET,current,checked});
(async()=>{
 await handlers.example();assert.strictEqual(sent.url,'/api/aircraft-sitl');assert.strictEqual(sent.body.profile.values.mass_g,'931');
 assert.strictEqual(stored.values.mass_g,'');assert.strictEqual(form.mass_g,'');assert(status.textContent.includes('Эксперимент завершён'));
 assert(summary.textContent.includes('крен 10.0'));assert(!saved.disabled&&!example.disabled);
 fail=true;await handlers.saved();assert(status.textContent.includes('Полная взлётная масса'));assert(status.textContent.includes('Запустить готовый пример 7″'));assert(!status.textContent.includes('mass_g'));
 form={mass_g:'931'};sent=null;await handlers.saved();assert(status.textContent.includes('несохранённые изменения'));assert.strictEqual(sent,null);
 stored=null;await handlers.saved();assert(status.textContent.includes('Нет сохранённого профиля'));
 fail=false;prep='building';timingErrors=1;let before=tries;await handlers.example();assert.strictEqual(tries-before,2);assert(status.textContent.includes('Эксперимент завершён'));assert.strictEqual(polls,1);assert.strictEqual(sent.body.profile.values.mass_g,'931');assert.strictEqual(stored,null);
 timingErrors=5;before=tries;await handlers.example();assert.strictEqual(tries-before,3);assert(status.textContent.includes('несколько раз не успел'));assert(!saved.disabled&&!example.disabled);
})().catch(err=>{console.error(err);process.exit(1)});
'''
    result = subprocess.run([node, '-e', harness, script], capture_output=True, text=True, timeout=10, check=False)
    assert result.returncode == 0, result.stderr


def test_build_preparation_runs_once_and_reports_failure(tmp_path, monkeypatch):
    import json
    import threading

    from bfemu import aircraft_sitl as sitl
    root = tmp_path / 'cache'
    cache = root / sitl.REVISION
    script = tmp_path / 'build.py'
    script.write_text('# mock builder')
    monkeypatch.setattr(sitl, 'CACHE', cache)
    monkeypatch.setattr(sitl, 'BINARY', cache / 'sitl.elf')
    monkeypatch.setattr(sitl, 'BUILD_SCRIPT', script)
    monkeypatch.setattr(sitl, '_BUILD_THREAD', None)
    monkeypatch.setattr(sitl, '_BUILD_ERROR', None)
    release = threading.Event()
    starts = []
    class FakeProcess:
        returncode = 0
        def __init__(self, *args, **kwargs):
            starts.append(args)
        def communicate(self, timeout=None):
            assert release.wait(3)
            cache.mkdir(parents=True)
            sitl.BINARY.write_bytes(b'test')
            (cache / 'bfemu-sitl.json').write_text(json.dumps(sitl.STAMP))
            return b'', b''
    monkeypatch.setattr(sitl.subprocess, 'Popen', FakeProcess)
    assert sitl.build_status()['state'] == 'missing'
    assert sitl.start_sitl_build()['state'] == 'building'
    assert sitl.start_sitl_build()['state'] == 'building'
    release.set()
    sitl._BUILD_THREAD.join(timeout=5)
    assert not sitl._BUILD_THREAD.is_alive()
    assert len(starts) == 1
    assert sitl.build_status()['state'] == 'ready'
    assert sitl.start_sitl_build()['state'] == 'ready'
    # Неполная установка не должна выглядеть как успешная сборка.
    sitl.BINARY.unlink()
    script.unlink()
    assert sitl.start_sitl_build()['state'] == 'building'
    sitl._BUILD_THREAD.join(timeout=5)
    status = sitl.build_status()
    assert status['state'] == 'error' and 'скрипт сборки' in status['error']


def test_pid_snapshot_modifies_only_selected_axis_and_verifies_readback(monkeypatch):
    from bfemu import aircraft_sitl as sitl
    class FakeClient:
        def __init__(self):
            self.payload = bytes(range(15))
            self.writes = []
        def request(self, code):
            assert code == 112
            return self.payload
    client = FakeClient()
    def write(client, code, payload):
        assert code == 202
        client.writes.append(payload)
        client.payload = payload
    monkeypatch.setattr(sitl, '_write', write)
    result = sitl.set_pid_snapshot(client, 'pitch', {'p': 70, 'i': 80, 'd': 40})
    assert result == {'axis': 'pitch', 'values': {'p': 70, 'i': 80, 'd': 40}}
    assert client.payload[:3] == bytes(range(3))
    assert client.payload[3:6] == bytes([70, 80, 40])
    assert client.payload[6:] == bytes(range(6, 15))
    for axis in ('roll', 'pitch', 'yaw'):
        client = FakeClient()
        sitl.set_pid_snapshot(client, axis, {'p': 11, 'i': 22, 'd': 33})
        offset = {'roll': 0, 'pitch': 3, 'yaw': 6}[axis]
        assert client.payload[offset:offset+3] == bytes([11, 22, 33])
    with pytest.raises(PhysicsError, match='целыми'):
        sitl.set_pid_snapshot(FakeClient(), 'roll', {'p': True, 'i': 2, 'd': 3})
    with pytest.raises(PhysicsError, match='Нужны ось'):
        sitl.set_pid_snapshot(FakeClient(), 'wrong', {'p': 1, 'i': 2, 'd': 3})
    client = FakeClient()
    monkeypatch.setattr(sitl, '_write', lambda *args: None)
    with pytest.raises(PhysicsError, match='не подтвердил'):
        sitl.set_pid_snapshot(client, 'roll', {'p': 70, 'i': 80, 'd': 40})
    assert client.payload == bytes(range(15))


@pytest.mark.skipif(not os.getenv('BFEMU_TEST_REAL_SITL'), reason='Нужен собранный Betaflight SITL и свободные порты')
def test_real_sitl_reads_back_transferred_roll_pid():
    requested = {'p': 55, 'i': 80, 'd': 40}
    result = experiment(build03(), axis='roll', pid=requested)
    assert result['verified_pid'] == {'axis': 'roll', 'values': requested}
    assert len(result['samples']) > 100
    assert result['axis'] == 'roll'


@pytest.mark.parametrize('axis,sign', [('roll', 1), ('pitch', -1), ('yaw', 1)])
@pytest.mark.skipif(not os.getenv('BFEMU_TEST_REAL_SITL'), reason='Нужен собранный Betaflight SITL и свободные порты')
def test_real_sitl_closed_loop_axis_and_diagnostics(axis, sign):
    requested = {'p': 45, 'i': 80, 'd': 40}
    result = experiment(build03(), axis=axis, pid=requested)
    assert result['verified_pid'] == {'axis': axis, 'values': requested}
    start, end = result['command_window_s']
    index = {'roll': 0, 'pitch': 1, 'yaw': 2}[axis]
    values = [row['rates'][index] for row in result['samples'] if start <= row['t'] <= end]
    assert max(sign*x for x in values) > 5
    info = result['diagnostics']
    assert info['peak_abs_dps'] >= max(abs(x) for x in values) - .01
    assert 0 < info['sensor_interval_max_ms'] < 20
    assert 0 <= info['motor_no_update_samples'] <= info['samples']


def test_diagnostics_use_all_physics_samples_and_command_window():
    import math
    from bfemu.aircraft_sitl import _diagnostics
    def row(t, rate, dt, next_commands, applied):
        return {'elapsed_s': t, 'dt_s': dt, 'next_commands': next_commands, 'applied_commands': applied,
                'state': {'angular_velocity_rad_s': [0, math.radians(rate), 0]}}
    rows = [row(.9, 900, .015, None, [1, 1, 1, 1]),
            row(1, -10, .004, None, [.2, .2, .2, .2]),
            row(1.1, 70, .006, [0, 0, 0, 0], [1, .2, .2, .2]),
            row(1.2, 30, .005, None, [.2, .2, .2, .2]),
            row(1.3, 900, .015, None, [1, 1, 1, 1])]
    result = _diagnostics(rows, 1, 1.2, 'pitch')
    assert result == {'peak_abs_dps': 70, 'sensor_interval_max_ms': 6,
                      'motor_no_update_samples': 2, 'motor_full_output_samples': 1, 'samples': 3}
    with pytest.raises(PhysicsError, match='нет отсчётов'):
        _diagnostics(rows, 2, 3, 'roll')


@pytest.mark.parametrize('period', [True, 0, 1, 3, 2.0, '2', None])
def test_experiment_rejects_unsupported_sensor_period_before_start(period):
    with pytest.raises(PhysicsError, match='Период датчиков'):
        experiment(build03(), sensor_period_ms=period)


@pytest.mark.skipif(not os.getenv('BFEMU_TEST_REAL_SITL'), reason='Нужен собранный Betaflight SITL и свободные порты')
def test_real_sitl_accepts_experimental_sensor_period():
    requested = {'p': 45, 'i': 80, 'd': 40}
    result = experiment(build03(), axis='roll', pid=requested, sensor_period_ms=2)
    assert result['sensor_period_ms'] == 2
    assert result['verified_pid'] == {'axis': 'roll', 'values': requested}
    assert result['diagnostics']['samples'] > 100

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


def test_sitl_page_uses_saved_profile_and_shows_errors():
    import shutil
    import subprocess

    from bfemu.aircraft_page import aircraft_page
    node = shutil.which('node')
    if node is None:
        pytest.skip('Node.js не установлен')
    script = aircraft_page().rsplit('<script>', 1)[1].split('</script>', 1)[0]
    harness = r'''
const vm=require('vm'),assert=require('assert');
let handler, sent, profile={schema:1,values:{mass_g:'931'}};
const ctx={clearRect(){},fillRect(){},beginPath(){},moveTo(){},lineTo(){},stroke(){},fillText(){}};
const button={disabled:false,addEventListener(_,fn){handler=fn}},status={},summary={},canvas={width:1000,height:300,getContext(){return ctx}};
const nodes={'run-sitl':button,'sitl-status':status,'sitl-summary':summary,'sitl-graph':canvas};
const document={getElementById(id){return nodes[id]}};
const localStorage={getItem(){return profile&&JSON.stringify(profile)}};
const fetch=async (url,opts)=>{sent={url,body:JSON.parse(opts.body)};return {ok:true,json:async()=>({samples:[{t:1,rates:[10,2,1]}],command_window_s:[.2,.8],firmware:'SITL',profile:'7',note:'модель'})}};
vm.runInNewContext(process.argv[1],{document,localStorage,fetch,JSON,Math,Error});
(async()=>{await handler();assert.strictEqual(sent.url,'/api/aircraft-sitl');assert.deepStrictEqual(sent.body.profile,profile);
assert(status.textContent.includes('Эксперимент завершён'));assert(summary.textContent.includes('крен 10.0'));
profile=null;await handler();assert(status.textContent.includes('Сначала нажмите'));assert.strictEqual(button.disabled,false)})().catch(err=>{console.error(err);process.exit(1)});
'''
    result = subprocess.run([node, '-e', harness, script], capture_output=True, text=True, timeout=10, check=False)
    assert result.returncode == 0, result.stderr

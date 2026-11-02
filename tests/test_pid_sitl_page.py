"""PID-тренажёр предлагает только подтверждённый покадровый SITL lockstep."""
import shutil
import subprocess

import pytest

from bfemu.pid_trainer_page import PID_TRAINER_PAGE


def test_pid_page_exposes_lockstep_only():
    assert 'id="sitl-lockstep-run"' in PID_TRAINER_PAGE
    assert 'Обычный UDP-эксперимент' not in PID_TRAINER_PAGE
    assert 'id="sitl-run"' not in PID_TRAINER_PAGE
    assert 'id="sitl-period"' not in PID_TRAINER_PAGE
    assert 'id="sitl-motor-mode"' not in PID_TRAINER_PAGE
    assert '/api/pid-trainer/sitl-lockstep' in PID_TRAINER_PAGE
    assert '/api/pid-trainer/sitl\'' not in PID_TRAINER_PAGE
    assert 'По поступлению' not in PID_TRAINER_PAGE


def test_lockstep_button_posts_only_applied_snapshot_and_confirms_frames():
    node = shutil.which('node')
    if node is None:
        pytest.skip('Node.js не установлен')
    start = PID_TRAINER_PAGE.index("$('sitl-lockstep-run').onclick=async()=>")
    end = PID_TRAINER_PAGE.index("window.addEventListener('resize',drawPidSitl);", start)
    handler = PID_TRAINER_PAGE[start:end]
    harness = r'''
const assert=require('assert');
let dirty=false,conflict=false,formDirty=false,formConflict=false,writeBusy=false;
const baseline={axis:'roll',profile:2,values:{p:45,i:80,d:40}};
const terms=['p','i','d'];
const nodes={
 'sitl-lockstep-run':{disabled:false}, 'pid-sitl-status':{className:'',textContent:''},
 'scenario':{value:'step'}, 'sitl-aircraft':{value:'example'},
 'pid-sitl-comparison':{textContent:''},'pid-sitl-summary':{textContent:''},
 'pid-sitl-graph-title':{textContent:''},'pid-sitl-axis-caption':{textContent:''}
};
const $=id=>nodes[id];
let prepared=0,posted=[];
const preparePidLockstep=async()=>{prepared++};
let repeatArgs=null,drawn=0,sitlResult=null,priorSitlResult=null;
const recordPidSitlAttempt=(...args)=>{repeatArgs=args};
const explainSitlComparison=()=> 'first run';
const sitlComparable=()=>false;
const drawPidSitl=()=>{drawn++};
const motorModeLabel=()=> 'lockstep · кадр 4 мс';
let responseResult={mode:'lockstep',axis:'roll',pid_profile:2,
 verified_pid:{axis:'roll',values:{p:45,i:80,d:40}},profile:'Build 03',note:'test',
 sensor_period_ms:4,motor_mode:'lockstep',pid_process_denom:1,
 diagnostics:{peak_abs_dps:30,pid_cycles_min:32,pid_cycles_max:32,motor_saturated_samples:0,samples:3},
 samples:[{frame:1,rates:[0,0,0],motors:[0,0,0,0]},
 {frame:2,rates:[1,2,3],motors:[0,0,0,0]},
 {frame:3,rates:[4,5,6],motors:[0,0,0,0]}]};
const fetch=async(url,options)=>{posted.push([url,JSON.parse(options.body)]);return {ok:true,json:async()=>responseResult}};
''' + handler + r'''
(async()=>{
 await nodes['sitl-lockstep-run'].onclick();
 assert.equal(prepared,1);assert.equal(posted.length,1);
 assert.equal(posted[0][0],'/api/pid-trainer/sitl-lockstep');
 assert.deepStrictEqual(posted[0][1],{axis:'roll',profile:2,expected:{p:45,i:80,d:40},aircraft:null});
 assert.equal(repeatArgs[1],30);assert.equal(drawn,1);
 assert(nodes['pid-sitl-status'].textContent.includes('каждый кадр подтверждены'));
 assert(nodes['pid-sitl-summary'].textContent.includes('PID cycles/кадр 32–32'));
 assert.equal(nodes['sitl-lockstep-run'].disabled,false);
 dirty=true;await nodes['sitl-lockstep-run'].onclick();assert.equal(posted.length,1);
 assert(nodes['pid-sitl-status'].textContent.includes('примените'));
 dirty=false;nodes.scenario.value='reverse';await nodes['sitl-lockstep-run'].onclick();assert.equal(posted.length,1);
 nodes.scenario.value='step';responseResult={...responseResult,samples:[...responseResult.samples.slice(0,2),{...responseResult.samples[2],frame:4}]};
 await nodes['sitl-lockstep-run'].onclick();assert.equal(posted.length,2);
 assert(nodes['pid-sitl-status'].textContent.includes('полную покадровую диагностику'));
})().catch(error=>{console.error(error);process.exit(1)});
'''
    result = subprocess.run([node, '-e', harness], capture_output=True, text=True, timeout=10, check=False)
    assert result.returncode == 0, result.stderr

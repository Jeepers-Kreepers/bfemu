"""Поведение отдельного SITL-графика и привязка к снимку применённых PID."""
import shutil
import subprocess

import pytest

from bfemu.pid_trainer_page import PID_TRAINER_PAGE


def test_sitl_graph_sends_applied_snapshot_and_compares_only_matching_aircraft():
    node = shutil.which('node')
    if node is None:
        pytest.skip('Node.js не установлен')
    script = PID_TRAINER_PAGE.split('let sitlResult=null,priorSitlResult=null,repeatKey=null,repeatPeaks=[];', 1)[1].split('function buildSettings(', 1)[0]
    harness = r'''
const assert=require('assert');
let sent=[],lastSeries=[],period='4',motorMode='poll',failed=false,dirty=false,conflict=false,formDirty=false,formConflict=false,writeBusy=false;
let baseline={axis:'roll',profile:0,values:{p:45,i:80,d:40}}, scenario='step', aircraftMode='example',stored=null;
let markerLines=[];const terms=['p','i','d'],ctx={setLineDash(){},beginPath(){},moveTo(x){this.x=x},lineTo(){},stroke(){markerLines.push([this.x,this.strokeStyle])}},nodes={
  'sitl-run':{},'pid-sitl-status':{},'pid-sitl-summary':{},'pid-sitl-diagnostics':{},'pid-sitl-graph':{getBoundingClientRect(){return {width:600,height:190}},getContext(){return ctx}},
  'pid-sitl-motors':{getBoundingClientRect(){return {width:600,height:190}},getContext(){return ctx}},
  'scenario':{get value(){return scenario}},'sitl-aircraft':{get value(){return aircraftMode}},'sitl-period':{get value(){return period}},'sitl-motor-mode':{get value(){return motorMode}}
};
const $=id=>nodes[id], window={addEventListener(){}},localStorage={getItem(){return stored}};
let motorSeries=[];const chart=(canvas,rows,series)=>{if(canvas===nodes['pid-sitl-graph'])lastSeries=series;else motorSeries=series};
const fetch=async (url,opts)=>{
 if(url==='/api/aircraft-sitl/prepare')return {ok:true,json:async()=>({state:'ready'})};
 let body=JSON.parse(opts.body);sent.push(body);if(failed)return {ok:false,json:async()=>({error:'Нарушен темп расчёта: задержка'})};
 return {ok:true,json:async()=>({axis:body.axis,pid_profile:body.profile,verified_pid:{axis:body.axis,values:body.expected},profile:'7',note:'model',sensor_period_ms:body.sensor_period_ms,motor_mode:body.motor_mode,diagnostics:{peak_abs_dps:30,sensor_interval_max_ms:4.1,motor_no_update_samples:1,motor_full_output_samples:0,samples:10},command_window_s:[4,5.2],disarm_requested_s:5.7,samples:[{t:4,rates:[20,0,0],motors:[0,.25,.5,1]},{t:5,rates:[30,0,0],motors:[1,.5,.25,0]}]})};
};
let sitlResult=null,priorSitlResult=null,repeatKey=null,repeatPeaks=[];
''' + script + r'''
(async()=>{
 dirty=true;await nodes['sitl-run'].onclick();assert.equal(sent.length,0);assert(nodes['pid-sitl-status'].textContent.includes('примените'));
 dirty=false;scenario='reverse';await nodes['sitl-run'].onclick();assert.equal(sent.length,0);scenario='step';
 await nodes['sitl-run'].onclick();assert.deepStrictEqual(sent[0],{axis:'roll',profile:0,expected:{p:45,i:80,d:40},aircraft:null,sensor_period_ms:4,motor_mode:'poll'});
 assert(nodes['pid-sitl-summary'].textContent.includes('P 45'));assert(nodes['pid-sitl-diagnostics'].textContent.includes('30,0'));assert.equal(lastSeries[0].rows,null);assert.equal(motorSeries.length,4);assert.deepStrictEqual(motorSeries[0].rows[0],[0.4,0,25,50,100]);assert.equal(markerLines.length,6);assert.deepStrictEqual(markerLines.slice(0,3).map(x=>x[1]),['#8ec5ff','#e3b45b','#fb8c8c']);assert(Math.abs(markerLines[0][0]-(46+0.4/3*542))<1e-6);assert(Math.abs(markerLines[1][0]-(46+1.6/3*542))<1e-6);assert(Math.abs(markerLines[2][0]-(46+2.1/3*542))<1e-6);
 await nodes['sitl-run'].onclick();assert(nodes['pid-sitl-diagnostics'].textContent.includes('30,0; 30,0'));sent.length=1;
 failed=true;await nodes['sitl-run'].onclick();assert.equal(sent.length,2);assert(nodes['pid-sitl-diagnostics'].textContent.includes('не завершён'));assert(nodes['pid-sitl-status'].textContent.includes('Нарушен темп'));failed=false;sent.length=1;
 period='2';await nodes['sitl-run'].onclick();assert.equal(sent[1].sensor_period_ms,2);assert(!nodes['pid-sitl-diagnostics'].textContent.includes('не завершён'));assert(nodes['pid-sitl-summary'].textContent.includes('период датчиков 2 мс'));assert(nodes['pid-sitl-summary'].textContent.includes('период датчиков 4 мс'));sent.length=1;
 motorMode='event';await nodes['sitl-run'].onclick();assert.equal(sent[1].motor_mode,'event');assert(nodes['pid-sitl-summary'].textContent.includes('моторы: по поступлению'));assert(nodes['pid-sitl-summary'].textContent.includes('моторы: опрос по шагу'));assert(!nodes['pid-sitl-diagnostics'].textContent.split('Учитываются')[0].includes(';'));sent.length=1;
 baseline.values.p=55;await nodes['sitl-run'].onclick();assert(nodes['pid-sitl-summary'].textContent.includes('Предыдущая кривая'));assert(lastSeries[0].rows);
 aircraftMode='saved';await nodes['sitl-run'].onclick();assert.equal(sent.length,2);assert(nodes['pid-sitl-status'].textContent.includes('нет сохранённого'));
 stored=JSON.stringify({schema:1,values:{mass_g:'931'}});await nodes['sitl-run'].onclick();assert.equal(sent.length,3);assert.equal(lastSeries[0].rows,null);assert(!nodes['pid-sitl-summary'].textContent.includes('Предыдущая кривая'));
 assert(!nodes['sitl-run'].disabled);
})().catch(e=>{console.error(e);process.exit(1)});
'''
    result = subprocess.run([node, '-e', harness], capture_output=True, text=True, timeout=10, check=False)
    assert result.returncode == 0, result.stderr

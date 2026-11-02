"""Поведение отдельного SITL-графика и привязка к снимку применённых PID."""
import shutil
import subprocess

import pytest

from bfemu.pid_trainer_page import PID_TRAINER_PAGE


def test_sitl_graph_sends_applied_snapshot_and_compares_only_matching_aircraft():
    node = shutil.which('node')
    if node is None:
        pytest.skip('Node.js не установлен')
    script = PID_TRAINER_PAGE.split('let sitlResult=null,priorSitlResult=null;', 1)[1].split('function buildSettings(', 1)[0]
    harness = r'''
const assert=require('assert');
let sent=[],lastSeries=[],dirty=false,conflict=false,formDirty=false,formConflict=false,writeBusy=false;
let baseline={axis:'roll',profile:0,values:{p:45,i:80,d:40}}, scenario='step', aircraftMode='example',stored=null;
const terms=['p','i','d'],ctx={setLineDash(){},beginPath(){},moveTo(){},lineTo(){},stroke(){}},nodes={
  'sitl-run':{},'pid-sitl-status':{},'pid-sitl-summary':{},'pid-sitl-graph':{getBoundingClientRect(){return {width:600,height:190}},getContext(){return ctx}},
  'scenario':{get value(){return scenario}},'sitl-aircraft':{get value(){return aircraftMode}}
};
const $=id=>nodes[id], window={addEventListener(){}},localStorage={getItem(){return stored}};
const chart=(canvas,rows,series)=>{lastSeries=series};
const fetch=async (url,opts)=>{
 if(url==='/api/aircraft-sitl/prepare')return {ok:true,json:async()=>({state:'ready'})};
 let body=JSON.parse(opts.body);sent.push(body);
 return {ok:true,json:async()=>({axis:body.axis,pid_profile:body.profile,verified_pid:{axis:body.axis,values:body.expected},profile:'7',note:'model',command_window_s:[4,5.2],samples:[{t:4,rates:[20,0,0]},{t:5,rates:[30,0,0]}]})};
};
let sitlResult=null,priorSitlResult=null;
''' + script + r'''
(async()=>{
 dirty=true;await nodes['sitl-run'].onclick();assert.equal(sent.length,0);assert(nodes['pid-sitl-status'].textContent.includes('примените'));
 dirty=false;baseline.axis='pitch';await nodes['sitl-run'].onclick();assert.equal(sent.length,0);assert(nodes['pid-sitl-status'].textContent.includes('ROLL'));
 baseline.axis='roll';scenario='reverse';await nodes['sitl-run'].onclick();assert.equal(sent.length,0);scenario='step';
 await nodes['sitl-run'].onclick();assert.deepStrictEqual(sent[0],{axis:'roll',profile:0,expected:{p:45,i:80,d:40},aircraft:null});
 assert(nodes['pid-sitl-summary'].textContent.includes('P 45'));assert.equal(lastSeries[0].rows,null);
 baseline.values.p=55;await nodes['sitl-run'].onclick();assert(nodes['pid-sitl-summary'].textContent.includes('Предыдущая кривая'));assert(lastSeries[0].rows);
 aircraftMode='saved';await nodes['sitl-run'].onclick();assert.equal(sent.length,2);assert(nodes['pid-sitl-status'].textContent.includes('нет сохранённого'));
 stored=JSON.stringify({schema:1,values:{mass_g:'931'}});await nodes['sitl-run'].onclick();assert.equal(sent.length,3);assert.equal(lastSeries[0].rows,null);assert(!nodes['pid-sitl-summary'].textContent.includes('Предыдущая кривая'));
 assert(!nodes['sitl-run'].disabled);
})().catch(e=>{console.error(e);process.exit(1)});
'''
    result = subprocess.run([node, '-e', harness], capture_output=True, text=True, timeout=10, check=False)
    assert result.returncode == 0, result.stderr

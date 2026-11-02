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
let sent=[],lastSeries=[],period='4',motorMode='poll',failed=false,saturation=0,dirty=false,conflict=false,formDirty=false,formConflict=false,writeBusy=false;
let baseline={axis:'roll',profile:0,values:{p:45,i:80,d:40}}, scenario='step', aircraftMode='example',stored=null;
let markerLines=[];const terms=['p','i','d'],ctx={setLineDash(){},beginPath(){},moveTo(x){this.x=x},lineTo(){},stroke(){markerLines.push([this.x,this.strokeStyle])}},nodes={
  'sitl-run':{},'sitl-lockstep-run':{},'pid-sitl-status':{},'pid-sitl-summary':{},'pid-sitl-diagnostics':{},'pid-sitl-warning':{},'pid-sitl-comparison':{},'pid-sitl-graph-title':{},'pid-sitl-axis-caption':{},'pid-sitl-graph':{getBoundingClientRect(){return {width:600,height:190}},getContext(){return ctx}},
  'pid-sitl-motors':{getBoundingClientRect(){return {width:600,height:190}},getContext(){return ctx}},
  'sitl-zoom':{value:'1'},'sitl-pan':{value:'0',max:'0'},'sitl-window':{},'sitl-rate-scale':{value:'auto'},'sitl-motor-scale':{value:'full'},'sitl-view-reset':{},
  ...Object.fromEntries(['current','previous','motor-1','motor-2','motor-3','motor-4'].map(x=>['sitl-show-'+x,{checked:true}])),
  'scenario':{get value(){return scenario}},'sitl-aircraft':{get value(){return aircraftMode}},'sitl-period':{get value(){return period}},'sitl-motor-mode':{get value(){return motorMode}}
};
const $=id=>nodes[id], window={addEventListener(){}},localStorage={getItem(){return stored}};
let motorSeries=[],rateRange=[],motorRange=[],rangeSeen=[],graphUnits={};const chart=(canvas,rows,series,min,max,decimals,range,unit)=>{graphUnits[canvas===nodes['pid-sitl-graph']?'rates':'motor']=unit;rangeSeen=range;if(canvas===nodes['pid-sitl-graph']){lastSeries=series;rateRange=[min,max]}else{motorSeries=series;motorRange=[min,max]}};
const fetch=async (url,opts)=>{
 if(url==='/api/aircraft-sitl/prepare'||url==='/api/pid-trainer/sitl-lockstep/prepare')return {ok:true,json:async()=>({state:'ready'})};
 let body=JSON.parse(opts.body);sent.push(body);if(failed)return {ok:false,json:async()=>({error:'Нарушен темп расчёта: задержка'})};
 if(url==='/api/pid-trainer/sitl-lockstep')return {ok:true,json:async()=>({mode:'lockstep',axis:body.axis,pid_profile:body.profile,verified_pid:{axis:body.axis,values:body.expected},profile:'7',note:'frame-locked',sensor_period_ms:4,motor_mode:'lockstep',pid_process_denom:1,diagnostics:{peak_abs_dps:30,pid_cycles_min:32,pid_cycles_max:32,motor_saturated_samples:saturation,samples:300},command_window_s:[.6,1.8],disarm_requested_s:2.3,samples:[{t:.6,rates:[20,0,0],motors:[0,.25,.5,1]},{t:1.8,rates:[30,0,0],motors:[1,.5,.25,0]}]})};
 return {ok:true,json:async()=>({axis:body.axis,pid_profile:body.profile,verified_pid:{axis:body.axis,values:body.expected},profile:'7',note:'model',sensor_period_ms:body.sensor_period_ms,motor_mode:body.motor_mode,diagnostics:{peak_abs_dps:30,sensor_interval_max_ms:4.1,motor_no_update_samples:1,motor_full_output_samples:saturation,samples:10},command_window_s:[4,5.2],disarm_requested_s:5.7,samples:[{t:4,rates:[20,0,0],motors:[0,.25,.5,1]},{t:5,rates:[30,0,0],motors:[1,.5,.25,0]}]})};
};
let sitlResult=null,priorSitlResult=null,repeatKey=null,repeatPeaks=[];
''' + script + r'''
(async()=>{
 dirty=true;await nodes['sitl-run'].onclick();assert.equal(sent.length,0);assert(nodes['pid-sitl-status'].textContent.includes('примените'));
 dirty=false;scenario='reverse';await nodes['sitl-run'].onclick();assert.equal(sent.length,0);scenario='step';
 await nodes['sitl-run'].onclick();assert.deepStrictEqual(sent[0],{axis:'roll',profile:0,expected:{p:45,i:80,d:40},aircraft:null,sensor_period_ms:4,motor_mode:'poll'});
 assert(nodes['pid-sitl-comparison'].textContent.includes('ещё один запуск'));assert(nodes['pid-sitl-graph-title'].textContent.includes('ROLL · крен'));assert(nodes['pid-sitl-axis-caption'].textContent.includes('правый борт опускается'));assert(nodes['pid-sitl-summary'].textContent.includes('P 45'));assert(nodes['pid-sitl-diagnostics'].textContent.includes('30,0'));assert.equal(nodes['pid-sitl-warning'].textContent,'');assert(nodes['pid-sitl-status'].textContent.includes('достоверность отклика пока не подтверждена'));assert.equal(lastSeries[0].rows,null);assert.equal(graphUnits.rates,'°/с');assert.equal(graphUnits.motor,'%');assert.equal(motorSeries.length,4);assert.deepStrictEqual(motorSeries[0].rows[0],[0.4,0,25,50,100]);assert.equal(markerLines.length,6);assert.deepStrictEqual(markerLines.slice(0,3).map(x=>x[1]),['#8ec5ff','#e3b45b','#fb8c8c']);assert(Math.abs(markerLines[0][0]-(72+0.4/3*516))<1e-6);assert(Math.abs(markerLines[1][0]-(72+1.6/3*516))<1e-6);assert(Math.abs(markerLines[2][0]-(72+2.1/3*516))<1e-6);
 assert.deepStrictEqual(visibleValues([[0,0],[2,20]],1,[.5,1.5]),[5,15]);
 nodes['sitl-show-motor-2'].checked=false;nodes['sitl-show-motor-2'].onchange();assert.equal(motorSeries[1].rows,null);assert(motorSeries[0].rows);
 nodes['sitl-motor-scale'].value='auto';nodes['sitl-motor-scale'].onchange();assert(motorRange[0]===0 && motorRange[1]===100);
 nodes['sitl-show-motor-4'].checked=false;nodes['sitl-show-motor-3'].checked=false;nodes['sitl-show-motor-4'].onchange();assert(motorRange[0]===0 && motorRange[1]===100);
 nodes['sitl-show-motor-1'].checked=false;nodes['sitl-show-motor-1'].onchange();assert.deepStrictEqual(motorRange,[0,100]);
 const originalMotors=sitlResult.samples.map(row=>row.motors.slice());sitlResult.samples.forEach(row=>row.motors=[.3,.5,.5,.5]);
 nodes['sitl-show-motor-1'].checked=true;nodes['sitl-show-motor-1'].onchange();assert.deepStrictEqual(motorRange,[27.5,32.5]);
 sitlResult.samples.forEach((row,i)=>row.motors=originalMotors[i]);nodes['sitl-show-motor-1'].checked=false;nodes['sitl-show-motor-1'].onchange();
 nodes['sitl-rate-scale'].value='100';nodes['sitl-rate-scale'].onchange();assert.deepStrictEqual(rateRange,[-100,100]);
 markerLines=[];nodes['sitl-zoom'].value='2';nodes['sitl-zoom'].onchange();assert.equal(markerLines.length,4);assert.deepStrictEqual(markerLines.slice(0,2).map(x=>x[1]),['#e3b45b','#fb8c8c']);assert(Math.abs(markerLines[0][0]-(72+(1.6-.75)/1.5*516))<1e-6);assert.deepStrictEqual(rangeSeen,[.75,2.25]);assert.equal(nodes['sitl-window'].textContent,'0,75–2,25 с');assert.equal(nodes['sitl-pan'].disabled,false);
 nodes['sitl-pan'].value='1.50';nodes['sitl-pan'].oninput();assert.deepStrictEqual(rangeSeen,[1.5,3]);assert.equal(nodes['sitl-window'].textContent,'1,50–3,00 с');
 nodes['sitl-zoom'].value='4';nodes['sitl-zoom'].onchange();assert.equal(rangeSeen[0],1.88);assert.equal(rangeSeen[1],2.63);
 // Видимость не запускает эксперимент и не меняет диагностику.
 const diagnostics=nodes['pid-sitl-diagnostics'].textContent,sendCount=sent.length;
 nodes['sitl-view-reset'].onclick();assert.equal(sent.length,sendCount);assert.equal(nodes['pid-sitl-diagnostics'].textContent,diagnostics);assert.deepStrictEqual(rangeSeen,[0,3]);assert.equal(nodes['sitl-pan'].disabled,true);assert.equal(nodes['sitl-rate-scale'].value,'auto');assert.equal(nodes['sitl-motor-scale'].value,'full');assert(nodes['sitl-show-motor-2'].checked);
 saturation=4;await nodes['sitl-run'].onclick();assert(nodes['pid-sitl-comparison'].textContent.includes('P, I и D Max не менялись'));assert(nodes['pid-sitl-comparison'].textContent.includes('30,0 → 30,0 °/с'));assert(nodes['pid-sitl-comparison'].textContent.includes('среднеквадратичная 0,00 °/с'));assert(nodes['pid-sitl-comparison'].textContent.includes('интерполяцией')); assert(nodes['pid-sitl-diagnostics'].textContent.includes('30,0; 30,0'));assert(nodes['pid-sitl-warning'].textContent.includes('4 из 10 интервалов'));assert(nodes['pid-sitl-warning'].textContent.includes('причина колебаний'));sent.length=1;
 priorSitlResult.samples[0].rates[0]=1000;drawPidSitl();assert.equal(rateRange[1],1150);
 nodes['sitl-show-previous'].checked=false;nodes['sitl-show-previous'].onchange();assert.equal(lastSeries[0].rows,null);assert.equal(rateRange[1],34.5);nodes['sitl-show-previous'].checked=true;nodes['sitl-show-previous'].onchange();assert(lastSeries[0].rows);
 failed=true;await nodes['sitl-run'].onclick();assert.equal(sent.length,2);assert(nodes['pid-sitl-diagnostics'].textContent.includes('не завершён'));assert(nodes['pid-sitl-status'].textContent.includes('Нарушен темп'));assert(nodes['pid-sitl-comparison'].textContent.includes('не завершился'));failed=false;sent.length=1;
 saturation=0;period='2';await nodes['sitl-run'].onclick();assert(nodes['pid-sitl-comparison'].textContent.includes('нельзя приписать'));assert.equal(nodes['pid-sitl-warning'].textContent,'');assert.equal(sent[1].sensor_period_ms,2);assert(!nodes['pid-sitl-diagnostics'].textContent.includes('не завершён'));assert(nodes['pid-sitl-summary'].textContent.includes('период датчиков 2 мс'));assert(nodes['pid-sitl-summary'].textContent.includes('период датчиков 4 мс'));sent.length=1;
 motorMode='event';await nodes['sitl-run'].onclick();assert.equal(sent[1].motor_mode,'event');assert(nodes['pid-sitl-summary'].textContent.includes('моторы: по поступлению'));assert(nodes['pid-sitl-summary'].textContent.includes('моторы: опрос по шагу'));assert(!nodes['pid-sitl-diagnostics'].textContent.split('Учитываются')[0].includes(';'));sent.length=1;
 baseline.values.p=55;await nodes['sitl-run'].onclick();assert(nodes['pid-sitl-comparison'].textContent.includes('P 45 → 55'));assert(nodes['pid-sitl-comparison'].textContent.includes('пропорциональной реакции на ошибку'));assert(nodes['pid-sitl-comparison'].textContent.includes('не доказывают'));assert(nodes['pid-sitl-summary'].textContent.includes('Предыдущая кривая'));assert(lastSeries[0].rows);
 aircraftMode='saved';await nodes['sitl-run'].onclick();assert.equal(sent.length,2);assert(nodes['pid-sitl-status'].textContent.includes('нет сохранённого'));
 stored=JSON.stringify({schema:1,values:{mass_g:'931'}});await nodes['sitl-run'].onclick();assert.equal(sent.length,3);assert.equal(lastSeries[0].rows,null);assert(nodes['pid-sitl-comparison'].textContent.includes('другому аппарату'));assert(!nodes['pid-sitl-summary'].textContent.includes('Предыдущая кривая'));
 for(const [axis,label,direction,index] of [['pitch','PITCH · тангаж','носом вверх',1],['yaw','YAW · рыскание','носом вправо',2]]){
  baseline.axis=axis;await nodes['sitl-run'].onclick();sitlResult.samples[0].rates=[1,2,3];drawPidSitl();
  assert(nodes['pid-sitl-graph-title'].textContent.includes(label));assert(nodes['pid-sitl-axis-caption'].textContent.includes(direction));
  assert.equal(lastSeries[1].rows[0][1],[1,2,3][index]);
 }
 const makeSeries=(values,axis='roll')=>({axis,command_window_s:[1,1.03],disarm_requested_s:1.04,samples:values.map((v,i)=>({t:1+i*.01,rates:axis==='roll'?[v,0,0]:[0,v,0]}))});
 const stable=sitlRepeatDifference(makeSeries([0,10,20,30,40]),makeSeries([0,10,20,30,40]));
 assert.equal(stable.samples,5);assert(Math.abs(stable.rms)<1e-9&&Math.abs(stable.max)<1e-9);
 const altered=sitlRepeatDifference(makeSeries([0,0,0,0,0]),makeSeries([10,10,10,10,10]));assert(Math.abs(altered.rms-10)<1e-9&&Math.abs(altered.max-10)<1e-9);
 const sparse=makeSeries([1,2]);sparse.samples[1].t=sparse.samples[0].t;assert.equal(sitlRepeatDifference(sparse,makeSeries([1,2])),null);
 const irregA={axis:'roll',command_window_s:[1,1.03],disarm_requested_s:1.04,samples:[{t:1,rates:[0,0,0]},{t:1.02,rates:[2,0,0]},{t:1.04,rates:[4,0,0]}]};
 const irregB={axis:'roll',command_window_s:[5,5.03],disarm_requested_s:5.04,samples:[{t:5,rates:[0,0,0]},{t:5.01,rates:[1,0,0]},{t:5.03,rates:[3,0,0]},{t:5.04,rates:[4,0,0]}]};
 const aligned=sitlRepeatDifference(irregA,irregB);assert.equal(aligned.samples,5);assert(aligned.rms<1e-9);
 assert.equal(sitlRepeatDifference(irregA,{...irregB,command_window_s:[8,8.03],disarm_requested_s:8.04}),null);
 const sample={axis:'pitch',aircraftKey:'same',pid_profile:0,sensor_period_ms:4,motor_mode:'poll',verified_pid:{values:{p:49,i:84,d:46}},diagnostics:{peak_abs_dps:100}};
 const changed={...sample,verified_pid:{values:{p:49,i:90,d:50}},diagnostics:{peak_abs_dps:130}};
 const explanation=explainSitlComparison(sample,changed);
 assert(explanation.includes('I 84 → 90')&&explanation.includes('D Max 46 → 50'));
 assert(explanation.includes('накопленной ошибки')&&explanation.includes('динамической составляющей D'));
 assert(explanation.includes('100,0 → 130,0 °/с')&&explanation.includes('не доказывают'));
 assert(explainSitlComparison(sample,{...changed,pid_profile:1}).includes('нельзя приписать'));
 assert(explainSitlComparison(sample,{...changed,axis:'yaw'}).includes('другой оси'));
 await nodes['sitl-lockstep-run'].onclick();
 assert.deepStrictEqual(sent.at(-1),{axis:'yaw',profile:0,expected:{p:55,i:80,d:40},aircraft:{schema:1,values:{mass_g:'931'}}});
 assert(nodes['pid-sitl-status'].textContent.includes('Lockstep завершён'));
 assert(nodes['pid-sitl-diagnostics'].textContent.includes('32–32 циклов PID'));
 assert(nodes['pid-sitl-summary'].textContent.includes('lockstep · кадр 4 мс'));
 assert(!nodes['sitl-run'].disabled);
})().catch(e=>{console.error(e);process.exit(1)});
'''
    result = subprocess.run([node, '-e', harness], capture_output=True, text=True, timeout=10, check=False)
    assert result.returncode == 0, result.stderr


def test_sensor_period_annotation_distinguishes_nominal_pid_frequency():
    assert 'а не частоту PID' in PID_TRAINER_PAGE
    assert 'целевой период PID — 125 мкс (8 кГц)' in PID_TRAINER_PAGE

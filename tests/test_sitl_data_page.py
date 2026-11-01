"""Отображаются только сохранённые байты ответов настоящего MSP."""
from __future__ import annotations

import shutil
import subprocess

import pytest

from bfemu.sitl_data_page import SITL_DATA_PAGE


def test_browser_parses_raw_msp_instead_of_untrusted_decoded_data():
    node = shutil.which("node")
    if not node:
        pytest.skip("Node.js не установлен")
    functions = SITL_DATA_PAGE.split("const $=id=>", 1)[1].split("$('file').onchange=", 1)[0]
    script = r'''
const assert=require('assert'); const document={getElementById(){return null}};
''' + '\nconst $=id=>' + functions + r'''
const head={type:'identity',variant:'BTFL',version:'4.5.4',board:'SITL',time_basis:'host_monotonic_after_response'};
const imu=Buffer.alloc(18);imu.writeInt16LE(-321,6);imu.writeInt16LE(300,8);imu.writeInt16LE(-4,10);
const motor=Buffer.alloc(16);[1000,1250,1500,1750,0,0,0,0].forEach((v,i)=>motor.writeUInt16LE(v,i*2));
const text=[head,{type:'msp',received_s:0.1,command:'MSP_RAW_IMU',payload_hex:imu.toString('hex'),decoded:{gyro_dps:[999,999,999]}},
 {type:'msp',received_s:0.2,command:'MSP_MOTOR',payload_hex:motor.toString('hex'),decoded:{motor_output_msp:[999]}}].map(JSON.stringify).join('\n');
const data=parseData(text);
assert.deepStrictEqual(data.gyro,[[0.1,-321*2000/32768,300*2000/32768,-4*2000/32768]]);
assert.deepStrictEqual(data.motor,[[0.2,1000,1250,1500,1750,0,0,0,0]]);
assert.throws(()=>parseData(text.replace('SITL','S405')));
assert.throws(()=>parseData(text.replace('"received_s":0.2','"received_s":0.05')));
assert.throws(()=>parseData(text.replace(imu.toString('hex'),'00')));
const strokes=[];const ctx={path:[],scale(){},clearRect(){},fillText(){},beginPath(){this.path=[]},moveTo(x,y){this.path.push([x,y])},lineTo(x,y){this.path.push([x,y])},stroke(){strokes.push({color:this.strokeStyle,path:this.path})},arc(){},fill(){}};
const canvas={getBoundingClientRect(){return {width:600,height:240}},getContext(){return ctx}};const window={devicePixelRatio:1};
draw(canvas,[[0,10],[1,-10]],1,'#abc');const curve=strokes.find(s=>s.color==='#abc');
assert(curve && curve.path.length===2 && curve.path[0][1]<curve.path[1][1]);
'''
    result = subprocess.run([node, "-e", script], capture_output=True, text=True, timeout=10, check=False)
    assert result.returncode == 0, result.stderr

"""Редактор хранит исходные значения и не подменяет их физической моделью."""
from __future__ import annotations

import shutil
import subprocess

import pytest

from bfemu.aircraft_page import aircraft_page


def test_aircraft_profile_script_validation_and_storage():
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node.js не установлен")
    script = aircraft_page().split('<script>', 1)[1].split('</script>', 1)[0]
    harness = r'''
const vm=require('vm'),assert=require('assert');
const script=process.argv[1];
function setup(initial=null,fail=false){
 let storage=initial,focus='';const listeners={};
 const make=(name,type='number',rule='')=>({name,type,value:'',dataset:{rule},validity:{badInput:false},tagName:'INPUT',maxLength:-1,parentElement:{firstChild:{textContent:name}},closest(){return null},focus(){focus=name},cloneNode(){return {value:''}}});
 const fields=[make('profile_name','text'),make('mass_g','number','positive'),make('cg_x'),make('battery_cells','number','positive_integer'),make('inertia_xx','number','positive'),make('motor_1_output','text'),make('motor_2_output','text')];
 const form={querySelectorAll(){return fields},addEventListener(n,f){listeners[n]=f}};
 const status={textContent:'',className:''},buttons={restore:{addEventListener(n,f){listeners.restore=f}},clear:{addEventListener(n,f){listeners.clear=f}}};
 const document={getElementById(id){return id==='aircraft-form'?form:id==='status'?status:buttons[id]}};
 const localStorage={getItem(){return storage},setItem(key,value){if(fail)throw Error('хранилище недоступно');storage=value}};
 const window={confirm(){return true},addEventListener(n,f){listeners[n]=f}};
 vm.runInNewContext(script,{document,window,localStorage});
 return {fields,form,status,listeners,get storage(){return storage},get focus(){return focus},field(n){return fields.find(f=>f.name===n)},submit(){listeners.submit({preventDefault(){}})}};
}
let s=setup();assert(s.status.textContent.includes('пока нет'));assert(s.fields.every(f=>f.value===''));
s.field('mass_g').value='-5';s.submit();assert(s.status.textContent.includes('Не удалось'));assert.strictEqual(s.storage,null);assert.strictEqual(s.focus,'mass_g');
s.field('mass_g').value='500';s.field('cg_x').value='-10';s.field('battery_cells').value='2.5';s.submit();assert.strictEqual(s.storage,null);
s.field('battery_cells').value='4';s.field('motor_1_output').value='1';s.field('motor_2_output').value='1';s.submit();assert(s.status.textContent.includes('двум моторам'));assert.strictEqual(s.storage,null);
s.field('motor_2_output').value='2';s.submit();const stored=s.storage;assert(s.status.textContent.includes('сохранён'));let data=JSON.parse(stored);assert.strictEqual(data.values.cg_x,'-10');assert.strictEqual(data.values.inertia_xx,'');
s.field('mass_g').value='600';s.listeners.input();assert(s.status.textContent.includes('не сохранены'));s.listeners.restore();assert.strictEqual(s.field('mass_g').value,'500');
s.listeners.clear();assert.strictEqual(s.field('mass_g').value,'');assert.strictEqual(s.storage,stored);s.listeners.restore();assert.strictEqual(s.field('mass_g').value,'500');
let t=setup(stored);assert.strictEqual(t.field('mass_g').value,'500');assert.strictEqual(t.field('cg_x').value,'-10');
let q=setup(null,true);q.field('mass_g').value='500';q.submit();assert(q.status.textContent.includes('хранилище недоступно'));assert.strictEqual(q.field('mass_g').value,'500');
let bad=setup('{"schema":9,"values":{}}');assert(bad.status.textContent.includes('не удалось загрузить')||bad.status.textContent.includes('Не удалось загрузить'));assert.strictEqual(bad.storage,'{"schema":9,"values":{}}');
'''
    result = subprocess.run([node, "-e", harness, script], capture_output=True, text=True, timeout=10, check=False)
    assert result.returncode == 0, result.stderr

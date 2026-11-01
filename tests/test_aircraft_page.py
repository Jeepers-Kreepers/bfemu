"""Профиль: данные примера, проверка ввода и сохранение пользовательских значений."""
from __future__ import annotations

import json
import shutil
import subprocess
from html.parser import HTMLParser

import pytest

from bfemu.aircraft_page import aircraft_page


class Fields(HTMLParser):
    def __init__(self):
        super().__init__()
        self.fields = []
        self.active_select = None

    def handle_starttag(self, tag, attrs):
        attr = dict(attrs)
        if tag in ('input', 'textarea', 'select') and 'name' in attr:
            self.fields.append({'name': attr['name'], 'type': attr.get('type', tag),
                                'rule': attr.get('data-rule', ''), 'numeric': attr.get('data-numeric', ''),
                                'tagName': tag.upper(), 'maxLength': int(attr.get('maxlength', -1)), 'options': []})
            if tag == 'select':
                self.active_select = self.fields[-1]
        elif tag == 'option' and self.active_select is not None:
            self.active_select['options'].append({'value': attr.get('value', '')})

    def handle_endtag(self, tag):
        if tag == 'select':
            self.active_select = None


def test_aircraft_profile_script_defaults_storage_and_validation():
    node = shutil.which('node')
    if node is None:
        pytest.skip('Node.js не установлен')
    page = aircraft_page()
    script = page.split('<script>', 1)[1].split('</script>', 1)[0]
    parser = Fields()
    parser.feed(page)
    harness = r'''
const vm=require('vm'),assert=require('assert');
const script=process.argv[1], definitions=JSON.parse(process.argv[2]);
function setup(initial=null,fail=false){
 let storage=initial,focus='',accept=true;const listeners={};
 const fields=definitions.map(d=>({...d,value:'',dataset:{rule:d.rule,numeric:d.numeric},validity:{badInput:false},parentElement:{firstChild:{textContent:d.name}},closest(){return null},focus(){focus=d.name}}));
 const form={querySelectorAll(){return fields},addEventListener(n,f){listeners[n]=f}};
 const status={textContent:'',className:''},presetStatus={textContent:'',className:''},info={textContent:'',appendChild(){}};
 const choice={value:'7',options:[{value:'7'}],appendChild(o){this.options=this.options.filter(x=>x!==o);this.options.push(o)},addEventListener(n,f){listeners.choice=f}};
 const buttons={};for(const id of ['restore','clear','defaults','import','export'])buttons[id]={addEventListener(n,f){listeners[id]=f}};
 const document={getElementById(id){return id==='aircraft-form'?form:id==='status'?status:id==='preset-status'?presetStatus:id==='preset-choice'?choice:id==='preset-info'?info:buttons[id]},createElement(){return {click(){}}}};
 const localStorage={getItem(){return storage},setItem(key,value){if(fail)throw Error('хранилище недоступно');storage=value}};
 const window={confirm(){return accept},addEventListener(n,f){listeners[n]=f}};
 const Blob=function(parts){this.parts=parts};const URL={createObjectURL(){return 'blob:fake'},revokeObjectURL(){}};
 vm.runInNewContext(script,{document,window,localStorage,Blob,URL,setTimeout(){}});
 return {fields,status,listeners,choice,get storage(){return storage},get focus(){return focus},set accept(v){accept=v},field(n){return fields.find(f=>f.name===n)},submit(){listeners.submit({preventDefault(){}})}};
}
(async()=>{
let s=setup();assert(s.status.textContent.includes('пример 7'));assert(s.fields.every(f=>f.value!==''),s.fields.filter(f=>f.value==='').map(f=>f.name));
for(const f of s.fields.filter(f=>f.tagName==='SELECT'))assert(f.options.some(o=>o.value===f.value),f.name);
assert.strictEqual(s.field('mass_g').value,'931');assert.strictEqual(s.field('battery_resistance').value,'Неизвестно');assert.strictEqual(s.field('sensor_delay').value,'Неизвестно');
assert.deepStrictEqual([1,2,3,4].map(i=>s.field('motor_'+i+'_output').value),['1','2','3','4']);
assert.deepStrictEqual([1,2,3,4].map(i=>s.field('motor_'+i+'_direction').value),['cw','ccw','ccw','cw']);
assert.deepStrictEqual([1,2,3,4].map(i=>[s.field('motor_'+i+'_x').value,s.field('motor_'+i+'_y').value]),[['-143','143'],['143','143'],['-143','-143'],['143','-143']]);
s.field('mass_g').value='-5';s.submit();assert(s.status.textContent.includes('Не удалось'));assert.strictEqual(s.storage,null);assert.strictEqual(s.focus,'mass_g');
s.field('mass_g').value='500';s.field('cg_x').value='-10';s.field('battery_cells').value='2.5';s.submit();assert.strictEqual(s.storage,null);
s.field('battery_cells').value='4';s.field('motor_1_output').value='2';s.submit();assert(s.status.textContent.includes('двум моторам'));assert.strictEqual(s.storage,null);
s.field('motor_1_output').value='1';s.field('sensor_delay').value='0x10';s.submit();assert(s.status.textContent.includes('Не удалось'));assert.strictEqual(s.storage,null);
s.field('sensor_delay').value='Неизвестно';s.submit();const stored=s.storage;assert(s.status.textContent.includes('сохранён'));let data=JSON.parse(stored);assert.strictEqual(data.values.cg_x,'-10');assert.strictEqual(data.values.inertia_xx,'0.00711');
s.field('mass_g').value='600';s.listeners.input();s.accept=false;s.listeners.defaults();assert.strictEqual(s.field('mass_g').value,'600');s.accept=true;s.listeners.defaults();assert.strictEqual(s.field('mass_g').value,'931');assert.strictEqual(s.storage,stored);
s.listeners.restore();assert.strictEqual(s.field('mass_g').value,'500');s.listeners.clear();assert.strictEqual(s.field('mass_g').value,'');assert.strictEqual(s.storage,stored);s.listeners.restore();assert.strictEqual(s.field('mass_g').value,'500');
assert.deepStrictEqual(s.choice.options.map(x=>x.value),['1.5','2','2.5','3','3.5','4','5','6','7','8','9','10']);
for(const id of ['1.5','2','2.5','3','3.5','4','5','6','8','9','10']){s.choice.value=id;s.listeners.defaults();assert(s.field('profile_name').value.includes('″'));assert.strictEqual(s.field('inertia_xx').value,'Неизвестно');assert.strictEqual(s.field('motor_1_output').value,'');assert.strictEqual(s.field('motor_1_max_thrust').value,'Неизвестно');assert.strictEqual(s.storage,stored);}
s.choice.value='3';s.listeners.defaults();assert.strictEqual(s.field('mass_g').value,'204');s.choice.value='2';s.listeners.defaults();assert.strictEqual(s.field('mass_g').value,'Неизвестно');
let imported={schema:1,values:{mass_g:'720',cg_x:'-3'}};await s.listeners.import({target:{files:[{size:50,text:async()=>JSON.stringify(imported)}],value:'x'}});assert.strictEqual(s.field('mass_g').value,'720');assert.strictEqual(s.field('cg_x').value,'-3');assert.strictEqual(s.storage,stored);
await s.listeners.import({target:{files:[{size:50,text:async()=>'{oops'}],value:'x'}});assert(s.status.textContent.includes('Не удалось загрузить файл'));assert.strictEqual(s.field('mass_g').value,'720');
await s.listeners.import({target:{files:[{size:50,text:async()=>JSON.stringify({schema:1,values:{mass_g:'-2'}})}],value:'x'}});assert.strictEqual(s.field('mass_g').value,'720');
await s.listeners.import({target:{files:[{size:50,text:async()=>JSON.stringify({schema:1,values:{evil:'x'}})}],value:'x'}});assert.strictEqual(s.field('mass_g').value,'720');
await s.listeners.import({target:{files:[{size:200000,text:async()=>''}],value:'x'}});assert.strictEqual(s.field('mass_g').value,'720');
s.accept=false;await s.listeners.import({target:{files:[{size:50,text:async()=>JSON.stringify({schema:1,values:{mass_g:'200'}})}],value:'x'}});assert.strictEqual(s.field('mass_g').value,'720');s.accept=true;
let t=setup(stored);assert.strictEqual(t.field('mass_g').value,'500');assert.strictEqual(t.field('cg_x').value,'-10');
let old=setup('{"schema":1,"values":{"mass_g":"700","cg_x":""}}');assert.strictEqual(old.field('mass_g').value,'700');assert.strictEqual(old.field('cg_x').value,'');assert.strictEqual(old.field('battery_cells').value,'');
let q=setup(null,true);q.submit();assert(q.status.textContent.includes('хранилище недоступно'));assert.strictEqual(q.field('mass_g').value,'931');
let bad=setup('{"schema":9,"values":{}}');assert(bad.status.textContent.includes('Не удалось загрузить'));assert.strictEqual(bad.storage,'{"schema":9,"values":{}}');
let tampered=setup('{"schema":1,"values":{"mass_g":"0x10"}}');assert(tampered.status.textContent.includes('некорректное число'));assert.strictEqual(tampered.field('mass_g').value,'931');
})().catch(e=>{console.error(e);process.exit(1)});
'''
    result = subprocess.run([node, '-e', harness, script, json.dumps(parser.fields)], capture_output=True, text=True,
                            timeout=10, check=False)
    assert result.returncode == 0, result.stderr


def test_catalog_sources_and_units_are_explicit():
    from urllib.parse import urlparse

    from bfemu.aircraft_presets import HARDWARE_PRESETS

    assert [p['id'] for p in HARDWARE_PRESETS] == [
        '1.5', '2', '2.5', '3', '3.5', '4', '5', '6', '8', '9', '10',
    ]
    for preset in HARDWARE_PRESETS:
        assert urlparse(preset['url']).scheme == 'https'
        assert preset['title'] and preset['notes'] and preset['motor'] and preset['prop']
        assert preset['kv'].isdigit()
        assert 'inertia_xx' not in preset and 'max_thrust' not in preset
    assert '40 мм' in HARDWARE_PRESETS[0]['title']
    assert 'не ровно 1,5' in HARDWARE_PRESETS[0]['notes']

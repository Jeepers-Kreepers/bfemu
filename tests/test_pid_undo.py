"""Отмена последнего применения в компактной панели PID."""
from __future__ import annotations

import shutil
import subprocess

import pytest

from bfemu.pid_trainer_page import PID_TRAINER_PAGE


def test_compact_undo_restores_last_application_and_refuses_conflicts():
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node.js не установлен: поведение страницы проверить нельзя")
    assert 'id="reload" disabled>Отменить изменения' in PID_TRAINER_PAGE
    assert 'id="discard" disabled>Отменить ввод' in PID_TRAINER_PAGE
    controller = PID_TRAINER_PAGE.split("const $=id=>", 1)[1].split("function chart(", 1)[0]
    script = r'''
const assert = require('assert');
const elements = {};
function elem(id) {
  if (!elements[id]) elements[id] = {value:'', disabled:false, textContent:'', className:'',
    listeners:{}, addEventListener(event, fn) {this.listeners[event] = fn}};
  return elements[id];
}
const document = {getElementById: elem};
const confirm = () => true;
let fc = {profile:0, values:{roll:{p:45,i:80,d:46}, pitch:{p:48,i:85,d:50}}};
let posts = [], blockGet = null, networkFailure = false;
function clone(value) {return JSON.parse(JSON.stringify(value))}
async function fetch(url, options) {
  if (!options || options.method !== 'POST') {
    if (blockGet) await blockGet;
    const params = new URL('http://fc' + url).searchParams;
    const axis = params.get('axis');
    return {ok:true, json:async()=>({profile:fc.profile, axis, values:clone(fc.values[axis])})};
  }
  if (networkFailure) throw Error('нет связи');
  const payload = JSON.parse(options.body); posts.push(clone(payload));
  const current = fc.values[payload.axis];
  if (payload.profile !== fc.profile || JSON.stringify(current) !== JSON.stringify(payload.expected))
    return {ok:false, json:async()=>({error:'Профиль или значения изменились; загрузите их заново.'})};
  fc.values[payload.axis] = clone(payload.values);
  return {ok:true, json:async()=>({profile:fc.profile, axis:payload.axis, values:clone(payload.values)})};
}
function observeAnalysis() {} function updateSettings() {} function drawAll() {}
elem('axis').value='roll';elem('scenario').value='step';
''' + "\nconst $=id=>" + controller + r'''
function edit(key,value) {elem(key).value=String(value);elem(key).listeners.input()}
async function apply(key,value) {edit(key,value);await elem('apply').onclick()}
(async()=>{
  await refresh(true);
  assert(elem('reload').disabled && elem('discard').disabled);
  await apply('p',65);
  assert.deepStrictEqual(fc.values.roll,{p:65,i:80,d:46});
  assert(!elem('reload').disabled);
  await elem('reload').onclick();
  assert.deepStrictEqual(fc.values.roll,{p:45,i:80,d:46});
  assert.strictEqual(elem('p').value,45);
  assert(elem('reload').disabled);
  assert(elem('status').textContent.includes('до последнего применения'));

  await apply('p',65); await apply('i',90);
  await elem('reload').onclick();
  assert.deepStrictEqual(fc.values.roll,{p:65,i:80,d:46},'один шаг до последнего применения');

  await apply('p',70);
  const count=posts.length;
  edit('d',75); await elem('reload').onclick();
  assert.strictEqual(posts.length,count,'отмена не теряет неприменённые правки');
  assert(elem('status').textContent.includes('неприменённые правки'));
  await elem('discard').onclick();
  assert.strictEqual(elem('d').value,46);
  assert(!elem('reload').disabled);

  fc.values.roll.p=120;
  await elem('reload').onclick();
  assert.strictEqual(fc.values.roll.p,120,'не перезаписывать внешнее изменение');
  assert(elem('status').textContent.includes('Отменить применение не удалось'));
  await refresh();
  assert(elem('status').textContent.includes('Отменить применение не удалось'), 'ошибка не стирается фоновым опросом');
  fc.values.roll.p=70; await refresh(true);
  fc.profile=1; await refresh(true); assert(elem('reload').disabled);
  fc.profile=0; await refresh(true); assert(!elem('reload').disabled);
  elem('axis').value='pitch'; elem('axis').onchange(); await refresh(true);
  assert(elem('reload').disabled);
  elem('axis').value='roll'; elem('axis').onchange(); await refresh(true);
  assert(!elem('reload').disabled);
  networkFailure=true; await elem('reload').onclick();
  assert.strictEqual(fc.values.roll.p,70);
  assert(elem('status').textContent.includes('нет связи'));
  networkFailure=false; await elem('reload').onclick();
  assert.strictEqual(fc.values.roll.p,65);

  // Применение во время незавершённого опроса дожидается повторной загрузки.
  let release; blockGet=new Promise(resolve=>release=resolve);
  const pending=refresh(); await Promise.resolve();
  edit('p',75); const writing=elem('apply').onclick();
  assert(elem('p').disabled && elem('reload').disabled);
  await Promise.resolve(); blockGet=null; release();
  await pending; await writing;
  assert.strictEqual(fc.values.roll.p,75);
  assert.strictEqual(elem('p').value,75);
  assert(!elem('reload').disabled && !elem('p').disabled);
  await elem('reload').onclick(); assert.strictEqual(fc.values.roll.p,65);
  console.log('compact undo OK');
})().catch(err=>{console.error(err);process.exitCode=1});
'''
    result = subprocess.run([node, "-e", script], capture_output=True, text=True, timeout=10, check=False)
    assert result.returncode == 0, result.stderr
    assert "compact undo OK" in result.stdout

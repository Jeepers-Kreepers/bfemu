"""Цветная, но понятная и без восприятия цвета оценка отдельных метрик."""
from __future__ import annotations

import shutil
import subprocess

import pytest

from bfemu.pid_trainer_page import PID_TRAINER_PAGE


def test_analysis_renders_each_outcome_and_unknown_results_neutrally():
    node = shutil.which('node')
    if node is None:
        pytest.skip('Node.js не установлен')
    functions = PID_TRAINER_PAGE.split('function element(', 1)[1].split('function flushAnalysis()', 1)[0]
    harness = r'''
const assert=require('assert');
class Node {
 constructor(tag){this.tag=tag;this.children=[];this.textContent='';this.className=''}
 append(...nodes){this.children.push(...nodes)}
 querySelector(s){return s==='summary'?this.summary:null}
}
const document={createElement(tag){return new Node(tag)}};
''' + '\nfunction element(' + functions + r'''
function render(report){
 const target=new Node('details');target.summary=new Node('summary');renderAnalysis(target,report);return target;
}
const change={label:'P',before:'1',after:'2',effect:'Смысл.',guidance:'Влияние.',model:'',related:false};
const texts=['Перерегулирование снизилось.','Время установления выросло.','Кривая изменилась.'];
const target=render({changes:[change],warnings:[],observations:texts,
 observation_details:[{text:texts[0],outcome:'positive'},{text:texts[1],outcome:'negative'},{text:texts[2],outcome:'neutral'}]});
const rows=target.children.filter(x=>x.tag==='p'&&x.className.startsWith('reading'));
assert.deepStrictEqual(rows.map(x=>x.className),['reading positive','reading negative','reading neutral']);
assert.deepStrictEqual(rows.map(x=>x.children[0].textContent),['Улучшение показателя. ','Ухудшение показателя. ','Без оценки. ']);
assert.deepStrictEqual(rows.map(x=>x.children[1].textContent),texts);
assert(target.children.some(x=>x.tag==='h3'&&x.textContent==='Что изменилось в учебной модели'));
// Старый ответ API и некорректная метка не должны окрашиваться как улучшение.
for(const details of [undefined,[{text:'другой текст',outcome:'positive'}],[{text:texts[0],outcome:'untrusted'}]]){
 let fallback=render({changes:[change],warnings:[],observations:[texts[0]],observation_details:details});
 let row=fallback.children.find(x=>x.tag==='p'&&x.className.startsWith('reading'));
 assert.strictEqual(row.className,'reading neutral');assert.strictEqual(row.children[0].textContent,'Без оценки. ');
}
'''
    result = subprocess.run([node, '-e', harness], capture_output=True, text=True, timeout=10, check=False)
    assert result.returncode == 0, result.stderr
    assert '.reading.positive{border-color:#6ee7a1' in PID_TRAINER_PAGE
    assert '.reading.negative{border-color:#ff7b82' in PID_TRAINER_PAGE
    assert '.reading{border-left:3px solid #8194a8' in PID_TRAINER_PAGE

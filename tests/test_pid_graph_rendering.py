"""Раздельная отрисовка составляющих и результирующего управляющего сигнала."""
from __future__ import annotations

import shutil
import subprocess

import pytest

from bfemu.pid_trainer_page import PID_TRAINER_PAGE


def test_separate_term_and_result_graphs_render_the_correct_series():
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node.js не установлен: отрисовку JavaScript проверить нельзя")
    chart_functions = PID_TRAINER_PAGE.split("function chart(", 1)[1].split("function settingStatus(", 1)[0]
    script = r'''
const assert = require('assert');
const elements = {}, strokes = [], clips = [], labels=[];
const samples = [
  [0, 0, 0, 0.1, 0.2, -0.1, 0, 0.3, 1300],
  [1.5, 200, 150, 2.1, 0.1, -1.2, 0.25, 0.6, 1300],
  [3, -200, -150, 1.5, 0.3, -2.0, -0.5, 0.2, 1300]
];
const latest = {points: samples, reference_points: [[0, 0], [1.5, 190], [3, -190]],
                metrics: {overshoot_pct: 0, settling_s: null, first_zero_error_s: null, end_error: 0}};
const saved = null, window = {devicePixelRatio: 1};
function $(id) {
  if (!elements[id]) {
    const ctx = {path: [], scale() {}, setLineDash() {}, fillText(text) {labels.push([id,text])}, save() {}, restore() {}, rect(x,y,w,h) {this.clipRect=[x,y,w,h]}, clip() {clips.push([id,...this.clipRect])},
      beginPath() {this.path = []}, moveTo(x,y) {this.path.push([x,y])},
      lineTo(x,y) {this.path.push([x,y])},
      stroke() {strokes.push({id, color:this.strokeStyle, width:this.lineWidth, path:this.path})}};
    elements[id] = {textContent: '', getBoundingClientRect() {return {width:600,height:190}},
                    getContext() {return ctx}};
  }
  return elements[id];
}
''' + "\nfunction chart(" + chart_functions + r'''
drawAll();
assert(labels.filter(([id])=>id==='graph').some(([,text])=>text.includes('°/с')));
for(const id of ['term-p','term-i','term-d','term-f','result-graph'])assert(labels.filter(([key])=>id===key).some(([,text])=>text.includes('б/р')));
const resultLines = strokes.filter(s => s.id === 'result-graph' && s.path.length === samples.length);
assert.strictEqual(resultLines.length, 1);
assert.strictEqual(resultLines[0].color, '#e6edf3');
assert.strictEqual(resultLines[0].width, 2);
const max = 0.5 * 1.1;
for (let i = 0; i < samples.length; i++) {
  const expectedY = 12 + (max - samples[i][6]) / (2 * max) * (190 - 12 - 25);
  assert(Math.abs(resultLines[0].path[i][1] - expectedY) < 1e-8);
}
const terms = [
  ['term-p', 3, '#65d6b3'], ['term-i', 4, '#e3b45b'],
  ['term-d', 5, '#ae9dfc'], ['term-f', 7, '#fb8c8c']
];
function checkTerms(rows) {
  for (const [id, col, color] of terms) {
    const paths = strokes.filter(s => s.id === id && s.path.length === rows.length);
    assert.strictEqual(paths.length, 1, id + ': только одна кривая');
    assert.strictEqual(paths[0].color, color);
    const scale = Math.max(0.05, ...rows.map(row => Math.abs(row[col]))) * 1.1;
    for (let i = 0; i < rows.length; i++) {
      const expectedY = 12 + (scale - rows[i][col]) / (2 * scale) * (190 - 12 - 25);
      const expectedX = 72 + rows[i][0] / 3 * (600 - 72 - 12);
      assert(Math.abs(paths[0].path[i][1] - expectedY) < 1e-8, id + ': собственная шкала и столбец');
      assert(Math.abs(paths[0].path[i][0] - expectedX) < 1e-8, id + ': единая шкала времени');
    }
  }
}
checkTerms(samples);
// После обновления данных все четыре графика строятся заново. Нулевая серия
// тоже получает конечную шкалу и располагается на нулевой линии.
strokes.length = 0;
latest.points = samples.map((row, i) => [row[0], row[1], row[2], row[3] * 2, 0, row[5] / 2, row[6], -row[7], row[8]]);
drawAll();
checkTerms(latest.points);
assert(!strokes.some(s => s.id === 'terms'), 'совмещённый график удалён');
// Увеличение меняет координаты и обрезает линии по границам графика, а не данные.
strokes.length = 0; clips.length = 0;
chart($('zoomed'),samples,[{rows:samples,col:2,color:'white'}],-100,100,null,[.75,2.25]);
const zoomLine=strokes.find(s=>s.id==='zoomed'&&s.path.length===3);
assert(zoomLine.path[0][0]<72 && zoomLine.path[2][0]>588);
assert(zoomLine.path[1][1]<12); // 150°/с находится за пределом ±100.
assert.deepStrictEqual(clips[0],['zoomed',72,12,516,153]);

'''
    result = subprocess.run([node, "-e", script], capture_output=True, text=True, timeout=10, check=False)
    assert result.returncode == 0, result.stderr

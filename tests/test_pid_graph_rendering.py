"""Результирующий график отрисовывает отдельную кривую тех же данных, что общая диаграмма."""
from __future__ import annotations

import shutil
import subprocess

import pytest

from bfemu.pid_trainer_page import PID_TRAINER_PAGE


def test_resultant_graph_renders_control_signal():
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node.js не установлен: отрисовку JavaScript проверить нельзя")
    chart_functions = PID_TRAINER_PAGE.split("function chart(", 1)[1].split("function settingStatus(", 1)[0]
    script = r'''
const assert = require('assert');
const elements = {}, strokes = [];
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
    const ctx = {path: [], scale() {}, setLineDash() {}, fillText() {},
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
const resultLines = strokes.filter(s => s.id === 'result-graph' && s.path.length === samples.length);
assert.strictEqual(resultLines.length, 1);
assert.strictEqual(resultLines[0].color, '#e6edf3');
assert.strictEqual(resultLines[0].width, 2);
const max = 0.5 * 1.1;
for (let i = 0; i < samples.length; i++) {
  const expectedY = 12 + (max - samples[i][6]) / (2 * max) * (190 - 12 - 25);
  assert(Math.abs(resultLines[0].path[i][1] - expectedY) < 1e-8);
}
const combined = strokes.find(s => s.id === 'terms' && s.color === '#e6edf3' && s.path.length === samples.length);
assert(combined, 'совмещённый график по-прежнему содержит ту же результирующую кривую');
'''
    result = subprocess.run([node, "-e", script], capture_output=True, text=True, timeout=10, check=False)
    assert result.returncode == 0, result.stderr

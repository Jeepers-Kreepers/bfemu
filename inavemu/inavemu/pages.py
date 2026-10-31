"""Страницы веб-интерфейса inavemu (стиль bfemu/pages.py).

Подсказки и правила те же, что у bfemu:

  * одинаковая кнопочная навигация на всех страницах, текущая подсвечена;
  * тёмная тема, никаких внешних ресурсов — работает в закрытой сети и без CDN;
  * состояние тянется из /api/state раз в секунду, переключатели шлют
    POST /api/equipment.

Что здесь есть:

  * NAV_STYLE + nav_html() — полоска кнопок «Монитор · Оборудование»;
  * monitor_page() — «Монитор»: карточки состояния виртуального FC и журнал;
  * equipment_page() — «Оборудование»: тумблеры подключения датчиков к
    виртуальному FC (гироскоп, акселерометр, магнитометр, барометр, GPS,
    оптический поток, сонар, pitot) + такая же полоска индикаторов, как в
    INAV Configurator (Gyro · Accel · Mag · Baro · GPS · Flow · Sonar · Speed).
"""
from __future__ import annotations

from html import escape as _esc_raw


def _esc(value) -> str:
    return _esc_raw(str(value), quote=True)


NAV_STYLE = """
 body{margin:0;background:#0d1117;color:#e6edf3;
  font:15px/1.5 -apple-system,Segoe UI,Roboto,sans-serif}
 .nav{display:flex;gap:8px;flex-wrap:wrap;align-items:center;padding:10px 24px;background:#0d1117;
  border-bottom:1px solid #30363d;position:sticky;top:0;z-index:5}
 .nav a{display:inline-block;background:#21262d;border:1px solid #30363d;border-radius:8px;
  padding:6px 13px;color:#e6edf3;text-decoration:none;font:600 13px/1.4 -apple-system,Segoe UI,Roboto,sans-serif}
 .nav a:hover{border-color:#58a6ff}
 .nav a.on{background:#1f6feb;border-color:#1f6feb;color:#fff}
 .nav .sp{flex:1}
 .nav .bd{color:#8b949e;font-size:12.5px;margin-right:8px}
 .nav .bd code{color:#7ee787}
 .wrap{max-width:980px;margin:0 auto;padding:18px 24px 40px}
 h1{font-size:21px;margin:14px 0 6px}
 h2{font-size:16px;margin:22px 0 10px;color:#e6edf3}
 .hint{color:#8b949e;font-size:13.5px;margin:0 0 16px}
 .grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(215px,1fr));gap:12px}
 .card{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:13px 15px}
 .card .nm{font-weight:600;font-size:14.5px}
 .card .en{color:#8b949e;font-size:12px;margin-top:1px}
 .card .st{margin:9px 0 11px;font-size:12.5px;color:#8b949e}
 .card .st b{color:#8b949e;font-weight:600}
 .card.on .st b{color:#7ee787}
 .sw{display:inline-block;width:46px;height:25px;border-radius:13px;background:#30363d;
  border:1px solid #30363d;position:relative;cursor:pointer;transition:background .15s}
 .sw::after{content:"";position:absolute;top:2px;left:2px;width:19px;height:19px;border-radius:50%;
  background:#8b949e;transition:left .15s,background .15s}
 .sw[aria-pressed="true"]{background:#1f6feb;border-color:#1f6feb}
 .sw[aria-pressed="true"]::after{left:23px;background:#fff}
 .strip{display:flex;gap:8px;flex-wrap:wrap;margin:12px 0 4px}
 .strip .t{flex:1 1 84px;min-width:84px;text-align:center;background:#161b22;border:1px solid #30363d;
  border-radius:10px;padding:12px 6px 10px;color:#484f58;transition:color .2s,border-color .2s}
 .strip .t .ic{font-size:22px;line-height:1}
 .strip .t .lb{font-size:11.5px;margin-top:6px;letter-spacing:.2px}
 .strip .t.on{color:#58a6ff;border-color:#1f6feb}
 .strip .t.on .ic{filter:drop-shadow(0 0 6px rgba(56,139,253,.45))}
 .cards2{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:12px}
 .kv{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:11px 14px}
 .kv .k{color:#8b949e;font-size:12px}
 .kv .v{font-size:17px;font-weight:650;margin-top:2px;word-break:break-all}
 .log{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:12px 15px;
  font:12.5px/1.65 ui-monospace,Consolas,monospace;color:#adbac7;white-space:pre-wrap}
 .warn{color:#e3b341}
 .step{display:flex;align-items:center;gap:12px;margin:8px 0 4px}
 .step button{width:38px;height:38px;border-radius:10px;border:1px solid #30363d;
  background:#21262d;color:#e6edf3;font-size:20px;font-weight:700;cursor:pointer}
 .step button:hover{border-color:#58a6ff}
 .step .num{font-size:27px;font-weight:700;min-width:46px;text-align:center}
 .step .lbl{color:#8b949e;font-size:13px}
 .quick{display:flex;gap:7px;flex-wrap:wrap;margin:6px 0 2px}
 .quick button{background:#21262d;border:1px solid #30363d;border-radius:7px;
  padding:4px 11px;color:#e6edf3;font-size:12.5px;cursor:pointer}
 .quick button:hover{border-color:#58a6ff}
 .quick button.on{background:#1f6feb;border-color:#1f6feb;color:#fff}
 .chips{display:flex;gap:7px;flex-wrap:wrap;margin:10px 0 4px}
 .chip{min-width:52px;text-align:center;background:#161b22;border:1px solid #30363d;
  border-radius:8px;padding:7px 8px;color:#484f58;font-size:12px;transition:color .2s,border-color .2s}
 .chip b{display:block;font-size:13px;margin-bottom:2px}
 .chip.on{color:#7ee787;border-color:#2ea043}
 .chip .val{font:11px/1.3 ui-monospace,Consolas,monospace;color:#8b949e}
"""


def nav_html(active: str, tcp: str = "", http_port: int = 0) -> str:
    items = (("monitor", "/", "Монитор"), ("equipment", "/equipment", "Оборудование"),
             ("outputs", "/outputs", "Моторы и сервы"), ("rc", "/rc", "Пульт"))
    links = "".join(
        f'<a href="{href}" class="{"on" if key == active else ""}">{label}</a>'
        for key, href, label in items
    )
    bd = ""
    if tcp:
        bd += f'<span class="bd">FC <code>{_esc(tcp)}</code></span>'
    return f'<div class="nav">{links}<span class="sp"></span>{bd}</div>'


_MONITOR_JS = """
<script>
const $ = (id) => document.getElementById(id);
function human(v){
  if (v && typeof v === 'object'){
    return Object.entries(v).map(([k,x]) => k + ': ' + x).join(' · ');
  }
  return String(v);
}
function tick(){
  fetch('/api/state').then(r => r.json()).then(st => {
    const g = $('kv'); if (!g) return;
    g.innerHTML = [
      ['Прошивка', 'INAV ' + st.fc_version],
      ['MSP API', st.api_version],
      ['Плата', st.board + ' / ' + st.target],
      ['Имя', st.craft_name],
      ['Клиентов', st.clients],
      ['Запросов MSP', st.requests],
      ['Настроек', st.settings],
      ['Высота, см', st.altitude_cm],
      ['Крен/Тангаж/Рыск', human(st.attitude)],
      ['GPS', 'fix ' + st.gps.fix + ' · спутников ' + st.gps.sats],
    ].map(([k, v]) => '<div class="kv"><div class="k">' + k + '</div><div class="v">' + v + '</div></div>').join('');
    const eq = Object.entries(st.equipment || {})
      .map(([k, on]) => (on ? '● ' : '○ ') + k).join('  ');
    if ($('eqline')) $('eqline').textContent = eq;
    if ($('log')) $('log').textContent = (st.log || []).join('\\n');
  }).catch(() => {});
}
tick();
setInterval(tick, 1000);
</script>
"""

_EQ_JS = """
<script>
const NAMES = [
  ['gyro', 'Гироскоп', 'Gyro'],
  ['acc', 'Акселерометр', 'Accel'],
  ['mag', 'Магнитометр', 'Mag'],
  ['baro', 'Барометр', 'Baro'],
  ['gps', 'GPS', 'GPS'],
  ['opflow', 'Оптический поток', 'Flow'],
  ['sonar', 'Сонар', 'Sonar'],
  ['pitot', 'Скорость (pitot)', 'Speed'],
];
let state = {};

function render(){
  for (const [key] of NAMES){
    const on = !!state[key];
    const sw = document.getElementById('sw_' + key);
    const card = document.getElementById('card_' + key);
    const st = document.getElementById('st_' + key);
    const tile = document.getElementById('tile_' + key);
    if (sw) sw.setAttribute('aria-pressed', on ? 'true' : 'false');
    if (card) card.classList.toggle('on', on);
    if (st) st.innerHTML = on ? '<b>подключено</b>' : '<b>нет</b>';
    if (tile) tile.classList.toggle('on', on);
  }
}

function load(){
  fetch('/api/state').then(r => r.json()).then(st => {
    state = st.equipment || {};
    render();
  }).catch(() => {});
}

function toggle(key){
  const on = !state[key];
  state[key] = on;
  render();
  fetch('/api/equipment', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({[key]: on}),
  }).then(r => {
    if (!r.ok) throw new Error('HTTP ' + r.status);
    return r.json();
  }).then(js => {
    state = js.equipment || state;
    render();
  }).catch(() => load());
}

document.addEventListener('DOMContentLoaded', () => {
  for (const [key] of NAMES){
    const sw = document.getElementById('sw_' + key);
    if (sw) sw.addEventListener('click', () => toggle(key));
  }
  load();
  setInterval(load, 2000);
});
</script>
"""


def monitor_page(nav: str, style: str) -> str:
    return f"""<!doctype html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>inavemu — Монитор</title><style>{style}</style></head>
<body>{nav}
<div class="wrap">
 <h1>inavemu — виртуальный FC INAV</h1>
 <p class="hint">Состояние виртуального контроллера. Подключайте и отключайте
 оборудование на странице «Оборудование». Полоска <b>Оборудование</b> ниже —
 те же индикаторы, что в INAV Configurator (Gyro · Accel · Mag · Baro · GPS ·
 Flow · Sonar · Speed).</p>
 <div class="cards2" id="kv"></div>
 <h2>Оборудование</h2>
 <div class="hint" id="eqline">…</div>
 <h2>Журнал</h2>
 <div class="log" id="log"></div>
</div>
{_MONITOR_JS}
</body></html>"""


def equipment_page(nav: str, style: str) -> str:
    tiles = "".join(
        f'<div class="t" id="tile_{key}"><div class="ic">{icon}</div>'
        f'<div class="lb">{en}</div></div>'
        for key, icon, en in (
            ("gyro", "◎", "Gyro"), ("acc", "✈", "Accel"), ("mag", "▲", "Mag"),
            ("baro", "♨", "Baro"), ("gps", "◈", "GPS"), ("opflow", "◉", "Flow"),
            ("sonar", "≋", "Sonar"), ("pitot", "↻", "Speed"),
        )
    )
    cards = "".join(
        f'<div class="card" id="card_{key}">'
        f'<div class="nm">{ru}</div><div class="en">{en}</div>'
        f'<div class="st" id="st_{key}">…</div>'
        f'<button class="sw" id="sw_{key}" aria-pressed="false" '
        f'aria-label="Подключить: {ru}"></button></div>'
        for key, ru, en in (
            ("gyro", "Гироскоп", "Gyro"),
            ("acc", "Акселерометр", "Accel"),
            ("mag", "Магнитометр", "Mag"),
            ("baro", "Барометр", "Baro"),
            ("gps", "GPS", "GPS"),
            ("opflow", "Оптический поток", "Optical flow"),
            ("sonar", "Сонар", "Sonar / rangefinder"),
            ("pitot", "Скорость (pitot)", "Airspeed / pitot"),
        )
    )
    return f"""<!doctype html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>inavemu — Оборудование</title><style>{style}</style></head>
<body>{nav}
<div class="wrap">
 <h1>Виртуальное оборудование</h1>
 <p class="hint">Тумблеры подключают оборудование к виртуальному FC — как
 датчики на настоящем контроллере. Индикаторы в INAV Configurator
 (Gyro · Accel · Mag · Baro · GPS · Flow · Sonar · Speed) следят за этим
 состоянием: включено — горит, отключено — погашено. Кнопки калибровки
 магнитометра/оптического потока и выбор датчиков во вкладке Configuration
 тоже подчиняются этим переключателям.</p>
 <h2>Индикаторы (как в Configurator)</h2>
 <div class="strip">{tiles}</div>
 <h2>Подключение</h2>
 <div class="grid">{cards}</div>
</div>
{_EQ_JS}
</body></html>"""


_OUT_JS = """
<script>
const $ = (id) => document.getElementById(id);
let st = {motors: 0, servos: 0, motor_values: [], servo_values: []};

function render(){
  $('mnum').textContent = st.motors;
  $('mnum2').textContent = st.motors;
  $('snum').textContent = st.servos;
  $('snum2').textContent = st.servos;
  for (let i = 1; i <= 12; i++){
    const chip = $('m' + i);
    const on = i <= st.motors;
    chip.classList.toggle('on', on);
    chip.querySelector('.val').textContent =
      on ? (st.motor_values[i - 1] ?? '—') : 'нет';
  }
  for (let i = 1; i <= 18; i++){
    const chip = $('s' + i);
    const on = i <= st.servos;
    chip.classList.toggle('on', on);
    chip.querySelector('.val').textContent =
      on ? (st.servo_values[i - 1] ?? '—') : 'нет';
  }
  for (const b of document.querySelectorAll('.quick [data-k]')){
    b.classList.toggle('on', +b.dataset.k === st[b.dataset.kind]);
  }
}

function load(){
  fetch('/api/state').then(r => r.json()).then(js => {
    st = js.outputs || st;
    render();
  }).catch(() => {});
}

function setCount(kind, val){
  fetch('/api/outputs', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({[kind]: val}),
  }).then(r => {
    if (!r.ok) throw new Error('HTTP ' + r.status);
    return r.json();
  }).then(js => {
    st = js.outputs || st;
    render();
  }).catch(() => load());
}

document.addEventListener('DOMContentLoaded', () => {
  $('mminus').addEventListener('click', () => setCount('motors', st.motors - 1));
  $('mplus').addEventListener('click', () => setCount('motors', st.motors + 1));
  $('sminus').addEventListener('click', () => setCount('servos', st.servos - 1));
  $('splus').addEventListener('click', () => setCount('servos', st.servos + 1));
  for (const b of document.querySelectorAll('.quick [data-k]')){
    b.addEventListener('click', () => setCount(b.dataset.kind, +b.dataset.k));
  }
  load();
  setInterval(load, 1000);
});
</script>
"""


def outputs_page(nav: str, style: str) -> str:
    motor_chips = "".join(
        f'<div class="chip" id="m{i}"><b>M{i}</b><span class="val">—</span></div>'
        for i in range(1, 13)
    )
    servo_chips = "".join(
        f'<div class="chip" id="s{i}"><b>S{i}</b><span class="val">—</span></div>'
        for i in range(1, 19)
    )
    motor_quick = "".join(
        f'<button data-k="{k}" data-kind="motors">{k}</button>'
        for k in (1, 2, 4, 6, 8, 12)
    )
    servo_quick = "".join(
        f'<button data-k="{k}" data-kind="servos">{k}</button>'
        for k in (0, 2, 3, 4, 6, 8, 18)
    )
    return f"""<!doctype html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>inavemu — Моторы и сервы</title><style>{style}</style></head>
<body>{nav}
<div class="wrap">
 <h1>Моторы и сервоприводы</h1>
 <p class="hint">Счётчики задают, сколько моторов и серв «подключено» к
 виртуальному FC. Это mixer-правила: во вкладке Outputs конфигуратора
 появляется столько слайдеров моторов и ярких полос серв, сколько
 подключено. Тест моторов (слайдеры Outputs) работает: значения
 возвращаются в MSP_MOTOR.</p>

 <h2>Моторы: <span id="mnum">0</span> из 12</h2>
 <div class="step">
  <button id="mminus" aria-label="меньше моторов">−</button>
  <div class="num" id="mnum2"></div>
  <button id="mplus" aria-label="больше моторов">+</button>
  <span class="lbl">от 1 до 12 · быстро:</span>
  <div class="quick">{motor_quick}</div>
 </div>
 <div class="chips">{motor_chips}</div>

 <h2>Сервоприводы: <span id="snum">0</span> из 18</h2>
 <div class="step">
  <button id="sminus" aria-label="меньше серв">−</button>
  <div class="num" id="snum2"></div>
  <button id="splus" aria-label="больше серв">+</button>
  <span class="lbl">от 0 до 18 · быстро:</span>
  <div class="quick">{servo_quick}</div>
 </div>
 <div class="chips">{servo_chips}</div>
</div>
{_OUT_JS}
</body></html>"""


# ======================== пульт: каналы RC + Gamepad ========================

_RC_STYLE = """
 .rcrow{display:flex;align-items:center;gap:12px;margin:8px 0 4px;flex-wrap:wrap}
 .rcrow .lbl{color:#8b949e;font-size:13px;min-width:126px}
 .rcrow input[type=range]{flex:1;min-width:160px;accent-color:#1f6feb}
 .rcrow input[type=number]{width:80px;background:#21262d;color:#e6edf3;
  border:1px solid #30363d;border-radius:7px;padding:6px 8px;font-size:14px}
 .maprow select{background:#21262d;color:#e6edf3;border:1px solid #30363d;
  border-radius:7px;padding:5px 8px;font-size:13px;min-width:168px}
 .maprow label{color:#8b949e;font-size:12.5px}
 .maprow .chip{min-width:58px}
"""

_RC_JS = """
<script>
const $ = (id) => document.getElementById(id);
const CH = 8, VMIN = 875, VMAX = 2125;
const clamp = (v) => Math.max(VMIN, Math.min(VMAX, v));
const esc = (s) => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;')
                            .replace(/"/g, '&quot;');
let channels = new Array(16).fill(1500);
channels[2] = 1000;
let map = Array.from({length: CH}, () => ({src: '', inv: false}));
let gpOn = false, gpSel = -1, sendT = null, lastKey = '';

function saveMap(){
  try { localStorage.setItem('inavemu_rc_map',
        JSON.stringify({map: map, gpOn: gpOn, gpSel: gpSel})); } catch (e) {}
}
(function loadMap(){
  try {
    const s = JSON.parse(localStorage.getItem('inavemu_rc_map') || 'null');
    if (s && Array.isArray(s.map)){
      map = s.map; gpOn = !!s.gpOn; gpSel = (s.gpSel == null ? -1 : s.gpSel);
    }
  } catch (e) {}
})();

function render(){
  for (let i = 0; i < CH; i++){
    const v = Math.round(channels[i]);
    const sl = $('sl' + i), nu = $('num' + i), gv = $('gpv' + i);
    if (document.activeElement !== sl) sl.value = v;
    if (document.activeElement !== nu) nu.value = v;
    if (gv) gv.textContent = v;
    $('row' + i).querySelectorAll('.quick button').forEach((b) => {
      b.classList.toggle('on', +b.dataset.v === v);
    });
  }
}

function doSend(){
  const body = channels.slice(0, CH).map((v) => Math.round(v));
  const key = body.join(',');
  if (key === lastKey) return;
  lastKey = key;
  fetch('/api/rc', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({channels: body}),
  }).then((r) => { if (!r.ok) throw new Error('http'); return r.json(); })
    .catch(() => { lastKey = ''; });
}
function send(){
  if (sendT) return;
  sendT = setTimeout(() => { sendT = null; doSend(); }, 45);
}
function setCh(i, v){
  if (!isFinite(v)) return;
  channels[i] = clamp(Math.round(v));
  render(); send();
}

for (let i = 0; i < CH; i++){
  $('sl' + i).addEventListener('input', (e) => setCh(i, +e.target.value));
  $('num' + i).addEventListener('input', (e) => setCh(i, +e.target.value));
}
$('rcrows').addEventListener('click', (e) => {
  const b = e.target.closest('button[data-v]');
  if (b) setCh(+b.dataset.ch, +b.dataset.v);
});
$('rc_neutral').addEventListener('click', () => {
  for (let i = 0; i < CH; i++) channels[i] = 1500;
  channels[2] = 1000;
  render(); send();
});

// ------------------------------ Gamepad ------------------------------
function mapValue(pad, m){
  const s = m.src || '';
  if (!s) return null;
  const k = s.charAt(0), n = +s.slice(1);
  if (k === 'a'){
    const a = pad.axes[n];
    if (typeof a !== 'number' || !isFinite(a)) return null;
    return clamp(Math.round(1500 + (m.inv ? -1 : 1) * a * 500));
  }
  const btn = (j) => !!(pad.buttons[j] && pad.buttons[j].pressed);
  if (k === 'b') return btn(n) ? 2000 : 1000;
  if (k === 't') return btn(n + 1) ? 2000 : (btn(n) ? 1000 : 1500);
  return null;
}

function refreshPads(){
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  const sel = $('gp_sel'), cur = sel.value;
  let html = '<option value="-1">автоматически</option>';
  for (const p of pads){
    if (!p) continue;
    html += '<option value="' + p.index + '">#' + p.index + ' ' +
            esc(p.id.slice(0, 52)) + '</option>';
  }
  sel.innerHTML = html;
  sel.value = cur;
  if (sel.selectedIndex < 0) sel.value = '-1';
}

function poll(){
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  let pad = null, found = 0;
  for (const p of pads){
    if (!p) continue;
    found++;
    if (!pad && (gpSel < 0 || p.index === gpSel)) pad = p;
  }
  if (!pad){
    $('gp_status').textContent = found
      ? 'устройство видно — выберите его в списке'
      : 'аппаратура не видна: USB + режим USB Joystick, затем шевельните палкой/кнопкой';
    $('gp_axes').innerHTML = '';
    $('gp_btns').innerHTML = '';
    return;
  }
  $('gp_status').textContent = pad.id.slice(0, 60) + ' · осей ' + pad.axes.length +
    ', кнопок ' + pad.buttons.length + (gpOn ? ' · УПРАВЛЕНИЕ ВКЛ' : '');
  $('gp_axes').innerHTML = pad.axes.map((a, j) =>
    '<div class="chip' + (Math.abs(a) > 0.02 ? ' on' : '') +
    '"><b>A' + j + '</b>' + a.toFixed(2) + '</div>').join('');
  $('gp_btns').innerHTML = pad.buttons.map((b, j) =>
    '<div class="chip' + (b.pressed ? ' on' : '') +
    '"><b>B' + j + '</b>' + (b.pressed ? 'ON' : '—') + '</div>').join('');
  if (!gpOn) return;
  let changed = false;
  for (let i = 0; i < CH; i++){
    const v = mapValue(pad, map[i]);
    if (v != null && Math.round(channels[i]) !== v){ channels[i] = v; changed = true; }
  }
  if (changed){ render(); send(); }
}

for (let i = 0; i < CH; i++){
  const sel = $('gps' + i), inv = $('gpi' + i);
  sel.value = map[i].src || '';
  inv.checked = !!map[i].inv;
  sel.addEventListener('change', () => { map[i].src = sel.value; saveMap(); });
  inv.addEventListener('change', () => { map[i].inv = inv.checked; saveMap(); });
}
$('gp_on').checked = gpOn;
$('gp_on').addEventListener('change', (e) => { gpOn = e.target.checked; saveMap(); });
$('gp_sel').value = String(gpSel);
$('gp_sel').addEventListener('change', (e) => { gpSel = +e.target.value; saveMap(); });
$('gp_preset').addEventListener('click', () => {
  // Radiomaster Zorro (EdgeTX, USB Joystick): стики — оси 0..3, тумблеры — кнопки
  const src = ['a0', 'a1', 'a2', 'a3', 'b0', 'b1', 't2', 'b4'];
  const inv = [false, true, true, false, false, false, false, false];
  for (let i = 0; i < CH; i++){
    map[i] = {src: src[i], inv: inv[i]};
    $('gps' + i).value = src[i];
    $('gpi' + i).checked = inv[i];
  }
  saveMap();
});
$('gp_clear').addEventListener('click', () => {
  for (let i = 0; i < CH; i++){
    map[i] = {src: '', inv: false};
    $('gps' + i).value = '';
    $('gpi' + i).checked = false;
  }
  saveMap();
});

window.addEventListener('gamepadconnected', refreshPads);
window.addEventListener('gamepaddisconnected', refreshPads);
setInterval(poll, 50);
fetch('/api/rc').then((r) => r.json()).then((js) => {
  if (js && js.channels) js.channels.forEach((v, i) => { channels[i] = v; });
  render();
}).catch(() => render());
refreshPads();
</script>
"""


def _map_options() -> str:
    """Опции <select> привязки канала → ось/кнопка Gamepad."""
    parts = ['<option value="">— не привязан</option>',
             '<optgroup label="Оси (стик/крутилка)">']
    for a in range(8):
        parts.append(f'<option value="a{a}">Ось {a}</option>')
    parts.append('</optgroup><optgroup label="Кнопка — 2 позиции">')
    for b in range(12):
        parts.append(f'<option value="b{b}">Кнопка {b}</option>')
    parts.append('</optgroup><optgroup label="Пара кнопок — 3 позиции">')
    for b in range(0, 12, 2):
        parts.append(f'<option value="t{b}">Кнопки {b}+{b + 1}</option>')
    parts.append('</optgroup>')
    return "".join(parts)


def rc_page(nav: str, style: str = "") -> str:
    labels = ("CH1 · AIL (Roll)", "CH2 · ELE (Pitch)", "CH3 · THR (Throttle)",
              "CH4 · RUD (Yaw)", "CH5 · AUX1", "CH6 · AUX2",
              "CH7 · AUX3", "CH8 · AUX4")
    opts = _map_options()
    rows, maprows = [], []
    for i, label in enumerate(labels):
        rows.append(f"""
<div class="rcrow" id="row{i}">
 <span class="lbl">{label}</span>
 <input type="range" id="sl{i}" min="875" max="2125" step="1" value="1500">
 <input type="number" id="num{i}" min="875" max="2125" step="1" value="1500">
 <div class="quick">
  <button data-ch="{i}" data-v="1000">1000</button>
  <button data-ch="{i}" data-v="1500">1500</button>
  <button data-ch="{i}" data-v="2000">2000</button>
 </div>
</div>""")
        maprows.append(f"""
<div class="step maprow">
 <span class="lbl" style="min-width:126px">{label}</span>
 <select id="gps{i}">{opts}</select>
 <label><input type="checkbox" id="gpi{i}"> инверсия</label>
 <span class="chip" id="gpv{i}">1500</span>
</div>""")
    return f"""<!doctype html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Пульт</title><style>{style}{_RC_STYLE}</style></head>
<body>
{nav}
<div class="wrap">
 <h1>Пульт: управление каналами RC</h1>
 <p class="hint">Каналы ведут виртуальный приёмник: значения отдаются через MSP_RC,
 и полосы каналов вкладки Receiver в INAV Configurator двигаются вместе с ними.
 Управлять можно слайдерами/кнопками ниже, а также с реальной аппаратуры
 (например Radiomaster Zorro), подключённой к USB порту компьютера — браузер
 видит её как джойстик (Gamepad API) и передаёт сюда.</p>

 <h2>Каналы CH1…CH8 (слайдеры и кнопки)</h2>
 <div id="rcrows">{''.join(rows)}</div>
 <div class="quick">
  <button id="rc_neutral">Нейтраль: стики 1500, газ 1000</button>
 </div>

 <h2>Аппаратура по USB (Gamepad)</h2>
 <p class="hint">1) Подключите аппаратуру к USB и включите режим USB Joystick
 (Radio Setup → USB → Joystick). 2) Нажмите/шевельните любую палку или кнопку —
 браузер разрешит доступ и покажет состояние ниже. 3) Включите «Управление
 с аппаратуры» и привяжите оси и кнопки к каналам (пресет Zorro — стики на CH1–CH4,
 тумблеры на CH5–CH8). Пока галочка включена, аппаратура перехватывает слайдеры.</p>
 <div class="card">
  <label><input type="checkbox" id="gp_on"> Управление с аппаратуры</label>
  <div class="step">
   <span class="lbl">Устройство</span>
   <select id="gp_sel"><option value="-1">автоматически</option></select>
  </div>
  <p class="hint" id="gp_status">аппаратура не видна</p>
  <div class="chips" id="gp_axes"></div>
  <div class="chips" id="gp_btns"></div>
  {''.join(maprows)}
  <div class="quick">
   <button id="gp_preset">Пресет: Radiomaster Zorro</button>
   <button id="gp_clear">Сбросить привязки</button>
  </div>
 </div>
</div>
{_RC_JS}
</body></html>"""

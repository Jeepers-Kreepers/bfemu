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
"""


def nav_html(active: str, tcp: str = "", http_port: int = 0) -> str:
    items = (("monitor", "/", "Монитор"), ("equipment", "/equipment", "Оборудование"))
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

"""Страница /failsafe — демонстрация работы failsafe для проектора.

Зачем отдельная страница: на вкладке «Failsafe» в Configurator видно только настройки, а
здесь показано, **что борт делает по шагам** — страж-время (Этап 1) с подстановкой значений
каналов, ТОЧКА ОТКАТА (вход в Этап 2), процедура (AUTO-LAND / DROP / GPS-RESCUE), моторы в 0
и запрет взвода, возврат сигнала и выход в IDLE. Времена и порядок — из failsafe.c 4.5.4,
масштаб времени (×1 / ×0.5 / ×0.25) только растягивает показ.

Как и остальные страницы эмулятора, без внешних ресурсов: ни CDN, ни шрифтов из сети —
страница должна работать в закрытой сети и на проекторе в аудитории.
"""

FAILSAFE_PAGE = r"""<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Failsafe — демонстрация</title>
<style>
 body{margin:0;background:#0b0f14;color:#e6edf3;font:15px/1.5 -apple-system,Segoe UI,Roboto,sans-serif}
 header{padding:16px 24px 12px;background:#161b22;border-bottom:1px solid #30363d}
 h1{margin:0 0 4px;font-size:22px}
 .sub{color:#8b949e;font-size:13px} .sub a,.sub code{color:#58a6ff} .sub code{color:#7ee787}
 main{padding:16px 24px 40px;display:grid;gap:14px}
 .card{background:#11161d;border:1px solid #30363d;border-radius:12px;padding:14px 16px}
 .card h2{margin:0 0 10px;font-size:15px;color:#8b949e;font-weight:600;letter-spacing:.2px;text-transform:uppercase}
 .btn{display:inline-block;background:#21262d;border:1px solid #30363d;border-radius:9px;padding:11px 16px;
  color:#e6edf3;text-decoration:none;font:600 15px/1.2 -apple-system,Segoe UI,Roboto,sans-serif;margin:0 8px 8px 0;
  cursor:pointer}
 .btn:hover{border-color:#58a6ff}
 .btn.red{background:#3d1f24;border-color:#a33;color:#ffb4b4}
 .btn.green{background:#16301f;border-color:#2ea043;color:#c9f7d4}
 .btn.blue{background:#152a4a;border-color:#1f6feb;color:#cfe3ff}
 .btn.on{outline:2px solid #1f6feb}
 .btn.small{padding:7px 11px;font-size:13.5px}
 .row{display:flex;flex-wrap:wrap;gap:14px}
 .row > .card{flex:1 1 340px;min-width:300px}
 .phase{font-size:26px;font-weight:700;margin:2px 0 6px}
 .phase .en{color:#8b949e;font-size:13px;font-weight:400;margin-left:8px}
 .big{font-size:19px}
 .bar{height:14px;background:#21262d;border-radius:7px;overflow:hidden;margin:8px 0 4px}
 .bar i{display:block;height:100%;background:#1f6feb;width:0}
 .bar.warn i{background:#d29922} .bar.bad i{background:#f85149}
 .plate{border-radius:10px;padding:10px 13px;font-weight:700;display:inline-block;margin:6px 0}
 .plate.ok{background:#16301f;border:1px solid #2ea043;color:#c9f7d4}
 .plate.no{background:#3d1f24;border:1px solid #f85149;color:#ffb4b4}
 .tl{display:grid;gap:8px;margin:4px 0 0}
 .tl .step{display:grid;grid-template-columns:26px 1fr auto;gap:10px;align-items:center;
  background:#161b22;border:1px solid #30363d;border-radius:9px;padding:9px 12px}
 .tl .step.done{border-color:#2ea043}
 .tl .step.active{border-color:#d29922;box-shadow:0 0 0 1px #d29922 inset}
 .tl .step.marker{border-color:#f85149;background:#2a1418}
 .tl .step.marker.active{box-shadow:0 0 0 2px #f85149 inset}
 .tl .tick{font-size:17px;text-align:center}
 .rstep{display:inline-block;margin:3px 6px 3px 0;padding:2px 9px;border-radius:7px;
  border:1px solid #30363d;font-size:12.5px}
 .rstep.past{color:#3fb950;border-color:#238636;background:#0d1f13}
 .rstep.now{color:#0d1117;background:#d29922;border-color:#d29922;font-weight:700}
 .rstep.ahead{color:#7d8590}
 .tl .left{color:#8b949e;font-size:13px;white-space:nowrap}
 table{border-collapse:collapse;width:100%;font-size:13.5px}
 th,td{border-bottom:1px solid #21262d;padding:6px 8px;text-align:left;vertical-align:top}
 th{color:#8b949e;font-weight:600;font-size:12.5px;text-transform:uppercase}
 td.num{text-align:right;font-variant-numeric:tabular-nums;white-space:nowrap}
 .mono{font-family:ui-monospace,SFMono-Regular,Consolas,monospace}
 .dim{color:#8b949e} .ok{color:#7ee787} .warn{color:#e3b341} .bad{color:#ff7b72}
 .log{max-height:300px;overflow:auto;font-family:ui-monospace,SFMono-Regular,Consolas,monospace;
  font-size:13px;background:#0d1117;border:1px solid #21262d;border-radius:9px;padding:10px 12px}
 .log div{white-space:pre-wrap}
 .log .t{color:#8b949e}
 .ch{display:grid;grid-template-columns:repeat(4,1fr);gap:6px}
 .ch div{background:#161b22;border:1px solid #21262d;border-radius:7px;padding:5px 8px;font-size:13px}
 .ch b{color:#8b949e;font-weight:600}
 .pill{display:inline-block;border-radius:999px;padding:2px 9px;font-size:12.5px;font-weight:600}
 .pill.on{background:#16301f;color:#7ee787;border:1px solid #2ea043}
 .pill.off{background:#21262d;color:#8b949e;border:1px solid #30363d}
 .hint{color:#8b949e;font-size:13px;margin-top:6px}
__NAV_STYLE__
</style>
</head>
<body>
__NAV__
<header>
 <h1>Failsafe — что делает борт, когда пропал сигнал</h1>
 <div class="sub">Времена, пороги и порядок — из прошивки Betaflight 4.5.4
 (<code>failsafe.c</code> / <code>rx.c</code> / <code>gps_rescue.c</code>). Масштаб времени меняет
 только скорость показа. Порт эмулятора <code>__HTTP_PORT__</code>, MSP — <code>tcp://127.0.0.1:__TCP_PORT__</code>.</div>
</header>
<main>

 <div class="card">
  <h2>Кнопки демонстрации</h2>
  <a class="btn green" href="/failsafe/on">Включить демо</a>
  <a class="btn" href="/failsafe/off">Выключить демо</a>
  <a class="btn red" href="/failsafe/loss">Потерять сигнал</a>
  <a class="btn blue" href="/failsafe/restore">Вернуть сигнал</a>
  <a class="btn" href="/failsafe/reset">Сброс демо</a>
  <span class="dim">|</span>
  <a class="btn small" href="/failsafe/arm_air">Взлететь (в воздухе)</a>
  <a class="btn small" href="/failsafe/arm_ground">Стоять на земле (взведён)</a>
  <a class="btn small" href="/failsafe/disarm">Выключить моторы</a>
  <span class="dim">|</span>
  <a class="btn small" href="/failsafe/box_on">Тумблер FAILSAFE ВКЛ</a>
  <a class="btn small" href="/failsafe/box_off">Тумблер FAILSAFE выкл</a>
  <div class="hint">«Сброс демо» возвращает всё к началу: борт в воздухе, взведён, связь есть,
   журнал очищен. Масштаб времени показа:
   <a class="btn small __SC1__" href="/failsafe/scale/1">×1 (честные тайминги)</a>
   <a class="btn small __SC05__" href="/failsafe/scale/0.5">×0.5</a>
   <a class="btn small __SC025__" href="/failsafe/scale/0.25">×0.25 (для показа)</a>
  </div>
  <div class="hint">Процедура Этапа 2 (вкладка Failsafe, <span class="mono">failsafe_procedure</span>):
   <a class="btn small __P0__" href="/failsafe/procedure/0">AUTO-LAND — посадка на месте</a>
   <a class="btn small __P1__" href="/failsafe/procedure/1">DROP — моторы сразу</a>
   <a class="btn small __P2__" href="/failsafe/procedure/2">GPS-RESCUE — вернуться домой</a>
  </div>
  <div class="hint">Сценарий проверок GPS-спасения:
   <a class="btn small __S0__" href="/failsafe/scenario/none">обычный полёт домой</a>
   <a class="btn small __S1__" href="/failsafe/scenario/low_sats">мало спутников</a>
   <a class="btn small __S2__" href="/failsafe/scenario/hover_low">газа зависания не хватает</a>
  </div>
 </div>

 <div class="card">
  <h2>Где сейчас борт</h2>
  <div class="phase" id="phase">—</div>
  <div id="plate"></div>
  <div class="bar" id="gbar"><i id="gfill"></i></div>
  <div class="dim" id="gtext">—</div>
  <table>
   <tr><th>что</th><th>сейчас</th></tr>
   <tr><td>Связь (кадры от аппаратуры)</td><td id="link">—</td></tr>
   <tr><td>Кадры считаются валидными (rxLinkState)</td><td id="receiving">—</td></tr>
   <tr><td>Тумблер FAILSAFE (вкладка «Режимы»)</td><td id="box">—</td></tr>
   <tr><td>Взвод и моторы</td><td id="armed">—</td></tr>
   <tr><td>Сигнал бипера</td><td id="beeper">—</td></tr>
   <tr><td>Режим тумблера (failsafe_switch_mode)</td><td id="swmode">—</td></tr>
   <tr><td>Процедура Этапа 2 (failsafe_procedure)</td><td id="proc">—</td></tr>
   <tr><td>Масштаб времени показа</td><td id="scale">—</td></tr>
  </table>
 </div>

 <div class="card">
  <h2>Таймлайн: Этап 1 → точка отката → Этап 2 → возврат</h2>
  <div class="tl" id="tl"></div>
  <div class="hint">ТОЧКА ОТКАТА — момент входа в Этап 2: до неё возврат сигнала отдаёт управление
   обратно, после неё процедура уже выполняется (моторы в 0, взвод заблокирован до восстановления связи).</div>
 </div>

 <div class="row">
  <div class="card">
   <h2>Каналы на приёмнике (MSP_RC, порядок A/E/R/T…)</h2>
   <div class="ch" id="channels"></div>
   <div class="hint">Первые 300 мс после пропадания кадров борт держит последние значения
    (<span class="mono">MAX_INVALID_PULSE_TIME_MS</span>), затем подставляет значения «Этапа 1»
    (<span class="mono">rxfail</span>: AUTO — центр/газ 885 мкс, HOLD — последнее, SET — заданное),
    в Этапе 2 газ идёт по <span class="mono">failsafe_throttle</span>.</div>
   <h2 style="margin-top:14px">Значения «Этапа 1» (rxfail)</h2>
   <div class="ch" id="stage1"></div>
  </div>
  <div class="card">
   <h2>Моторы и питание</h2>
   <div class="big" id="motors">—</div>
   <table>
    <tr><th>что</th><th>сейчас</th></tr>
    <tr><td>Флаги запрета взвода</td><td class="mono" id="flags">—</td></tr>
    <tr><td>Почему сняли с взвода</td><td id="reason">—</td></tr>
    <tr><td>GPS: спутники</td><td id="sats">—</td></tr>
    <tr><td>GPS: расстояние до дома, высота</td><td id="gpsdist">—</td></tr>
    <tr><td>Сценарий проверок</td><td id="scen">—</td></tr>
   </table>
  </div>
 </div>

 <div class="card">
  <h2>Параметры вкладки Failsafe → что каждый делает сейчас</h2>
  <table>
   <tr><th>параметр</th><th>значение</th><th>что делает сейчас</th></tr>
   <tbody id="params"></tbody>
  </table>
 </div>

 <div class="card">
  <h2>GPS-спасение (gps_rescue.c): параметры и фазы</h2>
  <div class="row">
   <div style="flex:1 1 320px">
    <div id="rsphase" class="big">—</div>
    <div id="rssteps" style="margin:6px 0 4px"></div>
    <div class="dim" id="rsnote"></div>
    <div class="hint" id="rschecks"></div>
   </div>
   <div style="flex:1 1 320px">
    <table><tbody id="rsparams"></tbody></table>
   </div>
  </div>
  <div class="hint">Санity-проверки (gps_rescue_sanity_checks): OFF — при неудаче 20 с неуправляемого
   спуска и выключение по удару; ON — сразу выключить моторы (ABORT); FS_ONLY — так же, но только
   когда связь потеряна по-настоящему.</div>
 </div>

 <div class="card">
  <h2>Журнал с таймкодами</h2>
  <div class="log" id="log"></div>
 </div>

 <div class="hint" style="margin:0 24px 24px;color:#8b949e">
  Та же демонстрация доступна из CLI Configurator'а: <span class="mono">sim_failsafe on|loss|restore|reset|status</span>.
  Сборка <span class="mono">__BUILD__</span>.
 </div>
</main>

<script>
const SC = __SC__;
function el(id){return document.getElementById(id)}
function cn(v){return '<div><b>'+v+'</b></div>'}
function fmtChannels(node, values, names){
  node.innerHTML = values.slice(0,16).map((v,i)=>'<div><b>'+(names[i]||('канал '+(i+1)))+
    '</b> <span class="mono">'+v+'</span></div>').join('');
}
function render(s){
  el('phase').innerHTML = s.phase_text + ' <span class="en">'+s.phase_en+'</span>';
  const pl = s.rollback || {possible:true,text:''};
  el('plate').innerHTML = '<span class="plate '+(pl.possible?'ok':'no')+'">'+
    (pl.possible?'ОТКАТ ВОЗМОЖЕН':'ОТКАТ НЕВОЗМОЖЕН')+'</span> <span class="dim">'+pl.text+'</span>';
  const cfg = s.config, step = (s.timeline||[]).find(t=>t.active);
  const left = step && step.left_ms ? step.left_ms : 0;
  const total = cfg.delay*1000;
  const frac = Math.max(0, Math.min(1, 1 - left/total));
  el('gfill').style.width = (frac*100).toFixed(1)+'%';
  el('gbar').className = 'bar' + (left>0 && left<400 ? ' warn' : (s.rollback && !s.rollback.possible ? ' bad' : ''));
  el('gtext').textContent = step && step.left_ms
    ? 'Этап 1: до точки отката '+ (left/1000).toFixed(1) +' с (failsafe_delay '+cfg.delay.toFixed(1)+' с)'
    : (s.demo ? 'страж-время не идёт' : 'демо выключено');
  el('link').innerHTML = s.link ? '<span class="ok">кадры идут</span>'
                                : '<span class="bad">кадров нет</span>';
  el('receiving').innerHTML = s.receiving ? 'да (rxLinkState UP)' : '<span class="bad">нет — ведёт Этап 2</span>';
  el('box').innerHTML = s.box_failsafe ? '<span class="pill on">включён</span>' : '<span class="pill off">выключен</span>';
  el('armed').innerHTML = (s.armed ? '<span class="ok">взведён</span>' : '<span class="bad">не взведён</span>')
     + ' · моторы ' + s.motors.join(' ');
  el('beeper').textContent = s.beeper || '—';
  el('swmode').textContent = s.switch_mode_text + ' (failsafe_switch_mode = '+s.switch_mode+')';
  el('proc').textContent = s.procedure_name + ' — ' + s.procedure_text;
  el('scale').textContent = '×' + s.scale;
  el('tl').innerHTML = (s.timeline||[]).map(t=>{
    const cls = t.marker ? 'marker' : (t.active ? 'active' : (t.done ? 'done' : ''));
    const tick = t.marker ? '★' : (t.done ? '✓' : (t.active ? '▶' : '○'));
    const left = t.left_ms ? '<span class="left">осталось '+(t.left_ms/1000).toFixed(1)+' с</span>' : '';
    return '<div class="step '+cls+'"><div class="tick">'+tick+'</div><div>'+t.name+'</div>'+left+'</div>';
  }).join('');
  fmtChannels(el('channels'), s.channels, s.channel_names);
  fmtChannels(el('stage1'), s.stage1, s.channel_names);
  el('motors').innerHTML = s.motors.map((v,i)=>'М'+(i+1)+' <span class="mono">'+v+'</span>').join(' &nbsp; ');
  el('flags').textContent = '0x' + (s.arming_flags>>>0).toString(16).toUpperCase().padStart(8,'0')
      + (s.arming_notes.length ? ' — ' + s.arming_notes.join(', ') : ' — запретов нет');
  el('reason').textContent = s.disarm_reason || '—';
  el('sats').textContent = s.sats + ' спутников (gps_rescue_min_sats = ' + s.rescue.min_sats + ')';
  el('gpsdist').textContent = s.rescue.dist_m.toFixed(1) + ' м от дома, ' + s.rescue.alt_m.toFixed(1) + ' м';
  el('scen').textContent = s.scenario_text;
  el('params').innerHTML = s.param_rows.map(r=>'<tr><td class="mono">'+r[0]+'</td><td class="num">'+r[1]+
      '</td><td>'+r[2]+'</td></tr>').join('');
  el('rsphase').textContent = 'Фаза спасения: ' + s.rescue.phase_name
      + ' — высота ' + s.rescue.alt_m.toFixed(1) + ' м, дом ' + s.rescue.dist_m.toFixed(1)
      + ' м, газ ' + s.rescue.throttle;
  const chain = ['INITIALIZE','ATTAIN_ALT','ROTATE','FLY_HOME','DESCENT','LANDING','COMPLETE'];
  const rp = s.rescue.phase;
  if (rp === 0) {
    el('rssteps').innerHTML = '<span class="dim">спасение не запущено (процедура «GPS-RESCUE»)</span>';
  } else if (rp === 7 || rp === 8) {
    el('rssteps').innerHTML = '<span class="dim">' + s.rescue.phase_name
        + ': цепочка фаз остановлена проверками gps_rescue.c</span>';
  } else {
    // фазы движка: 1 INITIALIZE, 2 ATTAIN_ALT, 3 ROTATE, 4 FLY_HOME, 5 DESCENT, 6 LANDING, 9 COMPLETE
    const at = (rp === 9) ? chain.length - 1 : rp - 1;
    el('rssteps').innerHTML = chain.map((ph, i) =>
        '<span class="rstep ' + (i < at ? 'past' : (i === at ? 'now' : 'ahead')) + '">'
        + (i < at ? '✓ ' : (i === at ? '▶ ' : '· ')) + ph + '</span>').join('');
  }
  el('rsnote').textContent = s.rescue.note || '';
  el('rschecks').innerHTML = (s.rescue.checks||[]).map(c=>'• '+c).join('<br>');
  el('rsparams').innerHTML = s.rescue_param_rows.map(r=>'<tr><td class="mono">'+r[0]+
      '</td><td class="num">'+r[1]+'</td><td>'+r[2]+'</td></tr>').join('');
  el('log').innerHTML = s.log.slice().reverse().map(l=>'<div><span class="t">'+l[0].toFixed(2).padStart(6)+' с</span> — '+l[1]+'</div>').join('');
}
async function tick(){
  try{ const r = await fetch('/api/failsafe', {cache:'no-store'}); render(await r.json()); }
  catch(e){ el('phase').textContent = 'нет связи с эмулятором'; }
}
document.querySelectorAll('.btn').forEach(b=>{
  const m = (b.getAttribute('href')||'').match(/^\/failsafe\/scale\/(.+)$/);
  if(m && parseFloat(m[1]) === SC){ b.classList.add('on'); }
});
tick(); setInterval(tick, 250);   // 4 раза в секунду: и часы демо идут ровно, и счётчики живые
</script>
</body></html>
"""


def failsafe_page(nav: str, style: str, build: str, http_port: int, tcp_port: str, scale: float,
                  procedure: int, scenario: str) -> str:
    """Собирает /failsafe: разметка одна, состояние страница тянет из /api/failsafe."""
    def mark(current, wanted):
        return "on" if current == wanted else ""

    return (FAILSAFE_PAGE
            .replace("__NAV_STYLE__", style)
            .replace("__NAV__", nav)
            .replace("__BUILD__", build)
            .replace("__HTTP_PORT__", str(http_port))
            .replace("__TCP_PORT__", str(tcp_port))
            .replace("__SC__", f"{scale:g}")
            .replace("__SC1__", mark(scale, 1.0))
            .replace("__SC05__", mark(scale, 0.5))
            .replace("__SC025__", mark(scale, 0.25))
            .replace("__P0__", mark(procedure, 0))
            .replace("__P1__", mark(procedure, 1))
            .replace("__P2__", mark(procedure, 2))
            .replace("__S0__", mark(scenario, "none"))
            .replace("__S1__", mark(scenario, "low_sats"))
            .replace("__S2__", mark(scenario, "hover_low")))

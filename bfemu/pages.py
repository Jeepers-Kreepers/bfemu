"""Страницы эмулятора, которые нужны сразу нескольким местам: навигация, /aux и /nav.

Отдельным модулем, потому что строки здесь обычные (не f-строки): подстановка идёт через
плейсхолдеры __NAV__, __NAV_STYLE__, __BUILD__ и порты. Так одинаковую кнопочную навигацию
получают и монитор (/), и страница аппаратуры (/tx), и чёрный ящик (/msc), и новые /aux и /nav.

Что здесь есть:

  * NAV_STYLE + nav_html() — полоска кнопок «Монитор · Аппаратура · Режимы · Чёрный ящик ·
    Переходы». Она одинаковая на всех страницах, текущая подсвечена;
  * NAV_PAGE — страница «Переходы»: крупные кнопки на /tx, /aux, /msc и / — с пояснением,
    что на каждой странице делать;
  * AUX_PAGE — страница «Режимы (AUX)»: та же картина, что на вкладке «Режимы» в Configurator
    (20 слотов, логика И/ИЛИ, связи), но в браузере и с обновлением раз в секунду; таблица
    рисуется и на сервере, и перерисовывается по /api/state, чтобы видеть, как тумблеры
    на /tx включают и выключают режимы.

Внешних ресурсов нет намеренно: страницы должны работать в закрытой сети и без CDN.
"""

from html import escape as _esc_raw


def _esc(value) -> str:
    """Экранирование строк FC (имена режимов, имя дрона) перед вставкой в разметку."""
    return _esc_raw(str(value), quote=True)

NAV_STYLE = """
 .nav{display:flex;gap:8px;flex-wrap:wrap;align-items:center;padding:10px 24px;background:#0d1117;
  border-bottom:1px solid #30363d;position:sticky;top:0;z-index:5}
 .nav a{display:inline-block;background:#21262d;border:1px solid #30363d;border-radius:8px;
  padding:6px 13px;color:#e6edf3;text-decoration:none;font:600 13px/1.4 -apple-system,Segoe UI,Roboto,sans-serif}
 .nav a:hover{border-color:#58a6ff}
 .nav a.on{background:#1f6feb;border-color:#1f6feb;color:#fff}
 .nav .sp{flex:1}
 .nav .bd{color:#8b949e;font-size:12.5px;margin-right:24px}
 .nav .bd code{color:#7ee787}
 .nav .ico{width:13px;height:13px;flex:0 0 13px}
 .nav .lbl{display:inline-block;vertical-align:-2px;margin-left:6px}
 .nav .bdwrap{display:flex;align-items:center;margin-right:24px}
"""

_ITEMS = (
    ("monitor", "/", "Монитор"),
    ("tx", "/tx", "Аппаратура (TX)"),
    ("aux", "/aux", "Режимы (AUX)"),
    ("failsafe", "/failsafe", "Failsafe (демо)"),
    ("msc", "/msc", "Чёрный ящик (MSC)"),
    ("nav", "/nav", "Переходы"),
)

_ICONS = {
    "monitor": '<path d="M2 3.5h12v7H2z" fill="none" stroke="currentColor" stroke-width="1.6"/>'
               '<path d="M5.5 13h5M8 10.5V13" stroke="currentColor" stroke-width="1.6"/>',
    "tx": '<rect x="1.5" y="4.5" width="13" height="7" rx="1.6" fill="none" stroke="currentColor" stroke-width="1.6"/>'
          '<path d="M4.2 8h2.4M5.4 6.8v2.4" stroke="currentColor" stroke-width="1.5"/>'
          '<circle cx="11" cy="7.4" r="0.9" fill="currentColor"/><circle cx="12.8" cy="9" r="0.9" fill="currentColor"/>',
    "aux": '<path d="M2.5 6.5h11M2.5 9.5h11" stroke="currentColor" stroke-width="1.6" fill="none"/>'
           '<circle cx="6" cy="6.5" r="1.8" fill="currentColor"/><circle cx="10.5" cy="9.5" r="1.8" fill="currentColor"/>',
    "msc": '<ellipse cx="8" cy="4" rx="5.5" ry="2" fill="none" stroke="currentColor" stroke-width="1.6"/>'
           '<path d="M2.5 4v8c0 1.1 2.5 2 5.5 2s5.5-.9 5.5-2V4" fill="none" stroke="currentColor" stroke-width="1.6"/>',
    "failsafe": '<path d="M8 1.8l5.6 2.6v4c0 3.2-2.3 5.4-5.6 6.3-3.3-.9-5.6-3.1-5.6-6.3v-4z" '
                'fill="none" stroke="currentColor" stroke-width="1.5"/>'
                '<path d="M8 5.4v3.2M8 10.6v.8" stroke="currentColor" stroke-width="1.6"/>',
    "nav": '<path d="M3 7.6 8 3.4l5 4.2V13H3z" fill="none" stroke="currentColor" stroke-width="1.6"/>'
           '<path d="M6.6 13V9.4h2.8V13" fill="none" stroke="currentColor" stroke-width="1.6"/>',
}


def nav_html(active: str, build: str = "") -> str:
    """Полоска кнопок. active — ключ текущей страницы ('tx', 'aux', ...)."""
    buttons = "".join(
        f'<a class="{"on" if key == active else ""}" href="{href}">'
        f'<svg class=ico viewBox="0 0 16 16" aria-hidden="true">{_ICONS[key]}</svg>'
        f'<span class=lbl>{label}</span></a>'
        for key, href, label in _ITEMS
    )
    build_note = (f'<span class=bdwrap><svg class=ico style="color:#8b949e" viewBox="0 0 16 16">'
                  f'<circle cx="8" cy="8" r="6" fill="none" stroke="currentColor" stroke-width="1.5"/>'
                  f'<path d="M8 5v4l2.6 1.6" fill="none" stroke="currentColor" stroke-width="1.5"/></svg>'
                  f'<span class="bd lbl">сборка&nbsp;<code>{build}</code></span></span>') if build else ""
    return f'<nav class=nav>{buttons}<span class=sp></span>{build_note}</nav>'


# --------------------------------------------------------------------- страница «Переходы»
NAV_PAGE = r"""<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Переходы — bfemu</title>
<style>
 body{margin:0;background:#101418;color:#e6edf3;font:14px/1.5 -apple-system,Segoe UI,Roboto,sans-serif}
 header{padding:18px 24px;background:#161b22;border-bottom:1px solid #30363d}
 h1{margin:0 0 4px;font-size:19px}
 .sub{color:#8b949e;font-size:13px} .sub a{color:#58a6ff}
 main{padding:20px 24px;display:grid;gap:16px;grid-template-columns:repeat(auto-fit,minmax(280px,1fr))}
 a.big{display:block;background:#161b22;border:1px solid #30363d;border-radius:12px;padding:18px 20px;
  color:#e6edf3;text-decoration:none;transition:border-color .12s,transform .12s}
 a.big:hover{border-color:#58a6ff;transform:translateY(-1px)}
 a.big b{display:block;font-size:16px;margin-bottom:6px}
 a.big .what{color:#8b949e;font-size:13px}
 a.big .url{color:#7ee787;font-size:12.5px;margin-top:10px;display:block}
 .hint{margin:4px 24px 24px;color:#8b949e;font-size:13px;border-top:1px solid #30363d;padding-top:12px}
 .hint b{color:#e6edf3}
__NAV_STYLE__
</style>
</head>
<body>
__NAV__
<header>
 <h1>Куда перейти</h1>
 <div class="sub">Эмулятор живёт на порту <code>__HTTP_PORT__</code>, MSP-порт для Configurator —
 <code>tcp://127.0.0.1:__TCP_PORT__</code>. Кнопки ниже открывают остальные страницы.</div>
</header>
<main>
 <a class="big" href="/tx"><b>Аппаратура (TX)</b>
  <span class="what">Тумблеры и стики: двигать каналы, «Связать (Bind)», подключить/выдернуть
  USB-аппаратуру, посмотреть источник последних изменений.</span>
  <span class="url">/tx</span></a>
 <a class="big" href="/aux"><b>Режимы (AUX)</b>
  <span class="what">20 слотов режимов: какие каналы и диапазоны что включают, логика И/ИЛИ,
  связи, маска MSP_STATUS. Обновляется живьём, пока вы щёлкаете тумблерами.</span>
  <span class="url">/aux</span></a>
 <a class="big" href="/msc"><b>Чёрный ящик (MSC)</b>
  <span class="what">Содержимое «флеша» файлами: LOG0000N.BBL по пролётам и BLACKBOX_ALL.BBL —
  то, что Configurator сохраняет кнопкой «Сохранено в файл…». Выход — кнопкой на странице.</span>
  <span class="url">/msc</span></a>
 <a class="big" href="/"><b>Монитор FC</b>
  <span class="what">Что происходит с бортом: активные фичи, порты, GPS, моторы, карточка «Режимы»,
  последние MSP-запросы и CLI-команды.</span>
  <span class="url">/</span></a>
</main>
<div class="hint">
 <b>Подсказка:</b> эти же кнопки есть в полоске сверху на каждой странице, включая /tx и /msc,
 поэтому возвращаться назад браузером не обязательно. Страницы открываются и по прямому адресу:
 <code>http://127.0.0.1:__HTTP_PORT__/aux</code>, <code>/tx</code>, <code>/msc</code>.
</div>
</body></html>
"""


# ------------------------------------------------------------------------ страница «Режимы»
AUX_PAGE = r"""<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Режимы (AUX) — bfemu</title>
<style>
 body{margin:0;background:#101418;color:#e6edf3;font:14px/1.5 -apple-system,Segoe UI,Roboto,sans-serif}
 header{padding:18px 24px;background:#161b22;border-bottom:1px solid #30363d}
 h1{margin:0 0 4px;font-size:19px}
 .sub{color:#8b949e;font-size:13px} .sub a{color:#58a6ff}
 main{padding:18px 24px;display:grid;gap:18px}
 .card{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:14px 16px}
 .card h2{margin:0 0 10px;font-size:13px;color:#58a6ff;text-transform:uppercase;letter-spacing:.05em}
 table{border-collapse:collapse;width:100%;font-size:13px}
 td,th{text-align:left;padding:5px 8px;border-bottom:1px solid #21262d}
 th{color:#8b949e;font-weight:600}
 .ok{color:#3fb950} .dim{color:#8b949e}
 .pill{display:inline-block;border:1px solid #30363d;border-radius:999px;padding:1px 9px;font-size:12px}
 .pill.on{background:#1f6feb33;border-color:#3fb950;color:#7ee787}
 .pill.off{color:#8b949e}
 .pill.and{background:#d2992233;border-color:#d29922;color:#e3b341}
 .pill.or{background:#21262d}
 code{color:#7ee787}
 pre{margin:0;background:#0d1117;border:1px solid #30363d;border-radius:8px;padding:10px 12px;
  overflow:auto;color:#e6edf3;font:12.5px/1.5 ui-monospace,SFMono-Regular,Consolas,monospace}
 .ref{display:flex;flex-wrap:wrap;gap:6px}
 .ref span{border:1px solid #30363d;border-radius:999px;padding:2px 10px;font-size:12.5px;color:#e6edf3}
 .ref b{color:#7ee787;font-weight:600}
 .bar{display:flex;gap:10px;flex-wrap:wrap;align-items:center;margin-bottom:8px}
 .big{font-variant-numeric:tabular-nums;font-size:15px}
 .big b{color:#7ee787}
 .note{color:#8b949e;font-size:12.5px}
__NAV_STYLE__
</style>
</head>
<body>
__NAV__
<header>
 <h1>Режимы (AUX) — то же, что на вкладке «Режимы» в Configurator</h1>
 <div class="sub">Страница читает протокол эмулятора (<code>MSP_MODE_RANGES</code> 34 и
 <code>MSP_MODE_RANGES_EXTRA</code> 238) и обновляется раз в секунду: щёлкайте тумблерами на
 <a href="/tx">странице аппаратуры</a> — здесь сразу видно, что включилось, а что нет.</div>
</header>
<main>
 <div class="card">
  <h2>Состояние</h2>
  <div class="bar">
   <span class="big">занято слотов <b id="used">__SLOTS_USED__</b> из __SLOTS_TOTAL__</span>
   <span class="dim">·</span>
   <span class="big">включены: <b id="active">__ACTIVE__</b></span>
   <span class="dim">·</span>
   <span class="big">маска MSP_STATUS: <code id="mask">__MASK__</code></span>
  </div>
  <div class="note">Строку вкладка рисует только при <code>start &lt; end</code> (или если есть связь):
  слот «900/900» — пустой заполнитель, поэтому свежий борт выглядит пустым.</div>
 </div>

 <div class="card">
  <h2>Слоты режимов (20)</h2>
  <table>
   <tr><th>слот</th><th>режим</th><th>канал</th><th>диапазон</th><th>логика</th><th>связь</th><th>состояние</th></tr>
   <tbody id="rows">__ROWS__</tbody>
  </table>
 </div>

 <div class="card">
  <h2>Он же в командной строке</h2>
  <pre id="cli">__CLI__</pre>
  <div class="note">Формат строки — <code>aux index permId aux MIN MAX logic linkedTo</code>, как
  <code>printAux()</code> из cli.c. Режимы, которых нет в списке платы, CLI примет, а вкладка
  (и эта таблица) не покажет.</div>
 </div>

 <div class="card">
  <h2>Режимы этой платы и их permanentId</h2>
  <div class="ref">__REF__</div>
  <div class="note">Номера не сквозные: 3, 9–11, 14, 16, 18, 21, 22, 38 — удалённые режимы
  (BARO, CAMTRIG, GPS HOME, GPS HOLD, …), их использовать нельзя.</div>
 </div>
</main>
<script>
'use strict';
async function tick(){
  let s;
  try { s = await (await fetch('/api/state', {cache:'no-store'})).json(); }
  catch (e) { return; }
  const m = s.modes || {};
  document.getElementById('used').textContent = m.slots_used;
  document.getElementById('active').textContent = (m.active && m.active.length) ? m.active.join(', ') : 'нет';
  document.getElementById('mask').textContent = '0x' + ((m.mask >>> 0).toString(16).padStart(8, '0'));
  const rows = [];
  const lines = [];
  for (const slot of (m.slots || [])) {
    lines.push('aux ' + slot.index + ' ' + slot.perm + ' ' + slot.aux + ' ' + slot.start + ' ' + slot.end
               + ' ' + slot.logic_value + ' ' + slot.linked_perm);
    if (!slot.configured) continue;
    const channel = slot.linked ? ('связь с ' + slot.linked) : ('AUX' + (slot.aux + 1));
    const span = slot.linked ? '—' : (slot.start + '…' + slot.end + ' мкс');
    const logic = '<span class="pill ' + (slot.logic === 'И' ? 'and' : 'or') + '">' + slot.logic + '</span>';
    const state = slot.active ? '<span class="pill on">включён</span>' : '<span class="pill off">выключен</span>';
    const listed = slot.listed ? '' : ' <span class="dim">(нет в списке платы)</span>';
    rows.push('<tr><td>' + slot.index + '</td><td><b>' + slot.box + '</b>' + listed + '</td><td>' + channel
              + '</td><td>' + span + '</td><td>' + logic + '</td><td>'
              + (slot.linked ? slot.linked + ' (' + slot.linked_perm + ')' : '—') + '</td><td>' + state + '</td></tr>');
  }
  document.getElementById('rows').innerHTML = rows.join('') ||
    '<tr><td colspan=7 class=dim>слотов нет — свежий борт после defaults</td></tr>';
  document.getElementById('cli').textContent = lines.join('\n');
}
tick();
setInterval(tick, 1000);
</script>
</body></html>
"""


def _slot_row(slot: dict) -> str:
    if not slot["configured"]:
        return ""
    channel = ("связь с " + str(slot["linked"])) if slot["linked"] else ("AUX" + str(slot["aux"] + 1))
    span = "—" if slot["linked"] else f'{slot["start"]}…{slot["end"]} мкс'
    logic = f'<span class="pill {"and" if slot["logic"] == "И" else "or"}">{slot["logic"]}</span>'
    state = ('<span class="pill on">включён</span>' if slot["active"]
             else '<span class="pill off">выключен</span>')
    listed = "" if slot["listed"] else ' <span class="dim">(нет в списке платы)</span>'
    link = f'{slot["linked"]} ({slot["linked_perm"]})' if slot["linked"] else "—"
    return (f'<tr><td>{slot["index"]}</td><td><b>{_esc(slot["box"])}</b>{listed}</td><td>{channel}</td>'
            f'<td>{span}</td><td>{logic}</td><td>{link}</td><td>{state}</td></tr>')


def aux_page(fc, nav: str, style: str, build: str, http_port: int, tcp_port: str) -> str:
    """Собирает /aux: первая отрисовка — на сервере, дальше страница обновляется сама."""
    state = fc.modes_state()
    rows = "".join(_slot_row(slot) for slot in state["slots"]) or \
        '<tr><td colspan=7 class=dim>слотов нет — свежий борт после defaults</td></tr>'
    reference = "".join(
        f'<span>{_esc(name)} <b>{fc.box_permanent_id(name)}</b></span>' for name in fc.active_boxes
    )
    active = ", ".join(state["active"]) if state["active"] else "нет"
    cli = "\n".join(fc.aux_lines())
    return (AUX_PAGE
            .replace("__NAV_STYLE__", style)
            .replace("__NAV__", nav)
            .replace("__SLOTS_USED__", str(state["slots_used"]))
            .replace("__SLOTS_TOTAL__", str(state["slots_total"]))
            .replace("__ACTIVE__", active)
            .replace("__MASK__", f'0x{state["mask"]:08x}')
            .replace("__ROWS__", rows)
            .replace("__CLI__", cli)
            .replace("__REF__", reference)
            .replace("__BUILD__", build)
            .replace("__HTTP_PORT__", str(http_port))
            .replace("__TCP_PORT__", str(tcp_port)))


def nav_page(nav: str, style: str, build: str, http_port: int, tcp_port: str) -> str:
    return (NAV_PAGE
            .replace("__NAV_STYLE__", style)
            .replace("__NAV__", nav)
            .replace("__BUILD__", build)
            .replace("__HTTP_PORT__", str(http_port))
            .replace("__TCP_PORT__", str(tcp_port)))

"""Страница виртуальной аппаратуры (TX) — отдаётся эмулятором по адресу /tx.

Отдельным модулем, чтобы не мешать CSS/JS с f-строками монитора: здесь обычная строка,
наружу подставляется только порт TCP (__TCP_PORT__) и порт страницы (__HTTP_PORT__).

Страница показывает:
  * бинд: кнопки «Связать (Bind)» / «Развязать» — как кнопка Bind на настоящем приёмнике;
  * подключена ли аппаратура (кнопки «Подключить» / «Отключить» — это «USB-кабель» к приёмнику);
  * на каком UART включён Serial RX — если во вкладке «Порты» приёмника нет, каналы не работают;
  * четыре стика; AUX-каналы по типам аппаратуры: двухпозиционные тумблеры, трёхпозиционные
    (AUX 3, AUX 4) и крутилки (AUX 7, AUX 10); кнопки «Стики в центр», «Газ вниз», «Демо»;
  * «Подключить аппаратуру» / «Отключить (выдернуть USB)» — управление переходит физической
    аппаратуре (ползунки гаснут) и обратно этой странице;
  * «Связать (Bind)» — создаёт связь с контроллером, только после неё каналы активируются;
  * источник последних изменений (эта страница, tools/tx.py или USB-джойстик).
"""

TX_PAGE = r"""<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Виртуальная аппаратура — bfemu</title>
<style>
 :root{--bg:#101418;--card:#161b22;--line:#30363d;--txt:#e6edf3;--dim:#8b949e;--acc:#58a6ff;--ok:#3fb950;--warn:#d29922;--bad:#f85149}
 *{box-sizing:border-box}
 body{margin:0;background:var(--bg);color:var(--txt);font:14px/1.5 -apple-system,Segoe UI,Roboto,sans-serif}
 header{padding:16px 24px;background:var(--card);border-bottom:1px solid var(--line)}
 h1{margin:0 0 4px;font-size:18px}
 .sub{color:var(--dim);font-size:13px}
 .sub a{color:var(--acc)}
 main{padding:18px 24px;display:grid;gap:18px;grid-template-columns:minmax(420px,1fr) minmax(340px,1fr);align-items:start}
 @media (max-width:960px){main{grid-template-columns:1fr}}
 .card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:14px 16px}
 .card h2{margin:0 0 12px;font-size:13px;color:var(--acc);text-transform:uppercase;letter-spacing:.05em}
 .row{display:grid;grid-template-columns:92px 1fr 86px;gap:10px;align-items:center;margin:9px 0}
 input[type=range]{width:100%;accent-color:var(--acc);height:22px}
 .val{font-variant-numeric:tabular-nums;text-align:right;color:var(--dim)}
 .val b{color:var(--txt)}
 .hint{color:var(--dim);font-size:12px;margin-top:10px}
 .sw{display:grid;grid-template-columns:repeat(auto-fill,minmax(112px,1fr));gap:8px}
 button{font:inherit;color:var(--txt);background:#21262d;border:1px solid var(--line);border-radius:8px;padding:7px 12px;cursor:pointer}
 button:hover:not(:disabled){border-color:var(--acc)}
 button:disabled{opacity:.5;cursor:not-allowed}
 button.swi{text-align:left}
 button.swi.on{background:#1f6feb33;border-color:var(--ok);color:#7ee787}
 button.swi.mid{background:#d2992233;border-color:var(--warn);color:#e3b341}
 .knob{padding:8px 10px;border:1px solid var(--line);border-radius:8px;background:#0d1117}
 .knob-title{color:var(--dim);font-size:12px;margin-bottom:2px}
 .knob-title b{color:var(--txt)}
 .knob input[type=range]{width:100%}
 button.act{background:#1f6feb}
 .bar{display:flex;gap:10px;flex-wrap:wrap;margin-top:6px}
 .state{margin-top:12px;padding:10px 12px;border:1px solid var(--line);border-radius:8px;background:#0d1117;color:var(--dim);font-size:12px;font-variant-numeric:tabular-nums;white-space:pre-wrap}
 .link{display:flex;align-items:center;gap:10px;font-size:15px;margin-bottom:10px}
 .dot{width:10px;height:10px;border-radius:50%;background:var(--bad);box-shadow:0 0 0 3px #f8514922}
 .dot.on{background:var(--ok);box-shadow:0 0 0 3px #3fb95022}
 .dot.no{background:var(--warn);box-shadow:0 0 0 3px #d2992222}
 code{color:#7ee787}
 .kbd{display:inline-block;background:#21262d;border:1px solid var(--line);border-radius:5px;padding:0 5px;font-size:12px}
 .dead{opacity:.45;pointer-events:none}
 .warnbox{margin-top:10px;padding:10px 12px;border-radius:8px;background:#d2992222;border:1px solid var(--warn);
  color:#e3b341;font-size:12.5px;line-height:1.55}
 .warnbox code{background:#d2992233}
__NAV_STYLE__
</style>
</head>
<body>
__NAV__
<header>
 <h1>Виртуальная аппаратура <span id="hdr_state" style="color:var(--dim)">…</span></h1>
 <div class="sub">Эмулятор bfemu · MSP: <code>tcp://127.0.0.1:__TCP_PORT__</code> · страница на порту <code>__HTTP_PORT__</code>
  · <a href="/">монитор FC</a> · сборка <code>__BUILD__</code> (страница аппаратуры v<code>__TX_PAGE__</code>)</div>
 <div class="sub">Двигайте стики — на вкладке «Приёмник» в Configurator поедут полосы, а коптер в «Предпросмотре» наклонится и повернётся.</div>
</header>
<main>
 <div>
  <div class="card" style="margin-bottom:18px">
   <h2>Подключение аппаратуры (USB)</h2>
   <div class="link"><span class="dot" id="link_dot"></span><b id="link_text">проверяю…</b></div>
   <div class="bar">
    <button class="act" id="btn_connect">Подключить аппаратуру</button>
    <button id="btn_disconnect">Отключить (выдернуть USB)</button>
   </div>
   <div class="hint" id="control_hint">…</div>
   <div class="warnbox" id="layout_note" style="display:none">…</div>
   <div class="warnbox" id="sources_note" style="display:none">…</div>
   <div class="hint"><b>«Подключить аппаратуру»</b> — управление переходит к <b>настоящей</b> аппаратуре
    (пульт по USB или приёмник в COM-порту): ползунки, тумблеры и крутилки на этой странице отключаются.<br>
    <b>«Отключить (выдернуть USB)»</b> — настоящая аппаратура отсоединена, управление снова у этой
    страницы: ползунки активны.<br>
    <b>«Связать (Bind)»</b> — после подключения аппаратуры создаёт связь с виртуальным контроллером:
    только после этого каналы активируются (до бинда вкладка «Приёмник» показывает «Этап 1»).</div>
   <div class="hint" id="port_hint">…</div>
  </div>

  <div class="card" style="margin-bottom:18px">
   <h2>Бинд (связывание с аппаратурой)</h2>
   <div class="link"><span class="dot" id="bind_dot"></span><b id="bind_text">проверяю…</b></div>
   <div class="bar">
    <button class="act" id="btn_bind">Связать (Bind)</button>
    <button id="btn_unbind">Развязать</button>
   </div>
   <div class="hint">«Связать» — как кнопка Bind на приёмнике: 2 секунды поиска, потом связь есть,
    <b>каналы активируются</b> и полосы на вкладке «Приёмник» оживают. «Развязать» — обратное:
    аппаратура не связана, полосы стоят и показываются значения «Этапа 1». Бинд нужен после
    каждого «Подключить аппаратуру» (физическую или виртуальную) — как в жизни.</div>
   <div class="hint">Инструмент ведёт каналы один: если запустить второе окно
    <code>python tools\tx.py joystick</code> (или забыть закрыть прежнее), каналы начнут «прыгать» —
    новое окно остановится с подсказкой, а страница покажет предупреждение ниже.</div>
   <div class="hint">Тумблеры и крутилки: <code>python tools\tx.py joystick --detect-aux</code> —
    переводите тумблер во все положения, крутилку крутите до упоров; инструмент сам найдёт ось,
    определит тип (2 положения / 3 положения / крутилка) и назначит AUX-канал. Раскладка видна
    ниже, в «Состоянии». Вручную: <code>--aux-map 3=switch2,4=switch3:4,5=knob:7</code>.</div>
   <div class="hint">Калибровка осей (<code>python tools\tx.py joystick --detect</code>): четыре шага —
    крен вправо, тангаж вверх, рыск вправо, затем «прокачайте» газ вверх-вниз и оставьте ручку вверху.
    У газа бывает трещотка, поэтому он определяется по размаху движения, а не по положению в покое.</div>
   <div class="hint"><b>Настоящая аппаратура (Zorro, TX16S, …):</b> включите на пульте режим
    <b>USB Joystick</b> (EdgeTX: MODEL SETUP → USB Joystick), подключите шнур и запустите
    <code>run_tx_joystick.bat</code> (или <code>python tools\tx.py joystick</code>) — стики живого пульта
    начнут двигать каналы Configurator. Библиотеки не нужны: пульт читается системным способом Windows.
    Если аппаратура идёт в эфир на свой приёмник, а приёмник подключён к USB-UART переходнику —
    <code>run_tx_wire.bat</code> (CRSF и SBUS читаются прямо из порта).</div>
  </div>

  <div class="card" id="sticks_card">
   <h2>Стики</h2>
   <div class="row"><span>Крен</span><input type="range" id="roll" min="1000" max="2000" value="1500"><span class="val" id="v_roll">1500</span></div>
   <div class="row"><span>Тангаж</span><input type="range" id="pitch" min="1000" max="2000" value="1500"><span class="val" id="v_pitch">1500</span></div>
   <div class="row"><span>Рыск</span><input type="range" id="yaw" min="1000" max="2000" value="1500"><span class="val" id="v_yaw">1500</span></div>
   <div class="row"><span>Газ</span><input type="range" id="throttle" min="1000" max="2000" value="1000"><span class="val" id="v_throttle">1000</span></div>
   <div class="bar">
    <button class="act" id="btn_center">Стики в центр</button>
    <button id="btn_demo">Демо: покачать стики</button>
    <button id="btn_throttle">Газ вниз</button>
   </div>
   <div class="hint">Клавиши: <span class="kbd">←</span><span class="kbd">→</span> крен, <span class="kbd">↑</span><span class="kbd">↓</span> тангаж,
    <span class="kbd">A</span><span class="kbd">D</span> рыск, <span class="kbd">W</span><span class="kbd">S</span> газ,
    <span class="kbd">Space</span> — центр. Клавиши держим — стик едет.</div>
  </div>
 </div>

 <div>
  <div class="card" style="margin-bottom:18px" id="aux_card">
   <h2>Тумблеры AUX</h2>
   <div class="sw" id="switches"></div>
   <div class="hint">Как на аппаратуре: <b>AUX 1–2, 5–6, 8–9, 11–12</b> — двухпозиционные тумблеры
    (1000/2000 мкс), <b>AUX 3 и AUX 4</b> — трёхпозиционные (1000/1500/2000), <b>AUX 7 и AUX 10</b> —
    крутилки (1000…2000). Так на вкладке «Режимы» видно, что аппаратура управляет режимами
    (ARM, BEEPER, ANGLE…).</div>
  </div>
  <div class="card">
   <h2>Состояние</h2>
   <div class="state" id="state">связь с виртуальным FC…</div>
  </div>
 </div>
</main>
<script>
const NAMES = ["roll","pitch","yaw","throttle","aux1","aux2","aux3","aux4","aux5","aux6","aux7","aux8","aux9","aux10","aux11","aux12"];
const LABELS = ["Крен","Тангаж","Рыск","Газ"].concat(NAMES.slice(4).map(n => "AUX " + n.slice(3)));
const SW_OFF = 1000, SW_ON = 2000, MID = 1500;
let channels = [1500,1500,1500,1000].concat(new Array(12).fill(1000));
let link = {connected: true, serial_ok: true, serial_port: "UART2", source: "—",
            bind: {state: "bound", text: "—", ready: true, seconds_left: 0},
            control: {mode: "virtual", physical: false, virtual: true, text: "—"},
            map: {}, aux_types: {}, switch3_values: [1000, 1500, 2000]};

// Органы управления AUX рисуются по типам из прошивки:
//   "switch"  — кнопка-тумблер (1000/2000),
//   "switch3" — кнопка-трёхпозиционник (1000/1500/2000, щёлкает по кругу),
//   "knob"    — крутилка: ползунок 1000…2000.
let auxBuilt = false;
let dragging = false, demoTimer = null, holding = false;

const el = id => document.getElementById(id);
const sliders = {roll: el("roll"), pitch: el("pitch"), yaw: el("yaw"), throttle: el("throttle")};

const swWrap = el("switches");
const auxWidgets = {};          // name -> {kind, button|input, title}

function auxKind(name) { return (link.aux_types && link.aux_types[name]) || "switch"; }
function canControl() { return link.serial_ok && (link.control ? link.control.virtual : true); }
function mapTextOf() {
    const map = link.map || {};
    return ["roll", "pitch", "throttle", "yaw"].filter(n => map[n] !== undefined)
        .map(n => ({roll: "Крен", pitch: "Тангаж", throttle: "Газ", yaw: "Рыск"})[n] +
                  " — ось " + map[n] + ((map.invert || []).includes(n) ? " (зеркалится)" : ""))
        .join(", ");
}

function buildAux() {
    swWrap.innerHTML = "";
    NAMES.slice(4).forEach((name, i) => {
        const index = 4 + i, kind = auxKind(name);
        if (kind === "knob") {
            const box = document.createElement("div");
            box.className = "knob"; box.id = "sw_" + name;
            const title = document.createElement("div");
            title.className = "knob-title";
            box.appendChild(title);
            const input = document.createElement("input");
            input.type = "range"; input.min = "1000"; input.max = "2000"; input.step = "5";
            input.value = channels[index];
            input.addEventListener("pointerdown", () => dragging = true);
            input.addEventListener("pointerup", () => dragging = false);
            input.addEventListener("input", () => setChannel(index, +input.value));
            box.appendChild(input);
            swWrap.appendChild(box);
            auxWidgets[name] = {kind, input, title};
        } else {
            const b = document.createElement("button");
            b.className = "swi"; b.id = "sw_" + name;
            b.onclick = () => {
                const value = channels[index];
                if (kind === "switch3") {          // низ -> середина -> верх -> низ
                    setChannel(index, value >= 1750 ? SW_OFF : (value >= 1250 ? SW_ON : MID));
                } else {
                    setChannel(index, value >= MID ? SW_OFF : SW_ON);
                }
            };
            swWrap.appendChild(b);
            auxWidgets[name] = {kind, button: b};
        }
    });
    auxBuilt = true;
}

function renderAux() {
    NAMES.slice(4).forEach((name, i) => {
        const index = 4 + i, value = channels[index], widget = auxWidgets[name];
        if (!widget) return;
        if (widget.kind === "knob") {
            if (!dragging) widget.input.value = value;
            widget.input.disabled = !canControl();
            widget.title.innerHTML = "AUX " + (i + 1) + ": <b>" + value + "</b> мкс · крутилка";
            return;
        }
        const three = widget.kind === "switch3";
        const on = value >= 1750, mid = three && value >= 1250 && value < 1750;
        widget.button.className = "swi" + (on ? " on" : (mid ? " mid" : ""));
        widget.button.textContent = "AUX " + (i + 1) + ": " + (mid ? "СРЕДНЕ" : (on ? "ВКЛ" : "выкл")) +
                                    " (" + value + ")";
        widget.button.title = (three ? "Трёхпозиционный тумблер 1000 / 1500 / 2000"
                                     : "Тумблер 1000 / 2000") + " — щёлкает по кругу";
        widget.button.disabled = !canControl();
    });
}

function render() {
    const live = canControl();
    for (const [name, input] of Object.entries(sliders)) {
        const value = channels[NAMES.indexOf(name)];
        if (!dragging) input.value = value;
        el("v_" + name).innerHTML = "<b>" + value + "</b> мкс";
        input.disabled = !live;
    }
    el("sticks_card").className = "card" + (live ? "" : " dead");
    el("aux_card").className = "card" + (live ? "" : " dead");
    if (!auxBuilt) buildAux();
    renderAux();

    const dot = el("link_dot"), text = el("link_text");
    if (!link.serial_ok) {
        dot.className = "dot no"; text.textContent = "приёмник не настроен";
    } else if (link.connected) {
        dot.className = "dot on"; text.textContent = "аппаратура подключена и в эфире";
    } else {
        dot.className = "dot"; text.textContent = "аппаратура отключена — потеря связи (Этап 1)";
    }
    el("hdr_state").textContent = !link.serial_ok ? "— нет приёмника" : (link.connected ? "подключена" : "отключена");
    el("hdr_state").style.color = !link.serial_ok ? "var(--warn)" : (link.connected ? "var(--ok)" : "var(--bad)");

    const control = link.control || {mode: "virtual", virtual: true, text: "—"};
    const mapText = mapTextOf();
    el("control_hint").innerHTML = control.physical
        ? "<b>Управление у физической аппаратуры.</b> Ползунки, тумблеры и крутилки на этой странице " +
          "отключены. Запустите <code>run_tx_joystick.bat</code> (пульт в режиме USB Joystick) или " +
          "<code>run_tx_wire.bat</code> (приёмник в COM-порту) и нажмите <b>«Связать (Bind)»</b>."
          + (mapText ? "<br>оси пульта (их нашёл инструмент): " + mapText +
             " <span style=\"color:var(--dim)\">(если стики двигают не свои каналы — " +
             "<code>python tools\\tx.py joystick --detect</code>)</span>"
                     : "<br><span style=\"color:var(--warn)\">раскладка осей пульта пока не приходила" +
                       "</span> — запустите <code>run_tx_joystick.bat</code> " +
                       "(<code>python tools\\tx.py joystick</code>), и она появится здесь")
        : "<b>Управление у этой страницы</b> (виртуальная аппаратура): двигайте стики, щёлкайте " +
          "тумблеры, крутите крутилки. Чтобы отдать управление настоящей аппаратуре — «Подключить " +
          "аппаратуру».";

    const srcBox = el("sources_note");
    if (link.multi_source) {
        srcBox.style.display = "";
        srcBox.innerHTML = "&#9888; <b>Каналы пишут несколько источников:</b> " +
            (link.sources || []).map(n => "<code>" + n + "</code>").join(", ") +
            "<br>Скорее всего, осталось открытым прежнее окно инструмента " +
            "(<code>run_tx_joystick.bat</code> или <code>python tools\\tx.py joystick</code>). " +
            "Пока оба работают, каналы «прыгают»: рыск может уводить AUX, а коптер в " +
            "«Предпросмотре» — крутиться. Нажмите в лишнем окне Ctrl+C.";
    } else {
        srcBox.style.display = "none";
    }

    const note = link.layout_note || "";
    const noteBox = el("layout_note");
    if (note && control.physical) {
        noteBox.style.display = "";
        noteBox.innerHTML = "&#9888; <b>Похоже, раскладка осей пульта не та.</b><br>" +
                            note.replace(/\n/g, "<br>") +
                            "<br><span style='color:var(--dim)'>Подробнее — " +
                            "docs\\HOWTO_real_tx_usb.md, раздел «Если стики двигают не свои каналы».</span>";
    } else {
        noteBox.style.display = "none";
    }

    el("port_hint").innerHTML = link.serial_ok
        ? "Приёмник работает на <b>" + link.serial_port + "</b> (функция Serial RX во вкладке «Порты») — каналы активны."
        : "Во вкладке «Порты» ни на одном UART не включён <b>Serial RX</b>: прошивка не поднимает приёмник, "
          + "и на вкладке «Приёмник» полосы каналов будут пустыми. Включите Serial RX (например, UART2) "
          + "и нажмите «Сохранить».";

    const bind = link.bind || {state: "bound", text: "—"};
    const bdot = el("bind_dot"), btext = el("bind_text");
    bdot.className = "dot" + (bind.state === "bound" ? " on" : (bind.state === "binding" ? " no" : ""));
    btext.textContent = bind.state === "bound" ? "связана: " + bind.text
                      : (bind.state === "binding" ? "идёт бинд: " + bind.text : "не связана: " + bind.text);
    el("btn_bind").disabled = !link.serial_ok || bind.state === "binding";
    el("btn_unbind").disabled = !link.serial_ok || bind.state === "none";

    el("btn_connect").disabled = !link.serial_ok || (link.control && link.control.physical);
    el("btn_disconnect").disabled = !link.serial_ok || !(link.control && link.control.physical);
    el("btn_demo").disabled = !live;
    el("btn_center").disabled = !live;
    el("btn_throttle").disabled = !live;
}

function stateText(extra) {
    const axes = ["A(крен)","E(тангаж)","R(рыск)","T(газ)"].map((n, i) => n + "=" + channels[i]).join("  ");
    const on = NAMES.slice(4).filter((n, i) => channels[4 + i] >= MID).map(n => n.toUpperCase());
    el("state").textContent =
        (extra ? extra + "\n" : "")
        + "приёмник: " + (link.serial_ok ? link.serial_port + " (Serial RX)" : "не настроен — включите во вкладке «Порты»") + "\n"
        + "управление: " + ((link.control && link.control.text) || "—") + "\n"
        + "биндинг: " + ((link.bind && link.bind.text) || "—") + "\n"
        + "аппаратура: " + (link.connected ? "в эфире" : "ОТКЛЮЧЕНА (значения «Этапа 1»)") + "\n"
        + "источник: " + (link.source || "—") +
          (link.updated_ago !== null && link.updated_ago !== undefined
             ? " (обновлено " + link.updated_ago + " с назад)" : "") + "\n"
        + "раскладка осей: " + (mapTextOf() || "ещё не приходила (запустите run_tx_joystick.bat)") + "\n"
        + "AUX-каналы: " + ((link.aux_map && link.aux_map.length)
             ? link.aux_map.join(", ")
             : "тумблеры и крутилки не откалиброваны (python tools\\tx.py joystick --detect-aux)") + "\n"
        + (link.layout_note ? "ВНИМАНИЕ: " + link.layout_note.replace(/\n/g, " ") + "\n" : "")
        + axes + "\nAUX включены: " + (on.length ? on.join(", ") : "нет");
}

async function apply(payload, extra) {
    try {
        payload.source = "страница /tx";
        const r = await fetch("/api/rc", {method: "POST", headers: {"Content-Type": "application/json"},
                                          body: JSON.stringify(payload)});
        const data = await r.json();
        if (!r.ok) { stateText("ошибка: " + (data.error || r.status)); return; }
        channels = data.channels; link = data;
        stateText(extra || ("отправлено " + new Date().toLocaleTimeString()));
    } catch (e) { stateText("нет связи с эмулятором: " + e); }
    render();
}

function setChannel(index, value) {
    stopDemo();
    channels[index] = Math.max(1000, Math.min(2000, Math.round(value)));
    render(); apply({channels});
}

for (const [name, input] of Object.entries(sliders)) {
    const index = NAMES.indexOf(name);
    input.addEventListener("pointerdown", () => dragging = true);
    input.addEventListener("pointerup", () => dragging = false);
    input.addEventListener("input", () => setChannel(index, +input.value));
}

el("btn_center").onclick = () => { stopDemo(); apply({reset: true}, "стики в центре"); };
el("btn_throttle").onclick = () => { stopDemo(); setChannel(3, 1000); };
el("btn_connect").onclick = () => {
    stopDemo();
    apply({control: "physical"}, "управление передано физической аппаратуре: страница ждёт кадры " +
          "(run_tx_joystick.bat или run_tx_wire.bat), затем нажмите «Связать»");
};
el("btn_disconnect").onclick = () => {
    stopDemo();
    apply({control: "virtual"}, "физическая аппаратура отключена: управление снова у этой страницы");
};
el("btn_bind").onclick = () => { stopDemo(); apply({bind: true}, "аппаратура ищет приёмник (биндинг)…"); };
el("btn_unbind").onclick = () => { stopDemo(); apply({bind: false}, "развязано: приёмник кадров не получает"); };

function stopDemo() { if (demoTimer) { clearInterval(demoTimer); demoTimer = null; } }

el("btn_demo").onclick = () => {
    stopDemo();
    const t0 = performance.now(), DURATION = 7000;
    stateText("демо: смотрим «Предпросмотр» в Configurator…");
    demoTimer = setInterval(() => {
        const t = (performance.now() - t0) / 1000;
        if ((performance.now() - t0) / DURATION >= 1) {
            stopDemo(); apply({reset: true}, "демо закончилось");
            return;
        }
        channels[0] = 1500 + Math.round(400 * Math.sin(t * 1.6));
        channels[1] = 1500 + Math.round(400 * Math.sin(t * 1.1 + 1.0));
        channels[2] = 1500 + Math.round(300 * Math.sin(t * 0.7 + 2.0));
        channels[3] = 1300 + Math.round(300 * Math.sin(t * 0.9));
        channels[4] = (Math.floor(t * 0.8) % 2) ? SW_ON : SW_OFF;
        render(); apply({channels}, "демо");
    }, 60);
};

const KEYS = {ArrowLeft:[0,-40], ArrowRight:[0,40], ArrowUp:[1,40], ArrowDown:[1,-40],
              KeyA:[2,-40], KeyD:[2,40], KeyW:[3,40], KeyS:[3,-40]};
document.addEventListener("keydown", e => {
    if (e.code === "Space") { el("btn_center").onclick(); e.preventDefault(); return; }
    const k = KEYS[e.code]; if (!k) return;
    e.preventDefault(); stopDemo();
    holding = true; setTimeout(() => holding = false, 400);
    channels[k[0]] = Math.max(1000, Math.min(2000, channels[k[0]] + k[1]));
    render(); apply({channels});
});

// --- синхронизация с FC (стики мог подвинуть другой клиент: tools/tx.py, USB-джойстик)
let sawState = false;
async function sync() {
    if (dragging || holding) return;
    try {
        const data = await (await fetch("/api/rc")).json();
        const moved = data.channels.some((v, i) => Math.abs(v - channels[i]) > 1) || data.source !== link.source;
        const changedLink = data.connected !== link.connected || data.serial_ok !== link.serial_ok
                         || (data.bind && data.bind.state) !== (link.bind && link.bind.state)
                         || (data.control && data.control.mode) !== (link.control && link.control.mode);
        const first = !sawState;
        channels = data.channels; link = data; sawState = true;
        if (moved) stateText("синхронизировано с FC (" + (data.source || "—") + ")");
        if (moved || changedLink || first) render();
        if (first) stateText();          // карточка «Состояние» заполняется сразу, а не после нажатия
    } catch (e) { /* эмулятор остановлен */ }
}
setInterval(sync, 500);

render();
fetch("/api/rc").then(r => r.json()).then(data => { channels = data.channels; link = data; render(); stateText("готово: аппаратура готова к работе"); });
</script>
</body>
</html>
"""

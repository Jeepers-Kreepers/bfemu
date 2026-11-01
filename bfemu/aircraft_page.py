"""Редактор профиля аппарата. Не рассчитывает физику и не изменяет настройки FC."""
from html import escape


def _input(name, label, *, kind="number", rule="", hint="", placeholder=""):
    attributes = f'name="{name}" type="{kind}"'
    if kind == "number":
        attributes += ' step="any" inputmode="decimal"'
        if rule:
            attributes += f' data-rule="{rule}"'
    else:
        attributes += ' maxlength="200"'
    if placeholder:
        attributes += f' placeholder="{escape(placeholder, quote=True)}"'
    text = f'<small>{escape(hint)}</small>' if hint else ''
    return f'<label>{escape(label)}<input {attributes}>{text}</label>'


def _source(name):
    return (f'<label>Источник данных<select name="{name}"><option value="">Не указан</option>'
            '<option value="measured">Измерены</option><option value="model">Из модели симулятора</option>'
            '<option value="manual">Введены вручную</option></select></label>')


def _textarea(name, label, hint):
    return (f'<label class="wide">{escape(label)}<textarea name="{name}" maxlength="12000" rows="4"></textarea>'
            f'<small>{escape(hint)}</small></label>')


def _motor(index):
    prefix = f'motor_{index}_'
    output = ''.join(f'<option value="{i}">{i}</option>' for i in range(1, 9))
    return (f'<details><summary>Мотор {index}</summary><div class="grid">'
            + _input(prefix + 'model', 'Модель мотора', kind='text')
            + _input(prefix + 'prop', 'Модель и размер пропеллера', kind='text')
            + f'<label>Выход FC<select name="{prefix}output"><option value="">Не указан</option>{output}</select></label>'
            + f'<label>Направление вращения, вид сверху<select name="{prefix}direction">'
              '<option value="">Не указано</option><option value="cw">По часовой стрелке</option>'
              '<option value="ccw">Против часовой стрелки</option></select></label>'
            + ''.join(_input(prefix + axis, f'Положение {axis.upper()}, мм') for axis in ('x', 'y', 'z'))
            + _input(prefix + 'kv', 'KV, об/мин на В', rule='positive')
            + _input(prefix + 'max_thrust', 'Измеренная максимальная тяга, Н', rule='positive',
                     hint='Укажите условия измерения в описании ниже: тяга зависит от пропеллера и питания.')
            + _input(prefix + 'rise_ms', 'Время нарастания отклика, мс', rule='positive',
                     hint='Укажите метод и диапазон изменения команды в описании ниже.')
            + _input(prefix + 'fall_ms', 'Время снижения отклика, мс', rule='positive')
            + _source(prefix + 'source')
            + _textarea(prefix + 'data', 'Данные тяги и реактивного момента',
                        'Запишите измерения или ссылку на них с единицами, командой, напряжением и условиями. '
                        'При необходимости укажите обороты, метод определения отклика и различия между моторами.')
            + '</div></details>')


def aircraft_page():
    """Полная HTML-страница с независимым локальным хранением профиля."""
    basic = (_input('profile_name', 'Название профиля', kind='text')
             + _input('model_source', 'Симулятор и название модели', kind='text',
                      hint='Пока только описание: соединение с симулятором не создаётся.')
             + _input('mass_g', 'Полная взлётная масса, г', rule='positive',
                      hint='Масса с аккумулятором и всей полезной нагрузкой.')
             + _source('frame_source'))
    center = ''.join(_input('cg_' + axis, f'Центр масс {axis.upper()}, мм') for axis in ('x', 'y', 'z'))
    inertia = ''.join(_input('inertia_' + axis, f'Элемент I{axis.upper()}, кг·м²',
                             rule='positive' if axis in ('xx', 'yy', 'zz') else '')
                      for axis in ('xx', 'yy', 'zz', 'xy', 'xz', 'yz'))
    power = (_input('battery_model', 'Модель аккумулятора', kind='text')
             + _input('battery_capacity', 'Ёмкость аккумулятора, мА·ч', rule='positive')
             + _input('battery_cells', 'Число последовательно соединённых банок', rule='positive_integer')
             + _input('battery_voltage', 'Напряжение при измерении, В', rule='positive')
             + _input('battery_resistance', 'Внутреннее сопротивление батареи, мОм', rule='nonnegative')
             + _input('esc_model', 'Модель и прошивка ESC', kind='text')
             + '<label>RPM-телеметрия ESC<select name="esc_rpm"><option value="">Не указана</option>'
               '<option value="yes">Доступна</option><option value="no">Недоступна</option></select></label>'
             + _source('power_source')
             + _textarea('power_notes', 'Дополнительные сведения о питании и ESC',
                         'Например, измеренное изменение напряжения под нагрузкой или условия измерения.'))
    sensor = (_input('sensor_model', 'Модель гироскопа / IMU', kind='text')
              + _input('sensor_rate', 'Частота данных IMU, Гц', rule='positive')
              + _input('sensor_delay', 'Задержка данных IMU, мс', rule='nonnegative')
              + _source('sensor_source')
              + _textarea('sensor_notes', 'Ориентация, шум и вибрации',
                          'Укажите ориентацию IMU относительно рамы и, при наличии, измерения шума или вибраций с единицами.'))
    return (AIRCRAFT_PAGE.replace('__BASIC__', basic).replace('__CENTER__', center)
            .replace('__INERTIA__', inertia).replace('__MOTORS__', ''.join(_motor(i) for i in range(1, 5)))
            .replace('__POWER__', power).replace('__SENSOR__', sensor))


AIRCRAFT_PAGE = r'''<!doctype html>
<html lang="ru"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Параметры аппарата — bfemu</title><style>
:root{color-scheme:dark}*{box-sizing:border-box}body{margin:0;background:#0d1117;color:#e6edf3;font:15px/1.5 system-ui,sans-serif}main{max-width:1100px;margin:auto;padding:28px}h1{font-size:28px;margin:0 0 8px}h2{font-size:19px;margin:0 0 7px}p{color:#a8b3c2;margin:5px 0 14px}a{color:#79c0ff}.card{background:#161b22;border:1px solid #30363d;border-radius:14px;padding:20px;margin:0 0 15px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:14px}.wide{grid-column:1/-1}label{display:block;color:#d6dee8;font-size:14px}input,select,textarea{display:block;width:100%;margin-top:5px;background:#0d1117;color:#e6edf3;border:1px solid #465362;border-radius:7px;padding:9px;font:inherit}textarea{resize:vertical}small{display:block;color:#a8b3c2;font-size:12px;line-height:1.4;margin-top:5px}details{border:1px solid #30363d;border-radius:8px;padding:12px;margin-top:10px}summary{cursor:pointer;font-weight:600}details .grid{margin-top:15px}button{font:600 14px system-ui;padding:10px 14px;border:1px solid #465362;border-radius:8px;background:#21262d;color:#e6edf3;cursor:pointer}button.primary{background:#1f6feb;border-color:#1f6feb}.actions{display:flex;gap:8px;flex-wrap:wrap}.status{min-height:1.5em;margin:10px 0 0}.error{color:#ffb4a9}.note{border-left:3px solid #d29922;padding-left:12px}.hint{font-size:13px}.required-note{color:#8de3c4}
__NAV_STYLE__
</style></head><body>__NAV__<main><h1>Параметры аппарата</h1>
<p>Профиль квадрокоптера: введите известные данные, а неизвестные оставьте пустыми. Единицы указаны у каждого поля.</p>
<p class="note">Профиль сохраняется только в этом браузере. Он пока не передаётся в Betaflight или симулятор и не влияет на графики или подбор PID. Название симулятора и данные ниже не создают физическую модель.</p>
<form id="aircraft-form" novalidate><section class="card"><h2>Профиль и масса</h2>
<p>Взлётная масса включает батарею и полезную нагрузку. Источник описывает данные этой группы и геометрию рамы.</p><div class="grid">__BASIC__</div></section>
<section class="card"><h2>Центр масс и инерция</h2>
<p>Координаты центра масс и моторов задаются от одного выбранного центра рамы: X вперёд, Y вправо, Z вниз. Моменты инерции относятся к центру масс и тем же осям. Значения IXY, IXZ и IYZ — внедиагональные элементы матрицы инерции, а не расстояния.</p>
<div class="grid">__CENTER____INERTIA__</div></section>
<section class="card"><h2>Моторы и пропеллеры</h2><p>Положения моторов задаются в той же системе координат, что и центр масс. Номера выходов должны соответствовать фактическому назначению в FC; сохранение здесь его не меняет.</p>__MOTORS__</section>
<section class="card"><h2>Питание и ESC</h2><p>Питание и ограничения приводов могут влиять на доступную тягу, если это учитывает подключённая физическая модель.</p><div class="grid">__POWER__</div></section>
<section class="card"><h2>IMU и вибрации</h2><p>Данные гироскопа, их задержка и вибрации важны для оценки обратной связи и фильтрации.</p><div class="grid">__SENSOR__</div></section>
<section class="card"><div class="actions"><button class="primary" type="submit">Сохранить профиль</button><button type="button" id="restore">Вернуть сохранённое</button><button type="button" id="clear">Очистить поля</button></div>
<p class="hint">Хранится один профиль для этого адреса в браузере. Очистка полей не удаляет сохранённый профиль, пока вы не нажмёте «Сохранить профиль».</p><p id="status" class="status" role="status" aria-live="polite"></p></section></form></main>
<script>
'use strict';
const STORAGE_KEY='bfemu.aircraft-profile.v1',form=document.getElementById('aircraft-form'),status=document.getElementById('status');
const fields=Array.from(form.querySelectorAll('input,select,textarea'));let baseline='';
function show(message,error=false){status.textContent=message;status.className='status'+(error?' error':'')}
function current(){return Object.fromEntries(fields.map(field=>[field.name,field.value]))}
function validate(){
 for(const field of fields){
  if(field.type!=='number')continue;
  if(field.validity.badInput){const details=field.closest('details');if(details)details.open=true;field.focus();throw Error('Введите корректное число в поле «'+field.parentElement.firstChild.textContent+'».')}
  if(field.value==='')continue;const n=Number(field.value),rule=field.dataset.rule;
  if(!Number.isFinite(n)||(rule==='positive'&&n<=0)||(rule==='nonnegative'&&n<0)||(rule==='positive_integer'&&(!Number.isSafeInteger(n)||n<=0))){const details=field.closest('details');if(details)details.open=true;field.focus();throw Error('Проверьте значение поля «'+field.parentElement.firstChild.textContent+'»: оно должно соответствовать указанным единицам и допустимому диапазону.')}
 }
 const outputs=fields.filter(field=>/^motor_\d_output$/.test(field.name)).map(field=>field.value).filter(Boolean);
 if(new Set(outputs).size!==outputs.length)throw Error('Один выход FC нельзя назначить двум моторам.');
}
function load(){
 const raw=localStorage.getItem(STORAGE_KEY);if(raw===null)return false;
 const data=JSON.parse(raw);if(!data||data.schema!==1||!data.values||typeof data.values!=='object'||Array.isArray(data.values))throw Error('Формат сохранённого профиля не распознан.');
 for(const field of fields){const value=data.values[field.name];if(value!==undefined&&typeof value!=='string')throw Error('Сохранённый профиль содержит некорректные данные.');if(value!==undefined&&value.length>(field.maxLength>0?field.maxLength:100))throw Error('Сохранённое значение слишком длинное.');
  if(value&&field.tagName==='SELECT'&&!Array.from(field.options).some(option=>option.value===value))throw Error('Сохранённый профиль содержит неизвестный вариант выбора.');
  if(value&&field.type==='number'){const probe=field.cloneNode();probe.value=value;if(probe.value!==value||!Number.isFinite(Number(value)))throw Error('Сохранённый профиль содержит некорректное число.');}}
 for(const field of fields)field.value=data.values[field.name]??'';return true;
}
form.addEventListener('submit',event=>{event.preventDefault();try{validate();localStorage.setItem(STORAGE_KEY,JSON.stringify({schema:1,values:current()}));baseline=JSON.stringify(current());show('Профиль сохранён в этом браузере. В симулятор и FC он не передан.')}catch(error){show('Не удалось сохранить профиль: '+error.message,true)}});
document.getElementById('restore').addEventListener('click',()=>{try{if(baseline!==JSON.stringify(current())&&!window.confirm('Отменить несохранённые изменения и вернуть сохранённый профиль?'))return;if(load()){baseline=JSON.stringify(current());show('Сохранённый профиль восстановлен.')}else show('Сохранённого профиля пока нет.')}catch(error){show('Не удалось восстановить профиль: '+error.message,true)}});
document.getElementById('clear').addEventListener('click',()=>{if(baseline!==JSON.stringify(current())&&!window.confirm('Очистить поля с несохранёнными изменениями?'))return;for(const field of fields)field.value='';show('Поля очищены. Сохранённый профиль пока не изменён.')});
form.addEventListener('input',()=>show('Изменения ещё не сохранены.'));form.addEventListener('change',()=>show('Изменения ещё не сохранены.'));
try{if(load())show('Сохранённый профиль загружен.');else show('Сохранённого профиля пока нет.')}catch(error){show('Не удалось загрузить профиль: '+error.message,true)}
baseline=JSON.stringify(current());window.addEventListener('beforeunload',event=>{if(baseline!==JSON.stringify(current())){event.preventDefault();event.returnValue=''}});
</script></body></html>'''

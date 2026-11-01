"""Редактор профиля аппарата. Не рассчитывает физику и не изменяет настройки FC."""
from html import escape
import json

from .aircraft_presets import HARDWARE_PRESETS


def _input(name, label, *, kind="number", rule="", hint="", placeholder=""):
    attributes = f'name="{name}" type="{"text" if kind == "number" else kind}"'
    if kind == "number":
        attributes += ' maxlength="100" inputmode="decimal" data-numeric="true"'
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
            '<option value="manual">Введены вручную</option>'
            '<option value="vendor">Данные производителя или продавца</option></select></label>')


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
            + _input(prefix + 'max_thrust', 'Максимальная тяга, Н', rule='positive',
                     hint='Укажите условия и источник: тяга зависит от пропеллера и питания.')
            + _input(prefix + 'rise_ms', 'Параметр отклика при увеличении, мс', rule='positive',
                     hint='Укажите метод и диапазон изменения команды. Постоянная времени модели не является измеренным временем разгона.')
            + _input(prefix + 'fall_ms', 'Параметр отклика при снижении, мс', rule='positive')
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
             + _input('battery_voltage', 'Напряжение питания, В', rule='positive',
                      hint='Уточните, измерено ли напряжение под нагрузкой или указано номинальное значение.')
             + _input('battery_resistance', 'Внутреннее сопротивление батареи, мОм', rule='nonnegative')
             + _input('esc_model', 'Модель и прошивка ESC', kind='text')
             + '<label>RPM-телеметрия ESC<select name="esc_rpm"><option value="">Не указана</option>'
               '<option value="yes">Доступна</option><option value="no">Недоступна</option><option value="unknown">Неизвестно</option></select></label>'
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
            .replace('__POWER__', power).replace('__SENSOR__', sensor)
            .replace('__CATALOG__', json.dumps(HARDWARE_PRESETS, ensure_ascii=True).replace('<', r'\u003c')))


AIRCRAFT_PAGE = r'''<!doctype html>
<html lang="ru"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Параметры аппарата — bfemu</title><style>
:root{color-scheme:dark}*{box-sizing:border-box}body{margin:0;background:#0d1117;color:#e6edf3;font:15px/1.5 system-ui,sans-serif}main{max-width:1100px;margin:auto;padding:28px}h1{font-size:28px;margin:0 0 8px}h2{font-size:19px;margin:0 0 7px}p{color:#a8b3c2;margin:5px 0 14px}a{color:#79c0ff}.card{background:#161b22;border:1px solid #30363d;border-radius:14px;padding:20px;margin:0 0 15px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:14px}.wide{grid-column:1/-1}label{display:block;color:#d6dee8;font-size:14px}input,select,textarea{display:block;width:100%;margin-top:5px;background:#0d1117;color:#e6edf3;border:1px solid #465362;border-radius:7px;padding:9px;font:inherit}textarea{resize:vertical}small{display:block;color:#a8b3c2;font-size:12px;line-height:1.4;margin-top:5px}details{border:1px solid #30363d;border-radius:8px;padding:12px;margin-top:10px}summary{cursor:pointer;font-weight:600}details .grid{margin-top:15px}button{font:600 14px system-ui;padding:10px 14px;border:1px solid #465362;border-radius:8px;background:#21262d;color:#e6edf3;cursor:pointer}button.primary{background:#1f6feb;border-color:#1f6feb}.actions{display:flex;gap:8px;flex-wrap:wrap}.status{min-height:1.5em;margin:10px 0 0}.error{color:#ffb4a9}.note{border-left:3px solid #d29922;padding-left:12px}.hint{font-size:13px}.required-note{color:#8de3c4}
__NAV_STYLE__
</style></head><body>__NAV__<main><h1>Параметры аппарата</h1>
<p>Выберите пример аппарата с пропеллерами от 1,5–1,6″ до 10″. Паспортные данные реальных сборок не являются физическими моделями; неизвестные параметры не подставляются из других размеров.</p>
<section class="card"><h2>Загрузить конфигурацию</h2><div class="grid"><label>Пример аппарата<select id="preset-choice"><option value="7">7″ · Gazebo Build 03</option></select></label></div><p id="preset-info" class="hint"></p><div class="actions"><button id="defaults" type="button">Загрузить выбранный пример</button><button id="export" type="button">Скачать профиль JSON</button></div><label>Загрузить сохранённый файл профиля <input id="import" type="file" accept=".json,application/json"></label><p id="preset-status" class="status" role="status" aria-live="polite"></p></section>
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
const hardware=__CATALOG__,choice=document.getElementById('preset-choice'),info=document.getElementById('preset-info');
const fields=Array.from(form.querySelectorAll('input,select,textarea'));let baseline='';
// Build 03: значения из зафиксированной модели; неизвестные данные не заменяются числами.
const PRESET={profile_name:'Build 03 · пример 7″',model_source:'Gazebo Build 03 · Safwanny/my_drone',mass_g:'931',frame_source:'model',
 cg_x:'0',cg_y:'0',cg_z:'0',inertia_xx:'0.00711',inertia_yy:'0.00754',inertia_zz:'0.01362',inertia_xy:'0',inertia_xz:'0',inertia_yz:'0',
 battery_model:'6S LiPo · модель не указана',battery_capacity:'1800',battery_cells:'6',battery_voltage:'22.2',battery_resistance:'Неизвестно',
 esc_model:'В модели не указаны',esc_rpm:'unknown',power_source:'model',power_notes:'6S 1800 мА·ч указана в бюджете масс модели; 22,2 В — номинальное напряжение. Сопротивление и RPM-телеметрия в источнике не заданы.',
 sensor_model:'Виртуальный IMU Gazebo',sensor_rate:'250',sensor_delay:'Неизвестно',sensor_source:'model',sensor_notes:'IMU расположен на base_link. В исходной модели оси X вперёд, Y влево, Z вверх; координаты на этой странице преобразованы к X вперёд, Y вправо, Z вниз. Угловой шум: стандартное отклонение 0,009 рад/с в модели.',
};
// Номера выходов соответствуют стандартному Quad X Betaflight 4.5.4 по положению мотора.
const MOTOR_LAYOUT=[[-143,143,'cw'],[143,143,'ccw'],[-143,-143,'ccw'],[143,-143,'cw']];
for(let i=0;i<4;i++){const [x,y,direction]=MOTOR_LAYOUT[i],p='motor_'+(i+1)+'_';Object.assign(PRESET,{[p+'model']:'2807SL 1300KV',[p+'prop']:'7040 × 3',[p+'output']:String(i+1),[p+'direction']:direction,[p+'x']:String(x),[p+'y']:String(y),[p+'z']:'-17.4',[p+'kv']:'1300',[p+'max_thrust']:'19.6',[p+'rise_ms']:'20',[p+'fall_ms']:'35',[p+'source']:'model',[p+'data']:'Оценка модели для 6S: максимум около 19,6 Н на мотор, максимум 2300 рад/с; коэффициент тяги 3,7e-6 Н·с²/рад², моментный коэффициент 0,016. Постоянные времени модели: 20 мс вверх и 35 мс вниз. Это не результаты измерения данного аппарата.'})}
function hardwareValues(item){
 const values={};for(const field of fields){if(field.tagName==='SELECT')values[field.name]='';else values[field.name]='Неизвестно'}
 Object.assign(values,{profile_name:item.title,model_source:'Паспортные данные · '+item.title,frame_source:'vendor',
  mass_g:item.mass_g||'Неизвестно',battery_model:item.battery||'Неизвестно',battery_capacity:item.capacity||'Неизвестно',battery_cells:item.cells||'Неизвестно',battery_voltage:item.voltage||'Неизвестно',
  esc_model:item.esc||'Неизвестно',esc_rpm:'unknown',power_source:'vendor',power_notes:item.notes+' Источник: '+item.url,
  sensor_model:item.sensor||'Неизвестно',sensor_source:'vendor',sensor_notes:'Источник: '+item.url+' Данные шума, ориентации и задержки не подтверждены.',
 });
 for(let i=1;i<=4;i++){const p='motor_'+i+'_';Object.assign(values,{[p+'model']:item.motor,[p+'prop']:item.prop,[p+'kv']:item.kv,[p+'source']:'vendor',[p+'data']:'Паспортные данные: '+item.url+' Тяга и отклик не указаны.'})}
 return values;
}
function defaults(values=PRESET){for(const field of fields)field.value=values[field.name]??''}
function describeChoice(){const item=hardware.find(p=>p.id===choice.value),url=item?item.url:'https://github.com/Safwanny/my_drone/tree/8466ad7ecc4d9ce65bdc88671579314a73384746';info.textContent=(item?item.notes:'Gazebo Build 03: расчётная модель; инерция base_link / quad_physics не включает отдельные звенья роторов.')+' ';const link=document.createElement('a');link.href=url;link.textContent='Источник';link.target='_blank';link.rel='noopener noreferrer';info.appendChild(link)}
for(const item of hardware){const option=document.createElement('option');option.value=item.id;option.textContent=item.title;choice.appendChild(option)}
// Помещаем пример 7″ в ряд по размеру, не меняя значения существующих профилей.
for(const option of Array.from(choice.options).sort((a,b)=>Number(a.value)-Number(b.value)))choice.appendChild(option);
choice.value='7';choice.addEventListener('change',describeChoice);describeChoice();
function numeric(value){return /^[+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?$/.test(value)&&Number.isFinite(Number(value))}

function show(message,error=false){for(const element of [status,document.getElementById('preset-status')]){element.textContent=message;element.className='status'+(error?' error':'')}}
function current(){return Object.fromEntries(fields.map(field=>[field.name,field.value]))}
function validate(){
 for(const field of fields){
  if(field.dataset.numeric!=='true')continue;
  if(field.validity.badInput){const details=field.closest('details');if(details)details.open=true;field.focus();throw Error('Введите корректное число в поле «'+field.parentElement.firstChild.textContent+'».')}
  if(field.value===''||field.value.trim().toLowerCase()==='неизвестно')continue;const n=Number(field.value),rule=field.dataset.rule;
  if(!numeric(field.value)||(rule==='positive'&&n<=0)||(rule==='nonnegative'&&n<0)||(rule==='positive_integer'&&(!Number.isSafeInteger(n)||n<=0))){const details=field.closest('details');if(details)details.open=true;field.focus();throw Error('Проверьте значение поля «'+field.parentElement.firstChild.textContent+'»: оно должно соответствовать указанным единицам и допустимому диапазону.')}
 }
 const outputs=fields.filter(field=>/^motor_\d_output$/.test(field.name)).map(field=>field.value).filter(Boolean);
 if(new Set(outputs).size!==outputs.length)throw Error('Один выход FC нельзя назначить двум моторам.');
}
function checked(data){
 if(!data||data.schema!==1||!data.values||typeof data.values!=='object'||Array.isArray(data.values))throw Error('Формат профиля не распознан.');
 const names=new Set(fields.map(field=>field.name));if(Object.keys(data.values).some(name=>!names.has(name)))throw Error('Профиль содержит неизвестные поля.');
 const values={};for(const field of fields){const value=Object.hasOwn(data.values,field.name)?data.values[field.name]:'';if(typeof value!=='string')throw Error('Профиль содержит некорректные данные.');if(value.length>(field.maxLength>0?field.maxLength:100))throw Error('Значение слишком длинное.');
  if(value&&field.tagName==='SELECT'&&!Array.from(field.options).some(option=>option.value===value))throw Error('Профиль содержит неизвестный вариант выбора.');
  if(value&&field.dataset.numeric==='true'&&value.trim().toLowerCase()!=='неизвестно'){const n=Number(value),rule=field.dataset.rule;if(!numeric(value)||(rule==='positive'&&n<=0)||(rule==='nonnegative'&&n<0)||(rule==='positive_integer'&&(!Number.isSafeInteger(n)||n<=0)))throw Error('Профиль содержит некорректное число.');}
  values[field.name]=value;
 }
 const outputs=fields.filter(field=>/^motor_\d_output$/.test(field.name)).map(field=>values[field.name]).filter(Boolean);
 if(new Set(outputs).size!==outputs.length)throw Error('Один выход FC указан у двух моторов.');return values;
}
function load(){const raw=localStorage.getItem(STORAGE_KEY);if(raw===null)return false;defaults(checked(JSON.parse(raw)));return true}
form.addEventListener('submit',event=>{event.preventDefault();try{validate();localStorage.setItem(STORAGE_KEY,JSON.stringify({schema:1,values:current()}));baseline=JSON.stringify(current());show('Профиль сохранён в этом браузере. В симулятор и FC он не передан.')}catch(error){show('Не удалось сохранить профиль: '+error.message,true)}});
document.getElementById('restore').addEventListener('click',()=>{try{if(baseline!==JSON.stringify(current())&&!window.confirm('Отменить несохранённые изменения и вернуть сохранённый профиль?'))return;if(load()){baseline=JSON.stringify(current());show('Сохранённый профиль восстановлен.')}else show('Сохранённого профиля пока нет.')}catch(error){show('Не удалось восстановить профиль: '+error.message,true)}});
document.getElementById('defaults').addEventListener('click',()=>{const item=hardware.find(p=>p.id===choice.value);if(choice.value!=='7'&&!item){show('Выберите известный пример.',true);return}if(!window.confirm('Заменить значения в форме выбранным примером? Сохранённый профиль не изменится до нажатия «Сохранить профиль».'))return;defaults(item?hardwareValues(item):PRESET);show('Выбранный пример загружен. Изменения ещё не сохранены.')});
document.getElementById('export').addEventListener('click',()=>{try{validate();const blob=new Blob([JSON.stringify({schema:1,values:current()},null,2)],{type:'application/json'});const url=URL.createObjectURL(blob),link=document.createElement('a');link.href=url;link.download='bfemu-aircraft-profile.json';link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);show('Профиль скачивается как JSON. Сохранённый в браузере профиль не изменён.')}catch(error){show('Не удалось скачать профиль: '+error.message,true)}});
let importGeneration=0;
document.getElementById('import').addEventListener('change',async event=>{const ticket=++importGeneration,file=event.target.files[0];event.target.value='';if(!file)return;try{if(file.size>128*1024)throw Error('Файл больше 128 КБ.');const text=await file.text();if(ticket!==importGeneration)return;const values=checked(JSON.parse(text));if(!window.confirm('Заменить значения в форме профилем из файла? Сохранённый профиль не изменится до нажатия «Сохранить профиль».'))return;defaults(values);show('Профиль из файла загружен в форму. Изменения ещё не сохранены.')}catch(error){if(ticket===importGeneration)show('Не удалось загрузить файл: '+error.message,true)}});
document.getElementById('clear').addEventListener('click',()=>{if(baseline!==JSON.stringify(current())&&!window.confirm('Очистить поля с несохранёнными изменениями?'))return;for(const field of fields)field.value='';show('Поля очищены. Сохранённый профиль пока не изменён.')});
form.addEventListener('input',()=>show('Изменения ещё не сохранены.'));form.addEventListener('change',()=>show('Изменения ещё не сохранены.'));
try{if(load())show('Сохранённый профиль загружен.');else{defaults();show('Загружен пример 7″. Профиль ещё не сохранён.')}}catch(error){defaults();show('Не удалось загрузить сохранённый профиль; показан пример 7″. '+error.message,true)}
baseline=JSON.stringify(current());window.addEventListener('beforeunload',event=>{if(baseline!==JSON.stringify(current())){event.preventDefault();event.returnValue=''}});
</script></body></html>'''

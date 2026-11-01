"""Просмотр сохранённых MSP-ответов SITL, без расчёта поведения контроллера."""

SITL_DATA_PAGE = r'''<!doctype html>
<html lang="ru"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Данные Betaflight SITL</title><style>
:root{color-scheme:dark}*{box-sizing:border-box}body{margin:0;background:#0d1117;color:#e6edf3;font:15px/1.5 system-ui,sans-serif}main{max-width:1040px;margin:auto;padding:28px}h1{font-size:28px;margin:0 0 8px}h2{font-size:17px;margin:22px 0 8px}p{color:#a8b3c2}label,select,input{font:inherit}select,input{color:#e6edf3;background:#0d1117;border:1px solid #465362;border-radius:7px;padding:8px}label{display:block;margin:12px 0}.card{background:#161b22;border:1px solid #30363d;border-radius:14px;padding:20px}.chart{width:100%;height:240px;display:block;background:#0d1117;border:1px solid #30363d;border-radius:8px}.small{font-size:12px;color:#a8b3c2}.error{color:#ffb4a9}.controls{display:flex;gap:22px;flex-wrap:wrap}code{color:#8de3c4}
__NAV_STYLE__
</style></head><body>__NAV__<main><div class="card"><h1>Данные Betaflight SITL</h1>
<p>Откройте файл JSONL, записанный командой <code>python3 -m bfemu.sitl_capture</code>. Файл обрабатывается в браузере и не отправляется на сервер.</p>
<p class="small">Показаны значения из MSP-ответов прошивки, без подстановки модели. Время — момент получения каждого ответа на компьютере; ответы поступают последовательно. Линии соединяют измеренные точки и не показывают, что происходило между ними. Идентификатор в файле не подтверждает подлинность его содержимого.</p>
<label>Файл записи <input id="file" type="file" accept=".jsonl,application/x-ndjson,text/plain"></label>
<div class="controls"><label>Ось гироскопа <select id="axis"><option value="0">Roll</option><option value="1">Pitch</option><option value="2">Yaw</option></select></label><label>Выход мотора <select id="motor"><option value="0">1</option><option value="1">2</option><option value="2">3</option><option value="3">4</option><option value="4">5</option><option value="5">6</option><option value="6">7</option><option value="7">8</option></select></label></div>
<p id="status" role="status" aria-live="polite">Файл ещё не выбран.</p>
<h2>Гироскоп · градусы в секунду</h2><p class="small">Для виртуального гироскопа SITL 4.5.4 значения MSP_RAW_IMU преобразуются из отсчётов в °/с с коэффициентом 2000/32768. Показания прошли фильтры FC и округление MSP.</p><canvas id="gyro" class="chart" aria-label="Записанная угловая скорость из MSP_RAW_IMU"></canvas>
<h2>Выход мотора · значение MSP</h2><canvas id="motor-chart" class="chart" aria-label="Записанный выход мотора из MSP_MOTOR"></canvas>
<p class="small">Значение выхода берётся непосредственно из MSP_MOTOR. Нулевой выход может означать, что мотор выключен или отсутствует. Составляющие P, I, D и Feedforward в этих MSP-ответах не передаются, поэтому здесь они не показаны.</p>
</div></main><script>
const $=id=>document.getElementById(id);let recorded=null;
function parseData(text){
 const lines=text.split(/\r?\n/).filter(line=>line.trim());if(!lines.length)throw Error('Файл пуст.');
 let identity;try{identity=JSON.parse(lines[0])}catch{throw Error('Не удалось прочитать начало файла.')}
 if(identity.type!=='identity'||identity.variant!=='BTFL'||identity.version!=='4.5.4'||identity.board!=='SITL'||identity.time_basis!=='host_monotonic_after_response')throw Error('Это не запись формата Betaflight 4.5.4 SITL.');
 const result={gyro:[],motor:[]};let previous=-1;
 for(let i=1;i<lines.length;i++){
  let row;try{row=JSON.parse(lines[i])}catch{throw Error('Ошибка JSON в строке '+(i+1)+'.')}
  if(row.type!=='msp'||!Number.isFinite(row.received_s)||row.received_s<previous||row.received_s<0)throw Error('Некорректный порядок или время ответа в строке '+(i+1)+'.');previous=row.received_s;
  if(row.command!=='MSP_RAW_IMU'&&row.command!=='MSP_MOTOR')continue;
  if(typeof row.payload_hex!=='string'||!/^(?:[0-9a-fA-F]{2})+$/.test(row.payload_hex))throw Error('Некорректные байты MSP в строке '+(i+1)+'.');
  const n=row.payload_hex.length/2,expected=row.command==='MSP_RAW_IMU'?18:16;if(n!==expected)throw Error('Неверная длина MSP в строке '+(i+1)+'.');
  let bytes=new Uint8Array(n);for(let j=0;j<n;j++)bytes[j]=parseInt(row.payload_hex.slice(j*2,j*2+2),16);const view=new DataView(bytes.buffer);
  if(row.command==='MSP_RAW_IMU')result.gyro.push([row.received_s,view.getInt16(6,true)*2000/32768,view.getInt16(8,true)*2000/32768,view.getInt16(10,true)*2000/32768]);
  else result.motor.push([row.received_s,...Array.from({length:8},(_,j)=>view.getUint16(j*2,true))]);
 }
 if(!result.gyro.length&&!result.motor.length)throw Error('В записи нет ответов MSP_RAW_IMU или MSP_MOTOR.');return result;
}
function draw(canvas,rows,index,color){let rect=canvas.getBoundingClientRect(),ratio=window.devicePixelRatio||1;canvas.width=rect.width*ratio;canvas.height=rect.height*ratio;let c=canvas.getContext('2d');c.scale(ratio,ratio);c.clearRect(0,0,rect.width,rect.height);if(!rows.length)return;
 let w=rect.width,h=rect.height,L=54,R=12,T=12,B=26,first=rows[0][0],last=rows[rows.length-1][0],span=Math.max(last-first,0.001),lo=Infinity,hi=-Infinity;for(const row of rows){lo=Math.min(lo,row[index]);hi=Math.max(hi,row[index])}if(lo===hi){lo-=1;hi+=1}else{let pad=(hi-lo)*0.08;lo-=pad;hi+=pad}let X=t=>L+(t-first)/span*(w-L-R),Y=v=>T+(hi-v)/(hi-lo)*(h-T-B);
 c.font='11px system-ui';c.fillStyle='#8b949e';c.strokeStyle='#27313d';c.lineWidth=1;for(let j=0;j<=4;j++){let val=lo+(hi-lo)*j/4,y=Y(val);c.beginPath();c.moveTo(L,y);c.lineTo(w-R,y);c.stroke();c.fillText(val.toFixed(1),2,y+4)}c.fillText(first.toFixed(2)+' с',L,h-6);c.fillText(last.toFixed(2)+' с',Math.max(L,w-R-58),h-6);
 c.strokeStyle=color;c.lineWidth=2;c.beginPath();rows.forEach((row,i)=>{if(i)c.lineTo(X(row[0]),Y(row[index]));else c.moveTo(X(row[0]),Y(row[index]))});c.stroke();if(rows.length===1){c.fillStyle=color;c.beginPath();c.arc(X(rows[0][0]),Y(rows[0][index]),2,0,2*Math.PI);c.fill()}}
function render(){if(!recorded)return;draw($('gyro'),recorded.gyro,Number($('axis').value)+1,'#65d6b3');draw($('motor-chart'),recorded.motor,Number($('motor').value)+1,'#e3b45b')}
$('file').onchange=async()=>{recorded=null;let file=$('file').files[0];if(!file){$('status').textContent='Файл ещё не выбран.';for(const id of ['gyro','motor-chart'])$(id).getContext('2d').clearRect(0,0,$(id).width,$(id).height);return}try{if(file.size>20*1024*1024)throw Error('Файл больше 20 МБ.');recorded=parseData(await file.text());$('status').className='';$('status').textContent='Ответов гироскопа: '+recorded.gyro.length+'. Ответов моторов: '+recorded.motor.length+'.';render()}catch(e){$('status').className='error';$('status').textContent=e.message;for(const id of ['gyro','motor-chart']){let c=$(id).getContext('2d');c.clearRect(0,0,$(id).width,$(id).height)}}};$('axis').onchange=render;$('motor').onchange=render;window.addEventListener('resize',render);
</script></body></html>'''

import json, shutil, tempfile, subprocess, time, threading, struct, hashlib
from pathlib import Path
from bfemu import aircraft_sitl
from tools import sitl_lockstep_probe as probe

profile = json.loads((Path('bfemu/data/aircraft_build03.json')).read_text())
with tempfile.TemporaryDirectory(prefix='ground-msp-') as tmp:
    work = Path(tmp); source=work/'source'; shutil.copytree(aircraft_sitl.CACHE, source)
    probe.patch_sources(source); sdk=work/'sdk'; sdk.mkdir()
    print('Сборка исследовательского SITL...',flush=True)
    with (work/'build.log').open('w') as log:
        subprocess.run(['make','TARGET=SITL',f'ARM_SDK_DIR={sdk}','-j4'],cwd=source,stdout=log,stderr=subprocess.STDOUT,check=True)
    binary=source/'obj/main/betaflight_SITL.elf'; eeprom=work/'eeprom';eeprom.mkdir()
    def start(): return subprocess.Popen([str(binary)], cwd=eeprom, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    process=start(); aircraft_sitl._configure_pwm(process,pid_process_denom=1);process=start()
    client=aircraft_sitl._connect(process)
    try:
        client.identify(); aircraft_sitl.verify_pid_denom(client,1)
        deadline=time.monotonic()+8
        while True:
            status=client.request(101)
            if len(status)>=6 and struct.unpack_from('<H',status,4)[0]&0x21==0x21: break
            if time.monotonic()>deadline: raise RuntimeError('Датчики SITL не готовы')
            time.sleep(.1)
        results=[]; errors=[]; stop=threading.Event()
        def poll():
            while not stop.is_set():
                try:
                    payload=client.request(108)
                    if len(payload)!=6: raise ValueError('Некорректный MSP_ATTITUDE')
                    results.append(struct.unpack('<hhh',payload))
                    stop.wait(.05)
                except Exception as exc:
                    errors.append(str(exc));break
        thread=threading.Thread(target=poll);thread.start()
        records,_=probe.run_frames(profile,1500,scenario='stationary',environment='ground-air-v1')
        stop.set();thread.join(timeout=4)
        print('Всего кадров:',len(records),'на земле:',sum(x['ground_contact'] for x in records))
        print('MSP_ATTITUDE ответов:',len(results),'значения (roll/10,pitch/10,yaw °):', sorted(set(results)),'ошибки:',errors,'поток активен:',thread.is_alive())
        if len(results)<2 or len(set(results)) != 1 or not all(x['ground_contact'] for x in records): raise RuntimeError('Проверка не прошла')
    finally:
        client.close();aircraft_sitl._stop(process)

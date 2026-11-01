"""Изолированный короткий эксперимент: профиль -> модель -> Betaflight SITL.

Параметры приводов — явно обозначенная теоретическая адаптация Build 03, не
измеренные характеристики. Процесс SITL использует только временную EEPROM.
"""
from __future__ import annotations

import io
import json
import math
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import threading
import time

from .flight_actuators import output_mapping, read_curves
from .flight_physics import Aircraft, PhysicsError
from .msp import FROM_FC, encode_v1
from .sitl_capture import CaptureError, SitlMspClient
from .sitl_closed_loop import run_loop

STAMP = {"revision": "25356b59a96dc975d777844f896b7c7855ea90e6",
         "archive_sha256": "0613fcc69675ae82891d97ee8aaa91545b73ba43f7e9120a8e1799e897dde4e3",
         "network": "loopback-v1"}

_LOCK = threading.Lock()
REVISION = '25356b59a96dc975d777844f896b7c7855ea90e6'
CACHE = Path.home() / '.cache' / 'bfemu' / 'betaflight-sitl' / REVISION
BINARY = CACHE / 'obj' / 'main' / 'betaflight_SITL.elf'
SOURCE = 'https://github.com/Safwanny/my_drone/tree/8466ad7ecc4d9ce65bdc88671579314a73384746'


def theoretical_curves(profile):
    """Адаптация Build 03: скорость линейна команде, тяга пропорциональна квадрату.

    Коэффициент момента/тяги 0,016 взят из модели Build 03; линейное
    соответствие команды SITL скорости — допущение этого адаптера.
    В отличие от Gazebo, фильтр первого порядка применяется к силе, не оборотам.
    """
    Aircraft.from_profile(profile)
    output_mapping(profile)
    values = profile['values']
    motors = []
    for i in range(1, 5):
        prefix = f'motor_{i}_'
        if values.get(prefix+'model') != '2807SL 1300KV' or values.get(prefix+'prop') != '7040 × 3':
            raise PhysicsError(f'Для мотора {i} эта модель поддерживает только 2807SL 1300KV и 7040 × 3 из Build 03.')
        direction = values.get(prefix+'direction')
        if direction not in ('cw', 'ccw'):
            raise PhysicsError(f'Укажите направление вращения мотора {i}.')
        try:
            maximum, up, down = [float(values[prefix+name]) for name in ('max_thrust', 'rise_ms', 'fall_ms')]
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            raise PhysicsError(f'Укажите числовые тягу и времена отклика мотора {i}.') from exc
        if not all(math.isfinite(v) and 0 < v <= limit for v, limit in ((maximum, 1000), (up, 10000), (down, 10000))):
            raise PhysicsError(f'Тяга и времена отклика мотора {i} должны быть положительными и конечными.')
        sign = -1 if direction == 'cw' else 1
        curve = []
        for j in range(101):
            command = j/100
            thrust = maximum*command**2
            curve.append({'command': command, 'thrust_n': thrust, 'reaction_nm': sign*.016*thrust})
        motors.append({'motor': i, 'time_constant_up_s': up/1000, 'time_constant_down_s': down/1000, 'curve': curve})
    result = {'schema': 1, 'provenance': f'Теоретическая адаптация Build 03: {SOURCE}; '
              'скорость пропорциональна команде SITL; инерция приложена к силе, а не оборотам.', 'motors': motors}
    read_curves(result)
    return result


def _owns_listening_port(process, port):
    """Не отправлять команды чужому FC при гонке за TCP-порт (Linux /proc)."""
    try:
        descriptors = set()
        for entry in (Path('/proc') / str(process.pid) / 'fd').iterdir():
            try:
                descriptors.add(entry.readlink().as_posix())
            except FileNotFoundError:
                continue  # Дескриптор мог закрыться во время перечисления.
        for table in ('tcp', 'tcp6'):
            for line in (Path('/proc/net') / table).read_text().splitlines()[1:]:
                columns = line.split()
                if (len(columns) > 9 and columns[3] == '0A'
                        and int(columns[1].rsplit(':', 1)[1], 16) == port
                        and f'socket:[{columns[9]}]' in descriptors):
                    return True
    except (OSError, ValueError, IndexError):
        return False
    return False


def _connect(process, deadline=15):
    until = time.monotonic()+deadline
    while time.monotonic() < until:
        if process.poll() is not None:
            raise PhysicsError('Изолированный SITL завершился до подключения.')
        try:
            client = SitlMspClient(timeout=2)
            if not _owns_listening_port(process, 5761):
                client.close()
                raise PhysicsError('Порт 5761 принадлежит другому процессу; эксперимент остановлен.')
            return client
        except OSError:
            time.sleep(.05)
    raise PhysicsError('Изолированный SITL не открыл порт MSP.')


def _write(client, code, payload):
    client.connection.sendall(encode_v1(code, payload))
    while True:
        data = client.connection.recv(4096)
        if not data:
            raise CaptureError('SITL закрыл MSP при настройке эксперимента.')
        client.parser.feed(data)
        for reply in client.parser:
            if reply.code != code or reply.direction != FROM_FC:
                raise CaptureError(f'SITL не подтвердил MSP {code}.')
            return reply.payload


def _configure_pwm(process):
    client = _connect(process)
    try:
        client.identify()
        connection = client.connection
        connection.sendall(b'#')
        def until(marker):
            data = b''
            while marker not in data:
                chunk = connection.recv(4096)
                if not chunk or len(data) > 100000:
                    raise PhysicsError('SITL не подтвердил настройку PWM.')
                data += chunk
        until(b'CLI Mode')
        connection.sendall(b'set motor_pwm_protocol = PWM\n')
        until(b'motor_pwm_protocol set to PWM')
        connection.sendall(b'save\n')
        until(b'saving')
    finally:
        client.close()
    process.wait(timeout=10)


def _stop(process):
    if process is None:
        return
    if process.poll() is None:
        process.terminate()
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=3)


def experiment(profile):
    """Один roll step с заводским PID отдельного SITL; не меняет пользовательский FC."""
    curves = theoretical_curves(profile)
    if not _LOCK.acquire(blocking=False):
        raise PhysicsError('Другой эксперимент SITL уже выполняется.')
    process = None
    try:
        try:
            built = BINARY.is_file() and json.loads((CACHE / 'bfemu-sitl.json').read_text()) == STAMP
        except (OSError, ValueError):
            built = False
        if not built:
            raise PhysicsError('Сначала соберите Betaflight SITL: python3 tools/betaflight_sitl.py build')
        for kind, port in ((socket.SOCK_STREAM, 5761), (socket.SOCK_DGRAM, 9002),
                           (socket.SOCK_DGRAM, 9003), (socket.SOCK_DGRAM, 9004)):
            with socket.socket(socket.AF_INET, kind) as probe:
                try:
                    probe.bind(('127.0.0.1', port))
                except OSError as exc:
                    raise PhysicsError(f'Порт {port} занят. Для эксперимента нужны свободные TCP 5761 и UDP 9002–9004; при необходимости запустите веб-эмулятор с --tcp 127.0.0.1:15777.') from exc
        with tempfile.TemporaryDirectory(prefix='bfemu-aircraft-sitl-') as workdir, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as motors, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as fdm, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as rc:
            motors.bind(('127.0.0.1', 9002))
            binary = str(BINARY)
            def start():
                return subprocess.Popen([binary], cwd=workdir, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            process = start()
            _configure_pwm(process)
            process = start()
            client = _connect(process)
            try:
                client.identify()
                _write(client, 35, bytes([0, 0, 0, 32, 48]))  # ARM на AUX1 1700–2100
                deadline = time.monotonic()+8
                while True:
                    status = client.request(101)
                    if len(status) >= 6 and struct.unpack_from('<H', status, 4)[0] & 0x21 == 0x21:
                        break
                    if time.monotonic() >= deadline:
                        raise PhysicsError('SITL не инициализировал датчики.')
                    time.sleep(.1)
                time.sleep(.55)
                channels = [1500, 1500, 1000, 1500, 1000] + [1000]*11
                done = threading.Event()
                output, errors, run_start = io.StringIO(), [], []
                def clock():
                    now = time.monotonic()
                    if not run_start:
                        run_start.append(now)
                    return now
                def send_rc():
                    start_time = time.monotonic()
                    while not done.is_set():
                        rc.sendto(struct.pack('=d16H', time.monotonic()-start_time, *channels), ('127.0.0.1', 9004))
                        done.wait(.02)
                def simulate():
                    try:
                        run_loop(profile, curves, motors, fdm, output, duration=12, period=.004,
                                 max_gap=.5, pressure_pa=101325, clock=clock)
                    except Exception as exc:
                        errors.append(exc)
                tx = threading.Thread(target=send_rc, daemon=True)
                worker = threading.Thread(target=simulate, daemon=True)
                tx.start()
                worker.start()
                try:
                    deadline = time.monotonic()+8
                    while True:
                        if errors:
                            raise PhysicsError(str(errors[0]))
                        if not worker.is_alive():
                            raise PhysicsError('Расчёт завершился до подготовки SITL.')
                        status = client.request(101)
                        if len(status) >= 21 and struct.unpack_from('<I', status, 17)[0] == 0:
                            break
                        if time.monotonic() >= deadline:
                            raise PhysicsError('SITL не завершил подготовку к ARM.')
                        time.sleep(.1)
                    channels[4] = 1900
                    deadline = time.monotonic()+2
                    while True:
                        status = client.request(101)
                        if len(status) >= 10 and struct.unpack_from('<I', status, 6)[0] & 1:
                            break
                        if time.monotonic() >= deadline:
                            raise PhysicsError('SITL не подтвердил ARM.')
                        time.sleep(.1)
                    channels[2] = 1300
                    time.sleep(.6)
                    if not worker.is_alive():
                        raise PhysicsError('Расчёт завершился до команды крена.')
                    event_start = time.monotonic()
                    channels[0] = 1650
                    time.sleep(1.2)
                    event_end = time.monotonic()
                    channels[0] = 1500
                    status = client.request(101)
                    if len(status) < 10 or not struct.unpack_from('<I', status, 6)[0] & 1:
                        raise PhysicsError('SITL вышел из ARM во время команды крена.')
                    time.sleep(.5)
                    channels[2], channels[4] = 1000, 1000
                    worker.join(timeout=15)
                    if worker.is_alive():
                        raise PhysicsError('Превышено время эксперимента SITL.')
                    if errors:
                        raise PhysicsError(str(errors[0]))
                    lines = output.getvalue().splitlines()
                    rows = [json.loads(line) for line in lines[1:]]
                    if not rows:
                        raise PhysicsError('SITL не вернул результаты эксперимента.')
                    if not run_start or rows[-1]['elapsed_s'] < event_end-run_start[0]:
                        raise PhysicsError('Расчёт не охватил всю команду крена.')
                    # Время переключения RC на стороне отправителя, не момент получения FC.
                    points = [{'t': round(row['elapsed_s'], 3),
                               'rates': [round(math.degrees(x), 2) for x in row['state']['angular_velocity_rad_s']]}
                              for row in rows[::5]]
                    return {'profile': profile['values'].get('profile_name', ''), 'samples': points,
                            'command_window_s': [round(event_start-run_start[0], 3), round(event_end-run_start[0], 3)],
                            'source': SOURCE, 'firmware': 'Betaflight 4.5.4 SITL',
                            'note': 'Скорости физической модели, не MSP-гироскоп. PID заводской отдельного SITL.'}
                finally:
                    channels[2], channels[4] = 1000, 1000
                    done.set()
                    tx.join(timeout=2)
                    # Не закрывать сокеты пока worker ещё использует их.
                    worker.join(timeout=13)
            finally:
                client.close()
                _stop(process)
                process = None
    except (OSError, subprocess.SubprocessError, CaptureError) as exc:
        raise PhysicsError(f'Эксперимент SITL не выполнен: {exc}') from exc
    finally:
        _stop(process)
        _LOCK.release()

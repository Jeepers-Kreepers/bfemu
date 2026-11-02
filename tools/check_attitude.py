#!/usr/bin/env python3
"""Снять несколько ответов MSP_ATTITUDE с локального TCP-порта, без записи настроек."""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bfemu.fc_constants import MSP_ATTITUDE
from bfemu.msp import FROM_FC, MspParser, encode_v1


def decode_attitude(payload):
    if len(payload) != 6:
        raise ValueError(f'MSP_ATTITUDE: ожидалось 6 байт, получено {len(payload)}.')
    roll, pitch, yaw = struct.unpack('<hhh', payload)
    return roll / 10, pitch / 10, yaw


def samples(port, count, interval):
    """Опрос только для чтения; каждое значение получено в отдельном ответе MSP."""
    with socket.create_connection(('127.0.0.1', port), timeout=3) as connection:
        connection.settimeout(3)
        parser = MspParser()
        for index in range(count):
            connection.sendall(encode_v1(MSP_ATTITUDE))
            reply = None
            while reply is None:
                data = connection.recv(4096)
                if not data:
                    raise ConnectionError('Соединение закрыто до ответа MSP.')
                parser.feed(data)
                for message in parser:
                    if message.code != MSP_ATTITUDE or message.direction != FROM_FC:
                        raise ValueError('Получен неожиданный ответ MSP вместо MSP_ATTITUDE.')
                    reply = decode_attitude(message.payload)
            yield reply
            if index + 1 < count:
                time.sleep(interval)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=5761, help='TCP-порт bfemu (по умолчанию 5761)')
    parser.add_argument('--samples', type=int, default=25, help='Число ответов (2–300, по умолчанию 25)')
    parser.add_argument('--interval', type=float, default=0.2, help='Пауза между запросами в секундах (0,05–10)')
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535 or not 2 <= args.samples <= 300 or not 0.05 <= args.interval <= 10:
        parser.error('Порт: 1–65535, число ответов: 2–300, интервал: 0,05–10 секунд.')
    print(f'Чтение MSP_ATTITUDE с 127.0.0.1:{args.port}; настройки не изменяются.')
    print('№   Крен, °   Тангаж, °   Курс, °')
    rows = []
    try:
        for index, row in enumerate(samples(args.port, args.samples, args.interval), 1):
            rows.append(row)
            print(f'{index:>2}  {row[0]:>8.1f}  {row[1]:>9.1f}  {row[2]:>7.0f}', flush=True)
    except (OSError, ValueError) as exc:
        print(f'Опрос не завершён: {exc}', file=sys.stderr)
        return 1
    print('Ответы изменялись.' if len(set(rows)) > 1 else 'Все полученные ответы одинаковы.')
    print('Опрос проверяет только ответы этого TCP-порта, а не источник данных внутри программы или отрисовку Configurator.')
    return 0


if __name__ == '__main__':
    sys.exit(main())

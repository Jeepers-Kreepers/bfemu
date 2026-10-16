"""Точка входа: python3 -m inavemu [--tcp HOST:PORT] [--http PORT] ..."""
from __future__ import annotations

import argparse
import sys

from .fc import InavFC
from .server import FCServer, MonitorServer


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="inavemu",
        description="Виртуальный полётный контроллер INAV: подключается INAV Configurator",
    )
    ap.add_argument("--tcp", default="127.0.0.1:5761",
                    help="адрес MSP-порта (по умолчанию 127.0.0.1:5761)")
    ap.add_argument("--http", type=int, default=None,
                    help="порт веб-монитора (без флага — не поднимать)")
    ap.add_argument("--http-host", default="127.0.0.1",
                    help="адрес веб-монитора (по умолчанию 127.0.0.1)")
    ap.add_argument("--fc-version", default="9.1.0",
                    help="версия прошивки для MSP_FC_VERSION (9.0.0–9.x: её проверяет "
                         "Configurator; по умолчанию 9.1.0)")
    ap.add_argument("--api-version", default="2.5.0",
                    help="версия MSP API (по умолчанию 2.5.0, как у INAV 9.1)")
    ap.add_argument("--board", default="SITL", help="идентификатор платы (4 символа)")
    ap.add_argument("--target", default="SITL", help="имя таргета для MSP_BOARD_INFO")
    ap.add_argument("--eeprom", default="inav_eeprom.json",
                    help="файл «флеша» (конфигурация между запусками)")
    args = ap.parse_args(argv)

    host, _, port = args.tcp.rpartition(":")
    host = host or "127.0.0.1"
    try:
        port = int(port)
    except ValueError:
        sys.exit(f"неверный адрес --tcp {args.tcp!r}, ожидается HOST:PORT")

    fc = InavFC(fc_version=args.fc_version, api_version=args.api_version,
                board=args.board, target=args.target, eeprom_path=args.eeprom)
    server = FCServer(fc, host=host, port=port)
    server.start()
    print(f"[FC] INAV {fc.fc_version} / MSP API {args.api_version} / "
          f"настроек в базе: {len(fc.store.settings)}")

    monitor = None
    if args.http is not None:
        monitor = MonitorServer(server, host=args.http_host, port=args.http)
        monitor.start()

    print("Для подключения: INAV Configurator → Manual connection → "
          f"tcp://{host}:{port}  (Ctrl+C — выход)")
    try:
        while True:
            import time
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[FC] останавливаюсь")
        fc.save_eeprom()
        server.stop()
        if monitor:
            monitor.stop()


if __name__ == "__main__":
    main()

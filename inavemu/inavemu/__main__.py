"""Точка входа: python3 -m inavemu [--tcp HOST:PORT] [--http PORT] ..."""
from __future__ import annotations

import argparse
import sys

from .fc import InavFC
from .server import FCServer, MonitorServer
from .single_instance import InstanceLock


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="inavemu",
        description="Виртуальный полётный контроллер INAV: подключается INAV Configurator",
    )
    ap.add_argument("--tcp", default="127.0.0.1:5761",
                    help="адрес MSP-порта (по умолчанию 127.0.0.1:5761)")
    ap.add_argument("--http", type=int, default=8080,
                    help="порт веб-интерфейса (0 — не поднимать; "
                         "по умолчанию 8080)")
    ap.add_argument("--http-host", default="0.0.0.0",
                    help="адрес веб-интерфейса (по умолчанию 0.0.0.0; "
                         "127.0.0.1 — только с этой машины)")
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

    # Запуск двух и более виртуальных контроллеров запрещён.
    lock = InstanceLock()
    if not lock.acquire():
        sys.exit(
            "[!] Виртуальный контроллер INAV уже запущен — второй экземпляр "
            "запрещён.\n"
            "    Закройте работающий inavemu (или освободите lock-файл "
            f"{lock.path}) и повторите запуск."
        )

    fc = InavFC(fc_version=args.fc_version, api_version=args.api_version,
                board=args.board, target=args.target, eeprom_path=args.eeprom)
    server = FCServer(fc, host=host, port=port)
    try:
        server.start()
    except OSError as exc:
        lock.release()
        sys.exit(
            f"[!] Не удалось занять {host}:{port} ({exc.strerror}).\n"
            "    Порт занят другим приложением (или уже запущен виртуальный "
            "FC). Смените --tcp или освободите порт."
        )
    print(f"[FC] INAV {fc.fc_version} / MSP API {args.api_version} / "
          f"настроек в базе: {len(fc.store.settings)}")

    monitor = None
    if args.http:
        try:
            monitor = MonitorServer(server, host=args.http_host, port=args.http)
            monitor.start()
        except OSError as exc:
            lock.release()
            server.stop()
            sys.exit(
                f"[!] Не удалось занять веб-порт {args.http_host}:{args.http} "
                f"({exc.strerror}).\n"
                "    Смените --http или укажите --http 0, чтобы запуститься "
                "без веб-интерфейса."
            )

    print(f"Для подключения: INAV Configurator → тип соединения TCP → "
          f"IP:Port {host}:{port} (без «tcp://») → Подключиться")
    if monitor:
        print(f"Веб-интерфейс (оборудование, монитор): "
              f"http://127.0.0.1:{monitor.port}/")
    print("(Ctrl+C — выход)")
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
        lock.release()


if __name__ == "__main__":
    main()

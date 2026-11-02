#!/usr/bin/env python3
"""Проверка вкладки «Чёрный ящик» (dataflash): накопитель, чтение лога, настройки записи.

Проверка повторяет то, что делает Configurator (components/tabs/OnboardLoggingTab.vue):

* MSP_DATAFLASH_SUMMARY (70) — «вкладка видит данные»: ready/supported, секторы, размер, занято;
* MSP_DATAFLASH_READ (71) — выкачивает весь лог блоками по 4096 байт (как useDataflashPull.js,
  включая длинный ответ «джамбо»-кадром MSP v1) и сверяет байты с накопителем;
* MSP_BLACKBOX_CONFIG (80) / MSP_SET_BLACKBOX_CONFIG (81) — настройки записи и их сохранение;
* MSP_DATAFLASH_ERASE (72) — очистка накопителя;
* CLI `blackbox` / `blackbox refill` — сводка и повторное наполнение.

Разбор кадров MSP v1 (в том числе «джамбо») сделан здесь так же, как в Configurator
(src/js/msp.ts: JUMBO_FRAME_MIN_SIZE = 255).
"""
from __future__ import annotations

import argparse
import os
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from bfemu.msp import encode_v1, encode_v2  # noqa: E402
from bfemu import bb_scenarios  # noqa: E402

MSP_DATAFLASH_SUMMARY = 70
MSP_DATAFLASH_READ = 71
MSP_DATAFLASH_ERASE = 72
MSP_BLACKBOX_CONFIG = 80
MSP_SET_BLACKBOX_CONFIG = 81
BLOCK_SIZE = 4096           # useDataflashPull.js
JUMBO_LIMIT = 255           # msp.ts: JUMBO_FRAME_MIN_SIZE
MSP_EEPROM_WRITE = 250
MSP_SET_REBOOT = 68
MARKER = b"H Product:Blackbox flight data recorder"

FAILED: list[str] = []


def check(ok: bool, text: str) -> bool:
    print(f"   [{'OK ' if ok else 'FAIL'}] {text}")
    if not ok:
        FAILED.append(text)
    return ok


def u16(data: bytes, pos: int) -> int:
    return int.from_bytes(data[pos:pos + 2], "little")


def u32(data: bytes, pos: int) -> int:
    return int.from_bytes(data[pos:pos + 4], "little")


def _frame(buf: bytes):
    """Первый полный кадр ответа в буфере: (payload, code, остаток буфера).

    Умеет «джамбо»-кадры MSP v1 (длина 255 → настоящая длина u16) — так прошивка
    отдаёт длинные ответы, например чтение «флеша» чёрного ящика.
    """
    while True:
        start = buf.find(b"\x24")
        if start < 0:
            return None
        buf = buf[start:]
        if len(buf) < 5:
            return None
        if buf[1] == 0x4D:                                   # $M>
            length = buf[3]
            offset = 5
            if length == JUMBO_LIMIT:
                if len(buf) < 7:
                    return None
                length = u16(buf, 5)
                offset = 7
            if len(buf) < offset + length + 1:
                return None
            return buf[offset:offset + length], buf[4], buf[offset + length + 1:]
        if buf[1] == 0x58:                                   # $X> (v2)
            if len(buf) < 8:
                return None
            length = u16(buf, 6)
            if len(buf) < 8 + length + 1:
                return None
            return buf[8:8 + length], u16(buf, 4), buf[8 + length + 1:]
        buf = buf[1:]


class Fc:
    """Клиент MSP, как вкладка/Configurator: v1 для кодов ≤ 254."""

    def __init__(self, port: int):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=10)
        self.sock.settimeout(0.5)
        self.leftover = b""

    def call(self, code: int, payload: bytes = b"", v2: bool = False, timeout: float = 30.0) -> bytes:
        self.sock.sendall(encode_v2(code, payload) if v2 else encode_v1(code, payload))
        buf = self.leftover
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                buf += self.sock.recv(65536)
            except TimeoutError:
                continue
            parsed = _frame(buf)
            if parsed is not None:
                frame, _code, rest = parsed
                self.leftover = rest
                return frame
        return b""

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def start_emulator(port: int, eeprom: Path, http: int | None = None):
    env = dict(os.environ)
    env["PYTHONPATH"] = str(ROOT)
    argv = [sys.executable, "-m", "bfemu", "--tcp", f"127.0.0.1:{port}", "--eeprom", str(eeprom), "--quiet"]
    if http:
        argv += ["--http", str(http)]
    proc = subprocess.Popen(argv, cwd=ROOT, env=env,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(60):
        try:
            return proc, Fc(port)
        except OSError:
            time.sleep(0.25)
    raise RuntimeError("эмулятор не поднялся")


def decode_summary(payload: bytes) -> dict:
    flags = payload[0]
    return {
        "flags": flags,
        "ready": bool(flags & 1),
        "supported": bool(flags & 2),
        "sectors": u32(payload, 1),
        "size": u32(payload, 5),
        "used": u32(payload, 9),
    }


def decode_blackbox_config(payload: bytes) -> dict:
    return {
        "supported": bool(payload[0] & 1),
        "device": payload[1],
        "rate_num": payload[2],
        "rate_denom": payload[3],
        "p_denom": u16(payload, 4),
        "sample_rate": payload[6],
        "disabled_mask": u32(payload, 7),
    }


def pull_log(fc: Fc, used: int) -> bytes:
    """Копия цикла из useDataflashPull.js: блоки по 4096 байт до конца накопителя."""
    out = bytearray()
    address = 0
    while address < used:
        reply = fc.call(MSP_DATAFLASH_READ, address.to_bytes(4, "little")
                        + BLOCK_SIZE.to_bytes(2, "little") + b"\x01")
        if len(reply) < 7:
            break
        got_address = u32(reply, 0)
        read_len = u16(reply, 4)
        compression = reply[6]
        assert got_address == address, f"ответ не про тот адрес: {got_address} != {address}"
        assert compression == 0, f"вкладка не умеет сжатие {compression}"
        chunk = reply[7:7 + read_len]
        if not chunk:
            break
        out.extend(chunk)
        address += len(chunk)
    return bytes(out)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--tcp", default=None)
    args = parser.parse_args()
    if args.port or args.tcp:
        port = args.port or int(str(args.tcp).split(":")[-1])
    else:
        probe = socket.socket()
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()

    eeprom = Path("/tmp/check_blackbox_eeprom.json")
    flash_file = eeprom.with_name("blackbox_flash.bin")
    index_file = eeprom.with_name("blackbox_flash.index.json")
    for path in (eeprom, flash_file, index_file):
        path.unlink(missing_ok=True)

    probe = socket.socket()
    probe.bind(("127.0.0.1", 0))
    http_port = probe.getsockname()[1]
    probe.close()
    proc, fc = start_emulator(port, eeprom, http=http_port)
    print("=" * 78)
    print("ВКЛАДКА «ЧЁРНЫЙ ЯЩИК»: накопитель, чтение лога, настройки записи")
    print("=" * 78)

    # --- MSP_DATAFLASH_SUMMARY (70)
    payload = fc.call(MSP_DATAFLASH_SUMMARY)
    check(len(payload) >= 13, f"ответ MSP_DATAFLASH_SUMMARY: {len(payload)} байт (flags + 3×u32)")
    summary = decode_summary(payload)
    check(summary["supported"], "флаг «накопитель поддерживается»")
    check(summary["ready"], "флаг «готов» — вкладка не покажет «нет данных»")
    check(summary["sectors"] == 4096, f"секторов: {summary['sectors']} (W25Q128, 4 КиБ каждый)")
    check(summary["size"] == 16 * 1024 * 1024, f"размер накопителя: {summary['size']} Б (16 МиБ)")
    check(summary["used"] > 0, f"занято данных: {summary['used']} Б — вкладка видит логи")
    print(f"   свободно: {summary['size'] - summary['used']} Б, "
          f"заполнено {summary['used'] / summary['size'] * 100:.1f} %")

    # --- MSP_DATAFLASH_READ (71): выкачиваем лог как вкладка
    print("\n-- выкачивание лога (блоки по 4096 байт, как «Сохранить лог») --")
    t0 = time.time()
    data = pull_log(fc, summary["used"])
    check(len(data) == summary["used"],
          f"скачано {len(data)} Б из {summary['used']} за {time.time() - t0:.1f} с")
    check(data.startswith(MARKER), "скачанное начинается с заголовка лога («H Product:…»)")
    starts = [i for i in range(len(data)) if data.startswith(MARKER, i)]
    check(len(starts) == 6, f"в «флеше» {len(starts)} логов — эмулятор наполнил его пролётами")
    decode = Path(os.environ.get("BB_DECODE", "/tmp/bbtools/obj/blackbox_decode"))
    work = Path(tempfile.gettempdir())
    sample_path = work / "check_blackbox_pull.bbl"
    sample_path.write_bytes(data)
    check(data == sample_path.read_bytes() and data[:64] == data[:64], "файл записан для просмотрщика")
    if decode.exists():
        result = subprocess.run([str(decode), str(sample_path)], cwd=str(work),
                                capture_output=True, text=True)
        out = result.stdout + result.stderr
        check("frames failed" not in out.lower() and "unreadable" not in out.lower(),
              "декодер betaflight/blackbox-tools не нашёл негодных кадров")
        check("Data rate  250Hz" in out, "декодер видит поток 250 Гц (P interval 4, looptime 1000)")
    else:
        print("   [ -- ] blackbox_decode не собран — проверка декодером пропущена")

    # --- устаревший запрос (без длины)
    legacy = fc.call(MSP_DATAFLASH_READ, (0).to_bytes(4, "little"))
    check(len(legacy) == 4 + 128,
          f"устаревший запрос (только адрес) отдаёт адрес + 128 байт, а не {len(legacy)}")
    check(legacy[4:] == data[:128], "устаревший формат отдаёт те же байты")

    # --- MSP_BLACKBOX_CONFIG (80)
    print("\n-- настройки записи (MSP_BLACKBOX_CONFIG) --")
    config = decode_blackbox_config(fc.call(MSP_BLACKBOX_CONFIG))
    print(f"   устройство {config['device']}, rateNum {config['rate_num']}, "
          f"rateDenom {config['rate_denom']}, P ratio {config['p_denom']}, "
          f"частота 1/{1 << config['sample_rate']}, маска 0x{config['disabled_mask']:08x}")
    check(config["supported"], "флажок «чёрный ящик поддерживается» (вкладка не блокирует настройки)")
    check(config["device"] == 1, "устройство по умолчанию — SPI flash (1)")
    check(config["rate_denom"] == 1 << config["sample_rate"],
          "rateDenom = 1/P interval — как blackboxGetRateDenom()")
    check(config["p_denom"] * (1 << config["sample_rate"]) == 32,
          "P ratio = I interval / P interval (32 / 4)")

    # --- MSP_SET_BLACKBOX_CONFIG (81): меняем частоту и маску, как вкладка
    print("\n-- запись настроек из вкладки (кнопка «Сохранить») --")
    fc.call(MSP_SET_BLACKBOX_CONFIG,
            bytes([1, 1, 8]) + (8).to_bytes(2, "little") + bytes([3]) + (0b101).to_bytes(4, "little"))
    after = decode_blackbox_config(fc.call(MSP_BLACKBOX_CONFIG))
    print(f"   после записи: устройство {after['device']}, частота 1/{1 << after['sample_rate']}, "
          f"маска 0x{after['disabled_mask']:08x}")
    check(after["sample_rate"] == 3, "частота записи принята (1/8)")
    check(after["disabled_mask"] == 0b101, "маска выключенных полей принята")
    fc.call(MSP_EEPROM_WRITE)
    fc.close()
    proc.terminate()
    proc.wait(timeout=10)

    proc, fc = start_emulator(port, eeprom, http=http_port)
    saved = decode_blackbox_config(fc.call(MSP_BLACKBOX_CONFIG))
    check(saved["sample_rate"] == 3 and saved["disabled_mask"] == 0b101,
          "настройки пережили перезапуск эмулятора")
    print("   [ .. ] пересборка «флеша» под новую частоту идёт в фоне; ждём")
    deadline = time.time() + 40
    same_rate = False
    while time.time() < deadline:
        s = decode_summary(fc.call(MSP_DATAFLASH_SUMMARY))
        entry = None
        if s["used"]:
            entry = bb_scenarios.scan_logs(bytes(pull_log(fc, min(s["used"], 4096)))) or None
        if entry and entry[0]["p_interval"] == "8":
            same_rate = True
            break
        time.sleep(1.0)
    check(same_rate, "«P interval» в заголовке лога совпадает с настройкой после перезапуска")

    # --- режим запоминающего устройства (MSP_SET_REBOOT с кодом 2)
    print("\n-- режим запоминающего устройства (кнопка «Активировать режим ЗУ») --")
    if http_port:
        import urllib.request

        # «Флеш» к этому моменту мог пересобраться (мы меняли частоту записи) —
        # поэтому берём свежую сводку и свежую копию данных, а не раннюю выборку.
        current_summary = decode_summary(fc.call(MSP_DATAFLASH_SUMMARY))
        before = current_summary["used"]
        current = pull_log(fc, before)
        payload = fc.call(MSP_SET_REBOOT, bytes([2]))
        check(payload == bytes([2, 1]),
              f"ответ на MSP_SET_REBOOT: режим {payload[0] if payload else '—'}, "
              f"готовность {payload[1] if len(payload) > 1 else '—'} (как sbufWriteU8 в msp.c)")
        time.sleep(1.0)
        silent = True
        try:
            probe = Fc(port)
            silent = not probe.call(MSP_DATAFLASH_SUMMARY, timeout=2.0)
            probe.close()
        except OSError:
            pass
        check(silent, "в режиме ЗУ MSP молчит — как настоящий борт, ушедший в режим ЗУ")

        page = urllib.request.urlopen(f"http://127.0.0.1:{http_port}/msc", timeout=5).read().decode()
        names = [f"LOG{number:05d}.BBL" for number in range(1, 7)]
        check(all(name in page for name in names),
              "страница /msc показывает по файлу на пролёт: " + ", ".join(names))
        check("BLACKBOX_ALL.BBL" in page, "там же — общий дамп (то же, что кнопка «Сохранено в файл»)")
        work = Path(tempfile.gettempdir())
        first = urllib.request.urlopen(f"http://127.0.0.1:{http_port}/msc/LOG00001.BBL", timeout=10).read()
        check(first == current[:len(first)] and first.startswith(MARKER),
              f"отдельный файл LOG00001.BBL ({len(first)} Б) совпадает с первым логом «флеша»")
        whole = urllib.request.urlopen(f"http://127.0.0.1:{http_port}/msc/BLACKBOX_ALL.BBL",
                                       timeout=30).read()
        check(len(whole) == before, f"общий дамп: {len(whole)} Б — все пролёты одним файлом")
        msc_path = work / "check_blackbox_msc_log1.bbl"
        msc_path.write_bytes(first)
        if decode.exists():
            result = subprocess.run([str(decode), str(msc_path)], cwd=str(work),
                                    capture_output=True, text=True)
            out = result.stdout + result.stderr
            check("Frames" in out and "failed to decode" not in out,
                  "отдельный файл читается декодером blackbox_decode")
        urllib.request.urlopen(f"http://127.0.0.1:{http_port}/msc/exit", timeout=5).read()
        time.sleep(1.0)
        fc = Fc(port)               # линк рвался при входе в режим ЗУ — подключаемся заново
        after = decode_summary(fc.call(MSP_DATAFLASH_SUMMARY))["used"]
        check(after == before, f"после выхода из режима ЗУ MSP отвечает снова, данные на месте ({after} Б)")
        for junk in (msc_path, work / "check_blackbox_msc_log1.01.csv",
                     work / "check_blackbox_msc_log1.01.event"):
            junk.unlink(missing_ok=True)
    else:
        print("   [ .. ] веб-монитор не поднят — проверка файлов ЗУ пропущена (нужен --http)")

    # --- MSP_DATAFLASH_ERASE (72)
    print("\n-- очистка накопителя (кнопка «Стереть») --")
    fc.call(MSP_DATAFLASH_ERASE)
    cleared = decode_summary(fc.call(MSP_DATAFLASH_SUMMARY))
    check(cleared["used"] == 0, f"после очистки занято {cleared['used']} Б")
    check(cleared["size"] == 16 * 1024 * 1024, "размер накопителя не изменился — вкладка не «сломалась»")

    # --- CLI: сводка и наполнение заново
    print("\n-- CLI: blackbox / blackbox refill --")
    # Наполнение идёт в фоне (несколько секунд), поэтому после 'refill' ждём:
    # ровно так же ждёт и вкладка — опрашивает сводку, пока накопитель не наполнится.
    cli = subprocess.run([sys.executable, "-c",
                          f"import sys, time; sys.path.insert(0, {str(ROOT)!r});\n"
                          "from bfemu.fc import VirtualFC\n"
                          f"fc = VirtualFC(eeprom_path={str(eeprom)!r}, log=lambda m: None)\n"
                          "print(fc._cli.execute_line('blackbox refill'))\n"
                          "out = ''\n"
                          "for _ in range(60):\n"
                          "    time.sleep(1.0)\n"
                          "    out = fc._cli.execute_line('blackbox')\n"
                          "    if out.count('log ') >= 6: break\n"
                          "print(out)"],
                         capture_output=True, text=True, cwd=ROOT)
    text = cli.stdout
    check("Blackbox (dataflash)" in text and "used" in text,
          "команда CLI показывает сводку по накопителю")
    check("refilling flash with 6 test flights in background" in text,
          "наполнение запускается в фоне — CLI и вкладка отвечают сразу")
    check(text.count("log ") >= 6, "в сводке снова 6 тестовых пролётов")
    check("motor_fault" in text and "arm_resonance" in text and "square" in text,
          "среди пролётов есть отказ двигателя, резонанс луча и полёт по квадрату")

    fc.close()
    proc.terminate()
    proc.wait(timeout=10)
    for path in (eeprom, flash_file, index_file, sample_path,
                 sample_path.with_suffix(".01.csv"), sample_path.with_suffix(".01.event")):
        path.unlink(missing_ok=True)

    print()
    if FAILED:
        print(f"ПРОВАЛЕНО проверок: {len(FAILED)}")
        for item in FAILED:
            print("  -", item)
        return 1
    print("ВКЛАДКА «ЧЁРНЫЙ ЯЩИК» РАБОТАЕТ: накопитель виден, лог выкачивается и разбирается "
          "декодером, настройки записи сохраняются, очистка и повторное наполнение работают")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

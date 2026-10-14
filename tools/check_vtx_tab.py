#!/usr/bin/env python3
"""Проверка вкладки «ВИДЕОПЕРЕДАТЧИК»: таблица VTX, запись и сохранение.

Скрипт повторяет то, что делает вкладка Configurator 2026.6.2 (composables/useVtx.js):

  1. загрузка: MSP_VTX_CONFIG → MSP_VTXTABLE_BAND на каждый банд → MSP_VTXTABLE_POWERLEVEL
     на каждый уровень → MSP2_GET_VTX_DEVICE_STATUS (панель «Состояние устройства»);
  2. правка: пользователь задаёт число бандов/каналов/уровней и заполняет сетку —
     ровно это уходит в MSP_SET_VTX_CONFIG + MSP_SET_VTXTABLE_BAND/POWERLEVEL;
  3. «Сохранить» → MSP_EEPROM_WRITE, затем «Сохранить и перезагрузить» → ребут;
  4. полный перезапуск процесса эмулятора: таблица должна вернуться;
  5. CLI `vtxtable` должен показывать ту же таблицу.

Запуск:  python3 tools/check_vtx_tab.py        (Windows: python tools\\check_vtx_tab.py)
"""
from __future__ import annotations

import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from bfemu.msp import encode_v1, encode_v2  # noqa: E402

PORT = 16041
FAILS: list[str] = []


# ----------------------------------------------------------------- транспорт
def _frame(buf: bytes):
    for i in range(len(buf) - 5):
        if buf[i] != 0x24:
            continue
        if buf[i + 1] == 0x4D and buf[i + 2] == 0x3E:                 # $M> (v1)
            size = buf[i + 3]
            if i + 5 + size <= len(buf):
                return buf[i + 5 : i + 5 + size]
        if buf[i + 1] == 0x58 and buf[i + 2] == 0x3E:                 # $X> (v2)
            size = buf[i + 6] | (buf[i + 7] << 8)
            if i + 8 + size <= len(buf):
                return buf[i + 8 : i + 8 + size]
    return None


class Fc:
    def __init__(self, port: int):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=3)
        self.sock.settimeout(0.4)

    def call(self, code: int, payload: bytes = b"", v2: bool = False) -> bytes:
        self.sock.sendall(encode_v2(code, payload) if v2 else encode_v1(code, payload))
        buf, deadline = b"", time.time() + 2.5
        while time.time() < deadline:
            try:
                buf += self.sock.recv(65536)
            except TimeoutError:
                break
            frame = _frame(buf)
            if frame is not None:
                return frame
            time.sleep(0.03)
        return b""

    def cli(self, command: str, window: int = 2.5) -> str:
        """Аналог MSP.send_cli_command: пейджинг через MSP2_CLI_COMMAND."""
        out, offset = b"", 0
        deadline = time.time() + window
        while time.time() < deadline:
            payload = command.encode() if offset == 0 else command.encode() + b"\x00" + offset.to_bytes(2, "little")
            resp = self.call(0x3012, payload, v2=True)
            if len(resp) < 3:
                break
            out += resp[3:]
            offset += len(resp) - 3
            if offset >= int.from_bytes(resp[0:2], "little") or len(resp) == 3:
                break
        return out.decode("latin-1", "replace").replace("\r\n", "\n")

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def start(port: int, eeprom: Path):
    proc = subprocess.Popen(
        [sys.executable, "-m", "bfemu", "--tcp", f"127.0.0.1:{port}", "--eeprom", str(eeprom), "--quiet"],
        cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
    )
    for _ in range(60):
        try:
            return proc, Fc(port)
        except OSError:
            time.sleep(0.25)
    raise RuntimeError("эмулятор не поднялся")


def stop(proc):
    proc.terminate()
    proc.wait(timeout=10)


def check(ok: bool, what: str):
    print(f"   {'[OK ]' if ok else '[НЕТ]'} {what}")
    if not ok:
        FAILS.append(what)


# ----------------------------------------------------------------- разбор ответов
def u8(b, off):
    return b[off] if off < len(b) else 0


def u16(b, off):
    return int.from_bytes(b[off : off + 2], "little") if off + 2 <= len(b) else 0


def u32(b, off):
    return int.from_bytes(b[off : off + 4], "little") if off + 4 <= len(b) else 0


def parse_vtx_config(b: bytes) -> dict:
    """Порядок полей — как в MSPHelper.js (MSP_VTX_CONFIG)."""
    return {
        "type": u8(b, 0), "band": u8(b, 1), "channel": u8(b, 2), "power": u8(b, 3),
        "pit_mode": u8(b, 4) != 0, "frequency": u16(b, 5), "device_ready": u8(b, 7) != 0,
        "low_power_disarm": u8(b, 8), "pit_mode_frequency": u16(b, 9), "table_available": u8(b, 11) != 0,
        "bands": u8(b, 12), "channels": u8(b, 13), "powerlevels": u8(b, 14),
    }


def parse_band(b: bytes) -> dict:
    number = u8(b, 0)
    name_len = u8(b, 1)
    name = "".join(chr(c) for c in b[2 : 2 + name_len])
    pos = 2 + name_len
    letter = chr(u8(b, pos))
    is_factory = u8(b, pos + 1) != 0
    count = u8(b, pos + 2)
    freqs = [u16(b, pos + 3 + 2 * i) for i in range(count)]
    return {"number": number, "name": name.rstrip(), "letter": letter,
            "is_factory": is_factory, "frequencies": freqs}


def parse_powerlevel(b: bytes) -> dict:
    level = u8(b, 0)
    value = u16(b, 1)
    label_len = u8(b, 3)
    label = "".join(chr(c) for c in b[4 : 4 + label_len])
    return {"level": level, "value": value, "label": label.rstrip()}


def parse_device_status(b: bytes) -> dict | None:
    """VtxDeviceStatus + SmartAudio-надстройка (VtxDeviceStatus.js/SmartAudioDeviceStatus.js)."""
    if not b:
        return None
    type_index = u8(b, 0)
    device_ready = u8(b, 1) != 0
    band_available = u8(b, 2) != 0
    band, channel = u8(b, 3), u8(b, 4)
    power_available, power_index = u8(b, 5) != 0, u8(b, 6)
    freq_available, frequency = u8(b, 7) != 0, u16(b, 8)
    status_available, status = u8(b, 10) != 0, u32(b, 11)
    count = u8(b, 15)
    levels = [(u16(b, 16 + 4 * i), u16(b, 18 + 4 * i)) for i in range(count)]
    pos = 16 + 4 * count
    custom_size = u8(b, pos)
    sa = None
    if custom_size >= 5:
        sa = {"version": u8(b, pos + 1), "mode": u8(b, pos + 2),
              "orfreq": u16(b, pos + 3), "will_boot_into_pit_mode": u8(b, pos + 5) != 0}
    return {"type": type_index, "ready": device_ready, "band_available": band_available,
            "band": band, "channel": channel, "power_available": power_available,
            "power_index": power_index, "frequency_available": freq_available, "frequency": frequency,
            "status_available": status_available, "status": status,
            "power_levels": levels, "smart_audio": sa}


def ports_vtx_banner(config: dict) -> bool:
    """Ровно то, что проверяет Configurator (usePortsState.js: vtxTableNotConfigured).

    Плашка «ВНИМАНИЕ: Таблица VTX не была правильно настроена» появляется, когда таблица
    доступна, но пустая — то есть когда полос, каналов или уровней мощности нет.
    """
    return bool(config.get("table_available")) and (
        config.get("bands") == 0 or config.get("channels") == 0 or config.get("powerlevels") == 0
    )


def read_tab(fc: Fc) -> dict:
    """Полная загрузка вкладки: конфиг, таблица, состояние устройства."""
    cfg = parse_vtx_config(fc.call(88))
    bands = [parse_band(fc.call(137, bytes([i]))) for i in range(1, cfg["bands"] + 1)]
    powers = [parse_powerlevel(fc.call(138, bytes([i]))) for i in range(1, cfg["powerlevels"] + 1)]
    # 0x3004 — код MSP2: отправляем сообщением версии 2, иначе младший байт
    # (0x04) прочитается как MSP_BOARD_INFO.
    status = fc.call(0x3004, v2=True)
    return {"config": cfg, "bands": bands, "powers": powers, "device": parse_device_status(status)}


def show(tab: dict, title: str):
    cfg = tab["config"]
    print(f"\n-- {title} --")
    print(f"   Тип VTX     : {cfg['type']} (3 = TBS SmartAudio), готовность: {cfg['device_ready']}")
    print(f"   Выбрано     : банд {cfg['band']}, канал {cfg['channel']}, мощность {cfg['power']}, "
          f"частота {cfg['frequency']} МГц")
    print(f"   Pit-режим   : {'вкл' if cfg['pit_mode'] else 'выкл'}, частота pit {cfg['pit_mode_frequency']}, "
          f"низкая мощность на disarm: {cfg['low_power_disarm']}")
    print(f"   Таблица     : бандов {cfg['bands']}, каналов {cfg['channels']}, уровней мощности {cfg['powerlevels']}")
    for band in tab["bands"]:
        flag = "FACTORY" if band["is_factory"] else "CUSTOM "
        print(f"     {band['number']}: {band['name']:<8} {band['letter']} {flag} "
              + " ".join(str(f) for f in band["frequencies"]))
    for power in tab["powers"]:
        print(f"     уровень {power['level']}: {power['value']} мВт, подпись {power['label']!r}")
    device = tab["device"]
    if device:
        sa = device.get("smart_audio") or {}
        print(f"   Устройство  : тип {device['type']}, готово {device['ready']}, "
              f"банд {device['band']}/канал {device['channel']}, мощность №{device['power_index']}, "
              f"{device['frequency']} МГц, флаги 0x{device['status']:x}")
        print("     уровни (dBm → мВт): " + ", ".join(f"{dbm} → {mw}" for dbm, mw in device["power_levels"]))
        print(f"     SmartAudio  : версия {sa.get('version')} (3 = 2.1), mode {sa.get('mode')}, "
              f"orfreq {sa.get('orfreq')}")


# ----------------------------------------------------------------- запись как из вкладки
def write_tab(fc: Fc, bands: list[dict], powers: list[dict], channels: int,
              band: int, channel: int, power: int, frequency: int = 0,
              low_power_disarm: int = 0, pit_mode_freq: int = 0):
    """MSP-записи в том же порядке, что saveVtx(): конфиг → уровни → банды."""
    cfg = bytearray(15)
    cfg[0:2] = frequency.to_bytes(2, "little")       # vtx_frequency (0 = режим бандов)
    cfg[2] = power
    cfg[3] = 0                                  # pit-режим выключен (как в интерфейсе)
    cfg[4] = low_power_disarm
    cfg[5:7] = pit_mode_freq.to_bytes(2, "little")
    cfg[7] = band
    cfg[8] = channel
    cfg[9:11] = frequency.to_bytes(2, "little")
    cfg[11] = len(bands)
    cfg[12] = channels
    cfg[13] = len(powers)
    cfg[14] = 1                                  # vtx_table_clear: таблица заполняется заново
    fc.call(89, bytes(cfg))
    for level, item in enumerate(powers, start=1):
        payload = bytes([level]) + item["value"].to_bytes(2, "little") + bytes([len(item["label"])]) \
            + item["label"].encode()
        fc.call(228, payload)
    for number, item in enumerate(bands, start=1):
        name = item["name"].encode()
        payload = (bytes([number, len(name)]) + name + item["letter"].encode()
                   + bytes([1 if item["is_factory"] else 0, channels])
                   + b"".join(int(f).to_bytes(2, "little") for f in item["frequencies"][:channels]))
        fc.call(227, payload)


def main() -> int:
    eeprom = Path(tempfile.mkdtemp()) / "eeprom.json"
    proc, fc = start(PORT, eeprom)

    print("=" * 78)
    print("ВКЛАДКА «ВИДЕОПЕРЕДАТЧИК»: что отдаёт эмулятор (как на живом FC с SmartAudio)")
    print("=" * 78)
    tab = read_tab(fc)
    check(len(fc.call(88)) == 15, f"MSP_VTX_CONFIG: {len(fc.call(88))} байт (нужно 15)")
    check(tab["config"]["type"] == 3, f"тип VTX = {tab['config']['type']} (3 = TBS SmartAudio, порт UART4)")
    check(tab["config"]["table_available"], "таблица VTX доступна (сборка с USE_VTX_TABLE)")
    check((tab["config"]["bands"], tab["config"]["channels"], tab["config"]["powerlevels"]) == (5, 8, 4),
          f"таблица уже настроена: полос {tab['config']['bands']}, каналов {tab['config']['channels']}, "
          f"уровней мощности {tab['config']['powerlevels']}")
    check(not ports_vtx_banner(tab["config"]),
          "жёлтой плашки «таблица VTX не настроена» на вкладке «Порты» НЕТ")
    check(tab["config"]["frequency"] == 5865,
          f"канал выбран: полоса {tab['config']['band']}, {tab['config']['frequency']} МГц (BOSCAM A)")
    check(tab["device"] is not None and tab["device"]["ready"], "устройство опрошено и готово (0x3004)")
    show(tab, "Заводская таблица (то, что видно во вкладке сразу после подключения)")

    # --------- сценарий A: заполняем таблицу заново, как это делает пользователь в интерфейсе
    print("\n" + "=" * 78)
    print("ПЕРЕЗАПИСЫВАЕМ ТАБЛИЦУ (банды A/B/E/F/R, 8 каналов, 4 уровня мощности)")
    print("=" * 78)
    standard = {
        "BOSCAM_A": ("A", [5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725]),
        "BOSCAM_B": ("B", [5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866]),
        "BOSCAM_E": ("E", [5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945]),
        "FATSHARK": ("F", [5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880]),
        "RACEBAND": ("R", [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917]),
    }
    bands = [{"name": name, "letter": letter, "is_factory": True, "frequencies": freqs}
             for name, (letter, freqs) in standard.items()]
    powers = [{"value": v, "label": str(v)} for v in (25, 100, 400, 800)]
    write_tab(fc, bands, powers, channels=8, band=5, channel=1, power=3, low_power_disarm=1, pit_mode_freq=0)
    fc.call(250)                                  # MSP_EEPROM_WRITE — кнопка «Сохранить»
    tab = read_tab(fc)
    show(tab, "Таблица после заполнения")
    check(tab["config"]["bands"] == 5 and tab["config"]["channels"] == 8,
          f"размеры таблицы: {tab['config']['bands']}×{tab['config']['channels']}")
    check(tab["config"]["powerlevels"] == 4, f"уровней мощности: {tab['config']['powerlevels']}")
    check(tab["bands"][4]["name"] == "RACEBAND" and tab["bands"][4]["frequencies"][0] == 5658,
          f"банд 5: {tab['bands'][4]['name']} / {tab['bands'][4]['frequencies'][0]} МГц")
    check(tab["bands"][3]["name"] == "FATSHARK" and tab["bands"][3]["letter"] == "F",
          f"банд 4: {tab['bands'][3]['name']} / буква {tab['bands'][3]['letter']}")
    check(tab["powers"][3]["value"] == 800 and tab["powers"][3]["label"] == "800",
          f"уровень 4: {tab['powers'][3]['value']} мВт, подпись {tab['powers'][3]['label']!r}")
    check(tab["config"]["band"] == 5 and tab["config"]["channel"] == 1,
          f"выбран банд {tab['config']['band']}/канал {tab['config']['channel']}")
    check(tab["config"]["frequency"] == 5658, f"частота выбрана из таблицы: {tab['config']['frequency']} МГц")
    check(tab["config"]["low_power_disarm"] == 1, f"низкая мощность на disarm: {tab['config']['low_power_disarm']}")
    check(tab["device"]["power_levels"][3] == (29, 800) if len(tab["device"]["power_levels"]) > 3 else False,
          f"устройство показывает уровни: {tab['device']['power_levels']}")

    # --------- CLI
    out = fc.cli("vtxtable")
    check("vtxtable bands 5" in out and "vtxtable channels 8" in out, "CLI: размеры таблицы")
    check("vtxtable band 1 BOSCAM_A A FACTORY 5865" in out.replace("  ", " ") or
          "vtxtable band 1 BOSCAM_A A FACTORY  5865" in out,
          "CLI: банд 1 = BOSCAM_A с частотами")
    check("vtxtable powerlabels 25 100 400 800" in out, "CLI: подписи уровней мощности")
    info = fc.cli("vtx_info")
    check("level 29 dBm, power 800 mW" in info, f"CLI vtx_info: {info.strip().splitlines()[-1]}")

    # --------- сохранение через ребут и полный перезапуск процесса
    print("\n" + "=" * 78)
    print("ПЕРЕЗАГРУЗКА И ПОЛНЫЙ ПЕРЕЗАПУСК ЭМУЛЯТОРА")
    print("=" * 78)
    fc.call(68, bytes([0]))                       # Save and Reboot
    time.sleep(1.2)
    fc.close()
    stop(proc)

    proc, fc2 = start(PORT, eeprom)
    tab2 = read_tab(fc2)
    check(tab2["config"]["bands"] == 5 and tab2["config"]["channels"] == 8
          and tab2["config"]["powerlevels"] == 4, "таблица пережила перезапуск процесса")
    check(tab2["bands"][4]["name"] == "RACEBAND" and tab2["bands"][4]["frequencies"][1] == 5695,
          "частоты RaceBand на месте")
    check(tab2["config"]["band"] == 5 and tab2["config"]["frequency"] == 5658,
          f"выбранный канал сохранён: банд {tab2['config']['band']}, {tab2['config']['frequency']} МГц")
    check(tab2["config"]["low_power_disarm"] == 1, "режим низкой мощности сохранён")

    # --------- сценарий B: смена таблицы на «свою» (как загрузка JSON из файла)
    print("\n-- Правка таблицы: меняем банд 2 и уровень 2, ставим pit-режим --")
    fc2.call(89, bytes([0, 0, 2, 1, 0, 0, 0, 2, 2, 0, 0, 2, 8, 2, 0]))   # 2 банда, 8 каналов, 2 уровня, без очистки
    fc2.call(228, bytes([2]) + (500).to_bytes(2, "little") + bytes([3]) + b"500")
    fc2.call(227, bytes([2, 4]) + b"TEAM" + b"T" + bytes([0, 8]) + b"".join((5690 + 10 * i).to_bytes(2, "little") for i in range(8)))
    fc2.call(250)
    fc2.call(68, bytes([0]))
    time.sleep(1.2)
    fc2.close()
    stop(proc)

    proc, fc3 = start(PORT, eeprom)
    tab3 = read_tab(fc3)
    check(tab3["config"]["bands"] == 2 and tab3["config"]["powerlevels"] == 2, "новые размеры таблицы")
    check(tab3["bands"][1]["name"] == "TEAM" and tab3["bands"][1]["frequencies"][7] == 5760,
          f"правка банда 2: {tab3['bands'][1]['name']} … {tab3['bands'][1]['frequencies'][7]} МГц")
    check(tab3["powers"][1]["value"] == 500, f"правка уровня 2: {tab3['powers'][1]['value']} мВт")
    check(tab3["config"]["pit_mode"] is True, "pit-режим включён и сохранён")
    check(tab3["device"] is not None and tab3["device"]["status"] & 1 == 1,
          f"устройство рапортует pit-режим: флаги 0x{tab3['device']['status']:x}")
    show(tab3, "После правки и перезапуска")
    fc3.close()
    stop(proc)

    print()
    print("=" * 78)
    if FAILS:
        print(f"ПРОВАЛЕНО проверок: {len(FAILS)}")
        for item in FAILS:
            print("  -", item)
        return 1
    print("ВКЛАДКА «ВИДЕОПЕРЕДАТЧИК» РАБОТАЕТ: таблица читается, пишется и сохраняется")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

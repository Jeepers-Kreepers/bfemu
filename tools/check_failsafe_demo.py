#!/usr/bin/env python3
"""Проверка демонстрации failsafe: страница /failsafe, CLI `sim_failsafe`, машина фаз.

Что проверяем (всё — по исходникам Betaflight 4.5.4, копии в docs/):

  * времена и пороги берутся из настроек вкладки Failsafe: failsafe_delay 1.5 с,
    failsafe_off_delay 1.0 с, failsafe_throttle 1000, failsafe_throttle_low_delay 10 с,
    failsafe_procedure DROP, failsafe_recovery_delay 500 мс, failsafe_stick_threshold 30 %;
  * потеря сигнала: кадры HOLD (rx.c MAX_INVALID_PULSE_TIME_MS 300 мс) -> «Этап 1»
    (rxfail: у газа rx_min_usec по сетке 25 мкс = 900 мкс) -> ТОЧКА ОТКАТА (страж-время
    истекло) -> «Этап 2»: стики midrc 1500, газ failsafe_throttle 1000 (rx.c:688-725);
  * процедуры: DROP — моторы сразу в 1000; AUTO-LAND — газ убирается за off_delay;
    GPS-RESCUE — фазы gps_rescue.c (INITIALIZE -> ATTAIN_ALT -> ROTATE -> FLY_HOME ->
    DESCENT -> LANDING -> COMPLETE), проверки (мало спутников, газа зависания не хватает);
  * тумблер FAILSAFE (вкладка «Режимы»): STAGE1 — как потеря связи, STAGE2 — сразу Этап 2,
    KILL — моторы сразу (failsafe.c, FAILSAFE_IDLE);
  * JustDisarm: газ ниже min_check дольше failsafe_throttle_low_delay;
  * взвод: во время «Этапа 2» поднят ARMING_DISABLED_FAILSAFE, после возврата связи снят;
  * флаги в MSP_STATUS (RX_FAILSAFE 1<<2, FAILSAFE 1<<1);
  * страница /failsafe (кнопки, «ОТКАТ ВОЗМОЖЕН/НЕВОЗМОЖЕН», журнал), /api/failsafe,
    карточка на мониторе, CLI `sim_failsafe status`.

Запуск:  python3 tools/check_failsafe_demo.py          (Windows: python tools\\check_failsafe_demo.py)
"""
from __future__ import annotations

import json
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from bfemu.msp import encode_v1, encode_v2  # noqa: E402

MSP_PORT = 16081
HTTP_PORT = 16082
FAILS: list[str] = []


def check(ok: bool, what: str):
    print(f"   {'[OK ]' if ok else '[НЕТ]'} {what}")
    if not ok:
        FAILS.append(what)


def _frame(buf: bytes):
    for i in range(len(buf) - 5):
        if buf[i] != 0x24:
            continue
        if buf[i + 1] == 0x4D and buf[i + 2] == 0x3E:
            size = buf[i + 3]
            if i + 5 + size <= len(buf):
                return buf[i + 5: i + 5 + size]
        if buf[i + 1] == 0x58 and buf[i + 2] == 0x3E:
            size = buf[i + 6] | (buf[i + 7] << 8)
            if i + 8 + size <= len(buf):
                return buf[i + 8: i + 8 + size]
    return None


class Fc:
    """MSP-клиент: то, что видит Configurator (вкладки «Приёмник», Failsafe, CLI)."""

    def __init__(self, port: int):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=3)
        self.sock.settimeout(0.3)

    def call(self, code: int, payload: bytes = b"", v2: bool = False) -> bytes:
        self.sock.sendall(encode_v2(code, payload) if v2 else encode_v1(code, payload))
        buf, deadline = b"", time.time() + 2.5
        while time.time() < deadline:
            try:
                buf += self.sock.recv(65536)
            except TimeoutError:
                pass
            frame = _frame(buf)
            if frame is not None:
                return frame
            time.sleep(0.02)
        return b""

    def cli(self, command: str) -> str:
        out, offset, deadline = b"", 0, time.time() + 3.0
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

    def rc(self) -> list:
        data = self.call(105)
        return [int.from_bytes(data[i * 2: i * 2 + 2], "little") for i in range(len(data) // 2)]

    def motors(self) -> list:
        data = self.call(104)
        return [int.from_bytes(data[i * 2: i * 2 + 2], "little") for i in range(len(data) // 2)]

    def arming_flags(self) -> int:
        data = self.call(101)                      # MSP_STATUS_EX
        if len(data) < 21:
            data = self.call(150)                  # MSP_STATUS
        return int.from_bytes(data[17:21], "little")

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def http(path: str, timeout: float = 6.0):
    with urllib.request.urlopen(f"http://127.0.0.1:{HTTP_PORT}{path}", timeout=timeout) as r:
        return r.status, r.read()


def api(path: str = "/api/failsafe") -> dict:
    return json.loads(http(path)[1])


def start():
    proc = subprocess.Popen(
        [sys.executable, "-m", "bfemu", "--tcp", f"127.0.0.1:{MSP_PORT}", "--http", str(HTTP_PORT),
         "--quiet"],
        cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
    )
    for _ in range(80):
        try:
            return proc, Fc(MSP_PORT)
        except OSError:
            time.sleep(0.25)
    raise RuntimeError("эмулятор не поднялся")


def wait_state(cond, timeout=8.0, step=0.08):
    """Ждём состояние демо (его обновляет опрос страницы, как в жизни)."""
    end = time.time() + timeout
    state = api()
    while time.time() < end:
        if cond(state):
            return state
        time.sleep(step)
        state = api()
    return state


def main() -> int:
    proc, fc = start()
    try:
        print("1. Страница /failsafe, монитор и CLI `sim_failsafe`")
        status, page = http("/failsafe")
        body = page.decode("utf-8")
        check(status == 200 and "Потерять сигнал" in body and "Вернуть сигнал" in body,
              "страница /failsafe отдаётся: кнопки «Потерять сигнал» и «Вернуть сигнал»")
        check("ТОЧКА ОТКАТА" in body and "ОТКАТ ВОЗМОЖЕН" in body, "на странице есть точка отката и плашка отката")
        check("failsafe_switch_mode" in body and "gps_rescue_min_sats" in body,
              "на странице есть таблицы параметров Failsafe и GPS-спасения")
        status, monitor = http("/")
        check("Failsafe — демонстрация" in monitor.decode("utf-8"),
              "карточка демо есть на мониторе (/), кнопки ведут на /failsafe")
        out = fc.cli("sim_failsafe")
        check("Failsafe demo (emulator)" in out and "state: OFF" in out,
              "CLI: `sim_failsafe` показывает состояние (по умолчанию демо выключено)")
        out = fc.cli("sim_failsafe on")
        check("state: ON" in out and "rollback point" in out, "CLI: `sim_failsafe on` включает демо")
        st = api()
        check(st["demo"] is True and st["scale"] == 0.25,
              f"демо включено, масштаб показа по умолчанию ×{st['scale']:g}")
        check(st["procedure_name"] == "DROP" and st["switch_mode_text"].startswith("как потеря"),
              f"процедура {st['procedure_name']}, тумблер «{st['switch_mode_text'][:28]}…» — как в прошивке")
        check(abs(st["config"]["delay"] - 1.5) < 1e-6 and abs(st["config"]["off_delay"] - 1.0) < 1e-6,
              f"страж-время {st['config']['delay']:.1f} с, off_delay {st['config']['off_delay']:.1f} с")

        print("2. Масштаб времени: ×1 — честные тайминги, ×0.25 — для показа")
        http("/failsafe/scale/1")
        http("/failsafe/loss")
        wait_state(lambda s: s["millis"] >= 500)          # 0.5 с «прошивочного» времени
        st = api()
        left_fast = st["rollback"]["left_s"]
        check(left_fast is not None and 0.6 <= left_fast <= 1.4,
              f"×1: через 0.5 с страж-времени осталось {left_fast} с (идёт по-настоящему)")
        wait_state(lambda s: s["millis"] >= 400)
        check(fc.rc()[:4] == [1500, 1500, 1500, 900],
              f"«Этап 1» в MSP_RC: стики в центре, газ {fc.rc()[3]} мкс (rx_min_usec по сетке 25 мкс)")
        check(fc.arming_flags() & (1 << 2) != 0,
              f"флаг ARMING_DISABLED_RX_FAILSAFE поднят (0x{fc.arming_flags():08X})")
        http("/failsafe/reset")
        http("/failsafe/scale/0.25")
        http("/failsafe/loss")
        wall = time.time()
        wait_state(lambda s: s["millis"] >= 250)          # четверть секунды прошивки
        st = api()
        check(st["rollback"]["left_s"] is not None and st["rollback"]["left_s"] >= 1.0
              and 0.5 <= time.time() - wall <= 2.5,
              f"×0.25: четверть секунды прошивки заняла {time.time() - wall:.1f} с показа — "
              f"осталось {st['rollback']['left_s']} с")
        http("/failsafe/reset")
        check(api()["rollback"]["possible"] and "борт в строю" in api()["rollback"]["text"],
              "«Сброс демо»: связь есть, борт в воздухе, откат возможен")

        print("3. Держим кадры, затем «Этап 1», затем ТОЧКА ОТКАТА и «Этап 2» (DROP)")
        http("/failsafe/scale/1")
        http("/failsafe/loss")
        wait_state(lambda s: s["millis"] >= 120)
        check(fc.rc()[:4] == [1500, 1500, 1500, 1500],
              f"первые 300 мс борт держит последние значения (rx.c): газ {fc.rc()[3]} мкс")
        st = wait_state(lambda s: s["phase_en"] == "RX_LOSS_MONITORING", timeout=6.0)
        check(st["phase_en"] == "RX_LOSS_MONITORING" and st["rollback"]["possible"] is False,
              "страж-время истекло: «Этап 2» идёт, откат невозможен")
        check(st["motors"][:4] == [1000, 1000, 1000, 1000],
              f"DROP: моторы сразу в 1000 ({st['motors'][:4]})")
        check(fc.rc()[:4] == [1500, 1500, 1500, 1000],
              f"«Этап 2» в MSP_RC: газ failsafe_throttle = {fc.rc()[3]} мкс")
        check(fc.arming_flags() & (1 << 1) != 0,
              f"поднят ARMING_DISABLED_FAILSAFE — взвод запрещён до восстановления связи "
              f"(0x{fc.arming_flags():08X})")
        check(st["armed"] is False and st["disarm_reason"] == "FAILSAFE",
              "борт снят с взвода процедурой failsafe (disarm reason FAILSAFE)")
        log = " | ".join(text for _ms, text in st["log"])
        check("страж-время 1.5 с истекло" in log and "DROP — моторы выключены сразу" in log,
              "журнал: страж-время истекло, процедура DROP")

        print("4. «Вернуть сигнал»: failsafe_recovery_delay -> IDLE, взвод разрешён")
        http("/failsafe/restore")
        st = wait_state(lambda s: s["phase_en"] == "IDLE", timeout=6.0)
        check(st["phase_en"] == "IDLE" and st["armed"] is False and fc.arming_flags() & (1 << 1) == 0,
              f"через {st['millis']} мс борт вышел из failsafe, ARMING_DISABLED_FAILSAFE снят "
              f"(0x{fc.arming_flags():08X})")
        log = " | ".join(text for _ms, text in st["log"])
        check("failsafe завершён" in log and "валидных данных" in log,
              "журнал: ждали failsafe_recovery_delay, затем «failsafe завершён»")

        print("5. Откат до точки: сигнал вернулся в страж-время — «Этап 2» не начинается")
        http("/failsafe/reset")
        http("/failsafe/loss")
        wait_state(lambda s: s["millis"] >= 500)
        http("/failsafe/restore")
        wait_state(lambda s: "Этап 1 прерван" in " | ".join(t for _m, t in s["log"]))
        st = api()
        log = " | ".join(text for _ms, text in st["log"])
        marker = next((step for step in st["timeline"] if step.get("marker")), None)
        check(st["phase_en"] == "IDLE" and "Этап 1 прерван" in log and st["rollback"]["possible"],
              "сигнал вернулся до точки отката: управление вернулось, процедура не запускалась")
        check(marker is not None and marker["done"] is False,
              "таймлайн: метка «точка отката» на месте и не пройдена (done=False)")

        print("6. AUTO-LAND: газ убирается за failsafe_off_delay")
        http("/failsafe/reset")
        http("/failsafe/procedure/0")
        http("/failsafe/loss")
        st = wait_state(lambda s: s["phase_en"] == "LANDING", timeout=8.0)
        check(st["phase_en"] == "LANDING", "начался «Этап 2»: посадка (LANDING)")
        wait_state(lambda s: s["phase_en"] != "LANDING" or s["motors"][0] < 1350, timeout=3.0)
        middle = api()["motors"][0]
        check(1000 < middle < 1400, f"газ посередине посадки убран не до конца: {middle} (off_delay 1.0 с)")
        st = wait_state(lambda s: s["phase_en"] == "RX_LOSS_MONITORING", timeout=6.0)
        check(st["motors"][:4] == [1000, 1000, 1000, 1000], "после off_delay моторы в 1000")
        log = " | ".join(text for _ms, text in st["log"])
        check("Этап 2: посадка, газ убирается за 1.0 с" in log, "журнал: посадка с off_delay 1.0 с")
        http("/failsafe/procedure/1")
        check(api()["procedure_name"] == "DROP", "кнопка процедуры вернула DROP")

        print("7. JustDisarm: борт взведён на земле (газ ниже min_check)")
        http("/failsafe/reset")
        http("/failsafe/arm_ground")
        http("/failsafe/loss")
        st = wait_state(lambda s: s["phase_en"] == "RX_LOSS_MONITORING", timeout=6.0)
        log = " | ".join(text for _ms, text in st["log"])
        check("JustDisarm" in log, "журнал: JustDisarm — моторы выключены без процедуры (failsafe.c)")
        check(st["motors"][:4] == [1000, 1000, 1000, 1000], "моторы выключены сразу после страж-времени")
        http("/failsafe/arm_air")
        check(api()["throttle"] == 1500, "кнопка «Взлететь» поднимает газ до 1500 мкс (борт в воздухе)")

        print("8. Тумблер FAILSAFE (вкладка «Режимы»): STAGE1 / STAGE2 / KILL")
        http("/failsafe/reset")
        http("/failsafe/box_on")
        time.sleep(0.6)
        check(api()["phase_en"] == "IDLE" and api()["box_failsafe"] is True,
              "STAGE1: тумблер включён — пока идёт страж-время, откат возможен")
        st = wait_state(lambda s: s["phase_en"] == "RX_LOSS_MONITORING", timeout=6.0)
        check(st["phase_en"] == "RX_LOSS_MONITORING", "STAGE1: по истечении страж-времени начался «Этап 2»")
        http("/failsafe/box_off")
        st = wait_state(lambda s: s["phase_en"] == "IDLE", timeout=4.0)
        check(st["phase_en"] == "IDLE", "тумблерный failsafe выходит сразу (receivingRxDataPeriodPreset = 0)")
        http("/failsafe/arm_air")      # «Этап 2» снял борт с взвода — перед опытом взводим снова
        http("/failsafe/switch/2")
        http("/failsafe/box_on")
        st = wait_state(lambda s: s["phase_en"] == "RX_LOSS_MONITORING", timeout=2.0)
        check(st["phase_en"] == "RX_LOSS_MONITORING", "STAGE2: «Этап 2» наступает сразу, без страж-времени")
        http("/failsafe/box_off")
        http("/failsafe/arm_air")
        http("/failsafe/switch/1")
        http("/failsafe/box_on")
        st = wait_state(lambda s: s["motors"][0] == 1000, timeout=2.0)
        log = " | ".join(text for _ms, text in st["log"])
        check(st["motors"][0] == 1000 and "KILL" in log, "KILL: моторы выключены сразу")
        http("/failsafe/box_off")
        http("/failsafe/switch/0")
        check(api()["switch_mode"] == 0, "режим тумблера вернулся в STAGE1")
        http("/failsafe/reset")

        print("9. GPS-спасение: фазы gps_rescue.c, возврат домой и посадка")
        http("/failsafe/procedure/2")
        http("/failsafe/scenario/none")
        http("/failsafe/loss")
        http("/failsafe/scale/1")
        st = wait_state(lambda s: s["rescue"]["phase"] not in (0, 1), timeout=10.0)
        phases, start_dist, prev = [], st["rescue"]["dist_m"], st["rescue"]["dist_m"]
        for _ in range(600):
            st = api()
            phase = st["rescue"]["phase_name"]
            if not phases or phases[-1] != phase:
                phases.append(phase)
            prev = st["rescue"]["dist_m"]
            if phase in ("COMPLETE", "ABORT", "DO_NOTHING"):
                break
            time.sleep(0.25)
        check(st["rescue"]["phase_name"] == "COMPLETE", f"спасение дошло до COMPLETE (фазы: {' → '.join(phases)})")
        canonical = ["INITIALIZE", "ATTAIN_ALT", "ROTATE", "FLY_HOME", "DESCENT", "LANDING", "COMPLETE"]
        order_ok = [ph for ph in phases if ph in canonical] == sorted(
            [ph for ph in phases if ph in canonical], key=canonical.index)
        check(order_ok and {"ROTATE", "FLY_HOME", "DESCENT", "COMPLETE"} <= set(phases),
              f"порядок фаз как в gps_rescue.c ({' → '.join(canonical[1:])}): {' → '.join(phases)}")
        check(prev < start_dist, f"борт шёл домой: расстояние {start_dist} м → {prev} м")
        check(st["motors"][:4] == [1000, 1000, 1000, 1000] and st["armed"] is False,
              "после посадки моторы выключены, борт снят с взвода")
        log = " | ".join(text for _ms, text in st["log"])
        check("высота возврата" in log and "разворот домой" in log and "посадка" in log,
              "журнал: высота возврата, разворот домой, посадка")

        print("10. Сценарии ошибок GPS-спасения")
        http("/failsafe/reset")
        http("/failsafe/procedure/2")
        http("/failsafe/scenario/low_sats")     # sanity = FS_ONLY, связь потеряна: ABORT
        http("/failsafe/loss")
        st = wait_state(lambda s: s["rescue"]["phase_name"] in ("ABORT", "DO_NOTHING"), timeout=8.0)
        aborted = st["rescue"]["phase_name"] == "ABORT"
        st = wait_state(lambda s: s["motors"][0] <= 1010, timeout=4.0)
        check(aborted and st["motors"][0] == 1000 and st["armed"] is False,
              "мало спутников + sanity FS_ONLY при потерянной связи: ABORT, моторы выключены "
              "(disarm как в gps_rescue.c)")
        check("мало спутников" in st["rescue"]["failure_text"],
              f"проверка названа как в gps_rescue.c: {st['rescue']['failure_text']}")
        http("/failsafe/reset")
        http("/failsafe/scenario/hover_low")
        http("/failsafe/loss")
        st = wait_state(lambda s: s["rescue"]["phase_name"] in ("LANDING", "COMPLETE"), timeout=25.0)
        log = " | ".join(text for _ms, text in st["log"])
        check(st["rescue"]["phase_name"] in ("LANDING", "COMPLETE") and "не меняется как задумано 10 с" in log,
              "газа зависания не хватает: 10 с без набора высоты → посадка на месте (gps_rescue.c)")
        http("/failsafe/scenario/none")
        http("/failsafe/procedure/1")
        http("/failsafe/reset")

        print("11. Параметры GPS-спасения из вкладки Failsafe (MSP_GPS_RESCUE, 135)")
        data = fc.call(135)
        check(len(data) == 26, f"MSP_GPS_RESCUE: {len(data)} байт (как в msp.c 4.5.4)")
        min_sats = data[15]
        check(min_sats == 8, f"gps_rescue_min_sats = {min_sats} (заводское)")
        sanity = data[14]
        check(sanity == 2, f"gps_rescue_sanity_checks = {sanity} (FS_ONLY, заводское)")
        hover = int.from_bytes(data[12:14], "little")
        check(hover == 1275, f"gps_rescue_throttle_hover = {hover} мкс (заводское)")
        st = api()
        check(len(st["rescue_param_rows"]) >= 12,
              f"на странице {len(st['rescue_param_rows'])} параметров спасения с пояснениями")
        check(all(row[2] for row in st["param_rows"]), "у каждого параметра Failsafe есть «что делает сейчас»")

        print("12. CLI `sim_failsafe status` и кнопки страницы")
        http("/failsafe/loss")
        time.sleep(0.6)
        out = fc.cli("sim_failsafe")
        check("rollback point: POSSIBLE" in out and "stage 1" in out.lower(),
              "CLI: во время страж-времени откат отмечен как возможный")
        http("/failsafe/scale/1")
        wait_state(lambda s: s["phase_en"] == "RX_LOSS_MONITORING", timeout=8.0)
        out = fc.cli("sim_failsafe status")
        check("IMPOSSIBLE" in out.upper() and "stage 2" in out.lower(),
              "CLI: после точки отката — IMPOSSIBLE, идёт stage 2")
        check("motors: 1000 1000 1000 1000" in out, "CLI: моторы в 1000 (как в Configurator на вкладке «Моторы»)")
        st = json.loads(http("/api/state")[1])
        current = st["power"]["current"]
        for _ in range(40):                      # выбег винта: обороты гаснут за ~0.5 с
            if current < 2.0:
                break
            time.sleep(0.1)
            current = json.loads(http("/api/state")[1])["power"]["current"]
        check(st["failsafe"]["phase_en"] == "RX_LOSS_MONITORING" and current < 2.0,
              f"/api/state: карточка failsafe и ток без моторов ({current:.2f} А, моторы в 1000)")
        http("/failsafe/reset")
        check(api()["motors"][0] > 1100, "«Сброс демо»: моторы снова висения (демо показывает полёт)")

        print()
        if FAILS:
            print(f"ДЕМО FAILSAFE: {len(FAILS)} провалов:")
            for item in FAILS:
                print(f"   - {item}")
            return 1
        print("ДЕМО FAILSAFE РАБОТАЕТ: страж-время, «Этап 1», точка отката, «Этап 2», "
              "GPS-спасение, тумблер, JustDisarm, страница и CLI — 0 провалов")
        return 0
    finally:
        fc.close()
        proc.terminate()
        proc.wait(timeout=10)


if __name__ == "__main__":
    sys.exit(main())

"""Батарея, питание и виртуальный полёт. Методы VirtualFC.

Примесь использует общее состояние контроллера; создание объекта — через bfemu.fc.VirtualFC.
"""
from __future__ import annotations

import math
import time
from .fc_constants import (
    BATTERY_CRITICAL,
    BATTERY_NOT_PRESENT,
    BATTERY_OK,
    BATTERY_PROFILE_COUNT,
    BATTERY_WARNING,
    CURRENT_METER_CONFIG_SUBFRAME,
    CURRENT_METER_ID_BATTERY_1,
    CURRENT_METER_ID_VIRTUAL_1,
    CURRENT_SENSOR_ADC,
    CURRENT_SENSOR_ADC_COUNT,
    CURRENT_SENSOR_VIRTUAL,
    MAX_AUTO_DETECT_CELL_COUNT,
    MAX_BATTERY_PROFILE_NAME_LENGTH,
    MAX_SUPPORTED_MOTORS,
    VBAT_PROFILE_DEFAULTS,
    VOLTAGE_METER_CONFIG_SUBFRAME,
    VOLTAGE_METER_ID_BATTERY_1,
    VOLTAGE_SENSOR_ADC_COUNT,
    VOLTAGE_SENSOR_ADC_RESISTOR_DIVIDER,
)


class PowerMixin:

    # ------------------------------------------------- «виртуальный полёт» (живое питание)
    # На столе вкладка «Питание и батарея» мертва: моторы стоят, ток нулевой, счётчик
    # мА·ч не растёт. Чтобы состояние питания было живым, есть «виртуальный полёт»:
    # кнопка на веб-мониторе (/sim_flight/on) или команда CLI `sim_flight on`.
    # В полёте эмулятор ведёт себя как летящий борт: газ по сценарию, ток 3…27 А,
    # напряжение проседает под нагрузкой и постепенно падает по мере разряда,
    # счётчик мА·ч растёт. Время в полёте идёт быстрее стенных часов (×10), чтобы
    # цифры на вкладке были видны глазом, а не менялись раз в минуту.
    SIM_FLIGHT_TIME_SCALE = 10.0      # во сколько раз быстрее «летит» время (можно менять)
    SIM_IDLE_CURRENT_A = 0.55         # электроника борта: FC, приёмник, VTX, LED — покой
    SIM_IDLE_RIPPLE_A = 0.06          # «дыхание» тока покоя: 0.49…0.61 А, как у живого датчика
    SIM_HOVER_GAS = 0.32              # газ висения → ≈6 А на четырёх моторах
    SIM_PUNCH_GAS = 0.42              # надбавка «газ в пол» → ≈27 А
    SIM_TAKEOFF_S = 3.0               # с — раскрутка на взлёте
    SIM_PUNCH_PERIOD_S = 25.0         # с — как часто «даём газу»
    SIM_PUNCH_S = 3.0                 # с — длительность «газа в пол»
    SIM_LAND_S = 2.5                  # с — посадка при разряженной батарее
    SIM_SWAP_S = 3.0                  # с — пауза «замена батареи»
    SIM_PACK_FULL_V = 4.20            # В на банку у заряженной батареи
    SIM_PACK_MIN_V = 3.50             # В на банку в конце полёта (уровень предупреждения)

    @staticmethod
    def _default_battery_profiles() -> list:
        """batteryProfiles[] — pgResetFn_batteryProfiles(): три профиля с одинаковыми порогами.

        Профили батареи появились после 4.5.4 (мастер BF): каждый хранит свои пороги ячеек,
        ёмкость, принудительное число банок, процент предупреждения по расходу и короткое
        имя (до 8 символов).
        """
        return [dict(VBAT_PROFILE_DEFAULTS, name="") for _ in range(BATTERY_PROFILE_COUNT)]

    def _battery_profiles_from_json(self, data) -> list:
        """Профили батареи из «флеша»: битое отбрасываем, недостающее добираем заводским."""
        profiles = self._default_battery_profiles()
        if not isinstance(data, list):
            return profiles
        for index, item in enumerate(data[:BATTERY_PROFILE_COUNT]):
            if not isinstance(item, dict):
                continue
            profile = dict(profiles[index])
            for field in ("vbatmaxcellvoltage", "vbatmincellvoltage", "vbatwarningcellvoltage",
                          "vbatfullcellvoltage"):
                profile[field] = self._clamp_int(item.get(field), 100, 500, profile[field])
            profile["batteryCapacity"] = self._clamp_int(item.get("batteryCapacity"), 0, 20000,
                                                         profile["batteryCapacity"])
            profile["forceBatteryCellCount"] = self._clamp_int(item.get("forceBatteryCellCount"), 0, 24,
                                                               profile["forceBatteryCellCount"])
            profile["consumptionWarningPercentage"] = self._clamp_int(
                item.get("consumptionWarningPercentage"), 0, 100,
                profile["consumptionWarningPercentage"])
            name = item.get("name")
            if isinstance(name, str):
                profile["name"] = name[:MAX_BATTERY_PROFILE_NAME_LENGTH]
            profiles[index] = profile
        return profiles

    # --- «флеш» для команд flash_read/flash_write (USE_FLASH_TOOLS)
    def flash_read_bytes(self, address: int, length: int) -> bytes:
        """flashfsReadAbs(): содержимое «флеша» эмулятора плюс то, что записал flash_write.

        «Флеш» здесь — накопитель чёрного ящика (store.data): адреса за его пределами
        прошивка тоже читает нулями, поэтому отдаём столько байт, сколько поместилось.
        """
        if address < 0 or length <= 0:
            return b""
        data = self.blackbox.data
        size = min(len(data), address + length)
        out = bytearray(data[address:size]) if size > address else bytearray()
        for start, chunk in self.flash_overlay.items():
            end = start + len(chunk)
            if end <= address or start >= address + length:
                continue
            need = min(address + length, end) - address
            if len(out) < need:
                out.extend(b"\x00" * (need - len(out)))     # записанное читается и на пустом накопителе
            for offset in range(max(start, address), min(end, address + length)):
                out[offset - address] = chunk[offset - start]
        return bytes(out)

    def flash_write_bytes(self, address: int, text: bytes):
        """flashfsWrite(): запись в память эмулятора — файл пролётов остаётся целым."""
        if address < 0:
            return
        for start in [start for start in self.flash_overlay if start < address < start + len(self.flash_overlay[start])]:
            # запись поверх уже записанного куска: хвост оставляем как есть
            prefix = self.flash_overlay[start][:address - start]
            rest = self.flash_overlay[start][address - start + len(text):]
            del self.flash_overlay[start]
            if prefix:
                self.flash_overlay[start] = prefix
            if rest:
                self.flash_overlay[address + len(text)] = rest
        self.flash_overlay[address] = bytes(text)

    def _battery_setting(self, name, lo=None, hi=None):
        """Значение настройки батареи с ограничением, как у VAR_UINT16 в прошивке."""
        entry = self.settings_by_name.get(name)
        default = entry["default"] if entry else 0
        value = int(self.settings.get(name, default) or 0)
        if lo is None or hi is None:
            return value
        return max(lo, min(hi, value))

    def _battery_meter_source(self):
        """voltageMeterSource: 0 NONE, 1 ADC, 2 ESC (battery_meter)."""
        return int(self.settings.get("battery_meter", 0) or 0)

    def _current_meter_source(self):
        """currentMeterSource: 0 NONE, 1 ADC, 2 VIRTUAL, 3 ESC, 4 MSP (current_meter)."""
        return int(self.settings.get("current_meter", 0) or 0)

    def _pack_rest_voltage(self) -> float:
        """Напряжение «покоя» батареи — с учётом разряда.

        Заряженная батарея — 4.20 В на банку; по мере расхода (mah_drawn против
        ёмкости bat_capacity) напряжение падает до SIM_PACK_MIN_V (3.50 В — уровень
        предупреждения, на нём эмулятор садится и «меняет батарею»). Ёмкость 0 —
        «ёмкость неизвестна», тогда напряжение не падает.
        """
        if self._battery_meter_source() == 0:
            return 0.0
        per_cell = self.SIM_PACK_FULL_V
        capacity = self._battery_setting("bat_capacity", 0, 20000)
        if capacity > 0:
            spent = min(1.0, self.mah_drawn / capacity)
            per_cell = self.SIM_PACK_FULL_V - (self.SIM_PACK_FULL_V - self.SIM_PACK_MIN_V) * spent
        return self.battery_cells * per_cell

    def _battery_voltage(self) -> float:
        """Напряжение батареи: 0, пока не выбран датчик напряжения (как на столе без батареи).

        Под нагрузкой напряжение проседает — считаем это здесь, в одном месте
        (rest - I · R), поэтому вкладки «Моторы», «Питание и батарея» и OSD показывают
        одну и ту же просадку. Напряжение «покоя» берётся у _pack_rest_voltage():
        разряженная батарея проседает сильнее, чем заряженная.
        """
        if self._battery_meter_source() == 0:
            return 0.0
        # Сначала добираем расход: «покой» зависит от счётчика мА·ч, и если посчитать
        # напряжение по прошлому значению, напряжение и расход в одном ответе MSP
        # разойдутся на один опрос (заметно при редком опросе вкладки).
        self._mah_update()
        rest = self._pack_rest_voltage()
        current = self._battery_current()
        return max(0.0, rest - current * self.MOTOR_PACK_RESISTANCE_OHM)

    def _battery_current(self) -> float:
        """Ток батареи: ток моторов + ток электроники борта.

        В покое — не ноль: FC, приёмник, VTX и LED потребляют около 0.55 А, и на вкладке
        «Питание и батарея» это видно («Ток потребления» 0.49…0.61 А). Ток чуть «дышит»,
        как настоящий датчик. Датчик тока не выбран — 0.
        """
        if self._current_meter_source() == 0:
            return 0.0
        idle = self.SIM_IDLE_CURRENT_A + self.SIM_IDLE_RIPPLE_A * math.sin(time.time() * 0.7)
        return round(max(0.0, idle) + sum(self.motor_current_a), 2)

    def _sim_scaled_dt(self, dt: float) -> float:
        """Время «виртуального полёта» в стенных секундах: ×SIM_FLIGHT_TIME_SCALE.

        Ускорение нужно только счётчикам расхода: напряжение, обороты и нагрев
        считаются по настоящему времени, поэтому картинка на вкладках остаётся
        физичной, а «Израсходовано mAh» бежит заметно для глаза.
        """
        return dt * self.sim_time_scale

    def _mah_update(self):
        """Интегрируем ток в мА·ч (getMAhDrawn в прошивке).

        Расход растёт и в покое (электроника), и в полёте; время масштабируется
        множителем _sim_scaled_dt().
        """
        now = time.time()
        if self._mah_t is None:
            self._mah_t = now
            return
        dt = max(0.0, now - self._mah_t)
        self._mah_t = now
        current = self._battery_current()
        if current > 0:
            self.mah_drawn += current * self._sim_scaled_dt(dt) / 3.6   # А·с -> мА·ч

    def _mah_drawn(self) -> int:
        self._mah_update()
        return int(self.mah_drawn)

    def _battery_filtered_voltage(self) -> int:
        """«Отфильтрованное» напряжение в 0.01 В — как voltageMeter.displayFiltered.

        Оно всегда на шаг ниже «живого»: фильтр в прошивке никогда не выходит точно
        на измеренное значение. Без этого ровно 16.80 В при vbat_max_cell_voltage = 420
        дало бы 5 банок вместо 4 (в batteryUpdatePresence() деление целочисленное).
        """
        millivolts = int(round(self._battery_voltage() * 100))
        return max(0, millivolts - 1)

    def _battery_cell_count(self) -> int:
        """getBatteryCellCount(): 0 — батарея не обнаружена, иначе число банок."""
        if self._battery_meter_source() == 0:
            return 0
        forced = int(self.settings.get("force_battery_cell_count", 0) or 0)
        if forced:
            return forced
        filtered = self._battery_filtered_voltage()
        not_present = self._battery_setting("vbat_detect_cell_voltage")
        per_cell_max = self._battery_setting("vbat_max_cell_voltage")
        # isVoltageFromBattery(): выше «порога отсутствия» и не выше напряжения 1S,
        # либо заведомо выше двойного порога (напряжение USB-питания).
        from_battery = (not_present <= filtered <= per_cell_max) or filtered > not_present * 2
        if not from_battery:
            return 0
        cells = filtered // per_cell_max + 1             # как в batteryUpdatePresence()
        return min(cells, MAX_AUTO_DETECT_CELL_COUNT)

    def _battery_state_code(self) -> int:
        """batteryState_e: 0 OK, 1 предупреждение, 2 критично, 3 батарея не подключена."""
        cells = self._battery_cell_count()
        if cells == 0:
            return BATTERY_NOT_PRESENT
        voltage = self._battery_voltage()
        if voltage < cells * self._battery_setting("vbat_min_cell_voltage") / 100:
            return BATTERY_CRITICAL
        if voltage < cells * self._battery_setting("vbat_warning_cell_voltage") / 100:
            return BATTERY_WARNING
        return BATTERY_OK

    # ------------------------------------------------------------------ виртуальный полёт
    def set_sim_flight(self, on: bool) -> None:
        """Включить/выключить «виртуальный полёт» (кнопка монитора, команда sim_flight).

        Выключение — как «стоп» на вкладке «Моторы»: выходы падают на 1000, моторы
        выбегают, ток возвращается к току электроники.
        """
        on = bool(on)
        if on and not self.sim_flight:
            self.sim_flight = True
            self.sim_flight_phase = "flight"
            self.sim_flight_since = time.time()
            self.sim_flight_note = "Полёт начат: газ по сценарию, расход мА·ч пошёл"
            self._log("[SIM] виртуальный полёт включён (газ по сценарию, расход мА·ч учитывается)")
        elif not on and self.sim_flight:
            self.sim_flight = False
            self.sim_flight_phase = "idle"
            self.motor_disarmed = [1000.0] * MAX_SUPPORTED_MOTORS
            self.sim_flight_note = "Полёт остановлен: моторы на 1000, ток — только электроника"
            self._log("[SIM] виртуальный полёт выключен (моторы на 1000)")

    def sim_flight_state(self) -> dict:
        """Снимок «виртуального полёта» для монитора, CLI и /api/state."""
        self._motors_step()
        self._mah_update()
        phase_names = {
            "idle": "выключен",
            "flight": "полёт",
            "landing": "посадка",
            "swap": "замена батареи",
        }
        return {
            "on": self.sim_flight,
            "phase": self.sim_flight_phase,
            "phase_text": phase_names.get(self.sim_flight_phase, self.sim_flight_phase),
            "note": self.sim_flight_note,
            "time_scale": self.sim_time_scale,
            "packs": self.sim_pack_events,
            "seconds": round(time.time() - self.sim_flight_since, 1) if self.sim_flight else 0.0,
        }

    def power_state(self) -> dict:
        """Состояние питания для веб-монитора: то же, что читает вкладка «Питание и батарея»."""
        self._motors_step()
        self._mah_update()
        voltage = self._battery_voltage()
        return {
            "cells": self._battery_cell_count(),
            "voltage": round(voltage, 2),
            "rest_voltage": round(self._pack_rest_voltage(), 2),
            "current": self._battery_current(),
            "mah": int(self.mah_drawn),
            "capacity": self._battery_setting("bat_capacity", 0, 20000),
            "state": self._battery_state_code(),
            "state_name": ("OK", "ПРЕДУПРЕЖДЕНИЕ", "КРИТИЧНО", "БАТАРЕЯ НЕ ПОДКЛЮЧЕНА")[
                self._battery_state_code()],
            "motors": [int(round(v)) for v in self.motor_output[:self._motor_count()]],
            "sim": self.sim_flight_state(),
        }

    def _sim_flight_outputs(self, now: float) -> list:
        """Выходы моторов (1000…2000) по сценарию «виртуального полёта».

        Сценарий: раскрутка на взлёте, висение с подруливанием и «газ в пол» каждые
        SIM_PUNCH_PERIOD_S секунд; у каждого мотора своя мелочь, поэтому токи ESC
        не одинаковые. При посадке газ линейно убирается до нуля.
        """
        t = now - self.sim_flight_since
        if self.sim_flight_phase == "landing":
            gas = self.SIM_HOVER_GAS * max(0.0, 1.0 - t / self.SIM_LAND_S)
        elif self.sim_flight_phase == "swap":
            gas = 0.0
        elif t < self.SIM_TAKEOFF_S:
            gas = self.SIM_HOVER_GAS * (t / self.SIM_TAKEOFF_S)          # взлёт
        else:
            gas = self.SIM_HOVER_GAS \
                + 0.05 * math.sin(t * 2 * math.pi / 9.0) \
                + 0.02 * math.sin(t * 2 * math.pi / 2.7)                  # висение с подруливанием
            punch = t % self.SIM_PUNCH_PERIOD_S
            if punch < self.SIM_PUNCH_S:                                  # «газ в пол»
                gas += self.SIM_PUNCH_GAS * (1.0 - punch / self.SIM_PUNCH_S)
        gas = max(0.0, min(1.0, gas))
        count = self._motor_count()                    # QUADX — четыре мотора, остальные стоят
        outputs = []
        for i in range(MAX_SUPPORTED_MOTORS):
            if i >= count:
                outputs.append(1000.0)
                continue
            wobble = 0.012 * math.sin(now * (9.0 + i) + i * 1.7)          # моторы неидеальны
            outputs.append(1000.0 + 1000.0 * max(0.0, min(1.0, gas + wobble)))
        return outputs

    def _sim_flight_step(self, now: float) -> None:
        """Шаг сценария: фазы полёта, посадка при разряде, «замена батареи».

        Разрядилась (расход дошёл до ёмкости) — садимся, ждём SIM_SWAP_S и ставим
        новую батарею: счётчик мА·ч обнуляется, напряжение снова 4.20 В на банку.
        Так вкладка «Питание и батарея» проходит весь цикл: OK → предупреждение →
        посадка → новая батарея.
        """
        if not self.sim_flight:
            return
        phase = self.sim_flight_phase
        if phase not in ("flight", "landing", "swap"):
            phase = self.sim_flight_phase = "flight"
            self.sim_flight_since = now
        t = now - self.sim_flight_since
        capacity = self._battery_setting("bat_capacity", 0, 20000)
        if phase == "flight" and capacity > 0 and self.mah_drawn >= capacity:
            self.sim_flight_phase = "landing"
            self.sim_flight_since = now
            self.sim_flight_note = ("Батарея разряжена (расход дошёл до ёмкости) — посадка, "
                                    "затем новая батарея")
            self._log("[SIM] батарея разряжена: посадка")
        elif phase == "landing" and t >= self.SIM_LAND_S:
            self.sim_flight_phase = "swap"
            self.sim_flight_since = now
        elif phase == "swap" and t >= self.SIM_SWAP_S:
            self.mah_drawn = 0.0
            self.motor_mah = [0.0] * MAX_SUPPORTED_MOTORS
            self.sim_pack_events += 1
            self.sim_flight_phase = "flight"
            self.sim_flight_since = now
            self.sim_flight_note = (f"Поставлена новая батарея (#{self.sim_pack_events}): "
                                    f"{self.battery_cells * self.SIM_PACK_FULL_V:.2f} В, 0 мА·ч")
            self._log(f"[SIM] новая батарея #{self.sim_pack_events}, расход сброшен")

    def _h_analog(self, w, r):
        """MSP_ANALOG (110) — устаревшая телеметрия в 0.1 В: то же, что MSP_BATTERY_STATE."""
        self._motors_step()   # просадка напряжения считается по току моторов
        voltage = self._battery_voltage()
        w.u8(int(voltage * 10) & 0xFF)
        w.u16(self._mah_drawn())
        w.u16(1023)                                        # RSSI 100 %
        w.i16(int(round(self._battery_current() * 100)))
        w.u16(int(round(voltage * 100)))

    def _h_battery_state(self, w, r):
        """MSP_BATTERY_STATE (130): 11 байт — банки, ёмкость, напряжение, мА·ч, ток, статус."""
        self._motors_step()   # просадка напряжения считается по току моторов
        voltage = self._battery_voltage()
        self._mah_update()
        w.u8(self._battery_cell_count())
        w.u16(self._battery_setting("bat_capacity", 0, 20000))
        w.u8(int(voltage * 10) & 0xFF)
        w.u16(int(self.mah_drawn))
        w.i16(int(round(self._battery_current() * 100)))
        w.u8(self._battery_state_code())
        w.u16(int(round(voltage * 100)))

    def _h_voltage_meters(self, w, r):
        """MSP_VOLTAGE_METERS (128): id метра и напряжение (после фильтра, в 0.1 В)."""
        voltage = self._battery_voltage()
        w.u8(VOLTAGE_METER_ID_BATTERY_1)
        w.u8(int(round(voltage * 10)))

    def _h_current_meters(self, w, r):
        """MSP_CURRENT_METERS (129): для каждого метра id, мА·ч и ток в мА (0.001 А)."""
        self._mah_update()
        milliamps = int(round(self._battery_current() * 1000))
        drawn = int(self.mah_drawn)
        w.u8(CURRENT_METER_ID_BATTERY_1).u16(drawn).u16(milliamps)
        if CURRENT_SENSOR_ADC_COUNT > 1:
            w.u8(CURRENT_METER_ID_VIRTUAL_1).u16(drawn).u16(milliamps)

    def _h_battery_config(self, w, r):
        """MSP_BATTERY_CONFIG (32): 13 байт, как в msp.c 4.5.4.

        Сначала три «округлённых до 0.1 В» байта (для старых GUI), затем ёмкость,
        источники датчиков и, в конце, точные напряжения в 0.01 В — Configurator
        читает их последними и они перекрывают округлённые.
        """
        w.u8((self._battery_setting("vbat_min_cell_voltage") + 5) // 10)
        w.u8((self._battery_setting("vbat_max_cell_voltage") + 5) // 10)
        w.u8((self._battery_setting("vbat_warning_cell_voltage") + 5) // 10)
        w.u16(self._battery_setting("bat_capacity", 0, 20000))
        w.u8(int(self.settings.get("battery_meter", 0)))
        w.u8(int(self.settings.get("current_meter", 0)))
        w.u16(self._battery_setting("vbat_min_cell_voltage"))
        w.u16(self._battery_setting("vbat_max_cell_voltage"))
        w.u16(self._battery_setting("vbat_warning_cell_voltage"))

    def _h_set_battery_config(self, w, r):
        """MSP_SET_BATTERY_CONFIG (33): 13 байт — три u8 (округлённые), ёмкость u16,
        два источника датчиков и точные напряжения u16 в 0.01 В (как в msp.c 4.5.4)."""
        data = bytes(r.rest())
        if len(data) < 13:
            return
        self._store_setting("bat_capacity", int.from_bytes(data[3:5], "little"))
        self._store_setting("battery_meter", data[5])
        self._store_setting("current_meter", data[6])
        self._store_setting("vbat_min_cell_voltage", int.from_bytes(data[7:9], "little"))
        self._store_setting("vbat_max_cell_voltage", int.from_bytes(data[9:11], "little"))
        self._store_setting("vbat_warning_cell_voltage", int.from_bytes(data[11:13], "little"))
        self._log(
            f"[MSP] BATTERY_CONFIG сохранён: {self.settings['vbat_min_cell_voltage']}/"
            f"{self.settings['vbat_max_cell_voltage']}/{self.settings['vbat_warning_cell_voltage']} (0.01 В), "
            f"ёмкость {self.settings['bat_capacity']} мАч, источники {self.settings['battery_meter']}/"
            f"{self.settings['current_meter']}"
        )
        return b""   # пустой ACK: вкладка «Питание и батарея» ждёт ответа на запись

    def _h_set_voltage_meter_config(self, w, r):
        """MSP_SET_VOLTAGE_METER_CONFIG (57): id, vbatscale, vbatresdivval, vbatresdivmultiplier."""
        data = bytes(r.rest())
        if len(data) < 4:
            return
        if data[0] != VOLTAGE_METER_ID_BATTERY_1:
            return
        self._store_setting("vbat_scale", max(0, min(255, data[1])))
        self._store_setting("vbat_divider", max(1, min(255, data[2])))
        self._store_setting("vbat_multiplier", max(1, min(255, data[3])))
        self._log(
            f"[MSP] VOLTAGE_METER_CONFIG: vbat_scale={self.settings['vbat_scale']}, "
            f"divider={self.settings['vbat_divider']}, multiplier={self.settings['vbat_multiplier']}"
        )
        return b""

    def _h_set_current_meter_config(self, w, r):
        """MSP_SET_CURRENT_METER_CONFIG (41): id, scale u16, offset u16 (offset — знаковый)."""
        data = bytes(r.rest())
        if len(data) < 5:
            return
        scale = int.from_bytes(data[1:3], "little", signed=True)
        offset = int.from_bytes(data[3:5], "little", signed=True)
        if data[0] == CURRENT_METER_ID_BATTERY_1:
            self._store_setting("ibata_scale", scale)
            self._store_setting("ibata_offset", offset)
        elif data[0] == CURRENT_METER_ID_VIRTUAL_1:
            self._store_setting("ibatv_scale", scale)
            self._store_setting("ibatv_offset", max(0, offset))
        else:
            return
        self._log(
            f"[MSP] CURRENT_METER_CONFIG (id {data[0]}): scale={scale}, offset={offset}"
        )
        return b""

    def _h_voltage_meter_config(self, w, r):
        """MSP_VOLTAGE_METER_CONFIG (56): count + подкадр [len=5, id, тип, scale, divider, multiplier].

        Раньше длина подкадра была 0: Configurator пропускал такой подкадр и оставлял
        FC.VOLTAGE_METER_CONFIGS пустым — вкладка не показывала калибровку, а «Сохранить»
        вообще ничего не отправлял по напряжению.
        """
        w.u8(VOLTAGE_SENSOR_ADC_COUNT)
        w.u8(VOLTAGE_METER_CONFIG_SUBFRAME)   # 5
        w.u8(VOLTAGE_METER_ID_BATTERY_1)      # 10
        w.u8(VOLTAGE_SENSOR_ADC_RESISTOR_DIVIDER)
        w.u8(int(self.settings.get("vbat_scale", 110)))
        w.u8(int(self.settings.get("vbat_divider", 10)))
        w.u8(int(self.settings.get("vbat_multiplier", 1)))

    def _h_current_meter_config(self, w, r):
        """MSP_CURRENT_METER_CONFIG (40): count + подкадр [len=6, id, тип, scale u16, offset u16]."""
        w.u8(CURRENT_SENSOR_ADC_COUNT)
        w.u8(CURRENT_METER_CONFIG_SUBFRAME)   # 6
        w.u8(CURRENT_METER_ID_BATTERY_1)      # 10
        w.u8(CURRENT_SENSOR_ADC)
        w.i16(int(self.settings.get("ibata_scale", 400)))
        w.i16(int(self.settings.get("ibata_offset", 0)))
        w.u8(CURRENT_METER_CONFIG_SUBFRAME)   # виртуальный датчик (индекс 1 = источник «VIRTUAL»)
        w.u8(CURRENT_METER_ID_VIRTUAL_1)      # 80
        w.u8(CURRENT_SENSOR_VIRTUAL)
        w.i16(int(self.settings.get("ibatv_scale", 0)))
        w.i16(int(self.settings.get("ibatv_offset", 0)))

    def _h_battery_profile(self, w, r):
        """MSP2_BATTERY_PROFILE (0x300E): раскладка мастера BF — 1 + 5·2 + 2 = 13 байт.

        Без аргумента отдаётся активный профиль (msp.c: `sbufBytesRemaining(src) ?
        sbufReadU8(src) : systemConfig()->activeBatteryProfile`), индекс вне диапазона —
        ошибка: отвечаем пустым payload, как прошивка на MSP_RESULT_ERROR.
        """
        index = r.u8() if len(r.data) else self.active_battery_profile
        if index >= BATTERY_PROFILE_COUNT:
            return b""
        profile = self.battery_profiles[index]
        w.u8(index)
        w.u16(profile["vbatmincellvoltage"]).u16(profile["vbatmaxcellvoltage"])
        w.u16(profile["vbatwarningcellvoltage"]).u16(profile["vbatfullcellvoltage"])
        w.u16(profile["batteryCapacity"])
        w.u8(profile["forceBatteryCellCount"]).u8(profile["consumptionWarningPercentage"])

    def _h_set_battery_profile(self, w, r):
        """MSP2_SET_BATTERY_PROFILE (0x300F): запись профиля — проверки как в msp.c мастера.

        Прошивка требует минимум 13 байт, проверяет порядок порогов (min ≤ warning ≤ full ≤
        max), число банок ≤ 24 и процент ≤ 100 — иначе MSP_RESULT_ERROR (у эмулятора —
        пустой ответ без изменения таблицы).
        """
        data = bytes(r.rest())
        if len(data) < 13:
            return b""
        index = data[0]
        if index >= BATTERY_PROFILE_COUNT:
            return b""
        values = [int.from_bytes(data[1 + 2 * i:3 + 2 * i], "little") for i in range(5)]
        cells, percent = data[11], data[12]
        vbat_min, vbat_max, vbat_warn, vbat_full, capacity = values
        if not (vbat_min <= vbat_warn <= vbat_full <= vbat_max):
            return b""
        if cells > 24 or percent > 100:
            return b""
        self.battery_profiles[index] = {
            "vbatmincellvoltage": vbat_min, "vbatmaxcellvoltage": vbat_max,
            "vbatwarningcellvoltage": vbat_warn, "vbatfullcellvoltage": vbat_full,
            "batteryCapacity": capacity, "forceBatteryCellCount": cells,
            "consumptionWarningPercentage": percent,
            "name": self.battery_profiles[index].get("name", ""),
        }
        self._log(f"[MSP] профиль батареи {index + 1}: {vbat_min}…{vbat_max} мВ/ячейка, {capacity} мА·ч")
        return b""

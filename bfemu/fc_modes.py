"""Режимы, диапазоны и корректировки. Методы VirtualFC.

Примесь использует общее состояние контроллера; создание объекта — через bfemu.fc.VirtualFC.
"""
from __future__ import annotations

from .fc_constants import (
    ADJUSTMENT_FUNCTION_COUNT,
    ADJUSTMENT_RANGE_COUNT,
    AIRPLANE_MIXERS,
    CHANNEL_RANGE_MAX,
    CHANNEL_RANGE_MIN,
    MAX_AUX_CHANNEL_COUNT,
    MAX_MODE_ACTIVATION_CONDITION_COUNT,
    MODELOGIC_AND,
    MODELOGIC_OR,
    MODE_RANGE_STEP,
    NON_AUX_CHANNEL_COUNT,
    SENSOR_ACC,
    SENSOR_MAG,
)


class ModesMixin:

    # ------------------------------------- «Корректирование в полёте» (adjustments)
    @classmethod
    def _default_adjustment_ranges(cls) -> list:
        """Заводские слоты корректировок — как pgResetFn_adjustmentRangeConfig(): все пустые.

        Слот = «когда AUX-канал в диапазоне → применить функцию к каналу-переключателю».
        Пустой слот: диапазон 900…900 (начало равно концу), функция 0 («без изменений») —
        прошивка такой слот не выполняет, и вкладка считает его выключенным.
        """
        return [{"aux": 0, "start": 0, "end": 0, "function": 0, "switch": 0}
                for _ in range(ADJUSTMENT_RANGE_COUNT)]

    @classmethod
    def _stock_adjustment_ranges(cls) -> list:
        """Слоты «настроенного квада»: несколько рабочих корректировок.

        Пустая таблица — валидное заводское состояние, но вкладка «Корректирование в полёте»
        по умолчанию показывает только включённые слоты, поэтому с пустой таблицей страница
        выглядит пустой. Здесь — обычный набор: тумблеры подстраивают RC Rate и RC Expo,
        ещё два выбирают профиль рейтов и силу режима Horizon.
        """
        ranges = cls._default_adjustment_ranges()
        demo = [
            # шаг = (импульс − 900) / 25 — в таком виде значение и уходит в MSP
            {"aux": 0, "start": 16, "end": 32, "function": 1, "switch": 0},   # AUX1 1300…1700 → RC Rate
            {"aux": 1, "start": 16, "end": 32, "function": 2, "switch": 1},   # AUX2 1300…1700 → RC Expo
            {"aux": 2, "start": 32, "end": 48, "function": 12, "switch": 2},  # AUX3 1700…2100 → профиль рейтов
            {"aux": 3, "start": 32, "end": 48, "function": 24, "switch": 3},  # AUX4 1700…2100 → сила Horizon
        ]
        for slot, values in enumerate(demo):
            ranges[slot] = values
        return ranges

    @classmethod
    def _adjustment_ranges_from_json(cls, data) -> list:
        """Слоты корректировок из «флеша»: битое и лишнее отбрасываем, слоты добираем."""
        ranges = cls._default_adjustment_ranges()
        if not isinstance(data, list):
            return ranges
        for index, item in enumerate(data[:ADJUSTMENT_RANGE_COUNT]):
            if not isinstance(item, dict):
                continue
            slot = dict(ranges[index])
            for key, maximum in (("aux", 15), ("start", 48), ("end", 48),
                                 ("function", ADJUSTMENT_FUNCTION_COUNT), ("switch", 15)):
                try:
                    slot[key] = max(0, min(maximum, int(item[key])))
                except (KeyError, TypeError, ValueError):
                    pass
            ranges[index] = slot
        return ranges

    def adjustment_slots_active(self) -> int:
        """Сколько слотов прошивка считает включёнными (начало диапазона не равно концу)."""
        return sum(1 for slot in self.adjustment_ranges if slot["start"] != slot["end"])

    # ------------------------------------------------------ режимы (вкладка «РЕЖИМЫ»)
    @staticmethod
    def _empty_mode_range(index: int) -> dict:
        """Пустой слот modeActivationConditions[]: ARM, 900/900, логика ИЛИ, без связи."""
        return {"index": index, "box": "ARM", "aux": 0, "start": CHANNEL_RANGE_MIN,
                "end": CHANNEL_RANGE_MIN, "logic": MODELOGIC_OR, "linked": 0}

    def _default_mode_ranges(self) -> list:
        return [self._empty_mode_range(i) for i in range(MAX_MODE_ACTIVATION_CONDITION_COUNT)]

    def _normalize_mode_ranges(self, raw) -> list:
        """Приводим список слотов к 20 записям с полями logic/linked (старый «флеш» их не знал)."""
        slots = self._default_mode_ranges()
        for item in raw or []:
            try:
                index = int(item.get("index"))
            except (AttributeError, TypeError, ValueError):
                continue
            if not 0 <= index < MAX_MODE_ACTIVATION_CONDITION_COUNT:
                continue
            name = item.get("box") or "ARM"
            if name not in {b["name"] for b in self.boxes}:
                name = "ARM"
            slots[index] = {
                "index": index,
                "box": name,
                "aux": max(0, min(MAX_AUX_CHANNEL_COUNT - 1, int(item.get("aux", 0)))),
                "start": max(CHANNEL_RANGE_MIN, min(CHANNEL_RANGE_MAX, int(item.get("start", CHANNEL_RANGE_MIN)))),
                "end": max(CHANNEL_RANGE_MIN, min(CHANNEL_RANGE_MAX, int(item.get("end", CHANNEL_RANGE_MIN)))),
                "logic": MODELOGIC_AND if int(item.get("logic", MODELOGIC_OR)) else MODELOGIC_OR,
                "linked": int(item.get("linked", 0) or 0),
            }
        return slots

    @staticmethod
    def _range_usable(slot: dict) -> bool:
        """IS_RANGE_USABLE(): startStep < endStep — иначе слот считается пустым."""
        return slot["start"] < slot["end"]

    @staticmethod
    def _range_configured(slot: dict) -> bool:
        return bool(slot["linked"]) or slot["start"] < slot["end"]

    @property
    def active_boxes(self):
        """Режимы, которые уходят в MSP_BOXNAMES/MSP_BOXIDS — initActiveBoxIds() из msp_box.c.

        Прошивка собирает этот список на старте: он зависит от фич, датчиков, микшера,
        протокола моторов и того, что вообще собрано в прошивке (USE_*). Именно по нему
        Configurator строит строки вкладки «Режимы», и по нему же packFlightModeFlags()
        нумерует биты маски в MSP_STATUS. Считаем на лету: включили фичу в CLI — список
        сразу поменялся, как после перезагрузки на железной плате.
        """
        return self._compute_active_boxes()

    def _compute_active_boxes(self):
        names = {b["name"] for b in self.boxes}
        feats = self.features
        sensors = self._active_sensor_mask()
        mixer = getattr(self, "mixer_name", "QUADX")
        enabled = set()

        def add(*items):
            enabled.update(n for n in items if n in names)

        add("ARM", "PREARM")
        if "AIRMODE" not in feats:                 # BOX нужен, только если фичи нет
            add("AIR MODE")
        if self._setting("anti_gravity_gain", 0) and "ANTI_GRAVITY" not in feats:
            add("ANTI GRAVITY")
        if sensors & SENSOR_ACC:
            add("ANGLE", "HORIZON", "HEADFREE", "HEADADJ", "FPV ANGLE MIX", "ACRO TRAINER")
            if "INFLIGHT_ACC_CAL" in feats:
                add("CALIB")
        if sensors & SENSOR_MAG:
            add("MAG")
        if "GPS" in feats:
            if "3D" not in feats and mixer not in AIRPLANE_MIXERS:
                add("GPS RESCUE")
            add("GPS BEEP SATELLITE COUNT")
        add("FAILSAFE")
        if mixer in AIRPLANE_MIXERS:
            add("PASSTHRU")
        add("BEEPER", "BEEPER MUTE")
        if "LED_STRIP" in feats:
            add("LEDLOW")
        add("BLACKBOX", "BLACKBOX ERASE")           # USE_BLACKBOX + USE_FLASHFS
        if "3D" in feats:
            add("3D DISABLE / SWITCH")
        if self._setting("motor_pwm_protocol", 7) in (5, 6, 7, 8):   # DShot → FLIP OVER AFTER CRASH
            add("FLIP OVER AFTER CRASH")
        if "SERVO_TILT" in feats:
            add("CAMSTAB")
        add("OSD DISABLE")
        if "TELEMETRY" in feats:
            add("TELEMETRY")
        if mixer == "CUSTOMAIRPLANE":
            add("SERVO1", "SERVO2", "SERVO3")
        add("CAMERA CONTROL 1", "CAMERA CONTROL 2", "CAMERA CONTROL 3")   # USE_RCDEVICE
        add("VTX PIT MODE", "VTX CONTROL DISABLE")                        # USE_VTX_SMARTAUDIO
        add("PARALYZE")
        # USER1..USER4 попадают в список, только если их слушает pinio (у нас pinio не настроен),
        # а PID AUDIO — только в сборках с USE_PID_AUDIO (в этой сборке его нет).
        add("LAUNCH CONTROL")                       # USE_LAUNCH_CONTROL
        if self._setting("msp_override_channels_mask", 0):                # USE_RX_MSP_OVERRIDE
            add("MSP OVERRIDE")
        add("STICK COMMANDS DISABLE", "READY")
        add("LAP TIMER RESET")                      # USE_GPS_LAP_TIMER
        return [b["name"] for b in sorted(self.boxes, key=lambda b: b["permanent_id"])
                if b["name"] in enabled]

    def box_permanent_id(self, name: str) -> int:
        box = next((b for b in self.boxes if b["name"] == name), None)
        return box["permanent_id"] if box else 0

    def set_mode_range(self, index, perm_id, aux, start, end, logic=MODELOGIC_OR, linked=0) -> bool:
        """Запись слота так, как это делает MSP_SET_MODE_RANGE / CLI 'aux'."""
        if not 0 <= index < MAX_MODE_ACTIVATION_CONDITION_COUNT:
            return False
        name = self.box_name_by_permanent_id.get(perm_id)
        if name is None:
            return False
        if not 0 <= aux < MAX_AUX_CHANNEL_COUNT:
            return False
        self.mode_ranges[index] = {
            "index": index,
            "box": name,
            "aux": aux,
            "start": max(CHANNEL_RANGE_MIN, min(CHANNEL_RANGE_MAX, int(start))),
            "end": max(CHANNEL_RANGE_MIN, min(CHANNEL_RANGE_MAX, int(end))),
            "logic": MODELOGIC_AND if int(logic) else MODELOGIC_OR,
            "linked": int(linked or 0),
        }
        return True

    def clear_mode_range(self, index) -> bool:
        if not 0 <= index < MAX_MODE_ACTIVATION_CONDITION_COUNT:
            return False
        self.mode_ranges[index] = self._empty_mode_range(index)
        return True

    def _mode_range_step(self, value: int) -> int:
        return max(0, min((CHANNEL_RANGE_MAX - CHANNEL_RANGE_MIN) // MODE_RANGE_STEP,
                          (int(value) - CHANNEL_RANGE_MIN) // MODE_RANGE_STEP))

    def _is_range_active(self, aux: int, start: int, end: int) -> bool:
        """isRangeActive() из rc_modes.c: канал AUX попадает в [start, end)."""
        if start >= end:
            return False
        channels = self._rc_channel_values()
        index = NON_AUX_CHANNEL_COUNT + int(aux)
        if not 0 <= index < len(channels):
            return False
        value = max(CHANNEL_RANGE_MIN, min(CHANNEL_RANGE_MAX - 1, int(channels[index])))
        return start <= value < end

    def active_mode_names(self) -> set:
        """updateActivatedModes() из rc_modes.c: какие режимы сейчас включены тумблерами.

        Логика И/ИЛИ, связи (linkedTo) и «залипающий» PARALYZE — как в прошивке.
        """
        and_mask, new_mask = set(), set()

        def update(mode: str, active: bool, logic: int) -> None:
            if mode in and_mask or mode not in new_mask:
                if logic != MODELOGIC_AND:            # OR-условие
                    if active:
                        and_mask.discard(mode)
                        new_mask.add(mode)
                else:                                 # AND-условие
                    and_mask.add(mode)
                    if not active:
                        new_mask.add(mode)

        for slot in sorted(self.mode_ranges, key=lambda s: s["index"]):
            if slot["linked"]:
                continue
            if not self._range_configured(slot):
                continue
            update(slot["box"], self._is_range_active(slot["aux"], slot["start"], slot["end"]),
                   slot["logic"])
        for slot in sorted(self.mode_ranges, key=lambda s: s["index"]):
            if not slot["linked"]:
                continue
            linked_name = self.box_name_by_permanent_id.get(slot["linked"], "ARM")
            active = (linked_name in and_mask) != (linked_name in new_mask)
            update(slot["box"], active, slot["logic"])
        return and_mask ^ new_mask

    def rc_mode_mask(self) -> int:
        """Маска режимов для MSP_STATUS (CONFIG.mode).

        Бит — позиция режима в списке MSP_BOXNAMES: именно так его читает Configurator
        (`bit_check(config.mode, mode.index)`, index — порядок в AUX_CONFIG), поэтому
        подсветка «включён / выключен» на вкладке «Режимы» совпадает с тумблерами.
        """
        names = self.active_mode_names()
        mask = 0
        for position, name in enumerate(self.active_boxes):
            if name in names:
                mask |= 1 << position
        return mask

    def modes_state(self) -> dict:
        """Снимок режимов для монитора эмулятора (/api/state)."""
        active = self.active_mode_names()
        slots = []
        for slot in sorted(self.mode_ranges, key=lambda s: s["index"]):
            linked_name = self.box_name_by_permanent_id.get(slot["linked"], "") if slot["linked"] else ""
            slots.append({
                "index": slot["index"],
                "box": slot["box"],
                "aux": slot["aux"],
                "start": slot["start"],
                "end": slot["end"],
                "logic": "И" if slot["logic"] == MODELOGIC_AND else "ИЛИ",
                "logic_value": int(slot["logic"]),          # как в протоколе: 0 ИЛИ / 1 И
                "linked": linked_name,
                "linked_perm": int(slot["linked"] or 0),    # permanentId связанного режима
                "perm": self.box_permanent_id(slot["box"]),  # permanentId режима слота
                "configured": self._range_configured(slot),
                "listed": slot["box"] in self.active_boxes,
                "active": slot["box"] in active and slot["box"] in self.active_boxes,
            })
        return {
            "slots_total": MAX_MODE_ACTIVATION_CONDITION_COUNT,
            "slots_used": sum(1 for s in slots if s["configured"]),
            "slots": slots,
            "active": sorted(active),
            "mask": self.rc_mode_mask(),
        }

    def aux_lines(self, only_diff: bool = False):
        """Строки 'aux i permId aux start end logic linkedPermanentId' — printAux() из cli.c.

        Прошивка печатает ВСЕ 20 слотов (в том числе пустые: 'aux 3 0 0 900 900 0 0'),
        а в diff-режиме — только настроенные (cliDumpPrintLinef с DO_DIFF).
        """
        out = []
        for r in sorted(self.mode_ranges, key=lambda r: r["index"]):
            if only_diff and not (self._range_configured(r) or r["box"] != "ARM"):
                continue
            out.append(f"aux {r['index']} {self.box_permanent_id(r['box'])} {r['aux']} "
                       f"{r['start']} {r['end']} {r['logic']} {r['linked']}")
        return out

    def _h_boxnames(self, w, r):
        page = r.u8() if len(r.data) else 0
        for name in self.active_boxes[page * 32 : (page + 1) * 32]:
            w.raw(name.encode() + b";")

    def _h_boxids(self, w, r):
        page = r.u8() if len(r.data) else 0
        active = [b for b in self.boxes if b["name"] in self.active_boxes]
        for b in active[page * 32 : (page + 1) * 32]:
            w.u8(b["permanent_id"])

    def _h_mode_ranges(self, w, r):
        """MSP_MODE_RANGES (34): ВСЕ 20 слотов по 4 байта — permanentId, AUX, startStep, endStep.

        Так же, как msp.c 4.5.4: цикл идёт по всему modeActivationConditions[], поэтому ответ
        всегда 80 байт. Пустые слоты уходят как «0 0 0 0» (режим ARM, 900/900) — вкладка
        «Режимы» считает их заполнителями и строк не рисует (см. modeRanges.js).
        """
        for slot in sorted(self.mode_ranges, key=lambda s: s["index"]):
            w.u8(self.box_permanent_id(slot["box"]))
            w.u8(slot["aux"])
            w.u8(self._mode_range_step(slot["start"]))
            w.u8(self._mode_range_step(slot["end"]))

    def _h_mode_ranges_extra(self, w, r):
        """MSP_MODE_RANGES_EXTRA (238): count + по 3 байта на слот — permanentId, logic, linkedTo.

        Без этого ответа вкладка не знает логику И/ИЛИ и связи режимов («Добавить связь»):
        Configurator читает его сразу после MSP_MODE_RANGES.
        """
        slots = sorted(self.mode_ranges, key=lambda s: s["index"])
        w.u8(len(slots))
        for slot in slots:
            w.u8(self.box_permanent_id(slot["box"]))
            w.u8(MODELOGIC_AND if slot["logic"] == MODELOGIC_AND else MODELOGIC_OR)
            w.u8(int(slot["linked"] or 0))

    def _h_serial_config(self, w, r):
        for p in self.serial_ports:
            w.u8(p["identifier"])
            w.u16(self._functions_to_mask(p["functions"]))
            w.u8(p.get("msp_baudrate", 5)).u8(p.get("gps_baudrate", 4)) \
             .u8(p.get("telemetry_baudrate", 0)).u8(p.get("blackbox_baudrate", 5))

    def _h_adjustment_ranges(self, w, r):
        """MSP_ADJUSTMENT_RANGES (52): все 30 слотов подряд по 6 байт — как в msp.c 4.5.4.

        Байта-счётчика в ответе нет: Configurator считает слоты как «длина ответа / 6»
        (MSPHelper.ts; при API ≥ 1.48 на слот приходится 10 байт — центр и масштаб). Первый
        байт прошивка пишет нулём («was adjRange->adjustmentIndex»), следом идут канал,
        начало, конец, функция и канал-переключатель.
        """
        for slot in self.adjustment_ranges:
            w.u8(0).u8(slot["aux"]).u8(slot["start"]).u8(slot["end"])
            w.u8(slot["function"]).u8(slot["switch"])

    def _h_set_adjustment_range(self, w, r):
        """MSP_SET_ADJUSTMENT_RANGE (53): запись одного слота (его индекс — первым байтом)."""
        index = r.u8()
        r.u8()                    # slotIndex: прошивка его игнорирует («was adjustmentIndex»)
        if index >= ADJUSTMENT_RANGE_COUNT:
            self.writes_log.append(
                f"слот корректировки {index}: вне таблицы (0…{ADJUSTMENT_RANGE_COUNT - 1})"
            )
            return
        slot = self.adjustment_ranges[index]
        slot["aux"] = r.u8() & 0x0F
        slot["start"] = min(48, r.u8())
        slot["end"] = min(48, r.u8())
        slot["function"] = min(ADJUSTMENT_FUNCTION_COUNT, r.u8())
        slot["switch"] = r.u8() & 0x0F
        self._log(f"[MSP] слот корректировки {index + 1}: AUX{slot['aux'] + 1} "
                  f"{900 + slot['start'] * 25}…{900 + slot['end'] * 25} импульсов → "
                  f"функция {slot['function']} через AUX{slot['switch'] + 1}")

    def _h_set_mode_range(self, w, r):
        """MSP_SET_MODE_RANGE (35): index, permanentId, AUX, startStep, endStep [, logic, linkedTo].

        Configurator шлёт семь байт (sendModeRanges), но прошивка принимает и пять —
        logic/linkedTo читаются, только если байты остались (sbufBytesRemaining). Пишем ровно
        в тот же слот массива: пустой слот — это диапазон с startStep = endStep.
        """
        index = r.u8()
        box_id = r.u8()
        aux = r.u8()
        start = CHANNEL_RANGE_MIN + r.u8() * MODE_RANGE_STEP
        end = CHANNEL_RANGE_MIN + r.u8() * MODE_RANGE_STEP
        logic = MODELOGIC_OR
        linked = 0
        if len(r.data) - r.pos >= 1:
            logic = r.u8()
        if len(r.data) - r.pos >= 1:
            linked = r.u8()
        if not self.set_mode_range(index, box_id, aux, start, end, logic, linked):
            self.writes_log.append(f"MSP_SET_MODE_RANGE: слот {index} не изменён "
                                   f"(режим {box_id}, AUX {aux})")

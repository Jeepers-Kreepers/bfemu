"""Модель failsafe. Методы VirtualFC.

Примесь использует общее состояние контроллера; создание объекта — через bfemu.fc.VirtualFC.
"""
from __future__ import annotations

import math
from .fc_constants import (
    CHANNEL_RANGE_MIN,
    MAX_SUPPORTED_MOTORS,
    NON_AUX_CHANNEL_COUNT,
    RX_FAILSAFE_MODE_HOLD,
    RX_FAILSAFE_MODE_SET,
    RX_MID_USEC,
    RX_MIN_USEC,
    TX_MAX_USEC,
    TX_MIN_USEC,
)


class FailsafeMixin:


    # ═════════════════════════════════════════════════════ «демо failsafe» (страница /failsafe)
    # Источники поведения — 4.5.4 (копии в docs/bf454-failsafe/):
    #   flight/failsafe.c — failsafeUpdateState(): страж-время (failsafe_delay), JustDisarm,
    #       «Этап 2» по процедуре, KILL/STAGE2 тумблер, восстановление (failsafe_recovery_delay);
    #   rx/rx.c — значения каналов: 0…300 мс держим последние, затем «Этап 1» (rxfail),
    #       в «Этапе 2» стики в центр, газ — failsafe_throttle, AUX — по rxfail;
    #   fc/rc_controls.c — calculateThrottleStatus(): газ ниже min_check = «низкий газ»;
    #   flight/gps_rescue.c — фазы спасения и проверки (спутники, sanity checks, удар).
    # Демо ничего не выдумывает: времена и порядок переходов прошивочные. Масштаб времени
    # (fs_scale) только растягивает показ: ×0.25 — 1.5 с страж-времени идут 6 секунд.
    FS_IDLE = 0
    FS_RX_LOSS_DETECTED = 1
    FS_LANDING = 2
    FS_LANDED = 3
    FS_RX_LOSS_MONITORING = 4
    FS_RX_LOSS_RECOVERED = 5
    FS_GPS_RESCUE = 6
    FS_PHASE_EN = ("IDLE", "RX_LOSS_DETECTED", "LANDING", "LANDED",
                   "RX_LOSS_MONITORING", "RX_LOSS_RECOVERED", "GPS_RESCUE")
    FS_PHASE_RU = ("дежурство (связь есть)",
                   "потеря сигнала: страж-время (Этап 1)",
                   "Этап 2: посадка (газ убирается)",
                   "Этап 2: моторы выключены",
                   "ждём восстановления связи",
                   "сигнал вернулся — выходим из failsafe",
                   "Этап 2: GPS-спасение")
    FS_SCALES = (1.0, 0.5, 0.25)
    FS_HOLD_MS = 300              # rx.c: MAX_INVALID_PULSE_TIME_MS — держим последние значения
    FS_PROCEDURE_NAMES = ("AUTO-LAND", "DROP", "GPS-RESCUE")
    FS_PROCEDURE_RU = ("посадка на месте", "сразу выключить моторы (падение)",
                       "GPS-спасение: вернуться домой")
    FS_SWITCH_MODE_NAMES = ("STAGE1", "KILL", "STAGE2")
    FS_SWITCH_MODE_EN = ("acts as signal loss (stage 1)",
                         "KILL - motors off at once",
                         "stage 2 at once, no guard time")
    FS_SWITCH_MODE_RU = ("как потеря сигнала (Этап 1)",
                         "сразу выключить моторы",
                         "сразу Этап 2, не дожидаясь страж-времени")
    FS_RESCUE_PHASES = ("IDLE", "INITIALIZE", "ATTAIN_ALT", "ROTATE", "FLY_HOME",
                        "DESCENT", "LANDING", "DO_NOTHING", "ABORT", "COMPLETE")
    FS_RESCUE_RU = ("не идёт", "инициализация", "набор высоты", "разворот домой",
                    "полёт домой", "снижение", "посадка", "неуправляемый спуск (20 с)",
                    "прервано: моторы выключены", "завершено")
    FS_RESCUE_FAILURES = ("HEALTHY", "FLYAWAY", "GPSLOST", "LOWSATS", "CRASH_FLIP",
                          "STALLED", "TOO_CLOSE", "NO_HOME_POINT")
    FS_RESCUE_FAILURES_EN = ("checks passed", "flyaway - drifting away from home",
                             "GPS fix lost", "too few satellites", "flip / crash detected",
                             "rescue is not gaining altitude", "too close to home",
                             "no home point")
    FS_RESCUE_FAILURES_RU = ("проверки пройдены", "уносит от дома (flyaway)",
                             "потерян GPS-фикс", "мало спутников", "переворот (краш)",
                             "спасение не набирает высоту", "слишком близко к дому",
                             "нет точки дома")

    def _fs_init_state(self) -> None:
        """Начальное состояние демо — как при включении борта (борт на столе, связь есть)."""
        self.fs_demo = False               # демо включено
        self.fs_scale = 0.25               # масштаб времени показа (×1 … ×0.25)
        self.fs_ms = 0.0                   # «прошивочные» миллисекунды демо
        self.fs_phase = self.FS_IDLE
        self.fs_active = False             # failsafeIsActive(): идёт «Этап 2»
        self.fs_events = 0
        self.fs_armed = False
        self.fs_was_ever_armed = False
        self.fs_valid_failed_at = None     # время, с которого сигнал «плохой» (мс демо)
        self.fs_recovery_started = None    # сигнал вернулся: отсчёт failsafe_recovery_delay
        self.fs_receiving_period = 0.0     # до какого времени связь ещё «не считается поднятой»
        self.fs_landing_finish_at = 0.0
        self.fs_throttle_low_period = 0.0
        self.fs_box_was_on = False
        self.fs_rollback_ms = None         # ТОЧКА ОТКАТА: момент перехода к «Этапу 2»
        self.fs_stage2_ms = None           # время начала «Этапа 2» (для таймлайна)
        self.fs_log = []                   # [(мс, текст)] — журнал событий
        self.fs_beeper = ""                # что «пищит» борт (BEEPER_RX_LOST и т.п.)
        self.fs_disarm_reason = ""
        self.fs_switch_on_ms = None        # когда включён тумблер FAILSAFE
        self.fs_sticks_active = False
        self.fs_receiving_period_preset = 100.0   # мс: задержка восстановления (failsafe.c)
        self.fs_receiving = True                  # снимок failsafeIsReceivingRxData() на шаге
        self.fs_rescue = {"phase": 0, "alt_m": 0.0, "dist_m": 0.0, "failure": 0,
                          "throttle": 1000, "note": "", "checks": []}
        self.fs_gps_north = None           # сценарий спасения двигает точку домой
        self.fs_gps_east = None
        self.fs_gps_alt = None
        self.fs_gps_prev = None
        self.fs_exit_hint = ""
        self.fs_scenario = "none"          # сценарий проверок: none | low_sats | hover_low

    # ---------------------------------------------------------------- конфигурация прошивки
    def _fs_config(self) -> dict:
        """Значения вкладки Failsafe так, как их читает прошивка (failsafeConfig_t)."""
        return {
            "delay_ms": max(100, self._setting("failsafe_delay", 15) * 100),
            "off_delay_ms": int(self._setting("failsafe_off_delay", 10)) * 100,
            "throttle": int(self._setting("failsafe_throttle", 1000)),
            "switch_mode": int(self._setting("failsafe_switch_mode", 0)),
            "throttle_low_ms": int(self._setting("failsafe_throttle_low_delay", 100)) * 100,
            "procedure": int(self._setting("failsafe_procedure", 1)),
            "recovery_ms": max(100, self._setting("failsafe_recovery_delay", 5) * 100),
            "stick_threshold": int(self._setting("failsafe_stick_threshold", 30)),
        }

    def _fs_rescue_config(self) -> dict:
        """Параметры GPS-спасения (вкладка Failsafe, блок «Этап 2 — Процедура Failsafe»)."""
        return {
            "min_sats": int(self._setting("gps_rescue_min_sats", 8)),
            "sanity": int(self._setting("gps_rescue_sanity_checks", 2)),
            "allow_no_fix": int(self._setting("gps_rescue_allow_arming_without_fix", 0)),
            "throttle_hover": int(self._setting("gps_rescue_throttle_hover", 1275)),
            "initial_climb": int(self._setting("gps_rescue_initial_climb", 10)),
            "ascend_rate": int(self._setting("gps_rescue_ascend_rate", 750)) / 100.0,   # м/с
            "descend_rate": int(self._setting("gps_rescue_descend_rate", 150)) / 100.0,  # м/с
            "return_alt": int(self._setting("gps_rescue_return_alt", 30)),
            "ground_speed": int(self._setting("gps_rescue_ground_speed", 750)) / 100.0,  # м/с
            "min_start_dist": int(self._setting("gps_rescue_min_start_dist", 15)),
            "disarm_threshold": int(self._setting("gps_rescue_disarm_threshold", 20)) / 10.0,
            "landing_alt": int(self._setting("gps_rescue_landing_alt", 4)),
            "descent_dist": int(self._setting("gps_rescue_descent_dist", 20)),
            "alt_mode": int(self._setting("gps_rescue_alt_mode", 0)),
            "max_angle": int(self._setting("gps_rescue_max_angle", 45)),
            "use_mag": int(self._setting("gps_rescue_use_mag", 1)),
        }

    # ---------------------------------------------------------------- управление демо
    def set_sim_failsafe(self, on: bool) -> dict:
        """Включить/выключить демо failsafe (кнопка страницы /failsafe, команда CLI).

        Включение: борт «в воздухе и взведён», связь есть, все таймеры с нуля; если был
        включён «виртуальный полёт» — выключаем его (моторы пишет сценарий failsafe).
        Выключение: борт снова на столе — моторы на 1000, фазы сброшены.
        """
        on = bool(on)
        if on:
            if not self.fs_demo:
                if self.sim_flight:
                    self.set_sim_flight(False)
                self._fs_init_state()
                self.fs_demo = True
                self.fs_armed = True
                self.fs_was_ever_armed = True
                self.fs_throttle_low_period = 0.0
                self.fs_log = []
                self._fs_hold_airborne()
                self._fs_log("демо включено: борт в воздухе (газ висения), связь есть, борт взведён",
                             "demo ON: craft airborne (hover throttle), link OK, armed")
                self._log("[SIM] демо failsafe включено (борт взведён, связь есть)")
        elif self.fs_demo:
            self.fs_demo = False
            self.fs_armed = False
            self.motor_disarmed = [1000.0] * MAX_SUPPORTED_MOTORS
            self.fs_gps_north = self.fs_gps_east = self.fs_gps_alt = None
            self.fs_scenario = getattr(self, "fs_scenario", "none")
            self._log("[SIM] демо failsafe выключено (борт на столе, моторы на 1000)")
        return self.sim_failsafe_state()

    def sim_failsafe_loss(self) -> dict:
        """«Потерять сигнал»: выдёргиваем аппаратуру — дальше всё решает прошивка.

        Как «Отключить (выдернуть USB)» на странице аппаратуры: `rx_link_connected = False`.
        Дальше идут страж-время, «Этап 1», «Этап 2» — по таймерам вкладки Failsafe.
        """
        if not self.fs_demo:
            self.set_sim_failsafe(True)
        self.set_rx_link(False)
        return self.sim_failsafe_state()

    def sim_failsafe_restore(self) -> dict:
        """«Вернуть сигнал»: аппаратура снова в эфире.

        До «точки отката» борт возвращается в строй; после неё процедура уже выполняется —
        видно, что сигнал вернулся поздно (или что управление вернётся только после
        failsafe_recovery_delay и фазы восстановления).
        """
        self.set_rx_link(True)
        return self.sim_failsafe_state()

    def sim_failsafe_reset(self) -> dict:
        """«Сброс демо»: связь есть, борт в воздухе и взведён, журнал очищен.

        Масштаб показа (×1/×0.5/×0.25) не сбиваем: это настройка показа для аудитории,
        а не часть сценария — преподаватель выбрал её один раз на всю демонстрацию.
        """
        scale = getattr(self, "fs_scale", 0.25)
        self._fs_init_state()      # журнал, фазы и таймеры — с нуля
        self.fs_scale = scale
        self.fs_demo = True        # «Сброс демо» оставляет демонстрацию включённой
        self.fs_armed = True
        self.fs_was_ever_armed = True
        self.set_rx_link(True)
        self.fs_bind_state = "bound"
        self._fs_hold_airborne()
        self._fs_log("демо сброшено: связь есть, борт в воздухе (газ висения), борт взведён",
                     "demo reset: link OK, craft airborne, armed")
        return self.sim_failsafe_state()

    def _fs_hold_airborne(self) -> None:
        """Борт «в воздухе»: газ висения на виртуальной аппаратуре (1500 мкс).

        Демо показывает полёт, поэтому газ поднят — иначе сработал бы JustDisarm
        (газ ниже min_check дольше failsafe_throttle_low_delay), как на столе.
        """
        self.set_rc_channels({"throttle": RX_MID_USEC})

    def set_sim_failsafe_box(self, on=True) -> dict:
        """Щёлкнуть тумблером FAILSAFE (режим во вкладке «Режимы») — как это делает пилот.

        Если режим FAILSAFE ни на чём не назначен, назначаем его на AUX1 (1300…2100) —
        ровно так его настраивают во вкладке «Режимы»; это видно в журнале демо.
        Тумблер действует и без потери связи: каналы становятся недействительными (rx.c:688),
        а дальше всё решает failsafe_switch_mode: STAGE1 — страж-время, KILL — моторы сразу,
        STAGE2 — Этап 2 немедленно.
        """
        on = bool(on)
        slot = next((s for s in sorted(self.mode_ranges, key=lambda s: s["index"])
                     if s["box"] == "FAILSAFE" and self._range_configured(s)), None)
        if slot is None:
            index = next((s["index"] for s in sorted(self.mode_ranges, key=lambda s: s["index"])
                          if not self._range_configured(s)), 0)
            self.set_mode_range(index, self.box_permanent_id("FAILSAFE"), 0, 1300, 2100)
            slot = self.mode_ranges[index]
            self._fs_log("режим FAILSAFE назначен на AUX1 (1300–2100) — как во вкладке «Режимы»",
                         "FAILSAFE mode assigned to AUX1 (1300-2100) as in the Modes tab")
        # внутри диапазона строго: isRangeActive() проверяет start <= value < end
        value = slot["end"] - 25 if on else max(CHANNEL_RANGE_MIN, slot["start"] - 200)
        self.set_rc_channel(NON_AUX_CHANNEL_COUNT + slot["aux"], value)
        self.fs_box_was_on = on
        mode_text = self.FS_SWITCH_MODE_RU[self._fs_config()["switch_mode"]]
        self._fs_log(f"тумблер FAILSAFE {'включён' if on else 'выключен'} "
                     f"(AUX{slot['aux'] + 1} = {value} мкс, режим {mode_text})",
                     f"FAILSAFE box switch {'ON' if on else 'off'} "
                     f"(AUX{slot['aux'] + 1} = {value} us, {mode_text})")
        return self.sim_failsafe_state()

    def sim_failsafe_arm(self, armed=True, throttle=None) -> dict:
        """Взвести борт: «в воздухе» (газ 1500), «на земле» (газ низкий) или выключить моторы.

        Кнопки страницы /failsafe: «Взлететь», «Стоять на земле (взведён)», «Выключить моторы».
        """
        armed = bool(armed)
        if throttle is not None:
            self.set_rc_channels({"throttle": int(throttle)})
        if armed and not self.fs_armed:
            self.fs_armed = True
            self.fs_was_ever_armed = True
            self.fs_disarm_reason = ""
            if not self._fs_throttle_low():
                self.fs_throttle_low_period = self.fs_ms + self._fs_config()["throttle_low_ms"]
            self._fs_log(f"борт взведён, газ {int(self.rc_channels[3])} мкс",
                         f"armed, throttle {int(self.rc_channels[3])} us")
        elif not armed and self.fs_armed:
            self.fs_armed = False
            self.fs_disarm_reason = "вручную"
            self._fs_log("моторы выключены (disarm вручную)",
                         "motors stopped (disarmed by hand)")
        return self.sim_failsafe_state()

    def set_sim_failsafe_switch(self, mode) -> dict:
        """Режим тумблера FAILSAFE (вкладка «Режимы»): 0 STAGE1, 1 KILL, 2 STAGE2.

        Это та же настройка failsafe_switch_mode, что правит вкладка Failsafe (MSP 75/77) и
        CLI: кнопки страницы /failsafe пишут её через _store_setting(), и машина фаз сразу
        читает новое значение — STAGE1 идёт через страж-время, KILL гасит моторы немедленно,
        STAGE2 входит в «Этап 2» без ожидания (failsafe.c, FAILSAFE_IDLE).
        """
        try:
            value = int(mode)
        except (TypeError, ValueError):
            value = 0
        value = min(2, max(0, value))
        self._store_setting("failsafe_switch_mode", value)
        self._fs_log(f"режим тумблера FAILSAFE: {self.FS_SWITCH_MODE_NAMES[value]} — "
                     f"{self.FS_SWITCH_MODE_RU[value]}",
                     f"FAILSAFE switch mode: {self.FS_SWITCH_MODE_NAMES[value]} - "
                     f"{self.FS_SWITCH_MODE_EN[value]}")
        return self.sim_failsafe_state()

    def set_sim_failsafe_scale(self, value) -> dict:
        """Масштаб времени показа: ×1 (честные тайминги), ×0.5, ×0.25 (для аудитории)."""
        try:
            scale = float(value)
        except (TypeError, ValueError):
            scale = self.fs_scale
        self.fs_scale = scale if scale in self.FS_SCALES else min(self.FS_SCALES,
                                                                 key=lambda s: abs(s - scale))
        self._fs_log(f"масштаб времени показа ×{self.fs_scale:g}",
                     f"show speed x{self.fs_scale:g} (1 = real firmware timing)")
        return self.sim_failsafe_state()

    FS_SCENARIOS = ("none", "low_sats", "hover_low")
    FS_SCENARIO_NAMES = ("обычный полёт домой", "мало спутников",
                         "газа зависания не хватает (не набирает высоту)")

    def set_sim_failsafe_scenario(self, name) -> dict:
        """Сценарий проверок GPS-спасения: обычный / мало спутников / газа зависания не хватает.

        Это единственное, что демо «подстраивает» под сценарий: дальше срабатывают
        прошивочные проверки gps_rescue.c (RESCUE_LOWSATS, переход в RESCUE_LANDING,
        если борт 10 с не набирает/не теряет высоту как задумано).
        """
        key = str(name or "").strip().lower()
        aliases = {"none": "none", "обычный": "none", "ok": "none",
                   "low_sats": "low_sats", "спутники": "low_sats", "sats": "low_sats",
                   "hover_low": "hover_low", "газ": "hover_low", "hover": "hover_low"}
        chosen = aliases.get(key, key if key in self.FS_SCENARIOS else "none")
        self.fs_scenario = chosen
        text = self.FS_SCENARIO_NAMES[self.FS_SCENARIOS.index(chosen)]
        self._fs_log(f"сценарий проверок GPS-спасения: {text}",
                     f"GPS rescue check scenario: {chosen}")
        return self.sim_failsafe_state()

    def _fs_log(self, text: str, en: str = "") -> None:
        """Событие в журнал демо: «1.5 с — сработал failsafe, Этап 1».

        Русский текст — для страницы /failsafe и монитора, английский — для CLI: в консоли
        Configurator'а кириллица приходит мусором, поэтому журнал ведём на двух языках.
        """
        self.fs_log.append((round(self.fs_ms / 1000.0, 2), text, en or text))
        del self.fs_log[:-40]

    # ---------------------------------------------------------------- связь и каналы
    def _fs_box_on(self) -> bool:
        """IS_RC_MODE_ACTIVE(BOXFAILSAFE): включён ли режим FAILSAFE тумблером (вкладка «Режимы»)."""
        return "FAILSAFE" in self.active_mode_names()

    def _fs_signal_bad(self) -> bool:
        """rx.c: rxFlightChannelsValid = rxSignalReceived && !BOXFAILSAFE (rx.c:688).

        Тумблер FAILSAFE делает каналы недействительными при любой настройке (STAGE1/KILL/STAGE2) —
        отличается только то, что дальше делает машина фаз.
        """
        return (not self.rx_link_connected) or self._fs_box_on()

    def _fs_receiving(self) -> bool:
        """failsafeIsReceivingRxData(): rxLinkState == UP.

        Становится false по истечении страж-времени (failsafe_delay), становится true
        сразу после возврата тумблера или через failsafe_recovery_delay после возврата связи.
        """
        cfg = self._fs_config()
        if self._fs_signal_bad():
            self.fs_recovery_started = None
            if self.fs_valid_failed_at is None:
                self.fs_valid_failed_at = self.fs_ms
                self._fs_log(f"сигнала нет: пошло страж-время {cfg['delay_ms'] / 1000:.1f} с",
                             "no signal: stage 1 guard time "
                             f"{cfg['delay_ms'] / 1000:.1f} s started")
            return (self.fs_ms - self.fs_valid_failed_at) < cfg["delay_ms"]
        if self.fs_valid_failed_at is not None:
            self._fs_log("сигнал вернулся до точки отката: управление вернулось, Этап 1 прерван",
                         "signal is back before the rollback point: control returned, stage 1 aborted")
        self.fs_valid_failed_at = None
        if self.fs_phase != self.FS_IDLE or self.fs_active or self.fs_recovery_started is not None:
            if self.fs_recovery_started is None:
                self.fs_recovery_started = self.fs_ms
                if self.fs_receiving_period_preset <= 0:
                    self._fs_log("сигнал вернулся: тумблерный failsafe выходит сразу "
                                 "(receivingRxDataPeriodPreset = 0)",
                                 "signal is back: switch-induced failsafe recovers at once "
                                 "(receivingRxDataPeriodPreset = 0)")
                else:
                    self._fs_log(f"сигнал вернулся: ждём {self.fs_receiving_period_preset / 1000:.1f} с "
                                 f"валидных данных (failsafe_recovery_delay)",
                                 f"signal is back: waiting "
                                 f"{self.fs_receiving_period_preset / 1000:.1f} s of valid data "
                                 "(failsafe_recovery_delay)")
            return (self.fs_ms - self.fs_recovery_started) >= self.fs_receiving_period_preset
        return True

    def _fs_throttle_low(self) -> bool:
        """rc_controls.c: THROTTLE_LOW — газ ниже min_check (вкладка «Приёмник», rxConfig)."""
        channels = self._rx_link_values(self.rc_channels)
        return channels[3] < self._setting("min_check", 1050)

    def _fs_stick_deflection(self) -> int:
        """Насколько стики ушли от центра (в процентах от половины хода) — для выхода из спасения."""
        channels = self._rx_link_values(self.rc_channels)
        deflection = max(abs(channels[0] - RX_MID_USEC), abs(channels[1] - RX_MID_USEC),
                         abs(channels[2] - RX_MID_USEC))
        return int(round(deflection / 5.0))     # 500 мкс = 100 %

    def _rxfail_values(self, channels: list) -> list:
        """«Этап 1» из rx.c: AUTO — центр (газ — rx_min_usec), HOLD — последнее, SET — заданное."""
        values = []
        for index, channel in enumerate(self._rxfail_channels()):
            mode = channel["mode"]
            if mode == RX_FAILSAFE_MODE_HOLD:
                values.append(channels[index])
            elif mode == RX_FAILSAFE_MODE_SET:
                values.append(channel["value"])
            elif index == 3:                     # THROTTLE: AUTO → rx_min_usec
                values.append(RX_MIN_USEC)
            else:                                # ROLL/PITCH/YAW и AUX: AUTO → midrc
                values.append(RX_MID_USEC)
        return values

    def _rx_link_values(self, channels: list) -> list:
        """Значения каналов, которые прошивка отдаёт приложению (rx.c: rcData[]).

        Кадров нет: первые FS_HOLD_MS держим последние значения (rx.c HOLD), затем идут
        значения «Этапа 1» (rxfail), а в «Этапе 2» — стики в центр, газ failsafe_throttle,
        у AUX-каналов значения «Этапа 1» (rx.c, ветка failsafeIsActive()).
        """
        if self.rx_link_connected and not self._fs_signal_bad():
            return list(channels)
        values = self._rxfail_values(channels)
        if not self.fs_demo:
            return values                        # без демо — как раньше: сразу «Этап 1»
        if self.fs_active:
            values[0] = values[1] = values[2] = RX_MID_USEC
            values[3] = self._fs_config()["throttle"]
            return values
        elapsed = self.fs_ms - self.fs_valid_failed_at if self.fs_valid_failed_at is not None \
            else self.fs_ms - (self.fs_switch_on_ms or self.fs_ms)
        if elapsed < self.FS_HOLD_MS:
            return list(channels)                # rx.c: MAX_INVALID_PULSE_TIME_MS
        return values

    # ---------------------------------------------------------------- машина фаз
    def _fs_step(self, dt: float) -> None:
        """Шаг демо: часы (с масштабом показа), затем failsafeUpdateState() и сценарий спасения."""
        if not self.fs_demo:
            return
        # Часы демо идут по-настоящему: шаг — сколько времени реально прошло, но не больше
        # секунды (если эмулятор долго не опрашивали, показ не «проскакивает» целые фазы).
        step = min(dt, 1.0)
        self.fs_ms += step * 1000.0 * self.fs_scale
        box_on = self._fs_box_on()
        if box_on and self.fs_switch_on_ms is None:
            self.fs_switch_on_ms = self.fs_ms
            self._fs_log("тумблер FAILSAFE включён (вкладка «Режимы»)",
                         "FAILSAFE box switch is ON (Modes tab)")
        elif not box_on and self.fs_switch_on_ms is not None:
            self.fs_switch_on_ms = None
            self._fs_log("тумблер FAILSAFE выключен",
                         "FAILSAFE box switch is off")
        self.fs_sticks_active = self._fs_stick_deflection() >= self._fs_config()["stick_threshold"]
        self.fs_receiving = self._fs_receiving()
        self._fs_update_state()
        if self.fs_phase == self.FS_GPS_RESCUE:
            self._fs_rescue_step(step)      # тот же шаг, что у часов демо

    def _fs_update_state(self) -> None:
        """Порт failsafeUpdateState() из failsafe.c 4.5.4 (с циклом reprocessState)."""
        cfg = self._fs_config()
        now = self.fs_ms
        armed = self.fs_armed
        box_on = self._fs_box_on()
        beeper = ""
        reprocess = True
        guard = 0
        while reprocess:
            reprocess = False
            guard += 1
            if guard > 64:
                # страховка демо: любое зацикливание машины фаз останавливаем,
                # а не «вешаем» эмулятор (иначе страница и монитор перестают отвечать)
                self._fs_log("машина фаз: слишком много переходов за шаг — переходы остановлены "
                             "(защита демо)", "failsafe state machine: too many transitions "
                             "in one step - stopped (demo guard)")
                break
            receiving = self._fs_receiving()
            if box_on and cfg["switch_mode"] == 2:
                receiving = False              # STAGE2: «как будто сигнала нет» сразу
            if not receiving and (armed or self.fs_was_ever_armed):
                beeper = "RX_LOST"

            if self.fs_phase == self.FS_IDLE:
                self.fs_box_was_on = box_on
                if armed:
                    if not self._fs_throttle_low():
                        self.fs_throttle_low_period = now + cfg["throttle_low_ms"]
                    if self.fs_box_was_on and cfg["switch_mode"] == 1:
                        self._fs_log("тумблер FAILSAFE = KILL: моторы выключены сразу",
                                     "FAILSAFE switch mode KILL: motors off at once")
                        self.fs_active = True
                        self.fs_events += 1
                        self.fs_rollback_ms = now
                        self.fs_stage2_ms = now
                        self.fs_phase = self.FS_LANDED
                        # failsafe.c: receivingRxDataPeriodPreset = rxDataRecoveryPeriod —
                        # тумблерный KILL восстанавливается по failsafe_recovery_delay
                        self.fs_receiving_period_preset = cfg["recovery_ms"]
                        reprocess = True
                    elif not receiving:
                        if now > self.fs_throttle_low_period and cfg["procedure"] != 2:
                            self.fs_active = True
                            self.fs_events += 1
                            self.fs_rollback_ms = now
                            self.fs_stage2_ms = now
                            self.fs_phase = self.FS_LANDED
                            self.fs_receiving_period_preset = cfg["recovery_ms"]
                            self._fs_log("JustDisarm: газ ниже min_check дольше "
                                         f"failsafe_throttle_low_delay ({cfg['throttle_low_ms'] / 1000:.0f} с) — "
                                         f"моторы выключены без процедуры (failsafe.c)",
                                         "JustDisarm: throttle below min_check for longer than "
                                         f"failsafe_throttle_low_delay ({cfg['throttle_low_ms'] / 1000:.0f} s) "
                                         "- motors off without a procedure (failsafe.c)")
                            reprocess = True
                        else:
                            self.fs_phase = self.FS_RX_LOSS_DETECTED
                            self._fs_log(f"страж-время {cfg['delay_ms'] / 1000:.1f} с истекло: "
                                         f"сработал failsafe, процедура "
                                         f"{self.FS_PROCEDURE_NAMES[cfg['procedure']]}",
                                         f"guard time {cfg['delay_ms'] / 1000:.1f} s expired: failsafe "
                                         f"fired, procedure {self.FS_PROCEDURE_NAMES[cfg['procedure']]} - "
                                         "this is the ROLLBACK POINT")
                            reprocess = True
                else:
                    if self.fs_box_was_on:
                        beeper = beeper or ""

            elif self.fs_phase == self.FS_RX_LOSS_DETECTED:
                if receiving:
                    self.fs_phase = self.FS_RX_LOSS_RECOVERED
                else:
                    self.fs_active = True
                    self.fs_events += 1
                    self.fs_rollback_ms = now
                    self.fs_stage2_ms = now
                    procedure = cfg["procedure"]
                    if procedure == 0:
                        self.fs_phase = self.FS_LANDING
                        self.fs_landing_finish_at = now + cfg["off_delay_ms"]
                        self._fs_log(f"Этап 2: посадка, газ убирается за "
                                     f"{cfg['off_delay_ms'] / 1000:.1f} с, затем моторы в 0",
                                     "stage 2: auto landing, throttle fades out over "
                                     f"{cfg['off_delay_ms'] / 1000:.1f} s, then motors to 0 "
                                     "(failsafe_off_delay)")
                    elif procedure == 1:
                        self.fs_phase = self.FS_LANDED
                        self._fs_log("Этап 2: DROP — моторы выключены сразу, этапа посадки нет",
                                     "stage 2: DROP - motors off at once, no landing phase")
                    else:
                        self.fs_phase = self.FS_GPS_RESCUE
                        self._fs_rescue_start(self._fs_rescue_config())
                    # тумблерный failsafe восстанавливается сразу, потеря связи — по
                    # failsafe_recovery_delay (failsafe.c: receivingRxDataPeriodPreset)
                    self.fs_receiving_period_preset = 0 if self.fs_box_was_on else cfg["recovery_ms"]
                reprocess = True

            elif self.fs_phase == self.FS_LANDING:
                if receiving:
                    self.fs_phase = self.FS_RX_LOSS_RECOVERED
                    self._fs_log("сигнал вернулся во время посадки: управление возвращается, "
                                 "но борт уже снижался (failsafe.c: FAILSAFE_LANDING → RECOVERED)",
                                 "signal back during landing: control returns, but the craft was "
                                 "already descending (failsafe.c: FAILSAFE_LANDING -> RECOVERED)")
                    reprocess = True
                else:
                    if armed:
                        beeper = "RX_LOST_LANDING"
                    if now > self.fs_landing_finish_at or not armed:
                        self.fs_phase = self.FS_LANDED
                        reprocess = True

            elif self.fs_phase == self.FS_GPS_RESCUE:
                if receiving:
                    if self.fs_sticks_active or self.fs_box_was_on:
                        self.fs_phase = self.FS_RX_LOSS_RECOVERED
                        self._fs_log("спасение прервано: стики ушли дальше "
                                     f"{cfg['stick_threshold']} % (failsafe.c: areSticksActive)",
                                     "rescue stopped: sticks moved beyond "
                                     f"{cfg['stick_threshold']} % (failsafe.c: areSticksActive)")
                        reprocess = True
                else:
                    if armed:
                        beeper = "RX_LOST_LANDING"
                    elif self.fs_phase == self.FS_GPS_RESCUE:
                        self.fs_phase = self.FS_LANDED
                        reprocess = True

            elif self.fs_phase == self.FS_LANDED:
                if self.fs_armed:
                    self.fs_armed = False
                    self.fs_disarm_reason = "FAILSAFE"
                    self._fs_log("Этап 2: борт снят с взвода (disarm), "
                                 "взвод заблокирован до восстановления связи",
                                 "stage 2: disarmed, arming blocked until the link is back")
                self.fs_receiving_period = now + self.fs_receiving_period_preset
                self.fs_phase = self.FS_RX_LOSS_MONITORING
                reprocess = True

            elif self.fs_phase == self.FS_RX_LOSS_MONITORING:
                if receiving:
                    if now >= self.fs_receiving_period:
                        self.fs_phase = self.FS_RX_LOSS_RECOVERED
                        reprocess = True
                else:
                    self.fs_receiving_period = now + self.fs_receiving_period_preset

            elif self.fs_phase == self.FS_RX_LOSS_RECOVERED:
                self.fs_throttle_low_period = now + cfg["throttle_low_ms"]
                self.fs_phase = self.FS_IDLE
                self.fs_active = False
                self.fs_rollback_ms = None
                self.fs_stage2_ms = None
                self.fs_recovery_started = None
                self.fs_valid_failed_at = None
                self._fs_log("failsafe завершён: борт снова в строю (IDLE), взвод разрешён",
                             "failsafe finished: craft back in service (IDLE), arming allowed")
                reprocess = True

        if beeper and beeper in self.beepers_disabled:
            beeper = ""            # beeper() (io/beeper.c 4.5.4): выключенный сигнал не пищит
        self.fs_beeper = beeper

    # ---------------------------------------------------------------- GPS-спасение
    def _fs_rescue_start(self, cfg: dict) -> None:
        """Начало спасения: запоминаем точку дома и высоту (gps_rescue.c, RESCUE_INITIALIZE).

        cfg — параметры GPS-спасения вкладки Failsafe (gps_rescue_*), не failsafeConfig_t.
        """
        solution = self.gps_solution()
        # координаты «сейчас» берём из живого GPS (в метрах от дома) и дальше ведём их сами
        dist = float(solution["home_dist_m"])
        angle = math.radians(solution["home_dir_deg"])
        self.fs_gps_north = dist * math.cos(angle)
        self.fs_gps_east = dist * math.sin(angle)
        self.fs_gps_alt = max(30.0, float(solution["alt_m"]))
        self.fs_gps_prev = (self.fs_gps_north, self.fs_gps_east, self.fs_ms)
        self.fs_rescue = {
            "phase": 1,                        # INITIALIZE
            "prev_dist": None,
            "dist_m": round(math.hypot(self.fs_gps_north, self.fs_gps_east), 1),
            "alt_m": round(self.fs_gps_alt, 1),
            "target_alt": 0.0, "target_landing_alt": 0.0,
            "failure": 0, "throttle": cfg["throttle_hover"], "note": "",
            "checks": [], "started_ms": self.fs_ms, "do_nothing_ms": None,
            "prev_alt": round(self.fs_gps_alt, 2), "progress_at": None, "seconds_failing": 0,
        }
        self._fs_log(f"GPS-спасение: борт в {self.fs_rescue['dist_m']:.0f} м от дома, "
                     f"высота {self.fs_rescue['alt_m']:.0f} м",
                     f"GPS rescue: craft {self.fs_rescue['dist_m']:.0f} m from home, "
                     f"altitude {self.fs_rescue['alt_m']:.0f} m")

    def _fs_rescue_step(self, dt: float) -> None:
        """Сценарий спасения по параметрам вкладки Failsafe (flight/gps_rescue.c 4.5.4)."""
        cfg = self._fs_rescue_config()
        rescue = self.fs_rescue
        solution = self.gps_solution()
        sats = solution["sats"]
        if self.fs_scenario == "low_sats":
            sats = max(0, cfg["min_sats"] - 2)     # сценарий: спутников меньше min_sats
        seconds = dt * self.fs_scale
        # заведомо малый газ зависания: в модели борт не держит высоту (порог модели 1050 мкс)
        hover_low = (self.fs_scenario == "hover_low") or cfg["throttle_hover"] <= 1050
        checks = []
        checks.append(f"точка дома: есть (GPS-фикс, {sats} спутников)")
        if sats < cfg["min_sats"]:
            checks.append(f"спутников {sats} < {cfg['min_sats']} (gps_rescue_min_sats) — "
                          f"спасение не считается доступным")
            rescue["failure"] = 3              # RESCUE_LOWSATS
        if rescue["phase"] not in (7, 8, 9):   # не DO_NOTHING/ABORT/COMPLETE
            reason = rescue["failure"]
            if reason:
                sanity = cfg["sanity"]
                hard = not self.rx_link_connected
                if sanity == 1 or (sanity == 2 and hard):
                    rescue["phase"] = 8        # ABORT
                    self._fs_log(f"проверки спасения не пройдены ({self.FS_RESCUE_FAILURES_RU[reason]}): "
                                 f"sanity checks = {('OFF', 'ON', 'FS_ONLY')[sanity]} → моторы выключены",
                                 f"rescue checks failed ({self.FS_RESCUE_FAILURES_EN[reason]}): "
                                 f"sanity checks = {('OFF', 'ON', 'FS_ONLY')[sanity]} - motors off "
                                 "(RESCUE_ABORT)")
                else:
                    rescue["phase"] = 7        # DO_NOTHING
                    rescue["do_nothing_ms"] = self.fs_ms
                    self._fs_log(f"проверки спасения не пройдены ({self.FS_RESCUE_FAILURES_RU[reason]}): "
                                 f"sanity checks = {('OFF', 'ON', 'FS_ONLY')[sanity]} → "
                                 f"20 с неуправляемого спуска с выключением по удару",
                                 f"rescue checks failed ({self.FS_RESCUE_FAILURES_EN[reason]}): "
                                 f"sanity checks = {('OFF', 'ON', 'FS_ONLY')[sanity]} - "
                                 "20 s of uncontrolled descent, motors off on impact (RESCUE_DO_NOTHING)")
        rescue["checks"] = checks

        west, south = self.fs_gps_north, self.fs_gps_east
        dist = math.hypot(west, south)
        alt = self.fs_gps_alt
        phase = rescue["phase"]
        if phase == 1:                          # INITIALIZE
            rescue["target_landing_alt"] = float(cfg["landing_alt"])
            if dist < 5.0 and alt < cfg["landing_alt"]:
                rescue["phase"] = 8
                self._fs_log("спасение у самого дома и на земле: мгновенный disarm (RESCUE_ABORT)",
                             "rescue started at home and on the ground: immediate disarm "
                             "(RESCUE_ABORT)")
            else:
                if dist < cfg["min_start_dist"]:
                    rescue["target_alt"] = max(10.0, alt + cfg["initial_climb"])
                    rescue["note"] = (f"борт ближе {cfg['min_start_dist']} м от дома — сначала "
                                      f"отлетаем, набор высоты как минимум 10 м")
                else:
                    if cfg["alt_mode"] == 0:    # MAX_ALT
                        rescue["target_alt"] = max(alt, float(cfg["return_alt"]))
                    elif cfg["alt_mode"] == 1:  # FIXED_ALT
                        rescue["target_alt"] = float(cfg["return_alt"])
                    else:                       # CURRENT_ALT
                        rescue["target_alt"] = max(alt, alt + cfg["initial_climb"])
                rescue["phase"] = 2
                self._fs_log(f"высота возврата {rescue['target_alt']:.0f} м "
                             f"(gps_rescue_return_alt = {cfg['return_alt']} м, режим "
                             f"{('MAX_ALT', 'FIXED_ALT', 'CURRENT_ALT')[cfg['alt_mode']]})",
                             f"return altitude {rescue['target_alt']:.0f} m "
                             f"(gps_rescue_return_alt = {cfg['return_alt']} m, "
                             f"{('MAX_ALT', 'FIXED_ALT', 'CURRENT_ALT')[cfg['alt_mode']]})")
        elif phase == 2:                        # ATTAIN_ALT — набор высоты
            if hover_low:
                # газа зависания не хватает: борт «висит» на месте, высота не растёт
                rescue["note"] = (f"газ зависания {cfg['throttle_hover']} мкс не держит борт — "
                                   f"высота не растёт")
                if not rescue.get("hover_note") and self.fs_scenario != "hover_low":
                    rescue["hover_note"] = True
                    self._fs_log(f"газ зависания {cfg['throttle_hover']} мкс (gps_rescue_throttle_hover) "
                                 f"слишком мал — борт не держит высоту",
                                 f"hover throttle {cfg['throttle_hover']} us "
                                 "(gps_rescue_throttle_hover) is too low - the craft cannot hold "
                                 "altitude")
            elif alt < rescue["target_alt"]:
                alt = min(rescue["target_alt"], alt + cfg["ascend_rate"] * seconds)
            elif alt > rescue["target_alt"]:
                alt = max(rescue["target_alt"], alt - cfg["descend_rate"] * seconds)
            else:
                rescue["phase"] = 3
                self._fs_log(f"высота {alt:.0f} м набрана (скорость подъёма "
                             f"{cfg['ascend_rate']:.1f} м/с) — разворот домой",
                             f"altitude {alt:.0f} m reached (ascend rate "
                             f"{cfg['ascend_rate']:.1f} m/s) - rotating to head home")
        elif phase == 3:                        # ROTATE — разворот (1 с)
            if not rescue.get("rotate_until"):
                rescue["rotate_until"] = self.fs_ms + 1000.0
            if self.fs_ms >= rescue["rotate_until"]:
                rescue["phase"] = 4
                self._fs_log("разворот домой выполнен (gps_rescue_yaw_p) — полёт домой",
                             "heading home (gps_rescue_yaw_p) - flying home at "
                             f"{cfg['ground_speed']:.1f} m/s")
        elif phase == 4:                        # FLY_HOME
            step = cfg["ground_speed"] * seconds
            if dist <= cfg["descent_dist"]:
                rescue["phase"] = 5
                self._fs_log(f"осталось {dist:.0f} м (gps_rescue_descent_dist = "
                             f"{cfg['descent_dist']} м) — снижение",
                             f"{dist:.0f} m to home (gps_rescue_descent_dist = "
                             f"{cfg['descent_dist']} m) - starting to descend")
            elif step >= dist:
                rescue["phase"] = 5
            else:
                ratio = (dist - step) / dist if dist else 0.0
                west *= ratio
                south *= ratio
                previous = rescue.get("prev_dist")
                if previous is not None and dist >= previous and dist > cfg["min_start_dist"]:
                    rescue["flyaway_s"] = rescue.get("flyaway_s", 0.0) + seconds
                    if rescue["flyaway_s"] > 10.0:
                        rescue["failure"] = 1
                else:
                    rescue["flyaway_s"] = 0.0
                rescue["prev_dist"] = dist
        elif phase == 5:                        # DESCENT — снижается и продолжает идти домой
            alt = max(0.0, alt - cfg["descend_rate"] * seconds)
            if dist > 3.0:
                # gps_rescue.c: descend() гасит скорость; в демо борт идёт домой, пока не окажется
                # над точкой дома, и оттуда снижается вниз
                ratio = max(0.0, (dist - cfg["ground_speed"] * seconds)) / dist
                west *= ratio
                south *= ratio
            if alt < max(1.0, float(cfg["landing_alt"])):
                rescue["phase"] = 6
                self._fs_log(f"высота ниже gps_rescue_landing_alt = {cfg['landing_alt']} м — посадка",
                             f"altitude below gps_rescue_landing_alt = {cfg['landing_alt']} m - landing")
        elif phase == 6:                        # LANDING
            alt = 0.0
            rescue["throttle"] = max(1000, int(cfg["throttle_hover"] * 0.4))
            rescue["phase"] = 9
            self._fs_log(f"посадка завершена: борт сел в {dist:.0f} м от дома, "
                         f"disarm по gps_rescue_disarm_threshold = {cfg['disarm_threshold']:.1f} м/с²",
                         f"landed {dist:.0f} m from home, disarm on impact per "
                         f"gps_rescue_disarm_threshold = {cfg['disarm_threshold']:.1f} m/s2")
        elif phase == 7:                        # DO_NOTHING — 20 с спуска и выключение
            elapsed = (self.fs_ms - rescue["do_nothing_ms"]) / 1000.0
            alt = max(0.0, alt - 2.0 * seconds)
            rescue["throttle"] = max(1000, int(cfg["throttle_hover"] * 0.5))
            if elapsed >= 20.0 or alt <= 0.5:
                rescue["phase"] = 9
                self._fs_log("20 с неуправляемого спуска истекли: борт на земле, моторы выключены",
                             "20 s of uncontrolled descent expired: craft on the ground, motors off")

        # gps_rescue.c (1 Гц): если борт 10 с не меняет высоту как задумано (ratio <= 0.5) —
        # спасение переходит в посадку на месте (RESCUE_ATTAIN_ALT/DESCENT → RESCUE_LANDING)
        if rescue["phase"] in (2, 5):
            now_s = self.fs_ms / 1000.0
            if rescue["progress_at"] is None:
                rescue["progress_at"] = now_s
                rescue["prev_alt"] = alt
            elif now_s - rescue["progress_at"] >= 1.0:
                seconds = now_s - rescue["progress_at"]
                expected = ((cfg["ascend_rate"] if rescue["phase"] == 2 else cfg["descend_rate"])
                            * seconds)
                actual = abs(alt - float(rescue["prev_alt"]))
                ratio = (actual / expected) if expected > 1e-6 else 1.0
                rescue["seconds_failing"] = max(0, min(10, rescue["seconds_failing"]
                                                      + (-1 if ratio > 0.5 else 1)))
                rescue["prev_alt"], rescue["progress_at"] = alt, now_s
                if rescue["seconds_failing"] >= 10:
                    rescue["seconds_failing"] = 0
                    rescue["phase"] = 6
                    self._fs_log("высота не меняется как задумано 10 с (проверка gps_rescue.c) — "
                                 "посадка на месте, домой борт не вернулся",
                                 "altitude has not changed as intended for 10 s (gps_rescue.c check) "
                                 "- landing in place, the craft did not come home")

        self.fs_gps_alt = alt
        self.fs_gps_north, self.fs_gps_east = west, south
        rescue["alt_m"] = round(alt, 1)
        rescue["dist_m"] = round(math.hypot(west, south), 1)
        if rescue["phase"] in (8, 9):
            # RESCUE_ABORT: disarm(DISARM_REASON_FAILSAFE); RESCUE_COMPLETE: rescueStop() + disarm
            rescue["throttle"] = 1000
            if self.fs_armed:
                self.fs_armed = False
                self.fs_disarm_reason = "GPS-RESCUE"

    # ---------------------------------------------------------------- выходы моторов
    def _fs_outputs(self, now: float) -> list:
        """Выходы моторов в демо: висение, посадка (газ вниз), спасение, выключено."""
        count = self._motor_count()
        outputs = [1000.0] * MAX_SUPPORTED_MOTORS
        if not self.fs_armed:
            return outputs
        cfg = self._fs_config()
        phase = self.fs_phase
        if phase in (self.FS_IDLE, self.FS_RX_LOSS_DETECTED, self.FS_RX_LOSS_MONITORING,
                     self.FS_RX_LOSS_RECOVERED):
            gas = self.SIM_HOVER_GAS + 0.02 * math.sin(now * 2 * math.pi / 9.0)
        elif phase == self.FS_LANDING:
            total = max(0.001, cfg["off_delay_ms"] / 1000.0)
            left = max(0.0, (self.fs_landing_finish_at - self.fs_ms) / 1000.0)
            gas = self.SIM_HOVER_GAS * (left / total)
        elif phase == self.FS_GPS_RESCUE:
            rescue_phase = self.fs_rescue["phase"]
            hover = (self.fs_rescue["throttle"] - 1000.0) / 1000.0
            if rescue_phase in (8, 9):
                # RESCUE_ABORT / RESCUE_COMPLETE: gps_rescue.c делает disarm — моторы в 0
                gas = 0.0
            elif rescue_phase in (6, 7):
                gas = max(0.0, hover * 0.4)      # посадка / неуправляемый спуск
            else:
                gas = hover
            if rescue_phase == 2:
                gas = min(1.0, hover + 0.05)          # набор высоты
        else:                                          # LANDED
            gas = 0.0
        for i in range(count):
            wobble = 0.008 * math.sin(now * (9.0 + i) + i * 1.7)
            outputs[i] = 1000.0 + 1000.0 * max(0.0, min(1.0, gas + wobble))
        return outputs

    # ---------------------------------------------------------------- снимок для страницы
    def sim_failsafe_state(self) -> dict:
        """Состояние демо для страницы /failsafe, монитора и CLI."""
        cfg = self._fs_config()
        rescue_cfg = self._fs_rescue_config()

        def wire(value):
            """Как значение уходит в MSP_RC: та же отсечка по краям, что в _h_rc.
            Поэтому «Этап 1» газа виден в Configurator как 900 (rx.c: 885 мкс, отсечка)."""
            return max(TX_MIN_USEC - 100, min(TX_MAX_USEC + 100, int(value)))

        channels = [wire(v) for v in self._rx_link_values(self.rc_channels)]
        stage1 = [wire(v) for v in self._rxfail_values(self.rc_channels)]
        now = self.fs_ms
        # снимок ничего не меняет: значение обновляет шаг демо (_fs_step → _fs_receiving)
        receiving = self.fs_receiving if self.fs_demo else self.rx_link_connected
        timeline = []
        if self.fs_demo:
            timeline = [
                {"name": "связь есть", "done": bool(self.fs_valid_failed_at), "ms": None,
                 "active": self.fs_valid_failed_at is None},
                {"name": (f"Этап 1: страж-время {cfg['delay_ms'] / 1000:.1f} с, "
                          f"значения каналов — по «Этапу 1» (rxfail)"),
                 "done": bool(self.fs_stage2_ms),
                 "left_ms": max(0.0, cfg["delay_ms"] - (now - self.fs_valid_failed_at))
                 if self.fs_valid_failed_at is not None else cfg["delay_ms"],
                 "active": self.fs_valid_failed_at is not None and not self.fs_stage2_ms},
                {"name": "ТОЧКА ОТКАТА: начало Этапа 2", "done": bool(self.fs_stage2_ms),
                 "left_ms": max(0.0, cfg["delay_ms"] - (now - self.fs_valid_failed_at))
                 if self.fs_valid_failed_at is not None and not self.fs_stage2_ms else None,
                 "active": False, "marker": True},
                {"name": {"0": "посадка + моторы в 0", "1": "моторы сразу в 0",
                          "2": "GPS-спасение"}[str(cfg["procedure"])],
                 "done": self.fs_phase in (self.FS_LANDED, self.FS_RX_LOSS_MONITORING,
                                           self.FS_RX_LOSS_RECOVERED)
                 or (self.fs_phase == self.FS_GPS_RESCUE and self.fs_rescue["phase"] == 9),
                 "active": self.fs_phase in (self.FS_LANDING, self.FS_GPS_RESCUE)
                 or (self.fs_phase == self.FS_LANDED), "left_ms": None},
                {"name": "связь восстановлена → IDLE", "done": self.fs_phase == self.FS_IDLE and bool(self.fs_events),
                 "active": self.fs_phase in (self.FS_RX_LOSS_MONITORING, self.FS_RX_LOSS_RECOVERED),
                 "left_ms": None},
            ]
        # английские названия шагов — для CLI (кириллица в консоли Configurator'а не читается)
        for entry, en in zip(timeline, ("link OK",
                                        "stage 1: guard time and stage 1 channel values (rxfail)",
                                        "ROLLBACK POINT: stage 2 starts",
                                        f"stage 2: {self.FS_PROCEDURE_NAMES[cfg['procedure']]}",
                                        "link back, leaving failsafe -> IDLE"), strict=False):
            entry["en"] = en
        rollback = None
        if self.fs_demo:
            if self.fs_stage2_ms:
                rollback = {"possible": False, "value": self.fs_stage2_ms, "left_s": None,
                            "text": "откат невозможен: Этап 2 идёт — процедура уже выполняется"}
            elif self.fs_valid_failed_at is not None:
                left = max(0.0, (cfg["delay_ms"] - (now - self.fs_valid_failed_at)) / 1000.0)
                rollback = {"possible": True, "value": self.fs_stage2_ms, "left_s": round(left, 1),
                            "text": f"откат возможен: сигнал вернёт управление, осталось {left:.1f} с "
                                    f"(страж-время)"}
            else:
                rollback = {"possible": True, "value": None, "left_s": None,
                            "text": "связь есть — борт в строю"}
        return {
            "demo": self.fs_demo,
            "scale": self.fs_scale,
            "millis": round(self.fs_ms),
            "phase": self.fs_phase,
            "phase_en": self.FS_PHASE_EN[self.fs_phase],
            "phase_text": (f"Этап 1: страж-время {cfg['delay_ms'] / 1000:.1f} с идёт, "
                           f"каналы — по «Этапу 1» (rxfail)"
                           if self.fs_phase == self.FS_IDLE and self.fs_valid_failed_at is not None
                           and not self.fs_stage2_ms else self.FS_PHASE_RU[self.fs_phase]),
            "active": self.fs_active,
            "events": self.fs_events,
            "armed": self.fs_armed,
            "throttle": int(round(self.rc_channels[3])),
            "was_ever_armed": self.fs_was_ever_armed,
            "link": bool(self.rx_link_connected),
            "receiving": bool(receiving),
            "beeper": self.fs_beeper,
            "disarm_reason": self.fs_disarm_reason,
            "box_failsafe": self._fs_box_on(),
            "sticks_active": self.fs_sticks_active,
            "procedure": cfg["procedure"],
            "procedure_name": self.FS_PROCEDURE_NAMES[cfg["procedure"]],
            "procedure_text": self.FS_PROCEDURE_RU[cfg["procedure"]],
            "switch_mode": cfg["switch_mode"],
            "switch_mode_text": self.FS_SWITCH_MODE_RU[cfg["switch_mode"]],
            "timeline": timeline,
            "rollback": rollback,
            "log": [(ms, ru) for ms, ru, _en in self.fs_log],
            "log_en": [(ms, en) for ms, _ru, en in self.fs_log],
            "channels": [int(round(v)) for v in channels],
            "stage1": [int(round(v)) for v in stage1],
            "config": {
                "delay": cfg["delay_ms"] / 1000.0,
                "off_delay": cfg["off_delay_ms"] / 1000.0,
                "throttle": cfg["throttle"],
                "throttle_low_delay": cfg["throttle_low_ms"] / 1000.0,
                "recovery_delay": cfg["recovery_ms"] / 1000.0,
                "stick_threshold": cfg["stick_threshold"],
            },
            "scenario": self.fs_scenario,
            "scenario_text": self.FS_SCENARIO_NAMES[self.FS_SCENARIOS.index(self.fs_scenario)],
            "rescue": dict(self.fs_rescue, phase_name=self.FS_RESCUE_PHASES[self.fs_rescue["phase"]],
                           phase_text=self.FS_RESCUE_RU[self.fs_rescue["phase"]],
                           failure_name=self.FS_RESCUE_FAILURES[self.fs_rescue["failure"]],
                           failure_text=self.FS_RESCUE_FAILURES_RU[self.fs_rescue["failure"]]) if self.fs_demo else None,
            "rescue_config": rescue_cfg,
            "exit_hint": self.fs_exit_hint,
        }

    def _h_failsafe_config(self, w, r):
        """MSP_FAILSAFE_CONFIG (75): как pgResetFn_failsafeConfig() в failsafe.c 4.5.4.

        Заводские значения: задержка 15 (1.5 с), время посадки 10 (1.0 с), газ 1000,
        тумблер «Этап 1» (0), задержка низкого газа 100 (10 с), процедура DROP (1).
        """
        w.u8(int(self.settings.get("failsafe_delay", 15)))
        w.u8(int(self.settings.get("failsafe_off_delay", 10)))
        w.u16(int(self.settings.get("failsafe_throttle", 1000)))
        w.u8(int(self.settings.get("failsafe_switch_mode", 0)))
        w.u16(int(self.settings.get("failsafe_throttle_low_delay", 100)))
        w.u8(int(self.settings.get("failsafe_procedure", 1)))

    def _h_set_failsafe_config(self, w, r):
        """MSP_SET_FAILSAFE_CONFIG (76): пишем в те же настройки, что видны в CLI."""
        data = bytes(r.rest())
        if len(data) < 8:
            return b""
        self._store_setting("failsafe_delay", data[0])
        self._store_setting("failsafe_off_delay", data[1])
        self._store_setting("failsafe_throttle", int.from_bytes(data[2:4], "little"))
        self._store_setting("failsafe_switch_mode", data[4])
        self._store_setting("failsafe_throttle_low_delay", int.from_bytes(data[5:7], "little"))
        self._store_setting("failsafe_procedure", data[7])
        self._log(
            f"[MSP] FAILSAFE сохранён: задержка {self.settings['failsafe_delay']} (0.1 с), "
            f"посадка {self.settings['failsafe_off_delay']} (0.1 с), газ {self.settings['failsafe_throttle']}, "
            f"тумблер {self.settings['failsafe_switch_mode']}, низкий газ "
            f"{self.settings['failsafe_throttle_low_delay']} (0.1 с), процедура "
            f"{self.settings['failsafe_procedure']}"
        )
        return b""   # пустой ACK: вкладка Failsafe ждёт ответа на запись

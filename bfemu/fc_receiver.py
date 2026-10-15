"""Каналы приёмника и связь с аппаратурой. Методы VirtualFC.

Примесь использует общее состояние контроллера; создание объекта — через bfemu.fc.VirtualFC.
"""
from __future__ import annotations

import itertools
import math
import time
from .fc_constants import (
    AUX_KNOBS,
    AUX_SWITCH3_VALUES,
    AUX_THREE_POSITION,
    MAX_SUPPORTED_RC_CHANNEL_COUNT,
    NON_AUX_CHANNEL_COUNT,
    PWM_PULSE_MIN,
    RC_AUX_NAMES,
    RC_CHANNEL_COUNT,
    RC_MAPPABLE_CHANNEL_COUNT,
    RC_STICK_NAMES,
    RX_FAILSAFE_MODE_AUTO,
    RX_FAILSAFE_MODE_HOLD,
    RX_FAILSAFE_MODE_SET,
    RX_LOW_USEC,
    RX_MID_USEC,
    RX_MIN_USEC,
    RX_SERIAL_FUNCTION,
    TX_MAX_USEC,
    TX_MID_USEC,
    TX_MIN_USEC,
    TX_SWITCH_ON_USEC,
    state_based_read_codes,
)


class ReceiverMixin:

    # ---------------------------------------------------- виртуальная аппаратура (TX)
    def state_based_read_codes(self) -> set:
        """Коды чтения, ответы на которые строятся из состояния FC (см. state_based_read_codes())."""
        cached = getattr(self, "_state_read_codes", None)
        if cached is None:
            cached = state_based_read_codes()
            self._state_read_codes = cached
        return cached

    @staticmethod
    def _default_rc_channels() -> list:
        """Стики в центре, газ внизу, тумблеры AUX выключены — «аппаратура включена, но стоит»."""
        return ([TX_MID_USEC, TX_MID_USEC, TX_MID_USEC, RX_LOW_USEC]
                + [RX_LOW_USEC] * (RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT))

    def reset_rc_channels(self) -> list:
        """Вернуть стики в исходное положение (кнопка «Сброс» на странице аппаратуры)."""
        self.rc_channels = self._default_rc_channels()
        return list(self.rc_channels)

    def set_rc_channel(self, channel, value) -> bool:
        """Один канал: номер (0…15) или имя (`roll`, `pitch`, `yaw`, `throttle`, `aux1`…`aux12`)."""
        index = self._rc_channel_index(channel)
        if index is None:
            return False
        self.rc_channels[index] = max(TX_MIN_USEC, min(TX_MAX_USEC, int(value)))
        return True

    def set_rc_channels(self, values) -> list:
        """Несколько каналов сразу: список из 16 значений или словарь {имя/номер: значение}."""
        if isinstance(values, dict):
            for channel, value in values.items():
                self.set_rc_channel(channel, value)
        else:
            for index, value in enumerate(list(values)[:RC_CHANNEL_COUNT]):
                if value is not None:
                    self.set_rc_channel(index, value)
        return list(self.rc_channels)

    @staticmethod
    def _rc_channel_index(channel):
        if isinstance(channel, int) or (isinstance(channel, str) and channel.lstrip("-").isdigit()):
            index = int(channel)
            return index if 0 <= index < RC_CHANNEL_COUNT else None
        name = str(channel).strip().lower().replace("_", "").replace(" ", "")
        name = {"a": "roll", "e": "pitch", "r": "yaw", "t": "throttle"}.get(name, name)
        names = list(RC_STICK_NAMES) + list(RC_AUX_NAMES)
        return names.index(name) if name in names else None

    def rc_state(self) -> dict:
        """Снимок каналов для веб-страницы аппаратуры и `/api/rc`."""
        self._resolve_bind()
        values = list(self.rc_channels)
        link = self.rx_link_state()
        return {
            "channels": values,
            "connected": link["connected"],
            "serial_ok": link["serial_ok"],
            "serial_port": link["serial_port"],
            "serial_functions": link["serial_functions"],
            "failsafe": link["failsafe"],
            "bind": link["bind"],
            "control": self.control_state(),
            "map": self.rc_map_info,
            "layout_note": self.rc_layout_note,
            "aux_map": list(self.rc_aux_map),
            **self.rc_source_state(),
            "aux_types": self.aux_types(),
            "switch3_values": list(AUX_SWITCH3_VALUES),
            "source": self.rc_source,
            "active_channels": RC_CHANNEL_COUNT if link["serial_ok"] else 0,
            "names": ["roll", "pitch", "yaw", "throttle", "aux1", "aux2", "aux3", "aux4", "aux5",
                      "aux6", "aux7", "aux8", "aux9", "aux10", "aux11", "aux12"],
            "labels": ["Крен", "Тангаж", "Рыск", "Газ"] + [f"AUX {i}" for i in range(1, 13)],
            "order": "A/E/R/T — как rcData[] в прошивке",
            "switches": {"off": RX_LOW_USEC, "on": TX_SWITCH_ON_USEC},
        }

    # ---------------------------------------------------------------- батарея
    @staticmethod
    def _default_rxfail() -> list:
        """Заводские значения «Этапа 1» (pgResetFn_rxFailsafeChannelConfigs в rx.c).

        Первые четыре канала — AUTO (0): у газа значение RX_MIN_USEC, у остальных
        среднее; AUX-каналы — HOLD (1) со средним значением. Значения идут по сетке
        25 мкс: прошивка хранит «шаг» и переводит его обратно через
        CHANNEL_VALUE_TO_RXFAIL_STEP()/RXFAIL_STEP_TO_CHANNEL_VALUE().
        """
        def snap(value):
            step = (max(PWM_PULSE_MIN, min(2250, value)) - PWM_PULSE_MIN) // 25
            return PWM_PULSE_MIN + 25 * step

        channels = []
        for index in range(RC_CHANNEL_COUNT):
            if index < NON_AUX_CHANNEL_COUNT:
                mode = RX_FAILSAFE_MODE_AUTO
                value = snap(RX_MIN_USEC) if index == 3 else snap(RX_MID_USEC)
            else:
                mode = RX_FAILSAFE_MODE_HOLD
                value = snap(RX_MID_USEC)
            channels.append({"mode": mode, "value": value})
        return channels

    # ------------------- таблицы «только для CLI» (rxrange / led / color / mode_color / mmix)
    @staticmethod
    def _default_rx_ranges() -> list:
        """Заводские диапазоны каналов приёмника (pgResetFn_rxChannelRangeConfigs).

        Прошивка печатает их командой `rxrange`: строка на канал 0…3, заводские 1000…2000.
        Границы правки — PWM_PULSE_MIN/PWM_PULSE_MAX (750…2250) в rx.h, как в cliRxRange().
        """
        return [{"min": 1000, "max": 2000} for _ in range(NON_AUX_CHANNEL_COUNT)]

    def _h_rc(self, w, r):
        """MSP_RC (105): 16 каналов в порядке A/E/R/T — как rcData[] в прошивке.

        Прошивка хранит rcData[] в порядке `rcChannelLetters = "AERT…"` (rx.c), то есть
        индекс 2 — это РЫСК, а индекс 3 — ГАЗ. Configurator подписывает первые четыре полосы
        вкладки жёстко (крен/тангаж/рыск/газ) и вращает модель предпросмотра по каналам 0…2,
        поэтому перепутанные рыск с газом превращаются в «коптер постоянно вращается».

        На столе: стики ровно в центре, газ внизу, тумблеры выключены. Стики отдаём без
        микрошума намеренно: предпросмотр интегрирует поворот от каналов 0…2, и даже ±2 мкс
        шума при ACTUAL-рейтах (rc_rate 7000) дают ~30 °/с — модель уезжает. «Живость» видно
        по AUX 1: там ±2 мкс ни на что не влияют (режимы включаются от 1300 мкс).
        """
        if not self.rx_serial_ready():
            # Во вкладке «Порты» нет ни одного UART с функцией Serial RX: прошивка не
            # поднимает приёмник, channelCount = 0 — MSP_RC отдаёт пустой ответ.
            return

        self._resolve_bind()          # биндинг закончился по времени — кадры пошли
        # rx.c: кадры есть — rcData из эфира; кадров нет — «Этап 1»/«Этап 2» (см. _rx_link_values)
        values = self._rx_link_values(self.rc_channels)

        t = self._t()
        for index in range(RC_CHANNEL_COUNT):
            value = values[index]
            if index == NON_AUX_CHANNEL_COUNT and self.rx_link_connected:
                value += int(math.sin(t * 1.7) * 2)   # AUX 1 — «живой» тумблер
            w.u16(max(TX_MIN_USEC - 100, min(TX_MAX_USEC + 100, int(value))))

    def _mixer_index(self):
        try:
            return self.mixer_names.index(self.mixer_name)
        except ValueError:
            return 0

    # Идентификаторы SOFTSERIAL1/2 из serialPortIdentifier_e (Configurator: portNames.js)
    SOFT_SERIAL_IDS = (30, 31)

    # На нашей виртуальной плате пины SOFTSERIAL выведены (в прошивке это
    # serialPortUsageList + softSerialPinConfig); если бы пинов не было, порты
    # не появлялись бы даже при включённой фиче.
    SOFT_SERIAL_PINS_ASSIGNED = True

    SERIAL_PORT_NAMES = {0: "UART1", 1: "UART2", 2: "UART3", 3: "UART4", 4: "UART5",
                         20: "USB VCP", 30: "SOFTSERIAL1", 31: "SOFTSERIAL2"}

    # -------------------------------------- приёмник: «в эфире» и связь с аппаратурой
    def rx_serial_port(self):
        """Порт, на котором включён Serial RX (в прошивке это проверяет serialRxInit())."""
        for port in self.serial_ports:
            if RX_SERIAL_FUNCTION in port["functions"]:
                return port
        return None

    def rx_serial_ready(self) -> bool:
        """Есть ли во вкладке «Порты» конфигурация приёмника: UART с функцией Serial RX.

        Если такого порта нет, прошивка не поднимает приёмник: `rxRuntimeState.channelCount`
        остаётся нулевым, и MSP_RC отдаёт пустой ответ — Configurator показывает полосы
        приглушёнными, без значений.
        """
        return self.rx_serial_port() is not None

    def set_rx_link(self, connected: bool):
        """Подключить/отключить виртуальную аппаратуру (USB-«пульт» к приёмнику).

        Включённая аппаратура означает, что она уже связана с приёмником; выключенная —
        это потеря связи (не «развязка»): состояние бинда при этом не меняется.
        """
        self.rx_link_connected = bool(connected)
        if self.rx_link_connected:
            self.rx_bind_state = "bound"
            self.rx_bind_deadline = 0.0
            self.rx_bind_seconds = 0.0
        return self.rx_link_connected

    RX_BIND_SECONDS = 2.0          # сколько «ищет приёмник» кнопка «Связать»

    @classmethod
    def aux_type(cls, channel) -> str:
        """Тип органа управления для AUX-канала: `switch`, `switch3` или `knob`."""
        name = str(channel).strip().lower()
        if name.isdigit():
            # номер канала 0…15, как в MSP_RC: AUX начинаются с NON_AUX_CHANNEL_COUNT
            index = int(name)
            if index < NON_AUX_CHANNEL_COUNT:
                return "stick"
            name = f"aux{index - NON_AUX_CHANNEL_COUNT + 1}"
        if name in AUX_KNOBS:
            return "knob"
        if name in AUX_THREE_POSITION:
            return "switch3"
        return "switch"

    def aux_types(self) -> dict:
        """Типы всех AUX-каналов — страница /tx рисует по ним тумблеры, трёхпозиционники и крутилки."""
        return {name: self.aux_type(name) for name in RC_AUX_NAMES}

    def set_rc_control(self, name: str) -> str:
        """Передать управление виртуальной аппаратуре (`virtual`) или физической (`physical`).

        Как в жизни: сменилась аппаратура — связь теряется, и её нужно «связать» заново
        (кнопка «Связать (Bind)» или автоматически, когда пойдут кадры от физической аппаратуры).
        """
        wanted = "physical" if str(name).strip().lower().startswith(("phys", "физ", "реал")) else "virtual"
        if wanted != self.rc_control:
            self.rc_control = wanted
            self.rx_link_connected = False
            self.rx_bind_state = "none"
            self.rx_bind_deadline = 0.0
            self.rc_source = ("физическая аппаратура — ждём кадры "
                              "(run_tx_joystick.bat / run_tx_wire.bat)"
                              if wanted == "physical" else "виртуальная аппаратура (страница /tx)")
        return self.rc_control

    RC_SOURCE_MEMORY = 10.0             # сколько секунд помним прежних писателей каналов
    RC_SOURCE_WINDOW = 5.0              # окно, в котором ищем «пишут несколько источников»

    def note_rc_source(self, source: str) -> None:
        """Запомнить, кто писал каналы: по чередованию источников видно два окна инструмента.

        Симптом: старый запуск `tools/tx.py` (с прежней раскладкой осей) продолжает работать в
        другом окне, а рядом запущен новый. Каналы «прыгают», на вкладке «Приёмник» рыск уезжает
        в AUX, а коптер крутится. Сам по себе инструмент этого не видит — а эмулятор видит.
        """
        now = self._t()
        text = str(source or "")[:96]
        self.rc_channels_updated = now
        if not text:
            return
        if not self.rc_sources or self.rc_sources[-1][1] != text:
            self.rc_sources.append((now, text))
        else:
            self.rc_sources[-1] = (now, text)
        self.rc_sources = [(t, name) for t, name in self.rc_sources
                           if now - t <= self.RC_SOURCE_MEMORY][-12:]

    def rc_source_state(self) -> dict:
        """Кто пишет каналы: последний источник, сколько секунд назад и нет ли чередования."""
        now = self._t()
        fresh = [(t, name) for t, name in self.rc_sources if now - t <= self.RC_SOURCE_WINDOW]
        names = []
        for _t, name in fresh:
            if name not in names:
                names.append(name)
        # Один переход (страница отдала управление инструменту) — это норма. «Пишут двое» — это
        # когда источники ЧЕРЕДУЮТСЯ: A, B, A, B… Тогда каналы и «прыгают».
        changes = sum(1 for (_, a), (_, b) in itertools.pairwise(fresh) if a != b)
        return {
            "source": fresh[-1][1] if fresh else "",
            "updated_ago": round(now - self.rc_channels_updated, 2) if self.rc_channels_updated else None,
            "sources": names,
            "source_changes": changes,
            "multi_source": len(names) > 1 and changes >= 3,
        }

    def set_rc_layout_note(self, text) -> str:
        """Заметка от инструмента аппаратуры: «раскладка осей не та» (её показывает страница /tx).

        Инструмент (`tools/tx.py`) считает это по настоящим положениям осей пульта — страница
        просто показывает текст, чтобы предупреждение было видно и без консоли.
        """
        self.rc_layout_note = str(text or "")[:300]
        return self.rc_layout_note

    def set_rc_aux_map(self, items) -> list:
        """Раскладка AUX от инструмента: какие оси ведут тумблеры и крутилки (показывает /tx).

        Инструмент (`tools/tx.py`) находит их калибровкой `--detect-aux` и присылает готовые
        строки вида «AUX1 ← тумблер (2 положения) — ось 3» — страница просто их показывает.
        """
        if isinstance(items, (str, bytes)):
            items = [items]
        self.rc_aux_map = [str(item)[:80] for item in (items or []) if str(item).strip()][:16]
        return self.rc_aux_map

    def set_rc_map_info(self, info) -> dict:
        """Запомнить раскладку осей физической аппаратуры (для страницы /tx)."""
        if not isinstance(info, dict):
            return self.rc_map_info
        clean = {}
        for name in RC_STICK_NAMES:
            value = info.get(name)
            if value is None:
                continue
            try:
                clean[name] = int(value)
            except (TypeError, ValueError):
                continue
        invert = [str(x) for x in (info.get("invert") or []) if str(x) in RC_STICK_NAMES]
        if clean:
            clean["invert"] = invert
            if info.get("device"):
                clean["device"] = str(info["device"])[:40]
            self.rc_map_info = clean
        return self.rc_map_info

    def control_state(self) -> dict:
        """У кого управление и что это значит для страницы /tx."""
        physical = self.rc_control == "physical"
        return {
            "mode": self.rc_control,
            "physical": physical,
            "virtual": not physical,
            "text": ("управление у физической аппаратуры (пульт по USB или приёмник в COM-порту)"
                     if physical else "управление у виртуальной аппаратуры (эта страница)"),
        }

    def rx_provider_name(self) -> str:
        """Имя протокола приёмника из `serialrx_provider` (CRSF, SBUS, SPEKTRUM2048…)."""
        entry = self.settings_by_name.get("serialrx_provider") or {}
        values = entry.get("values") or []
        index = int(self.settings.get("serialrx_provider", 0))
        if 0 <= index < len(values):
            return str(values[index])
        return str(index)

    def start_bind(self, seconds=None, connected: bool = True):
        """«Связать»: аппаратура ищет приёмник и через несколько секунд связывается.

        Это эмуляция того, что делает кнопка Bind на настоящем приёмнике: пока идёт поиск,
        каналы в эфир не идут (как и при живом бинде — полосы стоят), через `seconds`
        (по умолчанию 2 с) состояние становится «связано» и каналы возвращаются.
        """
        self.rx_bind_seconds = max(0.0, float(self.RX_BIND_SECONDS if seconds is None else seconds))
        self.rx_bind_deadline = time.time() + self.rx_bind_seconds
        if self.rx_bind_seconds <= 0:
            self.rx_bind_state = "bound"
            self.rx_link_connected = bool(connected)
        else:
            # пока аппаратура ищет приёмник, кадров в эфире нет — как при настоящем бинде
            self.rx_bind_state = "binding"
            self.rx_link_connected = False
        return self.bind_state()

    def _resolve_bind(self):
        """Завершить биндинг по времени: связь установлена, кадры пошли."""
        if self.rx_bind_state == "binding" and time.time() >= self.rx_bind_deadline:
            self.rx_bind_state = "bound"
            self.rx_link_connected = True

    def unbind(self):
        """«Развязать»: аппаратура больше не связана с приёмником — кадров нет, «Этап 1»."""
        self.rx_bind_state = "none"
        self.rx_bind_deadline = 0.0
        self.rx_link_connected = False
        return self.bind_state()

    def bind_state(self) -> dict:
        """Состояние бинда для страницы /tx, `tools/tx.py` и вкладки «Приёмник»."""
        self._resolve_bind()
        left = max(0.0, self.rx_bind_deadline - time.time()) if self.rx_bind_state == "binding" else 0.0
        port = self.rx_serial_port()
        if port is None:
            text = "приёмник не поднят: во вкладке «Порты» нет UART с Serial RX"
        elif self.rx_bind_state == "binding":
            text = f"идёт связывание ({self.rx_provider_name()})… осталось {left:.1f} с"
        elif self.rx_bind_state == "bound":
            text = f"связана ({self.rx_provider_name()} на {self.SERIAL_PORT_NAMES.get(port['identifier'], port['identifier'])})"
        elif self.rc_control == "physical":
            text = ("физическая аппаратура не отдаёт кадры — запустите run_tx_joystick.bat (пульт) "
                    "или run_tx_wire.bat (приёмник), затем нажмите «Связать»")
        else:
            text = "НЕ связана: нажмите «Связать» — каналы заработают"
        return {
            "state": self.rx_bind_state,
            "text": text,
            "protocol": self.rx_provider_name(),
            "seconds_left": round(left, 1),
            "seconds": round(self.rx_bind_seconds, 2),
            "ready": port is not None and self.rx_bind_state == "bound",
        }

    def _rx_link_hold_values(self, channels: list) -> list:
        """Значения «Этапа 1» из rx.c: AUTO — центр (газ — rx_min_usec), HOLD — последнее, SET — заданное."""
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

    def rx_link_state(self) -> dict:
        """Состояние приёмника для вкладки «Приёмник» и страницы аппаратуры."""
        self._resolve_bind()
        port = self.rx_serial_port()
        return {
            "connected": self.rx_link_connected,
            "serial_ok": port is not None,
            "serial_port": self.SERIAL_PORT_NAMES.get(port["identifier"], str(port["identifier"])) if port else None,
            "serial_functions": list(port["functions"]) if port else [],
            "protocol": (self.settings_by_name.get("serialrx_provider") or {}).get("values") or [],
            "provider": int(self.settings.get("serialrx_provider", 0)),
            "failsafe": (not self.rx_link_connected) and port is not None,
            "bind": self.bind_state(),
            "control": self.control_state(),
            "stage1": [{"mode": c["mode"], "value": c["value"]} for c in self._rxfail_channels()],
        }

    @staticmethod
    def _default_serial_ports() -> list:
        """Заводская таблица портов — pgResetFn_serialConfig() из io/serial.c 4.5.4.

        Нужна не только на старте: `defaults` (resetConfig() в config.c) сбрасывает
        serialConfig_t вместе с остальными PG-таблицами, то есть функции на портах
        возвращаются к заводским.

        Идентификаторы — как в serialPortIdentifier_e: UART1..UART5 = 0..4, USB VCP = 20,
        SOFTSERIAL1/2 = 30/31. Порядок строк — serialPortIdentifiers[]: USB VCP первым
        (Configurator рисует таблицу в том порядке, в каком пришли строки).
        Скорости — заводские: MSP 115200, датчики/GPS 57600, телеметрия AUTO, «чёрный
        ящик» 115200 (baudRate_e: AUTO=0, 9600=1, ..., 57600=4, 115200=5).
        """
        def port(identifier, functions):
            return {"identifier": identifier, "functions": list(functions), "msp_baudrate": 5,
                    "gps_baudrate": 4, "telemetry_baudrate": 0, "blackbox_baudrate": 5}

        return [
            port(20, ["MSP"]),              # USB VCP
            port(0, ["GPS"]),               # UART1 — GPS-модуль
            port(1, ["RX_SERIAL"]),         # UART2 — приёмник
            port(2, ["ESC_SENSOR"]),        # UART3 — ESC-телеметрия
            port(3, ["TBS_SMARTAUDIO"]),    # UART4 — VTX
            port(4, []),                    # UART5 — свободен
        ]

    def sync_soft_serial_ports(self):
        """Фича SOFTSERIAL включает порты SOFTSERIAL1/2 — как на настоящем FC.

        Прошивка собрана с USE_SOFTSERIAL, поэтому в списке портов они появляются
        тогда, когда фича включена: во вкладке «Порты» видны строки SOFTSERIAL1/2.
        """
        has_soft = {port["identifier"] for port in self.serial_ports} & set(self.SOFT_SERIAL_IDS)
        wanted = "SOFTSERIAL" in self.features and self.SOFT_SERIAL_PINS_ASSIGNED
        if wanted and not has_soft:
            for ident in self.SOFT_SERIAL_IDS:
                self.serial_ports.append({
                    "identifier": ident, "functions": [],
                    "msp_baudrate": 5, "gps_baudrate": 4,
                    "telemetry_baudrate": 0, "blackbox_baudrate": 5,
                })
            self._log("[FC] фича SOFTSERIAL: добавлены порты SOFTSERIAL1/2")
        elif not wanted and has_soft:
            self.serial_ports = [
                port for port in self.serial_ports if port["identifier"] not in self.SOFT_SERIAL_IDS
            ]
            self._log("[FC] фича SOFTSERIAL выключена: порты SOFTSERIAL1/2 убраны")

    def _h_rx_config(self, w, r):
        """MSP_RX_CONFIG — ровно как sbufWrite* в msp.c (API 1.46, 38 байт).

        Раскладка: provider, maxcheck, midrc, mincheck, sat_bind, rx_min_usec, rx_max_usec,
        два устаревших байта, порог airmode (значение*10+1000), RX-SPI (3 поля),
        угол камеры, два устаревших байта, срезы RC-smoothing, два устаревших байта,
        USB CDC HID, авто-фактор и режим RC-smoothing, UID ExpressLRS (6 байт).
        """
        airmode_wire = self._setting("airmode_start_throttle_percent", 30) * 10 + 1000
        w.u8(self._setting("serialrx_provider", 0))
        w.u16(self._setting("max_check", 2000))
        w.u16(self._setting("mid_rc", 1500))
        w.u16(self._setting("min_check", 1050))
        w.u8(self._setting("spektrum_sat_bind", 0))
        w.u16(self._setting("rx_min_usec", 885))
        w.u16(self._setting("rx_max_usec", 2115))
        w.u8(0)  # устарело: rcInterpolation
        w.u8(0)  # устарело: rcInterpolationInterval
        w.u16(airmode_wire)
        w.u8(0)  # RX-SPI протокол (на плате не используется)
        w.u32(0)  # RX-SPI id
        w.u8(0)  # RX-SPI rf channel count
        w.u8(self._setting("fpv_mix_degrees", 0))
        w.u8(0)  # устарело: rcInterpolationChannels
        w.u8(0)  # устарело: rcSmoothingType
        w.u8(self._setting("rc_smoothing_setpoint_cutoff", 0))
        w.u8(self._setting("rc_smoothing_feedforward_cutoff", 0))
        w.u8(0)  # устарело: rcSmoothingInputType
        w.u8(0)  # устарело: rcSmoothingDerivativeType
        w.u8(self._setting("usb_hid_cdc", 0))
        w.u8(self._setting("rc_smoothing_auto_factor", 30))
        w.u8(self._setting("rc_smoothing", 0))
        w.raw(bytes(6))  # UID ExpressLRS (пусто)

    def _h_set_rx_config(self, w, r):
        """MSP_SET_RX_CONFIG: пишем в те же CLI-настройки, что видны командой get."""
        data = bytes(r.rest())
        if len(data) < 32:
            return
        self._store_setting("serialrx_provider", data[0])
        self._store_setting("max_check", int.from_bytes(data[1:3], "little"))
        self._store_setting("mid_rc", int.from_bytes(data[3:5], "little"))
        self._store_setting("min_check", int.from_bytes(data[5:7], "little"))
        self._store_setting("spektrum_sat_bind", data[7])
        self._store_setting("rx_min_usec", int.from_bytes(data[8:10], "little"))
        self._store_setting("rx_max_usec", int.from_bytes(data[10:12], "little"))
        airmode_wire = int.from_bytes(data[14:16], "little")
        self._store_setting("airmode_start_throttle_percent", max(0, (airmode_wire - 1000)) // 10)
        self._store_setting("fpv_mix_degrees", data[22])
        self._store_setting("rc_smoothing_setpoint_cutoff", data[25])
        self._store_setting("rc_smoothing_feedforward_cutoff", data[26])
        self._store_setting("usb_hid_cdc", data[29])
        self._store_setting("rc_smoothing_auto_factor", data[30])
        self._store_setting("rc_smoothing", data[31])
        self._log(
            "[MSP] RX_CONFIG сохранён: provider={serialrx_provider}, rx_min_usec={rx_min_usec}, "
            "rx_max_usec={rx_max_usec}, airmode={airmode_start_throttle_percent}%, "
            "rc_smoothing={rc_smoothing}".format(**self.settings)
        )

    def _h_rssi_config(self, w, r):
        w.u8(int(self.settings.get("rssi_channel", 0)))

    def _h_set_rssi_config(self, w, r):
        """MSP_SET_RSSI_CONFIG (51): номер канала RSSI — в ту же настройку, что видит CLI.

        Раньше код шёл «сквозной записью» и подменял только ответ вкладке: «Приёмник»
        показывал выбранный канал, а `get rssi_channel` возвращал 0.
        """
        data = bytes(r.rest())
        if data and self._store_setting("rssi_channel", data[0]):
            self._log(f"[MSP] RSSI-канал: {self.settings['rssi_channel']} (rssi_channel)")
        return b""

    def _h_rxfail_config(self, w, r):
        """MSP_RXFAIL_CONFIG (77): по 3 байта на канал (mode, value).

        Раньше отдавалось 18 байт подряд, и вкладка Failsafe собирала из них
        6 «каналов» с мусорными значениями вместо 16 настоящих, а у всех каналов
        сразу стоял режим HOLD — в том числе у стиков, где должен быть AUTO.
        """
        for channel in self._rxfail_channels():
            w.u8(channel["mode"])
            w.u16(channel["value"])

    def _rxfail_channels(self) -> list:
        """Список каналов «Этапа 1» ровно длиной RC_CHANNEL_COUNT."""
        channels = [dict(ch) for ch in (self.rxfail or [])]
        channels += self._default_rxfail()[len(channels):]
        return channels[:RC_CHANNEL_COUNT]

    def _rxfail_channels_all(self) -> list:
        """Все каналы «Этапа 1» (MAX_SUPPORTED_RC_CHANNEL_COUNT = 18), как их читает CLI.

        Команда `rxfail` печатает и правит 18 каналов, а MSP-ответ вкладке Failsafe — 16:
        каналы 16 и 17 (AUX13, AUX14) живут во «флеше» и видны только из CLI.
        """
        channels = [dict(ch) for ch in (self.rxfail or [])]
        defaults = self._default_rxfail()
        while len(channels) < MAX_SUPPORTED_RC_CHANNEL_COUNT:
            # каналы 16 и 17 (AUX13/AUX14) — заводские значения AUX-каналов (HOLD, середина)
            channels.append(dict(defaults[-1]))
        return channels[:MAX_SUPPORTED_RC_CHANNEL_COUNT]

    def _h_set_rxfail_config(self, w, r):
        """MSP_SET_RXFAIL_CONFIG (78): index, mode, value — как в msp.c.

        Прошивка переводит значение в «шаг» и обратно, то есть округляет его
        до сетки 25 мкс (CHANNEL_VALUE_TO_RXFAIL_STEP), и отвергает каналы
        с индексом >= MAX_SUPPORTED_RC_CHANNEL_COUNT.
        """
        data = bytes(r.rest())
        if len(data) < 4:
            return b""
        index = data[0]
        if index >= MAX_SUPPORTED_RC_CHANNEL_COUNT:
            return b""
        mode = data[1]
        value = int.from_bytes(data[2:4], "little")
        step = (max(PWM_PULSE_MIN, min(2250, value)) - PWM_PULSE_MIN) // 25
        channel = {"mode": mode, "value": PWM_PULSE_MIN + 25 * step}
        channels = self._rxfail_channels_all()
        channels[index] = channel
        self.rxfail = channels
        self._log(f"[MSP] RXFAIL канал {index + 1}: режим {mode}, значение {channel['value']} мкс")
        return b""   # пустой ACK: вкладка Failsafe ждёт ответа на каждую запись

    def _h_rx_map(self, w, r):
        """MSP_RX_MAP (64): ровно RX_MAPPABLE_CHANNEL_COUNT байт — как sbufWriteData в msp.c.

        Лишние два байта «маски» сбивали разбор: Configurator клал их в FC.RC_MAP
        и подписывал каналы приёмника мусорными буквами.
        """
        letters = {"A": 0, "E": 1, "R": 2, "T": 3, "1": 4, "2": 5, "3": 6, "4": 7}
        for ch in self.rx_map[:RC_MAPPABLE_CHANNEL_COUNT].ljust(RC_MAPPABLE_CHANNEL_COUNT, "1"):
            w.u8(letters.get(ch.upper(), 0))

    def _h_pid_controller(self, w, r):
        w.u8(2)  # PID_CONTROLLER_BETAFLIGHT

    def _h_common_serial_config(self, w, r):
        """MSP2_COMMON_SERIAL_CONFIG: count + (identifier u8, functions u32, 4 скорости u8).

        Именно так порты читает вкладка «Порты» в Configurator 2026.x
        (composables/ports/useFeaturePort.js -> MSP.promise(MSP2_COMMON_SERIAL_CONFIG)).
        """
        w.u8(len(self.serial_ports))
        for p in self.serial_ports:
            w.u8(p["identifier"])
            w.u32(self._functions_to_mask(p["functions"]))
            w.u8(p.get("msp_baudrate", 5)).u8(p.get("gps_baudrate", 4))
            w.u8(p.get("telemetry_baudrate", 0)).u8(p.get("blackbox_baudrate", 5))

    def _h_set_serial_config(self, w, r):
        """MSP_SET_CF_SERIAL_CONFIG (v1): 7 байт на порт (identifier u8, mask u16, 4 скорости)."""
        ports = []
        while r.pos + 7 <= len(r.data):
            ident = r.u8()
            mask = r.u16()
            ports.append({
                "identifier": ident,
                "functions": self._mask_to_functions(mask),
                "msp_baudrate": r.u8(), "gps_baudrate": r.u8(),
                "telemetry_baudrate": r.u8(), "blackbox_baudrate": r.u8(),
            })
        if ports:
            self.serial_ports = ports

    def _h_set_common_serial_config(self, w, r):
        """MSP2_COMMON_SET_SERIAL_CONFIG: count + 9 байт на порт (identifier u8, mask u32, 4 скорости)."""
        count = r.u8()
        ports = []
        for _ in range(count):
            ident = r.u8()
            mask = r.u32()
            ports.append({
                "identifier": ident,
                "functions": self._mask_to_functions(mask),
                "msp_baudrate": r.u8(), "gps_baudrate": r.u8(),
                "telemetry_baudrate": r.u8(), "blackbox_baudrate": r.u8(),
            })
        if ports:
            self.serial_ports = ports

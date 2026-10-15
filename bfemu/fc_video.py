"""OSD и видеопередатчик. Методы VirtualFC.

Примесь использует общее состояние контроллера; создание объекта — через bfemu.fc.VirtualFC.
"""
from __future__ import annotations

import math
from .fc_constants import (
    OSD_CANVAS_COLS,
    OSD_CANVAS_ROWS,
    OSD_DEFAULT_LAYOUT,
    OSD_FLAG_DETECTED,
    OSD_FLAG_FEATURE,
    OSD_FLAG_MAX7456,
    OSD_ITEMS,
    OSD_ITEM_COUNT,
    OSD_PROFILE_COUNT,
    OSD_STATS_DEFAULT,
    OSD_STAT_COUNT,
    OSD_TIMER_COUNT,
    OSD_VISIBLE,
    OSD_WARNINGS_OFF,
    OSD_WARNING_COUNT,
    SA_MODE_UNLOCKED,
    SA_VERSION,
    VTXDEV_SMARTAUDIO,
    VTXDEV_TRAMP,
    VTXDEV_UNKNOWN,
    VTX_COMMON_MSP_BANDCHAN_CHKVAL,
    VTX_STATUS_PIT_MODE,
    VTX_TABLE_BAND_NAME_LENGTH,
    VTX_TABLE_MAX_BANDS,
    VTX_TABLE_MAX_CHANNELS,
    VTX_TABLE_MAX_POWER_LEVELS,
    VTX_TABLE_POWER_LABEL_LENGTH,
)


class VideoMixin:

    # ------------------------------------------------------------------ OSD
    @classmethod
    def _osd_config_from_json(cls, data) -> dict:
        """Настройки OSD из «флеша»: неизвестное и битое заменяется заводскими значениями."""
        cfg = cls._default_osd_config()
        if not isinstance(data, dict):
            return cfg
        simple = ("flags", "video_system", "units", "rssi_alarm", "cap_alarm", "alt_alarm",
                  "link_quality_alarm", "enabled_warnings", "enabled_stats", "osd_profile_index",
                  "overlay_radio_mode", "camera_frame_width", "camera_frame_height",
                  "canvas_cols", "canvas_rows")
        for key in simple:
            try:
                cfg[key] = int(data[key])
            except (KeyError, TypeError, ValueError):
                pass
        positions = data.get("item_pos")
        if isinstance(positions, list) and len(positions) == OSD_ITEM_COUNT:
            cfg["item_pos"] = [int(value) & 0xFFFF for value in positions]
        timers = data.get("timers")
        if isinstance(timers, list) and len(timers) == OSD_TIMER_COUNT:
            cfg["timers"] = [int(value) & 0xFFFF for value in timers]
        if not 1 <= cfg["osd_profile_index"] <= OSD_PROFILE_COUNT:
            cfg["osd_profile_index"] = 1
        return cfg

    @classmethod
    def _default_osd_config(cls) -> dict:
        """Заводские настройки OSD — pgResetFn_osdConfig() из 4.5.4.

        Счётчики (80 элементов, 31 статистика, 19 предупреждений, 2 таймера, 3 профиля) —
        ровно те, что ждёт Configurator по своей версии API. Раскладка по умолчанию повторяет
        привычный экран настроенного квада: напряжение, предупреждения, таймер, газ, высота,
        спутники, имя и RSSI.
        """
        warnings = 0
        for index in range(OSD_WARNING_COUNT):
            if index not in OSD_WARNINGS_OFF:
                warnings |= 1 << index
        stats = 0
        for index in OSD_STATS_DEFAULT:
            stats |= 1 << index
        positions = [0] * OSD_ITEM_COUNT
        for name, (x, y) in cls._osd_default_layout().items():
            index = OSD_ITEMS.index(name)
            if 0 <= index < OSD_ITEM_COUNT:
                positions[index] = cls._osd_position(x, y, 1)   # видно в профиле 1
        return {
            "flags": OSD_FLAG_FEATURE | OSD_FLAG_MAX7456 | OSD_FLAG_DETECTED,
            "video_system": 0,                       # VIDEO_SYSTEM_AUTO — как в pgResetFn_vcdProfile
            "units": 1,                              # UNIT_METRIC
            "rssi_alarm": 20,
            "cap_alarm": 2200,
            "alt_alarm": 100,
            "link_quality_alarm": 80,
            "timers": [cls._osd_timer(0, 0, 10), cls._osd_timer(1, 0, 10)],
            "enabled_warnings": warnings,
            "enabled_stats": stats,
            "osd_profile_index": 1,
            "overlay_radio_mode": 2,
            "camera_frame_width": 24,
            "camera_frame_height": 11,
            "canvas_cols": OSD_CANVAS_COLS,
            "canvas_rows": OSD_CANVAS_ROWS,
            "item_pos": positions,
        }

    @classmethod
    def _osd_default_layout(cls) -> dict:
        return dict(OSD_DEFAULT_LAYOUT)

    @staticmethod
    def _osd_position(x: int, y: int, profile: int = 0) -> int:
        """Упаковать место элемента: биты 0…4 — столбец, 5…9 — строка, 11+номер профиля — «видно».

        Точно как OSD_POSITION/VISIBLE в osd.h: Configurator читает x = bits & 0x1f,
        y = (bits >> 5) & 0x1f, видимость — (bits & (0x0800 << профиль)).
        """
        value = (int(x) & 0x1F) | ((int(y) & 0x1F) << 5)
        if profile:
            value |= OSD_VISIBLE << (int(profile) - 1)
        return value

    @staticmethod
    def _osd_timer(source: int, precision: int, alarm: int) -> int:
        """OSD_TIMER(src, prec, alarm) из osd.h: src — биты 0…3, prec — 4…7, alarm — 8…15."""
        return (int(source) & 0x0F) | ((int(precision) & 0x0F) << 4) | ((int(alarm) & 0xFF) << 8)

    @staticmethod
    def _osd_unpack_position(value: int) -> tuple:
        """Разложить место элемента обратно в (столбец, строка) — для страниц и логов."""
        return (int(value) & 0x1F, (int(value) >> 5) & 0x1F)

    def osd_item_positions_info(self) -> list:
        """Что стоит на экране: [(индекс, имя, x, y, видно ли), …] — для монитора и тестов."""
        rows = []
        for index, value in enumerate(self.osd_config["item_pos"]):
            if not value:
                continue
            x, y = self._osd_unpack_position(value)
            visible = [bool(value & (OSD_VISIBLE << profile))
                       for profile in range(OSD_PROFILE_COUNT)]
            rows.append({"index": index, "name": OSD_ITEMS[index] if index < len(OSD_ITEMS) else "?",
                         "x": x, "y": y, "visible": visible})
        return rows

    # ------------------------------------------------------------ VTX
    @staticmethod
    def _pad(text: str, length: int) -> str:
        """vtxTableStrncpyWithPad(): обрезать и добрать пробелами до нужной длины."""
        return text[:length].ljust(length)

    @staticmethod
    def _default_vtx_activation() -> list:
        """vtxChannelActivationConditions[10] как с завода: все слоты нулевые
        (MAX_CHANNEL_ACTIVATION_CONDITION_COUNT = 10, vtx_control.h 4.5.4)."""
        return [{"aux": 0, "band": 0, "channel": 0, "power": 0,
                 "start_step": 0, "end_step": 0}
                for _ in range(10)]

    def _vtx_activation_from_json(self, value) -> list:
        """Слоты из старого/нового «флеша»: битое отбрасываем, недостающие добираем нулями."""
        out = []
        for item in (value or [])[:10] if isinstance(value, list) else []:
            if isinstance(item, dict):
                out.append({
                    "aux": int(item.get("aux", 0)), "band": int(item.get("band", 0)),
                    "channel": int(item.get("channel", 0)), "power": int(item.get("power", 0)),
                    "start_step": int(item.get("start_step", 0)), "end_step": int(item.get("end_step", 0)),
                })
        while len(out) < 10:
            out.append({"aux": 0, "band": 0, "channel": 0, "power": 0,
                        "start_step": 0, "end_step": 0})
        return out

    @classmethod
    def _empty_vtx_table(cls) -> dict:
        """Пустая таблица — ровно то, что оставляет pgResetFn_vtxTableConfig().

        Заглушки как в прошивке: имена BAND1..BAND8, буквы 1..8, подписи LV0..LV7,
        частоты 0. Числа bands/channels/powerlevels при этом нулевые, и вкладка
        показывает «таблица не настроена» — с этого и начинается живой FC.
        """
        return {
            "bands": 0,
            "channels": 0,
            "powerlevels": 0,
            "band_names": [cls._pad(f"BAND{i + 1}", VTX_TABLE_BAND_NAME_LENGTH)
                           for i in range(VTX_TABLE_MAX_BANDS)],
            "band_letters": [str(i + 1) for i in range(VTX_TABLE_MAX_BANDS)],
            "is_factory": [False] * VTX_TABLE_MAX_BANDS,
            "frequencies": [[0] * VTX_TABLE_MAX_CHANNELS for _ in range(VTX_TABLE_MAX_BANDS)],
            "power_values": [0] * VTX_TABLE_MAX_POWER_LEVELS,
            "power_labels": [cls._pad(f"LV{i}", VTX_TABLE_POWER_LABEL_LENGTH)
                             for i in range(VTX_TABLE_MAX_POWER_LEVELS)],
        }

    # Стандартная таблица 5.8 ГГц: полосы из файла «Betaflight VTX Config», который
    # производители (BNF/RTF) и Configurator грузят во вкладке «ВИДЕОПЕРЕДАТЧИК».
    # Тем она и отличается от pgResetFn_vtxTableConfig(): у «голой» прошивки таблица пуста,
    # а у готового квада — заполнена. С ней жёлтая плашка «таблица VTX не настроена»
    # на вкладке «Порты» не появляется.
    STOCK_BANDS = (
        ("BOSCAM A", "A", [5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725]),
        ("BOSCAM B", "B", [5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866]),
        ("BOSCAM E", "E", [5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945]),
        ("FATSHARK", "F", [5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880]),
        ("RACEBAND", "R", [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917]),
    )
    STOCK_POWERLEVELS = ((25, "25"), (100, "100"), (400, "400"), (800, "800"))

    @classmethod
    def _stock_vtx_table(cls) -> dict:
        """Заполненная таблица передатчика — то, что стоит в «уже настроенном» кваде."""
        table = cls._empty_vtx_table()
        table["bands"] = len(cls.STOCK_BANDS)
        table["channels"] = len(cls.STOCK_BANDS[0][2])
        table["powerlevels"] = len(cls.STOCK_POWERLEVELS)
        for index, (name, letter, frequencies) in enumerate(cls.STOCK_BANDS):
            table["band_names"][index] = cls._pad(name, VTX_TABLE_BAND_NAME_LENGTH)
            table["band_letters"][index] = letter
            table["is_factory"][index] = True
            for channel, frequency in enumerate(frequencies):
                table["frequencies"][index][channel] = frequency
        for index, (value, label) in enumerate(cls.STOCK_POWERLEVELS):
            table["power_values"][index] = value
            table["power_labels"][index] = cls._pad(label, VTX_TABLE_POWER_LABEL_LENGTH)
        return table

    def _select_default_vtx_channel(self) -> None:
        """Выбрать полосу/канал/мощность по таблице — как настроенный передатчик на кваде."""
        table = self.vtx_table
        if not table["bands"] or not table["channels"] or not table["powerlevels"]:
            return
        if self._setting("vtx_band", 0) or self._setting("vtx_channel", 0):
            return                     # человек уже что-то выбрал — не трогаем
        self._store_setting("vtx_band", 1)
        self._store_setting("vtx_channel", 1)
        self._store_setting("vtx_power", 1)
        self._store_setting("vtx_freq", int(table["frequencies"][0][0]))

    def _vtx_device_type(self) -> int:
        """Тип найденного передатчика (VTXDEV_UNKNOWN, если на портах нет функции VTX)."""
        return self.vtx_device_type

    def init_vtx_device(self) -> int:
        """Определение передатчика при загрузке — как vtxInit()/vtxSmartAudioInit().

        Прошивка ищет устройство один раз на старте — по порту с функцией VTX. Поэтому
        включение функции на вкладке «Порты» подхватывается после перезагрузки.
        """
        detected = VTXDEV_UNKNOWN
        for port in self.serial_ports:
            functions = port.get("functions") or []
            if "TBS_SMARTAUDIO" in functions:
                detected = VTXDEV_SMARTAUDIO
                break
            if "IRC_TRAMP" in functions:
                detected = VTXDEV_TRAMP
                break
        names = {VTXDEV_SMARTAUDIO: "TBS SmartAudio", VTXDEV_TRAMP: "IRC Tramp", VTXDEV_UNKNOWN: "не найден"}
        self._log(f"[FC] VTX: {names.get(detected)} (тип {detected})")
        self.vtx_device_type = detected
        return detected

    def _vtx_lookup_frequency(self, band: int, channel: int) -> int:
        """vtxCommonLookupFrequency(): частота из таблицы по банду и каналу."""
        table = self.vtx_table
        if 1 <= band <= table["bands"] and 1 <= channel <= table["channels"]:
            return int(table["frequencies"][band - 1][channel - 1])
        return 0

    def _vtx_clear_table(self) -> None:
        """vtxTableConfigClearBand/ClearPowerValues/ClearPowerLabels — как при флаге clear."""
        self.vtx_table = self._empty_vtx_table()
        self.vtx_activation = self._default_vtx_activation()

    @classmethod
    def _vtx_table_from_json(cls, data: dict) -> dict:
        """Восстановление таблицы VTX из «флеша» с проверкой размеров."""
        table = cls._empty_vtx_table()
        table["bands"] = max(0, min(VTX_TABLE_MAX_BANDS, int(data.get("bands", 0))))
        table["channels"] = max(0, min(VTX_TABLE_MAX_CHANNELS, int(data.get("channels", 0))))
        table["powerlevels"] = max(0, min(VTX_TABLE_MAX_POWER_LEVELS, int(data.get("powerlevels", 0))))
        names = data.get("band_names") or []
        letters = data.get("band_letters") or []
        factory = data.get("is_factory") or []
        freqs = data.get("frequencies") or []
        values = data.get("power_values") or []
        labels = data.get("power_labels") or []
        for i in range(min(len(names), VTX_TABLE_MAX_BANDS)):
            table["band_names"][i] = cls._pad(str(names[i]), VTX_TABLE_BAND_NAME_LENGTH)
        for i in range(min(len(letters), VTX_TABLE_MAX_BANDS)):
            table["band_letters"][i] = str(letters[i])[:1]
        for i in range(min(len(factory), VTX_TABLE_MAX_BANDS)):
            table["is_factory"][i] = bool(factory[i])
        for i, row in enumerate(freqs[:VTX_TABLE_MAX_BANDS]):
            for j, freq in enumerate(list(row)[:VTX_TABLE_MAX_CHANNELS]):
                table["frequencies"][i][j] = max(0, min(5999, int(freq)))
        for i in range(min(len(values), VTX_TABLE_MAX_POWER_LEVELS)):
            table["power_values"][i] = max(0, min(65535, int(values[i])))
        for i in range(min(len(labels), VTX_TABLE_MAX_POWER_LEVELS)):
            table["power_labels"][i] = cls._pad(str(labels[i]).upper(), VTX_TABLE_POWER_LABEL_LENGTH)
        return table

    def _h_osd_config(self, w, r):
        """MSP_OSD_CONFIG (84) — ровно тот порядок полей, что в msp.c 4.5.4.

        Configurator читает ответ по этой раскладке (osd.js: OSD.msp.decode): сначала флаги,
        потом видеосистему, единицы, тревоги, количество элементов и сами места элементов,
        статистику, таймеры, включённые предупреждения, профили, кадр камеры и порог LQ.
        Если флаги нулевые, вкладка «OSD» остаётся пустой — поэтому флаги обязательно непустые.
        """
        cfg = self.osd_config
        w.u8(cfg["flags"])
        w.u8(cfg["video_system"])
        w.u8(cfg["units"])
        w.u8(cfg["rssi_alarm"])
        w.u16(cfg["cap_alarm"])
        w.u8(0)                       # устаревшая «тревога таймера» — теперь тут номер, ниже
        w.u8(OSD_ITEM_COUNT)
        w.u16(cfg["alt_alarm"])
        for value in cfg["item_pos"]:
            w.u16(int(value))
        w.u8(OSD_STAT_COUNT)
        for index in range(OSD_STAT_COUNT):
            w.u8(1 if cfg["enabled_stats"] & (1 << index) else 0)
        w.u8(OSD_TIMER_COUNT)
        for value in cfg["timers"]:
            w.u16(int(value))
        w.u16(cfg["enabled_warnings"] & 0xFFFF)     # младшее слово — для старых версий
        w.u8(OSD_WARNING_COUNT)
        w.u32(cfg["enabled_warnings"])
        w.u8(OSD_PROFILE_COUNT)
        w.u8(cfg["osd_profile_index"])
        w.u8(cfg["overlay_radio_mode"])
        w.u8(cfg["camera_frame_width"])
        w.u8(cfg["camera_frame_height"])
        w.u16(cfg["link_quality_alarm"])            # API >= 1.46

    def _h_set_osd_config(self, w, r):
        """MSP_SET_OSD_CONFIG (85) — как mspProcessInCommand: адрес -1 = общее, -2 = таймер.

        Иначе адрес — номер элемента (значение u16 и байт «экрана»: 0 — статистика после полёта,
        1 и выше — элементы на экране). Так Configurator сохраняет перетаскивание элементов,
        галочки статистики, тревоги и пороги.
        """
        data = bytes(r.rest())
        if not data:
            return
        addr = data[0]
        pos = 1
        unsigned_addr = addr if addr < 128 else addr - 256      # (int8_t)addr, как в прошивке
        cfg = self.osd_config
        if unsigned_addr == -1:                                  # общие настройки
            def take(count):
                nonlocal pos
                chunk = data[pos:pos + count]
                pos += count
                return chunk
            if pos < len(data):
                cfg["video_system"] = take(1)[0]
            if pos < len(data):
                cfg["units"] = take(1)[0]
            if pos < len(data):
                cfg["rssi_alarm"] = take(1)[0]
            if pos + 2 <= len(data):
                cfg["cap_alarm"] = int.from_bytes(take(2), "little")
            if pos + 2 <= len(data):                             # устаревший таймер — пропускаем
                take(2)
            if pos + 2 <= len(data):
                cfg["alt_alarm"] = int.from_bytes(take(2), "little")
            if pos + 2 <= len(data):
                cfg["enabled_warnings"] |= int.from_bytes(take(2), "little")
            if pos + 4 <= len(data):
                cfg["enabled_warnings"] = int.from_bytes(take(4), "little")
            if pos < len(data):
                index = take(1)[0]
                cfg["osd_profile_index"] = index if 1 <= index <= OSD_PROFILE_COUNT else 1
            if pos < len(data):
                cfg["overlay_radio_mode"] = take(1)[0]
            if pos + 2 <= len(data):
                cfg["camera_frame_width"] = take(1)[0]
                cfg["camera_frame_height"] = take(1)[0]
            if pos + 2 <= len(data):
                cfg["link_quality_alarm"] = int.from_bytes(take(2), "little")
            self._log("[MSP] SET_OSD_CONFIG: общие настройки — видео "
                      f"{cfg['video_system']}, единицы {cfg['units']}, предупреждения "
                      f"0x{cfg['enabled_warnings']:x}, профиль {cfg['osd_profile_index']}")
            return
        if unsigned_addr == -2:                                  # таймер
            if pos < len(data):
                index = data[pos]
                pos += 1
                if index < OSD_TIMER_COUNT and pos + 2 <= len(data):
                    cfg["timers"][index] = int.from_bytes(data[pos:pos + 2], "little")
                    self._log(f"[MSP] SET_OSD_CONFIG: таймер {index + 1} = {cfg['timers'][index]}")
            return
        # элемент: u16 значение, затем байт экрана (0 — статистика)
        if pos + 2 > len(data):
            return
        value = int.from_bytes(data[pos:pos + 2], "little")
        pos += 2
        screen = data[pos] if pos < len(data) else 1
        if screen == 0:
            if addr < OSD_STAT_COUNT:
                if value:
                    cfg["enabled_stats"] |= 1 << addr
                else:
                    cfg["enabled_stats"] &= ~(1 << addr)
        elif addr < OSD_ITEM_COUNT:
            cfg["item_pos"][addr] = value
        else:
            return
        self._log(f"[MSP] SET_OSD_CONFIG: экран {screen}, адрес {addr} = {value}")

    def _h_osd_canvas(self, w, r):
        """MSP_OSD_CANVAS (189) — размер «полотна» OSD: столбцы и строки."""
        w.u8(self.osd_config["canvas_cols"])
        w.u8(self.osd_config["canvas_rows"])

    def _h_set_osd_canvas(self, w, r):
        """MSP_SET_OSD_CANVAS (190) — установка размера полотна (как в msp.c)."""
        data = bytes(r.rest())
        if len(data) >= 2:
            self.osd_config["canvas_cols"] = data[0]
            self.osd_config["canvas_rows"] = data[1]
            self._log(f"[MSP] SET_OSD_CANVAS: {data[0]}×{data[1]}")

    def _h_vtx_config(self, w, r):
        """MSP_VTX_CONFIG — 15 байт, как sbufWrite* в msp.c (API 1.42).

        Раскладка: type, band, channel, power, pit_mode, freq u16, device_ready,
        low_power_disarm, pit_mode_freq u16, table_available, bands, channels, powerlevels.
        """
        device_type = self._vtx_device_type()
        table = self.vtx_table
        w.u8(device_type)
        w.u8(self._setting("vtx_band", 0))
        w.u8(self._setting("vtx_channel", 0))
        w.u8(self._setting("vtx_power", 0))
        w.u8(1 if self.vtx_pit_mode else 0)
        w.u16(self._setting("vtx_freq", 0))
        w.u8(1 if device_type != VTXDEV_UNKNOWN else 0)  # deviceIsReady
        w.u8(self._setting("vtx_low_power_disarm", 0))
        w.u16(self._setting("vtx_pit_mode_freq", 0))
        w.u8(1)  # таблица доступна (сборка с USE_VTX_TABLE)
        w.u8(table["bands"]).u8(table["channels"]).u8(table["powerlevels"])

    def _h_vtx_table_band(self, w, r):
        """MSP_VTXTABLE_BAND: в запросе номер банда, в ответ — имя, буква, флаг и частоты."""
        band = r.u8() if r.pos < len(r.data) else 0
        if not 1 <= band <= VTX_TABLE_MAX_BANDS:
            self._log(f"[MSP] VTXTABLE_BAND: банду {band} нет — как на железе, ошибка")
            return
        table = self.vtx_table
        w.u8(band)
        w.u8(VTX_TABLE_BAND_NAME_LENGTH)
        for char in self._pad(table["band_names"][band - 1], VTX_TABLE_BAND_NAME_LENGTH):
            w.u8(ord(char))
        letter = table["band_letters"][band - 1] or " "
        w.u8(ord(letter[0]))
        w.u8(1 if table["is_factory"][band - 1] else 0)
        w.u8(table["channels"])
        for channel in range(table["channels"]):
            w.u16(int(table["frequencies"][band - 1][channel]))

    def _h_vtx_table_powerlevel(self, w, r):
        """MSP_VTXTABLE_POWERLEVEL: значение в мВт и подпись уровня."""
        level = r.u8() if r.pos < len(r.data) else 0
        if not 1 <= level <= VTX_TABLE_MAX_POWER_LEVELS:
            self._log(f"[MSP] VTXTABLE_POWERLEVEL: уровня {level} нет — как на железе, ошибка")
            return
        table = self.vtx_table
        w.u8(level)
        w.u16(int(table["power_values"][level - 1]))
        w.u8(VTX_TABLE_POWER_LABEL_LENGTH)
        for char in self._pad(table["power_labels"][level - 1], VTX_TABLE_POWER_LABEL_LENGTH):
            w.u8(ord(char))

    def _h_vtx_device_status(self, w, r):
        """MSP2_GET_VTX_DEVICE_STATUS (0x3004) — vtxCommonSerializeDeviceStatus().

        Без устройства прошивка не пишет ничего; со SmartAudio — тип, готовность,
        банд/канал, индекс мощности, частоту, флаги состояния, список уровней (dBm и мВт)
        и «паспорт» передатчика: размер, версия, режим, частота pit-режима, признак загрузки в pit.
        """
        device_type = self._vtx_device_type()
        if device_type == VTXDEV_UNKNOWN:
            return
        table = self.vtx_table
        band = self._setting("vtx_band", 0)
        channel = self._setting("vtx_channel", 0)
        w.u8(device_type).u8(1)                  # type, deviceIsReady
        w.u8(1).u8(band).u8(channel)             # bandAndChannelAvailable
        w.u8(1).u8(self._setting("vtx_power", 0))  # powerIndexAvailable
        w.u8(1).u16(self._setting("vtx_freq", 0))  # frequencyAvailable
        status = VTX_STATUS_PIT_MODE if self.vtx_pit_mode else 0
        w.u8(1).u32(status)                      # vtxStatusAvailable
        w.u8(table["powerlevels"])
        for index in range(table["powerlevels"]):
            power_mw = int(table["power_values"][index])
            dbm = round(10 * math.log10(power_mw)) if power_mw > 0 else 0
            w.u16(dbm).u16(power_mw)
        w.u8(5)                                   # VTX_CUSTOM_DEVICE_STATUS_SIZE
        w.u8(SA_VERSION).u8(SA_MODE_UNLOCKED)
        w.u16(self._setting("vtx_pit_mode_freq", 0))
        w.u8(0)                                   # willBootIntoPitMode

    def _h_set_vtx_config(self, w, r):
        """MSP_SET_VTX_CONFIG — та же логика, что в msp.c: частота/банд-канал и условные поля."""
        data = bytes(r.rest())
        if len(data) < 2:
            return
        new_frequency = int.from_bytes(data[0:2], "little")
        if new_frequency <= VTX_COMMON_MSP_BANDCHAN_CHKVAL:      # упакованные банд и канал
            band = new_frequency // 8 + 1
            channel = new_frequency % 8 + 1
            self._store_setting("vtx_band", band)
            self._store_setting("vtx_channel", channel)
            self._store_setting("vtx_freq", self._vtx_lookup_frequency(band, channel))
        elif new_frequency <= 5999:                              # частота в МГц
            self._store_setting("vtx_band", 0)
            self._store_setting("vtx_freq", new_frequency)

        if len(data) >= 4:
            self._store_setting("vtx_power", data[2])
            self.vtx_pit_mode = bool(data[3])                    # делегируется передатчику

        if len(data) >= 5:
            self._store_setting("vtx_low_power_disarm", data[4])

        if len(data) >= 7:
            self._store_setting("vtx_pit_mode_freq", int.from_bytes(data[5:7], "little"))

        if len(data) >= 11:                                      # банд/канал/частота отдельными полями
            band, channel = data[7], data[8]
            frequency = int.from_bytes(data[9:11], "little")
            if band:
                frequency = self._vtx_lookup_frequency(band, channel)
            self._store_setting("vtx_band", band)
            self._store_setting("vtx_channel", channel)
            self._store_setting("vtx_freq", frequency)

        if len(data) >= 15:                                      # размеры таблицы и её очистка
            bands, channels, powerlevels, clear = data[11], data[12], data[13], data[14]
            if (bands > VTX_TABLE_MAX_BANDS or channels > VTX_TABLE_MAX_CHANNELS
                    or powerlevels > VTX_TABLE_MAX_POWER_LEVELS):
                self._log("[MSP] SET_VTX_CONFIG: размеры таблицы больше допустимых — как на железе, ошибка")
                return
            self.vtx_table["bands"] = bands
            self.vtx_table["channels"] = channels
            self.vtx_table["powerlevels"] = powerlevels
            if clear:
                self._vtx_clear_table()
                self.vtx_table["bands"] = bands
                self.vtx_table["channels"] = channels
                self.vtx_table["powerlevels"] = powerlevels
        self._log(
            "[MSP] VTX_CONFIG сохранён: band={vtx_band}, channel={vtx_channel}, "
            "power={vtx_power}, freq={vtx_freq}, pit_mode={}".format(
                "on" if self.vtx_pit_mode else "off", **self.settings
            )
        )

    def _h_set_vtx_table_band(self, w, r):
        """MSP_SET_VTXTABLE_BAND: имя и буква в верхний регистр, частоты по числу каналов."""
        data = bytes(r.rest())
        if len(data) < 4:
            return
        band = data[0]
        name_length = data[1]
        name = "".join(chr(c) for c in data[2:2 + name_length]).upper()
        pos = 2 + name_length
        if pos + 2 > len(data):
            return
        letter = chr(data[pos]).upper()
        is_factory = bool(data[pos + 1])
        channel_count = data[pos + 2] if pos + 2 < len(data) else 0
        frequencies = [
            int.from_bytes(data[pos + 3 + 2 * i:pos + 5 + 2 * i], "little")
            for i in range(channel_count)
            if pos + 5 + 2 * i <= len(data)
        ]
        table = self.vtx_table
        if not 1 <= band <= table["bands"]:
            self._log(f"[MSP] SET_VTXTABLE_BAND: банду {band} нет в таблице — ошибка")
            return
        table["band_names"][band - 1] = self._pad(name, VTX_TABLE_BAND_NAME_LENGTH)
        table["band_letters"][band - 1] = letter or " "
        table["is_factory"][band - 1] = is_factory
        for channel in range(table["channels"]):
            table["frequencies"][band - 1][channel] = (
                frequencies[channel] if channel < len(frequencies) else 0
            )
        if band == self._setting("vtx_band", 0):                 # это выбранный банд — частота меняется
            current_channel = self._setting("vtx_channel", 0)
            if 1 <= current_channel <= table["channels"]:
                self._store_setting("vtx_freq", table["frequencies"][band - 1][current_channel - 1])
        self._log(f"[MSP] VTXTABLE_BAND {band}: имя {name!r}, каналов {table['channels']}")

    def _h_set_vtx_table_powerlevel(self, w, r):
        """MSP_SET_VTXTABLE_POWERLEVEL: значение в мВт и подпись уровня."""
        data = bytes(r.rest())
        if len(data) < 4:
            return
        level = data[0]
        value = int.from_bytes(data[1:3], "little")
        label_length = data[3]
        label = "".join(chr(c) for c in data[4:4 + label_length]).upper()
        table = self.vtx_table
        if not 1 <= level <= table["powerlevels"]:
            self._log(f"[MSP] SET_VTXTABLE_POWERLEVEL: уровня {level} нет в таблице — ошибка")
            return
        table["power_values"][level - 1] = value
        table["power_labels"][level - 1] = self._pad(label, VTX_TABLE_POWER_LABEL_LENGTH)
        self._log(f"[MSP] VTXTABLE_POWERLEVEL {level}: {value} мВт, подпись {label!r}")

    def _h_osd_warnings(self, w, r):
        pass

    def _h_mcu_info(self, w, r):
        w.u8(self.mcu_type_id).u16(8192).u32(0).u32(0)

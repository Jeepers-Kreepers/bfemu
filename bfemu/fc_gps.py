"""GPS и спасение. Методы VirtualFC.

Примесь использует общее состояние контроллера; создание объекта — через bfemu.fc.VirtualFC.
"""
from __future__ import annotations

import math


class GpsMixin:

    # ------------------------------------------------------------ GPS (виртуальный приёмник)
    # На виртуальной плате GPS сидит на UART1 (см. serial_ports), модуль — UBLOX.
    # Координаты медленно «ходят» вокруг точки взлёта: вкладка «GPS» и OSD показывают живые
    # данные, а не замороженные константы. Курс — в 0.1°, как gpsSol.groundCourse в прошивке.
    GPS_BASE_LAT = 48.137154          # база (тот же Мюнхен, что в примерах Betaflight)
    GPS_BASE_LON = 11.575382
    GPS_ALT_BASE_M = 12
    GPS_TRACK_M = 25.0                # радиус «прогулки» вокруг базы, м
    # Спутники: (система, номер, C/N0). Системы — как в прошивке (gnssId): 0 GPS, 1 SBAS,
    # 2 Galileo, 3 BeiDou, 6 Glonass; порядок — как отдаёт настоящий модуль (по системам).
    # Список длиннее 16 спутников, и это принципиально: Configurator (GpsTab.vue) включает
    # подробную таблицу сигналов — имена систем в колонке Gnss ID, зелёные метки
    # «используется», подписи качества — ТОЛЬКО когда спутников больше 16. При 16 и меньше
    # он рисует упрощённый список: прочерки в Gnss ID, числовое качество, все метки красные.
    GPS_SATELLITES = (
        (0, 2, 44), (0, 5, 47), (0, 10, 28), (0, 12, 46), (0, 15, 42),
        (0, 18, 38), (0, 24, 35), (0, 26, 26), (0, 29, 33),
        (1, 123, 37), (1, 136, 30),
        (2, 1, 45), (2, 11, 39), (2, 15, 34), (2, 19, 41), (2, 36, 27),
        (3, 7, 31), (3, 12, 25),
        (6, 3, 43), (6, 4, 28), (6, 9, 37), (6, 10, 29), (6, 17, 32), (6, 21, 24),
    )

    # Пустое решение: без фичи GPS у прошивки нулевой gpsSol, поэтому и ответы нулевые.
    GPS_NO_FIX = {"fix": 0, "sats": 0, "lat": 0.0, "lon": 0.0, "alt_m": 0, "speed_cms": 0,
                  "course_deg10": 0, "pdop": 0, "home_dist_m": 0, "home_dir_deg": 0,
                  "satellites": 0}

    @property
    def gps_enabled(self) -> bool:
        """Приёмник работает, только пока включена фича GPS.

        `gpsInit()` прошивки не поднимает модуль без `FEATURE_GPS`: `gpsSol` остаётся нулевым,
        `STATE(GPS_FIX)` не выставляется, каналов спутников нет — вкладка показывает «нет фикса»,
        а не живые координаты (находка повторного ревью, часть 4).
        """
        return "GPS" in self.features

    def gps_solution(self) -> dict:
        """Текущее решение GPS: координаты, скорость, курс, высота, спутники, «дом»."""
        if not self.gps_enabled:
            return dict(self.GPS_NO_FIX)
        t = self._t()
        north = self.GPS_TRACK_M * math.sin(t / 37.0)          # м от базы
        east = self.GPS_TRACK_M * math.cos(t / 53.0)
        north_v = self.GPS_TRACK_M / 37.0 * math.cos(t / 37.0)  # м/с
        east_v = -self.GPS_TRACK_M / 53.0 * math.sin(t / 53.0)
        lat = self.GPS_BASE_LAT + north / 111_320.0
        lon = self.GPS_BASE_LON + east / (111_320.0 * math.cos(math.radians(self.GPS_BASE_LAT)))
        used, sats = self.gps_satellites(t)
        if self.fs_gps_north is not None:
            # Демо GPS-спасения: борт реально летит домой — координаты, высота, скорость
            # и курс берём из сценария, чтобы вкладка «GPS» показывала возврат.
            north, east, alt = self.fs_gps_north, self.fs_gps_east, self.fs_gps_alt
            previous = self.fs_gps_prev
            if previous and self.fs_ms > previous[2]:
                dt_s = (self.fs_ms - previous[2]) / 1000.0
                north_v = (north - previous[0]) / dt_s
                east_v = (east - previous[1]) / dt_s
            else:
                north_v = east_v = 0.0
            self.fs_gps_prev = (north, east, self.fs_ms)
            return {
                "fix": 2,
                "sats": used,
                "lat": self.GPS_BASE_LAT + north / 111_320.0,
                "lon": self.GPS_BASE_LON + east / (111_320.0 * math.cos(math.radians(self.GPS_BASE_LAT))),
                "alt_m": int(round(alt)),
                "speed_cms": int(round(math.hypot(north_v, east_v) * 100)),
                "course_deg10": int(round((math.degrees(math.atan2(east_v, north_v)) % 360.0) * 10)),
                "pdop": 105,
                "home_dist_m": int(round(math.hypot(north, east))),
                "home_dir_deg": int(round(math.degrees(math.atan2(east, north)) % 360.0)),
                "satellites": sats,
            }
        return {
            "fix": 2,                                   # STATE(GPS_FIX) = 1 << 1 (runtime_config.h)
            "sats": used,
            "lat": lat,
            "lon": lon,
            "alt_m": int(round(self.GPS_ALT_BASE_M + 1.5 * math.sin(t / 23.0))),
            "speed_cms": int(round(math.hypot(north_v, east_v) * 100)),
            "course_deg10": int(round((math.degrees(math.atan2(east_v, north_v)) % 360.0) * 10)),
            "pdop": 105 + int(round(15 * math.sin(t / 31.0))),   # pdop = 0.01 на единицу
            "home_dist_m": int(round(math.hypot(north, east))),
            "home_dir_deg": int(round(math.degrees(math.atan2(east, north)) % 360.0)),
            "satellites": sats,
        }

    def gps_satellites(self, t: float | None = None) -> tuple:
        """Спутники для MSP_GPSSVINFO: (число спутников в решении, [(система, svid, quality, cno)]).

        В решении — самые сильные по сигналу (как отбирает сам приёмник), остальные видны,
        но «не используются». quality: младшие 3 бита — качество (0 нет сигнала, 1 поиск,
        2/4 отслеживается, 5…7 полностью отслеживается), бит 3 — «используется в решении»
        (Configurator читает: (quality & 0x8) >> 3 и quality & 0x7).
        """
        if not self.gps_enabled:
            return 0, []                              # GPS_numCh = 0: модуль не поднят
        if t is None:
            t = self._t()
        used = 9 + int(round(math.sin(t / 29.0)))     # 8…10 спутников идут в решение
        entries = []
        for i, (gnss, svid, base_cno) in enumerate(self.GPS_SATELLITES):
            cno = max(10, min(52, base_cno + int(round(2 * math.sin(t / 7.0 + i)))))
            entries.append((gnss, svid, cno))
        strongest = {index for index, _ in
                     sorted(enumerate(entries), key=lambda pair: -pair[1][2])[:used]}
        rows = []
        for index, (gnss, svid, cno) in enumerate(entries):
            if cno >= 40:
                quality_value = 7                       # полностью отслеживается
            elif cno >= 37:
                quality_value = 6
            elif cno >= 34:
                quality_value = 5
            elif cno >= 31:
                quality_value = 4                       # «отслеживается» (жёлтая метка)
            elif cno >= 28:
                quality_value = 2                       # слабый — «приобретён» (серая метка)
            else:
                quality_value = 1                       # едва виден — «поиск»
            quality = quality_value | (0x08 if index in strongest else 0x00)
            rows.append((gnss, svid, quality, cno))
        return used, rows

    def _h_raw_gps(self, w, r):
        """MSP_RAW_GPS (106) — раскладка msp.c 4.5.4: fix, спутники, координаты, высота,
        скорость, курс (0.1°) и pdop (0.01)."""
        g = self.gps_solution()
        w.u8(g["fix"]).u8(g["sats"])
        w.i32(int(round(g["lat"] * 1e7))).i32(int(round(g["lon"] * 1e7)))
        w.u16(g["alt_m"]).u16(g["speed_cms"]).u16(g["course_deg10"]).u16(g["pdop"])

    def _h_comp_gps(self, w, r):
        """MSP_COMP_GPS (107): дистанция до дома (м), направление (град) и признак обновления."""
        g = self.gps_solution()
        w.u16(g["home_dist_m"]).u16(g["home_dir_deg"])
        # GPS_update & 1 — мигает, как на железе; без фичи данных нет и флаг снят
        w.u8(1 if self.gps_enabled and int(self._t() * 10) % 2 else 0)

    def _h_sensor_status(self, w, r):
        w.u8(1).u8(1).u8(1 if "MAG" in self.active_boxes else 0).u8(1)

    def _h_gps_sv_info(self, w, r):
        """MSP_GPSSVINFO (164): число спутников, затем по 4 байта на спутник —
        система (gnss), номер (svid), качество, уровень сигнала (C/N0, дБ)."""
        _used, rows = self.gps_satellites()
        w.u8(len(rows))
        for gnss, svid, quality, cno in rows:
            w.u8(gnss).u8(svid).u8(quality).u8(cno)

    # --- конфигурация (только чтение/эхо)
    def _h_mixer_config(self, w, r):
        idx = max(0, min(len(self.mixer_names) - 1, self._mixer_index()))
        w.u8(idx).u8(0)

    def _h_gps_config(self, w, r):
        """MSP_GPS_CONFIG (132): provider, ublox_sbas, auto_config, auto_baud, home_point_once,
        ublox_use_galileo — ровно в том порядке, в каком их читает MSPHelper.js."""
        w.u8(self._setting("gps_provider", 1))
        w.u8(self._setting("gps_sbas_mode", 0))
        w.u8(self._setting("gps_auto_config", 1))
        w.u8(self._setting("gps_auto_baud", 0))
        w.u8(self._setting("gps_set_home_point_once", 0))
        w.u8(self._setting("gps_ublox_use_galileo", 0))

    def _h_set_gps_config(self, w, r):
        """MSP_SET_GPS_CONFIG (223) — то, что пишет кнопка «Сохранить» во вкладке «GPS»:
        те же шесть полей в том же порядке, что и чтение."""
        names = ("gps_provider", "gps_sbas_mode", "gps_auto_config", "gps_auto_baud",
                 "gps_set_home_point_once", "gps_ublox_use_galileo")
        values = list(r.data[r.pos : r.pos + len(names)])
        for name, value in zip(names, values, strict=False):
            self._store_setting(name, value)
        self._log("[MSP] GPS_CONFIG сохранён: " + ", ".join(f"{n}={v}" for n, v in zip(names, values, strict=False)))
        return b""

    def _h_gps_rescue(self, w, r):
        """MSP_GPS_RESCUE (API 1.46) — 26 байт, ровно как sbufWrite* в msp.c.

        Раскладка: 7 значений u16 (угол, высоты, скорости, газ), 2 байта проверок,
        скорости подъёма/спуска u16, 2 байта режима, дистанция старта u16,
        высота начального набора u16 (добавлена в 1.46).
        """
        w.u16(self._gps_rescue_setting("gps_rescue_max_angle"))       # maxRescueAngle, °
        w.u16(self._gps_rescue_setting("gps_rescue_return_alt"))      # returnAltitudeM
        w.u16(self._gps_rescue_setting("gps_rescue_descent_dist"))    # descentDistanceM
        w.u16(self._gps_rescue_setting("gps_rescue_ground_speed"))    # groundSpeedCmS
        w.u16(self._gps_rescue_setting("gps_rescue_throttle_min"))    # throttleMin
        w.u16(self._gps_rescue_setting("gps_rescue_throttle_max"))    # throttleMax
        w.u16(self._gps_rescue_setting("gps_rescue_throttle_hover"))  # throttleHover
        w.u8(self._gps_rescue_setting("gps_rescue_sanity_checks"))    # sanityChecks
        w.u8(self._gps_rescue_setting("gps_rescue_min_sats"))         # minSats
        w.u16(self._gps_rescue_setting("gps_rescue_ascend_rate"))     # ascendRate, см/с
        w.u16(self._gps_rescue_setting("gps_rescue_descend_rate"))    # descendRate, см/с
        w.u8(self._gps_rescue_setting("gps_rescue_allow_arming_without_fix"))
        w.u8(self._gps_rescue_setting("gps_rescue_alt_mode"))
        w.u16(self._gps_rescue_setting("gps_rescue_min_start_dist"))  # minStartDistM
        w.u16(self._gps_rescue_setting("gps_rescue_initial_climb"))   # initialClimbM (API 1.46+)

    def _gps_rescue_setting(self, name) -> int:
        entry = self.settings_by_name.get(name)
        default = entry["default"] if entry else 0
        return int(self.settings.get(name, default) or 0)

    def _h_set_gps_rescue(self, w, r):
        """MSP_SET_GPS_RESCUE (225): те же поля, что отдаёт чтение (msp.c 4.5.4)."""
        data = bytes(r.rest())
        if len(data) < 16:
            return b""
        pairs = [
            ("gps_rescue_max_angle", int.from_bytes(data[0:2], "little")),
            ("gps_rescue_return_alt", int.from_bytes(data[2:4], "little")),
            ("gps_rescue_descent_dist", int.from_bytes(data[4:6], "little")),
            ("gps_rescue_ground_speed", int.from_bytes(data[6:8], "little")),
            ("gps_rescue_throttle_min", int.from_bytes(data[8:10], "little")),
            ("gps_rescue_throttle_max", int.from_bytes(data[10:12], "little")),
            ("gps_rescue_throttle_hover", int.from_bytes(data[12:14], "little")),
            ("gps_rescue_sanity_checks", data[14]),
            ("gps_rescue_min_sats", data[15]),
        ]
        # Поля добавлялись по мере роста API: 1.43 — скорости и режим, 1.44 — дистанция старта,
        # 1.46 — высота начального набора. Payload короче — просто не трогаем эти настройки.
        if len(data) >= 20:
            pairs += [
                ("gps_rescue_ascend_rate", int.from_bytes(data[16:18], "little")),
                ("gps_rescue_descend_rate", int.from_bytes(data[18:20], "little")),
            ]
        if len(data) >= 22:
            pairs += [
                ("gps_rescue_allow_arming_without_fix", data[20]),
                ("gps_rescue_alt_mode", data[21]),
            ]
        if len(data) >= 24:
            pairs.append(("gps_rescue_min_start_dist", int.from_bytes(data[22:24], "little")))
        if len(data) >= 26:
            pairs.append(("gps_rescue_initial_climb", int.from_bytes(data[24:26], "little")))
        for name, value in pairs:
            self._store_setting(name, value)
        self._log(
            "[MSP] GPS_RESCUE сохранён: возврат "
            f"{self._gps_rescue_setting('gps_rescue_return_alt')} м, газ "
            f"{self._gps_rescue_setting('gps_rescue_throttle_min')}…"
            f"{self._gps_rescue_setting('gps_rescue_throttle_max')}"
        )
        return b""

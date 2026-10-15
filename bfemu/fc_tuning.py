"""PID, рейты и фильтры. Методы VirtualFC.

Примесь использует общее состояние контроллера; создание объекта — через bfemu.fc.VirtualFC.
"""
from __future__ import annotations

from .fc_constants import (
    BATTERYPROFILE_MASK,
    BATTERY_PROFILE_COUNT,
    CONTROL_RATE_PROFILE_COUNT,
    DTERM_LPF1_DYN_MAX_HZ_DEFAULT,
    DTERM_LPF1_DYN_MIN_HZ_DEFAULT,
    DTERM_LPF2_HZ_DEFAULT,
    DYN_LPF_MAX_HZ,
    F_GAIN_MAX,
    GYRO_LPF1_DYN_MAX_HZ_DEFAULT,
    GYRO_LPF1_DYN_MIN_HZ_DEFAULT,
    GYRO_LPF2_HZ_DEFAULT,
    LPF_MAX_HZ,
    PID_GAIN_MAX,
    PID_ITEM_COUNT,
    PID_NAMES,
    PID_PROFILE_ALIASES,
    PID_PROFILE_COUNT,
    PID_PROFILE_DEFAULTS,
    RATEPROFILE_MASK,
    RATES_TYPE_ACTUAL,
    RATE_PROFILE_DEFAULTS,
    SIMPLIFIED_TUNING_OFF,
)


class TuningMixin:
    # Протоколы моторов, для которых шим не синхронизирован (analog), — как motorPwmProtocol_e.
    UNSYNCED_PROTOCOLS = {0, 1, 2, 3, 4}  # PWM, ONESHOT125, ONESHOT42, MULTISHOT, BRUSHED
    DEBUG_COUNT = 90  # DEBUG_COUNT из build/debug.h (4.5.4)

    def _h_rc_tuning(self, w, r):
        """MSP_RC_TUNING (111): 23 байта — раскладка sbufWrite* в msp.c 4.5.4.

        Значения — из настроек профиля рейтов (`roll_rc_rate`, `roll_expo`, `roll_srate`, …),
        то есть ровно те, что видны в CLI через 'get roll_rc_rate'.
        """
        s = lambda n, d=0: int(self.settings.get(n, d))
        w.u8(s("roll_rc_rate", 7)).u8(s("roll_expo", 0))
        w.u8(s("roll_srate", 67)).u8(s("pitch_srate", 67)).u8(s("yaw_srate", 67))
        w.u8(0)  # был tpa_rate (переехал в PID-профиль)
        w.u8(s("thr_mid", 50)).u8(s("thr_expo", 0))
        w.u16(0)  # был tpa_breakpoint (переехал в PID-профиль)
        w.u8(s("yaw_expo", 0)).u8(s("yaw_rc_rate", 7)).u8(s("pitch_rc_rate", 7)).u8(s("pitch_expo", 0))
        w.u8(s("throttle_limit_type", 0)).u8(s("throttle_limit_percent", 100))
        w.u16(s("roll_rate_limit", 1998)).u16(s("pitch_rate_limit", 1998)).u16(s("yaw_rate_limit", 1998))
        w.u8(s("rates_type", RATES_TYPE_ACTUAL))

    def _h_set_rc_tuning(self, w, r):
        """MSP_SET_RC_TUNING (204): вкладка «Rates» пишет те же 23 байта."""
        d = bytes(r.rest())
        if len(d) < 23:
            return b""
        # Смещения — как в MSP_RC_TUNING из msp.c 4.5.4 (8-9 и 16-21 — двухбайтовые поля).
        self.settings["roll_rc_rate"], self.settings["roll_expo"] = d[0], d[1]
        self.settings["roll_srate"], self.settings["pitch_srate"], self.settings["yaw_srate"] = d[2], d[3], d[4]
        self.settings["thr_mid"], self.settings["thr_expo"] = d[6], d[7]
        self.settings["yaw_expo"] = d[10]
        self.settings["yaw_rc_rate"], self.settings["pitch_rc_rate"], self.settings["pitch_expo"] = d[11], d[12], d[13]
        self.settings["throttle_limit_type"], self.settings["throttle_limit_percent"] = d[14], d[15]
        for index, name in enumerate(("roll_rate_limit", "pitch_rate_limit", "yaw_rate_limit")):
            self.settings[name] = int.from_bytes(d[16 + index * 2:18 + index * 2], "little")
        self.settings["rates_type"] = d[22]
        self._save_active_rate_profile()
        self._log(f"[MSP] RC_TUNING сохранён: rates_type={self.settings['rates_type']}, "
                  f"roll_rc_rate={self.settings['roll_rc_rate']}")
        return b""

    def _h_pid_advanced(self, w, r):
        """MSP_PID_ADVANCED (94): 61 байт — ровно как в msp.c 4.5.4 (порядок полей фиксирован).

        Configurator читает их в том же порядке (MSPHelper.js), поэтому раскладку нельзя менять:
        TPA, «Ослабление I», антигравитация, вращение I, Feedforward, D Max, «Другие настройки».
        """
        s = lambda n, d=0: int(self.settings.get(n, PID_PROFILE_DEFAULTS.get(n, d)))
        w.u16(0).u16(0).u16(0)                        # устаревшие rollPitch/yaw iterm ignore, yaw_p_limit
        w.u8(0).u8(0)                                 # зарезервировано, был vbatPidCompensation
        w.u8(s("feedforward_transition"))
        w.u8(0).u8(0).u8(0).u8(0)                     # зарезервировано (был dtermSetpointWeight)
        w.u16(s("acc_limit")).u16(s("acc_limit_yaw"))
        w.u8(s("angle_limit")).u8(0)                  # «Макс. угол наклона» (была levelSensitivity)
        w.u16(0)                                      # был itermThrottleThreshold
        w.u16(s("anti_gravity_gain"))
        w.u16(0)                                      # был dtermSetpointWeight
        w.u8(s("iterm_rotation")).u8(0)               # был smart_feedforward
        w.u8(s("iterm_relax")).u8(s("iterm_relax_type"))
        w.u8(s("abs_control_gain")).u8(s("throttle_boost")).u8(s("acro_trainer_angle_limit"))
        w.u16(s("f_roll")).u16(s("f_pitch")).u16(s("f_yaw"))   # Feedforward по осям
        w.u8(0)                                       # был antiGravityMode
        w.u8(s("d_min_roll")).u8(s("d_min_pitch")).u8(s("d_min_yaw"))
        w.u8(s("d_max_gain")).u8(s("d_max_advance"))
        w.u8(s("use_integrated_yaw")).u8(s("integrated_yaw_relax"))
        w.u8(s("iterm_relax_cutoff"))
        w.u8(s("motor_output_limit")).u8(s("auto_profile_cell_count") & 0xFF)
        w.u8(s("dyn_idle_min_rpm"))
        w.u8(s("feedforward_averaging")).u8(s("feedforward_smooth_factor")).u8(s("feedforward_boost"))
        w.u8(s("feedforward_max_rate_limit")).u8(s("feedforward_jitter_factor"))
        w.u8(s("vbat_sag_compensation")).u8(s("thrust_linear"))
        w.u8(s("tpa_mode")).u8(s("tpa_rate")).u16(s("tpa_breakpoint"))

    def _h_set_pid_advanced(self, w, r):
        """MSP_SET_PID_ADVANCED (95) — разбор тех же 61 байта (msp.c читает поля по порядку)."""
        d = bytes(r.rest())
        if len(d) < 61:
            return b""
        def u16(off):
            return int.from_bytes(d[off:off + 2], "little")
        # Смещения — ровно те, что собирает crunch(MSP_SET_PID_ADVANCED) в MSPHelper.js:
        # 0-5 три устаревших u16, 6 deltaMethod, 7 vbatPidCompensation, 8 feedforward_transition,
        # 9 старый dtermSetpointWeight (байт), 10-12 прочие устаревшие байты, далее — как ниже.
        self.settings["feedforward_transition"] = d[8]
        self.settings["acc_limit"], self.settings["acc_limit_yaw"] = u16(13), u16(15)
        self.settings["angle_limit"] = d[17]
        self.settings["anti_gravity_gain"] = u16(21)
        self.settings["iterm_rotation"] = d[25]
        self.settings["iterm_relax"] = d[27]
        self.settings["iterm_relax_type"] = d[28]
        self.settings["abs_control_gain"] = d[29]
        self.settings["throttle_boost"] = d[30]
        self.settings["acro_trainer_angle_limit"] = d[31]
        self.settings["f_roll"], self.settings["f_pitch"], self.settings["f_yaw"] = u16(32), u16(34), u16(36)
        self.settings["d_min_roll"], self.settings["d_min_pitch"], self.settings["d_min_yaw"] = d[39], d[40], d[41]
        self.settings["d_max_gain"], self.settings["d_max_advance"] = d[42], d[43]
        self.settings["use_integrated_yaw"], self.settings["integrated_yaw_relax"] = d[44], d[45]
        self.settings["iterm_relax_cutoff"] = d[46]
        self.settings["motor_output_limit"] = d[47]
        self.settings["auto_profile_cell_count"] = d[48] - 256 if d[48] > 127 else d[48]
        self.settings["dyn_idle_min_rpm"] = d[49]
        self.settings["feedforward_averaging"] = d[50]
        self.settings["feedforward_smooth_factor"] = d[51]
        self.settings["feedforward_boost"] = d[52]
        self.settings["feedforward_max_rate_limit"] = d[53]
        self.settings["feedforward_jitter_factor"] = d[54]
        self.settings["vbat_sag_compensation"] = d[55]
        self.settings["thrust_linear"] = d[56]
        self.settings["tpa_mode"] = d[57]
        self.settings["tpa_rate"] = d[58]
        self.settings["tpa_breakpoint"] = u16(59)
        self._save_active_pid_profile()
        self._log(
            f"[MSP] PID_ADVANCED сохранён: TPA {self.settings['tpa_mode']}/{self.settings['tpa_rate']}% "
            f"@ {self.settings['tpa_breakpoint']}, угол {self.settings['angle_limit']}, "
            f"feedforward boost {self.settings['feedforward_boost']}"
        )
        return b""

    def _h_pid(self, w, r):
        """MSP_PID (112): PID_ITEM_COUNT × 3 байта — P, I, D по осям ROLL/PITCH/YAW/LEVEL/MAG.

        Значения берём из тех же настроек, что видит CLI (`p_roll`, `i_roll`, `d_roll`, …):
        MspHelper.js раскладывает ответ по строкам таблицы «PID-настройки».
        """
        for axis in ("roll", "pitch", "yaw", "level", "mag"):
            w.u8(self._pid_value(f"p_{axis}")).u8(self._pid_value(f"i_{axis}")).u8(self._pid_value(f"d_{axis}"))

    def _h_set_pid(self, w, r):
        """MSP_SET_PID (202): 15 байт P/I/D по осям — та же раскладка, что у MSP_PID."""
        data = bytes(r.rest())
        if len(data) < PID_ITEM_COUNT * 3:
            return b""
        axes = ("roll", "pitch", "yaw", "level", "mag")
        for index, axis in enumerate(axes):
            base = index * 3
            for prefix, offset in (("p", 0), ("i", 1), ("d", 2)):
                name = f"{prefix}_{axis}"
                key = PID_PROFILE_ALIASES.get(name, name)
                self.settings[key] = data[base + offset]
        self._save_active_pid_profile()
        self._log(
            "[MSP] PID сохранены: roll {}/{}/{}, pitch {}/{}/{}, yaw {}/{}/{}".format(
                *[self.settings[f"{p}_roll"] for p in "pid"],
                *[self.settings[f"{p}_pitch"] for p in "pid"],
                *[self.settings[f"{p}_yaw"] for p in "pid"],
            )
        )
        return b""   # пустой ACK: вкладка ждёт ответ на каждую запись

    def _h_pidnames(self, w, r):
        """MSP_PIDNAMES (117): имена PID-осей строкой, как pidNames[] в flight/pid.c."""
        w.raw(PID_NAMES.encode())

    def _pid_value(self, name):
        """Значение PID-параметра.

        ROLL/PITCH/YAW — обычные настройки профиля (`p_roll` и т.д.). Строка LEVEL — те же поля,
        что и `angle_p_gain`/`horizon_level_strength`/`horizon_limit_sticks`. MAG в 4.5.4 не имеет
        настроек CLI, поэтому берём значение профиля (40/0/0 по умолчанию).
        """
        alias = PID_PROFILE_ALIASES.get(name)
        if alias and alias in self.settings:
            return int(self.settings[alias])
        default = PID_PROFILE_DEFAULTS.get(name, 0)
        if name in self.settings:
            return int(self.settings[name])
        return int(self.pid_profiles[self.current_profile].get(name, default))

    def _save_active_pid_profile(self):
        """Переносит значения активного профиля из настроек в хранилище профилей."""
        profile = self.pid_profiles[self.current_profile]
        for name in PID_PROFILE_DEFAULTS:
            key = PID_PROFILE_ALIASES.get(name, name)
            if key in self.settings:
                profile[name] = int(self.settings[key])

    def _h_arming_config(self, w, r):
        """MSP_ARMING_CONFIG: auto_disarm_delay, (бывший kill switch), small_angle — 3 байта."""
        w.u8(int(self.settings.get("auto_disarm_delay", 5))).u8(0)
        w.u8(int(self.settings.get("small_angle", 25)))

    def _h_set_arming_config(self, w, r):
        """MSP_SET_ARMING_CONFIG: те же значения, что настройки auto_disarm_delay/small_angle."""
        self.settings["auto_disarm_delay"] = r.u8()
        r.u8()  # устаревший kill switch
        self.settings["small_angle"] = r.u8()
        self._log(
            f"[MSP] arming: auto_disarm_delay={self.settings['auto_disarm_delay']}, "
            f"small_angle={self.settings['small_angle']}"
        )

    def _h_loop_time(self, w, r):
        w.u16(125)

    def _h_misc(self, w, r):
        w.u16(int(self.settings.get("midrc", 1500)))
        w.u16(int(self.settings.get("min_throttle", 1070)))
        w.u16(int(self.settings.get("max_throttle", 2000)))
        w.u16(int(self.settings.get("min_command", 1000)))
        w.u16(int(self.settings.get("failsafe_throttle", 1000)))
        w.u8(int(self.settings.get("gps_provider", 1)))
        w.u8(int(self.settings.get("gps_baudrate", 0)))
        w.u8(int(self.settings.get("gps_ublox_use_galileo", 0)))
        w.u8(0).u8(int(self.settings.get("rssi_channel", 0))).u8(0)
        w.u16(0)
        w.u8(int(self.settings.get("vbat_scale", 110)))
        w.u8(self._battery_setting("vbat_min_cell_voltage") // 10)
        w.u8(self._battery_setting("vbat_max_cell_voltage") // 10)
        w.u8(self._battery_setting("vbat_warning_cell_voltage") // 10)

    # ------------------------------------------------- слайдеры (упрощённая настройка)
    def _slider(self, name):
        return int(self.settings.get(name, PID_PROFILE_DEFAULTS.get(name, 0)))

    def _simplified_pids_bytes(self):
        """17 байт «Ползунков настройки PID»: 9 значений + 8 зарезервированных (writeSimplifiedPids)."""
        keep = lambda v: max(0, min(255, int(v)))
        return bytes([
            keep(self._slider("simplified_pids_mode")),
            keep(self._slider("simplified_master_multiplier")),
            keep(self._slider("simplified_pitch_d_gain")),      # simplified_roll_pitch_ratio
            keep(self._slider("simplified_i_gain")),
            keep(self._slider("simplified_d_gain")),
            keep(self._slider("simplified_pi_gain")),
            keep(self._slider("simplified_dmax_gain")),         # simplified_dmin_ratio
            keep(self._slider("simplified_feedforward_gain")),
            keep(self._slider("simplified_pitch_pi_gain")),
        ] + [0] * 8)                                     # 8 байт зарезервировано, как в msp.c

    def _simplified_dterm_bytes(self):
        """18 байт упрощённого D-term фильтра (writeSimplifiedDtermFilters)."""
        s = lambda n, d=0: int(self.settings.get(n, d))
        return (bytes([1 if s("simplified_dterm_filter", 1) else 0,
                       max(0, min(255, s("simplified_dterm_filter_multiplier", 100)))])
                + s("dterm_lpf1_static_hz", 75).to_bytes(2, "little")
                + s("dterm_lpf2_static_hz", 150).to_bytes(2, "little")
                + s("dterm_lpf1_dyn_min_hz", 75).to_bytes(2, "little")
                + s("dterm_lpf1_dyn_max_hz", 150).to_bytes(2, "little")
                + bytes(8))                                   # зарезервировано

    def _simplified_gyro_bytes(self):
        """18 байт упрощённого фильтра гироскопа (writeSimplifiedGyroFilters)."""
        s = lambda n, d=0: int(self.settings.get(n, d))
        return (bytes([1 if s("simplified_gyro_filter", 1) else 0,
                       max(0, min(255, s("simplified_gyro_filter_multiplier", 100)))])
                + s("gyro_lpf1_static_hz", 250).to_bytes(2, "little")
                + s("gyro_lpf2_static_hz", 500).to_bytes(2, "little")
                + s("gyro_lpf1_dyn_min_hz", 250).to_bytes(2, "little")
                + s("gyro_lpf1_dyn_max_hz", 500).to_bytes(2, "little")
                + bytes(8))                                   # зарезервировано

    def _h_simplified_tuning(self, w, r):
        """MSP_SIMPLIFIED_TUNING (140): ползунки PID + фильтры D-term и гироскопа (53 байта)."""
        w.raw(self._simplified_pids_bytes() + self._simplified_dterm_bytes() + self._simplified_gyro_bytes())

    def _h_set_simplified_tuning(self, w, r):
        """MSP_SET_SIMPLIFIED_TUNING (141): сохранить ползунки и применить их (applySimplifiedTuning)."""
        d = bytes(r.rest())
        # Configurator шлёт 35 байт (17 ползунков PID + 18 D-term); часть фильтра гироскопа
        # (ещё 18 байт) добавляется только при сохранении из вкладки «Фильтры».
        if len(d) < 35:
            return b""
        names = ["simplified_pids_mode", "simplified_master_multiplier", "simplified_pitch_d_gain",
                 "simplified_i_gain", "simplified_d_gain", "simplified_pi_gain", "simplified_dmax_gain",
                 "simplified_feedforward_gain", "simplified_pitch_pi_gain"]
        for index, name in enumerate(names):
            if name in self.settings:
                self.settings[name] = d[index]
        self.settings["simplified_dterm_filter"] = 1 if d[17] else 0
        self.settings["simplified_dterm_filter_multiplier"] = d[18]
        if len(d) >= 53:                     # полный набор: PID (17) + D-term (18) + гироскоп (18)
            self.settings["simplified_gyro_filter"] = 1 if d[35] else 0
            self.settings["simplified_gyro_filter_multiplier"] = d[36]
        self._apply_simplified_tuning()
        self._save_active_pid_profile()
        self._log("[MSP] SIMPLIFIED_TUNING сохранены: режим "
                  f"{self.settings.get('simplified_pids_mode')}, "
                  f"master {self.settings.get('simplified_master_multiplier')}")
        return b""

    def _h_calculate_simplified_pid(self, w, r):
        """MSP_CALCULATE_SIMPLIFIED_PID (142): посчитать PID по ползункам во временном профиле."""
        d = bytes(r.rest())
        if len(d) >= 9:
            names = ["simplified_pids_mode", "simplified_master_multiplier", "simplified_pitch_d_gain",
                     "simplified_i_gain", "simplified_d_gain", "simplified_pi_gain", "simplified_dmax_gain",
                     "simplified_feedforward_gain", "simplified_pitch_pi_gain"]
            saved = {name: self.settings.get(name) for name in names}
            for index, name in enumerate(names):
                self.settings[name] = d[index]
            result = self._calculate_simplified_pids()
            for name, value in saved.items():
                self.settings[name] = value
        else:
            result = self._calculate_simplified_pids()
        for axis in ("roll", "pitch", "yaw"):
            w.u8(result[f"p_{axis}"]).u8(result[f"i_{axis}"]).u8(result[f"d_{axis}"])
            w.u8(result[f"d_min_{axis}"])
            w.u16(result[f"f_{axis}"])

    def _h_calculate_simplified_gyro(self, w, r):
        """MSP_CALCULATE_SIMPLIFIED_GYRO (143): пересчитать фильтры гироскопа по мультипликатору."""
        d = bytes(r.rest())
        saved = {n: self.settings.get(n) for n in ("simplified_gyro_filter", "simplified_gyro_filter_multiplier")}
        if len(d) >= 18:
            self.settings["simplified_gyro_filter"] = 1 if d[0] else 0
            self.settings["simplified_gyro_filter_multiplier"] = d[1]
        self._calculate_simplified_gyro_filters()
        w.raw(self._simplified_gyro_bytes())
        for name, value in saved.items():
            self.settings[name] = value

    def _h_calculate_simplified_dterm(self, w, r):
        """MSP_CALCULATE_SIMPLIFIED_DTERM (144): пересчитать D-term фильтры по мультипликатору."""
        d = bytes(r.rest())
        saved = {n: self.settings.get(n) for n in ("simplified_dterm_filter", "simplified_dterm_filter_multiplier")}
        if len(d) >= 18:
            self.settings["simplified_dterm_filter"] = 1 if d[0] else 0
            self.settings["simplified_dterm_filter_multiplier"] = d[1]
        self._calculate_simplified_dterm_filters()
        w.raw(self._simplified_dterm_bytes())
        for name, value in saved.items():
            self.settings[name] = value

    def _h_validate_simplified_tuning(self, w, r):
        """MSP_VALIDATE_SIMPLIFIED_TUNING (145): 3 байта — совпадают ли PID/фильтры с ползунками.

        Configurator по этим флагам выключает ползунки, если значения разошлись с профилем.
        """
        pids = self._calculate_simplified_pids()
        pids_ok = all(int(self.settings.get(f"{p}_{axis}", 0)) == pids[f"{p}_{axis}"]
                      for axis in ("roll", "pitch", "yaw") for p in ("p", "i", "d"))
        pids_ok = pids_ok and all(int(self.settings.get(f"d_min_{axis}", 0)) == pids[f"d_min_{axis}"]
                                  for axis in ("roll", "pitch", "yaw"))
        pids_ok = pids_ok and all(int(self.settings.get(f"f_{axis}", 0)) == pids[f"f_{axis}"]
                                  for axis in ("roll", "pitch", "yaw"))
        gyro = self._calculate_simplified_gyro_filters(apply=False)
        gyro_ok = all(int(self.settings.get(name, 0)) == value for name, value in gyro.items())
        dterm = self._calculate_simplified_dterm_filters(apply=False)
        dterm_ok = all(int(self.settings.get(name, 0)) == value for name, value in dterm.items())
        w.u8(1 if pids_ok else 0).u8(1 if gyro_ok else 0).u8(1 if dterm_ok else 0)

    # --- арифметика ползунков: порт config/simplified_tuning.c прошивки 4.5.4
    _PID_DEFAULTS_FOR_SLIDERS = {
        "roll": (45, 80, 40, 120), "pitch": (47, 84, 46, 125), "yaw": (45, 80, 0, 120),
    }
    _D_MIN_DEFAULTS_FOR_SLIDERS = {"roll": 30, "pitch": 34, "yaw": 0}

    def _calculate_simplified_pids(self) -> dict:
        """calculateNewPidValues(): PID = заводские × множители ползунков (без записи в настройки)."""
        mode = self._slider("simplified_pids_mode")
        master = self._slider("simplified_master_multiplier") / 100.0
        pi = self._slider("simplified_pi_gain") / 100.0
        d_gain = self._slider("simplified_d_gain") / 100.0
        ff = self._slider("simplified_feedforward_gain") / 100.0
        i_gain = self._slider("simplified_i_gain") / 100.0
        ratio = self._slider("simplified_pitch_d_gain") / 100.0
        pitch_pi = self._slider("simplified_pitch_pi_gain") / 100.0
        dmin_ratio = self._slider("simplified_dmax_gain") / 100.0
        clamp = lambda v, hi: max(0, min(hi, int(v)))
        result = {}
        for index, axis in enumerate(("roll", "pitch", "yaw")):
            if index > mode:                          # mode 0 = выключено, 1 = RP, 2 = RPY
                result[f"p_{axis}"] = int(self.settings.get(f"p_{axis}", 0))
                result[f"i_{axis}"] = int(self.settings.get(f"i_{axis}", 0))
                result[f"d_{axis}"] = int(self.settings.get(f"d_{axis}", 0))
                result[f"d_min_{axis}"] = int(self.settings.get(f"d_min_{axis}", 0))
                result[f"f_{axis}"] = int(self.settings.get(f"f_{axis}", 0))
                continue
            pitch_d = ratio if axis == "pitch" else 1.0
            pitch_pi_factor = pitch_pi if axis == "pitch" else 1.0
            p_default, i_default, d_default, f_default = self._PID_DEFAULTS_FOR_SLIDERS[axis]
            d_min_default = self._D_MIN_DEFAULTS_FOR_SLIDERS[axis]
            result[f"p_{axis}"] = clamp(p_default * master * pi * pitch_pi_factor, PID_GAIN_MAX)
            result[f"i_{axis}"] = clamp(i_default * master * pi * i_gain * pitch_pi_factor, PID_GAIN_MAX)
            if d_min_default > 0:
                dmin_k = 1.0 + ((d_default - d_min_default) / d_min_default) * dmin_ratio
            else:
                dmin_k = 1.0
            result[f"d_{axis}"] = clamp(d_min_default * master * d_gain * pitch_d * dmin_k, PID_GAIN_MAX)
            result[f"d_min_{axis}"] = clamp(d_min_default * master * d_gain * pitch_d, PID_GAIN_MAX)
            result[f"f_{axis}"] = clamp(f_default * master * pitch_pi_factor * ff, F_GAIN_MAX)
        return result

    def _calculate_simplified_dterm_filters(self, apply=True) -> dict:
        """calculateNewDTermFilterValues(): фильтры D-term от мультипликатора ползунка."""
        multiplier = self._slider("simplified_dterm_filter_multiplier") / 100.0
        out = {}
        if int(self.settings.get("simplified_dterm_filter", 1)) == 0:
            return out
        if int(self.settings.get("dterm_lpf1_dyn_min_hz", 0)):
            out["dterm_lpf1_dyn_min_hz"] = min(DYN_LPF_MAX_HZ, int(DTERM_LPF1_DYN_MIN_HZ_DEFAULT * multiplier))
            out["dterm_lpf1_dyn_max_hz"] = min(DYN_LPF_MAX_HZ, int(DTERM_LPF1_DYN_MAX_HZ_DEFAULT * multiplier))
        if int(self.settings.get("dterm_lpf1_static_hz", 0)):
            out["dterm_lpf1_static_hz"] = min(DYN_LPF_MAX_HZ, int(DTERM_LPF1_DYN_MIN_HZ_DEFAULT * multiplier))
        if int(self.settings.get("dterm_lpf2_static_hz", 0)):
            out["dterm_lpf2_static_hz"] = min(LPF_MAX_HZ, int(DTERM_LPF2_HZ_DEFAULT * multiplier))
        if apply:
            self.settings.update(out)
        return out

    def _calculate_simplified_gyro_filters(self, apply=True) -> dict:
        """calculateNewGyroFilterValues(): фильтры гироскопа от мультипликатора ползунка."""
        multiplier = self._slider("simplified_gyro_filter_multiplier") / 100.0
        out = {}
        if int(self.settings.get("simplified_gyro_filter", 1)) == 0:
            return out
        if int(self.settings.get("gyro_lpf1_dyn_min_hz", 0)):
            out["gyro_lpf1_dyn_min_hz"] = min(DYN_LPF_MAX_HZ, int(GYRO_LPF1_DYN_MIN_HZ_DEFAULT * multiplier))
            out["gyro_lpf1_dyn_max_hz"] = min(DYN_LPF_MAX_HZ, int(GYRO_LPF1_DYN_MAX_HZ_DEFAULT * multiplier))
        if int(self.settings.get("gyro_lpf1_static_hz", 0)):
            out["gyro_lpf1_static_hz"] = min(DYN_LPF_MAX_HZ, int(GYRO_LPF1_DYN_MIN_HZ_DEFAULT * multiplier))
        if int(self.settings.get("gyro_lpf2_static_hz", 0)):
            out["gyro_lpf2_static_hz"] = min(LPF_MAX_HZ, int(GYRO_LPF2_HZ_DEFAULT * multiplier))
        if apply:
            self.settings.update(out)
        return out

    def _apply_simplified_tuning(self):
        """applySimplifiedTuning(): пересчитать и записать PID и фильтры по ползункам."""
        if self._slider("simplified_pids_mode") != SIMPLIFIED_TUNING_OFF:
            self.settings.update(self._calculate_simplified_pids())
        self._calculate_simplified_dterm_filters()
        self._calculate_simplified_gyro_filters()

    # --------------------------------------------------- профили PID и профили рейтов
    def _save_active_rate_profile(self):
        """Переносит значения активного профиля рейтов из настроек в хранилище профилей."""
        profile = self.rate_profiles[self.current_rate_profile]
        for name in RATE_PROFILE_DEFAULTS:
            if name in self.settings:
                profile[name] = int(self.settings[name])

    def get_battery_profile_index(self) -> int:
        """getCurrentBatteryProfileIndex() мастера: активный профиль батареи (systemConfig)."""
        return self.active_battery_profile

    def change_battery_profile(self, index: int):
        """changeBatteryProfile() мастера: вне 0…BATTERY_PROFILE_COUNT-1 индекс не меняется."""
        if 0 <= index < BATTERY_PROFILE_COUNT:
            self.active_battery_profile = index
        self._log(f"[FC] профиль батареи выбран: {self.active_battery_profile}")

    def select_profile(self, value: int):
        """MSP_SELECT_SETTING (210): профиль PID, рейтов (бит 0x80) или батареи (бит 0x40).

        Разбор и ограничения — как в msp.c мастера: при батарейном бите значение вне
        диапазона приводится к 0, остальные ветки — как раньше.
        """
        if value & BATTERYPROFILE_MASK:
            index = value & ~BATTERYPROFILE_MASK
            if index >= BATTERY_PROFILE_COUNT:
                index = 0
            self.change_battery_profile(index)
            return
        if value & RATEPROFILE_MASK:
            index = value & ~RATEPROFILE_MASK
            if index >= CONTROL_RATE_PROFILE_COUNT:
                index = 0
            self._save_active_rate_profile()
            self.current_rate_profile = index
            for name, default in self.rate_profiles[index].items():
                self.settings[name] = default
        else:
            index = value if value < PID_PROFILE_COUNT else 0
            self._save_active_pid_profile()
            self.current_profile = index
            for name, default in self.pid_profiles[index].items():
                key = PID_PROFILE_ALIASES.get(name, name)
                self.settings[key if key in self.settings else name] = default
        self._log(f"[FC] профиль выбран: PID {self.current_profile}, рейты {self.current_rate_profile}")

    def _h_select_setting(self, w, r):
        value = r.u8() if len(r.data) else 0
        self.select_profile(value)
        return b""

    def _h_copy_profile(self, w, r):
        """MSP_COPY_PROFILE (183): 3 байта — тип (0 = PID, 1 = рейты), приёмник, источник."""
        d = bytes(r.rest())
        if len(d) < 3:
            return b""
        kind, dst, src = d[0], d[1], d[2]
        if kind == 0 and src < PID_PROFILE_COUNT and dst < PID_PROFILE_COUNT:
            self._save_active_pid_profile()
            self.pid_profiles[dst] = dict(self.pid_profiles[src])
            if dst == self.current_profile:
                for name, value in self.pid_profiles[dst].items():
                    self.settings[name] = value
            self._log(f"[FC] профиль PID {src + 1} скопирован в {dst + 1}")
        elif kind == 1 and src < CONTROL_RATE_PROFILE_COUNT and dst < CONTROL_RATE_PROFILE_COUNT:
            self._save_active_rate_profile()
            self.rate_profiles[dst] = dict(self.rate_profiles[src])
            if dst == self.current_rate_profile:
                for name, value in self.rate_profiles[dst].items():
                    self.settings[name] = value
            self._log(f"[FC] профиль рейтов {src + 1} скопирован в {dst + 1}")
        return b""

    def _h_set_reset_curr_pid(self, w, r):
        """MSP_SET_RESET_CURR_PID (219): resetPidProfile() — только для текущего профиля."""
        profile = dict(PID_PROFILE_DEFAULTS)
        self.pid_profiles[self.current_profile] = dict(profile)
        for name, value in profile.items():
            key = PID_PROFILE_ALIASES.get(name, name)
            if key in self.settings:
                self.settings[key] = value
            elif name in self.settings:
                self.settings[name] = value
        self._log(f"[FC] профиль PID {self.current_profile + 1} сброшен к заводским")
        return b""

    def _h_advanced_config(self, w, r):
        """MSP_ADVANCED_CONFIG — 20 байт, как sbufWrite* в msp.c.

        Второй байт (pid_process_denom) задаёт частоту цикла PID в интерфейсе, поэтому
        берём его из настройки: get pid_process_denom показывает то же значение.
        """
        protocol = self._setting("motor_pwm_protocol", 7)
        w.u8(1)  # устаревшее gyro_sync_denom (прошивка всегда пишет 1)
        w.u8(self._setting("pid_process_denom", 1))
        w.u8(1 if protocol in self.UNSYNCED_PROTOCOLS else 0)
        w.u8(protocol)
        w.u16(self._setting("motor_pwm_rate", 480))
        w.u16(self._setting("dshot_idle_value", 550))  # digitalIdleOffsetValue
        w.u8(0)  # устаревшее gyro_use_32kHz
        w.u8(self._setting("motor_pwm_inversion", 0))
        w.u8(self._setting("gyro_to_use", 0))
        w.u8(self._setting("gyro_high_range", 0))          # gyro_high_fsr
        w.u8(self._setting("gyro_calib_noise_limit", 48))  # gyroMovementCalibrationThreshold
        w.u16(self._setting("gyro_calib_duration", 125))
        w.u16(self._setting("gyro_offset_yaw", 0) & 0xFFFF)
        w.u8(self._setting("gyro_overflow_detect", 1))
        w.u8(self._setting("debug_mode", 0))
        w.u8(self.DEBUG_COUNT)

    def _h_set_advanced_config(self, w, r):
        """MSP_SET_ADVANCED_CONFIG (19 байт из интерфейса): пишем то, что есть в CLI-настройках."""
        data = bytes(r.rest())
        if len(data) < 19:
            return
        self._store_setting("pid_process_denom", data[1])
        self._store_setting("motor_pwm_protocol", data[3])
        self._store_setting("motor_pwm_rate", int.from_bytes(data[4:6], "little"))
        self._store_setting("dshot_idle_value", int.from_bytes(data[6:8], "little"))
        self._store_setting("motor_pwm_inversion", data[9])
        self._store_setting("gyro_to_use", data[10])
        self._store_setting("gyro_high_range", data[11])
        self._store_setting("gyro_calib_noise_limit", data[12])
        self._store_setting("gyro_calib_duration", int.from_bytes(data[13:15], "little"))
        self._store_setting("gyro_offset_yaw", int.from_bytes(data[15:17], "little", signed=True))
        self._store_setting("gyro_overflow_detect", data[17])
        self._store_setting("debug_mode", data[18])
        self._log(
            "[MSP] ADVANCED_CONFIG сохранён: pid_process_denom={pid_process_denom}, "
            "motor_pwm_protocol={motor_pwm_protocol}, motor_pwm_rate={motor_pwm_rate}, "
            "dshot_idle_value={dshot_idle_value}, debug_mode={debug_mode}".format(**self.settings)
        )

    def _h_rc_deadband(self, w, r):
        """MSP_RC_DEADBAND (125): 5 байт — deadband, yaw_deadband, alt_hold_deadband, 3D-дедбенд.

        Заводские значения rcControlsConfig: deadband 0, yaw_deadband 0, alt_hold_deadband 40;
        flight3DConfig: deadband3d_throttle 50 (fc/rc_controls.c в 4.5.4).
        """
        s = lambda n, d=0: int(self.settings.get(n, d))
        w.u8(s("deadband", 0)).u8(s("yaw_deadband", 0)).u8(s("alt_hold_deadband", 40))
        w.u16(s("3d_deadband_throttle", 50))

    def _h_set_rc_deadband(self, w, r):
        """MSP_SET_RC_DEADBAND (218): 5 байт — та же раскладка, что у ответа 125."""
        d = bytes(r.rest())
        if len(d) < 5:
            return b""
        self.settings["deadband"], self.settings["yaw_deadband"] = d[0], d[1]
        self.settings["alt_hold_deadband"] = d[2]
        self._store_setting("3d_deadband_throttle", int.from_bytes(d[3:5], "little"))
        self._log(f"[MSP] RC_DEADBAND сохранены: deadband={self.settings['deadband']}, "
                  f"yaw_deadband={self.settings['yaw_deadband']}, 3D={self.settings['3d_deadband_throttle']}")
        return b""

    def _h_filter_config(self, w, r):
        """MSP_FILTER_CONFIG (92): 49 байт строго в порядке sbufWrite* из msp.c 4.5.4.

        Порядок фиксирован Configurator'ом (MSPHelper.js читает поля подряд), поэтому
        «улучшать» раскладку нельзя: гироскоп и D-term идут вперемешку.
        """
        s = lambda n, d=0: int(self.settings.get(n, d))
        w.u8(s("gyro_lpf1_static_hz", 250) & 0xFF)            # устаревший байт gyro_lpf1_static_hz
        w.u16(s("dterm_lpf1_static_hz", 75))
        w.u16(s("yaw_lowpass_hz", 100))
        w.u16(s("gyro_notch1_hz", 0)).u16(s("gyro_notch1_cutoff", 0))
        w.u16(s("dterm_notch_hz", 0)).u16(s("dterm_notch_cutoff", 0))
        w.u16(s("gyro_notch2_hz", 0)).u16(s("gyro_notch2_cutoff", 0))
        w.u8(s("dterm_lpf1_type", 0))
        w.u8(s("gyro_hardware_lpf", 0))
        w.u8(0)                                               # устаревший gyro_32khz_hardware_lpf
        w.u16(s("gyro_lpf1_static_hz", 250)).u16(s("gyro_lpf2_static_hz", 500))
        w.u8(s("gyro_lpf1_type", 0)).u8(s("gyro_lpf2_type", 0))
        w.u16(s("dterm_lpf2_static_hz", 150))
        w.u8(s("dterm_lpf2_type", 0))
        w.u16(s("gyro_lpf1_dyn_min_hz", 250)).u16(s("gyro_lpf1_dyn_max_hz", 500))
        w.u16(s("dterm_lpf1_dyn_min_hz", 75)).u16(s("dterm_lpf1_dyn_max_hz", 150))
        w.u8(0).u8(0)                                         # устаревшие dyn_notch_range/width
        w.u16(s("dyn_notch_q", 300)).u16(s("dyn_notch_min_hz", 100))
        w.u8(self._rpm_setting("rpm_filter_harmonics", 3))
        w.u8(self._rpm_setting("rpm_filter_min_hz", 100))
        w.u16(s("dyn_notch_max_hz", 600))
        w.u8(s("dterm_lpf1_dyn_expo", 5))
        w.u8(s("dyn_notch_count", 3))

    def _rpm_setting(self, name, default):
        """Настройки RPM-фильтра: в bf_data.json они носят имена gyro_rpm_notch_* (из msp.c)."""
        alias = {"rpm_filter_harmonics": "gyro_rpm_notch_harmonics",
                 "rpm_filter_min_hz": "gyro_rpm_notch_min_hz"}
        for key in (name, alias.get(name, name)):
            if key in self.settings:
                return int(self.settings[key])
        return default

    def _h_set_filter_config(self, w, r):
        """MSP_SET_FILTER_CONFIG (93): разбор той же раскладки (вкладки «Фильтры» и «PID»)."""
        d = bytes(r.rest())
        if len(d) < 49:
            return b""
        u16 = lambda o: int.from_bytes(d[o:o + 2], "little")
        self.settings["dterm_lpf1_static_hz"] = u16(1)
        self.settings["yaw_lowpass_hz"] = u16(3)
        self.settings["gyro_notch1_hz"], self.settings["gyro_notch1_cutoff"] = u16(5), u16(7)
        self.settings["dterm_notch_hz"], self.settings["dterm_notch_cutoff"] = u16(9), u16(11)
        self.settings["gyro_notch2_hz"], self.settings["gyro_notch2_cutoff"] = u16(13), u16(15)
        self.settings["dterm_lpf1_type"] = d[17]
        self.settings["gyro_hardware_lpf"] = d[18]
        self.settings["gyro_lpf1_static_hz"], self.settings["gyro_lpf2_static_hz"] = u16(20), u16(22)
        self.settings["gyro_lpf1_type"], self.settings["gyro_lpf2_type"] = d[24], d[25]
        self.settings["dterm_lpf2_static_hz"] = u16(26)
        self.settings["dterm_lpf2_type"] = d[28]
        self.settings["gyro_lpf1_dyn_min_hz"], self.settings["gyro_lpf1_dyn_max_hz"] = u16(29), u16(31)
        self.settings["dterm_lpf1_dyn_min_hz"], self.settings["dterm_lpf1_dyn_max_hz"] = u16(33), u16(35)
        self.settings["dyn_notch_q"], self.settings["dyn_notch_min_hz"] = u16(39), u16(41)
        self.settings["gyro_rpm_notch_harmonics"], self.settings["gyro_rpm_notch_min_hz"] = d[43], d[44]
        self.settings["dyn_notch_max_hz"] = u16(45)
        self.settings["dterm_lpf1_dyn_expo"] = d[47]
        self.settings["dyn_notch_count"] = d[48]
        self._save_active_pid_profile()
        self._log(f"[MSP] FILTER_CONFIG сохранён: dterm_lpf1={self.settings['dterm_lpf1_static_hz']}, "
                  f"gyro_notch1={self.settings['gyro_notch1_hz']}")
        return b""

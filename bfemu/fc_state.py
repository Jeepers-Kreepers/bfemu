"""Настройки и EEPROM. Методы VirtualFC.

Примесь использует общее состояние контроллера; создание объекта — через bfemu.fc.VirtualFC.
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from .fc_constants import (
    BAUD_RATES,
    CONTROL_RATE_PROFILE_COUNT,
    FEATURE_DEFAULTS,
    LED_CONFIGURABLE_COLOR_COUNT,
    LED_DIRECTION_COUNT,
    LED_MODE_COUNT,
    LED_SPECIAL_COLOR_COUNT,
    MAX_AUX_CHANNEL_COUNT,
    MAX_NAME_LENGTH,
    MAX_SUPPORTED_MOTORS,
    MSC_SECONDS,
    MSP_REBOOT_MSC,
    PID_PROFILE_COUNT,
    PID_PROFILE_DEFAULTS,
    PWM_PULSE_MAX,
    PWM_PULSE_MIN,
    RANGEFINDER_NONE,
    RATE_PROFILE_DEFAULTS,
    SENSOR_AUTO,
    SENSOR_NONE,
    SERIAL_FUNCTIONS,
    STRING_DEFAULTS,
    STRING_LENGTHS,
)


class StateMixin:

    # --------------------- таблицы команд CLI 4.5.4 во «флеше» (rxrange / led / color / mmix)
    def _load_cli_table(self, name: str, data):
        """Читает таблицы `rxrange`, `led`, `color`, `mode_color` и `mmix` из «флеша».

        Прошивка держит их в PG-таблицах (rxChannelRangeConfigs, ledStripStatusModeConfig,
        customMotorMixer), поэтому эмулятор хранит их рядом с остальной конфигурацией: битое
        и лишнее отбрасываем, недостающие записи добираем заводскими.
        """
        if name == "rx_ranges":
            ranges = self._default_rx_ranges()
            if isinstance(data, list):
                for index, item in enumerate(data[:len(ranges)]):
                    if isinstance(item, dict):
                        ranges[index] = {
                            "min": self._clamp_int(item.get("min"), PWM_PULSE_MIN, PWM_PULSE_MAX,
                                                   ranges[index]["min"]),
                            "max": self._clamp_int(item.get("max"), PWM_PULSE_MIN, PWM_PULSE_MAX,
                                                   ranges[index]["max"]),
                        }
            self.rx_ranges = ranges
        elif name == "led_configs":
            configs = self._default_led_configs()
            if isinstance(data, list):
                for index, item in enumerate(data[:len(configs)]):
                    if isinstance(item, str) and item:      # строка формата generateLedConfig()
                        configs[index] = item[:40]
            self.led_configs = configs
        elif name == "led_colors":
            colors = self._default_led_colors()
            if isinstance(data, list):
                for index, item in enumerate(data[:len(colors)]):
                    if isinstance(item, list) and len(item) == 3:
                        h = min(359, max(0, self._clamp_int(item[0], 0, 359, 0)))
                        colors[index] = [h, self._clamp_int(item[1], 0, 255, 0),
                                         self._clamp_int(item[2], 0, 255, 0)]
            self.led_colors = colors
        elif name == "mode_colors":
            colors = self._default_mode_colors()
            if isinstance(data, dict):
                mode = data.get("mode")
                if isinstance(mode, list):
                    for index, row in enumerate(mode[:LED_MODE_COUNT]):
                        if isinstance(row, list) and len(row) == LED_DIRECTION_COUNT:
                            colors["mode"][index] = [self._clamp_int(v, 0, LED_CONFIGURABLE_COLOR_COUNT - 1, 0)
                                                     for v in row]
                special = data.get("special")
                if isinstance(special, list) and len(special) >= LED_SPECIAL_COLOR_COUNT:
                    colors["special"] = [self._clamp_int(v, 0, LED_CONFIGURABLE_COLOR_COUNT - 1, 0)
                                        for v in special[:LED_SPECIAL_COLOR_COUNT]]
                colors["aux_channel"] = self._clamp_int(data.get("aux_channel"), 0, MAX_AUX_CHANNEL_COUNT,
                                                        colors["aux_channel"])
            self.mode_colors = colors
        elif name == "custom_motor_mix":
            mix = []
            if isinstance(data, list):
                for item in data[:MAX_SUPPORTED_MOTORS]:
                    if not isinstance(item, dict):
                        continue
                    entry = {}
                    for field in ("throttle", "roll", "pitch", "yaw"):
                        try:
                            entry[field] = float(item[field])
                        except (KeyError, TypeError, ValueError):
                            entry[field] = 0.0
                    if entry["throttle"] == 0.0:      # printMotorMix() останавливается здесь
                        break
                    mix.append(entry)
            self.custom_motor_mix = mix

    @staticmethod
    def _clamp_int(value, low: int, high: int, fallback: int) -> int:
        """Значение из «флеша» с ограничением: не число — берём заводское."""
        try:
            return max(low, min(high, int(value)))
        except (TypeError, ValueError):
            return fallback

    def _load_settings(self):
        for s in self.data["settings"]:
            entry = dict(s)
            if entry.get("mode") == "MODE_STRING":
                # MODE_STRING: у прошивки заданы min/max длины (cli/settings.c) — их в
                # таблице формата нет, дописываем по именам параметров.
                entry["string_min"], entry["string_max"] = STRING_LENGTHS.get(
                    entry["name"], (1, MAX_NAME_LENGTH))
                if entry.get("default") is None:
                    entry["default"] = self._string_default(entry["name"])
            elif entry.get("default") is None:
                entry["default"] = self._heuristic_default(entry)
            self.settings_list.append(entry)
            self.settings_by_name[entry["name"]] = entry
            self.settings[entry["name"]] = entry["default"]
        self._apply_overrides()
        self._apply_profile_defaults()
        # _defaults — заводские настройки прошивки (их же показывает 'diff all' и 'defaults'),
        # self.settings — активная конфигурация, которая читается из «флеша» при старте.
        self._defaults = dict(self.settings)

    def _apply_profile_defaults(self):
        """Заводские значения PID-профиля и профиля рейтов (resetPidProfile / pgResetFn_controlRateProfiles)."""
        for table in (PID_PROFILE_DEFAULTS, RATE_PROFILE_DEFAULTS):
            for name, value in table.items():
                if name in self.settings_by_name:
                    self.settings[name] = value

    def _apply_overrides(self):
        """Небольшая таблица «хорошо известных» заводских значений.

        Большая часть defaults извлекается автоматически из pgResetFn_*/PG_RESET_TEMPLATE,
        но некоторые значения задаются выражениями, которые не разбираются генератором.
        Значения ниже соответствуют стоковой прошивке Betaflight 4.5.x.
        """
        override_path = Path(__file__).resolve().parent / "data" / "fc_overrides.json"
        if not override_path.exists():
            return
        for name, value in json.loads(override_path.read_text()).items():
            entry = self.settings_by_name.get(name)
            if entry is None:
                continue
            if isinstance(value, str) and entry["mode"] == "MODE_LOOKUP" and entry.get("values"):
                value = self._parse_lookup(entry, value)
                if value is None:
                    continue
            self.settings[name] = value
            entry["default"] = value

    @staticmethod
    def _heuristic_default(entry):
        if entry["mode"] == "MODE_LOOKUP":
            return 0
        return entry["min"] if entry["min"] > 0 else 0

    @staticmethod
    def _string_default(name: str) -> str:
        """Заводская строка параметра (пустая — «не задано», показывается как '-')."""
        return STRING_DEFAULTS.get(name, "")

    def reset_to_defaults(self):
        """'defaults': заводские настройки прошивки (из bf_data + fc_overrides).

        Сбрасывается ВСЯ конфигурация, а не только таблица настроек: в прошивке
        cliDefaults() зовёт resetConfig() (config.c), который возвращает к reset-шаблонам
        все PG-таблицы — фичи, пищалку, датчики, порты, микшер, VTX-условия и т.д.
        Поэтому здесь же сбрасываются поля, которые вкладки пишут не через `set`:
        фичи, выключенные сигналы бипера, выбор датчиков, таблица портов, микшер,
        слоты `vtx` и «эхо» записей вкладок (иначе вкладка после сброса видела бы
        прежние значения — ровно то, на что указало повторное ревью).

        Таблица VTX тоже возвращается к заводской (у эмулятора она заполнена — как у квада,
        которому производитель залил таблицу своего передатчика).
        """
        self.settings = dict(self._defaults)
        # pgResetFn_featureConfig(): набор фич возвращается к заводскому, а вслед за ним
        # пересобирается список портов (SOFTSERIAL1/2 существуют только при включённой фиче).
        self.features = set(FEATURE_DEFAULTS)
        self.beepers_disabled = set()          # beeper_off_flags = 0 (pg/beeper.c 4.5.4)
        self.sensor_config = {                 # выбор датчиков во вкладке «Датчики»: AUTO/NONE
            "acc_hardware": SENSOR_AUTO,
            "baro_hardware": SENSOR_AUTO,
            "mag_hardware": SENSOR_NONE,
            "sonar_hardware": RANGEFINDER_NONE,
        }
        # serialConfig_t: функции на портах возвращаются к заводским (pgResetFn_serialConfig).
        self.serial_ports = self._default_serial_ports()
        self.mixer_name = self.MIXER_DEFAULT   # mixerConfig.mixerMode = QUADX
        # vtxChannelActivationConditions: все 10 слотов `vtx` снова нулевые.
        self.vtx_activation = self._default_vtx_activation()
        # Конфигурация, записанная вкладками «эхом» (блоки, которые эмулятор не моделирует):
        # после сброса она больше не отражает состояние FC и должна уйти.
        self.config_payloads = {}
        self.vtx_table = self._stock_vtx_table()
        self.vtx_pit_mode = False
        self._select_default_vtx_channel()
        self.osd_config = self._default_osd_config()
        # pgResetFn_adjustmentRangeConfig(): заводская таблица корректировок пустая —
        # 'defaults' убирает все слоты (вкладка покажет «Слотов активно 0/30»).
        self.adjustment_ranges = self._default_adjustment_ranges()
        # Как pgResetFn_accelerometerConfig(): калибровка акселерометра теряется,
        # и вкладка «Датчики» снова показывает «акселерометр не откалиброван».
        self.acc_calibrated = False
        self.acc_calibration_until = 0.0
        self.rxfail = self._default_rxfail()   # pgResetFn_rxFailsafeChannelConfigs()
        # Таблицы команд CLI — как их сбрасывают PG-функции прошивки: rxrange снова
        # 1000…2000, лента и цвета не настроены, свой микшер пуст, DShot-пищалка включена.
        self.resources = self._default_resources()             # pgResetFn_* пинов платы
        self.rx_ranges = self._default_rx_ranges()
        self.led_configs = self._default_led_configs()
        self.led_colors = self._default_led_colors()
        self.mode_colors = self._default_mode_colors()
        self.custom_motor_mix = self._default_custom_motor_mix()
        self.dshot_beacon_disabled = {"RX_LOST", "RX_SET"}   # DEFAULT_DSHOT_BEACON_OFF_FLAGS
        self.dshot_beacon_tone = 1
        self.battery_profiles = self._default_battery_profiles()
        self.active_battery_profile = 0
        self.signature = b"\x00" * len(self.signature)
        # modeActivationConditions[] регистрируется без шаблона сброса: 'defaults' обнуляет
        # все 20 слотов (режим ARM, 900/900) — вкладка «Режимы» снова пустая.
        self.mode_ranges = self._default_mode_ranges()
        self.servo_params = self._default_servo_params()      # pgResetFn_servoParams()
        self.servo_mix_rules = self._default_servo_mix_rules()
        # resetPidProfile() + pgResetFn_controlRateProfiles(): все профили — заводские
        self.pid_profiles = [dict(PID_PROFILE_DEFAULTS) for _ in range(PID_PROFILE_COUNT)]
        self.rate_profiles = [dict(RATE_PROFILE_DEFAULTS) for _ in range(CONTROL_RATE_PROFILE_COUNT)]
        self.pid_profile_names = {i: f"Profile {i + 1}" for i in range(PID_PROFILE_COUNT)}
        self.rate_profile_names = {i: "Actual" for i in range(CONTROL_RATE_PROFILE_COUNT)}
        self.current_profile = 0
        self.current_rate_profile = 0
        # Состояние виртуальной аппаратуры 'defaults' не трогает: стики — не конфиг FC.
        # pgResetFn_rxConfig(): карта каналов — часть конфигурации приёмника,
        # поэтому 'defaults' возвращает заводскую AETR1234 (иначе вкладка «Приёмник»
        # показывала бы прежнюю карту после сброса).
        self.rx_map = "AETR1234"
        # pgResetFn_pilotConfig(): имена дрона и пилота — тоже часть настроек,
        # 'defaults' возвращает заводские (и CLI, и MSP видят одно и то же).
        for name in ("craft_name", "pilot_name"):
            if isinstance(self.settings.get(name), str):
                self._mirror_name_setting(name, self.settings[name])
        # Фичи сброшены — значит список портов мог поменяться (SOFTSERIAL1/2).
        self.sync_soft_serial_ports()

    def find_setting(self, name: str):
        """Настройка по имени без учёта регистра — cliGetSettingIndex() из cli.c.

        Прошивка сравнивает имя через `strncasecmp` и требует совпадения длины целиком:
        `set GYRO_LPF1_TYPE = PT1` работает, `set gyro = 1` — нет.
        """
        entry = self.settings_by_name.get(name)
        if entry is not None:
            return entry
        lowered = name.lower()
        for key, candidate in self.settings_by_name.items():
            if key.lower() == lowered:
                return candidate
        return None

    def setting_range_lines(self, entry) -> list:
        """Строки cliPrintVarRange(): границы числовой настройки, имена списка, длины строки."""
        if entry["mode"] == "MODE_LOOKUP" and entry.get("values"):
            return ["Allowed values: " + ", ".join(entry["values"])]
        if entry["mode"] == "MODE_STRING":
            lo, hi = self.setting_lengths(entry)
            return [f"String length: {lo} - {hi}"]
        if entry["mode"] == "MODE_ARRAY":
            # Длину массива берём из таблицы (cli_settings.c: .config.array.length = N);
            # если генератор её не разобрал, лучше промолчать, чем напечатать ноль.
            length = entry.get("array_length")
            return [f"Array length: {length}"] if length else []
        if entry["mode"] == "MODE_BITSET":
            return ["Allowed values: OFF, ON"]
        return [f"Allowed range: {entry['min']} - {entry['max']}"]

    def set_setting(self, name: str, raw: str):
        """`set` как cliSet(): (получилось, текст дополнительной ошибки).

        Второй элемент нужен только MODE_STRING («STRING MUST BE 1-N CHARACTERS OR
        '-' FOR EMPTY»); при любом другом отказе прошивка печатает `INVALID VALUE` и
        границы настройки — это делает вызывающий (cli.py::cmd_set).
        """
        entry = self.find_setting(name)
        if entry is None:
            return False, None
        name = entry["name"]
        if entry["mode"] == "MODE_STRING":
            # cliSet(): пустая строка и '-' (emptyName) стирают значение, длиннее max —
            # «STRING MUST BE 1-N CHARACTERS», а строка без завершающего нуля вообще
            # не принимается (границы в прошивке заданы как «до следующего параметра»).
            lengths = self.setting_lengths(entry)
            if raw == "":
                return False, f"STRING MUST BE 1-{lengths[1]} CHARACTERS OR '-' FOR EMPTY"
            if raw == "-":
                self.settings[name] = ""
                self._mirror_name_setting(name, "")
                return True, None
            if len(raw) > lengths[1]:
                return False, f"STRING MUST BE 1-{lengths[1]} CHARACTERS OR '-' FOR EMPTY"
            self.settings[name] = raw
            self._mirror_name_setting(name, raw)
            return True, None
        if entry["mode"] == "MODE_LOOKUP" and entry.get("values"):
            # cliSet() ищет значение перебором `strcasecmp(tableEntry->values[i], eqptr)`:
            # номер элемента списка числом не принимается, только имя.
            value = self._parse_lookup(entry, raw)
            if value is None:
                return False, None
            self.settings[name] = value
            return True, None
        value = self._atoi(raw)
        lo, hi = entry["min"], entry["max"]
        if not (lo <= value <= hi):
            return False, None
        self.settings[name] = value
        return True, None

    @staticmethod
    def _atoi(text: str) -> int:
        """atoi()/strtoul() из cliSet(): десятичное число в начале строки, хвост игнорируется."""
        i, sign = 0, 1
        while i < len(text) and text[i].isspace():
            i += 1
        if i < len(text) and text[i] in "+-":
            sign = -1 if text[i] == "-" else 1
            i += 1
        digits = ""
        while i < len(text) and text[i].isdigit():
            digits += text[i]
            i += 1
        return sign * int(digits or 0)

    def _mirror_name_setting(self, name: str, value: str) -> None:
        """Имя дрона/пилота живёт и в таблице настроек, и отдельным полем («MSP2_GET_TEXT»).

        В прошивке это одно поле pilotConfig_t: 'set craft_name=…' меняет то же,
        что вкладка пишет через MSP_SET_TEXT.
        """
        if name == "craft_name":
            self.craft_name = value
        elif name == "pilot_name":
            self.pilot_name = value

    @staticmethod
    def setting_lengths(entry) -> tuple:
        """Границы длины строкового параметра: из таблицы, либо по имени."""
        return (int(entry.get("string_min", 1)),
                int(entry.get("string_max", STRING_LENGTHS.get(entry["name"], (1, MAX_NAME_LENGTH))[1])))

    @staticmethod
    def _parse_lookup(entry, raw: str):
        """Значение списка по имени — как cliSet(): только `strcasecmp` с элементами.

        Номер элемента числом прошивка не принимает: `set gyro_lpf1_type = 1` отвечает
        `INVALID VALUE` и печатает список, хотя элемент с этим номером существует.
        """
        raw = raw.strip().upper()
        for i, v in enumerate(entry["values"]):
            if v.upper() == raw:
                return i
        return None

    def format_setting(self, entry):
        value = self.settings[entry["name"]]
        if entry["mode"] == "MODE_STRING":
            return value if value else "-"
        if entry["mode"] == "MODE_LOOKUP" and entry.get("values"):
            if 0 <= value < len(entry["values"]):
                return entry["values"][value]
            return str(value)
        return str(value)

    def default_display(self, entry):
        dv = entry.get("default")
        if dv is None:
            return None
        if entry["mode"] == "MODE_LOOKUP" and entry.get("values") and 0 <= dv < len(entry["values"]):
            return entry["values"][dv]
        if entry["mode"] == "MODE_STRING":
            return dv if dv else "-"
        return str(dv)

    def setting_lines(self, section: str):
        out = []
        for entry in self.settings_list:
            if entry["section"] != section:
                continue
            if entry["mode"] in ("MODE_ARRAY", "MODE_BITSET"):  # упрощение: массивы не печатаем
                continue
            out.append(f"set {entry['name']} = {self.format_setting(entry)}")
        return out

    def changed_setting_lines(self):
        out = []
        for entry in self.settings_list:
            if self.settings[entry["name"]] != self._defaults.get(entry["name"]):
                out.append(f"set {entry['name']} = {self.format_setting(entry)}")
        return out

    # --------------------------------------------------------- dump-помощники
    def banner(self):
        api = self.api_version
        if api.endswith(".0"):
            api = api[:-2]
        return (f"Betaflight / {self.target_name} ({self.board_identifier}) {self.fc_version} "
                f"{self.build_date} / {self.build_time} ({self.git_revision}) MSP API: {api}")

    def features_items(self):
        for name, _bit in sorted(self.feature_bits_by_name.items(), key=lambda kv: kv[1]):
            yield name, name in self.features

    @property
    def feature_mask(self):
        mask = 0
        for name in self.features:
            mask |= self.feature_bits_by_name.get(name, 0)
        return mask

    def feature_lines(self, only_diff=False):
        out = []
        for name, _bit in sorted(self.feature_bits_by_name.items(), key=lambda kv: kv[1]):
            enabled = name in self.features
            default_enabled = name in FEATURE_DEFAULTS
            if only_diff and enabled == default_enabled:
                continue
            out.append(("feature " if enabled else "feature -") + name)
        return out

    @staticmethod
    def _mask_to_functions(mask):
        return [name for name, bit in SERIAL_FUNCTIONS.items() if mask & bit]

    def _functions_to_mask(self, functions):
        mask = 0
        for name in functions:
            mask |= SERIAL_FUNCTIONS.get(name, 0)
        return mask

    def serial_lines(self):
        """Как printSerial() в cli.c: идентификатор, маска функций и реальные скорости (не индексы)."""
        def baud(idx):
            return BAUD_RATES[idx] if 0 <= idx < len(BAUD_RATES) else "AUTO"

        out = []
        for p in self.serial_ports:
            out.append(f"serial {p['identifier']} {self._functions_to_mask(p['functions'])} "
                       f"{baud(p['msp_baudrate'])} {baud(p['gps_baudrate'])} "
                       f"{baud(p['telemetry_baudrate'])} {baud(p['blackbox_baudrate'])}")
        return out

    def beeper_lines(self, only_diff=False):
        out = []
        for name in self.beeper_names:
            disabled = name in self.beepers_disabled
            out.append(("beeper -" if disabled else "beeper ") + name)
        return out

    @property
    def uid_hex(self):
        return "".join(f"{u:08x}" for u in self.uids)

    # ------------------------------------------------------------ «флеш» (EEPROM)
    _EEPROM_SECTIONS = (
        "sensor_config", "config_payloads",
        "features", "beepers_disabled", "serial_ports", "mode_ranges", "rx_map", "rxfail",
        "servo_params", "servo_mix_rules", "mixer_name",
        "pid_profiles", "rate_profiles",
        "craft_name", "pilot_name", "board_name", "manufacturer_id", "pid_profile_names",
        "rate_profile_names", "current_profile", "current_rate_profile",
        "battery_profiles", "active_battery_profile",
        "uids", "signature",
        "vtx_table", "vtx_pit_mode", "vtx_activation", "acc_calibrated",
        "osd_config", "adjustment_ranges",
        # таблицы команд CLI 4.5.4: rxrange / led / color / mode_color / mmix / beacon
        "rx_ranges", "led_configs", "led_colors", "mode_colors", "custom_motor_mix",
        "dshot_beacon_disabled", "dshot_beacon_tone", "resources",
    )

    def eeprom_state(self) -> dict:
        state = {"version": 1, "settings": dict(self.settings)}
        for name in self._EEPROM_SECTIONS:
            value = getattr(self, name, None)
            if isinstance(value, set):
                value = sorted(value)
            if isinstance(value, tuple):
                value = list(value)
            if isinstance(value, (bytes, bytearray)):
                value = bytes(value).hex()          # signature: во «флеше» — hex-строка
            if isinstance(value, dict):
                value = {
                    str(k): (v.hex() if isinstance(v, (bytes, bytearray)) else v) for k, v in value.items()
                }
            state[name] = value
        return state

    def load_eeprom(self):
        """Читает конфигурацию из файла при старте — как прошивка из флеша.

        «Флеш» может быть каким угодно: чужим, обрезанным, собранным другой версией
        эмулятора. Поэтому старт здесь ломать нельзя ни при каких данных: файл — валидный
        JSON, но не тот объект (список, строка), уже ронял запуск (находка повторного
        ревью). Разделы применяются поштучно и с проверкой типов; непонятный раздел
        пропускается с записью в лог, остаётся заводское значение.
        """
        if not self.eeprom_path or not self.eeprom_path.exists():
            return False
        try:
            state = json.loads(self.eeprom_path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            self._log(f"[FC] «флеш» {self.eeprom_path} повреждён ({exc}) — стартую с заводскими настройками")
            return False
        if not isinstance(state, dict):
            self._log(f"[FC] «флеш» {self.eeprom_path}: на верхнем уровне {type(state).__name__}, "
                      "а нужен объект — стартую с заводскими настройками")
            return False
        try:
            self._apply_eeprom(state)
        except Exception as exc:  # «флеш» не должен мешать запуску
            self._log(f"[FC] «флеш» {self.eeprom_path} не разобран "
                      f"({type(exc).__name__}: {exc}) — состояние сброшено к заводскому")
            self.reset_to_defaults()
            return False
        self.eeprom_loaded = True
        self._log(f"[FC] конфигурация загружена из «флеша»: {self.eeprom_path}")
        return True

    def _apply_eeprom(self, state: dict) -> None:
        """Переносит состояние из «флеша» в FC (сам файл уже прочитан и проверен)."""
        settings = state.get("settings")
        if settings is None:
            settings = {}
        elif not isinstance(settings, dict):
            self._log(f"[FC] «флеш»: раздел settings — {type(settings).__name__}, "
                      "а нужен объект: настройки остаются заводскими")
            settings = {}
        for name, value in settings.items():
            entry = self.settings_by_name.get(name)
            if entry is None:
                continue
            if entry["mode"] == "MODE_STRING" and isinstance(value, str):
                self.settings[name] = value            # строки (имена) — как есть
            elif isinstance(value, (int, float)) and not isinstance(value, bool):
                self.settings[name] = int(value)
        for name in self._EEPROM_SECTIONS:
            if name not in state or state[name] is None:
                continue
            try:
                self._apply_eeprom_section(name, state[name])
            except Exception as exc:  # битый раздел не должен мешать старту
                self._log(f"[FC] «флеш»: раздел «{name}» не разобран "
                          f"({type(exc).__name__}: {exc}) — беру заводское значение")
        self._apply_eeprom_tail()

    def _apply_eeprom_section(self, name: str, saved) -> None:
        """Один раздел «флеша»: у каждого свои правила разбора (битое — заводское)."""
        if name == "serial_ports":
            # Записи из старого «флеша» могли не иметь скоростей — добираем значения
            # как в serialConfig_t по умолчанию, чтобы ответ вкладке «Порты» не падал,
            # а не-объекты в списке отбрасываем: иначе старт падал на port.get().
            ports = []
            for port in saved if isinstance(saved, list) else []:
                if not isinstance(port, dict):
                    continue
                functions = port.get("functions")
                ports.append({
                    "identifier": self._clamp_int(port.get("identifier"), 0, 255, 0),
                    "functions": [f for f in functions if isinstance(f, str)]
                    if isinstance(functions, list) else [],
                    "msp_baudrate": self._clamp_int(port.get("msp_baudrate"), 0, 7, 0),
                    "gps_baudrate": self._clamp_int(port.get("gps_baudrate"), 0, 7, 4),
                    "telemetry_baudrate": self._clamp_int(port.get("telemetry_baudrate"), 0, 7, 0),
                    "blackbox_baudrate": self._clamp_int(port.get("blackbox_baudrate"), 0, 7, 5),
                })
            self.serial_ports = ports or self._default_serial_ports()
        elif name == "config_payloads":
            # «Эхо» записей вкладок (коды, которые эмулятор не моделирует): код → payload,
            # payload во «флеше» — hex-строка. Ключей в поле может не быть вовсе (свежий
            # «флеш»), поэтому тип берём не из поля, а из самой секции: hex → байты.
            # Значения не из hex отбрасываем: иначе в конфигурацию попала бы строка,
            # которую MSP отдал бы клиенту вместо payload'а.
            payloads = {}
            if isinstance(saved, dict):
                for code, payload in saved.items():
                    data = self._hex_to_bytes(payload)
                    if isinstance(code, str) and code.lstrip("-").isdigit() and data is not None:
                        payloads[int(code)] = data
            self.config_payloads = payloads
        elif name == "vtx_table":
            # Таблицу VTX восстанавливаем явно: в ней лежат строки и вложенные списки,
            # а общий разбор «флеша» умеет только числа и hex-строки.
            self.vtx_table = self._vtx_table_from_json(saved)
        elif name == "osd_config":
            # Настройки OSD — тоже со списками (места элементов, таймеры): разбираем явно и
            # добираем недостающее заводскими значениями, чтобы старый «флеш» не ломал вкладку.
            self.osd_config = self._osd_config_from_json(saved)
        elif name == "adjustment_ranges":
            # Слоты корректировок — список словарей: общий разбор умеет только числа,
            # поэтому читаем сами и добираем недостающие слоты.
            self.adjustment_ranges = self._adjustment_ranges_from_json(saved)
        elif name == "mode_ranges":
            # Слоты режимов: старый «флеш» знал только index/box/aux/start/end — добираем
            # остальные слоты и поля (logic/linked), иначе вкладка «Режимы» увидела бы
            # «дырки» в массиве из 20 записей.
            self.mode_ranges = self._normalize_mode_ranges(saved)
        elif name == "battery_profiles":
            # Таблица профилей батареи — список словарей: разбираем сами, битое
            # отбрасываем, недостающие профили добираем заводскими.
            self.battery_profiles = self._battery_profiles_from_json(saved)
        elif name == "resources":
            # Назначения пинов: битые строки отбрасываем, длины выравниваем по таблице,
            # чтобы старый «флеш» не ломал `resource`.
            self.resources = self._resources_from_json(saved)
        elif name == "vtx_activation":
            # Слоты `vtx`: битое отбрасываем, недостающие добираем нулевыми,
            # чтобы старый «флеш» не ломал команду.
            self.vtx_activation = self._vtx_activation_from_json(saved)
        elif name in ("rx_ranges", "led_configs", "led_colors", "mode_colors", "custom_motor_mix"):
            # Таблицы команд CLI (rxrange/led/color/mmix): числа лежат вложенными списками,
            # поэтому разбираем их сами — битое отбрасываем, недостающее добираем заводским.
            self._load_cli_table(name, saved)
        else:
            # Остальные разделы — поля простых типов: числа, строки, наборы, кортежи, словари.
            # Значение берём только подходящего типа, иначе остаётся заводское.
            setattr(self, name, self._eeprom_value(getattr(self, name, None), saved))

    @classmethod
    def _eeprom_value(cls, current, saved):
        """Значение из «флеша», приведённое к типу поля (словари — в _eeprom_dict).

        JSON не знает про типы Python: во «флеше» может лежать строка вместо числа или
        число вместо списка. Такое значение не применяем — лучше остаться на заводском,
        чем потом падать в обработчике MSP.
        """
        if isinstance(current, dict):
            return cls._eeprom_dict(current, saved)
        if isinstance(current, bool):
            return saved if isinstance(saved, bool) else current
        if isinstance(current, int):
            return int(saved) if isinstance(saved, int) and not isinstance(saved, bool) else current
        if isinstance(current, float):
            return float(saved) if isinstance(saved, (int, float)) and not isinstance(saved, bool) else current
        if isinstance(current, str):
            return saved if isinstance(saved, str) else current
        if isinstance(current, (bytes, bytearray)):
            return cls._eeprom_bytes(saved, bytes(current))
        if isinstance(current, set):
            return {item for item in saved if isinstance(item, str)} if isinstance(saved, list) else current
        if isinstance(current, tuple):
            if isinstance(saved, list) and len(saved) == len(current) and all(isinstance(i, int) for i in saved):
                return tuple(int(i) for i in saved)
            return current
        if isinstance(current, list):
            if not isinstance(saved, list):
                return current
            out = [cls._eeprom_value(current[i], saved[i])
                   for i in range(min(len(current), len(saved)))]
            out.extend(current[len(out):])       # недостающие записи — заводские
            return out
        return current

    @classmethod
    def _eeprom_dict(cls, current: dict, saved) -> dict:
        """Словарь из «флеша»: тип значения берём из текущего поля, лишние ключи отбрасываем.

        Так же разбираются и «байтовые» словари (`config_payloads`: код — записанный
        вкладкой payload). Декодировать hex «на глаз» нельзя: имя профиля вида «BEEF»
        тоже состоит из hex-символов, и после перезапуска оно превращалось в байты
        b"\\xbe\\xef" — MSP2_GET_TEXT падал на чтении имени (находка повторного ревью).
        Поэтому hex разбирается только там, где в поле действительно лежат байты.
        """
        if not isinstance(saved, dict):
            return dict(current)
        result = dict(current)
        for key, item in saved.items():
            if not isinstance(key, str):
                continue
            name = int(key) if key.lstrip("-").isdigit() else key
            if name not in current:
                continue                          # ключа нет в поле: чужой «флеш»
            result[name] = cls._eeprom_value(current[name], item)
        return result

    @staticmethod
    def _hex_to_bytes(saved):
        """Байты во «флеше» записаны hex-строкой: разобрали — bytes, иначе None."""
        if isinstance(saved, str) and len(saved) % 2 == 0 and all(c in "0123456789abcdef" for c in saved.lower()):
            return bytes.fromhex(saved)
        return None

    @classmethod
    def _eeprom_bytes(cls, saved, current: bytes) -> bytes:
        """Байтовое поле из «флеша»; не разобрали — остаётся заводское."""
        data = cls._hex_to_bytes(saved)
        return current if data is None else data

    def _apply_eeprom_tail(self):
        """Догоняющие правки после чтения «флеша» (миграции старых сборок и зависимости).

        Пищалка из старых «флешей»: сборки до 2026-10-08-b шли с «заводским» набором
        выключенных сигналов (RX_LOST, USB, CAM_CONNECTION_CLOSE, RC_SMOOTHING_INIT_FAIL),
        хотя в прошивке beeper_off_flags = 0 (pg/beeper.c), а DShot-бикон выключен по
        RX_LOST/RX_SET (DEFAULT_DSHOT_BEACON_OFF_FLAGS, io/beeper.h). Если во «флеше» лежит
        ровно старый заводской набор — подменяем на прошивочный: пользователь его не выбирал.
        """
        if self.beepers_disabled == {"RX_LOST", "USB", "CAM_CONNECTION_CLOSE", "RC_SMOOTHING_INIT_FAIL"}:
            self.beepers_disabled = set()
        if not self.dshot_beacon_disabled:
            self.dshot_beacon_disabled = {"RX_LOST", "RX_SET"}
        # В старых «флешах» для этих кодов лежало эхо записанного payload; теперь ответы
        # строятся из настроек, поэтому эхо не должно подменять их. Чистим сразу все такие
        # коды — иначе, например, включённый во вкладке «Конфигурация» SOFTSERIAL/SERVO_TILT
        # «не сохранялся»: чтение отдавало старое эхо.
        stale = [code for code in list(self.config_payloads) if code in self.state_based_read_codes()]
        for code in stale:
            self.config_payloads.pop(code, None)
        if stale:
            self._log("[FC] «флеш»: устаревшее эхо для кодов " +
                      ", ".join(f"0x{c:04X}" for c in sorted(stale)) + " отброшено")
        self.sync_soft_serial_ports()
        self.check_feature_bits()
        # Имена дрона и пилота лежат и в таблице настроек, и отдельными полями
        # (pilotConfig_t): приводим второе к первому, чтобы CLI и MSP не расходились.
        for name in ("craft_name", "pilot_name"):
            if isinstance(self.settings.get(name), str):
                self._mirror_name_setting(name, self.settings[name])

    def save_to_eeprom(self):
        """EEPROM_WRITE / 'save': пишем активную конфигурацию в «флеш».

        Как и у настоящего FC, пишется всё состояние сразу, а при следующем старте
        прошивка поднимает его из флеша.
        """
        if not self.eeprom_path:
            self._log("[FC] EEPROM_WRITE: файл «флеша» не задан, конфигурация живёт только в памяти")
            return False
        try:
            self.eeprom_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.eeprom_path.with_suffix(self.eeprom_path.suffix + ".tmp")
            tmp.write_text(json.dumps(self.eeprom_state(), indent=1, ensure_ascii=False), encoding="utf-8")
            tmp.replace(self.eeprom_path)  # атомарная замена: «запись во флеш» не может побиться
        except OSError as exc:
            self._log(f"[FC] не удалось записать «флеш» {self.eeprom_path}: {exc}")
            return False
        self._log(f"[FC] конфигурация записана в «флеш»: {self.eeprom_path}")
        return True

    def reload_after_reboot(self):
        """Ребут FC: конфигурация поднимается из «флеша», несохранённое теряется.

        Если «флеш» отключён (--eeprom none), состояние остаётся в памяти процесса —
        так эмулятор ведёт себя как «FC без флеша».
        """
        if not self.eeprom_path:
            return
        self.reset_to_defaults()
        if not self.load_eeprom():
            self._log("[FC] «флеш» пуст: после перезагрузки — заводские настройки")
        self.init_vtx_device()
        self._battery_volatile_reset()

    def wipe_eeprom(self):
        """Полный сброс: как «Erase» флеша перед первым запуском."""
        if self.eeprom_path and self.eeprom_path.exists():
            try:
                self.eeprom_path.unlink()
            except OSError:
                pass
        self._defaults_restore()

    def _defaults_restore(self):
        """Полный сброс, как resetConfig() в config.c: всё, что переживает ребут, — заводское.

        Отдельного набора полей здесь больше нет: reset_to_defaults() сбрасывает
        конфигурацию целиком (включая фичи, порты и «эхо» записей вкладок), а этот
        метод оставлен точкой входа для путей, где нужен именно полный сброс:
        MSP_RESET_CONF (кнопка Configurator) и erase «флеша».
        """
        self.reset_to_defaults()
        self.init_vtx_device()     # передатчик ищется заново: функции портов сброшены

    def _apply_overrides(self):
        """Небольшая таблица «хорошо известных» заводских значений.

        Большая часть defaults извлекается автоматически из pgResetFn_*/PG_RESET_TEMPLATE,
        но некоторые значения задаются выражениями, которые не разбираются генератором.
        Значения ниже соответствуют стоковой прошивке Betaflight 4.5.x.
        """
        override_path = Path(__file__).resolve().parent / "data" / "fc_overrides.json"
        if not override_path.exists():
            return
        for name, value in json.loads(override_path.read_text()).items():
            entry = self.settings_by_name.get(name)
            if entry is None:
                continue
            if isinstance(value, str) and entry["mode"] == "MODE_LOOKUP" and entry.get("values"):
                value = self._parse_lookup(entry, value)
                if value is None:
                    continue
            self.settings[name] = value
            entry["default"] = value

    def _battery_volatile_reset(self):
        """Счётчик мА·ч и тест моторов живут только до перезагрузки — как на железе."""
        self._fs_init_state()               # демо failsafe после ребута выключено: борт на столе
        self.mah_drawn = 0.0
        self._mah_t = None
        self.motor_disarmed = [1000.0] * MAX_SUPPORTED_MOTORS   # motor_disarmed[] в нуле → внешние 1000
        self.motor_output = list(self.motor_disarmed)
        self.motor_rpm = [0.0] * MAX_SUPPORTED_MOTORS
        self.motor_temp_c = [self.MOTOR_AMBIENT_C] * MAX_SUPPORTED_MOTORS
        self.motor_current_a = [0.0] * MAX_SUPPORTED_MOTORS
        self.motor_mah = [0.0] * MAX_SUPPORTED_MOTORS
        self.motor_voltage_v = 0.0
        self._motors_t = time.time()
        self.motor_test_arming_block = False
        # Полёт — не конфигурация: после перезагрузки борт снова «на столе».
        self.sim_flight = False
        self.sim_flight_phase = "idle"
        self.sim_flight_since = 0.0
        self.sim_flight_note = ""
        self.sim_pack_events = 0

    def reboot(self, bootloader=False):
        self.pending_reboot = True
        self.pending_bootloader = bootloader

    # ---------------------------------------------------------- режим ЗУ (MSC)
    def enter_msc(self, mode: int = MSP_REBOOT_MSC) -> None:
        """Перезагрузка «в режим запоминающего устройства» — кнопка во вкладке «Чёрный ящик».

        Настоящий борт после этого перестаёт отвечать по MSP и появляется на компьютере
        как флешка. У эмулятора флешки нет, поэтому содержимое «флеша» отдаёт страница
        /msc (файлы по пролётам + один общий дамп), а MSP перестаёт отвечать — как у
        настоящего борта, который «ушёл» в режим ЗУ.
        """
        self.msc_mode = True
        self.msc_deadline = time.time() + MSC_SECONDS
        self.reboot()
        self._log(f"[MSC] режим запоминающего устройства включён (перезагрузка {mode}): "
                  f"файлы «флеша» — на странице /msc, MSP не отвечает; "
                  f"выйти — кнопка на странице или перезапуск эмулятора")

    def exit_msc(self, reason: str = "") -> None:
        """Выход из режима ЗУ: как «передёрнуть питание» у настоящего борта."""
        if not self.msc_mode:
            return
        self.msc_mode = False
        self.msc_deadline = 0.0
        self._log("[MSC] режим запоминающего устройства выключен"
                  + (f" ({reason})" if reason else "") + ": MSP снова отвечает")

    def msc_active(self) -> bool:
        """Активен ли режим ЗУ (с автовыходом по времени, чтобы эмулятор не «залипал»)."""
        if self.msc_mode and self.msc_deadline and time.time() > self.msc_deadline:
            self.exit_msc("истекло время ожидания")
        return self.msc_mode

    def _setting(self, name, default=0):
        """Значение настройки (если она есть в прошивке), иначе значение по умолчанию."""
        value = self.settings.get(name)
        return default if value is None else int(value)

    def _store_setting(self, name, value):
        """Запись значения так, как это сделал бы CLI 'set': с ограничением по диапазону."""
        entry = self.settings_by_name.get(name)
        if entry is None:
            return False
        if entry["mode"] == "MODE_LOOKUP":
            lo, hi = 0, max(len(entry.get("values") or []) - 1, 0)
        else:
            lo, hi = entry["min"], entry["max"]
        self.settings[name] = max(lo, min(hi, int(value)))
        return True

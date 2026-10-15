"""Виртуальный полётный контроллер: композиция тематических примесей.

Публичные VirtualFC и MSP-константы по-прежнему импортируются из bfemu.fc.
"""
from __future__ import annotations

# ruff: noqa: F403, F405 -- legacy public MSP constants are re-exported from here.
import json
import time
from pathlib import Path

from . import bb_scenarios
from .cli import CliEngine
from .msp import Reader, Writer
from .fc_constants import *
from .fc_modes import ModesMixin
from .fc_state import StateMixin
from .fc_video import VideoMixin
from .fc_identity import IdentityMixin
from .fc_motors import MotorsMixin
from .fc_resources import ResourcesMixin
from .fc_power import PowerMixin
from .fc_receiver import ReceiverMixin
from .fc_failsafe import FailsafeMixin
from .fc_gps import GpsMixin
from .fc_tuning import TuningMixin
from .fc_blackbox import BlackboxMixin


class VirtualFC(
    ModesMixin,
    StateMixin,
    VideoMixin,
    IdentityMixin,
    MotorsMixin,
    ResourcesMixin,
    PowerMixin,
    ReceiverMixin,
    FailsafeMixin,
    GpsMixin,
    TuningMixin,
    BlackboxMixin,
):
    """State and MSP/CLI entry point; domain behavior is inherited from mixins."""
    # ------------------------------------------------------------------ init
    def __init__(
        self,
        data_path: Path | str | None = None,
        fc_version: str = "4.5.4",
        api_version: str = "1.46.0",
        eeprom_path: Path | str | None = None,
        log=None,
    ):
        path = Path(data_path) if data_path else DATA_PATH
        self.data = json.loads(path.read_text())
        self.fc_version = fc_version
        self.api_version = api_version
        self.msp_protocol_version = 0
        self._log = log or (lambda _msg: None)
        # «флеш» эмулятора: обычный JSON-файл. None — живём только в памяти.
        self.eeprom_path = Path(eeprom_path) if eeprom_path else None
        self.eeprom_loaded = False
        # «Чёрный ящик»: накопитель с тестовыми пролётами (MSP_DATAFLASH_SUMMARY/READ).
        # Данные лежат рядом с «флешем» настроек и наполняются эмулятором пролётов.
        flash_path = (self.eeprom_path.with_name("blackbox_flash.bin")
                      if self.eeprom_path else None)
        self.blackbox = bb_scenarios.BlackboxStore(flash_path, log=self._log)

        self.settings_list = []
        self.settings_by_name = {}
        self.settings = {}
        self._load_settings()

        # идентичность платы
        self.board_identifier = "S405"
        self.target_name = "STM32F405"
        self.board_name = "VIRTUALFC"
        self.manufacturer_id = "BFR"
        self.mcu_type_id = 1  # F40X
        self.mcu = "F40X"
        self.signature = b"\x00" * 32
        # Возможности платы (MSP_BOARD_INFO): прошивка выставляет их из msp.c —
        #   bit 0 TARGET_HAS_VCP        (USE_VCP)
        #   bit 1 TARGET_HAS_SOFTSERIAL (USE_SOFTSERIAL)
        # Configurator по биту SOFTSERIAL понимает, что сборка вообще умеет софт-серийные
        # порты (fc.js: boardHasSoftSerial -> unreportedSoftSerialIdentifiers).
        self.target_capabilities = 0x01 | 0x02
        self.configuration_state = 1  # CONFIGURED
        self.configuration_state_name = "CONFIGURED"
        self.config_rev = "0"
        self.eeprom_size = 4166
        self.git_revision = "emu0bf"
        self.build_date = "Sep 24 2026"
        self.build_time = "12:00:00"

        # изменяемое состояние
        self.features = set(FEATURE_DEFAULTS)
        # Биты фич: из bf_data.json ("feature_bits" — сгенерировано из featureNames[] прошивки).
        # ВАЖНО: нумеровать по порядку непустых имён нельзя — в прошивке биты идут с пропусками,
        # и Configurator (Features.js) ждёт именно «дырявую» нумерацию: SOFTSERIAL = 1 << 6.
        bits = self.data.get("feature_bits") or {}
        self.feature_bits_by_name = {name: (1 << bit) for name, bit in bits.items() if isinstance(bit, int)}
        self.feature_bits_by_name = {name: bit for name, bit in self.feature_bits_by_name.items() if bit}
        if not self.feature_bits_by_name:  # страховка, если данные не сгенерированы
            self.feature_bits_by_name = {
                "RX_PPM": 1 << 0, "INFLIGHT_ACC_CAL": 1 << 2, "RX_SERIAL": 1 << 3, "MOTOR_STOP": 1 << 4,
                "SERVO_TILT": 1 << 5, "SOFTSERIAL": 1 << 6, "GPS": 1 << 7, "RANGEFINDER": 1 << 9,
                "TELEMETRY": 1 << 10, "3D": 1 << 12, "RX_PARALLEL_PWM": 1 << 13, "RX_MSP": 1 << 14,
                "RSSI_ADC": 1 << 15, "LED_STRIP": 1 << 16, "DASHBOARD": 1 << 17, "OSD": 1 << 18,
                "CHANNEL_FORWARDING": 1 << 20, "TRANSPONDER": 1 << 21, "AIRMODE": 1 << 22,
                "RX_SPI": 1 << 25, "ESC_SENSOR": 1 << 27, "ANTI_GRAVITY": 1 << 28,
            }
        self.beepers_disabled: set[str] = set()   # beeper_off_flags: поля нет в reset-шаблоне
        # (pg/beeper.c 4.5.4) → по умолчанию 0, то есть выключенных сигналов нет
        self.beeper_names = [b for b in BEEPER_NAMES]
        # Порты — заводская таблица pgResetFn_serialConfig(); она же возвращается
        # командой `defaults` (см. _default_serial_ports()).
        self.serial_ports = self._default_serial_ports()
        self.boxes = self.data.get("boxes") or []
        if not self.boxes:
            self.boxes = [
                {"box_id": "BOXARM", "name": "ARM", "permanent_id": 0},
                {"box_id": "BOXANGLE", "name": "ANGLE", "permanent_id": 1},
                {"box_id": "BOXHORIZON", "name": "HORIZON", "permanent_id": 2},
                {"box_id": "BOXANTIGRAVITY", "name": "ANTI GRAVITY", "permanent_id": 4},
                {"box_id": "BOXMAG", "name": "MAG", "permanent_id": 5},
                {"box_id": "BOXHEADFREE", "name": "HEADFREE", "permanent_id": 6},
                {"box_id": "BOXHEADADJ", "name": "HEADADJ", "permanent_id": 7},
                {"box_id": "BOXCAMSTAB", "name": "CAMSTAB", "permanent_id": 8},
                {"box_id": "BOXGPSHOME", "name": "GPS HOME", "permanent_id": 11},
                {"box_id": "BOXGPSHOLD", "name": "GPS HOLD", "permanent_id": 12},
                {"box_id": "BOXFAILSAFE", "name": "FAILSAFE", "permanent_id": 15},
                {"box_id": "BOXAIRMODE", "name": "AIRMODE", "permanent_id": 17},
                {"box_id": "BOX3DDISABLE", "name": "3D DISABLE", "permanent_id": 22},
                {"box_id": "BOXFPVANGLEMIX", "name": "FPV ANGLE MIX", "permanent_id": 23},
                {"box_id": "BOXBLACKBOXERASE", "name": "BLACKBOX ERASE", "permanent_id": 24},
                {"box_id": "BOXCAMERA1", "name": "CAMERA CONTROL 1", "permanent_id": 25},
                {"box_id": "BOXFLIPOVERAFTERCRASH", "name": "FLIP OVER AFTER CRASH", "permanent_id": 26},
                {"box_id": "BOXPREARM", "name": "PREARM", "permanent_id": 27},
                {"box_id": "BOXPARALYZE", "name": "PARALYZE", "permanent_id": 28},
                {"box_id": "BOXGPSRESCUE", "name": "GPS RESCUE", "permanent_id": 29},
            ]
        self.box_name_by_permanent_id = {b["permanent_id"]: b["name"] for b in self.boxes}
        # «стандартные» диапазоны AUX, как на свежей прошивке после дефолтов: все 20 слотов
        # пустые (режим ARM, диапазон 900/900) — вкладка «Режимы» показывает их как «нет строк»,
        # а CLI печатает 20 строк «aux i 0 0 900 900 0 0».
        self.mode_ranges = self._default_mode_ranges()
        self.rx_map = "AETR1234"
        # Виртуальная аппаратура: 16 каналов в порядке A/E/R/T, как rcData[] в прошивке.
        # Стики в центре, газ внизу, тумблеры AUX выключены. Это состояние НЕ пишется во
        # «флеш»: настоящая аппаратура тоже не хранится в FC — после перезагрузки стики
        # снова в центре (см. tools/tx.py и страницу http://127.0.0.1:8088/tx).
        self.rc_channels = self._default_rc_channels()
        # «В эфире» ли аппаратура (виртуальный USB-пульт). Как и настоящий пульт, это не
        # конфигурация FC: во «флеш» не пишется и после ребута сбрасывается.
        self.rx_link_connected = True
        self.rc_source = "—"           # кто последним двигал стики (страница /tx, tools/tx.py)
        # Бинд («связана ли аппаратура с приёмником»). Как и каналы — это состояние «в эфире»,
        # а не конфигурация FC: во «флеш» не пишется, после ребута сбрасывается в «связано»
        # (виртуальный FC «включён вместе с уже связанной аппаратурой»).
        #   "bound"   — связь установлена (кадры идут);
        #   "binding" — идёт поиск/связывание, через rx_bind_seconds станет "bound";
        #   "none"    — не связана: приёмник кадров не отдаёт, на вкладке «Приёмник» «Этап 1».
        # У кого управление: "virtual" — страница /tx (ползунки, тумблеры, крутилки),
        # "physical" — настоящая аппаратура (tools/tx.py joystick|wire). Кнопки страницы
        # переключают это состояние: «Подключить аппаратуру» — управление физической,
        # «Отключить (выдернуть USB)» — обратно виртуальной. Смена источника = другая
        # аппаратура: связь сбрасывается, каналы активирует «Связать (Bind)».
        self.rc_control = "virtual"
        # Какая ось пульта каким стиком управляет — это знает только инструмент, читающий
        # пульт (`tools/tx.py joystick`). Он сообщает раскладку сюда, чтобы её было видно
        # на странице /tx: если стики двигают не свои каналы, это сразу заметно.
        self.rc_map_info = {}
        self.rc_layout_note = ""            # «раскладка осей не та» — считает tools/tx.py
        self.rc_aux_map = []                # «AUX1 ← тумблер (2 положения) — ось 3» — из tools/tx.py
        self.rc_sources = []                # [(время, источник)] — кто писал каналы
        self.rc_channels_updated = 0.0      # когда каналы обновлялись в последний раз
        self.rx_bind_state = "bound"
        # «RX rate» модели виртуальной аппаратуры — то же число, что печатает status
        # («RX rate: 33»): команда rc_smoothing_info берёт частоту приёма отсюда.
        self.RX_RATE_HZ = 33
        self.rx_bind_deadline = 0.0
        self.rx_bind_seconds = VirtualFC.RX_BIND_SECONDS
        self.mixer_name = self.MIXER_DEFAULT
        self.mixer_names = ["TRI", "QUADP", "QUADX", "BI", "GIMBAL", "Y6", "HEX6", "FLYING_WING", "Y4",
                            "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX", "AIRPLANE", "HELI_120_CCPM",
                            "HELI_90_DEG", "VTAIL4", "HEX6H", "PPM_TO_SERVO", "DUALCOPTER", "SINGLECOPTER",
                            "ATAIL4", "CUSTOM", "CUSTOMAIRPLANE", "CUSTOMTRI", "QUADX1234"]
        self.detected_sensors = ["GYRO=MPU6000, ACC=MPU6000, BARO=BMP280 (2)"]
        self.battery_cells = 4
        self.battery_cell_voltage = 4.20   # 4S, каждая банка ровно 4.20 В (стол)
        self.mah_drawn = 0.0               # счётчик израсходованного: тикает только при работе моторов
        self._mah_t = None
        # Моторы: motor_disarmed[] из mixer.c — то, что пишет MSP_SET_MOTOR (214) со слайдеров
        # вкладки «Моторы». Прошивка НЕ сбрасывает эти значения по таймауту: пока борт не взведён,
        # mixer.c отдаёт выходы напрямую (`motor[i] = motor_disarmed[i]`), и мотор продолжает
        # крутиться на выставленном газе, пока Configurator не пришлёт нули.
        self.motor_disarmed = [1000.0] * MAX_SUPPORTED_MOTORS   # motor_disarmed[] в нуле → внешние 1000
        self.motor_output = list(self.motor_disarmed)     # MSP_MOTOR (104): внешние значения 1000…2000
        self.motor_rpm = [0.0] * MAX_SUPPORTED_MOTORS     # MSP_MOTOR_TELEMETRY (139): обороты
        self.motor_temp_c = [self.MOTOR_AMBIENT_C] * MAX_SUPPORTED_MOTORS
        self.motor_current_a = [0.0] * MAX_SUPPORTED_MOTORS
        self.motor_mah = [0.0] * MAX_SUPPORTED_MOTORS
        self.motor_voltage_v = 0.0
        self._motors_t = time.time()
        self.motor_test_arming_block = False               # флаг ARMING_DISABLED_MSP
        # «Виртуальный полёт» (SIM_* выше): сценарий газа, чтобы вкладка «Питание и батарея»
        # была живой — ток, расход мА·ч и напряжение меняются сами. Пока выключен, эмулятор
        # ведёт себя как стол: электроника ест ≈0.55 А, моторы стоят (тест со слайдеров).
        self.sim_flight = False
        self.sim_flight_phase = "idle"      # idle | flight | landing | swap
        self.sim_flight_since = 0.0         # время начала текущей фазы
        self.sim_flight_note = ""           # последнее событие: посадка, замена батареи…
        self.sim_pack_events = 0            # сколько раз батарея «кончилась» за сеанс
        self.sim_time_scale = self.SIM_FLIGHT_TIME_SCALE
        self._fs_init_state()               # «демо failsafe»: фазы, таймеры, журнал
        self.motor_output_reordering = list(range(MAX_SUPPORTED_MOTORS))
        self.dshot_commands_log = []
        # Таблицы, к которым в прошивке 4.5.4 обращаются только команды CLI (rxrange, led,
        # color, mode_color, mmix, beacon). Живут во «флеше» рядом с остальной конфигурацией,
        # чтобы Save/Reboot их не терял — как PG-таблицы прошивки.
        # Ресурсы платы (команда `resource`): назначения пинов — это конфигурация,
        # поэтому они живут во «флеше» и сбрасываются `defaults`, как PG-таблицы прошивки.
        self.resources = self._default_resources()
        self.rx_ranges = self._default_rx_ranges()               # rxChannelRangeConfigs[]
        self.led_configs = self._default_led_configs()            # ledStripStatusModeConfig.ledConfigs[]
        self.led_colors = self._default_led_colors()              # ...colors[]
        self.mode_colors = self._default_mode_colors()            # ...modeColors[]/specialColors
        self.custom_motor_mix = self._default_custom_motor_mix()  # customMotorMixer[]
        # beeperConfig.dshotBeaconOffFlags: DEFAULT_DSHOT_BEACON_OFF_FLAGS =
        # DSHOT_BEACON_ALLOWED_MODES (io/beeper.h, без USE_RACE_PRO) → RX_LOST и RX_SET выключены
        self.dshot_beacon_disabled: set[str] = {"RX_LOST", "RX_SET"}
        self.dshot_beacon_tone = 1            # dshotBeaconTone из reset-шаблона pg/beeper.c
        # flash_read/flash_write (USE_FLASH_TOOLS): «флеш» эмулятора — файл чёрного ящика.
        # Запись держим в памяти: прошивка пишет прямо в накопитель, но нам нельзя портить
        # журнал пролётов, поэтому наложение живёт до перезапуска (и сбрасывается flash_erase).
        self.flash_overlay: dict[int, bytes] = {}
        # rxFailsafeChannelConfigs: «Этап 1» вкладки Failsafe. Живёт во «флеше»,
        # заводские значения — как в pgResetFn_rxFailsafeChannelConfigs().
        self.rxfail = self._default_rxfail()
        # Сервоприводы (PG_SERVO_PARAMS / PG_SERVO_MIXER): живут во «флеше».
        self.servo_params = self._default_servo_params()
        self.servo_mix_rules = self._default_servo_mix_rules()
        self.arming_disable_names = "ARMING_DISABLED_ARM_SWITCH"
        self.uids = (0x1A2B3C4D, 0x5E6F7081, 0x92A3B4C5)
        self.cycle_time = 125
        self.cpu_load = 12
        self.boot_time = time.time()
        self.cli_active = False
        self.batch_mode = False
        self.pending_reboot = False
        self.pending_bootloader = False
        # Режим запоминающего устройства (MSP_SET_REBOOT с кодом 2/3): как настоящий борт,
        # эмулятор «отключает» MSP и отдаёт содержимое «флеша» файлами — только не по USB,
        # а на странице веб-монитора (см. /msc). Живёт ограниченное время, чтобы не застрять.
        self.msc_mode = False
        self.msc_deadline = 0.0
        self.craft_name = STRING_DEFAULTS["craft_name"]
        self.pilot_name = STRING_DEFAULTS["pilot_name"]
        self.pid_profile_names = {0: "Profile 1", 1: "Profile 2", 2: "Profile 3", 3: "Profile 4"}
        self.rate_profile_names = {0: "Actual", 1: "Actual", 2: "Actual", 3: "Actual"}
        # Профили PID и рейтов (PID_PROFILE_COUNT / CONTROL_RATE_PROFILE_COUNT).
        # Активный профиль хранится в self.settings — так CLI и MSP показывают одно и то же,
        # а переключение профиля (MSP_SELECT_SETTING) переносит значения между ними.
        self.pid_profiles = [dict(PID_PROFILE_DEFAULTS) for _ in range(PID_PROFILE_COUNT)]
        self.rate_profiles = [dict(RATE_PROFILE_DEFAULTS) for _ in range(CONTROL_RATE_PROFILE_COUNT)]
        # Профили батареи (BATTERY_PROFILE_COUNT) и активный индекс — как systemConfig
        # мастера. В 4.5.4 таких профилей нет: таблица живёт только ради CLI/MSP новой
        # прошивки, настройки vbat_* из 4.5.4 при этом не трогаются.
        self.battery_profiles = self._default_battery_profiles()
        self.active_battery_profile = 0
        self.build_key = "0" * 32
        self.current_profile = 0
        self.current_rate_profile = 0
        self.writes_log: list[str] = []
        self.stats: dict[int, int] = {}
        self.recent_msp: list[tuple[float, int, str, int, int]] = []  # (время, код, имя, запрос, ответ)
        self.recent_cli: list[tuple[float, str, str]] = []  # (время, команда, ответ)
        self._echo_store: dict[int, bytes] = {}
        # Конфигурация, записанная вкладками через MSP_SET_* (код GET -> payload)
        self.config_payloads: dict[int, bytes] = {}
        # Выбор железа датчиков во вкладке «Датчики»: 0 = AUTO, 1 = NONE
        self.sensor_config = {
            "acc_hardware": SENSOR_AUTO,
            "baro_hardware": SENSOR_AUTO,
            "mag_hardware": SENSOR_NONE,   # магнитометра нет
            "sonar_hardware": RANGEFINDER_NONE,
        }
        # Настройки OSD (вкладка «OSD»): элементы, статистика, таймеры, предупреждения.
        self.osd_config = self._default_osd_config()
        # Слоты «Корректирования в полёте» (вкладка «Корректирование в полёте»): пара
        # «диапазон AUX-канала → функция корректировки». Заполнены как у настроенного квада:
        # с пустой таблицей вкладка выглядит пустой (она показывает только включённые слоты).
        self.adjustment_ranges = self._stock_adjustment_ranges()
        # Таблица VTX (вкладка «ВИДЕОПЕРЕДАТЧИК») и состояние самого передатчика.
        # Заполненная (как у настроенного квада), чтобы Configurator не показывал
        # «таблица VTX не настроена» на вкладке «Порты».
        self.vtx_table = self._stock_vtx_table()
        self.vtx_pit_mode = False
        self.vtx_device_type = VTXDEV_UNKNOWN
        # vtxChannelActivationConditions[10] (vtx_control.h): переключение каналов VTX
        # по AUX-каналу; команда CLI `vtx`. Как во «флеше» с завода — все слоты нулевые.
        self.vtx_activation = self._default_vtx_activation()
        # Калибровка акселерометра (PG_ACCELEROMETER_CONFIG в прошивке): после «defaults»
        # флаг снят, кнопка «Калибровать» во вкладке «Датчики» его выставляет.
        self.acc_calibrated = False
        self.acc_calibration_until = 0.0
        self._cli = CliEngine(self)
        self._cli_log: list[str] = []
        # «Флеш» читаем последним: к этому моменту созданы все поля, которые он восстанавливает
        # (фичи, порты, имена, профили, UID), иначе значения перетирались бы инициализацией.
        self.load_eeprom()
        # «Чёрный ящик»: наполняем накопитель тестовыми пролётами. Если «флеш» уже собран
        # (файл рядом с настройками), он просто читается с диска — старт остаётся быстрым.
        self.blackbox.start(self.blackbox_p_interval())
        self._select_default_vtx_channel()
        self.check_feature_bits()
        # Передатчик ищется после чтения «флеша»: тип зависит от функций портов.
        self.init_vtx_device()

    # ----------------------------------------------------------------- CLI
    def note_msp(self, code: int, name: str, req_len: int, resp_len: int):
        self.stats[code] = self.stats.get(code, 0) + 1
        self.recent_msp.append((time.time(), code, name, req_len, resp_len))
        del self.recent_msp[:-40]

    def cli_execute(self, line: str) -> str:
        """CLI-строка через MSP2_CLI_COMMAND: текст с \\r\\n, без приглашения."""
        text = self._cli.execute_line(line)
        self._cli_log.extend(text.split("\r\n"))
        del self._cli_log[:-400]              # держим не больше 400 строк вывода
        self.recent_cli.append((time.time(), line, text))
        del self.recent_cli[:-40]
        if self.pending_reboot:
            # 'save'/'exit'/'defaults' перезагружают FC: связь оборвётся
            pass
        return text

    def terminal(self):
        """Терминал «сырого» CLI-порта для текущего соединения."""
        from .cli import CliTerminal
        return CliTerminal(self)

    # -------------------------------------------------------------- MSP API
    def _handle_write(self, code: int, payload: bytes, version: int) -> bytes:
        """Сквозная запись конфигурации из вкладок.

        payload запоминается под кодом «чтения» и отдаётся на GET, если размер совпадает
        с генерируемым ответом — страховка от расхождения раскладок между версиями API.
        """
        read_code = WRITE_TO_READ[code]
        self.config_payloads[read_code] = bytes(payload)
        self._log(f"[MSP] 0x{code:04X}: конфигурация сохранена ({len(payload)} байт -> 0x{read_code:04X})")
        return b""

    def handle(self, code: int, payload: bytes, version: int):
        """Возвращает payload ответа (bytes) или None, если ответа нет."""
        if self.acc_calibration_until and time.time() >= self.acc_calibration_until:
            self.acc_calibration_until = 0.0
            self.acc_calibrated = True
            self._log("[FC] калибровка акселерометра завершена")
        # 1) Запись конфигурации из вкладки: любой MSP_SET_* из таблицы пар сохраняется.
        # 45 (RX_CONFIG) и 91 (ADVANCED_CONFIG) обрабатываются по смыслу — см. ниже
        # 220 (SENSOR_ALIGNMENT) и 224 (COMPASS_CONFIG) разбираются по смыслу ниже,
        # 45 (RX_CONFIG) и 91 (ADVANCED_CONFIG) — тоже.
        # 76 (FAILSAFE), 223 (GPS_CONFIG) и 225 (GPS_RESCUE) тоже разбираются по смыслу — ниже:
# вкладка «GPS» пишет провайдера/SBAS/Galileo, и CLI должен видеть то же самое (get gps_*).
        # 93/95/202/204/218/222 (фильтры, PID, PID Advanced, Rates, дедбенды, параметры моторов)
        # разбираются по смыслу: значения уходят в те же настройки, что видит CLI, — иначе вкладка
        # «PID настройки» показывала бы одно, а CLI и «Моторы» другое.
        if code in WRITE_TO_READ and code not in SEMANTIC_SET_CODES:
            return self._handle_write(code, payload, version)

        # Смысловая запись: ответ на чтение этого блока строится из состояния, поэтому старое
        # эхо (из «флеша» прежней сборки или из прежней записи) убираем — иначе вкладка
        # увидела бы прежние значения и «изменения не сохранялись».
        read_after_write = WRITE_TO_READ.get(code) or STATE_PAIRS_EXTRA.get(code)
        if read_after_write is not None and read_after_write in self.config_payloads:
            self.config_payloads.pop(read_after_write, None)
            self._log(f"[MSP] 0x{code:04X}: старое эхо 0x{read_after_write:04X} удалено — "
                      f"ответ строится из состояния FC")

        handler = self._handlers().get(code)
        if handler is None:
            stored = self.config_payloads.get(code)
            if stored is not None:  # конфигурация, записанная вкладкой, — отдаём её
                return stored
            self.writes_log.append(f"неизвестный код 0x{code:04x} (MSP{'2' if code > 255 else '1'})")
            return b""  # отвечаем пустым payload, чтобы Configurator не ждал таймаут
        w = Writer()
        handler(w, Reader(payload))
        response = w.bytes()

        # 2) Чтение: если вкладка раньше что-то записала, отдаём записанное — но только для
        # блоков, которые эмулятор не моделирует. Для «состоянийных» блоков (фичи, сенсоры,
        # failsafe, PID, порты, сервы, VTX…) приоритет у сгенерированного ответа.
        stored = None if code in self.state_based_read_codes() else self.config_payloads.get(code)
        if stored is not None:
            # Эхо записанного значения допустимо только при совпадении длины: иначе ответ
            # вкладке разошёлся бы с прошивкой (пример: BEEPER_CONFIG — раскладка 9 байт,
            # а записать могли более короткий payload). Тогда отдаём канонический ответ.
            if stored and response and len(stored) == len(response):
                return stored
            self._log(
                f"[MSP] 0x{code:04X}: записанное значение не подходит по размеру "
                f"({len(stored)} vs {len(response)}) — отдаю сгенерированное"
            )
        return response

    def _handlers(self):
        if getattr(self, "_handler_cache", None) is None:
            self._handler_cache = {
                MSP_API_VERSION: self._h_api_version,
                MSP_FC_VARIANT: self._h_fc_variant,
                MSP_FC_VERSION: self._h_fc_version,
                MSP_BUILD_INFO: self._h_build_info,
                MSP_BOARD_INFO: self._h_board_info,
                MSP_UID: self._h_uid,
                MSP_STATUS: self._h_status,
                MSP_STATUS_EX: self._h_status_ex,
                MSP_RAW_IMU: self._h_raw_imu,
                MSP_ATTITUDE: self._h_attitude,
                MSP_ALTITUDE: self._h_altitude,
                MSP_ANALOG: self._h_analog,
                MSP_VOLTAGE_METERS: self._h_voltage_meters,
                MSP_CURRENT_METERS: self._h_current_meters,
                MSP_BATTERY_STATE: self._h_battery_state,
                MSP_RC: self._h_rc,
                MSP_MOTOR: self._h_motor,
                MSP_SERVO_CONFIGURATIONS: self._h_servo_configurations,
                MSP_SERVO_MIX_RULES: self._h_servo_mix_rules,
                MSP_SERVO: self._h_servo,
                MSP_MOTOR_TELEMETRY: self._h_motor_telemetry,
                MSP_MOTOR_3D_CONFIG: self._h_motor_3d_config,
                MSP_SET_MOTOR_3D_CONFIG: self._h_set_motor_3d_config,
                MSP2_MOTOR_OUTPUT_REORDERING: self._h_motor_output_reordering,
                MSP2_SET_MOTOR_OUTPUT_REORDERING: self._h_set_motor_output_reordering,
                MSP2_SEND_DSHOT_COMMAND: self._h_send_dshot_command,
                MSP_RAW_GPS: self._h_raw_gps,
                MSP_COMP_GPS: self._h_comp_gps,
                MSP_MIXER_CONFIG: self._h_mixer_config,
                MSP_FEATURE_CONFIG: self._h_feature_config,
                MSP_BEEPER_CONFIG: self._h_beeper_config,
                MSP_BOARD_ALIGNMENT_CONFIG: self._h_board_alignment,
                MSP_RC_TUNING: self._h_rc_tuning,
                MSP_PID: self._h_pid,
                MSP_ARMING_CONFIG: self._h_arming_config,
                MSP_SET_ARMING_CONFIG: self._h_set_arming_config,
                MSP_LOOP_TIME: self._h_loop_time,
                MSP_MISC: self._h_misc,
                MSP_MOTOR_CONFIG: self._h_motor_config,
                MSP_BOXNAMES: self._h_boxnames,
                MSP_BOXIDS: self._h_boxids,
                MSP_MODE_RANGES: self._h_mode_ranges,
                MSP_MODE_RANGES_EXTRA: self._h_mode_ranges_extra,
                MSP_CF_SERIAL_CONFIG: self._h_serial_config,
                MSP_FAILSAFE_CONFIG: self._h_failsafe_config,
                MSP_GPS_CONFIG: self._h_gps_config,
                MSP_SENSOR_CONFIG: self._h_sensor_config_read,
                MSP_SENSOR_ALIGNMENT: self._h_sensor_alignment,
                MSP_COMPASS_CONFIG: self._h_compass_config,
                MSP_ACC_TRIM: self._h_acc_trim,
                MSP2_SENSOR_CONFIG_ACTIVE: self._h_sensor_config_active,
                MSP2_COMMON_SERIAL_CONFIG: self._h_common_serial_config,
                MSP_RX_CONFIG: self._h_rx_config,
                MSP_RSSI_CONFIG: self._h_rssi_config,
                MSP_NAME: self._h_name,
                MSP2_GET_TEXT: self._h_get_text,
                MSP_ADVANCED_CONFIG: self._h_advanced_config,
                MSP_FILTER_CONFIG: self._h_filter_config,
                MSP_ADJUSTMENT_RANGES: self._h_adjustment_ranges,
                MSP_OSD_CONFIG: self._h_osd_config,
                MSP_VTX_CONFIG: self._h_vtx_config,
                MSP_VTXTABLE_BAND: self._h_vtx_table_band,
                MSP_VTXTABLE_POWERLEVEL: self._h_vtx_table_powerlevel,
                MSP2_GET_VTX_DEVICE_STATUS: self._h_vtx_device_status,
                MSP_BATTERY_CONFIG: self._h_battery_config,
                MSP_VOLTAGE_METER_CONFIG: self._h_voltage_meter_config,
                MSP_CURRENT_METER_CONFIG: self._h_current_meter_config,
                MSP_TRANSPONDER_CONFIG: self._h_transponder_config,
                MSP_BLACKBOX_CONFIG: self._h_blackbox_config,
                MSP_DATAFLASH_READ: self._h_dataflash_read,
                MSP_DATAFLASH_SUMMARY: self._h_dataflash_summary,
                MSP_SDCARD_SUMMARY: self._h_sdcard_summary,
                MSP_RXFAIL_CONFIG: self._h_rxfail_config,
                MSP_RX_MAP: self._h_rx_map,
                MSP_PID_CONTROLLER: self._h_pid_controller,
                MSP_GPS_RESCUE: self._h_gps_rescue,
                MSP_DEBUG: self._h_debug,
                MSP_SENSOR_STATUS: self._h_sensor_status,
                MSP_GPS_SV_INFO: self._h_gps_sv_info,
                # запись
                MSP_SET_FEATURE_CONFIG: self._h_set_feature_config,
                MSP_SET_SENSOR_CONFIG: self._h_set_sensor_config,
                MSP_SET_SENSOR_ALIGNMENT: self._h_set_sensor_alignment,
                MSP_SET_COMPASS_CONFIG: self._h_set_compass_config,
                MSP_SET_ACC_TRIM: self._h_set_acc_trim,
                MSP_SET_BEEPER_CONFIG: self._h_set_beeper_config,
                MSP_SET_MODE_RANGE: self._h_set_mode_range,
                MSP_SET_NAME: self._h_set_name,
                MSP_SET_BOARD_ALIGNMENT_CONFIG: self._h_set_echo,
                MSP_SET_MIXER_CONFIG: self._h_set_echo,
                MSP_SET_RX_CONFIG: self._h_set_rx_config,
                MSP_SET_RSSI_CONFIG: self._h_set_rssi_config,
                MSP_SET_FAILSAFE_CONFIG: self._h_set_failsafe_config,
                MSP_SET_PID: self._h_set_pid,
                MSP_PIDNAMES: self._h_pidnames,
                MSP_PID_ADVANCED: self._h_pid_advanced,
                MSP_SET_PID_ADVANCED: self._h_set_pid_advanced,
                MSP_SET_RC_TUNING: self._h_set_rc_tuning,
                MSP_RC_DEADBAND: self._h_rc_deadband,
                MSP_SET_RC_DEADBAND: self._h_set_rc_deadband,
                MSP_SET_FILTER_CONFIG: self._h_set_filter_config,
                MSP_SET_MOTOR_CONFIG: self._h_set_motor_config,
                MSP_SIMPLIFIED_TUNING: self._h_simplified_tuning,
                MSP_SET_SIMPLIFIED_TUNING: self._h_set_simplified_tuning,
                MSP_CALCULATE_SIMPLIFIED_PID: self._h_calculate_simplified_pid,
                MSP_CALCULATE_SIMPLIFIED_GYRO: self._h_calculate_simplified_gyro,
                MSP_CALCULATE_SIMPLIFIED_DTERM: self._h_calculate_simplified_dterm,
                MSP_VALIDATE_SIMPLIFIED_TUNING: self._h_validate_simplified_tuning,
                MSP_SELECT_SETTING: self._h_select_setting,
                MSP_COPY_PROFILE: self._h_copy_profile,
                MSP_SET_RESET_CURR_PID: self._h_set_reset_curr_pid,
                MSP_SET_MISC: self._h_set_echo,
                MSP_SET_GPS_CONFIG: self._h_set_gps_config,
                MSP_SET_ADVANCED_CONFIG: self._h_set_advanced_config,
                MSP_SET_ADJUSTMENT_RANGE: self._h_set_adjustment_range,
                MSP_SET_OSD_CONFIG: self._h_set_osd_config,
                MSP_OSD_CANVAS: self._h_osd_canvas,
                MSP_SET_OSD_CANVAS: self._h_set_osd_canvas,
                MSP_SET_VTX_CONFIG: self._h_set_vtx_config,
                MSP_SET_VTXTABLE_BAND: self._h_set_vtx_table_band,
                MSP_SET_VTXTABLE_POWERLEVEL: self._h_set_vtx_table_powerlevel,
                MSP_SET_BATTERY_CONFIG: self._h_set_battery_config,
                MSP_SET_VOLTAGE_METER_CONFIG: self._h_set_voltage_meter_config,
                MSP_SET_CURRENT_METER_CONFIG: self._h_set_current_meter_config,
                MSP_SET_BLACKBOX_CONFIG: self._h_set_blackbox_config,
                MSP_SET_RXFAIL_CONFIG: self._h_set_rxfail_config,
                MSP_SET_RX_MAP: self._h_set_rx_map,
                MSP_SET_GPS_RESCUE: self._h_set_gps_rescue,
                MSP_SET_CF_SERIAL_CONFIG: self._h_set_serial_config,
                MSP2_COMMON_SET_SERIAL_CONFIG: self._h_set_common_serial_config,
                MSP_SET_ARMING_DISABLED: self._h_set_arming_disabled,
                MSP_SET_MOTOR: self._h_set_motor,
                MSP_SET_SERVO_CONFIGURATION: self._h_set_servo_configuration,
                MSP_SET_SERVO_MIX_RULE: self._h_set_servo_mix_rule,
                MSP_SET_RAW_RC: self._h_ack,
                MSP_ACC_CALIBRATION: self._h_acc_calibration,
                MSP_MAG_CALIBRATION: self._h_ack,
                MSP_EEPROM_WRITE: self._h_eeprom_write,
                MSP_RESET_CONF: self._h_reset_conf,
                MSP_REBOOT: self._h_reboot,
                MSP_SET_RTC: self._h_ack,
                MSP_DATAFLASH_ERASE: self._h_dataflash_erase,
                MSP_SET_PASSTHROUGH: self._h_ack,
                MSP_CAMERA_CONTROL: self._h_ack,
                MSP2_SET_TEXT: self._h_set_text,
                MSP2_GET_OSD_WARNINGS: self._h_osd_warnings,
                MSP2_MCU_INFO: self._h_mcu_info,
                MSP2_GYRO_SENSOR_ACTIVE: self._h_gyro_sensor_active,
                # MSP2_SEND_DSHOT_COMMAND и MSP2_SET_MOTOR_OUTPUT_REORDERING объявлены выше,
                # в блоке моторов, — повторять их здесь нельзя (вторая запись молча затирает первую).
                MSP2_BATTERY_PROFILE: self._h_battery_profile,
                MSP2_SET_BATTERY_PROFILE: self._h_set_battery_profile,
                MSP2_CLI_SETTING: self._h_cli_setting,
                MSP2_CLI_SETTING_INFO: self._h_cli_setting_info,
                MSP2_CLI_COMMAND: self._h_cli_command,
            }
        return self._handler_cache

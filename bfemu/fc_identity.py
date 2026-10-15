"""Плата, датчики и прочие MSP-команды. Методы VirtualFC.

Примесь использует общее состояние контроллера; создание объекта — через bfemu.fc.VirtualFC.
"""
from __future__ import annotations

import math
import time
from .fc_constants import (
    ARMING_DISABLED_ARM_SWITCH,
    ARMING_DISABLED_FAILSAFE,
    ARMING_DISABLED_MSP,
    ARMING_DISABLED_RX_FAILSAFE,
    ARMING_DISABLE_FLAGS_COUNT,
    BUILD_OPTIONS,
    LED_CONFIGURABLE_COLOR_COUNT,
    LED_STRIP_MAX_LENGTH,
    MAX_BATTERY_PROFILE_NAME_LENGTH,
    MAX_NAME_LENGTH,
    MSP_REBOOT_BOOTLOADER_FLASH,
    MSP_REBOOT_BOOTLOADER_ROM,
    MSP_REBOOT_COUNT,
    MSP_REBOOT_FIRMWARE,
    MSP_REBOOT_MSC,
    MSP_REBOOT_MSC_UTC,
    PID_PROFILE_COUNT,
    RANGEFINDER_NONE,
    RC_CHANNEL_COUNT,
    RC_MAPPABLE_CHANNEL_COUNT,
    SENSOR_ACC,
    SENSOR_BARO,
    SENSOR_GPS,
    SENSOR_GYRO,
    SENSOR_MAG,
    SENSOR_NONE,
    SENSOR_RANGEFINDER,
    TEXT_BATTERY_PROFILE_NAME,
    TEXT_BUILDKEY,
    TEXT_CRAFT_NAME,
    TEXT_PID_PROFILE_NAME,
    TEXT_PILOT_NAME,
    TEXT_RATE_PROFILE_NAME,
)


class IdentityMixin:

    # ------------------------------------------------------------ настройки
    # Найденное «железо» виртуальной платы (id из таблиц прошивки).
    DETECTED_GYRO = 15        # ICM42688P
    DETECTED_ACC = 16         # ICM42688P
    DETECTED_BARO = 4         # BMP280
    DETECTED_MAG = 5          # QMC5883
    DETECTED_RANGEFINDER = 2  # TFMINI
    GYRO_1_MASK = 1 << 0      # gyroDetectionFlags: найден гироскоп 1

    FEATURE_BITS_EXPECTED = {"SOFTSERIAL": 1 << 6, "GPS": 1 << 7, "OSD": 1 << 18, "LED_STRIP": 1 << 16}

    def check_feature_bits(self):
        """Сверка с feature.h/Features.js: SOFTSERIAL = бит 6, GPS = 7, LED_STRIP = 16, OSD = 18."""
        wrong = [
            name for name, bit in self.FEATURE_BITS_EXPECTED.items()
            if self.feature_bits_by_name.get(name) != bit
        ]
        if wrong:
            self._log(
                "[FC] ВНИМАНИЕ: биты фич не совпадают с прошивкой: "
                + ", ".join(f"{n}=0x{self.feature_bits_by_name.get(n, 0):08x}" for n in wrong)
            )
        return not wrong

    # --- идентификация
    def _h_api_version(self, w, r):
        w.u8(self.msp_protocol_version)
        major, minor, _ = (int(x) for x in self.api_version.split("."))
        w.u8(major).u8(minor)

    def _h_fc_variant(self, w, r):
        w.raw(b"BTFL")

    def _h_fc_version(self, w, r):
        major, minor, patch = (int(x) for x in self.fc_version.split("."))
        w.u8(major).u8(minor).u8(patch)

    def _h_build_info(self, w, r):
        """MSP_BUILD_INFO (5): дата 11 Б, время 8 Б, git-ревизия 7 Б, затем build options.

        После даты раньше писался лишний пробел: он сдвигал разбор, из-за чего
        git-ревизия в консоли Configurator выглядела мусором, а build options
        вообще не читались (API 1.46 и новее берёт их именно отсюда).

        По build options Configurator решает, какие вкладки и функции показывать:
        вкладка «Сервоприводы» есть только при USE_SERVOS или USE_WING.
        """
        w.raw(self.build_date.ljust(11).encode()[:11])
        w.raw(self.build_time.ljust(8).encode()[:8])
        w.raw(self.git_revision.ljust(7).encode()[:7])
        for _name, code in BUILD_OPTIONS:
            w.u16(code)
        w.u16(0)  # терминатор списка

    def _h_acc_calibration(self, w, r):
        """MSP_ACC_CALIBRATION: прошивка минуту держит MCU в калибровке — у нас это ~1 с.

        Configurator ждёт, что флаг «акселерометр не откалиброван» в MSP_BOARD_INFO снимется.
        """
        if not self.acc_calibrated:
            self.acc_calibration_until = time.time() + 1.0
            self._log("[FC] калибровка акселерометра запущена (≈1 с, как на железе)")
        return

    def _h_board_info(self, w, r):
        w.raw(self.board_identifier.encode()[:4])
        w.u16(0)  # board version
        w.u8(0)  # board type
        w.u8(self.target_capabilities)
        # msp.c: sbufWriteU8(strlen(name)); sbufWriteData(name, strlen(name))
        w.ltext(self.target_name)
        w.ltext(self.board_name)
        w.ltext(self.manufacturer_id)
        w.raw(self.signature)
        w.u8(self.mcu_type_id)
        w.u8(self.configuration_state)
        w.u16(self.GYRO_SAMPLE_RATE_HZ)  # sample rate Hz (API 1.43+)
        w.u32(self._configuration_problems())  # configuration problems (API 1.43+)

    GYRO_SAMPLE_RATE_HZ = 8000
    PROBLEM_ACC_NEEDS_CALIBRATION = 1 << 0
    PROBLEM_MOTOR_PROTOCOL_DISABLED = 1 << 1

    def _configuration_problems(self) -> int:
        """Как configurationProblems в msp.c: некалиброванный акселерометр и выключенный протокол."""
        problems = 0
        if not self.acc_calibrated:
            problems |= self.PROBLEM_ACC_NEEDS_CALIBRATION
        if self._setting("motor_pwm_protocol", 0) == 9:  # DISABLED
            problems |= self.PROBLEM_MOTOR_PROTOCOL_DISABLED
        return problems

    def _h_uid(self, w, r):
        for u in self.uids:
            w.u32(u)

    # --- телеметрия/статус
    def _t(self):
        return time.time() - self.boot_time

    def _h_sensor_alignment(self, w, r):
        """MSP_SENSOR_ALIGNMENT (126) — 7 байт, как в msp.c для API < 1.47.

        Раскладка: align_gyro, align_acc (в 4.x равен гироскопу), align_mag,
        флаги найденных гироскопов, gyro_to_use, gyro_1_align, gyro_2_align.
        """
        gyro_align = self._setting("gyro_1_sensor_align", 0)
        w.u8(gyro_align)
        w.u8(gyro_align)                       # в прошивке крен/тангаж акселерометра = гироскопу
        w.u8(self._setting("align_mag", 0))
        w.u8(self.GYRO_1_MASK)                 # найден один гироскоп (gyroDetectionFlags)
        w.u8(self._setting("gyro_to_use", 0))
        w.u8(gyro_align)
        w.u8(self._setting("gyro_2_sensor_align", 0))

    def _h_set_sensor_alignment(self, w, r):
        """MSP_SET_SENSOR_ALIGNMENT (220): раскладка та же, что у чтения."""
        data = bytes(r.rest())
        if len(data) < 3:
            return
        self._store_setting("gyro_1_sensor_align", data[0])
        self._store_setting("align_mag", data[2])          # data[1] — устаревшее align_acc
        if len(data) >= 6:
            self._store_setting("gyro_to_use", data[3])
            self._store_setting("gyro_1_sensor_align", data[4])
            self._store_setting("gyro_2_sensor_align", data[5])
        names = {0: "DEFAULT", 1: "CW0", 2: "CW90", 3: "CW180", 4: "CW270", 5: "CW0FLIP",
                 6: "CW90FLIP", 7: "CW180FLIP", 8: "CW270FLIP", 9: "CUSTOM"}
        self._log(
            "[MSP] SENSOR_ALIGNMENT сохранён: гироскоп={}, магнитометр={}, gyro_to_use={}".format(
                names.get(self._setting("gyro_1_sensor_align"), self._setting("gyro_1_sensor_align")),
                names.get(self._setting("align_mag"), self._setting("align_mag")),
                self._setting("gyro_to_use"),
            )
        )

    def _h_compass_config(self, w, r):
        """MSP_COMPASS_CONFIG (133): склонение магнитометра в десятых долях градуса."""
        w.u16(self._setting("mag_declination", 0))

    def _h_set_compass_config(self, w, r):
        """MSP_SET_COMPASS_CONFIG (224)."""
        data = bytes(r.rest())
        if len(data) < 2:
            return
        self._store_setting("mag_declination", int.from_bytes(data[0:2], "little"))
        self._log(f"[MSP] COMPASS_CONFIG сохранён: склонение {self._setting('mag_declination') / 10:.1f}°")

    def _h_status(self, w, r):
        w.u16(self.cycle_time).u16(0).u16(self._active_sensor_mask())
        w.u32(self._mode_flags())
        w.u8(self.current_profile)
        w.u16(0)  # gyro cycle time

    def _h_status_ex(self, w, r):
        w.u16(self.cycle_time).u16(0).u16(self._active_sensor_mask())
        w.u32(self._mode_flags())
        w.u8(self.current_profile)
        w.u16(self.cpu_load)
        w.u8(PID_PROFILE_COUNT)  # количество профилей PID
        w.u8(self.current_rate_profile)
        w.u8(0)  # byteCount: дополнительных байт флагов режимов нет
        w.u8(ARMING_DISABLE_FLAGS_COUNT)
        w.u32(self.arming_disable_flags)
        w.u8(0)  # config state flags: перезагрузка не требуется
        w.u16(42)  # температура ядра, °C (API 1.46+)

    def _active_sensor_mask(self):
        """Что FC видит «живым». Учитываем выбор во вкладке «Датчики»: NONE = датчик выключен."""
        cfg = self.sensor_config
        mask = SENSOR_GYRO
        if "GPS" in self.features:
            # sensors(SENSOR_GPS) поднимает сама gpsInit(), то есть только с фичей GPS:
            # без неё Configurator и не должен спрашивать MSP_RAW_GPS (часть 4 ревью).
            mask |= SENSOR_GPS
        if cfg["acc_hardware"] != SENSOR_NONE:
            mask |= SENSOR_ACC
        if cfg["baro_hardware"] != SENSOR_NONE:
            mask |= SENSOR_BARO
        if cfg["mag_hardware"] != SENSOR_NONE:
            mask |= SENSOR_MAG
        if cfg["sonar_hardware"] != RANGEFINDER_NONE:
            mask |= SENSOR_RANGEFINDER
        return mask

    @property
    def arming_disable_flags(self):
        # На свежем FC арм-переключатель выключен — это и есть базовый флаг.
        flags = ARMING_DISABLED_ARM_SWITCH
        if self.motor_test_arming_block:
            # Вкладка «Моторы» на время теста шлёт MSP_SET_ARMING_DISABLED(1): борт взвести нельзя,
            # в Setup появляется причина «MSP» — как на железной плате.
            flags |= ARMING_DISABLED_MSP
        if self.rx_serial_ready() and not self.rx_link_connected:
            # rx.c: при потере сигнала прошивка поднимает ARMING_DISABLED_RX_FAILSAFE.
            # Configurator по этому биту показывает на вкладке «Приёмник» предупреждение
            # о потерянной связи и подсвечивает полосы каналов жёлтым.
            flags |= ARMING_DISABLED_RX_FAILSAFE
        if self.fs_demo and self.fs_armed:
            flags &= ~ARMING_DISABLED_ARM_SWITCH      # борт взведён: ARM-переключатель поднят
        if self.fs_demo and self.fs_phase in (self.FS_LANDED, self.FS_RX_LOSS_MONITORING):
            # failsafe.c: FAILSAFE_LANDED — setArmingDisabled(ARMING_DISABLED_FAILSAFE),
            # снимается только в FAILSAFE_RX_LOSS_RECOVERED. Пока флаг стоит, Configurator
            # в Setup показывает «Взвод запрещён: failsafe».
            flags |= ARMING_DISABLED_FAILSAFE
        return flags

    def _mode_flags(self):
        """Маска включённых режимов для MSP_STATUS — по положению тумблеров.

        В демо failsafe борт «в воздухе» и взведён, поэтому к маске добавляется позиция
        режима ARM: Configurator подсвечивает «ARM», а вкладка «Режимы» показывает борт
        взведённым — как на реальном борде под газом.
        """
        mask = self.rc_mode_mask()
        if self.fs_demo and self.fs_armed and "ARM" in self.active_boxes:
            mask |= 1 << self.active_boxes.index("ARM")
        return mask

    def _h_raw_imu(self, w, r):
        """MSP_RAW_IMU: 9 знаковых 16-битных значений — акселерометр, гироскоп, магнитометр.

        Дрон стоит на столе: по Z ровно 1 g (в единицах FC это 2048), по остальным осям —
        только шум младших разрядов. Configurator делит акселерометр на 2048 (1.0 g),
        гироскоп переводит в °/с. Раньше ответ был 24 байта вместо 18 — разбор сдвигался.
        """
        t = self._t()
        w.i16(int(math.sin(t * 31) * 4))                     # X: покой, только шум
        w.i16(int(math.sin(t * 31 + 1) * 4))                 # Y: покой, только шум
        w.i16(2048 + int(math.sin(t * 31 + 2) * 4))          # Z: 1 g (2048) + шум
        for axis in range(3):
            w.i16(int(math.sin(t * 47 + axis) * 12))         # гироскоп: ±1 °/с шума
        if self.sensor_config["mag_hardware"] != SENSOR_NONE:
            fields = (300, -120, 410)                         # ровное магнитное поле стенда
            for axis in range(3):
                w.i16(fields[axis] + int(math.sin(t * 29 + axis) * 2))
        else:
            for _ in range(3):
                w.i16(0)                                      # магнитометра нет

    def _h_attitude(self, w, r):
        """MSP_ATTITUDE: дрон неподвижен — крен, тангаж и курс 0.

        Roll/pitch прошивка пишет в десятых долях градуса, а курс — уже в градусах
        (sbufWriteU16(DECIDEGREES_TO_DEGREES(attitude.values.yaw))), и Configurator
        делит на 10 только первые два. Раньше курс уходил в десятых долях (в интерфейсе
        появлялось «3412.0»), а значения ещё и менялись во времени — фигурка коптера
        вращалась сама по себе.
        """
        w.i16(int(self._setting("acc_trim_roll", 0)) * 0)  # крен, десятые доли градуса
        w.i16(0)  # тангаж
        w.i16(0)  # курс, градусы

    def _h_acc_trim(self, w, r):
        """MSP_ACC_TRIM (240): смещения акселерометра — как accelerometerTrims в прошивке."""
        w.i16(self._setting("acc_trim_pitch", 0)).i16(self._setting("acc_trim_roll", 0))

    def _h_set_acc_trim(self, w, r):
        """MSP_SET_ACC_TRIM (239): как MSP_SET_ACC_TRIM в msp.c — pitch, затем roll."""
        data = bytes(r.rest())
        if len(data) < 4:
            return
        self._store_setting("acc_trim_pitch", int.from_bytes(data[0:2], "little", signed=True))
        self._store_setting("acc_trim_roll", int.from_bytes(data[2:4], "little", signed=True))
        self._log(
            f"[MSP] ACC_TRIM сохранён: pitch={self._setting('acc_trim_pitch')}, "
            f"roll={self._setting('acc_trim_roll')}"
        )
        return b""   # пустой ACK: вкладка «Датчики» ждёт ответа на запись

    def _h_altitude(self, w, r):
        t = self._t()
        w.i32(int((math.sin(t / 5) * 2) * 100))  # см
        w.i16(0)

    @staticmethod
    def _default_led_configs() -> list:
        """Строки ленты (ledConfigs[]): у свежего борта пусто — «0,0::C:0» на каждую позицию.

        Формат строки — generateLedConfig() из ledstrip.c: «x,y:направления:функции:цвет»
        (базовые функции «C F A L S G R», направления «N E S W U D», наложения «T Y O B V I W»).
        """
        return ["0,0::C:0"] * LED_STRIP_MAX_LENGTH

    @staticmethod
    def _default_led_colors() -> list:
        """Таблица цветов ленты: первые LED_COLOR_COUNT — hsv[] из ledstrip.c, остальные нули.

        pgResetFn_ledStripStatusModeConfig() копирует в colors[] ровно ARRAYLEN(hsv) записей.
        """
        colors = [[0, 0, 0] for _ in range(LED_CONFIGURABLE_COLOR_COUNT)]
        for index, (hue, saturation, value) in enumerate((
            (0, 0, 0),      # COLOR_BLACK
            (0, 255, 255),  # COLOR_WHITE
            (0, 0, 255),    # COLOR_RED
            (30, 0, 255),   # COLOR_ORANGE
            (60, 0, 255),   # COLOR_YELLOW
            (90, 0, 255),   # COLOR_LIME_GREEN
            (120, 0, 255),  # COLOR_GREEN
            (150, 0, 255),  # COLOR_MINT_GREEN
            (180, 0, 255),  # COLOR_CYAN
            (210, 0, 255),  # COLOR_LIGHT_BLUE
            (240, 0, 255),  # COLOR_BLUE
            (270, 0, 255),  # COLOR_DARK_VIOLET
            (300, 0, 255),  # COLOR_MAGENTA
            (330, 0, 255),  # COLOR_DEEP_PINK
        )):
            colors[index] = [hue, saturation, value]
        return colors

    @staticmethod
    def _default_mode_colors() -> dict:
        """Цвета режимов и спец-цвета ленты — defaultModeColors/defaultSpecialColors (ledstrip.c).

        Направления идут как LED_DIRECTION_COUNT в ledstrip.h: N, E, S, W, U, D. Для режима
        LED_MODE_BARO значений в заводской таблице нет (копируется только 5 строк), поэтому он
        нулевой; AUX-канал ленты по умолчанию THROTTLE (rc_alias_e: ROLL 0, PITCH 1, YAW 2,
        THROTTLE 3) — в эмуляторе газ тоже третий канал (порядок A/E/R/T).
        """
        return {
            "mode": [
                [1, 11, 2, 13, 10, 3],    # ORIENTATION: WHITE, DARK_VIOLET, RED, DEEP_PINK, BLUE, ORANGE
                [5, 11, 3, 13, 10, 3],    # HEADFREE: LIME_GREEN …
                [10, 11, 4, 13, 10, 3],   # HORIZON: BLUE, DARK_VIOLET, YELLOW …
                [8, 11, 4, 13, 10, 3],    # ANGLE: CYAN …
                [7, 11, 3, 13, 10, 3],    # MAG: MINT_GREEN …
                [0, 0, 0, 0, 0, 0],       # BARO: в заводской таблице пусто
            ],
            "special": [6, 10, 1, 0, 0, 2, 3, 6, 0, 0, 0],   # DISARMED…GPSLOCKED, дальше нули
            "aux_channel": 3,         # THROTTLE
        }

    def _h_set_feature_config(self, w, r):
        """MSP_SET_FEATURE_CONFIG: u32 маска фич — вкладка «Конфигурация» и порты."""
        mask = r.u32()
        self.features = {name for name, bit in self.feature_bits_by_name.items() if mask & bit}
        self.sync_soft_serial_ports()
        self._log(f"[MSP] фичи сохранены: {', '.join(sorted(self.features)) or '(нет)'}")

    def _h_set_name(self, w, r):
        """MSP_SET_NAME (устаревший, но вкладки ещё пишут через него): символы имени дрона."""
        name = bytes(r.rest()).decode("latin-1", "replace").strip("\x00")[:MAX_NAME_LENGTH]
        self._mirror_name_setting("craft_name", name)
        self.settings["craft_name"] = name
        self._log(f"[MSP] имя дрона: {name!r}")

    def _h_set_rx_map(self, w, r):
        """MSP_SET_RX_MAP (65): RX_MAPPABLE_CHANNEL_COUNT букв карты каналов."""
        names = {0: "A", 1: "E", 2: "R", 3: "T", 4: "1", 5: "2", 6: "3", 7: "4"}
        data = bytes(r.rest())[:RC_MAPPABLE_CHANNEL_COUNT]
        self.rx_map = "".join(names.get(byte, "1") for byte in data)
        self._log(f"[MSP] карта каналов: {self.rx_map}")
        return b""

    def _h_feature_config(self, w, r):
        w.u32(self.feature_mask)

    def _h_beeper_config(self, w, r):
        """MSP_BEEPER_CONFIG (184): флаги выключенных сигналов и флаги DShot-пищалки.

        Второе поле — dshotBeaconOffFlags: их правит команда CLI `beacon`, поэтому вкладка
        «Конфигурация» показывает то же состояние, что и консоль (beeperConfig в прошивке).
        """
        mask = 0
        for i, name in enumerate(self.beeper_names):
            if name in self.beepers_disabled:
                mask |= 1 << i
        beacon = 0
        for i, name in enumerate(self.beeper_names):
            if name in self.dshot_beacon_disabled:
                beacon |= 1 << i
        w.u32(mask).u8(self.dshot_beacon_tone).u32(beacon)

    def _h_set_beeper_config(self, w, r):
        """MSP_SET_BEEPER_CONFIG (185): та же раскладка, что у GET (msp.c 4.5.4):
        u32 off-флаги, u8 тон DShot, u32 off-флаги DShot-пищалки. Пишем в те же поля,
        что трогают CLI `beeper`/`beacon`, — вкладка «Конфигурация» и консоль не расходятся.
        """
        if len(r.data) - r.pos >= 4:
            mask = r.u32()
            self.beepers_disabled = {n for i, n in enumerate(self.beeper_names) if mask & (1 << i)}
        if len(r.data) - r.pos >= 1:
            self.dshot_beacon_tone = r.u8()
        if len(r.data) - r.pos >= 4:
            beacon = r.u32()
            self.dshot_beacon_disabled = {n for i, n in enumerate(self.beeper_names) if beacon & (1 << i)}

    def _h_board_alignment(self, w, r):
        w.i16(self.settings.get("board_align_roll", 0))
        w.i16(self.settings.get("board_align_pitch", 0))
        w.i16(self.settings.get("board_align_yaw", 0))


    def _h_set_sensor_config(self, w, r):
        """MSP_SET_SENSOR_CONFIG: acc, baro, mag, sonar (API 1.46 — 4 байта).

        Так вкладка «Датчики» выключает датчики: 1 = NONE, 0 = AUTO.
        """
        keys = ("acc_hardware", "baro_hardware", "mag_hardware", "sonar_hardware")
        for key in keys:
            if r.pos < len(r.data):
                self.sensor_config[key] = r.u8()
        self._log(
            "[MSP] конфигурация датчиков: acc={acc_hardware} baro={baro_hardware} "
            "mag={mag_hardware} sonar={sonar_hardware}".format(**self.sensor_config)
        )

    def _h_sensor_config_read(self, w, r):
        """MSP_SENSOR_CONFIG (96): acc, baro, mag, sonar — что выбрано во вкладке «Датчики»."""
        cfg = self.sensor_config
        w.u8(cfg["acc_hardware"]).u8(cfg["baro_hardware"])
        w.u8(cfg["mag_hardware"]).u8(cfg["sonar_hardware"])

    def _h_sensor_config_active(self, w, r):
        """MSP2_SENSOR_CONFIG_ACTIVE (0x300A): какие датчики реально найдены (detectedSensors[]).

        id — из таблиц прошивки (gyro.h, acceleration.h, barometer.h, compass.h,
        rangefinder.h); Configurator показывает по ним название чипа. 0 — «не найден».
        """
        cfg = self.sensor_config
        mag = self.DETECTED_MAG if cfg["mag_hardware"] != SENSOR_NONE else 0
        sonar = self.DETECTED_RANGEFINDER if cfg["sonar_hardware"] != RANGEFINDER_NONE else 0
        w.u8(self.DETECTED_GYRO).u8(self.DETECTED_ACC).u8(self.DETECTED_BARO).u8(mag).u8(sonar)

    def _h_name(self, w, r):
        # msp.c (MSP_NAME): символы без длины и без завершающего нуля
        w.raw(self.craft_name.encode("latin-1", "replace"))

    def _h_get_text(self, w, r):
        text_type = r.u8()
        mapping = {
            TEXT_PILOT_NAME: self.pilot_name,
            TEXT_CRAFT_NAME: self.craft_name,
            TEXT_PID_PROFILE_NAME: self.pid_profile_names.get(self.current_profile, "Profile 1"),
            TEXT_RATE_PROFILE_NAME: self.rate_profile_names.get(self.current_rate_profile, "Actual"),
            TEXT_BUILDKEY: self.build_key,
            TEXT_BATTERY_PROFILE_NAME: self.battery_profiles[self.active_battery_profile]["name"],
        }
        w.u8(text_type).ltext(mapping.get(text_type, ""))

    def _h_transponder_config(self, w, r):
        w.u8(0).u8(0).u8(0)

    def _h_debug(self, w, r):
        for _ in range(4):
            w.i16(0)

    # --- запись
    def _h_ack(self, w, r):
        self.writes_log.append("ack")

    def _h_set_raw_rc(self, w, r):
        """MSP_SET_RAW_RC (200): кадр каналов от внешнего источника — rxMspFrameReceive().

        В прошивке приёмник по MSP включается фичей RX_MSP (FEATURE_RX_MSP, без неё
        msp.c только отвечает ACK и кадр никуда не идёт). Поэтому и здесь каналы
        принимаются лишь при включённой фиче — иначе пишем в журнал, что кадр отброшен.
        Больше 16 каналов (MAX_SUPPORTED_RC_CHANNEL_COUNT) прошивка отвергает ошибкой.
        """
        data = bytes(r.rest())
        if len(data) % 2:
            self._log("[MSP] MSP_SET_RAW_RC: нечётная длина кадра — отвергнут")
            return
        count = len(data) // 2
        if count > RC_CHANNEL_COUNT:
            self._log(f"[MSP] MSP_SET_RAW_RC: {count} каналов — больше {RC_CHANNEL_COUNT}, "
                      "прошивка отвечает ошибкой")
            return
        frame = [int.from_bytes(data[i * 2:i * 2 + 2], "little") for i in range(count)]
        if "RX_MSP" not in self.features:
            self.writes_log.append("MSP_SET_RAW_RC: фича RX_MSP выключена — кадр не принят")
            return
        self.set_rc_channels(frame)
        self.rx_link_connected = True
        self.rc_source = f"MSP_SET_RAW_RC ({count} кан.)"
        self.note_rc_source(self.rc_source)
        self._log(f"[MSP] MSP_SET_RAW_RC: принято {count} каналов "
                  f"({frame[:4]}…) — фича RX_MSP включена")

    def _h_set_echo(self, w, r):
        """Сохраняем payload записи и отдаём его же в соответствующем GET."""
        return None

    def _h_set_arming_disabled(self, w, r):
        """MSP_SET_ARMING_DISABLED (99): msp.c читает u8 «команда» и необязательный u8
        «разрешить обход проверки на взлёт». Команда 1 — запретить взвод (ставим
        ARMING_DISABLED_MSP и, если борт уже взведён, расснаряжаем), 0 — разрешить снова.
        Именно это вкладка «Моторы» шлёт при включении и выключении теста.
        """
        data = bytes(r.rest())
        command = data[0] if data else 0
        if command:
            self.motor_test_arming_block = True
            self._log("[MSP] тест моторов: взвод запрещён (ARMING_DISABLED_MSP)")
        else:
            self.motor_test_arming_block = False
            self._log("[MSP] тест моторов выключен: взвод снова разрешён")
        self.writes_log.append(f"arming disabled set ({'on' if command else 'off'})")
        return b""

    def _h_eeprom_write(self, w, r):
        self.save_to_eeprom()

    def _h_reset_conf(self, w, r):
        """MSP_RESET_CONF.

        В прошивке это resetEEPROM() = resetConfig() + writeUnmodifiedConfigToEEPROM()
        (config.c), то есть сброс к заводским НАСТРОЙКАМ С ЗАПИСЬЮ ВО ФЛЕШ, затем ребут.
        """
        self._defaults_restore()
        self.save_to_eeprom()
        self.reboot()

    def _h_reboot(self, w, r):
        """MSP_SET_REBOOT (68): перезагрузка с указанием режима — как handleMspReboot в msp.c.

        Ответ прошивки: [режим] и, для режима запоминающего устройства, ещё байт готовности
        (1 — «флеш готов», 0 — «не готово»; тогда Configurator показывает ошибку).
        """
        mode = r.u8() if r.remaining else MSP_REBOOT_FIRMWARE
        if mode >= MSP_REBOOT_COUNT:
            self._log(f"[FC] MSP_SET_REBOOT: режим {mode} не поддерживается")
            return
        if mode in (MSP_REBOOT_MSC, MSP_REBOOT_MSC_UTC):
            busy = self.blackbox.store_ready()
            w.u8(mode).u8(1 if self.blackbox.used or busy else 0)
            if not self.blackbox.used and not busy:
                self._log("[MSC] «флеш» чёрного ящика ещё пуст — Configurator покажет «не готово»")
                return
            self.enter_msc(mode)
            return
        w.u8(mode)
        self.reboot(bootloader=mode in (MSP_REBOOT_BOOTLOADER_ROM, MSP_REBOOT_BOOTLOADER_FLASH))

    def _h_set_text(self, w, r):
        text_type = r.u8()
        length = r.u8()  # формат запроса: type, длина, символы
        raw = r.data[r.pos : r.pos + length]
        text = raw.decode("latin-1", "replace")
        if text_type == TEXT_CRAFT_NAME:
            text = text[:MAX_NAME_LENGTH]
            self._mirror_name_setting("craft_name", text)
            self.settings["craft_name"] = text
        elif text_type == TEXT_PILOT_NAME:
            text = text[:MAX_NAME_LENGTH]
            self._mirror_name_setting("pilot_name", text)
            self.settings["pilot_name"] = text
        elif text_type == TEXT_PID_PROFILE_NAME:
            self.pid_profile_names[self.current_profile] = text[:16]   # MAX_PROFILE_NAME_LENGTH
        elif text_type == TEXT_RATE_PROFILE_NAME:
            self.rate_profile_names[self.current_rate_profile] = text[:16]
        elif text_type == TEXT_BATTERY_PROFILE_NAME:
            # мастера: textSpace = MAX_BATTERY_PROFILE_NAME_LENGTH (8) у активного профиля
            self.battery_profiles[self.active_battery_profile]["name"] = \
                text[:MAX_BATTERY_PROFILE_NAME_LENGTH]

    def _h_gyro_sensor_active(self, w, r):
        w.u8(0)

    # --- CLI поверх MSP
    def _h_cli_setting(self, w, r):
        request = r.rest().decode("latin-1", "replace").split("\x00")[0].strip()
        if not request:
            return
        name, _, value = request.partition("=")
        name = name.strip()
        if value.strip():
            ok, _err = self.set_setting(name, value.strip())
            if not ok:
                # прошивка возвращает пустой ответ => Configurator считает, что FC отказал
                return
        entry = self.settings_by_name.get(name)
        if entry is None:
            return
        w.raw(f"{name} = {self.format_setting(entry)}".encode())

    def _h_cli_setting_info(self, w, r):
        raw = r.data
        if b"\x00" in raw:
            name, rest = raw.split(b"\x00", 1)
            offset = int.from_bytes(rest[:2], "little") if len(rest) >= 2 else 0
        else:
            name, offset = raw, 0
        entry = self.settings_by_name.get(name.decode("latin-1", "replace").strip())
        if entry is None:
            return
        lines = [f"pgn={entry['pg']}", f"type={entry['type']}"]
        if entry["mode"] == "MODE_LOOKUP" and entry.get("values"):
            lines.append("values=" + ",".join(entry["values"]))
        else:
            lines.append(f"min={entry['min']}")
            lines.append(f"max={entry['max']}")
        block = ("\n".join(lines)).encode()
        window = block[offset : offset + 480]
        w.u16(len(block)).raw(window)

    def _h_cli_command(self, w, r):
        raw = r.data
        if b"\x00" in raw:
            cmdline, rest = raw.split(b"\x00", 1)
            offset = int.from_bytes(rest[:2], "little")
        else:
            cmdline, offset = raw, 0
        line = cmdline.decode("latin-1", "replace")
        if offset == 0:
            # Вывод запоминаем в буфере соединения, а не в общем поле FC: иначе вторая
            # вкладка CLI (или второй инструмент) подменяла бы страницы чужой командой.
            self.store_cli_output(self.cli_execute(line))
        out = self.cli_output()
        block = out.encode("latin-1", "replace")
        window = block[offset : offset + 480]
        w.u16(len(block)).u8(0).raw(window)

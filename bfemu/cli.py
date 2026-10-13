"""
CLI-движок виртуального FC: команды, вывод, ошибки — в стиле Betaflight 4.5.4.

Два режима использования:

  * interactive_feed() — то, что идёт по «сырому» CLI-порту (Configurator пишет '#',
    ожидает "CLI", эхо символов, приглашение "\\r\\n# ", вывод до ETX 0x03);
  * execute_line() — то, что возвращает MSP2_CLI_COMMAND (0x3012): текст с \\r\\n
    без STX/ETX и без приглашения.
"""
from __future__ import annotations

import random
import time

MODELOGIC_OR = 0   # MODELOGIC_OR (rc_modes.h): логика «ИЛИ» у режимов
# Те же константы, что в fc.py (rc_modes.h): импортировать их нельзя — циклический импорт.
MODE_RANGE_STEP = 25
CHANNEL_RANGE_MIN = 900
CHANNEL_RANGE_MAX = 2100
MAX_AUX_CHANNEL_COUNT = 14

STX = 0x02
ETX = 0x03


def _pad(name: str, width: int = 24) -> str:
    return name + " " * max(0, width - len(name))


class CliEngine:
    def __init__(self, fc):
        self.fc = fc
        self.buffer = ""
        self.echo = False
        self.out = []  # накопленный текст ответа
        self._last_sound = -1   # lastSoundIdx в cliPlaySound(): следующий сигнал по кругу

    # ------------------------------------------------------------------ вывод
    def _w(self, text: str = ""):
        self.out.append(text + "\r\n")

    def _w_raw(self, text: str):
        self.out.append(text)

    def _error(self, cmd: str, message: str):
        self._w(f"###ERROR IN {cmd}: {message}")

    def _hash(self, text: str):
        self._w(f"# {text}")

    @staticmethod
    def _atoi(text: str) -> int:
        """atoi() из C: знак и цифры в начале строки, остальное игнорируется."""
        i, sign, digits = 0, 1, ""
        while i < len(text) and text[i].isspace():
            i += 1
        if i < len(text) and text[i] in "+-":
            sign = -1 if text[i] == "-" else 1
            i += 1
        while i < len(text) and text[i].isdigit():
            digits += text[i]
            i += 1
        return sign * int(digits or 0)

    def _take(self) -> str:
        text = "".join(self.out)
        self.out = []
        return text

    # ------------------------------------------------- сырой интерактивный порт
    def interactive_feed(self, data: bytes) -> bytes:
        """Возвращает байты ответа для CLI-порта (включая эхо и приглашение)."""
        out = bytearray()
        for ch in data:
            if ch in (0x0A, 0x0D):
                if self.buffer:
                    out += b"\r\n"
                    # Через fc.cli_execute — чтобы команда попала в журнал монитора
                    # (раздел «CLI-команды»), как это делает путь MSP2_CLI_COMMAND.
                    out += self.fc.cli_execute(self.buffer).encode("latin-1", "replace")
                    self.buffer = ""
                out += b"\r\n# "
            elif ch in (0x08, 0x7F):  # backspace
                if self.buffer:
                    self.buffer = self.buffer[:-1]
                    out += b"\b \b"
            elif 32 <= ch <= 126:
                self.buffer += chr(ch)
                out += bytes([ch])
        return bytes(out)

    def enter_interactive(self) -> bytes:
        self.echo = True
        self.buffer = ""
        return b"\r\nEntering CLI Mode, type 'exit' to reboot, or 'help'\r\n# "

    # ------------------------------------------------------ CLI поверх MSP
    def execute_line(self, line: str) -> str:
        self.out = []
        cmdline = line.strip()
        # комментарии и пустые строки
        if "#" in cmdline:
            cmdline = cmdline.split("#", 1)[0].strip()
        if not cmdline:
            return ""
        name, _, args = cmdline.partition(" ")
        args = args.strip()
        handler = self.COMMANDS.get(name.lower())
        if handler is None:
            if self.echo:
                self._error("input", "UNKNOWN COMMAND, TRY 'HELP'")
            else:
                self._w(f"ERR_CMD_NA: {cmdline}")
            return self._take()
        handler(self, args)
        return self._take()

    # ---------------------------------------------------------------- команды
    def cmd_help(self, args=""):
        """`help` [строка] — cliHelp(): имя, « - » описание, аргументы второй строкой.

        Строка поиска ищется и в имени, и в описании (strcasestr); без совпадений —
        «###ERROR IN help: NO MATCHES FOR '…'».
        """
        needle = args.strip().lower()
        any_matches = False
        for name, (description, call_args) in sorted(self.HELP.items()):
            if needle and needle not in name.lower() and needle not in description.lower():
                continue
            any_matches = True
            line = name
            if description:
                line += f" - {description}"
            if call_args:
                line += "\r\n\t" + call_args
            self._w(line)
        if needle and not any_matches:
            self._error("help", f"NO MATCHES FOR '{args.strip()}'")

    def cmd_version(self, args=""):
        self._hash(self.fc.banner())
        self._hash(f"config rev: {self.fc.config_rev}")
        self._hash(f"board: manufacturer_id: {self.fc.manufacturer_id}, board_name: {self.fc.board_name}")

    def cmd_status(self, args=""):
        f = self.fc
        up = int(time.time() - f.boot_time)
        sensors = ", ".join(f.detected_sensors)
        self._w(f"MCU {f.mcu} Clock=168MHz (PLLP-HSE), Vref=3.30V, Core temp={41 + int(5 * random.random())}degC")
        self._w("Stack size: 2048, Stack address: 0x1000fff0")
        self._w(f"Configuration: {f.configuration_state_name}, size: {f.eeprom_size}, max available: 8192")
        self._w("Devices detected: SPI:2, I2C:1")
        self._w("Gyros detected: gyro 1 locked dma")
        self._w(sensors)
        self._w(f"OSD: MAX7456 (30 x 13)")
        now = time.strftime("%Y-%m-%dT%H:%M:%S.000+00:00", time.gmtime())
        self._w(f"System Uptime: {up} seconds, Current Time: {now}")
        self._w("CPU:12%, cycle time: 125, GYRO rate: 8000, RX rate: 33, System rate: 9")
        self._w(f"Voltage: 1 * 0.01V ({f.battery_cells}S battery - OK)")
        self._w("I2C Errors: 0")
        self._w("SD card: Not configured")
        self._w(f"Arming disable flags: {f.arming_disable_names}")

    def cmd_tasks(self, args=""):
        self._hash("Task list")
        self._hash("")
        self._w(_pad("Task", 18) + _pad("Enabled", 9) + _pad("Period", 8) + _pad("Rate", 8) + _pad("Max", 8)
                + _pad("Avg", 8) + "Latency")
        tasks = [
            ("SYSTEM", "8192", "87", "46", "0"),
            ("PID", "125", "1", "0", "0"),
            ("GYRO", "125", "0", "0", "0"),
            ("ACCEL", "1000", "9", "3", "0"),
            ("ATTITUDE", "1000", "9", "3", "0"),
            ("RX", "30000", "1", "1", "0"),
            ("SERIAL", "1", "0", "0", "0"),
            ("BATTERY", "32", "1", "0", "0"),
            ("OSD", "60", "1", "0", "0"),
            ("MSP", "1", "0", "0", "0"),
        ]
        for t in tasks:
            self._w(_pad("1 - " + t[0], 18) + _pad("YES", 9) + _pad(t[1], 8) + _pad(t[2], 8) + _pad(t[3], 8)
                    + _pad(t[4], 8) + "0")

    # ------------------------------------------------------------ get / set
    def cmd_get(self, args=""):
        if not args:
            self._error("get", "INVALID NAME: ")
            return
        needle = args.strip().lower()
        matches = [s for s in self.fc.settings_list if needle in s["name"].lower()]
        if not matches:
            self._error("get", f"INVALID NAME: {args.strip()}")
            return
        for i, s in enumerate(matches):
            if i:
                self._w()
            self._w(f"{s['name']} = {self.fc.format_setting(s)}")
            if s["mode"] == "MODE_LOOKUP" and s.get("values"):
                self._w("Allowed values: " + ", ".join(s["values"]))
            elif s["mode"] == "MODE_DIRECT":
                self._w(f"Allowed range: {s['min']} - {s['max']}")
            elif s["mode"] == "MODE_STRING":
                lo, hi = self.fc.setting_lengths(s)
                self._w(f"String length: {lo} - {hi}")
            dv = self.fc.default_display(s)
            # cliPrintVarDefault(): «Default value» печатается, только если значение не заводское
            if dv is not None and str(dv) != str(self.fc.format_setting(s)):
                self._w(f"Default value: {dv}")

    def cmd_set(self, args=""):
        if args.strip() in ("", "*"):
            self._w("Current settings: ")
            for s in self.fc.settings_list:
                self._w(f"{s['name']} = {self.fc.format_setting(s)}")
            return
        if "=" not in args:
            self._error("set", f"INVALID NAME: {args}")
            return
        name, _, raw_value = args.partition("=")
        name = name.strip()
        raw_value = raw_value.strip()
        s = self.fc.settings_by_name.get(name)
        if s is None:
            self._error("set", f"INVALID NAME: {name}")
            return
        ok, err = self.fc.set_setting(name, raw_value)
        if not ok:
            self._error("set", err)
            return
        # прошивка на успешный set ничего не печатает

    # ------------------------------------------------------- dump / diff
    def _dump_header(self):
        self._hash("version")
        self._hash(self.fc.banner())
        self._hash(f"config rev: {self.fc.config_rev}")
        self._w()

    def _dump_blocks(self, only_diff=False):
        f = self.fc
        # printBoardName() / printManufacturerId() / printCraftName() — блок «кто это»:
        # в дампе прошивки имя дрона идёт строкой '# name: <имя или ->'.
        self._w(f"board_name {f.board_name}")
        self._w(f"manufacturer_id {f.manufacturer_id}")
        if not only_diff or f.craft_name != f._defaults.get("craft_name", ""):
            self._w()
            self._hash(f"name: {f.craft_name or '-'}")
        self._w()
        # resources / timer / dma — printResource(), printTimer(), printDmaopt() из cliDump():
        # в дампе печатаются все слоты (свободные — «NONE»), в diff — только изменённые;
        # таймеры и DMA в diff не отличаются от заводских, поэтому их там нет.
        res_lines = f.resource_dump_lines(hide_unused=False, only_diff=only_diff)
        if res_lines:
            self._hash("resources")
            for line in res_lines:
                self._w(line)
            self._w()
        if not only_diff:
            self._hash("timer")
            for line in f.timer_dump_lines():
                self._w(line)
            self._w()
            self._hash("dma")
            for line in f.dma_dump_lines(hide_unused=False):
                self._w(line)
            self._w()
        # feature
        self._hash("feature")
        for line in f.feature_lines(only_diff):
            self._w(line)
        self._w()
        # serial
        self._hash("serial")
        for line in f.serial_lines():
            self._w(line)
        self._w()
        # servo / smix — printServo() + printServoMix() из cliDump()
        servo_lines = f.servo_lines(only_diff)
        if servo_lines:
            self._hash("servo")
            for line in servo_lines:
                self._w(line)
            for line in f.smix_reverse_lines(only_diff):
                self._w(line)
            self._w()
        rules = f.smix_lines(only_diff)
        if not only_diff or f.servo_mix_rules[0]["rate"] != 0:
            # в прошивке секция печатается всегда, а 'smix reset' затем отделяется пустой строкой
            self._hash("servo mixer")
            self._w("smix reset")
            self._w()
            for line in rules:
                self._w(line)
            self._w()
        # beeper
        self._hash("beeper")
        for line in f.beeper_lines(only_diff):
            self._w(line)
        self._w()
        # map / aux
        self._hash("map")
        self._w(f"map {f.rx_map}")
        self._w()
        self._hash("aux")
        for line in f.aux_lines(only_diff):
            self._w(line)
        self._w()
        # vtx
        self._hash("vtx")
        self._w(f"set vtx_band = {f.settings['vtx_band']}")
        self._w(f"set vtx_channel = {f.settings['vtx_channel']}")
        self._w(f"set vtx_power = {f.settings['vtx_power']}")
        # printVtx(): слоты переключения каналов; в diff — только ненулевые
        for i, slot in enumerate(f.vtx_activation):
            if only_diff and not (slot["aux"] or slot["band"] or slot["channel"]
                                  or slot["power"] or slot["start_step"] or slot["end_step"]):
                continue
            self._w("vtx %d %d %d %d %d %d %d" % (
                i, slot["aux"], slot["band"], slot["channel"], slot["power"],
                900 + 25 * slot["start_step"], 900 + 25 * slot["end_step"]))
        self._w()

    def cmd_dump(self, args=""):
        f = self.fc
        self._dump_header()
        self._hash("start the command batch")
        self._w("batch start")
        self._w()
        self._dump_blocks(only_diff=False)
        for section, heading in (("HARDWARE_VALUE", "master"), ("MASTER_VALUE", "master"),
                                 ("PROFILE_VALUE", "profile"), ("PROFILE_RATE_VALUE", "rateprofile"),
                                 ("PROFILE_BATTERY_VALUE", "battery_profile")):
            lines = f.setting_lines(section)
            if not lines:
                continue
            self._hash(heading)
            for line in lines:
                self._w(line)
            self._w()
        self._hash("end the command batch")
        self._w("batch end")
        self._w()

    def cmd_diff(self, args="all"):
        f = self.fc
        self._dump_header()
        self._hash("start the command batch")
        self._w("batch start")
        self._w()
        self._hash("reset configuration to default settings")
        self._w("defaults nosave")
        self._w()
        self._dump_blocks(only_diff=True)
        changed = f.changed_setting_lines()
        if changed:
            self._hash("master")
            for line in changed:
                self._w(line)
            self._w()
        self._hash("save configuration")
        self._w("save")
        self._hash("end the command batch")
        self._w("batch end")
        self._w()

    def cmd_defaults(self, args=""):
        """'defaults' [nosave] — как cliDefaults(): сброс, затем запись во флеш и ребут."""
        tokens = args.split()
        save = "nosave" not in [t.lower() for t in tokens]
        unknown = [t for t in tokens if t.lower() != "nosave"]
        if unknown:
            self._error("defaults", "PARSING FAILED")
            return
        self.fc.reset_to_defaults()
        self._hash("resetting to defaults")
        if save:
            self.fc.save_to_eeprom()
            self.fc.reboot()

    def cmd_save(self, args=""):
        """'save' [noreboot] — пишем во флеш; без аргументов ещё и перезагружаемся."""
        token = args.strip().lower()
        if token not in ("", "noreboot"):
            self._error("save", "PARSING FAILED")
            return
        self.fc.save_to_eeprom()
        self._hash("saving")
        if token != "noreboot":
            self.fc.reboot()

    def cmd_exit(self, args=""):
        if args == "norereboot":
            self._hash("leaving CLI mode, no reboot")
            self.fc.cli_active = False
            return
        self._hash("leaving CLI mode, unsaved changes lost")
        if not self.fc.batch_mode:  # вне batch-режима теряем несохранённое — перечитываем «флеш»
            self.fc.reset_to_defaults()
            self.fc.load_eeprom()
        self.fc.cli_active = False
        self.fc.reboot()

    # ------------------------------------------------------------ feature
    def cmd_feature(self, args=""):
        """Команда 'feature' — ровно как cliFeature() в cli.c:

        без аргументов  -> 'Enabled: <включённые фичи>'
        'feature list'  -> 'Available: <все фичи сборки>'
        'feature NAME'  -> включает, печатает 'Enabled'
        'feature -NAME' -> выключает, печатает 'Disabled'
        Совпадение по началу имени (strncasecmp), как в прошивке.
        """
        f = self.fc
        names = list(f.feature_bits_by_name)
        arg = args.strip()

        if not arg:
            enabled = [name for name in names if name in f.features]
            self._w("Enabled: " + (" ".join(enabled) if enabled else ""))
            return

        if arg.lower().startswith("list"):
            self._w("Available:" + "".join(f" {name}" for name in names))
            return

        remove = arg.startswith("-")
        name = arg.lstrip("-")
        token = name.upper()
        match = next((candidate for candidate in names if candidate.startswith(token)), None)
        if match is None:
            self._error("feature", f"INVALID NAME: {name}")
            return

        if remove:
            f.features.discard(match)
            self._w("Disabled")
        else:
            f.features.add(match)
            self._w("Enabled")
        f.sync_soft_serial_ports()  # SOFTSERIAL включает/выключает соответствующие порты

    def cmd_serial(self, args=""):
        f = self.fc
        if not args:
            for line in f.serial_lines():
                self._w(line)
            return
        self._hash("WARNING: 'serial' is deprecated, assign ports with the <feature>_uart settings")

    def cmd_aux(self, args=""):
        """'aux' [index permId aux MIN MAX [logic] [linkedTo]] — cliAux() из cli.c 4.5.4.

        Без аргументов печатает все 20 слотов (в том числе пустые: 'aux 3 0 0 900 900 0 0').
        С аргументами разбор идёт прямо в слот, как в прошивке: каждое успешно разобранное
        число записывается сразу, а номер режима, которого нет в таблице, просто не считается
        (в слот при этом ничего не пишется). Потом — правила обратной совместимости: четыре
        разобранных значения → логика ИЛИ и «связи нет», пять → «связи нет», меньше четырёх
        или больше шести → слот обнуляется целиком (memset). MIN/MAX переводятся в шаги по
        25 мкс, поэтому значение вне сетки округляется вниз — как CHANNEL_VALUE_TO_STEP().
        """
        f = self.fc
        parts = args.split()
        if not parts:
            for line in f.aux_lines():
                self._w(line)
            return
        try:
            index = int(parts[0])
        except ValueError:
            self._error("aux", "PARSING FAILED")
            return
        if not (0 <= index < 20):
            self._error("aux", "INDEX NOT BETWEEN 0 AND 19")
            return
        values = []
        for token in parts[1:]:
            try:
                values.append(int(token))
            except ValueError:
                self._error("aux", "PARSING FAILED")
                return

        # В прошивке разбор пишет прямо в modeActivationCondition_t; повторяем это, чтобы
        # ошибка в одном числе не «съедала» остальные — ровно как в cliAux().
        slot = f.mode_ranges[index]
        valid = 0
        box = f.box_name_by_permanent_id.get(values[0]) if values else None
        if box is not None:
            slot["box"] = box
            valid += 1
        if len(values) > 1 and 0 <= values[1] < MAX_AUX_CHANNEL_COUNT:
            slot["aux"] = values[1]
            valid += 1
        for field, value in zip(("start", "end"), values[2:4]):
            step = (value - CHANNEL_RANGE_MIN) // MODE_RANGE_STEP
            if 0 <= step <= (CHANNEL_RANGE_MAX - CHANNEL_RANGE_MIN) // MODE_RANGE_STEP:
                slot[field] = CHANNEL_RANGE_MIN + MODE_RANGE_STEP * step
                valid += 1
        if len(values) > 4 and values[4] in (0, 1):
            slot["logic"] = values[4]
            valid += 1
        if len(values) > 5 and values[5] in f.box_name_by_permanent_id:
            slot["linked"] = values[5]
            valid += 1

        if valid == 4:                     # обратная совместимость: только диапазон
            slot["logic"] = MODELOGIC_OR
            slot["linked"] = 0
        elif valid == 5:                   # логика задана, связи нет
            slot["linked"] = 0
        elif valid != 6:                   # меньше четырёх значений — слот обнуляется
            f.clear_mode_range(index)
        self._w(f.aux_lines()[index])

    def cmd_map(self, args=""):
        if not args:
            self._w(f"map {self.fc.rx_map}")
            return
        value = args.split()[0].upper()
        if len(value) < 4 or any(c not in "AETR1234" for c in value):
            self._error("map", "INVALID NAME: " + value)
            return
        self.fc.rx_map = value

    def cmd_mixer(self, args=""):
        f = self.fc
        if not args:
            self._w(f"mixer {f.mixer_name}")
            return
        token = args.split()[0]
        if token.upper() not in f.mixer_names:
            self._error("mixer", f"INVALID NAME: {token}")
            return
        f.mixer_name = token.upper()

    def cmd_servo(self, args=""):
        """'servo' [<index> <min> <max> <middle> <rate> <forward>] — cliServo() из cli.c 4.5.4."""
        from .fc import (MAX_SUPPORTED_RC_CHANNEL_COUNT, MAX_SUPPORTED_SERVOS, SERVO_PWM_MAX,
                         SERVO_PWM_MIN)
        f = self.fc
        text = args.strip()
        if not text:
            for line in f.servo_lines():
                self._w(line)
            return
        tokens = text.split()
        for token in tokens:                  # в прошивке допустимы только цифры, минус и пробелы
            if not token.lstrip("-").isdigit():
                self._error("servo", "PARSING FAILED")
                return
        if len(tokens) != 6:
            self._error("servo", "INVALID ARGUMENT COUNT")
            return
        index, low, high, middle, rate, forward = (int(t) for t in tokens)
        if not 0 <= index < MAX_SUPPORTED_SERVOS:
            self._error("servo", "INVALID ARGUMENT COUNT")
            return
        if (not SERVO_PWM_MIN <= low <= SERVO_PWM_MAX or not SERVO_PWM_MIN <= high <= SERVO_PWM_MAX
                or not low <= middle <= high or not -100 <= rate <= 100
                or forward >= MAX_SUPPORTED_RC_CHANNEL_COUNT):
            self._error("servo", "ARGUMENT OUT OF RANGE")
            return
        f.servo_params[index] = {"min": low, "max": high, "middle": middle, "rate": rate & 0xFF,
                                 "forwardFromChannel": forward & 0xFF,
                                 "reversedSources": f.servo_params[index]["reversedSources"]}
        self._w(f"servo {index} {low} {high} {middle} {rate} {forward & 0xFF}")

    def cmd_smix(self, args=""):
        """'smix' [reset | load <mixer> | reverse <servo> <source> r|n | <8 чисел>] — cliServoMix()."""
        from .fc import (INPUT_SOURCE_COUNT, MAX_SERVO_BOXES, MAX_SERVO_RULES, MAX_SERVO_SPEED,
                         MAX_SUPPORTED_SERVOS)
        f = self.fc
        text = args.strip()
        if not text:
            for line in f.smix_lines():
                self._w(line)
            return
        lower = text.lower()
        if lower.startswith("reset"):
            for index in range(MAX_SERVO_RULES):
                f.servo_mix_rules[index] = {"targetChannel": 0, "inputSource": 0, "rate": 0,
                                            "speed": 0, "min": 0, "max": 0, "box": 0}
            for params in f.servo_params:
                params["reversedSources"] = 0
            return
        if lower.startswith("load"):
            self._error("smix", f"INVALID NAME: {text[4:].strip()}")
            return
        if lower.startswith("reverse"):
            rest = text[7:].strip().split()
            if not rest:
                self._smix_print_reverse()
                return
            if len(rest) != 3:
                self._error("smix", "INVALID ARGUMENT COUNT")
                return
            if not rest[0].lstrip("-").isdigit() or not rest[1].lstrip("-").isdigit():
                self._error("smix", "PARSING FAILED")
                return
            servo, source, flag = int(rest[0]), int(rest[1]), rest[2].lower()
            if not (0 <= servo < MAX_SUPPORTED_SERVOS and 0 <= source < INPUT_SOURCE_COUNT
                    and flag in ("r", "n")):
                self._error("smix", "ARGUMENT OUT OF RANGE")
                return
            if flag == "r":
                f.servo_params[servo]["reversedSources"] |= 1 << source
            else:
                f.servo_params[servo]["reversedSources"] &= ~(1 << source)
            self._smix_print_reverse()        # как в прошивке: после правки печатается таблица
            return
        tokens = text.split()
        if len(tokens) != 8 or any(not t.lstrip("-").isdigit() for t in tokens):
            self._error("smix", "PARSING FAILED" if len(tokens) == 8 else "INVALID ARGUMENT COUNT")
            return
        rule, target, source, rate, speed, low, high, box = (int(t) for t in tokens)
        if not (0 <= rule < MAX_SERVO_RULES and 0 <= target < MAX_SUPPORTED_SERVOS
                and 0 <= source < INPUT_SOURCE_COUNT and -100 <= rate <= 100
                and 0 <= speed <= MAX_SERVO_SPEED and 0 <= low <= 100 and 0 <= high <= 100
                and low < high and 0 <= box <= MAX_SERVO_BOXES):
            self._error("smix", "ARGUMENT OUT OF RANGE")
            return
        f.servo_mix_rules[rule] = {"targetChannel": target, "inputSource": source, "rate": rate & 0xFF,
                                   "speed": speed, "min": low, "max": high, "box": box}
        for line in f.smix_lines():
            self._w(line)

    def _smix_print_reverse(self):
        """Таблица направлений: шапка 's\\ti0\\ti1…', затем строка на каждый сервопривод."""
        from .fc import INPUT_SOURCE_COUNT
        f = self.fc
        self._w("s" + "".join(f"\ti{source}" for source in range(INPUT_SOURCE_COUNT)))
        for index, params in enumerate(f.servo_params):
            row = "".join("\tr  " if params["reversedSources"] & (1 << source) else "\tn  "
                          for source in range(INPUT_SOURCE_COUNT))
            self._w(f"{index}{row}")

    def cmd_resource(self, args=""):
        """`resource` — cliResource() 4.5.4.

        * без аргументов — печатает назначенные ресурсы строками
          `resource <имя> <индекс> <пин>` (printResource для дампа, свободные слоты скрыты);
        * `show` — «Currently active IO resource assignments: (reboot to update)» и список
          «пин: владелец [индекс]»; `show all` добавляет таблицы таймеров и DMA;
        * `<имя> <индекс> <пин|none>` — назначение и освобождение пина с проверками
          прошивки: неизвестное имя — INVALID RESOURCE NAME, индекс вне 1…N —
          INDEX NOT BETWEEN, разбор пина — strToPin(), занятый пин — NOTE из resourceCheck().
        """
        f = self.fc
        words = args.split()
        if not words:
            for line in f.resource_dump_lines():
                self._w(line)
            return
        if words[0].lower() == "show":
            for line in f.resource_show_lines():
                self._w(line)
            if len(words) > 1 and words[1].lower() == "all":
                for line in f.timer_show_lines():
                    self._w(line)
                for line in f.dma_show_lines():
                    self._w(line)
            return

        # Имя владельца: прошивка сравнивает ровно strlen(name) символов аргумента и имени —
        # strncasecmp(pch, resourceName, strlen(resourceName)), первый совпавший по таблице
        # побеждает. То есть аргумент обязан начинаться с ПОЛНОГО имени ресурса: «LED»
        # попадает в LED, а не в LED_STRIP (у того имя длиннее), «LED_STRIP» — в LED_STRIP.
        owner = None
        typed = words[0].lower()
        for name, _pins in f.RESOURCE_OWNERS:
            if typed.startswith(name.lower()):
                owner = name
                break
        if owner is None:
            self._error("resource", f"INVALID RESOURCE NAME: '{words[0]}'")
            return

        slots = f.resource_slots(owner)
        index = 0
        tokens = words[1:]
        # Прошивка читает индекс всегда, когда у владельца больше одного слота
        # (RESOURCE_VALUE_MAX_INDEX(?)=1 для одиночных): «resource BEEPER C13» проходит,
        # «resource MOTOR A00» — нет.
        value = self._atoi(tokens[0]) if tokens else 0
        if slots > 1 or value > 0:
            if not 1 <= value <= slots:
                self._error("resource", f"INDEX NOT BETWEEN 1 AND {slots}")
                return
            index = value - 1
            tokens = tokens[1:]

        kind, pin = f.parse_pin(tokens[0] if tokens else "")
        if kind is None:
            self._error("resource", "PARSING FAILED")
            return
        if kind == "none":
            f.resources[owner][index] = None
            self._w("Resource is freed")
            return
        for note in f.resource_note_lines(owner, index, pin):
            self._w(note)          # resourceCheck(): «NOTE: … already assigned to …»
        f.resources[owner][index] = pin
        self._w(f"\r\nResource is set to {pin}")

    def cmd_timer(self, args=""):
        """`timer` — cliTimer() 4.5.4.

        Прошивка сверяет строку аргументов с «list» и «show» по её длине, поэтому работают
        и сокращения (`timer sh`), а вот `timer show all` разбирается уже как пин. Формы:
        без аргументов — printTimer() (задание и расшифровка канала); `show` — showTimers();
        `list` — «NOT IMPLEMENTED YET»; `<пин>` — printTimerDetails(); `<пин> list` —
        возможные AF пина; `<пин> AF<n>` — выбор таймера; `<пин> none` — снятие таймера
        (таймеры в эмуляторе — часть платы, поэтому NOT SUPPORTED IN THIS EMULATOR).
        """
        f = self.fc
        raw = args.strip()
        if not raw:
            for line in f.timer_dump_lines():
                self._w(line)
            return
        if "list".startswith(raw.lower()):
            self._error("timer", "NOT IMPLEMENTED YET")
            return
        if "show".startswith(raw.lower()):
            for line in f.timer_show_lines():
                self._w(line)
            return
        words = raw.split()
        kind, pin = f.parse_pin(words[0])
        if kind != "pin":
            self._error("timer", "PARSING FAILED")
            return
        if len(words) == 1:
            for line in f.timer_opt_lines(pin):
                self._w(line)
            return
        timer = f.RESOURCE_PIN_TIMERS.get(pin)
        option = words[1].lower()
        if option == "list":
            if timer:
                self._w(f"# AF{timer[2]}: {timer[0]} CH{timer[1]}")
            return
        if option.startswith("af"):
            if timer and option == f"af{timer[2]}":
                self._w(f"# timer {pin}: no change: AF{timer[2]}")
                return
            self._error("timer", f"INVALID ALTERNATE FUNCTION FOR {pin}: '{words[1]}'")
            return
        if option == "none":
            self._error("timer", "NOT SUPPORTED IN THIS EMULATOR")
            return
        self._error("timer", f"INVALID TIMER OPTION FOR {pin}: '{words[1]}'")

    def cmd_dma(self, args=""):
        """`dma` — cliDmaopt() 4.5.4 (чтение заданий: сами потоки у платы фиксированы).

        * без аргументов — printDmaopt(DUMP_MASTER | HIDE_UNUSED);
        * `show` (прошивка сверяет строку аргументов по длине, так что и `dma sh`) — showDma();
        * `list` — «NOT IMPLEMENTED YET»;
        * `<устройство> <индекс>` — текущее задание: «dma ADC 1 0» и комментарий с потоком,
          пустой слот — «dma ADC 2 NONE»;
        * `pin <пин>` — задание DMA пина таймера («dma pin A00 NONE»);
        * смена задания эмулятору недоступна: потоки распаяны под железо платы.
        """
        f = self.fc
        raw = args.strip()
        if raw and "show".startswith(raw.lower()):
            for line in f.dma_show_lines():
                self._w(line)
            return
        if not raw:
            for line in f.dma_dump_lines():
                self._w(line)
            return
        words = raw.split()
        if words[0].lower() == "list":
            self._error("dma", "NOT IMPLEMENTED YET")
            return
        if words[0].lower() == "pin":
            kind, pin = f.parse_pin(words[1] if len(words) > 1 else "")
            if kind != "pin":
                self._error("dma", f"INVALID PIN: '{words[1] if len(words) > 1 else ''}'")
                return
            if len(words) == 2:
                for line in f.dma_pin_lines(pin):
                    self._w(line)
                return
            self._error("dma", "NOT SUPPORTED IN THIS EMULATOR")
            return
        device = words[0].upper()
        if device not in f.RESOURCE_DMA_DEVICES:
            self._error("dma", f"BAD DEVICE: {words[0]}")
            return
        raw_index = words[1] if len(words) > 1 else ""
        index = self._atoi(raw_index)
        if not 1 <= index <= f.RESOURCE_DMA_DEVICES[device]:
            self._error("dma", f"BAD INDEX: '{raw_index}'")
            return
        if len(words) == 2:
            for line in f.dma_opt_lines(device, index):
                self._w(line)
            return
        self._error("dma", "NOT SUPPORTED IN THIS EMULATOR")

    def cmd_board(self, args=""):
        f = self.fc
        if not args:
            self._w(f"Board name: {f.board_name}")
            self._w(f"Manufacturer id: {f.manufacturer_id}")
            return
        name = args.split()[0]
        self._hash(f"board_name set to {name}")
        f.board_name = name

    def cmd_mcu_id(self, args=""):
        self._w(self.fc.uid_hex)

    def cmd_uid(self, args=""):
        self._w(self.fc.uid_hex)

    def cmd_blackbox(self, args=""):
        """«Чёрный ящик» эмулятора: что лежит в накопителе, наполнить заново, стереть.

        Вывод — как у прошивки, только латиницей: CLI Configurator'а передаёт байты
        как есть (String.fromCodePoint), поэтому кириллица показалась бы мусором.
        Названия пролётов по-русски остаются в сводке веб-страницы эмулятора.
        """
        f = self.fc
        store = f.blackbox
        action = (args.split()[0].lower() if args else "info")
        if action in ("refill", "fill"):
            # Наполнение занимает несколько секунд (модель пролётов + кодирование),
            # поэтому запускаем в фоне: CLI и MSP остаются отзывчивыми.
            store.force_refill(f.blackbox_p_interval())
            self._hash("blackbox: refilling flash with 6 test flights in background")
        elif action == "erase":
            store.erase()
        elif action not in ("info", "show", ""):
            self._error("blackbox", "UNKNOWN ARGUMENT, TRY 'blackbox info|refill|erase'")
            return
        p_interval = f.blackbox_p_interval() or 1
        used, total = store.used, store.size
        self._hash("Blackbox (dataflash)")
        self._w(f"MCU flash: W25Q128, sectors {store.sectors}")
        self._w(f"device {int(f.settings.get('blackbox_device', 1) or 0)} (0 NONE, 1 SPIFLASH), "
               f"record 1/{1 << f.blackbox_sample_rate()} loop "
               f"= {1_000_000 // (p_interval * 1000)} Hz, P interval {p_interval}")
        self._w(f"used {used} of {total} bytes, free {max(0, total - used)}")
        for entry in store.catalog():
            # Ключ пролёта — латиницей (takeoff_landing, motor_fault, …): он же в индексе.
            name = entry["key"] or f"log{entry['index']}"
            self._w(f"log {entry['index']}: {name} - {entry['size']} bytes, "
                    f"{entry['duration']:.1f} s, {entry['rate_hz']} Hz, {entry['start_datetime']}")
        self._w(f"note: all {len(store.catalog())} logs are stored in ONE dataflash dump; "
                f"the viewer switches between them in its 'Log' list")
        self._w("buttons: 'Save to file' writes that dump; 'reboot to MSC' publishes "
                "LOG0000N.BBL files on the emulator web page /msc")
        self._w("enter 'blackbox refill' to fill the flash with test flights again")

    def cmd_sim_flight(self, args=""):
        """`sim_flight` — команда эмулятора: «виртуальный полёт» (живое питание).

        На столе вкладка «Питание и батарея» стоит мёртвой: моторы не крутятся, ток
        нулевой, счётчик мА·ч не растёт. `sim_flight on` включает сценарий полёта:
        газ по кривой (взлёт, висение с подруливанием, «газ в пол» каждые 25 с),
        ток 3…27 А, напряжение проседает и постепенно падает по мере разряда,
        счётчик мА·ч растёт. Выключить — `sim_flight off` (то же делает кнопка на
        веб-мониторе). Печатаем латиницей: CLI Configurator'а передаёт байты как есть.
        """
        f = self.fc
        words = args.split()
        action = words[0].lower() if words else "status"
        if action in ("on", "off"):
            f.set_sim_flight(action == "on")
        elif action == "reset":
            f.mah_drawn = 0.0
            f.motor_mah = [0.0] * len(f.motor_mah)
            self._hash("sim_flight: drawn counter cleared (0 mAh)")
            return
        elif action == "scale":
            if len(words) < 2 or not words[1].isdigit() or not 1 <= int(words[1]) <= 600:
                self._error("sim_flight", "SCALE MUST BE 1..600")
                return
            f.sim_time_scale = float(int(words[1]))
        elif action not in ("status", "show", ""):
            self._error("sim_flight", "UNKNOWN ARGUMENT, TRY 'sim_flight on|off|reset|status|scale <1..600>'")
            return

        power = f.power_state()
        sim = power["sim"]
        # Фазу печатаем латиницей: русский текст CLI Configurator'а показал бы мусором.
        phases = {"idle": "idle", "flight": "flying", "landing": "landing after low battery",
                  "swap": "new pack being fitted"}
        drawn_per_second = power["current"] * sim["time_scale"] / 3.6
        self._hash("Virtual flight (emulator)")
        self._w(f"state: {'ON - flight' if sim['on'] else 'OFF - bench, motors idle'} "
                f"({phases.get(sim['phase'], sim['phase'])}, {sim['seconds']:.1f} s)")
        self._w(f"battery: {power['cells']}S, {power['voltage']:.2f} V "
                f"(rest {power['rest_voltage']:.2f} V), {power['mah']} of {power['capacity']} mAh, "
                f"status {power['state_name']}")
        self._w(f"current: {power['current']:.2f} A "
                f"(electronics {f.SIM_IDLE_CURRENT_A:.2f} A + motors)")
        self._w(f"drawn: {power['mah']} mAh, now about {drawn_per_second:.1f} mAh per second "
                f"(time scale x{sim['time_scale']:.0f})")
        self._w(f"motors: {' '.join(str(v) for v in power['motors'])} "
                f"(1000 = stopped, gas by scenario when ON)")
        if sim["packs"]:
            self._w(f"packs replaced: {sim['packs']} (battery emptied, landed, new pack fitted)")
        self._w("'sim_flight on' starts it, 'off' stops, 'reset' clears mAh, "
                "'scale <1..600>' sets the time scale, 'status' shows this")

    # ------------------------------------------------------------------ демо failsafe
    def cmd_sim_failsafe(self, args=""):
        """`sim_failsafe` — команда эмулятора: демонстрация работы failsafe (для студентов).

        Порт машины фаз failsafe.c 4.5.4: по кнопке теряется сигнал, идут страж-время
        (Этап 1, каналы по rxfail), затем ТОЧКА ОТКАТА — вход в Этап 2 с процедурой
        AUTO-LAND / DROP / GPS-RESCUE, моторы в 0 и запрет взвода; возврат сигнала
        выводит борт из failsafe. Времена — прошивочные (failsafe.c), масштаб времени
        только растягивает показ: `sim_failsafe scale 1|0.5|0.25`.

        Печатаем латиницей: CLI Configurator'а передаёт байты как есть.
        """
        f = self.fc
        words = args.split()
        action = words[0].lower() if words else "status"
        # Демо тикает по опросу страницы; из одной консоли шаг делать некому —
        # поэтому команда сама двигает часы демо (иначе фазы «стоят» между командами).
        f._motors_step()

        if action in ("on", "off"):
            f.set_sim_failsafe(action == "on")
        elif action == "loss":
            f.set_sim_failsafe(True)
            f.sim_failsafe_loss()
        elif action == "restore":
            f.sim_failsafe_restore()
        elif action == "reset":
            f.sim_failsafe_reset()
        elif action == "arm":
            f.set_sim_failsafe(True)
            f.sim_failsafe_arm(True, throttle=1500)      # «в воздухе»: газ висения
        elif action == "disarm":
            f.sim_failsafe_arm(False)
        elif action == "box":
            if len(words) < 2 or words[1].lower() not in ("on", "off"):
                self._error("sim_failsafe", "BOX MUST BE 'on' OR 'off'")
                return
            f.set_sim_failsafe_box(words[1].lower() == "on")
        elif action == "scenario":
            if len(words) < 2 or words[1].lower() not in ("none", "low_sats", "hover_low"):
                self._error("sim_failsafe", "SCENARIO MUST BE none|low_sats|hover_low")
                return
            f.set_sim_failsafe_scenario(words[1])
        elif action == "switch":
            # тумблер FAILSAFE (вкладка «Режимы»): 0 STAGE1, 1 KILL, 2 STAGE2
            if len(words) < 2 or words[1].lower() not in ("0", "1", "2", "stage1", "kill", "stage2"):
                self._error("sim_failsafe", "SWITCH MUST BE 0|1|2 (STAGE1|KILL|STAGE2)")
                return
            modes = {"0": 0, "stage1": 0, "1": 1, "kill": 1, "2": 2, "stage2": 2}
            f.set_sim_failsafe_switch(modes[words[1].lower()])
        elif action == "procedure":
            names = {"auto-land": 0, "auto": 0, "drop": 1, "gps-rescue": 2, "gps": 2}
            if len(words) < 2 or words[1].lower() not in names:
                self._error("sim_failsafe", "PROCEDURE MUST BE auto-land|drop|gps-rescue")
                return
            f._store_setting("failsafe_procedure", names[words[1].lower()])
            self._hash(f"failsafe_procedure set to {f._fs_config()['procedure']} "
                       f"({f.FS_PROCEDURE_NAMES[f._fs_config()['procedure']]})")
        elif action == "scale":
            if len(words) < 2 or words[1] not in ("1", "0.5", "0.25"):
                self._error("sim_failsafe", "SCALE MUST BE 1, 0.5 OR 0.25")
                return
            f.set_sim_failsafe_scale(float(words[1]))
        elif action not in ("status", "show", ""):
            self._error("sim_failsafe", "UNKNOWN ARGUMENT, TRY 'sim_failsafe on|off|loss|restore|"
                        "reset|arm|disarm|box on|off|switch 0|1|2|scenario none|low_sats|"
                        "hover_low|procedure auto-land|drop|gps-rescue|scale 1|0.5|0.25|status'")
            return

        f._motors_step()                    # и второй раз — чтобы цифры были «сейчас»
        st = f.sim_failsafe_state()
        cfg = st["config"]
        self._hash("Failsafe demo (emulator)")
        if not st["demo"]:
            self._w("state: OFF - bench, failsafe demo not running")
        else:
            self._w(f"state: ON - {'armed, link OK' if st['link'] else 'SIGNAL LOST'}"
                    f"{', receiving (rxLinkState UP)' if st['receiving'] else ', not receiving'}"
                    f"{', by box switch' if st['box_failsafe'] else ''}")
            self._w(f"phase: {st['phase_en']} - {self._en_phase(st)}")
            self._w(f"timeline: {' '.join(self._en_timeline(st))}")
            self._w(f"rollback point: {self._en_rollback(st)}")
            self._w(f"procedure: {st['procedure_name']} - {self._en_procedure(st)} "
                   f"(failsafe_procedure = {st['procedure']})")
            self._w(f"switch mode: {f.FS_SWITCH_MODE_EN[st['switch_mode']]} "
                   f"(failsafe_switch_mode = {st['switch_mode']}, "
                   f"BOX FAILSAFE {'ON' if st['box_failsafe'] else 'off'})")
            self._w(f"stage 1: delay {cfg['delay']:.1f} s, recovery {cfg['recovery_delay']:.1f} s, "
                   f"throttle low {cfg['throttle_low_delay']:.0f} s, stick threshold {cfg['stick_threshold']} %")
            self._w(f"channels: {' '.join(str(v) for v in st['channels'][:8])} ... "
                   f"(16 channels, MSP_RC)")
            self._w(f"stage 1 values: {' '.join(str(v) for v in st['stage1'][:8])} ... "
                   f"(rxfail: AUTO/HOLD/SET)")
            self._w(f"motors: {' '.join(str(int(round(v))) for v in f.motor_disarmed)} "
                   f"(1000 = stopped)")
            self._w(f"arming: {'ALLOWED' if not f.arming_disable_flags else 'DISABLED'} "
                   f"(flags 0x{f.arming_disable_flags:08X})"
                   f"{', ' + st['disarm_reason'] if st['disarm_reason'] else ''}"
                   f"{', beeper ' + st['beeper'] if st['beeper'] else ''}")
            if st["procedure"] == 2 or st["rescue"]["phase"] not in (0,):
                r = st["rescue"]
                self._w(f"gps rescue: {r['phase_name']} - alt {r['alt_m']:.1f} m, "
                       f"home {r['dist_m']:.1f} m, throttle {r['throttle']}, "
                       f"sats {int(f.gps_solution()['sats'])}, scenario {st['scenario']}")
            self._w(f"time scale: x{st['scale']:g} ({st['millis'] / 1000.0:.1f} s of flight shown)")
            for ms, text in st["log_en"][-8:]:
                self._w(f"  {ms:5.2f} s  {text}")
        self._w("'sim_failsafe on' enables it ('loss' cuts the link), 'restore' brings the link back, "
                "'arm'/'disarm' set the craft, 'box on|off' works the FAILSAFE switch, "
                "'scenario' picks a rescue check, 'scale 1|0.5|0.25' sets the show speed")

    def _en_phase(self, st) -> str:
        """Фаза демо по-английски: русские подписи в CLI Configurator'а показываются мусором."""
        if st["phase_en"] == "IDLE" and st["rollback"] and st["rollback"]["possible"] \
                and st["rollback"]["left_s"] is not None:
            return "idle, stage 1 guard time running (stage 1 values on the channels)"
        return {"IDLE": "idle (link up, failsafe watching)",
                "RX_LOSS_DETECTED": "rx loss detected - stage 1 guard time running",
                "LANDING": "stage 2 - auto landing, throttle fading out",
                "LANDED": "stage 2 - motors stopped",
                "RX_LOSS_MONITORING": "stage 2 done, waiting for the link",
                "RX_LOSS_RECOVERED": "link back - leaving failsafe",
                "GPS_RESCUE": "stage 2 - GPS rescue"}.get(st["phase_en"], st["phase_en"])

    def _en_timeline(self, st) -> list:
        out = []
        for step in st["timeline"]:
            text = step.get("en") or step["name"]
            if step.get("done"):
                out.append(f"[x] {text}")
            elif step.get("active"):
                left = f" ({step['left_ms'] / 1000.0:.1f} s left)" if step.get("left_ms") else ""
                out.append(f"[>] {text}{left}")
            else:
                out.append(f"[ ] {text}")
        return out

    @staticmethod
    def _en_rollback(st) -> str:
        if not st["demo"]:
            return "n/a"
        if st["rollback"]["possible"]:
            left = st["rollback"].get("left_s")
            tail = f", {left:.1f} s of guard time left" if left is not None else ""
            return f"POSSIBLE - signal recovery returns control{tail}"
        return "IMPOSSIBLE - stage 2 is running, the procedure cannot be undone"

    @staticmethod
    def _en_procedure(st) -> str:
        return {"AUTO-LAND": "land where it is, motors off after the off delay",
                "DROP": "cut the motors at once",
                "GPS-RESCUE": "fly home by GPS rescue"}.get(st["procedure_name"], "")

    def cmd_flash_info(self, args=""):
        self._hash("Flash sectors")
        self._w("Flash: JEDEC ID: 0xEF4018 0xEF4018")
        self._w("Total flash size: 8388608")
        self._w("Flash size: 8192, sectors: 16")

    def cmd_batch(self, args=""):
        f = self.fc
        token = args.split()[0].lower() if args else ""
        if token == "start":
            f.batch_mode = True
        elif token == "end":
            f.batch_mode = False
        else:
            self._error("batch", "INVALID ARGUMENT COUNT")

    def cmd_bl(self, args=""):
        self._hash("restarting in ROM bootloader mode")
        self.fc.reboot(bootloader=True)

    def cmd_vtx(self, args=""):
        """`vtx` — cliVtx() из cli.c 4.5.4: переключение каналов VTX по AUX-каналу.

        Без аргументов — все 10 слотов `vtx i aux band channel power start end`
        (start/end — из шагов 900+25·step). С аргументами пишет слот: aux 0…13,
        band/channel/power — в границах таблицы (включая 0), диапазон 900…2100.
        Шесть валидных аргументов — слот пишется и печатается; иначе слот обнуляется
        и печатается INVALID ARGUMENT COUNT (как в прошивке).
        """
        f = self.fc
        words = args.split()
        if not words:
            for i, slot in enumerate(f.vtx_activation):
                self._w("vtx %d %d %d %d %d %d %d" % (
                    i, slot["aux"], slot["band"], slot["channel"], slot["power"],
                    900 + 25 * slot["start_step"], 900 + 25 * slot["end_step"]))
            return
        index = self._atoi(words[0])
        if not 0 <= index < len(f.vtx_activation):
            self._error("vtx", f"INDEX NOT BETWEEN 0 AND {len(f.vtx_activation) - 1}")
            return
        table = f.vtx_table
        slot = f.vtx_activation[index]
        valid = 0
        if len(words) > 1:                                  # auxChannelIndex
            val = self._atoi(words[1])
            if 0 <= val < 14:                               # MAX_AUX_CHANNEL_COUNT = 18 - 4
                slot["aux"] = val
                valid += 1
        if len(words) > 2:                                  # band
            val = self._atoi(words[2])
            if 0 <= val <= table["bands"]:
                slot["band"] = val
                valid += 1
        if len(words) > 3:                                  # channel
            val = self._atoi(words[3])
            if 0 <= val <= table["channels"]:
                slot["channel"] = val
                valid += 1
        if len(words) > 4:                                  # power
            val = self._atoi(words[4])
            if 0 <= val <= table["powerlevels"]:
                slot["power"] = val
                valid += 1
        for pos, key in ((5, "start_step"), (6, "end_step")):   # processChannelRangeArgs
            if len(words) > pos:
                # CHANNEL_VALUE_TO_STEP: constrain(900…2100) → шаг всегда валиден
                val = max(900, min(2100, self._atoi(words[pos])))
                slot[key] = (val - 900) // 25
                valid += 1
        if valid != 6:
            f.vtx_activation[index] = {"aux": 0, "band": 0, "channel": 0,
                                       "power": 0, "start_step": 0, "end_step": 0}
            self._error("vtx", "INVALID ARGUMENT COUNT")
            return
        self._w("vtx %d %d %d %d %d %d %d" % (
            index, slot["aux"], slot["band"], slot["channel"], slot["power"],
            900 + 25 * slot["start_step"], 900 + 25 * slot["end_step"]))

    def cmd_vtxtable(self, args=""):
        """`vtxtable` — таблица частот VTX (cliVtxTable в cli.c).

        Без аргументов печатает таблицу в том же виде, в каком её принимает на вход
        (`vtxtable band ...`), поэтому вывод можно скопировать и вставить обратно.
        """
        table = self.fc.vtx_table
        tokens = args.split()
        if not tokens:
            self._w(f"vtxtable bands {table['bands']}")
            self._w(f"vtxtable channels {table['channels']}")
            for band in range(table["bands"]):
                name = table["band_names"][band].rstrip()
                letter = table["band_letters"][band]
                flag = " FACTORY" if table["is_factory"][band] else " CUSTOM "
                freqs = "".join(f" {table['frequencies'][band][ch]:4d}" for ch in range(table["channels"]))
                self._w(f"vtxtable band {band + 1} {name} {letter}{flag}{freqs}")
            self._w(f"vtxtable powerlevels {table['powerlevels']}")
            values = "".join(f" {v}" for v in table["power_values"][: table["powerlevels"]])
            self._w(f"vtxtable powervalues{values}")
            labels = "".join(" " + table["power_labels"][i].replace(" ", "")
                             for i in range(table["powerlevels"]))
            self._w(f"vtxtable powerlabels{labels}")
            return

        sub, rest = tokens[0].lower(), tokens[1:]

        if sub == "bands":
            if not rest:
                self._error("vtxtable", "PARSING FAILED")
                return
            count = self._atoi(rest[0])
            if count < 0 or count > 8:
                self._error("vtxtable", "BAND COUNT NOT BETWEEN 0 AND 8")
                return
            if count < table["bands"]:
                for band in range(count, table["bands"]):
                    self._vtx_clear_band(band)
            table["bands"] = count
            return

        if sub == "channels":
            if not rest:
                self._error("vtxtable", "PARSING FAILED")
                return
            count = self._atoi(rest[0])
            if count < 0 or count > 8:
                self._error("vtxtable", "CHANNEL COUNT NOT BETWEEN 0 AND 8")
                return
            if count < table["channels"]:
                for band in range(8):
                    for channel in range(count, 8):
                        table["frequencies"][band][channel] = 0
            table["channels"] = count
            return

        if sub == "powerlevels":
            if not rest:
                return
            count = self._atoi(rest[0])
            if count < 0 or count > 8:
                self._error("vtxtable", "POWER LEVEL COUNT NOT BETWEEN 0 AND 8")
                return
            if count < table["powerlevels"]:
                for i in range(count, 8):
                    table["power_values"][i] = 0
                    table["power_labels"][i] = self.fc._pad(f"LV{i}", 3)
            table["powerlevels"] = count
            return

        if sub == "powervalues":
            levels = table["powerlevels"]
            if len(rest) < levels:
                self._error("vtxtable", f"NOT ENOUGH VALUES (EXPECTED {levels})")
                return
            if len(rest) > levels:
                self._error("vtxtable", f"TOO MANY VALUES (EXPECTED {levels})")
                return
            values = [self._atoi(v) for v in rest]
            for i in range(8):
                table["power_values"][i] = values[i] if i < len(values) else 0
            return

        if sub == "powerlabels":
            levels = table["powerlevels"]
            if len(rest) < levels:
                self._error("vtxtable", f"NOT ENOUGH LABELS (EXPECTED {levels})")
                return
            if len(rest) > levels:
                self._error("vtxtable", f"TOO MANY LABELS (EXPECTED {levels})")
                return
            for i, label in enumerate(rest):
                table["power_labels"][i] = self.fc._pad(label[:3].upper(), 3)
            return

        if sub == "band":
            if len(rest) < 3:
                self._error("vtxtable", "PARSING FAILED")
                return
            band = self._atoi(rest[0]) - 1
            if band < 0 or band >= table["bands"]:
                self._error("vtxtable", f"BAND NUMBER NOT BETWEEN 1 AND {table['bands']}")
                return
            name = rest[1].upper()
            letter = rest[2].upper()[:1] or " "
            freqs, is_factory = [], False
            for index, token in enumerate(rest[3:]):
                if index == 0 and not token[:1].isdigit():
                    if token.upper() == "FACTORY":
                        is_factory = True
                    elif token.upper() == "CUSTOM":
                        is_factory = False
                    else:
                        self._error("vtxtable", f"INVALID FACTORY FLAG {token} (EXPECTED FACTORY OR CUSTOM)")
                        return
                    continue
                if self._atoi(token) < 0:
                    self._error("vtxtable", f"INVALID FREQUENCY {token}")
                    return
                freqs.append(self._atoi(token))
            channels = table["channels"]
            if len(freqs) < channels:
                self._error("vtxtable", f"NOT ENOUGH FREQUENCIES (EXPECTED {channels})")
                return
            if len(freqs) > channels:
                self._error("vtxtable", f"TOO MANY FREQUENCIES (EXPECTED {channels})")
                return
            table["band_names"][band] = self.fc._pad(name, 8)
            table["band_letters"][band] = letter
            table["is_factory"][band] = is_factory
            for channel in range(channels):
                table["frequencies"][band][channel] = freqs[channel]
            if band + 1 == self.fc._setting("vtx_band", 0):
                current = self.fc._setting("vtx_channel", 0)
                if 1 <= current <= channels:
                    self.fc._store_setting("vtx_freq", table["frequencies"][band][current - 1])
            return

        self._error("vtxtable", f"INVALID SUBCOMMAND {tokens[0]}")

    def _vtx_clear_band(self, band: int):
        """vtxTableConfigClearBand(): заглушки как в прошивке."""
        table = self.fc.vtx_table
        for channel in range(8):
            table["frequencies"][band][channel] = 0
        table["band_names"][band] = self.fc._pad(f"BAND{band + 1}", 8)
        table["band_letters"][band] = str(band + 1)
        table["is_factory"][band] = False

    def cmd_vtx_info(self, args=""):
        """`vtx_info` — уровни мощности передатчика (cliVtxInfo в cli.c)."""
        if self.fc._vtx_device_type() == 255:   # VTXDEV_UNKNOWN
            self._error("vtx_info", "NO VTX")
            return
        table = self.fc.vtx_table
        if not table["powerlevels"]:
            self._error("vtx_info", "NO POWER VALUES DEFINED")
            return
        import math as _math
        for index in range(table["powerlevels"]):
            power = table["power_values"][index]
            dbm = round(10 * _math.log10(power)) if power > 0 else 0
            self._w(f"level {dbm} dBm, power {power} mW")

    def cmd_gpspassthrough(self, args=""):
        self._hash("Gps passthrough started")

    def cmd_osd_layout(self, args=""):
        self._error("osd_layout", "PARSING FAILED")

    # =====================================================================================
    # Команды прошивки 4.5.4, которых в эмуляторе не было: раньше они отвечали
    # «###ERROR IN input: UNKNOWN COMMAND, TRY 'HELP'». Вывод — по cli.c 4.5.4 (форматы
    # cliPrintLinef разобраны построчно), состояние — из таблиц VirtualFC.
    # Три команды (escprog, gyroregisters, serialpassthrough) работают с оборудованием и
    # драйверами, которых в эмуляторе нет: они отвечают ошибкой эмулятора, но уже не
    # «UNKNOWN COMMAND» — подробности в README, раздел про CLI.
    # =====================================================================================

    ALL_MOTORS = 255        # ALL_MOTORS (motor.h): индекс «все моторы» в motor/dshotprog/escprog
    RXFAIL_MODE_CHARACTERS = "ahs"     # rxFailsafeModeCharacters[]: AUTO/HOLD/SET в cli.c

    # ------------------------------------------------------------------ профили PID/рейтов
    def cmd_battery_profile(self, args=""):
        """`battery_profile` [index] — cliBatteryProfile() мастера BF.

        Команды нет в 4.5.4 — профили батареи появились после неё. Поведение и текст ошибки —
        по cli.c мастера: без аргумента печатает активный профиль, с индексом переключает,
        вне 0…BATTERY_PROFILE_COUNT-1 — «BATTERY PROFILE OUTSIDE OF [0..N]».
        """
        from .fc import BATTERY_PROFILE_COUNT
        f = self.fc
        if not args.strip():
            self._w(f"battery_profile {f.get_battery_profile_index()}")
            return
        index = self._atoi(args)        # atoi(): 'battery_profile abc' даёт профиль 0
        if 0 <= index < BATTERY_PROFILE_COUNT:
            f.change_battery_profile(index)
            self._w(f"battery_profile {f.get_battery_profile_index()}")
        else:
            self._error("battery_profile", f"BATTERY PROFILE OUTSIDE OF [0..{BATTERY_PROFILE_COUNT - 1}]")

    def cmd_profile(self, args=""):
        """`profile` [index] — cliProfile(): печатает активный профиль, с индексом — меняет.

        В 4.5.4 это глобальный профиль PID (`profile 0` → `profile N` через changePidProfile),
        а не «PID profile» с подкомандами; индекс вне 0…3 — «PROFILE OUTSIDE OF [0..3]».
        """
        from .fc import PID_PROFILE_COUNT
        if not args.strip():
            self._w(f"profile {self.fc.current_profile}")
            return
        index = self._atoi(args)        # atoi(): 'profile abc' в прошивке тоже даёт профиль 0
        if 0 <= index < PID_PROFILE_COUNT:
            self.fc.select_profile(index)
            self._w(f"profile {self.fc.current_profile}")
        else:
            self._error("profile", f"PROFILE OUTSIDE OF [0..{PID_PROFILE_COUNT - 1}]")

    def cmd_rateprofile(self, args=""):
        """`rateprofile` [index] — cliRateProfile(): то же для профиля рейтов (0…3)."""
        from .fc import CONTROL_RATE_PROFILE_COUNT, RATEPROFILE_MASK
        if not args.strip():
            self._w(f"rateprofile {self.fc.current_rate_profile}")
            return
        index = self._atoi(args)
        if 0 <= index < CONTROL_RATE_PROFILE_COUNT:
            self.fc.select_profile(index | RATEPROFILE_MASK)
            self._w(f"rateprofile {self.fc.current_rate_profile}")
        else:
            self._error("rateprofile", f"RATE PROFILE OUTSIDE OF [0..{CONTROL_RATE_PROFILE_COUNT - 1}]")

    # ------------------------------------------------------------------ приёмник
    def cmd_rxrange(self, args=""):
        """`rxrange` — cliRxRange(): 4 строки диапазонов, `reset`, `rxrange <канал> <мин> <макс>`.

        Границы правки — PWM_PULSE_MIN…PWM_PULSE_MAX (750…2250) из rx.h, как в cli.c;
        заводские значения каналов — 1000…2000 (pgResetFn_rxChannelRangeConfigs).
        """
        from .fc import NON_AUX_CHANNEL_COUNT, PWM_PULSE_MAX, PWM_PULSE_MIN
        f = self.fc
        text = args.strip()
        if not text:
            for index, entry in enumerate(f.rx_ranges):
                self._w(f"rxrange {index} {entry['min']} {entry['max']}")
            return
        if text.lower() == "reset":
            f.rx_ranges = f._default_rx_ranges()       # resetAllRxChannelRangeConfigurations()
            return
        tokens = text.split()
        index = self._atoi(tokens[0])
        if not 0 <= index < NON_AUX_CHANNEL_COUNT:
            self._error("rxrange", f"CHANNEL NOT BETWEEN 0 AND {NON_AUX_CHANNEL_COUNT - 1}")
            return
        if len(tokens) < 3:
            self._error("rxrange", "INVALID ARGUMENT COUNT")
            return
        low, high = self._atoi(tokens[1]), self._atoi(tokens[2])
        if not (PWM_PULSE_MIN <= low <= PWM_PULSE_MAX and PWM_PULSE_MIN <= high <= PWM_PULSE_MAX):
            self._error("rxrange", f"range min/max NOT BETWEEN {PWM_PULSE_MIN} AND {PWM_PULSE_MAX}")
            return
        f.rx_ranges[index] = {"min": low, "max": high}
        self._w(f"rxrange {index} {low} {high}")

    def _rxfail_channels_all(self) -> list:
        """Каналы «Этапа 1» длиной MAX_SUPPORTED_RC_CHANNEL_COUNT (18), как их читает CLI.

        Прошивка печатает и правит все 18 каналов (MAX_SUPPORTED_RC_CHANNEL_COUNT), а вкладка
        Failsafe работает с 16-ю (RC_CHANNEL_COUNT) — поэтому недостающие добираем заводскими.
        """
        return self.fc._rxfail_channels_all()

    def cmd_rxfail(self, args=""):
        """`rxfail` — cliRxFailsafe(): без аргументов печатает все каналы, с аргументами правит.

        Формат строки: `rxfail <канал> <a|h|s>` и `rxfail <канал> s <мкс>` (значение только для
        режима SET). Первые четыре канала — тип FLIGHT (AUTO/HOLD/SET), остальные AUX: у них
        AUTO недоступен (rxFailsafeModesTable[AUX][0] = INVALID). Значение переводится в «шаг»
        по 25 мкс от PWM_PULSE_MIN (constrain по 750…2250) — то есть округляется до сетки 25 мкс.
        """
        from .fc import (MAX_RXFAIL_RANGE_STEP, MAX_SUPPORTED_RC_CHANNEL_COUNT, NON_AUX_CHANNEL_COUNT,
                         PWM_PULSE_MAX, PWM_PULSE_MIN, RX_FAILSAFE_MODE_SET)
        text = args.strip()
        if not text:
            for channel in range(MAX_SUPPORTED_RC_CHANNEL_COUNT):
                self.cmd_rxfail(str(channel))
            return
        tokens = text.split()
        channel = self._atoi(tokens[0])
        if not 0 <= channel < MAX_SUPPORTED_RC_CHANNEL_COUNT:
            self._error("rxfail", f"CHANNEL NOT BETWEEN 0 AND {MAX_SUPPORTED_RC_CHANNEL_COUNT - 1}")
            return
        channels = self._rxfail_channels_all()
        entry = channels[channel]
        rest = tokens[1:]
        if rest:
            char = rest[0][:1]
            position = self.RXFAIL_MODE_CHARACTERS.find(char)
            if position < 0:
                mode = -1
            elif channel < NON_AUX_CHANNEL_COUNT:
                mode = position                       # FLIGHT: a → AUTO, h → HOLD, s → SET
            else:
                mode = position if position > 0 else -1   # AUX: AUTO = INVALID
            if mode < 0:
                self._error("rxfail", "PARSING FAILED")
                return
            require_value = mode == RX_FAILSAFE_MODE_SET
            if len(rest) > 1:
                if not require_value:
                    self._error("rxfail", "PARSING FAILED")
                    return
                # CHANNEL_VALUE_TO_RXFAIL_STEP(): constrain(value, PWM_PULSE_MIN, PWM_PULSE_MAX),
                # затем /25 — вне 750…2250 значение прижимается к границе (не ошибка). Прошивка
                # после этого ещё проверяет step > MAX_RXFAIL_RANGE_STEP, но после constrain
                # эта ветка недостижима — повторяем как есть.
                raw = self._atoi(rest[1])
                value = min(max(raw, PWM_PULSE_MIN), PWM_PULSE_MAX)
                step = (value - PWM_PULSE_MIN) // 25
                if step > MAX_RXFAIL_RANGE_STEP:
                    self._error("rxfail", f"value out of range: {step}")
                    return
                entry = {"mode": mode, "value": PWM_PULSE_MIN + 25 * step}
            elif require_value:
                self._error("rxfail", "INVALID ARGUMENT COUNT")
                return
            else:
                entry = {"mode": mode, "value": entry["value"]}
            channels[channel] = entry
            self.fc.rxfail = channels
        char = self.RXFAIL_MODE_CHARACTERS[entry["mode"]]
        if entry["mode"] == RX_FAILSAFE_MODE_SET:
            self._w(f"rxfail {channel} {char} {entry['value']}")
        else:
            self._w(f"rxfail {channel} {char}")

    def cmd_adjrange(self, args=""):
        """`adjrange` — cliAdjustmentRange(): 30 слотов «Корректирования в полёте».

        Формат вывода — `adjrange %u 0 %u %u %u %u %u %u %u` (индекс, ноль «was slot», канал
        AUX, начало, конец, функция, канал-переключатель, центр, масштаб) в микросекундах.
        Диапазон задаётся в микросекундах и переводится в шаги по 25 мкс от 900 мкс
        (CHANNEL_VALUE_TO_STEP), как processChannelRangeArgs() в cli.c. Если разобрано не
        ровно 6 обязательных значений — слот обнуляется целиком (memset) и печатается ошибка.
        """
        from .fc import ADJUSTMENT_FUNCTION_COUNT, ADJUSTMENT_RANGE_COUNT, MAX_AUX_CHANNEL_COUNT, MODE_RANGE_STEP
        f = self.fc
        min_step, max_step = 0, (2100 - 900) // MODE_RANGE_STEP     # MIN/MAX_MODE_RANGE_STEP

        def line(index, slot) -> str:
            return (f"adjrange {index} 0 {slot['aux']} {900 + slot['start'] * MODE_RANGE_STEP} "
                    f"{900 + slot['end'] * MODE_RANGE_STEP} {slot['function']} {slot['switch']} "
                    f"{slot.get('center', 0)} {slot.get('scale', 0)}")

        text = args.strip()
        if not text:
            for index, slot in enumerate(f.adjustment_ranges):
                self._w(line(index, slot))
            return
        tokens = text.split()
        index = self._atoi(tokens[0])
        if not 0 <= index < ADJUSTMENT_RANGE_COUNT:
            self._error("adjrange", f"INDEX NOT BETWEEN 0 AND {ADJUSTMENT_RANGE_COUNT - 1}")
            return
        slot = dict(f.adjustment_ranges[index])
        values = tokens[1:]
        valid = 0
        if values:                                  # «was slot»: прошивка значение не разбирает
            valid += 1
        if len(values) > 1:
            aux = self._atoi(values[1])
            if 0 <= aux < MAX_AUX_CHANNEL_COUNT:
                slot["aux"] = aux
                valid += 1
        for field, token in zip(("start", "end"), values[2:4]):
            step = int((self._atoi(token) - 900) / MODE_RANGE_STEP)
            if min_step <= step <= max_step:
                slot[field] = step
                valid += 1
        if len(values) > 4:
            function = self._atoi(values[4])
            if 0 <= function < ADJUSTMENT_FUNCTION_COUNT:
                slot["function"] = function
                valid += 1
        if len(values) > 5:
            switch = self._atoi(values[5])
            if 0 <= switch < MAX_AUX_CHANNEL_COUNT:
                slot["switch"] = switch
                valid += 1
        if valid != 6:
            f.adjustment_ranges[index] = {"aux": 0, "start": 0, "end": 0, "function": 0,
                                          "switch": 0, "center": 0, "scale": 0}
            self._error("adjrange", "INVALID ARGUMENT COUNT")
            return
        # Необязательные значения — как в прошивке: без проверок, просто atoi()
        slot["center"] = self._atoi(values[6]) if len(values) > 6 else 0
        slot["scale"] = self._atoi(values[7]) if len(values) > 7 else 0
        f.adjustment_ranges[index] = slot
        self._w(line(index, slot))

    # ------------------------------------------------------------------ плата и подпись
    def cmd_board_name(self, args=""):
        """`board_name` [имя] — cliBoardName(): «# Set board_name.» при правке и строка значения.

        Переименовать уже заданную плату прошивка не даёт: если имя задано и не совпадает
        (даже префиксом), ответ — «###ERROR IN board_name: BOARD_NAME CANNOT BE CHANGED. …».
        """
        f = self.fc
        text = args.strip()
        if text and f.board_name and text != f.board_name[:len(text)]:
            self._error("board_name", f"BOARD_NAME CANNOT BE CHANGED. CURRENT VALUE: '{f.board_name}'")
            return
        if text:
            f.board_name = text
            self._hash("Set board_name.")
        self._w(f"board_name {f.board_name}")

    def cmd_manufacturer_id(self, args=""):
        """`manufacturer_id` [id] — cliManufacturerId(): то же для идентификатора производителя."""
        f = self.fc
        text = args.strip()
        if text and f.manufacturer_id and text != f.manufacturer_id[:len(text)]:
            self._error("manufacturer_id",
                        f"MANUFACTURER_ID CANNOT BE CHANGED. CURRENT VALUE: '{f.manufacturer_id}'")
            return
        if text:
            f.manufacturer_id = text
            self._hash("Set manufacturer_id.")
        self._w(f"manufacturer_id {f.manufacturer_id}")

    def cmd_signature(self, args=""):
        """`signature` [hex] — cliSignature(): подпись платы как hex-строка (2 × SIGNATURE_LENGTH).

        Прошивка принимает ровно 64 hex-символа; при другой длине — «INVALID LENGTH: N
        (EXPECTED: 64)», при не-hex-символе — «INVALID CHARACTER FOUND: <символ>».
        """
        f = self.fc
        text = args.strip()
        if text:
            expected = 2 * len(f.signature)
            if len(text) != expected:
                self._error("signature", f"INVALID LENGTH: {len(text)} (EXPECTED: {expected})")
                return
            raw = bytearray()
            for i in range(len(f.signature)):
                pair = text[2 * i:2 * i + 2]
                bad = next((c for c in pair if c not in "0123456789abcdefABCDEF"), None)
                if bad is not None:
                    self._error("signature", f"INVALID CHARACTER FOUND: {bad}")
                    return
                raw.append(int(pair, 16))
            if any(f.signature) and bytes(raw) != bytes(f.signature):
                self._error("signature",
                            f"SIGNATURE CANNOT BE CHANGED. CURRENT VALUE: '{bytes(f.signature).hex()}'")
                return
            f.signature = bytes(raw)
            self._hash("Set signature.")
        self._w(f"signature {bytes(f.signature).hex()}")

    # ------------------------------------------------------------------ накопители
    def cmd_sd_info(self, args=""):
        """`sd_info` — cliSdInfo(): карта SD у эмулятора не настроена (SDCARD_MODE_NONE)."""
        self._w("SD card: Not configured")

    def cmd_msc(self, args=""):
        """`msc` — cliMsc(): «# Restarting in mass storage mode» и переход в режим MSC."""
        self._hash("Restarting in mass storage mode")
        self._w()
        self._w("Rebooting")
        self.fc.enter_msc()

    def cmd_flash_erase(self, args=""):
        """`flash_erase` — cliFlashErase(): стирает «флеш» и печатает «Erasing…» и «Done.»."""
        self._w("Erasing, please wait ... ")
        self.fc.blackbox.erase()
        self.fc.flash_overlay.clear()
        self._w()
        self._w("Done.")

    def cmd_flash_scan(self, args=""):
        """`flash_scan` — cliFlashVerify(): «Verifying» и «Success» (эмулятор всегда проверяется)."""
        self._w("Verifying")
        self._w("Success")

    def cmd_flash_read(self, args=""):
        """`flash_read` <адрес> <длина> — cliFlashRead(): строка и СЫРЫЕ байты «флеша».

        Прошивка пишет байты как есть (cliWrite), без hex-дампа: так же отдаёт и эмулятор —
        содержимое накопителя чёрного ящика плюс то, что записал `flash_write`.
        """
        tokens = args.split()
        if len(tokens) < 2:
            self._error("flash_read", "INVALID ARGUMENT COUNT")
            return
        address, length = self._atoi(tokens[0]), self._atoi(tokens[1])
        self._w(f"Reading {length} bytes at {address}:")
        data = self.fc.flash_read_bytes(address, length)
        if data:
            self._w_raw(data.decode("latin-1"))
        self._w()

    def cmd_flash_write(self, args=""):
        """`flash_write` <адрес> <текст> — cliFlashWrite(): «Wrote N bytes at A.»

        Текст берётся вместе с ведущим пробелом (в прошивке это strchr(cmdline, ' ')),
        поэтому счётчик на единицу больше длины сообщения — как в оригинале. Запись живёт
        в памяти эмулятора: файл пролётов не портится, а flash_erase наложение снимает.
        """
        text = args.strip()
        if " " not in text:
            self._error("flash_write", "INVALID ARGUMENT COUNT")
            return
        address = self._atoi(text)
        message = text[text.index(" "):]
        self.fc.flash_write_bytes(address, message.encode("latin-1", "replace"))
        self._w(f"Wrote {len(message)} bytes at {address}.")

    # ------------------------------------------------------------------ привязка и передатчик
    def cmd_bind_rx(self, args=""):
        """`bind_rx` — cliRxBind(): «Binding...» (приёмник эмулятора виртуальный, привязка мгновенна)."""
        self._w("Binding...")
        self.fc._log("[CLI] bind_rx: команда привязки принята")

    def cmd_play_sound(self, args=""):
        """`play_sound` [index] — cliPlaySound(): «Playing sound %d: %s» по таблице пищалки.

        Индексы — по таблице сигналов эмулятора (BEEPER_NAMES; тот же список показывает
        вкладка «Конфигурация» в Configurator). В таблице прошивки есть ещё ARMING_GPS_NO_FIX,
        которого в списке Configurator'а нет, поэтому у прошивки номера после ARMING_GPS_FIX
        на единицу больше; имена при этом совпадают. Без аргумента играется следующий сигнал
        по кругу, как lastSoundIdx + 1 в cli.c.
        """
        names = self.fc.beeper_names
        text = args.strip()
        if not text:
            self._last_sound = (self._last_sound + 1) % len(names)
            index = self._last_sound
        else:
            index = self._atoi(text)
            if not 0 <= index < len(names):
                self._w(f"No sound for index {index}")
                return
            self._last_sound = index
        self._w(f"Playing sound {index}: {names[index]}")

    # ------------------------------------------------------------------ светодиодная лента
    # Коды строки ленты из ledstrip.c: направления, базовые функции и наложения
    LED_DIRECTION_CODES = "NESWUD"
    LED_FUNCTION_CODES = "CFALSGR"
    LED_OVERLAY_CODES = "TYOBVIW"

    def _led_config_canonical(self, config: str):
        """parseLedStripConfig() из ledstrip.c: «x,y:направления:функции+наложения:цвет».

        Прошивка читает пять кусков (разделители «, : : : конец»), собирает биты направлений,
        базовой функции (из куска функций остаётся последняя подходящая буква) и наложений,
        а печатает уже результат generateLedConfig() — буквы в каноническом порядке.
        Возвращает None, если строка не разобрана (тогда CLI отвечает PARSING FAILED).
        """
        chunks, rest = [], config
        for separator in (",", ":", ":", ":"):
            head, found, rest = rest.partition(separator)
            if not found:
                return None
            chunks.append(head)
        chunks.append(rest)
        x, y = self._atoi(chunks[0]), self._atoi(chunks[1])
        directions = "".join(code for code in self.LED_DIRECTION_CODES if code in chunks[2])
        function = 0
        for char in chunks[3]:
            if char in self.LED_FUNCTION_CODES:
                function = self.LED_FUNCTION_CODES.index(char)
        overlays = "".join(code for code in self.LED_OVERLAY_CODES if code in chunks[3])
        color = self._atoi(chunks[4])
        return (f"{x & 0x0F},{y & 0x0F}:{directions}:"
                f"{self.LED_FUNCTION_CODES[function]}{overlays}:{color & 0x0F}")

    def cmd_led(self, args=""):
        """`led` [index <строка>] — cliLed(): строка на каждую из 32 позиций ленты.

        Вывод — через generateLedConfig(), поэтому у свежего борта это 32 строки
        «led i 0,0::C:0» (x=0, y=0, без направлений, функция COLOR, цвет 0).
        """
        from .fc import LED_STRIP_MAX_LENGTH
        f = self.fc
        text = args.strip()
        if not text:
            for index, config in enumerate(f.led_configs):
                self._w(f"led {index} {config}")
            return
        tokens = text.split(None, 1)
        index = self._atoi(tokens[0])
        if not 0 <= index < LED_STRIP_MAX_LENGTH:
            self._error("led", f"INDEX NOT BETWEEN 0 AND {LED_STRIP_MAX_LENGTH - 1}")
            return
        config = self._led_config_canonical(tokens[1].strip()) if len(tokens) > 1 else None
        if config is None:                 # пустая строка у прошивки тоже PARSING FAILED
            self._error("led", "PARSING FAILED")
            return
        f.led_configs[index] = config
        self._w(f"led {index} {config}")

    def cmd_color(self, args=""):
        """`color` [index h,s,v] — cliColor(): таблица цветов ленты (16 записей).

        Формат вывода — `color %u %d,%u,%u`; у ненастроенной ленты таблица нулевая.
        Правка принимает HSV: тон 0…359, насыщенность и яркость 0…255 (parseColor()).
        """
        from .fc import LED_CONFIGURABLE_COLOR_COUNT
        f = self.fc
        text = args.strip()
        if not text:
            for index, (hue, saturation, value) in enumerate(f.led_colors):
                self._w(f"color {index} {hue},{saturation},{value}")
            return
        tokens = text.split()
        index = self._atoi(tokens[0])
        if not 0 <= index < LED_CONFIGURABLE_COLOR_COUNT:
            self._error("color", f"INDEX NOT BETWEEN 0 AND {LED_CONFIGURABLE_COLOR_COUNT - 1}")
            return
        if len(tokens) < 2:
            self._error("color", "PARSING FAILED")
            return
        # parseColor(): три компоненты через запятую, каждая через atoi(); проверяется только
        # верхняя граница (HSV_HUE_MAX 359, HSV_SATURATION_MAX/VALUE_MAX 255), иначе PARSING FAILED
        parts = tokens[1].split(",")
        if len(parts) != 3:
            self._error("color", "PARSING FAILED")
            return
        hue, saturation, value = (self._atoi(part) for part in parts)
        if hue > 359 or saturation > 255 or value > 255:
            self._error("color", "PARSING FAILED")
            return
        f.led_colors[index] = [hue, saturation, value]
        self._w(f"color {index} {hue},{saturation},{value}")

    def cmd_mode_color(self, args=""):
        """`mode_color` [режим функция цвет] — cliModeColor(): цвета режимов и спец-цвета.

        Строки идут по таблице прошивки: режимы 0…5 (LED_MODE_COUNT) × направления 0…5
        (LED_DIRECTION_COUNT: N, E, S, W, U, D), затем спец-цвета (строка LED_SPECIAL = 6)
        и AUX-канал ленты (LED_AUX_CHANNEL = 7, у него «цвет» — это номер канала AUX).
        """
        from .fc import (LED_AUX_CHANNEL, LED_CONFIGURABLE_COLOR_COUNT, LED_DIRECTION_COUNT,
                         LED_MODE_COUNT, LED_SPECIAL, LED_SPECIAL_COLOR_COUNT)
        f = self.fc
        special, aux = LED_SPECIAL, LED_AUX_CHANNEL
        text = args.strip()
        if not text:
            for mode in range(LED_MODE_COUNT):
                for direction in range(LED_DIRECTION_COUNT):
                    self._w(f"mode_color {mode} {direction} {f.mode_colors['mode'][mode][direction]}")
            for index in range(LED_SPECIAL_COLOR_COUNT):
                self._w(f"mode_color {special} {index} {f.mode_colors['special'][index]}")
            self._w(f"mode_color {aux} 0 {f.mode_colors['aux_channel']}")
            return
        tokens = text.split()
        if len(tokens) != 3:
            self._error("mode_color", "INVALID ARGUMENT COUNT")
            return
        mode, function, color = (self._atoi(token) for token in tokens)
        if not 0 <= color < LED_CONFIGURABLE_COLOR_COUNT:          # setModeColor(): сначала цвет
            self._error("mode_color", "PARSING FAILED")
            return
        if mode == special:                                        # LED_SPECIAL: цвет спец-строки
            if not 0 <= function < LED_SPECIAL_COLOR_COUNT:
                self._error("mode_color", "PARSING FAILED")
                return
            f.mode_colors["special"][function] = color
        elif mode == aux:                                          # LED_AUX_CHANNEL: значение = канал
            if function != 0:
                self._error("mode_color", "PARSING FAILED")
                return
            f.mode_colors["aux_channel"] = color
        elif 0 <= mode < LED_MODE_COUNT and 0 <= function < LED_DIRECTION_COUNT:
            f.mode_colors["mode"][mode][function] = color
        else:
            self._error("mode_color", "PARSING FAILED")
            return
        self._w(f"mode_color {mode} {function} {color}")

    # ------------------------------------------------------------------ моторы и микшер
    def _parse_output_index(self, cmd: str, token: str, allow_all: bool = True):
        """parseOutputIndex() из cli.c: «Using output N.» / «Using all outputs.» или ошибка."""
        index = self._atoi(token)
        count = self.fc._motor_count()
        if 0 <= index < count:
            self._w(f"Using output {index}.")
            return index
        if allow_all and index == self.ALL_MOTORS:
            self._w("Using all outputs.")
            return index
        self._error(cmd, f"INVALID OUTPUT NUMBER. RANGE: 0 - {count - 1}.")
        return None

    def cmd_motor(self, args=""):
        """`motor` <index> <значение> — cliMotor(): тест моторов (1000…2000, 255 — все).

        Эмулятор хранит внешние значения (1000…2000), как MSP_SET_MOTOR и вкладка «Моторы»,
        поэтому и печатает их же: прошивка печатает результат motorConvertFromExternal().
        """
        from .fc import MAX_SUPPORTED_MOTORS
        text = args.strip()
        if not text:
            self._error("motor", "INVALID ARGUMENT COUNT")
            return
        tokens = text.split()
        index = self._parse_output_index("motor", tokens[0])
        if index is None:
            return
        if len(tokens) != 2:
            self._error("motor", "INVALID ARGUMENT COUNT")
            return
        value = self._atoi(tokens[1])
        if not 1000 <= value <= 2000:
            self._error("motor", "VALUE NOT BETWEEN 1000 AND 2000")
            return
        f = self.fc
        if index == self.ALL_MOTORS:
            for i in range(f._motor_count()):
                f.motor_disarmed[i] = float(value)
            self._w(f"all motors: {value}")
        else:
            f.motor_disarmed[min(index, MAX_SUPPORTED_MOTORS - 1)] = float(value)
            self._w(f"motor {index}: {value}")
        f.writes_log.append(f"motor {index}: {value} (CLI)")

    def cmd_mmix(self, args=""):
        """`mmix` — cliMotorMix(): свой микшер моторов и `mmix reset`.

        Печать — `mmix %d %s %s %s %s` с тремя знаками после запятой (ftoa), цикл
        останавливается на первой записи с нулевым газом (printMotorMix). Готовых таблиц
        микшеров (mixer_init.c) в эмуляторе нет, поэтому `mmix load` только проверяет имя.
        """
        from .fc import MAX_SUPPORTED_MOTORS
        f = self.fc

        def lines() -> list:
            out = []
            for index, entry in enumerate(f.custom_motor_mix):
                values = " ".join(f"{float(entry[field]):.3f}" for field in ("throttle", "roll", "pitch", "yaw"))
                out.append(f"mmix {index} {values}")
            return out

        text = args.strip()
        if not text:
            for line in lines():
                self._w(line)
            return
        lower = text.lower()
        if lower.startswith("reset"):
            f.custom_motor_mix = []          # memset таблицы customMotorMixer
            return
        if lower.startswith("load"):
            name = text[4:].strip().upper()
            match = next((candidate for candidate in f.mixer_names if candidate.startswith(name)), None)
            if match is None or not name:
                self._error("mmix", f"INVALID NAME: {text[4:].strip()}")
                return
            self._hash("mmix load: this emulator has no built-in mixer tables")
            self._hash(f"mixer {match} is set by 'mixer {match}'; mmix values are set by hand")
            return
        tokens = text.split()
        index = self._atoi(tokens[0])
        if not 0 <= index < MAX_SUPPORTED_MOTORS:
            self._error("mmix", f"INDEX NOT BETWEEN 0 AND {MAX_SUPPORTED_MOTORS - 1}")
            return
        values = tokens[1:]
        if len(values) != 4:
            self._error("mmix", "INVALID ARGUMENT COUNT")
            return
        try:
            entry = {field: float(token) for field, token in
                     zip(("throttle", "roll", "pitch", "yaw"), values)}
        except ValueError:
            self._error("mmix", "INVALID ARGUMENT COUNT")
            return
        mix = [dict(item) for item in f.custom_motor_mix]
        while len(mix) <= index:
            mix.append({"throttle": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0})
        mix[index] = entry
        f.custom_motor_mix = [item for item in mix if item["throttle"] != 0.0]
        for line in lines():
            self._w(line)

    # ------------------------------------------------------------------ пищалка и DShot
    def _beeper_command(self, cmd: str, args: str, disabled: set, allowed: tuple):
        """processBeeperCommand() из cli.c: состояние и правка таблицы сигналов.

        Без аргументов — «Disabled: …» (или «  none»), `list` — доступные для этой команды,
        `<имя>`/`-<имя>` — включить/выключить с печатью «Enabled …»/«Disabled …». Имя ищется
        по началу (strncasecmp), первое подходящее из разрешённых; «ALL» — как последняя
        запись таблицы прошивки (для DShot-пищалки разрешены только RX_SET и RX_LOST).
        """
        names = list(self.fc.beeper_names)
        text = args.strip()
        if not text:
            off = [name for name in names if name in disabled]
            self._w("Disabled:" + ("  " + "  ".join(off) if off else "  none"))
            return
        needle = text.lower()
        if len(needle) <= 4 and "list".startswith(needle):
            self._w("Available:" + "".join(f" {name}" for name in allowed))
            return
        remove = text.startswith("-")
        token = (text[1:] if remove else text).strip()
        token = token.split()[0] if token else ""
        low = token.lower()
        match = next((name for name in names if low and name.lower().startswith(low)), None)
        if match is not None and match not in allowed:
            match = None                     # вне allowedFlags прошивка имя не принимает
        if match is None:
            if low and "all".startswith(low):
                if remove:
                    disabled.update(allowed)
                else:
                    disabled.clear()
                self._w(("Disabled" if remove else "Enabled") + " ALL")
                return
            self._error(cmd, f"INVALID NAME: {token}")
            return
        if remove:
            disabled.add(match)
        else:
            disabled.discard(match)
        self._w(("Disabled" if remove else "Enabled") + f" {match}")

    def cmd_beeper(self, args=""):
        """`beeper` — cliBeeper(): включение/выключение сигналов (формат processBeeperCommand)."""
        self._beeper_command("beeper", args, self.fc.beepers_disabled, tuple(self.fc.beeper_names))

    def cmd_beacon(self, args=""):
        """`beacon` — cliBeacon(): то же для DShot-пищалки (только RX_LOST и RX_SET, как в прошивке)."""
        self._beeper_command("beacon", args, self.fc.dshot_beacon_disabled, ("RX_LOST", "RX_SET"))

    def cmd_dshotprog(self, args=""):
        """`dshotprog` <index> <команда>+ — cliDshotProg(): команды ESC по DShot.

        Прошивка требует протокол DShot (моторы эмулятора на DSHOT600), сначала глушит моторы
        (motorDisable), затем на каждую команду печатает «Command Sent: %d»; команда ESC_INFO
        без датчика ESC отвечает «Not supported.». Принятые команды попадают в журнал
        dshot_commands_log — тот же, что у MSP-команд вкладки «Моторы».
        """
        from .fc import DSHOT_CMD_ESC_INFO, DSHOT_MIN_THROTTLE
        f = self.fc
        text = args.strip()
        if not text or f._setting("motor_pwm_protocol", 7) not in (5, 6, 7, 8):
            self._error("dshotprog", "PARSING FAILED")
            return
        tokens = text.split()
        index = self._parse_output_index("dshotprog", tokens[0])
        if index is None:
            return
        commands = tokens[1:]
        if not commands:
            self._error("dshotprog", "PARSING FAILED")
            return
        first = True
        for token in commands:
            command = self._atoi(token)
            if not 0 <= command < DSHOT_MIN_THROTTLE:
                self._error("dshotprog", f"INVALID COMMAND. RANGE: 1 - {DSHOT_MIN_THROTTLE - 1}.")
                continue
            if first:
                for i in range(f._motor_count()):   # motorDisable() перед первой командой
                    f.motor_disarmed[i] = 1000.0
                first = False
            if command == DSHOT_CMD_ESC_INFO:
                self._w("Not supported.")
            f.dshot_commands_log.append({"esc": index, "command": command, "source": "cli"})
            f.dshot_commands_log = f.dshot_commands_log[-10:]
            self._w(f"Command Sent: {command}")

    def cmd_dshot_telemetry_info(self, args=""):
        """`dshot_telemetry_info` — cliDshotTelemetryInfo(): состояние DShot-телеметрии.

        Бидирекшн DShot в эмуляторе включён (dshot_bidir = ON), поэтому печатается полная
        таблица, как в прошивке: счётчики кадров, шапка и строка на каждый мотор. Обороты и
        температура — из модели моторов (eRPM = обороты · полюса / 2), числа в колонках,
        которых у эмулятора нет (события ESC и отладочные), нулевые. Два дампа входного
        GCR-буфера (MAX_GCR_EDGES) не печатаются: регистры приёмного буфера не моделируются.
        """
        f = self.fc
        if not f._setting("dshot_bidir", 0):
            return                      # без телеметрии прошивка молчит (cliDshotTelemetryInfo)
        poles = max(1, f._setting("motor_poles", 14))
        self._w("Dshot reads: 0")
        self._w("Dshot invalid pkts: 0")
        self._w("Dshot directionChange cycles: 0, micros: 0")
        self._w()
        # Заголовки — дословно cli.c (ветка USE_DSHOT_TELEMETRY_STATS: у неё есть колонка Invalid)
        self._w("Motor    Type   eRPM    RPM     Hz Invalid   TEMP    VCC   CURR  ST/EV   DBG1   DBG2   DBG3")
        self._w("=====  ====== ====== ====== ====== ======= ====== ====== ====== ====== ====== ====== ======")
        for i in range(f._motor_count()):
            rpm = int(round(f.motor_rpm[i]))
            erpm = int(round(f.motor_rpm[i] * poles / 2))      # печатается сам eRPM (erpm * 100 LSB)
            if f.motor_telemetry_active():
                percent = f.motor_invalid_percent(i)
                invalid = f" {percent // 100:3d}.{percent % 100:02d}%"
            else:
                invalid = " NO DATA"
            volts = int(round(f.motor_voltage_v * 4))          # ESC шлёт напряжение в 0.25 В
            self._w(f"{i + 1:5d}   RTVC- {erpm:6d} {rpm:6d} {rpm // 60:6d}{invalid} "
                    f"{int(round(f.motor_temp_c[i])):6d} {volts // 4:3d}.{25 * (volts % 4):02d} "
                    f"{int(round(f.motor_current_a[i])):6d} {0:6d} {0:6d} {0:6d} {0:6d}")
        self._w()

    def cmd_rc_smoothing_info(self, args=""):
        """`rc_smoothing_info` — cliRcSmoothing(): тип сглаживания и активные срезы.

        Тип сглаживания и признак (manual)/(auto) — из настроек, как их читает прошивка;
        частота приёма — из модели приёмника эмулятора (без связи — «NO SIGNAL», как у
        прошивки при нулевом интервале кадров). Сами активные срезы считает задача фильтра
        (fc/rc.c), которой в эмуляторе нет: активный срез в авторежиме печатается равным
        настройке (0 — «не рассчитано») с той же пометкой (auto), что и в прошивке.
        """
        f = self.fc
        if not f._setting("rc_smoothing", 0):
            self._hash("RC Smoothing Type: OFF")
            return
        self._hash("RC Smoothing Type: FILTER")
        if f.rx_link_connected:
            self._hash(f"Detected Rx frequency: {f.RX_RATE_HZ}Hz")
        else:
            self._hash("Detected Rx frequency: NO SIGNAL")
        for title, name in (("setpoint", "rc_smoothing_setpoint_cutoff"),
                            ("FF", "rc_smoothing_feedforward_cutoff"),
                            ("throttle", "rc_smoothing_throttle_cutoff")):
            value = f._setting(name, 0)
            mode = "manual" if value else "auto"
            self._hash(f"Active {title} cutoff: {value}hz ({mode})")

    def cmd_simplified_tuning(self, args=""):
        """`simplified_tuning` apply|disable — cliSimplifiedTuning().

        `apply` пересчитывает PID и фильтры по ползункам во ВСЕХ профилях (в эмуляторе профили
        переключаются через тот же select_profile, что и у MSP), `disable` снимает ползунки и
        выключает упрощённые фильтры гироскопа/D-term — как disableSimplifiedTuning().
        """
        from .fc import CONTROL_RATE_PROFILE_COUNT, PID_PROFILE_COUNT, SIMPLIFIED_TUNING_DEFAULT, SIMPLIFIED_TUNING_OFF
        f = self.fc
        text = args.strip().lower()
        if text == "apply":
            active = f.current_profile
            for profile in range(PID_PROFILE_COUNT):
                f.select_profile(profile)
                f._apply_simplified_tuning()          # applySimplifiedTuning() для профиля
                f._save_active_pid_profile()
            f.select_profile(active)
            self._w("Applied simplified tuning.")
        elif text == "disable":
            active = f.current_profile
            for profile in range(PID_PROFILE_COUNT):
                f.select_profile(profile)
                f.settings["simplified_pids_mode"] = SIMPLIFIED_TUNING_OFF
                f.settings["simplified_dterm_filter"] = 0
                f.settings["simplified_gyro_filter"] = 0
                for name in ("simplified_master_multiplier", "simplified_pitch_d_gain", "simplified_i_gain",
                             "simplified_d_gain", "simplified_pi_gain", "simplified_dmax_gain",
                             "simplified_feedforward_gain", "simplified_pitch_pi_gain",
                             "simplified_dterm_filter_multiplier", "simplified_gyro_filter_multiplier"):
                    if name in f.settings:
                        f.settings[name] = SIMPLIFIED_TUNING_DEFAULT
                f._save_active_pid_profile()
            f.select_profile(active)
            self._w("Disabled simplified tuning.")
        else:
            self._error("simplified_tuning", "PARSING FAILED")

    # ---------------------------------------- команды, которым нужно железо (в эмуляторе его нет)
    def cmd_escprog(self, args=""):
        """`escprog` <режим> <index> — cliEscPassthrough(): эмулятор не умеет пробрасывать ESC.

        Режимы и индекс разбираются как в прошивке (sk/bl/ki/cc, «Using output N.»), а вместо
        проброса — та же ошибка, что прошивка отдаёт, когда порт ESC не открылся.
        """
        text = args.strip()
        if not text:
            self._error("escprog", "INVALID ARGUMENT COUNT")
            return
        tokens = text.split()
        if tokens[0].lower() not in ("sk", "bl", "ki", "cc"):
            self._error("escprog", "PARSING FAILED")
            return
        if len(tokens) != 2:
            self._error("escprog", "INVALID ARGUMENT COUNT")
            return
        if self._parse_output_index("escprog", tokens[1], allow_all=tokens[0].lower() != "ki") is None:
            return
        self._error("escprog", "Error starting ESC connection")

    def cmd_gyroregisters(self, args=""):
        """`gyroregisters` — cliDumpGyroRegisters(): дампа регистров гироскопа у эмулятора нет.

        Прошивка печатает регистры драйвера гироскопа (# Gyro 1 и содержимое конфигурации),
        но сами регистры живут в драйвере железа — в эмуляторе их взять негде, поэтому
        команда отвечает ошибкой эмулятора (а не «UNKNOWN COMMAND»).
        """
        self._error("gyroregisters", "NOT SUPPORTED IN THIS EMULATOR")

    def cmd_serialpassthrough(self, args=""):
        """`serialpassthrough` — cliSerialPassthrough(): проброс портов в эмуляторе невозможен.

        Прошивка уводит CLI в прозрачный проброс между двумя UART (`Forwarding, power cycle
        to exit.`), пока сырьевые порты эмулятора — это одна виртуальная пара MSP/CLI.
        """
        self._error("serialpassthrough", "NOT SUPPORTED IN THIS EMULATOR")

    COMMANDS = {}
    HELP = {
        # Таблица команд — дословно из cli.c 4.5.4 (cmdTable[]): имя, описание, аргументы.
        # Ветки #ifdef выбраны по сборке эмулятора: bl → [rom], dma → show, msc → без
        # часового пояса, serialpassthrough → без PinIO, vtx → полный (не MINIMAL_CLI).
        "adjrange": ("configure adjustment ranges", "<index> <unused> <range channel> <start> <end> <function> <select channel> [<center> <scale>]"),
        "aux": ("configure modes", "<index> <mode> <aux> <start> <end> <logic>"),
        "batch": ("start or end a batch of commands", "start | end"),
        "beacon": ("enable/disable Dshot beacon for a condition", "list\r\n\t<->[name]"),
        "beeper": ("enable/disable beeper for a condition", "list\r\n\t<->[name]"),
        "bind_rx": ("initiate binding for RX SPI, SRXL2 or CRSF", ""),
        "bl": ("reboot into bootloader", "[rom]"),
        "board_name": ("get / set the name of the board model", "[board name]"),
        "color": ("configure colors", ""),
        "defaults": ("reset to defaults and reboot", "{nosave}"),
        "diff": ("list configuration changes from default", "[master|profile|rates|hardware|all] {defaults|bare}"),
        "dma": ("show DMA assignments", "show"),
        "dshot_telemetry_info": ("display dshot telemetry info and stats", ""),
        "dshotprog": ("program DShot ESC(s)", "<index> <command>+"),
        "dump": ("dump configuration", "[master|profile|rates|hardware|all] {defaults|bare}"),
        "escprog": ("passthrough esc to serial", "<mode [sk/bl/ki/cc]> <index>"),
        "exit": ("exit command line interface and reboot (default)", "[noreboot]"),
        "feature": ("configure features", "list\r\n\t<->[name]"),
        "flash_erase": ("erase flash chip", ""),
        "flash_info": ("show flash chip info", ""),
        "flash_read": ("", "<length> <address>"),
        "flash_scan": ("scan flash device for errors", ""),
        "flash_write": ("", "<address> <message>"),
        "get": ("get variable value", "[name]"),
        "gpspassthrough": ("passthrough gps to serial", ""),
        "gyroregisters": ("dump gyro config registers contents", ""),
        "help": ("display command help", "[search string]"),
        "led": ("configure leds", ""),
        "manufacturer_id": ("get / set the id of the board manufacturer", "[manufacturer id]"),
        "map": ("configure rc channel order", "[<map>]"),
        "mcu_id": ("id of the microcontroller", ""),
        "mixer": ("configure mixer", "list\r\n\t<name>"),
        "mmix": ("custom motor mixer", ""),
        "mode_color": ("configure mode and special colors", ""),
        "motor": ("get/set motor", "<index> [<value>]"),
        "msc": ("switch into msc mode", ""),
        "play_sound": ("", "[<index>]"),
        # battery_profile — из cmdTable МАСТЕРА BF (после 4.5.4): в 4.5.4 профилей батареи
        # нет вообще. Порядок записей и текст — по cli.c мастера:
        # CLI_COMMAND_DEF("battery_profile", "change battery profile", "[<index>]", cliBatteryProfile)
        "battery_profile": ("change battery profile", "[<index>]"),
        "profile": ("change profile", "[<index>]"),
        "rateprofile": ("change rate profile", "[<index>]"),
        "rc_smoothing_info": ("show rc_smoothing operational settings", ""),
        "resource": ("show/set resources", "<> | <resource name> <index> [<pin>|none] | show [all]"),
        "rxfail": ("show/set rx failsafe settings", ""),
        "rxrange": ("configure rx channel ranges", ""),
        "save": ("save and reboot (default)", "[noreboot]"),
        "sd_info": ("sdcard info", ""),
        "serial": ("configure serial ports", ""),
        "serialpassthrough": ("passthrough serial data from port 1 to VCP / port 2", "<id1> [<baud1>] [<mode1>] [none|reset] [<id2>] [<baud2>] [<mode2>]"),
        "servo": ("configure servos", ""),
        "set": ("change setting", "[<name>=<value>]"),
        "signature": ("get / set the board type signature", "[signature]"),
        "simplified_tuning": ("applies or disables simplified tuning", "apply | disable"),
        "smix": ("servo mixer", "<rule> <servo> <source> <rate> <speed> <min> <max> <box>\r\n\treset\r\n\tload <mixer>\r\n\treverse <servo> <source> r|n"),
        "status": ("show status", ""),
        "tasks": ("show task stats", ""),
        "timer": ("show/set timers", "<> | <pin> list | <pin> [af<alternate function>|none|<option(deprecated)>] | list | show"),
        "version": ("show version", ""),
        "vtx": ("vtx channels on switch", "<index> <aux_channel> <vtx_band> <vtx_channel> <vtx_power> <start_range> <end_range>"),
        "vtx_info": ("vtx power config dump", ""),
        "vtxtable": ("vtx frequency table", "<band> <bandname> <bandletter> [FACTORY|CUSTOM] <freq> ... <freq>\r\n"),
        # Команды самого эмулятора (в прошивке их нет): держим их в той же таблице.
        "blackbox": ("emulator: blackbox flash info | refill | erase", "info | refill | erase"),
        "sim_flight": ("emulator: virtual flight — live current, mAh drawn, voltage sag",
                       "on | off | reset | status | scale <1..600>"),
        "sim_failsafe": ("emulator: failsafe demo — rx loss, stage 1, rollback point, stage 2",
                         "on | off | loss | restore | reset | arm | disarm | box on|off | status"),
        "board": ("emulator: show/set board name", "[name]"),
        "osd_layout": ("emulator: show/set OSD layout", "[name=value]"),
        "uid": ("emulator: show unique id", ""),
    }


class CliTerminal:
    """Терминал «сырого» CLI-порта: два режима, как в прошивке.

    * интерактивный (вход по байту '#') — эхо символов, приглашение "\r\n# ";
    * неинтерактивный (вход по байту 0x02, так делает Configurator через
      MSP.send_cli_command) — вывод обрамляется STX ... ETX, приглашения нет.
    """

    def __init__(self, fc):
        self.fc = fc
        self.engine = fc._cli
        self.mode = None  # None | "interactive" | "noninteractive"
        self.line = ""

    def enter_interactive(self) -> bytes:
        self.mode = "interactive"
        self.line = ""
        self.fc.cli_active = True
        self.engine.echo = True
        return b"\r\nEntering CLI Mode, type 'exit' to reboot, or 'help'\r\n# "

    def enter_noninteractive(self) -> bytes:
        self.mode = "noninteractive"
        self.line = ""
        self.fc.cli_active = True
        self.engine.echo = False
        return bytes([STX])

    def exit(self) -> bytes:
        self.mode = None
        self.fc.cli_active = False
        self.engine.echo = False
        return b""

    def feed(self, chunk: bytes) -> bytes:
        """Возвращает байты для отправки клиенту."""
        if self.mode == "interactive":
            return self._feed_interactive(chunk)
        if self.mode == "noninteractive":
            return self._feed_noninteractive(chunk)
        return b""

    # ------------------------------------------------------------- режимы
    def _feed_interactive(self, chunk: bytes) -> bytes:
        out = bytearray()
        for ch in chunk:
            if ch in (0x0A, 0x0D):
                out += b"\r\n"
                if self.line.strip():
                    self.engine.out = []
                    text = self.fc.cli_execute(self.line)
                    out += text.encode("latin-1", "replace")
                self.line = ""
                out += b"# "
            elif ch == 0x03:  # Ctrl-C — выход из CLI
                out += b"\r\n"
                self.exit()
                return bytes(out)
            elif ch == 0x09:  # TAB — автодополнение
                out += self._autocomplete()
            elif ch in (0x08, 0x7F):
                if self.line:
                    self.line = self.line[:-1]
                    out += b"\x08 \x08"
            elif 32 <= ch <= 126:
                self.line += chr(ch)
                out += bytes([ch])  # эхо
        return bytes(out)

    def _feed_noninteractive(self, chunk: bytes) -> bytes:
        out = bytearray()
        for ch in chunk:
            if ch == ETX:  # Ctrl-C/ETX завершает поток управления
                self.exit()
                out += bytes([ETX])
                return bytes(out)
            if ch in (0x0A, 0x0D):
                if self.line.strip():
                    self.engine.out = []
                    text = self.fc.cli_execute(self.line)
                    out += text.encode("latin-1", "replace")
                self.line = ""
            else:
                self.line += chr(ch)
        return bytes(out)

    def _autocomplete(self) -> bytes:
        """TAB: дописать имя команды/настройки или показать варианты."""
        if not self.line:
            return b""
        prefix, _, arg = self.line.partition(" ")
        if arg and prefix.lower() in ("get", "set"):
            names = sorted(s["name"] for s in self.fc.settings_list if s["name"].startswith(arg.strip()))
        else:
            names = sorted(n for n in CliEngine.COMMANDS if n.startswith(self.line.strip().lower()))
        if not names:
            return b"\x07"  # звонок, как у прошивки
        if len(names) == 1:
            completion = names[0]
            if arg:
                added = completion[len(arg.strip()):]
                self.line = f"{prefix} {completion}"
            else:
                added = completion[len(self.line.strip()):]
                self.line = completion
            return added.encode()
        out = b"\r\n"
        out += ("  ".join(names)).encode()
        out += b"\r\n# " + self.line.encode()
        return out


def _bind_commands(cls):
    for name in list(cls.HELP):
        fn = getattr(cls, "cmd_" + name, None)
        if fn is not None:
            cls.COMMANDS[name] = fn
    return cls


_bind_commands(CliEngine)

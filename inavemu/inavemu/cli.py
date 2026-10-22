"""
CLI INAV: режим «#» поверх MSP-порта, эхо, приглашение «# », TAB-автодополнение
и команды cmdTable[] прошивки INAV 9.1.0 (src/main/fc/cli.c).

Поведение сверено с cliProcess/cliEnter/cliExit прошивки и табом CLI
конфигуратора: '#' входит в CLI, 'save'/'exit'/'defaults' печатают «Rebooting»
и рвут линк.
"""
from __future__ import annotations

import time

from .settings import MODE_LOOKUP, SettingsStore

# таблица baudRates[] прошивки (индекс → бит/с; 0 = AUTO)
BAUD_RATES = [0, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400,
              250000, 460800, 921600]

RC_CHANNEL_LETTERS = "AETR1234"

# cmdTable[] из src/main/fc/cli.c (name, description, args)
COMMANDS = [
    ('adjrange', 'configure adjustment ranges', ''),
    ('assert', '', ''),
    ('aux', 'configure modes', ''),
    ('batch', 'start or end a batch of commands', 'start | end'),
    ('beeper', 'turn on/off beeper', 'list\n\t<+|->[name]'),
    ('bind_msp_rx', 'initiate binding for MSP receivers (mLRS)', '<port>'),
    ('bind_rx', 'initiate binding for RX SPI or SRXL2', ''),
    ('bootlog', 'show boot log', ''),
    ('color', 'configure colors', ''),
    ('mode_color', 'configure mode and special colors', ''),
    ('cli_delay', 'CLI Delay', 'Delay in ms'),
    ('defaults', 'reset to defaults and reboot', ''),
    ('dfu', 'DFU mode on reboot', ''),
    ('diff', 'list configuration changes from default', '[master|battery_profile|control_profile|mixer_profile|rates|all] {showdefaults}'),
    ('dump', 'dump configuration', '[master|battery_profile|control_profile|mixer_profile|rates|all] {showdefaults}'),
    ('eleres_bind', '', ''),
    ('exit', '', ''),
    ('feature', 'configure features', 'list\n\t<+|->[name]'),
    ('blackbox', 'configure blackbox fields', 'list\n\t<+|->[name]'),
    ('flash_erase', 'erase flash chip', ''),
    ('flash_info', 'show flash chip info', ''),
    ('flash_read', '', '<length> <address>'),
    ('flash_write', '', '<address> <message>'),
    ('fwapproach', 'Fixed Wing Approach Settings', ''),
    ('get', 'get variable value', '[name]'),
    ('geozone', 'get or set geo zones', ''),
    ('gpspassthrough', 'passthrough gps to serial', ''),
    ('gpssats', 'show GPS satellites', ''),
    ('help', '', ''),
    ('led', 'configure leds', ''),
    ('ledpinpwm', 'start/stop PWM on LED pin, 0..100 duty ratio', '[<value>]\n'),
    ('map', 'configure rc channel order', '[<map>]'),
    ('memory', 'view memory usage', ''),
    ('mmix', 'custom motor mixer', ''),
    ('motor', 'get/set motor', '<index> [<value>]'),
    ('msc', 'switch into msc mode', ''),
    ('play_sound', '', '[<index>]\n'),
    ('control_profile', 'change control profile', '[<index>]'),
    ('mixer_profile', 'change mixer profile', '[<index>]'),
    ('battery_profile', 'change battery profile', '[<index>]'),
    ('resource', 'view currently used resources', ''),
    ('rxrange', 'configure rx channel ranges', ''),
    ('safehome', 'safe home list', ''),
    ('save', 'save and reboot', ''),
    ('serial', 'configure serial ports', ''),
    ('serialpassthrough', 'passthrough serial data to port', '<id> [baud] [mode] [options]: passthrough to serial'),
    ('servo', 'configure servos', ''),
    ('logic', 'configure logic conditions', '<rule> <enabled> <activatorId> <operation> <operand A type> <operand A value> <operand B type> <operand B value> <flags>\n\treset\n'),
    ('gvar', 'configure global variables', '<gvar> <default> <min> <max>\n\treset\n'),
    ('pid', 'configurable PID controllers', '<#> <enabled> <setpoint type> <setpoint value> <measurement type> <measurement value> <P gain> <I gain> <D gain> <FF gain>\n\treset\n'),
    ('osd_custom_elements', 'configurable OSD custom elements', '<#> <part0 type> <part0 value> <part1 type> <part1 value> <part2 type> <part2 value> <visibility type> <visibility value> <text>\n'),
    ('set', 'change setting', '[<name>=<value>]'),
    ('smix', 'servo mixer', '<rule> <servo> <source> <rate> <speed> <conditionId>\n\treset\n'),
    ('sd_info', 'sdcard info', ''),
    ('showdebug', 'Show debug fields.', ''),
    ('status', 'show status', ''),
    ('tasks', 'show task stats', ''),
    ('temp_sensor', 'change temp sensor settings', ''),
    ('version', 'show version', ''),
    ('wp', 'waypoint list', ''),
    ('osd_layout', 'get or set the layout of OSD items', '[<layout> [<item> [<col> <row> [<visible>]]]]'),
    ('timer_output_mode', 'get or set the outputmode for a given timer.', '[<timer> [<AUTO|MOTORS|SERVOS>]]'),
]


def ftoa(value) -> str:
    """Печать float как в прошивке: без лишних нулей."""
    s = f"{float(value):.6f}".rstrip("0").rstrip(".")
    return s if s not in ("", "-") else "0"


class CliSession:
    """Сессия CLI на одном TCP-соединении."""

    def __init__(self, fc):
        self.fc = fc
        self.store: SettingsStore = fc.store
        self.buffer = ""
        self.reboot_requested = False
        self.closed = False          # выход из CLI (exit)
        self.batch = False

    # ------------------------------------------------------------------
    # поток байтов
    # ------------------------------------------------------------------
    def enter(self) -> bytes:
        return b"\r\nEntering CLI Mode, type 'exit' to return, or 'help'\r\n# "

    def feed(self, data: bytes) -> bytes:
        out = bytearray()
        for c in data:
            self._char(c, out)
        return bytes(out)

    def _prompt(self, out: bytearray):
        out += b"\r\n# "

    def _char(self, c: int, out: bytearray):
        if c in (ord("\n"), ord("\r")):
            if not self.buffer:
                return
            out += b"\r\n"
            line = self.buffer
            self.buffer = ""
            self._execute(line, out)
            if not self.closed and not self.reboot_requested:
                self._prompt(out)
        elif c == 127:  # backspace
            if self.buffer:
                self.buffer = self.buffer[:-1]
                out += b"\010 \010"
        elif c in (ord("\t"), ord("?")):
            self._complete(out)
        elif c == 4 and not self.buffer:  # CTRL-D
            self._cmd_exit("", out)
        elif c == 12:  # CTRL-L
            out += b"\033[2J\033[1;1H"
            self._prompt(out)
        elif 32 <= c <= 126:
            if not self.buffer and c == ord(" "):
                return  # ведущие пробелы игнорируем
            self.buffer += chr(c)
            out.append(c)

    def _complete(self, out: bytearray):
        matches = [cmd for cmd in COMMANDS
                   if cmd[0].startswith(self.buffer.lower())]
        if not matches:
            return
        # общий префикс
        prefix = matches[0][0]
        for name, _, _ in matches[1:]:
            while not name.startswith(prefix):
                prefix = prefix[:-1]
        if len(matches) == 1:
            # однозначно — дополнить пробелом
            self.buffer = matches[0][0] + " "
            out += b"\r\033[K" + self.buffer.encode()
            return
        if prefix and prefix != self.buffer:
            self.buffer = prefix
            out += b"\r\033[K" + self.buffer.encode()
            return
        # список вариантов
        out += b"\r\033[K"
        for name, _, _ in matches:
            out += name.encode() + b"\t"
        self._prompt(out)
        out += self.buffer.encode()

    # ------------------------------------------------------------------
    # разбор и выполнение строки
    # ------------------------------------------------------------------
    def _execute(self, line: str, out: bytearray):
        if "#" in line:
            line = line[:line.index("#")]
        line = line.rstrip(" ")
        if not line:
            return
        for name, _desc, _args in COMMANDS:
            if line.lower().startswith(name) and (
                len(line) == len(name) or not line[len(name)].isalnum()
            ):
                args = line[len(name):].lstrip(" ")
                handler = getattr(self, "_cmd_" + name, None)
                if handler is None:
                    self._stub(name, args, out)
                else:
                    handler(args, out)
                return
        out += b"### ERROR: Unknown command, try 'help'"

    def _stub(self, name, args, out: bytearray):
        """Команда без сохраняемого состояния — тихий успех."""
        if name in ("flash_erase",):
            out += b"Erasing flash chip..."
        elif name == "dfu":
            out += b"\r\nRestarting in DFU mode\r\nRebooting"
            self.reboot_requested = True
        elif name == "msc":
            out += b"\r\nStorage not present or failed to initialize!"
        elif name == "gpspassthrough":
            out += b"### ERROR: GPS not available"
        elif name == "bind_rx":
            out += b"Binding..."
        elif name in ("eleres_bind",):
            out += b"### ERROR: Unknown command, try 'help'"

    # --- identity / info ---
    def _cmd_help(self, args, out):
        for name, desc, cargs in COMMANDS:
            out += name.encode()
            if desc:
                out += f" - {desc}".encode()
            if cargs:
                out += f"\r\n\t{cargs}".encode()
            out += b"\r\n"

    def _cmd_version(self, args, out):
        fc = self.fc
        out += (f"# INAV/{fc.target} {fc.fc_version} {fc.build_date} "
                f"{fc.build_time} ({fc.git_revision}) \r\n").encode()
        out += b"# GCC-13.2.1\r\n"

    def _cmd_status(self, args, out):
        fc = self.fc
        out += (f"INAV/{fc.target} {fc.fc_version} {fc.build_date} "
                f"{fc.build_time} ({fc.git_revision}) \r\n").encode()
        out += b"GCC-13.2.1\r\n"
        uptime = int(time.time() - fc.t0) + 35
        out += f"System Uptime: {uptime} seconds\r\n".encode()
        out += time.strftime("Current Time: %Y-%m-%d @ %H:%M:%S\r\n").encode()
        voltage = 1260
        out += (f"Voltage: {voltage // 100}.{voltage % 100:02d}V "
                f"({fc.battery_cell_count()}S battery - OK)\r\n").encode()
        out += b"CPU Clock=216MHz, GYRO=MPU6000, ACC=MPU6000, MAG=HMC5883, BARO=MS5611\r\n"
        out += b"Stack size: 2048, Stack address: 0x1000fff0\r\n"
        out += b"I2C Errors: 0, config size: 5268, max available: 16384\r\n"
        out += b"ADC channel usage:\r\n        BATTERY: ADC1 CH11 10 bits\r\n"
        out += b"System load: 3, cycle time: 1000, PID rate: 250, RX rate: 50, Air speed: 0 cm/s\r\n"

    def _cmd_tasks(self, args, out):
        out += b"Task list         rate/hz  max/us  avg/us maxload avgload     total/ms\r\n"
        rows = [
            (0, "SYSTEM", 10, 2, 1, 1), (1, "PID", 250, 120, 85, 4200),
            (2, "SERIAL", 100, 15, 6, 350), (3, "BEEPER", 100, 2, 1, 20),
            (4, "BATTERY", 50, 25, 12, 210), (5, "RX", 50, 45, 30, 900),
            (6, "GPS", 10, 250, 90, 430), (7, "COMPASS", 10, 90, 45, 130),
            (8, "BARO", 20, 80, 35, 180), (9, "ALTITUDE", 20, 25, 12, 90),
            (10, "ATTITUDE", 25, 35, 18, 240), (11, "NAV", 25, 120, 55, 610),
            (12, "OSD", 60, 45, 22, 540), (13, "TELEMETRY", 25, 15, 6, 80),
            (14, "LEDSTRIP", 60, 12, 5, 70),
        ]
        for tid, name, rate, mx, avg, total in rows:
            maxload = (mx * rate + 5000) // 1000
            avgload = (avg * rate + 5000) // 1000
            out += (f"{tid:2d} - {name:>12s}  {rate:6d}   {mx:5d}   {avg:5d} "
                    f"{maxload // 10:4d}.{maxload % 10}% {avgload // 10:4d}.{avgload % 10}%  "
                    f"{total:8d}\r\n").encode()
        out += b"Task check function            12       5                         1\r\n"
        out += b"Total (excluding SERIAL)                                 23.5%  11.2%\r\n"

    # --- настройки ---
    def _fmt_value(self, s: dict, full: bool = False) -> str:
        val = self.store.values.get(s["name"], 0)
        if s.get("mode") == MODE_LOOKUP:
            values = s.get("values") or []
            try:
                return values[int(val)]
            except (ValueError, IndexError):
                return str(val)
        if s["type"] == "string":
            return str(val)
        if s["type"] == "float":
            return ftoa(val)
        if s["type"] == "uint32":
            return str(int(val) & 0xFFFFFFFF)
        text = str(int(val))
        if full:
            text += f" {s.get('min', 0)} {s.get('max', 0)}"
        return text

    def _range_line(self, s: dict) -> str:
        if s.get("mode") == MODE_LOOKUP:
            return "Allowed values: " + ", ".join(s.get("values") or [])
        if s["type"] == "string":
            return f"Max. length: {s.get('max', 0)}"
        return f"Allowed range: {s.get('min', 0)} - {s.get('max', 0)}"

    def _cmd_get(self, args, out):
        name = args.strip()
        matched = 0
        for s in self.store.settings:
            if name.lower() in s["name"].lower():
                value = self._fmt_value(s)
                if s["name"] == "name" and value.startswith(" "):
                    value = f'"{value}"'
                out += f"{s['name']} = {value}\r\n".encode()
                out += (self._range_line(s) + "\r\n").encode()
                matched += 1
        if not matched:
            out += b"### ERROR: Invalid name"

    def _cmd_set(self, args, out):
        args = args.strip()
        if not args or args == "*":
            out += b"Current settings:\r\n"
            for s in self.store.settings:
                out += f"{s['name']} = {self._fmt_value(s, full=(args == '*'))}\r\n".encode()
            return
        if "=" not in args:
            self._cmd_get(args, out)
            return
        name, _, value = args.partition("=")
        name = name.rstrip(" ")
        value = value.lstrip(" ")
        for s in self.store.settings:
            if s["name"] == name.lower() or s["name"] == name:
                if s["type"] == "string":
                    text = value.upper()
                    if name in ("name", "pilot_name") and text.startswith('"') and text.endswith('"'):
                        text = text[1:-1]
                    self.store.set(s["name"], text)
                    return
                if s.get("mode") == MODE_LOOKUP:
                    for idx, candidate in enumerate(s.get("values") or []):
                        if candidate.lower() == value.lower():
                            self.store.set(s["name"], idx)
                            out += f"{s['name']} set to {self._fmt_value(s)}".encode()
                            return
                else:
                    try:
                        number = float(value) if s["type"] == "float" else int(value, 10)
                    except ValueError:
                        number = None
                    if (number is not None
                            and s.get("min", 0) <= number <= s.get("max", 0)):
                        self.store.set(s["name"], number)
                        out += f"{s['name']} set to {self._fmt_value(s)}".encode()
                        return
                out += b"### ERROR: Invalid value. "
                out += self._range_line(s).encode()
                return
        out += b"### ERROR: Invalid name"

    # --- dump / diff ---
    def _print_config(self, args, do_diff: bool, out):
        options = args.strip().split()
        scope = "master"
        showdefaults = False
        for opt in options:
            if opt in ("master", "control_profile", "mixer_profile",
                       "battery_profile", "all"):
                scope = opt
            elif opt == "showdefaults":
                showdefaults = True

        out += b"\r\n# version\r\n"
        self._cmd_version("", out)
        out += b"batch start\r\n"
        if do_diff and scope == "all":
            out += b"# reset configuration to default settings\r\n"
            out += b"defaults noreboot\r\n"

        def dump_section(sections, header=None):
            if header:
                out.extend(f"\r\n# {header}\r\n".encode())
            for s in self.store.settings:
                if s.get("section") not in sections:
                    continue
                value = self.store.values.get(s["name"], 0)
                equals_default = value == s.get("default", 0)
                if do_diff and equals_default:
                    continue
                if showdefaults and not equals_default:
                    default = s.get("default", 0)
                    shown = dict(s)
                    shown_val = default
                    if s.get("mode") == MODE_LOOKUP:
                        values = s.get("values") or []
                        shown_text = values[default] if 0 <= default < len(values) else str(default)
                    elif s["type"] == "string":
                        shown_text = str(default)
                    else:
                        shown_text = ftoa(default) if s["type"] == "float" else str(default)
                    out.extend(f"#{s['name']} = {shown_text}\r\n".encode())
                out.extend(f"set {s['name']} = {self._fmt_value(s)}\r\n".encode())

        def dump_features():
            out.extend(b"\r\n# features\r\n")
            features = self.store.features
            enabled = [f for f in features if self.fc.features & (1 << f["bit"])]
            for f in enabled:
                out.extend(f"feature {f['name']}\r\n".encode())

        def dump_serial():
            out.extend(b"\r\n# serial\r\n")
            fmt = "serial {} {} {} {} {} {}\r\n"
            for port in self.fc.serial_ports:
                out.extend(fmt.format(
                    port["id"], port["functions"],
                    BAUD_RATES[port["msp"]] if port["msp"] < len(BAUD_RATES) else 0,
                    BAUD_RATES[port["sensors"]] if port["sensors"] < len(BAUD_RATES) else 0,
                    BAUD_RATES[port["telemetry"]] if port["telemetry"] < len(BAUD_RATES) else 0,
                    BAUD_RATES[port["peripherals"]] if port["peripherals"] < len(BAUD_RATES) else 0,
                ).encode())

        def dump_aux():
            out.extend(b"\r\n# aux\r\n")
            boxes = self.store.boxes
            for i, mr in enumerate(self.fc.mode_ranges):
                if mr["start"] == 900 and mr["end"] == 900:
                    continue
                permanent = boxes[i]["permanent_id"] if i < len(boxes) else 0
                out.extend((f"aux {i} {permanent} {mr['aux']} "
                           f"{(mr['start'] - 900) // 25} {(mr['end'] - 900) // 25}\r\n").encode())

        def dump_map():
            out.extend(b"\r\n# map\r\n")
            letters = ["A", "E", "T", "R", "1", "2", "3", "4"]
            outmap = [""] * len(self.fc.rx_map)
            for i, slot in enumerate(self.fc.rx_map[:len(letters)]):
                if 0 <= slot < len(outmap):
                    outmap[slot] = letters[i]
            out.extend(f"map {''.join(outmap)}\r\n".encode())

        if scope in ("master", "all"):
            dump_section({"MASTER_VALUE"}, "master")
            dump_features()
            dump_serial()
            dump_aux()
            dump_map()
        if scope in ("control_profile", "all"):
            out.extend(b"\r\n# control_profile\r\n")
            out += f"control_profile {self.fc.profile + 1}\r\n\r\n".encode()
            dump_section({"PROFILE_VALUE", "CONTROL_VALUE", "EZ_TUNE_VALUE"})
        if scope in ("battery_profile", "all"):
            out.extend(b"\r\n# battery_profile\r\n")
            out += f"battery_profile {self.fc.battery_profile + 1}\r\n\r\n".encode()
            dump_section({"BATTERY_CONFIG_VALUE"})
        if scope in ("mixer_profile", "all"):
            out.extend(b"\r\n# mixer_profile\r\n")
            out += f"mixer_profile {self.fc.mixer_profile + 1}\r\n\r\n".encode()
            dump_section({"MIXER_CONFIG_VALUE"})

        out += b"\r\n# save configuration\r\nsave\r\n"
        out += b"batch end\r\n"

    def _cmd_dump(self, args, out):
        self._print_config(args, do_diff=False, out=out)

    def _cmd_diff(self, args, out):
        self._print_config(args, do_diff=True, out=out)

    # --- сохранение / выход ---
    def _cmd_save(self, args, out):
        out += b"Saving\r\nRebooting"
        self.fc.save_eeprom()
        self.reboot_requested = True

    def _cmd_defaults(self, args, out):
        out += b"Resetting to defaults"
        self.fc.store.reset_defaults()
        self.fc.features = 0
        self.fc.save_eeprom()
        if "noreboot" not in args:
            out += b"\r\nRebooting"
            self.reboot_requested = True

    def _cmd_exit(self, args, out):
        out += b"\r\nLeaving CLI mode, unsaved changes lost.\r\nRebooting"
        self.closed = True
        self.reboot_requested = True

    def _cmd_batch(self, args, out):
        if args.strip() == "start":
            self.batch = True
        else:
            self.batch = False

    # --- конфигурация ---
    def _cmd_feature(self, args, out):
        args = args.strip()
        features = self.store.features
        if not args:
            for f in features:
                state = "ON" if self.fc.features & (1 << f["bit"]) else "OFF"
                out += f"Enabled: {state:3s} {f['name']}\r\n".encode()
            return
        for token in args.split():
            enable = not token.startswith("-")
            name = token.lstrip("-").upper()
            for f in features:
                if f["name"] == name:
                    if enable:
                        self.fc.features |= 1 << f["bit"]
                    else:
                        self.fc.features &= ~(1 << f["bit"])
                    break
            else:
                out += f"### ERROR: Invalid name {name}\r\n".encode()
        self._cmd_feature("", out)

    def _cmd_serial(self, args, out):
        args = args.strip()
        if args:
            parts = args.split()
            try:
                port_id = int(parts[0])
                for port in self.fc.serial_ports:
                    if port["id"] == port_id:
                        if len(parts) >= 2:
                            port["functions"] = int(parts[1])
                        keys = ("msp", "sensors", "telemetry", "peripherals")
                        for key, text in zip(keys, parts[2:6]):
                            try:
                                want = int(text)
                            except ValueError:
                                continue
                            port[key] = BAUD_RATES.index(want) if want in BAUD_RATES else port[key]
                        return
                out += f"### ERROR: Invalid port ID {port_id}\r\n".encode()
                return
            except ValueError:
                out += b"### ERROR: Parse error\r\n"
                return
        for port in self.fc.serial_ports:
            bauds = [BAUD_RATES[port[k]] if port[k] < len(BAUD_RATES) else 0
                     for k in ("msp", "sensors", "telemetry", "peripherals")]
            out += f"serial {port['id']} {port['functions']} " \
                   f"{bauds[0]} {bauds[1]} {bauds[2]} {bauds[3]}\r\n".encode()

    def _cmd_aux(self, args, out):
        args = args.strip()
        boxes = self.store.boxes
        if not args:
            for i, mr in enumerate(self.fc.mode_ranges):
                if mr["start"] == 900 and mr["end"] == 900:
                    continue
                permanent = boxes[i]["permanent_id"] if i < len(boxes) else 0
                out += (f"aux {i} {permanent} {mr['aux']} "
                        f"{(mr['start'] - 900) // 25} {(mr['end'] - 900) // 25}\r\n").encode()
            return
        parts = args.split()
        try:
            idx = int(parts[0])
            if idx < len(self.fc.mode_ranges) and len(parts) >= 5:
                self.fc.mode_ranges[idx].update({
                    "aux": int(parts[2]),
                    "start": 900 + int(parts[3]) * 25,
                    "end": 900 + int(parts[4]) * 25,
                })
            elif len(parts) < 5 and idx < len(self.fc.mode_ranges):
                self.fc.mode_ranges[idx].update({"start": 900, "end": 900})
        except ValueError:
            out += b"### ERROR: Parse error\r\n"

    def _cmd_adjrange(self, args, out):
        args = args.strip()
        if not args:
            for i, ar in enumerate(self.fc.adjustment_ranges):
                if ar["start"] == 900 and ar["end"] == 900:
                    continue
                out += (f"adjrange {i} {ar['aux']} "
                        f"{(ar['start'] - 900) // 25} {(ar['end'] - 900) // 25} "
                        f"{ar['function']} {ar['switch']}\r\n").encode()
            return
        parts = args.split()
        try:
            idx = int(parts[0])
            if idx < len(self.fc.adjustment_ranges) and len(parts) >= 6:
                self.fc.adjustment_ranges[idx].update({
                    "aux": int(parts[1]), "start": 900 + int(parts[2]) * 25,
                    "end": 900 + int(parts[3]) * 25, "function": int(parts[4]),
                    "switch": int(parts[5])})
        except ValueError:
            out += b"### ERROR: Parse error\r\n"

    def _cmd_rxrange(self, args, out):
        args = args.strip()
        if not args:
            for i in range(8):
                out += f"rxrange {i} 1100 1900\r\n".encode()
            return
        pass

    def _cmd_map(self, args, out):
        args = args.strip().upper()
        if len(args) == len(self.fc.rx_map):
            try:
                slots = [RC_CHANNEL_LETTERS.index(ch) for ch in args]
                if sorted(slots) == list(range(len(args))):
                    self.fc.rx_map = slots
            except ValueError:
                out += b"### ERROR: Parse error\r\n"
                return
        elif args:
            out += b"### ERROR: Parse error\r\n"
            return
        letters = list(RC_CHANNEL_LETTERS)
        outmap = [""] * len(self.fc.rx_map)
        for i, slot in enumerate(self.fc.rx_map):
            if 0 <= slot < len(outmap) and i < len(letters):
                outmap[slot] = letters[i]
        out += f"Map: {''.join(outmap)}\r\n".encode()

    def _cmd_mmix(self, args, out):
        m = self.fc.mixer
        out += b"mmix 0  1.000  1.000 -1.000  1.000\r\n"
        out += b"mmix 1  1.000 -1.000 -1.000 -1.000\r\n"
        out += b"mmix 2  1.000 -1.000  1.000  1.000\r\n"
        out += b"mmix 3  1.000  1.000  1.000 -1.000\r\n"
        del m

    def _cmd_smix(self, args, out):
        out += b"\r\n"

    def _cmd_servo(self, args, out):
        for i in range(8):
            out += f"servo {i} 1000 2000 1500 100 -1\r\n".encode()

    def _cmd_motor(self, args, out):
        parts = args.split()
        if not parts:
            out += b"### ERROR: Parse error\r\n"
            return
        try:
            idx = int(parts[0])
            if len(parts) >= 2:
                self.fc.motors[idx] = int(parts[1])
            out += f"motor {idx}: {self.fc.motors[idx]}\r\n".encode()
        except (ValueError, IndexError):
            out += b"### ERROR: Parse error\r\n"

    def _cmd_control_profile(self, args, out):
        args = args.strip()
        if args:
            try:
                idx = int(args) - 1
                if 0 <= idx < 3:
                    self.fc.profile = idx
            except ValueError:
                pass
        out += f"control_profile {self.fc.profile + 1}\r\n".encode()

    def _cmd_battery_profile(self, args, out):
        args = args.strip()
        if args:
            try:
                idx = int(args) - 1
                if 0 <= idx < 3:
                    self.fc.battery_profile = idx
            except ValueError:
                pass
        out += f"battery_profile {self.fc.battery_profile + 1}\r\n".encode()

    def _cmd_mixer_profile(self, args, out):
        args = args.strip()
        if args:
            try:
                idx = int(args) - 1
                if 0 <= idx < 2:
                    self.fc.mixer_profile = idx
            except ValueError:
                pass
        out += f"mixer_profile {self.fc.mixer_profile + 1}\r\n".encode()

    def _cmd_pid(self, args, out):
        names = ["Roll", "Pitch", "Yaw", "Alt", "PosH", "PosR", "NavR", "Level", "Mag", "Vel"]
        for i, name in enumerate(names):
            out += f"{name:6s} 40 30 23  0\r\n".encode() if i < 3 else \
                   f"{name:6s}  0  0  0  0\r\n".encode()

    def _cmd_wp(self, args, out):
        out += b"wp 0: #0 LAYTHRU 0 0 0 0 0\r\n"

    def _cmd_stats(self, args, out):
        out += b"\r\n"

    def _cmd_resource(self, args, out):
        out += b"Currently active IO:\r\n"
        out += b"resource BEEPER 1 C13\r\nresource MOTOR 1 A08\r\n"
        out += b"resource SERIAL_TX 1 A09\r\nresource SERIAL_RX 1 A10\r\n"

    def _cmd_memory(self, args, out):
        out += b"Memory region blocks used\r\nFLASH: 0\r\n"
        out += b"  FLASH    0: 5268 of 16384 bytes used (32%)\r\n"

    def _cmd_sd_info(self, args, out):
        out += b"SD card: Not present\r\n"

    def _cmd_flash_info(self, args, out):
        out += b"Flash chip JEDEC ID: 0x000000\r\nSize: 0 MB\r\n"

    def _cmd_bootlog(self, args, out):
        out += b"log size written: 0 of 0 bytes reserved\r\n"

    def _cmd_temp_sensor(self, args, out):
        out += b"temp_sensor 0 1 0 1 0 0 1 0\r\n"

    def _cmd_showdebug(self, args, out):
        out += b"DEBUG 0: 0\r\nDEBUG 1: 0\r\nDEBUG 2: 0\r\nDEBUG 3: 0\r\n"

    def _cmd_gpssats(self, args, out):
        out += b"GPS is not UBLOX or does not report satelites."

    def _cmd_play_sound(self, args, out):
        out += b"Playing sound 0: GYRO_CALIBRATED\r\n"

    def _cmd_cli_delay(self, args, out):
        pass

    def _cmd_assert(self, args, out):
        out += b"### ERROR: Assertion failed at line 1\r\n" if args else b"\r\n"

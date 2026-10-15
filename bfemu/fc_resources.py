"""Таблица ресурсов, таймеры и DMA. Методы VirtualFC.

Примесь использует общее состояние контроллера; создание объекта — через bfemu.fc.VirtualFC.
"""
from __future__ import annotations



class ResourcesMixin:

    # ------------------------------------------------------------ ресурсы (команда resource)
    # Пины виртуальной платы BFR/VIRTUALFC (STM32F405) — как resourceTable[] в cli.c 4.5.4:
    # имя владельца, число слотов и назначенные пины. Порядок записей — порядок прошивки
    # (BEEPER, MOTOR, LED_STRIP, SERIAL_TX/RX, SOFTSERIAL_TX/RX, I2C, LED, SPI, ADC,
    # FLASH_CS, OSD_CS, GYRO_*, USB_DETECT); в нём же печатают `resource` и `resource show`.
    # Пины согласованы с тем, что эмулятор уже показывает во вкладке «Порты»: UART1…UART5,
    # GPS на UART1, приёмник на UART2, ESC_SENSOR на UART3, VTX (TBS SmartAudio) на UART4.
    RESOURCE_OWNERS = (
        ("BEEPER",        ["C13"]),
        ("MOTOR",         ["A00", "A01", "A02", "A03", None, None, None, None]),   # QUADX: 4 из 8
        ("LED_STRIP",     ["B08"]),
        ("SERIAL_TX",     ["A09", "D05", "D08", "C10", "C12"]),
        ("SERIAL_RX",     ["A10", "D06", "D09", "C11", "D02"]),
        ("SOFTSERIAL_TX", ["A08", "B00"]),
        ("SOFTSERIAL_RX", ["A11", "B11"]),
        ("I2C_SCL",       ["B06", None]),
        ("I2C_SDA",       ["B07", None]),
        ("LED",           ["C14"]),
        ("SPI_SCK",       ["A05", "B13"]),
        ("SPI_SDI",       ["A06", "B14"]),
        ("SPI_SDO",       ["A07", "B15"]),
        ("ADC_BATT",      ["C00"]),
        ("ADC_RSSI",      ["C02"]),
        ("ADC_CURR",      ["C01"]),
        ("FLASH_CS",      ["C15"]),
        ("OSD_CS",        ["B12"]),
        ("GYRO_EXTI",     ["C04"]),
        ("GYRO_CS",       ["A04"]),
        ("USB_DETECT",    ["C05"]),
    )
    # Пины с таймерами — как timerHardware[] в target-файле: (таймер, канал, AF).
    # Нужны для `timer` и для раздела таймеров в `resource show all`.
    RESOURCE_PIN_TIMERS = {
        "A00": ("TIM2", 1, 1), "A01": ("TIM2", 2, 1),
        "A02": ("TIM2", 3, 1), "A03": ("TIM2", 4, 1),
        "B08": ("TIM4", 3, 2),
    }
    RESOURCE_TIMERS = ("TIM1", "TIM2", "TIM3", "TIM4", "TIM5", "TIM8")   # таймеры F405 с каналами
    # Таблица заданий DMA — dmaoptEntryTable 4.5.4 (cli.c:5321) для виртуальной платы:
    # SPI_SDO/SPI_SDI и алиасы SPI_TX/SPI_RX (та же настройка SPI, совместимость
    # с конфигами 4.2.x), ADC 1…3 (ADCDEV_COUNT для F405), UART_TX/UART_RX 1…11
    # (UARTDEV_CONFIG_MAX = 11). Элемент: (устройство, индекс для CLI, поток или None);
    # None — DMA_OPT_UNUSED, такой слот прошивка печатает как «dma <устройство> <индекс> NONE».
    # SDIO в таблице прошивки тоже есть, но у него maxIndex = 0 — строк он не даёт.
    RESOURCE_DMA_OPT = (("SPI_SDO", 1, (2, 3, 3)), ("SPI_SDI", 1, (2, 0, 3)), ("SPI_TX", 1, (2, 3, 3)), ("SPI_RX", 1, (2, 0, 3)), ("ADC", 1, (2, 4, 0)), ("ADC", 2, None), ("ADC", 3, None), *tuple(("UART_TX", i, {1: (2, 7, 4), 2: (1, 6, 4)}.get(i)) for i in range(1, 12)), *tuple(("UART_RX", i, {1: (2, 5, 4), 2: (1, 5, 4)}.get(i)) for i in range(1, 12)))
    # Сколько индексов у устройства в dmaoptEntryTable (для `dma <устройство> <индекс>`).
    RESOURCE_DMA_DEVICES = {
        "SPI_SDO": 1, "SPI_SDI": 1, "SPI_TX": 1, "SPI_RX": 1,
        "ADC": 3, "SDIO": 0, "UART_TX": 11, "UART_RX": 11,
    }
    # Занятые DMA-потоки F405 (два контроллера по 8 потоков) — что печатает `dma show`.
    RESOURCE_DMA = {
        ("DMA1", 5): ("SERIAL_RX", 2), ("DMA1", 6): ("SERIAL_TX", 2),
        ("DMA2", 0): ("SPI_SDI", 1), ("DMA2", 3): ("SPI_SDO", 1),
        ("DMA2", 4): ("ADC_BATT", 1), ("DMA2", 5): ("SERIAL_RX", 1),
        ("DMA2", 7): ("SERIAL_TX", 1),
    }

    # ------------------------------------------------------------------ ресурсы платы
    def _default_resources(self) -> dict:
        """Заводские назначения пинов — pgResetFn_* / target-файл платы."""
        return {name: list(pins) for name, pins in self.RESOURCE_OWNERS}

    def _resources_from_json(self, data) -> dict:
        """Ресурсы из «флеша»: битые строки отбрасываем, длины выравниваем по таблице."""
        table = self._default_resources()
        if not isinstance(data, dict):
            return table
        for name, pins in table.items():
            saved = data.get(name)
            if not isinstance(saved, list):
                continue
            for i in range(min(len(pins), len(saved))):
                value = saved[i]
                if value is None:
                    pins[i] = None
                elif isinstance(value, str):
                    kind, pin = self.parse_pin(value)
                    pins[i] = pin if kind == "pin" else None
        return table

    @staticmethod
    def parse_pin(text) -> tuple:
        """strToPin() из cli.c: 'NONE' — освобождение, 'A00'…'D15' — пин.

        Возвращает («none», None) | («pin», 'A04') | (None, None), если разобрать не удалось.
        Порты платы — A…D (у STM32F405 четыре порта), пины 0…15: 'E01' прошивка тоже
        принимает по формату, но такого IO на плате нет — получается PARSING FAILED.
        """
        if text is None:
            return (None, None)
        word = str(text).strip().upper()
        if word == "NONE":
            return ("none", None)
        if len(word) >= 2 and word[0] in "ABCD" and word[1:].isdigit():
            number = int(word[1:])
            if 0 <= number <= 15:
                return ("pin", f"{word[0]}{number:02d}")
        return (None, None)

    def resource_owner_names(self) -> list:
        return [name for name, _pins in self.RESOURCE_OWNERS]

    def resource_slots(self, owner: str) -> int:
        """Число слотов владельца (RESOURCE_VALUE_MAX_INDEX): у одиночных — 1."""
        for name, pins in self.RESOURCE_OWNERS:
            if name == owner:
                return max(1, len(pins))
        return 0

    def resource_dump_lines(self, hide_unused=True, only_diff=False) -> list:
        """printResource(): дамп таблицы ресурсов в порядке таблицы прошивки.

        `resource` без аргументов зовёт printResource(DUMP_MASTER | HIDE_UNUSED) — только
        назначенные пины; `dump` — без HIDE_UNUSED, поэтому свободные слоты идут строками
        «resource MOTOR 5 NONE». В diff печатаются только слоты, отличающиеся от заводских.
        """
        defaults = self._default_resources()
        lines = []
        for name, _pins in self.RESOURCE_OWNERS:
            for i, pin in enumerate(self.resources[name]):
                if only_diff and pin == defaults[name][i]:
                    continue
                if pin:
                    lines.append(f"resource {name} {i + 1} {pin}")
                elif not hide_unused:
                    lines.append(f"resource {name} {i + 1} NONE")
        return lines

    def resource_show_lines(self) -> list:
        """`resource show`: «Currently active IO resource assignments» — как ioRecs[].

        Индекс печатается у многоместных владельцев (SERIAL_TX 1, MOTOR 2), у одиночных
        его нет — ровно как printResourceOwner()/IOInit() считают RESOURCE_INDEX.
        """
        lines = ["Currently active IO resource assignments:", "(reboot to update)", "-" * 20]
        for name, pins in self.RESOURCE_OWNERS:
            for i, pin in enumerate(self.resources[name]):
                if pin:
                    index = f" {i + 1}" if len(pins) > 1 else ""
                    lines.append(f"{pin}: {name}{index}")
        return lines

    def resource_note_lines(self, owner: str, index: int, pin: str) -> list:
        """resourceCheck(): пин уже занят — печатаем NOTE и освобождаем свой же дубль.

        Как в прошивке: дубль внутри того же владельца освобождается («… disabled»),
        занятость чужим владельцем только помечается — прошивка пин не отбирает.
        """
        notes = []
        if not pin:
            return notes
        for name, pins in self.RESOURCE_OWNERS:
            for i, current in enumerate(self.resources[name]):
                if current != pin:
                    continue
                cleared = False
                if name == owner:
                    if i == index:
                        continue
                    self.resources[name][i] = None
                    cleared = True
                number = f" {i + 1}" if len(pins) > 1 else ""
                # Прошивка печатает это через cliPrintf("\r\nNOTE: …") — с пустой строкой
                # перед предупреждением.
                note = f"\r\nNOTE: {pin} already assigned to {name}{number}"
                if cleared:
                    note += f". {name}{number} disabled"
                note += "."
                notes.append(note)
        return notes

    def timer_dump_lines(self) -> list:
        """`timer` без аргументов и секция «# timer» в dump: printTimer(DUMP_MASTER, NULL).

        На каждый пин две строки — задание и расшифровка канала, как printTimerDetails():
            timer A00 AF1
            # pin A00: TIM2 CH1 (AF1)
        """
        lines = []
        for pin in sorted(self.RESOURCE_PIN_TIMERS):
            lines.extend(self.timer_opt_lines(pin))
        return lines

    def timer_opt_lines(self, pin: str) -> list:
        """printTimerDetails() для одного пина: у пина без таймера — «timer <пин> NONE»."""
        timer = self.RESOURCE_PIN_TIMERS.get(pin)
        if not timer:
            return [f"timer {pin} NONE"]
        name, channel, af = timer
        return [f"timer {pin} AF{af}", f"# pin {pin}: {name} CH{channel} (AF{af})"]

    def dma_dump_lines(self, hide_unused=True) -> list:
        """printDmaopt(): `dma` без аргументов — DUMP_MASTER | HIDE_UNUSED, в `dump` — без него.

        Задание с DMA печатается парой строк (задание и комментарий с потоком и каналом
        из dmaGetChannelSpecByPeripheral), пустой слот — «dma ADC 2 NONE»; в конце идут
        задания DMA для пинов таймеров — «dma pin A00 NONE» (printTimerDmaoptDetails).
        """
        lines = []
        for device, index, spec in self.RESOURCE_DMA_OPT:
            if spec is None:
                if not hide_unused:
                    lines.append(f"dma {device} {index} NONE")
                continue
            controller, stream, channel = spec
            lines.append(f"dma {device} {index} 0")
            lines.append(f"# {device} {index}: DMA{controller} Stream {stream} Channel {channel}")
        if not hide_unused:
            for pin in sorted(self.RESOURCE_PIN_TIMERS):
                lines.append(f"dma pin {pin} NONE")
        return lines

    def dma_pin_lines(self, pin: str) -> list:
        """Задание DMA для пина таймера — printTimerDmaoptDetails(): у платы его нет."""
        if pin in self.RESOURCE_PIN_TIMERS:
            return [f"dma pin {pin} NONE"]
        return []

    def dma_opt_lines(self, device: str, index: int) -> list:
        """Строки текущего задания DMA — printPeripheralDmaoptDetails() для `dma <устр> <инд>`."""
        for name, number, spec in self.RESOURCE_DMA_OPT:
            if name.upper() != device.upper() or number != index:
                continue
            if spec is None:
                return [f"dma {name} {number} NONE"]
            controller, stream, channel = spec
            return [f"dma {name} {number} 0",
                    f"# {name} {number}: DMA{controller} Stream {stream} Channel {channel}"]
        return []

    def timer_show_lines(self) -> list:
        """`timer show`: showTimers() — активные таймеры, каналы и их владельцы."""
        owners = {}
        for name, pins in self.RESOURCE_OWNERS:
            for i, pin in enumerate(self.resources[name]):
                timer = self.RESOURCE_PIN_TIMERS.get(pin) if pin else None
                if timer:
                    owners[(timer[0], timer[1])] = (name, i + 1 if len(pins) > 1 else 0)
        lines = ["", "Currently active Timers:", "-" * 23]
        for timer in self.RESOURCE_TIMERS:
            used = sorted((channel, owner) for (name, channel), owner in owners.items() if name == timer)
            if not used:
                lines.append(f"{timer}: FREE")
                continue
            lines.append(f"{timer}:")
            for channel, (name, index) in used:
                suffix = f" {index}" if index else ""
                lines.append(f"    CH{channel} : {name}{suffix}")
        return lines

    def dma_show_lines(self) -> list:
        """`dma show`: showDma() — «DMA<контроллер> Stream <поток>: <владелец> [индекс]»."""
        lines = ["", "Currently active DMA:", "-" * 20]
        for controller in (1, 2):
            for stream in range(8):
                owner = self.RESOURCE_DMA.get((f"DMA{controller}", stream))
                if owner:
                    lines.append(f"DMA{controller} Stream {stream}: {owner[0]} {owner[1]}")
                else:
                    lines.append(f"DMA{controller} Stream {stream}: FREE")
        return lines

# Эталонные исходники прошивки Betaflight 4.5.4

Скопированы из https://github.com/betaflight/betaflight (тег `4.5.4`) — это версия,
которую эмулятор изображает (`[FC] Betaflight 4.5.4 / MSP API 1.46.0`).

Зачем они здесь: по этим файлам сверяются вещи, которые иначе приходится угадывать.

* `serial.c` (`src/main/io/serial.c`) — порядок портов в MSP и CLI. Массив
  `serialPortIdentifiers[]` начинается с `SERIAL_PORT_USB_VCP`, затем UART1…UART10, и именно
  в этом порядке `pgResetFn_serialConfig()` наполняет `portConfigs[]`; этот же порядок уходит
  в `MSP_SERIAL_CONFIG`/`MSP2_COMMON_SERIAL_CONFIG` и в CLI-дамп `serial`. Отсюда правило:
  **в таблице «Порты» USB VCP — первая строка** (Configurator строки не сортирует).
  Там же заводские скорости: MSP 115200, датчики/GPS 57600, телеметрия AUTO, «чёрный ящик» 115200.
* `io_serial.h` — `serialPortIdentifier_e` (USART1=0 … USB_VCP=20, SOFTSERIAL1/2=30/31, LPUART1=40),
  `baudRate_e` (AUTO=0, 9600=1, 19200=2, 38400=3, 57600=4, 115200=5, …) и `serialPortFunction_e`
  (MSP=1<<0, GPS=1<<1, RX_SERIAL=1<<6, BLACKBOX=1<<7, ESC_SENSOR=1<<10, VTX_SMARTAUDIO=1<<11 …).
* `serial_uart.c` (`src/main/pg/serial_uart.c`) — список UART-устройств платы (что вообще
  существует и может попасть в таблицу).
* `msp.c` (`src/main/msp/msp.c`) — обработчики MSP: раскладки `MSP_SERIAL_CONFIG`,
  `MSP_RAW_GPS`/`MSP_COMP_GPS`/`MSP_GPSSVINFO`, `MSP_BUILD_INFO` и т. д.
* `feature.h` (`src/main/config/feature.h`) — все существующие фичи. **Фичи `BLACKBOX` здесь нет**:
  её бит (11) давно вырезан, вместе с битами 1, 8, 19, 23, 24, 26, 29.
* `cli.c` (`src/main/cli/cli.c`) — тексты CLI. `featureNames[]` (строка 233) — список фич
  команды `feature`; на месте чёрного ящика стоит пустая строка `""`, поэтому
  `feature BLACKBOX` отвечает `###ERROR IN feature: INVALID NAME: BLACKBOX`
  (`ERROR_INVALID_NAME`, строка 217).
* `cli_settings.c` (`src/main/cli/settings.c`) — таблица CLI-настроек. Чёрный ящик — это настройки:
  `blackbox_device` (`lookupTableBlackboxDevice`: `NONE, SPIFLASH, SDCARD, SERIAL`),
  `blackbox_sample_rate` (`1/1`…`1/16`), `blackbox_mode`, `blackbox_high_resolution`,
  `blackbox_disable_*` (строка ~813).
* `blackbox.h` / `blackbox.c` (`src/main/blackbox/`) — `BlackboxDevice_e`
  (`BLACKBOX_DEVICE_NONE = 0, FLASH = 1, SDCARD = 2, SERIAL = 3`) и сама запись журнала.

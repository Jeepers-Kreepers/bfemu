# inavemu — виртуальный полётный контроллер INAV

Эмулятор полётного контроллера, к которому **INAV Configurator подключается как к
настоящему FC**: проходит рукопожатие, читает телеметрию и настраивает параметры —
вкладки Setup, Ports, Configuration, Receiver, Modes, Sensors.

Внутри — только стандартная библиотека Python 3.10+, никаких зависимостей.

Эталон: **INAV 9.1.0** (MSP API 2.5), проверено с **INAV Configurator 9.1.1**.
Версия прошивки меняется флагом `--fc-version` (Configurator 9.1.x принимает 9.0.0–9.x).

```
$ python3 -m inavemu --tcp 127.0.0.1:5761 --http 8088
[FC] INAV 9.1.0 / MSP API 2.5.0 / настроек в базе: 677
[TCP] виртуальный FC INAV слушает 127.0.0.1:5761
```

## Что эмулируется

| Слой | Что реализовано |
|---|---|
| Транспорт MSP | кадры v1 (`$M<`) и v2 (`$X<`), CRC XOR и CRC8-DVB-S2, «джамбо»-кадры, потоковый парсер с ресинхронизацией, ответ-ошибка `!` |
| Рукопожатие | `MSP_API_VERSION` (2.5) → `MSP_FC_VARIANT` (INAV) → `MSP_FC_VERSION` → `MSP_NAME` → `MSP_BUILD_INFO` → `MSP_BOARD_INFO` → `MSP_UID` → `MSP_BOXIDS` → `MSP_DATAFLASH_SUMMARY` — ровно как в `serial_backend.js` |
| Телеметрия | `MSP2_INAV_STATUS`, `MSP2_INAV_ANALOG`, `MSP_SENSOR_STATUS`, `MSP_ACTIVEBOXES`, `MSP_ATTITUDE`, `MSP_ALTITUDE`, `MSP_RAW_IMU`, `MSP_RC`, `MSP_RAW_GPS`, `MSP_COMP_GPS`, `MSP_SONAR`, `MSP2_INAV_AIR_SPEED`, `MSP2_INAV_TEMPERATURES`, `MSP2_INAV_DEBUG` (значения «живые»: синусоиды, батарея) |
| Настройки | 677 реальных настроек INAV 9.1.0 из `settings.yaml` (типы, min/max, lookup-таблицы, секции профилей) + протокол `MSP2_COMMON_SETTING_INFO` (0x1007), `MSP2_COMMON_SETTING` (0x1003), `MSP2_COMMON_SET_SETTING` (0x1004) |
| Конфигурация | `MSP_FEATURE`, `MSP2_INAV_MISC`, `MSP2_INAV_BATTERY_CONFIG`, `MSP2_INAV_MIXER`, `MSP2_CF_SERIAL_CONFIG` (11 портов как `serialPortIdentifiers[]` прошивки: USB VCP + **UART1..UART8** + SOFTSERIAL1/2; baud-индексы 115200/115200/AUTO/115200; SET пишет по identifier — неизвестный id даёт ошибку, отсутствующие порты не трогаются), `MSP_RX_CONFIG`, `MSP_RX_MAP`, `MSP_RC_DEADBAND`, `MSP2_INAV_RATE_PROFILE`, `MSP_MODE_RANGES` (40), `MSP_ADJUSTMENT_RANGES` (20), `MSP_SENSOR_ALIGNMENT`, `MSP_ADVANCED_CONFIG`, `MSP_VTX_CONFIG`, `MSP_BOARD_ALIGNMENT`, `MSP_CURRENT_METER_CONFIG`, `MSP_LOOP_TIME`, `MSP_INAV_PID` — и их SET-варианты |
| PID / тюнинг | `MSP2_PID` (11 банков) + `MSP2_SET_PID`, `MSP2_INAV_RATE_DYNAMICS`, `MSP2_INAV_EZ_TUNE`, `MSP_SENSOR_CONFIG`, `MSP2_INAV_SERVO_MIXER` — вкладка PID Tuning |
| OSD | `MSP2_INAV_OSD_LAYOUTS` (4 раскладки × 169 элементов), `MSP2_INAV_OSD_ALARMS`, `MSP2_INAV_OSD_PREFERENCES`, `MSP2_INAV_CUSTOM_OSD_ELEMENTS`, `MSP2_INAV_LOGIC_CONDITIONS_CONFIGURED` — вкладка OSD открывается и сохраняет позиции (заводская раскладка как в прошивке) |
| GPS | `MSP_RAW_GPS`, `MSP_COMP_GPS`, `MSP_GPSSTATISTICS`, `MSP2_ADSB_VEHICLE_LIST` — вкладка GPS |
| Калибровка | `MSP_CALIBRATION_DATA` (27 байт: флаги позиций + accZero/accGain + magZero + opflow.Scale + magGain — как читает JS), `MSP_SET_CALIBRATION_DATA` (26 байт, без байта флагов), `MSP_ACC_CALIBRATION`, `MSP_MAG_CALIBRATION`, `MSP2_INAV_OPFLOW_CALIBRATION` — вкладка Calibration. `MSP_SENSOR_CONFIG` отдаёт настройки `*_hardware` после сымитированной автодетекции при старте (стенд оснащён: MPU6000/MS5611/HMC5883 + pitot VIRTUAL + rangefinder/opflow MSP) — кнопки магнитометра и оптического потока активны |
| Миссии / навигация | `MSP_WP_GETINFO`, `MSP_WP`, `MSP_SET_WP` (21 байт, порядковая загрузка, JUMP p1↔индекс), `MSP_WP_MISSION_LOAD`/`SAVE` (миссия в NVM), `MSP2_INAV_SAFEHOME` (8), `MSP2_INAV_FW_APPROACH` (17), `MSP2_INAV_GEOZONE` + `MSP2_INAV_GEOZONE_VERTEX` (63 зоны / 126 вершин, круговые с радиусом) — вкладка Mission Control загружает и сохраняет миссии |
| Failsafe | `MSP_FAILSAFE_CONFIG` / `MSP_SET_FAILSAFE_CONFIG` (20 байт) — вкладка Failsafe |
| Программирование | `MSP2_INAV_LOGIC_CONDITIONS_SINGLE`/`SET` (64 условия + маска `CONFIGURED`), `MSP2_INAV_LOGIC_CONDITIONS_STATUS`, `MSP2_INAV_GVAR_STATUS`, `MSP2_INAV_PROGRAMMING_PID` (+SET/STATUS) — вкладки Programming |
| Blackbox | `MSP2_BLACKBOX_CONFIG` / `MSP2_SET_BLACKBOX_CONFIG` (9 байт), `MSP_DATAFLASH_SUMMARY`/`READ`/`ERASE`, `MSP_SDCARD_SUMMARY` — вкладка Onboard Logging (Blackbox) |
| Режимы | 55 боксов INAV (ARM, ANGLE, NAV RTH, NAV WP, POSHOLD, CRUISE…) из `fc_msp_box.c` |
| CLI | «сырой» CLI-порт (`#`, эхо, приглашение `# `, TAB-автодополнение, CTRL-D/CTRL-L): **62 команды `cmdTable[]` прошивки** — `help`, `version`, `status`, `tasks`, `get`, `set`, `dump`, `diff`, `defaults`, `save`, `exit`, `feature`, `serial`, `aux`, `adjrange`, `map`, `mmix`, `smix`, `servo`, `motor`, `pid`, `wp`, `resource`, `batch`…; `save`/`exit`/`defaults` печатают «Rebooting» и рвут линк |
| «Флеш» | `MSP_EEPROM_WRITE` и CLI `save` пишут конфигурацию в `inav_eeprom.json`; после перезагрузки (`MSP_SET_REBOOT` — линк рвётся) она поднимается обратно |

Неизвестные коды MSP отвечают кадром-ошибкой (как прошивка), так что
непокрытые вкладки не роняют конфигуратор. Сплошной свип по исходникам
inav-configurator 9.1.1: обрабатываются **все 163 MSP-кода**, которые
конфигуратор умеет отправлять (вкладки Calibration, LED Strip, Mixer,
Outputs, Advanced Tuning и остальные).

Батарейный блок `MSP2_INAV_MISC`/`MSP2_INAV_BATTERY_CONFIG` связан с базой
настроек в единый источник (как в прошивке: `batteryMetersConfig` /
`batteryProfile`): CLI `set bat_voltage_src = SAG_COMP` переключает
«Voltage source» на вкладке Configuration, а Save вкладки меняет значение
CLI (`get`). То же для `vbat_scale`, `bat_cells`, `vbat_cell_*_voltage`,
`battery_capacity*`, `failsafe_throttle`, `mag_declination`, `min_command`,
`current_meter_*`. Единицы провода — как в `MSPHelper.js`: напряжение ячеек
в сотых вольта (425 = 4.25 В), `mag_declination` в десятых градуса
(настройка — в сотых). Значения по умолчанию — стоковые INAV 9.1.0
(`bat_voltage_src = RAW`, `bat_cells = 0` — автоопределение ячеек).

Вкладка Receiver — так же: `MSP_RX_CONFIG`/`MSP_RC_DEADBAND`/
`MSP2_INAV_RATE_PROFILE` читают и пишут те же настройки (`rssi_channel`,
`deadband`, `yaw_deadband`, `3d_deadband_throttle`, `serialrx_provider`,
`min_check`/`max_check`, `rx_min_usec`/`rx_max_usec`, `receiver_type`,
`thr_mid`/`thr_expo`, `rc_expo`, `tpa_rate`/`tpa_breakpoint`, rates):
`set rssi_channel = 5` переключает RSSI channel, `set deadband` — поле
Deadband, Save вкладки меняет значения CLI.

## Куда смотреть

| Если нужно | Файл |
|---|---|
| запустить и подключить Configurator | этот README, «Быстрый старт» |
| обработчики MSP и состояние FC | [`inavemu/fc.py`](inavemu/fc.py) |
| веб-интерфейс: оборудование и монитор | [`inavemu/pages.py`](inavemu/pages.py), [`inavemu/server.py`](inavemu/server.py) |
| база настроек | [`inavemu/settings.py`](inavemu/settings.py), [`inavemu/data/inav_data.json`](inavemu/data/inav_data.json) |
| перегенерация данных из исходников INAV | [`tools/gen_data_inav.py`](tools/gen_data_inav.py) |
| проверки | `tests/` (pytest + сквозной smoke) |

## Быстрый старт

```bash
cd inavemu
python3 -m inavemu                        # TCP 5761 для Configurator + веб 8080
python3 tools/demo_session.py             # «клиент-Configurator» в терминале
```

Веб-интерфейс поднимается по умолчанию на `http://127.0.0.1:8080/`
(`--http PORT` — другой порт, `--http 0` — без веба, `--http-host` — адрес,
по умолчанию `0.0.0.0`):

Подключение из INAV Configurator (настольное приложение):

1. в списке портов сначала выберите **TCP** — это выбор *типа соединения*, а не адреса;
2. в отдельном поле **IP:Port** введите `127.0.0.1:5761` — **без** префикса `tcp://`
   (конфигуратор разбирает поле как `IP:порт`, схема туда не пишется;
   поле `tcp://…` уходит в парсер как IP «tcp» — и соединение не открывается);
3. нажмите **Подключиться**.

> В браузерной PWA-версии Configurator TCP недоступен — нужен именно настольный вариант.

**Запуск двух и более виртуальных контроллеров запрещён:** второй экземпляр
`inavemu` (даже с другим `--tcp`) завершается с ошибкой `виртуальный контроллер
INAV уже запущен`. Блокировка снимается автоматически при завершении процесса.

### Веб-интерфейс: оборудование

Страница **«Оборудование»** (`/equipment`) в браузере подключает и отключает
датчики виртуального FC — как оборудование на настоящем контроллере:

* **Гироскоп, Акселерометр, Магнитометр, Барометр, GPS, Оптический поток,
  Сонар, Скорость (pitot)** — восемь тумблеров; по умолчанию всё подключено;
* индикаторы в INAV Configurator (Gyro · Accel · Mag · Baro · GPS · Flow ·
  Sonar · Speed) следят за этим состоянием: подключено — горит, отключено —
  погашено; такая же полоска индикаторов нарисована и на самой странице;
* кнопки калибровки магнитометра/оптического потока и выбор датчиков во
  вкладке Configuration подчиняются тем же переключателям (через
  `MSP_SENSOR_CONFIG` и настройки `*_hardware`);
* «отключённые» датчики ведут себя как на настоящем FC: GPS без фикса,
  сонар и pitot показывают ноль;
* переключателям можно командовать и напрямую: `GET /api/state`,
  `POST /api/equipment` с JSON вида `{"mag": false, "gps": true}`.

Страница **«Моторы и сервы»** (`/outputs`) эмулирует подключение выходов:

* **Моторы (1–12)** и **Сервоприводы (0–18)** — счётчики и быстрые пресеты;
  подключение пишет mixer-правила, поэтому во вкладке Outputs конфигуратора
  появляется столько слайдеров моторов и ярких полос серв, сколько
  подключено (как у настоящего FC с соответствующим микшером);
* тест моторов в Outputs работает: слайдеры шлют `MSP_SET_MOTOR`, значения
  эхом возвращаются в `MSP_MOTOR`; «подключённые» сервы живут в `MSP_SERVO`;
* API: `POST /api/outputs` с JSON вида `{"motors": 6, "servos": 3}`
  (частичные обновления, диапазоны проверяются).

Страница **«Монитор»** (`/`) — состояние FC и журнал запросов.

Нужен доступ с другой машины — откройте адрес явно:

```bash
python3 -m inavemu --tcp 0.0.0.0:5761 --http 8088 --http-host 0.0.0.0
```

### Установка как пакета (необязательно)

```bash
pip install .          # появится команда inavemu
inavemu --tcp 127.0.0.1:5761
```

### Проверки

```bash
python3 -m pytest tests -q   # 78 проверок: кодек, настройки, обработчики, CLI (~3 сек)
python3 tests/smoke_test.py  # 69 сквозных проверок «Configurator + CLI по TCP» (~3 сек)
```

## Данные: откуда 677 настроек

`inavemu/data/inav_data.json` сгенерирован из исходников INAV 9.1.0:

```bash
python3 tools/gen_data_inav.py /path/to/inav-9.1.0 \
    --configurator /path/to/inav-configurator-9.1.1 --tag 9.1.0
```

Генератор (только stdlib + gcc):

* разбирает `src/main/fc/settings.yaml` (78 lookup-таблиц, 55 групп, 677 настроек);
* определяет типы полей через gcc тем же трюком, что и `src/utils/settings.rb`
  прошивки (`поле.__type_detect_` → тип из сообщения об ошибке), — сверено:
  165 настроек с явным типом в YAML совпали с gcc, расхождений нет;
* разрешает min/max/дефолты через константы из заголовков; `default_value:
  :target` (дефолт задаёт `target.h`) берётся из таблицы `TARGET_DEFAULTS`
  generic-таргета (`vbat_scale = 1100` — `battery.h`, `current_meter_scale
  = 400`, `current_meter_offset = 0`);
* берёт список боксов из `fc/fc_msp_box.c` и смысл битов `MSP_FEATURE` из
  исходников конфигуратора.

## Как устроен проект

```
inavemu/
├── inavemu/
│   ├── msp.py            # кодек MSP v1/v2
│   ├── server.py         # TCP-порт FC + веб-интерфейс (+ режим CLI)
│   ├── pages.py          # страницы веб-интерфейса: Монитор, Оборудование
│   ├── fc.py             # состояние и обработчики MSP (в т.ч. оборудование)
│   ├── settings.py       # база настроек + протокол настроек MSP
│   ├── cli.py            # CLI: команды cmdTable[] прошивки
│   ├── single_instance.py
│   └── data/inav_data.json
├── tools/
│   ├── gen_data_inav.py  # генератор данных из исходников INAV
│   └── demo_session.py   # демо-клиент
└── tests/                # pytest + smoke_test.py
```

Фаза 3 реализована: миссии Mission Control (`MSP_SET_WP`, `wp`/`geozone`/`safehome`),
failsafe, логические условия и программирование (logic conditions / gvar),
blackbox-вкладка. Дальше — доводка по результатам проверок с реальным Configurator.

## Лицензия

GPL-3.0-or-later (как и INAV/Betaflight). Форматы протокола сверены с исходниками
iNavFlight/inav 9.1.0 (GPLv3) и iNavFlight/inav-configurator 9.1.1 (GPLv3);
данные `inav_data.json` сгенерированы из этих исходников.

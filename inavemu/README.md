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

## Что эмулируется (фаза 1)

| Слой | Что реализовано |
|---|---|
| Транспорт MSP | кадры v1 (`$M<`) и v2 (`$X<`), CRC XOR и CRC8-DVB-S2, «джамбо»-кадры, потоковый парсер с ресинхронизацией, ответ-ошибка `!` |
| Рукопожатие | `MSP_API_VERSION` (2.5) → `MSP_FC_VARIANT` (INAV) → `MSP_FC_VERSION` → `MSP_NAME` → `MSP_BUILD_INFO` → `MSP_BOARD_INFO` → `MSP_UID` → `MSP_BOXIDS` → `MSP_DATAFLASH_SUMMARY` — ровно как в `serial_backend.js` |
| Телеметрия | `MSP2_INAV_STATUS`, `MSP2_INAV_ANALOG`, `MSP_SENSOR_STATUS`, `MSP_ACTIVEBOXES`, `MSP_ATTITUDE`, `MSP_ALTITUDE`, `MSP_RAW_IMU`, `MSP_RC`, `MSP_RAW_GPS`, `MSP_COMP_GPS`, `MSP_SONAR`, `MSP2_INAV_AIR_SPEED`, `MSP2_INAV_TEMPERATURES`, `MSP2_INAV_DEBUG` (значения «живые»: синусоиды, батарея) |
| Настройки | 677 реальных настроек INAV 9.1.0 из `settings.yaml` (типы, min/max, lookup-таблицы, секции профилей) + протокол `MSP2_COMMON_SETTING_INFO` (0x1007), `MSP2_COMMON_SETTING` (0x1003), `MSP2_COMMON_SET_SETTING` (0x1004) |
| Конфигурация | `MSP_FEATURE`, `MSP2_INAV_MISC`, `MSP2_INAV_BATTERY_CONFIG`, `MSP2_INAV_MIXER`, `MSP2_CF_SERIAL_CONFIG`, `MSP_RX_CONFIG`, `MSP_RX_MAP`, `MSP_RC_DEADBAND`, `MSP2_INAV_RATE_PROFILE`, `MSP_MODE_RANGES` (40), `MSP_ADJUSTMENT_RANGES` (20), `MSP_SENSOR_ALIGNMENT`, `MSP_ADVANCED_CONFIG`, `MSP_VTX_CONFIG`, `MSP_BOARD_ALIGNMENT`, `MSP_CURRENT_METER_CONFIG`, `MSP_LOOP_TIME`, `MSP_INAV_PID` — и их SET-варианты |
| Режимы | 55 боксов INAV (ARM, ANGLE, NAV RTH, NAV WP, POSHOLD, CRUISE…) из `fc_msp_box.c` |
| «Флеш» | `MSP_EEPROM_WRITE` пишет конфигурацию в `inav_eeprom.json`; после перезагрузки (`MSP_SET_REBOOT` — линк рвётся) она поднимается обратно |

Неизвестные коды MSP отвечают кадром-ошибкой (как прошивка), так что
непокрытые вкладки не роняют конфигуратор.

## Куда смотреть

| Если нужно | Файл |
|---|---|
| запустить и подключить Configurator | этот README, «Быстрый старт» |
| обработчики MSP и состояние FC | [`inavemu/fc.py`](inavemu/fc.py) |
| база настроек | [`inavemu/settings.py`](inavemu/settings.py), [`inavemu/data/inav_data.json`](inavemu/data/inav_data.json) |
| перегенерация данных из исходников INAV | [`tools/gen_data_inav.py`](tools/gen_data_inav.py) |
| проверки | `tests/` (pytest + сквозной smoke) |

## Быстрый старт

```bash
cd inavemu
python3 -m inavemu --tcp 127.0.0.1:5761 --http 8088   # TCP для Configurator + веб-монитор
python3 tools/demo_session.py                         # «клиент-Configurator» в терминале
```

Подключение из INAV Configurator (настольное приложение, сборка Tauri):
включите *Manual connection*, в поле порта введите `tcp://127.0.0.1:5761` → **Connect**.

> В браузерной PWA-версии Configurator TCP недоступен — нужен именно настольный вариант.

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
python3 -m pytest tests -q   # 41 проверка: кодек, настройки, обработчики (~2 сек)
python3 tests/smoke_test.py  # 51 сквозная проверка «Configurator по TCP» (~2 сек)
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
* разрешает min/max/дефолты через константы из заголовков;
* берёт список боксов из `fc/fc_msp_box.c` и смысл битов `MSP_FEATURE` из
  исходников конфигуратора.

## Как устроен проект

```
inavemu/
├── inavemu/
│   ├── msp.py            # кодек MSP v1/v2
│   ├── server.py         # TCP-порт FC + веб-монитор
│   ├── fc.py             # состояние и обработчики MSP
│   ├── settings.py       # база настроек + протокол настроек MSP
│   └── data/inav_data.json
├── tools/
│   ├── gen_data_inav.py  # генератор данных из исходников INAV
│   └── demo_session.py   # демо-клиент
└── tests/                # pytest + smoke_test.py
```

Планируемое (фазы 2–3): полный CLI (`get`/`set`/`dump`/`diff`/`save`),
навигация и миссии (`MSP_WP`, Mission Control), OSD, failsafe, логические условия.

## Лицензия

GPL-3.0-or-later (как и INAV/Betaflight). Форматы протокола сверены с исходниками
iNavFlight/inav 9.1.0 (GPLv3) и iNavFlight/inav-configurator 9.1.1 (GPLv3);
данные `inav_data.json` сгенерированы из этих исходников.

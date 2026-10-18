# Курс INAV CLI 9.1.0 — черновик 1

## Назначение

Это **отдельный полный курс INAV CLI**, созданный как редакторский аналог уже существующего курса Betaflight CLI. Повторяется его учебная последовательность: содержание, 13 теоретических уроков, ответы на контрольные вопросы и четыре самостоятельные лабораторные. Но каждая команда, настройка, термин, пример и ссылка в INAV-материалах привязаны к **INAV firmware 9.1.0**; Betaflight-текст не выдаётся за урок INAV.

Материалы предназначены для последующих авторских правок пользователя. Их не следует считать окончательно утверждённой редакцией до проверки/правок.

## Структура

Для уникальности названий материалов на technobee.ru к названию каждой записи курса добавлен суффикс «— INAV». Суффикс применяется в поле `Name`, в заголовке статьи `<h1>` и в тексте внутренних ссылок на статьи курса.

1. Введение: CLI INAV и вкладки Configurator.
2. Конфигурация: `diff`, `dump`, `defaults`, `save`.
3. Порты (UART).
4. Основная конфигурация и Mixer.
5. Приёмник.
6. Режимы полёта (Auxiliary/Modes, Adjustments).
7. PID и Rates.
8. Питание и батарея.
9. Моторы, ESC и Outputs.
10. Failsafe.
11. OSD.
12. LED-лента и бипер.
13. VTX, GPS, Navigation и Blackbox.
14. Ответы на контрольные вопросы.
15–18. Четыре отдельные лабораторные работы: 3.5″, 5″, 7″, 10″.

Полная карта вкладок INAV Configurator 9.1.0 размещена в уроке 1 и на странице содержания. Учитываются INAV-специфические страницы Mixer, Outputs, Mission Control, Landing, Adjustments, Receiver MSP, Logging и диагностические/служебные вкладки. Для UI-действий без универсальной CLI-команды это явно отмечено. В INAV Configurator 9.1.0 нет отдельных вкладок Battery, Motors и VTX: соответствующие настройки распределены между Configuration, Mixer/Outputs, Ports и CLI.

## Основные файлы

- `soderzhanie-kursa-inav-9-1-0.html` — содержание и карта курса.
- `urok-01-vvedenie-chto-takoe-cli-inav-9-1-0.html` … `urok-13-vtx-gps-i-blackbox-inav-9-1-0.html` — статьи уроков.
- `otvety-na-kontrolnye-voprosy-inav-9-1-0.html` — ответы к урокам.
- `laboratornaya-rabota-35-speedybee-f405-mini-inav-9-1-0.html`
- `laboratornaya-rabota-5-speedybee-f405-v4-inav-9-1-0.html`
- `laboratornaya-rabota-7-speedybee-f7-v3-inav-9-1-0.html`
- `laboratornaya-rabota-10-matek-h743-slim-v3-inav-9-1-0.html`
- `inav-9-1-0-zoo-full-course.csv` — отдельная CSV-копия полного INAV-курса.
- `betaflight-cli-kurs-zoo-with-laboratory-works-and-inav-9-1-0-full-course.csv` — полная объединённая копия (исходная Betaflight-версия + полные INAV-материалы).
- `soderzhanie-kursa-inav-link-patch.csv` — патч для отдельной ссылки из содержания Betaflight на курс INAV.

Предыдущие частичные CSV имена сохранены как промежуточные файлы; не использовать их вместо `inav-9-1-0-zoo-full-course.csv`. В import-ready CSV ссылки из оглавления преобразованы в Zoo-пути `/index.php/pilotazh/obuchenie-kursy/cli-v-inav/item/<alias>.html`; HTML-файлы сохраняют относительные ссылки для локального чтения.

## Технические ограничения

- Все INAV-команды и параметры проверяются по официальному firmware tag [`9.1.0`](https://github.com/iNavFlight/inav/tree/9.1.0/docs); названия вкладок — по tag [`9.1.0` INAV Configurator](https://github.com/iNavFlight/inav-configurator/tree/9.1.0/tabs).
- Фактических результатов стендов нет. В отчётах нет сфабрикованных `version`, `status`, `serial`, `resource`, `diff all`, ESC/RPM/Blackbox показаний.
- Дефолты из документации указываются именно как defaults, а не как данные аппаратуры.
- Для INAV 9.1.0 RPM gyro filter включается только при проверенно работающей ESC telemetry и верно отображаемых RPM; Betaflight `dshot_bidir_enabled`/`dshot_bidir` не используются.
- Значения `osd_video_system` сверяются с Settings reference 9.1.0: `AUTO`, `PAL`, `NTSC`, `HDZERO`, `DJIWTF`, `AVATAR`, `BF43COMPAT`, `BFHDCOMPAT`, `DJI_NATIVE`. Имеющееся расхождение в отдельных OSD-страницах помечено; для CLI enum приоритет у version-pinned Settings reference и фактических опций Configurator.
- В INAV CLI docs tag 9.1.0 есть пример `control_profile 0`, но tagged source `src/main/fc/cli.c` реализует аргумент выбора профиля как 1-based. Урок 2 не предлагает копировать конфликтующий пример; активный профиль читается командой `control_profile` без аргумента, переключение отмечено как изменение состояния.
- Подтверждённые targets для четырёх лабораторных: `SPEEDYBEEF405MINI`, `SPEEDYBEEF405V4`, `SPEEDYBEEF7V3`, `MATEKH743`. Альтернативы и версии указаны только при наличии источника. Не подтверждённый target не подставляется.
- В начале каждой лабораторной сохранён заголовок «Задание на проведение работ (описание стенда)»; каждая работа отдельная.

## QA черновика 1

- INAV CSV: 19 записей (содержание + 13 уроков + ответы + 4 лабораторные), 25 столбцов, aliases уникальны.
- Полная объединённая копия: 38 записей (19 Betaflight + 19 INAV), 25 столбцов; исходные Betaflight CSV не перезаписаны.
- Все 30 модулей вкладок INAV Configurator 9.1.0 имеют отдельную строку в карте «вкладка → CLI / нет прямого эквивалента» урока 1.
- Проверены 25 имён настроек и 39 CLI-команд по тегированной документации INAV 9.1.0; 50 ссылок на официальные страницы GitHub проверены на доступность.
- Проверены структура 19 HTML-страниц, ссылки на локальные страницы и соответствие CSV `Text1` тексту HTML `main` (у содержания ссылки для импорта преобразованы в Zoo-пути).
- Исходный Betaflight CSV не менялся: SHA-256 `a3036edbee41de05641079874d7b36223f0bc5bd5f436bd022f441c0336e7a58`; проверенный Betaflight source CSV: `d7fa334a0917d3da366f8e056c04a384db5b12e3209957000f5b994b0c8a6abf`.

SHA-256 созданных CSV:

- `inav-9-1-0-zoo-full-course.csv`: `a527802a491c72c8a815c133bebaaddb5534082df342e1458f3595b0ee5e5cb5`
- `betaflight-cli-kurs-zoo-with-laboratory-works-and-inav-9-1-0-full-course.csv`: `4d00f06dfb14303d2004815dcf7cb0bc9dc6fcb0e44dfe4652758f71109ddce4`
- `soderzhanie-kursa-inav-link-patch.csv`: `36ca5797cf47351df00246060518fb27977978f480054404d416c7812ccd40f5`

## Источники

- [INAV 9.1.0 firmware docs](https://github.com/iNavFlight/inav/tree/9.1.0/docs)
- [INAV CLI reference](https://github.com/iNavFlight/inav/blob/9.1.0/docs/Cli.md)
- [INAV 9.1.0 Configurator tabs](https://github.com/iNavFlight/inav-configurator/tree/9.1.0/tabs)

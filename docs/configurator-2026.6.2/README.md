# Эталонные исходники Betaflight Configurator 2026.6.2

Скопированы из https://github.com/betaflight/betaflight-configurator (тег `2026.6.2`) —
это ровно та версия, которой подключаются к эмулятору (2026.6.2, 091127c).

Зачем они здесь: по этим файлам проверяются неочевидные правила интерфейса, которые
определяют, что именно видно в Configurator:

* `sidebar_items.js` — какие вкладки существуют и при каких условиях.
  Вкладка «GPS» показывается только при build-опции `USE_GPS`:
  `{ key: "gps", mode: "connected", i18n: "tabGPS", buildOptions: ["USE_GPS"] }`.
* `Features.js` — список функций вкладки «Конфигурация» (биты и `dependsOn`).
* `MSPHelper.js` — раскладка MSP-ответов (`MSP_GPS_CONFIG` и др. читаются здесь).
* `GpsTab.vue` — логика самой вкладки: порядок запросов, разбор спутников и — важно —
  `if (channels > 16)`: подробная таблица сигналов (имена систем, зелёные метки
  «используется», подписи качества) рисуется, только когда спутников больше 16;
  при 16 и меньше — упрощённый список с прочерками и числовым качеством.
* `fc.js` — таблица `FIRMWARE_BUILD_OPTIONS` (коды build-опций) и `processBuildOptions()`.

* `AuxiliaryTab.vue`, `modeRanges.js`, `rcChannel.js`, `bit.js` — вкладка «Режимы». Она
  строит строки из `MSP_MODE_RANGES` (34) и `MSP_MODE_RANGES_EXTRA` (238): строка рисуется
  только при `start < end`, а слот «900/900» считается заполнителем. Состояние режима
  (`bit_check(config.mode, mode.index)`) берётся из `MSP_STATUS`, причём `index` — это
  позиция режима в списке `MSP_BOXNAMES`, а не permanent id. Слайдеры диапазона ходят
  шагами по 25 мкс (`modeRanges.js`: CHANNEL_STEP = 25, CHANNEL_MIN = 900, CHANNEL_MAX = 2100).

"""Запись чёрного ящика и MSP Dataflash. Методы VirtualFC.

Примесь использует общее состояние контроллера; создание объекта — через bfemu.fc.VirtualFC.
"""
from __future__ import annotations

from . import bb_scenarios
from .fc_constants import (
    MSP_DATAFLASH_BUFFER_SIZE,
)


class BlackboxMixin:

    # --- чёрный ящик: накопитель, чтение логов, настройки записи -----------------
    def blackbox_sample_rate(self) -> int:
        """blackboxConfig()->sample_rate: 0…4 → «P interval» = 1 << rate."""
        return max(0, min(4, int(self.settings.get("blackbox_sample_rate", 1) or 0)))

    def blackbox_p_interval(self) -> int:
        """blackboxPInterval из blackboxInit(): период кадров P в циклах PID."""
        return bb_scenarios.log_every_for_sample_rate(self.blackbox_sample_rate())

    def _h_blackbox_config(self, w, r):
        """MSP_BLACKBOX_CONFIG (80): настройки записи — как sbufWrite в msp.c 4.5.4."""
        p_interval = self.blackbox_p_interval() or 1
        w.u8(1)                                        # Blackbox supported
        w.u8(int(self.settings.get("blackbox_device", 1) or 0))
        w.u8(1)                                        # Rate numerator (не используется)
        w.u8(p_interval)                               # blackboxGetRateDenom()
        w.u16(max(1, 32 // p_interval))                # blackboxGetPRatio() = I / P
        w.u8(self.blackbox_sample_rate())              # sample_rate (API 1.44+)
        w.u32(bb_scenarios.mask_from_settings(self.settings))   # fields_disabled_mask (API 1.45+)

    def _h_set_blackbox_config(self, w, r):
        """MSP_SET_BLACKBOX_CONFIG (81): device, rateNum, rateDenom, [pRatio], [sample_rate], [mask].

        Прошивка применяет настройки, только если blackboxMayEditConfig() — у эмулятора
        запись не идёт никогда, поэтому принимаем всегда. Частота записи определяет
        «P interval» в заголовках логов, поэтому при её смене «флеш» пересобирается.
        """
        device = r.u8()
        r.u8()                       # rateNum — устаревшее поле
        rate_denom = r.u8()
        p_ratio = None
        if r.remaining >= 2:
            p_ratio = r.u16()
        sample_rate = None
        if r.remaining >= 1:
            sample_rate = r.u8()
        mask = None
        if r.remaining >= 4:
            mask = r.u32()
        if sample_rate is None:
            # sample_rate в MSP не пришёл — считаем из «P ratio» (blackboxCalculateSampleRate)
            p_ratio = p_ratio or (32 * max(1, rate_denom) // max(1, rate_denom))
            sample_rate = max(0, min(4, (max(1, p_ratio)).bit_length() - 1))
        sample_rate = max(0, min(4, int(sample_rate)))
        old = self.blackbox_sample_rate()
        self.settings["blackbox_device"] = device
        self.settings["blackbox_sample_rate"] = sample_rate
        if mask is not None:
            bb_scenarios.apply_mask_to_settings(self.settings, mask)
        self._log(f"[ЧЯ] настройки записи от вкладки: устройство {device}, "
                  f"частота 1/{1 << sample_rate} (P interval {1 << sample_rate})"
                  + (f", маска полей 0x{mask:08x}" if mask is not None else ""))
        if sample_rate != old:
            # «Флеш» пересобирается под новую частоту записи — в фоне, не держа вкладку
            self.blackbox.ensure_async(self.blackbox_p_interval())

    def _h_dataflash_summary(self, w, r):
        """MSP_DATAFLASH_SUMMARY (70): ready/supported, число секторов, размер и занятое.

        Обработчик ничего не ждёт: если «флеш» собран под другую частоту записи,
        наполнение уходит в фон, а вкладка получает то, что уже лежит в накопителе
        (пустой накопитель — ноль занятых байт, обычный ответ прошивки).
        """
        store = self.blackbox
        store.ensure_async(self.blackbox_p_interval())
        w.u8(store.flags)
        w.u32(store.sectors)
        w.u32(store.size)
        w.u32(store.used)

    def _h_dataflash_read(self, w, r):
        """MSP_DATAFLASH_READ (71): запрос [addr u32, len u16, allowCompression u8] → кусок лога.

        Ответ: [addr u32, readLen u16, compression u8 (0 = без сжатия), data].
        Запрос короче 6 байт — устаревший формат: 128 байт без заголовка сжатия,
        хвост добивается нулями (serializeDataflashReadReply в msp.c).
        """
        store = self.blackbox
        store.ensure_async(self.blackbox_p_interval())
        data_size = r.remaining
        address = r.u32()
        if data_size >= 6:
            request_len = r.u16()
            if r.remaining:
                r.u8()                                     # allowCompression: сжатие не шлём
            legacy = False
        else:
            request_len = 128
            legacy = True
        read_len = min(int(request_len), MSP_DATAFLASH_BUFFER_SIZE)
        chunk = store.read(address, read_len)
        w.u32(address)
        if legacy:
            w.raw(chunk + b"\x00" * (read_len - len(chunk)))
        else:
            w.u16(len(chunk))
            w.u8(0)                                        # NO_COMPRESSION
            w.raw(chunk)

    def _h_dataflash_erase(self, w, r):
        """MSP_DATAFLASH_ERASE (72): blackboxEraseAll() — стираем «флеш» чёрного ящика."""
        self.blackbox.erase()

    def _h_sdcard_summary(self, w, r):
        w.u8(0).u8(0).u32(0).u32(0)

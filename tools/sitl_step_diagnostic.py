"""Диагностировать ранний отказ опытного подтверждения кадров, с записью stderr SITL.

Счётчики и запись изменяют тайминг и не относятся к штатному SITL. Результаты
не являются отдельной проверкой повторяемости.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from bfemu import aircraft_sitl
from tools import sitl_step_probe as probe


def add_diagnostics(source):
    probe.patch_sources(source)
    gyro = source/'src/main/drivers/accgyro/accgyro_virtual.c'
    target = source/'src/main/target/SITL/sitl.c'
    include = source/'src/main/target/SITL/bfemu_step_probe.inc'
    g = gyro.read_text()
    g = probe.replace_unique(g, '    bfemu_gyro_sequence++;',
                             '    bfemu_gyro_sequence++;\n'
                             '    if (bfemu_gyro_sequence < 6) fprintf(stderr, "[STEP] SET %u\\n", bfemu_gyro_sequence);')
    g = probe.replace_unique(g, '    __atomic_store_n(&bfemu_gyro_consumed_sequence, bfemu_gyro_sequence, __ATOMIC_RELEASE);',
                             '    if (bfemu_gyro_sequence < 6) fprintf(stderr, "[STEP] READ %u\\n", bfemu_gyro_sequence);\n'
                             '    __atomic_store_n(&bfemu_gyro_consumed_sequence, bfemu_gyro_sequence, __ATOMIC_RELEASE);')
    t = target.read_text()
    t = probe.replace_unique(t, '    const uint64_t realtime_now = micros64_real();',
                             '    const uint64_t realtime_now = micros64_real();\n'
                             '    static unsigned bfemu_fdm_count;\n'
                             '    if (++bfemu_fdm_count < 6) fprintf(stderr, "[STEP] FDM %.8f realtime %lu\\n", '
                             'pkt->timestamp, (unsigned long)realtime_now);')
    i = include.read_text()
    i = probe.replace_unique(i, '    if (!consumed || consumed == last_ack) return;',
                             '    static unsigned bfemu_ack_calls;\n'
                             '    if (++bfemu_ack_calls < 6 || (consumed && consumed != last_ack)) '
                             'fprintf(stderr, "[STEP] PID consumed=%u last=%u mask=%u\\n", consumed, last_ack, bfemu_step_mask);\n'
                             '    if (!consumed || consumed == last_ack) return;')
    gyro.write_text(g)
    target.write_text(t)
    include.write_text(i)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    try:
        index = argv.index('--output-dir')
        out = Path(argv[index+1]).expanduser().resolve()
    except (ValueError, IndexError):
        return probe.main(argv)  # стандартная ошибка argparse
    original_patch, original_popen = probe.patch_sources, aircraft_sitl.subprocess.Popen
    def launch(cmd, *args, **kwargs):
        if isinstance(cmd, list) and cmd and str(cmd[0]).endswith('betaflight_SITL.elf'):
            with (out/'sitl-stderr.log').open('ab', buffering=0) as log:
                kwargs['stderr'] = log
                return original_popen(cmd, *args, **kwargs)
        return original_popen(cmd, *args, **kwargs)
    probe.patch_sources = add_diagnostics
    aircraft_sitl.subprocess.Popen = launch
    try:
        # Сохранить исходную функцию в замыкании вместо рекурсивного вызова.
        def patch(source):
            probe.patch_sources = original_patch
            try:
                add_diagnostics(source)
            finally:
                probe.patch_sources = patch
        probe.patch_sources = patch
        return probe.main(argv)
    finally:
        probe.patch_sources = original_patch
        aircraft_sitl.subprocess.Popen = original_popen


if __name__ == '__main__':
    sys.exit(main())

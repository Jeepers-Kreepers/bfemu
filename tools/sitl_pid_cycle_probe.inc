/* Diagnostic-only insertion before Betaflight's subTaskPidController call.
 * Accumulates exact task-entry intervals into contiguous monotonic 100 ms bins;
 * the write and clock_gettime calls can perturb scheduling. No PID math is changed.
 */
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

struct __attribute__((packed)) bfemu_pid_record {
    uint64_t start_ns, end_ns, entries, min_host_ns, max_host_ns, sim_sum_us;
    uint32_t min_sim_us, max_sim_us;
};

static void bfemu_pid_probe(uint32_t current_time_us)
{
    static int fd = -2;
    static uint64_t previous_ns;
    static uint32_t previous_sim;
    static struct bfemu_pid_record r;
    if (fd == -2) {
        const char *path = getenv("BFEMU_PIDTRACE");
        fd = path && *path ? open(path, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0600) : -1;
    }
    if (fd < 0) return;
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return;
    uint64_t now = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
    if (!previous_ns) {
        previous_ns = now;
        previous_sim = current_time_us;
        r.start_ns = now;
        r.min_host_ns = UINT64_MAX;
        r.min_sim_us = UINT32_MAX;
        return;
    }
    uint64_t dt = now - previous_ns;
    uint32_t sim_dt = current_time_us - previous_sim; /* unsigned wrap is intentional */
    previous_ns = now;
    previous_sim = current_time_us;
    r.end_ns = now;
    r.entries++;
    if (dt < r.min_host_ns) r.min_host_ns = dt;
    if (dt > r.max_host_ns) r.max_host_ns = dt;
    r.sim_sum_us += sim_dt;
    if (sim_dt < r.min_sim_us) r.min_sim_us = sim_dt;
    if (sim_dt > r.max_sim_us) r.max_sim_us = sim_dt;
    if (now - r.start_ns >= 100000000ULL) {
        (void)write(fd, &r, sizeof r);
        r = (struct bfemu_pid_record){.start_ns = now, .min_host_ns = UINT64_MAX, .min_sim_us = UINT32_MAX};
    }
}

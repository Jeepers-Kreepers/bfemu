/* Diagnostic-only sampler after Betaflight PID, before motor mixing.
 * Separate SITL build only. Sampling and file I/O perturb execution timing.
 * Record layout is little-endian on the supported platforms; see Python decoder.
 */
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

struct __attribute__((packed)) bfemu_terms_axis {
    float p, i, d, f, sum, gyro, setpoint;
};
struct __attribute__((packed)) bfemu_terms_record {
    uint64_t monotonic_ns;
    uint32_t sim_us, armed;
    struct bfemu_terms_axis axis[3];
};

static void bfemu_terms_probe(uint32_t sim_us)
{
    static int fd = -2;
    static unsigned counter, count;
    static struct bfemu_terms_record records[32];
    int saved_errno = errno;
    if (++counter % 16 != 0) goto end;
    if (fd == -2) {
        const char *path = getenv("BFEMU_TERMS_TRACE");
        fd = path && *path ? open(path, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0600) : -1;
    }
    if (fd < 0) goto end;
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) goto end;
    struct bfemu_terms_record *r = &records[count++];
    r->monotonic_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
    r->sim_us = sim_us;
    r->armed = ARMING_FLAG(ARMED) ? 1 : 0;
    for (int axis = 0; axis < 3; axis++) {
        r->axis[axis] = (struct bfemu_terms_axis){
            pidData[axis].P, pidData[axis].I, pidData[axis].D, pidData[axis].F,
            pidData[axis].Sum, gyro.gyroADCf[axis], pidGetPreviousSetpoint(axis)
        };
    }
    if (count == 32) {
        const char *ptr = (const char *)records;
        size_t left = sizeof(records);
        while (left) {
            ssize_t n = write(fd, ptr, left);
            if (n < 0 && errno == EINTR) continue;
            if (n <= 0) { close(fd); fd = -1; break; }
            ptr += n;
            left -= (size_t)n;
        }
        count = 0;
    }
end:
    errno = saved_errno;
}

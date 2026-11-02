/* Diagnostic-only virtual gyro observations. Included inside USE_VIRTUAL_GYRO.
 * All calls run while the existing gyro device mutex is held. Observations and
 * block writes perturb timing. The final incomplete block is not flushed on SIGTERM.
 */
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

struct __attribute__((packed)) bfemu_gyro_record {
    uint64_t monotonic_ns;
    uint32_t sequence, reads_ok, reads_empty, overwritten;
    int16_t x, y, z;
    uint16_t reserved;
};
static uint32_t bfemu_gyro_ok, bfemu_gyro_empty;

static void bfemu_gyro_read(bool ok)
{
    if (ok) bfemu_gyro_ok++;
    else bfemu_gyro_empty++;
}

static void bfemu_gyro_set(bool pending, int16_t x, int16_t y, int16_t z)
{
    static int fd = -2;
    static uint32_t sequence, count;
    static struct bfemu_gyro_record records[32];
    int saved_errno = errno;
    if (fd == -2) {
        const char *path = getenv("BFEMU_GYRO_TRACE");
        fd = path && *path ? open(path, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0600) : -1;
    }
    if (fd < 0) goto end;
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) goto end;
    records[count++] = (struct bfemu_gyro_record){
        (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec,
        ++sequence, bfemu_gyro_ok, bfemu_gyro_empty, pending ? 1U : 0U, x, y, z, 0
    };
    bfemu_gyro_ok = bfemu_gyro_empty = 0;
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

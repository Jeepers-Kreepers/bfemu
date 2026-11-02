/* Observational probe for ONE fingerprinted x86-64 Linux Betaflight SITL ELF.
 * Addresses and callsites are verified against the SHA-256 in the Python runner.
 * Captures only successful motor-packet send calls; this is not a PID-cycle trace.
 * LD_PRELOAD and file writes change scheduling. Do not use on a flight controller.
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <link.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

static int (*real_trylock)(pthread_mutex_t *);
static ssize_t (*real_sendto)(int, const void *, size_t, int, const struct sockaddr *, socklen_t);
static uintptr_t base;
static int logfd = -1;
static pthread_mutex_t *target;
static float *motor, *piddata;
static int16_t *idle;
static double *simrate;
/* One mixer pass writes motors in FC order, then calls updateComplete. The mask
 * covers that pass, not skipped passes or packet delivery. */
static _Thread_local unsigned count, mask, last_count, last_mask;
static _Thread_local int send_ready;
/* Calls to updateComplete, not a direct count of every PID execution. */
static _Thread_local unsigned attempts_since_send;

struct __attribute__((packed)) record {
    double t, simrate;
    unsigned count, mask;
    float pkt[4], motor[4], pid[5];
    int idle;
    unsigned attempts_since_send;
};

static int find_base(struct dl_phdr_info *info, size_t size, void *result)
{
    (void)size;
    if (!info->dlpi_name || !info->dlpi_name[0]) {
        *(uintptr_t *)result = info->dlpi_addr;
        return 1;
    }
    return 0;
}

__attribute__((constructor)) static void init(void)
{
    real_trylock = dlsym(RTLD_NEXT, "pthread_mutex_trylock");
    real_sendto = dlsym(RTLD_NEXT, "sendto");
    dl_iterate_phdr(find_base, &base);
    target = (pthread_mutex_t *)(base + 0x555c0);
    motor = (float *)(base + 0x55840);
    idle = (int16_t *)(base + 0x4c7e4);
    piddata = (float *)(base + 0x55c40);
    simrate = (double *)(base + 0x4a280);
    const char *path = getenv("BFEMU_MASKTRACE");
    if (path && *path) {
        logfd = open(path, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0600);
    }
}

int pthread_mutex_trylock(pthread_mutex_t *mutex)
{
    int rc = real_trylock(mutex);
    uintptr_t caller = (uintptr_t)__builtin_return_address(0) - base;
    if (mutex == target) {
        if (caller == 0xbe8a || caller == 0xbf07) {
            if (count < 32 && rc == 0) mask |= 1u << count;
            count++;
        } else if (caller == 0xc00f) {
            attempts_since_send++;
            last_count = count;
            last_mask = mask;
            count = mask = 0;
            send_ready = (rc == 0);
        }
    }
    return rc;
}

ssize_t sendto(int fd, const void *buf, size_t len, int flags, const struct sockaddr *addr, socklen_t n)
{
    ssize_t rc = real_sendto(fd, buf, len, flags, addr, n);
    if (logfd >= 0 && send_ready && rc == 16 && len == 16 && addr && n >= sizeof(struct sockaddr_in) &&
        addr->sa_family == AF_INET && ntohs(((const struct sockaddr_in *)addr)->sin_port) == 9002) {
        int saved = errno;
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        struct record r = {0};
        r.t = ts.tv_sec + ts.tv_nsec * 1e-9;
        r.simrate = *simrate;
        r.count = last_count;
        r.mask = last_mask;
        memcpy(r.pkt, buf, 16);
        memcpy(r.motor, motor, 16);
        memcpy(r.pid, piddata, 20);
        r.idle = *idle;
        r.attempts_since_send = attempts_since_send;
        attempts_since_send = 0;
        (void)write(logfd, &r, sizeof r);
        errno = saved;
        send_ready = 0;
    }
    return rc;
}

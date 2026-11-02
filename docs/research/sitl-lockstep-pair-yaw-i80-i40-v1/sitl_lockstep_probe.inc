/* Research only: a single-threaded FDM/RC input and simulated clock.
 * The upstream PID/mixer remain intact; the scheduler and SITL target change.
 * Startup and MSP configuration occur in wall time before activation.
 */
#include "fc/runtime_config.h"
#include "fc/rc.h"
#include "flight/pid.h"
#include "sensors/gyro.h"
extern uint32_t bfemu_gyro_consumed_sequence;
static udpLink_t bfemu_sync_input, bfemu_sync_output;
static uint32_t bfemu_step_mask, bfemu_frame_pid_count;
static uint32_t bfemu_frame_sequence;
static uint64_t bfemu_simulated_us;
static bool bfemu_simclock_active;

void bfemu_step_begin(void) { bfemu_step_mask = 0; }
void bfemu_step_ack(void) { bfemu_frame_pid_count++; }

/* Captured inside pidController for each invocation, before motor mixing. */
#define BFEMU_PID_SAMPLES 32
struct __attribute__((packed)) bfemu_pid_sample {
    uint32_t time_us;
    float axis[3][7]; /* actual setpoint, gyro, P, I, D, F, Sum */
};
static struct bfemu_pid_sample bfemu_pid_trace[BFEMU_PID_SAMPLES];
static float bfemu_pid_setpoint[3], bfemu_pid_gyro[3];
static uint32_t bfemu_pid_trace_count;

void bfemuLockstepAxis(unsigned axis, float setpoint, float gyro)
{
    if (!bfemu_simclock_active || axis >= 3) return;
    bfemu_pid_setpoint[axis] = setpoint;
    bfemu_pid_gyro[axis] = gyro;
}

void bfemuLockstepPidDone(uint32_t time_us)
{
    if (!bfemu_simclock_active) return;
    if (bfemu_pid_trace_count >= BFEMU_PID_SAMPLES) {
        fprintf(stderr, "[bfemu] Too many PID samples in frame.\n");
        exit(2);
    }
    struct bfemu_pid_sample *sample = &bfemu_pid_trace[bfemu_pid_trace_count++];
    sample->time_us = time_us;
    for (unsigned axis = 0; axis < 3; axis++) {
        sample->axis[axis][0] = bfemu_pid_setpoint[axis];
        sample->axis[axis][1] = bfemu_pid_gyro[axis];
        sample->axis[axis][2] = pidData[axis].P;
        sample->axis[axis][3] = pidData[axis].I;
        sample->axis[axis][4] = pidData[axis].D;
        sample->axis[axis][5] = pidData[axis].F;
        sample->axis[axis][6] = pidData[axis].Sum;
    }
}

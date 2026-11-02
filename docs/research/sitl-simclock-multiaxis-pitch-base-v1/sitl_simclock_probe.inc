/* Research only: a single-threaded FDM/RC input and simulated clock.
 * The upstream PID/mixer remain intact; the scheduler and SITL target change.
 * Startup and MSP configuration occur in wall time before activation.
 */
#include "fc/runtime_config.h"
#include "fc/rc.h"
extern uint32_t bfemu_gyro_consumed_sequence;
static udpLink_t bfemu_sync_input, bfemu_sync_output;
static uint32_t bfemu_step_mask, bfemu_frame_pid_count;
static uint32_t bfemu_frame_sequence;
static uint64_t bfemu_simulated_us;
static bool bfemu_simclock_active;

void bfemu_step_begin(void) { bfemu_step_mask = 0; }
void bfemu_step_ack(void) { bfemu_frame_pid_count++; }

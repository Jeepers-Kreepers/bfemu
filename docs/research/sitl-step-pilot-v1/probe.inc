/* Experimental acknowledgement after a real Betaflight PID/motor cycle.
 * Not upstream SITL. The fixed-size response is local-only, one sensor frame in
 * flight. RC and FC clocks remain asynchronous; this is NOT full lockstep.
 */
extern uint32_t bfemu_gyro_consumed_sequence;
static udpLink_t bfemu_step_link;
static uint32_t bfemu_step_mask;

void bfemu_step_begin(void)
{
    bfemu_step_mask = 0;
}

void bfemu_step_ack(void)
{
    static uint32_t last_ack;
    const uint32_t consumed = __atomic_load_n(&bfemu_gyro_consumed_sequence, __ATOMIC_ACQUIRE);
    if (!consumed || consumed == last_ack || (bfemu_step_mask & 15U) != 15U) return;
    struct __attribute__((packed)) {
        uint32_t sequence, mask;
        servo_packet motors;
    } response = { consumed, bfemu_step_mask, pwmPkt };
    if (udpSend(&bfemu_step_link, &response, sizeof(response)) == sizeof(response)) {
        last_ack = consumed;
    }
}

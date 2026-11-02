/* Called on the main scheduler thread, after upstream RC functions are defined. */
struct bfemu_frame_input {
    uint32_t sequence, reserved;
    fdm_packet fdm;
    rc_packet rc;
};
struct __attribute__((packed)) bfemu_frame_output {
    uint32_t sequence, read_sequence, mask, pid_cycles, armed, disable_flags;
    uint64_t sim_us;
    servo_packet motors;
    uint32_t scheduler_enabled, scheduler_period, scheduler_target, scheduler_now;
};

bool bfemuSimclockActive(void)
{
    return __atomic_load_n(&bfemu_simclock_active, __ATOMIC_ACQUIRE);
}

extern void bfemuSchedRebase(void);
bool bfemuSimclockInput(void)
{
    struct bfemu_frame_input input;
    struct sockaddr_in sender;
    socklen_t length = sizeof(sender);
    int n = recvfrom(bfemu_sync_input.fd, &input, sizeof(input), MSG_DONTWAIT,
                     (struct sockaddr *)&sender, &length);
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return false;
    if (n != sizeof(input) || sender.sin_addr.s_addr != htonl(INADDR_LOOPBACK) || input.reserved ||
        input.sequence != bfemu_frame_sequence + 1) {
        fprintf(stderr, "[bfemu] Invalid frame, aborting.\n");
        exit(2);
    }
    if (!bfemuSimclockActive()) {
        uint64_t now = micros64();
        __atomic_store_n(&bfemu_simulated_us, (now / 25) * 25, __ATOMIC_RELAXED);
        __atomic_store_n(&bfemu_simclock_active, true, __ATOMIC_RELEASE);
        bfemuSchedRebase();
    }
    bfemu_frame_sequence = input.sequence;
    bfemu_frame_pid_count = 0;
    bfemu_step_mask = 0;
    rcPkt = input.rc;
    if (!rc_received) {
        rxRuntimeState.channelCount = SIMULATOR_MAX_RC_CHANNELS;
        rxRuntimeState.rcReadRawFn = readRCSITL;
        rxRuntimeState.rcFrameStatusFn = rxRCFrameStatus;
        rxRuntimeState.rxProvider = RX_PROVIDER_UDP;
        rc_received = true;
    }
    updateState(&input.fdm);
    return true;
}

void bfemuSimclockAdvance(void)
{
    __atomic_add_fetch(&bfemu_simulated_us, 25, __ATOMIC_RELAXED);
}

extern uint32_t bfemuSchedEnabled(void), bfemuSchedPeriod(void), bfemuSchedTarget(void), bfemuSchedNow(void);
void bfemuSimclockReply(void)
{
    struct bfemu_frame_output output = {
        bfemu_frame_sequence,
        __atomic_load_n(&bfemu_gyro_consumed_sequence, __ATOMIC_ACQUIRE),
        bfemu_step_mask, bfemu_frame_pid_count, ARMING_FLAG(ARMED) ? 1U : 0U,
        getArmingDisableFlags(), __atomic_load_n(&bfemu_simulated_us, __ATOMIC_RELAXED), pwmPkt,
        bfemuSchedEnabled(), bfemuSchedPeriod(), bfemuSchedTarget(), bfemuSchedNow()
    };
    if (udpSend(&bfemu_sync_output, &output, sizeof(output)) != sizeof(output)) {
        fprintf(stderr, "[bfemu] Frame acknowledgement send failed.\n");
        exit(2);
    }
}

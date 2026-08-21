uint64 timestamp

uint8 TAIL_STATE_TRACKING  = 0
uint8 TAIL_STATE_RAMP_DOWN = 1
uint8 TAIL_STATE_DWELL     = 2
uint8 TAIL_STATE_RAMP_UP   = 3

bool simulation_model
bool tail_limited
uint8 tail_state
uint32 reversal_count

float32 tail_force_requested
float32 tail_force_commanded
float32 tail_force_error
float32 tail_output_normalized
float32 tail_pwm_estimate
float32 tail_rpm_estimate
float32 reversal_dwell_required
float32 reversal_dwell_elapsed
float32 pitch_unallocated
float32 pitch_unallocated_nm

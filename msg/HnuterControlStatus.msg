uint64 timestamp

bool active
float32[4] attitude_setpoint_q
float32[3] attitude_error
float32 attitude_error_angle
float32 governor_scale
float32[3] rate_setpoint
float32[3] torque_p
float32[3] torque_d
float32[3] torque_i
float32[3] torque_gravity
float32[3] torque_bias
float32[3] torque_trim
float32[3] torque_command
float32[3] force_body

# JEPA SITL diagnostics; effective mass equals HNTR_MASS on hardware builds.
float32 effective_mass
float32 mass_target
float32 mass_update_force_error
bool mass_updated
bool mass_update_rejected

# Hnuter Sphere Run Analysis

- Input: `/home/hnuter/px4_ws_ros2/hnuter_sphere_direct_1782820934.csv`
- Surface-analysis interval: `39.05-359.56 s`
- Estimated contact force: `4.59 +/- 1.12 N`
- Estimated contact-force 5-95 percentile: `3.02-6.59 N`
- Position RMSE `[x, y, z]`: `[0.310, 0.076, 0.097] m`
- Position-error norm RMSE / P95: `0.333 / 0.524 m`
- SO(3) attitude-error RMSE / P95: `3.61 / 5.39 deg`
- Continuous-pitch MAE / P95: `1.67 / 3.41 deg`

The log does not contain a Gazebo contact-wrench sensor. Contact force is
estimated from rigid-body normal dynamics:

`N_est = (F_thrust + F_gravity) dot n_in - m * a dot n_in`.

Acceleration is obtained from velocity after uniform resampling and a
Savitzky-Golay derivative. The radial offset is measured at the vehicle origin,
not at the collision-mesh contact point.

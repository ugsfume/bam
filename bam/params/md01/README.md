MD01 (Mangdang 3-in-1 servo, one channel) identified on servo 6 (inner coaxial shaft) of the
bench module, dataset `data_md01-6v2` (175 logs, 2026-09-21: 0.15 m arm with 0 / 79 / 160 /
238 g and 0.10 m arm with 121 / 238 / 303 g, kp 40..160 with kp 120 held out), actuator class
`md01i` (MD01CurrentActuator: current setpoint = kp * 0.030 * ratio * error, clipped to the
fitted current_limit, tau = kt * I), CMA-ES to a plateau (16k-31k trials).

Configuration identified (written and logged on every run): AT32 flash gains kp_current 6e-4,
kff_current 3e-4, max_pwm 0.99, kd_position 0, frame current cap 1500 mA (the robot's), 12 V
bench supply. The loop was measured at a blocked output: setpoint = kp_position [mA/deg] *
error, duty = kp_current * (setpoint - I) + kff_current * setpoint, I ~ (duty * 12 V - 1.2 V) / 11 ohm,
0.82 A at the cap. See docs/mp3_actuator_control_architecture.md.

Position MAE [mrad] (reference simulator, held-out kp 120 / all) and current MAE [mA]:
  m1 29.6 / 32.3, 125    m2 29.6 / 32.3, 126    m3 24.7 / 26.0, 81
  m4 26.1 / 26.6, 124    m5 22.4 / 23.3, 167    m6 22.2 / 23.3, 88
Log-vs-log repeatability of the actuator: 3.3 mrad, 5 mA.

Working model: m3 (kt 0.81 Nm/A, current_limit 0.566 A, load_friction_base 0.14, armature 9e-4);
alternative m6 (kt 0.64, limit 0.67, load_friction_external 0.26). kt is a gauge in this law
(rescaling it with error_gain_ratio and current_limit leaves the position MAE unchanged); the
physical quantity is tau_max = kt * current_limit: 0.43-0.46 Nm for m1/m3/m5/m6 (m2 0.35, m4 0.51).
m3 and m6 reproduce the measured current; m5 does not (kt 0.47 with a 1.6 gain ratio). Physical
kt from the breakaway pair of the static holds ~ 0.65 Nm/A. Known deficiencies: (1) the two
heavy blocks (>= 0.3 Nm) fit at 31-48 mrad because the housing heated 18 C during each kp sweep
and the real stall angle is non-monotonic in kp - a data confound, not a friction-structure
limit; (2) identified at kd_position 0 while the robot preset runs 800 (up to 133 mrad log-vs-log
on step targets); (3) the effective torque ceiling is calibrated to a 35-65 C session.

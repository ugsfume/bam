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
alternative m6 (kt 0.64, limit 0.67, load_friction_external 0.26). In this law kt and the limit
are only identified as a product (kt * limit = 0.43-0.46 Nm = the stall torque; the balance
current of the static holds gives kt ~ 0.55-0.6 Nm/A); m3 and m6 reproduce the measured current,
m5 does not (kt 0.47 with a 1.6 gain ratio). Known deficiency: heavy loads (>= 0.3 Nm) at high
kp stall the real arm at 1.2-1.3 rad because the gearbox's static friction is ~45 % of the
transmitted torque; BAM's friction budgets cannot carry that, and the models lift the arm
further (31-48 mrad on those blocks vs 10-25 elsewhere).

MD01 (Mangdang 3-in-1 servo, one channel) identified on servo 3 of the bench rig,
dataset `data_md01-3` (90 logs: no arm / 0.101 kg / 0.171 kg on a 0.15 m, 20.3 g arm,
kp 60..140, kp = 140 held out), actuator class `md01i` (MD01CurrentActuator),
CMA-ES 23k-32k trials per model (plateaued), 2026-09-20.

Position MAE [mrad] (reference simulator; MuJoCo backend agrees within 0.3):
  m1 28.8 (val 31.8)  m2 27.7 (35.1)  m3 27.8 (31.7)  m4 25.5 (32.0)  m5 24.1 (32.9)  m6 23.8 (28.4)
m1..m5 share kt ~0.57-0.60 Nm/A and current_limit 0.42-0.54 A and reproduce the measured
current; m6 trades kt (0.40) against current_limit (0.75) with a quadratic term at its bound.
Caveats: current_limit reflects the bench current-loop configuration (~0.45 A; robot presets
reach ~0.9 A); the 0.171 kg block stalls the actuator, so its MAE (~55) is a stall-angle error.

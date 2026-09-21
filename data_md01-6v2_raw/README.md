# data_md01-6v2_raw — campaign 2, servo 6 (MD01 inner coaxial shaft), 2026-09-21

Raw logs of `bam.mangdang.record` (branch `md01-campaign2`), one sample per
USB round trip (4.6 ms mean, 0.26 ms std over 228k intervals), feedback
stamped with the previous transaction (`sample_scheme = set_pos_reply_lag1`).
Bench supply 12.0 V, >= 3 A. AT32 gains written and verified on every run:
kp_position swept, kd_position 0, kp_current 6e-4, kff_current 3e-4, max_pwm
0.99 (the flash values), frame cap 1500 mA (the robot's). Every log carries
them as `at32_*`, plus `zero_dd`, `cur_cap_ma`, `supply_*`, `temp_start_c`
and the housing temperature per sample (`temp`).

| Block | Arm (bare mass) | Payload | `zero_dd` | Logs |
|---|---|---|---|---|
| A | 0.15 m (19.62 g) | none | 1577 | 25 |
| B | 0.15 m | 78.99 g | 1577 | 25 |
| C | 0.15 m | 160.01 g | 1577 | 25 |
| D | 0.15 m | 238.24 g | 1577 | 25 (first 5 = pipeline block) |
| E | 0.10 m (15.95 g) | 120.75 g | 1533 | 25 |
| F | 0.10 m | 238.26 g | 1533 | 25 |
| G | 0.10 m | 303.46 g | 1533 | 25 |

Per block: kp 40, 60, 80, 120, 160 x lift_and_drop, up_and_down,
sin_time_square, sin_sin, steps. `length` = arm length = axis to payload
mass centre (payload stack centred on the arm's end hole). Housing
temperature ran 35-65 C over the session (heaviest blocks D and G).

Extras (`extra_*`, block E payload, kept out of the fit set): `repeat` (kp 80,
noise floor), `kd800` (kp 80, kd_position 800), `strong` (kp 120, kp_current
6e-5, kff_current 1.1e-3).

Also here: `at32_dump_before.txt` (flash configuration before any write),
`at32_stall_test.csv` (blocked-output loop test, four configurations),
`zero.json` (0.15 m arm mount) and `zero_arm10/zero.json` (0.10 m arm).
Processed with `bam.process --dt 0.005` into `data_md01-6v2/` and
`data_md01-6v2_extra/`.

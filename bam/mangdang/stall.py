# Copyright 2026 Mangdang

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""Blocked-output test of the AT32 current loop.

With the arm resting against a rigid stop, the position loop is given a goal
a few degrees beyond the stop. The error is then constant, the motor stalls
against the stop, and the live values show what the inner loop delivers
against its setpoint: ``setpoint_cur_mA = f(kp_position * error)``,
``present_cur_mA`` and ``pwm_duty`` at zero speed. From those, per level:
``R ~ duty * V / I`` and the loop law
``duty ~ kp_current * (setpoint - present) + kff_current * setpoint``.

    uv run python -m bam.mangdang.stall --id 6 --direction +1 \
        --config normal --out data_raw/at32_stall_test.csv

``--config`` selects the current-loop gains written before the run (RAM only;
the flash values are restored afterwards unless ``--no-restore``):
``normal`` = the flash configuration (kp_c 6e-4, kff 3e-4, max_pwm 0.99),
``kff0`` = the same with kff_current 0, ``strong`` = the robot's Strong preset
(6e-5 / 1.1e-3 / 0.99). Holds are short and separated by idle periods.
"""

import argparse
import csv
import os
import time

import numpy as np

from bam.mangdang.spi import LIVE, PARAM, SpiMd01IO, pick_port

CONFIGS = {
    "normal": {"kp_current": 6e-4, "kff_current": 3e-4, "max_pwm_duty_cycle": 0.99},
    "kff0": {"kp_current": 6e-4, "kff_current": 0.0, "max_pwm_duty_cycle": 0.99},
    "strong": {"kp_current": 6e-5, "kff_current": 1.1e-3, "max_pwm_duty_cycle": 0.99},
}

parser = argparse.ArgumentParser(description="Blocked-output current-loop test")
parser.add_argument("--id", type=int, required=True)
parser.add_argument("--port", type=str, default=None)
parser.add_argument("--kp", type=float, default=80.0)
parser.add_argument("--cur", type=int, default=1500, help="frame current cap [mA]")
parser.add_argument("--errors", type=str, default="2.5,5,7.5,10,15,20", help="deg beyond the stop")
parser.add_argument("--direction", type=int, choices=(-1, 1), required=True, help="+1 pushes toward increasing dd")
parser.add_argument("--hold", type=float, default=3.0, help="seconds per level")
parser.add_argument("--rest", type=float, default=3.0, help="idle seconds between levels")
parser.add_argument("--config", type=str, choices=sorted(CONFIGS), default="normal")
parser.add_argument("--label", type=str, default="", help="free text stored with every row (supply used)")
parser.add_argument("--out", type=str, required=True, help="CSV, appended to")
parser.add_argument("--no-restore", action="store_true")
parser.add_argument("--max-ma", type=int, default=1600, help="abort if the frame current exceeds this")
args = parser.parse_args()

FIELDS = [
    "config", "label", "t", "level_deg", "setpoint_pos_deg", "present_pos_deg", "error_pos_deg",
    "max_current_mA", "setpoint_cur_mA", "present_cur_mA", "error_cur_mA", "pwm_duty",
    "frame_cur_mA", "temp_c", "kp_position", "kp_current", "kff_current", "max_pwm_duty_cycle",
]


def read_live(io, servo, names):
    return {name: float(io.get_live(servo, LIVE[name])["val"]) for name in names}


def main() -> None:
    io = SpiMd01IO(pick_port(args.port), cur_ma=args.cur)
    servo = args.id
    io.sync_scale_from_board(servo)
    flash = io.dump_params(servo)
    gains = CONFIGS[args.config]
    io.set_P_coefficient({servo: args.kp})
    io.set_D_coefficient({servo: 0.0})
    for name, value in gains.items():
        io.set_param(servo, PARAM[name], value)
    active = io.dump_params(servo)
    for name, value in gains.items():
        assert abs(active[name] - value) < 1e-6 * max(1.0, value), (name, active[name], value)
    print(f"config {args.config}: kp {active['kp_position']} kd {active['kd_position']} "
          f"kp_c {active['kp_current']:.2e} kff {active['kff_current']:.2e} max_pwm {active['max_pwm_duty_cycle']}")

    exists = os.path.exists(args.out)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    out = open(args.out, "a", newline="")
    writer = csv.DictWriter(out, fieldnames=FIELDS)
    if not exists:
        writer.writeheader()

    stop_dd = float(np.mean([io.ping(servo)["pos_dd"] for _ in range(10)]))
    print(f"arm on the stop at {stop_dd:.0f} dd ({io.dd_to_deg(stop_dd):+.1f} deg from centre)")
    summary = []
    t0 = time.monotonic()
    try:
        for level in [float(x) for x in args.errors.split(",")]:
            goal_dd = int(round(stop_dd + args.direction * level * 10.0))
            rows = []
            t_start = time.monotonic()
            while time.monotonic() - t_start < args.hold:
                r = io.set_pos_dd(servo, goal_dd)
                if abs(r["cur_mA"]) > args.max_ma:
                    raise RuntimeError(f"frame current {r['cur_mA']} mA above --max-ma; aborting")
                live = read_live(io, servo, [
                    "setpoint_pos_deg", "present_pos_deg", "error_pos_deg", "max_current_mA",
                    "setpoint_cur_mA", "present_cur_mA", "error_cur_mA", "pwm_duty",
                ])
                row = {
                    "config": args.config, "label": args.label, "t": round(time.monotonic() - t0, 3),
                    "level_deg": level, **{k: round(v, 3) for k, v in live.items()},
                    "frame_cur_mA": r["cur_mA"], "temp_c": io.last_res / 10.0,
                    "kp_position": active["kp_position"], "kp_current": active["kp_current"],
                    "kff_current": active["kff_current"], "max_pwm_duty_cycle": active["max_pwm_duty_cycle"],
                }
                writer.writerow(row)
                rows.append(row)
                time.sleep(0.05)
            io.set_idle(servo)
            tail = [r for r in rows if r["t"] - rows[0]["t"] > args.hold * 0.5]
            m = {k: float(np.mean([r[k] for r in tail])) for k in
                 ("error_pos_deg", "setpoint_cur_mA", "present_cur_mA", "pwm_duty", "frame_cur_mA", "temp_c")}
            moved = io.dd_to_deg(np.mean([r["present_pos_deg"] * 10 for r in tail])) - io.dd_to_deg(stop_dd)
            summary.append((level, m, moved))
            print(f"  level {level:5.1f} deg: err {m['error_pos_deg']:6.2f} deg  setpoint {m['setpoint_cur_mA']:6.0f} mA  "
                  f"present {m['present_cur_mA']:6.0f} mA (frame {m['frame_cur_mA']:6.0f})  duty {m['pwm_duty']:+.3f}  "
                  f"T {m['temp_c']:.1f} C  arm moved {moved:+.2f} deg")
            time.sleep(args.rest)
    finally:
        io.set_idle(servo)
        out.close()
        if not args.no_restore:
            for name in gains:
                io.set_param(servo, PARAM[name], flash[name])
            io.set_D_coefficient({servo: flash["kd_position"]})
            io.set_P_coefficient({servo: flash["kp_position"]})
            print("flash gains restored in RAM")
        io.close()
    print(f"{len(summary)} levels -> {args.out}")


if __name__ == "__main__":
    main()

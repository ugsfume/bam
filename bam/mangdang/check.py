# Copyright 2026 Mangdang

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""Bench-side sanity table for raw MD01 logs, so a bad capture is caught now.

    uv run python -m bam.mangdang.check --logdir data_raw [--i-stall 850]

One line per log: samples, median / max period, max |current|, the fraction
of samples above 0.8 x the stall current, max |position error|, the position
range in raw deci-degrees (with the distance to the pot ends), the rest
position after a torque-off drop, the pre-roll hold error, the temperature,
and whether the AT32 configuration matches the first log's.
"""

import argparse
import glob
import json
import os

import numpy as np

parser = argparse.ArgumentParser(description="Sanity-check raw MD01 logs")
parser.add_argument("--logdir", type=str, required=True)
parser.add_argument("--i-stall", type=float, default=850.0, help="stall current [mA] for the saturation column")
parser.add_argument("--end-margin", type=float, default=100.0, help="dd of margin to the pot ends to flag")
args = parser.parse_args()

CONFIG_KEYS = [
    "at32_kp_current", "at32_kff_current", "at32_max_pwm_duty_cycle", "at32_kd_position",
    "cur_cap_ma", "zero_dd", "supply_v", "supply_ilim_a",
]


def main() -> None:
    files = sorted(glob.glob(os.path.join(args.logdir, "*.json")))
    files = [f for f in files if os.path.basename(f) != "zero.json"]
    if not files:
        raise SystemExit(f"no logs in {args.logdir}")
    reference = None
    print(
        f"{'file':22s} {'traj':16s} {'kp':>4s} {'m':>6s} {'n':>5s} {'dt50':>5s} {'dtmax':>6s} "
        f"{'|I|max':>6s} {'sat%':>4s} {'|err|':>5s} {'dd min..max':>12s} {'edge':>4s} "
        f"{'rest':>6s} {'hold':>6s} {'T':>5s} cfg"
    )
    for f in files:
        d = json.load(open(f))
        e = d["entries"]
        t = np.array([x["timestamp"] for x in e])
        q = np.array([x["position"] for x in e])
        goal = np.array([x["goal_position"] for x in e])
        en = np.array([x["torque_enable"] for x in e], dtype=bool)
        cur = np.abs(np.array([x["load"] for x in e]))
        temp = np.array([x.get("temp", 0.0) for x in e])
        dt = np.diff(t) * 1e3
        zero_dd = d.get("zero_dd", d.get("scale_dd", 3100.0) / 2.0)
        scale = d.get("scale_dd", 3100.0)
        dd = q / (0.1 * np.pi / 180.0) + zero_dd
        edge = min(dd.min(), scale - dd.max())
        err = np.abs(goal[en] - q[en]).max() if en.any() else 0.0
        sat = 100.0 * np.mean(cur[en] > 0.8 * args.i_stall) if en.any() else 0.0
        rest = np.mean(q[-len(q) // 6:]) if (~en).any() else float("nan")
        cfg = {k: d.get(k) for k in CONFIG_KEYS}
        if reference is None:
            reference = cfg
            cfg_flag = "ref"
        else:
            diff = [k for k in CONFIG_KEYS if cfg[k] != reference[k]]
            cfg_flag = "ok" if not diff else "DIFF:" + ",".join(k.replace("at32_", "") for k in diff)
        flags = []
        if np.median(dt) > 5.5:
            flags.append("slow")
        if dt.max() > 20:
            flags.append("gap")
        if edge < args.end_margin:
            flags.append("EDGE")
        if sat > 30:
            flags.append("STALL")
        if abs(d.get("q_offset", 0.0)) > 0.02:
            flags.append("hold")
        print(
            f"{os.path.basename(f)[:22]:22s} {d['trajectory'][:16]:16s} {d['kp']:4.0f} {d['mass']:6.3f} "
            f"{len(e):5d} {np.median(dt):5.2f} {dt.max():6.1f} {cur.max():6.0f} {sat:4.0f} "
            f"{err:5.2f} {dd.min():5.0f}..{dd.max():5.0f} {edge:4.0f} "
            f"{rest:+6.3f} {d.get('q_offset', 0.0):+6.3f} {temp.max():5.1f} {cfg_flag} {' '.join(flags)}"
        )


if __name__ == "__main__":
    main()

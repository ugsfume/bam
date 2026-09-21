#!/usr/bin/env python
"""Evaluate a directory of fitted BAM params on a log directory.

For every ``m*.json`` in ``--fits``: print the parameters with a flag when a
value sits within 2 % of its optimisation bound, then the position MAE [mrad]
split into train / validation (``--validation_kp``) and per (mass, length,
trajectory) block, using the reference simulator or, with ``--mujoco``, the
MuJoCo CPU backend.

``--max-current`` overrides ``MD01Actuator.max_current`` so a params file can
be evaluated with the class it was fitted with (variant A used 1.4 A).

    .venv/bin/python fits/eval_fits.py --fits fits/md01_B --logdir data_md01-3
    .venv/bin/python fits/eval_fits.py --fits fits/md01_A --logdir data_md01-3 --max-current 1.4
"""

import argparse
import collections
import json
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

parser = argparse.ArgumentParser()
parser.add_argument("--fits", required=True)
parser.add_argument("--logdir", required=True)
parser.add_argument("--validation_kp", type=float, default=140.0)
parser.add_argument("--max-current", type=float, default=None)
parser.add_argument("--mujoco", action="store_true")
parser.add_argument("--json", default=None, help="write the summary here")
parser.add_argument("--extra", action="append", default=[], help="extra log directories evaluated separately (repeatable)")
parser.add_argument("--current", action="store_true", help="also report the simulated-vs-measured current MAE [mA]")
parser.add_argument(
    "--register",
    default=None,
    help="python file to import first (registers extra actuators, e.g. a scratch class)",
)
args = parser.parse_args()

if args.register:
    import importlib.util

    spec = importlib.util.spec_from_file_location("extra_actuators", args.register)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

if args.max_current is not None:
    import bam.mangdang.actuator as md01

    _init = md01.MD01Actuator.__init__

    def _patched(self, testbench_class):
        _init(self, testbench_class)
        self.max_current = args.max_current

    md01.MD01Actuator.__init__ = _patched

from bam import simulate
from bam.actuators import actuators
from bam.logs import Logs
from bam.model import load_model, models

if args.mujoco:
    from bam import mujoco as mujoco_backend

logs = Logs(args.logdir)


def rollout(model, log):
    """Position MAE [mrad] and, with --current, the current MAE [mA] while torque is on."""
    if args.mujoco:
        sim = mujoco_backend.Simulator(model, command_delay=True)
        positions, _, controls = sim.rollout_log(log)
    else:
        sim = simulate.Simulator(model)
        positions, _, controls = sim.rollout_log(log, simulate_control=True)
    ref = np.array([e["position"] for e in log["entries"]])
    n = min(len(ref), len(positions))
    pmae = float(np.mean(np.abs(np.array(positions[:n]) - ref[:n]))) * 1000.0
    if not args.current:
        return pmae, None
    en = np.array([e["torque_enable"] for e in log["entries"]])[:n]
    meas = np.array([e["load"] for e in log["entries"]])[:n] / 1000.0
    ctrl = np.array([c if c is not None else 0.0 for c in controls[:n]], dtype=float)
    if model.actuator.control_unit() != "amps":
        dq = np.gradient(ref[:n], log["dt"])
        ctrl = (ctrl * log["vin"] - model.kt.value * dq) / model.R.value
    sign = np.sign(np.sum(meas[en] * ctrl[en])) or 1.0
    return pmae, float(np.mean(np.abs(sign * ctrl[en] - meas[en]))) * 1000.0


summary = {}
for params_file in sorted(Path(args.fits).glob("m*.json")):
    data = json.load(open(params_file))
    if not data:
        print(f"== {params_file.name}: empty (fit not started?)")
        continue
    model = load_model(str(params_file))

    # Bounds from a freshly built model of the same variant.
    fresh = models[data["model"]]()
    fresh.set_actuator(actuators[data["actuator"]]())
    bounds = {k: (p.min, p.max) for k, p in fresh.get_parameters().items()}

    print(f"== {params_file.name}  ({data['model']}, actuator {data['actuator']}, "
          f"max_current={getattr(model.actuator, 'max_current', None)})")
    for key, value in data.items():
        if key in bounds:
            lo, hi = bounds[key]
            near = "  <-- at bound" if (value - lo) < 0.02 * (hi - lo) or (hi - value) < 0.02 * (hi - lo) else ""
            print(f"   {key:32s} {value:12.5g}   [{lo:g}, {hi:g}]{near}")

    per_block = collections.defaultdict(list)
    train, val, cur = [], [], []
    for log in logs.logs:
        mae, imae = rollout(model, log)
        per_block[(log["mass"], log["length"], log["trajectory"])].append(mae)
        (val if float(log["kp"]) == args.validation_kp else train).append(mae)
        if imae is not None and log["mass"] > 0:
            cur.append(imae)
    print(f"   MAE train {np.mean(train):6.1f} mrad   validation(kp={args.validation_kp:g}) "
          f"{np.mean(val):6.1f} mrad   all {np.mean(train + val):6.1f} mrad"
          + (f"   current MAE {np.mean(cur):5.0f} mA (loaded logs)" if cur else ""))
    extras = {}
    for extra_dir in args.extra:
        e_pos, e_cur = [], []
        for log in Logs(extra_dir).logs:
            mae, imae = rollout(model, log)
            e_pos.append(mae)
            if imae is not None:
                e_cur.append(imae)
        extras[os.path.basename(extra_dir.rstrip("/"))] = float(np.mean(e_pos))
        print(f"   extra {os.path.basename(extra_dir.rstrip('/')):8s} {np.mean(e_pos):6.1f} mrad"
              + (f"   current {np.mean(e_cur):5.0f} mA" if e_cur else ""))
    blocks = collections.defaultdict(list)
    for (mass, length, traj), v in sorted(per_block.items()):
        blocks[(mass, length)] += v
        print(f"     m={mass:.3f} l={length:.3f} {traj:16s} {np.mean(v):6.1f}")
    for (mass, length), v in sorted(blocks.items()):
        print(f"     m={mass:.3f} l={length:.3f} {'(block)':16s} {np.mean(v):6.1f}")
    summary[params_file.stem] = {
        "train": float(np.mean(train)),
        "validation": float(np.mean(val)),
        "all": float(np.mean(train + val)),
        "blocks": {f"m{m:.3f}_l{l:.3f}": float(np.mean(v)) for (m, l), v in blocks.items()},
        "current_mae_mA": float(np.mean(cur)) if cur else None,
        "extras": extras,
        "params": {k: v for k, v in data.items() if k in bounds},
    }

if args.json:
    json.dump(summary, open(args.json, "w"), indent=2)
    print(f"wrote {args.json}")

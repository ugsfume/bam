"""Held-out-set overview: one panel per (block, trajectory), position only.

    .venv/bin/python bam/mangdang/tools/plot_validation.py --logdir data_md01-6v2 --kp 120 \
        --params bam/params/md01/m1.json bam/params/md01/m3.json bam/params/md01/m6.json --out fits/figures/validation_kp120.png

Goal, measured and each model's rollout (reference simulator, control
recomputed from the model) are overlaid; the panel title carries each model's
MAE so the picture and the number agree.
"""
import argparse, json, sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from bam import simulate
from bam.logs import Logs
from bam.model import load_model

ap = argparse.ArgumentParser()
ap.add_argument("--logdir", required=True)
ap.add_argument("--kp", type=float, required=True)
ap.add_argument("--validation_kp", type=float, default=120.0, help="the held-out gain, for the title")
ap.add_argument("--params", nargs="+", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--trajectories", default="lift_and_drop,up_and_down,sin_time_square,sin_sin,steps")
args = ap.parse_args()

MEASURED, GOAL = "#2a78d6", "#555555"
SERIES = ["#eb6834", "#1baf7a", "#4a3aa7"]  # fixed slots: 1st, 2nd, 3rd params file
models = [(Path(p).stem, load_model(p)) for p in args.params]
trajs = args.trajectories.split(",")
logs = [l for l in Logs(args.logdir).logs if float(l["kp"]) == args.kp]
blocks = sorted({(l["mass"], l["length"]) for l in logs}, key=lambda b: (-b[1], b[0]))
fig, axes = plt.subplots(len(blocks), len(trajs), figsize=(3.4 * len(trajs), 2.1 * len(blocks)), sharex=True)
for i, (mass, length) in enumerate(blocks):
    tau = (mass * length + 0.0196 * length / 2) * 9.81 if length > 0.12 else (mass * length + 0.016 * length / 2) * 9.81
    for j, tr in enumerate(trajs):
        ax = axes[i, j]
        log = next((l for l in logs if l["mass"] == mass and l["length"] == length and l["trajectory"] == tr), None)
        if log is None:
            ax.set_visible(False); continue
        t = np.array([e["timestamp"] for e in log["entries"]])
        q = np.array([e["position"] for e in log["entries"]])
        g = np.array([e["goal_position"] for e in log["entries"]])
        en = np.array([e["torque_enable"] for e in log["entries"]])
        g_plot = np.where(en, g, np.nan)
        ax.plot(t, g_plot, color=GOAL, lw=1.0, ls="--", label="goal")
        ax.plot(t, q, color=MEASURED, lw=2.0, label="measured")
        maes = []
        for k, (name, model) in enumerate(models):
            sim = simulate.Simulator(model)
            pos, _, _ = sim.rollout_log(log, simulate_control=True)
            pos = np.array(pos[: len(q)])
            maes.append(np.mean(np.abs(pos - q)) * 1e3)
            ax.plot(t[: len(pos)], pos, color=SERIES[k], lw=1.4, label=name)
        ax.set_title(f"{tr.replace('_', ' ')}   " + "  ".join(f"{n} {m:.0f}" for (n, _), m in zip(models, maes)) + " mrad",
                     fontsize=8, loc="left")
        ax.grid(alpha=0.25); ax.tick_params(labelsize=7)
        if j == 0:
            ax.set_ylabel(f"{mass * 1000:.0f} g @ {length * 100:.0f} cm\n{tau:.2f} Nm\nangle [rad]", fontsize=8)
        if i == len(blocks) - 1:
            ax.set_xlabel("time [s]", fontsize=8)
handles, labels = axes[0, 0].get_legend_handles_labels()
fig.legend(handles, labels, loc="upper right", ncol=len(labels), fontsize=9, frameon=False)
label = "held-out set, never seen by the fit" if args.kp == args.validation_kp else "training set"
fig.suptitle(f"MD01 servo 6 — kp {args.kp:g} ({label}): measured vs model rollouts", fontsize=11, x=0.01, ha="left")
fig.tight_layout(rect=(0, 0, 1, 0.97))
Path(args.out).parent.mkdir(parents=True, exist_ok=True)
fig.savefig(args.out, dpi=130)
print(f"saved {args.out} ({len(logs)} logs)")

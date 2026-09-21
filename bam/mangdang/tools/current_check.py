"""Position MAE and simulated-vs-measured current per log for one params file.

    .venv/bin/python fits/current_check.py --params fits/pipe/md01i_m1.json --logdir data_md01-6v2_pipe
"""
import argparse, json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from bam import simulate
from bam.logs import Logs
from bam.model import load_model

ap = argparse.ArgumentParser()
ap.add_argument("--params", required=True); ap.add_argument("--logdir", required=True)
args = ap.parse_args()
model = load_model(args.params)
unit = model.actuator.control_unit()
print(f"{Path(args.params).name}: control unit {unit}")
print(f"{'traj':16s} {'kp':>4s} {'m':>6s} {'posMAE':>7s} {'|I|meas':>8s} {'|I|sim':>7s} {'I MAE':>6s} {'corr':>5s}")
tot_p, tot_i = [], []
for log in sorted(Logs(args.logdir).logs, key=lambda l: (l["mass"], l["kp"], l["trajectory"])):
    sim = simulate.Simulator(model)
    pos, _, ctrl = sim.rollout_log(log, simulate_control=True)
    q = np.array([e["position"] for e in log["entries"]]); en = np.array([e["torque_enable"] for e in log["entries"]])
    meas = np.array([e["load"] for e in log["entries"]]) / 1000.0
    ctrl = np.array([c if c is not None else 0.0 for c in ctrl], dtype=float)
    if unit != "amps":  # voltage law: duty -> current through the motor model
        kt, R = model.kt.value, model.R.value
        dq = np.gradient(q, log["dt"]); ctrl = (ctrl * log["vin"] - kt * dq) / R
    n = min(len(q), len(pos)); q, pos, meas, ctrl, en = q[:n], pos[:n], meas[:n], ctrl[:n], en[:n]
    sign = np.sign(np.sum(meas[en] * ctrl[en])) or 1.0
    pmae = np.mean(np.abs(pos - q)) * 1e3
    imae = np.mean(np.abs(sign * ctrl[en] - meas[en])) * 1e3
    corr = np.corrcoef(sign * ctrl[en], meas[en])[0, 1] if en.sum() > 2 else float("nan")
    tot_p.append(pmae); tot_i.append(imae)
    print(f"{log['trajectory']:16s} {log['kp']:4.0f} {log['mass']:6.3f} {pmae:7.1f} {np.mean(np.abs(meas[en]))*1e3:8.0f} {np.mean(np.abs(ctrl[en]))*1e3:7.0f} {imae:6.0f} {corr:5.2f}")
print(f"mean position MAE {np.mean(tot_p):.1f} mrad, mean current MAE {np.mean(tot_i):.0f} mA (sign {'flipped' if sign < 0 else 'same'})")

"""Offline checks of the MD01 recorder (dry-run) and the SPI driver's zero handling."""

import glob
import json
import subprocess
import sys

import numpy as np

from bam.mangdang.spi import PARAM, SpiMd01IO


def test_zero_dd_maps_zero_rad_to_the_measured_vertical():
    io = SpiMd01IO(None, dry_run=True, zero_dd=1540)
    assert io.rad_to_dd(0.0) == 1540
    assert abs(io.dd_to_rad(1540)) < 1e-12
    io.zero_dd = None
    assert io.rad_to_dd(0.0) == 1550


def test_set_param_round_trips_in_dry_run():
    io = SpiMd01IO(None, dry_run=True)
    assert abs(io.set_param(6, PARAM["kff_current"], 3e-4) - 3e-4) < 1e-9
    assert abs(io.get_param(6, PARAM["kff_current"])["val"] - 3e-4) < 1e-9


def test_dry_run_log_carries_the_configuration(tmp_path):
    cmd = [
        sys.executable, "-m", "bam.mangdang.record", "--dry-run", "--id", "6",
        "--mass", "0.17", "--arm-mass", "0.02", "--length", "0.15", "--kp", "80",
        "--trajectory", "lift_and_drop", "--logdir", str(tmp_path),
        "--kp-current", "6e-4", "--kff-current", "3e-4", "--max-pwm-duty-cycle", "0.99",
        "--zero-dd", "1540", "--supply-v", "12", "--supply-ilim", "3",
    ]
    subprocess.run(cmd, check=True, capture_output=True)
    log = json.load(open(glob.glob(str(tmp_path / "*.json"))[0]))
    for key, value in {
        "at32_kp_position": 80.0, "at32_kd_position": 0.0, "at32_kp_current": 6e-4,
        "at32_kff_current": 3e-4, "at32_max_pwm_duty_cycle": 0.99, "cur_cap_ma": 1500,
        "zero_dd": 1540.0, "supply_v": 12.0, "supply_ilim_a": 3.0,
        "sample_scheme": "set_pos_reply", "servo_id": 6,
    }.items():
        assert log[key] == value, key
    entries = log["entries"]
    assert {"position", "speed", "load", "goal_position", "torque_enable", "timestamp", "temp"} <= set(entries[0])
    enable = np.array([e["torque_enable"] for e in entries])
    assert enable[0] and not enable[-1]  # lift, then the drop
    t = np.array([e["timestamp"] for e in entries])
    assert np.all(np.diff(t) > 0)


def test_recorder_refuses_a_zero_near_the_pot_end(tmp_path):
    cmd = [
        sys.executable, "-m", "bam.mangdang.record", "--dry-run", "--id", "6",
        "--mass", "0.17", "--length", "0.15", "--logdir", str(tmp_path), "--zero-dd", "1100",
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    assert result.returncode != 0 and "re-mount" in result.stderr

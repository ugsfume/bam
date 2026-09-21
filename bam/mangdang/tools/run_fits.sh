#!/bin/bash
# Usage: fits/run_fits.sh <tag> <logdir> <actuator> <validation_kp> [trials]
# Fits M1..M6 in parallel (one process each) into fits/<tag>/, logs in fits/logs/.
set -e
cd "$(dirname "$0")/../../.."
tag=$1; logdir=$2; actuator=$3; val_kp=$4; trials=${5:-100000}
[ -n "$val_kp" ] || { echo "usage: $0 <tag> <logdir> <actuator> <validation_kp> [trials]"; exit 1; }
mkdir -p fits/$tag fits/logs
for m in m1 m2 m3 m4 m5 m6; do
  OMP_NUM_THREADS=1 WANDB_MODE=disabled env -u PYTHONPATH nohup .venv/bin/python -m bam.fit \
    --actuator $actuator --model $m --logdir $logdir --validation_kp $val_kp \
    --trials $trials --output fits/$tag/$m.json > fits/logs/${tag}_$m.log 2>&1 &
done
echo "launched 6 fits for $tag ($actuator on $logdir, kp $val_kp held out, $trials trials each)"

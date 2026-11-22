"""Run with PYTHONPATH pointing at the current or original simulator checkout."""
import argparse
import csv
from pathlib import Path
import platform
import sys

import mujoco
from simulacrum import params, spin_fit

root = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser()
parser.add_argument('--model-label', required=True)
args = parser.parse_args()
v1 = params.values(params.load(model='vector_1'))
v2 = params.values(params.load(model='vector_2'))
fit = spin_fit.fit(v1, path=root/'evidence/hw-rec-20261118/run-e-spins.jsonl')
out = csv.writer(sys.stdout, lineterminator='\n')
out.writerow(['model','platform','mujoco','unit1_crossing_mm','inset_mm','spin','yaw_ratio','pivot_x_mm',
              'roll_radps','pitch_radps'])
for inset in (3.1, 3.242031550898073, 3.75):
    for r in spin_fit.compare(root/'evidence/camera-spins-unit3-20261120/robot.jsonl',
                              dict(v2, tread_contact_inset_mm=inset)):
        m = r['model']
        out.writerow([args.model_label, platform.machine(), mujoco.__version__, fit['inset_mm'], inset,
                      r['label'], r['ratio'], m['centre_x_mm'], m['roll'], m['pitch']])

"""Replay the camera-registered pre-contact approaches through the browser sensor."""
import csv
import json
from pathlib import Path
import subprocess
import sys

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
root = here.parents[1]
sys.path.insert(0, str(here.parent/'unit3-cube-contact-replay'))
from replay import K, rotation, start_pose

cal = json.loads((root/'web/camera_calibration_vector2.json').read_text())
K[:2] = [[cal['fx_px'], 0, cal['cx_px']], [0, cal['fy_px'], cal['cy_px']]]
picks = json.loads((here.parent/'unit3-paired-cube-push/endpoint-picks.json').read_text())
states = np.genfromtxt(here.parent/'unit3-cube-contact-replay/inputs.csv', delimiter=',', names=True)
headings = np.genfromtxt(here.parent/'unit3-printed-fork-contact/overhead-yaw.csv', delimiter=',', names=True)
model = json.loads((root/'web/public/robot/vector2/model.json').read_text())
period = model['values']['state_period_ms']/1000
runs, recordings = [], []
for trial in range(1, 6):
    p = next(p for p in picks if p['trial'] == trial and p['camera'] == 'on')
    center = rotation(np.deg2rad(p['state']['yaw'])) @ start_pose(p)[0] + p['state']['xy']
    yaw = np.deg2rad(headings[headings['trial'] == trial]['degrees'][0])
    rows = states[(states['trial'] == trial) & (states['sec'] >= 0) & (states['sec'] <= 4.25)]
    poses = []
    path = states[states['trial'] == trial]
    ticks = np.arange(0, rows['sec'].max()+period, period)
    x, y, heading, lift = [np.interp(ticks, path['sec'], path[k])
                           for k in ('x_mm', 'y_mm', 'yaw_deg', 'lift_mm')]
    for cx, cy, h, height in zip(x, y, np.deg2rad(heading), lift):
        xy = rotation(-h) @ (center - [cx, cy])
        poses.append([*xy, yaw-h, height])
    runs.append(dict(trial=trial, poses=poses))
    recordings.append(rows)

pred = json.loads(subprocess.check_output(['node', str(root/'web/tools/cube-range-replay.mjs')],
                                          input=json.dumps(runs), text=True))
fig, axes = plt.subplots(1, 5, figsize=(12, 3.5), sharey=True, layout='constrained')
with (here/'browser-range.csv').open('w') as f:
    writer = csv.writer(f, lineterminator='\n')
    writer.writerow(['trial', 'sec_from_drive', 'physical_mm', 'browser_mm', 'physical_found', 'browser_found', 'lag71_mm'])
    for trial, (rows, predictions, ax) in enumerate(zip(recordings, pred, axes), 1):
        t = rows['sec'] - 1.5
        sim, found = np.array(predictions)[np.floor(rows['sec']/period).astype(int)].T
        # Previously estimated on trials 1–2 (approach.py); no fit on this replay.
        lag_index = np.maximum(0, np.floor((rows['sec']-.071)/period).astype(int))
        delayed = np.array(predictions)[lag_index, 0]
        writer.writerows(zip([trial]*len(t), t, rows['prox_mm'], sim, rows['found'], found, delayed))
        ax.plot(t, np.where(found, sim, np.nan), label='Browser')
        ax.plot(t, np.where(rows['found'], rows['prox_mm'], np.nan), '.', label='Recorded')
        ax.plot(t, delayed, '--', alpha=.7, label='71 ms delay')
        ax.axvspan(1, 2.75, color='k', alpha=.06)
        ax.set(title=f'Trial {trial}', xlabel='Time from drive (s)')
axes[0].set_ylabel('Reported range (mm)')
axes[0].legend()
fig.savefig(here/'browser-range.png', dpi=155)

"""Replay both frozen contact midpoints with nominal and measured steady targets."""
import csv
from dataclasses import replace
from functools import partial
from pathlib import Path
import sys
from unittest.mock import patch

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
root = here.parents[1]
sys.path[:0] = [str(here.parent/'recorded-pose-inset'), str(root)]
import fit_p2
from contact import contact_xml

data = here.parent / 'camera-spins'
with (here / 'repeat-landmark.csv').open() as f:
    survey = next(csv.DictReader(f))
lens_x = float(survey['tip_x_mm'])
observed = np.array([s['centre_x_mm'] for s in fit_p2.opt.turn_centres(
    data/'camera-picks.csv', height=float(survey['tip_height_mm']))['spins']])
observed += lens_x - fit_p2.opt.LENS_X_MM
runs, _, measurements = fit_p2.read_inputs(data)
rows = []
for (segment, start, times, values), obs, center in zip(runs, measurements, observed):
    for mode in ('command', 'measured'):
        seg = segment if mode == 'command' else replace(segment,
            left=np.copysign(obs['wl'], segment.left), right=np.copysign(obs['wr'], segment.right))
        for shift in (0, 1):
            with patch.object(fit_p2, 'contact_xml', partial(contact_xml, forward_shift_mm=shift)), \
                 patch.object(fit_p2.opt, 'LENS_X_MM', lens_x):
                _, sim = fit_p2.simulated(3.75, seg, start, times, dict(values, tread_contact_inset_mm=0))
            rows.append(dict(spin=seg.label, target=mode, forward_mm=shift,
                             wheel_left_error_mmps=sim['wl']-obs['wl'], wheel_right_error_mmps=sim['wr']-obs['wr'],
                             yaw_error_pct=100*(sim['yaw']/obs['yaw']-1), pivot_error_mm=sim['center_mm']-center,
                             roll_error_radps=sim['roll']-obs['roll'], pitch_error_radps=sim['pitch']-obs['pitch']))
fit_p2.dump(here/'shift-speed.csv', rows)
fig, axes = plt.subplots(1, 2, layout='constrained', figsize=(8, 3.6))
for ax, metric, label in zip(axes, ('yaw_error_pct','pivot_error_mm'), ('Yaw RMS error (%)','Pivot RMS error (mm)')):
    for mode in ('command','measured'):
        rms = [np.sqrt(np.mean([r[metric]**2 for r in rows if r['target']==mode and
                               r['forward_mm']==shift and r['spin'].endswith('_2')])) for shift in (0,1)]
        print(mode, metric, rms)
        ax.plot([0,1],rms,'o-',label=mode)
    ax.set(xticks=[0,1], xlabel='Forward contact shift (mm)', ylabel=label)
axes[0].legend(title='Wheel targets')
fig.savefig(here/'shift-speed.png',dpi=160)

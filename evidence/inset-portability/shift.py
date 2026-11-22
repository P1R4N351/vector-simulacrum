"""Shift both sets of auxiliary contacts together, holding the span at 40.5 mm."""
from pathlib import Path
from unittest.mock import patch
import sys
from functools import partial

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
root = here.parents[1]
sys.path[:0] = [str(here.parent/'recorded-pose-inset'), str(root)]
import c7_transfer
import fit_p2
import contact
from simulacrum import params, recording, replay
from contact import contact_xml

runs, _, measurements = fit_p2.read_inputs(here.parent/'camera-spins')
shifts = np.arange(3.5, -3.51, -0.5)
rows = []
for shift in shifts:
    for (seg, start, times, values), physical in zip(runs, measurements):
        with patch.object(fit_p2, 'contact_xml', partial(contact_xml, forward_shift_mm=shift)):
            _, sim = fit_p2.simulated(3.75, seg, start, times, dict(values, tread_contact_inset_mm=0.0))
        rows.append(dict(shift_mm=shift, spin=seg.label, yaw_error_pct=100*(sim['yaw']/physical['yaw']-1),
                         pivot_error_mm=sim['center_mm']-physical['center_mm'],
                         measured_roll=physical['roll'],sim_roll=sim['roll'],
                         measured_pitch=physical['pitch'],sim_pitch=sim['pitch']))
fit_p2.dump(here/'shift.csv',rows)
fig,axes=plt.subplots(1,2,figsize=(9,4),layout='constrained')
for cycle in (1,2):
    for ax,field in zip(axes, ('pivot_error_mm','yaw_error_pct')):
        vals=[np.sqrt(np.mean([r[field]**2 for r in rows if r['shift_mm']==s and r['spin'].endswith(str(cycle))])) for s in shifts]
        ax.plot(-np.array(shifts),vals,'o-',label=f'CW/CCW pair {cycle}')
        print(cycle,field,np.round(vals,2))
for ax,title in zip(axes,('Turn center RMS error (mm)','Chassis yaw RMS error (%)')):
    ax.set(xlabel='Midpoint shift (mm, positive toward rear)',ylabel=title)
    ax.legend()
fig.savefig(here/'shift.png',dpi=160)

# Choose on pair 1, requiring yaw RMS no worse than the unshifted pair.
def pair1_rms(shift, key):
    return np.sqrt(np.mean([r[key]**2 for r in rows if r['shift_mm']==shift and r['spin'].endswith('1')]))

best = min((s for s in shifts if pair1_rms(s,'yaw_error_pct') <= pair1_rms(0,'yaw_error_pct')),
           key=lambda s:pair1_rms(s,'pivot_error_mm'))
values = dict(params.values(params.load(model='vector_1')), tread_contact_inset_mm=0.0)
states, commands = recording.load(here.parent/'hw-rec-20261118/run-e-spins.jsonl')
transfer = []
for seg in recording.segments(commands, values).values():
    if seg.kind != 'wheels' or seg.left * seg.right >= 0:
        continue
    start = replay._state_at(states,seg.t_cmd)
    dur = seg.t_stop-seg.t_cmd
    real = [c7_transfer.gyro_axes(r['head']+np.deg2rad(values['imu_pitch_offset_deg']),r['gyro']) for r in states
            if seg.t_cmd+dur/2 <= r['t'] <= seg.t_stop]
    for shift in (0,best):
        with patch.object(contact,'contact_xml',partial(contact_xml,forward_shift_mm=shift)):
            sim = c7_transfer.score(c7_transfer.simulate(values,start,seg,3.75),dur,'shifted')
        transfer.append(dict(spin=seg.label,forward_shift_mm=shift, **{
            key+'_error_pct':100*(sim['mean_abs_'+key+'_rad_s']/np.mean([abs(r[key+'_rad_s']) for r in real])-1)
            for key in ('yaw','roll','pitch')}))
fit_p2.dump(here/'shift-c7.csv',transfer)

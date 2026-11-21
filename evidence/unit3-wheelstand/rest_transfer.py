"""Held-out pre-wheelstand rest. Preserve packet order and use only earlier data for the gain."""
import csv
from pathlib import Path
import sys
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[1]))
from simulacrum import params
from simulacrum.cube import CubeAccelerometer

def vectors(rows, keys):
    return np.column_stack([rows[k] for k in keys]) / 8192

previous = np.genfromtxt(here.parent/'unit3-cube-accelerometer/resting.csv', delimiter=',',names=True)
gain = np.linalg.norm(vectors(previous, ['x_counts','y_counts','z_counts']).mean(axis=0))
repeat = '--repeat' in sys.argv
suffix = '-repeat' if repeat else ''
held = np.genfromtxt(here/f'stationary{suffix}.csv.gz',delimiter=',',names=True)
assert np.array_equal(held['slot'], np.tile([0,1,2], len(held)//3))
physical = vectors(held, 'xyz')
v = params.values(params.load(model='vector_2'))
dt = v['sim_timestep_ms']/1000
def simulate(scale, seed):
    chip = CubeAccelerometer(dict(v,cube_accel_scale=[scale]*3), dt, seed=seed)
    gravity = np.array([0.,0.,1.])
    chip.reset(gravity,0)
    i = 0
    while len(chip.messages)*3 < len(physical):
        i += 1
        chip.step(i*dt, gravity)
    return np.vstack([np.frombuffer(payload[2:],'<i2').reshape(3,3) for _,payload in chip.messages])/8192

def metrics(values):
    norm = np.linalg.norm(values,axis=1)
    differences = np.diff(values.reshape(-1,3,3),axis=1)
    return [norm.mean(),norm.std(),*np.sqrt(np.mean(differences**2,axis=(0,1)))]

fig, axes = plt.subplots(1,2,figsize=(10,4),layout='constrained')
with (here/f'rest-transfer{suffix}.csv').open('w') as f:
    writer = csv.writer(f,lineterminator='\n')
    writer.writerow(['series','seed','mean_g','sd_g','within_packet_rms_dx_g','within_packet_rms_dy_g','within_packet_rms_dz_g'])
    for label, scale in [('Shelf cube',None),('Frozen model',1.),('Earlier shelf gain',gain)]:
        trials = [physical] if scale is None else [simulate(scale,i) for i in range(16)]
        for i, values in enumerate(trials):
            writer.writerow([label,i if scale else '',*metrics(values)])
        norms=np.sort(np.concatenate([np.linalg.norm(a,axis=1) for a in trials]))
        line,=axes[0].plot(norms,np.arange(1,len(norms)+1)/len(norms),label=label)
        axes[1].plot(list('XYZ'),np.mean([metrics(a)[2:] for a in trials],axis=0)*1000,'o-',c=line.get_color(),label=label)
axes[0].set(xlabel='Specific force magnitude (g)',ylabel='Fraction at or below')
axes[1].set(xlabel='Sensor axis',ylabel='Within-packet RMS change (mg)')
axes[0].legend(fontsize=8)
fig.savefig(here/f'rest-transfer{suffix}.png',dpi=150)
print('prior gain',gain)

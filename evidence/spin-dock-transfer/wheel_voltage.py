"""Unfitted battery-voltage control on the original and repeated spin sessions."""
import csv
import io
import json
from pathlib import Path
import subprocess
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
data = here.parent/'hw-rec-20261119'
scores = []
angles = []
fig, axes = plt.subplots(1,2,figsize=(10,4),layout='constrained',sharey=True)
for filename in ('unit2-repeat-stops.jsonl','unit2-ccw100.jsonl'):
    events = [json.loads(line) for line in (data/filename).read_text().splitlines()]
    states = [e for e in events if e['k']=='s']
    for command in (e for e in events if e.get('name')=='wheels'):
        start = command['t']
        end = next(e['issued_t'] for e in events if e.get('name')=='stop' and e['t']>start)
        prior = next(e for e in reversed(states) if e['t']<start)
        volts = next(e['volts'] for e in reversed(events) if e['k']=='b' and e['t']<start)
        label = command['label']
        base = dict(left=command['left'],right=command['right'],duration=end-start,
                    head=np.degrees(prior['head']),lift=prior['lift'])
        cases = [dict(base,spin='default'),dict(base,spin='measured',volts=volts)]
        sim = np.genfromtxt(io.StringIO(subprocess.check_output(['node',str(here/'commands.mjs')],
                            input=json.dumps(cases),text=True)),delimiter=',',names=True,dtype=None,encoding='utf8')
        t = np.array([e['t']-start for e in states])
        # Check gaps in the steady window independently of the late stop telemetry.
        lower,upper=.55,end-start-.25
        bounding=t[np.searchsorted(t,lower)-1:np.searchsorted(t,upper)+1]
        if np.max(np.diff(bounding))>.18:
            continue
        steady=t[(t>lower)&(t<upper)]
        first,last=-.3,end-start+.65
        boundary=t[np.searchsorted(t,first)-1:np.searchsorted(t,last)+1]
        if np.max(np.diff(boundary))<.18:
            grid=np.r_[first,t[(t>first)&(t<last)],last]
            sign=np.sign(command['right'])
            real=np.degrees([e['gyro'][2] for e in states])
            observed=sign*np.trapezoid(np.interp(grid,t,real),grid)
            for variant in ('default','measured'):
                r=sim[sim['spin']==variant]
                predicted=sign*np.trapezoid(np.interp(grid,r['t'],r['gyro']),grid)
                angles.append((label,variant,observed,predicted,predicted-observed))
        for side,ax in zip(('wl','wr'),axes):
            physical=np.mean(np.abs([e[side] for e in states if lower<e['t']-start<upper]))
            direction=command['left' if side=='wl' else 'right']
            for variant,marker in [('default','o'),('measured','x')]:
                r=sim[sim['spin']==variant]
                speed=np.mean(abs(np.interp(steady,r['t'],r[side])))
                scores.append((label,side,direction,volts,len(steady),physical,variant,speed,speed-physical))
                ax.scatter(label.replace('spin_ccw_100','Prior CCW'),speed-physical,marker=marker,
                           color='#c96329' if variant=='measured' else '#24588b',
                           label=variant if len(ax.collections)<2 else None)
            ax.set_title('Left tread' if side=='wl' else 'Right tread')
for ax in axes:
    ax.axhline(0,color='.5',linewidth=1)
    ax.tick_params(axis='x',labelrotation=25)
axes[0].legend()
axes[0].set_ylabel('Simulated − physical speed magnitude (mm/s)')
fig.savefig(here/'wheel-voltage.png',dpi=165)
with (here/'wheel-voltage.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['spin','side','target_mmps','measured_v','n','physical_mmps','variant','sim_mmps','error_mmps']);w.writerows(scores)

with (here/'voltage-angle.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['spin','variant','physical_deg','sim_deg','error_deg']);w.writerows(angles)

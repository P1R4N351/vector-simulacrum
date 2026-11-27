"""Replay the separate SDK session with host-stamped commands and observed starting pose."""
import csv
import io
import json
from pathlib import Path
import subprocess
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import cumulative_trapezoid

here = Path(__file__).resolve().parent
rows = [json.loads(line) for line in (here.parent/'hw-rec-20261119/unit2-repeat-stops.jsonl').read_text().splitlines()]
states = [r for r in rows if r['k']=='s']
commands = [r for r in rows if r['k']=='c']
trials = []
for start in (r for r in commands if r['name']=='wheels'):
    stop = next(r for r in commands if r['name']=='stop' and r['t']>start['t'])
    prior = [r for r in states if r['t']<=start['t']][-1]
    trials.append(dict(spin=start['label'], left=start['left'], right=start['right'],
                       duration=stop['issued_t']-start['t'], start=start['t'],
                       head=np.degrees(prior['head']), lift=prior['lift']))
sim = np.genfromtxt(io.StringIO(subprocess.check_output(['node',str(here/'commands.mjs')],
                        input=json.dumps(trials),text=True)),names=True,delimiter=',',dtype=None,encoding='utf8')
fig,axes=plt.subplots(2,4,figsize=(14,6),layout='constrained',sharex='col',sharey='row')
scores=[]
for col, trial in enumerate(trials):
    name=trial['spin']
    sign=np.sign(trial['right'])
    ht=np.array([r['t']-trial['start'] for r in states])
    begin,end=-.3,trial['duration']+.65
    t=np.r_[begin,ht[(ht>begin)&(ht<end)],end]
    # Detect missing packets before interpolation could conceal them.
    obs=ht[np.searchsorted(ht,begin)-1:np.searchsorted(ht,end)+1]
    gap=max(np.diff(obs))
    good=gap<.18 and obs[0]<=begin and obs[-1]>=end
    ax,bx=axes[:,col]
    for source in ('Physical SDK','Canonical'):
        if source=='Physical SDK':
            if not good: continue
            raw_t=ht
            wheel=np.array([(r['wr']-r['wl'])/2 for r in states])
            gyro=np.degrees([r['gyro'][2] for r in states])
        else:
            r=sim[sim['spin']==name]
            raw_t=r['t'];wheel=(r['wr']-r['wl'])/2;gyro=r['gyro']
        # Both go through identical receipt-time subsampling for the paired totals.
        if not good: t=raw_t[(raw_t>=begin)&(raw_t<=end)]
        w=sign*np.interp(t,raw_t,wheel); g=sign*np.interp(t,raw_t,gyro)
        dist=cumulative_trapezoid(w,t,initial=0);angle=cumulative_trapezoid(g,t,initial=0)
        scores.append((name,source,trial['duration'],gap,good,dist[-1],angle[-1],np.mean(w[(t>.55)&(t<trial['duration']-.25)]),np.mean(g[(t>.55)&(t<trial['duration']-.25)])))
        ax.plot(t,w,label=source)
        bx.plot(t,angle)
    for a in (ax,bx): a.axvline(trial['duration'],color='0.55',ls=':')
    ax.set_title(name.upper() if good else name.upper()+' (SDK gap)')
    bx.set_xlabel('Seconds from wheel command')
axes[0,0].set_ylabel('Differential speed / 2 (mm/s)')
axes[1,0].set_ylabel('Integrated head gyro (°)')
axes[0,1].legend(fontsize=8)
fig.savefig(here/'command-spins.png',dpi=150)
with (here/'command-scores.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['spin','source','issued_duration_s','max_sdk_gap_s','usable_pair','travel_mm','head_deg','steady_wheel_mmps','steady_head_dps']);w.writerows(scores)

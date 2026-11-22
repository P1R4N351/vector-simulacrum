"""Test whether the model sees the fork moving before the failed-grip stop."""
import csv
import gzip
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from simulacrum import params
from simulacrum.robot import VectorSim

root = Path(__file__).resolve().parent
phase, duration = .14685757398605348, .110
observation_delay = .11552706003188984


def pre_span(times, heights, dispatch, window):
    # Include the last sample at/before the left boundary and all subsequent
    # samples available at dispatch. Never look ahead to detect movement.
    first = np.searchsorted(times, dispatch-window, side='right') - 1
    end = np.searchsorted(times, dispatch, side='right')
    if first < 0 or end-first < 2 or dispatch-times[end-1] > .1 or dispatch-window-times[first] > .1:
        return np.nan
    return np.ptp(heights[first:end])


rows = []
for capture in ('stop-control', 'stop-holdout', 'stop-prospective'):
    with gzip.open(root/f'{capture}.jsonl.gz', 'rt') as f:
        data = [json.loads(line) for line in f]
    states = [r for r in data if r['kind'] == 'state']
    times, heights = [np.array([r[k] for r in states]) for k in ('t','lift')]
    for event in (r for r in data if r.get('action') == 'stop'):
        rows.append([capture,event['n'],*[pre_span(times,heights,event['t'],w) for w in (.1,.15,.2)]])

data = np.genfromtxt(root/'dock-status.csv',names=True,delimiter=',')
t,h = data['seconds'],data['encoder_mm']
bit = (data['status'].astype(int)&256)!=0
dispatch = 1.518  # cleanup stop follows response; exact call timestamp unavailable
def replay(cut_at=None):
    sim = VectorSim(params.values(params.load(model='vector_2')),lift_mm=33.02,head_deg=21.5,charger_contact=True)
    sim.command_lift_height(47,duration_s=1)
    sent = False
    while sim.tick*sim.dt < t[-1]:
        now = sim.tick*sim.dt
        if cut_at is not None and now >= cut_at:
            aid = sim.model.actuator('lift').id
            sim.model.actuator_gainprm[aid,:] = 0
            sim.model.actuator_biasprm[aid,:] = 0
            cut_at = None
        if not sent and now >= dispatch:
            assert sim.stop_all_motors()
            sent = True
        sim.step()
    times, heights, bits = [np.array([getattr(s,k) for s in sim.states]) for k in ('time_s','lift_height_mm','lift_in_pos')]
    return times + observation_delay, heights, bits

st,sh,sb = replay()
# Bound the counterfactual torque cutoff with the measured first return edge.
edge = np.flatnonzero((~bit[:-1]) & bit[1:])[0]
cuts = [(label, *replay(t[i]-observation_delay)[:2]) for label,i in
        [('Early torque cutoff',edge),('Late torque cutoff',edge+1)]]
for label,times,heights in [('failed grip physical',t,h),('failed grip simulated',st,sh), *cuts]:
    rows.append([label,'cleanup',*[pre_span(times,heights,dispatch,w) for w in (.1,.15,.2)]])
with (root/'stop-posture.csv').open('w') as f:
    w=csv.writer(f)
    w.writerow(['recording','event','pre_100ms_span_mm','pre_150ms_span_mm','pre_200ms_span_mm'])
    w.writerows(rows)

baseline = sb[np.maximum(0,np.searchsorted(st,t,side='right')-1)]
pulse = baseline.copy()
after = t>=dispatch
pulse[after] = (t[after]<dispatch+phase)|(t[after]>=dispatch+phase+duration)
fig,axes=plt.subplots(2,1,figsize=(8,5),layout='constrained',sharex=True)
axes[0].plot(t,h,'.-',label='Physical encoder')
axes[0].step(st,sh,where='post',label='Simulated encoder, frozen receipt delay')
for label, times, heights in cuts:
    axes[0].step(times, heights, where='post', label=label)
axes[0].set_ylabel('Lift height (mm)')
for values,offset,label in [(bit,0,'Physical'),(baseline,.035,'Default settling'),(pulse,-.035,'Stationary pulse')]:
    axes[1].step(t,values.astype(float)+offset,where='post',label=label)
axes[1].set(yticks=[0,1],yticklabels=['Clear','Set'],xlabel='Seconds from raise command',xlim=(.95,2.35))
for ax in axes:
    ax.axvspan(dispatch-.1,dispatch,color='grey',alpha=.13)
    ax.axvline(dispatch,color='grey',ls=':')
    ax.legend(fontsize=8)
fig.savefig(root/'stop-posture.png',dpi=160)
for prediction in (baseline,pulse):
    print('after response',sum(prediction[after]==bit[after]),sum(after))

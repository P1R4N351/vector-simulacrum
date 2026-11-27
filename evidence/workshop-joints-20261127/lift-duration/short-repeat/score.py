"""Test previous session's fixed timing advances on twelve new short drops."""
from pathlib import Path
import csv
import matplotlib.pyplot as plt
import numpy as np

folder = Path(__file__).parent
def read(name):
    with (folder / name).open() as f:
        return list(csv.DictReader(f))

raw = read('recording.csv')
states = [r for r in raw if r['event'] == 'state']
sim = read('replay.csv')
commands = [r for r in raw if r['event'] == 'command']
assert len(commands) == 24 and sum(r['event']=='return' and r['code']=='0' for r in raw)==24
times = np.array([float(r['s']) for r in states])
assert np.array_equal(times, [float(r['s']) for r in sim])
assert all(int(r['on_dock']) for r in states+sim)
fig, axes = plt.subplots(1,2,figsize=(9,4),sharey=True,layout='constrained')
out = []
for i, command in enumerate(commands[1::2], 1):
    t = times-float(command['s'])
    use = (t > -.2) & (t < 1.05)
    duration = float(command['duration_s'])
    advance = {0.132:93,0.600:72}[duration]
    ax = axes[int(duration>.2)]
    crossings = []
    for source,color,label in [(states,'tab:blue','Physical'),(sim,'tab:orange','Canonical')]:
        v = np.array([float(r['lift_mm']) for r in source])[use]
        x = t[use]
        start = np.median(v[x<0])
        travel = (start-v)/(start-32)
        ax.plot(x,travel,'.-',color=color,alpha=.6,ms=2,label=label if i in (1,2) else None)
        intervals = []
        for level in (.2,.5,.8):
            k = np.flatnonzero((travel[:-1]<level)&(travel[1:]>=level))
            intervals.append([np.nan]*3 if len(k)==0 else
                             [x[k[0]],np.interp(level,travel[k[0]:k[0]+2],x[k[0]:k[0]+2]),x[k[0]+1]])
        crossings.append(1000*np.array(intervals))
    a,b = crossings
    for j,level in enumerate((20,50,80)):
        lo,mid,hi = b[j,0]-a[j,2],b[j,1]-a[j,1],b[j,2]-a[j,0]
        out.append((i,duration,level,lo,mid,hi,mid-advance))
    ax.set_title(f'44 → 32 mm · {1000*duration:.0f} ms request')
for ax in axes:
    ax.set(xlabel='Seconds after command',xlim=(-.1,.9))
    ax.grid(alpha=.2)
    ax.legend()
axes[0].set_ylabel('Fraction of measured start-to-target travel')
fig.savefig(folder/'comparison.png',dpi=160)
with (folder/'scores.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow('trial duration_s threshold_pct lower_ms error_ms upper_ms shifted_error_ms'.split())
    w.writerows(out)
for dur in (.132,.600):
    for level in (20,50,80):
        a=np.array([r[3:] for r in out if r[1:3]==(dur,level)])
        print(dur,level,'n',np.isfinite(a[:,1]).sum(),'median error, abs, shifted abs',
              np.round([np.nanmedian(a[:,1]),np.nanmedian(abs(a[:,1])),np.nanmedian(abs(a[:,3]))]),
              'interval excludes zero',sum((a[:,0]>0)|(a[:,2]<0)))
print(len(states),'packets; median interval ms',1000*np.median(np.diff(times)))

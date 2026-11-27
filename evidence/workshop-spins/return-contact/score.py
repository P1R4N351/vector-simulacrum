import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parent
fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True, layout='constrained')
summary = []
for label, folder in [('Workshop 1','.'),('Workshop 2','speed-holdout'),('Workshop 3','interleaved'),
                      ('Canonical','sim.csv'),('30 mm/s experiment','candidate.csv')]:
    simulated = folder.endswith('.csv')
    with (root / folder if simulated else root.parent / folder / 'states.csv').open() as f:
        rows = [r for r in csv.DictReader(f) if simulated or r['phase'] == 'return']
    def values(key):
        return np.array([float(r[key]) for r in rows])
    on = values('on').astype(bool) if simulated else (values('flags').astype(int) & 4096) != 0
    edges = np.flatnonzero(on[1:] & ~on[:-1]) + 1
    if not len(edges):
        print(label, 'no captured contact')
        continue
    i = edges[0]
    t = values('time_s') - values('time_s')[i]
    speed = (values('left_mm_s') + values('right_mm_s')) / 2
    pitch = np.rad2deg(values('pitch_rad'))
    travel = np.concatenate(([0], np.cumsum(.5*(speed[1:]+speed[:-1])*np.diff(t))))
    pre = (t >= -.72) & (t < 0)
    print(label,round(np.median(speed[pre]),2),round(pitch[i],2),round(-t[i-1]*1000),round(np.hypot(np.diff(values('x_mm'))[i-1], np.diff(values('y_mm'))[i-1]),2))
    # Compare encoder travel after the last downward crossing of -5 degrees before contact.
    j = np.flatnonzero((pitch[1:i+1] <= -5) & (pitch[:i] > -5))[-1]+1
    fraction = (-5-pitch[j-1])/(pitch[j]-pitch[j-1])
    t5 = t[j-1]+fraction*(t[j]-t[j-1])
    span = np.interp(t5,t,travel)-travel[i]
    summary.append(dict(series=label,pitch_at_contact_deg=pitch[i],median_precontact_wheel_mm_s=np.median(speed[pre]),
                        seconds_from_minus5=-t5,reverse_encoder_travel_mm=span,flag_bracket_ms=-1000*t[i-1]))
    window = (t >= -3.5) & (t <= .24)
    for ax,y in zip(axes,[speed,pitch,travel-travel[i]]):
        ax.plot(t[window],y[window],label=label, lw=2 if simulated else 1.2)
for ax,name in zip(axes,['Mean wheel speed (mm/s)','SDK pitch (degrees)','Integrated wheel travel\nrelative to first flag (mm)']):
    ax.set_ylabel(name)
    ax.axvline(0,lw=.7,color='black',ls=':')
axes[0].legend(ncols=3,fontsize=8)
axes[-1].set_xlabel('Seconds relative to first on-charger report')
fig.savefig(root/'return-contact.png',dpi=160)
with (root/'summary.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=summary[0].keys())
    w.writeheader()
    w.writerows(summary)

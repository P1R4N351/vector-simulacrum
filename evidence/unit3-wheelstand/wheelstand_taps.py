"""Score transmitted counters and test what can be recovered from retained Z samples alone."""
import csv
from pathlib import Path
import sys
import numpy as np
import matplotlib.pyplot as plt

here = Path(__file__).resolve().parent
sys.path.insert(0,str(here.parents[1]))
from simulacrum.cube import TapCounter, TapListener
from simulacrum import params

out = here/'W4'
v = params.values(params.load(model='vector_2'))
steps = round(v['cube_tick_ms']/v['cube_accel_frame_period_ms'])
rows = np.genfromtxt(out/'cube.csv.gz',names=True,delimiter=',')
packets = rows.reshape(-1,3)
assert np.all(packets['slot']==[0,1,2])
assert np.all(packets['tap']==packets['tap'][:,:1])
physical = TapListener()
estimated = {name:TapCounter() for name in ('hold','linear')}
changes = {name:[] for name in estimated}
previous = int(rows['z'][0])
for detector in estimated.values():
    detector.last = previous
for pkt in packets:
    t = pkt['sec'][0]
    xyz = np.column_stack([pkt[k] for k in 'xyz']).astype('<i2')
    physical.receive(t, bytes([2,int(pkt['tap'][0])])+xyz.tobytes())
    for z in xyz[:,2].astype(int):
        for name,detector in estimated.items():
            old = detector.count
            # Each tick retains its last FIFO frame. Both reconstructions end at that same sample.
            for i in range(1,steps+1):
                value = (previous if i<steps else z) if name=='hold' else round(previous+(z-previous)*i/steps)
                detector.feed(value)
            if detector.count!=old:
                changes[name].append(t)
        previous=z
sdk = [float(r['sec']) for r in csv.DictReader((out/'events.csv').open()) if r['event']=='object_tapped']
with (here.parent/'unit3-physical-repeats/tap-receipt-transfer.csv').open() as f:
    delay=float(next(csv.DictReader(f))['prior_delay_s'])
zero=next(float(r['sec']) for r in csv.DictReader((out/'actions.csv').open()) if r['action']=='pop_a_wheelie')
streams={'SDK event':sdk,'Transmitted counter':physical.events,'Counter + earlier delay':[t+delay for t in physical.events],
         'Z only · hold':changes['hold'],'Z only · linear':changes['linear']}
fig,ax=plt.subplots(figsize=(8,3.5),layout='constrained')
with (out/'tap-transfer.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n');w.writerow(['stream','event_count','event_times_s_from_wheelstand'])
    for i,(name,times) in enumerate(streams.items()):
        secs=np.asarray(times)-zero
        ax.scatter(secs,[i]*len(secs),marker='|',s=150)
        w.writerow([name,len(times),' '.join(f'{t:.6f}' for t in secs)])
ax.set(yticks=range(len(streams)),yticklabels=list(streams),xlabel='Seconds after wheelstand command',title='Two reported taps during wheelstand contact')
ax.invert_yaxis()
fig.savefig(out/'tap-transfer.png',dpi=155)
print(streams)
print('SDK minus delayed counter, ms',1000*(np.asarray(sdk)-physical.events-delay))

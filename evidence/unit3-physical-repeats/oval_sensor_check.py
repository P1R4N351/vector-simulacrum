"""Check the frozen Vector 2 cube sensor on independent physical captures."""
import csv
import gzip
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[1]))
sys.path.insert(0, str(here.parent/'unit3-cube-accelerometer'))
from stationary_noise import scatter, simulate, v
from simulacrum.cube import TapListener, MovementListener, UpAxisListener, engine_samples

fig, axes = plt.subplots(2, 2, figsize=(11, 6), layout='constrained')
results = []
for col, (run, lo, hi) in enumerate([('E', 43.4, 46.1), ('F', 17.7, 19.1)]):
    directory = here/run
    tap, move, up = TapListener(), MovementListener(), UpAxisListener()
    still = []
    coverage = []
    with gzip.open(directory/'radio.csv.gz', 'rt') as f:
        for row in csv.DictReader(f):
            t = float(row['receipt_s'])
            p = bytes.fromhex(row['payload_hex'])
            if len(p) != 20 or p[0] != 2:
                continue
            coverage.append(t)
            tap.receive(t, p)
            up.receive(t, p)
            for a in engine_samples(p):
                move.update(t, a)
            if lo <= t <= hi:
                still.append(np.frombuffer(p, '<i2', offset=2).reshape(3,3)/8192)
    still = np.stack(still)
    observed = scatter(still)
    predicted = scatter(simulate(v['cube_accel_noise_density_ug'], np.array([0.,0.,1.]), 13,
                                 word_format=v['cube_accel_word_format']))
    ax = axes[0,col]
    for offset, data, label in [(-.17, observed, 'Physical'), (.17, predicted, 'Frozen model')]:
        ax.bar(np.arange(3)+offset, data, .34, label=label)
    ax.set(xticks=range(3), xticklabels=list('XYZ'), ylabel='Within-packet scatter (mg RMS)',
           title=f'{run} · {len(still)} packets · {lo}–{hi} s')
    ax.legend()
    pairs = [('tap',tap.events,'object_tapped'), ('move',[t for t,on in move.events if on],'object_moved'),
             ('stop',[t for t,on in move.events if not on],'object_stopped_moving'),
             ('axis',[t for t,_ in up.events],'object_up_axis_changed')]
    with (directory/'object-events.csv').open() as f:
        events = list(csv.DictReader(f))
    for i,(kind,modeled,event) in enumerate(pairs):
        sdk = [float(e['receipt_s']) for e in events if e['event']==event]
        for dy, ts, marker, color in [(-.13,modeled,'|','darkorange'),(.13,sdk,'o','teal')]:
            axes[1,col].scatter(ts,[i+dy]*len(ts),marker=marker,color=color)
        results.append([run,kind,len(modeled),len(sdk),' '.join(f'{t:.4f}' for t in modeled),
                        ' '.join(f'{t:.4f}' for t in sdk)])
    axes[1,col].set(yticks=range(4),yticklabels=[p[0] for p in pairs],ylim=(-.6,3.6), xlim=(coverage[0],coverage[-1]),
                    xlabel='Seconds after undock command',title='Packet replay (orange) and SDK (teal)')
    print(run, len(still), observed, predicted)
fig.savefig(here/'oval-sensor-check.png',dpi=150)
with (here/'oval-event-check.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(['run','event','replay_count','sdk_count','replay_seconds','sdk_seconds'])
    w.writerows(results)

# Derive a host-receipt offset from the older pickup, then freeze it for A and E.
with (here.parent/'unit3-cube-accelerometer/event-replay.csv').open() as f:
    prior = list(csv.DictReader(f))
def prior_times(stream):
    return np.array([float(r['seconds_from_pickup']) for r in prior if r['stream']==stream])
delay = np.median(prior_times('SDK tap')-prior_times('Packet tap'))
with (here/'tap-receipt-transfer.csv').open('w') as f:
    writer=csv.writer(f,lineterminator='\n')
    writer.writerow(['run','packet_receipt_s','sdk_receipt_s','prior_delay_s','residual_ms'])
    for run in ('A','E'):
        listener = TapListener()
        with gzip.open(here/run/'radio.csv.gz','rt') as packets:
            for row in csv.DictReader(packets):
                payload=bytes.fromhex(row['payload_hex'])
                if len(payload)==20 and payload[0]==2:
                    listener.receive(float(row['receipt_s']),payload)
        with (here/run/'object-events.csv').open() as observed:
            times=[float(row['receipt_s']) for row in csv.DictReader(observed) if row['event']=='object_tapped']
        print(run, times, listener.events)
        if len(times)!=len(listener.events):
            continue
        for packet,sdk in zip(listener.events,times):
            writer.writerow([run,packet,sdk,round(delay,6),round(1000*(sdk-packet-delay),2)])

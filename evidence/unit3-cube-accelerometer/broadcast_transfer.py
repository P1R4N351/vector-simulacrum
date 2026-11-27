"""Replay recorded tap-counter changes through the physical engine grouping clock."""
import csv
from pathlib import Path
import sys
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
root = next((p for p in here.parents if (p / 'simulacrum').is_dir()), None)
if root:
    sys.path.insert(0, str(root))
from simulacrum.cube import TapBroadcasts

with (here / 'sdk-tap-transfer.csv').open() as f:
    rows = list(csv.DictReader(f))
matches = []
by_run = {}
for index, row in enumerate(rows):
    by_run.setdefault(row['run'], []).append((index, row))

def replay(phase):
    predicted = {}
    for run in by_run.values():
        listener = TapBroadcasts()
        for index, row in run:
            # A unique stamp identifies which packet wins a group. Phase is
            # a sensitivity check for the unknown origin of the engine clock.
            listener.receive(float(row['robot_uptime_s']) + phase, index)
        listener.advance(float(run[-1][1]['robot_uptime_s']) + 1)
        for when, index in listener.events:
            predicted[index] = 1000*(when-phase-float(rows[index]['robot_uptime_s']))
    return predicted

predicted = replay(0)
with (here / 'broadcast-transfer.csv').open('w', newline='') as f:
    out = csv.writer(f, lineterminator='\n')
    out.writerow(['run','packet_index','callback_delay_ms','broadcast_delay_ms','residual_ms'])
    for index, row in enumerate(rows):
        measured = row['sdk_host_offset_ms']
        delay = predicted.get(index)
        residual = float(measured)-delay if measured and delay is not None else None
        out.writerow([row['run'], index, measured, delay, residual])
        if residual is not None:
            matches.append((row['run'], float(measured), delay, residual))
fig, ax = plt.subplots(figsize=(9,3.9),layout='constrained')
ax.scatter(range(len(matches)),[x[1] for x in matches],label='Recorded SDK callback',color='#156b8c')
ax.scatter(range(len(matches)),[x[2] for x in matches],label='Simulated engine broadcast',marker='x',color='#bc4f25')
ax.axhline(145.906, color='#597b4c', ls='--', label='Earlier empirical host delay')
ax.axhline(0,color='#777',ls=':',label='Packet counter change')
ax.set(xticks=range(len(matches)),xticklabels=[x[0] for x in matches],ylabel='Delay from packet receipt (ms)',xlabel='Matched packet, by recording')
ax.legend(fontsize=8)
fig.savefig(here/'broadcast-transfer.png',dpi=160)
print('changes, broadcasts, callbacks, matched',len(rows),len(predicted),sum(bool(r['sdk_host_offset_ms']) for r in rows),len(matches))
print('median absolute timing error before/after',np.median([abs(x[1]) for x in matches]),np.median([abs(x[3]) for x in matches]))
for phase in (0,.01,.02,.03,.04,.05):
    estimates=replay(phase)
    common=[abs(float(rows[i]['sdk_host_offset_ms'])-v) for i,v in estimates.items() if rows[i]['sdk_host_offset_ms']]
    print('phase ms',round(phase*1000),'broadcasts',len(estimates),'matched',len(common),'median absolute residual',np.median(common))

"""Compare browser and native listener transitions on archived shelf packets, then SDK receipts."""
import csv
import json
from pathlib import Path
import subprocess
import sys

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
root = here.parents[2]
sys.path.insert(0, str(root))
from simulacrum.cube import MovementListener, UpAxisListener, engine_samples, packet

paths = sorted(here.parent.glob('*/cube.csv.gz')) + sorted(here.glob('*/cube.csv.gz'))
browser = json.loads(subprocess.check_output(['node', str(root/'web/tools/replay-cube-listeners.mjs'),
                                             *map(str, paths)], text=True))
fig, ax = plt.subplots(figsize=(7,3.2), layout='constrained')
with (here/'browser-events.csv').open('w') as out:
    writer = csv.writer(out,lineterminator='\n')
    writer.writerow(['run','slots','listener_starts','listener_stops','sdk_starts','sdk_stops','axis_changes'])
    for path, events in zip(paths,browser):
        rows = np.genfromtxt(path,delimiter=',',names=True)
        if not len(rows): continue
        move, up = MovementListener(), UpAxisListener()
        for group in rows.reshape(-1,3):
            payload = packet(int(group['tap'][0]), np.column_stack([group[k] for k in 'xyz']).astype(int))
            for sample in engine_samples(payload):
                move.update(group[0]['sec'],sample)
                up.update(group[0]['sec'],sample)
        assert [(e['time_s'],e['moving']) for e in events if 'moving' in e] == move.events
        assert [(e['time_s'],e['axis']) for e in events if 'axis' in e] == up.events
        with (path.parent/'events.csv').open() as f:
            sdk = [(float(r['sec']),r['event']=='object_moved') for r in csv.DictReader(f)
                   if r['event'] in ('object_moved','object_stopped_moving')
                   and rows['sec'][0]<=float(r['sec'])<=rows['sec'][-1]]
        counts = lambda pairs: [sum(v==flag for _,v in pairs) for flag in (True,False)]
        writer.writerow([path.parent.name,len(rows),*counts(move.events),*counts(sdk),len(up.events)])
        if path.parent.name=='S22B':
            for j,(event,ref) in enumerate(zip(move.events,sdk)):
                assert event[1]==ref[1]
                offset = (ref[0]-event[0])*1000
                ax.plot([0,offset],[j,j],color='.65')
                ax.scatter(0,j,marker='|',s=170,c='C0',label='Browser packet' if not j else None)
                ax.scatter(offset,j,marker='o',c='C1',label='SDK receipt' if not j else None)
ax.set(yticks=range(4),yticklabels=['Start 1','Stop 1','Start 2','Stop 2'],
       xlabel='Milliseconds relative to the triggering cube packet',title='Shelf roll movement notifications')
ax.legend()
fig.savefig(here/'browser-events.png',dpi=150)

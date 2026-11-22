"""Compare SDK cube rectangles around the three physical wheelstand commands."""
import csv
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

out = Path(__file__).resolve().parent
logs = Path(sys.argv[1])
fig, axes = plt.subplots(1, 3, figsize=(10, 3.4), layout='constrained')
width, height = 800, 600
with (out / 'sightings.csv').open('w') as stream:
    writer = csv.writer(stream, lineterminator='\n')
    writer.writerow(['trial','seconds_from_command','x','y','width','height','fraction_inside_image'])
    for trial, ax in zip(('W4','W8','W11'), axes):
        events = []
        for line in (logs / f'p2wheel-{trial}.jsonl').read_text().splitlines():
            line = line.strip()
            while line:
                event, end = json.JSONDecoder().raw_decode(line)
                events.append(event)
                line = line[end:].strip()
        start = next(e['t'] for e in events if e.get('action')=='pop_a_wheelie' and e['kind']=='begin')
        seen = [e for e in events if e['kind']=='robot_observed_object'
                and e['data'].get('object_family')=='LIGHT_CUBE']
        before = max((e for e in seen if e['t']<=start),key=lambda e:e['t'])
        after = [e for e in seen if start<e['t']<=start+3.85]
        for e in [before,*after]:
            box=e['data']['img_rect']
            x,y,w,h=[box[k] for k in ('x_top_left','y_top_left','width','height')]
            visible_w=max(0,min(width,x+w)-max(0,x))
            visible_h=max(0,min(height,y+h)-max(0,y))
            fraction=visible_w*visible_h/(w*h)
            writer.writerow([trial,round(e['t']-start,6),x,y,w,h,fraction])
            if e is before or (after and e is after[0]):
                ax.add_patch(Rectangle((x,y),w,h,fill=False,edgecolor='#c56024' if e is before else '#187587',
                                       label='Before request' if e is before else 'First reacquisition',lw=2))
        ax.add_patch(Rectangle((0,0),width,height,fill=False,color='black'))
        ax.set(xlim=(-100,850),ylim=(730,-50),aspect='equal',title=trial,xlabel='Image x (pixels)')
        ax.legend(loc='upper right',fontsize=7)
axes[0].set_ylabel('Image y (pixels)')
fig.savefig(out/'sightings.png',dpi=160)

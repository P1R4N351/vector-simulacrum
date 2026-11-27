"""Run the same commands on the canonical robot, sampled at the physical SDK times."""
import csv
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory

root=Path(__file__).resolve().parents[1]
runner=root.parent/'workshop-joints-20261127/lift-duration/replay.mjs'
with open('sim-pauses.csv','w') as output:
    out=csv.writer(output,lineterminator='\n')
    out.writerow(('session','lower','pause_s','quantized_pause_s'))
    for name in ('recovery','height-crossover','speed-crossover'):
        with (root/name/'sdk.csv').open() as f:
            rows=list(csv.DictReader(f))
        with TemporaryDirectory() as d:
            folder=Path(d)
            with (folder/'recording.csv').open('w') as f:
                w=csv.writer(f)
                w.writerow(('s','event','target_mm','duration_s','lift_mm','head_deg','gy','left','right','dock','code','in_position'))
                for r in rows:
                    w.writerow([r['relative_s'],r['event'],r['target'],r.get('duration_s') or .6,r['lift_mm'],r['head_deg'],r['gy'],r['left'],r['right'],r['dock'],'',r['in_position']])
            subprocess.run(['node',str(runner),folder.as_uri()+'/'],check=True)
            with (folder/'replay.csv').open() as f:
                sim=list(csv.DictReader(f))
        t=[float(r['s']) for r in sim]
        h=[float(r['lift_mm']) for r in sim]
        starts=[float(r['relative_s']) for r in rows if r['event']=='command' and r['target']=='52']
        for n,s in enumerate(starts,1):
            first=next(i for i,r in enumerate(sim) if t[i]>=s and r['in_position']=='0')
            ready=next(i for i in range(first+1,len(sim)) if sim[i]['in_position']=='1')
            trough=min(range(first,ready+1),key=h.__getitem__)
            durations=[]
            for heights in (h,[round(v/.34)*.34 for v in h]):
                spans=[0]
                i=first
                while i<trough:
                    j=i
                    while j<trough and abs(heights[j+1]-heights[i])<.05:
                        j+=1
                    if 53<=heights[i]<=58 and j-i>=2:
                        spans.append(t[j]-t[i])
                    i=j+1
                durations.append(max(spans))
            out.writerow((name,n,*durations))

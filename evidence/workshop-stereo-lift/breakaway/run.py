"""Select a friction candidate on six lowers, then freeze it for two other sessions."""
from concurrent.futures import ThreadPoolExecutor
import csv
from pathlib import Path
import subprocess
import tempfile
import numpy as np

here=Path(__file__).resolve().parent
runner=here.parents[1]/'workshop-joints-20261127/lift-duration/replay.mjs'


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def replay(settings):
    name,peak=settings
    sdk=read(here.parent/name/'sdk.csv')
    commands=[r for r in sdk if r['event']=='command']
    states=[r for r in sdk if r['event']=='state']
    with tempfile.TemporaryDirectory() as d:
        folder=Path(d)
        output=folder/'output'
        output.mkdir()
        with (folder/'recording.csv').open('w') as f:
            w=csv.writer(f)
            w.writerow(('s','event','target_mm','duration_s','lift_mm','head_deg','gy','left','right','dock','code','in_position'))
            for r in sdk:
                w.writerow([r['relative_s'],r['event'],r['target'],r.get('duration_s') or .6,r['lift_mm'],r['head_deg'],r['gy'],r['left'],r['right'],r['dock'],'',r['in_position']])
        args=['node',str(runner),folder.as_uri()+'/']
        if peak:
            args+=['','0.003',output.as_uri()+'/']
            if peak>3:
                args+=['','',str(peak/1000)]
        subprocess.run(args,check=True)
        sim=read((output if peak else folder)/'replay.csv')
    t=np.array([float(r['s']) for r in sim])
    actual=np.array([float(r['lift_mm']) for r in states])
    pred=np.array([float(r['lift_mm']) for r in sim])
    flags=np.array([int(r['in_position']) for r in sim])
    assert all(int(r['on_dock']) for r in sim)
    assert np.allclose(t,[float(r['relative_s']) for r in states],atol=1e-7,rtol=0)
    rows=[]
    for n,cmd in enumerate(commands):
        start=float(cmd['relative_s'])
        target=int(cmd['target'])
        # Common observation length, bounded by next command.
        stop=min(start+1.4,float(commands[n+1]['relative_s']) if n+1<len(commands) else t[-1])
        mask=(t>=start)&(t<stop)
        error=float(np.mean((pred[mask]-actual[mask])**2)**.5)
        pauses=[]
        for heights,flag in ((actual,np.array([int(r['in_position']) for r in states])),(pred,flags)):
            pause=0
            if target!=64:
                first=np.flatnonzero((t>=start)&(flag==0))[0]
                ready=first+np.flatnonzero(flag[first:]==1)[0]
                assert t[ready]<stop or n==len(commands)-1 or t[ready]<float(commands[n+1]['relative_s'])
                trough=first+np.argmin(heights[first:ready+1])
                pre=np.median(heights[(t>=start-.2)&(t<start)])
                i=first
                while i<trough:
                    j=i
                    while j<trough and abs(heights[j+1]-heights[i])<.05:
                        j+=1
                    if j-i>=2 and pre-heights[i]>=1 and heights[i]-target>=1 and heights[i]-heights[trough]>=1:
                        pause=max(pause,t[j]-t[i])
                    i=j+1
            pauses.append(pause)
        rows.append((name,peak,n+1,target,error,*pauses))
    return rows


if __name__=='__main__':
    with ThreadPoolExecutor(max_workers=3) as ex:
        train=list(ex.map(replay,[('recovery',p) for p in (0,9,18,30)]))
        def cost(rows):
            return np.mean([r[4] for r in rows if r[3]==52])
        selected=min(train[1:],key=cost)[0][1]
        tests=list(ex.map(replay,[(n,p) for n in ('target-crossover','target-transfer') for p in (0,selected)]))
        controls=list(ex.map(replay,[(n,3) for n in ('recovery','target-crossover','target-transfer')]))
    with (here/'scores.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(('session','peak_mNm','command','target_mm','rmse_mm','actual_pause_s','sim_pause_s'))
        for rows in train+tests+controls:
            w.writerows(rows)
    for rows in train+tests+controls:
        print(rows[0][:2],[(np.mean([r[4] for r in rows if (r[3]==64)==up])) for up in (True,False)],sum(r[-1]>0 for r in rows))

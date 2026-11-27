import csv,json
from pathlib import Path
here=Path(__file__).parent
def read(folder,name):
    with (folder/name).open() as f:return list(csv.DictReader(f))
runs=[]
for sub in ['.','speed-holdout','interleaved']:
    folder=here.parent/sub
    events=read(folder,'events.csv');states=read(folder,'states.csv')
    starts=[e for e in events if e['event']=='spin']
    stops=[e for e in events if e['event']=='stop']
    for index,(cue,stop) in enumerate(zip(starts,stops),1):
        start=float(cue['time_s']);end=float(stop['time_s'])
        before=next(r for r in reversed(states) if float(r['time_s'])<start)
        runs.append(dict(session=sub,turn=index,speed=int(cue['detail'])*(100 if sub=='.' else 1),duration=end-start,
                         head_rad=float(before['head_rad']),lift_mm=float(before['lift_mm'])))
assert len(runs)==14
(here/'inputs.json').write_text(json.dumps(runs,indent=2)+'\n')

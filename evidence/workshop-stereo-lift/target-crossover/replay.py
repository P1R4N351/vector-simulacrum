"""Map relative SDK records to the canonical physical-command replay schema."""
import csv
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory
from shutil import copyfile

here=Path(__file__).resolve().parent
with TemporaryDirectory() as d:
    folder=Path(d)
    with (here/'sdk.csv').open() as src,(folder/'recording.csv').open('w') as f:
        w=csv.writer(f)
        w.writerow(('s','event','target_mm','duration_s','lift_mm','head_deg','gy','left','right','dock','code','in_position'))
        for r in csv.DictReader(src):
            w.writerow([r[k] for k in ('relative_s','event','target','duration_s','lift_mm','head_deg','gy','left','right','dock')]+['',r['in_position']])
    script=here.parents[1]/'workshop-joints-20261127/lift-duration/replay.mjs'
    subprocess.run(['node',str(script),folder.as_uri()+'/'],check=True)
    copyfile(folder/'replay.csv',here/'replay.csv')

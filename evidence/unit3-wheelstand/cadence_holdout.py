"""Test the previously published rest-based delivery model on separate sessions."""
import csv
import gzip
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
calibration = here.parent / 'unit3-cube-accelerometer'
sys.path.insert(0, str(calibration))
from packet_cadence import clocks, estimate

normal, pause, period = estimate(clocks('resting')[0])
rows = []
fig, axes = plt.subplots(2, 1, figsize=(9, 6), layout='constrained')
for name in ['A', 'B', 'C', 'D', 'E', 'F', 'W1', 'W2', 'W3', 'W4']:
    if name.startswith('W'):
        with gzip.open(here / name / 'cube.csv.gz', 'rt') as f:
            timestamps = [float(r['uptime']) for r in csv.DictReader(f) if r['slot'] == '0']
    else:
        with gzip.open(here.parent / 'unit3-physical-repeats' / name / 'radio.csv.gz', 'rt') as f:
            timestamps = [float(r['robot_uptime_s']) for r in csv.DictReader(f)
                          if len(r['payload_hex']) == 40 and r['payload_hex'].startswith('02')]
    if len(timestamps) < 2:
        rows.append([name,len(timestamps),0,'','','','',''])
        continue
    elapsed = np.array(timestamps) - timestamps[0]
    idx = np.arange(len(elapsed))
    prediction = normal * idx + (pause-normal) * np.floor(idx/period)
    error = prediction - elapsed
    uniform = .03 * idx[-1]
    rows.append([name,len(idx),sum(np.diff(elapsed)>.15),elapsed[-1],uniform,
                 prediction[-1],100*error[-1]/elapsed[-1],max(abs(error))])
    axes[0].plot(elapsed,error,label=name)
    axes[1].scatter(name,100*(uniform/elapsed[-1]-1),color='grey')
    axes[1].scatter(name,100*error[-1]/elapsed[-1],color='tab:blue')
axes[0].set(xlabel='Elapsed robot receipt time (s)',ylabel='Rest-model time error (s)')
axes[0].legend(ncol=5)
axes[1].scatter([],[],color='grey',label='Uniform 30 ms')
axes[1].scatter([],[],color='tab:blue',label='Rest-based pauses')
axes[1].set(xlabel='Physical recording',ylabel='Endpoint error (%)')
axes[1].legend()
fig.savefig(here/'cadence-holdout.png',dpi=150)
with (here/'cadence-holdout.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow('run packets long_gaps measured_s uniform_s prediction_s endpoint_error_pct peak_timeline_error_s'.split())
    w.writerows(rows)

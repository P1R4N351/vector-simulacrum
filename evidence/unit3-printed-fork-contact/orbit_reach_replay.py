"""Freeze the offset from the side photo before evaluating the five cube pushes."""
import csv
import numpy as np
from reach import score, here
from push_phase import read

fit=read(here.parent/'unit3-fork-silhouette/orbit-reach.csv')
train=fit[(fit['heading']==60)&(fit['wrist_height_mm']==32)]
reach=train['reach_offset_mm'][np.argmin(train['mean_abs_native_px'])]
with (here/'orbit-reach-replay.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(['trial','offset_mm','pre_raise_error_mm','endpoint_error_mm','peak_tilt_deg'])
    for trial in range(1,6):
        for offset in (0,reach):
            w.writerow(score((trial,offset)))

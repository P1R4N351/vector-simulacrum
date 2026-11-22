"""Audit front and rear normal load at the recorded unit 3 starting posture."""
from pathlib import Path
from unittest.mock import patch
import sys
import csv

import mujoco
import numpy as np

here = Path(__file__).resolve().parent
sys.path[:0] = [str(here.parents[1]), str(here.parent/'recorded-pose-inset')]
from contact import contact_xml
from simulacrum import mjcf, params, replay
from simulacrum.robot import VectorSim

base = mjcf.build_mjcf
values = dict(params.values(params.load(model="vector_2")), **replay.OPEN_TABLE, tread_contact_inset_mm=0.)
with (here/'shift-load.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(['forward_shift_mm','left_command_mm_s','front_normal_share','rear_normal_share','other_normal_share'])
    for shift in (-1,0,1):
        for left in (-150,150):
            with patch.object(mjcf,'build_mjcf',lambda *args:contact_xml(base(*args),3.75,shift)):
                sim=VectorSim(values,head_deg=22.5,lift_mm=32.7)
            sim.advance(.48)
            sim.set_wheel_motors(left,-left)
            load=np.zeros(3)
            for tick in range(280):
                sim.step()
                if tick<139:
                    continue
                for index,c in enumerate(sim.data.contact):
                    names=[sim.model.geom(int(g)).name for g in c.geom]
                    if 'table' not in names:
                        continue
                    other=names[1-names.index('table')]
                    group=0 if other.startswith('wheel_patch_f') else 1 if other.startswith('wheel_patch_r') else 2
                    wrench=np.zeros(6)
                    mujoco.mj_contactForce(sim.model,sim.data,index,wrench)
                    load[group]+=wrench[0]
            w.writerow([shift,left,*(load/load.sum())])

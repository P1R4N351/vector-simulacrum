"""Inspect support and axle-to-roller speed transfer for the recorded pose."""
from unittest.mock import patch
import math
import sys
import mujoco
import numpy as np
from simulacrum import mjcf, params, replay
from simulacrum.robot import VectorSim
from contact import contact_xml


def inspect(inset, sign):
    values = dict(params.values(params.load(model='vector_2')), **replay.OPEN_TABLE)
    build = mjcf.build_mjcf
    with patch.object(mjcf, 'build_mjcf', lambda *args: contact_xml(build(*args), inset)):
        sim = VectorSim(values, head_deg=22.5, lift_mm=32.7)
    assert sim.set_wheel_motors(-146*sign, 146*sign)
    forces = {'patch': 0., 'axle': 0., 'other': 0.}
    speeds = []
    for i in range(round(1.4/sim.dt)):
        sim.step()
        if sim.data.time < .7:
            continue
        for j, c in enumerate(sim.data.contact):
            g1, g2 = (sim.model.geom(int(k)).name or '' for k in c.geom)
            if 'table' not in (g1, g2):
                continue
            n = g2 if g1 == 'table' else g1
            key = 'patch' if n.startswith('wheel_patch_') else 'axle' if n.startswith('wheel_') else 'other'
            wrench = np.zeros(6)
            mujoco.mj_contactForce(sim.model, sim.data, j, wrench)
            forces[key] += wrench[0]
        for side in ('fl','fr','rl','rr'):
            shaft, roller = (sim.data.joint(n).qvel[0] for n in ('wheel_'+side+'_joint','patch_'+side))
            r = values['wheel_front_radius_mm' if side[0]=='f' else 'wheel_rear_radius_mm']
            speeds.append(abs(shaft-roller)*r)
    return [inset, sign, *(forces[k]/sum(forces.values()) for k in ('patch','axle','other')),
            np.mean(speeds), np.percentile(speeds,95)]


if __name__ == '__main__':
    import csv
    write = csv.writer(sys.stdout)
    write.writerow(('inset_mm','ccw_sign','normal_load_patch_fraction','normal_load_axle_fraction',
                    'normal_load_other_fraction','mean_abs_shaft_minus_roller_mm_s','p95_abs_shaft_minus_roller_mm_s'))
    for inset in map(float,sys.argv[1:]):
        for sign in (-1,1):
            write.writerow(inspect(inset, sign))

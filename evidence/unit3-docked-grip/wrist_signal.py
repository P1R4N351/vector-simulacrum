"""Replay a face-square first-touch placement at the measured wrist trajectory."""
import argparse
import csv
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np

from movement_drive import compare, root
from simulacrum import mjcf, params
from simulacrum.cube import CubeAccelerometer, packet


def replay(path, pitched, v, clock, heights, pitches, contact_observer=None, *,
           yaw_deg=0, lateral_mm=0):
    tree = ET.parse(path)
    cube = tree.find(".//body[@name='cube']")
    cube.append(ET.fromstring(mjcf._cube_chip_xml(v)))
    sensor = ET.SubElement(tree.getroot(), 'sensor')
    ET.SubElement(sensor, 'accelerometer', site='cube_chip')
    model = mj.MjModel.from_xml_string(ET.tostring(tree.getroot(), encoding='unicode'))
    d = mj.MjData(model)
    fork, cube = [model.body(n).id for n in ('fork', 'cube')]
    base, _, zbase = v['lift_base_position_mm']
    x = base + np.sqrt(v['lift_arm_length_mm']**2 - (heights-zbase)**2)
    pitch = pitches if pitched else np.zeros_like(pitches)
    axle = v['wheel_front_radius_mm']
    world_x = x*np.cos(pitch) - (heights-axle)*np.sin(pitch)
    world_z = axle + x*np.sin(pitch) + (heights-axle)*np.cos(pitch) + (3 if pitched else 0)

    def pose(i):
        d.mocap_pos[0] = [world_x[i]/1000, 0, world_z[i]/1000]
        d.mocap_quat[0] = [np.cos(pitch[i]/2), 0, -np.sin(pitch[i]/2), 0]

    pose(0)
    joint = d.joint('cube_root')
    half_yaw = np.deg2rad(yaw_deg) / 2
    joint.qpos[:] = [.15, lateral_mm / 1000, .02215, np.cos(half_yaw), 0, 0, np.sin(half_yaw)]
    def touching(xpos):
        joint.qpos[0] = xpos
        mj.mj_forward(model, d)
        return any(c.dist < 0 and set(model.geom_bodyid[c.geom]) == {fork, cube} for c in d.contact)

    for xpos in np.arange(.15, .025, -.00025):
        if touching(xpos):
            break
    else:
        raise ValueError('First contact not found')
    inside, clear = xpos, xpos + .00025
    for _ in range(20):
        mid = (inside+clear)/2
        if touching(mid):
            inside = mid
        else:
            clear = mid
    assert not touching(clear)
    mj.mj_step(model, d, nstep=500)
    start = d.time - clock[0]
    chip = CubeAccelerometer(v, model.opt.timestep, seed=2)
    g = np.linalg.norm(model.opt.gravity)
    chip.reset(d.sensordata.copy()/g, d.time)
    for i in range(len(clock)):
        pose(i)
        mj.mj_step(model, d)
        chip.step(d.time, d.sensordata/g)
        if contact_observer is not None:
            contact_observer(clock[i], model, d, fork, cube)
    return [(t-start, p) for t, p in chip.messages]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('printed_fixture')
    args = parser.parse_args()
    v = params.values(params.load(model='vector_2'))
    clock = np.arange(-.35, 2.45, .001)
    def interp(filename, col):
        a = np.loadtxt(root/filename, delimiter=',', skiprows=1)
        return np.interp(clock, a[:,0], a[:,col])
    heights = interp('encoder.csv', 1)
    pitches = interp('dock-pitch.csv', 1)
    source = np.loadtxt(root/'cube.csv', delimiter=',', skiprows=1).reshape(-1,3,5)
    packets = {'unit 3': [(p[0,0], packet(0, p[:,2:].astype(int).tolist())) for p in source]}
    for name, path in [('Boxes',root.parent/'unit3-cube-contact-replay/fixture.xml'),
                       ('Printed',args.printed_fixture)]:
        for pitched in [False, True]:
            label = name + (' · dock posture' if pitched else ' · level')
            packets[label] = replay(path, pitched, v, clock, heights, pitches)
    fig, ax = plt.subplots(figsize=(8,4), layout='constrained')
    rows = []
    for name, signal in packets.items():
        trace = compare([(t,p) for t,p in signal if -.21 <= t <= 2.45])
        t, score = trace[trace[:,0]>=0,:2].T
        ax.plot(t,score,'.' if name=='unit 3' else '-',ms=3,label=name)
        rows.append([name,score.max(),t[score.argmax()],score[t>=1.35].max()])
    ax.axhline(10000,ls=':',color='grey',label='Movement start')
    ax.set(xlabel='Seconds from lift command',ylabel='Movement listener score')
    ax.legend(fontsize=8)
    fig.savefig(root/'wrist-signal.png', dpi=160)
    with (root/'wrist-signal.csv').open('w') as f:
        w=csv.writer(f)
        w.writerow(['source','peak','peak_packet_s','peak_after_1.35s'])
        w.writerows(rows)


if __name__ == '__main__':
    main()

"""Locate contact and integrate normal impulse over the measured wrist replay."""
import argparse
import csv

import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np

from wrist_signal import compare, params, replay, root


def run(fixture, v, clock, heights, pitches, yaw, lateral):
    hits = []
    force = np.zeros(6)
    def observe(t, model, d, fork, cube):
        if t < 0:
            return
        rotation = d.xmat[fork].reshape(3, 3)
        for i, contact in enumerate(d.contact):
            if contact.efc_address < 0 or set(model.geom_bodyid[contact.geom]) != {fork,cube}:
                continue
            mj.mj_contactForce(model, d, i, force)
            if force[0] > 1e-5:
                local = rotation.T @ (contact.pos - d.xpos[fork])
                hits.append((local[2]*1000, force[0]*model.opt.timestep))
    packets = replay(fixture, True, v, clock, heights, pitches, observe,
                     yaw_deg=yaw, lateral_mm=lateral)
    trace = compare([(t,p) for t,p in packets if -.21 <= t <= 2.45])
    late = trace[trace[:,0]>=1.35,1].max()
    if not hits:
        return (yaw,lateral,np.nan,0,0,late)
    z, impulse = np.asarray(hits).T
    return (yaw,lateral,z.min(),impulse.sum(),impulse[z < -12].sum(),late)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('printed_fixture')
    args = parser.parse_args()
    clock = np.arange(-.35, 2.45, .001)
    def channel(file):
        a = np.loadtxt(root/file, delimiter=',', skiprows=1)
        return np.interp(clock,a[:,0],a[:,1])
    heights, pitches = channel('encoder.csv'), channel('dock-pitch.csv')
    v = params.values(params.load(model='vector_2'))
    rows = [run(args.printed_fixture,v,clock,heights,pitches,y,l)
            for y in (-9,0,2) for l in (-3,0,3)]
    with (root/'oblique-contact.csv').open('w') as f:
        w = csv.writer(f)
        w.writerow(['yaw_deg','lateral_mm','lowest_contact_mm','normal_impulse_Ns',
                    'normal_impulse_below_minus12mm_Ns','peak_score_after_1.35s'])
        w.writerows(rows)
    fig,axes = plt.subplots(1,2,figsize=(9,3.8), layout='constrained')
    for y in (-9,0,2):
        subset = np.asarray([r for r in rows if r[0]==y])
        for ax,column in zip(axes,(2,5)):
            ax.plot(subset[:,1],subset[:,column],'o-',label=f'{y}°')
    axes[0].axhline(-12,color='grey',ls=':',label='Top of damage region')
    axes[0].set_ylabel('Lowest loaded contact above wrist origin (mm)')
    axes[1].set_ylabel('Peak movement score after 1.35 s')
    for ax in axes:
        ax.set_xlabel('Initial lateral offset (mm)')
        ax.legend(fontsize=8)
    fig.savefig(root/'oblique-contact.png',dpi=165)


if __name__ == '__main__':
    main()

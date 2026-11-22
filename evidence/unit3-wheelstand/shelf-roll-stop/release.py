"""Compare normalized 90%-to-10% fall times for the glyph and the free cube."""
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import mujoco
import numpy as np

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[2]))
from simulacrum import mjcf, params


def crossing(t, y, level):
    i = np.flatnonzero((y[:-1] > level) & (y[1:] <= level))[0]
    return t[i] + (level-y[i]) * (t[i+1]-t[i]) / (y[i+1]-y[i])


def initial_state(axis, angle):
    v = params.values(params.load(model='vector_2'))
    model = mujoco.MjModel.from_xml_string(mjcf.build_mjcf(v, cube='free'))
    data = mujoco.MjData(model)
    j = model.joint('cube_root').qposadr.item()
    half = np.deg2rad(angle)/2
    direction = np.eye(3)[axis] if np.isscalar(axis) else np.asarray(axis)
    assert direction.shape == (3,) and np.isclose(np.linalg.norm(direction), 1.)
    data.qpos[j+3:j+7] = [np.cos(half), *(np.sin(half)*direction)]
    mujoco.mj_forward(model, data)
    cube = model.body('cube').id
    geoms = np.flatnonzero(model.geom_bodyid == cube)
    low = min(data.geom_xpos[g,2] - abs(data.geom_xmat[g].reshape(3,3)[2]) @ model.geom_size[g] for g in geoms)
    data.qpos[j+2] -= low
    return model, data, cube, v


def replay(axis, angle):
    model, data, cube, _ = initial_state(axis, angle)
    times, tilts = [], []
    while data.time < .25:
        mujoco.mj_forward(model, data)
        times.append(data.time)
        tilts.append(np.rad2deg(np.arccos(np.clip(data.xmat[cube,8],-1,1))))
        mujoco.mj_step(model,data)
    return np.array(times), np.array(tilts)/abs(angle)


def plot(t,y,label):
    start,end = (crossing(t,y,k) for k in (.9,.1))
    print(label, round(1000*(end-start),1),'ms')
    plt.plot(1000*(t-start),y,label=label)


if __name__ == '__main__':
    t,a = np.loadtxt(here/'native-face-angle.csv',delimiter=',',skiprows=1).T
    high = np.nanmedian(a[(t>5.15)&(t<5.23)])
    low = np.nanmedian(a[(t>6)&(t<10)])
    use = (t>=5.19)&(t<=5.45)
    plot(t[use], (a[use]-low)/(high-low),'Camera, projected glyph')
    for axis in (0,1):
        for sign in (-1,1):
            plot(*replay(axis,sign*15.76), f'Model, {"xy"[axis]} {sign:+d}')
    plt.axhline(.1,color='.6',ls=':')
    plt.xlim(-40,140)
    plt.xlabel('Milliseconds from 90% remaining tilt')
    plt.ylabel('Fraction of tilt remaining')
    plt.title('Free cube settling from one edge')
    plt.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(here/'release.png',dpi=150)

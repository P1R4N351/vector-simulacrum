"""Compute world moments around the cube COM and project onto an increasing-tilt axis."""
import mujoco as mj
import numpy as np


def moments(model, data, cube, floor, fork):
    rotation = data.xmat[cube].reshape(3,3)
    axis = np.cross([0.,0.,1.],rotation[:,2])
    axis /= np.linalg.norm(axis)
    torque = np.zeros((3,3))
    wrench = np.empty(6)
    for j,c in enumerate(data.contact):
        bodies = model.geom_bodyid[c.geom]
        if cube not in bodies:
            continue
        side = int(bodies[1] == cube)
        other = c.geom[1-side]
        group = 0 if other == floor else (1 if model.geom_bodyid[other] == fork else 2)
        mj.mj_contactForce(model,data,j,wrench)
        basis = c.frame.reshape(3,3)
        force, couple = (wrench.reshape(2,3) @ basis) * (1 if side else -1)
        torque[group] += np.cross(c.pos-data.xipos[cube],force)+couple
    # The fixture's isotropic free cube has its body origin at its COM.
    joint = model.body_jntadr[cube]
    adr = model.jnt_dofadr[joint]+3
    solver = rotation @ data.qfrc_constraint[adr:adr+3]
    np.testing.assert_allclose(torque.sum(axis=0), solver, atol=1e-8, rtol=1e-7)
    return torque @ axis * 1000

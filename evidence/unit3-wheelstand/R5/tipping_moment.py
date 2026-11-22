"""Compute world moments around the cube COM and project onto an increasing-tilt axis."""
import mujoco as mj
import numpy as np


def moments(model, data, cube, floor, fork, fork_yaw=None):
    rotation = data.xmat[cube].reshape(3,3)
    axis = np.cross([0.,0.,1.],rotation[:,2])
    axis_length = np.linalg.norm(axis)
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
        lever = c.pos - data.xipos[cube]
        total = np.cross(lever, force) + couple
        torque[group] += total
        if fork_yaw is not None and group == 1 and wrench[0] > 0:
            normal = basis[0] * wrench[0] * (1 if side else -1)
            normal_yaw = np.cross(lever, normal)[2]
            local = (c.pos - data.xpos[fork]) @ data.xmat[fork].reshape(3,3)
            fork_yaw.append((model.geom(c.geom[side]).name, *local, wrench[0],
                             normal_yaw, total[2] - normal_yaw, *(lever @ rotation)))
    # The fixture's isotropic free cube has its body origin at its COM.
    joint = model.body_jntadr[cube]
    adr = model.jnt_dofadr[joint]+3
    solver = rotation @ data.qfrc_constraint[adr:adr+3]
    np.testing.assert_allclose(torque.sum(axis=0), solver, atol=1e-8, rtol=1e-7)
    projected = torque @ axis / axis_length if axis_length > 1e-10 else np.full(3,np.nan)
    return projected * 1000, torque, axis

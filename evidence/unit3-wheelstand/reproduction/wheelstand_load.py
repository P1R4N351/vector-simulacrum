"""First 10 mN normal contact between the cube and either fixed body or fork."""
import mujoco
import numpy as np


class FirstLoads:
    def __init__(self, model):
        self.times = {}
        self.wrench = np.empty(6)
        self.cube = model.body("cube").id
        self.fork = model.body("lift_fork").id
        self.box = model.geom("chassis_geom").id

    def step(self, run, t):
        if len(self.times) == 2:
            return
        bodies = run.m.geom_bodyid
        for i, contact in enumerate(run.d.contact):
            a, b = contact.geom
            if bodies[a] == self.cube:
                other = b
            elif bodies[b] == self.cube:
                other = a
            else:
                continue
            if other == self.box:
                kind = "box"
            elif bodies[other] == self.fork:
                kind = "fork"
            else:
                continue
            if kind in self.times:
                continue
            mujoco.mj_contactForce(run.m, run.d, i, self.wrench)
            if self.wrench[0] >= .01:
                self.times[kind] = t

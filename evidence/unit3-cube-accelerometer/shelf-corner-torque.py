"""Integrate shelf and robot yaw moments about the free cube center during the push."""
import csv
import importlib
import sys
from xml.etree import ElementTree as ET
import mujoco as mj
import numpy as np
run=importlib.import_module("fork-rake")
variant=importlib.import_module("shelf-corner-replay").shelf_variant
with (run.here/"shelf-corner-torque.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("mu dt_ms start_s end_s shelf_uNms fork_uNms rail_uNms measured_delta_L_uNms".split())
    for mu in (.9,.95,1.05):
        tree=variant(sys.argv[1],mu,1)
        run.add_links(tree,run.repo)
        model=mj.MjModel.from_xml_string(ET.tostring(tree.getroot(),encoding="unicode"))
        cube=model.body("cube").id
        table=model.geom("table").id
        fork=model.body("fork").id
        inertia=model.body_inertia[cube]
        assert np.ptp(inertia)<1e-12
        trace=[]
        def observe(m,d,t,i):
            if not 2.09<=t<=2.87: return
            torque=np.zeros(3)
            for j,c in enumerate(d.contact):
                ids=c.geom
                bodies=m.geom_bodyid[ids]
                if cube not in bodies: continue
                a=int(bodies[1]==cube)
                other=1-a
                key=0 if ids[other]==table else 1 if bodies[other]==fork else 2
                wrench=np.zeros(6)
                mj.mj_contactForce(m,d,j,wrench)
                axes=c.frame.reshape(3,3).T
                moment=np.cross(c.pos-d.xipos[cube],axes@wrench[:3])+axes@wrench[3:]
                torque[key]+=moment[2]*(1 if a else -1)
            quat=d.joint("cube_root").qpos[3:]
            omega=d.joint("cube_root").qvel[3:]
            world=np.empty(3)
            mj.mju_rotVecQuat(world,omega,quat)
            trace.append((t,*torque,inertia[0]*world[2]))
        for result in run.replay(model,[0],observe=observe):
            arr=np.array(trace)
            for lo,hi in [(2.095,2.595),(2.595,2.86)]:
                i,j=[np.argmin(abs(arr[:,0]-t)) for t in (lo,hi)]
                imp=arr[i+1:j+1,1:4].sum(axis=0)*model.opt.timestep*1e6
                dl=(arr[j,4]-arr[i,4])*1e6
                np.testing.assert_allclose(imp.sum(),dl,atol=.02)
                w.writerow([mu,result[0],lo,hi,*imp,dl])
            trace.clear()

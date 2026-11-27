"""Separate the four rubber footprints on the shelf from the physical fork contact geometry."""
import csv
from copy import deepcopy
import importlib
from pathlib import Path
import sys
from xml.etree import ElementTree as ET

import mujoco as mj
import numpy as np

def shelf_variant(source, mu, width):
    tree=ET.parse(source)
    # Keep the original pad collisions with the robot. Give the floor its own
    # material and footprint; the shelf still collides with plastic shell parts.
    cube=tree.find(".//body[@name='cube']")
    table=tree.find(".//geom[@name='table']")
    table.set("conaffinity","11")
    for g in list(cube.findall("geom")):
        floor=deepcopy(g)
        floor.attrib.update(name="floor_"+g.get("name"),contype="8",conaffinity="0")
        cube.append(floor)
        if g.get("name").startswith("cube_rubber"):
            size=np.fromstring(g.get("size"),sep=" ")
            size[:2]*=width
            floor.set("size"," ".join(map(str,size)))
            floor.set("friction",f"{mu} .005 .0001")
    return tree

def main():
    run=importlib.import_module("fork-rake")
    source=Path(sys.argv[1])
    with (run.here/"shelf-corner-replay.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow("mu pad_width_fraction dt_ms slide_2p59_mm slide_2p86_mm yaw_2p59_deg yaw_2p86_deg max_rise_mm held_rise_mm".split())
        for mu,width in [(None,1)]+[(m,b) for m in (.5,.9,1.3) for b in (1,.5,.125)]+[(m,1) for m in (.95,1.,1.05,1.1,1.2)]:
            tree=ET.parse(source) if mu is None else shelf_variant(source,mu,width)
            run.require_printed_fork(tree)
            run.add_links(tree,run.repo)
            model=mj.MjModel.from_xml_string(ET.tostring(tree.getroot(),encoding="unicode"))
            samples=[]
            def observe(m,d,t,i):
                if i%round(.005/m.opt.timestep) or t>2.88: return
                qw,qx,qy,qz=d.joint("cube_root").qpos[3:]
                yaw=np.arctan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
                samples.append([t,yaw])
            for result in run.replay(model,[0],observe=observe):
                t,y=np.array(samples).T
                y=np.degrees(np.unwrap(y))
                yaw=np.interp([2.5944,2.8593],t,y)-np.median(y[t<0])
                w.writerow([mu if mu is not None else "unsplit",width,result[0],*result[3:5],*yaw,*result[5:7]])
                samples.clear()
if __name__=="__main__":
    main()

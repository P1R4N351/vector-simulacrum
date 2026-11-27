"""Propagate the camera-to-SDK shell heading discrepancy into free-cube pickup."""
import csv
import importlib
import sys
from xml.etree import ElementTree as ET
import mujoco as mj
import numpy as np

run=importlib.import_module("fork-rake")
data=np.genfromtxt(run.here/"approach-heading.csv",names=True,delimiter=",")
times=np.unique(data["time_s"])
residual=data["shell_yaw_change_deg"]-data["sdk_yaw_change_deg"]
tree=ET.parse(sys.argv[1])
run.require_printed_fork(tree)
run.add_links(tree,run.repo)
m=mj.MjModel.from_xml_string(ET.tostring(tree.getroot(),encoding="unicode"))
with (run.here/"approach-heading-replay.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("steering sighting dt_ms pitch_offset_deg slide_cross_s slide_2p59_mm slide_2p86_mm max_rise_mm held_rise_mm tilt_3p11_deg tilt_3p38_deg tilt_3p64_deg".split())
    for label,func in [("sdk",None),("minimum",np.min),("median",np.median),("maximum",np.max)]:
        correction=None if func is None else (times,np.radians([func(residual[data["time_s"]==t]) for t in times]))
        for source,sighting in [("last",run.sights[-1]),("pre-command",run.sights[run.sights["time_s"]<0][-1])]:
            for row in run.replay(m,[0],report_damage=True,sighting=sighting,yaw_correction=correction):
                w.writerow([label,source,*row])

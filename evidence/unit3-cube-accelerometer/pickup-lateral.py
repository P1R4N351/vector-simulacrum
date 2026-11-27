"""Sensitivity to lateral fork registration at the wrist on the measured pickup path."""
import csv
import importlib
import sys
from xml.etree import ElementTree as ET

import mujoco as mj
run=importlib.import_module("fork-rake")
tree=ET.parse(sys.argv[1])
run.require_printed_fork(tree)
run.add_links(tree,run.repo)
m=mj.MjModel.from_xml_string(ET.tostring(tree.getroot(),encoding="unicode"))
with (run.here/"pickup-lateral.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow(["sighting","lateral_mm","dt_ms","pitch_offset_deg","slide_cross_s","slide_2p59_mm",
                "slide_2p86_mm","max_rise_mm","held_rise_mm","tilt_3p11_deg","tilt_3p38_deg","tilt_3p64_deg"])
    for label,s in [("last",run.sights[-1]),("pre-command",run.sights[run.sights["time_s"]<0][-1])]:
        for shift in [-2,-1,0,1,2]:
            for row in run.replay(m,[0],report_damage=True,sighting=s,lateral_mm=shift):
                w.writerow([label,shift,*row])

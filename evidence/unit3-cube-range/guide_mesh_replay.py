"""Replay five mounts with sectioned convex guides sampled from the pinned charger mesh."""
import csv
import io
import json
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from guide_sections import mesh_triangles, ray_hits
from recorded_mount import ROOT, read_trial, table


def main():
    triangles = mesh_triangles()
    export = """import {rearEnvelope} from './rear_envelope.mjs'; import {readText} from '../../web/test/helpers.js';
    process.stdout.write(rearEnvelope(readText('public/robot/vector2/vector-free-dock.xml'),'driven_rear_envelope'));"""
    model = ET.fromstring(subprocess.check_output(["node", "--input-type=module", "-e", export], cwd=ROOT, text=True))
    dock = model.find(".//body[@name='dock']")
    assets = model.find("asset")
    if assets is None:
        assets = ET.SubElement(model, "asset")
    for side, sign in [("l",1),("r",-1)]:
        original = dock.find(f"geom[@name='dock_funnel_{side}']")
        if "--box-control" in sys.argv:
            half = np.fromstring(original.get("size"), sep=" ")
            vertices = np.array([[i,j,k] for i in [-1,1] for j in [-1,1] for k in [-1,1]]) * half
            name = f"funnel_box_{side}"
            ET.SubElement(assets, "mesh", name=name, vertex=" ".join(map(str, vertices.flat)))
            original.set("type","mesh")
            original.set("mesh", name)
            del original.attrib["size"]
            continue
        for start in range(24,48,3):
            points = []
            for x in np.linspace(start, start+3, 7):
                for z in np.arange(9.5,22.1,1.25):
                    hits = ray_hits(triangles,x,z)
                    wall = sign*hits[(sign*hits>22) & (sign*hits<46)]
                    if len(wall)>=2 and np.ptp(wall)>.01:
                        points.extend([[x,sign*y,z] for y in [min(wall),max(wall)]])
            if len(points)<4 or np.linalg.matrix_rank(np.array(points)-points[0])<3:
                continue
            name = f"section_{side}_{start}"
            ET.SubElement(assets,"mesh",name=name,vertex=" ".join(f"{v*.001:.7f}" for v in np.ravel(points)))
            fields = {k:v for k,v in original.attrib.items() if k not in ["name","type","size","quat","pos"]}
            ET.SubElement(dock,"geom",name=name,type="mesh",mesh=name,**fields)
        dock.remove(original)
    trials = [read_trial(c,o)[0] for c,o in zip(table("return-scores.csv"),table("mount-target-inputs.csv"),strict=True)]
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp)/"model.xml"
        ET.ElementTree(model).write(path)
        raw = subprocess.check_output(["node",str(ROOT/"recorded_mount.mjs"),str(path)],input=json.dumps(trials),text=True)
    data=np.genfromtxt(io.StringIO(raw),names=True,delimiter=",",dtype=None,encoding="utf8")
    filename = "guide-box-control.csv" if "--box-control" in sys.argv else "guide-mesh-replay.csv"
    with (ROOT/filename).open("w") as stream:
        writer=csv.writer(stream,lineterminator="\n")
        writer.writerow("session first_contact_s out_at_recorded_on_mm lateral_mm yaw_deg".split())
        for trial in trials:
            run=data[data["session"]==trial["name"]]
            on=run["t_s"][run["on"]==1]
            writer.writerow([trial["name"],on[0] if len(on) else "",*[np.interp(0,run["t_s"],run[f]) for f in ["out_mm","lat_mm","yaw_deg"]]])


if __name__ == "__main__":
    main()

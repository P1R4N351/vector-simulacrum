"""Transfer the photo-tested fore-aft registration to the recorded acquisition."""
import csv
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from xml.etree import ElementTree as ET

import numpy as np
import free_pickup

fixture=Path(sys.argv[1]).resolve()
out=free_pickup.here
with TemporaryDirectory() as temp,(out/"flank-transfer.csv").open("w") as stream:
    temp=Path(temp)
    for filename in ("pickup-path.csv","pickup-sightings.csv","pickup.tsv","action-times.csv"):
        (temp/filename).symlink_to(out/filename)
    free_pickup.here=temp
    writer=csv.writer(stream,lineterminator="\n")
    writer.writerow(["fork_dx_mm","initialization","slide_2p86_mm","yaw_change_deg","max_center_mm","held_floor_fraction"])
    for dx in (0,-2,2):
        tree=ET.parse(fixture)
        for geom in tree.findall(".//body[@name='fork']/geom[@type='mesh']"):
            pos=np.fromstring(geom.get("pos","0 0 0"),sep=" ")
            pos[0]+=dx/1000
            geom.set("pos"," ".join(map(str,pos)))
        path=temp/"fork.xml"
        tree.write(path)
        sys.argv[1]=str(path)
        free_pickup.main()
        def read(name):
            return np.genfromtxt(temp/name,delimiter=",",names=True,encoding=None,dtype=None)
        raw=read("free-pickup.csv")
        angles=read("shelf-yaw-sim.csv")
        for name in dict.fromkeys(raw["initialization"]):
            run=raw[raw["initialization"]==name]
            p=run[np.argmin(abs(run["time_s"]-2.86))]
            yaw=angles[(angles["initialization"]==name)&np.isclose(angles["time_s"],2.86)][0]
            hold=run[(run["time_s"]>6.5)&(run["time_s"]<8.3)]
            writer.writerow([dx,name,round(np.hypot(p["cube_dx_mm"],p["cube_dy_mm"]),3),
                             yaw["yaw_change_deg"],round(max(run["center_z_mm"]),3),hold["touches_floor"].mean()])

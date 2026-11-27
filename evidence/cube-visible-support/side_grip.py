"""Change sliding friction for cube contacts with one side of the fork."""
import csv
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

repo, fixture, scene = (Path(arg).resolve() for arg in sys.argv[1:4])
sys.path.insert(0,str(repo/"evidence/unit3-printed-fork-contact"))
from evaluate import run

canonical = ET.parse(scene).find('.//body[@name="cube"]')
cases = [("Intact",0,1),("Left slippery",1,.05),("Right slippery",-1,.05),
         ("Left high friction",1,3),("Right high friction",-1,3)]
if "--neutral" in sys.argv[4:]:
    cases = [("Left neutral",1,1),("Right neutral",-1,1)]
prefix = "side-grip-controls" if "--neutral" in sys.argv[4:] else "side-grip"
fig, (a,b) = plt.subplots(1,2,figsize=(9,4),layout="constrained")
with tempfile.TemporaryDirectory() as tmp, open(prefix+".csv","w",newline="") as stream:
    writer=csv.writer(stream,lineterminator="\n")
    writer.writerow(["case","trial","right_at_peak_mm","final_right_mm","error_mm","peak_tilt_deg","changed_contact_ticks"])
    for label,side,factor in cases:
        tree=ET.parse(fixture)
        fork=tree.find('.//body[@name="fork"]')
        count=0
        for g in fork.findall("geom[@mesh]"):
            pts=tree.find(f".//asset/mesh[@name='{g.get('mesh')}']")
            y=np.fromstring(pts.get("vertex"),sep=" ").reshape(-1,3)[:,1]
            if side and min(side*y)>0:
                g.set("contype","8")
                g.set("conaffinity","0")
                count+=1
        assert count or not side
        cube=tree.find('.//body[@name="cube"]')
        cube.find("inertial").attrib.update(canonical.find("inertial").attrib)
        for old in cube.findall("geom"):
            cube.remove(old)
        for source in canonical.findall("geom"):
            g=ET.SubElement(cube,"geom",source.attrib.copy())
            g.attrib.update(contype="4",conaffinity="3")
            if side:
                copy=ET.SubElement(cube,"geom",g.attrib.copy())
                sliding,*other=g.get("friction").split()
                copy.attrib.update(name="altered_"+g.get("name"),contype="0",conaffinity="8",
                                   friction=" ".join([str(float(sliding)*factor),*other]))
        path=Path(tmp)/"fixture.xml"
        tree.write(path)
        sys.argv[2]=str(path)
        errors=[]
        for trial in range(1,6):
            touched=[0]
            def inspect(m,d,t,l):
                if any(m.geom(i).name.startswith("altered_") for c in d.contact for i in c.geom):
                    touched[0]+=1
            r,tr=run((trial,"stl","overhead","weld",0,0,0),inspect,observer_stride=10,rails=True)
            tr=np.asarray(tr)
            right=np.interp(5.498,tr[:,0]-1.5,-tr[:,2])
            writer.writerow((label,trial,right,-r[9],r[-3],r[-1],touched[0]))
            errors.append(r[-3])
            if trial==1:
                a.plot(tr[:,0]-1.5,-tr[:,2],label=label)
        b.plot(range(2,6),errors[1:],"o-")
a.vlines(5.498,8.63,14.64,color="black",lw=3,label="Physical peak range")
a.scatter([12],[26.72],marker="*",color="black",label="Physical settled¹")
a.set(xlabel="Seconds from drive onset (¹settled shown at right)",ylabel="Rightward travel (mm)",xlim=(3,12.5))
a.legend(fontsize=7)
b.set(xlabel="Recorded push",ylabel="Settled endpoint error (mm)",xticks=range(2,6))
fig.savefig(prefix+".png",dpi=180)

"""Replay recorded pushes. Arguments: repository root, printed fork fixture XML."""
from pathlib import Path
import csv
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

repo, fork_path = (Path(arg).resolve() for arg in sys.argv[1:3])
sys.path.insert(0,str(repo / "evidence/unit3-printed-fork-contact"))
from evaluate import run

scene = ET.parse(sys.argv[3] if len(sys.argv) > 3 else repo / "web/public/robot/vector2/vector-free-dock.xml")
canonical = scene.find('.//body[@name="cube"]')
with tempfile.TemporaryDirectory() as tmp, open("push-width.csv","w",newline="") as stream:
    writer = csv.writer(stream,lineterminator="\n")
    writer.writerow(["shell_mm","rubber_mm","trial","forward_mm","left_mm","endpoint_error_mm","peak_tilt_deg"])
    for shell in (45.,43.5):
        for rubber in (45.30681828,44.7):
            tree=ET.parse(fork_path)
            body=tree.find('.//body[@name="cube"]')
            body.find("inertial").attrib.update(canonical.find("inertial").attrib)
            for old in body.findall("geom"):
                body.remove(old)
            for source in canonical.findall("geom"):
                g=ET.SubElement(body,"geom",source.attrib.copy())
                g.set("contype","4")
                g.set("conaffinity","3")
                p,h=[np.fromstring(g.get(k),sep=" ")*1000 for k in ("pos","size")]
                is_pad=g.get("name").startswith("cube_rubber")
                old_half=22.65340914 if is_pad else 22.5
                new_half=(rubber if is_pad else shell)/2
                # Keep interior features through 21 mm, including the pickup pocket.
                for axis in (0,1):
                    edges=p[axis]+np.array([-1,1])*h[axis]
                    ends=np.sign(edges)*np.interp(abs(edges),[0,21,old_half],[0,21,new_half])
                    p[axis],h[axis]=np.mean(ends),np.diff(ends)[0]/2
                for k,v in (("pos",p),("size",h)):
                    g.set(k," ".join(map(str,v/1000)))
            variant=Path(tmp)/"fixture.xml"
            tree.write(variant)
            sys.argv[2]=str(variant)
            for trial in range(1,6):
                r,_=run((trial,"stl","overhead","weld",0,0,0),rails=True)
                writer.writerow((shell,rubber,trial,r[8],r[9],r[-3],r[-1]))
                stream.flush()

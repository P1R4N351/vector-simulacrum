"""Create a diagnostic R5 fixture with only the first loaded hard corner trimmed."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

p=argparse.ArgumentParser()
p.add_argument("source",type=Path)
p.add_argument("dest",type=Path)
p.add_argument("inset_mm",type=float,choices=(0.5,1.0))
args=p.parse_args()
tree=ET.parse(args.source)
g=tree.find(".//body[@name='cube']/geom[@name='cube_shell_12']")
pos,half=[np.fromstring(g.get(k),sep=" ") for k in ("pos","size")]
np.testing.assert_allclose(pos[:2],[-.018,.018])
np.testing.assert_allclose(half[:2],[.004,.004])
# Trim the two outward faces, retaining the original inward faces and heights.
trim=.0005*args.inset_mm
pos[:2]-=np.sign(pos[:2])*trim
half[:2]-=trim
for key,values in (("pos",pos),("size",half)):
    g.set(key," ".join(map(str,values)))
tree.write(args.dest)

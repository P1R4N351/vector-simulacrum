"""Sensitivity test of the thicker center web inferred from the front aperture."""
import csv
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET
import numpy as np
from evaluate import here,run

with tempfile.TemporaryDirectory() as tmp, (here/'aperture-replay.csv').open('w',newline='') as f:
    source=Path(sys.argv[2]).resolve()
    tree=ET.parse(source)
    for geom in tree.findall(".//body[@name='cube']/geom"):
        p=np.fromstring(geom.get('pos'),sep=' ')*1000
        h=np.fromstring(geom.get('size'),sep=' ')*1000
        if abs(p[2])+h[2]>8.82:
            continue
        for axis in (0,1):
            bounds=np.array([p[axis]-h[axis],p[axis]+h[axis]])
            ends=np.sign(bounds)*np.interp(abs(bounds),[0,12.47,22],[0,14,22])
            p[axis],h[axis]=ends.mean(),np.diff(ends)[0]/2
        geom.set('pos',' '.join(map(str,p/1000)))
        geom.set('size',' '.join(map(str,h/1000)))
    variant=Path(tmp)/'wide.xml'
    tree.write(variant)
    w=csv.writer(f,lineterminator='\n')
    w.writerow(('trial','profile','endpoint_error_mm','peak_tilt_deg','left_mm'))
    for trial in range(1,6):
        for name,path in [('original',source),('wider web',variant)]:
            sys.argv[2]=str(path)
            r,_=run((trial,'stl','overhead','weld',0,0,0),rails=True)
            w.writerow((trial,name,r[-3],r[-1],r[9]))

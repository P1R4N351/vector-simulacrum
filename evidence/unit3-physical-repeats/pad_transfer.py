"""Transfer the E candidate to the five previously recorded shelf pushes."""
import csv
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET
import numpy as np

here=Path(__file__).resolve().parent
repo=here.parents[1]
fixture=Path(sys.argv[1]).resolve()
sys.path.insert(0,str(here.parent/'unit3-printed-fork-contact'))
sys.argv=[sys.argv[0],str(repo),str(fixture)]
from evaluate import run

with tempfile.TemporaryDirectory() as temp, (here/'E/pad-transfer.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(['shelf_ms','trial','endpoint_error_mm','cube_yaw_deg','camera_yaw_deg','max_tilt_deg'])
    for ms in (8,32):
        tree=ET.parse(fixture)
        pair=ET.SubElement(tree.getroot(),'contact')
        for pad in tree.findall(".//body[@name='cube']/geom"):
            if pad.get('name').startswith('cube_rubber_'):
                ET.SubElement(pair,'pair',geom1='table',geom2=pad.get('name'),condim='3',
                              friction='.9 .9 .005 .0001 .0001',solref=f'{ms*.001} 1')
        path=Path(temp)/f'pad-{ms}.xml'
        tree.write(path)
        sys.argv[2]=str(path)
        for trial in range(1,6):
            result,_=run((trial,'stl','overhead','weld',0,0,0))
            w.writerow([ms,trial,result[-3],result[-5],result[-4],result[-1]])

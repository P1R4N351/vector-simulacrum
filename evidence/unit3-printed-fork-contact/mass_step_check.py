"""Check step-size sensitivity of the uniform 50 g and 85 g trial 1 controls."""
import csv
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from evaluate import here,run

source=Path(sys.argv[2]).resolve()
with tempfile.TemporaryDirectory() as folder,(here/'mass-step-check.csv').open('w',newline='') as f:
    fixture=Path(folder)/'step.xml'
    w=csv.writer(f,lineterminator='\n')
    w.writerow(('mass_g','step_ms','common_20ms_peak_tilt_deg','endpoint_error_mm'))
    for grams in (50,85):
        for divisions in (1,2,4):
            tree=ET.parse(source)
            tree.find('option').set('timestep',str(.001/divisions))
            body=tree.find(".//body[@name='cube']/inertial")
            body.set('mass',str(grams/1000))
            body.set('diaginertia',' '.join([str(grams/1000*.044**2/6)]*3))
            tree.write(fixture)
            sys.argv[2]=str(fixture)
            result,trace=run((1,'stl','overhead','weld',0,0,0),rails=True)
            peak=max(row[-1] for row in trace[:-1:divisions])
            w.writerow((grams,1/divisions,peak,result[-3]))

"""Score camera-plus-odometry endpoints using P2Y8's reported intrinsics."""
import csv
import json
from pathlib import Path
import sys
import numpy as np
here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parent/'p2y8-cube-contact-replay'))
import replay
measurements = here.parent/'p2y8-paired-cube-push'
observed = np.genfromtxt(measurements/'comparison.csv', names=True, delimiter=',')
picks = json.loads((measurements/'endpoint-picks.json').read_text())
cal = json.loads((here.parents[1]/'web/camera_calibration_vector2.json').read_text())
replay.K[:2] = [[cal['fx_px'],0,cal['cx_px']], [0,cal['fy_px'],cal['cy_px']]]
with (here/'camera-endpoints.csv').open('w') as fp:
    output = csv.writer(fp, lineterminator='\n')
    output.writerow(['trial','on_forward','on_left','over_forward','over_left','vector_error_mm'])
    for i, row in enumerate(observed,1):
        points = []
        for pick in (p for p in picks if p['trial']==i and p['camera']=='on'):
            state = pick['state']
            center, _ = replay.start_pose(pick)
            points.append(replay.rotation(np.deg2rad(state['yaw'])) @ center + state['xy'])
        delta = points[1]-points[0]
        target = np.array([row['over_forward'],row['over_left']])
        output.writerow([i,*delta,*target,np.linalg.norm(delta-target)])

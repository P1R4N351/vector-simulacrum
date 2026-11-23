"""Score the raised restart on the next overhead exposure in native crop pixels."""
import csv
import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
from adjacent_face import checks, edge, folder, measure, project_face, residual_segment
from evaluate import run, here
from front_edge import basis, origin
from optical_clearance import reconstruct, rows, quads
import mujoco as mj
times = rows['sec']
start, end = [int(np.argmin(abs(times-t))) for t in (5.498, 5.998)]

reference = np.median(quads[times < 0], axis=0)
stamp, _, a, b = checks[1]
image = cv2.imread(str(folder / (stamp + '.png')))
observed = measure(image, a, b)
points = np.linspace(*edge, 25)
fig, ax = plt.subplots(figsize=(7, 5))
ax.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
ax.plot(*observed.T, color='white', lw=4, label='Measured side border')
scores = []

def score(label, rot, delta):
    line = project_face(reference, -1.499, 473.11, rot.apply(points) + delta)
    errors = np.min([np.linalg.norm(residual_segment(observed, np.array(pair)), axis=1)
                     for pair in zip(line[:-1], line[1:])], axis=0)
    scores.append((label, len(errors), np.median(errors)))
    ax.plot(*line.T, label=f'{label} ({np.median(errors):.1f} px)')

for shelf in (False, True):
    poses = reconstruct(-1.499, 473.11, shelf)
    name = 'Shelf' if shelf else 'Free'
    result = []
    initialized = []
    def observe(m, d, t, lift):
        if t < times[start] or result:
            return
        joint = d.joint('cube_root')
        if not initialized:
            initialized.append(t)
            rot, delta, _ = poses[start]
            joint.qpos[:] = np.r_[(origin + basis.apply(delta))/1000,
                                  (basis*rot).as_quat(scalar_first=True)]
            joint.qvel[:] = 0
            d.qacc_warmstart[:] = 0
            mj.mj_forward(m, d)
        if t >= times[end]:
            result.append(joint.qpos.copy())
    run((1,'stl','overhead','weld',0,0,0), observe, observer_stride=1, rails=True)
    p, = result
    score(name + ' restart', basis.inv()*Rotation.from_quat(p[3:], scalar_first=True),
          basis.inv().apply(1000*p[:3]-origin))
    score(name + ' optical', *poses[end][:2])

ax.set(xlim=(65,310), ylim=(310,55), title='Lowering at 5.998 s')
ax.legend(fontsize=8)
fig.tight_layout()
fig.savefig(here/'restart-pixels.png', dpi=160)
with (here/'restart-pixels.csv').open('w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(('prediction','observed_columns','median_segment_error_px'))
    writer.writerows(scores)

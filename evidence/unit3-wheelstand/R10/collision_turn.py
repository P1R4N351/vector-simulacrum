"""Resolve the endpoint square ambiguity using the side-face transition in camera A."""
import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt
from planar_slide import HERE, VIEWS, estimate


def main():
    cap = cv2.VideoCapture(str(HERE/'paired.mp4'))
    fig = plt.figure(figsize=(10, 6.3), layout='constrained')
    grid = fig.add_gridspec(3, 4, height_ratios=[1, 1, 1.3])
    # Consecutive 12 fps frames straddling the first change of visible side.
    for ix, frame in enumerate(range(75, 83)):
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame)
        ok, image = cap.read()
        assert ok
        ax = fig.add_subplot(grid[ix//4, ix%4])
        ax.imshow(image[107:264, 355:540, ::-1])
        ax.set_title(f'{frame/12-2:+.2f} s')
        ax.axis('off')
    cap.release()
    cube = np.genfromtxt(HERE/'cube.csv.gz', names=True, delimiter=',')
    with (HERE/'actions.csv').open() as f:
        dock = float(next(r['sec'] for r in csv.DictReader(f) if r['action']=='dock' and r['kind']=='begin'))
    t = cube['sec']-dock
    g = np.linalg.norm(np.column_stack([cube[k] for k in 'xyz']), axis=1)/8192
    ax = fig.add_subplot(grid[2, :])
    ax.scatter(t, g, s=6, color='#233b50')
    for i, tt in enumerate(t[1:][np.diff(cube['tap']) != 0]):
        ax.axvline(tt, color='#a54f20', alpha=.6, label='Tap counter advance' if i==0 else None)
    ax.set(xlim=(3.8,5.65), xlabel='Seconds from dock command', ylabel='Cube acceleration (g)')
    ax.legend()
    fig.savefig(HERE/'collision-turn.png', dpi=150)
    rng = np.random.default_rng(10)
    with (HERE/'collision-turn.csv').open('w') as f:
        w = csv.writer(f, lineterminator='\n')
        w.writerow(['view','ccw_deg','pick_p05_deg','pick_p95_deg'])
        for name, offset, _, coords in VIEWS:
            p = np.array(coords)/3 + offset
            # Positive angles in the picked (right, near) basis rotate clockwise
            # from above. The original near face swings toward image right and
            # its left neighbor takes its place, selecting the -90 degree branch.
            yaw = 90-estimate(p)[1]
            sampled = [90-estimate(p+rng.normal(0,.75,p.shape))[1] for _ in range(160)]
            # Recenter the periodic perturbations on the selected physical branch.
            sampled = yaw+(np.array(sampled)-yaw+45)%90-45
            row = [name, yaw, *np.percentile(sampled,[5,95])]
            w.writerow(row)
            print(row)


if __name__=='__main__':
    main()

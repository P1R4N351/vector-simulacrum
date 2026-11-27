"""Measure fork-post descent at encoder-defined heights in the published stereo crops."""
import csv
from pathlib import Path
import cv2
import matplotlib.pyplot as plt
import numpy as np


def rows(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def displacement(ref, frame, x):
    patch = ref[54:75, x-12:x+13]
    scores = cv2.matchTemplate(frame[30:99, x-19:x+20], patch, cv2.TM_CCOEFF_NORMED)
    _, score, _, (u, v) = cv2.minMaxLoc(scores)
    if u in (0, 14) or v in (0, 48) or score < .85:
        return np.nan
    a, b, c = scores[v-1:v+2, u]
    return v - 24 + (.5*(a-c)/(a-2*b+c) if a-2*b+c else 0)


def measure(folder):
    sdk = rows(folder/'sdk.csv')
    states = [r for r in sdk if r['event'] == 'state']
    t, height, flag = (np.array([float(r[key]) for r in states])
                       for key in ('relative_s', 'lift_mm', 'in_position'))
    cam = np.array([float(r['relative_s']) for r in rows(folder/'camera.csv')])
    video = cv2.VideoCapture(str(folder/'lift.mp4'))
    frames = []
    while True:
        ok, frame = video.read()
        if not ok:
            break
        frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    video.release()
    assert len(cam) == len(frames)
    commands = [r for r in sdk if r['event'] == 'command' and r['target'] == '52']
    summary = rows(folder/'summary.csv')
    assert len(commands) == len(summary)
    for n, (cmd, result) in enumerate(zip(commands, summary), 1):
        start = float(cmd['relative_s'])
        first = np.flatnonzero((t >= start) & (flag == 0))[0]
        ready = first + np.flatnonzero(flag[first:] == 1)[0]
        ref = frames[np.argmin(abs(cam - (t[ready]+.2)))]
        for plane in (54, 52):
            row = dict(session=folder.name, lower=n, plane_mm=plane,
                       duration_s=cmd.get('duration_s') or .6,
                       recovery_mm=float(result['encoder_recovery_mm']),
                       crossing_s=np.nan, interval_s=np.nan, left_px_s=np.nan, right_px_s=np.nan)
            candidates = np.flatnonzero((height[:-1] > plane) & (height[1:] <= plane)
                                        & (t[1:] >= start) & (t[1:] <= t[ready]))
            if len(candidates):
                i = candidates[0]
                at = t[i]+(plane-height[i])/(height[i+1]-height[i])*(t[i+1]-t[i])
                j = np.searchsorted(cam, at)
                assert 0 < j < len(cam)
                dt = cam[j]-cam[j-1]
                row.update(crossing_s=at-start, interval_s=dt)
                for x, key in ((209,'left_px_s'), (295,'right_px_s')):
                    row[key] = (displacement(ref,frames[j],x)-displacement(ref,frames[j-1],x))/dt
            yield row


if __name__ == '__main__':
    data = [r for name in ('recovery','height-crossover','speed-crossover') for r in measure(Path('..')/name)]
    with open('crossings.csv','w') as f:
        writer = csv.DictWriter(f, data[0], lineterminator='\n')
        writer.writeheader()
        writer.writerows(data)
    fig, axes = plt.subplots(1,2,figsize=(10,4),sharey=True,layout='constrained')
    for ax, plane in zip(axes,(54,52)):
        for name, marker in [('recovery','o'), ('height-crossover','s'), ('speed-crossover','^')]:
            points = [r for r in data if r['session']==name and r['plane_mm']==plane
                      and np.isfinite([r['left_px_s'],r['right_px_s']]).all()]
            for r in points:
                ax.plot([r['left_px_s'],r['right_px_s']], [r['recovery_mm']]*2, color='gray', lw=1)
            ax.scatter([(r['left_px_s']+r['right_px_s'])/2 for r in points],
                       [r['recovery_mm'] for r in points],label=name,marker=marker)
        ax.set(title=f'{plane} mm crossing', xlabel='Apparent downward speed (px/s)')
    axes[0].set_ylabel('Encoder recovery (mm)')
    axes[1].legend()
    fig.savefig('crossings.png', dpi=160)

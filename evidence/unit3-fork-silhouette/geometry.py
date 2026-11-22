"""Project the registered fork STL and measure the crossbar in two native crops."""
import json
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
camera = json.loads((ROOT / "web/camera_calibration_vector2.json").read_text())
triangles = np.fromfile(
    ROOT / "web/public/robot/sdk/print/forks7.stl",
    offset=84, dtype=[("normal", "<f4", 3), ("tri", "<f4", (3, 3)), ("attr", "<u2")]
)["tri"].astype(float)
x, y, z = triangles.transpose(2, 0, 1)
mesh = np.stack((-z - 22.52, -x - 48.01, y + 8.29), axis=-1)


def silhouette(height, head, pitch_delta=0, center_z_delta=0, extra_reach_mm=0, wrist_pitch_deg=0, wrist_roll_deg=0, *, fork_mesh=mesh):
    pitch = head - np.deg2rad(4) + pitch_delta
    c, s = np.cos(head), np.sin(head)
    camera_center = [-13 + 18.84*c + 7.96*s, 0, 49 + 18.84*s - 7.96*c + center_z_delta]
    wrist = [extra_reach_mm - 41 + np.sqrt(66**2 - (height-45)**2), 0, height]
    angle = np.deg2rad(wrist_pitch_deg)
    ca, sa = np.cos(angle), np.sin(angle)
    roll = np.deg2rad(wrist_roll_deg)
    cr, sr = np.cos(roll), np.sin(roll)
    rolled = fork_mesh @ np.array([[1, 0, 0], [0, cr, sr], [0, -sr, cr]])
    rel = rolled @ np.array([[ca, 0, -sa], [0, 1, 0], [sa, 0, ca]]) + wrist - camera_center
    depth = rel @ [np.cos(pitch), 0, np.sin(pitch)]
    assert depth.min() > 0
    u = camera["cx_px"] - camera["fx_px"] * rel[:, :, 1] / depth
    v = camera["cy_px"] + camera["fy_px"] * (rel @ [np.sin(pitch), 0, -np.cos(pitch)]) / depth
    mask = np.zeros((600, 800), dtype=np.uint8)
    for tri in np.stack((u, v), axis=-1).round().astype(np.int32):
        cv2.fillConvexPoly(mask, tri, 1)
    return mask


def edges(mask):
    rows = np.where(mask[:, 570])[0]
    return rows[0], rows[-1]


if __name__ == "__main__":
    fig, axes = plt.subplots(1, 2, figsize=(6, 5))
    for ax, trial, lift in zip(axes, (5, 4), (66.160782, 67.467987)):
        frame = cv2.imread(str(HERE / f"frame{trial}.png"))
        profile = np.median(frame[:, 40:81].mean(axis=2), axis=1)
        observed = np.flatnonzero(profile[230:295] > 0.5*(np.median(profile[230:245])+np.median(profile[280:290])))[0]+230
        head = -0.212401658
        if trial == 5:
            candidates = np.arange(60, 63, 0.05)
            best = min(candidates, key=lambda h: abs(edges(silhouette(h, head))[1] - observed))
            scale = (best-32)/(lift-32)
        ax.imshow(frame[:, :, ::-1], extent=(509.5, 629.5, 319.5, -0.5))
        for value, label, color in [(lift, 'Original', '#ffc857'), (32+(lift-32)*scale, 'Trial 5 fit', '#35cbc2')]:
            mask = silhouette(value, head)
            ax.contour(np.arange(510,630), np.arange(320), mask[:320,510:630], levels=[0.5], colors=[color])
            print(trial, label, value, edges(mask), 'observed lower edge', observed)
        ax.set(title=f'Trial {trial}', xlabel='Native column (px)', ylabel='Native row (px)')
    print('Scale', scale)
    fig.text(.5,.015,'Gold = original     Teal = excursion fitted on trial 5',ha='center')
    fig.tight_layout(rect=[0,.045,1,1])
    fig.savefig(HERE/'overlay.png',dpi=160)

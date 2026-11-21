"""Score the lowered fork's right inner edge at four native image rows."""
import csv
import json
from pathlib import Path
import cv2
import matplotlib.pyplot as plt
import numpy as np
from geometry import silhouette

HERE = Path(__file__).resolve().parent / "low"
X, Y = 540, 310
ROWS = (335, 360, 385, 410)


def main():
    poses = json.loads((HERE / "poses.json").read_text())
    fig, axes = plt.subplots(1, len(poses), figsize=(13, 3.8), sharey=True)
    with (HERE / "edges.csv").open("w") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(["trial", "row", "optics", "reach_mm", "observed_x", "predicted_x"])
        for pose, ax in zip(poses, axes):
            trial = pose["trial"]
            img = cv2.imread(str(HERE / f"trial{trial}.png"), cv2.IMREAD_UNCHANGED) / 2
            ax.imshow(img, cmap="gray", vmin=0, vmax=np.percentile(img, 97),
                      extent=(X-.5, X+149.5, Y+125.5, Y-.5))
            smooth = cv2.GaussianBlur(img, (0,0), 3)
            picks = []
            for y in ROWS:
                profile = smooth[y-Y-4:y-Y+5].mean(axis=0)
                picks.append(570 + np.argmin(np.gradient(profile)[570-X:661-X]))
            ax.scatter(picks, ROWS, s=16, color="white", edgecolor="black", zorder=4)
            for optics, pitch, dz in (("original", 0, 0), ("shelf", 5.16, -.52)):
                for reach, color in ((0, "#ffa600"), (2, "#00cdbb")):
                    mask = silhouette(pose["lift"], pose["head"], np.deg2rad(pitch), dz, reach)
                    for y, pick in zip(ROWS, picks):
                        predicted = 500 + np.flatnonzero(mask[y,500:])[0]
                        writer.writerow([trial,y,optics,reach,pick,predicted])
                    if optics == "shelf":
                        ax.contour(np.arange(X,X+150), np.arange(Y,Y+126),
                                   mask[Y:Y+126,X:X+150], [.5], colors=[color])
            ax.set(title=f"Trial {trial}", xlabel="Native column")
    axes[0].set_ylabel("Native row")
    fig.suptitle("Lowered fork: measured inner edge (dots), nominal reach (orange), +2 mm (teal)")
    fig.tight_layout()
    fig.savefig(HERE / "overlay.png", dpi=160)


if __name__ == "__main__":
    main()

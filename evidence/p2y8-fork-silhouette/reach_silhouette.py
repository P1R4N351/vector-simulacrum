"""Condition on the lower edge and score the independent top edge of the crossbar."""
import csv
import matplotlib.pyplot as plt
import numpy as np
import cv2
from geometry import HERE, silhouette, edges

HEAD = -0.212401658


def boundaries(frame, fraction):
    profile = np.median(frame[:, 40:81].mean(axis=2), axis=1)
    dark = np.median(profile[130:230])
    upper = dark + fraction * (np.median(profile[20:50]) - dark)
    lower = dark + fraction * (np.median(profile[280:290]) - dark)
    return (50 + np.flatnonzero(profile[50:120] < upper)[0],
            230 + np.flatnonzero(profile[230:295] > lower)[0])


def matched(target, offset, pitch, dz, roll=0):
    low, high = 52., 69.
    for _ in range(12):
        h = (low + high) / 2
        mask = silhouette(h, HEAD, np.deg2rad(pitch), dz, offset, wrist_roll_deg=roll)
        if edges(mask)[1] > target:
            low = h
        else:
            high = h
    assert abs(edges(mask)[1] - target) <= 1
    return h, mask


if __name__ == "__main__":
    fig, axes = plt.subplots(1, 2, figsize=(7, 5), sharey=True)
    with (HERE / "reach-silhouette.csv").open("w") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(["trial", "edge_fraction", "pitch_correction_deg", "height_correction_mm",
                         "extra_reach_mm", "matched_lift_mm", "observed_top_px", "observed_bottom_px",
                         "model_top_px", "model_bottom_px"])
        for trial, ax in zip((4, 5), axes):
            frame = cv2.imread(str(HERE / f"frame{trial}.png"))
            ax.imshow(frame[:, :, ::-1], extent=(509.5, 629.5, 319.5, -.5))
            for frac in (.35, .5, .65):
                top, bottom = boundaries(frame, frac)
                for pitch, dz in ((0, 0), (5.16, -.52)):
                    for offset, color in ((0, "#ffa600"), (2, "#00cdbb")):
                        h, mask = matched(bottom, offset, pitch, dz)
                        writer.writerow([trial, frac, pitch, dz, offset, h, top, bottom, *edges(mask)])
                        if frac == .5 and pitch == 0:
                            ax.contour(np.arange(510,630), np.arange(320), mask[:320,510:630],
                                       levels=[.5], colors=[color])
                if frac == .5:
                    ax.scatter([570,570], [top,bottom], s=20, color="white", edgecolors="black")
            ax.set(title=f"P2Y8 trial {trial}", xlabel="Native column (px)")
    axes[0].set_ylabel("Native row (px)")
    fig.suptitle("Fork silhouettes aligned at the measured lower edge")
    fig.text(.5, .015, "Orange: nominal reach     Teal: +2 mm     Dots: measured edges", ha="center")
    fig.tight_layout(rect=[0,.045,1,1])
    fig.savefig(HERE / "reach-silhouette.png", dpi=160)

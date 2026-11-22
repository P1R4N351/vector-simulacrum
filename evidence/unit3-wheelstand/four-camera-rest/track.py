"""Measure stationary single-template tracker repeatability in processed shelf views."""
import csv
from pathlib import Path
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "twin-timing"))
from shelf_view import SERIALS, ShelfView

# Points in the processed view. A sees the crown of the cube, B and Arducam see a side.
ROIS = {"A": (738, 272, 129, 60), "B": (318, 168, 63, 58), "Arducam": (781, 75, 79, 91)}


def process_arducam(im):
    gains = np.clip(np.median(im) / np.maximum(np.median(im, axis=(0, 1)), 1), .8, 1.25)
    ycc = cv2.cvtColor(np.uint8(np.clip(im*gains, 0, 255)), cv2.COLOR_BGR2YCrCb)
    lum = cv2.bilateralFilter(ycc[:, :, 0], 5, 9, 1.2)
    ycc[:, :, 0] = cv2.addWeighted(lum, 1.3, cv2.GaussianBlur(lum, (0, 0), 1), -.3, 0)
    upright = cv2.rotate(cv2.cvtColor(ycc, cv2.COLOR_YCrCb2BGR), cv2.ROTATE_180)
    return upright[int(.69 * upright.shape[0]):]


def position(search, template):
    scores = cv2.matchTemplate(search, template, cv2.TM_CCOEFF_NORMED)
    _, peak, _, (x, y) = cv2.minMaxLoc(scores)
    assert 0 < x < scores.shape[1]-1 and 0 < y < scores.shape[0]-1
    def vertex(a, b, c):
        return .5 * (a-c) / (a-2*b+c)
    return x+vertex(*scores[y, x-1:x+2]), y+vertex(*scores[y-1:y+2, x]), peak


def main(folder):
    fig, axes = plt.subplots(2, 3, figsize=(10, 5.5), layout="constrained")
    rows = []
    crops = {}
    archive = None if folder else np.load(HERE / "glyph-crops.npz")
    for col, (name, (x, y, w, h)) in enumerate(ROIS.items()):
        stream = cv2.VideoCapture(str(folder / (name+".nut"))) if folder else None
        crops[name] = []
        for i in range(100):
            if stream is not None:
                ok, raw = stream.read()
                assert ok
                if i == 0 and name in ("A","B"):
                    view = ShelfView(SERIALS[col], raw)
                frame = process_arducam(raw) if name == "Arducam" else view.apply(raw)
                patch = frame[y-8:y+h+8, x-8:x+w+8]
                gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
                crops[name].append(gray.copy())
            else:
                gray = archive[name][i]
            if i == 0:
                template = gray[8:-8,8:-8].copy()
                bx, by, _ = position(gray, template)
                axes[0,col].imshow(gray, cmap="gray", vmin=0, vmax=255)
                axes[0,col].set_title(name)
                axes[0,col].axis("off")
            px, py, score = position(gray, template)
            rows.append(dict(camera=name, frame=i, dx_px=px-bx, dy_px=py-by, ncc=score))
        if stream is not None:
            stream.release()
        data = [r for r in rows if r["camera"]==name]
        ax = axes[1,col]
        for k in ("dx_px","dy_px"):
            ax.plot([r[k] for r in data], label=k[:2])
        ax.set(xlabel="Frame", ylabel="Apparent shift (px)", ylim=(-.5,.5))
        ax.legend()
        print(name, np.percentile([np.hypot(r["dx_px"], r["dy_px"]) for r in data[1:]],95),
              min(r["ncc"] for r in data))
    if folder:
        np.savez_compressed(HERE / "glyph-crops.npz", **crops)
    fig.savefig(HERE / "glyph-repeatability.png", dpi=150)
    with (HERE / "glyph-offsets.csv").open("w") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0], lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main(Path(sys.argv[1]) if len(sys.argv)>1 else None)

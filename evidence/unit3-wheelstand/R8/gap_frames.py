"""Inspect camera frames inside the R8 cube packet gap, including a fixed dock control."""
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
gap = (4.327100, 4.598888)
clock = (59.754, 59.875)


def read_frames(name, indices):
    capture = cv2.VideoCapture(str(here / name))
    frames = []
    for index in indices:
        capture.set(cv2.CAP_PROP_POS_FRAMES, index)
        ok, frame = capture.read()
        if not ok:
            raise ValueError(f'Missing {name} frame {index}')
        frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    capture.release()
    return frames


def main():
    frames = np.arange(70, 81)
    times = 58 + frames / 12
    inside = frames[(times - clock[1] > gap[0]) & (times - clock[0] < gap[1])]
    print('Frames strictly within the gap for all clock offsets:', inside)
    assert list(inside) == [75, 76]
    crops = read_frames('cube-settling.mkv', inside)
    full = read_frames('roll.mp4', inside + 48)  # Clip begins at 54 s.
    # Compare the fixed dock glyph with the moving fork/cube silhouette.
    dock = [f[58:101, 267:317] for f in full]
    contact = [f[5:32, 16:56] for f in crops]
    fig, axes = plt.subplots(2, 3, figsize=(8, 4.6), layout='constrained')
    for row, (name, pair) in enumerate([('Fork and cube', contact), ('Dock glyph', dock)]):
        diff = cv2.absdiff(*pair)
        print(f'{name}: mean absolute change {diff.mean():.2f}/255')
        for col, pixels in enumerate([*pair, diff]):
            ax = axes[row, col]
            ax.imshow(pixels, cmap='gray' if col < 2 else 'inferno',
                      vmin=0, vmax=255, interpolation='nearest')
            ax.set_xticks([])
            ax.set_yticks([])
            if row == 0:
                ax.set_title(['64.250 s', '64.333 s', 'Absolute change'][col])
        axes[row, 0].set_ylabel(name)
    fig.savefig(here / 'gap-frames.png', dpi=160)


if __name__ == '__main__':
    main()

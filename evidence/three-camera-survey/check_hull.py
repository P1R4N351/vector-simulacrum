"""Reproject the calibrated voxel envelope and score the six reserved silhouettes."""

from pathlib import Path

import cv2
import numpy as np


def main():
    views = np.load(Path(__file__).with_name("orbit-views.npz"))
    masks = np.unpackbits(views["masks"], axis=2) * 255
    axes = [np.arange(lo, hi, 1.5) for lo, hi in [(-56, 56), (-39, 39), (-1.5, 81)]]
    points = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)
    signed_distances = []
    rays = []
    for i, mask in enumerate(masks):
        p = points @ views["rotations"][i].T + views["translations"][i]
        uv = np.rint(p[:, :2] / p[:, 2, None] * views["focal"] + views["principal"]).astype(int)
        uv = np.clip(uv, 0, np.array(mask.shape[::-1]) - 1)
        rays.append(uv)
        if i in views["validation"]:
            continue
        distance = cv2.distanceTransform(mask, cv2.DIST_L2, 3)
        distance -= cv2.distanceTransform(255 - mask, cv2.DIST_L2, 3)
        signed_distances.append(distance[uv[:, 1], uv[:, 0]])

    # Allow two bad segmentations per voxel. Translation at each held-out
    # heading uses its centroid; yaw and the reconstructed shape are fixed.
    inside = (np.partition(signed_distances, 2, axis=0)[2] >= 0) & (points[:, 2] >= 0)
    for i in views["validation"]:
        mask = masks[i]
        predicted = np.zeros_like(mask)
        x, y = rays[i][inside].T
        predicted[y, x] = 255
        predicted = cv2.morphologyEx(predicted, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        iou = np.count_nonzero(mask & predicted) / np.count_nonzero(mask | predicted)
        print(views["labels"][i], f"{iou:.4f}")


if __name__ == "__main__":
    main()

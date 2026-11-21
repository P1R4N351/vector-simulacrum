"""Local rectification for the mounted See3CAM_24CUG and its fitted lens."""

from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from replay_camera_spins import WORLD

DATA = Path(__file__).resolve().parent / "evidence/heading-crossover"
GLYPH = np.array(
    [[437.5, 84.5], [506.5, 84.5], [507.5, 150.5], [438.5, 150], [466, 256]], float
) + [390, 455]
SHELF = np.array([[225, 154], [729, 165], [69, 263], [882, 292]], float) + [390, 455]


def edge_errors(line, k, center, theta, intercept):
    # Signed normal distance in original sensor pixels (first-order distance).
    d = (line - center) / 960
    normal = np.array([np.cos(theta), np.sin(theta)])
    return (
        960
        * (d @ normal - intercept * (1 + k * (d * d).sum(axis=1)))
        / np.linalg.norm(normal - 2 * intercept * k * d, axis=1)
    )


def line_seed(line):
    d = (line - [960, 540]) / 960
    n = np.linalg.svd(d - d.mean(axis=0), full_matrices=False)[2][-1]
    return [np.arctan2(n[1], n[0]), d.mean(axis=0) @ n]


def lens_fit(groups, ids=(0, 1, 2)):
    def errors(q):
        chunks = [
            edge_errors(groups[i], q[0], q[1:3], *q[3 + 2 * j : 5 + 2 * j])
            for j, i in enumerate(ids)
        ]
        return np.r_[np.concatenate(chunks), (q[1:3] - [960, 540]) / 30]

    seed = [-0.75, 960, 540]
    for i in ids:
        seed.extend(line_seed(groups[i]))
    return least_squares(errors, seed, loss="soft_l1").x[:3]


def rectify(uv, lens):
    k, c = lens[0], lens[1:]
    d = uv - c
    return c + d / (1 + k * np.sum((d / 960) ** 2, axis=-1, keepdims=True))


def distort(uv, lens):
    k, c = lens[0], lens[1:]
    d = uv - c
    return c + 2 * d / (
        1 + np.sqrt(1 - 4 * k * np.sum((d / 960) ** 2, axis=-1, keepdims=True))
    )


def image_points(xyz, pose, lens):
    ray = Rotation.from_rotvec(pose[:3]).apply(xyz) + pose[3:6]
    return distort(lens[1:] + pose[-1] * ray[:, :2] / ray[:, 2, None], lens)


def locate(uv, height, pose, lens):
    rays = (rectify(np.atleast_2d(uv), lens) - lens[1:]) / pose[-1]
    inverse = Rotation.from_rotvec(pose[:3]).inv()
    origin = inverse.apply(-pose[3:6])
    direction = inverse.apply(np.c_[rays, np.ones(len(rays))])
    return (origin + (height - origin[2]) * direction / direction[:, 2, None])[:, :2]


def camera_pose(lens, glyph=GLYPH, shelf=SHELF, fprior=580, edgeweight=5):
    def errors(p):
        a, b, c, d = locate(shelf, 0, p, lens)
        across = (b - a) / np.linalg.norm(b - a)
        return np.r_[
            ((image_points(WORLD, p, lens) - glyph) / 1.5).ravel(),
            (p[-1] - fprior) / 150,
            ((c - a) @ across) / edgeweight,
            ((d - b) @ across) / edgeweight,
        ]

    return least_squares(
        errors,
        [1.8, -0.08, 0.12, -35, 71, 389, 595],
        bounds=([-7] * 3 + [-2000] * 2 + [50, 250], [7] * 3 + [2000] * 3 + [1400]),
    ).x


def edge_groups():
    data = np.loadtxt(DATA / "edge-pixels.csv", delimiter=",", skiprows=1)
    return [data[data[:, 0] == i, 1:] for i in range(7)]


if __name__ == "__main__":
    groups = edge_groups()
    lens = lens_fit(groups)
    p = camera_pose(lens)
    print("Lens and camera pose", lens, p)
    for i, l in enumerate(groups):
        med = []
        for k in (0, lens[0]):
            f = least_squares(
                lambda ab, edge=l, coefficient=k: edge_errors(
                    edge, coefficient, lens[1:], *ab
                ),
                line_seed(l),
                loss="soft_l1",
            )
            med.append(np.median(abs(f.fun)))
        print(i, np.round(med, 2))
    print(
        "Dock RMS px",
        np.sqrt(np.mean(np.sum((image_points(WORLD, p, lens) - GLYPH) ** 2, axis=1))),
    )

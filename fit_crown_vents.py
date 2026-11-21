"""Fit crown aperture placement at B and evaluate shelf spot A independently."""

import numpy as np
from scipy.optimize import least_squares

from see3cam_shelf import camera_pose, edge_groups, image_points, lens_fit, locate

lens = lens_fit(edge_groups())
pose = camera_pose(lens)
off = np.array([390, 455])

# Near wheel hub centers fix chassis heading independently of the vent measurements.
# Each tuple gives fixed strip tip, front and rear hubs, then the two end slots of each bank (aft to forward).
V = {
    "Bminus": (
        [640, 251],
        [524, 409],
        [655, 411],
        [[599, 247], [569.5, 241], [599, 259], [569.5, 251]],
    ),
    "Bminus2": (
        [589, 275],
        [451, 400],
        [578, 446],
        [[555, 258], [524, 247], [546.5, 270], [514.5, 257.5]],
    ),
    "Bplus2": (
        [512, 254],
        [562, 442],
        [439, 390],
        [[558, 260], [587, 264], [549, 269], [578.5, 275]],
    ),
    "Aminus2": (
        [430, 241],
        [321, 353],
        [430, 402],
        [[395, 227], [369, 215], [387, 236], [361, 224]],
    ),
}
head_angles = {
    "Bminus": -7.873773574829102e-05,
    "Bminus2": -0.0215253084897995,
    "Bplus2": 0.002065986394882202,
    "Aminus2": -7.873773574829102e-05,
}


def setup(lab):
    tip, front, rear, pts = V[lab]
    origin = locate([np.array(tip) + off], 65.0, pose, lens)[0]
    h = (
        locate([np.array(front) + off], 14.5, pose, lens)[0]
        - locate([np.array(rear) + off], 10.75, pose, lens)[0]
    )
    h /= np.linalg.norm(h)
    return origin, np.array([h, [-h[1], h[0]]]), np.array(pts)


xp = np.array([-31.1, -30.95, -30.1, -28.3, -25.7, -22.9, -20, -16.5, -12.8, -9.5])
heights = np.array([0.35, 6.8, 11.3, 13.6, 15.3, 16.5, 17.4, 17.9, 18.2, 18.3])
widths = np.array([0.3, 6.2, 12.1, 15.6, 17.2, 18.3, 19, 19.4, 19.6, 19.6])


def surface(x, y):
    # Analytic roof of the actual procedural head mesh, as used for the slot rims.
    a = np.pi / 2
    for _ in range(4):
        a = np.arccos(
            np.clip(
                (abs(y) / (np.interp(x, xp, widths) * (1 + 0.018 * np.sin(a) ** 0.72)))
                ** (1 / 0.64),
                0,
                1,
            )
        )
    return 49 + np.interp(x, xp, heights) * np.sin(a) ** 0.72 + 0.64


def project(lab, p):
    origin, Q, _ = setup(lab)
    head = head_angles[lab]
    d, span, bank = p
    # Both measurement rows are listed far then near in the camera.
    sign = 1 if "plus" in lab else -1
    x = -36.3 + d + np.tile([-span / 2, span / 2], 2)
    y = sign * np.repeat([bank / 2, -bank / 2], 2)
    z = surface(x, y)
    # Near-level head rotation around the pivot. At these four poses this term is < 1 pixel.
    co, si = np.cos(head), np.sin(head)
    xx = -11 + (x + 11) * co - (z - 47.5) * si
    zz = 47.5 + (x + 11) * si + (z - 47.5) * co
    xy = origin + np.c_[xx + 36.3, y] @ Q
    return image_points(np.c_[xy, zz], pose, lens) - off


def err(lab, p):
    return project(lab, p) - setup(lab)[2]


old = np.array([23.85, 12.8, 7.1])
fit = least_squares(
    lambda p: np.concatenate([err(k, p).ravel() for k in V if k[0] == "B"]),
    old,
    bounds=([12, 6, 3], [25, 15, 10]),
).x
if __name__ == "__main__":
    print("fit", fit)
    for k in V:
        print(
            k,
            [
                round(np.sqrt(np.mean(np.sum(err(k, p) ** 2, axis=1))), 2)
                for p in [old, [15.31, 8.32, 5.60]]
            ],
            "bias",
            np.mean(err(k, fit), axis=0).round(1),
        )

from pathlib import Path
import numpy as np
import cv2

here = Path(__file__).resolve().parent
p = np.genfromtxt(here / "top-picks.csv", names=True, delimiter=",")
square = 11.5 * np.array([[-1.0, -1, 0], [1, -1, 0], [1, 1, 0], [-1, 1, 0]])


def poses(rows, k, f):
    c = np.array([912.9463, 541.1996])
    K = np.array([[f, 0, c[0]], [0, f, c[1]], [0, 0, 1.0]])

    def solve(q):
        d = q + np.array([420, 485]) - c
        uv = c + d / (1 + k * np.sum((d / 960) ** 2, axis=1)[:, None])
        ok, r, t = cv2.solvePnP(square, uv, K, None, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            raise ValueError("pose solve failed")
        n = cv2.Rodrigues(r)[0][:, 2]
        proj = cv2.projectPoints(square, r, t, K, None)[0][:, 0]
        return n, t[:, 0] + 22 * n, np.sqrt(np.mean(np.sum((proj - uv) ** 2, axis=1)))

    qs = np.array([[row[x] for x in rows.dtype.names[2:]] for row in rows]).reshape(
        -1, 4, 2
    )
    n0, c0, _ = solve(np.median(qs[rows["sec"] < 0], axis=0))
    out = []
    for row, q in zip(rows, qs):
        n, center, error = solve(q)
        out.append(
            [
                row["trial"],
                row["sec"],
                np.rad2deg(np.arccos(np.clip(n @ n0, -1, 1))),
                (center - c0) @ (-n0),
                error,
            ]
        )
    return np.array(out)


if __name__ == "__main__":
    for k, f in [(-1.499, 473.11), (0, 700)]:
        print("model", k)
        for i in range(1, 6):
            r = poses(p[p["trial"] == i], k, f)
            print(i, np.round(r[(r[:, 1] > 4) & (r[:, 1] < 7), 1:], 1))

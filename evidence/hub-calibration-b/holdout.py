"""Validate a frozen B calibration on the original, untouched microphone picks."""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
NAMES = ("main", "ultra", "see1")
LABELS = ("Flip main", "Flip ultrawide", "See3CAM")
POSES = ("Bm-15", "Am-15", "S-Am-15")


def reconstruct(obs, cameras):
    """Use the survey's original perpendicular ray-distance objective."""
    used = [i for i, row in enumerate(obs) if row is not None]
    points, leave_one_out = [], {}
    pair_points = {}
    for port in range(2):
        rays = {i: ray(obs[i][port], cameras[i]) for i in used}
        point = triangulate(list(rays.values()))
        points.append(point)
        if len(used) == 3:
            for i in used:
                left = triangulate([rays[j] for j in used if j != i])
                pair = "+".join(NAMES[j] for j in used if j != i)
                pair_points.setdefault(pair, []).append(left)
                predicted = project(left, cameras[i])
                leave_one_out.setdefault(NAMES[i], []).append({
                    "projected_px": predicted.tolist(),
                    "error_px": float(np.linalg.norm(predicted - obs[i][port])),
                })
    projected = {NAMES[i]: [project(p, cameras[i]).tolist() for p in points] for i in used}
    residual = {NAMES[i]: np.linalg.norm(np.array(projected[NAMES[i]]) - obs[i], axis=1).tolist()
                for i in used}
    return {
        "chord_mm": float(np.linalg.norm(points[1] - points[0])),
        "points_mm": np.array(points).tolist(),
        "projected_px": projected,
        "error_px": residual,
        "omitted_camera": leave_one_out,
        "pair_chord_mm": {k: float(np.linalg.norm(v[1] - v[0])) for k, v in pair_points.items()},
    }


def pick_sensitivity(obs, cams):
    """Root-sum-square response to independent 1 working-pixel coordinate errors."""
    used = [i for i, row in enumerate(obs) if row is not None]
    gradients = []
    for i in used:
        for port in range(2):
            for axis in range(2):
                chord = []
                for delta in (-1, 1):
                    variant = [None if row is None else np.array(row, dtype=float).copy()
                               for row in obs]
                    variant[i][port, axis] += delta
                    chord.append(reconstruct(variant, cams)["chord_mm"])
                gradients.append((chord[1] - chord[0]) / 2)
    return float(np.linalg.norm(gradients))


def render(archive, survey_dir, observations, results, out, omit_camera=False):
    import cv2
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    sys.path.insert(0, str(survey_dir.parent.parent))
    import see3cam_shelf as see
    lens = see.lens_fit(see.edge_groups())
    tags = ("Bm-15", "S-Am-15") if omit_camera else POSES
    fig, axes = plt.subplots(len(tags), 3, figsize=(11, len(tags)*2.2+.7), layout="constrained")
    photo_labels = ("B", "A holdout", "A repeat")
    for j, tag in enumerate(tags):
        for i, (name, picks) in enumerate(zip(NAMES, observations[tag])):
            ax = axes[j, i]
            ax.set_aspect("equal")
            ax.axis("off")
            if picks is None:
                ax.text(.5, .5, "Main lens unavailable", ha="center", va="center", color="0.35")
                continue
            pred = {m: np.array([row["projected_px"] for row in
                                 results[tag][m]["omitted_camera"][name]]) if omit_camera
                       else np.array(results[tag][m]["projected_px"][name])
                    for m in ("old", "new")}
            extent = np.vstack([picks, *pred.values()]) if omit_camera else np.asarray(picks)
            center = (np.min(extent, axis=0)+np.max(extent, axis=0))/2
            width = max(80, np.ptp(extent[:, 0])+35, (np.ptp(extent[:, 1])+16)/.58)
            height = width * .58
            cols = 510
            rows = round(cols * height / width)
            yy, xx = np.indices((rows, cols), dtype=float)
            x0, y0 = center - np.array([width, height]) / 2
            xy = np.stack((x0 + (xx + .5) * width / cols,
                           y0 + (yy + .5) * height / rows), axis=-1)
            img = cv2.imread(str(archive / f"{tag}-{name}.jpg"))
            if img is None:
                raise FileNotFoundError(f"{tag}-{name}.jpg")
            if i == 2:
                img = cv2.rotate(img, cv2.ROTATE_180)
                xy = see.distort(xy + [310, 395], lens)
            else:
                xy = (xy + .5) * (img.shape[1] / 1000) - .5
            crop = cv2.remap(img, xy.astype("float32"), None, cv2.INTER_CUBIC)
            ax.imshow(crop[:, :, ::-1], extent=(x0, x0 + width, y0 + height, y0))
            for model, marker, color in [("old", "s", "#ed7598"), ("new", "o", "#23e4ce")]:
                px = pred[model]
                ax.plot(*px.T, marker=marker, color=color, mfc="none", lw=1, ms=7.5)
            ax.plot(*np.asarray(picks).T, "+", color="white", mew=2.3, ms=11)
            ax.set(xlim=(x0, x0+width), ylim=(y0+height, y0),
                   title=f"{photo_labels[POSES.index(tag)]}   ·   {LABELS[i]}")
    handles = [Line2D([], [], marker=m, color=c, markerfacecolor="none", ls=ls,
                      label=label, ms=8, lw=1.3)
               for m, c, ls, label in [("+", ".3", "", "Observed port"),
                                       ("s", "#c65278", "-", "Previous calibration"),
                                       ("o", "#008c82", "-", "48 mm B calibration")]]
    fig.legend(handles=handles, loc="outside lower center", ncol=3, frameon=False)
    title = ("Predicting the scallops in the omitted camera from the other two" if omit_camera
             else "Fixed backpack microphone scallops · same picks in both calibrations")
    fig.suptitle(title, fontsize=15)
    filename = "microphone-omitted-camera.jpg" if omit_camera else "microphone-overlay.jpg"
    fig.savefig(out / filename, dpi=165)
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--survey", type=Path, default=HERE.parent / "three-camera-survey")
    parser.add_argument("--calibration", type=Path, default=HERE / "cameras.json")
    parser.add_argument("--fixed-reference", type=Path, default=HERE / "fixed-reference.json")
    parser.add_argument("--archive", type=Path, help="Original photo archive for rendering")
    parser.add_argument("--out", type=Path, default=HERE)
    args = parser.parse_args()
    sys.path.insert(0, str(args.survey.resolve()))
    from check_survey import project, ray, triangulate
    data = json.loads((args.survey / "survey.json").read_text())
    new = json.loads(args.calibration.read_text())
    if isinstance(new, dict):
        new = new["cameras"]
    old = data["cameras"]
    observations = data["microphone_ports"]
    results = {}
    for tag in POSES:
        results[tag] = {label: reconstruct(observations[tag], cams)
                        for label, cams in [("old", old), ("new", new)]}
        for label, cams in [("old", old), ("new", new)]:
            results[tag][label]["linear_sensitivity_mm_at_one_px_per_coordinate"] = pick_sensitivity(
                observations[tag], cams)
        print(tag, *[round(results[tag][label]["chord_mm"], 3) for label in ("old", "new")])
    if args.fixed_reference.is_file():
        refs = json.loads(args.fixed_reference.read_text())
        shifts = np.array([[p["dx"], p["dy"]] for p in refs if p["camera"] == "main"
                           and p["frame"] == "S-Am-15" and p["ncc"] > .9])
        shift = np.median(shifts, axis=0)
        base = observations["S-Am-15"]
        # These uniform shifts test the sensitivity to static-background registration.
        # They do not enter the primary estimate or change any camera parameters.
        chords = [reconstruct([base[0]-s, *base[1:]], new)["chord_mm"]
                  for s in [shift, *shifts]]
        results["repeat_main_shift_sensitivity"] = {
            "median_shift_working_px": shift.tolist(), "chord_mm_median_shift": chords[0],
            "chord_mm_range_individual_patches": [min(chords[1:]), max(chords[1:])]}
    (args.out / "microphone-holdout.json").write_text(json.dumps(results, indent=2) + "\n")
    if args.archive:
        for omit_camera in (False, True):
            render(args.archive, args.survey, observations, results, args.out, omit_camera)

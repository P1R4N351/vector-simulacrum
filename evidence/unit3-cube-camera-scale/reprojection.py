"""Transfer real marker corners between settled views through the simulated camera."""
import csv
import json
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

here = Path(__file__).resolve().parent
web = here.parents[1] / "web"
cal = json.loads((web / "camera_calibration_vector2.json").read_text())
model = mujoco.MjModel.from_xml_path(str(web / "public/robot/vector2/vector-free-shelf.xml"))
data = mujoco.MjData(model)
square = 0.0115 * np.array([[-1,-1,0],[1,-1,0],[1,1,0],[-1,1,0]])
cv_axes = np.diag([1,-1,-1])


def camera_pose(sample):
    s = sample["state"]
    data.joint("root").qpos[:3] = [0.001*s["xy"][0], 0.001*s["xy"][1], 0]
    data.joint("root").qpos[3:] = Rotation.from_euler("z", s["yaw"], degrees=True).as_quat(scalar_first=True)
    data.joint("head_joint").qpos[0] = s["head"]
    mujoco.mj_forward(model, data)
    cam = data.camera("head_cam")
    return cam.xpos.copy(), cam.xmat.reshape(3,3) @ cv_axes


def predict(first, last, K):
    assert first["state"]["origin"] == last["state"]["origin"]
    xy = np.asarray(first["quad"])
    ok, r, t = cv2.solvePnP(square, xy, K, None, flags=cv2.SOLVEPNP_ITERATIVE)
    assert ok
    points = square @ cv2.Rodrigues(r)[0].T + t.ravel()
    p, R = camera_pose(first)
    q, S = camera_pose(last)
    end = (points @ R.T + p - q) @ S
    assert np.all(end[:,2] > 0)
    projected = end @ K.T
    return projected[:,:2] / projected[:,2,None]


picks = json.loads((here.parent / "unit3-paired-cube-push/endpoint-picks.json").read_text())
on = [p for p in picks if p["camera"] == "on"]
pairs = [("Reorientation",on[1],on[2])]
pairs += [(f"Rest {i}",on[2*i+1],on[2*i+2]) for i in range(1,4)]
pairs.append(("Retreat",*json.loads((here/"retreat.json").read_text())))
models = {
    "Legacy": np.array([[537.,0,400],[0,537.,300],[0,0,1]]),
    "Unit 3": np.array([[cal["fx_px"],0,cal["cx_px"]],[0,cal["fy_px"],cal["cy_px"]],[0,0,1]])
}
if __name__ == "__main__":
    rows = []
    fig, axes = plt.subplots(1,2, figsize=(8,4), layout="constrained")
    for name, a, b in pairs:
        observed = np.array(b["quad"])
        curves = {"Recorded":observed}
        for label, K in models.items():
            uv = predict(a,b,K)
            curves[label] = uv
            for i, (actual, predicted) in enumerate(zip(observed, uv)):
                rows.append([name,label,i,*actual,*predicted,np.linalg.norm(actual-predicted)])
        if name in ("Reorientation","Retreat"):
            ax = axes[int(name=="Retreat")]
            crop = plt.imread(here / f"{name.lower()}-crop.jpg")
            ax.imshow(crop, extent=(239.5,499.5,399.5,189.5))
            for (label, uv), style in zip(curves.items(),["y.-","r:","c--"]):
                ax.plot(*uv[[0,1,2,3,0]].T, style, label=label)
            ax.set(title=name, xlabel="Image x (px)", ylabel="Image y (px)", aspect="equal")
            ax.set(xlim=(240,500), ylim=(400,190))
    axes[1].legend()
    fig.savefig(here/"reprojection.png",dpi=160)
    with (here/"reprojection.csv").open("w") as f:
        w = csv.writer(f,lineterminator="\n")
        w.writerow("move,camera,corner,u_obs,v_obs,u_pred,v_pred,error_px".split(","))
        w.writerows(rows)
    for name, _, _ in pairs:
        print(name, [round(np.sqrt(np.mean([r[-1]**2 for r in rows if r[:2]==[name,label]])),2) for label in models])

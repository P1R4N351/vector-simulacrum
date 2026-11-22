"""Transfer each native camera start through approach yaw and later filmed carry."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from pickup_onset import read

here = Path(__file__).resolve().parent
starts = read(here/"sightings.csv")
raw = read(here/"approach-glyph-axes.csv")
obs = raw[raw["width_mm"]==23]
frames = read(here/"approach-links.csv")
frames = frames[(frames["link_condition"]=="Full contact") & (frames["step_ms"]==.25)]
t85,t106 = [frames["receipt_s_since_cmd"][frames["frame"]==f].item() for f in (85,106)]
height = read(here/"height-transfer.csv")
ref = starts[1]
rows=[]
for s,folder in zip(starts,sys.argv[1:],strict=True):
    d = read(Path(folder)/"pickup-replay.csv")
    yaw = read(Path(folder)/"cube-heading.csv")
    def sample(col,t):
        return np.interp(t,d["sec"],d[col])
    # Evaluate motion since this individual sighting; avoid counting SDK pose
    # jumps as physical travel. The camera stayed on the glyph at each start.
    sim_yaw = np.rad2deg(np.interp(t85,yaw["sec"],np.unwrap(yaw["yaw_rad"]))-yaw["yaw_rad"][0])
    # Outside the clip, zero denotes the precommand median registered glyph pose.
    def observed_change(trace, column):
        values = trace[column]
        return values[-1] - np.interp(s["sec"], trace["sec"], values, left=0.)
    observed_yaw = observed_change(obs,"yaw_change_deg")
    dx,dy = [sample(col,t85) for col in ("dx_mm","dy_mm")]
    cam_shift=[]
    for width in (20,23):
        trace = raw[raw["width_mm"]==width]
        axes = [observed_change(trace,col) for col in ("along_edge_mm","across_edge_mm")]
        cam_shift.append(np.linalg.norm(axes))
    heading = Rotation.from_quat([s[k] for k in ("qx","qy","qz","qw")]).as_euler("xyz",degrees=True)[2]
    rows.append([s["sec"],s["x"]-ref["x"],s["y"]-ref["y"],heading,observed_yaw,sim_yaw,
                 *cam_shift,np.hypot(dx,dy),sample("center_height_mm",t106)-sample("center_height_mm",t85)])
a=np.array(rows)
fig,axes=plt.subplots(3,1,figsize=(8,6.7),layout="constrained",sharex=True)
axes[0].scatter(a[:,0],a[:,5],label="Replay")
axes[0].scatter(a[:,0],a[:,4],label="Shelf camera")
axes[0].set_ylabel("Approach yaw (°)")
axes[1].scatter(a[:,0],a[:,8])
axes[1].vlines(a[:,0],a[:,6],a[:,7],color="C1",lw=3)
axes[1].set_ylabel("Approach travel (mm)")
axes[2].scatter(a[:,0],a[:,9])
high=height[height["frame"]==106]
axes[2].axhspan(*sorted(high["rise_mm"]),color="C1",alpha=.3,label="Filmed · two glyph scales")
axes[2].set(ylabel="Later center rise (mm)",xlabel="SDK sighting used to start replay (seconds from command)")
axes[0].legend()
axes[2].legend()
fig.savefig(here/"sighting-yaw.png",dpi=160)
with (here/"sighting-yaw.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("sighting_s start_dx_mm start_dy_mm start_yaw_deg filmed_yaw_change_deg replay_yaw_change_deg filmed_travel_20mm filmed_travel_23mm replay_travel_mm frame85_to106_replay_rise_mm".split())
    w.writerows(rows)

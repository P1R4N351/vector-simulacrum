"""Compare all three workshop visits at common physical SDK packet times."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).parent
def rows(path):
    with path.open() as f:
        return list(csv.DictReader(f))

summary = []
for folder,name in [(root.parent,"100 only"),(root.parent/"speed-holdout","60 and 150"),(root,"Interleaved")]:
    events = rows(folder/"events.csv")
    starts = [r for r in events if r["event"]=="spin"]
    stops = [float(r["time_s"]) for r in events if r["event"]=="stop"]
    states = rows(folder/"states.csv")
    predictions = rows(folder/"predictions.csv")
    for i,(cue,end) in enumerate(zip(starts,stops)):
        speed = int(cue["detail"])*(100 if folder==root.parent else 1)
        start = float(cue["time_s"])
        window = [r for r in states if (start+end)/2<=float(r["time_s"])<end]
        def obs(key):
            return np.array([float(r[key]) for r in window])
        angle = obs("head_rad")+np.deg2rad(-.66)
        yaw = float(np.mean(obs("gyro_x")*np.sin(angle)+obs("gyro_z")*np.cos(angle)))
        wheels = float(np.mean(obs("right_mm_s")-obs("left_mm_s")))
        sim = [r for r in predictions if int(r["turn"])==i+1]
        model = np.mean(np.interp(obs("time_s")-start,[float(r["t_s"]) for r in sim],[float(r["yaw_rad_s"]) for r in sim]))
        summary.append(dict(session=name,turn=i+1,signed_right_mm_s=speed,samples=len(window),
                            head_deg=float(np.mean(np.rad2deg(obs("head_rad")))), lift_mm=float(np.mean(obs("lift_mm"))),mean_yaw_rad_s=yaw,mean_wheel_difference_mm_s=wheels,gain=yaw/wheels,
                            model_yaw_rad_s=model,error_pct=100*(model/yaw-1),
                            encoder_gain_error_pct=100*(.013230645327085067*wheels/yaw-1)))
with (root/"sessions.csv").open("w") as f:
    w=csv.DictWriter(f,fieldnames=list(summary[0]));w.writeheader();w.writerows(summary)
fig,axes=plt.subplots(1,2,figsize=(9,4),layout="constrained")
for ax,sgn,title in zip(axes,[1,-1],["Counterclockwise","Clockwise"]):
    ax.axhspan(-5,5,color="#eef4ec")
    ax.axhline(0,ls=":",color="gray")
    for name,mark,offset in [("100 only","o",-1.4),("60 and 150","^",-1.4),("Interleaved","D",1.4)]:
        data=[r for r in summary if r["session"]==name and np.sign(r["signed_right_mm_s"])==sgn]
        ax.scatter([abs(r["signed_right_mm_s"])+offset for r in data],[r["error_pct"] for r in data],label=name,marker=mark)
    ax.set_title(title);ax.set_xticks([60,100,150]);ax.set_xlabel("Commanded wheel speed (mm/s)")
axes[0].set_ylabel("Model steady yaw magnitude error (%)")
axes[0].legend(fontsize=8)
fig.savefig(root/"sessions.png",dpi=160)
for speed in (60,100,150):
    for mixed in [False,True]:
        selected=[r for r in summary if abs(r["signed_right_mm_s"])==speed and (r["session"]=="Interleaved")==mixed]
        print(speed, mixed,[round(r["error_pct"],2) for r in selected],round(np.mean([r["error_pct"] for r in selected]),2))

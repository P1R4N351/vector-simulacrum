import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

root=Path(__file__).parent
def scores(file):
    with (root/file).open() as f:
        return list(csv.DictReader(f))

old=scores("ack-before.csv")
new=scores("ack-scores.csv")
assert [(r["session"],r["cue"]) for r in old]==[(r["session"],r["cue"]) for r in new]
fig,axes=plt.subplots(1,2,figsize=(10,4),layout="constrained")
actual=np.array([float(r["physical_s"]) for r in new])
for group,label,color in [(old,"First entry into tolerance",".55"),(new,"Settled target","C0")]:
    predicted=np.array([float(r["sim_action_s"]) for r in group])
    axes[0].scatter(actual,predicted,label=label,color=color,s=22,alpha=.7)
axes[0].plot([.2,1],[.2,1],"k:",lw=1)
axes[0].set(xlabel="Physical SDK response (s)",ylabel="Simulated completion (s)")
axes[0].legend(fontsize=8)
for idx,name in enumerate(("unit2","holds/holds","holds/check")):
    group=[r for r in new if r["session"]==name]
    delays=[1000*(float(r["physical_s"])-float(r["physical_pos_s"])) for r in group]
    axes[1].scatter(range(len(group)),delays,label=name,marker=("o","s","^")[idx],alpha=.7)
axes[1].set(xlabel="Command index in recording",ylabel="SDK response after first in-position packet (ms)")
axes[1].legend(fontsize=8)
fig.savefig(root/"ack-timing.png",dpi=160)

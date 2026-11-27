import matplotlib.pyplot as plt
import numpy as np

r = np.genfromtxt("push-width.csv", delimiter=",", names=True)
fig, (a,b) = plt.subplots(1,2,figsize=(9,4),layout="constrained")
for shell,rubber,label in [(45,45.30681828,"Canonical widths"),(45,44.7,"Narrower pads"),
                           (43.5,45.30681828,"Narrower shell"),(43.5,44.7,"Both narrower")]:
    t = r[(r["shell_mm"]==shell)&(r["rubber_mm"]==rubber)]
    a.plot(t["trial"],t["endpoint_error_mm"],"o-",label=label)
    b.plot(t["trial"],t["peak_tilt_deg"],"o-")
    print(label,t["endpoint_error_mm"][0],t["endpoint_error_mm"][1:].mean(),t["peak_tilt_deg"][0],t["left_mm"][0])
for ax in (a,b):
    ax.set(xticks=range(1,6),xlabel="Recorded push")
a.set(ylabel="Settled endpoint error (mm)",yscale="log")
b.set(ylabel="Peak simulated tilt (degrees)",ylim=(0,None))
a.legend(fontsize=8)
fig.suptitle("Cube width sensitivity with recorded wrist motion")
fig.savefig("push-width.png",dpi=180)

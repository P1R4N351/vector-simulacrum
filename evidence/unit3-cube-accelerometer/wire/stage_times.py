"""Compare same-slot intervals across the browser, relay, kernel and application."""
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
fig, panels = plt.subplots(1, 3, figsize=(14, 4), sharey=True)
for ax, path, title in zip(panels, ["remote-stage-times.csv", "local-stage-times.csv", "tunnel-stage-times.csv"],
                           ["Mac browser and relay → Linux", "Browser, relay and receiver on Linux", "Mac browser → SSH → Linux relay"]):
    data = np.genfromtxt(HERE/path, names=True, delimiter=",")
    print(title, "missing", np.flatnonzero(~np.isfinite(data["linux_kernel_s"])))
    for key, label in [("browser_s","Browser WebSocket"),("relay_s","Node UDP send"),
                       ("linux_kernel_s","Linux UDP arrival"),("linux_read_s","Receiver read")]:
        # Only compare complete adjacent triples across a common 6-slot window.
        complete = np.isfinite(data["linux_kernel_s"])
        ok = np.convolve(complete.astype(int),np.ones(6,dtype=int),mode="valid")[::3]==6
        diff = 1000*np.diff(data[key][2::3])
        vals = np.sort(diff[ok])
        print(label,len(vals), "p05/median/p95 ms",np.round(np.percentile(vals,[5,50,95]),2),
              "within 5%",round(100*np.mean(abs(vals-30)<=1.5),1))
        ax.step(vals,100*np.arange(1,len(vals)+1)/len(vals),where="post",label=label)
    ax.axvline(30,ls=":",color="black")
    ax.set_xlim(0,100)
    ax.set_title(title)
    ax.set_xlabel("Three-slot interval (ms)")
    ax.grid(alpha=.2)
panels[0].set_ylabel("Cumulative share of intervals (%)")
panels[1].legend(fontsize=8)
fig.tight_layout()
fig.savefig(HERE/"stage-times.png",dpi=150)

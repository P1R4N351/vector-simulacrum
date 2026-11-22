"""Forecast changes of P2Y8's docked range/quality pair without resynchronizing."""
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
fig, ax = plt.subplots(figsize=(7.4, 4), layout="constrained")
# Public syscon timer.h, hardware.h, schema/messages.h and opto.cpp.
body_tick_s = 80 * 96 * 2 * 16 / 48_000_000
period = 200 * body_tick_s
result = []
for run in "gjklmoq":
    path, = here.parent.glob(f"hw-rec-*/run-{run}-*.jsonl")
    records = [json.loads(line) for line in path.read_text().splitlines()]
    states = [r for r in records if r.get("k") == "s" and "proxq" in r]
    t = np.array([r["t"] for r in states])
    pair = np.array([[r["prox"], r["proxq"]] for r in states])
    stationary = np.array([bool(r["status"] & 4096) and r["wl"] == r["wr"] == 0 for r in states])
    changes = 1 + np.flatnonzero(np.any(pair[1:] != pair[:-1], axis=1))
    # Only preceding observations enter the anchor selection.
    anchor = next(b for a, b in zip(changes, changes[1:])
                  if .8 <= t[b]-t[a] <= 1.2 and stationary[a:b+1].all())
    horizon = 3.25
    if t[-1] < t[anchor] + horizon:
        raise ValueError(f"Run {run} ends before forecast horizon")
    observed = changes[(t[changes] > t[anchor]) & (t[changes] <= t[anchor]+horizon)]
    if len(observed) != 3:
        raise ValueError(f"Run {run}: {len(observed)} changes in the forecast window; score unmatched updates")
    residual = []
    for n, idx in enumerate(observed, 1):
        arrival = t[idx] - t[anchor]
        # Endpoint intervals bound acquisition times only if transport latency is constant.
        lo = t[idx-1] - t[anchor] - n
        hi = t[idx] - t[anchor-1] - n
        result.append((run.upper(), n, t[anchor], arrival, 1000*(arrival-n), 1000*lo, 1000*hi,
                       1000*(arrival-n*period), 1000*(lo+n*(1-period)), 1000*(hi+n*(1-period))))
        residual.append(1000*(arrival-n))
    ax.plot([1, 2, 3], residual, "o-", label=run.upper())
ax.plot([1,2,3], [1000*n*(period-1) for n in (1,2,3)], color="black", ls="--", label="Syscon timer")
ax.axhline(0, color="black", lw=1)
ax.set(xticks=[1, 2, 3], xlabel="Predicted seconds after the anchoring report",
       ylabel="Observed minus predicted arrival (ms)", title="Recorded update times and the 1.024 s body clock")
ax.legend(title="Run", ncols=4)
fig.savefig(here/"dock-clock.png", dpi=160)
with (here/"dock-clock.csv").open("w") as f:
    w = csv.writer(f, lineterminator="\n")
    w.writerow(["run","cycle","anchor_s","observed_elapsed_s","arrival_error_ms","bracket_low_ms","bracket_high_ms","syscon_error_ms","syscon_low_ms","syscon_high_ms"])
    w.writerows(result)
err = np.array(result, dtype=object)[:, 4:].astype(float)
for label, (e, low, high) in zip(("nominal", "syscon"), (err[:, :3].T, err[:, 3:].T)):
    print(label, "MAE, max abs, compatible brackets:", np.abs(e).mean(), np.abs(e).max(),
          np.count_nonzero((low <= 0) & (high >= 0)))

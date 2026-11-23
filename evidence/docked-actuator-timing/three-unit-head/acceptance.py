"""Score old and current acceptance times against SDK receipt-time brackets, without fitting."""
import csv
import json
from pathlib import Path
import statistics

here = Path(__file__).resolve().parent
timing = here.parent


def read(path):
    return list(csv.DictReader(path.read_text().splitlines()))


# The delay and tick are shared across these standalone models. summarize.py
# extracts the first false bit from an actual control-tick replay of each move.
new = read(here/"unit3.csv")
tick_times = {round(float(r["sim_unsettled_s"]), 6) for r in new}
assert len(tick_times) == 1
accept = tick_times.pop()
groups = {"Unit 3 new": [(float(r["unsettled_lo"]),float(r["unsettled_hi"])) for r in new]}
for bot, name, path in [("Unit 1","Unit 1","repeat/unit1"),("Unit 2","Unit 2","unit2"),
                        ("Unit 3","Unit 3 earlier","repeat/unit3")]:
    states = [s for s in map(json.loads,(timing/(path+".jsonl")).read_text().splitlines()) if s["k"]=="s"]
    brackets = []
    for m in read(timing/"repeat/moves.csv"):
        if (m["source"],m["joint"]) != (bot,"head"): continue
        start = float(m["command_s"])
        window = [s for s in states if -.20 < s["t"]-start < .65]
        falling = [(a["t"]-start,b["t"]-start) for a,b in zip(window,window[1:])
                   if a["status"] & 512 and not b["status"] & 512]
        assert len(falling) == 1
        brackets.extend(falling)
    groups[name] = brackets
for name, brackets in groups.items():
    for clock in (0,.0916):
        for model in (0,accept):
            pred = model+clock
            errors = [max(lo-pred,pred-hi,0) for lo,hi in brackets]
            print(name, round(pred*1000,1), errors.count(0),len(errors),round(1000*statistics.median(errors),1))

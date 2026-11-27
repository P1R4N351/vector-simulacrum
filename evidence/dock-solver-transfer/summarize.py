"""Summarize signed solver changes without pooling different acquisition rates."""
import csv
from itertools import product
from statistics import mean

with open("scores.csv") as file:
    rows = list(csv.DictReader(file))
with open("summary.csv","w") as file:
    w = csv.writer(file,lineterminator="\n")
    w.writerow(["unit","scene","phase","turns","baseline_dps","candidate_dps","worse_turns"])
    for unit,scene,phase in product(["unit1","unit2","unit3"],["vector-free-dock","vector-contact42-free-dock"],["steady","stop"]):
        pairs = [[float(row["rmse_dps"]) for row in rows if
                  (row["unit"],row["scene"],row["phase"],row["iterations"])==(unit,scene,phase,it)] for it in ["0","4"]]
        assert len(pairs[0]) == len(pairs[1]) == 4
        w.writerow([unit,scene,phase,4,*map(mean,pairs),sum(b>a for a,b in zip(*pairs))])

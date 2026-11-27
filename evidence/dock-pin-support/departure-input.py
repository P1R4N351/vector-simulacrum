import csv, gzip, json, math
with gzip.open("../unit3-w34-return/continuation.csv.gz","rt") as source:
    rows=list(csv.DictReader(source))
edge=next(float(r["receipt_s"]) for r in rows if int(r["status"]) & 4096 == 0)
sample=[r for r in rows if -1.8 <= float(r["receipt_s"])-edge < 0.75]
print(json.dumps(dict(head=math.degrees(float(sample[0]["head_rad"])),lift=float(sample[0]["lift_mm"]),
                      wheels=[[float(r["receipt_s"])-edge,float(r["left_mmps"]),float(r["right_mmps"])] for r in sample])))

import csv,gzip,json,math,sys
from pathlib import Path
p=Path(sys.argv[1])
out=[]
for name in [*'ABCDEF','W35']:
    f=p/'unit3-w34-return/continuation.csv.gz' if name=='W35' else p/'unit3-physical-repeats'/name/'robot.csv.gz'
    with gzip.open(f,'rt') as stream: rows=list(csv.DictReader(stream))
    def on(row): return int(row['status'])&4096
    i=next(i for i in range(1,len(rows)) if on(rows[i-1]) and not on(rows[i]) and any(float(rows[i-1][k]) for k in ('left_mmps','right_mmps')))
    z=float(rows[i]['receipt_s'])
    w=[[float(r['receipt_s'])-z,float(r['left_mmps']),float(r['right_mmps'])] for r in rows if -1.8<=float(r['receipt_s'])-z<=1.6]
    r=next(r for r in rows if float(r['receipt_s'])>=z+w[0][0]-1e-9)
    out.append(dict(name=name,wheels=w,head=math.degrees(float(r['head_rad'])),lift=float(r['lift_mm'])))
print(json.dumps(out))

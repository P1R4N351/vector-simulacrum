"""Distance of physical SDK angles to each simulator encoder lattice."""
import csv
import json
import math
from pathlib import Path
from statistics import median

here=Path(__file__).resolve().parent
evidence=here.parents[1]
model=json.loads((evidence.parent/'web/public/robot/model.json').read_text())['values']
step=model['head_rad_per_encoder_count']
low=math.radians(model['head_angle_min_deg'])
sessions={'first unit':'workshop-stereo-lift/target-crossover/sdk.csv',
          'second unit':'docked-actuator-timing/unit2.jsonl',
          'third unit':'docked-actuator-timing/unit3.jsonl',
          'third unit repeat':'workshop-stereo-lift/target-transfer/sdk.csv'}
with (here/'scores.csv').open('w') as out:
    w=csv.writer(out,lineterminator='\n')
    w.writerow(('session','joint','packets','old_median_abs_deg','new_median_abs_deg','new_max_abs_deg'))
    for name,filename in sessions.items():
        with (evidence/filename).open() as f:
            if filename.endswith('.csv'):
                pairs=[(math.radians(float(r['head_deg'])),float(r['lift_mm'])) for r in csv.DictReader(f) if r['event']=='state']
            else:
                records=(json.loads(line) for line in f)
                pairs=[(r['head'],r['lift']) for r in records if r['k']=='s']
        def shoulder(height):
            return math.asin((height-model['lift_base_position_mm'][2])/model['lift_arm_length_mm'])
        lift_low=shoulder(model['lift_height_min_mm'])
        for joint,angles,old_origin,new_origin,old_step,new_step in (
            ('head',[x for x,_ in pairs],0,low,.0021447,step),
            ('lift',[shoulder(h) for _,h in pairs],lift_low,lift_low,.0052465,model['lift_rad_per_encoder_count'])):
            def residuals(origin,spacing):
                return [abs(math.degrees(x-(origin+round((x-origin)/spacing)*spacing))) for x in angles]
            old,new=residuals(old_origin,old_step),residuals(new_origin,new_step)
            w.writerow((name,joint,len(angles),median(old),median(new),max(new)))

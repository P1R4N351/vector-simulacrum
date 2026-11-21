"""Revise the verified 0.02 mm Cura preview, in its existing absolute Marlin coordinates."""
from pathlib import Path
import math,re,hashlib

here=Path(__file__).resolve().parent
source=here.parent/'kp3s-0.02-preview.gcode'
digest=hashlib.sha256(source.read_bytes()).hexdigest()
assert digest=='3a5a52146ebc4450c8ccafd00502f093e026043946de4692e6d08e4d7d58ea82', 'Only for the verified KP3S 0.02 mm preview'
def shoulder(y):
    # Constant at the caps; fade smoothly to zero in the surrounding perimeter.
    u=max(0,min(1,(abs(y)-.3)/1.2))
    return 1-u*u*(3-2*u)
def mapped(x,y):
    v=y-90
    return round(y-(math.copysign(.05*shoulder(v),v) if x>115 and v else 0),3)
pos=dict(X=0.,Y=0.,Z=0.,E=0.); extrusion=0.
lines=[]; layer=-1; finished=False
for line in source.read_text().splitlines():
    if line.startswith(';LAYER:'):layer=int(line[7:])
    tokens=line.partition(';')[0].split()
    if not tokens or tokens[0] not in ('G0','G1','G92'):
        lines.append(line);continue
    move=pos.copy()
    for token in tokens[1:]:
        if token[0] in move:move[token[0]]=float(token[1:])
    if tokens[0]=='G92':
        assert tokens[1:]==['E0'];extrusion=0.;pos=move;lines.append(line);continue
    if layer==57 and tokens[0]=='G0' and move['Z']<pos['Z']:
        # Last printing lap has returned to its seam. Drop Cura's lower, partial retrace.
        assert abs(move['X']-pos['X'])<.001 and abs(move['Y']-pos['Y'])<.001
        finished=True
    if finished:continue
    y1=mapped(pos['X'],pos['Y']);y2=mapped(move['X'],move['Y'])
    dx=move['X']-pos['X'];dz=move['Z']-pos['Z']
    before=math.hypot(dx,move['Y']-pos['Y'],dz);after=math.hypot(dx,y2-y1,dz)
    if move['E']!=pos['E']:
        extrusion+=(move['E']-pos['E'])*(after/before if before else 1)
        line=re.sub(r'\bE[-\d.]+',f'E{extrusion:.5f}',line)
    if y2!=move['Y']:
        if ' Y' in line:line=re.sub(r'\bY[-\d.]+',f'Y{y2:.3f}',line)
        else:line+=f' Y{y2:.3f}'
    elif y1!=pos['Y'] and ' Y' not in line:line+=f' Y{y2:.3f}'
    lines.append(line);pos=move
assert finished
(here/'kp3s-seam-preview.gcode').write_text('; Offline seam study: 0.050 mm inward per cap, no terminal retrace\n'+'\n'.join(lines)+'\n')

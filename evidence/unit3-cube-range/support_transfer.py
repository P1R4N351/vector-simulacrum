"""Check the ramp candidate on native flat turns and free tipping releases."""
import csv
import subprocess
import sys
from pathlib import Path
from unittest.mock import patch

import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))
from replay_measured_turn import run
from simulacrum import mjcf, params
import wheelstand_check as tip


def read(name):
    return np.genfromtxt(ROOT/'evidence/hw-rec-20261119'/name, names=True, dtype=None,
                         delimiter=',', encoding='utf8')


def support_models(fixed=None, four=None):
    if fixed is None:
        from web.tools.export_model import SCENE_OBSTACLES
        v = params.values(params.load(model='vector_2'))
        fixed = (ROOT/'web/public/robot/vector2/vector.xml').read_text()
        assert fixed.strip() == mjcf.build_mjcf(v, SCENE_OBSTACLES).strip()
        four = mjcf.build_mjcf(dict(v, tip_supports_driven=1), SCENE_OBSTACLES)
    module = "import{rearEnvelope}from'./rear_envelope.mjs';import{readFileSync}from'node:fs';"
    driven = subprocess.check_output(['node','--input-type=module','-e',module+
        "process.stdout.write(rearEnvelope(readFileSync(0,'utf8'),'driven_rear_envelope'));"],
        input=fixed,text=True,cwd=HERE)
    assert fixed != driven
    assert four is not None
    return [('Fixed',fixed),('Driven rear',driven),('Four driven',four)]


def main():
    v = params.values(params.load(model="vector_2"))
    hybrid = read('hybrid-encoder-replay.csv')
    native = read('unit2-native-replay-input.csv')
    fig, axes = plt.subplots(2, 4, figsize=(14,6), layout='constrained')
    scores, angles = [], []
    for label, xml in support_models():
        with patch.object(mjcf,'build_mjcf',return_value=xml):
            balance = tip.check(v)
        for kind in tip.wheelstand.KINDS:
            b = balance[kind]
            angles.append([label,kind,b['sim_deg'],*b['bracket_deg'],b['closed_form_deg']])
        for col, name in enumerate(['ccw_1','cw_1','ccw_2','cw_2']):
            rows = hybrid[hybrid['turn']==name]
            t = rows['ms_from_last_edge'] / 1000
            wheels = np.column_stack([rows['left_input_mmps'],rows['right_input_mmps']])
            head = native[native['turn']==name]['head_rad'][0]
            clock, out = run([dict(head=head,lift=30.6450119)],t,wheels,None,model_xml=xml,dense=True)
            predicted = np.interp(t-.017,clock,out[:,0])
            max_tracking = max(np.max(np.abs(out[:,k+1]-np.interp(clock,t,wheels[:,k]))) for k in (0,1))
            real = rows['measured_head_dps']
            for ax,window in zip(axes[:,col],[(-.85,-.25),(-.10,.10)]):
                use = (t>=window[0]) & (t<=window[1])
                if label=='Fixed':
                    ax.plot(t[use],real[use],c='black',lw=1.5,label='Recorded head gyro')
                ax.plot(t[use],predicted[use],lw=1.2,ls={'Driven rear':'--','Four driven':':','Fixed':'-'}[label],label=label)
                scores.append([name,label,*window,np.sqrt(np.mean((predicted[use]-real[use])**2)),max_tracking])
            axes[0,col].set_title(name.upper().replace('_',' '))
            axes[1,col].set_xlabel('Seconds from last encoder edge')
    axes[0,0].set_ylabel('Steady head gyro (°/s)')
    axes[1,0].set_ylabel('Braking head gyro (°/s)')
    axes[0,-1].legend(fontsize=8)
    fig.savefig(HERE/'support-transfer.png',dpi=160)
    for path,header,rows in [('support-transfer.csv','turn variant start_s end_s gyro_rmse_dps max_wheel_error_mmps',scores),
                             ('support-balance.csv','variant direction release_deg low_deg high_deg geometric_deg',angles)]:
        with (HERE/path).open('w') as f:
            w=csv.writer(f,lineterminator='\n')
            w.writerow(header.split())
            w.writerows(rows)


if __name__=='__main__':
    main()

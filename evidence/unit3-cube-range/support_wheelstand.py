"""Run the dock candidate through the archived W4/W8 fork fixture and pose uncertainty."""
import argparse
import hashlib
import importlib.util
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from unittest.mock import patch

from support_transfer import HERE, ROOT, mjcf, params, support_models

spec = importlib.util.spec_from_file_location('wheelstand_replay', ROOT/'evidence/unit3-wheelstand/reproduction/replay.py')
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


def main(fixture):
    assert hashlib.sha256(fixture.read_bytes()).hexdigest() == '4867e81c8d8d92ad3abc621d920af9b550c001e5d8f81d8443d1e84a069b8230'
    v = params.values(params.load(model='vector_2'))
    scores = []
    fig, axes = plt.subplots(2,2,figsize=(11,6),layout='constrained',sharey='row',sharex=True)
    models = support_models(mjcf.build_mjcf(v,(),cube='free'),
                           mjcf.build_mjcf(dict(v, tip_supports_driven=1),(),cube='free'))
    colors = {'Fixed':'#845a29','Driven rear':'#007b79','Four driven':'#644297'}
    for label, xml in models:
        with patch.object(mjcf,'build_mjcf',return_value=xml):
            m = replay.build(v,fixture)
        m.opt.timestep = v['sim_timestep_ms'] * 1e-3 / 4
        for (upper,lower), name in zip(axes.T,('W4','W8')):
            trial = replay.Trial(name)
            if label=='Fixed':
                upper.plot(trial.state['sec']-trial.zero-trial.late, np.degrees(trial.state['pitch']),
                           color='black',lw=2,label='Recorded',zorder=3)
                specific = np.linalg.norm(np.stack([trial.cube[k] for k in 'xyz']),axis=0)/8192
                lower.plot(trial.cube['sec']-trial.zero-trial.late,specific,color='black',lw=1.3,zorder=3)
            for i,(start,onset) in enumerate((s+trial.shift,o) for s in (3.5,3.8) for o in (trial.early,trial.late)):
                trace,pk,taps,_ = replay.run(m,v,trial,start,onset,'servo:60',17)
                score = replay.score(trial,trace,pk,taps,onset)
                scores.append([name,label,start,onset,*[score[k] for k in ('peak_pitch_deg','hold_pitch_deg','pitch_rmse_deg','peak_g','taps')]])
                style = dict(color=colors[label], alpha=.55,lw=1)
                upper.plot(trace[:,0]-trial.late,trace[:,1],label=label if i==0 else None,**style)
                lower.plot(pk[:,0]-trial.late,pk[:,1],**style)
            upper.set(title=name,xlim=(-.3,1.2))
            lower.set_xlabel('Time from last raised-lift report (s)')
    axes[0,0].set_ylabel('Chassis pitch (°)')
    axes[1,0].set_ylabel('Cube specific force (g)')
    axes[0,-1].legend(fontsize=8)
    fig.savefig(HERE/'support-wheelstand.png',dpi=160)
    with (HERE/'support-wheelstand.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow('trial variant start_s onset_s peak_deg held_deg pitch_rmse_deg peak_g taps'.split())
        w.writerows(scores)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('fixture',type=Path)
    main(parser.parse_args().fixture)

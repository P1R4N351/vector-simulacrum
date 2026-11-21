"""Compare radial and heading-constrained pivot estimates in each camera pose."""
import csv
import json
import sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
import camera_spins as cam


def estimate(lens, heading):
    center=cam.circle(lens)[:2]
    radial=cam.LENS_X_MM+np.median(np.sum((center-lens)*heading,axis=1))
    lateral=np.column_stack((-heading[:,1],heading[:,0]))
    # lens + forward * dx + left * dy = one common world pivot.
    basis=np.stack([heading,lateral],axis=2)
    design=(basis-basis.mean(axis=0)).reshape(-1,2)
    offset=np.linalg.lstsq(design,(lens.mean(axis=0)-lens).ravel(),rcond=None)[0]
    world=lens+basis@offset
    return radial, cam.LENS_X_MM+offset[0], np.sqrt(np.mean(np.sum((world-world.mean(axis=0))**2,axis=1)))


def main():
    rows=np.genfromtxt(HERE.parent.parent/'camera-spins/camera-picks.csv',delimiter=',',names=True)
    traces=np.genfromtxt(HERE.parent/'traces.csv',delimiter=',',names=True,dtype=None,encoding='utf-8')
    with (HERE.parent.parent/'camera-spins/robot.jsonl').open() as f:
        commands=[j for j in map(json.loads,f) if j.get('k')=='c' and j.get('name')=='wheels']
    results=[]
    for label,prior,weight in [('dock_and_shelf',580,4),('dock_only_502',502,1e9),('dock_only_580',580,1e9)]:
        pose=cam.calibration(prior,weight)
        for turn in range(1,5):
            r=rows[rows['turn']==turn]
            lens=cam.intersect(np.column_stack([r['u_px'],r['v_px']]),cam.LENS_HEIGHT_MM,pose)
            heading=cam.intersect(np.column_stack([r['aim_u_px'],r['aim_v_px']]),cam.LENS_HEIGHT_MM,pose)-lens
            heading/=np.linalg.norm(heading,axis=1)[:,None]
            a,b,res=estimate(lens,heading)
            loo=[estimate(np.delete(lens,i,axis=0),np.delete(heading,i,axis=0))[1] for i in range(len(r))]
            cmd=commands[turn-1]
            tr=traces[(traces['series']=='proxy_3.75') & (traces['spin']==cmd['label'])]
            times=r['time_s']+.018-cmd['t']
            angle=np.interp(times,tr['t_s'],np.unwrap(tr['heading_rad']))
            ahead=np.column_stack([np.cos(angle),np.sin(angle)])
            xy=np.column_stack([np.interp(times,tr['t_s'],tr[k]) for k in ('x_mm','y_mm')])
            model_circle,model_rigid,model_res=estimate(xy+cam.LENS_X_MM*ahead,ahead)
            results.append(dict(model_circle_mm=model_circle,model_rigid_mm=model_rigid,
                                model_landmark_rms_mm=model_res,calibration=label,turn=turn,focal_px=pose[6],circle_x_mm=a,rigid_x_mm=b,
                                landmark_rms_mm=res,loo_min_mm=min(loo),loo_max_mm=max(loo)))
    with (HERE/'optical.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=results[0]); w.writeheader();w.writerows(results)


if __name__=='__main__':
    main()

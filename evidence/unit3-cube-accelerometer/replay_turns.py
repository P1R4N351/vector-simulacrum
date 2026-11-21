"""Predict cube packets during the recorded turns, conditional on the pre-turn orientation."""
import csv
import sys
from pathlib import Path
from xml.etree import ElementTree
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parent/'unit3-native-pickup'))
from replay_carry import fixture, run, values
from simulacrum.cube import CubeAccelerometer
from decode import decode


def predicted(times, robot, quat, sensor_values=None):
    def configure(root):
        fixture(root, True)
        root.find(".//body[@name='cube']").set("quat", " ".join(map(str,quat)))
        ElementTree.SubElement(root.find("sensor"),"accelerometer",name="payload",site="cube_chip")

    chip = CubeAccelerometer(values if sensor_values is None else sensor_values, values["sim_timestep_ms"]/1000)
    def observe(m,d,t):
        g = d.sensor("payload").data / (values["gravity_mmps2"]/1000)
        if t == times[0]:
            chip.reset(g,t)
        else:
            chip.step(t,g)
    first = robot[0]
    run([{"head":first["head_rad"],"lift":first["lift_mm"]}],times,
        np.column_stack([robot["left_mmps"],robot["right_mmps"]]),40.5,configure=configure,observe=observe)
    clock = np.array([t for t,_ in chip.app.messages])
    # Compare packet means; SDK command and cube delivery clocks share the recorder.
    samples = np.array([np.frombuffer(p[2:],dtype='<i2').reshape(3,3).mean(axis=0)/8192
                        for _,p in chip.app.messages])
    return clock,samples


def main():
    events=list(csv.DictReader((here/'action-times.csv').open()))
    outward,back=[float(e['pod_unix_s']) for e in events if e['action']=='turn loaded' and e['event']=='begin']
    end=next(float(e['pod_unix_s']) for e in events if e['action']=='place' and e['event']=='begin')
    packets,_=decode(here/'pickup.tsv')
    t=np.array([p[0]*1e-9-outward for p in packets])
    meas=np.array([np.frombuffer(p[3],dtype='<i2').reshape(3,3).mean(axis=0)/8192 for p in packets])
    pre=(t>=-1.7)&(t<=-.2)
    assert pre.sum()>20
    stationary=np.median(meas[pre],axis=0)
    a=stationary/np.linalg.norm(stationary)
    z=np.array([0.,0.,1.])
    q=np.r_[1+a@z,np.cross(a,z)]
    q/=np.linalg.norm(q)
    robot=np.genfromtxt(here/'robot.csv',delimiter=',',names=True)
    clock=robot['pod_unix_s']-outward
    window=(clock>-2.1)&(clock<end-outward+.2)
    predictions={'Pre-turn constant':np.tile(stationary,(len(t),1))}
    orientations = [('Level',[1,0,0,0]),('Tilted',q)]
    # Gravity leaves rotation about vertical undetermined. Rotate the entire
    # fitted attitude about fork Z, preserving its pre-turn gravity projection.
    w,x,y,zq = q
    for angle in (90,180,270):
        c,s = np.cos(np.deg2rad(angle/2)),np.sin(np.deg2rad(angle/2))
        orientations.append((f'Tilted +{angle} deg',[c*w-s*zq,c*x-s*y,c*y+s*x,c*zq+s*w]))
    for label,orientation in orientations:
        rt,v=predicted(clock[window],robot[window],orientation)
        predictions[label]=np.column_stack([np.interp(t,rt,v[:,i]) for i in range(3)])
    scores=[]
    fig,axes=plt.subplots(3,1,sharex=True,figsize=(9,6))
    show=(t>=-.25)&(t<end-outward)
    for i,ax in enumerate(axes):
        ax.plot(t[show],meas[show,i],'.-',color='black',ms=3,label='Cube')
        for name,v in predictions.items():
            if '+' not in name:
                ax.plot(t[show],v[show,i],label=name,alpha=.85)
        ax.axvline(back-outward,color='.6',ls=':')
        ax.set_ylabel('XYZ'[i]+' (g)')
    axes[0].legend(ncol=4,fontsize=8)
    axes[-1].set_xlabel('Seconds from first 20° turn command')
    fig.tight_layout()
    fig.savefig(here/'turn-replay.png',dpi=150)
    for label,mask in [('outward',(t>=0)&(t<back-outward)),('return',(t>=back-outward)&(t<end-outward))]:
        for name,v in predictions.items():
            error=v[mask]-meas[mask]
            scores.append([label,name,mask.sum(),np.sqrt(np.mean(np.sum(error**2,axis=1))),
                           np.percentile(np.linalg.norm(error,axis=1),95)])
    with (here/'turn-replay.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(['turn','prediction','packets','vector_rms_error_g','p95_error_g'])
        w.writerows(scores)
    print(stationary,scores)

if __name__=='__main__':
    main()

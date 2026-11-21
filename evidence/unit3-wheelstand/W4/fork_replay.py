"""Replay the measured W4 fork motion against a free cube. Pass the built forks7 fixture XML."""
import json,sys,csv
from pathlib import Path
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import numpy as np
import mujoco as mj
here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parents[2]))
from simulacrum import mjcf, params
from simulacrum.cube import CubeAccelerometer

v=params.values(params.load(model='vector_2'))
scene=ET.parse(sys.argv[1]).getroot()
scene.find(".//body[@name='cube']").append(ET.fromstring(mjcf._cube_chip_xml(v)))
ET.SubElement(ET.SubElement(scene,'sensor'),'accelerometer',name='cube_accel',site='cube_chip')
model=mj.MjModel.from_xml_string(ET.tostring(scene,encoding='unicode'))
sensor_dt=v['sim_timestep_ms']*.001
dt=sensor_dt/4
model.opt.timestep=dt
r=np.genfromtxt(here/'approach.csv',delimiter=',',names=True)
target=json.loads((here/'initial-cube.json').read_text())
w,x,y,z=[target['q'+str(i)] for i in range(4)]
cube_yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
times=np.arange(-.4,6.,dt)
def interpolate(key):
    return np.interp(times,r['s'],r[key])

def run(freeze):
    x,y,yaw,pitch,h=map(interpolate,('x_mm','y_mm','yaw','pitch','lift_mm'))
    cut=np.flatnonzero((times>3)&(pitch>np.deg2rad(5)))[0]
    if freeze:
        x[cut:]=x[cut];y[cut:]=y[cut]
    pivot_x=-v['wheel_base_mm']
    pivot_z=v['wheel_rear_radius_mm']
    reach=-41+np.sqrt(66**2-(h-45)**2)
    along=pivot_x+(reach-pivot_x)*np.cos(pitch)-(h-pivot_z)*np.sin(pitch)
    height=pivot_z+(reach-pivot_x)*np.sin(pitch)+(h-pivot_z)*np.cos(pitch)
    path=np.column_stack([x+along*np.cos(yaw),y+along*np.sin(yaw),height])*.001
    a,b=np.cos(yaw/2),np.sin(yaw/2)
    c,s=np.cos(pitch/2),np.sin(pitch/2)
    quat=np.column_stack([a*c,b*s,-a*s,b*c])
    data=mj.MjData(model)
    center=np.array([target['x'],target['y']])
    data.qpos[:3]=[* (.001*center),.02215]
    data.qpos[3:]=[np.cos(cube_yaw/2),0,0,np.sin(cube_yaw/2)]
    sensor=CubeAccelerometer(v,sensor_dt,seed=17)
    trace=[]
    for i,t in enumerate(times):
        data.mocap_pos[0]=path[i];data.mocap_quat[0]=quat[i]
        mj.mj_step(model,data)
        if (i+1)%4:continue
        sensor.step((i+1)*dt,data.sensor('cube_accel').data/9.81)
        w,xq,yq,zq=data.qpos[3:]
        angle=np.arctan2(2*(w*zq+xq*yq),1-2*(yq*yq+zq*zq))
        top_xy=data.qpos[:2]*1000+22*np.array([2*(xq*zq+w*yq),2*(yq*zq-w*xq)])
        trace.append([t,np.linalg.norm(top_xy-center),np.rad2deg(angle-cube_yaw)])
    packets=np.asarray([[t+times[0],np.linalg.norm(a)/8192] for t,p in sensor.messages for a in np.frombuffer(p[2:],'<i2').reshape(3,3)])
    return np.array(trace),packets,len(sensor.taps.events)

fig,axes=plt.subplots(3,1,figsize=(9,7),layout='constrained',sharex=True)
optical=np.genfromtxt(here/'optical-travel.csv',delimiter=',',names=True)
visible=optical['correlation']>.7
for ax,key in zip(axes,('travel_mm','yaw_deg')):
    ax.scatter(optical['s_after_command'][visible],optical[key][visible],color='black',s=14,label='Overhead')
force=np.genfromtxt(here/'cube.csv.gz',delimiter=',',names=True)
axes[2].scatter(force['sec']-22.330433,np.linalg.norm([force[k] for k in 'xyz'],axis=0)/8192,s=5,color='black',label='Physical cube')
with (here/'fork-replay.csv').open('w') as f:
    writer=csv.writer(f,lineterminator='\n');writer.writerow(['translation','settled_glyph_travel_mm','settled_yaw_deg','peak_g','tap_count'])
    for freeze in (False,True):
        label='Stop XY at 5° pitch' if freeze else 'SDK XY throughout'
        trace,pk,taps=run(freeze)
        for j,ax in enumerate(axes[:2],1):ax.plot(trace[:,0],trace[:,j],label=label)
        axes[2].plot(pk[:,0],pk[:,1],label=label,alpha=.7)
        writer.writerow([label,*np.median(trace[trace[:,0]>5,1:],axis=0),pk[pk[:,0]>0,1].max(),taps])
for ax,label in zip(axes,('Cube travel (mm)','Cube yaw (° CCW)','Specific force (g)')):
    ax.set(xlim=(3.5,5.9),ylabel=label)
axes[0].legend(fontsize=8);axes[-1].set_xlabel('Seconds from wheelstand command')
fig.savefig(here/'fork-replay.png',dpi=150)

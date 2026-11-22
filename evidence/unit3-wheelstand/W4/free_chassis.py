"""W4 finite-actuator replay. Supply XML from the printed-fork fixture builder."""
import csv
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parents[2]))
from simulacrum import mjcf,params
from simulacrum.cube import CubeAccelerometer
from simulacrum.drive import TreadController

held_out="--w8" in sys.argv[2:]
if held_out and '--firmware' not in sys.argv[2:]:
    raise ValueError('Use --firmware for the W8 transfer test')
if held_out:
    here=here.parent/"W8"
v=params.values(params.load(model='vector_2'))
tree=ET.fromstring(mjcf.build_mjcf(v,(),cube='free'))
fixture=ET.parse(sys.argv[1])
asset=ET.SubElement(tree,'asset')
asset.extend(fixture.findall('.//asset/mesh'))
fork=tree.find(".//body[@name='lift_fork']")
attrs={k:x for k,x in fork.find('geom').attrib.items() if k not in ('name','type','size','pos','mass')}
for geom in fork.findall('geom'):
    geom.set('contype','0');geom.set('conaffinity','0')
pieces=fixture.findall(".//body[@name='fork']/geom[@type='mesh']")
assert len(pieces)==58
for piece in pieces:
    ET.SubElement(fork,'geom',dict(attrs,type='mesh',mesh=piece.get('mesh'),name=piece.get('name'),mass='0'))
m=mj.MjModel.from_xml_string(ET.tostring(tree,encoding='unicode'))
sensor_dt=v['sim_timestep_ms']*.001
dt=sensor_dt/4
m.opt.timestep=dt
approach=np.genfromtxt(here/'approach.csv',names=True,delimiter=',')
state=np.genfromtxt(here/'state.csv.gz',names=True,delimiter=',')
with (here/'actions.csv').open() as f:
    zero=next(float(r['sec']) for r in csv.DictReader(f) if r['action']=='pop_a_wheelie')
# Time relative to the last raised-lift observation; the internal action timestamp is unavailable.
late_onset=4.015283
early_onset=3.950686
if held_out:
    drop=np.flatnonzero((approach['lift_mm'][1:]-approach['lift_mm'][:-1]) < -10)[0]+1
    early_onset,late_onset=approach['s'][drop-2:drop]
shift=late_onset-4.015283
target=json.loads((here/'initial-cube.json').read_text())
w,x,y,z=[target['q'+str(i)] for i in range(4)]
angle=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
camera_xy='--camera-xy' in sys.argv[2:]
if camera_xy:
    if not held_out:
        raise ValueError('Camera XY registration is available for W8 only')
    with (here/'camera-pose.csv').open() as f:
        ref=next(r for r in csv.DictReader(f) if r['case']=='geometry' and
                 [float(r[k]) for k in ('focal_px','lens_height_mm','fork_height_mm')]==[498.4,65.5,93.])
    image_time=3.065  # host frame receipt relative to wheelstand command
    x,y,yaw=[np.interp(image_time,approach['s'],approach[k]) for k in ('x_mm','y_mm','yaw')]
    forward,left=float(ref['forward_mm']),float(ref['left_mm'])
    target['x']=x+forward*np.cos(yaw)-left*np.sin(yaw)
    target['y']=y+forward*np.sin(yaw)+left*np.cos(yaw)
cube_id=m.body('cube').id
cube=m.joint('cube_root').qposadr[0]
def obs(t,k):
    return np.interp(t,approach['s'],approach[k]) if k in approach.dtype.names else np.interp(t+zero,state['sec'],state[k])
def lowering_goal(elapsed, start_angle):
    # Symmetric velocity-limited profile, from rest, using the stock action bounds.
    goal=mjcf.lift_angle_of_height(v,32)
    distance=max(0,start_angle-goal)
    vmax=min(10.,np.sqrt(200.*distance))
    ramp=vmax/200.
    if vmax==0:return goal
    cruise=max(0.,distance/vmax-ramp)
    end=2*ramp+cruise
    if elapsed<ramp:return start_angle-100.*elapsed**2
    if elapsed<ramp+cruise:return start_angle-.5*vmax*ramp-vmax*(elapsed-ramp)
    return goal+100.*max(0.,end-elapsed)**2

def run(start,limit,onset=None,clear_cube=False,drive_during_descent=True):
    d=mj.MjData(m)
    m.actuator('lift').forcerange[:]=[-limit*.001,limit*.001]
    yaw=obs(start,'yaw')
    d.qpos[:2]=np.array([obs(start,'x_mm'),obs(start,'y_mm')])*.001
    d.qpos[3:7]=[np.cos(yaw/2),0,0,np.sin(yaw/2)]
    d.qpos[cube:cube+2]=np.array([target['x'],target['y']])*.001
    if clear_cube:
        d.qpos[cube:cube+2] += .30*np.array([-np.sin(yaw),np.cos(yaw)])
    d.qpos[cube+3:cube+7]=[np.cos(angle/2),0,0,np.sin(angle/2)]
    lift=mjcf.lift_angle_of_height(v,obs(start,'lift_mm'))
    for name,val in [('lift',lift),('wrist',-lift),('head',obs(start,'head'))]:
        d.joint(name+'_joint').qpos[0]=val
    d.ctrl[m.actuator('head').id]=obs(start,'head')
    d.ctrl[m.actuator('lift').id]=lift
    if held_out:
        mj.mj_forward(m,d)
        depths=[-contact.dist*1000 for contact in d.contact
                if cube_id in m.geom_bodyid[contact.geom] and contact.dist<0
                and 0 not in m.geom_bodyid[contact.geom]]
        initial_xy=d.qpos[cube:cube+2].copy()
    for _ in range(round(.20/dt)):mj.mj_step(m,d)
    if held_out:
        initial_checks.append([start,onset,max(depths,default=0.),
                               np.linalg.norm(d.qpos[cube:cube+2]-initial_xy)*1000])
    center=d.qpos[cube:cube+2].copy()
    speed=np.mean([obs(start,'left'),obs(start,'right')])
    d.qvel[:2]=.001*speed*np.array([np.cos(yaw),np.sin(yaw)])
    for axle,radius in [('f',v['wheel_front_radius_mm']),('r',v['wheel_rear_radius_mm'])]:
        for side,key in [('l','left'),('r','right')]:
            d.joint('wheel_'+axle+side+'_joint').qvel[0]=obs(start,key)/radius
    controllers=[TreadController(v),TreadController(v)]
    sensor=CubeAccelerometer(v,sensor_dt,seed=17)
    trace=[];peak_load=0
    drive_ended=False
    lowering_start=mjcf.lift_angle_of_height(v,obs(onset,"lift_mm")) if onset is not None else lift
    support_test='--rear-support' in sys.argv
    if support_test:
        table=m.geom('table').id
        rear_support={m.geom('tip_rear_'+side).id for side in ('l','r')}
        rear_wheels={m.geom('wheel_r'+side+'_geom').id for side in ('l','r')}
    for i,t in enumerate(np.arange(start,5.8+shift,dt)):
        if i%round(.001*v['control_period_ms']/dt)==0:
            lowering=onset is not None and t>=onset
            if lowering:
                chassis=m.jnt_bodyid[m.joint('root').id]
                pitch_rad=np.arcsin(np.clip(d.xmat[chassis].reshape(3,3)[2,0],-1,1))
                drive_ended |= pitch_rad>1.2 or t-onset>.5
            for controller,side in zip(controllers,['left','right']):
                measured=d.joint('wheel_f'+side[0]+'_joint').qvel[0]*v['wheel_front_radius_mm']
                requested=obs(t,side) if t<4.84+shift else 0
                if lowering:requested=0 if drive_ended or not drive_during_descent else 150
                duty=controller.update(requested,measured)
                d.actuator(side+'_wheel').ctrl[0]=duty*3.92/v['motor_reference_voltage_v']
            d.actuator('lift').ctrl[0]=(lowering_goal(t-onset,lowering_start) if lowering
                                           else mjcf.lift_angle_of_height(v,obs(t,'lift_mm')))
            d.actuator('head').ctrl[0]=obs(t,'head')
        mj.mj_step(m,d)
        downward=0.
        support_loads=np.zeros(3) if support_test else None
        for j,c in enumerate(d.contact):
            a,b=m.geom_bodyid[c.geom]
            if support_test and table in c.geom and cube_id not in (a,b):
                other=c.geom[1] if c.geom[0]==table else c.geom[0]
                index=0 if other in rear_support else 1 if other in rear_wheels else 2
                wrench=np.zeros(6)
                mj.mj_contactForce(m,d,j,wrench)
                world_force=c.frame.reshape(3,3).T@wrench[:3]
                support_loads[index]+=world_force[2]*(1 if c.geom[0]==table else -1)
            if cube_id in (a,b) and m.body('lift_fork').id in (a,b):
                wrench=np.zeros(6);mj.mj_contactForce(m,d,j,wrench)
                force=c.frame.reshape(3,3).T@wrench[:3]
                downward+=force[2]*(1 if a==cube_id else -1)
        peak_load=max(peak_load,downward)
        if (i+1)%4:continue
        sensor.step((i+1)*dt,d.sensor('cube_accel').data/(v['gravity_mmps2']*.001))
        rot=d.xmat[cube_id].reshape(3,3)
        top=d.xpos[cube_id,:2]+.022*rot[:2,2]
        pitch=np.rad2deg(np.arcsin(np.clip(d.xmat[m.jnt_bodyid[m.joint('root').id]].reshape(3,3)[2,0],-1,1)))
        trace.append([t,pitch,np.linalg.norm(top-center)*1000,np.rad2deg(np.arctan2(rot[1,0],rot[0,0])-angle),
                      mjcf.lift_height_of_angle(v,float(d.joint("lift_joint").qpos[0])),
                      1000*d.actuator("lift").force[0],downward,float(d.sensor("gyro").data[1]),
                      *[float(d.joint('wheel_f'+side+'_joint').qvel[0])*v['wheel_front_radius_mm']
                        for side in ('l','r')]])
        if support_test:trace[-1].extend(support_loads)
    packets=np.array([[start+t,np.linalg.norm(a)/8192,p[1]] for t,p in sensor.messages for a in np.frombuffer(p[2:],'<i2').reshape(3,3)])
    return np.array(trace),packets,len(sensor.taps.events),peak_load

initial_checks=[]
firmware='--firmware' in sys.argv[2:]
if '--tap-phase' in sys.argv:
    raw=np.genfromtxt(here/'cube.csv.gz',delimiter=',',names=True)
    assert np.all(raw['slot'].reshape(-1,3)==[0,1,2])
    native_packets=raw[::3]
    def receipts(t,counter):
        idx=np.flatnonzero(np.diff(counter)!=0)+1
        assert np.all((counter[idx]-counter[idx-1])%256==1)
        return t[idx]
    recorded=receipts(native_packets['sec']-zero,native_packets['tap'])
    fig,ax=plt.subplots(figsize=(8,3.7),layout='constrained')
    labels=[]
    with (here/'tap-phase.csv').open('w') as f:
        writer=csv.writer(f,lineterminator='\n')
        writer.writerow(['release_s','onset_s','shared_end_s','stream','event_times_s_from_command'])
        for start in (3.5+shift,3.8+shift):
            for onset in (early_onset,late_onset):
                _,pk,count,_=run(start,30,onset)
                packets=pk[::3]
                taps=receipts(packets[:,0],packets[:,2])
                assert len(taps)==count
                lo=max(packets[0,0],native_packets['sec'][0]-zero)
                hi=min(packets[-1,0],native_packets['sec'][-1]-zero)
                label=f'Release {start:.2f}, onset {onset:.2f}'
                row=len(labels)
                labels.append(label)
                for offset,events,name,color in ((.12,recorded,'Recorded','black'),(-.12,taps,'Replay','tab:blue')):
                    within=events[(events>=lo)&(events<=hi)]
                    writer.writerow([start,onset,hi,name,' '.join(f'{t:.6f}' for t in within)])
                    ax.scatter(within-late_onset,np.full(len(within),row+offset),color=color,
                               marker='|',s=160,label=name if row==0 else None)
    ax.axvline(0,color='0.6',lw=.8)
    ax.set(yticks=range(len(labels)),yticklabels=labels,xlabel='Seconds from last raised-lift sample')
    ax.legend()
    fig.savefig(here/'tap-phase.png',dpi=150)
    sys.exit(0)
if '--rear-support' in sys.argv:
    fig,axes=plt.subplots(2,1,figsize=(8,6),sharex=True,layout='constrained')
    axes[0].plot(state['sec']-zero-late_onset,np.degrees(state['pitch']),'.-',color='black',label='Recorded')
    with (here/'rear-support.csv').open('w') as f:
        writer=csv.writer(f,lineterminator='\n')
        writer.writerow(['cap_mNm','rear_support_mu','fraction_robot_table_impulse_on_support_300ms',
                         'fraction_robot_table_impulse_on_rear_wheels_300ms','peak_pitch_300ms_deg',
                         'peak_pitch_deg'])
        for cap in (30,80):
            for mu in (v['tread_friction'],.001):
                for side in ('l','r'):
                    m.geom('tip_rear_'+side).friction[0]=mu
                tr,*_=run(3.5+shift,cap,late_onset)
                t=tr[:,0]-late_onset
                use=(t>=0)&(t<=.30)
                impulse=np.trapezoid(tr[use,10:13],t[use],axis=0)
                writer.writerow([cap,mu,*(impulse[:2]/sum(impulse)),max(tr[use,1]),max(tr[:,1])])
                label=f'{cap} mN m, rear support μ={mu:g}'
                axes[0].plot(t,tr[:,1],label=label)
                axes[1].plot(t,tr[:,10])
    axes[0].set_ylabel('Chassis pitch (°)')
    axes[0].legend(fontsize=8,ncol=2)
    axes[1].set(ylabel='Rear support load (N)',xlabel='Seconds from last raised lift',xlim=(-.1,.6))
    fig.savefig(here/'rear-support.png',dpi=150)
    sys.exit(0)
if '--descent-drive' in sys.argv:
    fig,axes=plt.subplots(3,1,sharex=True,figsize=(9,7),layout='constrained')
    def score(label,t,left,right,pitch):
        style='.-' if label=='Recorded' else '-'
        for ax,series in zip(axes,(left,right,pitch)):
            ax.plot(t,series,style,label=label)
        # Compare simulated shaft travel and reported encoder travel on the same time window.
        times=np.r_[0.,t[(t>0)&(t<.3)],.3]
        paths=[np.trapezoid(np.interp(times,t,speed),times) for speed in (left,right)]
        sdk_time=state['sec']-zero-late_onset
        use=(sdk_time>=0)&(sdk_time<=.3)
        rms=[np.sqrt(np.mean((np.interp(sdk_time[use],t,speed)-state[key][use])**2))
             for speed,key in ((left,'left'),(right,'right'))]
        writer.writerow([label,*paths,max(np.interp(times,t,pitch)),max(pitch[t>=0]),sum(use),*rms])
    with (here/'descent-drive.csv').open('w') as f:
        writer=csv.writer(f,lineterminator='\n')
        writer.writerow(['case','left_mm_300ms','right_mm_300ms','peak_pitch_300ms_deg','peak_pitch_deg',
                         'sdk_samples_300ms','left_rmse_mm_s','right_rmse_mm_s'])
        score('Recorded',state['sec']-zero-late_onset,state['left'],state['right'],np.degrees(state['pitch']))
        for cap in (30,80):
            for powered in (True,False):
                tr,*_=run(3.5+shift,cap,late_onset,drive_during_descent=powered)
                label=f'{cap} mN m, '+('drive' if powered else 'coast')
                score(label,tr[:,0]-late_onset,tr[:,8],tr[:,9],tr[:,1])
    for ax,label in zip(axes,('Left wheel (mm/s)','Right wheel (mm/s)','Pitch (°)')):
        ax.set(ylabel=label,xlim=(-.15,.6))
        ax.axvline(0,color='0.7',lw=.8)
        ax.relim(visible_only=True)
    axes[0].legend(ncol=2,fontsize=8)
    axes[-1].set_xlabel('Seconds from last raised-lift sample')
    fig.savefig(here/'descent-drive.png',dpi=150)
    sys.exit(0)
if '--tip-rate' in sys.argv:
    fig,axes=plt.subplots(2,1,sharex=True,figsize=(8,5),layout='constrained')
    def compare_rate(label,t,height,pitch,rate):
        hits=np.flatnonzero((height[:-1]>35)&(height[1:]<=35))
        if not len(hits):
            writer.writerow([label,'','','','',''])
            return
        i=hits[0]
        alpha=(height[i]-35)/(height[i]-height[i+1])
        crossing=t[i]+alpha*(t[i+1]-t[i])
        angle_at_crossing=pitch[i]+alpha*(pitch[i+1]-pitch[i])
        following=pitch[(t>=crossing)&(t<=crossing+.30)]
        writer.writerow([label,t[i+1]-t[i],rate[i],rate[i+1],angle_at_crossing,
                         max(following)-angle_at_crossing])
        style='.-' if label=='P2Y8' else '-'
        axes[0].plot(t-crossing,rate,style,label=label)
        axes[1].plot(t-crossing,pitch,style)
    with (here/'tip-rate.csv').open('w') as f:
        writer=csv.writer(f,lineterminator='\n')
        writer.writerow(['case','crossing_bracket_s','gyro_before_deg_s','gyro_after_deg_s',
                         'pitch_at_35mm_deg','additional_peak_pitch_in_300ms_deg'])
        take=(state['sec']-zero>=late_onset-.1)
        compare_rate('P2Y8',state['sec'][take]-zero,state['lift'][take],
                     np.degrees(state['pitch'][take]),-np.degrees(state['gy'][take]))
        for cap in (30,80):
            tr,*_=run(3.5+shift,cap,late_onset)
            compare_rate(f'{cap} mN m',tr[:,0],tr[:,4],tr[:,1],-np.degrees(tr[:,7]))
    axes[0].set(ylabel='Head gyro, nose-up (°/s)')
    axes[1].set(ylabel='Chassis pitch (°)',xlabel='Seconds from descending through 35 mm',
                xlim=(-.15,.30))
    for ax in axes:ax.axvline(0,color='0.6',lw=.8)
    axes[0].legend()
    fig.savefig(here/'tip-rate.png',dpi=155)
    sys.exit(0)
if '--effort' in sys.argv:
    fig,axes=plt.subplots(2,1,figsize=(8,5),sharex=True,layout='constrained')
    with (here/'lift-effort.csv').open('w') as f:
        writer=csv.writer(f,lineterminator='\n')
        writer.writerow(['cap_mNm','onset_s','loaded_samples_first_160ms','lowering_saturated_samples','mean_downward_torque_mNm','first_load_ms'])
        for cap in (30,80):
            for onset in (early_onset,late_onset):
                tr,*_=run(3.5+shift,cap,onset)
                t=tr[:,0]-onset
                loaded=(t>=0)&(t<.160)&(tr[:,6]>.05)
                effort=-tr[loaded,5]
                writer.writerow([cap,onset,len(effort),sum(effort>=.99*cap),
                                 np.mean(effort) if len(effort) else '',
                                 1000*t[loaded][0] if len(effort) else ''])
                label=f'{cap} mN m, onset {onset:.3f} s'
                axes[0].plot(t,-tr[:,5],label=label)
                axes[1].plot(t,tr[:,6])
    axes[0].set(ylabel='Lowering torque (mN m)')
    axes[0].legend(fontsize=8)
    axes[1].set(xlim=(-.02,.32),xlabel='Seconds from assumed command',ylabel='Downward fork load (N)')
    fig.savefig(here/'lift-effort.png',dpi=150)
    sys.exit(0)
if '--clear-cube' in sys.argv:
    fig,ax=plt.subplots(figsize=(8,4),layout='constrained')
    with (here/'clear-cube.csv').open('w') as f:
        writer=csv.writer(f,lineterminator='\n')
        writer.writerow(['case','onset_s','elapsed_since_onset_70mm_s','elapsed_since_onset_55mm_s','peak_downward_fork_N'])
        for onset in (early_onset,late_onset):
            for clear in (False,True):
                tr,_,_,load=run(3.5+shift,30,onset,clear_cube=clear)
                if clear:assert load==0
                label='Cube aside' if clear else 'Cube in place'
                row=[label,onset]
                for h in (70,55):
                    hits=np.flatnonzero((tr[:-1,4]>h)&(tr[1:,4]<=h)&(tr[1:,0]>=onset))
                    if len(hits):
                        i=hits[0]
                        frac=(tr[i,4]-h)/(tr[i,4]-tr[i+1,4])
                        row.append(tr[i,0]+frac*(tr[i+1,0]-tr[i,0])-onset)
                    else:row.append('')
                writer.writerow(row+[load])
                ax.plot(tr[:,0]-late_onset,tr[:,4],label=f'{label}, command {onset:.3f} s')
    ax.plot(approach['s']-late_onset,approach['lift_mm'],'k.-',label='Physical wheelstand')
    ax.set(xlim=(-.12,.70),xlabel='Seconds from last raised-lift sample',ylabel='Lift height (mm)')
    ax.legend(fontsize=8)
    fig.savefig(here/'clear-cube.png',dpi=155)
    sys.exit(0)
if '--lift-phase' in sys.argv:
    if not firmware:
        raise ValueError('Lift phase comparison requires firmware targets')
    fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
    def phase(label,t,height,pitch):
        peak=np.argmax(pitch)
        axes[0].plot(t-late_onset,height,label=label)
        axes[1].plot(height[:peak+1],pitch[:peak+1],label=label)
        for threshold in (70,55,40,35):
            crossed=np.flatnonzero((height[:-1]>threshold)&(height[1:]<=threshold))
            if len(crossed):
                i=crossed[0]
                f=(height[i]-threshold)/(height[i]-height[i+1])
                w.writerow([label,threshold,t[i]+f*(t[i+1]-t[i])-late_onset,
                            pitch[i]+f*(pitch[i+1]-pitch[i]),pitch[peak]])
            else:w.writerow([label,threshold,'','',pitch[peak]])
    with (here/'lift-phase.csv').open('w') as out:
        w=csv.writer(out,lineterminator='\n')
        w.writerow(['case','descending_height_mm','s_from_last_raised_sample','pitch_at_height_deg','peak_pitch_deg'])
        use=(approach['s']>=late_onset-.10)
        phase('Physical',approach['s'][use],approach['lift_mm'][use],np.degrees(approach['pitch'][use]))
        for cap in (10,30,80):
            tr,pk,taps,load=run(3.5+shift,cap,late_onset)
            phase(f'{cap} mN m',tr[:,0],tr[:,4],tr[:,1])
    axes[0].set(xlim=(-.1,.65),xlabel='Seconds from last raised-lift sample',ylabel='Lift height (mm)')
    axes[1].set(xlabel='Lift height (mm)',ylabel='Chassis pitch through peak (degrees)')
    axes[1].invert_xaxis()
    axes[0].legend()
    fig.savefig(here/'lift-phase.png',dpi=150)
    sys.exit(0)
fig,axes=plt.subplots(4 if firmware else 3,1,sharex=True,figsize=(9,9 if firmware else 7),layout='constrained')
if firmware:axes[3].plot(approach['s'],approach['lift_mm'],color='black')
axes[0].plot(approach['s'],np.rad2deg(approach['pitch']),color='black',label='Physical')
if not held_out:
    opt=np.genfromtxt(here/'optical-travel.csv',delimiter=',',names=True)
    ok=opt['correlation']>.7
    axes[1].scatter(opt['s_after_command'][ok],opt['travel_mm'][ok],color='black',s=9)
force=np.genfromtxt(here/'cube.csv.gz',delimiter=',',names=True)
axes[2].scatter(force['sec']-zero,np.linalg.norm([force[k] for k in 'xyz'],axis=0)/8192,color='black',s=4)
stem='firmware-drive' if firmware else 'free-chassis'
if camera_xy:stem='camera-xy-drive'
with (here/(stem+'.csv')).open('w') as f:
    writer=csv.writer(f,lineterminator='\n')
    writer.writerow((['onset_s'] if firmware else [])+['release_s','limit_mNm','peak_pitch_deg','settled_travel_mm','settled_yaw_deg','peak_g','taps','peak_downward_fork_N'])
    cases=([(start+shift,30,onset) for start in (3.5,3.8) for onset in (early_onset,late_onset)] if firmware
           else [(3.5,30,None),(3.8,30,None),(3.8,10,None),(3.8,80,None)])
    for start,limit,onset in cases:
        tr,pk,taps,load=run(start,limit,onset)
        writer.writerow(([onset] if firmware else [])+[start,limit,tr[:,1].max(),*np.median(tr[tr[:,0]>5.5+shift,2:4],axis=0),pk[:,1].max(),taps,load])
        if limit==30:
            for ax,j in zip(axes[:2],[1,2]):ax.plot(tr[:,0],tr[:,j],label=(f'{start:.1f} s release, {onset:.3f} s command' if firmware else f'Free from {start} s'))
            axes[2].plot(pk[:,0],pk[:,1])
            if firmware:axes[3].plot(tr[:,0],tr[:,4])
for ax,label in zip(axes,['Chassis pitch (°)','Top glyph travel (mm)','Cube specific force (g)','Lift height (mm)']):
    ax.set(xlim=(3.45+shift,5.75+shift),ylabel=label)
axes[0].legend();axes[-1].set_xlabel('Seconds after wheelstand command')
fig.savefig(here/(stem+'.png'),dpi=155)

if held_out:
    with (here/('camera-xy-contact-check.csv' if camera_xy else 'initial-contact-check.csv')).open('w') as f:
        writer=csv.writer(f,lineterminator='\n')
        writer.writerow(['release_s','onset_s','initial_robot_cube_overlap_mm','cube_settling_displacement_mm'])
        writer.writerows(initial_checks)

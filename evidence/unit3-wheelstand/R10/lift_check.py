"""Inspect shoulder tracking in the impact fixture at unchanged motor torque limit."""
import csv
import matplotlib.pyplot as plt
import numpy as np
from simulacrum import params


class LiftCheck:
    def __init__(self,folder):
        self.folder=folder
        self.values=params.values(params.load(model="vector_2"))
        self.rows=[]

    def override(self,gain):
        # Keep the damping ratio of the linear servo approximately constant.
        kp='lift_servo_kp_mNm_per_rad'
        kv='lift_servo_kv_mNms_per_rad'
        return {kp:self.values[kp]*gain,kv:self.values[kv]*np.sqrt(gain)}

    def begin(self):
        self.samples=[]

    def sample(self,data):
        self.samples.append([data.joint('lift_joint').qpos[0],data.joint('wrist_joint').qpos[0],
                             data.actuator('lift').force[0]])

    def finish(self,view,gain,start,dt,out,state,dock):
        times=start+(np.arange(len(out))+1)*dt
        shoulder,wrist,torque=np.asarray(self.samples).T
        height=self.values['lift_base_position_mm'][2]+self.values['lift_arm_length_mm']*np.sin(shoulder)
        yaw=np.degrees(np.unwrap(out[:,2])-out[0,2])
        samples=state[(state['sec']>=dock+4.15)&(state['sec']<=dock+4.83)]
        for r in samples:
            t=r['sec']-dock
            vals=[np.interp(t,times,v) for v in (height,np.degrees(wrist+shoulder),torque*1000,yaw)]
            self.rows.append(dict(view=view,gain=gain,dock_s=t,encoder_mm=r['lift'],sim_mm=vals[0],
                                  wrist_pitch_relative_deg=vals[1],servo_mNm=vals[2],cube_ccw_deg=vals[3]))

    def save(self):
        with (self.folder/'impact-lift.csv').open('w') as stream:
            w=csv.DictWriter(stream,fieldnames=self.rows[0],lineterminator='\n')
            w.writeheader()
            w.writerows(self.rows)
        with (self.folder/'impact-path-camera.csv').open() as stream:
            cam=[r for r in csv.DictReader(stream) if r['view']=='A' and 4.15<float(r['dock_s'])<4.83]
        fig,axes=plt.subplots(2,2,sharex=True,figsize=(9,6),layout='constrained')
        for col,view in enumerate('AB'):
            for gain in (1,4,16):
                rows=[r for r in self.rows if (r['view'],r['gain'])==(view,gain)]
                t=[r['dock_s'] for r in rows]
                for ax,key in zip(axes[:,col],('sim_mm','cube_ccw_deg')):
                    ax.plot(t,[r[key] for r in rows],'.-',label=f'{gain}× gain')
            axes[0,col].plot(t,[r['encoder_mm'] for r in rows],'ko',ms=3,label='Encoder')
            axes[1,col].plot([float(r['dock_s']) for r in cam],[float(r['fit_ccw_deg']) for r in cam],
                             'ko',ms=3,label='Camera A fit')
            axes[0,col].set_title('Initial registration '+view)
            axes[1,col].set_xlabel('Seconds from dock command')
            axes[1,col].legend(fontsize=8)
        axes[0,0].set_ylabel('Shoulder-derived height (mm)')
        axes[1,0].set_ylabel('Cube yaw (° CCW)')
        axes[0,0].legend(fontsize=8)
        fig.savefig(self.folder/'impact-lift.png',dpi=150)

"""Record angular impulse about the cube COM, split by contact partner."""
import gzip
from pathlib import Path
import mujoco as mj
import numpy as np

HERE=Path(__file__).resolve().parent
PARTS=('fork','shelf','tread','other')


class ImpactWrench:
    def __init__(self):
        self.rows=[]

    def sample(self, model, data):
        cube=model.body('cube').id
        sums=np.zeros((len(PARTS),4))
        wrench=np.empty(6)
        for j,c in enumerate(data.contact):
            a,b=model.geom_bodyid[c.geom]
            if cube not in (a,b):
                continue
            other=a if b==cube else b
            group=1 if other==0 else 0 if other==model.body('fork').id else 2 if other==model.body('tread').id else 3
            mj.mj_contactForce(model,data,j,wrench)
            force,moment=(1 if b==cube else -1)*(wrench.reshape(2,3)@c.frame.reshape(3,3))
            arm=c.pos-data.xipos[cube]
            sums[group]+=[force[0],force[1],force[2],arm[0]*force[1]-arm[1]*force[0]+moment[2]]
        # Freejoint angular velocity uses the body's local coordinates. The fixture
        # has centered, isotropic inertia, so Lz = I * world angular velocity z.
        inertia=model.body_inertia[cube]
        assert np.allclose(inertia,inertia[0],rtol=1e-7,atol=0)
        vel=data.joint('cube_root').qvel
        world_wz=data.xmat[cube].reshape(3,3)[2]@vel[3:]
        self.rows.append([data.time,*sums.ravel(),inertia[0]*world_wz])

    def save(self,view,offset,dt):
        a=np.asarray(self.rows)
        t=a[:,0]+offset
        # N s and N m s, integrated from the stationary replay initialization.
        totals=np.cumsum(a[:,1:-1],axis=0)*dt
        headers=[p+'_'+k for p in PARTS for k in ('jx_Ns','jy_Ns','vertical_Ns','jz_Nms')]
        predicted=totals[:,3::4].sum(axis=1)
        residual=np.max(abs(predicted-a[:,-1]))
        print(view,'maximum angular momentum closure error (N m s)',residual,flush=True)
        assert residual < max(abs(a[:,-1]))*.02
        with gzip.open(HERE/f'impact-impulse-{view}.csv.gz','wt') as stream:
            np.savetxt(stream,np.column_stack([t,totals,a[:,-1]])[::4],delimiter=',',fmt='%.10g',
                       header=','.join(['dock_s',*headers,'cube_Lz_Nms']),comments='')

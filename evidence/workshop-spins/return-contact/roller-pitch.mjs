import {writeFileSync} from 'node:fs';
import {mujoco,readJson,readText} from '../../../web/test/helpers.js';
import {VectorSim,CONTROL_OVERRIDE} from '../../../web/src/sim/robot.js';
import {rearRollers} from './rear-rollers.mjs';

const mj=await mujoco(), model=readJson('public/robot/model.json');
const original=readText('public/robot/vector-free-dock.xml');
const rows=['variant,direction,threshold_Nm,base_deg,peak_abs_deg'];
for (const mode of ['default','welded','passive','driven']) {
  for (const direction of [-1,1]) {
    const xml=mode==='default'?original:rearRollers(original,mode);
    const sim=new VectorSim(mj,{xml,...model,controlLevel:CONTROL_OVERRIDE,headDeg:22,liftMm:32,noiseSeed:1});
    try {
      const chassis=mj.mj_name2id(sim.model,mj.mjtObj.mjOBJ_BODY.value,'chassis');
      sim.setWheelMotors(0,0);
      sim.tick(100);
      const base=sim._pitch()*180/Math.PI;
      let threshold=null,peak=0;
      for(let i=0;i<2000;i++) {
        const torque=(i+1)*.00005;
        sim.data.xfrc_applied[6*chassis+4]=direction*torque;
        sim.tick(1);
        const pitch=Math.abs(sim._pitch()*180/Math.PI-base);
        peak=Math.max(peak,pitch);
        if(pitch>=10) { threshold=torque;break; }
      }
      rows.push([mode,direction,threshold,base,peak].join(','));
    } finally {sim.dispose();}
  }
}
writeFileSync(new URL('roller-pitch.csv',import.meta.url),rows.join('\n')+'\n');

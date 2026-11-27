import {writeFileSync} from 'node:fs';
import {mujoco,readJson,readText} from '../../../web/test/helpers.js';
import {VectorSim,CONTROL_OVERRIDE} from '../../../web/src/sim/robot.js';

const runtime = await mujoco();
const model = readJson('public/robot/model.json');
const original = readText('public/robot/vector-free-dock.xml');
const rows = ['variant,lateral_mm,yaw_deg,contact,time_s,min_pitch_deg,max_depth_mm'];
for (const variant of ['default','clear','axles']) {
  let xml=original;
  if (variant !== 'default') {
    xml=xml.replace(/(<geom name="tip_rear_[lr]"[^>]*?contype=")1/g,'$12');
    xml=xml.replace(/(<geom name="(?:table|floor)"[^>]*?conaffinity=")1/g,'$13');
  }
  if (variant === 'axles') {
    xml=xml.replace(/(<body name="wheel_r[lr]" pos=")-0.04459/g,'$1-0.048')
           .replace(/(<body name="wheel_f[lr]" pos=")-0.00341/g,'$10.000');
  }
  for (const [lat,yaw] of [[0,0],[-3,-1],[3,1],[-3,1],[3,-1]]) {
    const sim = new VectorSim(runtime,{xml,values:model.values,derived:model.derived,camera:model.camera,
      startXyMm:[-20,lat],headingDeg:yaw,headDeg:0,liftMm:45,controlLevel:CONTROL_OVERRIDE,noiseSeed:1});
    let pitch=0,depth=-Infinity;
    try {
      if (!sim.setWheelMotors(-30,-30)) throw Error('reverse command refused');
      for(let i=0;i<Math.ceil(8/sim.dt) && !sim.chargerContact;i++){
        sim.tick(1);
        pitch=Math.min(pitch,sim.state.pose_pitch_rad*180/Math.PI);
        depth=Math.max(depth,-120-sim.poseMm()[0]);
      }
      rows.push([variant,lat,yaw,Number(sim.chargerContact),sim.tick_*sim.dt,pitch,depth].join(','));
    } finally {sim.dispose();}
  }
}
writeFileSync(new URL('bench.csv',import.meta.url),rows.join('\n')+'\n');

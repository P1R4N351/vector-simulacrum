import {readFileSync,writeFileSync} from 'node:fs';
import {mujoco,readJson,readText,sha256} from '../../../web/test/helpers.js';
import {VectorSim,CONTROL_OVERRIDE} from '../../../web/src/sim/robot.js';
import {rearRollers} from './rear-rollers.mjs';
const local = name => new URL(name,import.meta.url);
const runs = JSON.parse(readFileSync(local('../web-replay/inputs.json')));
const model = readJson('public/robot/model.json');
const mj = await mujoco();
const rows = ['scene,session,turn,t_s,head_rad,gx,gz,left,right'];
for (const name of ['passive','driven']) {
  const original = readText('public/robot/vector-free-dock.xml');
  const xml = rearRollers(original,name);
  if (sha256(original)!==model.source['vector-free-dock_sha256']) throw Error('Asset checksum mismatch');
  for (const run of runs.filter(r=>r.session==='interleaved')) {
    const sim=new VectorSim(mj,{xml, values:model.values,derived:model.derived,camera:model.camera,
      noiseSeed:1,controlLevel:CONTROL_OVERRIDE,chargerContact:false,
      headDeg:run.head_rad*180/Math.PI,liftMm:run.lift_mm});
    try {
      const pre=Math.round(.30/sim.dt);
      const end=pre+Math.round(run.duration/sim.dt);
      for (let tick=0;tick<end+Math.round(.66/sim.dt);tick++) {
        if (tick===pre && !sim.setWheelMotors(-run.speed,run.speed)) throw Error('wheel command refused');
        if (tick===end && !sim.setWheelMotors(0,0)) throw Error('stop refused');
        sim.tick(1);
        const s=sim.state;
        if (Math.abs(s.time_s-(tick+1)*sim.dt)>1e-8) continue;
        rows.push([name,run.session,run.turn,s.time_s-pre*sim.dt,s.head_angle_rad,
          s.gyro_radps[0],s.gyro_radps[2],s.left_wheel_speed_mmps,s.right_wheel_speed_mmps].join(','));
      }
    } finally {sim.dispose();}
  }
}
writeFileSync(local('roller-spins.csv'),rows.join('\n')+'\n');

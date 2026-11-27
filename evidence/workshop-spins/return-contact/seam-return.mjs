import {writeFileSync} from 'node:fs';
import {mujoco,readJson,readText} from '../../../web/test/helpers.js';
import {VectorSim} from '../../../web/src/sim-dock-trial/robot.js';

import {rearRollers} from './rear-rollers.mjs';
import {closeSeams} from './close-seams.mjs';
const mode=process.argv[2];
const runtime = await mujoco();
const model = readJson('public/robot/model.json');
const sim = new VectorSim(runtime, {
  xml: rearRollers(closeSeams(readText('public/robot/vector-free-dock.xml')),mode), values: model.values,
  derived: model.derived, camera: model.camera, ...(process.argv[3] ? {startXyMm:[100,Number(process.argv[3])],headingDeg:45*Math.sign(Number(process.argv[3]))} : {startDocked:true}), noiseSeed:1,
});
const rows = ['time_s,pitch_rad,left_mm_s,right_mm_s,x_mm,y_mm,on'];
try {
  if (!process.argv[3]) {
  if (!sim.driveOffCharger()) throw Error('Undock refused');
  for (let i=0; i<1500 && sim.actionRunning; i++) sim.tick(1);
  if (sim.lastAction.result !== 'completed') throw Error('Undock failed');
  }
  if (!sim.dockWithCharger()) throw Error('Return unavailable');
  let last = -1;
  for (let i=0; i<30000 && sim.actionRunning; i++) {
    sim.tick(1);
    const s = sim.state;
    if (s.time_s === last) continue;
    last = s.time_s;
    rows.push([last,s.pose_pitch_rad,s.left_wheel_speed_mmps,s.right_wheel_speed_mmps,
      s.x_mm,s.y_mm,Number(sim.chargerContact)].join(','));
  }
  console.log(sim.lastAction, sim.chargerContact);
  writeFileSync(new URL('seam-'+mode+(process.argv[3] || '')+'.csv',import.meta.url),rows.join('\n')+'\n');
} finally {
  sim.dispose();
}

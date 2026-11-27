import {writeFileSync} from 'node:fs';
import {mujoco,readJson,readText} from '../../../web/test/helpers.js';
import {VectorSim} from '../../../web/src/sim/robot.js';

const runtime = await mujoco();
const model = readJson('public/robot/model.json');
const sim = new VectorSim(runtime, {
  xml: readText('public/robot/vector-free-dock.xml'), values: model.values,
  derived: model.derived, camera: model.camera, startDocked: true, noiseSeed: 1,
});
const rows = ['time_s,pitch_rad,left_mm_s,right_mm_s,x_mm,y_mm,on'];
try {
  if (!sim.driveOffCharger()) throw Error('Undock refused');
  for (let i=0; i<1500 && sim.actionRunning; i++) sim.tick(1);
  if (sim.lastAction.result !== 'completed' || !sim.dockWithCharger()) throw Error('Return unavailable');
  let last = -1;
  for (let i=0; i<14000 && sim.actionRunning; i++) {
    sim.tick(1);
    const s = sim.state;
    if (s.time_s === last) continue;
    last = s.time_s;
    rows.push([last,s.pose_pitch_rad,s.left_wheel_speed_mmps,s.right_wheel_speed_mmps,
      s.x_mm,s.y_mm,Number(sim.chargerContact)].join(','));
  }
  if (sim.lastAction.result !== 'completed' || !sim.chargerContact) throw Error('Dock failed');
  writeFileSync(new URL('sim.csv',import.meta.url),rows.join('\n')+'\n');
} finally {
  sim.dispose();
}

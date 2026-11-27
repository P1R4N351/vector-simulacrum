import {writeFileSync} from 'node:fs';
import {mujoco,readJson,readText} from '../../../web/test/helpers.js';
import {VectorSim} from '../../../web/src/sim-dock-trial/robot.js';

const runtime = await mujoco();
const model = readJson('public/robot/model.json');
const sim = new VectorSim(runtime, {
  xml: readText('public/robot/vector-free-dock.xml'), values: model.values,
  derived: model.derived, camera: model.camera, startDocked: true, noiseSeed: Number(process.argv[3] || 1),
});
const names = Array.from({length:sim.model.ngeom}, (_, i) => runtime.mj_id2name(sim.model,runtime.mjtObj.mjOBJ_GEOM.value,i));
const rows=['t,x,y,pitch_deg,stage,left,right,power_l,power_r,pair,normal_N'];
try {
  if (!sim.driveOffCharger()) throw Error('Undock refused');
  for (let i=0; i<1500 && sim.actionRunning; i++) sim.tick(1);
  if (sim.lastAction.result !== 'completed' || !sim.dockWithCharger()) throw Error('Return unavailable');
  let last = -1;
  for (let i=0; i<30000 && sim.actionRunning; i++) {
    if (sim._action?.autonomy?.stage === 'backup' && process.argv[2] === 'clear') {
      for (const name of ['tip_rear_l','tip_rear_r']) sim.model.geom_contype[names.indexOf(name)] = 2;
      for (const name of ['table','floor']) sim.model.geom_conaffinity[names.indexOf(name)] |= 2;
    }
    sim.tick(1);
    const s = sim.state;
    if (s.time_s === last) continue;
    last = s.time_s;
    const stage = sim._action?.autonomy?.stage;
    if (stage !== 'backup') continue;
    const loads = new Map();
    const contacts = sim.data.contact;
    for (let idx=0; idx<sim.data.ncon; idx++) {
      const contact = contacts.get(idx);
      if (contact.efc_address < 0) continue;
      const pair = [names[contact.geom1], names[contact.geom2]];
      if (!pair.some(n => n?.startsWith('wheel_') || n?.startsWith('dock_'))) continue;
      const key = pair.sort().join('/');
      loads.set(key,(loads.get(key) ?? 0) + sim.data.efc_force[contact.efc_address]);
    }
    for (const [pair,force] of loads)
      rows.push([last,s.x_mm,s.y_mm,s.pose_pitch_rad*180/Math.PI,stage,s.left_wheel_speed_mmps,
        s.right_wheel_speed_mmps,...sim.simTruth().motor_power,pair,force].join(','));

  }
  console.log(sim.lastAction, sim.chargerContact);
  writeFileSync(new URL('load-'+process.argv[2]+'-'+process.argv[3]+'.csv',import.meta.url),rows.join('\n')+'\n');
} finally {
  sim.dispose();
}

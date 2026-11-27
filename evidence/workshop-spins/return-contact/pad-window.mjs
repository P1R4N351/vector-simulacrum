import {writeFileSync} from 'node:fs';
import {mujoco,readJson,readText} from '../../../web/test/helpers.js';
import {VectorSim} from '../../../web/src/sim/robot.js';
import {rearRollers} from './rear-rollers.mjs';

const runtime = await mujoco();
const model = readJson('public/robot/model.json');
const width=Number(process.argv[2]);
const rows=['width_mm,start,phase,time_s,contact,depth_mm,pitch_deg,left_mm_s,right_mm_s'];
const outcomes=[];
for(const start of ['seated','direct','diagonal−','diagonal+']){
  const side=start==='diagonal−'?-1:1;
  const pose=start.startsWith('diagonal')?{startXyMm:[100,side*60],headingDeg:side*45}:{startDocked:true};
  const sim=new VectorSim(runtime,{xml:rearRollers(readText('public/robot/vector-free-dock.xml'),'driven'),
    values:model.values,derived:model.derived,camera:model.camera,noiseSeed:1,...pose});
  let last=-1;
  function record(phase) {
    const s=sim.state;
    if(s.time_s===last)return;
    last=s.time_s;
    rows.push([width,start,phase,s.time_s,Number(sim.chargerContact),-120-sim.poseMm()[0],s.pose_pitch_rad*180/Math.PI,
      s.left_wheel_speed_mmps,s.right_wheel_speed_mmps].join(','));
  }
  try {
    if(start==='seated'){
      record('rest');
      for(let i=0;i<600;i++){sim.tick(1);record('rest');}
      continue;
    }
    if(start==='direct'){
      if(!sim.driveOffCharger())throw Error('Undock refused');
      for(let i=0;i<1500 && sim.actionRunning;i++)sim.tick(1);
      if(sim.lastAction.result!=='completed')throw Error('Undock failed');
    }
    if(!sim.dockWithCharger())throw Error('Dock refused');
    for(let i=0;i<15000 && sim.actionRunning;i++){sim.tick(1);record('return');}
    outcomes.push([width,start,sim.lastAction.result].join(','));
    for(let i=0;i<600;i++){sim.tick(1);record('rest');}
  } finally {sim.dispose();}
}
writeFileSync(new URL('pad-'+width+(process.argv[3]==='filtered'?'-filtered':'')+'.csv',import.meta.url),rows.join('\n')+'\n');
console.log(outcomes.join('\n'));

import {VectorSim} from "../../web/src/sim/robot.js";
import {mujoco,readJson,readText} from "../../web/test/helpers.js";
const mj = await mujoco();
const variant = process.argv[2] ?? "baseline";
const headDeg = Number(process.argv[3] ?? 0);
const liftMm = Number(process.argv[4] ?? 32);
const dir = process.argv[5] === "v2" ? "public/robot/vector2" : "public/robot";
const {values,derived} = readJson(dir+"/model.json");
let xml = readText(dir+"/vector-contact42-free-dock.xml");
if (variant === "noslip") xml = xml.replace("<option ", '<option noslip_iterations="4" ');
if (variant === "flat") xml = xml.replace('pos="-0.12 0 0"', 'pos="-0.12 0 -1"');
const sim = new VectorSim(mj,{values,derived,xml,startDocked:variant !== "flat",headDeg,liftMm,calmMode:true,noiseSeed:42});
const joint = name => sim.model.jnt_qposadr[mj.mj_name2id(sim.model,mj.mjtObj.mjOBJ_JOINT.value,name)];
const wheels = ["wheel_fl_joint","wheel_rl_joint","patch_fl","patch_rl"].map(joint);
try {
  console.log("s,x_mm,y_mm,z_mm,pitch_deg,head_deg,ax,front_rad,rear_rad,front_patch_rad,rear_patch_rad,contact");
  for(let tick=0;tick<=12500;tick++) {
    if(tick%50===0) console.log([tick*sim.dt,...sim.poseMm(),sim._pitch()*180/Math.PI,sim.headAngleRad()*180/Math.PI,sim.state.accel_mmps2[0],...wheels.map(i=>sim.data.qpos[i]),Number(sim.chargerContact)].join(","));
    sim.tick(1);
  }
} finally { sim.dispose(); }

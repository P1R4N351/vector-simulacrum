import {readFileSync,writeFileSync} from "node:fs";
import {VectorSim,CONTROL_OVERRIDE} from "../../../web/src/sim/robot.js";
import {mujoco,readJson,readText} from "../../../web/test/helpers.js";

// Optional diagnostic variant writes to a separate output folder.
const [source="./", readyFile, friction, destination, derivative, powerMode, staticFriction] = process.argv.slice(2);
const evidence = new URL(source, import.meta.url);
const output = destination ? new URL(destination,import.meta.url) : evidence;
let xml = readText("public/robot/vector-contact42-free-dock.xml");
if (friction !== undefined) {
  const torque=Number(friction);
  if(!destination || output.href===evidence.href || !Number.isFinite(torque) || torque<0) throw Error("Specify friction and separate destination");
  const joint='<joint name="lift_joint"';
  if(xml.split(joint).length!==2) throw Error("Expected one lift hinge");
  xml=xml.replace(joint,joint+' frictionloss="'+torque+'"');
}
if (derivative && !["error","target"].includes(derivative)) throw Error("Unknown derivative mode");
if (derivative==="error") {
  const actuator=/<position name="lift"[^>]* kv="[^"]+"/;
  if(!actuator.test(xml)) throw Error("Missing lift velocity damping");
  xml=xml.replace(actuator, match=>match.replace(/ kv="[^"]+"/,' kv="0"'));
}
const rows = readFileSync(new URL("recording.csv",evidence),"utf8").trim().split(/\r?\n/).slice(1).map(s=>s.split(","));
const first = rows.find(row=>row[1]==="state");
const commands = rows.filter(row=>row[1]==="command");
const {values,derived} = readJson("public/robot/model.json");
if(powerMode){
  if (!destination || output.href===evidence.href || !["proportional","both"].includes(powerMode)) throw Error("Invalid power scaling diagnostic");
  const stall = values.lift_servo_torque_max_mNm / 1000;
  const kp = stall * values.lift_power_gains[0];
  const ratio = kp / (values.lift_servo_kp_mNm_per_rad / 1000);
  const lift = /<position name="lift"[^>]+>/;
  if (!lift.test(xml)) throw Error("Missing lift actuator");
  xml=xml.replace(lift, actuator => {
    const scaled=actuator.replace(/ kp="[^"]+"/, ' kp="'+kp+'"');
    return powerMode==="both" ? scaled.replace(/ kv="([^"]+)"/, (_,kv)=>' kv="'+(Number(kv)*ratio)+'"') : scaled;
  });
}
const sim = new VectorSim(await mujoco(),{values,derived,xml,
  startDocked:true,controlLevel:CONTROL_OVERRIDE,headDeg:Number(first[5]),liftMm:Number(first[4]),noiseSeed:42});
try {
  if(derivative){
    if(output.href===evidence.href) throw Error("Use separate output for diagnostic replay");
    (await import("./derivative/derivative.mjs")).addDerivative(sim,derivative);
  }
  let updateFriction;
  if (staticFriction !== undefined) {
    const peak = Number(staticFriction);
    const sliding = Number(friction);
    if (!destination || output.href === evidence.href || !Number.isFinite(peak) || !(peak > sliding && sliding > 0)) {
      throw Error("Specify positive sliding friction, higher static friction, and separate output");
    }
    const joint = sim.mujoco.mj_name2id(sim.model, sim.mujoco.mjtObj.mjOBJ_JOINT.value, "lift_joint");
    if (joint < 0) throw Error("Missing lift hinge");
    const dof = sim.model.jnt_dofadr[joint];
    updateFriction = () => {
      const speed = sim.data.qvel[dof];
      sim.model.dof_frictionloss[dof] = sliding + (peak - sliding) * Math.exp(-((speed / 0.1) ** 2));
    };
  }
  const includeFlag = rows.some(r => r[1] === "state" && r.length > 11 && r[11] !== "");
  const samples=[includeFlag ? "s,lift_mm,on_dock,in_position" : "s,lift_mm,on_dock"];
  const finishes=["command_s,target_mm,duration_s,sim_elapsed_s,result"];
  const ready=["command_s,ready_s"];
  let waitingForTarget = null;
  let next=0;
  let pending=null;
  for(const row of rows.filter(r=>r[1]==="state")){
    const receipt=Number(row[0]);
    while(sim.tickCount < Math.floor((receipt-Number(first[0]))/sim.dt)){
      const t=Number(first[0])+sim.tickCount*sim.dt;
      if(next<commands.length && Number(commands[next][0])<=t){
        const cmd=commands[next++];
        pending={cmd,previous:sim.lastAction};
        waitingForTarget=cmd;
        if(!sim.setLiftHeight(Number(cmd[2]),null,Number(cmd[3]))) throw Error("Rejected recorded command at "+t);
      }
      if (updateFriction) updateFriction();
      sim.tick();
      if(waitingForTarget && sim._liftServoCmd===sim._liftTarget){
        ready.push([waitingForTarget[0],Number(first[0])+sim.tickCount*sim.dt].join(","));
        waitingForTarget=null;
      }
      if(pending && sim.lastAction!==pending.previous){
        finishes.push([pending.cmd[0],pending.cmd[2],pending.cmd[3],sim.lastAction.ticks*sim.dt,sim.lastAction.result].join(","));
        pending=null;
      }
    }
    const sample = [receipt,sim.state.lift_height_mm,+sim.chargerContact];
    if(includeFlag) sample.push(+sim.state.lift_in_pos);
    samples.push(sample.join(","));
  }
  if(next!==commands.length || finishes.length!==commands.length+1)throw Error("Incomplete replay");
  if(readyFile){
    if(ready.length!==commands.length+1) throw Error("Missing final servo target");
    writeFileSync(new URL(readyFile,output),ready.join("\n")+"\n");
  }
  writeFileSync(new URL("returns.csv",output),finishes.join("\n")+"\n");
  writeFileSync(new URL("replay.csv",output),samples.join("\n")+"\n");
} finally {sim.dispose();}

import {readFileSync,writeFileSync} from "node:fs";
import {VectorSim,CONTROL_OVERRIDE} from "../../../web/src/sim/robot.js";
import {mujoco,readJson,readText} from "../../../web/test/helpers.js";

const evidence = new URL(process.argv[2] || "./", import.meta.url);
const rows = readFileSync(new URL("recording.csv",evidence),"utf8").trim().split(/\r?\n/).slice(1).map(s=>s.split(","));
const first = rows.find(row=>row[1]==="state");
const commands = rows.filter(row=>row[1]==="command");
const {values,derived} = readJson("public/robot/model.json");
const sim = new VectorSim(await mujoco(),{values,derived,xml:readText("public/robot/vector-contact42-free-dock.xml"),
  startDocked:true,controlLevel:CONTROL_OVERRIDE,headDeg:Number(first[5]),liftMm:Number(first[4]),noiseSeed:42});
try {
  const samples=["s,lift_mm,on_dock"];
  const finishes=["command_s,target_mm,duration_s,sim_elapsed_s,result"];
  let next=0;
  let pending=null;
  for(const row of rows.filter(r=>r[1]==="state")){
    const receipt=Number(row[0]);
    while(sim.tickCount < Math.floor((receipt-Number(first[0]))/sim.dt)){
      const t=Number(first[0])+sim.tickCount*sim.dt;
      if(next<commands.length && Number(commands[next][0])<=t){
        const cmd=commands[next++];
        pending={cmd,previous:sim.lastAction};
        if(!sim.setLiftHeight(Number(cmd[2]),null,Number(cmd[3]))) throw Error("Rejected recorded command at "+t);
      }
      sim.tick();
      if(pending && sim.lastAction!==pending.previous){
        finishes.push([pending.cmd[0],pending.cmd[2],pending.cmd[3],sim.lastAction.ticks*sim.dt,sim.lastAction.result].join(","));
        pending=null;
      }
    }
    samples.push([receipt,sim.state.lift_height_mm,+sim.chargerContact].join(","));
  }
  if(next!==commands.length || finishes.length!==commands.length+1)throw Error("Incomplete replay");
  writeFileSync(new URL("returns.csv",evidence),finishes.join("\n")+"\n");
  writeFileSync(new URL("replay.csv",evidence),samples.join("\n")+"\n");
} finally {sim.dispose();}

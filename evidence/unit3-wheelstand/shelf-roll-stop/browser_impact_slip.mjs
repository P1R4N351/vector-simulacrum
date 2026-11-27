import { readFileSync, writeFileSync } from "node:fs";
import { createHash } from "node:crypto";
import { gunzipSync } from "node:zlib";
import { mujoco, readJson, readText } from "../../../web/test/helpers.js";
import { VectorSim } from "../../../web/src/sim/robot.js";

const file = name => new URL(name, import.meta.url);
const pickup = process.argv[2] === "pickup";
const capture = gunzipSync(readFileSync(file(pickup ? "../R5/cube.csv.gz" : "S22B/cube.csv.gz"))).toString().trim().split(/\r?\n/).slice(1).map(line => line.split(",").map(Number));
const slot = pickup ? 0 : 1;
const at = capture.findIndex(row => row[0] === (pickup ? 39.980848 : 37.020336) && row[2] === slot);
if (at < 0) throw new Error("missing recorded impact");
const observed = capture.slice(at, at+5).map(row => row.slice(4,7).map(x => x/8192));
const pose = readFileSync(file("compound-tilt.csv"),"utf8").split(/\r?\n/).find(line=>line.startsWith("late,flat-corrected,")).split(",");
const gravity = pickup ? readFileSync(file("../R5/carry-attitude.csv"),"utf8").split(/\r?\n/)
  .find(line=>line.startsWith("Post-reverse hold,")).split(",").slice(4,7).map(Number) : pose.slice(2,5).map(Number);
const magnitude = Math.hypot(...gravity);
const [gx,gy,gz] = gravity.map(x=>x/magnitude);
const w = Math.sqrt((1+gz)/2);
const orientation = [w, gy/(2*w), -gx/(2*w), 0];
const mj = await mujoco();
const {values,derived} = readJson("public/robot/vector2/model.json");
const sim = new VectorSim(mj,{values,derived,xml:readText("public/robot/vector2/vector-free.xml")});
// Optional paired sensor experiment. Both paths get the same settling interval.
const packetOutput = process.env.VECTOR_PACKET_CSV;
const firmwarePath = process.env.VECTOR_CUBE_FIRMWARE;
if (firmwarePath) {
  if (!packetOutput) throw new Error("set VECTOR_PACKET_CSV for firmware comparisons");
  const cubeBytes = readFileSync(firmwarePath);
  if (createHash("sha256").update(cubeBytes).digest("hex") !==
      "c53e7f1edae6bb034cd18782d01b8fb01ec4e1200b68187931060be59db02bee") {
    throw new Error("unexpected cube firmware image");
  }
  sim.attachFirmware({cubeBytes});
}
const packets = [["velocity_mmps","gap_mm","phase_ms","step_after_release","packet_ms","taps","slot","x","y","z"]];
const joint = mj.mj_name2id(sim.model,mj.mjtObj.mjOBJ_JOINT.value,"cube_root");
const q = sim.model.jnt_qposadr[joint], dof = sim.model.jnt_dofadr[joint];
const rows = [["velocity_mmps","gap_mm","phase_ms","low_pairs","nearest_peak_x_g","xyz_rmse_g"]];
const travel = [["velocity_mmps","gap_mm","phase_ms","matches_low_pair","x_mm","y_mm","horizontal_mm"]];
const cornerRows = [["velocity_mmps","gap_mm","phase_ms","matches_low_pair","corner","dx_mm","dy_mm","initial_z_mm"]];
function bottomCorners() {
  const rotation = sim.data.xmat, body = sim._cubeBody, pos = sim.data.xpos;
  return [[-1,-1],[1,-1],[1,1],[-1,1]].map(([x,y]) =>
    [0,1,2].map(axis => {
      const k = 9*body+3*axis;
      return 1000*pos[3*body+axis]+22.15*(x*rotation[k]+y*rotation[k+1]-rotation[k+2]);
    }));
}
try {
  for (const velocity of [-150,-75,0,75,150]) for (const gap of [1,2,3]) for (let phase=0;phase<30;phase+=5) {
    sim.reset();
    sim.tick((packetOutput ? 210 : 30)+phase/5);
    if (packetOutput && !sim.cubeSensor.packet) throw new Error("cube sensor has not started");
    sim.data.qpos.set(orientation,q+3);
    mj.mj_forward(sim.model,sim.data);
    let low=Infinity;
    for (let g=0;g<sim.model.ngeom;g++) {
      if (sim.model.geom_bodyid[g]!==sim._cubeBody) continue;
      let bottom=sim.data.geom_xpos[3*g+2];
      for (let k=0;k<3;k++) bottom-=Math.abs(sim.data.geom_xmat[9*g+6+k])*sim.model.geom_size[3*g+k];
      low=Math.min(low,bottom);
    }
    sim.data.qpos[q+2]+=gap/1000-low;
    sim.data.qvel.fill(0,dof,dof+6);
    sim.data.qvel[dof]=velocity/1000;
    mj.mj_forward(sim.model,sim.data);
    const initialXY = sim.data.qpos.slice(q,q+2);
    const initialCorners=bottomCorners();
    const slots=[];
    let previous=sim.cubeSensor.packet;
    for(let i=0;i<40;i++){
      sim.tick(1);
      const p=sim.cubeSensor.packet;
      if(p && p!==previous) {
        if (packetOutput) {
          p.slots.forEach((axes, index) => packets.push([velocity,gap,phase,i+1,
            Math.round(p.time_s*1000),p.taps,index,...axes]));
        }
        slots.push(...p.slots.map(slot=>slot.map(x=>x/values.cube_accel_counts_per_g)));
        previous=p;
      }
    }
    const candidates=[];
    for(let i=slot;i+4<slots.length;i+=3){
      if(Math.max(Math.hypot(...slots[i]),Math.hypot(...slots[i+1]))>=.05) continue;
      const err=slots.slice(i+2,i+5).flatMap((slot,s)=>slot.map((x,k)=>(x-observed[s+2][k])**2));
      candidates.push([Math.sqrt(err.reduce((a,b)=>a+b,0)/err.length),slots[i+3][0]]);
    }
    candidates.sort((a,b)=>a[0]-b[0]);
    const dx=1000*(sim.data.qpos[q]-initialXY[0]), dy=1000*(sim.data.qpos[q+1]-initialXY[1]);
    if (process.argv.includes("--pivot")) {
      mj.mj_forward(sim.model,sim.data);
      bottomCorners().forEach((p,i)=>cornerRows.push([velocity,gap,phase,Number(candidates.length>0),i,
        p[0]-initialCorners[i][0],p[1]-initialCorners[i][1],initialCorners[i][2]]));
    }
    travel.push([velocity,gap,phase,Number(candidates.length>0),dx,dy,Math.hypot(dx,dy)]);
    rows.push([velocity,gap,phase,candidates.length,candidates[0]?.[1]??"",candidates[0]?.[0]??""]);
  }
} finally { sim.dispose(); }
if (packetOutput) writeFileSync(packetOutput, packets.map(row=>row.join(",")).join("\n")+"\n");
else writeFileSync(file(pickup ? "browser-impact-slip-pickup.csv" : "browser-impact-slip.csv"),rows.map(r=>r.join(",")).join("\n")+"\n");

if (process.argv.includes("--travel")) writeFileSync(file("browser-release-travel.csv"),travel.map(r=>r.join(",")).join("\n")+"\n");

if (process.argv.includes("--pivot")) writeFileSync(file("browser-pivot-corners.csv"),cornerRows.map(r=>r.join(",")).join("\n")+"\n");

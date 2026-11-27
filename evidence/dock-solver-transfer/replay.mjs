import {readFileSync} from "node:fs";
import {VectorSim} from "../../web/src/sim/robot.js";
import {mujoco,readJson,readText} from "../../web/test/helpers.js";

const mj = await mujoco();
const trials = new Map();
for (const line of readFileSync(new URL("inputs.csv",import.meta.url),"utf8").trim().split("\n").slice(1)) {
  const [unit,turn,...row] = line.split(",");
  const key = unit + "," + turn;
  if (!trials.has(key)) trials.set(key,[]);
  trials.get(key).push(row);
}
console.log("unit,turn,scene,iterations,t,phase,measured,predicted");
for (const [key,rows] of trials) {
  const dir = key.startsWith("unit2") ? "public/robot/vector2/" : "public/robot/";
  const {values,derived} = readJson(dir+"model.json");
  for (const scene of ["vector-free-dock","vector-contact42-free-dock"]) {
    for (const iterations of [0,4]) {
      const xml = readText(dir+scene+".xml").replace("<option ",`<option noslip_iterations="${iterations}" `);
      const sim = new VectorSim(mj,{values,derived,xml,startXyMm:[0,140],headDeg:+rows[0][4],liftMm:+rows[0][5],noiseSeed:1});
      try {
        sim.controlLevel = "override";
        let active = 0, observation = 0;
        for (let tick = 0; observation < rows.length; tick++) {
          const t = +rows[0][0]+tick*sim.dt;
          // Use the closest simulated tick for observation, and causal hold for inputs.
          while (observation < rows.length && +rows[observation][0] <= t+sim.dt/2) {
            const [time,,,measured,,,phase] = rows[observation++];
            console.log([key,scene,iterations,time,phase,measured,sim.reportedNow().gyro_z_dps].join(","));
          }
          while (active+1 < rows.length && +rows[active+1][0] <= t+1e-10) active++;
          sim.setWheelMotors(+rows[active][1],+rows[active][2]);
          sim.tick(1);
        }
      } finally { sim.dispose(); }
    }
  }
}

import { readFileSync, writeFileSync } from "node:fs";
import { CONTROL_OVERRIDE, VectorSim } from "../../web/src/sim/robot.js";
import { mujoco, readJson, readText } from "../../web/test/helpers.js";

const { values, derived } = readJson("public/robot/model.json");
for (const prefix of ["", "repeat-", "third-"]) {
  const csv = readFileSync(new URL(prefix + "joints.csv", import.meta.url), "utf8");
  const rows = csv.trim().split(/\r?\n/).slice(1).map(line => line.split(","));
  const first = rows.find(row => row[1] === "state");
  const commands = rows.filter(row => ["head", "lift"].includes(row[1]));
  const sim = new VectorSim(await mujoco(), {
    values, derived, xml: readText("public/robot/vector-contact42-free-dock.xml"),
    startDocked: true, controlLevel: CONTROL_OVERRIDE, noiseSeed: 42,
    headDeg: Number(first[3]), liftMm: Number(first[4]),
  });
  try {
    let next = 0;
    const output = ["s,head_deg,lift_mm,gyro_y,on_dock"];
    for (let i = 0; ; i++) {
      const t = Number(first[0]) + i * sim.dt;
      if (t > Number(rows.at(-1)[0]) + sim.dt) break;
      if (next < commands.length && t >= Number(commands[next][0])) {
        const [, axis, raw] = commands[next++];
        const value = Number(raw);
        const action = axis === "head" ? sim.setHeadAngle(value)
          : sim.setLiftHeight(32 + 60 * value);
        if (!action || action.done && !action.ok) throw Error("Command refused at " + t);
      }
      const s = sim.state;
      output.push([t,s.head_angle_rad * 180 / Math.PI,s.lift_height_mm,s.gyro_radps[1],+sim.chargerContact].join(","));
      sim.tick(1);
    }
    if (next !== commands.length) throw Error("Incomplete replay");
    writeFileSync(new URL(prefix + "canonical-joints.csv", import.meta.url), output.join("\n") + "\n");
  } finally { sim.dispose(); }
}

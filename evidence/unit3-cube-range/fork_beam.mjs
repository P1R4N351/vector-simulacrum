// Measure geometric interception separately from the firmware lift-in-view flag.
import { VectorSim, liftAngleOfHeight } from "../../web/src/sim/robot.js";
import { mujoco, readJson, readText } from "../../web/test/helpers.js";

const mj = await mujoco();
const base = "public/robot/vector2";
const { values, derived } = readJson(base + "/model.json");
console.log("fork,lift_mm,ray_hits,nearest_mm,farthest_mm");
for (const [name, directory] of [["boxes", base], ["printed", base + "/printed"]]) {
  const sim = new VectorSim(mj, { xml: readText(directory + "/vector.xml"), values, derived });
  try {
    const wrist = mj.mj_name2id(sim.model, mj.mjtObj.mjOBJ_BODY.value, "lift_fork");
    // Reserve a ray group containing only the fork's active collision geometry.
    sim.model.geom_group.fill(1);
    for (let i = 0; i < sim.model.ngeom; i++) {
      if (sim.model.geom_bodyid[i] === wrist && sim.model.geom_contype[i]) sim.model.geom_group[i] = 0;
    }
    for (let h = Math.ceil(values.lift_height_min_mm); h <= values.lift_height_max_mm; h += 0.25) {
      const a = liftAngleOfHeight(values, h);
      sim.data.qpos[sim._q.lift] = a;
      sim.data.qpos[sim._q.wrist] = -a;
      mj.mj_forward(sim.model, sim.data);
      const hits = sim._castTof().filter(d => d >= 0);
      console.log([name, h, hits.length, hits.length ? Math.min(...hits) : "", hits.length ? Math.max(...hits) : ""].join(","));
    }
  } finally {
    sim.dispose();
  }
}

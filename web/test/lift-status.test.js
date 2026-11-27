import assert from "node:assert/strict";
import test from "node:test";
import { VectorSim } from "../src/sim/robot.js";
import { mujoco, readJson, readText } from "./helpers.js";

test("lift-in-position clears on a raise and stays set on a settled repeat", async () => {
  const { values, derived } = readJson("public/robot/vector2/model.json");
  const sim = new VectorSim(await mujoco(), { values, derived, xml: readText("public/robot/vector2/vector-free.xml") });
  try {
    assert.equal(sim.state.lift_in_pos, true);
    sim.commandLiftHeight(62, 0.7);
    sim.tick(sim._stateTicks);
    assert.equal(sim.state.lift_in_pos, false);
    sim.tick(Math.round(1.5 / sim.dt));
    assert.equal(sim.state.lift_in_pos, true);
    sim.commandLiftHeight(62);
    sim.tick(sim._stateTicks);
    assert.equal(sim.state.lift_in_pos, true);
    sim.stopAllMotors();
    sim.tick(sim._stateTicks);
    assert.equal(sim.state.lift_in_pos, false);
    sim.tick(Math.round(0.5 / sim.dt));
    assert.equal(sim.state.lift_in_pos, true);
  } finally {
    sim.dispose();
  }
});

test("both lift links start parallel before the solver settles a posed robot", async () => {
  const { values, derived } = readJson("public/robot/vector2/model.json");
  const original = readText("public/robot/vector2/vector-free.xml");
  // Exercise the optional hinge without downloading the body mesh assets.
  const xml = original.replace('<body name="lift_arm"', `
    <body name="lift_lower" pos="-0.041 0 0.033">
      <joint name="lift_lower_joint" axis="0 -1 0"/>
      <geom size=".002" mass=".001" contype="0" conaffinity="0"/>
    </body>
    <body name="lift_arm"`).replace("</equality>", `
    <joint joint1="lift_lower_joint" joint2="lift_joint" polycoef="0 1 0 0 0"/>
    </equality>`);
  const mj = await mujoco();
  const sim = new VectorSim(mj, { values: { ...values, settle_time_ms: 0 }, derived, xml });
  try {
    const angle = name => {
      const joint = mj.mj_name2id(sim.model, mj.mjtObj.mjOBJ_JOINT.value, name);
      assert.ok(joint >= 0);
      return sim.data.qpos[sim.model.jnt_qposadr[joint]];
    };
    for (const liftMm of [values.lift_height_min_mm, 62, values.lift_height_max_mm]) {
      sim.reset({ liftMm });
      assert.equal(angle("lift_lower_joint"), angle("lift_joint"));
      assert.equal(angle("wrist_joint"), -angle("lift_joint"));
    }
  } finally {
    sim.dispose();
  }
});

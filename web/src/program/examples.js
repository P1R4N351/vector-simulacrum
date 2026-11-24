// Example programs for the page's editor. test/program.test.js runs each one headless.
// The robot starts in the middle of a 600 mm square table, facing +x.

export const EXAMPLES = [
  {
    id: "square",
    title: "Drive a square",
    source: `// Four sides of 120 mm, a quarter turn at each corner.
for (let side = 1; side <= 4; side++) {
  const drove = await robot.drive_straight(120, 80);
  const turned = await robot.turn_in_place(90);
  print(\`side \${side}: \${drove.result}, \${drove.moved_mm.toFixed(1)} mm,\`,
        \`turned \${turned.turned_deg.toFixed(1)} deg\`);
}
const s = robot.state;
print(\`back at x \${s.x_mm.toFixed(1)} mm, y \${s.y_mm.toFixed(1)} mm\`);
`,
  },
  {
    id: "look",
    title: "Head, lift and wheels",
    source: `// Look up, raise the lift, drive an arc on the wheel motors, then put everything back.
await robot.set_head_angle(40);
await robot.set_lift_height(1.0);
print("head", robot.state.head_angle_deg.toFixed(1), "deg, lift",
      robot.state.lift_height_mm.toFixed(1), "mm");

await robot.set_wheel_motors(40, 80);  // right tread faster: curves to the left
await wait(2);
await robot.stop_all_motors();
await wait(0.5);
print("after the arc: heading", robot.state.pose_angle_deg.toFixed(1), "deg");

await robot.set_lift_height(0.0);
await robot.set_head_angle(0);
print("range sensor reads", robot.state.prox_distance_mm, "mm");
`,
  },
  {
    id: "edge",
    title: "Drive at the edge",
    source: `// Drive toward the table edge. With the cliff reaction on (the default),
// the robot stops at the edge, takes control away and backs off.
const run = await robot.drive_straight(400, 100);
print("drive ended:", run.result, "after", run.moved_mm.toFixed(0), "mm");

await wait(1.5);  // let the reaction play out
print("has control:", robot.state.has_control);
print("control back:", await robot.request_control());

// Back well away first: the robot turns about the middle of its treads, so its rear
// swings out behind it, and here that is toward the edge.
await robot.drive_straight(-120, 80);
await robot.turn_in_place(180);
await robot.drive_straight(150, 80);
print("away from the edge at x", robot.state.x_mm.toFixed(0), "mm, heading",
      robot.state.pose_angle_deg.toFixed(0), "deg");
`,
  },
  {
    id: "explore-dock",
    title: "Explore and self-dock",
    source: `// Autonomous mapping and self-docking using optical markers and occupancy grid.
const scan = await robot.explore(8);
print("explore:", scan.result, "cells:", scan.cells_mapped, "cube:", scan.cube_seen, "charger:", scan.charger_seen);
const docked = await robot.dock_with_charger();
print("dock:", docked.result, "attempts:", docked.attempts);
const s = robot.state;
print("on_charger:", s.on_charger, "volts:", s.battery_volts);
`,
  },
  {
    id: "nomad-nav",
    title: "NoMaD visual explore & navigate",
    source: `// Goal-Masked NoMaD diffusion: capture current view, explore, then return visually.
const goal = await robot.nomad_capture_goal("home_view");
print("captured goal node", goal.node_id);
const exp = await robot.nomad_explore(3);
print("explore nodes:", exp.nodes, "edges:", exp.edges);
const nav = await robot.nomad_navigate(4, goal.node_id);
print("navigate:", nav.result, "pred_dist:", nav.pred_dist, "nodes:", robot.state.nomad_nodes);
`,
  },
  {
    id: "rescue",
    title: "Rescue fallen Vector",
    source: `// Two-robot continuous righting macro (?env=rescue): direct and cube-assisted.
if (!robot.state.has_patient) {
  print("Single-robot table scene: open ?env=rescue to stage a sideways patient Vector.");
  await robot.set_lift_height(0.25);
  await robot.drive_straight(40, 60);
  await robot.set_lift_height(1.0);
  await robot.set_lift_height(0.0);
} else {
  for (const side of ["right", "left"]) {
    await robot.reset_rescue(side, "auto");
    const r = await robot.rescue_patient("auto");
    print(side, r.mode, r.result, "tilt:", r.final_tilt_deg.toFixed(1) + "°", "clear:", r.rescuer_clear);
  }
}
`,
  },
];

export function exampleById(id) {
  const found = EXAMPLES.find((e) => e.id === id);
  if (found === undefined) throw new Error(`no example named ${id}`);
  return found;
}

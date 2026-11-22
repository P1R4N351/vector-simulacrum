import test from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { mappedCubeXY } from "../src/sim/shelf_cube_pose.js";
const map = JSON.parse(readFileSync(new URL("../public/robot/vector2/shelf-navmap.json",import.meta.url)));
test("dock and cube resolve separately in the captured map", () => {
  const [x,y] = mappedCubeXY(map,44,-66,30);
  assert.ok(Math.abs(x-103.19)<.02 && Math.abs(y-.90)<.02);
});
test("ambiguous or absent observable regions retain the scenario placement", () => {
  const ahead = map.latest_objects_and_visibility.filter(([x,,,k])=>k===3 && x>100);
  const duplicate = ahead.map(([x,y,...rest])=>[x,y+100,...rest]);
  for (const rows of [[], [...ahead,...duplicate]]) {
    assert.equal(mappedCubeXY({...map,latest_objects_and_visibility:rows},44,-66,30),null);
  }
});

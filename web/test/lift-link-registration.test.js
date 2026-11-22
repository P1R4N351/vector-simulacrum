import assert from "node:assert/strict";
import { execFileSync } from "node:child_process";
import { fileURLToPath } from "node:url";
import test from "node:test";
import { readJson } from "./helpers.js";

test("replay lower pin follows the configured shoulder with the SDK pin spacing", () => {
  const script = fileURLToPath(new URL("../../evidence/unit3-printed-fork-contact/export_links.mjs", import.meta.url));
  const { viewer_pose: pose } = readJson("public/robot/sdk/manifest.json");
  for (const shoulder of [[-41, 45], [-39, 46]]) {
    const { links } = JSON.parse(execFileSync(process.execPath, [script, ...shoulder.map(String), "66"]));
    for (const axis of [0, 1]) {
      const spacing = 10 * (pose.lower_arm_pivot[axis] - pose.upper_arm_pivot[axis]);
      assert.ok(Math.abs(links[1].pivot[axis] - shoulder[axis] - spacing) < 1e-8);
    }
  }
});

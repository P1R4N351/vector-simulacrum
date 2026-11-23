import assert from "node:assert/strict";
import { execFileSync } from "node:child_process";
import { existsSync } from "node:fs";
import { join } from "node:path";
import { fileURLToPath } from "node:url";
import test from "node:test";
import { ROOT, readJson } from "./helpers.js";

// export_links.mjs reads the SDK's vector.obj and manifest.json, which are git-ignored and copied in by
// tools/fetch_sdk_model.py; without them the check cannot run, as in model3d.test.js.
const HAVE_SDK = existsSync(join(ROOT, "public/robot/sdk/vector.obj"))
  && existsSync(join(ROOT, "public/robot/sdk/manifest.json"));

test("replay lower pin follows the configured shoulder with the SDK pin spacing", { skip: !HAVE_SDK }, () => {
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

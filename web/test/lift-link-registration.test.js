import assert from "node:assert/strict";
import { execFileSync } from "node:child_process";
import { existsSync, mkdtempSync, rmSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { fileURLToPath } from "node:url";
import test from "node:test";
import { ROOT, readJson } from "./helpers.js";

const SCRIPT = fileURLToPath(new URL("../../evidence/unit3-printed-fork-contact/export_links.mjs", import.meta.url));
// VIEWER_POSE in tools/fetch_sdk_model.py: the SDK viewer's own constants (Apache-2.0), which that tool writes
// into manifest.json. Only vector.obj is under the Anki model licence, and the pins do not depend on it.
const VIEWER_POSE = {
  unit_m: 0.01, lift_arm_length_mm: 66.0, lift_pivot_height_mm: 45.0,
  lift_angle_in_default_pose_rad: -0.1136, fork_pivot: [3.0, 3.4], upper_arm_pivot: [-3.73, 4.47],
  lower_arm_pivot: [-3.74, 3.27], head_pivot: [-1.1, 4.75],
};
// The git-ignored copy that tools/fetch_sdk_model.py makes; checked as well when it is there.
const SDK = join(ROOT, "public/robot/sdk");
const HAVE_SDK = existsSync(join(SDK, "vector.obj")) && existsSync(join(SDK, "manifest.json"));

// A stand-in SDK folder: that manifest, and one wheel_FL_geo triangle whose axle is at x = 1.5 cm,
// so a pin still offset by the axle shows.
function withStandInSdk(fn) {
  const dir = mkdtempSync(join(tmpdir(), "vector-sdk-stand-in-"));
  try {
    writeFileSync(join(dir, "manifest.json"), JSON.stringify({ viewer_pose: VIEWER_POSE }));
    writeFileSync(join(dir, "vector.obj"), "o wheel_FL_geo\nv 0.4 2 0\nv 2.6 2 0\nv 1.5 2 2.2\nf 1 2 3\n");
    fn(dir);
  } finally {
    rmSync(dir, { recursive: true, force: true }); // this test's own temporary folder
  }
}

function assertLowerPin(sdk, pose) {
  for (const shoulder of [[-41, 45], [-39, 46]]) {
    const { links } = JSON.parse(execFileSync(process.execPath, [SCRIPT, ...shoulder.map(String), "66", sdk]));
    for (const axis of [0, 1]) {
      const spacing = 10 * (pose.lower_arm_pivot[axis] - pose.upper_arm_pivot[axis]);
      assert.ok(Math.abs(links[1].pivot[axis] - shoulder[axis] - spacing) < 1e-8, `${sdk} axis ${axis}`);
    }
  }
}

test("replay lower pin follows the configured shoulder with the SDK pin spacing", () => {
  withStandInSdk((dir) => assertLowerPin(dir, VIEWER_POSE));
  if (HAVE_SDK) assertLowerPin(SDK, readJson("public/robot/sdk/manifest.json").viewer_pose);
});

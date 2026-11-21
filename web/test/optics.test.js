// The head camera's geometry, from the real robot's calibration..
import assert from "node:assert/strict";
import { test } from "node:test";
import { edgeAnglesDeg, frustumCorners, pixelOf, project, projectionRows, rayOf } from "../src/sim/optics.js";
import { readJson } from "./helpers.js";

const camera = readJson("public/robot/model.json").camera;
const calibration = readJson("camera_calibration.json");
const near = (a, b, tol) => Math.abs(a - b) <= tol;

test("the exported camera is the real calibration, not a document's field of view", () => {
  for (const key of ["fx_px", "fy_px", "cx_px", "cy_px", "width_px", "height_px"]) assert.equal(camera[key], calibration[key], key);
  // The robot reports twice the centre-to-edge angle; the calibration must reproduce what it reported.
  const fromCentre = (size, focal) => (2 * Math.atan(size / 2 / focal) * 180) / Math.PI;
  assert.ok(near(fromCentre(640, camera.fx_px), calibration.reported_fov_x_deg, 0.01), "horizontal field of view");
  assert.ok(near(fromCentre(360, camera.fy_px), calibration.reported_fov_y_deg, 0.01), "vertical field of view");
  assert.ok(!near(camera.fov_horizontal_deg, camera.documents.manual_deg[0], 3), "not the manual's 90 degrees");
  assert.ok(!near(camera.fov_vertical_deg, camera.documents.model_fovy_deg, 3), "not the model's 50 degrees");
});

test("edge angles follow the off-centre optical centre and add up to the field of view", () => {
  const edges = edgeAnglesDeg(camera);
  assert.ok(edges.right > edges.left, "the optical centre is left of the middle, so more is seen to the right");
  assert.ok(edges.up > edges.down, "the optical centre is below the middle, so more is seen above");
  assert.ok(near(edges.left + edges.right, camera.fov_horizontal_deg, 1e-9));
  assert.ok(near(edges.up + edges.down, camera.fov_vertical_deg, 1e-9));
  for (const key of ["left", "right", "up", "down"]) assert.ok(near(edges[key], camera.edge_angles_deg[key], 1e-9), key);
});

test("a pixel's ray projects back to that pixel, and the image corners to the corners of the view", () => {
  for (const [u, v] of [[0, 0], [640, 360], [camera.cx_px, camera.cy_px], [123.4, 300.5]]) {
    const back = pixelOf(camera, rayOf(camera, u, v, 0.37));
    assert.ok(near(back[0], u, 1e-9) && near(back[1], v, 1e-9), `pixel ${u}, ${v}`);
  }
  const rows = projectionRows(camera, 0.005, 10);
  const expected = [[-1, 1], [1, 1], [1, -1], [-1, -1]];
  frustumCorners(camera, 0.2).forEach((corner, i) => {
    const ndc = project(rows, corner);
    assert.ok(near(ndc[0], expected[i][0], 1e-9) && near(ndc[1], expected[i][1], 1e-9), `corner ${i}: ${ndc}`);
    assert.ok(ndc[2] > -1 && ndc[2] < 1, "inside the depth range");
  });
  const centre = project(rows, [0, 0, -1]);
  assert.ok(near(centre[0], (2 * camera.cx_px) / 640 - 1, 1e-12) && near(centre[1], 1 - (2 * camera.cy_px) / 360, 1e-12), "the optical axis lands on the optical centre");
  assert.ok(near(project(rows, [0, 0, -0.005])[2], -1, 1e-9) && near(project(rows, [0, 0, -10])[2], 1, 1e-9), "near and far planes");
  assert.equal(pixelOf(camera, [0, 0, 0.1]), null, "a point behind the camera has no pixel");
});

test("a point to the right and above lands right of and above the optical centre", () => {
  const pixel = pixelOf(camera, [0.05, 0.02, -0.5]);
  assert.ok(pixel[0] > camera.cx_px && pixel[1] < camera.cy_px, `${pixel}`);
  assert.ok(near(pixel[0] - camera.cx_px, camera.fx_px * 0.1, 1e-9) && near(camera.cy_px - pixel[1], camera.fy_px * 0.04, 1e-9));
});

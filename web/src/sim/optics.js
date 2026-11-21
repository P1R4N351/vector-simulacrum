// Pinhole camera geometry from calibrated intrinsics..
//
// c is the camera block of public/robot/model.json: image size, focal lengths
// fx and fy in pixels, optical centre cx, cy in pixels (measured from the top
// left corner, y down). The optical centre need not be the middle of the image
// and fx need not equal fy; a real calibration has neither.
//
// Camera frame here is the one MuJoCo and three.js share: the camera looks
// along -z, +x is to the right in the image and +y is up. No lens distortion.
// No browser or three.js code; tested in test/optics.test.js.

import { assert } from "./util.js";

function check(c) {
  assert(c.width_px > 0 && c.height_px > 0 && c.fx_px > 0 && c.fy_px > 0, "bad camera intrinsics");
  assert(c.cx_px > 0 && c.cx_px < c.width_px && c.cy_px > 0 && c.cy_px < c.height_px, "optical centre outside the image");
}

// Where a point in the camera frame (metres) lands in the image, in pixels. Points behind the camera give null.
export function pixelOf(c, point) {
  check(c);
  const depth = -point[2];
  if (!(depth > 0)) return null;
  return [c.cx_px + (c.fx_px * point[0]) / depth, c.cy_px - (c.fy_px * point[1]) / depth];
}

// The direction a pixel looks along, as a point at the given depth in the camera frame.
export function rayOf(c, u, v, depth) {
  check(c);
  assert(depth > 0, "depth must be positive");
  return [((u - c.cx_px) / c.fx_px) * depth, ((c.cy_px - v) / c.fy_px) * depth, -depth];
}

// Angle from the optical axis to each edge of the image, degrees.
export function edgeAnglesDeg(c) {
  check(c);
  const deg = (x) => (Math.atan(x) * 180) / Math.PI;
  return {
    left: deg(c.cx_px / c.fx_px), right: deg((c.width_px - c.cx_px) / c.fx_px),
    up: deg(c.cy_px / c.fy_px), down: deg((c.height_px - c.cy_px) / c.fy_px),
  };
}

// The four image corners at a depth: top left, top right, bottom right, bottom left.
export function frustumCorners(c, depth) {
  return [rayOf(c, 0, 0, depth), rayOf(c, c.width_px, 0, depth), rayOf(c, c.width_px, c.height_px, depth), rayOf(c, 0, c.height_px, depth)];
}

// The 4x4 projection matrix, row by row, for a renderer whose clip space runs
// -1..1 on each axis (OpenGL's). The image's left edge maps to x = -1, its top to y = +1.
export function projectionRows(c, near, far) {
  check(c);
  assert(near > 0 && far > near, "bad clip distances");
  const w = c.width_px;
  const h = c.height_px;
  return [
    (2 * c.fx_px) / w, 0, 1 - (2 * c.cx_px) / w, 0,
    0, (2 * c.fy_px) / h, (2 * c.cy_px) / h - 1, 0,
    0, 0, -(far + near) / (far - near), (-2 * far * near) / (far - near),
    0, 0, -1, 0,
  ];
}

// Apply projectionRows to a camera-frame point: normalised device coordinates [x, y, z].
export function project(rows, point) {
  const [x, y, z] = point;
  const clip = [0, 1, 2, 3].map((r) => rows[4 * r] * x + rows[4 * r + 1] * y + rows[4 * r + 2] * z + rows[4 * r + 3]);
  assert(clip[3] !== 0, "point on the camera plane");
  return [clip[0] / clip[3], clip[1] / clip[3], clip[2] / clip[3]];
}

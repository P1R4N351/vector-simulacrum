// The robot's head camera: a three.js camera at the model's camera, looking
// where the robot's engine says it looks, with the real calibration's optics.
//
// Position: the physics model's camera (head_camera_position_mm from the
// firmware, relative to the neck), read each frame from the solver, so it
// moves with the head. Direction: the current MJCF includes the engine's
// downward mounting angle. Older level-camera exports receive it here. Optics: a pinhole from
// the calibrated focal lengths and optical centre (../sim/optics.js). No lens
// distortion, no exposure, no noise.

import * as THREE from "three";
import { frustumCorners, projectionRows } from "../sim/optics.js";

const FAR_M = 10;
const FRUSTUM_DEPTH_M = 0.12; // how far the drawn outline of the view reaches
export const HIDDEN_FROM_ROBOT = 1; // layer for things the robot's camera must not see

function frustumLines(spec) {
  const corners = frustumCorners(spec, FRUSTUM_DEPTH_M);
  const points = [];
  for (let i = 0; i < 4; i++) {
    points.push(0, 0, 0, ...corners[i]);
    points.push(...corners[i], ...corners[(i + 1) % 4]);
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(points, 3));
  const lines = new THREE.LineSegments(geometry, new THREE.LineBasicMaterial({ color: 0x6ab7ff, transparent: true, opacity: 0.8 }));
  lines.matrixAutoUpdate = false;
  lines.frustumCulled = false;
  lines.layers.set(HIDDEN_FROM_ROBOT);
  return lines;
}

// parent: the group whose coordinates are the physics model's (z up, metres).
export function makeHeadCamera(mujoco, sim, spec, parent) {
  const id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_CAMERA.value, spec.name);
  if (id < 0) throw new Error(`the model has no camera named ${spec.name}`);
  const camera = new THREE.PerspectiveCamera();
  camera.matrixAutoUpdate = false;
  camera.projectionMatrix.set(...projectionRows(spec, spec.near_mm * 0.001, FAR_M));
  camera.projectionMatrixInverse.copy(camera.projectionMatrix).invert();
  // Looking down is a turn about the camera's own x axis (to the right in the image) by a negative angle.
  const mountDown = spec.pitch_down_in_model ? 0 : spec.pitch_down_deg;
  const rgbUp = spec.rgb_pitch_up_deg ?? 0;
  const tilt = new THREE.Matrix4().makeRotationX(((rgbUp - mountDown) * Math.PI) / 180);
  const frustum = frustumLines(spec);
  parent.add(camera, frustum);
  return {
    camera,
    frustum,
    aspect: spec.width_px / spec.height_px,
    // Copy the model camera's pose from the solver, then apply the tilt.
    update(data) {
      const p = data.cam_xpos;
      const m = data.cam_xmat;
      const o = 9 * id;
      camera.matrix.set(
        m[o], m[o + 1], m[o + 2], p[3 * id],
        m[o + 3], m[o + 4], m[o + 5], p[3 * id + 1],
        m[o + 6], m[o + 7], m[o + 8], p[3 * id + 2],
        0, 0, 0, 1,
      ).multiply(tilt);
      camera.matrixWorldNeedsUpdate = true;
      frustum.matrix.copy(camera.matrix);
      frustum.matrixWorldNeedsUpdate = true;
    },
  };
}

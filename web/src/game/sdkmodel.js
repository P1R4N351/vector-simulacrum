// The Vector SDK's 3D robot model, placed and moved by the physical robot's
// geometry rather than by the SDK viewer's..
//
// The model is anki_vector/opengl/assets/vector.obj. It is not in this
// repository (Anki Image and 3D Model License); tools/fetch_sdk_model.py copies
// it into public/robot/sdk/. When it is absent this module returns null and
// the page draws the physics model's boxes instead.
//
// Where the SDK's viewer and the robot disagree, the robot wins:
//
// - Origin. The robot's origin is on the ground under the front axle. The
//   model file's front wheels sit about 3 mm ahead of its own origin, so the
//   whole model is shifted to put its front axle on the robot's.
// - Head. It turns about the robot's neck joint (neck_joint_position_mm), not
//   the viewer's pivot.
// - Lift. The arm turns about the robot's lift shoulder (lift_base_position_mm)
//   by the physics joint's angle, and the fork rides on the wrist at the end
//   of the robot's arm length, staying level.
// - Wheels. Each turns about the robot's axle for it, at the robot's radius
//   above the ground: front wheels are the motor-driven ones, rear wheels are
//   turned by the track. Cleats run round the path of the robot's track
//   (tracks.js), not the outline of the model's.
//
// Two things still come from the model, because the robot's numbers do not
// cover them: the angle the arm is drawn at in the file (the viewer's
// lift_angle_in_default_pose), and where the lower arm of the linkage pivots.
// So parts of the model can sit a millimetre or so off each other; that is
// the model not matching the robot, shown rather than hidden.
// The wheel colour, the spokes and the cleats are added here.

import * as THREE from "three";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";
import { makeCleats, makeSpokes } from "./tracks.js";

const DIR = "./robot/sdk";
const CM = 0.1; // model units (cm) per mm
const PARTS = {
  body: ["body_geo", "trackBase_L_geo", "tracks_L_geo", "trackBase_R_geo", "tracks_R_geo"],
  head: ["head_geo", "backScreen_mat", "screenEdge_geo", "overscan_1_geo", "eye_L_geo", "eye_R_geo",
    "eyeLid_R_top_geo", "eyeLid_L_top_geo", "eyeLid_L_btm_geo", "eyeLid_R_btm_geo", "front_Screen_geo"],
  upperArm: ["uprArm_L_geo", "uprArm_geo"],
  lowerArm: ["lwrArm_L_geo", "lwrArm_R_geo"],
  fork: ["fork_geo"],
};
// Front left, front right, rear left, rear right: the order of VectorSim.wheelAnglesRad().
const WHEELS = ["wheel_FL_geo", "wheel_FR_geo", "wheel_BL_geo", "wheel_BR_geo"];
const EYE_MATERIAL = "shadingMap1SG";
const GLASS_MATERIALS = new Set(["anisotropic1SG", "ScreenOp_matSG"]);
const EYE_TINT = { alarm: 0xff5a4d, dim: 0x123a55 };

async function fetchIfPresent(url, kind) {
  const response = await fetch(url, { cache: "no-cache" });
  const type = response.headers.get("content-type") ?? "";
  // A dev server answers a missing file with the page itself: check the type as well as the status.
  if (!response.ok || type.includes("text/html")) return null;
  return kind === "json" ? response.json() : response.text();
}

// newmtl name -> diffuse colour, from the model's material file.
function diffuseColours(mtlText) {
  const colours = new Map();
  let current = null;
  for (const line of mtlText.split("\n")) {
    const words = line.trim().split(/\s+/);
    if (words[0] === "newmtl") current = words[1];
    else if (words[0] === "Kd" && current !== null) colours.set(current, words.slice(1, 4).map(Number));
  }
  return colours;
}

function makeMaterials(colours) {
  const made = new Map();
  for (const [name, rgb] of colours) {
    const colour = new THREE.Color(rgb[0], rgb[1], rgb[2]);
    if (name === EYE_MATERIAL) made.set(name, new THREE.MeshBasicMaterial({ color: colour, toneMapped: false }));
    else if (GLASS_MATERIALS.has(name)) {
      made.set(name, new THREE.MeshStandardMaterial({ color: colour, roughness: 0.1, transparent: true, opacity: 0.25, depthWrite: false }));
    } else made.set(name, new THREE.MeshStandardMaterial({ color: colour, roughness: 0.42, metalness: 0.25 }));
  }
  return made;
}

function boundsOf(mesh) {
  mesh.geometry.computeBoundingBox();
  return mesh.geometry.boundingBox;
}

// A group that turns about `pivot` (x, z, in the file's coordinates); `meshes` keep their coordinates.
function pivotGroup(pivot, meshes) {
  const group = new THREE.Group();
  group.position.set(pivot[0], 0, pivot[1]);
  for (const mesh of meshes) {
    mesh.position.set(-pivot[0], 0, -pivot[1]);
    group.add(mesh);
  }
  return group;
}

// Each wheel moved onto the robot's axle for it, with spokes on its outer face.
// toFile: robot millimetres (x, z) -> the file's coordinates.
function mountWheels(holder, wheelMeshes, v, toFile) {
  const material = new THREE.MeshStandardMaterial({ color: 0x8a9099, roughness: 0.4, metalness: 0.45 });
  const axles = [
    [0, v.wheel_front_radius_mm], [0, v.wheel_front_radius_mm],
    [-v.wheel_base_mm, v.wheel_rear_radius_mm], [-v.wheel_base_mm, v.wheel_rear_radius_mm],
  ];
  return wheelMeshes.map((mesh, i) => {
    const box = boundsOf(mesh);
    const drawnAxle = [0.5 * (box.min.x + box.max.x), 0.5 * (box.min.z + box.max.z)];
    const radius = 0.5 * (box.max.x - box.min.x);
    const outer = Math.abs(box.max.y) > Math.abs(box.min.y) ? box.max.y : box.min.y;
    mesh.material = material;
    const group = pivotGroup(drawnAxle, [mesh]);
    const real = toFile(axles[i]);
    group.position.set(real[0], 0, real[1]);
    group.add(makeSpokes(0.9 * radius, outer + Math.sign(outer) * 0.012, "y", 0.02));
    holder.add(group);
    return { group, y: 0.5 * (box.min.y + box.max.y), width: box.max.y - box.min.y };
  });
}

// Cleats round the robot's track: its wheel radii at its axles.
function mountCleats(holder, wheels, v, toFile) {
  const front = toFile([0, v.wheel_front_radius_mm]);
  const rear = toFile([-v.wheel_base_mm, v.wheel_rear_radius_mm]);
  const proud = 0.05;
  const cleats = makeCleats({
    front: { x: front[0], z: front[1], r: v.wheel_front_radius_mm * CM - proud },
    rear: { x: rear[0], z: rear[1], r: v.wheel_rear_radius_mm * CM - proud },
    sideY: [wheels[0].y, wheels[1].y], width: 0.92 * wheels[0].width, pitch: 0.42, along: 0.14, proud,
  });
  holder.add(cleats.mesh);
  return cleats;
}

function assemble(meshes, pose, v) {
  const take = (names) => names.map((name) => {
    const mesh = meshes.get(name);
    if (!mesh) throw new Error(`the SDK model has no part named ${name}`);
    return mesh;
  });
  // Shift the file so its front axle is on the robot's origin; `holder` then holds the file's coordinates.
  const frontBox = boundsOf(take([WHEELS[0]])[0]);
  const frontAxleX = 0.5 * (frontBox.min.x + frontBox.max.x);
  const toFile = ([xMm, zMm]) => [xMm * CM + frontAxleX, zMm * CM];
  const root = new THREE.Group();
  root.name = "sdk_robot";
  root.scale.setScalar(pose.unit_m);
  const holder = new THREE.Group();
  holder.position.x = -frontAxleX;
  root.add(holder);
  for (const mesh of take(PARTS.body)) holder.add(mesh);

  const neck = toFile([v.neck_joint_position_mm[0], v.neck_joint_position_mm[2]]);
  const shoulder = toFile([v.lift_base_position_mm[0], v.lift_base_position_mm[2]]);
  const drawnAngle = pose.lift_angle_in_default_pose_rad;
  const arm = v.lift_arm_length_mm * CM;
  const wrist = [shoulder[0] + arm * Math.cos(drawnAngle), shoulder[1] + arm * Math.sin(drawnAngle)];
  const head = pivotGroup(neck, take(PARTS.head));
  const upper = pivotGroup(shoulder, take(PARTS.upperArm));
  const lower = pivotGroup(pose.lower_arm_pivot, take(PARTS.lowerArm));
  // The fork swings with the arm about the shoulder, then turns back about the wrist so it stays level.
  const forkSwing = new THREE.Group();
  forkSwing.position.set(shoulder[0], 0, shoulder[1]);
  const forkLevel = pivotGroup(wrist, take(PARTS.fork));
  forkLevel.position.set(wrist[0] - shoulder[0], 0, wrist[1] - shoulder[1]);
  forkSwing.add(forkLevel);
  holder.add(head, upper, lower, forkSwing);
  const wheels = mountWheels(holder, take(WHEELS), v, toFile);
  const cleats = mountCleats(holder, wheels, v, toFile);
  return { root, head, upper, lower, forkSwing, forkLevel, wheels, cleats, drawnAngle, frontAxleShiftMm: frontAxleX / CM };
}

// values: the parameter table's values (model.json). Returns null when the model
// has not been fetched; throws when it is present but unusable.
export async function loadSdkModel(values) {
  const manifest = await fetchIfPresent(`${DIR}/manifest.json`, "json");
  if (manifest === null) return null;
  const [objText, mtlText] = await Promise.all([fetchIfPresent(`${DIR}/vector.obj`, "text"), fetchIfPresent(`${DIR}/vector.mtl`, "text")]);
  if (objText === null || mtlText === null) throw new Error("manifest.json is present but the model files are not");
  const materials = makeMaterials(diffuseColours(mtlText));
  const pick = (m) => {
    const found = materials.get(m.name);
    if (!found) throw new Error(`the SDK model uses a material its file does not define: ${m.name}`);
    return found;
  };
  const meshes = new Map();
  new OBJLoader().parse(objText).traverse((node) => {
    if (!node.isMesh) return;
    node.material = Array.isArray(node.material) ? node.material.map(pick) : pick(node.material);
    node.castShadow = true;
    meshes.set(node.name, node);
  });
  const rig = assemble(meshes, manifest.viewer_pose, values);
  const eye = materials.get(EYE_MATERIAL);
  const eyeBase = eye.color.getHex();
  const frontR = values.wheel_front_radius_mm * CM;
  return {
    root: rig.root,
    head: rig.head,
    source: manifest.source,
    frontAxleShiftMm: rig.frontAxleShiftMm,
    // headRad: head angle, up positive. liftRad: the lift's shoulder angle, up positive.
    // wheelAngles: the four wheel joint angles, front left, front right, rear left, rear right.
    setPose(headRad, liftRad, mood, wheelAngles) {
      rig.head.rotation.y = -headRad;
      const lift = -(liftRad - rig.drawnAngle); // the file draws the arm at drawnAngle
      rig.upper.rotation.y = lift;
      rig.lower.rotation.y = lift;
      rig.forkSwing.rotation.y = lift;
      rig.forkLevel.rotation.y = -lift;
      for (let i = 0; i < 4; i++) rig.wheels[i].group.rotation.y = wheelAngles[i];
      rig.cleats.update(wheelAngles[0] * frontR, wheelAngles[1] * frontR);
      eye.color.setHex(EYE_TINT[mood] ?? eyeBase);
    },
  };
}

// Blended 3D robot assembly combining Anki's detailed animation rig meshes
// (public/robot/sdk/rig/) with live animated 42-cleat treads, eye/mood
// blinking, WireOS LCD textures, faceplate-off internal carrier, and optional
// Vector 2 replacement STL forks and wheels.
import * as THREE from "three";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";
import { repairDockNormals } from "./dock.js";
import { addFaceCarrier } from "./face.js";
import { loadPrintParts, outerWheel, printFork } from "./printparts.js";
import { beltGeometry, makeBeltPath, makeCleats } from "./tracks.js";

const RIG_DIR = "./robot/sdk/rig";
const MM_TO_CM = 0.1;
const CM_TO_MM = 10.0;
const EYE_TINT = { alarm: 0xff5a4d, dim: 0x123a55 };
const EYE_BASE = 0x40fc65;
const RIG_PARTS = [
  "chassis",
  "head_shell",
  "head_visor",
  "eye_l",
  "eye_r",
  "lift_upper",
  "lift_lower",
  "fork",
  "wheel_fl",
  "wheel_fr",
  "wheel_bl",
  "wheel_br",
];

async function fetchIfPresent(url, kind) {
  const response = await fetch(url, { cache: "no-cache" });
  const type = response.headers.get("content-type") ?? "";
  if (!response.ok || type.includes("text/html")) return null;
  return kind === "json" ? response.json() : response.text();
}

export function makeRigMaterials(shellVersion = 1) {
  const trimHex = shellVersion === 2 ? 0x626469 : 0xc4af7b;
  const eyeMat = new THREE.MeshBasicMaterial({ color: EYE_BASE, toneMapped: false });
  const eyeGlowMat = new THREE.MeshBasicMaterial({
    color: EYE_BASE,
    transparent: true,
    opacity: 0.32,
    toneMapped: false,
    depthWrite: false,
  });
  const byTag = new Map([
    ["gloss_black", new THREE.MeshStandardMaterial({ color: 0x141518, roughness: 0.28, metalness: 0.08 })],
    ["dark_grey", new THREE.MeshStandardMaterial({ color: 0x3b3c40, roughness: 0.42, metalness: 0.06 })],
    ["gunmetal", new THREE.MeshStandardMaterial({ color: 0x626469, roughness: 0.34, metalness: 0.25 })],
    ["ear_ring", new THREE.MeshStandardMaterial({ color: trimHex, roughness: 0.30, metalness: 0.43 })],
    ["backpack_trim", new THREE.MeshStandardMaterial({ color: trimHex, roughness: 0.32, metalness: 0.40 })],
    ["contact_pad", new THREE.MeshStandardMaterial({ color: 0xc7a054, roughness: 0.31, metalness: 0.70 })],
    ["light_diffuser", new THREE.MeshStandardMaterial({
      color: 0x7f8c96,
      roughness: 0.35,
      emissive: 0x162428,
      emissiveIntensity: 0.35,
    })],
    ["screen", new THREE.MeshStandardMaterial({ color: 0x090a0d, roughness: 0.22, metalness: 0.05 })],
    ["visor", new THREE.MeshPhysicalMaterial({
      color: 0xe6e3ef,
      transmission: 0.98,
      thickness: 0.9,
      ior: 1.55,
      attenuationColor: 0x585165,
      attenuationDistance: 0.035,
      roughness: 0.095,
      clearcoat: 1,
      clearcoatRoughness: 0.08,
      envMapIntensity: 1.3,
    })],
    ["eye", eyeMat],
    ["eye_glow", eyeGlowMat],
  ]);
  return { byTag, eyeMat, eyeGlowMat };
}

export function parseRigPartGroup(name, objText, materialsByTag) {
  const group = new THREE.Group();
  group.name = name;
  const parts = [];
  new OBJLoader().parse(objText).traverse((node) => {
    if (!node.isMesh || !node.geometry?.attributes?.position) return;
    const tag = Array.isArray(node.material) ? node.material[0]?.name : node.material?.name;
    const mat = materialsByTag.get(tag) ?? materialsByTag.get("gloss_black");
    node.geometry = repairDockNormals(node.geometry);
    node.material = mat;
    const isEyeOrVisor = tag === "visor" || tag === "eye" || tag === "eye_glow";
    node.castShadow = !isEyeOrVisor;
    node.receiveShadow = tag !== "eye" && tag !== "eye_glow";
    if (tag === "eye") node.renderOrder = 1;
    if (tag === "eye_glow") node.renderOrder = 1;
    if (tag === "visor") node.renderOrder = 2;
    parts.push(node);
  });
  for (const part of parts) group.add(part);
  return group;
}

function makeWireOsLcd(values, neckMm) {
  const widthMm = 23.4;
  const pxW = values.display_resolution_px?.[0] ?? 184;
  const pxH = values.display_resolution_px?.[1] ?? 96;
  const heightMm = widthMm * pxH / pxW;
  const display = new THREE.Mesh(
    new THREE.PlaneGeometry(widthMm, heightMm),
    new THREE.MeshBasicMaterial({ toneMapped: false }),
  );
  display.name = "wireos_lcd";
  display.quaternion.setFromAxisAngle(new THREE.Vector3(1, 1, 1).normalize(), 2 * Math.PI / 3);
  display.position.set(1.02 - neckMm[0], 0.36, 52.2 - neckMm[2]);
  display.visible = false;
  display.renderOrder = 1;
  return display;
}

function makeRearBadge() {
  const closedEllipse = (cx, cy, rx, ry, segments = 24) => {
    const s = new THREE.Path();
    for (let i = 0; i < segments; i++) {
      const a = -(2 * Math.PI * i) / segments;
      const x = cx + rx * Math.cos(a), y = cy + ry * Math.sin(a);
      if (i === 0) s.moveTo(x, y);
      else s.lineTo(x, y);
    }
    return s;
  };
  const badge = new THREE.Shape();
  badge.moveTo(-3.0, 2.0);
  badge.lineTo(1.5, 2.0);
  badge.quadraticCurveTo(1.6, 0.6, 2.8, -0.5);
  badge.quadraticCurveTo(3.6, -2.3, 0.4, -1.9);
  badge.quadraticCurveTo(-1.1, -2.3, -2.9, -1.7);
  badge.closePath();
  badge.holes.push(closedEllipse(-0.5, 0, 1.6, 1.2));
  const mesh = new THREE.Mesh(
    new THREE.ExtrudeGeometry(badge, { depth: 0.09, bevelEnabled: false }),
    new THREE.MeshStandardMaterial({ color: 0xb6b5b1, metalness: 0.7, roughness: 0.32 }),
  );
  mesh.name = "Rear badge";
  const right = new THREE.Vector3(0, -1, 0);
  const up = new THREE.Vector3(0.34, 0, 0.94).normalize();
  const normal = right.clone().cross(up);
  mesh.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(right, up, normal));
  mesh.position.set(-64.2, 0, 34.8);
  return mesh;
}

function mountRigTreads(holder, values, frames) {
  const left = frames.tread_contact_left_y_mm ?? [18.2, 29.7];
  const right = frames.tread_contact_right_y_mm ?? [-29.7, -18.2];
  const halfSpacing = 0.25 * ((left[0] + left[1]) - (right[0] + right[1]));
  const beltWidth = 0.5 * ((left[1] - left[0]) + (right[1] - right[0]));
  const sideY = [halfSpacing, -halfSpacing];
  const proud = 0.55;
  const frontSpec = { x: 0, z: values.wheel_front_radius_mm, r: values.wheel_front_radius_mm - proud };
  const rearSpec = { x: -values.wheel_base_mm, z: values.wheel_rear_radius_mm, r: values.wheel_rear_radius_mm - proud };
  const beltLen = makeBeltPath(frontSpec, rearSpec).length;
  const cleats = makeCleats({
    front: frontSpec,
    rear: rearSpec,
    sideY,
    width: beltWidth,
    pitch: beltLen / 42,
    along: 1.9,
    proud,
  });
  const band = beltGeometry(cleats.path, beltWidth, 1.05);
  const rubber = new THREE.MeshStandardMaterial({ color: 0x222224, roughness: 0.86 });
  for (const y of sideY) {
    const belt = new THREE.Mesh(band, rubber);
    belt.position.y = y;
    belt.castShadow = true;
    holder.add(belt);
  }
  holder.add(cleats.mesh);
  return cleats;
}

function mountRigWheels(holder, partGroups, frames, parts, printedWheels) {
  const specs = [
    ["wheel_fl", frames.wheel_front_mm, 1, 0],
    ["wheel_fr", frames.wheel_front_mm, -1, 0],
    ["wheel_bl", frames.wheel_rear_mm, 1, 1],
    ["wheel_br", frames.wheel_rear_mm, -1, 1],
  ];
  const mat = new THREE.MeshStandardMaterial({ color: 0x76787b, roughness: 0.4, metalness: 0.45 });
  return specs.map(([key, origin, side, partIdx]) => {
    const wGroup = partGroups.get(key);
    wGroup.position.set(origin[0], origin[1], origin[2]);
    const printGeo = printedWheels ? parts[partIdx] : null;
    if (printGeo) {
      wGroup.scale.y = 0.35;
      const capHolder = new THREE.Group();
      capHolder.scale.setScalar(CM_TO_MM);
      capHolder.add(outerWheel(printGeo, side * 2.95, mat));
      wGroup.add(capHolder);
    }
    holder.add(wGroup);
    return wGroup;
  });
}

function mountInternalCarrier(head, values, neckMm) {
  const scratch = new THREE.Mesh(new THREE.BufferGeometry());
  addFaceCarrier(scratch, values, 0);
  const carrier = scratch.children[0];
  scratch.geometry.dispose();
  if (!carrier) return;
  const wrap = new THREE.Group();
  wrap.name = "face_carrier_mount";
  wrap.position.set(-neckMm[0], -neckMm[1], -neckMm[2]);
  wrap.scale.setScalar(CM_TO_MM);
  wrap.add(carrier);
  head.add(wrap);
}

export function assembleRigBlend(
  manifest,
  partTexts,
  values,
  parts = [null, null, null],
  { faceplate = true, shellVersion = 1, printedFork = false, printedWheels = false } = {},
) {
  const frames = manifest.frames;
  const mats = makeRigMaterials(shellVersion);
  const partGroups = new Map();
  for (const name of RIG_PARTS) {
    const txt = partTexts.get(`${name}.obj`);
    if (!txt) throw new Error(`missing rig mesh ${name}.obj`);
    partGroups.set(name, parseRigPartGroup(name, txt, mats.byTag));
  }

  const root = new THREE.Group();
  root.name = "blended_rig_robot";
  root.scale.setScalar(0.001);
  const holder = new THREE.Group();
  holder.name = "rig_mm";
  root.add(holder);

  const chassis = partGroups.get("chassis");
  chassis.add(makeRearBadge());
  holder.add(chassis);

  const neck = frames.neck_mm;
  const head = new THREE.Group();
  head.name = "rig_head";
  head.position.set(neck[0], neck[1], neck[2]);
  const shell = partGroups.get("head_shell");
  const visor = partGroups.get("head_visor");
  visor.visible = faceplate;
  const eyeL = partGroups.get("eye_l");
  eyeL.name = "eye_L_geo";
  eyeL.position.set(...frames.eye_l_offset_mm);
  const eyeR = partGroups.get("eye_r");
  eyeR.name = "eye_R_geo";
  eyeR.position.set(...frames.eye_r_offset_mm);
  head.add(shell, visor, eyeL, eyeR, makeWireOsLcd(values, neck));
  if (!faceplate) mountInternalCarrier(head, values, neck);
  holder.add(head);

  const upper = partGroups.get("lift_upper");
  upper.position.set(...frames.upper_origin_mm);
  const lower = partGroups.get("lift_lower");
  lower.position.set(...frames.lower_origin_mm);
  const forkSwing = new THREE.Group();
  forkSwing.position.set(...frames.upper_origin_mm);
  const forkLevel = new THREE.Group();
  forkLevel.position.set(values.lift_arm_length_mm, 0, 0);
  if (printedFork && parts[2]) {
    const pfWrap = new THREE.Group();
    pfWrap.scale.setScalar(CM_TO_MM);
    pfWrap.add(printFork(parts[2], values, new THREE.MeshStandardMaterial({ color: 0x242426, roughness: 0.52 })));
    forkLevel.add(pfWrap);
  } else {
    forkLevel.add(partGroups.get("fork"));
  }
  forkSwing.add(forkLevel);
  holder.add(upper, lower, forkSwing);

  const wheels = mountRigWheels(holder, partGroups, frames, parts, printedWheels);
  const cleats = mountRigTreads(holder, values, frames);
  const frontR = values.wheel_front_radius_mm;
  const eyes = [eyeL, eyeR];
  return {
    root,
    head,
    upper,
    lower,
    forkSwing,
    forkLevel,
    wheels,
    cleats,
    eyes,
    source: manifest.source,
    setPose(headRad, liftRad, mood, wheelAngles, open = 1) {
      head.rotation.y = -headRad;
      const lift = -liftRad;
      upper.rotation.y = lift;
      lower.rotation.y = lift;
      forkSwing.rotation.y = lift;
      forkLevel.rotation.y = -lift;
      for (let i = 0; i < 4; i++) wheels[i].rotation.y = wheelAngles[i];
      cleats.update(wheelAngles[0] * frontR, wheelAngles[1] * frontR);
      const scaleZ = Math.max(0.12, (mood === "squint" ? 0.42 : 1) * open);
      for (const eye of eyes) eye.scale.z = scaleZ;
      const tint = EYE_TINT[mood] ?? EYE_BASE;
      mats.eyeMat.color.setHex(tint);
      mats.eyeGlowMat.color.setHex(tint);
    },
  };
}

export async function loadRigBlend(values, options = {}) {
  const manifest = await fetchIfPresent(`${RIG_DIR}/rig_manifest.json`, "json");
  if (manifest === null) return null;
  const texts = await Promise.all(
    RIG_PARTS.map(async (name) => [fName(name), await fetchIfPresent(`${RIG_DIR}/${name}.obj`, "text")]),
  );
  const partTexts = new Map();
  for (const [fileName, body] of texts) {
    if (body === null) throw new Error(`rig_manifest.json is present but ${fileName} is missing`);
    partTexts.set(fileName, body);
  }
  const wantParts = options.printedFork || options.printedWheels;
  const parts = wantParts ? await loadPrintParts() : [null, null, null];
  const rig = assembleRigBlend(manifest, partTexts, values, parts, options);
  return {
    root: rig.root,
    head: rig.head,
    source: rig.source,
    frontAxleShiftMm: 0,
    setPose: rig.setPose,
  };
}

function fName(part) {
  return `${part}.obj`;
}

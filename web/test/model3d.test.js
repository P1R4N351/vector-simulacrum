import assert from "node:assert/strict";
import { existsSync } from "node:fs";
import { join } from "node:path";
import { test } from "node:test";
import * as THREE from "three";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";
import { CUBE_FACE_ATLAS, repairCubeUVs, repairMeshNormals } from "../src/game/cube.js";
import { printFork } from "../src/game/printparts.js";
import { assembleRigBlend, makeRigMaterials } from "../src/game/rigmodel.js";
import { assemble } from "../src/game/sdkmodel.js";
import { addShellTrim } from "../src/game/shell.js";
import { addCrownAndBackpack } from "../src/game/topology.js";
import { beltGeometry, makeBeltPath } from "../src/game/tracks.js";
import { addUnderbody } from "../src/game/underbody.js";
import { ROOT, readJson, readText } from "./helpers.js";

function withMockDocument(fn) {
  const prev = globalThis.document;
  const noop = () => {};
  globalThis.document = {
    createElement: () => ({
      width: 0,
      height: 0,
      getContext: () => ({
        fillStyle: "",
        strokeStyle: "",
        lineWidth: 1,
        shadowColor: "",
        shadowBlur: 0,
        font: "",
        fillRect: noop,
        translate: noop,
        scale: noop,
        beginPath: noop,
        moveTo: noop,
        lineTo: noop,
        quadraticCurveTo: noop,
        roundRect: noop,
        stroke: noop,
        fill: noop,
        closePath: noop,
        fillText: noop,
        createRadialGradient: () => ({ addColorStop: noop }),
      }),
    }),
  };
  try {
    return fn();
  } finally {
    if (prev === undefined) delete globalThis.document;
    else globalThis.document = prev;
  }
}

function inspectMesh(mesh) {
  const g = mesh.geometry;
  const pos = g.attributes.position;
  const nor = g.attributes.normal;
  const idx = g.index;
  assert.ok(pos && nor, `${mesh.name || "mesh"} lacks position or normal attributes`);
  const triCount = idx ? idx.count / 3 : pos.count / 3;
  const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3();
  const e1 = new THREE.Vector3(), e2 = new THREE.Vector3(), fn = new THREE.Vector3(), vn = new THREE.Vector3();
  let inverted = 0, degenerate = 0, zeroNormals = 0, nonUnit = 0, minDot = 1;
  for (let t = 0; t < triCount; t++) {
    const i0 = idx ? idx.getX(t * 3) : t * 3;
    const i1 = idx ? idx.getX(t * 3 + 1) : t * 3 + 1;
    const i2 = idx ? idx.getX(t * 3 + 2) : t * 3 + 2;
    a.fromBufferAttribute(pos, i0);
    b.fromBufferAttribute(pos, i1);
    c.fromBufferAttribute(pos, i2);
    fn.crossVectors(e1.subVectors(b, a), e2.subVectors(c, a));
    const area = fn.length();
    if (area <= 1e-10) {
      degenerate += 1;
      continue;
    }
    fn.divideScalar(area);
    for (const k of [i0, i1, i2]) {
      vn.fromBufferAttribute(nor, k);
      const len = vn.length();
      if (len < 1e-4) {
        zeroNormals += 1;
      } else {
        if (Math.abs(len - 1) > 1e-3) nonUnit += 1;
        const d = vn.dot(fn) / len;
        if (d < minDot) minDot = d;
        if (d < -1e-3) inverted += 1;
      }
    }
  }
  return { verts: pos.count, tris: triCount, inverted, degenerate, zeroNormals, nonUnit, minDot };
}

function assertCleanTree(root, label) {
  root.traverse((obj) => {
    if (!obj.isMesh || !obj.geometry || !obj.geometry.attributes.position) return;
    const stats = inspectMesh(obj);
    assert.equal(stats.inverted, 0, `${label}/${obj.name}: inverted=${stats.inverted}`);
    assert.equal(stats.degenerate, 0, `${label}/${obj.name}: degenerate=${stats.degenerate}`);
    assert.equal(stats.zeroNormals, 0, `${label}/${obj.name}: zeroNormals=${stats.zeroNormals}`);
    assert.equal(stats.nonUnit, 0, `${label}/${obj.name}: nonUnit=${stats.nonUnit}`);
  });
}

function makeSyntheticSdkMeshes() {
  const meshes = new Map();
  const addBox = (name, size, center) => {
    const geo = new THREE.BoxGeometry(...size).translate(...center);
    meshes.set(name, new THREE.Mesh(geo, new THREE.MeshStandardMaterial()));
  };
  addBox("body_geo", [6.0, 3.0, 2.0], [-2.5, 0, 1.5]);
  addBox("head_geo", [3.0, 3.2, 2.4], [0, 0, 4.5]);
  addBox("eye_L_geo", [0.1, 1.0, 1.0], [1.2, 0.72, 4.7]);
  addBox("eye_R_geo", [0.1, 1.0, 1.0], [1.2, -0.72, 4.7]);
  addBox("front_Screen_geo", [0.1, 3.0, 2.0], [1.5, 0, 4.5]);
  addBox("wheel_FL_geo", [2.6, 1.167, 2.6], [0, 2.2015, 1.385]);
  addBox("wheel_FR_geo", [2.6, 1.167, 2.6], [0, -2.2015, 1.385]);
  addBox("wheel_BL_geo", [1.9, 1.167, 1.9], [-4.7285, 2.2015, 1.01]);
  addBox("wheel_BR_geo", [1.9, 1.167, 1.9], [-4.7285, -2.2015, 1.01]);
  // Fork geometry spans z=[0.27, 3.47] cm around fork_pivot=[3.0, 3.4] so low lift clears z=0 by +0.7 mm.
  addBox("fork_geo", [0.8, 3.8, 3.2], [3.0, 0, 1.87]);
  for (const [name, side] of [["trackBase_L_geo", 1], ["trackBase_R_geo", -1]]) {
    const geo = new THREE.CylinderGeometry(1.5, 1.5, 3.4, 24).rotateZ(Math.PI / 2).translate(-2.18, side * 1.3, 1.34);
    meshes.set(name, new THREE.Mesh(geo, new THREE.MeshStandardMaterial()));
  }
  return meshes;
}

test("procedural shell and underbody have zero inverted/degenerate triangles and bounded pan size", () => {
  const v = readJson("public/robot/model.json").values;
  withMockDocument(() => {
    for (const sv of [1, 2]) {
      const group = new THREE.Group();
      addCrownAndBackpack(group, group, sv);
      addUnderbody(group, v, 0.295, sv);
      assertCleanTree(group, `shellVersion-${sv}`);
      const pan = group.getObjectByName("Underside pan");
      assert.ok(pan, "missing Underside pan mesh");
      assert.ok(pan.geometry.attributes.position.count < 30000,
        `Underside pan bloated to ${pan.geometry.attributes.position.count} verts`);
    }
  });
});

test("beltGeometry emits split faces with exact outward analytic normals", () => {
  const path = makeBeltPath({ x: 0, z: 1.385, r: 1.33 }, { x: -4.7285, z: 1.01, r: 0.955 });
  const geom = beltGeometry(path, 1.167, 0.105);
  const stats = inspectMesh(new THREE.Mesh(geom));
  assert.equal(stats.verts, 1280);
  assert.equal(stats.tris, 1280);
  assert.equal(stats.nonUnit, 0);
  assert.ok(stats.minDot > 0.995, `expected exact analytic face normals, got minDot=${stats.minDot}`);
});

test("addShellTrim normalizes decal normals and centers eye geometry for blinking", () => {
  const v = readJson("public/robot/model.json").values;
  const meshes = makeSyntheticSdkMeshes();
  withMockDocument(() => {
    addShellTrim(meshes, v, 0.0, 2);
  });
  for (const name of ["trackBase_L_geo", "trackBase_R_geo"]) {
    const carrier = meshes.get(name);
    assert.ok(carrier.children.length > 0, `${name} missing decal child`);
    for (const child of carrier.children) {
      const stats = inspectMesh(child);
      assert.equal(stats.nonUnit, 0, `${name} decal has ${stats.nonUnit} non-unit normals`);
    }
  }
  const eyeL = meshes.get("eye_L_geo");
  eyeL.geometry.computeBoundingBox();
  const center = eyeL.geometry.boundingBox.getCenter(new THREE.Vector3());
  assert.ok(center.length() < 1e-6, `eye geometry should be centered at origin, got ${center.toArray()}`);
  assert.ok(Math.abs(eyeL.position.x - 0.397) < 1e-6, "eye position x mismatch");
});

test("assemble pivots stock fork at fork_pivot, offsets lower_arm_pivot, aligns 42 cleats, and squints symmetrically", () => {
  const v = readJson("public/robot/model.json").values;
  const pose = {
    unit_m: 0.01,
    lift_angle_in_default_pose_rad: -0.1136,
    upper_arm_pivot: [-3.73, 4.47],
    lower_arm_pivot: [-3.74, 3.27],
    fork_pivot: [3.0, 3.4],
  };
  const meshes = makeSyntheticSdkMeshes();
  const rig = withMockDocument(() => assemble(meshes, pose, v, [null, null, null], 2));

  assert.equal(rig.cleats.mesh.count, 84, "expected 42 cleats per track");
  const expectedLowerX = v.lift_base_position_mm[0] * 0.1 + pose.lower_arm_pivot[0] - pose.upper_arm_pivot[0];
  const expectedLowerZ = v.lift_base_position_mm[2] * 0.1 + pose.lower_arm_pivot[1] - pose.upper_arm_pivot[1];
  assert.ok(Math.abs(rig.lower.position.x - expectedLowerX) < 1e-6);
  assert.ok(Math.abs(rig.lower.position.z - expectedLowerZ) < 1e-6);

  // Set lift to lowest firmware angle (32 mm wrist height): stock fork should not clip below floor z=0.
  const lowLift = Math.asin((v.lift_height_min_mm - v.lift_base_position_mm[2]) / v.lift_arm_length_mm);
  rig.setPose(0, lowLift, "calm", [0, 0, 0, 0], 1.0);
  rig.root.updateMatrixWorld(true);
  const dummy = new THREE.Matrix4();
  rig.cleats.mesh.getMatrixAt(0, dummy);
  const cleatPos = new THREE.Vector3().setFromMatrixPosition(dummy);
  assert.ok(Math.abs(Math.abs(cleatPos.y) - 2.2015) < 1e-3,
    `cleat Y=${cleatPos.y} did not align with wheel carrier center 2.2015`);
  const forkBox = new THREE.Box3().setFromObject(meshes.get("fork_geo"));
  assert.ok(forkBox.min.z > 0.0 && forkBox.min.z < 0.002,
    `expected low-lift fork clearance in (0, 2) mm, got ${(forkBox.min.z * 1000).toFixed(3)} mm`);

  const eyeL = meshes.get("eye_L_geo");
  const zBefore = new THREE.Box3().setFromObject(eyeL).getCenter(new THREE.Vector3()).z;
  rig.setPose(0, lowLift, "squint", [0, 0, 0, 0], 0.5);
  rig.root.updateMatrixWorld(true);
  const zAfter = new THREE.Box3().setFromObject(eyeL).getCenter(new THREE.Vector3()).z;
  assert.ok(Math.abs(eyeL.scale.z - 0.21) < 1e-6, `expected squint*open scale.z=0.21, got ${eyeL.scale.z}`);
  assert.ok(Math.abs(zAfter - zBefore) < 1e-5, `eye Z center drifted during blink: ${zBefore} -> ${zAfter}`);
});

test("printFork centers symmetric print geometry on robot Y=0 and repairMeshNormals fixes inverted cube normals", () => {
  const v = readJson("public/robot/model.json").values;
  const forkGeo = new THREE.BoxGeometry(2.0, 1.0, 1.0).translate(-4.8005, 0, 0);
  const forkMesh = printFork(forkGeo, v, new THREE.MeshStandardMaterial());
  forkMesh.geometry.computeBoundingBox();
  forkMesh.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(forkMesh);
  const center = box.getCenter(new THREE.Vector3());
  assert.ok(Math.abs(center.y) < 1e-6, `printFork Y center off-axis by ${center.y}`);

  const cubeGeo = new THREE.BoxGeometry(1, 1, 1);
  const nor = cubeGeo.attributes.normal;
  for (let i = 0; i < nor.count; i++) nor.setXYZ(i, -nor.getX(i), -nor.getY(i), -nor.getZ(i));
  assert.ok(inspectMesh(new THREE.Mesh(cubeGeo)).inverted > 0);
  repairMeshNormals(cubeGeo);
  assert.equal(inspectMesh(new THREE.Mesh(cubeGeo)).inverted, 0);
});

function verifyCubeMarkerLayout(geo, label) {
  const pos = geo.attributes.position, uv = geo.attributes.uv, idx = geo.index;
  const count = idx ? idx.count : pos.count;
  const facesHit = new Map();
  const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3(), n = new THREE.Vector3();
  for (let t = 0; t + 2 < count; t += 3) {
    const k = [idx ? idx.getX(t) : t, idx ? idx.getX(t + 1) : t + 1, idx ? idx.getX(t + 2) : t + 2];
    if (!k.every((i) => uv.getX(i) < 0.51 && uv.getY(i) < 0.73)) continue;
    a.fromBufferAttribute(pos, k[0]); b.fromBufferAttribute(pos, k[1]); c.fromBufferAttribute(pos, k[2]);
    n.crossVectors(b.clone().sub(a), c.clone().sub(a));
    if (n.lengthSq() < 1e-18) continue;
    const key = ["negX", "posX", "negY", "posY", "posZ", "negZ"].reduce((best, id) => {
      const f = CUBE_FACE_ATLAS[id];
      const dot = n.x * f.n[0] + n.y * f.n[1] + n.z * f.n[2];
      return dot > best.dot ? { id, dot, f } : best;
    }, { id: "", dot: -1, f: null });
    const f = key.f;
    const p3 = [a, b, c].map((v) => v.x * f.right[0] + v.y * f.right[1] + v.z * f.right[2]);
    const q3 = [a, b, c].map((v) => v.x * f.up[0] + v.y * f.up[1] + v.z * f.up[2]);
    const area3d = (p3[1] - p3[0]) * (q3[2] - q3[0]) - (q3[1] - q3[0]) * (p3[2] - p3[0]);
    const du1 = uv.getX(k[1]) - uv.getX(k[0]), dv1 = -(uv.getY(k[1]) - uv.getY(k[0]));
    const du2 = uv.getX(k[2]) - uv.getX(k[0]), dv2 = -(uv.getY(k[2]) - uv.getY(k[0]));
    const areaUv = du1 * dv2 - dv1 * du2;
    assert.ok(area3d * areaUv > 0, `${label}/${key.id}: UV orientation mirrored or inverted`);
    const uAvg = (uv.getX(k[0]) + uv.getX(k[1]) + uv.getX(k[2])) / 3;
    const vAvg = (uv.getY(k[0]) + uv.getY(k[1]) + uv.getY(k[2])) / 3;
    assert.ok(Math.abs(uAvg - f.cx) <= f.h * 1.1 && Math.abs(vAvg - f.cy) <= f.h * 1.1,
      `${label}/${key.id}: UV escaped cell center (${uAvg.toFixed(3)}, ${vAvg.toFixed(3)})`);
    facesHit.set(key.id, (facesHit.get(key.id) ?? 0) + 1);
  }
  assert.equal(facesHit.size, 6, `${label}: expected all 6 distinct cube faces mapped, got ${facesHit.size}`);
}

test("repairCubeUVs assigns six distinct un-mirrored, upright atlas faces", () => {
  // 23 mm marker quads embedded inside a 44 mm outer cube envelope (matches cube.obj layout).
  const marker = new THREE.BoxGeometry(23, 23, 23).toNonIndexed();
  const shell = new THREE.BoxGeometry(44, 44, 44).toNonIndexed();
  const mUv = marker.attributes.uv, sUv = shell.attributes.uv;
  for (let i = 0; i < mUv.count; i++) mUv.setXY(i, 0.1 + 0.1 * (i % 2), 0.2 + 0.1 * ((i + 1) % 2));
  for (let i = 0; i < sUv.count; i++) sUv.setXY(i, 0.85, 0.85);
  const pos = new Float32Array([...marker.attributes.position.array, ...shell.attributes.position.array]);
  const uv = new Float32Array([...mUv.array, ...sUv.array]);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  geo.setAttribute("uv", new THREE.BufferAttribute(uv, 2));
  repairCubeUVs(geo);
  verifyCubeMarkerLayout(geo, "synthetic-cube");
});

const HAVE_SDK = existsSync(join(ROOT, "public/robot/sdk/vector.obj"))
  && existsSync(join(ROOT, "public/robot/sdk/cube.obj"))
  && existsSync(join(ROOT, "public/robot/sdk/manifest.json"));

test("optional SDK vector.obj and cube.obj pass 3D geometry and normal checks", { skip: !HAVE_SDK }, () => {
  const v = readJson("public/robot/model.json").values;
  const manifest = readJson("public/robot/sdk/manifest.json");
  const meshes = new Map();
  new OBJLoader().parse(readText("public/robot/sdk/vector.obj")).traverse((node) => {
    if (node.isMesh) meshes.set(node.name, node);
  });
  const rig = withMockDocument(() => assemble(meshes, manifest.viewer_pose, v, [null, null, null], 2));
  assertCleanTree(meshes.get("body_geo"), "sdk-body");
  assertCleanTree(meshes.get("head_geo"), "sdk-head");
  const lowLift = Math.asin((v.lift_height_min_mm - v.lift_base_position_mm[2]) / v.lift_arm_length_mm);
  rig.setPose(0, lowLift, "calm", [0, 0, 0, 0], 1.0);
  rig.root.updateMatrixWorld(true);
  const forkZMinMm = new THREE.Box3().setFromObject(meshes.get("fork_geo")).min.z * 1000;
  assert.ok(forkZMinMm > 0.5 && forkZMinMm < 2.0, `unexpected SDK fork clearance ${forkZMinMm} mm`);

  const cubeGroup = new OBJLoader().parse(readText("public/robot/sdk/cube.obj"));
  cubeGroup.traverse((node) => {
    if (node.isMesh) {
      repairMeshNormals(node.geometry);
      repairCubeUVs(node.geometry);
      assert.equal(inspectMesh(node).inverted, 0);
      verifyCubeMarkerLayout(node.geometry, "sdk-cube");
    }
  });
});

function makeSyntheticBoxObj(tag, hx = 4, hy = 4, hz = 4, cx = 0, cy = 0, cz = 0) {
  const verts = [
    [cx - hx, cy - hy, cz - hz], [cx + hx, cy - hy, cz - hz],
    [cx + hx, cy + hy, cz - hz], [cx - hx, cy + hy, cz - hz],
    [cx - hx, cy - hy, cz + hz], [cx + hx, cy - hy, cz + hz],
    [cx + hx, cy + hy, cz + hz], [cx - hx, cy + hy, cz + hz],
  ];
  const tris = [
    [1, 3, 2], [1, 4, 3], [5, 6, 7], [5, 7, 8],
    [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6],
    [3, 4, 8], [3, 8, 7], [4, 1, 5], [4, 5, 8],
  ];
  const vLines = verts.map(([x, y, z]) => `v ${x} ${y} ${z}`);
  const fLines = tris.map(([a, b, c]) => `f ${a} ${b} ${c}`);
  return [`o box_${tag}`, `usemtl ${tag}`, ...vLines, ...fLines].join("\n") + "\n";
}

function makeSyntheticRigInputs(v) {
  const partTexts = new Map([
    ["chassis.obj", makeSyntheticBoxObj("gloss_black", 20, 18, 12, -28, 0, 15)],
    ["head_shell.obj", makeSyntheticBoxObj("dark_grey", 12, 14, 10, 0, 0, 0)],
    ["head_visor.obj", makeSyntheticBoxObj("visor", 2, 13, 9, 13, 0, 2)],
    ["eye_l.obj", makeSyntheticBoxObj("eye", 0.5, 4, 5, 0, 0, 0)],
    ["eye_r.obj", makeSyntheticBoxObj("eye", 0.5, 4, 5, 0, 0, 0)],
    ["lift_upper.obj", makeSyntheticBoxObj("gunmetal", 20, 3, 3, 18, 0, 0)],
    ["lift_lower.obj", makeSyntheticBoxObj("gunmetal", 20, 3, 3, 18, 0, 0)],
    ["fork.obj", makeSyntheticBoxObj("dark_grey", 10, 18, 2, 12, 0, -20)],
    ["wheel_fl.obj", makeSyntheticBoxObj("dark_grey", 12, 5, 12, 0, 0, 0)],
    ["wheel_fr.obj", makeSyntheticBoxObj("dark_grey", 12, 5, 12, 0, 0, 0)],
    ["wheel_bl.obj", makeSyntheticBoxObj("dark_grey", 9, 5, 9, 0, 0, 0)],
    ["wheel_br.obj", makeSyntheticBoxObj("dark_grey", 9, 5, 9, 0, 0, 0)],
  ]);
  const manifest = {
    source: { rig: "synthetic" },
    frames: {
      neck_mm: v.neck_position_mm ?? [0, 0, 44],
      eye_l_offset_mm: [13.67, 5.22, 3.55],
      eye_r_offset_mm: [13.67, -4.38, 3.55],
      upper_origin_mm: v.lift_base_position_mm ?? [-33, 0, 31],
      lower_origin_mm: [-31, 0, 21],
      wheel_front_mm: [0, 0, v.wheel_front_radius_mm ?? 13.85],
      wheel_rear_mm: [-(v.wheel_base_mm ?? 47.285), 0, v.wheel_rear_radius_mm ?? 10.1],
      tread_contact_left_y_mm: [18.15, 29.65],
      tread_contact_right_y_mm: [-29.73, -18.23],
    },
  };
  return { manifest, partTexts };
}

test("assembleRigBlend supports shell versions, faceplate toggle, level fork, 84 cleats, and symmetric eyes", () => {
  assert.equal(makeRigMaterials(1).byTag.get("ear_ring").color.getHex(), 0xc4af7b);
  assert.equal(makeRigMaterials(2).byTag.get("ear_ring").color.getHex(), 0x626469);
  const v = readJson("public/robot/model.json").values;
  const { manifest, partTexts } = makeSyntheticRigInputs(v);

  const covered = withMockDocument(() => assembleRigBlend(manifest, partTexts, v, [null, null, null], { faceplate: true, shellVersion: 1 }));
  assert.equal(covered.cleats.mesh.count, 84);
  assert.ok(covered.head.getObjectByName("wireos_lcd"));
  assert.ok(covered.root.getObjectByName("Rear badge"));
  assert.equal(covered.head.getObjectByName("head_visor").visible, true);

  covered.setPose(0, -0.2, "squint", [0, 0, 0, 0], 0.5);
  covered.root.updateMatrixWorld(true);
  const eyeLZ = new THREE.Box3().setFromObject(covered.head.getObjectByName("eye_L_geo")).getCenter(new THREE.Vector3()).z;
  const eyeRZ = new THREE.Box3().setFromObject(covered.head.getObjectByName("eye_R_geo")).getCenter(new THREE.Vector3()).z;
  assert.ok(Math.abs(eyeLZ - eyeRZ) < 1e-5, `eye Z asymmetry ${eyeLZ} vs ${eyeRZ}`);
  for (const liftAngle of [-0.5, 0.0, 0.45]) {
    covered.setPose(0, liftAngle, "calm", [0, 0, 0, 0], 1);
    assert.ok(Math.abs(covered.forkSwing.rotation.y + covered.forkLevel.rotation.y) < 1e-9);
  }

  const exposed = withMockDocument(() => assembleRigBlend(manifest, partTexts, v, [null, null, null], { faceplate: false, shellVersion: 2 }));
  assert.equal(exposed.head.getObjectByName("head_visor").visible, false);
  assert.ok(exposed.head.getObjectByName("face_carrier_mount"));
});

const HAVE_RIG = existsSync(join(ROOT, "public/robot/sdk/rig/rig_manifest.json"));

test("optional blended Victor_rig_01 meshes pass topology, eye symmetry, 84 cleats, and low-lift checks", { skip: !HAVE_RIG }, () => {
  const v = readJson("public/robot/model.json").values;
  const manifest = readJson("public/robot/sdk/rig/rig_manifest.json");
  const parts = [
    "chassis", "head_shell", "head_visor", "eye_l", "eye_r",
    "lift_upper", "lift_lower", "fork", "wheel_fl", "wheel_fr", "wheel_bl", "wheel_br",
  ];
  const partTexts = new Map(parts.map((p) => [`${p}.obj`, readText(`public/robot/sdk/rig/${p}.obj`)]));
  const rig = withMockDocument(() => assembleRigBlend(manifest, partTexts, v, [null, null, null], { faceplate: true, shellVersion: 1 }));
  assertCleanTree(rig.root, "rig-blend");
  assert.equal(rig.cleats.mesh.count, 84);

  for (const open of [1.0, 0.35]) {
    rig.setPose(0, 0, open < 1 ? "squint" : "calm", [0, 0, 0, 0], open);
    rig.root.updateMatrixWorld(true);
    const eyeLZ = new THREE.Box3().setFromObject(rig.head.getObjectByName("eye_L_geo")).getCenter(new THREE.Vector3()).z;
    const eyeRZ = new THREE.Box3().setFromObject(rig.head.getObjectByName("eye_R_geo")).getCenter(new THREE.Vector3()).z;
    assert.ok(Math.abs(eyeLZ - eyeRZ) < 1e-5, `open=${open}: eye Z mismatch ${eyeLZ} vs ${eyeRZ}`);
  }

  const lowLift = Math.asin((v.lift_height_min_mm - v.lift_base_position_mm[2]) / v.lift_arm_length_mm);
  rig.setPose(0, lowLift, "calm", [0, 0, 0, 0], 1.0);
  rig.root.updateMatrixWorld(true);
  const forkZMinMm = new THREE.Box3().setFromObject(rig.forkLevel).min.z * 1000;
  assert.ok(forkZMinMm > 0.2 && forkZMinMm < 3.0, `unexpected blended fork low-lift Z ${forkZMinMm} mm`);
});

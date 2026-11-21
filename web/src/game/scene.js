// Everything drawn: the model's own geoms, plus labelled cosmetics. Authored-by: CPCS.
//
// The physics model is boxes and cylinders: one mesh per MuJoCo geom, placed
// each frame from the solver's geom poses. The scenery is always drawn that
// way. The robot is drawn with the Vector SDK's 3D model when it has been
// fetched (sdkmodel.js), with the physics shapes available as an overlay;
// otherwise the robot is its physics shapes plus tread bands and a face.
// None of the SDK model, the tread bands, the face, the sensor beams or the
// trail is in the physics model, and none changes it.
//
// Coordinates: the model is z-up in metres. Everything hangs off one root
// group that turns z-up into three.js y-up, so poses are copied unconverted.

import * as THREE from "three";
import { HIDDEN_FROM_ROBOT, makeHeadCamera } from "./headcam.js";
import { makeCleats, makeSpokes } from "./tracks.js";

const PALETTE = {
  floor: { color: 0x1a1d21, roughness: 0.95 },
  table: { color: 0xc8a97a, roughness: 0.7 },
  obstacle: { color: 0xd9dde2, roughness: 0.45 },
  chassis_geom: { color: 0x666c76, roughness: 0.42, metalness: 0.15 },
  head_geom: { color: 0x3a3e45, roughness: 0.35, metalness: 0.15 },
  lift_arm_geom: { color: 0x8b929d, roughness: 0.4, metalness: 0.2 },
  lift_fork_geom: { color: 0xa9905a, roughness: 0.4, metalness: 0.35 },
  wheel: { color: 0x24272b, roughness: 0.85 },
};
const EYE = "#3df58c";
const RAY_OK = new THREE.Color(0x3df58c);
const RAY_CLIFF = new THREE.Color(0xff4d4d);
const RAY_IDLE = new THREE.Color(0x4a5560);
const RAY_WEAK = new THREE.Color(0xffb547); // the ray reached something, but the signal is too weak to count
const AXIS = [0, 0, 1];
const TOF_MISS_M = 0.08; // drawn length of a fan ray that hit nothing
const TRAIL_POINTS = 2000;
const SHAPE_OVERLAY = new THREE.MeshBasicMaterial({ color: 0x3df58c, wireframe: true, transparent: true, opacity: 0.55 });
const CLIFF_RAY_MAX_M = 0.06; // drawn length of a cliff beam that sees nothing

function styleFor(name) {
  if (name.startsWith("wheel_")) return PALETTE.wheel;
  if (name.startsWith("obstacle_")) return PALETTE.obstacle;
  return PALETTE[name] ?? null;
}

function geomMesh(mujoco, model, index, name) {
  const type = model.geom_type[index];
  const size = model.geom_size.subarray(3 * index, 3 * index + 3);
  const kinds = mujoco.mjtGeom;
  let geometry = null;
  if (type === kinds.mjGEOM_PLANE.value) geometry = new THREE.PlaneGeometry(8, 8);
  else if (type === kinds.mjGEOM_BOX.value) geometry = new THREE.BoxGeometry(2 * size[0], 2 * size[1], 2 * size[2]);
  else if (type === kinds.mjGEOM_CYLINDER.value) {
    geometry = new THREE.CylinderGeometry(size[0], size[0], 2 * size[1], 40);
    geometry.rotateX(Math.PI / 2); // three's cylinder runs along y, MuJoCo's along z
  }
  if (geometry === null) throw new Error(`geom ${name} has a type this viewer does not draw (${type})`);
  const rgba = model.geom_rgba.subarray(4 * index, 4 * index + 4);
  const style = styleFor(name) ?? { color: new THREE.Color(rgba[0], rgba[1], rgba[2]), roughness: 0.6 };
  const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial(style));
  mesh.name = name;
  mesh.matrixAutoUpdate = false;
  const scenery = model.geom_bodyid[index] === 0;
  mesh.castShadow = !scenery || name.startsWith("obstacle_");
  mesh.receiveShadow = scenery;
  return mesh;
}

// The face: two eyes on a dark screen. Cosmetic.
function makeFace(headHalf) {
  const canvas = document.createElement("canvas");
  canvas.width = 256;
  canvas.height = 128;
  const ctx = canvas.getContext("2d");
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(2 * headHalf[1] * 0.84, 2 * headHalf[2] * 0.66),
    new THREE.MeshBasicMaterial({ map: texture, toneMapped: false }),
  );
  // Plane axes (right, up, normal) -> head axes (left, up, forward).
  mesh.matrixAutoUpdate = false;
  mesh.matrix.makeBasis(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 1), new THREE.Vector3(1, 0, 0));
  mesh.matrix.setPosition(headHalf[0] + 0.0003, 0, 0.001);
  let drawn = "";
  const draw = (mood, open) => {
    const key = `${mood}|${open.toFixed(1)}`;
    if (key === drawn) return;
    drawn = key;
    ctx.fillStyle = "#050607";
    ctx.fillRect(0, 0, 256, 128);
    ctx.fillStyle = mood === "alarm" ? "#ff5a4d" : mood === "dim" ? "#1f6b44" : EYE;
    const w = mood === "alarm" ? 50 : 44;
    const h = Math.max(6, (mood === "squint" ? 22 : mood === "alarm" ? 64 : 52) * open);
    for (const cx of [84, 172]) {
      ctx.beginPath();
      ctx.roundRect(cx - w / 2, 64 - h / 2, w, h, Math.min(14, h / 2));
      ctx.fill();
    }
    texture.needsUpdate = true;
  };
  draw("calm", 1);
  return { mesh, draw };
}

// Cleats that travel round each track, for the robot drawn as its physics shapes. Cosmetic.
function makeShapeCleats(v) {
  const mm = 0.001;
  const sink = 0.0003; // half of each cleat sits inside the wheel's surface
  return makeCleats({
    front: { x: 0, z: v.wheel_front_radius_mm * mm, r: v.wheel_front_radius_mm * mm - sink },
    rear: { x: -v.wheel_base_mm * mm, z: v.wheel_rear_radius_mm * mm, r: v.wheel_rear_radius_mm * mm - sink },
    sideY: [0.5 * v.tread_spacing_mm * mm, -0.5 * v.tread_spacing_mm * mm],
    width: 0.98 * v.tread_width_mm * mm, pitch: 0.0045, along: 0.0014, proud: 2 * sink,
  });
}

// Tread bands between the wheels of each side. Cosmetic: contact is the wheels'.
function makeTreads(v) {
  const group = new THREE.Group();
  const mm = 0.001;
  const length = v.wheel_base_mm * mm;
  const height = 2 * v.wheel_rear_radius_mm * mm * 0.92;
  const material = new THREE.MeshStandardMaterial(PALETTE.wheel);
  for (const side of [1, -1]) {
    const band = new THREE.Mesh(new THREE.BoxGeometry(length, v.tread_width_mm * mm * 0.96, height), material);
    band.position.set(-0.5 * length, side * 0.5 * v.tread_spacing_mm * mm, 0.5 * height + 0.0006);
    band.castShadow = true;
    group.add(band);
  }
  return group;
}

// One line per beam: the four cliff sensors, then every ray of the range sensor's fan.
function makeRays(fanSize) {
  const segments = 4 + fanSize;
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(segments * 2 * 3), 3));
  geometry.setAttribute("color", new THREE.BufferAttribute(new Float32Array(segments * 2 * 3), 3));
  const lines = new THREE.LineSegments(geometry, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.9 }));
  lines.frustumCulled = false;
  return lines;
}

function makeTrail() {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(TRAIL_POINTS * 3), 3));
  geometry.setDrawRange(0, 0);
  const line = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: 0x1d7a4b, transparent: true, opacity: 0.85 }));
  line.frustumCulled = false;
  return line;
}

function addLights(scene) {
  scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x3a342c, 1.3));
  const key = new THREE.DirectionalLight(0xffffff, 2.2);
  key.position.set(0.5, 1.2, 0.6);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  const cam = key.shadow.camera;
  cam.left = cam.bottom = -0.5;
  cam.right = cam.top = 0.5;
  cam.near = 0.2;
  cam.far = 3;
  key.shadow.bias = -0.0004;
  scene.add(key);
  const rim = new THREE.DirectionalLight(0x9fd6ff, 0.5);
  rim.position.set(-0.6, 0.5, -0.8);
  scene.add(rim);
}

// Put an object and everything under it out of the robot camera's sight (the main view still sees it).
function hideFromRobot(object) {
  object.traverse((node) => node.layers.set(HIDDEN_FROM_ROBOT));
}

// sdk: the loaded SDK robot model, or null to draw the robot as its physics shapes.
// cameraSpec: the camera block of model.json.
export function buildVisuals(mujoco, sim, scene, sdk, cameraSpec, cube = null) {
  const { model } = sim;
  const v = sim.v;
  const nameOf = (kind, index) => mujoco.mj_id2name(model, mujoco.mjtObj[kind].value, index);
  const idOf = (kind, name) => mujoco.mj_name2id(model, mujoco.mjtObj[kind].value, name);
  const root = new THREE.Group();
  root.name = "model_root";
  root.rotation.x = -Math.PI / 2;
  scene.add(root);
  addLights(scene);
  const grid = new THREE.GridHelper(8, 32, 0x323a43, 0x242a31);
  grid.position.y = -v.table_height_mm * 0.001 + 0.0005;
  scene.add(grid);

  const meshes = [];
  const robotMeshes = [];
  const cubeId = idOf("mjOBJ_BODY", "cube");
  for (let i = 0; i < model.ngeom; i++) {
    const mesh = geomMesh(mujoco, model, i, nameOf("mjOBJ_GEOM", i));
    if (model.geom_bodyid[i] === cubeId) {
      mesh.visible = cube === null;
    } else if (sdk !== null && model.geom_bodyid[i] !== 0) {
      mesh.material = SHAPE_OVERLAY;
      mesh.castShadow = false;
      robotMeshes.push(mesh);
    }
    meshes.push(mesh);
    root.add(mesh);
  }
  const bodies = cube ? [["cube", cube]] : [];
  let face = null;
  let shapeCleats = null;
  if (sdk !== null) bodies.push(["chassis", sdk.root]); // the model's frame is the robot's: origin under the front axle
  else {
    const treads = makeTreads(v);
    shapeCleats = makeShapeCleats(v);
    treads.add(shapeCleats.mesh);
    bodies.push(["chassis", treads]);
    for (const mesh of meshes) {
      if (!mesh.name.startsWith("wheel_")) continue;
      // A wheel geom's own z axis is its axle: spokes on both faces show it turning.
      const { radiusTop: radius, height } = mesh.geometry.parameters;
      for (const side of [1, -1]) mesh.add(makeSpokes(0.9 * radius, side * (0.5 * height + 0.0002), "z", 0.0003));
    }
    face = makeFace(v.head_box_mm.map((s) => 0.0005 * s));
    const headGroup = new THREE.Group();
    headGroup.add(face.mesh);
    bodies.push(["head", headGroup]);
  }
  const bodyGroups = bodies.map(([name, group]) => {
    root.add(group);
    return { id: idOf("mjOBJ_BODY", name), group };
  });
  const rays = makeRays(sim.tofRays.length);
  const trail = makeTrail();
  root.add(rays, trail);
  // The robot's camera sits inside its own head and must not see it, nor the drawing aids.
  const headCamera = makeHeadCamera(mujoco, sim, cameraSpec, root);
  for (const aid of [rays, trail, ...robotMeshes]) hideFromRobot(aid);
  if (sdk !== null) hideFromRobot(sdk.head);
  else {
    hideFromRobot(face.mesh);
    hideFromRobot(meshes.find((mesh) => mesh.name === "head_geom"));
  }
  const siteIds = [...["front_left", "front_right", "back_left", "back_right"].map((n) => idOf("mjOBJ_SITE", `cliff_${n}`)),
    idOf("mjOBJ_SITE", "tof")];
  const trailState = { count: 0, lastX: NaN, lastY: NaN };

  function syncGeoms(data) {
    const pos = data.geom_xpos;
    const mat = data.geom_xmat;
    for (let i = 0; i < meshes.length; i++) {
      const p = 3 * i;
      const m = 9 * i;
      meshes[i].matrix.set(
        mat[m], mat[m + 1], mat[m + 2], pos[p],
        mat[m + 3], mat[m + 4], mat[m + 5], pos[p + 1],
        mat[m + 6], mat[m + 7], mat[m + 8], pos[p + 2],
        0, 0, 0, 1,
      );
    }
    const xpos = data.xpos;
    const xquat = data.xquat;
    for (const { id, group } of bodyGroups) {
      group.position.set(xpos[3 * id], xpos[3 * id + 1], xpos[3 * id + 2]);
      group.quaternion.set(xquat[4 * id + 1], xquat[4 * id + 2], xquat[4 * id + 3], xquat[4 * id]);
    }
  }

  // Beams from the four cliff sensors, and the fan of rays that samples the range sensor's cone.
  // A fan ray is drawn to what it hit; one that hit nothing is drawn short and dim.
  function syncRays(data, truth) {
    const position = rays.geometry.attributes.position;
    const color = rays.geometry.attributes.color;
    const sitePos = data.site_xpos;
    const siteMat = data.site_xmat;
    const put = (index, id, dir, length, tint) => {
      const ox = sitePos[3 * id];
      const oy = sitePos[3 * id + 1];
      const oz = sitePos[3 * id + 2];
      const m = 9 * id;
      position.setXYZ(2 * index, ox, oy, oz);
      position.setXYZ(2 * index + 1,
        ox + length * (siteMat[m] * dir[0] + siteMat[m + 1] * dir[1] + siteMat[m + 2] * dir[2]),
        oy + length * (siteMat[m + 3] * dir[0] + siteMat[m + 4] * dir[1] + siteMat[m + 5] * dir[2]),
        oz + length * (siteMat[m + 6] * dir[0] + siteMat[m + 7] * dir[1] + siteMat[m + 8] * dir[2]));
      color.setXYZ(2 * index, tint.r, tint.g, tint.b);
      color.setXYZ(2 * index + 1, tint.r, tint.g, tint.b);
    };
    for (let i = 0; i < 4; i++) {
      const reading = truth.cliff_ranges_mm[i];
      put(i, siteIds[i], AXIS, reading >= 0 ? reading * 0.001 : CLIFF_RAY_MAX_M, truth.cliff_flags[i] ? RAY_CLIFF : RAY_OK);
    }
    const found = sim.state.prox_found_object;
    sim.tofRays.forEach((dir, j) => {
      const hit = truth.tof_hits_mm[j] ?? -1;
      put(4 + j, siteIds[4], dir, hit >= 0 ? hit * 0.001 : TOF_MISS_M, hit < 0 ? RAY_IDLE : found ? RAY_OK : RAY_WEAK);
    });
    position.needsUpdate = true;
    color.needsUpdate = true;
  }

  function syncTrail(x, y, z) {
    if (z < -0.005) return; // off the table: the trail is drawn on the table top
    const moved = Math.hypot(x - trailState.lastX, y - trailState.lastY);
    if (moved < 0.002) return; // NaN on the first call compares false, so the first point is kept
    const position = trail.geometry.attributes.position;
    if (trailState.count === TRAIL_POINTS) {
      position.array.copyWithin(0, 3);
      trailState.count -= 1;
    }
    position.setXYZ(trailState.count, x, y, 0.0006);
    trailState.count += 1;
    trailState.lastX = x;
    trailState.lastY = y;
    position.needsUpdate = true;
    trail.geometry.setDrawRange(0, trailState.count);
  }

  return {
    // truth: sim.simTruth(); mood: "calm" | "squint" | "alarm" | "dim"; open: eyelid 0..1
    headCamera,
    sync(truth, mood, open, showRays, showShapes, showCamera) {
      const data = sim.data;
      syncGeoms(data);
      headCamera.update(data);
      headCamera.frustum.visible = showCamera;
      rays.visible = showRays;
      if (showRays) syncRays(data, truth);
      const q = data.qpos;
      syncTrail(q[0], q[1], q[2]);
      if (sdk !== null) {
        sdk.setPose(sim.headAngleRad(), sim.liftAngleRad(), mood, truth.wheel_angles_rad);
        for (const mesh of robotMeshes) mesh.visible = showShapes;
      } else {
        face.draw(mood, open);
        const frontR = 0.001 * v.wheel_front_radius_mm;
        shapeCleats.update(truth.wheel_angles_rad[0] * frontR, truth.wheel_angles_rad[1] * frontR);
      }
    },
    clearTrail() {
      trailState.count = 0;
      trailState.lastX = NaN;
      trailState.lastY = NaN;
      trail.geometry.setDrawRange(0, 0);
    },
    trailPoints() {
      return { array: trail.geometry.attributes.position.array, count: trailState.count };
    },
  };
}

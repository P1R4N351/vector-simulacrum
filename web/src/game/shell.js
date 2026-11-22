// Stock black and gold trim, projected onto the curved SDK shells so it
// follows the head and backpack. Profiles estimated from iFixit guide 123280.
import * as THREE from "three";
import { addUnderbody } from "./underbody.js";
import { addCrownAndBackpack } from "./topology.js";
import { addFaceCarrier, visorGeometry } from "./face.js";
import { DecalGeometry } from "three/addons/geometries/DecalGeometry.js";

function rounded(ctx, x, y, w, h, r, color) {
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.roundRect(x, y, w, h, r);
  ctx.fill();
}

function surface(paint) {
  const canvas = document.createElement("canvas");
  canvas.width = canvas.height = 256;
  const ctx = canvas.getContext("2d");
  paint(ctx);
  const map = new THREE.CanvasTexture(canvas);
  map.colorSpace = THREE.SRGBColorSpace;
  return new THREE.MeshStandardMaterial({ map, alphaTest: 0.5, metalness: 0.35, roughness: 0.43,
    polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2 });
}

// Rounded cheek around the ear insert. Its outer edge turns into the
// slotted crown and front bezel while the glossy inner edge meets the gold.
function cheek(side) {
  const vertices = [], indices = [], count = 80;
  const sections = [[1.27, 1.38, 1.55], [1.26, 1.37, 1.69], [1.24, 1.35, 1.83],
    [1.21, 1.32, 1.96], [1.18, 1.28, 2.004], [1.15, 1.22, 2.012],
    [1.07, 1.13, 2.038], [0.88, 0.93, 2.018], [0.78, 0.81, 1.978]];
  const geometry = new THREE.BufferGeometry();
  for (const [i, [w, h, depth]] of sections.entries()) {
    for (let j = 0; j < count; j++) {
      const t = 2 * Math.PI * j / count, c = Math.cos(t), v = Math.sin(t);
      const z = h * Math.sign(v) * Math.abs(v) ** 0.84;
      vertices.push(-1.16 + w * Math.sign(c) * Math.abs(c) ** 0.84 - 0.10 * z,
        side * (depth - 0.072 * z * z + 0.024 * c * z), 4.73 + z);
      if (!i) continue;
      const k = i * count + j, n = i * count + (j + 1) % count;
      if (side > 0) indices.push(k, n - count, k - count, k, n, n - count);
      else indices.push(k - count, n - count, k, n - count, n, k);
    }
    if (i) geometry.addGroup((i - 1) * count * 6, count * 6, i < 5 ? 0 : 1);
  }
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(vertices, 3));
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  return new THREE.Mesh(geometry, [
    new THREE.MeshStandardMaterial({ color: 0x626367, roughness: 0.38 }),
    new THREE.MeshStandardMaterial({ color: 0x27282b, roughness: 0.21 }),
  ]);
}

function decal(shell, position, rotation, size, material) {
  const geometry = new DecalGeometry(shell, new THREE.Vector3(...position),
    new THREE.Euler(...rotation), new THREE.Vector3(...size));
  shell.add(new THREE.Mesh(geometry, material));
}

function earCap(side) {
  const profile = new THREE.Shape();
  profile.moveTo(-0.77, -0.36);
  profile.quadraticCurveTo(-0.77, -0.77, -0.42, -0.79);
  profile.quadraticCurveTo(0, -0.86, 0.49, -0.78);
  profile.quadraticCurveTo(0.77, -0.73, 0.77, -0.39);
  profile.quadraticCurveTo(0.83, 0.42, 0.54, 0.66);
  profile.quadraticCurveTo(0.20, 0.89, -0.39, 0.81);
  profile.quadraticCurveTo(-0.71, 0.77, -0.77, 0.38);
  profile.closePath();
  const geometry = new THREE.ExtrudeGeometry(profile, { depth: 0.08, bevelSize: 0.09,
    bevelThickness: 0.07, bevelSegments: 4, curveSegments: 10 });
  const cap = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({
    color: 0xc4af7b, metalness: 0.43, roughness: 0.30,
  }));
  geometry.rotateZ(-0.105);
  cap.scale.y = -side;
  cap.rotation.x = -side * Math.PI / 2;
  cap.position.set(-1.04, side * 1.95, 4.7);
  return cap;
}

// Rounded front apron between the two wheel housings. Profile in the
// SDK's x/z plane, swept across the chassis, from the stock front view.
export function frontApron() {
  const profile = new THREE.Shape();
  profile.moveTo(0.3, 0.44);
  profile.lineTo(1.48, 0.44);
  profile.quadraticCurveTo(1.78, 0.48, 1.80, 0.79);
  profile.lineTo(1.70, 1.14);
  profile.quadraticCurveTo(1.45, 1.27, 1.34, 1.58);
  profile.lineTo(0.35, 1.62);
  profile.closePath();
  const geometry = new THREE.ExtrudeGeometry(profile, {
    depth: 2.86, bevelEnabled: true, bevelThickness: 0.13,
    bevelSize: 0.11, bevelSegments: 4, curveSegments: 12,
  });
  geometry.translate(0, 0, -1.43);
  geometry.rotateX(Math.PI / 2);
  const apron = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color: 0x303135, roughness: 0.30 }));
  apron.castShadow = true;
  return apron;
}

// Call before rig assembly, while both shells are in the OBJ coordinate frame.
export function addShellTrim(meshes, values, frontAxleX, shellVersion) {
  const head = meshes.get("head_geo");
  meshes.get("body_geo").add(frontApron());
  addUnderbody(meshes.get("body_geo"), values, frontAxleX, shellVersion);
  addCrownAndBackpack(head, meshes.get("body_geo"), shellVersion);
  for (const side of [-1, 1]) {
    head.add(earCap(side));
    head.add(cheek(side));
  }

  // The pressed side cover narrows toward the idler. Mirror its outline on
  // the two sides; each decal projects onto the existing wheel carrier.
  for (const [name, side] of [["trackBase_L_geo", 1], ["trackBase_R_geo", -1]]) {
    const panel = surface(ctx => {
      ctx.translate(128, 128);
      ctx.scale(1, side);
      ctx.beginPath();
      ctx.moveTo(113, -92);
      ctx.quadraticCurveTo(78, -88, -72, -52);
      ctx.quadraticCurveTo(-93, -48, -92, -29);
      ctx.lineTo(-92, 37);
      ctx.quadraticCurveTo(-92, 53, -70, 55);
      ctx.lineTo(110, 88);
      ctx.strokeStyle = "#515154";
      ctx.lineWidth = 6;
      ctx.stroke();
      ctx.fillStyle = "#252528";
      ctx.fill();
    });
    decal(meshes.get(name), [-2.18, side * 2.785, 1.34],
      [-side * Math.PI / 2, 0, 0], [2.95, 1.75, 0.28], panel);
  }

  // The SDK draws the eyes as isolated dots. Diffuse their light across
  // the LCD behind the curved cover, keeping the SDK eye material for mood.
  const eyeMask = document.createElement("canvas");
  eyeMask.width = eyeMask.height = 128;
  const ctx = eyeMask.getContext("2d");
  ctx.fillStyle = "#060708";
  ctx.fillRect(0, 0, 128, 128);
  ctx.shadowColor = "white";
  ctx.shadowBlur = 12;
  const phosphor = ctx.createRadialGradient(62, 58, 12, 64, 64, 70);
  phosphor.addColorStop(0, "white");
  phosphor.addColorStop(0.62, "#dcecdc");
  phosphor.addColorStop(1, "#748874");
  rounded(ctx, 17, 15, 94, 98, 24, phosphor);
  const eyeMap = new THREE.CanvasTexture(eyeMask);
  eyeMap.colorSpace = THREE.SRGBColorSpace;
  for (const name of ["eye_L_geo", "eye_R_geo"]) {
    const eye = meshes.get(name);
    eye.geometry.computeBoundingBox();
    const center = eye.geometry.boundingBox.getCenter(new THREE.Vector3());
    eye.geometry = new THREE.PlaneGeometry(1.02, 1.07);
    eye.geometry.rotateY(Math.PI / 2);
    eye.geometry.translate(0.397, center.y * 1.15, center.z);
    eye.material.map = eyeMap;
    eye.material.transparent = false;
    eye.material.depthWrite = true;
    eye.renderOrder = 1;
  }
  const visor = meshes.get("front_Screen_geo");
  visor.geometry.dispose();
  visor.geometry = visorGeometry();
  // Transmission samples the opaque LCD and camera through the curved
  // shield. The tint and wall thickness are visual estimates in SDK cm.
  visor.material = new THREE.MeshPhysicalMaterial({
    color: 0xe6e3ef, transmission: 0.98, thickness: 0.09, ior: 1.55,
    attenuationColor: 0x585165, attenuationDistance: 0.0035, roughness: 0.095,
    clearcoat: 1, clearcoatRoughness: 0.08, envMapIntensity: 1.3,
  });
  visor.castShadow = false;
  visor.renderOrder = 2;

  const [proxX, proxY, proxZ] = values.tof_position_mm;
  const rangeWindow = surface(ctx => {
    rounded(ctx, 4, 4, 248, 248, 50, "#24262a");
    rounded(ctx, 12, 16, 232, 224, 44, "#10131a");
    rounded(ctx, 102, 81, 52, 94, 18, "#24252f");
  });
  decal(meshes.get("body_geo"), [frontAxleX + proxX / 10, proxY / 10, proxZ / 10],
    [0, Math.PI / 2, Math.PI / 2], [1.91, 0.84, 0.9], rangeWindow);

  addFaceCarrier(head, values, frontAxleX);
}

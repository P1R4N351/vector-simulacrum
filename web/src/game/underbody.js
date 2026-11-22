// Chassis details in the SDK centimetre frame. Sensor centres come from
// firmware. The external photos in Anki's FCC exhibit 4074220 supply the profiles.
import * as THREE from "three";
import { RoundedBoxGeometry } from "three/addons/geometries/RoundedBoxGeometry.js";

import { alignGeometryNormals, capsule, closedEllipse } from "./topology.js";

function subdivideRearZone(geometry, xLimit = -4.95, maxEdge = 0.07, passes = 4) {
  let cur = geometry.index ? geometry.toNonIndexed() : geometry;
  const maxEdgeSq = maxEdge * maxEdge;
  const edgeSq = (arr, i, j) => (arr[i] - arr[j]) ** 2 + (arr[i + 1] - arr[j + 1]) ** 2 + (arr[i + 2] - arr[j + 2]) ** 2;
  const pushV = (outP, outN, p, n, idx) => {
    outP.push(p[idx], p[idx + 1], p[idx + 2]);
    outN.push(n[idx], n[idx + 1], n[idx + 2]);
  };
  const pushMid = (outP, outN, p, n, i, j) => {
    outP.push(0.5 * (p[i] + p[j]), 0.5 * (p[i + 1] + p[j + 1]), 0.5 * (p[i + 2] + p[j + 2]));
    const nx = n[i] + n[j], ny = n[i + 1] + n[j + 1], nz = n[i + 2] + n[j + 2];
    const len = Math.hypot(nx, ny, nz) || 1;
    outN.push(nx / len, ny / len, nz / len);
  };
  for (let pass = 0; pass < passes; pass++) {
    const p = cur.attributes.position.array;
    const n = cur.attributes.normal.array;
    const outP = [], outN = [];
    let split = false;
    for (let k = 0; k < p.length; k += 9) {
      const i0 = k, i1 = k + 3, i2 = k + 6;
      const inZone = Math.min(p[i0], p[i1], p[i2]) < xLimit;
      const d01 = inZone ? edgeSq(p, i0, i1) : 0;
      const d12 = inZone ? edgeSq(p, i1, i2) : 0;
      const d20 = inZone ? edgeSq(p, i2, i0) : 0;
      const longest = Math.max(d01, d12, d20);
      if (longest <= maxEdgeSq) {
        for (const vi of [i0, i1, i2]) pushV(outP, outN, p, n, vi);
        continue;
      }
      split = true;
      const [a, b, c] = d01 === longest ? [i0, i1, i2] : d12 === longest ? [i1, i2, i0] : [i2, i0, i1];
      pushV(outP, outN, p, n, a); pushMid(outP, outN, p, n, a, b); pushV(outP, outN, p, n, c);
      pushMid(outP, outN, p, n, a, b); pushV(outP, outN, p, n, b); pushV(outP, outN, p, n, c);
    }
    if (!split) break;
    const next = new THREE.BufferGeometry();
    next.setAttribute("position", new THREE.Float32BufferAttribute(outP, 3));
    next.setAttribute("normal", new THREE.Float32BufferAttribute(outN, 3));
    if (cur !== geometry) cur.dispose();
    cur = next;
  }
  return cur;
}

export function addUnderbody(body, v, frontAxleX, shellVersion = 1) {
  const plastic = new THREE.MeshStandardMaterial({ color: 0x242529, roughness: 0.35 });
  const lens = new THREE.MeshPhysicalMaterial({ color: 0x101318, clearcoat: 0.9, roughness: 0.16 });
  const pan = capsule(-5.24, -1.77, 6.36, 3.54, 0.23);
  function slab(name, x, y, z, w, h, d, r, material) {
    const mesh = new THREE.Mesh(new RoundedBoxGeometry(w, h, d, 3, r), material);
    mesh.position.set(x, y, z);
    mesh.name = name;
    body.add(mesh);
  }
  for (const xMm of [v.cliff_front_x_mm, v.cliff_rear_x_mm]) {
    for (const side of [-1, 1]) {
      const x = frontAxleX + xMm / 10, y = side * v.cliff_y_mm / 10;
      pan.holes.push(capsule(x - 0.277, y - 0.208, 0.554, 0.416, 0.048));
      slab("Cliff sensor window", x, y, 0.212, 0.565, 0.43, 0.024, 0.009, lens);
    }
  }
  const tray = new THREE.Mesh(new THREE.ExtrudeGeometry(pan, {
    depth: 0.07, bevelEnabled: true, bevelThickness: 0.016, bevelSize: 0.022, bevelSegments: 3,
  }), plastic);
  if (shellVersion === 2) {
    // The forward edge of the grey return rises at its ends. Bring the
    // black pan up to the same seam, starting behind the rear windows.
    const flat = tray.geometry;
    tray.geometry = subdivideRearZone(flat, -4.95, 0.07, 4);
    flat.dispose();
    const { position, normal } = tray.geometry.attributes;
    for (let i = 0; i < position.count; i++) {
      const x = position.getX(i), y = position.getY(i);
      const xClamped = Math.max(-5.24, x);
      const d = Math.max(0, (-4.99 - xClamped) / 0.25);
      if (!d) continue;
      const u = THREE.MathUtils.clamp((Math.abs(y) - 0.68) / 0.73, 0, 1);
      const rise = 0.035 + 0.84 * u * u * (3 - 2 * u);
      const slopeY = 0.84 * 6 * u * (1 - u) * Math.sign(y) / 0.73;
      position.setZ(i, position.getZ(i) + rise * d ** 2);
      const nz = normal.getZ(i);
      const nx = normal.getX(i) + (x < -5.24 ? 0 : 8 * rise * d * nz);
      const ny = normal.getY(i) - slopeY * d ** 2 * nz;
      const n = Math.hypot(nx, ny, nz);
      normal.setXYZ(i, nx / n, ny / n, nz / n);
    }
    alignGeometryNormals(tray.geometry);
  }
  tray.position.z = 0.151;
  tray.name = "Underside pan";
  body.add(tray);

  if (shellVersion === 2) {
    addServiceCover(body, frontAxleX, slab);
    return;
  }

  const plate = new THREE.MeshStandardMaterial({ color: 0x393a39, roughness: 0.57 });
  const gold = new THREE.MeshStandardMaterial({ color: 0xc7a054, roughness: 0.31, metalness: 0.66 });
  slab("Charge contact carrier", -3.15, 0, 0.124, 2.53, 2.65, 0.048, 0.022, plate);
  for (const y of [-0.99, 0.99]) slab("Charging contact", -3.59, y, 0.094, 1.1, 0.55, 0.03, 0.012, gold);

  const canvas = document.createElement("canvas");
  canvas.width = 384; canvas.height = 192;
  const ctx = canvas.getContext("2d");
  ctx.fillStyle = "#292a29"; ctx.fillRect(0, 0, 384, 192);
  ctx.fillStyle = "#ceccc2"; ctx.font = "37px Arial"; ctx.fillText("Vector", 12, 43);
  ctx.font = "20px Arial"; ctx.fillText("Anki, Inc.", 12, 90);
  ctx.fillText("FCC ID  2AAIC00010", 12, 120);
  ctx.font = "38px Arial"; ctx.fillText("CE", 291, 172);
  const map = new THREE.CanvasTexture(canvas);
  map.colorSpace = THREE.SRGBColorSpace;
  const label = new THREE.Mesh(new THREE.PlaneGeometry(2.36, 1.26),
    new THREE.MeshStandardMaterial({ map, roughness: 0.68 }));
  label.rotation.y = Math.PI;
  label.position.set(-3.16, 0, 0.098);
  label.name = "Vector identification label";
  body.add(label);
}

// Coordinates relative to the front axle, estimated by registering the four
// window centres in the owner's Vector 2.0 underside photo to the firmware.
function addServiceCover(body, axle, slab) {
  const outline = new THREE.Shape();
  outline.moveTo(-4.35, -1.44);
  outline.quadraticCurveTo(-4.35, -1.51, -4.28, -1.51);
  for (const [x, y] of [[-0.28,-1.51], [-0.28,-0.91], [0.52,-0.91],
    [0.52,0.94], [-0.25,0.94], [-0.25,1.52], [-4.28,1.52]]) outline.lineTo(x, y);
  outline.quadraticCurveTo(-4.35, 1.52, -4.35, 1.45);
  outline.closePath();
  outline.holes.push(closedEllipse(0.39, 0, 0.102, 0.102, true, 24));
  const cover = new THREE.Mesh(new THREE.ExtrudeGeometry(outline, {
    depth: 0.035, bevelThickness: 0.009, bevelSize: 0.013, bevelSegments: 2,
  }), new THREE.MeshStandardMaterial({color: 0x272729, roughness: 0.49}));
  cover.name = "Vector 2.0 battery door";
  cover.position.set(axle, 0, 0.097);
  body.add(cover);

  const metal = new THREE.MeshStandardMaterial({color: 0x525259, metalness: 0.7, roughness: 0.37});
  const screw = new THREE.Mesh(new THREE.CylinderGeometry(0.085, 0.085, 0.018, 20), metal);
  screw.rotation.x = Math.PI / 2;
  screw.position.set(axle + 0.39, 0, 0.116);
  screw.name = "Battery door screw";
  body.add(screw);
  const groove = new THREE.MeshStandardMaterial({color: 0x121315});
  for (const [w,h] of [[0.104,0.023],[0.023,0.104]]) {
    slab("Screw drive", screw.position.x, 0, 0.105, w,h,0.004,0.001,groove);
  }
  const brass = new THREE.MeshStandardMaterial({color: 0xb69656, metalness: 0.72, roughness: 0.37});
  for (const y of [-0.99, 0.99]) {
    slab("Charging contact", axle - 3.52, y, 0.078, 1.22, 0.55, 0.022, 0.009, brass);
  }

  const canvas = document.createElement("canvas");
  canvas.width = 640; canvas.height = 448;
  const ctx = canvas.getContext("2d");
  ctx.fillStyle = "#39393c"; ctx.fillRect(0, 0, 640, 448);
  ctx.fillStyle = "#d3d3d1"; ctx.font = "bold 28px Arial";
  ctx.fillText("AI Robot Companion", 24, 51);
  ctx.font = "24px Arial"; ctx.fillText("Digital Dream Labs, Inc.", 24, 126);
  ctx.fillText("Pittsburgh, PA 15206", 24, 161);
  ctx.fillText("MODEL: Vector 2.0", 24, 326);
  ctx.font = "bold 41px Arial"; ctx.fillText("CE", 401, 56);
  ctx.font = "24px Arial"; ctx.fillText("UKCA", 397, 132);
  ctx.fillText("RATED: 3.7 V   600 mAh", 313, 367);
  const map = new THREE.CanvasTexture(canvas);
  map.colorSpace = THREE.SRGBColorSpace;
  const label = new THREE.Mesh(new THREE.PlaneGeometry(2.47,1.37),
    new THREE.MeshStandardMaterial({map, roughness: 0.74}));
  label.rotation.y = Math.PI;
  label.position.set(axle - 2.88, 0.025, 0.08);
  label.name = "Vector 2.0 identification label";
  body.add(label);

  const silk = document.createElement("canvas");
  silk.width = 640; silk.height = 64;
  const print = silk.getContext("2d");
  print.fillStyle = "#dfc8a5";
  print.font = "bold 47px Arial";
  print.fillText("S/N: —       ESN: —", 4, 49);
  const ink = new THREE.CanvasTexture(silk);
  ink.colorSpace = THREE.SRGBColorSpace;
  const serial = new THREE.Mesh(new THREE.PlaneGeometry(2.24, 0.23),
    new THREE.MeshStandardMaterial({map: ink, transparent: true, depthWrite: false}));
  serial.rotation.y = Math.PI;
  serial.position.set(axle - 1.50, 1.24, 0.083);
  serial.name = "Serial number fields";
  body.add(serial);
}

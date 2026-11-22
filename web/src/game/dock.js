// Vector charger 3D visuals: Home marker + ramp alignment decals, and optional
// assembly of kercre123/vector-animations-raw OBJ meshes (fetched by
// tools/fetch_dock_model.py into public/robot/sdk/).
import * as THREE from "three";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";
import { toCreasedNormals } from "three/addons/utils/BufferGeometryUtils.js";
import { alignGeometryNormals } from "./topology.js";

// Proper cyclic transform (det = +1) from Maya mm (X_m, Y_m, Z_m) to canonical charger metres:
//   x_c = (Z_m + 81.2401) mm, y_c = X_m mm, z_c = (Y_m + 0.5586) mm.
export const CHARGER_FROM_MAYA_M = new THREE.Matrix4().set(
  0, 0, 0.001, 0.0812401,
  0.001, 0, 0, 0,
  0, 0.001, 0, 0.0005586,
  0, 0, 0, 1,
);

// Normalized [0, 1]^2 contours extracted from Anki Vision::MARKER_CHARGER_HOME (chargerHome.png).
export const HOME_MARKER_POLYGONS = {
  house: [
    [0.500, 0.215], [0.535, 0.229], [0.805, 0.520], [0.795, 0.539], [0.719, 0.539],
    [0.719, 0.752], [0.703, 0.785], [0.600, 0.785], [0.611, 0.410], [0.551, 0.393],
    [0.543, 0.371], [0.457, 0.371], [0.451, 0.393], [0.395, 0.402], [0.402, 0.785],
    [0.307, 0.787], [0.287, 0.772], [0.283, 0.541], [0.205, 0.539], [0.195, 0.520],
    [0.465, 0.229],
  ],
  chimney: [
    [0.609, 0.242], [0.613, 0.240], [0.709, 0.240], [0.717, 0.248],
    [0.717, 0.375], [0.711, 0.381], [0.703, 0.379], [0.606, 0.277], [0.604, 0.250],
  ],
  bolt: [
    [0.549, 0.484], [0.551, 0.488], [0.506, 0.582], [0.563, 0.582], [0.565, 0.586],
    [0.463, 0.670], [0.461, 0.666], [0.488, 0.607], [0.443, 0.606],
  ],
};

function tracePolygon(ctx, pts, size) {
  ctx.beginPath();
  pts.forEach(([x, y], i) => {
    if (i === 0) ctx.moveTo(x * size, y * size);
    else ctx.lineTo(x * size, y * size);
  });
  ctx.closePath();
  ctx.fill();
}

export function drawHomeMarker(ctx, sizePx = 512) {
  ctx.fillStyle = "#24262a";
  ctx.fillRect(0, 0, sizePx, sizePx);
  ctx.fillStyle = "#eef1f4";
  ctx.beginPath();
  ctx.roundRect(0.03 * sizePx, 0.03 * sizePx, 0.94 * sizePx, 0.94 * sizePx, 0.12 * sizePx);
  ctx.fill();
  ctx.fillStyle = "#24262a";
  ctx.beginPath();
  ctx.roundRect(0.115 * sizePx, 0.115 * sizePx, 0.77 * sizePx, 0.77 * sizePx, 0.055 * sizePx);
  ctx.fill();
  ctx.fillStyle = "#eef1f4";
  tracePolygon(ctx, HOME_MARKER_POLYGONS.house, sizePx);
  tracePolygon(ctx, HOME_MARKER_POLYGONS.chimney, sizePx);
  tracePolygon(ctx, HOME_MARKER_POLYGONS.bolt, sizePx);
}

export function makeHomeMarkerTexture(sizePx = 512) {
  const canvas = document.createElement("canvas");
  canvas.width = sizePx;
  canvas.height = sizePx;
  drawHomeMarker(canvas.getContext("2d"), sizePx);
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

function makeStripeSegment(x0Mm, z0Mm, x1Mm, z1Mm, widthMm, material) {
  const dx = (x1Mm - x0Mm) * 0.001, dz = (z1Mm - z0Mm) * 0.001;
  const len = Math.hypot(dx, dz);
  const pitch = Math.atan2(dz, dx);
  const nx = -dz / len, nz = dx / len;
  const geo = new THREE.PlaneGeometry(len, widthMm * 0.001);
  // Rotate PlaneGeometry (XY plane, normal +Z) so local +X follows slope tangent and normal points +nz up.
  geo.rotateX(0);
  const mesh = new THREE.Mesh(geo, material);
  mesh.position.set(0.5 * (x0Mm + x1Mm) * 0.001 + 0.00025 * nx, 0, 0.5 * (z0Mm + z1Mm) * 0.001 + 0.00025 * nz);
  mesh.rotation.y = -pitch;
  return mesh;
}

export function makeDockDecals({ includePins = true } = {}) {
  const group = new THREE.Group();
  group.name = "dock_decals";
  const markerMat = new THREE.MeshStandardMaterial({
    map: makeHomeMarkerTexture(512),
    roughness: 0.52,
    polygonOffset: true,
    polygonOffsetFactor: -1,
  });
  // Marker plane faces -X_charger (right = +Y_charger, up = +Z_charger -> lookAt(-X) with up +Z).
  const marker = new THREE.Mesh(new THREE.PlaneGeometry(0.046, 0.046), markerMat);
  marker.name = "dock_home_marker";
  marker.up.set(0, 0, 1);
  marker.position.set(0.09295, 0, 0.0485);
  marker.lookAt(0, 0, 0.0485);
  group.add(marker);

  const stripeMat = new THREE.MeshStandardMaterial({
    color: 0xd9dde2,
    roughness: 0.68,
    polygonOffset: true,
    polygonOffsetFactor: -1,
  });
  const apronStripe = makeStripeSegment(6, 2.55, 44, 8.92, 10.0, stripeMat);
  apronStripe.name = "dock_apron_stripe";
  const bedStripe = makeStripeSegment(44, 8.92, 80, 13.49, 10.0, stripeMat);
  bedStripe.name = "dock_bed_stripe";
  group.add(apronStripe, bedStripe);

  if (includePins) {
    const brass = new THREE.MeshStandardMaterial({ color: 0xc7a054, roughness: 0.31, metalness: 0.68 });
    for (const [name, yM] of [["dock_pin_l_visual", -0.00985], ["dock_pin_r_visual", 0.00985]]) {
      const pin = new THREE.Mesh(new THREE.BoxGeometry(0.0062, 0.0020, 0.0054), brass);
      pin.name = name;
      pin.position.set(0.05971, yM, 0.0110);
      group.add(pin);
    }
  }
  return group;
}

export function repairDockNormals(geometry) {
  const src = geometry.index ? geometry.toNonIndexed() : geometry.clone();
  const pos = src.attributes.position;
  const uv = src.attributes.uv ?? null;
  const keepPos = [];
  const keepUv = [];
  const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3();
  const e1 = new THREE.Vector3(), e2 = new THREE.Vector3(), cross = new THREE.Vector3();
  for (let t = 0; t + 2 < pos.count; t += 3) {
    a.fromBufferAttribute(pos, t);
    b.fromBufferAttribute(pos, t + 1);
    c.fromBufferAttribute(pos, t + 2);
    cross.crossVectors(e1.subVectors(b, a), e2.subVectors(c, a));
    if (cross.length() <= 1e-10) continue;
    for (let k = 0; k < 3; k++) {
      const idx = t + k;
      keepPos.push(pos.getX(idx), pos.getY(idx), pos.getZ(idx));
      if (uv) keepUv.push(uv.getX(idx), uv.getY(idx));
    }
  }
  const clean = new THREE.BufferGeometry();
  clean.setAttribute("position", new THREE.Float32BufferAttribute(keepPos, 3));
  if (uv) clean.setAttribute("uv", new THREE.Float32BufferAttribute(keepUv, 2));
  const creased = toCreasedNormals(clean, Math.PI / 4);
  if (clean !== src) clean.dispose();
  if (src !== geometry) src.dispose();
  alignGeometryNormals(creased, 0.05);
  return creased;
}

function materialForPart(name) {
  const lower = (name || "").toLowerCase();
  if (lower.includes("pin") || lower.includes("pplane")) {
    return new THREE.MeshStandardMaterial({ color: 0xc7a054, roughness: 0.30, metalness: 0.72 });
  }
  if (lower.includes("base")) {
    return new THREE.MeshStandardMaterial({ color: 0x17181b, roughness: 0.85, metalness: 0.02 });
  }
  if (lower.includes("chordin")) {
    return new THREE.MeshStandardMaterial({ color: 0x1d1e22, roughness: 0.62, metalness: 0.04 });
  }
  return new THREE.MeshStandardMaterial({ color: 0x26282c, roughness: 0.48, metalness: 0.06 });
}

export function assembleDockObj(objRoot) {
  const group = new THREE.Group();
  group.name = "sdk_dock";
  const parts = [];
  objRoot.traverse((node) => {
    if (!node.isMesh || !node.geometry?.attributes?.position) return;
    node.geometry.applyMatrix4(CHARGER_FROM_MAYA_M);
    node.geometry = repairDockNormals(node.geometry);
    node.material = materialForPart(node.name);
    node.castShadow = true;
    node.receiveShadow = true;
    parts.push(node);
  });
  for (const part of parts) group.add(part);
  group.add(makeDockDecals({ includePins: false }));
  return group;
}

export async function loadDockLook(base = "./robot/sdk/", options = {}) {
  try {
    if (options.requireManifest !== false) {
      const resp = await fetch(`${base}dock_manifest.json`, { cache: "no-cache" });
      if (!resp.ok || (resp.headers.get("content-type") ?? "").includes("text/html")) return null;
      const manifest = await resp.json();
      if (!manifest.sha256?.["Vic_charger_hp_geo.obj"] && !manifest.sha256?.["vic_charger_midPoly.obj"]) return null;
    }
    const objName = options.midPoly ? "vic_charger_midPoly.obj" : "Vic_charger_hp_geo.obj";
    const resp = await fetch(`${base}${objName}`, { cache: "no-cache" });
    if (!resp.ok || (resp.headers.get("content-type") ?? "").includes("text/html")) return null;
    const objText = await resp.text();
    const parsed = new OBJLoader().parse(objText);
    return assembleDockObj(parsed);
  } catch (err) {
    console.warn("Dock OBJ unavailable; using collision boxes", err);
    return null;
  }
}

// Crown and backpack in the SDK's centimetre frame. Widths, edge profiles
// and relief are estimates from the owner's top/side photograph.
import * as THREE from "three";
import { headRoof } from "./face.js";
import { TessellateModifier } from "three/addons/modifiers/TessellateModifier.js";

export function capsule(x, y, w, h, r) {
  const p = new THREE.Shape();
  p.moveTo(x + r, y);
  p.lineTo(x + w - r, y);
  p.quadraticCurveTo(x + w, y, x + w, y + r);
  p.lineTo(x + w, y + h - r);
  p.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
  p.lineTo(x + r, y + h);
  p.quadraticCurveTo(x, y + h, x, y + h - r);
  p.lineTo(x, y + r);
  p.quadraticCurveTo(x, y, x + r, y);
  return p;
}

// Warp both faces of the extrusion onto the same roof. Preserve the top
// surface's analytic normal so the thin rounded covers reflect light smoothly.
function cover(shell, roof) {
  return function layer(name, shape, offset, depth, color, roughness, dome) {
    const blank = new THREE.ExtrudeGeometry(shape, { depth, bevelEnabled: false, curveSegments: 7 });
    const geo = new TessellateModifier(0.12, 9).modify(blank);
    const pos = geo.attributes.position;
    const normals = geo.attributes.normal;
    const normal = new THREE.Vector3();
    const height = (x, y) => roof(x, y) + (dome ? dome(x, y) : 0);
    for (let i = 0; i < pos.count; i++) {
      const x = pos.getX(i), y = pos.getY(i);
      pos.setZ(i, height(x, y) + offset + pos.getZ(i));
      normal.fromBufferAttribute(normals, i);
      // Inverse transpose of the vertical displacement's Jacobian.
      normal.x -= normal.z * (height(x + 0.002, y) - height(x - 0.002, y)) / 0.004;
      normal.y -= normal.z * (height(x, y + 0.002) - height(x, y - 0.002)) / 0.004;
      normal.normalize();
      normals.setXYZ(i, normal.x, normal.y, normal.z);
    }
    const mesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color, roughness }));
    mesh.name = name;
    shell.add(mesh);
    blank.dispose();
  };
}

// Follow the centreline from the underside to the top of the touch plate.
// Every transverse section rolls over the black pivot housing. The indicator
// and electrode use the same roof equation as the last section.
function addRearShell(body, roof, shellVersion) {
  const joinX = -5.82, joinZ = roof(joinX, 0) + 0.108;
  const profile = new THREE.CurvePath();
  const point = (x, z) => new THREE.Vector2(x, z);
  profile.add(new THREE.CubicBezierCurve(point(-5.15, 0.36), point(-5.58, 0.56),
    point(-6.18, 2.21), point(-6.21, 3.46)));
  const slope = (roof(joinX + 0.001, 0) - roof(joinX - 0.001, 0)) / 0.002;
  profile.add(new THREE.CubicBezierCurve(point(-6.21, 3.46), point(-6.23, 4.29),
    point(joinX - 0.28, joinZ - 0.28 * slope), point(joinX, joinZ)));
  // The 2.0 cover turns almost parallel to the belly before ending just
  // aft of the rear cliff windows. Keep the upper shell on its original arc.
  const belly = new THREE.CurvePath();
  // The two tape edges trace a gradual longitudinal bend from lip to seam.
  // Keep the centre broad across the robot while letting it turn along its length.
  const shoulder = point(-6.08, 2.08), lastControl = point(-5.857, 1.081);
  belly.add(new THREE.CubicBezierCurve(point(-5.22, 0.113), point(-5.444, 0.113),
    lastControl, shoulder));
  const shoulderTangent = shoulder.clone().sub(lastControl);
  belly.add(new THREE.CubicBezierCurve(shoulder, shoulder.clone().addScaledVector(shoulderTangent, 0.44),
    point(-6.19, 2.63), point(-6.21, 3.46)));
  // In the registered view the outboard lip is already turning up while
  // the centre is still on the belly. Loft the cross sections to that rail.
  const rail = new THREE.CubicBezierCurve(point(-5.30, 0.94), point(-5.47, 1.04),
    point(-6.14, 2.35), point(-6.18, 3.46));
  const lowerEnd = profile.curves[0].getLength() / profile.getLength();
  const vertices = [], grey = [], black = [], cols = 80, lowerRows = 100, upperRows = 40;
  const smooth = THREE.MathUtils.smoothstep;
  for (let row = 0; row <= lowerRows + upperRows; row++) {
    const t = Math.min(row / lowerRows, 1);
    const under = shellVersion === 2 && t < lowerEnd;
    const centre = under ? belly.getPoint(t / lowerEnd) : profile.getPoint(t);
    const x = row <= lowerRows ? centre.x : joinX + (row - lowerRows) / upperRows * (-3.08 - joinX);
    const z = row <= lowerRows ? centre.y : roof(x, 0) + 0.108;
    const angle = under ? belly.getTangent(t / lowerEnd)
      : row <= lowerRows ? profile.getTangent(t) : point(1, 0);
    const wrap = under ? 1 - smooth(z, 0.10, 1.55) : 0;
    const blend = smooth(z, 3.6, joinZ);
    const stockWidth = 1.56 + 0.33 * smooth(z, 0.36, 2.92)
      - 0.29 * (1 - smooth(z, 0.36, 0.8)) - 0.12 * smooth(x, -3.5, -3.08);
    const railT = under ? t / lowerEnd : 1;
    const railPoint = under ? rail.getPoint(railT) : centre;
    const width = under ? 1.38 + 0.28 * smooth(railT, 0, 0.23)
      + 0.23 * smooth(railT, 0.56, 1) : stockWidth;
    const pivotBand = smooth(z, 1.88, 2.38) * (1 - smooth(z, 4.9, 5.82));
    const greyEdge = 1 - 0.16 * pivotBand;
    // On the 2.0 the rear shoulders begin rounding closer to the badge.
    // Blend to a quadratic section while keeping the centreline and lip fixed.
    const shoulderBlend = shellVersion === 2
      ? smooth(z, 2.5, 3.6) * (1 - smooth(z, 4.5, 5.5)) : 0;
    for (let col = -2; col <= cols + 2; col++) {
      const across = 2 * THREE.MathUtils.clamp(col, 0, cols) / cols - 1;
      // Keep two mesh strips on the colour boundary as the glossy return narrows.
      const boundary = pivotBand > 0.001 ? greyEdge : 0.999;
      const u = Math.sign(across) * (Math.abs(across) <= 0.85
        ? Math.abs(across) * boundary / 0.85
        : boundary + (Math.abs(across) - 0.85) * (1 - boundary) / 0.15);
      const edge = Math.max(0, -col, col - cols);
      const y = u * (width + (edge === 1 ? 0.028 : edge === 2 ? 0.012 : 0));
      const side = Math.abs(u);
      // Across the sticker the generators are parallel. Begin the
      // transverse radius only on the approach to either tread.
      const flank = Math.max(0, (side - 0.58) / 0.42) ** 2;
      const flatPanel = under ? smooth(z, 0.12, 0.48) * (1 - smooth(z, shoulder.y, 2.9)) : 0;
      const section = 0.11 * u ** 2 + 0.25 * THREE.MathUtils.lerp(u ** 4, u ** 2, shoulderBlend);
      const roll = (1 - blend) * THREE.MathUtils.lerp(section,
        0.36 * flank, flatPanel);
      // Rounded inverted-U relief flows into the lower panel across the parting line.
      const arch = 3.07 - 0.34 * (Math.abs(y) / 0.65) ** 4;
      const recess = 0.065 * (1-smooth(Math.abs(y), 0.59, 0.70)) * (1-smooth(z, arch-0.12, arch));
      const seam = 0.020 * Math.exp(-(((z-2.08)/0.033)**2));
      const pivotRelief = 0.075 * pivotBand * smooth(Math.abs(u), greyEdge, 0.99);
      const relief = under ? smooth(z, 1.70, 2.14) : 1;
      const inset = roll * (1 - 0.90 * wrap) + recess * relief + seam - pivotRelief;
      const weight = under ? THREE.MathUtils.lerp(u ** 4, flank, flatPanel)
        * (1 - smooth(railT, 0.48, 1)) : 0;
      const sx = x + angle.y * inset;
      const sz = z - angle.x * inset + blend * (roof(x, y) - roof(x, 0));
      // Turn the edge inward after lofting; otherwise the two outboard
      // strips collapse back onto each other where the rail has full weight.
      const returnDepth = edge === 1 ? 0.04 : edge === 2 ? 0.16 : 0;
      vertices.push(THREE.MathUtils.lerp(sx, railPoint.x, weight) + angle.y * returnDepth, y,
        THREE.MathUtils.lerp(sz, railPoint.y, weight) - angle.x * returnDepth);
      if (!row || col === -2) continue;
      const stride = cols + 5, k = row * stride + col + 2;
      const indices = Math.abs(across - 1 / cols) > 0.85 && pivotBand > 0.001 ? black : grey;
      indices.push(k, k - stride, k - 1, k - 1, k - stride, k - stride - 1);
    }
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.Float32BufferAttribute(vertices, 3));
  geo.setIndex(grey.concat(black));
  geo.addGroup(0, grey.length, 0);
  geo.addGroup(grey.length, black.length, 1);
  geo.computeVertexNormals();
  const shell = new THREE.Mesh(geo, [
    new THREE.MeshStandardMaterial({color: 0x56575b, roughness: 0.45, side: THREE.DoubleSide}),
    new THREE.MeshStandardMaterial({color: 0x202124, roughness: 0.26, side: THREE.DoubleSide}),
  ]);
  shell.name = "Backpack and rear cover";
  body.add(shell);

  // Small silver outline below the indicator, seen head-on in the owner's rear view.
  const badge = new THREE.Shape();
  badge.moveTo(-0.30, 0.20);
  badge.lineTo(0.15, 0.20);
  badge.quadraticCurveTo(0.16, 0.06, 0.28, -0.05);
  badge.quadraticCurveTo(0.36, -0.23, 0.04, -0.19);
  badge.quadraticCurveTo(-0.11, -0.23, -0.29, -0.17);
  badge.closePath();
  const cutout = new THREE.Path();
  cutout.absellipse(-0.05, 0, 0.16, 0.12, 0, 2 * Math.PI, true);
  badge.holes.push(cutout);
  const mark = new THREE.Mesh(new THREE.ExtrudeGeometry(badge, {depth: 0.009, bevelEnabled: false}),
    new THREE.MeshStandardMaterial({color: 0xb6b5b1, metalness: 0.7, roughness: 0.32}));
  const station = 0.68, centre = profile.getPoint(station), tangent = profile.getTangent(station);
  const up = new THREE.Vector3(tangent.x, 0, tangent.y);
  const right = new THREE.Vector3(0, -1, 0);
  const normal = right.clone().cross(up);
  mark.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(right, up, normal));
  mark.position.set(centre.x, 0, centre.y).addScaledVector(normal, 0.012);
  mark.name = "Rear badge";
  body.add(mark);
  return profile;

}

// A quarter-round return connects the gray roof to the dark throat of each vent.
// The measured dark core is narrower than the outline of its molded opening.
function addVentRims(head, centers, width, length) {
  const positions = [], indices = [];
  const radius = 0.016; // cm, estimated from the half-contrast width
  for (const [cx, cy] of centers) {
    let previous = 0;
    for (let row = 0; row <= 6; row++) {
      const theta = Math.min(row, 5) * Math.PI / 10;
      const inset = radius * Math.sin(theta);
      const height = row === 6 ? 0.018 : 0.064 - radius * (1 - Math.cos(theta));
      const w = width - 2 * inset, h = length - 2 * inset;
      const ring = capsule(cx - w / 2, cy - h / 2, w, h, 0.035 - inset).getPoints(7);
      // The closed shape repeats its first point. Share that vertex at the seam.
      ring.pop();
      const start = positions.length / 3;
      for (const { x, y } of ring) {
        positions.push(x, y, headRoof(x, y) + height);
      }
      if (row) {
        for (let j = 0; j < ring.length; j++) {
          const next = (j + 1) % ring.length;
          indices.push(previous + j, previous + next, start + j,
            previous + next, start + next, start + j);
        }
      }
      previous = start;
    }
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  const rims = new THREE.Mesh(geometry,
    new THREE.MeshStandardMaterial({ color: 0x58595d, roughness: 0.46 }));
  rims.name = "Crown vent returns";
  head.add(rims);
}

export function addCrownAndBackpack(head, body, shellVersion) {
  const crown = cover(head, headRoof);
  const cap = capsule(-2.66, -1.54, 2.16, 3.08, 0.34);
  // Vector 2.0 crown openings, measured in the opposing See3CAM views.
  // Dimensions are cm in the OBJ frame; the first station is a slot centre.
  const [first, pitch, bank, width, length] = shellVersion === 2
    ? [-2.515, 0.208, 0.280, 0.11, 0.36]
    : [-1.885, 0.32, 0.355, 0.13, 0.53];
  const centers = [];
  for (let i = 0; i < 5; i++) {
    for (const y of [-bank, bank]) {
      centers.push([first + i * pitch, y]);
      cap.holes.push(capsule(first + i * pitch - width / 2, y - length / 2,
        width, length, 0.035));
    }
  }
  if (shellVersion === 2) addVentRims(head, centers, width, length);
  const cavity = shellVersion === 2
    ? capsule(first - 0.12, -0.56, 4 * pitch + 0.24, 1.12, 0.07)
    : capsule(-2.08, -0.72, 1.72, 1.44, 0.1);
  crown("Vent cavity", cavity, 0.007, 0.012, 0x101115, 0.85);
  crown("Slotted head cover", cap, -0.012, 0.076, 0x58595d, 0.46);

  // The SDK body has a cutout under the touch assembly. Bridge it with
  // the observed arch, tangent to the intact centreline on either side.
  const backpackRoof = (x, y) => 6.39 - 0.19 * (x + 3.98) ** 2
    - 0.92 * Math.max(0, -5.25 - x) ** 2 - 0.055 * y ** 2 - 0.055 * y ** 4;
  const pack = cover(body, backpackRoof);
  const rear = addRearShell(body, backpackRoof, shellVersion);
  // The centre lens runs down the steeper part of the back. Sample the
  // monotone upper arc once; the cover builder also uses it for its normals.
  const arc = rear.curves[1].getPoints(256);
  const rearRoof = (x, y) => {
    if (x >= arc.at(-1).x) return backpackRoof(x, y);
    let lo = 0, hi = arc.length - 1;
    while (hi - lo > 1) {
      const mid = (lo + hi) >>> 1;
      if (arc[mid].x < x) lo = mid;
      else hi = mid;
    }
    const a = arc[lo], b = arc[hi];
    const z = THREE.MathUtils.lerp(a.y, b.y, (x - a.x) / (b.x - a.x));
    return z - 0.108 + (backpackRoof(x, y) - backpackRoof(x, 0))
      * THREE.MathUtils.smoothstep(z, 3.6, arc.at(-1).y);
  };
  const lensCover = cover(body, rearRoof);

  // One continuous U-shaped electrode. Work around the open rear channel
  // and the two microphone scallops on each flank.
  const gold = new THREE.Shape();
  gold.moveTo(-5.58, -0.47);
  gold.lineTo(-3.93, -0.47);
  gold.bezierCurveTo(-3.38, -0.47, -3.38, 0.47, -3.93, 0.47);
  gold.lineTo(-5.58, 0.47);
  for (const side of [1, -1]) {
    const rear = side === 1;
    gold.quadraticCurveTo(rear ? -5.56 : -3.22, side * 1.20, rear ? -5.26 : -3.62, side * 1.20);
    for (const x of rear ? [-5.20, -3.70] : [-3.70, -5.20]) {
      gold.lineTo(x - side * 0.059, side * 1.20);
      gold.quadraticCurveTo(x, side * 0.996, x + side * 0.059, side * 1.20);
    }
    gold.lineTo(rear ? -3.62 : -5.26, side * 1.20);
    gold.quadraticCurveTo(rear ? -3.22 : -5.56, side * 1.20, rear ? -3.22 : -5.58, side * 0.47);
    if (rear) gold.quadraticCurveTo(-3.15, 0, -3.22, -0.47);
  }
  gold.closePath();
  pack("Gold touch plate", gold, 0.112, 0.073, 0xc6ac6b, 0.32);
  for (const x of [-5.20, -3.70]) {
    for (const side of [1, -1]) {
      const port = new THREE.Shape();
      port.absarc(x, side * 1.163, 0.034, 0, 2 * Math.PI);
      pack("Microphone port", port, 0.109, 0.002, 0x16171a, 0.86);
    }
  }
  lensCover("Indicator channel", capsule(-6.135, -0.405, 2.605, 0.81, 0.39), 0.109, 0.007, 0x48494c, 0.46);
  // The shallow moat exposes the base of the glossy, convex lens.
  const dome = (x, y) => {
    const dx = x - THREE.MathUtils.clamp(x, -5.84, -3.895);
    return 0.14 * Math.sqrt(Math.max(0, 1 - (dx * dx + y * y) / (0.265 ** 2)));
  };
  lensCover("Raised indicator lens", capsule(-6.105, -0.265, 2.475, 0.53, 0.265), 0.119, 0.012, 0x131419, 0.15, dome);
  for (const [i, x] of [-5.08, -4.72, -4.36].entries()) {
    pack("Indicator window " + i, capsule(x, -0.144, 0.265, 0.288, 0.047),
      0.134, 0.006, 0xa3afbb, 0.27, dome);
  }
  pack("Indicator dot", capsule(-3.93, -0.055, 0.11, 0.11, 0.055), 0.134, 0.006, 0x899095, 0.28, dome);
}

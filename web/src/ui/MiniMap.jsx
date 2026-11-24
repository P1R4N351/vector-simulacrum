// Top view of the table: edges, the block, the path driven, the robot..
import { useEffect, useRef } from "react";
import { gameApi, useGame } from "../store.js";

const SIZE = 168;
const PAD = 14;

function draw(ctx, model, t) {
  const v = model.values;
  const scale = (SIZE - 2 * PAD) / Math.max(v.table_length_mm, v.table_width_mm);
  // Screen: robot +x is up, +y is left, so the view matches looking along the start heading.
  const sx = (yMm) => SIZE / 2 - yMm * scale;
  const sy = (xMm) => SIZE / 2 - xMm * scale;
  ctx.clearRect(0, 0, SIZE, SIZE);
  ctx.fillStyle = "#1b1712";
  ctx.strokeStyle = t.cliff_detected ? "#ff4d4d" : "#c8a97a";
  ctx.lineWidth = 1.5;
  const w = v.table_width_mm * scale;
  const h = v.table_length_mm * scale;
  ctx.fillRect(SIZE / 2 - w / 2, SIZE / 2 - h / 2, w, h);
  ctx.strokeRect(SIZE / 2 - w / 2, SIZE / 2 - h / 2, w, h);
  ctx.fillStyle = "#d9dde2";
  for (const [cx, cy, , bx, by] of model.obstacles) ctx.fillRect(sx(cy) - (by * scale) / 2, sy(cx) - (bx * scale) / 2, by * scale, bx * scale);
  const navMap = gameApi.navMap?.();
  if (navMap) {
    const cellPx = 20 * scale;
    ctx.fillStyle = "rgba(61, 245, 140, 0.14)";
    for (const [x, y] of navMap.clear ?? []) ctx.fillRect(sx(y) - cellPx / 2, sy(x) - cellPx / 2, cellPx, cellPx);
    ctx.fillStyle = "rgba(255, 181, 71, 0.55)";
    for (const [x, y] of navMap.obstacles ?? []) ctx.fillRect(sx(y) - cellPx / 2, sy(x) - cellPx / 2, cellPx, cellPx);
    ctx.fillStyle = "rgba(255, 90, 77, 0.72)";
    for (const [x, y] of navMap.cliffs ?? []) ctx.fillRect(sx(y) - cellPx / 2, sy(x) - cellPx / 2, cellPx, cellPx);
    if (navMap.charger) {
      ctx.fillStyle = "#5cb3ff";
      ctx.fillRect(sx(navMap.charger.lipY) - 4, sy(navMap.charger.lipX) - 4, 8, 8);
    }
    if (navMap.cube) {
      ctx.fillStyle = "#38bdf8";
      ctx.fillRect(sx(navMap.cube.y) - 3, sy(navMap.cube.x) - 3, 6, 6);
    }
    if (navMap.frontier) {
      ctx.strokeStyle = "#ffb547";
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.arc(sx(navMap.frontier[1]), sy(navMap.frontier[0]), 3.5, 0, Math.PI * 2);
      ctx.stroke();
    }
  }
  const heading = (t.heading_deg * Math.PI) / 180;
  const nomad = gameApi.nomad?.();
  if (nomad) {
    if (nomad.edges?.length) {
      ctx.strokeStyle = "rgba(168, 85, 247, 0.5)";
      ctx.lineWidth = 1;
      for (const e of nomad.edges) {
        const a = nomad.nodes[e.u], b = nomad.nodes[e.v];
        if (!a || !b) continue;
        ctx.beginPath();
        ctx.moveTo(sx(a.y_mm), sy(a.x_mm));
        ctx.lineTo(sx(b.y_mm), sy(b.x_mm));
        ctx.stroke();
      }
    }
    if (nomad.candidates?.length) {
      const ch = Math.cos(heading), sh = Math.sin(heading);
      for (let ci = 0; ci < nomad.candidates.length; ci++) {
        const selected = ci === nomad.selectedIdx;
        ctx.strokeStyle = selected ? "#c084fc" : "rgba(148, 163, 184, 0.45)";
        ctx.lineWidth = selected ? 1.8 : 1;
        ctx.beginPath();
        ctx.moveTo(sx(t.y_mm), sy(t.x_mm));
        for (const [lx, ly] of nomad.candidates[ci]) {
          const wx = t.x_mm + ch * lx - sh * ly;
          const wy = t.y_mm + sh * lx + ch * ly;
          ctx.lineTo(sx(wy), sy(wx));
        }
        ctx.stroke();
      }
    }
    for (const n of nomad.nodes ?? []) {
      ctx.fillStyle = n.id === nomad.goalNodeId ? "#22d3ee" : (n.id === nomad.currentId ? "#f472b6" : "#a855f7");
      ctx.beginPath();
      ctx.arc(sx(n.y_mm), sy(n.x_mm), n.id === nomad.goalNodeId ? 3 : 2.1, 0, Math.PI * 2);
      ctx.fill();
    }
  }
  const trail = gameApi.trail?.();
  if (trail && trail.count > 1) {
    ctx.strokeStyle = "#1d7a4b";
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let i = 0; i < trail.count; i++) {
      const px = sx(trail.array[3 * i + 1] * 1000);
      const py = sy(trail.array[3 * i] * 1000);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    }
    ctx.stroke();
  }
  const rescue = gameApi.rescue?.();
  if (rescue?.hasPatient) {
    if (rescue.cubePosMm) {
      ctx.fillStyle = "#38bdf8";
      ctx.fillRect(sx(rescue.cubePosMm[1]) - 3, sy(rescue.cubePosMm[0]) - 3, 6, 6);
    }
    if (rescue.patientPosMm) {
      ctx.save();
      ctx.translate(sx(rescue.patientPosMm[1]), sy(rescue.patientPosMm[0]));
      ctx.rotate(-((rescue.patientYawDeg ?? 0) * Math.PI) / 180);
      ctx.fillStyle = rescue.patientUpright ? "#38bdf8" : "#ffb547";
      ctx.beginPath();
      ctx.moveTo(0, -6);
      ctx.lineTo(4.5, 4.5);
      ctx.lineTo(-4.5, 4.5);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
    }
  }
  ctx.save();
  ctx.translate(sx(t.y_mm), sy(t.x_mm));
  ctx.rotate(-heading);
  ctx.fillStyle = t.fallen ? "#6b7280" : "#3df58c";
  ctx.beginPath();
  ctx.moveTo(0, -7);
  ctx.lineTo(5, 5);
  ctx.lineTo(-5, 5);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

export default function MiniMap() {
  const ref = useRef(null);
  const model = useGame((s) => s.model);
  const telemetry = useGame((s) => s.telemetry);
  useEffect(() => {
    const canvas = ref.current;
    if (!canvas || !model || !telemetry) return;
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    if (canvas.width !== SIZE * ratio) {
      canvas.width = SIZE * ratio;
      canvas.height = SIZE * ratio;
    }
    const ctx = canvas.getContext("2d");
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    draw(ctx, model, telemetry);
  }, [model, telemetry]);
  return <canvas ref={ref} className="minimap" style={{ width: SIZE, height: SIZE }} aria-label="Top view of the table" />;
}

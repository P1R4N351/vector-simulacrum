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
  const heading = (t.heading_deg * Math.PI) / 180;
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

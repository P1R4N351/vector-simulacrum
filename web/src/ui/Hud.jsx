// Status, readouts and controls over the 3D view..
import { useEffect, useState } from "react";
import { gameApi, useGame } from "../store.js";
import MiniMap from "./MiniMap.jsx";

// toFixed keeps the sign of a value that rounds to zero ("-0"); drop it.
const fmt = (value, digits = 0) => (Number.isFinite(value) ? value.toFixed(digits).replace(/^-(0(\.0+)?)$/, "$1") : "-");
// What the robot makes of its range reading, by its own rules.
const rangeVerdict = (t) => (t.prox_lift ? "lift in the beam" : t.prox_found ? "object" : t.prox_unobstructed ? "clear" : "not counted");
const pair = (values, digits = 0) => `${fmt(values[0], digits)} / ${fmt(values[1], digits)}`;
const KEYS = [
  ["W A S D / arrows", "drive (Shift: faster)"], ["R F", "head up / down, while held"], ["T G", "lift up / down, while held"],
  ["1 2", "forward / back 100 mm"], ["3 4", "turn left / right 90°"], ["5 6", "head to its top / bottom"],
  ["7 8", "lift to its top / bottom"], ["Space", "stop"],
  ["X", "take control back"], ["O", "cliff reaction on / off"], ["Backspace", "reset"],
  ["C  V  B  Q  P", "follow camera, sensor beams, physics shapes, robot camera, pause"],
];
const RESULT_TEXT = {
  completed: "completed", cancelled_cliff: "cancelled by a cliff", no_control: "refused: no control",
  timeout: "timed out", aborted_off_ground: "aborted: off the ground",
};

function Chip({ tone, children }) {
  return <span className={`chip ${tone}`}>{children}</span>;
}

function StatusChips({ t, level }) {
  const control = t.fallen ? ["bad", "FELL OFF THE TABLE"]
    : t.reacting ? ["warn", "REACTING TO A CLIFF"]
      : t.has_control ? ["ok", "HAS CONTROL"] : ["warn", "NO CONTROL · PRESS X"];
  return (
    <div className="chips">
      <Chip tone={level === "default" ? "ok" : "bad"}>{level === "default" ? "CLIFF REACTION ON" : "CLIFF REACTION OFF"}</Chip>
      <Chip tone={control[0]}>{control[1]}</Chip>
      {t.cliff_detected ? <Chip tone="bad">CLIFF</Chip> : null}
      {t.picked_up ? <Chip tone="warn">PICKED UP</Chip> : null}
      {t.falling ? <Chip tone="bad">FALLING</Chip> : null}
      {t.action_running ? <Chip tone="info">ACTION RUNNING</Chip> : null}
    </div>
  );
}

function CliffPads({ cliff }) {
  const names = ["front left", "front right", "back left", "back right"];
  return (
    <div className="cliffpads" aria-label="Cliff sensors">
      {cliff.map((flag, i) => <span key={names[i]} className={flag ? "pad bad" : "pad"} title={`${names[i]}: ${flag ? "cliff" : "surface"}`} />)}
    </div>
  );
}

function Readouts({ t }) {
  const rows = [
    ["position", `${fmt(t.x_mm)}, ${fmt(t.y_mm)} mm`], ["heading", `${fmt(t.heading_deg, 1)}°`],
    ["to nearest edge", t.edge_margin_mm < 0 ? "off the table" : `${fmt(t.edge_margin_mm)} mm`], ["wheels commanded", `${pair(t.wheel_cmd)} mm/s`],
    ["wheels measured", `${pair(t.wheel_meas)} mm/s`], ["motor power", pair(t.motor_power, 2)],
    ["front wheels (motor)", `${pair(t.front_rpm)} rpm`], ["rear wheels (idler)", `${pair(t.rear_rpm)} rpm`],
    ["battery", `${fmt(t.battery_v, 2)} V`], ["tread heat", pair(t.tread_heat, 3)],
    ["head", `${fmt(t.head_deg, 1)}°`], ["lift", `${fmt(t.lift_mm, 1)} mm`],
    ["range sensor", `${fmt(t.prox_mm)} mm \u00b7 ${rangeVerdict(t)}`], ["range signal", fmt(t.prox_quality, 3)],
    ["robot time", `${fmt(t.time_s, 1)} s`], ["speed", `${fmt(t.rate, 2)}× real time`],
  ];
  return (
    <dl className="readouts">
      {rows.map(([label, value]) => (
        <div key={label}><dt>{label}</dt><dd>{value}</dd></div>
      ))}
    </dl>
  );
}

function Notice() {
  const notice = useGame((s) => s.notice);
  const [shown, setShown] = useState(null);
  useEffect(() => {
    setShown(notice);
    if (notice === null) return undefined;
    const timer = setTimeout(() => setShown(null), 2600);
    return () => clearTimeout(timer);
  }, [notice]);
  return shown ? <div className="notice" role="status">{shown.text}</div> : null;
}

function TouchPad() {
  const hold = (fwd, turn) => ({
    onPointerDown: (e) => { e.currentTarget.setPointerCapture(e.pointerId); gameApi.setPad?.(fwd, turn); },
    onPointerUp: () => gameApi.setPad?.(0, 0),
    onPointerCancel: () => gameApi.setPad?.(0, 0),
  });
  return (
    <div className="touchpad" aria-label="Drive">
      <button type="button" className="up" aria-label="Forward" {...hold(1, 0)}>{"▲"}</button>
      <button type="button" className="left" aria-label="Turn left" {...hold(0, 1)}>{"◀"}</button>
      <button type="button" className="right" aria-label="Turn right" {...hold(0, -1)}>{"▶"}</button>
      <button type="button" className="down" aria-label="Back" {...hold(-1, 0)}>{"▼"}</button>
    </div>
  );
}

// The frame round the robot's own camera view. The picture itself is drawn on the canvas behind it.
function CameraView({ camera }) {
  return (
    <figure className="camview">
      <div className="camview-window" style={{ aspectRatio: `${camera.width_px} / ${camera.height_px}` }} />
      <figcaption>
        Robot camera {"·"} {camera.width_px} {"×"} {camera.height_px} {"·"} {fmt(camera.fov_horizontal_deg, 1)}{"°"} {"×"}{" "}
        {fmt(camera.fov_vertical_deg, 1)}{"°"} {"·"} {fmt(camera.pitch_down_deg)}{"°"} down
      </figcaption>
    </figure>
  );
}

function About({ model, robotLook }) {
  const c = model.provenance_counts;
  const total = Object.values(c).reduce((a, b) => a + b, 0);
  return (
    <details className="about">
      <summary>What this is</summary>
      <p>
        The same MuJoCo model and firmware control law as the Python simulacrum, running in this page.
        Of its {total} parameters, {c.measured} are measured on a recording of one real robot, {c.fitted} are
        fitted to it, {c.manual} come from the manual, {c.sdk} from vendor or household software, {c.assumed} are
        still assumed and {c.chosen} are scene and solver settings.
      </p>
      <p>
        {robotLook === "sdk"
          ? "The robot drawn is the 3D model from the Vector Python SDK's viewer. The physics underneath is boxes and cylinders (press B to see them). Each wheel turns by the physics model's angle for it: the front wheels are the motor-driven ones and the rear wheels are turned by the track. The tread cleats follow the belt. When installed, the three replacement-part STLs supply the outer wheels and lift fork."
          : "The robot is drawn as the physics model's own boxes and cylinders. The tread bands and the face are added for the eye."}
        {" "}The pose shown is the simulator's true pose.
      </p>
      <p>
        The robot camera view uses one real Vector's factory calibration ({fmt(model.camera.fov_horizontal_deg, 1)}{"°"} by{" "}
        {fmt(model.camera.fov_vertical_deg, 1)}{"°"}; the manual says {model.camera.documents.manual_deg[0]}{"°"} by{" "}
        {model.camera.documents.manual_deg[1]}{"°"}), at the firmware's mounting point, tilted {fmt(model.camera.pitch_down_deg)}{"°"} down as
        the robot's engine has it. It has no lens distortion, exposure or noise.
      </p>
      <p className="hash">table {model.source.table_sha256.slice(0, 12)} · model {model.source.xml_sha256.slice(0, 12)}</p>
    </details>
  );
}

export default function Hud() {
  const t = useGame((s) => s.telemetry);
  const level = useGame((s) => s.level);
  const paused = useGame((s) => s.paused);
  const chase = useGame((s) => s.chase);
  const model = useGame((s) => s.model);
  const lastAction = useGame((s) => s.lastAction);
  const padConnected = useGame((s) => s.padConnected);
  const robotLook = useGame((s) => s.robotLook);
  const shapes = useGame((s) => s.shapes);
  const camView = useGame((s) => s.camView);
  if (!t || !model) return null;
  const press = (name) => () => gameApi.press?.(name);
  return (
    <div className="hud">
      <header className="panel top-left">
        <h1>Vector simulacrum</h1>
        <StatusChips t={t} level={level} />
        {paused ? <div className="paused">PAUSED</div> : null}
      </header>
      <section className="panel right">
        <MiniMap />
        <CliffPads cliff={t.cliff} />
        <Readouts t={t} />
        {lastAction ? (
          <p className="last-action">
            Last action: {RESULT_TEXT[lastAction.result] ?? lastAction.result}, moved {fmt(lastAction.moved_mm, 1)} mm,
            turned {fmt(lastAction.turned_deg, 1)}{"°"} in {fmt(lastAction.ticks * 0.005, 2)} s
          </p>
        ) : null}
        <About model={model} robotLook={robotLook} />
      </section>
      <section className="panel bottom-left">
        <div className="buttons">
          <button type="button" onClick={press("forward100")}>Forward 100 mm</button>
          <button type="button" onClick={press("back100")}>Back 100 mm</button>
          <button type="button" onClick={press("turnLeft90")}>Turn left 90{"°"}</button>
          <button type="button" onClick={press("turnRight90")}>Turn right 90{"°"}</button>
          <button type="button" onClick={press("stop")}>Stop</button>
        </div>
        <div className="buttons">
          <button type="button" onClick={press("headUp")}>Head up</button>
          <button type="button" onClick={press("headLevel")}>Head level</button>
          <button type="button" onClick={press("headDown")}>Head down</button>
          <button type="button" onClick={press("liftUp")}>Lift up</button>
          <button type="button" onClick={press("liftDown")}>Lift down</button>
        </div>
        <div className="buttons">
          <button type="button" onClick={press("control")}>Take control</button>
          <button type="button" className={level === "default" ? "" : "danger"} onClick={press("level")}>
            {level === "default" ? "Turn cliff reaction off" : "Turn cliff reaction on"}
          </button>
          <button type="button" onClick={press("chase")}>{chase ? "Free camera" : "Follow robot"}</button>
          {robotLook === "sdk" ? (
            <button type="button" onClick={press("shapes")}>{shapes ? "Hide physics shapes" : "Show physics shapes"}</button>
          ) : null}
          <button type="button" onClick={press("camera")}>{camView ? "Hide robot camera" : "Show robot camera"}</button>
          <button type="button" onClick={press("pause")}>{paused ? "Resume" : "Pause"}</button>
          <button type="button" onClick={press("reset")}>Reset</button>
        </div>
        <ul className="keys">
          {KEYS.map(([key, what]) => <li key={key}><kbd>{key}</kbd><span>{what}</span></li>)}
          {padConnected ? <li><kbd>Gamepad</kbd><span>left stick drives, right stick tilts the head</span></li> : null}
        </ul>
      </section>
      {camView ? <CameraView camera={model.camera} /> : null}
      <TouchPad />
      <Notice />
    </div>
  );
}

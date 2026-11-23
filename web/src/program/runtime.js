// The program's end: the robot object a program written in the page sees, and running it.
//
// A program is the body of an async JavaScript function with three names in scope:
//   robot   calls named as in the Vector Python SDK, each returning a promise
//   print   writes a line under the editor
//   wait    the same as robot.wait
// It runs in a Web Worker in the page (game/program.worker.js), so Stop can end any
// program, a loop that never awaits included. Here nothing touches the browser: the
// tests run programs the same way, in-thread, against ProgramHost.

const AsyncFunction = Object.getPrototypeOf(async function () {}).constructor;
const SOURCE_NAME = "program.js";
const PARAMS = ["robot", "print", "wait"];
const PRINT_LIMIT = 10000; // lines one run may print; the rest are dropped, and the page says so
const PRINT_WIDTH = 2000; // characters kept of one printed line

// What the robot object offers, for the page's reference list. Units are the SDK's.
export const ROBOT_CALLS = [
  ["await robot.drive_straight(distance_mm, speed_mmps)", "drive along the heading; negative distance is backwards"],
  ["await robot.turn_in_place(angle_deg, speed_dps?)", "turn on the spot; positive is to the left"],
  ["await robot.set_head_angle(angle_deg)", "tilt the head (-22 to 45 degrees)"],
  ["await robot.set_lift_height(height)", "move the lift: 0.0 is the bottom, 1.0 the top"],
  ["await robot.set_wheel_motors(left_mmps, right_mmps)", "set tread speeds and return at once"],
  ["await robot.stop_all_motors()", "treads to zero, head and lift held"],
  ["await robot.wait(seconds)", "let robot time pass; also plain wait(seconds)"],
  ["await robot.request_control()", "take control back after a cliff reaction"],
  ["robot.state", "the last reported state: x_mm, y_mm, pose_angle_deg, head_angle_deg, lift_height_mm, prox_distance_mm, cliff_detected, has_control, now_s, ..."],
  ["print(...values)", "write a line below"],
];
// Each action resolves to { result, moved_mm, turned_deg, time_s }; result is "completed",
// "cancelled_cliff", "no_control", "timeout" or "aborted_off_ground".

const METHODS = ["drive_straight", "turn_in_place", "set_head_angle", "set_lift_height", "set_wheel_motors",
  "stop_all_motors", "request_control", "wait"];

// An error the robot end refused a call with.
export function robotError(message) {
  const err = new Error(String(message));
  err.name = "RobotError";
  return err;
}

// The robot object. call(name, args) returns a promise for the call's value; state() the latest state.
// A refusal arrives from elsewhere, so the stack of the call itself is kept with it: it holds the
// program's line.
export function makeRobot(call, state) {
  const robot = {};
  for (const name of METHODS) {
    robot[name] = (...args) => {
      const site = new Error();
      const bad = args.findIndex((a) => a !== null && a !== undefined && typeof a !== "number");
      if (bad >= 0) return Promise.reject(new TypeError(`robot.${name}: argument ${bad + 1} must be a number`));
      return call(name, args.map((a) => (a === undefined ? null : a))).catch((err) => {
        if (err instanceof Error) err.callStack = site.stack;
        throw err;
      });
    };
  }
  Object.defineProperty(robot, "state", { get: () => state(), enumerable: true });
  return Object.freeze(robot);
}

function show(value) {
  if (typeof value === "string") return value;
  if (value instanceof Error) return `${value.name}: ${value.message}`;
  if (value === undefined || typeof value === "function" || typeof value === "symbol") return String(value);
  try {
    return JSON.stringify(value) ?? String(value);
  } catch {
    return String(value);
  }
}

export function formatPrint(args) {
  const text = args.map(show).join(" ");
  return text.length > PRINT_WIDTH ? `${text.slice(0, PRINT_WIDTH)}…` : text;
}

function wrap(source) {
  return `"use strict";\n${source}\n//# sourceURL=${SOURCE_NAME}`;
}

// The line in the program an error was raised at, or null when the engine does not say.
const LINE_RE = new RegExp(`${SOURCE_NAME.replace(".", "\\.")}:(\\d+):\\d+`);
function stackLine(err) {
  for (const stack of [err?.stack, err?.callStack]) {
    const match = typeof stack === "string" ? LINE_RE.exec(stack) : null;
    if (match !== null) return Number(match[1]);
  }
  return null;
}

// How many lines the engine puts before the program's first line. Measured, not assumed:
// engines wrap a constructed function differently.
let headerLines;
async function lineOffset() {
  if (headerLines === undefined) {
    const made = await new AsyncFunction(...PARAMS, wrap("return new Error();"))();
    const line = stackLine(made);
    headerLines = line === null ? null : line - 1;
  }
  return headerLines;
}

export async function errorLine(err) {
  const line = stackLine(err);
  const offset = await lineOffset();
  if (line === null || offset === null || line - offset < 1) return null;
  return line - offset;
}

// Run a program. io: { call(name, args) -> Promise, print(text), state() -> object }.
// Resolves (never rejects) to { ok: true } or { ok: false, error, line } with line null when unknown.
export async function runProgram(source, io) {
  if (typeof source !== "string") return { ok: false, error: "the program must be text", line: null };
  let printed = 0;
  const print = (...args) => {
    printed += 1;
    if (printed <= PRINT_LIMIT) io.print(formatPrint(args));
    else if (printed === PRINT_LIMIT + 1) io.print(`(print limit of ${PRINT_LIMIT} lines reached; later lines dropped)`);
  };
  const robot = makeRobot(io.call, io.state);
  let body;
  try {
    body = new AsyncFunction(...PARAMS, wrap(source));
  } catch (err) {
    return { ok: false, error: `${err.name}: ${err.message}`, line: await errorLine(err) };
  }
  try {
    await body(robot, print, robot.wait);
    return { ok: true };
  } catch (err) {
    const name = err?.name ?? "Error";
    return { ok: false, error: err instanceof Error ? `${name}: ${err.message}` : `thrown: ${show(err)}`, line: await errorLine(err) };
  }
}

// A call function for runProgram that talks to a ProgramHost in the same thread (the tests).
export function directCall(host) {
  let next = 1;
  return (name, args) => new Promise((resolve, reject) => {
    host.call({ id: next++, name, args }, (reply) => (reply.ok ? resolve(reply.value) : reject(robotError(reply.error))));
  });
}

// The worker's side of the message protocol, kept here so the tests can run it without a worker.
//   page to worker:  { type: "run", source, state }  { type: "reply", id, ok, value | error, state }
//   worker to page:  { type: "call", id, name, args }  { type: "print", text }  { type: "done", ok, error, line }
// post(message) sends to the page. Returns the handler for messages from the page.
export function programEndpoint(post) {
  let state = null;
  let nextId = 1;
  let started = false;
  const waiting = new Map(); // call id -> { resolve, reject }
  const call = (name, args) => new Promise((resolve, reject) => {
    const id = nextId++;
    waiting.set(id, { resolve, reject });
    post({ type: "call", id, name, args });
  });
  const onReply = (message) => {
    const entry = waiting.get(message.id);
    if (entry === undefined) return; // an answer to a call already given up on
    waiting.delete(message.id);
    if (message.state) state = message.state;
    if (message.ok) entry.resolve(message.value);
    else entry.reject(robotError(message.error));
  };
  const onRun = async (message) => {
    if (started) return; // one program per endpoint; the page starts a new worker for the next run
    started = true;
    state = message.state;
    const outcome = await runProgram(message.source, { call, print: (text) => post({ type: "print", text }), state: () => state });
    post({ type: "done", ...outcome });
  };
  return (message) => {
    if (message?.type === "reply") onReply(message);
    else if (message?.type === "run") onRun(message);
  };
}

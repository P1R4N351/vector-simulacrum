// The page's side of a program run: starts a worker for it, answers the worker's robot
// calls through a ProgramHost, and keeps what the program prints for the panel.
//
// The frame loop calls advance(ticks) instead of ticking the robot itself while a
// program runs; see host.js for why robot time then moves only while the program waits.
// Nothing here touches the DOM: the worker comes from makeWorker, so the tests can pass
// one that runs the program in the same thread.
import { ProgramHost, RobotCallError } from "./host.js";

const LOG_LIMIT = 500; // lines kept for the panel; older ones are dropped
const QUIET_HINT_MS = 3000; // a running program silent this long gets a hint in the log

// clock() is milliseconds, for the quiet-program hint only; robot time is the simulator's.
export function createProgramRunner(sim, makeWorker, clock = () => performance.now()) {
  let host = null;
  let worker = null;
  let status = "idle"; // idle | running | finished | failed | stopped
  let errorLine = null;
  let startTick = 0;
  let heardAt = 0;
  let hinted = false;
  let log = [];
  let version = 0;

  const changed = () => {
    version += 1;
  };
  const note = (kind, text) => {
    log.push({ kind, text: String(text) });
    if (log.length > LOG_LIMIT) log = log.slice(log.length - LOG_LIMIT);
    changed();
  };
  const robotSeconds = () => ((sim.tickCount - startTick) * sim.dt).toFixed(2);

  function end(finalStatus, reason) {
    if (status !== "running") return;
    host.finish(reason);
    worker.terminate();
    worker = null;
    status = finalStatus;
    changed();
  }

  function onCall(message) {
    const ok = Number.isInteger(message.id) && typeof message.name === "string" && Array.isArray(message.args);
    if (!ok) throw new RobotCallError("the program sent a call the page cannot read");
    host.call(message, (reply) => worker?.postMessage({ type: "reply", ...reply }));
  }

  function onDone(message) {
    if (message.ok === true) {
      note("info", `Finished after ${robotSeconds()} s of robot time.`);
      end("finished", "the program has ended");
      return;
    }
    errorLine = Number.isInteger(message.line) ? message.line : null;
    note("error", `${errorLine === null ? "" : `Line ${errorLine}: `}${message.error ?? "the program failed"}`);
    end("failed", "the program has ended");
  }

  function onMessage(message) {
    if (status !== "running") return;
    heardAt = clock();
    hinted = false;
    try {
      if (message?.type === "call") onCall(message);
      else if (message?.type === "print") note("print", message.text);
      else if (message?.type === "done") onDone(message);
    } catch (err) {
      note("error", `Stopped: ${err?.message ?? err}`);
      end("failed", "the page could not answer a call");
    }
  }

  return {
    get running() {
      return status === "running";
    },
    // The call the program is waiting on (null while it computes, or when nothing runs).
    get waitingOn() {
      return status === "running" ? host.waitingOn : null;
    },
    get version() {
      return version;
    },
    view() {
      return { status, errorLine, log: log.slice(), waitingOn: this.waitingOn };
    },
    run(source) {
      if (status === "running") end("stopped", "a new run replaced the program");
      host = new ProgramHost(sim);
      worker = makeWorker();
      status = "running";
      errorLine = null;
      startTick = sim.tickCount;
      heardAt = clock();
      hinted = false;
      log = [];
      note("info", `Started at ${(sim.tickCount * sim.dt).toFixed(2)} s of robot time.`);
      worker.onmessage = (event) => onMessage(event.data);
      worker.onerror = (event) => {
        event.preventDefault?.();
        note("error", `The program's worker failed: ${event.message ?? "no detail given"}`);
        end("failed", "the worker failed");
      };
      worker.postMessage({ type: "run", source: String(source), state: host.snapshot() });
    },
    stop() {
      if (status !== "running") return;
      note("info", `Stopped after ${robotSeconds()} s of robot time.`);
      end("stopped", "the program was stopped");
    },
    // Advance the pending call by at most ticks; returns the ticks run.
    advance(ticks) {
      if (status !== "running") return 0;
      const ran = host.advance(ticks);
      const now = clock();
      if (ran > 0) heardAt = now;
      if (host.waitingOn === null && !hinted && now - heardAt > QUIET_HINT_MS) {
        hinted = true;
        note("info", "No robot call for 3 s. Robot time stands still until the program calls the robot; Stop ends it.");
      }
      return ran;
    },
    clearLog() {
      log = [];
      changed();
    },
  };
}

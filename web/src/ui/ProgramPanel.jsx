// The program editor: write a short robot program, run it against the simulated robot, read what it prints.
import { useEffect, useRef, useState } from "react";
import { EXAMPLES } from "../program/examples.js";
import { ROBOT_CALLS } from "../program/runtime.js";
import { gameApi, useGame } from "../store.js";

const SAVED_KEY = "vector-simulacrum.program";
const INDENT = "  ";

// The editor's text is kept in this browser between visits. Storage can be absent or refuse
// (a private window, blocked site data): the page then starts from the first example.
function loadSaved() {
  try {
    return window.localStorage.getItem(SAVED_KEY);
  } catch {
    return null;
  }
}

function save(text) {
  try {
    window.localStorage.setItem(SAVED_KEY, text);
  } catch {
    // not kept: storage refused
  }
}

function statusText(program) {
  if (program.status === "running") {
    return program.waitingOn === null ? "Running · computing (robot time waits)" : `Running · ${program.waitingOn}`;
  }
  if (program.status === "failed") return program.errorLine === null ? "Failed" : `Failed at line ${program.errorLine}`;
  return { idle: "Ready", finished: "Finished", stopped: "Stopped" }[program.status] ?? program.status;
}

function Editor({ source, setSource, errorLine, onRun }) {
  const gutterRef = useRef(null);
  const lines = source.split("\n").length;
  const onKeyDown = (event) => {
    const el = event.currentTarget;
    if ((event.metaKey || event.ctrlKey) && event.key === "Enter") {
      event.preventDefault();
      onRun();
    } else if (event.key === "Escape") {
      el.blur();
    } else if (event.key === "Tab" && !event.shiftKey && !event.altKey) {
      event.preventDefault();
      const [from, to] = [el.selectionStart, el.selectionEnd];
      setSource(source.slice(0, from) + INDENT + source.slice(to));
      requestAnimationFrame(() => {
        el.selectionStart = el.selectionEnd = from + INDENT.length;
      });
    }
  };
  return (
    <div className="editor">
      <pre className="gutter" ref={gutterRef} aria-hidden="true">
        {Array.from({ length: lines }, (_, i) => (
          <span key={i} className={i + 1 === errorLine ? "bad" : undefined}>{i + 1}{"\n"}</span>
        ))}
      </pre>
      <textarea
        value={source} spellCheck={false} autoCapitalize="off" autoComplete="off" autoCorrect="off"
        aria-label="Robot program (JavaScript)" onChange={(e) => setSource(e.target.value)} onKeyDown={onKeyDown}
        onScroll={(e) => {
          if (gutterRef.current) gutterRef.current.scrollTop = e.currentTarget.scrollTop;
        }}
      />
    </div>
  );
}

export default function ProgramPanel() {
  const program = useGame((s) => s.program);
  const [source, setSource] = useState(() => loadSaved() ?? EXAMPLES[0].source);
  const logRef = useRef(null);
  const running = program.status === "running";
  useEffect(() => save(source), [source]);
  useEffect(() => {
    const el = logRef.current;
    if (el) el.scrollTop = el.scrollHeight;
  }, [program.log]);
  const run = () => gameApi.runProgram?.(source);
  const pick = (event) => {
    const example = EXAMPLES.find((e) => e.id === event.target.value);
    event.target.value = "";
    if (example === undefined) return;
    const edited = source.trim() !== "" && !EXAMPLES.some((e) => e.source === source);
    if (edited && !window.confirm(`Replace the program in the editor with "${example.title}"?`)) return;
    setSource(example.source);
  };
  return (
    <div className="program">
      <div className="program-bar">
        <button type="button" className="run" onClick={run} title="Ctrl+Enter or ⌘+Enter in the editor">
          {running ? "Run again" : "Run"}
        </button>
        <button type="button" className="danger" onClick={() => gameApi.stopProgram?.()} disabled={!running}>Stop</button>
        <select aria-label="Load an example program" defaultValue="" onChange={pick}>
          <option value="" disabled>Examples…</option>
          {EXAMPLES.map((e) => <option key={e.id} value={e.id}>{e.title}</option>)}
        </select>
        <span className={`program-status ${program.status}`} role="status">{statusText(program)}</span>
      </div>
      <Editor source={source} setSource={setSource} errorLine={program.errorLine} onRun={run} />
      <ol className="program-log" ref={logRef} aria-label="Program output">
        {program.log.length === 0 ? <li className="info">Output appears here. Ctrl+Enter runs; Tab indents; Esc leaves the editor.</li> : null}
        {program.log.map((line, i) => <li key={i} className={line.kind}>{line.text}</li>)}
      </ol>
      <details className="program-help">
        <summary>Robot calls</summary>
        <p>
          JavaScript, run in this page. Robot time moves only while the program awaits an action or a wait, so a
          run comes out the same every time. When the program ends the treads stop.
        </p>
        <dl>
          {ROBOT_CALLS.map(([call, what]) => (
            <div key={call}><dt><code>{call}</code></dt><dd>{what}</dd></div>
          ))}
        </dl>
        <p>Each action resolves to {"{ result, moved_mm, turned_deg, time_s }"}.</p>
      </details>
      <button type="button" className="clear" onClick={() => gameApi.clearProgramLog?.()}>Clear output</button>
    </div>
  );
}

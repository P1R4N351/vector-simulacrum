// Loading progress, and the error if loading or the simulation stops..
import { useGame } from "../store.js";

export default function BootOverlay() {
  const log = useGame((s) => s.bootLog);
  const error = useGame((s) => s.bootError);
  return (
    <div className="boot" role="status">
      <div className="boot-card">
        <div className="boot-title">VECTOR SIMULACRUM</div>
        <ul className="boot-log">
          {log.map((entry, index) => (
            <li key={`${index}-${entry.label}`}>
              <span>{entry.label}</span>
              <span className={entry.status === "FAILED" ? "bad" : entry.status ? "ok" : "wait"}>{entry.status ?? "..."}</span>
            </li>
          ))}
        </ul>
        {error !== null ? (
          <div className="boot-halt">
            <strong>STOPPED</strong>
            <p>{error}</p>
            <button type="button" onClick={() => window.location.reload()}>Reload</button>
          </div>
        ) : null}
      </div>
    </div>
  );
}

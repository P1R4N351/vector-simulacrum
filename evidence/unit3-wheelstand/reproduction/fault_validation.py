"""Break one thing per arm in a fresh copy of the repository and run tests/test_wheelstand_replay.py there.

    WHEELSTAND_FORK_FIXTURE=fixture.xml python evidence/unit3-wheelstand/reproduction/fault_validation.py

Every arm except the unchanged copy and the neutral one must turn the module red. A needle that does not
occur exactly once, or a replacement that leaves the file unchanged, is reported as SETUP-ERROR, not as a
result about the test.
"""
import concurrent.futures as cf
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ARMS = [
    ("unchanged copy", None, None, None),
    ("comment added (neutral)", "simulacrum/mjcf.py", "def _tip_equality_xml(", "# neutral\ndef _tip_equality_xml("),
    ("tip supports never driven", "simulacrum/mjcf.py",
     '    return flag == 1 and v["tread_contact_inset_mm"] > 0.0', "    return False"),
    ("tip-support gear 0", "simulacrum/mjcf.py",
     "polycoef=\"0 {gear} 0 0 0\" {stiff}/>\\n')\n    return out", "polycoef=\"0 0 0 0 0\" {stiff}/>\\n')\n    return out"),
    ("reference lift cap 30 mN m", "tests/test_wheelstand_replay.py",
     'REFERENCE = dict(driven_tips=1, lift="servo:60")', 'REFERENCE = dict(driven_tips=1, lift="servo:30")'),
    ("random start offset", "evidence/unit3-wheelstand/reproduction/replay.py",
     "        d.qpos[cube:cube + 2] = tr.cube_xy * 1e-3\n",
     "        d.qpos[cube:cube + 2] = tr.cube_xy * 1e-3 + np.random.default_rng().normal(0, 1e-7, 2)\n"),
]


def arm(name, rel, old, new):
    with tempfile.TemporaryDirectory() as tmp:
        dst = Path(tmp) / "repo"
        shutil.copytree(ROOT, dst, ignore=shutil.ignore_patterns(".git", "runs", "*.mp4", "__pycache__", "web"))
        if rel:
            f = dst / rel
            text = f.read_text()
            if text.count(old) != 1 or old == new:
                return name, f"SETUP-ERROR: needle occurs {text.count(old)} times"
            f.write_text(text.replace(old, new))
            if f.read_text() == text:
                return name, "SETUP-ERROR: file unchanged"
        r = subprocess.run([sys.executable, "-m", "unittest", "-v", "test_wheelstand_replay"], cwd=dst / "tests",
                           env=dict(os.environ), capture_output=True, text=True, timeout=3600)
        verdict = [line for line in r.stderr.splitlines() if " ... " in line or line.startswith(("OK", "FAILED"))]
        tests = [v.split(" (")[0] + ": " + v.split(" ... ")[-1] for v in verdict if " ... " in v]
        totals = [v for v in verdict if " ... " not in v]
        return name, f"exit {r.returncode}; " + "; ".join(tests + totals)


def main():
    assert os.environ.get("WHEELSTAND_FORK_FIXTURE"), "set WHEELSTAND_FORK_FIXTURE"
    with cf.ProcessPoolExecutor(len(ARMS)) as ex:
        futures = [ex.submit(arm, *a) for a in ARMS]
        for f in futures:
            name, out = f.result()
            print(f"{name}: {out}", flush=True)


if __name__ == "__main__":
    main()

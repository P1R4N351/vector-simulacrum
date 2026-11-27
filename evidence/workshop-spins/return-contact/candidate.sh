#!/usr/bin/env bash
# Run inside the evidence directory, using a temporary sibling of the sim modules.
set -eu
cd "$(dirname "$0")"
variant=$(mktemp -d ../../../web/src/sim-trial-XXXXXX)
runner=$(mktemp ./candidate-runner-XXXXXX)
runner_js="$runner.mjs"
trap 'rm -rf "$variant"; rm -f "$runner" "$runner_js"' EXIT
cp -R ../../../web/src/sim/. "$variant/"
python3 - "$variant" "$runner_js" "${1:-70}" <<'PY'
import sys
from pathlib import Path
variant, script = map(Path, sys.argv[1:3])
deadline = int(sys.argv[3])
assert deadline in (70, 140)
source = variant / 'autonomy.js'
old = '[-102, -112, -96, -122, -106]'
text = source.read_text()
assert text.count(old) == 1
source.write_text(text.replace(old, '[-30, -30, -30, -30, -30]').replace('Math.round(70 / this.sim.dt)', f'Math.round({deadline} / this.sim.dt)'))
text = Path('replay.mjs').read_text().replace('/src/sim/', '/src/'+variant.name+'/')
assert "throw Error('Dock failed')" in text
text = text.replace("throw Error('Dock failed')", "console.log(sim.lastAction)")
script.write_text(text.replace("'sim.csv'", repr("candidate.csv" if deadline == 70 else "extended.csv")).replace('14000','30000'))
PY
node "$runner_js"

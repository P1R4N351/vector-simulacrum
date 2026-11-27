#!/usr/bin/env bash
set -eu
cd "$(dirname "$0")"
variant=$(mktemp -d ../../../web/src/sim-load-XXXXXX)
runner=$(mktemp ./roller-runner-XXXXXX)
trap 'rm -rf "$variant"; rm -f "$runner" "$runner.mjs"' EXIT
cp -R ../../../web/src/sim/. "$variant/"
python3 - "$variant" "$runner.mjs" <<'PY'
import sys
from pathlib import Path
variant, runner = map(Path,sys.argv[1:])
source = variant/'autonomy.js'
old = '[-102, -112, -96, -122, -106]'
text = source.read_text()
assert text.count(old) == 1
source.write_text(text.replace(old, '[-30, -30, -30, -30, -30]'))
runner.write_text(Path('roller-return.mjs').read_text().replace('/src/sim-dock-trial/', '/src/'+variant.name+'/'))
PY
node "$runner.mjs" passive
node "$runner.mjs" driven
node "$runner.mjs" driven 60
node "$runner.mjs" driven -60

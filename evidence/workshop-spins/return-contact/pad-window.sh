#!/usr/bin/env bash
set -eu
cd "$(dirname "$0")"
variant=$(mktemp -d ../../../web/src/sim-pad-XXXXXX)
runner=$(mktemp ./pad-runner-XXXXXX)
trap 'rm -rf "$variant"; rm -f "$runner" "$runner.mjs"' EXIT
cp -R ../../../web/src/sim/. "$variant/"
python3 - "$variant" "$runner.mjs" "$1" "${2:-raw}" <<'PY'
import sys
from pathlib import Path
variant,runner=map(Path,sys.argv[1:3])
width=int(sys.argv[3])
assert width in [3,6,9]
for name,old,new in [('autonomy','[-102, -112, -96, -122, -106]','[-30, -30, -30, -30, -30]'),
                     ('robot','Math.abs(project(delta, 0)) <= 0.009','Math.abs(project(delta, 0)) <= '+str(width/1000))]:
    p=variant/(name+'.js')
    s=p.read_text()
    assert s.count(old)==1
    p.write_text(s.replace(old,new))
if sys.argv[4]=='filtered':
    p=variant/'robot.js'
    old='if (this._dockBody >= 0) this.chargerContact = this._padsTouchDock();'
    new="""if (this._dockBody >= 0) {
      const raw = this._padsTouchDock();
      this._previousPad ??= true;
      this._samePadTicks ??= 5;
      if (raw !== this._previousPad) this._samePadTicks = 0;
      else if (this._samePadTicks < 5) this._samePadTicks++;
      else this.chargerContact = raw;
      this._previousPad = raw;
    }"""
    text=p.read_text()
    assert text.count(old)==1
    p.write_text(text.replace(old,new))
runner.write_text(Path('pad-window.mjs').read_text().replace('/src/sim/', '/src/'+variant.name+'/'))
PY
node "$runner.mjs" "$1" "${2:-raw}"

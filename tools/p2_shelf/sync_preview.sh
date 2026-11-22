#!/bin/sh
# Refresh a running local web preview from the daughter pod's atomic export.
set -eu
dir=$(cd "$(dirname "$0")/../../web/public/robot/vector2" && pwd)
tmp=$(mktemp "$dir/.shelf-live.XXXXXX")
trap 'rm -f "$tmp"' EXIT
while :; do
    if ssh -o BatchMode=yes -o ConnectTimeout=8 web-host \
        'ssh -o BatchMode=yes -o ConnectTimeout=8 pi@vector-pod cat /var/lib/vector-maps/p2-shelf/shelf-navmap.json' > "$tmp" &&
       python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); assert d["frame"]=="seated_robot" and d["revisions"]>0' "$tmp"; then
        mv "$tmp" "$dir/shelf-navmap-live.json"
    fi
    sleep 60
done

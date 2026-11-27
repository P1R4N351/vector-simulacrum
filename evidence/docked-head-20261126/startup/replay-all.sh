#!/bin/sh
set -eu
for i in 1 2 3 4; do
  node ../replay.mjs "startup/session-$i.csv" "startup/canonical-$i.csv"
done

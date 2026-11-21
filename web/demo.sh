#!/usr/bin/env bash
# From a fresh clone to the simulator running in a browser, in one command.
# Authored-by: CPCS.
#
#   ./demo.sh                  check, test, build, then serve on http://127.0.0.1:5183
#   ./demo.sh --port 8080      serve on another port
#   ./demo.sh --build-only     stop after the build (the bundle is in dist/)
#   ./demo.sh --no-sdk-model   do not fetch the SDK's robot model; draw the physics shapes
#
# Needs node 20 or later with npm, and python3. Nothing here talks to a robot.
# Exit: 0 served or built, 2 bad arguments, 3 a needed program is missing,
# 5 install or build failed, 7 the drift check or the tests failed.
set -euo pipefail

readonly HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT=5183
BUILD_ONLY=0
SDK_MODEL=1

say() {
  printf '[demo %s] %s\n' "$(date -u +%H:%M:%SZ)" "$*"
}

usage() {
  sed -n '2,13p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' >&2
  exit 2
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --port) [[ $# -ge 2 && "$2" =~ ^[0-9]+$ ]] || usage; PORT="$2"; shift 2 ;;
      --build-only) BUILD_ONLY=1; shift ;;
      --no-sdk-model) SDK_MODEL=0; shift ;;
      *) usage ;;
    esac
  done
}

need_programs() {
  local program
  for program in node npm python3; do
    command -v "$program" >/dev/null 2>&1 || { say "missing program: $program"; exit 3; }
  done
  local major
  major="$(node -p 'process.versions.node.split(".")[0]')"
  [[ "$major" -ge 20 ]] || { say "node 20 or later is needed; this is $(node -v)"; exit 3; }
}

fetch_sdk_model() {
  # The robot's 3D model belongs to the Vector SDK and is not in this repository.
  if [[ -f "$HERE/public/robot/sdk/manifest.json" ]]; then
    say "SDK robot model already fetched"
  elif python3 "$HERE/tools/fetch_sdk_model.py" 2>/dev/null; then
    say "SDK robot model copied from the installed SDK"
  elif python3 "$HERE/tools/fetch_sdk_model.py" --download; then
    say "SDK robot model downloaded from PyPI (the SDK was not installed)"
  else
    say "SDK robot model not available: the robot will be drawn as its physics shapes"
  fi
}

main() {
  parse_args "$@"
  need_programs
  cd "$HERE"
  say "installing packages"
  npm ci --no-audit --no-fund >/dev/null || { say "npm ci failed"; exit 5; }
  say "checking the exported model against the simulacrum's parameter table"
  python3 tools/export_model.py --check || { say "the exported model is out of date"; exit 7; }
  if [[ "$SDK_MODEL" -eq 1 ]]; then
    fetch_sdk_model
    if [[ -f "$HERE/public/robot/sdk/manifest.json" ]]; then
      python3 tools/fetch_print_parts.py || say "print assets unavailable online; using local parts or SDK geometry"
    fi
  fi
  say "running the tests against the Python simulacrum's recorded runs"
  npm test >/dev/null || { say "tests failed: run npm test to see which"; exit 7; }
  say "building"
  npm run build >/dev/null || { say "build failed"; exit 5; }
  if [[ "$BUILD_ONLY" -eq 1 ]]; then
    say "built: $HERE/dist"
    exit 0
  fi
  say "serving on http://127.0.0.1:${PORT}  (Ctrl-C to stop)"
  exec npx vite preview --host 127.0.0.1 --port "$PORT" --strictPort
}

main "$@"

#!/usr/bin/env bash
# Build the reference MuJoCo 4.0.1 environment for this skill.
#
#   env/setup_mujoco_401.sh [--venv DIR] [--python PY] [--borrow SITE_PACKAGES | --no-borrow]
#
# Creates a virtual environment (default /root/vector-mujoco-401) and installs
# env/requirements-mujoco-401.txt with --require-hashes --no-deps. With --borrow
# (default: $VECTOR_TRAINING_SITE, when set) the training packages are read
# from that site-packages directory instead of installed: env/borrowed-training.json
# pins their versions and RECORD hashes, and the link is refused if they differ.
# The borrowed directory is read, never written; the environment it belongs to
# is not touched. An existing environment is checked, never deleted or rebuilt:
# if it does not pass, the script stops and says so.
#
# Exit: 0 ready (built now or already there and passing), 2 bad arguments,
# 3 environment missing or not as pinned.
set -euo pipefail

readonly HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SKILL="$(dirname "$HERE")"
readonly REQS="$HERE/requirements-mujoco-401.txt"
readonly BORROWED_SPEC="$HERE/borrowed-training.json"
readonly PTH_NAME="zz-borrowed-training.pth"
readonly EXIT_USAGE=2 EXIT_ENV=3
VENV="/root/vector-mujoco-401"
PY="python3.11"
BORROW=""

log() { printf '%s setup_mujoco_401: %s\n' "$(date -u +%FT%TZ)" "$*" >&2; }

parse_args() {
  local n=0
  BORROW="${VECTOR_TRAINING_SITE:-}"
  while [[ $# -gt 0 ]]; do
    (( ++n <= 16 )) || { log "too many arguments"; exit "$EXIT_USAGE"; }
    case "$1" in
      --venv) VENV="${2:?--venv needs a directory}"; shift 2 ;;
      --python) PY="${2:?--python needs an interpreter}"; shift 2 ;;
      --borrow) BORROW="${2:?--borrow needs a site-packages directory}"; shift 2 ;;
      --no-borrow) BORROW=""; shift ;;
      *) log "unknown argument: $1"; exit "$EXIT_USAGE" ;;
    esac
  done
  [[ "$VENV" == /* ]] || { log "--venv must be an absolute path: $VENV"; exit "$EXIT_USAGE"; }
  [[ -f "$REQS" && -f "$BORROWED_SPEC" ]] || { log "missing $REQS or $BORROWED_SPEC"; exit "$EXIT_ENV"; }
}

site_packages() {
  "$VENV/bin/python" -I -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])'
}

verify() {
  # The pin check runs from the skill directory, so it imports this skill's mujoco_pin.
  local args=()
  [[ -n "$BORROW" ]] && args=(--borrowed "$BORROWED_SPEC")
  (cd "$SKILL" && "$VENV/bin/python" -m simulacrum.mujoco_pin "${args[@]}")
}

build() {
  command -v "$PY" >/dev/null || { log "no interpreter $PY"; exit "$EXIT_ENV"; }
  log "creating $VENV with $("$PY" -V 2>&1)"
  "$PY" -m venv "$VENV"
  # The borrowed .pth is written only after this install: with it in place pip would see the
  # borrowed packages, decide mujoco-core is already installed and skip it (measured 2026-11-22).
  "$VENV/bin/python" -m pip install -q --disable-pip-version-check --require-hashes --no-deps -r "$REQS"
  if [[ -n "$BORROW" ]]; then
    [[ -d "$BORROW" ]] || { log "no such directory to borrow from: $BORROW"; exit "$EXIT_ENV"; }
    printf '%s\n' "$BORROW" > "$(site_packages)/$PTH_NAME"
    log "training packages read from $BORROW (checked against $BORROWED_SPEC)"
  fi
}

main() {
  parse_args "$@"
  if [[ -e "$VENV" ]]; then
    log "$VENV exists; checking it, not rebuilding"
  else
    build
  fi
  if verify; then
    log "ready: $VENV"
    return 0
  fi
  log "$VENV does not match the pin; move it aside and run again"
  exit "$EXIT_ENV"
}

main "$@"

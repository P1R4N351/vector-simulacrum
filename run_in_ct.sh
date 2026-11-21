#!/usr/bin/env bash
# Copy this skill into a GPU host (a GPU host) and run a file there with
# `the GPU host python`..
#
#   ./run_in_ct.sh tests/run_all.py            run the whole test suite
#   ./run_in_ct.sh tests/mutate.py             run the mutation check
#   ./run_in_ct.sh demo.py                     any file of this skill
#
# The files travel as a tar archive inside the script that is sent on
# stdin, so nothing is built into a shell string. The copy in the container
# lives in /root/vector-simulacrum and is replaced on every run.
set -euo pipefail

readonly HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly REMOTE_DIR="/root/vector-simulacrum"
readonly JUMP_HOST="${CAULDRON_JUMP_HOST:?set CAULDRON_JUMP_HOST to the SSH host that reaches the node}"
readonly NODE="${CAULDRON_NODE:?set CAULDRON_NODE to the Proxmox node that runs the container}"
readonly CTID="${CAULDRON_CTID:-930}"
SCRIPT_FILE=""

usage() {
  echo "usage: $0 <file-inside-this-skill> [args...]" >&2
  exit 2
}

build_script() {
  # $1: file to run, rest: its arguments. Writes the remote script to stdout.
  local target="$1"
  shift
  printf 'set -euo pipefail\n'
  printf 'mkdir -p %q\n' "$REMOTE_DIR"
  printf 'find %q -mindepth 1 -delete\n' "$REMOTE_DIR"
  printf 'base64 -d <<'"'"'VECTOR_SIMULACRUM_TAR'"'"' | tar -xzf - -C %q\n' "$REMOTE_DIR"
  COPYFILE_DISABLE=1 tar --no-xattrs -czf - -C "$HERE" --exclude '__pycache__' --exclude '.DS_Store' \
    --exclude './web/node_modules' --exclude './web/dist' --exclude './web/public/robot/sdk' . | base64
  printf 'VECTOR_SIMULACRUM_TAR\n'
  printf 'cd %q\n' "$REMOTE_DIR"
  printf 'the GPU host python %q' "$target"
  local arg
  for arg in "$@"; do
    printf ' %q' "$arg"
  done
  printf '\n'
}

main() {
  [[ $# -ge 1 ]] || usage
  [[ -f "$HERE/$1" ]] || { echo "no such file in this skill: $1" >&2; exit 2; }
  SCRIPT_FILE="$(mktemp -t vector-simulacrum-ct.XXXXXX)"
  trap 'rm -f "$SCRIPT_FILE"' EXIT
  build_script "$@" > "$SCRIPT_FILE"
  local remote
  remote="$(printf 'ssh %q "pct exec %q -- bash -s"' "$NODE" "$CTID")"
  ssh -o BatchMode=yes -o ConnectTimeout=15 "$JUMP_HOST" "$remote" < "$SCRIPT_FILE"
}

main "$@"

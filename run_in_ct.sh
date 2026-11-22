#!/usr/bin/env bash
# Copy this skill into a GPU host's container and run a file there on the
# reference MuJoCo, 4.0.1 (simulacrum/mujoco_pin.py).
#
#   ./run_in_ct.sh tests/run_all.py            run the whole test suite
#   ./run_in_ct.sh tests/mutate.py             run the mutation check
#   ./run_in_ct.sh demo.py                     any file of this skill
#   VECTOR_MUJOCO=container ./run_in_ct.sh ... the same on the container's own MuJoCo (3.11.0, kept for mjlab)
#
# CAULDRON_PYTHON is the container's own Python command (it sets the MUJOCO_*
# environment the simulation needs). The 4.0.1 environment is
# VECTOR_MUJOCO_VENV (default /root/vector-mujoco-401), built on first use, and
# checked on every run, by env/setup_mujoco_401.sh from the hashes in env/.
# With VECTOR_TRAINING_SITE set to the container environment's site-packages,
# torch, rsl_rl and onnxruntime are read from there, read-only, pinned by
# env/borrowed-training.json. Every file runs through pinned_run.py, which fails
# the run (exit 3) if the libmujoco actually loaded is not the expected version:
# 4.0.1, or 3.11.0 under VECTOR_MUJOCO=container.
#
# The files travel as a tar archive inside the script that is sent on
# stdin, so nothing is built into a shell string. The copy in the container
# lives in /root/vector-simulacrum (or CAULDRON_REMOTE_DIR, which must be a
# folder directly under /root) and is replaced on every run.
#
# The container is shared. Every run holds /root/.the GPU host-run.lock from
# before the copy is replaced until the run ends, so two runs never overlap:
# one cannot wipe another's copy mid-run, and the shared node never carries
# two test loads at once. A run waits for the lock up to CAULDRON_LOCK_WAIT_S
# seconds (default 1800) and then
# gives up with exit 75. Other lanes' scripts take the same lock to stay out
# of each other's way (2026-11-20: a sibling lane's copy replaced this one
# mid-suite and 78 tests errored).
set -euo pipefail

readonly HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly REMOTE_DIR="${CAULDRON_REMOTE_DIR:-/root/vector-simulacrum}"
readonly LOCK_FILE="${CAULDRON_LOCK_FILE:-/root/.the GPU host-run.lock}"
readonly LOCK_WAIT_S="${CAULDRON_LOCK_WAIT_S:-1800}"
readonly EXIT_BUSY=75
readonly JUMP_HOST="${CAULDRON_JUMP_HOST:?set CAULDRON_JUMP_HOST to the SSH host that reaches the node}"
readonly NODE="${CAULDRON_NODE:?set CAULDRON_NODE to the Proxmox node that runs the container}"
readonly CTID="${CAULDRON_CTID:-930}"
readonly CONTAINER_PYTHON="${CAULDRON_PYTHON:?set CAULDRON_PYTHON to the Python command of the container itself}"
readonly MUJOCO_SIDE="${VECTOR_MUJOCO:-pinned}"
readonly MUJOCO_VENV="${VECTOR_MUJOCO_VENV:-/root/vector-mujoco-401}"
readonly TRAINING_SITE="${VECTOR_TRAINING_SITE:-}"
readonly CONTAINER_MUJOCO_VERSION="3.11.0"
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
  printf 'exec 9>%q\n' "$LOCK_FILE"
  printf 'if ! flock -n 9; then echo "waiting for %s (another run in the container holds it)" >&2;\n' "$LOCK_FILE"
  printf '  flock -w %d 9 || { echo "gave up after %d s waiting for %s" >&2; exit %d; }; fi\n' \
    "$LOCK_WAIT_S" "$LOCK_WAIT_S" "$LOCK_FILE" "$EXIT_BUSY"
  printf 'mkdir -p %q\n' "$REMOTE_DIR"
  printf 'find %q -mindepth 1 -delete\n' "$REMOTE_DIR"
  printf 'base64 -d <<'"'"'VECTOR_SIMULACRUM_TAR'"'"' | tar -xzf - -C %q\n' "$REMOTE_DIR"
  COPYFILE_DISABLE=1 tar --no-xattrs -czf - -C "$HERE" --exclude '__pycache__' --exclude '.DS_Store' \
    --exclude './web/node_modules' --exclude './web/dist' --exclude './web/public/robot/sdk' . | base64
  printf 'VECTOR_SIMULACRUM_TAR\n'
  printf 'cd %q\n' "$REMOTE_DIR"
  interpreter_lines
  printf ' pinned_run.py %q' "$target"
  local arg
  for arg in "$@"; do
    printf ' %q' "$arg"
  done
  printf '\n'
}

interpreter_lines() {
  # Writes the lines that pick the interpreter; the caller appends pinned_run.py and the file.
  if [[ "$MUJOCO_SIDE" == "container" ]]; then
    printf 'export VECTOR_MUJOCO_EXPECT=%q\n' "$CONTAINER_MUJOCO_VERSION"
    printf '%s' "$CONTAINER_PYTHON"
    return 0
  fi
  if [[ -n "$TRAINING_SITE" ]]; then
    printf 'bash env/setup_mujoco_401.sh --venv %q --borrow %q >&2 || exit 3\n' "$MUJOCO_VENV" "$TRAINING_SITE"
  else
    printf 'bash env/setup_mujoco_401.sh --venv %q --no-borrow >&2 || exit 3\n' "$MUJOCO_VENV"
  fi
  printf 'unset VECTOR_MUJOCO_EXPECT\n'
  printf 'MJ_ENV=()\n'
  printf 'while IFS= read -r -d %q kv; do MJ_ENV+=("$kv"); done < <(%s -I -c %q)\n' "" "$CONTAINER_PYTHON" \
    'import os, sys; sys.stdout.write("".join(k + "=" + v + "\0" for k, v in os.environ.items() if k.startswith("MUJOCO")))'
  printf '%s\n' '[[ ${#MJ_ENV[@]} -gt 0 ]] || { echo "the container Python gave no MUJOCO_* environment" >&2; exit 3; }'
  printf 'env "${MJ_ENV[@]}" %q' "$MUJOCO_VENV/bin/python"
}

main() {
  [[ $# -ge 1 ]] || usage
  [[ "$REMOTE_DIR" =~ ^/root/[A-Za-z0-9._-]+$ ]] || { echo "CAULDRON_REMOTE_DIR must be one folder under /root: $REMOTE_DIR" >&2; exit 2; }
  [[ "$MUJOCO_SIDE" == "pinned" || "$MUJOCO_SIDE" == "container" ]] || { echo "VECTOR_MUJOCO must be pinned or container: $MUJOCO_SIDE" >&2; exit 2; }
  [[ "$MUJOCO_VENV" =~ ^/root/[A-Za-z0-9._-]+$ ]] || { echo "VECTOR_MUJOCO_VENV must be one folder under /root: $MUJOCO_VENV" >&2; exit 2; }
  [[ "$CONTAINER_PYTHON" =~ ^[A-Za-z0-9_./][A-Za-z0-9_./\ -]*$ ]] || { echo "CAULDRON_PYTHON must be a plain command: $CONTAINER_PYTHON" >&2; exit 2; }
  [[ -z "$TRAINING_SITE" || "$TRAINING_SITE" =~ ^/[A-Za-z0-9_./-]+$ ]] || { echo "VECTOR_TRAINING_SITE must be an absolute path: $TRAINING_SITE" >&2; exit 2; }
  [[ "$LOCK_WAIT_S" =~ ^[0-9]{1,5}$ ]] || { echo "CAULDRON_LOCK_WAIT_S must be whole seconds: $LOCK_WAIT_S" >&2; exit 2; }
  [[ -f "$HERE/$1" ]] || { echo "no such file in this skill: $1" >&2; exit 2; }
  SCRIPT_FILE="$(mktemp -t vector-simulacrum-ct.XXXXXX)"
  trap 'rm -f "$SCRIPT_FILE"' EXIT
  build_script "$@" > "$SCRIPT_FILE"
  local remote
  remote="$(printf 'ssh %q "pct exec %q -- bash -s"' "$NODE" "$CTID")"
  ssh -o BatchMode=yes -o ConnectTimeout=15 "$JUMP_HOST" "$remote" < "$SCRIPT_FILE"
}

main "$@"

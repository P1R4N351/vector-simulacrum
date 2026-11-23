#!/usr/bin/env bash
# Run a previously built Simulator vic-robot with an isolated filesystem.
set -eu
repo=$(cd "${1:?pass the wire-os-victor checkout}" && pwd)
sdk="${VICOS_SDK:-$HOME/.anki/vicos-sdk/dist/$(cat "$repo/project/VICOS_SDK_VERSION")}"
build="$repo/_build/vicos/Simulator"
test -x "$build/bin/vic-robot" || { echo "Build the vic-robot Simulator target first" >&2; exit 1; }
exec bwrap --die-with-parent --unshare-user --unshare-pid \
  --ro-bind "$sdk/sysroot/usr" /usr --symlink usr/lib /lib \
  --ro-bind "$build" /anki --ro-bind "$(readlink -f "$(command -v qemu-arm-static)")" /qemu-arm \
  --dev /dev --proc /proc --tmpfs /tmp --tmpfs /run --tmpfs /data --chdir /tmp \
  -- /qemu-arm -E LD_LIBRARY_PATH=/anki/lib /anki/bin/vic-robot

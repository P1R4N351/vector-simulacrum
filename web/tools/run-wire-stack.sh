#!/usr/bin/env bash
# Run the three local firmware processes against the browser MuJoCo body.
set -eu
repo=$(cd "${1:?pass the wire-os-victor checkout}" && pwd)
build="$repo/_build/vicos/Simulator"
sdk="${VICOS_SDK:-$HOME/.anki/vicos-sdk/dist/$(cat "$repo/project/VICOS_SDK_VERSION")}"
qemu=$(readlink -f "$(command -v qemu-arm-static)")
for name in vic-robot vic-engine vic-anim; do
  test -x "$build/bin/$name" || { echo "Build the Simulator $name target first" >&2; exit 1; }
done
# ARM asset enumeration under QEMU failed on the tested host ext4 build tree.
# Stage resource directories on tmpfs so all three processes see the same files.
state=$(mktemp -d /dev/shm/vector-wire.XXXXXXXX)
children=()
cleanup() {
  for pid in "${children[@]}"; do kill "$pid" 2>/dev/null || true; done
  wait || true
  rm -rf -- "$state"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
mkdir -p "$state"/{tmp,run,devsocket,data/data/com.anki.victor/{cache,persistent}}
# The stock engine forks to invoke sudo for appliance OS settings. If exec
# fails, the multithreaded ARM child can hang in its libc exit handlers.
# Return failure from a fresh process inside the sandbox instead.
printf '%s\n' 'int main(void) { return 1; }' | cc -static -x c -o "$state/deny-os-setting" -
cp -r "$build/data" "$state/resources"
for name in vic-robot vic-engine vic-anim; do
  bwrap --die-with-parent --unshare-user --unshare-pid \
    --ro-bind "$sdk/sysroot/usr/lib" /usr/lib --ro-bind "$sdk/sysroot/usr/share" /usr/share --symlink usr/lib /lib \
    --ro-bind "$state/deny-os-setting" /usr/bin/sudo \
    --ro-bind "$build" /anki --ro-bind "$state/resources" /anki/data \
    --ro-bind "$repo/3rd/avs-device-sdk/vicos/lib" /avs --ro-bind "$repo/3rd/mpg123/vicos/lib" /mpg \
    --ro-bind "$repo/platform/config/etc/config/platform_config.json" /platform.json \
    --ro-bind "$qemu" /qemu-arm --dev /dev --proc /proc --bind "$state/devsocket" /dev/socket \
    --bind "$state/tmp" /tmp --bind "$state/data" /data --bind "$state/run" /run --chdir /tmp \
    -- /qemu-arm -E LD_LIBRARY_PATH=/anki/lib:/avs:/mpg -E VIC_ENGINE_CONFIG=/platform.json \
       -E VIC_ANIM_CONFIG=/platform.json "/anki/bin/$name" &
  children+=("$!")
  sleep 1
done
wait -n

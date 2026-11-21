#!/bin/sh
# Preserve native MJPEG frames and arrival timestamps. Stop with q or Ctrl-C.
set -eu
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 OUTPUT.mkv SECONDS" >&2
    exit 2
fi
: "${VECTOR_CAMERA_STREAM:?Set this to the camera bridge live.mjpg URL}"
exec ffmpeg -hide_banner -loglevel warning -n -use_wallclock_as_timestamps 1 \
  -rw_timeout 5000000 -f mpjpeg -i "$VECTOR_CAMERA_STREAM" \
  -t "$2" -map 0:v:0 -c:v copy -f matroska "$1"

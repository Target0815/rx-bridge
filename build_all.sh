#!/bin/sh
# Builds rx-bridge for all supported boards into dist/
# The version is taken from FW_VERSION in src/config.h.
set -e
: "${PICO_SDK_PATH:?PICO_SDK_PATH muss auf das Pico SDK (2.x) zeigen}"
VERSION=$(sed -n 's/^#define FW_VERSION "\(.*\)".*/\1/p' src/config.h)
[ -n "$VERSION" ] || { echo "FW_VERSION in src/config.h nicht gefunden"; exit 1; }
# picotool is downloaded and built only once for all boards
export PICOTOOL_FETCH_FROM_GIT_PATH="${PICOTOOL_FETCH_FROM_GIT_PATH:-$PWD/build/_picotool}"
mkdir -p dist
for board in pico waveshare_rp2040_zero seeed_xiao_rp2040 pico2 waveshare_rp2350_zero seeed_xiao_rp2350; do
  cmake -G Ninja -B "build/$board" -DPICO_BOARD="$board" -DCMAKE_BUILD_TYPE=Release . >/dev/null
  ninja -C "build/$board" >/dev/null
  cp "build/$board/rx_bridge.uf2" "dist/rx-bridge-$VERSION-$board.uf2"
  echo "ok  $board"
done

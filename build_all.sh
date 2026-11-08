#!/bin/sh
# Builds rx-bridge for all supported boards into dist/
set -e
: "${PICO_SDK_PATH:?PICO_SDK_PATH muss auf das Pico SDK (2.x) zeigen}"
mkdir -p dist
for board in pico waveshare_rp2040_zero seeed_xiao_rp2040 pico2 waveshare_rp2350_zero seeed_xiao_rp2350; do
  cmake -G Ninja -B "build/$board" -DPICO_BOARD="$board" -DCMAKE_BUILD_TYPE=Release . >/dev/null
  ninja -C "build/$board" >/dev/null
  cp "build/$board/rx_bridge.uf2" "dist/rx-bridge-0.3.2-$board.uf2"
  echo "ok  $board"
done

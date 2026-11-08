#!/bin/sh
# Runs all rx-bridge tests. Needs a C compiler, Python 3 and a configured build (build/pico).
# Optional: path to an INAV checkout as $1 for the cross-check against the original INAV parsers.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT="$HERE/.."
TMP=${TMPDIR:-/tmp}
HDR=$(find "$ROOT/build/pico" -name uart.pio.h | head -1)
BHDR=$(find "$ROOT/build/pico" -name bridge.pio.h | head -1)
[ -n "$HDR" ] || { echo "erst bauen: cmake -B build/pico -DPICO_BOARD=pico && ninja -C build/pico"; exit 1; }
echo "== Protokollkern und Zeitsimulation"
"$HERE/host/run.sh" "$TMP/rxb_host_test"
echo "== PIO-Programme (taktgenaue Simulation)"
python3 "$HERE/test_uart_pio.py" "$HDR" "$BHDR"
if node -e "require('playwright')" 2>/dev/null || NODE_PATH=$(npm root -g 2>/dev/null) node -e "require('playwright')" 2>/dev/null; then
  echo "== Konfigurationsseite im Browser (Chromium, simuliertes Geraet)"
  NODE_PATH=${NODE_PATH:-$(npm root -g)} node "$HERE/web/test_page.js" "$TMP"
else
  echo "== Konfigurationsseite: uebersprungen (Node.js mit Playwright fehlt)"
fi
if [ -n "$1" ]; then
  echo "== Gegenprobe mit den Original-Parsern von INAV"
  "$HERE/inav/run.sh" "$1" "$TMP/rxb_inav_test"
fi

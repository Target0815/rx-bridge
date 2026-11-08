#!/bin/sh
# Builds and runs the INAV cross-check. INAV is not part of rx-bridge; pass the path
# of an INAV checkout (git clone https://github.com/iNavFlight/inav) as $1.
set -e
INAV=${1:?Pfad zum INAV-Quellcode angeben}/src/main
HERE=$(dirname "$0")
OUT=${2:-/tmp/rxb_inav_test}
cc -std=gnu11 -O1 -g -Wall -Wno-unused-function -Wno-address-of-packed-member \
   -I"$HERE/stubs" -I"$INAV" -o "$OUT" \
   "$HERE/test_inav.c" "$HERE/harness.c" \
   "$INAV/rx/crsf.c" "$INAV/rx/fport.c" "$INAV/rx/fport2.c" "$INAV/rx/sbus_channels.c" \
   "$INAV/rx/frsky_crc.c" "$INAV/common/crc.c" "$INAV/common/streambuf.c" "$INAV/telemetry/msp_shared.c" "$HERE/harness_msp.c" \
   "$HERE/../../src/proto/frsky.c" "$HERE/../../src/proto/crsf.c" "$HERE/../../src/proto/telem.c" "$HERE/../../src/proto/convert.c" -lm
"$OUT"

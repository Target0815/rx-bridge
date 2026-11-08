#!/bin/sh
set -e
HERE=$(dirname "$0")
OUT=${1:-/tmp/rxb_host_test}
cc -std=c11 -O2 -g -Wall -Wextra -Wno-unused-parameter -o "$OUT" "$HERE/test_host.c" \
   "$HERE/../../src/proto/frsky.c" "$HERE/../../src/proto/crsf.c" \
   "$HERE/../../src/proto/telem.c" "$HERE/../../src/proto/convert.c" -lm
"$OUT"
cc -std=c11 -O2 -Wall -Wextra -o "$OUT.webcmd" "$HERE/test_webcmd.c" "$HERE/../../src/webcmd.c"
"$OUT.webcmd" "$HERE/../web/device_lines.jsonl"
python3 -c "import json,sys; [json.loads(l) for l in open(sys.argv[1])]; print('webcmd: JSON-Zeilen gueltig')" "$HERE/../web/device_lines.jsonl"

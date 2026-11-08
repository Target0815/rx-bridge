#!/usr/bin/env python3
"""
Cycle tests of the converter PIO programs (uart.pio) and, as an independent second check,
the transparent bridge programs (bridge.pio), using pio_sim.py.

usage: test_uart_pio.py <uart.pio.h> <bridge.pio.h>
"""
import random, sys
from pio_sim import Prog, SM, uart_wave, uart_decode

FAIL = []
def check(c, msg):
    if not c:
        FAIL.append(msg); print("  FAIL:", msg)


def test_rx(h):
    print("uart_rx8")
    prog = Prog(h, "uart_rx8")
    rnd = random.Random(1)
    for err in (-0.025, -0.01, 0.0, 0.01, 0.025):          # transmitter baud error
        data = [rnd.randrange(256) for _ in range(60)] + [0x00, 0xFF, 0x55, 0x7E]
        w = uart_wave(data, 8 * (1 + err), gap_bits=0)
        sm = SM(prog, in_base=0, jmp_pin=0)
        got = []
        for lvl in w:
            sm.step(lvl)
            while sm.rxf: got.append(sm.rxf.pop(0) >> 24)
        check(got == data, "rx baud error %+.1f%%: %d/%d bytes" % (err * 100, sum(a == b for a, b in zip(got, data)), len(data)))
        print("  baud error %+.1f %%: %d/%d bytes correct" % (err * 100, sum(a == b for a, b in zip(got, data)), len(data)))
    # framing error is dropped and the receiver recovers
    w = uart_wave([0x41], 8) + [0] * 200 + [1] * 100 + uart_wave([0x42], 8)
    sm = SM(prog, in_base=0, jmp_pin=0)
    got = []
    for lvl in w:
        sm.step(lvl)
        while sm.rxf: got.append(sm.rxf.pop(0) >> 24)
    check(got == [0x41, 0x42], "break handling %s" % got)
    print("  break / framing error dropped, next byte ok:", got == [0x41, 0x42])


def test_tx(h):
    print("uart_tx8")
    prog = Prog(h, "uart_tx8")
    data = list(range(0, 256, 7)) + [0, 255]
    sm = SM(prog, out_base=0, ss_base=0)
    sm.out_val = 1; sm.out_dir = 1
    sm.txf = [b for b in data]
    lv = []
    for _ in range(len(data) * 80 + 200):
        sm.step(0)
        lv.append(sm.out_val & 1)
    dec = uart_decode(lv, 8)
    check([b for _, b, ok in dec] == data and all(ok for _, _, ok in dec), "tx bytes")
    check(lv[-1] == 1, "tx idle high")
    print("  %d bytes, decoded correctly, idle high afterwards: %s" % (len(data), lv[-1] == 1))


def test_hd_tx(h):
    print("hd_uart_tx8 (receiver line, releases after the last byte)")
    prog = Prog(h, "hd_uart_tx8")
    rnd = random.Random(2)
    for burst in ([0x08, 0x1B, 0x10, 0x00, 0x02, 0x10, 0x27, 0, 0, 0xC4], [0x7E], [rnd.randrange(256) for _ in range(19)]):
        idle0 = 50
        # feed the rest of the burst the way the CPU does (keep FIFO topped up)
        sm = SM(prog, out_base=0, set_base=0, ss_base=0, status_tx_lt=1)
        sm.out_val = 1
        pending = list(burst)
        val, d = [], []
        for i in range(idle0 + len(burst) * 100 + 400):
            if i >= idle0:
                while pending and len(sm.txf) < 4: sm.txf.append(pending.pop(0))
            sm.step(0)
            val.append(sm.out_val & 1)
            d.append(sm.out_dir & 1)
        # logical domain: a released line is held at the idle level by the pull resistor
        line = [v if dr else 1 for v, dr in zip(val, d)]
        dec = uart_decode(line, 8)
        check([b for _, b, ok in dec] == burst and all(ok for _, _, ok in dec), "hd tx bytes %s" % burst[:3])
        first = d.index(1)
        last = len(d) - 1 - d[::-1].index(1)
        start_bit = dec[0][0] if dec else 0
        end_stop = dec[-1][0] + 80 if dec else 0
        check(first <= start_bit and first >= start_bit - 2, "takes line just before start bit (%d vs %d)" % (first, start_bit))
        check(last >= end_stop - 1 and last <= end_stop + 24, "releases after stop bit: driven until %d, stop ends %d" % (last, end_stop))
        drives = sum(d)
        print("  %2d bytes: driven %d cycles (%.2f bit), released %d cycles (%.1f bit) after the last stop bit"
              % (len(burst), drives, drives / 8, last - end_stop + 1, (last - end_stop + 1) / 8))


def test_bridge(h):
    print("bridge.pio second check (independent simulator)")
    # fc_tx_drive: follows FC TX, releases after hold time
    prog = Prog(h, "fc_tx_drive")
    hold_clk = 1250
    sm = SM(prog, in_base=0, jmp_pin=0, set_base=1, set_count=1)
    sm.y = hold_clk // 2
    sm.out_val = 0b10
    w = [1] * 50 + uart_wave([0x55, 0xA3, 0x00, 0xFF], 271, idle_before=0, idle_after=0) + [1] * 2000
    line, dr = [], []
    for lvl in w:
        sm.step(lvl)
        line.append((sm.out_val >> 1) & 1); dr.append((sm.out_dir >> 1) & 1)
    lat = []
    for i in range(1, len(w)):
        if w[i] != w[i - 1]:
            k = next((j for j in range(8) if dr[i + j] and line[i + j] == w[i]), None)
            if k is not None: lat.append(k)
    rel = len(dr) - 1 - dr[::-1].index(1)
    last_edge = max(i for i in range(1, len(w)) if w[i] != w[i - 1])
    check(max(lat) <= 4, "bridge latency %d" % max(lat))
    check(abs((rel - last_edge) - hold_clk) < 20, "bridge release after %d clk" % (rel - last_edge))
    print("  fc_tx_drive: worst latency %d clk, released %d clk after the last edge (hold %d)" % (max(lat), rel - last_edge, hold_clk))


if __name__ == "__main__":
    test_rx(sys.argv[1])
    test_tx(sys.argv[1])
    test_hd_tx(sys.argv[1])
    test_bridge(sys.argv[2])
    print("RESULT:", "PASS" if not FAIL else "%d FAILURES" % len(FAIL))
    sys.exit(1 if FAIL else 0)

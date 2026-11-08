#!/usr/bin/env python3
"""
Small cycle-accurate model of one RP2040/RP2350 PIO state machine, written from the
RP2040 datasheet (chapter 3.4). Supports what rx-bridge uses: JMP (all conditions),
WAIT GPIO/PIN, IN, OUT, PUSH/PULL, MOV (incl. STATUS), SET, side-set (optional and
pindirs), delays, wrap. Program words and config are read from a pioasm C header.
"""
import re


class Prog:
    def __init__(self, header, name):
        txt = open(header).read()
        blk = re.search(r'%s_program_instructions\[\] = \{(.*?)\};' % name, txt, re.S).group(1)
        self.ops = [int(h, 16) for h in re.findall(r'0x([0-9a-f]{4})', blk)]
        self.wrap_target = int(re.search(r'#define %s_wrap_target (\d+)' % name, txt).group(1))
        self.wrap = int(re.search(r'#define %s_wrap (\d+)' % name, txt).group(1))
        cfg = re.search(r'%s_program_get_default_config\(uint offset\) \{(.*?)\n\}' % name, txt, re.S).group(1)
        m = re.search(r'sm_config_set_sideset\(&c, (\d+), (true|false), (true|false)\)', cfg)
        self.ss_bits = int(m.group(1)) if m else 0
        self.ss_opt = bool(m and m.group(2) == 'true')
        self.ss_pindirs = bool(m and m.group(3) == 'true')


class SM:
    def __init__(self, prog, in_base=0, out_base=0, out_count=1, set_base=0, set_count=1,
                 ss_base=0, jmp_pin=0, status_tx_lt=None, status_rx_lt=None):
        self.p = prog
        self.in_base, self.out_base, self.out_count = in_base, out_base, out_count
        self.set_base, self.set_count, self.ss_base, self.jmp_pin = set_base, set_count, ss_base, jmp_pin
        self.status_tx_lt, self.status_rx_lt = status_tx_lt, status_rx_lt
        self.pc = prog.wrap_target if False else 0
        self.x = self.y = 0
        self.isr = self.osr = 0
        self.isr_cnt = 0
        self.osr_cnt = 32
        self.txf, self.rxf = [], []
        self.delay = 0
        self.out_val = 0      # pin output latch (bit per gpio)
        self.out_dir = 0      # pin direction (1 = output)
        self.clock = 0

    # pin helpers -----------------------------------------------------------
    def _write(self, base, count, value, dirs=False):
        for i in range(count):
            bit = 1 << ((base + i) % 32)
            v = (value >> i) & 1
            if dirs:
                self.out_dir = (self.out_dir | bit) if v else (self.out_dir & ~bit)
            else:
                self.out_val = (self.out_val | bit) if v else (self.out_val & ~bit)

    def _pins_in(self, ext):
        # a pin driven by this SM reads back its own output, otherwise the external level
        return (self.out_val & self.out_dir) | (ext & ~self.out_dir)

    def step(self, ext):
        """Advance one PIO clock. ext = external pin levels (int bitmask)."""
        self.clock += 1
        if self.delay:
            self.delay -= 1
            return
        op = self.p.ops[self.pc]
        dsf = (op >> 8) & 0x1F
        nss = self.p.ss_bits
        ss_en, ss_val = False, 0
        if nss:
            if self.p.ss_opt:
                ss_en = bool(dsf & 0x10)
                ss_val = (dsf >> (5 - nss)) & ((1 << (nss - 1)) - 1)
            else:
                ss_en = True
                ss_val = dsf >> (5 - nss)
        delay = dsf & ((1 << (5 - nss)) - 1)
        if ss_en:
            cnt = nss - 1 if self.p.ss_opt else nss
            self._write(self.ss_base, cnt, ss_val, dirs=self.p.ss_pindirs)

        pins = self._pins_in(ext)
        kind = op >> 13
        nxt = self.pc + 1 if self.pc != self.p.wrap else self.p.wrap_target
        stall = False

        if kind == 0:  # JMP
            cond, addr = (op >> 5) & 7, op & 0x1F
            take = False
            if cond == 0: take = True
            elif cond == 1: take = self.x == 0
            elif cond == 2: take = self.x != 0; self.x = (self.x - 1) & 0xFFFFFFFF
            elif cond == 3: take = self.y == 0
            elif cond == 4: take = self.y != 0; self.y = (self.y - 1) & 0xFFFFFFFF
            elif cond == 5: take = self.x != self.y
            elif cond == 6: take = bool((pins >> self.jmp_pin) & 1)
            elif cond == 7: take = self.osr_cnt < 32
            if take: nxt = addr
        elif kind == 1:  # WAIT
            pol, src, idx = (op >> 7) & 1, (op >> 5) & 3, op & 0x1F
            gpio = idx if src == 0 else (self.in_base + idx) % 32
            if src > 1: raise NotImplementedError("wait irq")
            if ((pins >> gpio) & 1) != pol: stall = True
        elif kind == 2:  # IN
            src, n = (op >> 5) & 7, (op & 0x1F) or 32
            val = {0: pins >> self.in_base, 1: self.x, 2: self.y, 3: 0, 6: self.isr, 7: self.osr}[src] & ((1 << n) - 1)
            self.isr = ((self.isr >> n) | (val << (32 - n))) & 0xFFFFFFFF if n < 32 else val
            self.isr_cnt = min(32, self.isr_cnt + n)
        elif kind == 3:  # OUT
            dst, n = (op >> 5) & 7, (op & 0x1F) or 32
            val = self.osr & ((1 << n) - 1)
            self.osr = (self.osr >> n) if n < 32 else 0
            self.osr_cnt = min(32, self.osr_cnt + n)
            if dst == 0: self._write(self.out_base, min(n, self.out_count), val)
            elif dst == 1: self.x = val
            elif dst == 2: self.y = val
            elif dst == 4: self._write(self.out_base, min(n, self.out_count), val, dirs=True)
            elif dst == 5: nxt = val
            elif dst != 3: raise NotImplementedError("out dest %d" % dst)
        elif kind == 4:  # PUSH / PULL
            block = bool(op & 0x20)
            if op & 0x80:  # PULL
                if self.txf:
                    self.osr = self.txf.pop(0); self.osr_cnt = 0
                elif block:
                    stall = True
                else:
                    self.osr = self.x; self.osr_cnt = 0
            else:
                if len(self.rxf) < 8:
                    self.rxf.append(self.isr)
                elif block:
                    stall = True
                if not stall:
                    self.isr = 0; self.isr_cnt = 0
        elif kind == 5:  # MOV
            dst, opn, src = (op >> 5) & 7, (op >> 3) & 3, op & 7
            if src == 0: v = pins >> self.in_base
            elif src == 1: v = self.x
            elif src == 2: v = self.y
            elif src == 3: v = 0
            elif src == 5:
                if self.status_tx_lt is not None: v = 0xFFFFFFFF if len(self.txf) < self.status_tx_lt else 0
                elif self.status_rx_lt is not None: v = 0xFFFFFFFF if len(self.rxf) < self.status_rx_lt else 0
                else: raise RuntimeError("status not configured")
            elif src == 6: v = self.isr
            elif src == 7: v = self.osr
            else: raise NotImplementedError("mov src")
            v &= 0xFFFFFFFF
            if opn == 1: v ^= 0xFFFFFFFF
            elif opn == 2: v = int('{:032b}'.format(v)[::-1], 2)
            if dst == 0: self._write(self.out_base, self.out_count, v)
            elif dst == 1: self.x = v
            elif dst == 2: self.y = v
            elif dst == 5: nxt = v & 0x1F
            elif dst == 6: self.isr = v; self.isr_cnt = 0
            elif dst == 7: self.osr = v; self.osr_cnt = 0
            else: raise NotImplementedError("mov dst")
        elif kind == 7:  # SET
            dst, data = (op >> 5) & 7, op & 0x1F
            if dst == 0: self._write(self.set_base, self.set_count, data)
            elif dst == 1: self.x = data
            elif dst == 2: self.y = data
            elif dst == 4: self._write(self.set_base, self.set_count, data, dirs=True)
            else: raise NotImplementedError("set dst")
        else:
            raise NotImplementedError("irq")

        if not stall:
            self.pc = nxt
            self.delay = delay


def uart_wave(data, spb, idle_before=40, gap_bits=0, idle_after=40):
    """Logical UART levels, one entry per PIO cycle (spb cycles per bit)."""
    w = [1] * int(idle_before * spb)
    t = len(w)
    bits = []
    for b in data:
        bits += [0] + [(b >> k) & 1 for k in range(8)] + [1] + [1] * gap_bits
    n = int(len(bits) * spb)
    w += [bits[min(int(i / spb), len(bits) - 1)] for i in range(n)]
    w += [1] * int(idle_after * spb)
    return w


def uart_decode(levels, spb):
    """Reference decoder: find start bits, sample mid-bit. Returns list of (start_index, byte, stop_ok)."""
    out, i = [], 1
    while i < len(levels):
        if levels[i - 1] == 1 and levels[i] == 0:
            s = i
            mid = lambda k: levels[min(len(levels) - 1, int(s + (k + 0.5) * spb))]
            if mid(0) != 0:
                i += 1; continue
            byte = sum(mid(1 + k) << k for k in range(8))
            out.append((s, byte, mid(9) == 1))
            i = int(s + 9.5 * spb)
        else:
            i += 1
    return out

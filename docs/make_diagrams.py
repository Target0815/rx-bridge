#!/usr/bin/env python3
"""Generates the wiring diagrams docs/schaltbild-inverter.svg and docs/schaltbild-konverter.svg."""
import os

W, H = 1000, 500
INK, MUTED, RULE, BOX = "#1b2832", "#5b6b77", "#9aa6b0", "#f6f8fa"
SIG, TLM, PWR, GND = "#1f6fd1", "#23804f", "#c0392b", "#1b2832"


def text(x, y, s, size=15, weight=400, color=INK, anchor="start"):
    return f'<text x="{x}" y="{y}" font-size="{size}" font-weight="{weight}" fill="{color}" text-anchor="{anchor}">{s}</text>'


def block(x, y, w, h, title, sub, pins, side):
    """pins: list of (label, y). side: 'r' pins on the right edge, 'l' on the left edge."""
    out = [f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="10" fill="{BOX}" stroke="{INK}" stroke-width="1.5"/>',
           text(x + w / 2, y + 30, title, 19, 600, anchor="middle"),
           text(x + w / 2, y + 50, sub, 13, color=MUTED, anchor="middle")]
    for label, py in pins:
        px = x + w if side == "r" else x
        out.append(f'<circle cx="{px}" cy="{py}" r="4.5" fill="{INK}"/>')
        tx = px - 12 if side == "r" else px + 12
        out.append(text(tx, py + 5, label, 14, 500, anchor="end" if side == "r" else "start"))
    return out


def wire(points, color, width=2.5, dash=None):
    d = " ".join(f"{'M' if i == 0 else 'L'} {x} {y}" for i, (x, y) in enumerate(points))
    da = f' stroke-dasharray="{dash}"' if dash else ""
    return f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round"{da}/>'


def arrow(x1, y, x2, color, label, below=False):
    head = 8 if x2 > x1 else -8
    return [wire([(x1, y), (x2, y)], color, 2),
            f'<path d="M {x2} {y} L {x2 - head} {y - 5} L {x2 - head} {y + 5} Z" fill="{color}"/>',
            text((x1 + x2) / 2, y + (17 if below else -8), label, 13, 500, color, "middle")]


def diagram(mode):
    conv = mode == "konverter"
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" '
         f'font-family="Barlow, Segoe UI, Helvetica, Arial, sans-serif">',
         f'<rect width="{W}" height="{H}" fill="#ffffff"/>']
    title = "Konverter-Betrieb: F.Port / FBUS vom Empfänger, CRSF zur FC" if conv else \
            "Inverter-Betrieb: Protokoll bleibt gleich, nur die Polarität wird gedreht"
    o.append(text(30, 40, title, 20, 600))

    # pin rows
    D = 30                                      # vertical offset below the arrow band
    yS, yP, yM = 175 + D, 215 + D, 255 + D       # receiver S, +, -
    yL, yRX, yTX, y5, yG = 175 + D, 300 + D, 340 + D, 215 + D, 255 + D
    o += block(40, 110 + D, 170, 200, "Empfänger", "FrSky, F.Port / FBUS" if conv else "FrSky", [("S", yS), ("+", yP), ("−", yM)], "r")
    o += block(400, 110 + D, 200, 270, "rx-bridge", "RP2040 / RP2350", [("GPIO26 LEITUNG", yL), ("5V", y5), ("GND", yG)], "l")
    for label, py in (("GPIO27 RX", yRX), ("GPIO28 TX", yTX), ("5V", y5), ("GND", yG)):
        o.append(f'<circle cx="600" cy="{py}" r="4.5" fill="{INK}"/>')
        o.append(text(588, py + 5, label, 14, 500, anchor="end"))
    o += block(790, 110 + D, 170, 270, "Flight-Controller", "F405, INAV / ArduPilot", [("RX", yRX), ("TX", yTX), ("5V", y5), ("GND", yG)], "l")

    # signal wire with resistor
    o.append(wire([(210, yS), (265, yS)], SIG))
    o.append(f'<rect x="265" y="{yS - 9}" width="54" height="18" fill="#fff" stroke="{INK}" stroke-width="1.5"/>')
    o.append(text(292, yS - 16, "470 Ω", 13, 600, anchor="middle"))
    o.append(wire([(319, yS), (400, yS)], SIG))
    # power
    o.append(wire([(210, yP), (400, yP)], PWR))
    o.append(wire([(210, yM), (400, yM)], GND))
    o.append(wire([(600, y5), (790, y5)], PWR))
    o.append(wire([(600, yG), (790, yG)], GND))
    # crossed UART
    o.append(wire([(600, yRX), (670, yRX), (720, yTX), (790, yTX)], SIG))
    o.append(wire([(600, yTX), (670, yTX), (720, yRX), (790, yRX)], SIG))
    o.append(text(695, yRX - 14, "gekreuzt", 13, 500, MUTED, "middle"))

    # protocol annotations
    if conv:
        o += arrow(225, 92, 385, SIG, "F.Port / FBUS, invertiert")
        o += arrow(385, 108, 225, TLM, "Telemetrie zum Sender", below=True)
        o += arrow(615, 92, 775, SIG, "CRSF, 420 000 Bd")
        o += arrow(775, 108, 615, TLM, "CRSF-Telemetrie", below=True)
        cfg = ["INAV: serialrx_provider = CRSF", "ArduPilot: SERIALx_PROTOCOL = 23", "Empfänger bleibt auf F.Port / FBUS"]
        mod = "Betriebsart Konverter"
    else:
        o += arrow(225, 92, 385, SIG, "S.Port / F.Port / FBUS, invertiert")
        o += arrow(385, 108, 225, TLM, "Telemetrie, invertiert", below=True)
        o += arrow(615, 92, 775, SIG, "dasselbe, nicht invertiert")
        o += arrow(775, 108, 615, TLM, "Telemetrie der FC", below=True)
        cfg = ["INAV: Provider FPORT / FBUS,", "serialrx_inverted = ON, halfduplex = OFF", "ArduPilot: SERIALx_PROTOCOL = 23 bzw. 10"]
        mod = "Betriebsart Inverter"
    o.append(text(500, 405 + D, mod, 14, 600, SIG, "middle"))

    # FC settings note
    o.append(text(40, 365 + D, "FC-Einstellung", 14, 600))
    for i, line in enumerate(cfg):
        o.append(text(40, 387 + D + i * 20, line, 13, color=MUTED))

    # legend
    lx = 40
    for color, label in ((SIG, "Signal"), (TLM, "Rückweg"), (PWR, "5 V"), (GND, "GND")):
        o.append(wire([(lx, 482), (lx + 24, 482)], color, 3))
        o.append(text(lx + 32, 487, label, 13, color=MUTED))
        lx += 110
    o.append(text(960, 487, "Pins: RP2040-Zero „26/27/28“, XIAO D0/D1/D2, Pico 31/32/34", 12, color=MUTED, anchor="end"))
    o.append("</svg>")
    return "\n".join(o)


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    for m in ("inverter", "konverter"):
        with open(os.path.join(here, f"schaltbild-{m}.svg"), "w") as f:
            f.write(diagram(m))
        print("geschrieben:", f"schaltbild-{m}.svg")

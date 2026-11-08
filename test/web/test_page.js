// Browser test of web/index.html with a simulated rx-bridge behind navigator.serial.
// usage: node test_page.js <screenshot dir>
// Needs Playwright with Chromium (require('playwright')).
const { chromium } = require("playwright");
const fs = require("fs");
const path = require("path");

const PAGE = "file://" + path.resolve(__dirname, "../../web/index.html");
const DEVICE_LINES = fs.readFileSync(path.join(__dirname, "device_lines.jsonl"), "utf8").trim().split("\n");
const OUT = process.argv[2] || "/tmp";

// Runs inside the page before its scripts: a fake Web Serial API with a device model that
// follows the firmware's line protocol (src/webcmd.h). It answers with JSON in the same
// format as wc_fmt_* (field names checked against device_lines.jsonl in this test).
function installMock(opts) {
  const enc = new TextEncoder(), dec = new TextDecoder();
  const dev = { mode: 0, run: 0, rcsys: 0, inv: 1, hold: 10, saved: { mode: 0, rcsys: 0, inv: 1, hold: 10 }, json: false,
                secs: 0, old: !!opts.oldFirmware, received: [], scenario: opts.scenario || "normal" };
  window.__dev = dev;
  let ctrl = null, timer = null, alive = false;
  const cfgLine = () => JSON.stringify({ t: "cfg", mode: dev.mode, run: dev.run, rcsys: dev.rcsys, inv: dev.inv, hold: dev.hold,
    saved: dev.mode === dev.saved.mode && dev.rcsys === dev.saved.rcsys && dev.inv === dev.saved.inv && dev.hold === dev.saved.hold });
  const out = (s) => { if (ctrl && alive) ctrl.enqueue(enc.encode(s + "\r\n")); };
  function status() {
    dev.secs++;
    if (!dev.json) { out(`[${dev.secs} s] Leitung: kein Signal | FC sendet: nein | inv`); return; }
    if (dev.run === 0) out(JSON.stringify({ t: "st", run: 0, s: dev.secs, pulses: 1874, baud: 460800, raw: 461000, edges: 2210, inv: dev.inv }));
    else {
      const fs = dev.scenario === "failsafe" && dev.secs % 6 >= 3;
      out(JSON.stringify({ t: "st", run: 1, s: dev.secs, proto: "FBUS", baud: 460800, nch: 16, rc: fs ? 0 : 111, lq: fs ? 0 : 98, rssi: 87,
        fs: fs ? 1 : 0, tlm: 48, ap: 0, up: 37, null: 2, late: 0, crc_rx: fs ? 2 : 0, crc_fc: 0, ptdrop: 0, fc: dev.secs > 2 ? "INAV 8.0.1" : "" }));
    }
  }
  function command(line) {
    dev.received.push(line);
    if (dev.old) { if (line === "b") dev.bootloader = true; return; }      // 0.2 firmware ignores '#' lines
    const m = line.match(/^#(\w+)\s*(.*)$/);
    if (!m) { if (line === "b") dev.bootloader = true; return; }
    const [_, cmd, arg] = m;
    if (cmd === "hello") { out(JSON.stringify({ t: "hello", fw: "rx-bridge", ver: "0.3.2", board: "waveshare_rp2040_zero", pins: { line: 26, rx: 27, tx: 28 }, mhz: 125, rcsys: ["FrSky"] })); out(cfgLine()); }
    else if (cmd === "get") out(cfgLine());
    else if (cmd === "json") { dev.json = arg.trim() === "1"; out(JSON.stringify({ t: "ack", cmd: "json", ok: true, msg: "" })); }
    else if (cmd === "set") {
      const [k, v] = arg.split("=").map((x) => x.trim());
      const n = Number(v);
      if (k === "hold" && (n < 1 || n > 100)) { out(JSON.stringify({ t: "ack", cmd: "error", ok: false, msg: "hold: 1 bis 100 us" })); return; }
      dev[k === "inv" ? "inv" : k] = n;
      out(JSON.stringify({ t: "ack", cmd: "set", ok: true, msg: dev.mode !== dev.run ? "Betriebsart wird nach Speichern und Neustart aktiv" : "" }));
      out(cfgLine());
    } else if (cmd === "save") { dev.saved = { mode: dev.mode, rcsys: dev.rcsys, inv: dev.inv, hold: dev.hold }; out(JSON.stringify({ t: "ack", cmd: "save", ok: true, msg: "gespeichert" })); out(cfgLine()); }
    else if (cmd === "reboot") { out(JSON.stringify({ t: "ack", cmd: "reboot", ok: true, msg: "Neustart" })); if (!opts.rebootHangs) setTimeout(reboot, 50); }
    else if (cmd === "boot") { out(JSON.stringify({ t: "ack", cmd: "boot", ok: true, msg: "USB-Bootloader" })); dev.bootloader = true; setTimeout(() => unplug(), 50); }
  }
  function unplug() { alive = false; clearInterval(timer); try { ctrl.close(); } catch (e) {} port._open = false; }
  function reboot() {
    unplug();
    dev.reboots = (dev.reboots || 0) + 1;
    dev.mode = dev.saved.mode; dev.inv = dev.saved.inv; dev.hold = dev.saved.hold; dev.run = dev.mode; dev.json = false; dev.secs = 0;
    port._gone = true;
    setTimeout(() => { port._gone = false; }, 1200);                         // USB re-enumerates after ~1 s
  }
  let lineBuf = "";
  const port = {
    _open: false, _gone: false,
    getInfo: () => ({ usbVendorId: 0x2E8A, usbProductId: 0x000A }),
    async open() {
      if (this._gone) throw new DOMException("Device not found", "NetworkError");
      this._open = true; alive = true;
      this.readable = new ReadableStream({ start(c) { ctrl = c; } });
      this.writable = new WritableStream({ write(chunk) {
        lineBuf += dec.decode(chunk);
        for (;;) {
          const i = lineBuf.indexOf("\n");
          if (i < 0) { if (lineBuf === "b") { command("b"); lineBuf = ""; } break; }
          const l = lineBuf.slice(0, i); lineBuf = lineBuf.slice(i + 1); command(l);
        }
      } });
      clearInterval(timer);
      timer = setInterval(status, opts.statusMs || 300);
      if (opts.replay) setTimeout(() => opts.replay.forEach((l, k) => setTimeout(() => out(l), k * 40)), 100);
    },
    async close() { this._open = false; alive = false; clearInterval(timer); },
    async setSignals(sig) { dev.dtr = !!sig.dataTerminalReady; },
  };
  window.__port = port;
  const serial = new EventTarget();
  serial.requestPort = async () => port;
  serial.getPorts = async () => (port._gone ? [] : [port]);
  Object.defineProperty(navigator, "serial", { value: serial });
}

let fails = 0, checks = 0;
function check(c, msg) { checks++; if (!c) { fails++; console.log("  FAIL:", msg); } }
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

(async () => {
  const browser = await chromium.launch();

  // 1. full flow with a simulated 0.3 device
  {
    console.log("Ablauf: verbinden, Inverter-Status, auf Konverter umstellen, speichern + Neustart, Failsafe");
    const page = await browser.newPage({ viewport: { width: 1280, height: 900 } });
    const errors = [];
    page.on("pageerror", (e) => errors.push(e.message));
    page.on("console", (m) => { if (m.type() === "error" && !m.text().startsWith("Failed to load resource")) errors.push(m.text()); });
    await page.addInitScript(installMock, { scenario: "failsafe", statusMs: 300 });
    await page.goto(PAGE);
    await page.screenshot({ path: path.join(OUT, "web-1-start.png") });
    check(await page.isDisabled("#save"), "save disabled before connect");
    await page.click("#connect");
    await page.waitForSelector("text=Firmware 0.3.2 auf Waveshare RP2040-Zero");
    await sleep(800);
    const rec = await page.evaluate(() => window.__dev.received);
    check(rec[0] === "#json 1" && rec[1] === "#hello", "handshake " + rec.slice(0, 2));
    check(await page.evaluate(() => window.__dev.dtr === true), "DTR set on open");
    check(await page.isChecked("input[name=mode][value='0']"), "mode inverter shown");
    check((await page.textContent("#modename")) === "Inverter", "path shows inverter");
    check((await page.textContent("#w1proto")).includes("460,8 kBd"), "inverter baud on wire: " + (await page.textContent("#w1proto")));
    check((await page.textContent("#savestate")).includes("gespeichert"), "saved state");
    check(await page.isVisible("#holdfield"), "hold visible in inverter mode");
    await page.screenshot({ path: path.join(OUT, "web-2-inverter.png") });

    await page.selectOption("#hold", "20");
    await sleep(150);
    check((await page.evaluate(() => window.__dev.hold)) === 20, "hold sent");
    check((await page.textContent("#savestate")).includes("Nicht gespeichert"), "dirty after change");

    await page.check("input[name=mode][value='1']");
    await sleep(150);
    check((await page.textContent("#save")) === "Speichern und neu starten", "save label for mode change");
    check((await page.textContent("#modehint")).includes("nach Speichern und Neustart"), "pending hint");
    check(!(await page.isVisible("#holdfield")), "hold hidden for converter");
    await page.screenshot({ path: path.join(OUT, "web-3-pending.png") });

    await page.click("#save");
    await page.waitForSelector("#modename:text('Konverter')", { timeout: 15000 });
    await sleep(1200);
    const dev = await page.evaluate(() => window.__dev);
    check(dev.reboots === 1 && dev.run === 1 && dev.saved.hold === 20, "saved, rebooted into converter");
    check(dev.received.filter((l) => l === "#hello").length === 2, "page reconnected and said hello again");
    check((await page.textContent("#save")) === "Speichern" && (await page.isDisabled("#save")), "nothing to save after reboot");
    check((await page.textContent("#w2proto")) === "CRSF", "converter path shows CRSF");
    await page.waitForSelector(".stat dd.fail", { timeout: 5000 });
    check((await page.getAttribute("#led", "class")).includes("blink"), "LED blinks in failsafe");
    await page.screenshot({ path: path.join(OUT, "web-4-failsafe.png") });
    await page.waitForSelector(".stat dd.ok:text('nein')", { timeout: 5000 });
    await page.screenshot({ path: path.join(OUT, "web-5-converter.png") });
    const log = await page.textContent("#log");
    check(log.includes("Failsafe JA") && log.includes("FBUS 460800 Bd, 16 Kan."), "log lines like the console");
    check(log.includes("Gerät startet neu"), "reboot logged");
    check((await page.textContent("#fcname")).includes("INAV 8.0.1"), "FC name in signal path");
    check((await page.textContent("#stats")).includes("37davon 2 leer"), "telemetry wording");
    check(log.includes("FC INAV 8.0.1, Telemetrie 48/s | zum Empfänger 37/s, davon 2 leer"), "log with FC name");
    check((await page.textContent("#rcsys")).includes("FrSky (S.Port, F.Port, FBUS)"), "rc system order");
    check(!(await page.$("#boot")), "no firmware update button");

    // log is cleared on a new connection (default), kept when the box is unticked
    await page.click("#connect");
    await page.click("#connect");
    await sleep(500);
    check(!(await page.textContent("#log")).includes("Failsafe JA"), "log cleared on connect");
    await page.uncheck("#logauto");
    await page.click("#connect");
    await page.click("#connect");
    await sleep(500);
    check((await page.textContent("#log")).includes("Getrennt."), "log kept when unticked");
    await page.reload();
    check(!(await page.isChecked("#logauto")), "choice remembered");
    await page.check("#logauto");
    await page.click("#connect");
    await page.waitForSelector("text=Firmware 0.3");

    // invalid value from the device side is shown
    await page.evaluate(() => send("#set hold=999"));
    await page.waitForSelector("#msg:not([hidden])");
    check((await page.textContent("#msg")).includes("1 bis 100"), "error from device shown");
    await page.evaluate(() => send("#set hold=10"));
    await page.waitForSelector("#msg", { state: "hidden" });

    // mobile layout
    await page.setViewportSize({ width: 390, height: 1400 });
    await sleep(200);
    const overflow = await page.evaluate(() => document.documentElement.scrollWidth > window.innerWidth + 1);
    check(!overflow, "no horizontal scroll at 390 px");
    await page.screenshot({ path: path.join(OUT, "web-6-mobile.png"), fullPage: true });
    check(errors.length === 0, "page errors: " + errors.join(" | "));
    await page.close();
  }

  // 2. the exact JSON lines produced by the firmware code (test_webcmd) are understood
  {
    console.log("Firmware-Ausgaben aus webcmd.c an die Seite");
    const page = await browser.newPage();
    const errors = [];
    page.on("pageerror", (e) => errors.push(e.message));
    await page.addInitScript(installMock, { replay: DEVICE_LINES, statusMs: 100000 });
    await page.goto(PAGE);
    await page.click("#connect");
    await sleep(DEVICE_LINES.length * 40 + 600);
    const log = await page.textContent("#log");
    check(log.includes("rx-bridge 0.3.2 auf Waveshare RP2040-Zero, Leitung GPIO26, RX GPIO27, TX GPIO28"), "hello parsed");
    check(log.includes("Leitung: 1874 Pulse/s, ~460800 Baud | FC sendet: ja (2210 Flanken/s)"), "bridge status parsed");
    check(log.includes("FBUS 460800 Bd, 16 Kan. | Kanäle 111/s | LQ 98 % | RSSI 87 | Failsafe nein | FC ArduPlane V4.5.7 (5e1a7b3c), Telemetrie 48/s"), "converter status parsed");
    check(log.includes('FC Q"uote\\x'), "escaped FC name parsed");
    check(log.includes("Prüfsumme Empf. 3, FC 0, zu spät 1"), "error counters parsed");
    check(log.includes("kein F.Port/FBUS-Signal (suche, 115200 Baud)"), "search status parsed");
    check(log.includes("Fehler: hold: 1 bis 100 us"), "error ack parsed");
    check(errors.length === 0, "page errors: " + errors.join(" | "));
    await page.close();
  }

  // 3. old firmware without the web protocol
  {
    console.log("Alte Firmware 0.2");
    const page = await browser.newPage();
    await page.addInitScript(installMock, { oldFirmware: true, statusMs: 400 });
    await page.goto(PAGE);
    await page.click("#connect");
    await page.waitForSelector("#oldfw:not([hidden])", { timeout: 4000 });
    check((await page.textContent("#oldfw")).includes("Firmware 0.3"), "old firmware hint");
    await page.screenshot({ path: path.join(OUT, "web-7-oldfw.png") });
    await page.close();
  }

  // 4. module that does not restart (behaviour of 0.3.0 in converter mode)
  {
    console.log("Neustart bleibt aus");
    const page = await browser.newPage();
    await page.addInitScript(installMock, { rebootHangs: true, statusMs: 300 });
    await page.goto(PAGE);
    await page.click("#connect");
    await page.waitForSelector("text=Firmware 0.3");
    await page.click("#reboot");
    await page.waitForSelector("#msg:not([hidden])", { timeout: 6000 });
    check((await page.textContent("#msg")).includes("nicht neu gestartet"), "reboot failure reported");
    await page.close();
  }

  // 5. browser without Web Serial
  {
    console.log("Browser ohne WebSerial");
    const page = await browser.newPage();
    await page.addInitScript(() => { delete Navigator.prototype.serial; });
    await page.goto(PAGE);
    check(await page.isVisible("#nosupport"), "unsupported notice");
    check(await page.isDisabled("#connect"), "connect disabled");
    await page.close();
  }

  await browser.close();
  console.log(`Webseite: ${checks} Prüfungen, ${fails} Fehler`);
  process.exit(fails ? 1 : 0);
})().catch((e) => { console.error(e); process.exit(2); });

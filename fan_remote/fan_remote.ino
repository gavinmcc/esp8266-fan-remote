/*
 * fan_remote.ino
 *
 * ESP8266 WiFi ceiling fan remote controller.
 * Controls fans over HTTP and Alexa (Espalexa / Hue emulation).
 *
 * Device parameters live in devices.json at the repo root.
 * Run gen_config.py (or build.sh) to regenerate fan_config.h before compiling.
 *
 * Wiring:
 *   CC1101 VCC  -> 3.3V
 *   CC1101 GND  -> GND
 *   CC1101 MOSI -> GPIO13 (D7)
 *   CC1101 MISO -> GPIO12 (D6)
 *   CC1101 SCK  -> GPIO14 (D5)
 *   CC1101 CSN  -> GPIO15 (D8)
 */

#include <SPI.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Espalexa.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include "secrets.h"
#include "fan_config.h"

const char* SSID     = WIFI_SSID;
const char* PASSWORD = WIFI_PASSWORD;

#define PIN_CSN      15   // GPIO15 (D8)

// CC1101 register addresses
#define REG_PKTCTRL0 0x08
#define REG_MDMCFG2  0x12
#define REG_FREND0   0x22
#define REG_PATABLE  0x3E

// CC1101 strobe commands
#define STROBE_SFSTXON 0x31
#define STROBE_STX     0x35
#define STROBE_SIDLE   0x36

// Command slot names (index matches hi/med/low/off/light order in FanCmds)
static const char* CMD_KEYS[]   = { "hi",  "med",  "low",  "off",  "light" };
static const char* CMD_LABELS[] = { "HI",  "MED",  "LOW",  "OFF",  "LIGHT" };

ESP8266WebServer server(80);
Espalexa espalexa;

// ── FIFO NRZ stream buffer ────────────────────────────────
// Sized for worst case: 50 reps × ~105 bytes (Bedroom OFF)
static byte g_stream[5500];
static int  g_streamBit;
static int  g_streamLen;

static void streamClear() {
    memset(g_stream, 0, sizeof(g_stream));
    g_streamBit = 0;
}

static void streamAppend(int count, bool val) {
    for (int i = 0; i < count; i++) {
        int byteIdx = g_streamBit / 8;
        int bitIdx  = 7 - (g_streamBit % 8);
        if (byteIdx < (int)sizeof(g_stream)) {
            if (val) g_stream[byteIdx] |= (1 << bitIdx);
        }
        g_streamBit++;
    }
}

// ── Frame builders (one per protocol) ────────────────────

// UC7070T: 6-pulse header (LS LS LL SL SS LL) + 7-pulse N-encoded data
static void buildRepUC7070T(const FanDevice& fan, int N) {
    const FanTiming& t = fan.timing;
    streamAppend(t.sync0_h, true);  streamAppend(t.sync0_l, false);  // LS (sync)
    streamAppend(t.long_h,  true);  streamAppend(t.short_l, false);  // LS
    streamAppend(t.long_h,  true);  streamAppend(t.long_l,  false);  // LL
    streamAppend(t.short_h, true);  streamAppend(t.long_l,  false);  // SL
    streamAppend(t.short_h, true);  streamAppend(t.short_l, false);  // SS
    streamAppend(t.long_h,  true);  streamAppend(t.long_l,  false);  // LL
    for (int i = 0; i < N; i++) {
        streamAppend(t.short_h, true); streamAppend(t.long_l, false);
    }
    streamAppend(t.short_h, true);  streamAppend(t.short_l, false);  // SS marker
    if (N < 5) {
        streamAppend(t.long_h,  true);  streamAppend(t.long_l, false);
        for (int i = 0; i < (4 - N); i++) {
            streamAppend(t.short_h, true); streamAppend(t.long_l, false);
        }
        streamAppend(t.short_h, true);                                // S_term
    } else {
        streamAppend(t.long_h, true);                                 // L_term
    }
    streamAppend(t.gap, false);
}

// SMC5060RF: SL×N + SS + LL + SL×(7-N) + SS + LL + SS + L_term + GAP
static void buildRepSMC5060RF(const FanDevice& fan, int N) {
    const FanTiming& t = fan.timing;
    for (int i = 0; i < N; i++) {
        streamAppend(t.short_h, true);  streamAppend(t.long_l,  false);  // SL
    }
    streamAppend(t.short_h, true);  streamAppend(t.short_l, false);  // SS
    streamAppend(t.long_h,  true);  streamAppend(t.long_l,  false);  // LL
    for (int i = 0; i < (7 - N); i++) {
        streamAppend(t.short_h, true);  streamAppend(t.long_l,  false);  // SL
    }
    streamAppend(t.short_h, true);  streamAppend(t.short_l, false);  // SS
    streamAppend(t.long_h,  true);  streamAppend(t.long_l,  false);  // LL
    streamAppend(t.short_h, true);  streamAppend(t.short_l, false);  // SS
    streamAppend(t.long_h,  true);                                    // L_term
    streamAppend(t.gap, false);
}

static void buildStream(const FanDevice& fan, int N, int reps) {
    streamClear();
    for (int r = 0; r < reps; r++) {
        if (fan.protocol == PROTO_UC7070T) buildRepUC7070T(fan, N);
        else                               buildRepSMC5060RF(fan, N);
    }
    g_streamLen = (g_streamBit + 7) / 8;
}

// ── CC1101 init ───────────────────────────────────────────

void cc1101InitTx() {
    pinMode(PIN_CSN, OUTPUT);
    digitalWrite(PIN_CSN, HIGH);
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV16);

    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setMHZ(FANS[0].freq_mhz);
    ELECHOUSE_cc1101.setModulation(2);
    ELECHOUSE_cc1101.setPA(10);
    ELECHOUSE_cc1101.setDRate(40);
    ELECHOUSE_cc1101.SpiWriteReg(REG_PKTCTRL0, 0x02);
    ELECHOUSE_cc1101.SpiWriteReg(REG_MDMCFG2,  0x30);
    ELECHOUSE_cc1101.SpiWriteReg(REG_FREND0,   0x11);
    ELECHOUSE_cc1101.SpiWriteReg(0x17,         0x00);  // MCSM1: disable CCA
    ELECHOUSE_cc1101.SpiWriteReg(0x00,         0x56);  // IOCFG2: inverted PA_PD

    byte pa[8]; memset(pa, 0x00, sizeof(pa)); pa[1] = 0xC0;
    ELECHOUSE_cc1101.SpiWriteBurstReg(REG_PATABLE, pa, 8);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
}

// ── Core FIFO TX ──────────────────────────────────────────

static void fifoTransmit(float freq_mhz, const char* tag) {
    ELECHOUSE_cc1101.setMHZ(freq_mhz);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);

    int firstChunk = min(g_streamLen, 64);
    ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, g_stream, firstChunk);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SFSTXON);
    delay(3);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_STX);

    int sent = firstChunk;
    while (sent < g_streamLen) {
        byte txb = ELECHOUSE_cc1101.SpiReadStatus(0x3A);
        if (txb & 0x80) { Serial.printf("[%s] FIFO underflow at %d/%d!\n", tag, sent, g_streamLen); break; }
        int avail = 64 - (int)(txb & 0x7F);
        if (avail > 0) {
            int chunk = min(avail, g_streamLen - sent);
            ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, g_stream + sent, chunk);
            sent += chunk;
        }
        yield();
    }

    uint32_t deadline = millis() + 1000;
    while (millis() < deadline) {
        byte txb = ELECHOUSE_cc1101.SpiReadStatus(0x3A);
        if ((txb & 0x80) || (txb & 0x7F) == 0) break;
        yield();
    }
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
    ELECHOUSE_cc1101.SpiStrobe(0x3B);  // SFTX: flush TX FIFO
}

static void sendCommand(const FanDevice& fan, int N, int reps, const char* tag) {
    buildStream(fan, N, reps);
    Serial.printf("[TX:%s] N=%d %d reps %d bytes\n", fan.name, N, reps, g_streamLen);
    fifoTransmit(fan.freq_mhz, tag);
    ELECHOUSE_cc1101.setMHZ(FANS[0].freq_mhz);  // park at home frequency
    Serial.printf("[TX:%s] done\n", fan.name);
}

// ── Alexa static dispatch ─────────────────────────────────

struct AlexaEntry { int dev; int cmd; };
static AlexaEntry g_alexaTable[ALEXA_SLOTS];
static int        g_alexaCount = 0;

// Pending command queue (one slot) — filled by dispatchAlexa(), drained in loop().
// Alexa has a short response timeout; queuing lets Espalexa reply immediately
// and transmit RF on the next loop iteration after the response has gone out.
struct PendingCmd { int fanIdx; int N; int reps; char tag[32]; };
static bool        g_cmdPending = false;
static PendingCmd  g_pending;

// Debounce: Alexa sometimes sends two events for one voice command (e.g. a paired
// on+off brightness sequence). Track the last dispatched slot and ignore repeats
// within 2 seconds so toggle commands don't double-fire.
static int      g_lastAlexaSlot = -1;
static uint32_t g_lastAlexaMs   = 0;

void dispatchAlexa(int idx, uint8_t b) {
    const AlexaEntry& e   = g_alexaTable[idx];
    const FanDevice&  fan = FANS[e.dev];
    const FanCmd* cmds[5] = {
        &fan.cmds.hi, &fan.cmds.med, &fan.cmds.low, &fan.cmds.off, &fan.cmds.light
    };
    const FanCmd& cmd = *cmds[e.cmd];
    if (cmd.fire_always || b) {
        uint32_t now = millis();
        if (idx == g_lastAlexaSlot && now - g_lastAlexaMs < 2000) return;
        g_lastAlexaSlot = idx;
        g_lastAlexaMs   = now;
        g_pending.fanIdx = e.dev;
        g_pending.N      = cmd.N;
        g_pending.reps   = cmd.reps;
        snprintf(g_pending.tag, sizeof(g_pending.tag), "ALEXA:%s:%s", fan.name, CMD_LABELS[e.cmd]);
        g_cmdPending = true;
    }
}

// ── Diagnostic HTTP handlers ──────────────────────────────

void handleDumpregs() {
    struct { const char* name; uint8_t addr; } regs[] = {
        {"IOCFG2",   0x00},
        {"PKTCTRL0", 0x08},
        {"PKTCTRL1", 0x07},
        {"MDMCFG4",  0x10},
        {"MDMCFG3",  0x11},
        {"MDMCFG2",  0x12},
        {"MDMCFG1",  0x13},
        {"FREND0",   0x22},
        {"MCSM0",    0x18},
        {"MCSM1",    0x17},
        {"FREQ2",    0x0D},
        {"FREQ1",    0x0E},
        {"FREQ0",    0x0F},
        {"CHANNR",   0x0A},
        {"FSCTRL1",  0x0B},
    };

    String out = "CC1101 Register Dump\n";
    out += "Register   Addr   Read   Note\n";
    out += "---------- -----  -----  ----\n";

    for (auto& r : regs) {
        uint8_t val = ELECHOUSE_cc1101.SpiReadReg(r.addr);
        char line[80];
        if (r.addr == REG_MDMCFG2) {
            uint8_t modFmt   = (val >> 4) & 0x07;
            uint8_t syncMode = val & 0x07;
            snprintf(line, sizeof(line), "%-10s 0x%02X   0x%02X   MOD_FORMAT=%d(%s) SYNC_MODE=%d\n",
                r.name, r.addr, val, modFmt,
                modFmt == 3 ? "OOK" : modFmt == 0 ? "2-FSK" : "???", syncMode);
        } else if (r.addr == REG_FREND0) {
            snprintf(line, sizeof(line), "%-10s 0x%02X   0x%02X   PA_POWER=%d->PATABLE[%d]\n",
                r.name, r.addr, val, val & 0x07, val & 0x07);
        } else if (r.addr == REG_PKTCTRL0) {
            snprintf(line, sizeof(line), "%-10s 0x%02X   0x%02X   PKT_FMT=%d LEN_CFG=%d CRC=%d\n",
                r.name, r.addr, val, (val >> 5) & 0x03, val & 0x03, (val >> 2) & 1);
        } else {
            snprintf(line, sizeof(line), "%-10s 0x%02X   0x%02X\n", r.name, r.addr, val);
        }
        out += line;
        Serial.print(line);
    }

    byte pat[8];
    for (int i = 0; i < 8; i++) pat[i] = ELECHOUSE_cc1101.SpiReadReg(REG_PATABLE);
    char pline[64];
    snprintf(pline, sizeof(pline), "PATABLE    0x3E   %02X %02X %02X %02X %02X %02X %02X %02X\n",
        pat[0], pat[1], pat[2], pat[3], pat[4], pat[5], pat[6], pat[7]);
    out += pline;
    Serial.print(pline);
    server.send(200, "text/plain", out);
}

void handleStatus() {
    String out = "=== Fan Remote Status ===\n";
    for (int d = 0; d < FAN_COUNT; d++) {
        const FanDevice& fan = FANS[d];
        const FanTiming& t   = fan.timing;
        char buf[256];
        snprintf(buf, sizeof(buf),
            "%s (%s, %.2f MHz, 40kbaud):\n"
            "  short_h=%d (%dus)  long_h=%d (%dus)\n"
            "  short_l=%d (%dus)  long_l=%d (%dus)\n"
            "  gap=%d (%dus)\n",
            fan.name,
            fan.protocol == PROTO_UC7070T ? "UC7070T" : "SMC5060RF",
            fan.freq_mhz,
            t.short_h, t.short_h * 25, t.long_h, t.long_h * 25,
            t.short_l, t.short_l * 25, t.long_l, t.long_l * 25,
            t.gap,     t.gap * 25);
        out += buf;
        out += "  commands:";
        const FanCmd* cmds[5] = {
            &fan.cmds.hi, &fan.cmds.med, &fan.cmds.low, &fan.cmds.off, &fan.cmds.light
        };
        for (int c = 0; c < 5; c++) {
            char cb[32];
            snprintf(cb, sizeof(cb), "  %s(N=%d,r=%d)", CMD_LABELS[c], cmds[c]->N, cmds[c]->reps);
            out += cb;
        }
        out += "\n";
    }
    byte r0 = ELECHOUSE_cc1101.SpiReadReg(REG_PKTCTRL0);
    byte r1 = ELECHOUSE_cc1101.SpiReadReg(REG_MDMCFG2);
    byte r2 = ELECHOUSE_cc1101.SpiReadReg(REG_FREND0);
    byte r3 = ELECHOUSE_cc1101.SpiReadReg(REG_PATABLE);
    char rbuf[160];
    snprintf(rbuf, sizeof(rbuf),
        "Registers:\n  PKTCTRL0=0x%02X (want 0x02)\n  MDMCFG2 =0x%02X (want 0x30)\n"
        "  FREND0  =0x%02X (want 0x11)\n  PATABLE[0]=0x%02X (want 0x00)\n",
        r0, r1, r2, r3);
    out += rbuf;
    Serial.print(out);
    server.send(200, "text/plain", out);
}

void handleCarrier() {
    Serial.println("[CARRIER] starting 3s OOK carrier test...");
    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setMHZ(FANS[0].freq_mhz);
    ELECHOUSE_cc1101.setModulation(2);
    ELECHOUSE_cc1101.setPA(10);
    ELECHOUSE_cc1101.setDRate(0.3);
    ELECHOUSE_cc1101.SpiWriteReg(REG_MDMCFG2,  0x30);
    ELECHOUSE_cc1101.SpiWriteReg(REG_FREND0,   0x11);
    ELECHOUSE_cc1101.SpiWriteReg(REG_PKTCTRL0, 0x02);
    byte pa[8]; memset(pa, 0x00, sizeof(pa)); pa[1] = 0xC0;
    ELECHOUSE_cc1101.SpiWriteBurstReg(REG_PATABLE, pa, 8);

    byte buf[64]; memset(buf, 0xFF, sizeof(buf));
    ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, buf, sizeof(buf));
    ELECHOUSE_cc1101.SpiStrobe(STROBE_STX);

    delay(10);
    byte marc0 = ELECHOUSE_cc1101.SpiReadStatus(0x35);
    Serial.printf("[CARRIER] MARCSTATE after STX = 0x%02X (expect 0x13=TX)\n", marc0);

    uint32_t start = millis();
    uint32_t lastLog = 0;
    byte fill[32]; memset(fill, 0xFF, sizeof(fill));
    while (millis() - start < 3000) {
        byte txbytes = ELECHOUSE_cc1101.SpiReadStatus(0x3A);
        if (txbytes & 0x80) {
            ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
            ELECHOUSE_cc1101.SpiStrobe(0x3B);
            ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, buf, sizeof(buf));
            ELECHOUSE_cc1101.SpiStrobe(STROBE_STX);
        } else if ((txbytes & 0x7F) < 32) {
            ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, fill, sizeof(fill));
        }
        if (millis() - lastLog >= 500) {
            byte ms = ELECHOUSE_cc1101.SpiReadStatus(0x35);
            byte tb = ELECHOUSE_cc1101.SpiReadStatus(0x3A);
            Serial.printf("[CARRIER] t=%lums  MARC=0x%02X  TXBYTES=%d%s\n",
                millis() - start, ms, tb & 0x7F, (tb & 0x80) ? " UNDERFLOW!" : "");
            lastLog = millis();
        }
        yield();
    }
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
    cc1101InitTx();
    Serial.println("[CARRIER] done");
    char msg[128];
    snprintf(msg, sizeof(msg),
        "3s OOK carrier done (%.2f MHz).\nDid the physical remote FAIL during those 3 seconds?",
        FANS[0].freq_mhz);
    server.send(200, "text/plain", msg);
}

void handleFifogate() {
    server.send(200, "text/plain",
        "FIFO OOK gating test (9s).\n"
        "Watch capture unit RSSI:\n"
        "  0-3s:  0xFF carrier ON  -> expect -24 dBm\n"
        "  3-6s:  0x00 carrier OFF -> expect -97 dBm if gating works\n"
        "  6-9s:  0xFF carrier ON  -> expect -24 dBm\n");

    cc1101InitTx();
    byte on_buf[64];  memset(on_buf,  0xFF, sizeof(on_buf));
    byte off_buf[64]; memset(off_buf, 0x00, sizeof(off_buf));

    ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, on_buf, 64);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SFSTXON);
    delay(3);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_STX);
    Serial.printf("[FIFOGATE] started — MARC=0x%02X\n", ELECHOUSE_cc1101.SpiReadStatus(0x35));

    uint32_t start    = millis();
    int      phase    = 0;
    uint32_t phase_end = start + 3000;
    Serial.println("[FIFOGATE] t=0s  phase=ON (0xFF)");

    while (millis() - start < 9000) {
        uint32_t now = millis();
        if (now >= phase_end) {
            phase++;
            phase_end += 3000;
            bool isOn = (phase % 2 == 0);
            Serial.printf("[FIFOGATE] t=%lums  phase=%s\n", now - start, isOn ? "ON (0xFF)" : "OFF (0x00)");
        }
        bool  isOn     = (phase % 2 == 0);
        byte* fill_buf = isOn ? on_buf : off_buf;
        byte txb = ELECHOUSE_cc1101.SpiReadStatus(0x3A);
        if (txb & 0x80) {
            ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
            ELECHOUSE_cc1101.SpiStrobe(0x3B);
            ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, fill_buf, 64);
            ELECHOUSE_cc1101.SpiStrobe(STROBE_STX);
        } else if ((txb & 0x7F) < 48) {
            ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, fill_buf, 32);
        }
        yield();
    }
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
    Serial.println("[FIFOGATE] done");
}

// ── Web UI ────────────────────────────────────────────────

void handleRoot() {
    String html =
        "<!DOCTYPE html><html><head>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Fan Remote</title><style>"
        "body{font-family:sans-serif;max-width:420px;margin:40px auto;text-align:center;"
        "background:#111;color:#eee}"
        "h2{margin-bottom:8px} h3{margin:20px 0 8px;color:#aaa}"
        "a{display:block;margin:8px 0;padding:14px;border-radius:8px;font-size:1.05em;"
        "text-decoration:none;color:#fff;background:#333}"
        "a:hover{background:#555}"
        ".dim{background:#252525;font-size:0.85em}"
        "</style></head><body>"
        "<h2>Fan Remote</h2>";

    static const char* CMD_DISPLAY[] = { "HI", "MED", "LOW", "OFF", "LIGHT (toggle)" };
    for (int d = 0; d < FAN_COUNT; d++) {
        const FanDevice& fan = FANS[d];
        char heading[64];
        snprintf(heading, sizeof(heading), "<h3>%s (%.0f MHz)</h3>", fan.name, fan.freq_mhz);
        html += heading;
        for (int c = 0; c < 5; c++) {
            char link[96];
            snprintf(link, sizeof(link), "<a href='/%s/%s'>%s</a>",
                fan.path, CMD_KEYS[c], CMD_DISPLAY[c]);
            html += link;
        }
    }

    html +=
        "<h3>Diagnostics</h3>"
        "<a href='/carrier'  class='dim'>RF TEST (3s carrier)</a>"
        "<a href='/fifogate' class='dim'>OOK GATE TEST (9s)</a>"
        "<a href='/status'   class='dim'>STATUS / TIMING</a>"
        "<a href='/dumpregs' class='dim'>DUMP REGISTERS</a>"
        "</body></html>";

    server.send(200, "text/html", html);
}

// ── setup ─────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n\n=== ESP8266 Fan Remote ===");

    cc1101InitTx();

    if (ELECHOUSE_cc1101.getCC1101()) {
        Serial.println("CC1101: OK");
    } else {
        Serial.println("CC1101: NOT FOUND — check wiring!");
        while (true) delay(1000);
    }

    byte r_pktctrl0 = ELECHOUSE_cc1101.SpiReadReg(REG_PKTCTRL0);
    byte r_mdmcfg2  = ELECHOUSE_cc1101.SpiReadReg(REG_MDMCFG2);
    byte r_frend0   = ELECHOUSE_cc1101.SpiReadReg(REG_FREND0);
    byte r_patable0 = ELECHOUSE_cc1101.SpiReadReg(REG_PATABLE);
    Serial.printf("PKTCTRL0=0x%02X (want 0x02)  MDMCFG2=0x%02X (want 0x30)\n", r_pktctrl0, r_mdmcfg2);
    Serial.printf("FREND0  =0x%02X (want 0x11)  PATABLE[0]=0x%02X (want 0x00)\n", r_frend0, r_patable0);

    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWORD);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print('.'); }
    Serial.printf("\nIP: http://%s\n", WiFi.localIP().toString().c_str());

    server.on("/", handleRoot);

    // Register routes and Alexa devices from FANS[]
    for (int d = 0; d < FAN_COUNT; d++) {
        const FanCmd* cmdArr[5] = {
            &FANS[d].cmds.hi, &FANS[d].cmds.med, &FANS[d].cmds.low,
            &FANS[d].cmds.off, &FANS[d].cmds.light
        };
        const char* alexaNames[5] = {
            FANS[d].alexa.hi, FANS[d].alexa.med, FANS[d].alexa.low,
            FANS[d].alexa.off, FANS[d].alexa.light
        };

        for (int c = 0; c < 5; c++) {
            char path[32];
            snprintf(path, sizeof(path), "/%s/%s", FANS[d].path, CMD_KEYS[c]);

            int di   = d;
            int ci   = c;
            int N    = cmdArr[c]->N;
            int reps = cmdArr[c]->reps;

            server.on(String(path), [di, ci, N, reps]() {
                // Response body uses uppercase name for backward compatibility
                char nameUpper[32];
                strlcpy(nameUpper, FANS[di].name, sizeof(nameUpper));
                for (char* p = nameUpper; *p; p++) *p = toupper((unsigned char)*p);
                char resp[32];
                snprintf(resp, sizeof(resp), "OK: %s %s", nameUpper, CMD_LABELS[ci]);
                server.send(200, "text/plain", resp);
                delay(50);
                char tag[32];
                snprintf(tag, sizeof(tag), "%s:%s", FANS[di].name, CMD_LABELS[ci]);
                sendCommand(FANS[di], N, reps, tag);
            });

            int idx = g_alexaCount++;
            g_alexaTable[idx] = { d, c };
            espalexa.addDevice(alexaNames[c], ALEXA_THUNKS[idx]);
        }

        Serial.printf("  %s: /%s/{hi,med,low,off,light}  Alexa: %s..%s\n",
            FANS[d].name, FANS[d].path,
            FANS[d].alexa.hi, FANS[d].alexa.light);
    }

    server.on("/carrier",   handleCarrier);
    server.on("/fifogate",  handleFifogate);
    server.on("/status",    handleStatus);
    server.on("/dumpregs",  handleDumpregs);

    server.onNotFound([]() {
        if (!espalexa.handleAlexaApiCall(server.uri(), server.arg(0)))
            server.send(404, "text/plain", "Not found");
    });

    espalexa.begin(&server);
    Serial.printf("Ready. %d Alexa device(s) registered.\n", g_alexaCount);
}

void loop() {
    if (g_cmdPending) {
        g_cmdPending = false;
        sendCommand(FANS[g_pending.fanIdx], g_pending.N, g_pending.reps, g_pending.tag);
    }
    espalexa.loop();
}

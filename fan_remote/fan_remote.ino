/*
 * fan_remote.ino
 *
 * ESP8266 WiFi ceiling fan remote controller
 * Replays captured OOK/ASK codes via CC1101 (E07-M1101D-SMA)
 *
 * Captured from UC7070T remote at 303.92 MHz.
 *
 * Protocol: 13-pulse, position-encoded OOK frame
 *   Header (6 pulses): LS LS LL SL SS LL
 *     (pulse 0 uses shorter HIGH/LOW: SYNC0_H/SYNC0_L, measured ~420/~350µs)
 *   Data   (7 pulses): N×SL, SS, (N<5: LL+(4-N)×SL+S_terminal | N=5: L_terminal)
 *   Gap:    8000µs LOW between repetitions
 *   N encodes the command: 0=HI, 1=MED, 2=LOW, 4=OFF, 5=LIGHT
 *
 * Wiring:
 *   CC1101 VCC  -> 3.3V
 *   CC1101 GND  -> GND
 *   CC1101 MOSI -> GPIO13 (D7)
 *   CC1101 MISO -> GPIO12 (D6)
 *   CC1101 SCK  -> GPIO14 (D5)
 *   CC1101 CSN  -> GPIO15 (D8)
 *
 * HTTP endpoints:
 *   GET /hi    /med   /low   /off   /light  — send command
 *   GET /carrier   — 3s OOK carrier test (jams physical remote if RF OK)
 *   GET /fifogate  — 9s FIFO OOK gating sanity check
 *   GET /status    — timing constants + register readback
 *   GET /dumpregs  — full CC1101 register dump
 */

#include <SPI.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Espalexa.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include "secrets.h"

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

// ── FIFO OOK encoding @ 40 kbaud (25µs per bit) ──────────
// Pulse 0 of each frame is consistently shorter than other LONG_H pulses
// (~420µs vs ~600µs). All reps start with the shorter timing.
#define SHORT_H   10   // 10×25 = 250µs  (measured 195–296µs)
#define LONG_H    24   // 24×25 = 600µs  (measured 522–662µs, pulses 1+)
#define SYNC0_H   17   // 17×25 = 425µs  (pulse 0 only: measured 400–451µs)
#define SYNC0_L   14   // 14×25 = 350µs  (pulse 0 LOW:  measured 294–359µs)
#define SHORT_L   18   // 18×25 = 450µs  (measured 294–499µs)
#define LONG_L    32   // 32×25 = 800µs  (measured 743–848µs)
#define GAP      320   // 320×25 = 8000µs inter-rep gap

// Per-command rep counts. Physical remote sends 4 reps (~60ms total).
// More reps = more chances for fan to decode a valid frame at range.
// LIGHT has a dimmer: short burst (~72ms physical) = toggle, long burst = dim.
//   4 reps = ~84ms, well within "short press" territory.
// OFF (N=4) needs many reps: 4 SL symbols before SS creates more decoding failure points at range.
#define REPS_DEFAULT  25
#define REPS_LIGHT     4
#define REPS_OFF      50

// Command N values — re-captured with dedicated RX unit, all five buttons confirmed
#define CMD_HI     0
#define CMD_MED    1
#define CMD_LOW    2
#define CMD_OFF    4
#define CMD_LIGHT  5

static int commandReps(int N) {
    if (N == CMD_OFF)   return REPS_OFF;
    if (N == CMD_LIGHT) return REPS_LIGHT;
    return REPS_DEFAULT;
}

ESP8266WebServer server(80);
Espalexa espalexa;

// ── FIFO NRZ stream builder ───────────────────────────────
// Buffer for 50 reps × ~105 bytes = ~5250 bytes NRZ bitstream.
// 1-bits = OOK carrier on, 0-bits = carrier off.
static byte g_stream[5500];
static int  g_streamBit;   // current write position (bits)
static int  g_streamLen;   // final byte count after build

static void streamClear() {
    memset(g_stream, 0, sizeof(g_stream));
    g_streamBit = 0;
}

static void streamAppend(int count, bool val) {
    for (int i = 0; i < count; i++) {
        int byteIdx = g_streamBit / 8;
        int bitIdx  = 7 - (g_streamBit % 8);  // MSB first
        if (byteIdx < (int)sizeof(g_stream)) {
            if (val) g_stream[byteIdx] |= (1 << bitIdx);
        }
        g_streamBit++;
    }
}

static void buildRep(int N) {
    // Header pulse 0 (LS with sync timing — shorter than remaining LONG_H pulses)
    streamAppend(SYNC0_H, true);  streamAppend(SYNC0_L, false);
    // Header pulses 1-5 (fixed)
    streamAppend(LONG_H,  true);  streamAppend(SHORT_L, false);  // LS
    streamAppend(LONG_H,  true);  streamAppend(LONG_L,  false);  // LL
    streamAppend(SHORT_H, true);  streamAppend(LONG_L,  false);  // SL
    streamAppend(SHORT_H, true);  streamAppend(SHORT_L, false);  // SS
    streamAppend(LONG_H,  true);  streamAppend(LONG_L,  false);  // LL
    // Data: N × SL prefix
    for (int i = 0; i < N; i++) {
        streamAppend(SHORT_H, true); streamAppend(LONG_L, false);
    }
    // SS command marker
    streamAppend(SHORT_H, true);  streamAppend(SHORT_L, false);
    // Tail
    if (N < 5) {
        streamAppend(LONG_H,  true);  streamAppend(LONG_L, false);  // LL
        for (int i = 0; i < (4 - N); i++) {
            streamAppend(SHORT_H, true); streamAppend(LONG_L, false);  // SL×(4-N)
        }
        streamAppend(SHORT_H, true);  // terminal HIGH
    } else {
        streamAppend(LONG_H, true);   // N=5: single LONG_H terminal
    }
    // Inter-rep gap (LOW)
    streamAppend(GAP, false);
}

static void buildStream(int N) {
    streamClear();
    for (int r = 0; r < commandReps(N); r++) buildRep(N);
    g_streamLen = (g_streamBit + 7) / 8;
}

// ─────────────────────────────────────────────────────────
void cc1101InitTx() {
    pinMode(PIN_CSN, OUTPUT);
    digitalWrite(PIN_CSN, HIGH);
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV16);

    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setMHZ(303.92);
    ELECHOUSE_cc1101.setModulation(2);  // OOK/ASK
    ELECHOUSE_cc1101.setPA(10);

    // PKTCTRL0=0x02: FIFO mode, infinite packet length.
    // MDMCFG2=0x30:  OOK modulation (bits 6:4 = 0b011), no sync word (bits 2:0 = 0b000).
    // FREND0=0x11:   PA_POWER=1 → OOK=1 uses PATABLE[1], OOK=0 uses PATABLE[0].
    //   PATABLE[0]=0x00 (OOK=0 → PA off), PATABLE[1]=0xC0 (OOK=1 → full power).
    //   CC1101 datasheet: "PATABLE[0] should be set to 0 for OOK (PA switched off)."
    //   Prior bug: all PATABLE entries = 0xC0, so OOK=0 still drove carrier → gating never worked.
    ELECHOUSE_cc1101.setDRate(40);
    ELECHOUSE_cc1101.SpiWriteReg(REG_PKTCTRL0, 0x02);
    ELECHOUSE_cc1101.SpiWriteReg(REG_MDMCFG2,  0x30);
    ELECHOUSE_cc1101.SpiWriteReg(REG_FREND0,   0x11);
    // MCSM1=0x00: disable CCA (CCA_MODE=0 = always clear), TXOFF→IDLE, RXOFF→IDLE.
    // Default 0x30 (CCA_MODE=3) silently prevents STX from FSTXON if RSSI is above threshold.
    ELECHOUSE_cc1101.SpiWriteReg(0x17,         0x00);
    // IOCFG2=0x56 (inverted PA_PD): GDO2 LOW in IDLE/FSTXON (ext PA off), HIGH in TX (ext PA on).
    ELECHOUSE_cc1101.SpiWriteReg(0x00,         0x56);

    byte pa[8]; memset(pa, 0x00, sizeof(pa)); pa[1] = 0xC0;
    ELECHOUSE_cc1101.SpiWriteBurstReg(REG_PATABLE, pa, 8);

    ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
}

// ─────────────────────────────────────────────────────────
// FIFO NRZ OOK TX: pre-build bitstream, stream through FIFO while PA stays on.
// PA ramps up once on entry to TX state; FIFO data bits gate OOK on/off.
void sendCommand(int N, const char* name) {
    buildStream(N);
    Serial.printf("[TX] %s N=%d FIFO mode %d reps %d bytes\n", name, N, commandReps(N), g_streamLen);

    // Pre-fill first 64 bytes into FIFO, calibrate PLL, start TX
    int firstChunk = min(g_streamLen, 64);
    ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, g_stream, firstChunk);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SFSTXON);
    delay(3);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_STX);

    // Stream remaining bytes
    int sent = firstChunk;
    while (sent < g_streamLen) {
        byte txb = ELECHOUSE_cc1101.SpiReadStatus(0x3A);
        if (txb & 0x80) {
            Serial.printf("[TX] FIFO underflow at %d/%d bytes!\n", sent, g_streamLen);
            break;
        }
        int avail = 64 - (int)(txb & 0x7F);
        if (avail > 0) {
            int chunk = min(avail, g_streamLen - sent);
            ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, g_stream + sent, chunk);
            sent += chunk;
        }
        yield();
    }

    // Wait for FIFO to drain
    uint32_t deadline = millis() + 1000;
    while (millis() < deadline) {
        byte txb = ELECHOUSE_cc1101.SpiReadStatus(0x3A);
        if ((txb & 0x80) || (txb & 0x7F) == 0) break;
        yield();
    }

    ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
    ELECHOUSE_cc1101.SpiStrobe(0x3B);
    Serial.printf("[TX] %s done\n", name);
}

// ── HTTP handlers ─────────────────────────────────────────
void handleHi()    { server.send(200, "text/plain", "OK: HI");    delay(50); sendCommand(CMD_HI,    "HI"); }
void handleMed()   { server.send(200, "text/plain", "OK: MED");   delay(50); sendCommand(CMD_MED,   "MED"); }
void handleLow()   { server.send(200, "text/plain", "OK: LOW");   delay(50); sendCommand(CMD_LOW,   "LOW"); }
void handleOff()   { server.send(200, "text/plain", "OK: OFF");   delay(50); sendCommand(CMD_OFF,   "OFF"); }
void handleLight() { server.send(200, "text/plain", "OK: LIGHT"); delay(50); sendCommand(CMD_LIGHT, "LIGHT"); }

// GET /dumpregs — read back all key CC1101 registers for verification
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
            uint8_t modFmt = (val >> 4) & 0x07;
            uint8_t syncMode = val & 0x07;
            snprintf(line, sizeof(line), "%-10s 0x%02X   0x%02X   MOD_FORMAT=%d(%s) SYNC_MODE=%d\n",
                r.name, r.addr, val,
                modFmt,
                modFmt==3?"OOK":modFmt==0?"2-FSK":"???",
                syncMode);
        } else if (r.addr == REG_FREND0) {
            snprintf(line, sizeof(line), "%-10s 0x%02X   0x%02X   PA_POWER=%d→PATABLE[%d]\n",
                r.name, r.addr, val, val&0x07, val&0x07);
        } else if (r.addr == REG_PKTCTRL0) {
            snprintf(line, sizeof(line), "%-10s 0x%02X   0x%02X   PKT_FMT=%d LEN_CFG=%d CRC=%d\n",
                r.name, r.addr, val,
                (val>>5)&0x03, val&0x03, (val>>2)&1);
        } else {
            snprintf(line, sizeof(line), "%-10s 0x%02X   0x%02X\n", r.name, r.addr, val);
        }
        out += line;
        Serial.print(line);
    }

    // Read PATABLE
    byte pat[8];
    for (int i = 0; i < 8; i++) pat[i] = ELECHOUSE_cc1101.SpiReadReg(REG_PATABLE);
    char pline[64];
    snprintf(pline, sizeof(pline), "PATABLE    0x3E   %02X %02X %02X %02X %02X %02X %02X %02X\n",
        pat[0],pat[1],pat[2],pat[3],pat[4],pat[5],pat[6],pat[7]);
    out += pline;
    Serial.print(pline);

    server.send(200, "text/plain", out);
}

// GET /status — re-print timing constants and key register readback at any time
void handleStatus() {
    String out = "=== Fan Remote Status ===\n";
    out += "Timing (bits @ 40kbaud = x25us):\n";
    out += "  SYNC0_H=" + String(SYNC0_H) + " (" + String(SYNC0_H*25) + "us)\n";
    out += "  SYNC0_L=" + String(SYNC0_L) + " (" + String(SYNC0_L*25) + "us)\n";
    out += "  LONG_H="  + String(LONG_H)  + " (" + String(LONG_H*25)  + "us)\n";
    out += "  SHORT_H=" + String(SHORT_H) + " (" + String(SHORT_H*25) + "us)\n";
    out += "  LONG_L="  + String(LONG_L)  + " (" + String(LONG_L*25)  + "us)\n";
    out += "  SHORT_L=" + String(SHORT_L) + " (" + String(SHORT_L*25) + "us)\n";
    out += "  GAP="     + String(GAP)     + " (" + String(GAP*25)     + "us)\n";
    out += "  REPS_DEFAULT=" + String(REPS_DEFAULT) + "  REPS_LIGHT=" + String(REPS_LIGHT) + "  REPS_OFF=" + String(REPS_OFF) + "\n";
    byte r0 = ELECHOUSE_cc1101.SpiReadReg(REG_PKTCTRL0);
    byte r1 = ELECHOUSE_cc1101.SpiReadReg(REG_MDMCFG2);
    byte r2 = ELECHOUSE_cc1101.SpiReadReg(REG_FREND0);
    byte r3 = ELECHOUSE_cc1101.SpiReadReg(REG_PATABLE);
    char buf[160];
    snprintf(buf, sizeof(buf),
        "Registers:\n  PKTCTRL0=0x%02X (want 0x02)\n  MDMCFG2 =0x%02X (want 0x30)\n"
        "  FREND0  =0x%02X (want 0x11)\n  PATABLE[0]=0x%02X (want 0x00)\n",
        r0, r1, r2, r3);
    out += buf;
    Serial.print(out);
    server.send(200, "text/plain", out);
}

// GET /carrier → 3s OOK carrier test via FIFO.
// Physical remote should FAIL during those 3s if CC1101 is transmitting.
void handleCarrier() {
    Serial.println("[CARRIER] starting 3s OOK carrier test...");

    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setMHZ(303.92);
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
            Serial.printf("[CARRIER] underflow at %lums — restarted\n", millis() - start);
        } else if ((txbytes & 0x7F) < 32) {
            ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, fill, sizeof(fill));
        }
        if (millis() - lastLog >= 500) {
            byte ms = ELECHOUSE_cc1101.SpiReadStatus(0x35);
            byte tb = ELECHOUSE_cc1101.SpiReadStatus(0x3A);
            Serial.printf("[CARRIER] t=%lums  MARC=0x%02X  TXBYTES=%d%s\n",
                millis()-start, ms, tb & 0x7F, (tb & 0x80) ? " UNDERFLOW!" : "");
            lastLog = millis();
        }
        yield();
    }

    ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
    cc1101InitTx();
    Serial.println("[CARRIER] done");
    server.send(200, "text/plain",
        "3s OOK carrier done.\nDid the physical remote FAIL during those 3 seconds?");
}

// GET /fifogate — test whether FIFO data bytes actually gate the OOK carrier.
// Emits 3s 0xFF (carrier ON), 3s 0x00 (should be OFF), 3s 0xFF (ON again).
// Watch capture unit RSSI: if RSSI drops to noise floor during 0x00 phase, FIFO gating works.
void handleFifogate() {
    server.send(200, "text/plain",
        "FIFO OOK gating test (9s).\n"
        "Watch capture unit RSSI:\n"
        "  0-3s:  0xFF carrier ON  → expect -24 dBm\n"
        "  3-6s:  0x00 carrier OFF → expect -97 dBm if gating works\n"
        "  6-9s:  0xFF carrier ON  → expect -24 dBm\n");

    cc1101InitTx();

    byte on_buf[64];  memset(on_buf,  0xFF, sizeof(on_buf));
    byte off_buf[64]; memset(off_buf, 0x00, sizeof(off_buf));

    // Prime FIFO and start TX
    ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, on_buf, 64);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SFSTXON);
    delay(3);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_STX);
    Serial.printf("[FIFOGATE] started — MARC=0x%02X\n",
        ELECHOUSE_cc1101.SpiReadStatus(0x35));

    uint32_t start = millis();
    int phase = 0;  // 0=ON 1=OFF 2=ON
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
        bool isOn = (phase % 2 == 0);
        byte* fill_buf = isOn ? on_buf : off_buf;

        byte txb = ELECHOUSE_cc1101.SpiReadStatus(0x3A);
        if (txb & 0x80) {
            // Underflow — restart TX
            ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
            ELECHOUSE_cc1101.SpiStrobe(0x3B);
            ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, fill_buf, 64);
            ELECHOUSE_cc1101.SpiStrobe(STROBE_STX);
            Serial.printf("[FIFOGATE] underflow+restart at %lums\n", millis() - start);
        } else if ((txb & 0x7F) < 48) {
            ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, fill_buf, 32);
        }
        yield();
    }

    ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
    Serial.println("[FIFOGATE] done");
}

// ── Alexa (Espalexa) callbacks ────────────────────────────
// brightness > 0 = "turn on"; 0 = "turn off".
// Speed commands only fire on "turn on" to avoid accidental OFF on "turn off Fan High".
// Light and Fan Off fire on any state change (both are effectively one-shot toggles/stops).
void alexaFanHigh(uint8_t b)  { if (b) sendCommand(CMD_HI,    "ALEXA:HI"); }
void alexaFanMed(uint8_t b)   { if (b) sendCommand(CMD_MED,   "ALEXA:MED"); }
void alexaFanLow(uint8_t b)   { if (b) sendCommand(CMD_LOW,   "ALEXA:LOW"); }
void alexaFanOff(uint8_t b)   {        sendCommand(CMD_OFF,   "ALEXA:OFF"); }
void alexaFanLight(uint8_t b) {        sendCommand(CMD_LIGHT, "ALEXA:LIGHT"); }

void handleRoot() {
    server.send(200, "text/html",
        "<!DOCTYPE html><html><head>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Fan Remote</title><style>"
        "body{font-family:sans-serif;max-width:360px;margin:40px auto;text-align:center;background:#111;color:#eee}"
        "h2{margin-bottom:24px}"
        "a{display:block;margin:10px 0;padding:16px;border-radius:8px;font-size:1.1em;"
        "text-decoration:none;color:#fff;background:#333}"
        "a:hover{background:#555}"
        "</style></head><body>"
        "<h2>Fan Remote</h2>"
        "<a href='/hi'>HI</a>"
        "<a href='/med'>MED</a>"
        "<a href='/low'>LOW</a>"
        "<a href='/off'>OFF</a>"
        "<a href='/light'>LIGHT (toggle)</a>"
        "<a href='/carrier'  style='background:#555;font-size:0.85em'>RF TEST (3s carrier)</a>"
        "<a href='/fifogate' style='background:#555;font-size:0.85em'>OOK GATE TEST (9s)</a>"
        "<a href='/status'   style='background:#555;font-size:0.85em'>STATUS / TIMING</a>"
        "<a href='/dumpregs' style='background:#555;font-size:0.85em'>DUMP REGISTERS</a>"
        "</body></html>"
    );
}

// ─────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n\n=== ESP8266 Fan Remote ===");
    Serial.printf("Timing: SYNC0_H=%d(%dus) SYNC0_L=%d(%dus) LONG_H=%d(%dus)\n",
        SYNC0_H, SYNC0_H*25, SYNC0_L, SYNC0_L*25, LONG_H, LONG_H*25);

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
    Serial.printf("PKTCTRL0=0x%02X (expect 0x02)  MDMCFG2=0x%02X (expect 0x30)\n",
                  r_pktctrl0, r_mdmcfg2);
    Serial.printf("FREND0  =0x%02X (expect 0x11)  PATABLE[0]=0x%02X (expect 0x00)\n",
                  r_frend0, r_patable0);

    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWORD);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print('.');
    }
    Serial.printf("\nIP: http://%s\n", WiFi.localIP().toString().c_str());

    server.on("/",         handleRoot);
    server.on("/hi",       handleHi);
    server.on("/med",      handleMed);
    server.on("/low",      handleLow);
    server.on("/off",      handleOff);
    server.on("/light",    handleLight);
    server.on("/carrier",  handleCarrier);
    server.on("/fifogate", handleFifogate);
    server.on("/status",   handleStatus);
    server.on("/dumpregs", handleDumpregs);

    // Alexa discovery: forward unrecognised URIs to espalexa before returning 404.
    server.onNotFound([]() {
        if (!espalexa.handleAlexaApiCall(server.uri(), server.arg(0)))
            server.send(404, "text/plain", "Not found");
    });
    espalexa.addDevice("Bedroom Fan High",   alexaFanHigh);
    espalexa.addDevice("Bedroom Fan Medium", alexaFanMed);
    espalexa.addDevice("Bedroom Fan Low",    alexaFanLow);
    espalexa.addDevice("Bedroom Fan Off",    alexaFanOff);
    espalexa.addDevice("Bedroom Light",      alexaFanLight);
    espalexa.begin(&server);  // calls server.begin() internally
    Serial.println("Ready. Alexa devices registered.");
}

void loop() {
    espalexa.loop();  // handles server.handleClient() + UPnP discovery internally
}

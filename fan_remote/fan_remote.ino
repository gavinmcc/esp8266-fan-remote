/*
 * fan_remote.ino
 *
 * ESP8266 WiFi ceiling fan remote controller
 * Controls two ceiling fans over HTTP and Alexa (Espalexa/Hue emulation).
 *
 * Fan 1 — Bedroom  (UC7070T,     303.92 MHz, OOK)
 * Fan 2 — Girls    (SMC-5060-RF, 433.92 MHz, OOK)
 *
 * Both use 13-pulse position-encoded OOK frames through the CC1101 TX FIFO.
 * The CC1101 frequency is switched per-command; calibration runs at SFSTXON.
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

// ── Bedroom fan (UC7070T, 303.92 MHz) @ 40 kbaud (25µs/bit) ──
// Frame: Header (LS LS LL SL SS LL) + Data (N×SL SS tail) × REPS
// Pulse 0 of each rep uses shorter sync timing.
#define B_SHORT_H   10   // 10×25 = 250µs  (measured 195–296µs)
#define B_LONG_H    24   // 24×25 = 600µs  (measured 522–662µs)
#define B_SYNC0_H   17   // 17×25 = 425µs  (pulse 0 only: 400–451µs)
#define B_SYNC0_L   14   // 14×25 = 350µs  (pulse 0 LOW:  294–359µs)
#define B_SHORT_L   18   // 18×25 = 450µs  (measured 294–499µs)
#define B_LONG_L    32   // 32×25 = 800µs  (measured 743–848µs)
#define B_GAP      320   // 320×25 = 8000µs inter-rep gap

#define B_REPS_DEFAULT  25
#define B_REPS_LIGHT     4
#define B_REPS_OFF      50

// N values re-captured with dedicated RX unit
#define B_CMD_HI     0
#define B_CMD_MED    1
#define B_CMD_LOW    2
#define B_CMD_OFF    4
#define B_CMD_LIGHT  5

// ── Girls fan (SMC-5060-RF, 433.92 MHz) @ 40 kbaud (25µs/bit) ──
// Frame: SL×N + SS + LL + SL×(7-N) + SS + LL + SS + L_term + GAP
// N values decoded from raw pulse captures at 433.92 MHz.
#define G_SHORT_H    9   //  9×25 = 225µs  (measured 188–349µs)
#define G_LONG_H    17   // 17×25 = 425µs  (measured 405–452µs)
#define G_SHORT_L   10   // 10×25 = 250µs  (measured 158–280µs)
#define G_LONG_L    19   // 19×25 = 475µs  (measured 398–496µs)
#define G_GAP      310   // 310×25 = 7750µs inter-rep gap

#define G_REPS_DEFAULT  20
#define G_REPS_LIGHT     4

#define G_CMD_HI     4
#define G_CMD_MED    3
#define G_CMD_LOW    0
#define G_CMD_OFF    5
#define G_CMD_LIGHT  1

static int bCommandReps(int N) {
    if (N == B_CMD_OFF)   return B_REPS_OFF;
    if (N == B_CMD_LIGHT) return B_REPS_LIGHT;
    return B_REPS_DEFAULT;
}

static int gCommandReps(int N) {
    if (N == G_CMD_LIGHT) return G_REPS_LIGHT;
    return G_REPS_DEFAULT;
}

ESP8266WebServer server(80);
Espalexa espalexa;

// ── FIFO NRZ stream builder ───────────────────────────────
// Shared buffer for both fans. Sized for the largest burst:
//   Bedroom OFF: 50 reps × ~105 bytes ≈ 5250 bytes
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

// Bedroom fan rep builder (6-pulse header + 7-pulse data)
static void buildRepB(int N) {
    streamAppend(B_SYNC0_H, true);  streamAppend(B_SYNC0_L, false);  // LS (sync)
    streamAppend(B_LONG_H,  true);  streamAppend(B_SHORT_L, false);  // LS
    streamAppend(B_LONG_H,  true);  streamAppend(B_LONG_L,  false);  // LL
    streamAppend(B_SHORT_H, true);  streamAppend(B_LONG_L,  false);  // SL
    streamAppend(B_SHORT_H, true);  streamAppend(B_SHORT_L, false);  // SS
    streamAppend(B_LONG_H,  true);  streamAppend(B_LONG_L,  false);  // LL
    for (int i = 0; i < N; i++) {
        streamAppend(B_SHORT_H, true); streamAppend(B_LONG_L, false);
    }
    streamAppend(B_SHORT_H, true);  streamAppend(B_SHORT_L, false);  // SS marker
    if (N < 5) {
        streamAppend(B_LONG_H,  true);  streamAppend(B_LONG_L, false);
        for (int i = 0; i < (4 - N); i++) {
            streamAppend(B_SHORT_H, true); streamAppend(B_LONG_L, false);
        }
        streamAppend(B_SHORT_H, true);
    } else {
        streamAppend(B_LONG_H, true);
    }
    streamAppend(B_GAP, false);
}

// Girls fan rep builder (13-pulse symmetric structure)
static void buildRepG(int N) {
    // SL×N
    for (int i = 0; i < N; i++) {
        streamAppend(G_SHORT_H, true);  streamAppend(G_LONG_L, false);
    }
    streamAppend(G_SHORT_H, true);  streamAppend(G_SHORT_L, false);  // SS
    streamAppend(G_LONG_H,  true);  streamAppend(G_LONG_L,  false);  // LL
    // SL×(7-N)
    for (int i = 0; i < (7 - N); i++) {
        streamAppend(G_SHORT_H, true);  streamAppend(G_LONG_L, false);
    }
    streamAppend(G_SHORT_H, true);  streamAppend(G_SHORT_L, false);  // SS
    streamAppend(G_LONG_H,  true);  streamAppend(G_LONG_L,  false);  // LL
    streamAppend(G_SHORT_H, true);  streamAppend(G_SHORT_L, false);  // SS
    streamAppend(G_LONG_H,  true);                                    // L_term
    streamAppend(G_GAP, false);
}

static void buildStreamB(int N) {
    streamClear();
    for (int r = 0; r < bCommandReps(N); r++) buildRepB(N);
    g_streamLen = (g_streamBit + 7) / 8;
}

static void buildStreamG(int N) {
    streamClear();
    for (int r = 0; r < gCommandReps(N); r++) buildRepG(N);
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
    ELECHOUSE_cc1101.setMHZ(303.92);  // Bedroom fan default
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

// ─────────────────────────────────────────────────────────
// Core FIFO TX. Stream must already be built in g_stream/g_streamLen.
// freq_mhz: carrier frequency to set before transmission.
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
    ELECHOUSE_cc1101.SpiStrobe(0x3B);
}

void sendCommandB(int N, const char* name) {
    buildStreamB(N);
    Serial.printf("[TX-B] %s N=%d %d reps %d bytes\n", name, N, bCommandReps(N), g_streamLen);
    fifoTransmit(303.92, "TX-B");
    Serial.printf("[TX-B] %s done\n", name);
}

void sendCommandG(int N, const char* name) {
    buildStreamG(N);
    Serial.printf("[TX-G] %s N=%d %d reps %d bytes\n", name, N, gCommandReps(N), g_streamLen);
    fifoTransmit(433.92, "TX-G");
    ELECHOUSE_cc1101.setMHZ(303.92);  // restore default frequency
    Serial.printf("[TX-G] %s done\n", name);
}

// ── HTTP handlers — Bedroom fan ───────────────────────────
void handleHi()    { server.send(200, "text/plain", "OK: HI");    delay(50); sendCommandB(B_CMD_HI,    "HI"); }
void handleMed()   { server.send(200, "text/plain", "OK: MED");   delay(50); sendCommandB(B_CMD_MED,   "MED"); }
void handleLow()   { server.send(200, "text/plain", "OK: LOW");   delay(50); sendCommandB(B_CMD_LOW,   "LOW"); }
void handleOff()   { server.send(200, "text/plain", "OK: OFF");   delay(50); sendCommandB(B_CMD_OFF,   "OFF"); }
void handleLight() { server.send(200, "text/plain", "OK: LIGHT"); delay(50); sendCommandB(B_CMD_LIGHT, "LIGHT"); }

// ── HTTP handlers — Girls fan ─────────────────────────────
void handleGHi()    { server.send(200, "text/plain", "OK: GIRLS HI");    delay(50); sendCommandG(G_CMD_HI,    "G:HI"); }
void handleGMed()   { server.send(200, "text/plain", "OK: GIRLS MED");   delay(50); sendCommandG(G_CMD_MED,   "G:MED"); }
void handleGLow()   { server.send(200, "text/plain", "OK: GIRLS LOW");   delay(50); sendCommandG(G_CMD_LOW,   "G:LOW"); }
void handleGOff()   { server.send(200, "text/plain", "OK: GIRLS OFF");   delay(50); sendCommandG(G_CMD_OFF,   "G:OFF"); }
void handleGLight() { server.send(200, "text/plain", "OK: GIRLS LIGHT"); delay(50); sendCommandG(G_CMD_LIGHT, "G:LIGHT"); }

// GET /dumpregs — read back key CC1101 registers
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
                r.name, r.addr, val, modFmt,
                modFmt==3?"OOK":modFmt==0?"2-FSK":"???", syncMode);
        } else if (r.addr == REG_FREND0) {
            snprintf(line, sizeof(line), "%-10s 0x%02X   0x%02X   PA_POWER=%d→PATABLE[%d]\n",
                r.name, r.addr, val, val&0x07, val&0x07);
        } else if (r.addr == REG_PKTCTRL0) {
            snprintf(line, sizeof(line), "%-10s 0x%02X   0x%02X   PKT_FMT=%d LEN_CFG=%d CRC=%d\n",
                r.name, r.addr, val, (val>>5)&0x03, val&0x03, (val>>2)&1);
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
        pat[0],pat[1],pat[2],pat[3],pat[4],pat[5],pat[6],pat[7]);
    out += pline;
    Serial.print(pline);
    server.send(200, "text/plain", out);
}

// GET /status — timing constants and key register readback
void handleStatus() {
    String out = "=== Fan Remote Status ===\n";
    out += "Bedroom fan (UC7070T, 303.92 MHz, 40kbaud):\n";
    out += "  B_SYNC0_H=" + String(B_SYNC0_H) + " (" + String(B_SYNC0_H*25) + "us)\n";
    out += "  B_LONG_H="  + String(B_LONG_H)  + " (" + String(B_LONG_H*25)  + "us)\n";
    out += "  B_SHORT_H=" + String(B_SHORT_H) + " (" + String(B_SHORT_H*25) + "us)\n";
    out += "  B_LONG_L="  + String(B_LONG_L)  + " (" + String(B_LONG_L*25)  + "us)\n";
    out += "  B_SHORT_L=" + String(B_SHORT_L) + " (" + String(B_SHORT_L*25) + "us)\n";
    out += "  B_GAP="     + String(B_GAP)     + " (" + String(B_GAP*25)     + "us)\n";
    out += "  REPS: default=" + String(B_REPS_DEFAULT) + " off=" + String(B_REPS_OFF) + " light=" + String(B_REPS_LIGHT) + "\n";
    out += "Girls fan (SMC-5060-RF, 433.92 MHz, 40kbaud):\n";
    out += "  G_SHORT_H=" + String(G_SHORT_H) + " (" + String(G_SHORT_H*25) + "us)\n";
    out += "  G_LONG_H="  + String(G_LONG_H)  + " (" + String(G_LONG_H*25)  + "us)\n";
    out += "  G_SHORT_L=" + String(G_SHORT_L) + " (" + String(G_SHORT_L*25) + "us)\n";
    out += "  G_LONG_L="  + String(G_LONG_L)  + " (" + String(G_LONG_L*25)  + "us)\n";
    out += "  G_GAP="     + String(G_GAP)     + " (" + String(G_GAP*25)     + "us)\n";
    out += "  REPS: default=" + String(G_REPS_DEFAULT) + " light=" + String(G_REPS_LIGHT) + "\n";
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

// GET /carrier → 3s carrier test at 303.92 MHz
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

// GET /fifogate — FIFO OOK gating sanity check (9s)
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

    ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, on_buf, 64);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SFSTXON);
    delay(3);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_STX);
    Serial.printf("[FIFOGATE] started — MARC=0x%02X\n", ELECHOUSE_cc1101.SpiReadStatus(0x35));

    uint32_t start = millis();
    int phase = 0;
    uint32_t phase_end = start + 3000;
    Serial.println("[FIFOGATE] t=0s  phase=ON (0xFF)");

    while (millis() - start < 9000) {
        uint32_t now = millis();
        if (now >= phase_end) {
            phase++;
            phase_end += 3000;
            bool isOn = (phase % 2 == 0);
            Serial.printf("[FIFOGATE] t=%lums  phase=%s\n", now-start, isOn?"ON (0xFF)":"OFF (0x00)");
        }
        bool isOn = (phase % 2 == 0);
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

// ── Alexa callbacks — Bedroom fan ─────────────────────────
void alexaFanHigh(uint8_t b)  { if (b) sendCommandB(B_CMD_HI,    "ALEXA:BED:HI"); }
void alexaFanMed(uint8_t b)   { if (b) sendCommandB(B_CMD_MED,   "ALEXA:BED:MED"); }
void alexaFanLow(uint8_t b)   { if (b) sendCommandB(B_CMD_LOW,   "ALEXA:BED:LOW"); }
void alexaFanOff(uint8_t b)   {        sendCommandB(B_CMD_OFF,   "ALEXA:BED:OFF"); }
void alexaFanLight(uint8_t b) {        sendCommandB(B_CMD_LIGHT, "ALEXA:BED:LIGHT"); }

// ── Alexa callbacks — Girls fan ───────────────────────────
void alexaGirlsHigh(uint8_t b)  { if (b) sendCommandG(G_CMD_HI,    "ALEXA:G:HI"); }
void alexaGirlsMed(uint8_t b)   { if (b) sendCommandG(G_CMD_MED,   "ALEXA:G:MED"); }
void alexaGirlsLow(uint8_t b)   { if (b) sendCommandG(G_CMD_LOW,   "ALEXA:G:LOW"); }
void alexaGirlsOff(uint8_t b)   {        sendCommandG(G_CMD_OFF,   "ALEXA:G:OFF"); }
void alexaGirlsLight(uint8_t b) {        sendCommandG(G_CMD_LIGHT, "ALEXA:G:LIGHT"); }

void handleRoot() {
    server.send(200, "text/html",
        "<!DOCTYPE html><html><head>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Fan Remote</title><style>"
        "body{font-family:sans-serif;max-width:420px;margin:40px auto;text-align:center;background:#111;color:#eee}"
        "h2{margin-bottom:8px} h3{margin:20px 0 8px;color:#aaa}"
        "a{display:block;margin:8px 0;padding:14px;border-radius:8px;font-size:1.05em;"
        "text-decoration:none;color:#fff;background:#333}"
        "a:hover{background:#555}"
        ".dim{background:#252525;font-size:0.85em}"
        "</style></head><body>"
        "<h2>Fan Remote</h2>"
        "<h3>Bedroom (303 MHz)</h3>"
        "<a href='/hi'>HI</a>"
        "<a href='/med'>MED</a>"
        "<a href='/low'>LOW</a>"
        "<a href='/off'>OFF</a>"
        "<a href='/light'>LIGHT (toggle)</a>"
        "<h3>Girls (433 MHz)</h3>"
        "<a href='/girls/hi'>HI</a>"
        "<a href='/girls/med'>MED</a>"
        "<a href='/girls/low'>LOW</a>"
        "<a href='/girls/off'>OFF</a>"
        "<a href='/girls/light'>LIGHT (toggle)</a>"
        "<h3>Diagnostics</h3>"
        "<a href='/carrier'  class='dim'>RF TEST (3s carrier, 303 MHz)</a>"
        "<a href='/fifogate' class='dim'>OOK GATE TEST (9s)</a>"
        "<a href='/status'   class='dim'>STATUS / TIMING</a>"
        "<a href='/dumpregs' class='dim'>DUMP REGISTERS</a>"
        "</body></html>"
    );
}

// ─────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n\n=== ESP8266 Fan Remote (dual-band) ===");

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

    // Bedroom fan routes
    server.on("/",          handleRoot);
    server.on("/hi",        handleHi);
    server.on("/med",       handleMed);
    server.on("/low",       handleLow);
    server.on("/off",       handleOff);
    server.on("/light",     handleLight);
    // Girls fan routes
    server.on("/girls/hi",    handleGHi);
    server.on("/girls/med",   handleGMed);
    server.on("/girls/low",   handleGLow);
    server.on("/girls/off",   handleGOff);
    server.on("/girls/light", handleGLight);
    // Diagnostics
    server.on("/carrier",   handleCarrier);
    server.on("/fifogate",  handleFifogate);
    server.on("/status",    handleStatus);
    server.on("/dumpregs",  handleDumpregs);

    server.onNotFound([]() {
        if (!espalexa.handleAlexaApiCall(server.uri(), server.arg(0)))
            server.send(404, "text/plain", "Not found");
    });

    // Bedroom fan Alexa devices
    espalexa.addDevice("Bedroom Fan High",   alexaFanHigh);
    espalexa.addDevice("Bedroom Fan Medium", alexaFanMed);
    espalexa.addDevice("Bedroom Fan Low",    alexaFanLow);
    espalexa.addDevice("Bedroom Fan Off",    alexaFanOff);
    espalexa.addDevice("Bedroom Light",      alexaFanLight);
    // Girls fan Alexa devices
    espalexa.addDevice("Girls Fan High",     alexaGirlsHigh);
    espalexa.addDevice("Girls Fan Medium",   alexaGirlsMed);
    espalexa.addDevice("Girls Fan Low",      alexaGirlsLow);
    espalexa.addDevice("Girls Fan Off",      alexaGirlsOff);
    espalexa.addDevice("Girls Light",        alexaGirlsLight);

    espalexa.begin(&server);
    Serial.println("Ready. 10 Alexa devices registered (Bedroom + Girls).");
}

void loop() {
    espalexa.loop();
}

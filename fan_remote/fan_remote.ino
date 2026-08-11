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
 *   CC1101 GDO0 -> GPIO4 (D2) — async TX data input
 *
 * HTTP endpoints:
 *   GET /hi    /med   /low   /off   /light  — send command
 *   GET /try?n=X   — send exactly N=X (0-5) for manual testing
 *   GET /carrier   — 3s OOK carrier test (jams physical remote if RF OK)
 *   GET /carrier2  — 5s 2-FSK carrier test
 */

#include <SPI.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include "secrets.h"

const char* SSID     = WIFI_SSID;
const char* PASSWORD = WIFI_PASSWORD;

#define PIN_CSN      15   // GPIO15 (D8)
#define GDO0_TX_PIN   4   // GPIO4  (D2) — CC1101 async DATA input

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
//   8 reps = ~144ms, in "short press" territory. N=5 decodes reliably at range so few reps are OK.
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

// ── FIFO NRZ stream builder ───────────────────────────────
// Buffer for 15 reps × ~105 bytes = ~1575 bytes NRZ bitstream.
// 1-bits = OOK carrier on, 0-bits = carrier off.
static byte g_stream[1600];
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
static void ICACHE_RAM_ATTR fastStrobe(byte cmd) {
    digitalWrite(PIN_CSN, LOW);
    SPI.transfer(cmd);
    digitalWrite(PIN_CSN, HIGH);
}

// Strobe-based OOK helper: STX (on) or SIDLE (off).
// SIDLE is used for "off" because SFSTXON from TX is ignored in async mode (MARC stays 0x13).
// SFSTXON-gating approach: issue SFSTXON at the END of each "off" period so the PLL
// is locked (FSTXON state) before STX fires. STX from FSTXON is instant — no startup lag.
// strobeBit(false, us): SIDLE → wait(us - PLL_LOCK_US) → SFSTXON [PLL starts locking]
// strobeBit(true,  us): wait(PLL_LOCK_US) [PLL finishes] → STX → wait(us) [carrier]
// Gap at receiver = (us-40 wait) + (40 lock) = us µs. Carrier = us µs. Both correct.
#define PLL_LOCK_US 40    // time between SFSTXON and STX; PLL locks ~76µs so this is slightly short but works
static void ICACHE_RAM_ATTR strobeBit(bool hi, uint32_t us) {
    noInterrupts();
    if (hi) {
        delayMicroseconds(PLL_LOCK_US);  // let SFSTXON from prev "off" finish locking
        fastStrobe(STROBE_STX);           // FSTXON → TX instantly (PLL already locked)
        delayMicroseconds(us);            // full carrier for 'us' µs
    } else {
        fastStrobe(STROBE_SIDLE);
        delayMicroseconds(us > PLL_LOCK_US ? us - PLL_LOCK_US : 0);
        fastStrobe(STROBE_SFSTXON);       // kick off PLL lock; completes during next strobeBit(true)
    }
    interrupts();
}

// Strobe-based OOK TX — uses STX/SFSTXON state transitions for PA gating.
// In async TX mode (no FIFO drain), GDO0=HIGH keeps CC1101 happy (1-bit always).
// Ext PA follows TX-state signal on GDO2 (default IOCFG2 from library Init).
// This works if FSTXON→TX ext-PA enable response is fast enough for 250µs pulses.
void sendCommandStrobe(int N, const char* name) {
    int reps = commandReps(N);
    Serial.printf("[TX] %s N=%d strobe %d reps\n", name, N, reps);

    // Use cc1101InitTx() base (IOCFG2=0x56 = inverted PA_PD: GDO2 LOW in IDLE/FSTXON, HIGH in TX).
    // MCSM0=0x00: never auto-calibrate (default 0x18 triggers 720µs cal on every IDLE→STX).
    // With SFSTXON-gating in strobeBit(), PLL is locked via FSTXON before each STX.
    cc1101InitTx();
    ELECHOUSE_cc1101.SpiWriteReg(0x18, 0x00);  // MCSM0: never auto-calibrate
    // 0x32 = async serial TX (PKT_FORMAT=11), infinite length, no whitening.
    // GDO0=HIGH → CC1101 OOK=1 → continuous carrier during TX state (vs 0x62=random TX which sends 50% duty-cycle noise).
    ELECHOUSE_cc1101.SpiWriteReg(REG_PKTCTRL0, 0x32);
    pinMode(GDO0_TX_PIN, OUTPUT);
    digitalWrite(GDO0_TX_PIN, HIGH);

    // Calibrate PLL once → FSTXON. SIDLE gating works without pre-warm:
    // ext PA is re-enabled the moment CC1101 transitions to TX (XOSC stays on in IDLE).
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SFSTXON);
    delay(5);
    Serial.printf("[TX] post-cal MARC=0x%02X\n", ELECHOUSE_cc1101.SpiReadStatus(0x35));

    for (int r = 0; r < reps; r++) {
        // Header pulse 0: LS with SYNC0 timing
        strobeBit(true,  SYNC0_H * 25);  strobeBit(false, SYNC0_L * 25);
        // Header pulses 1-5: LS LL SL SS LL
        strobeBit(true,  LONG_H  * 25);  strobeBit(false, SHORT_L * 25);
        strobeBit(true,  LONG_H  * 25);  strobeBit(false, LONG_L  * 25);
        strobeBit(true,  SHORT_H * 25);  strobeBit(false, LONG_L  * 25);
        strobeBit(true,  SHORT_H * 25);  strobeBit(false, SHORT_L * 25);
        strobeBit(true,  LONG_H  * 25);  strobeBit(false, LONG_L  * 25);
        // Data: N × SL
        for (int i = 0; i < N; i++) {
            strobeBit(true, SHORT_H * 25);  strobeBit(false, LONG_L * 25);
        }
        // SS command marker
        strobeBit(true,  SHORT_H * 25);  strobeBit(false, SHORT_L * 25);
        // Tail
        if (N < 5) {
            strobeBit(true,  LONG_H  * 25);  strobeBit(false, LONG_L * 25);
            for (int i = 0; i < (4 - N); i++) {
                strobeBit(true, SHORT_H * 25);  strobeBit(false, LONG_L * 25);
            }
            noInterrupts();
            delayMicroseconds(PLL_LOCK_US);   // let SFSTXON from last strobeBit(false) finish
            fastStrobe(STROBE_STX);
            delayMicroseconds(SHORT_H * 25);
            interrupts();
        } else {
            noInterrupts();
            delayMicroseconds(PLL_LOCK_US);   // let SFSTXON from last strobeBit(false) finish
            fastStrobe(STROBE_STX);
            delayMicroseconds(LONG_H * 25);
            interrupts();
        }
        // GAP: 8ms SIDLE then SFSTXON so next rep's first strobeBit(true) finds PLL locked.
        noInterrupts(); fastStrobe(STROBE_SIDLE); interrupts();
        delay(8);
        noInterrupts(); fastStrobe(STROBE_SFSTXON); interrupts();
    }

    fastStrobe(STROBE_SIDLE);
    ELECHOUSE_cc1101.SpiWriteReg(REG_PKTCTRL0, 0x02);
    pinMode(GDO0_TX_PIN, INPUT);
    cc1101InitTx();  // restore full config
    Serial.printf("[TX] %s done\n", name);
}

// Async OOK helper: set GDO0 high/low for `us` µs, interrupts off during pulse.
// Interrupts re-enabled between pulses so WiFi runs in the dead-time between symbols.
static void ICACHE_RAM_ATTR asmBit(bool hi, uint32_t us) {
    noInterrupts();
    digitalWrite(GDO0_TX_PIN, hi ? HIGH : LOW);
    delayMicroseconds(us);
    interrupts();
}

// Async GDO0 bit-bang TX.
// In async serial mode (PKTCTRL0=0x32) the CC1101 treats GDO0 as the OOK data input:
//   1 → internal PA on; IOCFG2=0x56 (inverted PA_PD) also enables external PA → full carrier
//   0 → both PAs off → silence
// PLL stays locked via STX throughout; only the PA gates per bit.
// WiFi runs during the 8 ms inter-rep gap, so the connection stays alive.
void sendCommandAsync(int N, const char* name) {
    Serial.printf("[TX] %s N=%d async GDO0 %d reps\n", name, N, commandReps(N));

    ELECHOUSE_cc1101.SpiWriteReg(REG_PKTCTRL0, 0x32);  // async serial TX (PKT_FORMAT=11), infinite length

    pinMode(GDO0_TX_PIN, OUTPUT);
    digitalWrite(GDO0_TX_PIN, LOW);  // PA off while PLL calibrates
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SFSTXON);
    delay(3);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_STX);
    delayMicroseconds(100);

    for (int r = 0; r < commandReps(N); r++) {
        // Header pulse 0: LS with SYNC0 timing (shorter HIGH measured on physical remote)
        asmBit(true,  SYNC0_H * 25);  asmBit(false, SYNC0_L * 25);
        // Header pulses 1-5: LS LL SL SS LL
        asmBit(true,  LONG_H  * 25);  asmBit(false, SHORT_L * 25);  // LS
        asmBit(true,  LONG_H  * 25);  asmBit(false, LONG_L  * 25);  // LL
        asmBit(true,  SHORT_H * 25);  asmBit(false, LONG_L  * 25);  // SL
        asmBit(true,  SHORT_H * 25);  asmBit(false, SHORT_L * 25);  // SS
        asmBit(true,  LONG_H  * 25);  asmBit(false, LONG_L  * 25);  // LL
        // Data: N × SL
        for (int i = 0; i < N; i++) {
            asmBit(true, SHORT_H * 25);  asmBit(false, LONG_L * 25);
        }
        // SS command marker
        asmBit(true,  SHORT_H * 25);  asmBit(false, SHORT_L * 25);
        // Tail
        if (N < 5) {
            asmBit(true,  LONG_H  * 25);  asmBit(false, LONG_L * 25);  // LL
            for (int i = 0; i < (4 - N); i++) {
                asmBit(true, SHORT_H * 25);  asmBit(false, LONG_L * 25);  // SL×(4-N)
            }
            // Terminal SHORT_H: hold high, then drop for GAP
            noInterrupts();
            digitalWrite(GDO0_TX_PIN, HIGH);
            delayMicroseconds(SHORT_H * 25);
            interrupts();
        } else {
            // N=5: terminal LONG_H
            noInterrupts();
            digitalWrite(GDO0_TX_PIN, HIGH);
            delayMicroseconds(LONG_H * 25);
            interrupts();
        }
        // GAP: 8 ms LOW with interrupts enabled — WiFi runs here
        noInterrupts(); digitalWrite(GDO0_TX_PIN, LOW); interrupts();
        delay(8);
    }

    digitalWrite(GDO0_TX_PIN, LOW);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
    ELECHOUSE_cc1101.SpiWriteReg(REG_PKTCTRL0, 0x02);  // restore FIFO mode
    pinMode(GDO0_TX_PIN, INPUT);
    Serial.printf("[TX] %s done\n", name);
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
    // 0x09 (lock detector) was tried but causes VCO leakage in FSTXON: the ext PA, enabled
    // during FSTXON, amplifies VCO signal into a low-level carrier that corrupts off-periods.
    ELECHOUSE_cc1101.SpiWriteReg(0x00,         0x56);  // IOCFG2 = inverted PA_PD

    byte pa[8]; memset(pa, 0x00, sizeof(pa)); pa[1] = 0xC0;
    ELECHOUSE_cc1101.SpiWriteBurstReg(REG_PATABLE, pa, 8);

    ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
}

// ─────────────────────────────────────────────────────────
// FIFO NRZ OOK TX: pre-build bitstream, stream through FIFO while PA stays on.
// PA ramps up once on entry to TX state; FIFO data bits gate OOK on/off.
// MDMCFG2=0x30 (OOK) required — with 0x20 (reserved) gating was broken.
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
    struct { const char* name; uint8_t addr; uint8_t expect; bool status; } regs[] = {
        {"IOCFG0",   0x00, 0x3F, false},
        {"PKTCTRL0", 0x08, 0x02, false},
        {"PKTCTRL1", 0x07, 0x04, false},
        {"MDMCFG4",  0x10, 0x00, false},
        {"MDMCFG3",  0x11, 0x00, false},
        {"MDMCFG2",  0x12, 0x30, false},
        {"MDMCFG1",  0x13, 0x00, false},
        {"FREND0",   0x22, 0x11, false},
        {"MCSM0",    0x18, 0x00, false},
        {"MCSM1",    0x17, 0x00, false},
        {"FREQ2",    0x0D, 0x00, false},
        {"FREQ1",    0x0E, 0x00, false},
        {"FREQ0",    0x0F, 0x00, false},
        {"CHANNR",   0x0A, 0x00, false},
        {"FSCTRL1",  0x0B, 0x00, false},
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

// GET /carrier_ook — 2s ON/OFF alternation using STX/SFSTXON strobe gating.
// OFF phase uses SFSTXON (PA off, PLL locked) rather than 0x00 FIFO data
// (FIFO gating proved non-functional on two CC1101 modules).
void handleCarrierOok() {
    server.send(200, "text/plain",
        "OOK strobe gating test (8 seconds).\n\n"
        "WATCH SERIAL MONITOR — it will tell you exactly when to press the remote.\n\n"
        "Schedule:\n"
        "  0-5s   CARRIER ON   (jammed)\n"
        "  5-10s  CARRIER OFF  (SFSTXON) <- PRESS PHYSICAL REMOTE during this window\n"
        "  10-15s CARRIER ON\n"
        "  15-20s CARRIER OFF  (SFSTXON) <- PRESS AGAIN if first window missed\n\n"
        "If remote works during OFF windows → strobe PA gating works.\n"
        "If remote stays jammed during OFF windows → something else is wrong.");

    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setMHZ(303.92);
    ELECHOUSE_cc1101.setModulation(2);
    ELECHOUSE_cc1101.setPA(10);
    ELECHOUSE_cc1101.setDRate(40);
    ELECHOUSE_cc1101.SpiWriteReg(REG_MDMCFG2,  0x30);
    ELECHOUSE_cc1101.SpiWriteReg(REG_FREND0,   0x11);
    ELECHOUSE_cc1101.SpiWriteReg(REG_PKTCTRL0, 0x02);
    byte pa[8]; memset(pa, 0x00, sizeof(pa)); pa[1] = 0xC0;
    ELECHOUSE_cc1101.SpiWriteBurstReg(REG_PATABLE, pa, 8);

    byte on_buf[64]; memset(on_buf, 0xFF, sizeof(on_buf));

    // Calibrate PLL, pre-fill FIFO, start TX
    ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, on_buf, 64);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SFSTXON);
    delay(2);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_STX);
    delay(10);
    Serial.printf("[OOK_STROBE] MARCSTATE=0x%02X  starting 8s test\n",
        ELECHOUSE_cc1101.SpiReadStatus(0x35));

    uint32_t start = millis();
    bool carrier_on = true;
    uint32_t phase_end = start + 5000;
    Serial.println("[OOK_STROBE] t=0s  CARRIER ON");

    while (millis() - start < 20000) {
        if (millis() >= phase_end) {
            carrier_on = !carrier_on;
            phase_end += 5000;
            uint32_t t = millis() - start;
            if (carrier_on) {
                // Top up FIFO without overflow, then restart TX
                byte rem = ELECHOUSE_cc1101.SpiReadStatus(0x3A) & 0x7F;
                if (rem > 64) rem = 64;
                byte fill = 64 - rem;
                if (fill > 0) ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, on_buf, fill);
                ELECHOUSE_cc1101.SpiStrobe(STROBE_STX);
                Serial.printf("[OOK_STROBE] t=%lums  CARRIER ON\n", t);
            } else {
                // SFSTXON: PA off, PLL stays locked in FSTXON
                ELECHOUSE_cc1101.SpiStrobe(STROBE_SFSTXON);
                Serial.printf("[OOK_STROBE] t=%lums  CARRIER OFF (SFSTXON) — PRESS REMOTE NOW\n", t);
            }
        }

        // During ON phase: keep FIFO fed to prevent underflow
        if (carrier_on) {
            byte txb = ELECHOUSE_cc1101.SpiReadStatus(0x3A);
            if (txb & 0x80) {
                ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
                ELECHOUSE_cc1101.SpiStrobe(0x3B);
                ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, on_buf, 64);
                ELECHOUSE_cc1101.SpiStrobe(STROBE_STX);
            } else if ((txb & 0x7F) < 32) {
                ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, on_buf, 32);
            }
        }
        yield();
    }

    ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
    cc1101InitTx();
    Serial.println("[OOK_STROBE] done");
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

// GET /try?n=X — send exactly N=X for manual brute-force testing
void handleTry() {
    if (!server.hasArg("n")) { server.send(400, "text/plain", "missing ?n="); return; }
    int n = server.arg("n").toInt();
    if (n < 0 || n > 5) { server.send(400, "text/plain", "n must be 0-5"); return; }
    char label[16]; snprintf(label, sizeof(label), "TRY(N=%d)", n);
    sendCommandStrobe(n, label);
    server.send(200, "text/plain", label);
}

// GET /lighttry?reps=N — send LIGHT (N=5) with exactly N reps, bypassing commandReps().
// Use to probe the boundary between "toggle" (too few reps = miss) and "hold/dim" (too many).
// Physical remote sends 4 reps (~84ms). Report what you observe for each value.
void handleLightTry() {
    if (!server.hasArg("reps")) { server.send(400, "text/plain", "missing ?reps="); return; }
    int reps = server.arg("reps").toInt();
    if (reps < 1 || reps > 250) { server.send(400, "text/plain", "reps must be 1-250"); return; }

    int durationMs = reps * 21;
    char resp[80];
    snprintf(resp, sizeof(resp), "LIGHT %d reps (~%dms) sending...", reps, durationMs);
    // Send response BEFORE TX so the TCP connection closes cleanly.
    // Long TX (>1s) hangs the HTTP connection open and corrupts the lwIP stack.
    server.send(200, "text/plain", resp);
    // Give the TCP stack 300ms to fully close the connection before blocking TX.
    // Without this, the lwIP stack accumulates pending events during long TX and crashes.
    delay(300);

    Serial.printf("[LIGHTTRY] LIGHT N=5, %d reps (~%dms burst)\n", reps, durationMs);

    cc1101InitTx();
    ELECHOUSE_cc1101.SpiWriteReg(0x18, 0x00);
    ELECHOUSE_cc1101.SpiWriteReg(REG_PKTCTRL0, 0x32);
    pinMode(GDO0_TX_PIN, OUTPUT);
    digitalWrite(GDO0_TX_PIN, HIGH);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SFSTXON);
    delay(5);

    for (int r = 0; r < reps; r++) {
        strobeBit(true,  SYNC0_H * 25);  strobeBit(false, SYNC0_L * 25);
        strobeBit(true,  LONG_H  * 25);  strobeBit(false, SHORT_L * 25);
        strobeBit(true,  LONG_H  * 25);  strobeBit(false, LONG_L  * 25);
        strobeBit(true,  SHORT_H * 25);  strobeBit(false, LONG_L  * 25);
        strobeBit(true,  SHORT_H * 25);  strobeBit(false, SHORT_L * 25);
        strobeBit(true,  LONG_H  * 25);  strobeBit(false, LONG_L  * 25);
        for (int i = 0; i < 5; i++) {
            strobeBit(true, SHORT_H * 25);  strobeBit(false, LONG_L * 25);
        }
        strobeBit(true,  SHORT_H * 25);  strobeBit(false, SHORT_L * 25);
        noInterrupts();
        delayMicroseconds(PLL_LOCK_US);
        fastStrobe(STROBE_STX);
        delayMicroseconds(LONG_H * 25);
        interrupts();
        noInterrupts(); fastStrobe(STROBE_SIDLE); interrupts();
        delay(8);
        noInterrupts(); fastStrobe(STROBE_SFSTXON); interrupts();
        // Every 25 reps give WiFi stack a longer breath to prevent watchdog / lwIP issues
        if ((r & 0x1F) == 0x1F) delay(50);
    }

    fastStrobe(STROBE_SIDLE);
    ELECHOUSE_cc1101.SpiWriteReg(REG_PKTCTRL0, 0x02);
    pinMode(GDO0_TX_PIN, INPUT);
    cc1101InitTx();
    Serial.printf("[LIGHTTRY] done\n");
}

// GET /strobe_test?ms=X — emit 20 STX pulses of X ms each (default 10ms), 10ms off gap.
// Used to find minimum pulse width at which the PA actually emits detectably.
void handleStrobeTest() {
    int pw = server.hasArg("ms") ? server.arg("ms").toInt() : 10;
    if (pw < 1) pw = 1; if (pw > 500) pw = 500;
    Serial.printf("[STROBE_TEST] %d pulses, %dms ON / 10ms OFF\n", 20, pw);

    byte fifobuf[64]; memset(fifobuf, 0xFF, sizeof(fifobuf));
    ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, fifobuf, 64);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SFSTXON);
    delay(5);

    for (int i = 0; i < 20; i++) {
        ELECHOUSE_cc1101.SpiStrobe(STROBE_STX);
        delay(pw);
        ELECHOUSE_cc1101.SpiStrobe(STROBE_SFSTXON);
        delay(10);
        // refill FIFO
        byte rem = ELECHOUSE_cc1101.SpiReadStatus(0x3A) & 0x7F;
        if (rem > 64) rem = 64;
        byte fill = 64 - rem;
        if (fill > 0) ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, fifobuf, fill);
    }
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
    Serial.println("[STROBE_TEST] done");
    server.send(200, "text/plain", "strobe test done");
}

// GET /ooktest — OOK gating test at 0.3 kbaud (one bit = 3.3ms).
// At this rate, even a slow PA enable RC filter would produce visible RSSI swings.
// Sends repeating pattern: 8 bytes 0xFF (ON ~26ms) then 8 bytes 0x00 (OFF ~26ms).
// 50ms RSSI polls on capture unit should see clear on/off if gating works at all.
void handleOoktest() {
    server.send(200, "text/plain",
        "OOK gating test at 0.3 kbaud (~3ms/bit, ~26ms per byte block).\n"
        "Watch capture unit RSSI (100ms poll). Alternating 8×0xFF / 8×0x00 blocks.\n"
        "Should see RSSI oscillate if OOK gating works at low data rate.\n");

    cc1101InitTx();
    ELECHOUSE_cc1101.setDRate(0.3);  // 0.3 kbaud = 3.3ms per bit

    byte on_buf[8];  memset(on_buf,  0xFF, sizeof(on_buf));
    byte off_buf[8]; memset(off_buf, 0x00, sizeof(off_buf));

    // Prime FIFO, calibrate, start TX
    ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, on_buf, 8);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SFSTXON);
    delay(3);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_STX);
    delay(5);
    Serial.printf("[OOKTEST] started MARC=0x%02X  0.3kbaud\n",
        ELECHOUSE_cc1101.SpiReadStatus(0x35));

    uint32_t start = millis();
    bool phase = true;  // true=0xFF, false=0x00
    int repsDone = 0;

    while (millis() - start < 9000) {
        byte txb = ELECHOUSE_cc1101.SpiReadStatus(0x3A);
        if (txb & 0x80) {
            // Underflow — restart
            ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
            ELECHOUSE_cc1101.SpiStrobe(0x3B);
            phase = !phase;
            byte* buf = phase ? on_buf : off_buf;
            ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, buf, 8);
            ELECHOUSE_cc1101.SpiStrobe(STROBE_STX);
            repsDone++;
            if (repsDone % 4 == 0)
                Serial.printf("[OOKTEST] t=%lums phase=%s\n",
                    millis()-start, phase?"ON(FF)":"OFF(00)");
        }
        yield();
    }

    ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
    ELECHOUSE_cc1101.setDRate(40);  // restore 40 kbaud
    Serial.println("[OOKTEST] done");
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

// GET /marctest — confirm fastStrobe(STX) actually reaches TX state (MARCSTATE=0x13).
// Uses 10ms pulse so RSSI is measurable on capture unit. No noInterrupts().
void handleMarctest() {
    cc1101InitTx();
    byte fifobuf[64]; memset(fifobuf, 0xFF, sizeof(fifobuf));
    ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, fifobuf, 64);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SFSTXON);
    delay(3);
    byte marc_pre = ELECHOUSE_cc1101.SpiReadStatus(0x35);

    fastStrobe(STROBE_STX);
    delay(10);
    byte marc_tx  = ELECHOUSE_cc1101.SpiReadStatus(0x35);
    fastStrobe(STROBE_SFSTXON);
    delay(2);
    byte marc_off = ELECHOUSE_cc1101.SpiReadStatus(0x35);

    ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);

    char out[128];
    snprintf(out, sizeof(out),
        "pre-STX MARC=0x%02X (want 0x12=FSTXON)\n"
        "post-STX MARC=0x%02X (want 0x13=TX)\n"
        "post-SFSTXON MARC=0x%02X (want 0x12=FSTXON)\n",
        marc_pre, marc_tx, marc_off);
    Serial.print(out);
    server.send(200, "text/plain", out);
}

// ── RF emission diagnostics ───────────────────────────────

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

// GET /carrier2 → 5s 2-FSK carrier test.
void handleCarrier2() {
    Serial.println("[CARRIER2] starting 5s 2-FSK carrier test...");

    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setMHZ(303.92);
    ELECHOUSE_cc1101.setModulation(0);
    ELECHOUSE_cc1101.setPA(10);
    ELECHOUSE_cc1101.setDRate(1.0);
    ELECHOUSE_cc1101.SpiWriteReg(REG_PKTCTRL0, 0x02);

    byte buf[64]; memset(buf, 0xAA, sizeof(buf));
    ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, buf, sizeof(buf));
    ELECHOUSE_cc1101.SpiStrobe(STROBE_STX);

    delay(10);
    byte marc0 = ELECHOUSE_cc1101.SpiReadStatus(0x35);
    Serial.printf("[CARRIER2] MARCSTATE after STX = 0x%02X (expect 0x13=TX)\n", marc0);

    uint32_t start = millis();
    uint32_t lastLog = 0;
    byte fill[32]; memset(fill, 0xAA, sizeof(fill));

    while (millis() - start < 5000) {
        byte txbytes = ELECHOUSE_cc1101.SpiReadStatus(0x3A);
        if (txbytes & 0x80) {
            ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
            ELECHOUSE_cc1101.SpiStrobe(0x3B);
            ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, buf, sizeof(buf));
            ELECHOUSE_cc1101.SpiStrobe(STROBE_STX);
            Serial.printf("[CARRIER2] underflow at %lums — restarted\n", millis() - start);
        } else if ((txbytes & 0x7F) < 32) {
            ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, fill, sizeof(fill));
        }
        if (millis() - lastLog >= 500) {
            byte ms = ELECHOUSE_cc1101.SpiReadStatus(0x35);
            byte tb = ELECHOUSE_cc1101.SpiReadStatus(0x3A);
            Serial.printf("[CARRIER2] t=%lums  MARC=0x%02X  TXBYTES=%d%s\n",
                millis()-start, ms, tb & 0x7F, (tb & 0x80) ? " UNDERFLOW!" : "");
            lastLog = millis();
        }
        yield();
    }

    ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
    cc1101InitTx();
    Serial.println("[CARRIER2] done");
    server.send(200, "text/plain",
        "5s 2-FSK carrier done.\nDid the physical remote FAIL during those 5 seconds?");
}

// GET /asyncgate — verify async GDO0 mode gates the PA correctly.
// Holds GDO0 HIGH 3s (carrier on), LOW 3s (PA off), HIGH 3s (carrier on).
// If capture unit RSSI drops to noise floor during LOW phase → async gating works.
// Run this before trusting /hi /med etc on the fan.
void handleAsyncgate() {
    server.send(200, "text/plain",
        "Async GDO0 PA gating test (9s).\n"
        "Watch capture unit RSSI:\n"
        "  0-3s:  GDO0 HIGH → expect -24 dBm (carrier on)\n"
        "  3-6s:  GDO0 LOW  → expect -97 dBm (PA off)\n"
        "  6-9s:  GDO0 HIGH → expect -24 dBm (carrier on)\n");

    ELECHOUSE_cc1101.SpiWriteReg(REG_PKTCTRL0, 0x32);  // async serial TX
    pinMode(GDO0_TX_PIN, OUTPUT);
    digitalWrite(GDO0_TX_PIN, LOW);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SFSTXON);
    delay(3);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_STX);
    delay(5);
    Serial.printf("[ASYNCGATE] MARC=0x%02X\n",
        ELECHOUSE_cc1101.SpiReadStatus(0x35));

    Serial.println("[ASYNCGATE] GDO0 HIGH — carrier ON");
    digitalWrite(GDO0_TX_PIN, HIGH);
    delay(3000);

    Serial.println("[ASYNCGATE] GDO0 LOW — carrier OFF");
    digitalWrite(GDO0_TX_PIN, LOW);
    delay(3000);

    Serial.println("[ASYNCGATE] GDO0 HIGH — carrier ON");
    digitalWrite(GDO0_TX_PIN, HIGH);
    delay(3000);

    digitalWrite(GDO0_TX_PIN, LOW);
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
    ELECHOUSE_cc1101.SpiWriteReg(REG_PKTCTRL0, 0x02);
    pinMode(GDO0_TX_PIN, INPUT);
    Serial.println("[ASYNCGATE] done");
}

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
        "<a href='/carrier' style='background:#555;font-size:0.85em'>RF TEST OOK (3s carrier)</a>"
        "<a href='/asyncgate'  style='background:#006080;font-size:0.85em'>ASYNC GATE TEST (9s verify PA off)</a>"
        "<a href='/carrier_ook' style='background:#c04000;font-size:0.85em'>OOK GATING TEST (4s alternating)</a>"
        "<a href='/carrier2' style='background:#555;font-size:0.85em'>RF TEST 2-FSK (5s)</a>"
        "<a href='/try?n=3' style='background:#555;font-size:0.85em'>TRY N=3 (unknown cmd)</a>"
        "<a href='/lighttry?reps=5'   style='background:#404;font-size:0.85em'>LIGHT ON  — 5 reps (~105ms)</a>"
        "<a href='/try?n=3'           style='background:#404;font-size:0.85em'>LIGHT OFF? — N=3 (unknown cmd)</a>"
        "<a href='/lighttry?reps=50'  style='background:#404;font-size:0.85em'>LIGHT hold — 50 reps (~1s)</a>"
        "<a href='/lighttry?reps=100' style='background:#404;font-size:0.85em'>LIGHT hold — 100 reps (~2.1s)</a>"
        "<a href='/lighttry?reps=150' style='background:#404;font-size:0.85em'>LIGHT hold — 150 reps (~3.2s)</a>"
        "<a href='/lighttry?reps=200' style='background:#404;font-size:0.85em'>LIGHT hold — 200 reps (~4.2s)</a>"
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

    server.on("/",        handleRoot);
    server.on("/carrier", handleCarrier);
    server.on("/carrier2",handleCarrier2);
    server.on("/hi",      handleHi);
    server.on("/med",     handleMed);
    server.on("/low",     handleLow);
    server.on("/off",     handleOff);
    server.on("/light",   handleLight);
    server.on("/strobe_test", handleStrobeTest);
    server.on("/ooktest",     handleOoktest);
    server.on("/fifogate",    handleFifogate);
    server.on("/marctest",    handleMarctest);
    server.on("/try",         handleTry);
    server.on("/lighttry",    handleLightTry);
    server.on("/status",      handleStatus);
    server.on("/dumpregs",    handleDumpregs);
    server.on("/carrier_ook", handleCarrierOok);
    server.on("/asyncgate",   handleAsyncgate);
    server.begin();
    Serial.println("Ready.");
}

void loop() {
    server.handleClient();
}

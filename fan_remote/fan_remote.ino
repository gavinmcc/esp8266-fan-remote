/*
 * fan_remote.ino
 *
 * ESP8266 WiFi ceiling fan remote controller
 * Replays captured OOK/ASK codes via CC1101 (E07-M1010)
 *
 * Captured from UC7070T remote, DIP address E7 80 29
 *
 * Wiring (same as capture sketch):
 *   CC1101 VCC  -> 3.3V
 *   CC1101 GND  -> GND
 *   CC1101 MOSI -> GPIO13 (D7)
 *   CC1101 MISO -> GPIO12 (D6)
 *   CC1101 SCK  -> GPIO14 (D5)
 *   CC1101 CSN  -> GPIO15 (D8)
 *   CC1101 GDO0 -> not connected (FIFO mode — data via SPI, not GDO0)
 *
 * HTTP endpoints:
 *   GET /       → simple control page
 *   GET /hi     → Fan high speed
 *   GET /med    → Fan medium speed
 *   GET /low    → Fan low speed
 *   GET /off    → Fan off
 *   GET /light  → Toggle light
 */

#include <SPI.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include "secrets.h"  // copy secrets.h.example → secrets.h and fill in credentials

// ── WiFi ──────────────────────────────────────────────────
const char* SSID     = WIFI_SSID;
const char* PASSWORD = WIFI_PASSWORD;

// ── Pins ──────────────────────────────────────────────────
#define PIN_CSN   15   // GPIO15 (D8)

// ── CC1101 register addresses ─────────────────────────────
#define REG_PKTCTRL0 0x08  // packet control
#define REG_MDMCFG2  0x12  // modem config: modulation, sync mode
#define REG_FREND0   0x22  // front end TX config (PA table pointer)
#define REG_PATABLE  0x3E  // PA power table (8 bytes)

// CC1101 strobe commands
#define STROBE_SFSTXON 0x31  // calibrate + lock PLL, PA off (FSTXON state)
#define STROBE_STX     0x35  // start TX (PA on)
#define STROBE_SIDLE   0x36  // go to idle

// ── OOK FIFO encoding @ 40 kbaud (25µs per bit) ──────────
// Derived from captured pulse measurements.
// NRZ stream: 0xFF bytes = carrier on, 0x00 = carrier off.
// Bit counts chosen to round to nearest 25µs period.
#define OOK_DRATE_KBAUD  40    // data rate for CC1101 FIFO TX

#define SYNC_ON_BITS     30    // 30×25 = 750µs  (target 740µs)
#define SYNC_OFF_BITS    29    // 29×25 = 725µs  (target 720µs)
#define ONE_ON_BITS      20    // 20×25 = 500µs  (target 490µs)
#define ONE_OFF_BITS      9    //  9×25 = 225µs  (target 235µs)
#define ZERO_ON_BITS     11    // 11×25 = 275µs  (target 265µs)
#define ZERO_OFF_BITS    19    // 19×25 = 475µs  (target 465µs)

#define SYNC_COUNT        4    // sync pulses per preamble
#define REPS              4    // code repetitions per button press

// 4 reps × (4 sync × 59 bits + 73 data × 29 bits) = ~9600 bits = ~1200 bytes; round up
#define MAX_STREAM_BYTES  1300

// ── Command bit sequences (73 bits, MSB first) ────────────
// Fixed bytes across all commands:
//   B0=E7  B1=80  B2=29         (DIP switch address)
//   B5=AA  B6=55  B7=AA         (fixed protocol bytes)
// Varying bytes (command):
//   B3, B4, B8

#define NBITS 73

const uint8_t BITS_HI[NBITS] = {
    1,1,1,0,0,1,1,1,  // E7
    1,0,0,0,0,0,0,0,  // 80
    0,0,1,0,1,0,0,1,  // 29
    1,0,1,0,0,0,0,0,  // A0
    0,1,0,0,1,0,1,1,  // 4B
    1,0,1,0,1,0,1,0,  // AA
    0,1,0,1,0,1,0,1,  // 55
    1,0,1,0,1,0,1,0,  // AA
    0,1,0,0,0,1,0,0,  // 44
    0                 // +1
};
const uint8_t BITS_MED[NBITS] = {
    1,1,1,0,0,1,1,1,  // E7
    1,0,0,0,0,0,0,0,  // 80
    0,0,1,0,1,0,0,1,  // 29
    1,0,0,1,0,0,0,0,  // 90
    0,1,0,0,1,1,0,0,  // 4C
    1,0,1,0,1,0,1,0,  // AA
    0,1,0,1,0,1,0,1,  // 55
    1,0,1,0,1,0,1,0,  // AA
    0,1,0,1,0,1,0,0,  // 54
    0                 // +1
};
const uint8_t BITS_LOW[NBITS] = {
    1,1,1,0,0,1,1,1,  // E7
    1,0,0,0,0,0,0,0,  // 80
    0,0,1,0,1,0,0,1,  // 29
    1,0,1,0,0,0,0,0,  // A0
    0,1,0,0,1,1,0,0,  // 4C
    1,0,1,0,1,0,1,0,  // AA
    0,1,0,1,0,1,0,1,  // 55
    1,0,1,0,1,0,1,0,  // AA
    1,1,0,1,0,0,0,0,  // D0
    0                 // +1
};
const uint8_t BITS_OFF[NBITS] = {
    1,1,1,0,0,1,1,1,  // E7
    1,0,0,0,0,0,0,0,  // 80
    0,0,1,0,1,0,0,1,  // 29
    1,0,0,1,0,0,0,0,  // 90
    0,1,0,0,1,0,1,1,  // 4B
    1,0,1,0,1,0,1,0,  // AA
    0,1,0,1,0,1,0,1,  // 55
    1,0,1,0,1,0,1,0,  // AA
    1,1,0,0,0,0,0,0,  // C0
    0                 // +1
};
const uint8_t BITS_LIGHT[NBITS] = {
    1,1,1,0,0,1,1,1,  // E7
    1,0,0,0,0,0,0,0,  // 80
    0,0,1,0,1,0,0,1,  // 29
    1,0,0,0,0,0,0,0,  // 80
    0,1,0,0,1,1,0,0,  // 4C
    1,0,1,0,1,0,1,0,  // AA
    0,1,0,1,0,1,0,1,  // 55
    1,0,1,0,1,0,1,0,  // AA
    0,0,1,0,1,0,0,0,  // 28
    0                 // +1
};

ESP8266WebServer server(80);

// ─────────────────────────────────────────────────────────
// Minimal SPI strobe — no MISO-wait, safe to call from noInterrupts().
// Must be in RAM to avoid flash cache stalls during timing-critical TX.
static void ICACHE_RAM_ATTR fastStrobe(byte cmd) {
    digitalWrite(PIN_CSN, LOW);
    SPI.transfer(cmd);
    digitalWrite(PIN_CSN, HIGH);
}

// ─────────────────────────────────────────────────────────
void cc1101InitTx() {
    pinMode(PIN_CSN, OUTPUT);
    digitalWrite(PIN_CSN, HIGH);
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV16);

    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setMHZ(433.92);
    ELECHOUSE_cc1101.setModulation(2);  // OOK/ASK
    ELECHOUSE_cc1101.setPA(10);         // TX power ~10 dBm

    // PKTCTRL0 = 0x02: FIFO mode, infinite packet length.
    //   OOK carrier is controlled by FIFO data: 0xFF bytes = carrier on, 0x00 = off.
    // MDMCFG2 = 0x20: MOD_FORMAT bits [6:4] = 010 = OOK/ASK (library may write 0x30 = MSK).
    // FREND0 = 0x10: OOK=1 uses PATABLE[0] for carrier-on; OOK=0 = PA off.
    //   The library's setModulation(2) sets FREND0=0x11 (PATABLE[1] for OOK=1),
    //   but setPA() writes power to PATABLE[0], leaving PATABLE[1]=0 → no carrier.
    //   FREND0=0x10 fixes this: OOK=1 → PATABLE[0]=TX power, OOK=0 → PA off.
    ELECHOUSE_cc1101.setDRate(OOK_DRATE_KBAUD);
    ELECHOUSE_cc1101.SpiWriteReg(REG_PKTCTRL0, 0x02);
    ELECHOUSE_cc1101.SpiWriteReg(REG_MDMCFG2,  0x20);
    ELECHOUSE_cc1101.SpiWriteReg(REG_FREND0,   0x10);

    ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
}

// ─────────────────────────────────────────────────────────
// FIFO OOK stream buffer.
// NRZ encoding: 1-bits = carrier on (0xFF bytes), 0-bits = carrier off (0x00).
// At 40 kbaud each bit = 25µs. The bit counts in the #defines above map pulse
// timings to whole bits.
static byte g_stream[MAX_STREAM_BYTES];
static int  g_streamLen;

// Append `count` bits of value `val` (0 or 1) into buf, MSB-first, bit-addressed by *pos.
static void appendBits(byte* buf, int& pos, int count, byte val) {
    for (int i = 0; i < count; i++) {
        if (val) buf[pos >> 3] |= (0x80 >> (pos & 7));
        pos++;
    }
}

// Build full NRZ OOK stream for `nbits` data bits across REPS repetitions.
static void buildStream(const uint8_t* bits, int nbits) {
    memset(g_stream, 0, sizeof(g_stream));
    int pos = 0;
    for (int rep = 0; rep < REPS; rep++) {
        for (int s = 0; s < SYNC_COUNT; s++) {
            appendBits(g_stream, pos, SYNC_ON_BITS,   1);
            appendBits(g_stream, pos, SYNC_OFF_BITS,  0);
        }
        for (int i = 0; i < nbits; i++) {
            if (bits[i]) {
                appendBits(g_stream, pos, ONE_ON_BITS,   1);
                appendBits(g_stream, pos, ONE_OFF_BITS,  0);
            } else {
                appendBits(g_stream, pos, ZERO_ON_BITS,  1);
                appendBits(g_stream, pos, ZERO_OFF_BITS, 0);
            }
        }
    }
    g_streamLen = (pos + 7) >> 3;
}

// ─────────────────────────────────────────────────────────
// MARCSTATE register (0x35) values:
//   0x01 = IDLE,  0x12 = FSTXON (PLL locked, PA off),  0x13 = TX (PA on)
//
// TX approach:
//   1. SFSTXON: PLL calibrates + locks, PA stays off (FSTXON state, ~800µs).
//   2. Fill CC1101 FIFO with first 64 bytes of OOK stream.
//   3. STX from FSTXON: PLL already locked → TX starts instantly, no recalibration gap.
//   4. Keep streaming remaining bytes into FIFO before it empties.
void sendCommand(const uint8_t* bits, const char* name) {
    buildStream(bits, NBITS);
    Serial.printf("[TX] %s — %d bytes @ %d kbaud\n", name, g_streamLen, OOK_DRATE_KBAUD);

    // Pre-calibrate PLL (FSTXON state) before filling FIFO to minimise latency at TX start.
    ELECHOUSE_cc1101.SpiStrobe(STROBE_SFSTXON);
    uint32_t t0 = micros();
    byte marcstate;
    do {
        marcstate = ELECHOUSE_cc1101.SpiReadStatus(0x35);
    } while (marcstate != 0x12 && (micros() - t0) < 3000);

    if (marcstate != 0x12) {
        Serial.printf("[TX] %s ABORT — FSTXON timeout (MARCSTATE=0x%02X)\n", name, marcstate);
        ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
        return;
    }

    // Fill FIFO then kick TX. noInterrupts() keeps the FIFO streaming tight so we don't
    // underflow mid-message. Total stream ~30ms at 40kbaud — well within WDT limits.
    noInterrupts();

    int sent = min(64, g_streamLen);
    ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, g_stream, sent);
    fastStrobe(STROBE_STX);  // FSTXON → TX: PLL already locked, instant start

    while (sent < g_streamLen) {
        byte txb = ELECHOUSE_cc1101.SpiReadStatus(0x3A);
        if (txb & 0x80) break;           // TXFIFO_UNDERFLOW — bail
        byte space = 64 - (txb & 0x7F);
        if (space > 0) {
            int chunk = min((int)space, g_streamLen - sent);
            ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, g_stream + sent, chunk);
            sent += chunk;
        }
    }

    // Wait for FIFO to drain (underflow flag set means last byte clocked out).
    byte txb;
    uint32_t drain0 = micros();
    do {
        txb = ELECHOUSE_cc1101.SpiReadStatus(0x3A);
    } while (!(txb & 0x80) && (micros() - drain0) < 50000UL);

    interrupts();

    ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
    ELECHOUSE_cc1101.SpiStrobe(0x3B);  // SFTX: clear TXFIFO_UNDERFLOW flag

    Serial.printf("[TX] %s done (sent %d/%d bytes)\n", name, sent, g_streamLen);
}

// ── HTTP handlers ─────────────────────────────────────────
void handleHi()    { sendCommand(BITS_HI,    "HI");    server.send(200, "text/plain", "OK: HI"); }
void handleMed()   { sendCommand(BITS_MED,   "MED");   server.send(200, "text/plain", "OK: MED"); }
void handleLow()   { sendCommand(BITS_LOW,   "LOW");   server.send(200, "text/plain", "OK: LOW"); }
void handleOff()   { sendCommand(BITS_OFF,   "OFF");   server.send(200, "text/plain", "OK: OFF"); }
void handleLight() { sendCommand(BITS_LIGHT, "LIGHT"); server.send(200, "text/plain", "OK: LIGHT"); }

// ── RF emission diagnostic ─────────────────────────────────
// GET /carrier → FIFO-based 3s carrier (0xFF bytes = continuous OOK carrier).
// Bypasses async serial mode — uses the standard CC1101 packet TX path.
// TEST: press /carrier then immediately try the physical remote on the fan.
//   • Physical remote FAILS during those 3s → CC1101 IS transmitting RF.
//   • Physical remote still works → CC1101 hardware/power issue.
void handleCarrier() {
    Serial.println("[CARRIER] starting 3s carrier test (300 baud = ~1.7s per FIFO fill)...");

    // Re-init with very low DRATE so the 64-byte FIFO lasts ~1.7 seconds
    // (at default 4800 baud the FIFO only lasted ~106ms — previous test was invalid).
    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setMHZ(433.92);
    ELECHOUSE_cc1101.setModulation(2);  // OOK
    ELECHOUSE_cc1101.setPA(10);
    ELECHOUSE_cc1101.setDRate(0.3);     // 300 baud — 64 bytes lasts ~1.7s
    ELECHOUSE_cc1101.SpiWriteReg(REG_FREND0,   0x10);  // OOK: PA on = PATABLE[0]
    ELECHOUSE_cc1101.SpiWriteReg(REG_PKTCTRL0, 0x02);  // FIFO, infinite length

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
            // TXFIFO_UNDERFLOW — flush, refill, restart TX
            ELECHOUSE_cc1101.SpiStrobe(STROBE_SIDLE);
            ELECHOUSE_cc1101.SpiStrobe(0x3B);  // SFTX: flush TX FIFO
            ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, buf, sizeof(buf));
            ELECHOUSE_cc1101.SpiStrobe(STROBE_STX);
            Serial.printf("[CARRIER] underflow at %lums — restarted\n", millis() - start);
        } else if ((txbytes & 0x7F) < 32) {
            // Getting low — top up with 32 bytes
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
    cc1101InitTx();  // restore normal OOK async config

    Serial.println("[CARRIER] done");
    server.send(200, "text/plain",
        "3s carrier test done — check Serial for MARCSTATE log.\n"
        "Did the physical remote FAIL at any point during the 3 seconds?");
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
        "<a href='/carrier' style='background:#555;font-size:0.85em'>RF TEST (3s carrier)</a>"
        "</body></html>"
    );
}

// ─────────────────────────────────────────────────────────
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

    // Verify key registers were written correctly
    byte r_pktctrl0 = ELECHOUSE_cc1101.SpiReadReg(REG_PKTCTRL0);
    byte r_mdmcfg2  = ELECHOUSE_cc1101.SpiReadReg(REG_MDMCFG2);
    byte r_frend0   = ELECHOUSE_cc1101.SpiReadReg(REG_FREND0);
    byte r_patable0 = ELECHOUSE_cc1101.SpiReadReg(REG_PATABLE);
    Serial.printf("PKTCTRL0=0x%02X (expect 0x02)  MDMCFG2=0x%02X (expect 0x20)\n",
                  r_pktctrl0, r_mdmcfg2);
    Serial.printf("FREND0  =0x%02X (expect 0x10)  PATABLE[0]=0x%02X (library value)\n",
                  r_frend0, r_patable0);

    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWORD);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print('.');
    }
    Serial.printf("\nIP: http://%s\n", WiFi.localIP().toString().c_str());

    server.on("/",       handleRoot);
    server.on("/carrier", handleCarrier);
    server.on("/hi",    handleHi);
    server.on("/med",   handleMed);
    server.on("/low",   handleLow);
    server.on("/off",   handleOff);
    server.on("/light", handleLight);
    server.begin();
    Serial.println("Ready.");
}

void loop() {
    server.handleClient();
}

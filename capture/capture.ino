/*
 * capture.ino
 *
 * Captures raw OOK/ASK RF pulses from the UC7070T ceiling fan remote
 * via CC1101 (E07-M1010) on ESP8266. Outputs pulse timings over Serial
 * so you can identify the protocol before writing the replay firmware.
 *
 * Wiring (E07-M1010 -> ESP8266 NodeMCU/Wemos D1 Mini):
 *   VCC  -> 3.3V  (do NOT use 5V)
 *   GND  -> GND
 *   MOSI -> D7 (GPIO13)  -- verify against your wiring from the web UI session
 *   MISO -> D6 (GPIO12)
 *   SCK  -> D5 (GPIO14)
 *   CSN  -> D8 (GPIO15)
 *   GDO0 -> D2 (GPIO4)   <-- raw demodulated signal output
 *   GDO2 -> not connected
 *
 * Library required: ELECHOUSE_CC1101_SRC_DRV
 *   Install via Arduino Library Manager: search "ELECHOUSE CC1101"
 *   or: https://github.com/LSatan/SmartRC-CC1101-Driver-Lib
 */

#include <SPI.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>

// GDO0 outputs raw demodulated OOK signal from CC1101
// Using raw GPIO numbers to avoid board-definition issues
#define GDO0_PIN     5     // D1 on NodeMCU / Wemos D1 Mini (GPIO5)

// SPI pins (explicit for Generic ESP8266 board target)
#define PIN_SCK      14    // D5
#define PIN_MISO     12    // D6
#define PIN_MOSI     13    // D7
#define PIN_CSN      15    // D8

// Pulse capture limits
#define MAX_PULSES   300
#define MIN_PULSE_US 80    // below this = noise, skip
#define GAP_US       8000  // gap this long = end of packet

struct Pulse {
  uint32_t high;
  uint32_t low;
};

Pulse pulses[MAX_PULSES];
int pulseCount = 0;
int captureCount = 0;

// Read a CC1101 register directly via raw SPI for diagnostics
uint8_t cc1101ReadReg(uint8_t addr) {
  digitalWrite(PIN_CSN, LOW);
  delayMicroseconds(10);
  SPI.transfer(addr | 0x80); // read bit
  uint8_t val = SPI.transfer(0x00);
  delayMicroseconds(10);
  digitalWrite(PIN_CSN, HIGH);
  return val;
}

void setupCC1101() {
  // ESP8266 hardware SPI pins are fixed: SCK=14, MISO=12, MOSI=13, SS=15
  // These match our wiring exactly, so SPI.begin() with no args is correct.
  pinMode(PIN_CSN, OUTPUT);
  digitalWrite(PIN_CSN, HIGH);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16); // ~5 MHz, well within CC1101 spec

  // Raw diagnostic: read CC1101 PARTNUM (0x30) and VERSION (0x31) registers
  // CC1101 PARTNUM should return 0x00, VERSION should return 0x14
  uint8_t partnum = cc1101ReadReg(0x30);
  uint8_t version = cc1101ReadReg(0x31);
  Serial.printf("SPI diag: PARTNUM=0x%02X VERSION=0x%02X\n", partnum, version);
  Serial.println("(expect PARTNUM=0x00, VERSION=0x14 — 0xFF means no SPI response)");

  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setMHZ(303.92);   // carrier frequency
  ELECHOUSE_cc1101.setModulation(2); // 2 = OOK/ASK
  ELECHOUSE_cc1101.setDRate(3.79);   // data rate kbaud (rough estimate; adjust after analysis)
  ELECHOUSE_cc1101.setRxBW(812.50);  // wide RX bandwidth to catch signal
  ELECHOUSE_cc1101.setPA(12);        // RX mode, PA doesn't matter here
  ELECHOUSE_cc1101.SetRx();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== CC1101 Fan Remote Sniffer ===");

  // Must init SPI before calling getCC1101()
  setupCC1101();

  if (ELECHOUSE_cc1101.getCC1101()) {
    Serial.println("CC1101: OK");
  } else {
    Serial.println("CC1101: NOT FOUND - check wiring!");
    while (true) { delay(1000); }
  }

  pinMode(GDO0_PIN, INPUT);

  Serial.println("Listening on 303.92 MHz OOK (UC7070T band)...");
  Serial.println("Press a button on the remote.\n");
}

bool capturePulses() {
  pulseCount = 0;
  uint32_t t;

  // Wait for signal to go high (start of burst)
  uint32_t timeout = millis();
  while (digitalRead(GDO0_PIN) == LOW) {
    if (millis() - timeout > 100) return false; // nothing within 100ms
    delay(0);  // yield to SDK watchdog; safe since we're just waiting, not timing
  }

  while (pulseCount < MAX_PULSES) {
    // Measure HIGH portion
    t = micros();
    while (digitalRead(GDO0_PIN) == HIGH) {
      if (micros() - t > GAP_US) goto done;
    }
    uint32_t hi = micros() - t;

    // Measure LOW portion
    t = micros();
    while (digitalRead(GDO0_PIN) == LOW) {
      if (micros() - t > GAP_US) {
        // End of packet gap - store last high and break
        if (hi >= MIN_PULSE_US) {
          pulses[pulseCount].high = hi;
          pulses[pulseCount].low  = micros() - t;
          pulseCount++;
        }
        goto done;
      }
    }
    uint32_t lo = micros() - t;

    if (hi >= MIN_PULSE_US) {
      pulses[pulseCount].high = hi;
      pulses[pulseCount].low  = lo;
      pulseCount++;
    }
  }

done:
  return pulseCount >= 4;
}

void printCapture(const char* label) {
  // Compact single-line output to minimize Serial time between reps
  Serial.printf("@%lums #%d(%d): ", millis(), captureCount, pulseCount);
  for (int i = 0; i < pulseCount; i++) {
    Serial.printf("+%lu-%lu", pulses[i].high, pulses[i].low);
    if (i < pulseCount - 1) Serial.print(' ');
  }
  Serial.println();
}

// Simple stats to help identify bit timings
void printStats() {
  if (pulseCount < 4) return;

  uint32_t minHi = UINT32_MAX, maxHi = 0;
  uint32_t minLo = UINT32_MAX, maxLo = 0;

  for (int i = 0; i < pulseCount; i++) {
    minHi = min(minHi, pulses[i].high); maxHi = max(maxHi, pulses[i].high);
    minLo = min(minLo, pulses[i].low);  maxLo = max(maxLo, pulses[i].low);
  }
  Serial.printf("HIGH range: %lu - %lu us\n", minHi, maxHi);
  Serial.printf("LOW  range: %lu - %lu us\n", minLo, maxLo);
}

static uint32_t lastRssi = 0;
static uint32_t lastGdo  = 0;

void loop() {
  uint32_t now = millis();

  // Print RSSI + GDO0 state every 500ms so we can see if RX is getting any signal
  if (now - lastRssi >= 100) {
    lastRssi = now;
    int rssi = ELECHOUSE_cc1101.getRssi();
    int gdo  = digitalRead(GDO0_PIN);
    Serial.printf("[RSSI] %d dBm  GDO0=%d\n", rssi, gdo);
  }

  if (!capturePulses()) return;

  char label[32];
  snprintf(label, sizeof(label), "Capture #%d", ++captureCount);
  printCapture(label);
  printStats();

  // Re-arm immediately to catch consecutive reps from one button press
  ELECHOUSE_cc1101.SetRx();
  delay(2);
}

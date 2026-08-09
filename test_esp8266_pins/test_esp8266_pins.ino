/*
 * test_esp8266_pins.ino
 *
 * Flashes each SPI-related GPIO in turn (1s on, 200ms off).
 * Wire an LED + resistor (~220Ω) from each pin to GND.
 * Serial Monitor shows which pin is currently HIGH.
 *
 * Pins tested: D1/GPIO5, D5/GPIO14, D6/GPIO12, D7/GPIO13, D8/GPIO15
 * These are the CC1101 wiring pins (GDO0, SCK, MISO, MOSI, CSN).
 */

const int pins[]       = {  5,          14,         12,         13,         15  };
const char* names[]    = {"D1/GPIO5", "D5/GPIO14","D6/GPIO12","D7/GPIO13","D8/GPIO15"};
const int PIN_COUNT    = 5;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== ESP8266 pin test ===");
    for (int i = 0; i < PIN_COUNT; i++) {
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], LOW);
    }
}

void loop() {
    for (int i = 0; i < PIN_COUNT; i++) {
        Serial.printf("HIGH: %s\n", names[i]);
        digitalWrite(pins[i], HIGH);
        delay(1000);
        digitalWrite(pins[i], LOW);
        delay(200);
    }
}

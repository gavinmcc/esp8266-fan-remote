# Protocol Specification — RHINE UC7070T Fan Remote

## RF Parameters

| Parameter   | Value       |
|-------------|-------------|
| Frequency   | 433.92 MHz  |
| Modulation  | OOK / ASK   |
| Encoding    | PWM (pulse-width modulation) |
| Bit rate    | ~1.37 kbps (variable — pulse-width encoded) |

## Packet Structure

Each button press transmits **4 repetitions** of the following frame, back-to-back with no gap:

```
[4 sync pulses] [73 data bits]  ×4
```

There is no inter-repetition gap. The low phase of the last data bit of one repetition flows directly into the high phase of the first sync pulse of the next.

## Pulse Timings (µs)

Measured from captures of a physical UC7070T remote:

| Symbol  | HIGH (µs) | LOW (µs) | Total (µs) |
|---------|-----------|----------|------------|
| Sync    | ~740      | ~720     | ~1460      |
| Bit `1` | ~490      | ~235     | ~725       |
| Bit `0` | ~265      | ~465     | ~730       |

Encoding rule: **long HIGH = 1, short HIGH = 0** (threshold ~380 µs).

## Data Format (73 bits, MSB first)

Bytes B0–B2 and B5–B7 are fixed across all commands. B3, B4, and B8 carry the command.

| Byte | Bits   | Fixed value | Notes                     |
|------|--------|-------------|---------------------------|
| B0   | 0–7    | `E7`        | DIP switch address byte 0 |
| B1   | 8–15   | `80`        | DIP switch address byte 1 |
| B2   | 16–23  | `29`        | DIP switch address byte 2 |
| B3   | 24–31  | varies      | command byte              |
| B4   | 32–39  | varies      | command byte              |
| B5   | 40–47  | `AA`        | fixed protocol byte       |
| B6   | 48–55  | `55`        | fixed protocol byte       |
| B7   | 56–63  | `AA`        | fixed protocol byte       |
| B8   | 64–71  | varies      | command byte              |
| +1   | 72     | `0`         | trailing stop bit         |

### Command Bytes

| Command | B3   | B4   | B8   |
|---------|------|------|------|
| HI      | `A0` | `4B` | `44` |
| MED     | `90` | `4C` | `54` |
| LOW     | `A0` | `4C` | `D0` |
| OFF     | `90` | `4B` | `C0` |
| LIGHT   | `80` | `4C` | `28` |

### Adapting for a Different DIP Address

Change B0–B2 in the `BITS_*` arrays in `fan_remote.ino` to match your remote's DIP switch setting. B5–B7 and the encoding rule should remain the same across UC7070T units.

## CC1101 Configuration

TX is implemented using CC1101 FIFO mode. The OOK pulse pattern is pre-computed as an NRZ byte stream at 40 kbaud (25 µs/bit) and streamed through the CC1101 TX FIFO.

| Register  | Value  | Purpose                                              |
|-----------|--------|------------------------------------------------------|
| PKTCTRL0  | `0x02` | FIFO mode, infinite packet length, no CRC, no whitening |
| MDMCFG2   | `0x20` | OOK modulation, no sync word                         |
| FREND0    | `0x10` | OOK=1 uses PATABLE[0] (carrier on); OOK=0 = PA off  |
| DRATE     | 40 kbaud | via `setDRate(40)`                                 |
| PATABLE[0]| `0xC0` | ~10 dBm at 433 MHz, set by `setPA(10)`              |

> **FREND0 note:** The SmartRC library's `setModulation(2)` writes `FREND0=0x11`, which routes
> OOK=1 to PATABLE[1] (default 0x00 = no carrier). Explicitly writing `0x10` after library init
> fixes this so OOK=1 uses PATABLE[0] (the power level set by `setPA()`).

## FIFO Encoding (40 kbaud NRZ)

Each OOK pulse is expressed as a run of `1` bits (carrier on) followed by `0` bits (carrier off).
At 40 kbaud, 1 bit = 25 µs.

| Symbol  | ON bits | OFF bits | Actual timing        | Target timing        |
|---------|---------|----------|----------------------|----------------------|
| Sync    | 30      | 29       | 750 µs / 725 µs      | 740 µs / 720 µs      |
| Bit `1` | 20      | 9        | 500 µs / 225 µs      | 490 µs / 235 µs      |
| Bit `0` | 11      | 19       | 275 µs / 475 µs      | 265 µs / 465 µs      |

Total stream length: ~1198 bytes (4 reps × ~300 bytes).

## TX Sequence

1. `buildStream()` — pre-compute full NRZ byte stream into `g_stream[]`
2. `SFSTXON` strobe — PLL calibrates and locks (~800 µs), PA stays off (FSTXON state)
3. Write first 64 bytes into TX FIFO
4. `STX` strobe — PA on, transmission starts instantly (PLL already locked)
5. Polling loop — read TXBYTES, top up FIFO in chunks until all bytes sent
6. Wait for `TXFIFO_UNDERFLOW` flag (last byte transmitted)
7. `SIDLE` + `SFTX` — return to idle and clear underflow flag

The entire critical section (steps 3–6) runs inside `noInterrupts()`. At 40 kbaud, ~1200 bytes takes approximately 30 ms — well within the ESP8266 WDT limit.

## Capture Methodology

Raw pulses were captured using `capture/capture.ino`, which:
- Configures CC1101 in OOK RX mode at 433.92 MHz
- Reads the demodulated signal from CC1101 GDO0 (configured as digital data output in RX mode)
- Records HIGH/LOW pulse durations until a gap > 8 ms (end of burst)
- Prints raw `+HHH-LLL` tokens over Serial

`analyze.py` parses the raw tokens, strips sync pulses (HIGH ≥ 600 µs), decodes bits using the
380 µs threshold, and applies majority voting across the 4 repetitions to produce the final bit arrays.

## What Was Investigated

| Approach | Outcome |
|----------|---------|
| Async GDO0 bit-bang (PKTCTRL0=0x32) | CC1101 enters TX state (MARCSTATE=0x13) but GDO0 does not modulate the OOK carrier on E07-M1010 |
| FIFO mode (PKTCTRL0=0x02) | Streaming works correctly (TXBYTES drains, no underflows) |
| FREND0=0x11 (library default) | OOK carrier inverted — PA off when it should be on. Fixed with 0x10 |
| Explicit PATABLE write | Overrode library's setPA() value. Removed — setPA() handles it correctly |
| 10 ms inter-rep gaps | Tested and reverted. Raw captures confirm reps are back-to-back with no gap |
| RF emission confirmation | Inconclusive. MARCSTATE=0x13 confirms chip state; actual antenna output unverified |

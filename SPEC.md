# Protocol Specification — RHINE UC7070T Fan Remote

## RF Parameters

| Parameter   | Value       |
|-------------|-------------|
| Frequency   | 303.92 MHz  |
| Modulation  | OOK / ASK   |
| Encoding    | Position-encoded symbol pairs |

## Symbol Alphabet

Two pulse widths for each phase:

| Name    | Phase | Width (µs) | Measured range |
|---------|-------|------------|----------------|
| SHORT_H | HIGH  | ~250       | 195–296 µs     |
| LONG_H  | HIGH  | ~550       | 400–662 µs     |
| SHORT_L | LOW   | ~450       | 294–499 µs     |
| LONG_L  | LOW   | ~790       | 743–848 µs     |
| GAP     | LOW   | ~8000      | inter-rep gap  |

Each pulse is a (HIGH, LOW) pair. Symbol names use two letters: first = HIGH class, second = LOW class.

| Symbol | HIGH phase | LOW phase |
|--------|-----------|----------|
| LS     | LONG_H    | SHORT_L  |
| LL     | LONG_H    | LONG_L   |
| SL     | SHORT_H   | LONG_L   |
| SS     | SHORT_H   | SHORT_L  |

## Packet Structure

Each button press transmits **4 repetitions** of the following 13-pulse frame, each followed by an 8 ms gap:

```
[Header: 6 pulses] [Data: 7 pulses] [GAP 8ms]   × 4
```

### Header (6 pulses, fixed for all commands)

```
LS  LS  LL  SL  SS  LL
```

### Data Section (7 pulses, command-dependent)

The command is encoded by **N** — the position (0-indexed) of the `SS` symbol within the 7-pulse data section:

```
SL × N,  SS,  (tail)
```

**Tail when N < 5:**
```
LL,  SL × (4−N),  SHORT_H (terminal pulse before gap)
```

**Tail when N = 5:**
```
LONG_H (terminal pulse before gap)
```

All 5 valid N values produce identical total bit counts per rep (838 bits at 40 kbaud).

### Command Map

| Button | N | Data section (7 pulses)          |
|--------|---|----------------------------------|
| HI     | 0 | SS LL SL SL SL SL S_term         |
| MED    | 1 | SL SS LL SL SL SL S_term         |
| LOW    | 2 | SL SL SS LL SL SL S_term         |
| OFF    | 4 | SL SL SL SL SS LL S_term         |
| LIGHT  | 5 | SL SL SL SL SL SS L_term         |

N=3 not observed (may be unused or reserved).

## Capture Evidence

Re-captured with a dedicated second ESP8266+CC1101 RX unit. Each button pressed
individually; N value decoded from multiple clean 13-pulse frames per button.

| Button | N decoded | Notes                          |
|--------|-----------|--------------------------------|
| HI     | 0         | SS at data[0], LL+SL×4 tail   |
| MED    | 1         | SS at data[1], LL+SL×3 tail   |
| LOW    | 2         | SS at data[2], LL+SL×2 tail   |
| OFF    | 4         | SS at data[4], LL+SL×0 tail   |
| LIGHT  | 5         | SS at data[5], LONG_H tail     |

Earlier 12-capture set (physical remote captured without dedicated RX unit) had
HI/MED swapped and LOW/LIGHT swapped in the label column — the N decoding logic
was correct but the buttons were mislabelled. OFF (N=4) was correct in both sets.

## Hardware

- **CC1101 module**: E07-M1010 (Ebyte/CDEBYTE) — 300–348 MHz, 10 dBm max TX
- **MCU**: ESP8266 (Generic / Wemos D1 Mini)

## CC1101 Configuration

TX is implemented using CC1101 FIFO mode. The OOK pulse pattern is pre-computed as an NRZ byte stream at 40 kbaud (25 µs/bit) and streamed through the CC1101 TX FIFO.

| Register  | Value  | Purpose                                              |
|-----------|--------|------------------------------------------------------|
| PKTCTRL0  | `0x02` | FIFO mode, infinite packet length, no CRC, no whitening |
| MDMCFG2   | `0x20` | OOK modulation, no sync word                         |
| FREND0    | `0x10` | OOK=1 uses PATABLE[0] (carrier on); OOK=0 = PA off  |
| DRATE     | 40 kbaud | via `setDRate(40)`                                 |
| PATABLE[0]| `0xC0` | ~10 dBm TX power, set by `setPA(10)`                |

> **FREND0 note:** The SmartRC library's `setModulation(2)` writes `FREND0=0x11`, which routes
> OOK=1 to PATABLE[1] (default 0x00 = no carrier). Explicitly writing `0x10` after library init
> fixes this so OOK=1 uses PATABLE[0] (the power level set by `setPA()`).

## FIFO Encoding (40 kbaud NRZ)

At 40 kbaud, 1 bit = 25 µs. Each OOK pulse phase is a run of 1-bits (carrier on) or 0-bits (carrier off).

| Symbol  | ON bits | OFF bits | Actual µs  | Target µs |
|---------|---------|----------|------------|-----------|
| SHORT_H | 10      | —        | 250        | ~250      |
| LONG_H  | 22      | —        | 550        | ~550      |
| SHORT_L | —       | 18       | 450        | ~450      |
| LONG_L  | —       | 32       | 800        | ~790      |
| GAP     | —       | 320      | 8000       | ~8000     |

Total stream per button press: 4 reps × 838 bits = 3352 bits = ~420 bytes.

## TX Sequence

1. `buildStream(N)` — pre-compute NRZ byte stream into `g_stream[]`
2. `SFSTXON` strobe — PLL calibrates and locks (~800 µs), PA stays off (FSTXON state)
3. Write first 64 bytes into TX FIFO
4. `STX` strobe — PA on, transmission starts instantly (PLL already locked)
5. Polling loop — read TXBYTES, top up FIFO in chunks until all bytes sent
6. Wait for `TXFIFO_UNDERFLOW` flag (last byte transmitted)
7. `SIDLE` + `SFTX` — return to idle and clear underflow flag

## Capture Methodology

Raw pulses captured using `capture/capture.ino`:
- CC1101 in OOK RX mode at 303.92 MHz, 812.5 kHz RX bandwidth
- GDO0 pin configured as demodulated digital data output
- Records HIGH/LOW durations until gap > 8 ms (end of rep)
- Prints raw `+HHH-LLL` tokens over Serial

## Current Status

All registers confirmed correct (`/dumpregs` and `/status`). Firmware encodes protocol
correctly and FIFO streams cleanly (no underflows, TXBYTES drains). However, no fan
response observed across all 6 N values (0–5) with 15 reps.

**OOK gating test result**: `/carrier_ook` (2s ON / 2s OFF alternation at 40 kbaud)
showed the physical remote stayed jammed throughout both OFF windows. This means
the PA does not gate off when FIFO data is 0x00 — the carrier is continuous in TX state
regardless of FIFO contents.

Suspected cause: hardware defect on the specific E07-M1010 module unit. Two units
available; second unit swap is the immediate next diagnostic step.

## What Was Investigated

| Approach | Outcome |
|----------|---------|
| 73-bit byte-based protocol (original analysis) | Wrong — reanalysis of 12 captures shows a 13-pulse position-encoded protocol |
| Async GDO0 bit-bang (PKTCTRL0=0x32) | CC1101 enters TX state but GDO0 does not modulate OOK on E07-M1010 |
| FIFO mode (PKTCTRL0=0x02) | TXBYTES drains cleanly, no underflows — but OOK gating unconfirmed |
| FREND0=0x11 (library default) | OOK carrier broken — fixed with 0x10 |
| MDMCFG2=0x20 (reserved modulation) | Emits carrier but no OOK gating — fixed with 0x30 |
| OOK gating test (2s ON/OFF via FIFO) | PA stayed on during 0x00 phase — gating not working on current unit |
| Carrier jamming test | Physical remote reliably jammed at 303.92 MHz — RF path confirmed |

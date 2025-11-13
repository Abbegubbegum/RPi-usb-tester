# USB Port Field Tester - Hardware Documentation

## Purpose

This is a **field diagnostic tool** for quickly identifying obviously faulty USB ports on incoming computers. It is NOT a USB compliance certification tool.

**What it detects:**

-   ✓ Completely dead ports (no VBUS)
-   ✓ Ports that can't deliver 400mA
-   ✓ Ports with excessive voltage sag (worse than normal)
-   ✓ Data communication failures
-   ✓ Overcurrent protection issues
-   ✓ Dirty or corroded contacts (resistance > 3Ω)
-   ✓ Power supply instability (varying resistance under load)

**What it does NOT do:**

-   ✗ Certify USB 2.0 specification compliance
-   ✗ Measure to specification-grade accuracy
-   ✗ Qualify ports for specific USB certifications

---

## Hardware Characteristics

### Tester Power Path

**Measured resistances:**

-   PCB traces: 170mm × 0.5mm = **~1.0Ω**
-   Standard USB cable (2m): **~1.0Ω**
-   **Total series resistance: ~2.0Ω**

**Impact on measurements:**

-   At 500mA load: **~1V voltage drop in test fixture itself**
-   Good USB port (5.0V idle) will measure **~4.0V** at 500mA load
-   This is expected and accounted for in test limits

### Load Circuit

-   PWM-controlled electronic load
-   Load resistor: 0.51Ω
-   Load range: 0-500mA in 100mA steps
-   Load is applied as step (not ramped) to measure transient response

### Measurement Circuit

-   ADC: RP2040 internal 12-bit ADC (80kS/s)
-   Voltage divider: 68kΩ / 100kΩ (1.68:1 ratio)
-   VBUS measurement range: 0-5.5V
-   Resolution: ~1.3mV per ADC count
-   Capture window: 120ms (15ms transient + 105ms steady-state)

---

## Test Limits Explained

### Why Limits Are Relaxed

The test fixture itself has ~2Ω resistance, causing ~1V drop at 500mA. We cannot measure USB spec compliance (4.75V minimum) because our fixture drops the voltage below that.

**Test limits are calibrated to detect problems WORSE than the fixture:**

| Parameter      | USB 2.0 Spec | This Tester | Why Different                                         |
| -------------- | ------------ | ----------- | ----------------------------------------------------- |
| Min voltage    | 4.75V        | **4.10V**   | Fixture drops 0.9V; allows host to drop another ~0.2V |
| Max ripple     | 100mVpp      | **50mVpp**  | Flag noisy supplies                                   |
| Min current    | 500mA        | **400mA**   | Must deliver reasonable current                       |
| Max resistance | N/A          | **3000mΩ**  | Detects dirty/corroded contacts or damaged cables     |
| R variation    | N/A          | **500mΩ**   | Flags power supply instability vs pure resistive drop |

### What PASS Means

**PASS = Port is probably fine for normal use**

-   Port provides VBUS voltage
-   Can deliver at least 400mA
-   Data communication works
-   No obvious electrical problems

**FAIL = Port has a grave problem**

-   No VBUS detected
-   Cannot deliver required current
-   Voltage sags excessively (much worse than normal)
-   Data errors during bulk transfer
-   Overcurrent protection tripping prematurely
-   Dirty/corroded contacts (mean resistance > 3Ω)
-   Power supply instability (resistance varies > 500mΩ across loads)

### What PASS Does NOT Mean

-   ❌ Port meets USB 2.0 specification
-   ❌ Port suitable for USB certification
-   ❌ Port will work with all USB devices
-   ❌ Port power quality is good

A port can PASS this test but still be out of USB spec. This is acceptable because the goal is to catch **obvious failures**, not certify compliance.

---

## Understanding Test Results

### Output Format

The test outputs results in this format:

```
USB Port {N} — {PASS/FAIL}: {throughput} Mbps, Idle {V}V, Vmin {V}V, ripple {mVpp}mVpp, Imax {mA}mA, R {mean}±{variation}mΩ
```

If the port fails, failure reasons are listed on separate lines below with `->` prefix.

### Voltage Measurements

**Example result from good port:**

```
USB Port 1 — PASS: 2.04 Mbps, Idle 5.0V, Vmin 4.02V, ripple 15mVpp, Imax 443mA, R 2010±45mΩ
```

**Breakdown:**

-   Idle: 5.0V (normal)
-   Vmin: 4.02V at maximum load
-   Drop: 980mV (mostly fixture)
-   Resistance: 2010mΩ ± 45mΩ (consistent ~2Ω)
-   Status: ✓ PASS

**Example result from port with dirty contacts:**

```
USB Port 2 — FAIL: 1.98 Mbps, Idle 5.0V, Vmin 3.95V, ripple 55mVpp, Imax 380mA, R 3200±150mΩ
  -> Voltage ripple 55mVpp > 50mVpp (noisy supply)
  -> Maximum measured current 380mA < 400mA (insufficient current capability)
  -> Mean resistance 3200mΩ > 3000mΩ - indicates dirty/corroded contacts or damaged cable
```

**Analysis:**

-   Resistance: 3200mΩ (1200mΩ more than fixture)
-   Extra resistance indicates dirty/corroded contacts or damaged cable
-   High ripple suggests additional power supply problems
-   Status: ✗ FAIL (multiple issues)

**Example result from port with power supply instability:**

```
USB Port 3 — FAIL: 2.01 Mbps, Idle 5.0V, Vmin 3.75V, ripple 20mVpp, Imax 410mA, R 2800±650mΩ
  -> Resistance varies 650mΩ (2150-2800mΩ) - indicates power supply issue, not pure resistive drop
```

**Analysis:**

-   Resistance variation: 650mΩ (highly inconsistent)
-   Pure resistor would show constant ~2000mΩ across all loads
-   Varying resistance indicates power supply cannot maintain regulation
-   Status: ✗ FAIL (weak/unstable supply)

### Current Measurements

The tester attempts to draw 100, 200, 300, 400, 500mA. The measured current should be close to the target:

```
Target: 500mA
Measured: 443mA
```

If measured current is much lower than target, the port may:

-   Have current limiting enabled
-   Have high series resistance
-   Be in overcurrent protection

### Resistance Measurements

The tester calculates effective resistance for each load step using Ohm's law:

```
R (mΩ) = droop (mV) / current (mA) × 1000
```

**What resistance tells us:**

1. **Mean resistance (~2000mΩ):** Expected value for clean fixture

    - Fixture has ~2Ω (2000mΩ) series resistance
    - Mean around 1800-2200mΩ is normal
    - Mean > 3000mΩ indicates dirty contacts or damaged cable

2. **Resistance variation (±45mΩ):** Consistency across load steps

    - Good: <500mΩ variation (pure resistive behavior)
    - Problem: >500mΩ variation (power supply instability)

**Why this matters:**

-   **Pure resistor**: Resistance stays constant regardless of load
-   **Weak power supply**: Resistance appears to increase as supply sags under load
-   **Dirty contacts**: Mean resistance elevated but still consistent

**Examples:**

| Mean R  | Variation | Interpretation                                  |
| ------- | --------- | ----------------------------------------------- |
| 2010mΩ  | ±45mΩ     | ✓ Normal fixture (2Ω PCB + cable)              |
| 2500mΩ  | ±60mΩ     | ✓ Slightly higher but consistent (acceptable)   |
| 3200mΩ  | ±150mΩ    | ✗ Dirty contacts or damaged cable               |
| 2800mΩ  | ±650mΩ    | ✗ Power supply cannot maintain regulation       |
| 1200mΩ  | ±800mΩ    | ✗ Extremely unstable supply (critical failure)  |

### Ripple Measurements

**Ripple:** Voltage noise during steady operation

-   Good: <50mVpp (clean supply)
-   Problem: >50mVpp (noisy power supply or poor filtering)

---

## Typical Results

### Desktop PC (Good Motherboard)

```
USB Port 1 — PASS: 2.04 Mbps, Idle 5.1V, Vmin 4.02V, ripple 15mVpp, Imax 443mA, R 2010±45mΩ
USB Port 2 — PASS: 2.02 Mbps, Idle 5.0V, Vmin 4.01V, ripple 18mVpp, Imax 443mA, R 1990±50mΩ
```

**Interpretation:** Healthy ports with clean power and consistent resistance matching fixture.

### Laptop with Weak Power Supply

```
USB Port 1 — FAIL: 2.01 Mbps, Idle 4.85V, Vmin 3.75V, ripple 25mVpp, Imax 420mA, R 2600±580mΩ
  -> Resistance varies 580mΩ (2020-2600mΩ) - indicates power supply issue, not pure resistive drop
```

**Interpretation:** Power supply cannot maintain regulation under load. Voltage sags as current increases.

### Port with Dirty Contacts

```
USB Port 1 — FAIL: 1.98 Mbps, Idle 5.0V, Vmin 3.92V, ripple 60mVpp, Imax 375mA, R 3350±120mΩ
  -> Voltage ripple 60mVpp > 50mVpp (noisy supply)
  -> Maximum measured current 375mA < 400mA (insufficient current capability)
  -> Mean resistance 3350mΩ > 3000mΩ - indicates dirty/corroded contacts or damaged cable
```

**Interpretation:** Contacts or cable have 1.3Ω extra resistance (dirty/corroded). Clean contacts or replace cable.

### Port with Overcurrent Protection Issue

```
USB Port 1 — FAIL: 1.98 Mbps, Idle 5.05V, OCP triggered at 60% load (300mA)
```

**Interpretation:** Port current limit set too low or malfunctioning. Cannot deliver required current.

### Dead Port

```
USB Port 1 — FAIL: VBUS too low to test (450mV < 4800mV)
```

**Interpretation:** Port not providing VBUS. Complete hardware failure.

---

## Tester Limitations

### Known Limitations

1. **Cannot verify USB spec compliance**

    - Fixture resistance too high (2Ω)
    - Would need <0.35Ω fixture for spec testing

2. **Voltage measurements include fixture drop**

    - 4.0V reading = 5.0V at host (good)
    - 3.8V reading = 4.8V at host (marginal)
    - Cannot directly measure host voltage

3. **Single load profile**

    - Tests with 0.51Ω resistive load
    - Does not test capacitive or inductive loads
    - Does not test USB power delivery (PD) protocols

4. **Basic data testing only**
    - Simple bulk loopback test
    - Does not test all USB protocols
    - No enumeration stress testing

### What This Means in Practice

**Use this tester to:**

-   ✓ Quickly screen incoming computers for dead/broken ports
-   ✓ Identify ports that obviously won't work
-   ✓ Catch current limiting problems
-   ✓ Detect major power supply issues

**Do NOT use this tester to:**

-   ✗ Certify USB 2.0 compliance
-   ✗ Qualify ports for sensitive equipment
-   ✗ Measure precise voltage/current values
-   ✗ Test USB-C power delivery features

---

## Reference: Fixture Resistance Calculation

### PCB Trace Resistance

```
Trace: 170mm length × 0.5mm width × 35µm thick (1oz copper)
Copper resistivity: 1.68×10⁻⁸ Ω·m

Calculated: 163mΩ (ideal trace only)
Measured: 1.0Ω (includes vias, joints, connectors)

The difference comes from:
  - Via resistance (~10-20mΩ each)
  - Solder joint resistance
  - Connector contact resistance
```

### Cable Resistance

```
2m USB cable, unknown gauge (likely 28AWG)
Measured: 1.0Ω total (VBUS + GND return path)

Typical values:
  - 28AWG: 0.8-1.2Ω per 2m
  - 24AWG: 0.3-0.5Ω per 2m
  - 20AWG: 0.12-0.2Ω per 2m
```

### Expected Voltage Drops

At various load currents:

| Load  | Fixture Drop | Host @ 5.0V | Measured |
| ----- | ------------ | ----------- | -------- |
| 0mA   | 0mV          | 5.00V       | ~5.00V   |
| 100mA | 200mV        | 4.80V       | ~4.80V   |
| 200mA | 400mV        | 4.60V       | ~4.60V   |
| 300mA | 600mV        | 4.40V       | ~4.40V   |
| 400mA | 800mV        | 4.20V       | ~4.20V   |
| 500mA | 1000mV       | 4.00V       | ~4.00V   |

_Assumes ideal 5.0V host with no droop_

---

## Calibration & Validation

This tester is calibrated by measuring known resistances and comparing to multimeter readings.

**To validate tester accuracy:**

1. **Test with known good USB port:**

    - Should measure 4.8-5.2V idle
    - Should deliver 400-500mA
    - Should show ~1V droop at 500mA
    - Should pass throughput test

2. **Test with bench power supply:**

    - Set supply to 5.0V, current limit 1A
    - Connect through same cable
    - Measure voltage at supply output
    - Compare to tester readings
    - Difference should be ~fixture drop

3. **Measure fixture resistance directly:**
    - Disconnect cable from both ends
    - Measure VBUS to VBUS: should be ~1Ω
    - Measure GND to GND: should be ~1Ω

---

## Summary

This is a **go/no-go field tester**, not a precision instrument. It's designed to quickly identify obviously broken USB ports on computers being checked in. Test limits are intentionally relaxed to account for the 2Ω test fixture resistance. A passing result means "port is probably fine for normal use", not "port meets USB specification".

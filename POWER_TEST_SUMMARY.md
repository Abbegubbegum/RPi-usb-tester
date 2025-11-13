# USB Power Test - Improvements Summary

## What Was Changed

This document describes improvements made to the USB power test methodology to properly measure transient load response and detect grave port failures.

---

## 1. Test Methodology Fixed (Firmware)

**File:** [src/main.c](src/main.c#L514-L626)

### Previous Behavior (Incorrect)

The test ramped load continuously without returning to idle:

```
0mA → 100mA → 200mA → 300mA → 400mA → 500mA
```

Problems:

-   Only measured transitions between load levels (e.g., 200mA→300mA)
-   Never measured from idle, so couldn't see true load step response
-   Recovery time always 0µs (meaningless)
-   Didn't detect transient voltage problems

### New Behavior (Correct)

Each load step now starts from idle:

```c
for each load (100, 200, 300, 400, 500 mA):
    1. Turn off load (0mA)
    2. Wait 150ms for VBUS to fully recover
    3. Apply load instantly (step, not ramp)
    4. Capture 120ms: 15ms transient + 105ms steady-state
    5. Measure droop, recovery, steady voltage, ripple
```

This properly measures what happens when a device suddenly draws power, which is how real USB devices behave.

---

## 2. Test Limits Calibrated for Field Testing

**File:** [scripts/bulk_test.py](scripts/bulk_test.py#L11-L42)

### Purpose of This Tester

This is a **field diagnostic tool** for detecting obviously broken USB ports. It is NOT a USB compliance certification tool.

**Detects:**

-   ✓ Dead ports (no VBUS)
-   ✓ Current limiting problems
-   ✓ Excessive voltage sag
-   ✓ Data communication failures
-   ✓ Dirty or corroded contacts
-   ✓ Power supply instability

**Does NOT:**

-   ✗ Certify USB 2.0 compliance
-   ✗ Measure to spec accuracy

### Why Limits Are Relaxed

**The test fixture has 2Ω resistance:**

-   PCB traces: 1.0Ω
-   USB cable: 1.0Ω
-   Total: 2.0Ω

**At 500mA load: fixture drops 1V by itself**

This means we **cannot** test for USB spec compliance (4.75V minimum) because our fixture already drops voltage below that. Instead, limits are set to catch problems **worse than the fixture**.

### Test Limits

| Parameter      | USB 2.0 Spec | Field Tester | Why                                         |
| -------------- | ------------ | ------------ | ------------------------------------------- |
| Min voltage    | 4.75V        | **4.10V**    | Fixture drops 0.9V; allow host another 0.2V |
| Max ripple     | 100mVpp      | **50mVpp**   | Flag noisy supplies                         |
| Min current    | 500mA        | **400mA**    | Must deliver reasonable current             |
| Max resistance | N/A          | **3000mΩ**   | Detects dirty/corroded contacts             |

**What PASS means:** Port is probably fine for normal use
**What FAIL means:** Port has an obvious problem

---

## 3. Documentation Added

### [HARDWARE_NOTES.md](HARDWARE_NOTES.md)

Complete explanation of:

-   What this tester is for (field diagnostics)
-   Hardware characteristics (2Ω fixture)
-   Why limits are relaxed
-   How to interpret results
-   Typical results from good/bad ports
-   Tester limitations

---

## How to Use

### Running Tests

```bash
# Run test (uses field tester limits)
python3 scripts/bulk_test.py

# Results saved to usb_report.json
```

### Understanding Results

**Good port (typical):**

```
USB Port 1 — PASS: 2.04 Mbps, Idle 5.0V, Vmin 4.02V, ripple 15mVpp, Imax 443mA, R 2010±45mΩ
```

**What this means:**

-   Idle: 5.0V (normal)
-   Under load: 4.02V (after fixture drops 1V)
-   Ripple: 15mVpp (low noise, healthy supply)
-   Current: 443mA (close to 500mA target)
-   Resistance: 2010mΩ ± 45mΩ (consistent ~2Ω fixture)
-   **PASS:** Port is functional

**Failing port with dirty contacts:**

```
USB Port 2 — FAIL: 1.98 Mbps, Idle 5.0V, Vmin 3.95V, ripple 55mVpp, Imax 380mA, R 3200±150mΩ
  -> Voltage ripple 55mVpp > 50mVpp (noisy supply)
  -> Maximum measured current 380mA < 400mA (insufficient current capability)
  -> Mean resistance 3200mΩ > 3000mΩ - indicates dirty/corroded contacts or damaged cable
```

**What this means:**

-   Resistance: 3200mΩ (1200mΩ more than fixture)
-   Extra resistance indicates dirty contacts or damaged cable
-   High ripple suggests noisy power supply
-   **FAIL:** Port has multiple problems

**Port with power supply instability:**

```
USB Port 3 — FAIL: 2.01 Mbps, Idle 5.0V, Vmin 3.75V, ripple 20mVpp, Imax 410mA, R 2800±650mΩ
  -> Resistance varies 650mΩ (2150-2800mΩ) - indicates power supply issue, not pure resistive drop
```

**What this means:**

-   Resistance variation: 650mΩ (inconsistent)
-   Pure resistor would show constant resistance
-   Varying resistance indicates weak/unstable power supply
-   **FAIL:** Power supply cannot maintain regulation under load

**Dead port:**

```
USB Port 4 — FAIL: VBUS too low to test (450mV < 4800mV)
```

**What this means:**

-   No VBUS detected
-   **FAIL:** Port hardware failure

---

## Technical Details

### What The Test Measures

**For each load step (20%, 40%, 60%, 80%, 100% PWM duty cycle):**

Each step corresponds to a target current where 100% = 500mA:
- 20% = ~100mA
- 40% = ~200mA
- 60% = ~300mA
- 80% = ~400mA
- 100% = ~500mA

**Measurements taken:**

1. **Voltage under load (mV):** Steady-state voltage during load

    - Expected: ~4.0-4.8V (depends on host and load)
    - Problem: <4.0V (excessive sag)

2. **Ripple (mVpp):** Voltage noise during steady operation

    - Good: <50mVpp
    - Problem: >50mVpp (noisy supply)

3. **Current (mA):** Actual measured current draw

    - Expected: Close to target (within ~10%)
    - Problem: Much lower than target (current limiting or high resistance)

4. **Resistance (mΩ):** Calculated R = droop / current × 1000

    - Expected: ~2000mΩ ± 500mΩ (fixture resistance)
    - Problem: >3000mΩ (dirty contacts) or highly variable (power supply issue)

5. **Resistance consistency:** Variation across load steps

    - Good: <500mΩ variation (pure resistive drop)
    - Problem: >500mΩ variation (power supply cannot maintain regulation)

### Expected Results with This Fixture

Assuming perfect 5.0V host with no internal droop:

| Load  | Fixture Drop | Measured VBUS | Status |
| ----- | ------------ | ------------- | ------ |
| 0mA   | 0mV          | 5.00V         | -      |
| 100mA | 200mV        | 4.80V         | PASS   |
| 200mA | 400mV        | 4.60V         | PASS   |
| 300mA | 600mV        | 4.40V         | PASS   |
| 400mA | 800mV        | 4.20V         | PASS   |
| 500mA | 1000mV       | 4.00V         | PASS   |

All of these will **PASS** because they're within expected fixture tolerance.

---

## What Changed in the Code

### Firmware ([src/main.c](src/main.c))

**Key changes:**

-   Line 571-572: Turn off load and wait 150ms between steps
-   Line 576: Apply load instantly (removed ramping)
-   Line 593-594: Better debug output showing droop and recovery

**Result:** Now measures true transient response like real USB devices.

### Python Test Script ([scripts/bulk_test.py](scripts/bulk_test.py))

**Key changes:**

-   Lines 35-44: Updated test limits for field tester
-   Added resistance measurement and consistency checking
-   Removed all command-line argument parsing (argparse) - limits are hardcoded
-   Updated output format: status appears immediately after port number
-   Failure reasons printed on separate lines with `->` prefix
-   Now shows resistance: `R {mean}±{variation}mΩ`

**Result:** Tests detect grave failures including dirty contacts and power supply instability.

### Firmware Constants ([src/include.h](src/include.h))

**Key changes:**

-   Lines 72-88: Updated power_report_t struct
-   Renamed `loads_mA` → `load_pct` (stores PWM duty cycle percentage)
-   Added `resistance_mOhm[5]` field for calculated resistance values
-   Split voltage limits: IDLE_UNDERVOLT_MV = 4800mV, LOAD_UNDERVOLT_MV = 4000mV
-   Struct size increased from 71 to 81 bytes

**Result:** Firmware calculates resistance per load step to detect contact/supply issues.

---

## Validation

To verify the tester is working correctly:

**1. Test with known good port:**

-   Should show ~5V idle
-   Should show ~4V at 500mA load
-   Should PASS

**2. Test with bench power supply:**

-   Set to 5.0V, 1A limit
-   Measure at supply: should read 5V
-   Measure with tester: should read ~4V
-   Difference = fixture drop (1V) ✓

**3. Test with marginal port:**

-   Should catch obvious problems
-   May still PASS ports that are out of USB spec
-   This is expected (field tester, not compliance)

---

## Summary

### Before These Changes

-   ❌ Test measured ramped transitions (wrong)
-   ❌ Recovery time always 0µs (meaningless)
-   ❌ Couldn't detect transient problems
-   ❌ Test limits didn't account for fixture

### After These Changes

-   ✅ Test measures true load step response
-   ✅ Recovery time meaningful
-   ✅ Detects transient voltage problems
-   ✅ Limits calibrated for this specific fixture
-   ✅ Clear documentation of purpose and limitations
-   ✅ Resistance measurement distinguishes fixture vs contact/supply issues
-   ✅ Detects dirty/corroded contacts (R > 3000mΩ)
-   ✅ Detects power supply instability (resistance variation > 500mΩ)
-   ✅ Simplified test script with no command-line arguments
-   ✅ Clear output format with status first and detailed failure reasons

### What This Means

The tester now **correctly identifies obviously broken USB ports** while accounting for its own 2Ω resistance. It's a go/no-go field diagnostic tool that catches grave errors, not a compliance certification tool.

A **PASS** means: "Port is probably fine for normal use"
A **FAIL** means: "Port has an obvious problem"

For complete details, see [HARDWARE_NOTES.md](HARDWARE_NOTES.md).

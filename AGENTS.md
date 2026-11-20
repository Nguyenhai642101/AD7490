# AI Agent & Automated Auditor Guide (`AGENTS.md`)

This file provides structured context and verification instructions for any AI coding agent or automated review system inspecting or modifying this repository.

---

## 1. Repository Purpose & Scope

This repository implements a **safety-critical bare-metal C99 driver and periodic monitor** for the **Analog Devices AD7490** (16-channel, 12-bit, 1 MSPS SAR ADC) connected via a **Xilinx AXI Quad SPI v3.2 (PG153)** controller on an **ARM Cortex-A53** processor.

All code in this repository complies with **[`docs/C_rules_v3_addDO178C.md`](docs/C_rules_v3_addDO178C.md)**, which unifies:
- **DO-178C DAL A** (bi-directional traceability, MC/DC testability, deterministic WCET loops, SEU bit-flip resilience, deactivated/defensive code control, deterministic `-O1 -fno-inline` optimization profile)
- **MISRA C:2012**
- **BARR-C:2018**

---

## 2. Directory Layout

```text
.
├── Makefile                               # Target library (Cortex-A53), host unit tests, gcovr, style
├── README.md                              # Project overview, architecture, API usage, and build guide
├── AGENTS.md                              # AI Agent verification & maintenance guide (this file)
├── hal/
│   ├── axi_qspi_hal.h                     # Public API for Xilinx AXI Quad SPI v3.2 (8-bit, polled)
│   └── axi_qspi_hal.c                     # HAL implementation (LLR-QSPI-001 .. LLR-QSPI-031)
├── driver/
│   ├── ad7490_driver.h                    # Public API for AD7490 single read, power modes, sequencer
│   ├── ad7490_driver.c                    # AD7490 driver implementation (LLR-AD7490-001 .. LLR-AD7490-054)
│   ├── ad7490_monitor.h                   # Public API for 1 ms periodic scan, 16-sample moving average, 3-retry fault recovery
│   └── ad7490_monitor.c                   # Monitor implementation (LLR-AD7490-060 .. LLR-AD7490-081)
├── test/
│   ├── check_style.awk                    # Automated Rule 0 & Allman style checker used by 'make style'
│   ├── test_common.h, test_common.c       # Minimal host test harness & ASSERT contract hook
│   ├── fake_ad7490.h, fake_ad7490.c       # 1-frame pipeline ADC simulator & fault injector
│   ├── test_qspi_hal.c                    # Unit tests for hal/axi_qspi_hal.c (MMIO register model)
│   ├── test_frame_convert.c               # Unit tests for frame encoding & 835,584 exhaustive code->mV conversions
│   ├── test_device.c                      # Unit tests for ad7490_init, ad7490_read_channel, ad7490_set_power_mode
│   ├── test_sequencer.c                   # Unit tests for ad7490_sequencer_start / _read_scan / _stop & CR scrubbing
│   └── test_monitor.c                     # Unit tests for 16-sample moving average & 3-attempt fault recovery
├── docs/
│   ├── C_rules_v3_addDO178C.md            # Authoritative coding rules (MISRA + BARR-C + DO-178C)
│   ├── AD7490.PDF                         # Analog Devices AD7490 hardware datasheet
│   ├── LOW_LEVEL_REQUIREMENTS.md          # Complete Low-Level Requirements (LLR-QSPI-*, LLR-AD7490-*)
│   ├── SOFTWARE_DESIGN_DESCRIPTION.md     # Complete Software Design Description (SDD-QSPI-*, SDD-AD7490-*)
│   ├── TRACEABILITY_MATRIX.md             # Bi-directional traceability matrix (@req <-> @design <-> code <-> tests)
│   ├── TEST_REPORT.md                     # Unit test & structural coverage verification report
│   └── DEVIATION_AND_DEFENSIVE_LOG.md     # MISRA C:2012 deviation log, DO-178C defensive branches, SEU Hamming distances
└── scripts/
    ├── verify_compliance.py               # Cross-platform Python 3 static rule & traceability verifier
    └── verify_compliance.ps1              # Windows PowerShell static rule & traceability verifier
```

---

## 3. Hardware & Protocol Invariants (`docs/AD7490.PDF` + Xilinx PG153)

When auditing the code for functional correctness against `docs/AD7490.PDF` and Xilinx PG153, verify these invariants:

1. **SPI Bus Framing & Clock Mode (`hal/axi_qspi_hal.c`, `driver/ad7490_driver.h`)**:
   - The AXI Quad SPI IP is configured for 8-bit transaction width (`ACLK = 100 MHz`, prescaler `16x1 -> SCK = 6.25 MHz`, FIFO depth `256 bytes`).
   - One AD7490 conversion frame is `16 SCLK = 2 bytes` transferred MSB-first under a single `SS0` assertion (`AD7490_FRAME_BYTES = 2U`).
   - The AD7490 latches `DIN` and updates `DOUT` on falling `SCLK` edges with `SCLK` idle high, requiring **SPI Mode 2 (`CPOL = 1, CPHA = 0` -> `QSPI_CLOCK_MODE_2`)**.
   - `SS0` is only deasserted after `read_rx_count()` confirms all `byte_count` bytes have shifted into the RX FIFO (never on `Tx_Empty`, which asserts while the last byte is still shifting).
2. **Control Register & Power-Up Sequence (`driver/ad7490_driver.c`)**:
   - The 12-bit Control Register (`WRITE`, `SEQ`, `ADD3..0`, `PM1..0`, `SHADOW`, `WEAK/TRI`, `RANGE`, `CODING`) is clocked in on the first 12 `SCLK` cycles (`DIN[15:4]`, `AD7490_CR_FRAME_SHIFT = 4U`).
   - Per AD7490 Figure 16, `ad7490_init()` sends **2 dummy frames with `DIN = 0xFFFF`** followed by the target Control Register write frame.
3. **1-Frame Conversion Pipeline (`driver/ad7490_driver.c`)**:
   - Frame $N$ converts the channel selected at the end of Frame $N-1$.
   - Therefore, an isolated `ad7490_read_channel()` executes **2 frames in `NORMAL`**: Frame 1 selects `channel_id`, Frame 2 converts it. In **`AUTO_STANDBY` / `AUTO_SHUTDOWN` it executes 3 frames** (Fig. 14, 15): the CR write of Frame 1 puts the part to sleep, Frame 2 is the power-up dummy (`WRITE = 0`, invalid DOUT, discarded) and Frame 3 (`WRITE = 0`) converts `channel_id`. The result frame checks `DOUT[15:12] == channel_id`.
4. **Shadow Register Sequencer & CR Scrubbing (`driver/ad7490_driver.c`)**:
   - In the AD7490 Shadow Register (Figure 18), **`VIN0` is bit 15 (MSB) and `VIN15` is bit 0 (LSB)**. `mask_to_shadow()` reverses the bit order from the API's `bit n = VINn` convention.
   - `ad7490_sequencer_start()` sends 3 frames: (1) CR with `SEQ=0, SHADOW=1`, (2) the 16-bit Shadow word, and (3) a priming frame (`WRITE=0`) so the next scan starts cleanly on sequenced channels.
   - The first frame of every `ad7490_sequencer_read_scan()` sends a **CR scrub word (`WRITE=1, SEQ=1, SHADOW=0`)** per Table IV, repairing any `RANGE`/`CODING`/`PM` bit upset by EMI/SEU within one pass without interrupting the sequence.
5. **Conversion Math (`raw_to_mv`)**:
   - `DOUT[11:0]` is converted to millivolts at the `VINx` pin (`0..REFIN` for `1x`, `0..2*REFIN` for `2x`, with `REFIN` validated in `2475..2525 mV`).
   - Twos complement (`CODING = 0`) is converted to offset binary by flipping `DB11` (`^ 0x0800U`), then scaled with round-to-nearest integer arithmetic: `((offset_code * span_mv) + 2048U) / 4096U`.
6. **Periodic Monitor & Bounded Fault Recovery (`driver/ad7490_monitor.c`)**:
   - Maintains a 16-sample ring buffer (`ad7490_avg_t`) per channel without a cached running sum (re-sums active slots `0..fill_count-1` on read so any transient RAM corruption expires after 16 samples and cannot drift).
   - Allows at most `AD7490_MON_MAX_RECONFIG = 3` re-configuration attempts per fault episode. The attempt counter resets to `0` only after `AD7490_MON_STABLE_SCANS = 16` consecutive good scans. Exceeding 3 attempts latches `AD7490_MON_HEALTH_FAILED` (`AD7490_ERR_DEVICE_FAILED`).

---

## 4. Verification Commands for Agents

### Step 1: Static Rule & Traceability Verification (No Compiler Required)
On Linux / macOS / Windows with Python 3:
```bash
python3 scripts/verify_compliance.py
```
On Windows PowerShell (zero external dependencies):
```powershell
powershell -ExecutionPolicy Bypass -File scripts/verify_compliance.ps1
```

### Step 2: Style Check, Unit Tests, Target Build & Coverage (Linux / WSL / MSYS2 with GCC)
```bash
# Run Rule 0 style checks (80 cols, Allman, no //, no tabs, no "../" include,
# single trailing newline) via test/check_style.awk
make style

# Build and run all 5 host unit-test suites (test_qspi_hal, test_frame_convert, test_device, test_sequencer, test_monitor)
make test

# Generate line & branch coverage report via gcovr (hal/ and driver/)
make coverage

# MC/DC condition coverage (needs gcc-14 / gcov-14 and gcovr >= 8)
make mcdc

# Cross-compile Cortex-A53 static library and inspect per-function stack usage (.su)
make target CROSS_COMPILE=aarch64-none-elf-
make stack  CROSS_COMPILE=aarch64-none-elf-
```

---

## 5. Rules When Modifying Code

Before making or proposing any code change, read **[`docs/C_rules_v3_addDO178C.md`](docs/C_rules_v3_addDO178C.md)** §14 (Self-check) and run `scripts/verify_compliance.py` (or `.ps1`):
- Never introduce `//` comments, tabs, CRLF line endings, or lines > 80 characters.
- Never use cuddled braces (`if (...) {`); every `{` and `}` must sit on its own line.
- Never use identifiers shorter than 3 characters (except loop indices `i`, `j`, `k`) or forbidden stems (`value`, `val`, `data`, `tmp`, `buf`, `info`, `temp`).
- Keep a single `return` at the end of every function, and update `docs/LOW_LEVEL_REQUIREMENTS.md`, `docs/SOFTWARE_DESIGN_DESCRIPTION.md`, and `docs/TRACEABILITY_MATRIX.md` if any `@req` or `@design` tag is added or modified.

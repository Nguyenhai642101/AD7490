# MISRA Deviation Log & DO-178C Defensive Code Log

This document records all approved deviations from `C_rules_v3_addDO178C.md` (following *MISRA Compliance:2020*), all DO-178C defensive code branches (`[DO §6.4.4.2, §6.4.4.3]`), and the Single-Event Upset (SEU) Hamming distance verification (`[DO §2.4, §6.3.4]`).

**Reference convention:** every entry locates its site by **file and element name** (function, or struct field) plus its `LLR` / `SDD` IDs — never by line number, which silently drifts on every edit above the site. Each site carries its entry ID in the source comment (`deviation log entry DEV-…`, `defensive log entry DEF-…`), so the link holds in both directions and can be checked with `grep`.

---

## 1. MISRA C:2012 & Project Rule Deviations

| Deviation ID | Rule | Element (file, `LLR` / `SDD`) | Reason | Safety Justification & Mitigation |
|:---|:---|:---|:---|:---|
| `DEV-QSPI-001` | MISRA C:2012 Rule 11.4 (Conversion between pointer to object and integer type) | `reg_write32()` in `hal/axi_qspi_hal.c` (`LLR-QSPI-020`, `SDD-QSPI-3.1`) | Memory-mapped I/O register write requires converting `uintptr_t base_addr + offset_bytes` to `volatile uint32_t *`. | Isolated to a single `static inline` helper in the HAL (§1, §9). `base_addr` is validated `!= 0` and 4-byte aligned (`0U == (base_addr & 0x3U)`) by `is_hal_cfg_ok()` before any access. |
| `DEV-QSPI-002` | MISRA C:2012 Rule 11.4 (Conversion between pointer to object and integer type) | `reg_read32()` in `hal/axi_qspi_hal.c` (`LLR-QSPI-021`, `SDD-QSPI-3.2`) | Memory-mapped I/O register read requires converting `uintptr_t base_addr + offset_bytes` to `volatile uint32_t const *`. | Isolated to a single `static inline` helper in the HAL (§1, §9). `base_addr` is validated `!= 0` and 4-byte aligned (`0U == (base_addr & 0x3U)`) by `is_hal_cfg_ok()` before any access. |
| `DEV-AD7490-001` | Project Rule §11.1 (Raw sensor value escaping `.c` file) | Field `raw_lsb` of `ad7490_sample_t` in `driver/ad7490_driver.h` (`LLR-AD7490-008`, `SDD-AD7490-1.7`) | System Built-In Test (BIT) and calibration logging require access to the raw 12-bit ADC code alongside `voltage_mv`. | `raw_lsb` is a read-only diagnostic copy extracted from the exact same 16-bit DOUT frame as `voltage_mv`. All millivolt scaling is performed once inside `ad7490_driver.c` (`raw_to_mv()`). |

---

## 2. DO-178C Defensive Code Branches

Every defensive branch required by MISRA C:2012 (`[M 15.7]` final `else`, `[M 16.4]` `switch default:`) that guards against hardware bit-flips (SEU) or memory corruption is tagged in source with `/* DEFENSIVE [DO-178C]: ... defensive log entry DEF-… */` and verified directly via unit tests in `test_qspi_hal.c` and `test_frame_convert.c`.

| Defensive ID | Element (file, `LLR` / `SDD`) | Guarded Fault | Verification Method |
|:---|:---|:---|:---|
| `DEF-QSPI-001` | Final `else` of `build_control_word()` in `hal/axi_qspi_hal.c` (`LLR-QSPI-024`, `SDD-QSPI-3.5`) | RAM bit-flip corrupting validated `clock_mode` (`mode_idx >= QSPI_CLOCK_MODE_COUNT`). | Unit test `test_qspi_hal.c:test_init_sequence_and_faults()` calls `build_control_word((qspi_clock_mode_t)4U)` and verifies safe fallback to base control word. |
| `DEF-AD7490-001` | Final `else` of `encode_cr_word()` in `driver/ad7490_driver.c` (`LLR-AD7490-030`, `SDD-AD7490-3.11`) | RAM bit-flip corrupting internal `seq_mode` (`seq_idx >= AD7490_SEQ_MODE_COUNT`). | Unit test `test_frame_convert.c:test_cr_rejects_bad_enum()` calls `encode_cr_word(&good, 0U, (ad7490_seq_mode_t)4U)` and verifies safe fallback (`SEQ=0, SHADOW=0`). |
| `DEF-AD7490-002` | `switch` `default:` of `map_hal_status()` in `driver/ad7490_driver.c` (`LLR-AD7490-034`, `SDD-AD7490-3.15`) | RAM/register bit-flip producing an undefined `qspi_hal_status_t` enum value in `switch (hal_status)`. | Unit test `test_frame_convert.c:test_hal_status_map()` injects `(qspi_hal_status_t)9U` and verifies fail-safe mapping to `AD7490_ERR_SPI`. |

---

## 3. SEU / Bit-Flip Resilience (Rule §9 Verification)

Rule §9 requires critical state patterns to have a pairwise Hamming distance $\ge 4$ (and separated from `0x00000000` and `0xFFFFFFFF`).

| Pattern Constant | Hex Value | Min Hamming Distance to Other States | Hamming Distance to `0x00000000` | Hamming Distance to `0xFFFFFFFF` |
|:---|:---|:---|:---|:---|
| `AD7490_STATE_READY` | `0x5A3C96E1U` | 20 (`AD7490_STATE_SEQUENCING`), 22 (`AD7490_STATE_FAULT`) | 16 | 16 |
| `AD7490_STATE_SEQUENCING` | `0x3CA5E196U` | 20 (`AD7490_STATE_READY`), 22 (`AD7490_STATE_FAULT`) | 16 | 16 |
| `AD7490_STATE_FAULT` | `0xC35A693CU` | 22 (`AD7490_STATE_READY`), 22 (`AD7490_STATE_SEQUENCING`) | 16 | 16 |
| `AD7490_MON_HEALTH_ACTIVE` | `0x96E15A3CU` | 32 (`AD7490_MON_HEALTH_FAILED`), $\ge 14$ to all `AD7490_STATE_*` | 16 | 16 |
| `AD7490_MON_HEALTH_FAILED` | `0x691EA5C3U` | 32 (`AD7490_MON_HEALTH_ACTIVE`), $\ge 12$ to all `AD7490_STATE_*` | 16 | 16 |

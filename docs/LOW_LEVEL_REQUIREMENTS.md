# Low-Level Requirements Specification (DO-178C §5.1, §11.8)

**Project:** AD7490 16-Channel 12-Bit SAR ADC Bare-Metal Driver & Periodic Monitor
**Target Platform:** ARM Cortex-A53 Bare-Metal + Xilinx AXI Quad SPI v3.2 (PG153)
**Design Assurance Level:** DO-178C DAL A
**Hardware References:**
- Analog Devices AD7490 Datasheet ([`docs/AD7490.PDF`](AD7490.PDF))
- Xilinx LogiCORE IP AXI Quad SPI v3.2 Product Guide (`PG153`)
- Coding Standard ([`docs/C_rules_v3_addDO178C.md`](C_rules_v3_addDO178C.md))

---

## 1. AXI Quad SPI HAL Requirements (`LLR-QSPI-*`)

### 1.1 Module & Public Types (`hal/axi_qspi_hal.h`)

| Requirement ID | Title | Specification | Design Trace |
|:---|:---|:---|:---|
| `LLR-QSPI-001` | QSPI HAL Module Scope & Memory Mapping | The HAL module shall control a Xilinx AXI Quad SPI v3.2 IP configured in Standard SPI master mode, 8-bit transaction width, 256-byte TX/RX FIFOs, 1 slave select (`SS0`), and `100 MHz` AXI `ACLK` (`SCK = 6.25 MHz` with `16x1` prescaler). The MMIO register window shall be mapped as `Device-nGnRE` (or stronger). | `SDD-QSPI-1.0` |
| `LLR-QSPI-002` | QSPI HAL Status Codes | The HAL shall define `qspi_hal_status_t` with `QSPI_HAL_OK = 0U`, `QSPI_HAL_ERR_PARAM = 1U`, `QSPI_HAL_ERR_TIMEOUT = 2U`, and `QSPI_HAL_ERR_HW = 3U`. | `SDD-QSPI-1.1` |
| `LLR-QSPI-003` | SPI Clock Polarity & Phase Modes | The HAL shall define `qspi_clock_mode_t` for SPI Modes 0..3 (`QSPI_CLOCK_MODE_0 = 0U` [`CPOL=0, CPHA=0`], `QSPI_CLOCK_MODE_1 = 1U` [`CPOL=0, CPHA=1`], `QSPI_CLOCK_MODE_2 = 2U` [`CPOL=1, CPHA=0`], `QSPI_CLOCK_MODE_3 = 3U` [`CPOL=1, CPHA=1`]). | `SDD-QSPI-1.2` |
| `LLR-QSPI-004` | QSPI Instance Configuration Structure | The HAL shall define `qspi_hal_config_t` containing `uintptr_t base_addr` (`!= 0`, 4-byte aligned), `qspi_clock_mode_t clock_mode` (`0..3`), and `uint32_t poll_timeout_us` (`1..1,000,000 us` where `QSPI_HAL_MAX_TIMEOUT_US = 1000000U`). | `SDD-QSPI-1.3` |
| `LLR-QSPI-005` | Internal Contract Assertion Macro | The HAL header shall define `ASSERT(expr)` compiling to `((void)0)` in flight/release builds (unless overridden by host unit-test flags) to verify internal helper preconditions without introducing dead branches into flight object code. | `SDD-QSPI-1.4` |

### 1.2 Public QSPI HAL Operations (`hal/axi_qspi_hal.h`, `hal/axi_qspi_hal.c`)

| Requirement ID | Title | Specification | Design Trace |
|:---|:---|:---|:---|
| `LLR-QSPI-010` | QSPI Core Reset & Master Initialisation (`qspi_hal_init`) | Given a non-`NULL` `p_cfg`, `qspi_hal_init()` shall validate `p_cfg` (`LLR-QSPI-022`), write `0x0000000AU` to `SRR` (`0x40`), wait 32 AXI reads (`wait_reset_done`), verify `SPICR` (`0x60`) returned to `0x00000180U`, disable interrupts (`GIER = 0U`), deassert `SS0` (`SPISSR = 0xFFFFFFFFU`), flush TX/RX FIFOs (`LLR-QSPI-025`), configure master mode with manual SS and master inhibit set, and verify `SPISR.MODF == 0`. Returns `QSPI_HAL_OK`, `QSPI_HAL_ERR_PARAM`, or `QSPI_HAL_ERR_HW`. WCET bound: 70 AXI register accesses. | `SDD-QSPI-2.1` |
| `LLR-QSPI-011` | Full-Duplex Byte Transfer (`qspi_hal_transceive_bytes`) | Given valid `p_cfg`, non-`NULL` `p_tx_bytes` and `p_rx_bytes`, and `byte_count` in `1..256`, `qspi_hal_transceive_bytes()` shall flush TX/RX FIFOs while inhibited, load `byte_count` bytes from `p_tx_bytes` into `SPIDTR`, execute one `SS0`-framed bus cycle (`LLR-QSPI-026`), and drain `byte_count` received bytes into `p_rx_bytes` (`LLR-QSPI-027`) only if that cycle returned `QSPI_HAL_OK`. `p_rx_bytes` shall hold `byte_count` valid bytes if and only if the function returns `QSPI_HAL_OK`; a drain aborted by `QSPI_HAL_ERR_HW` leaves the bytes already popped written and the rest unchanged (no roll-back), so the caller shall discard the whole array. Returns `QSPI_HAL_OK`, `QSPI_HAL_ERR_PARAM`, `QSPI_HAL_ERR_TIMEOUT`, or `QSPI_HAL_ERR_HW`. | `SDD-QSPI-2.2` |
| `LLR-QSPI-012` | Transmit-Only Byte Transfer (`qspi_hal_write_bytes`) | Given valid `p_cfg`, non-`NULL` `p_tx_bytes`, and `byte_count` in `1..256`, `qspi_hal_write_bytes()` shall flush TX/RX FIFOs while inhibited, queue `byte_count` bytes into `SPIDTR`, and execute one `SS0`-framed bus cycle (`LLR-QSPI-026`), leaving MISO bytes in the RX FIFO to be flushed on the next transfer. Returns `QSPI_HAL_OK`, `QSPI_HAL_ERR_PARAM`, `QSPI_HAL_ERR_TIMEOUT`, or `QSPI_HAL_ERR_HW`. | `SDD-QSPI-2.3` |
| `LLR-QSPI-013` | Receive-With-Dummy-MOSI Transfer (`qspi_hal_read_bytes`) | Given valid `p_cfg`, `dummy_tx_byte`, non-`NULL` `p_rx_bytes`, and `byte_count` in `1..256`, `qspi_hal_read_bytes()` shall flush TX/RX FIFOs while inhibited, fill `SPIDTR` with `dummy_tx_byte` `byte_count` times, execute one `SS0`-framed bus cycle, and drain `byte_count` bytes into `p_rx_bytes` (`LLR-QSPI-027`) only if that cycle returned `QSPI_HAL_OK`. `p_rx_bytes` shall hold `byte_count` valid bytes if and only if the function returns `QSPI_HAL_OK`; a drain aborted by `QSPI_HAL_ERR_HW` leaves the bytes already popped written and the rest unchanged (no roll-back), so the caller shall discard the whole array. Returns `QSPI_HAL_OK`, `QSPI_HAL_ERR_PARAM`, `QSPI_HAL_ERR_TIMEOUT`, or `QSPI_HAL_ERR_HW`. | `SDD-QSPI-2.4` |

### 1.3 Private QSPI HAL Helpers (`hal/axi_qspi_hal.c`)

| Requirement ID | Title | Specification | Design Trace |
|:---|:---|:---|:---|
| `LLR-QSPI-020` | 32-Bit MMIO Register Write (`reg_write32`) | `reg_write32()` shall be the sole MMIO write accessor for the QSPI IP, writing `reg_word` to `*(volatile uint32_t *)(base_addr + offset_bytes)`. | `SDD-QSPI-3.1` |
| `LLR-QSPI-021` | 32-Bit MMIO Register Read (`reg_read32`) | `reg_read32()` shall be the sole MMIO read accessor for the QSPI IP, reading `*(volatile uint32_t const *)(base_addr + offset_bytes)`. | `SDD-QSPI-3.2` |
| `LLR-QSPI-022` | QSPI Configuration Validation (`is_hal_cfg_ok`) | `is_hal_cfg_ok()` shall return `true` if and only if `p_cfg != NULL`, `base_addr != 0U`, `(base_addr & 0x3U) == 0U`, `clock_mode < 4U`, and `1U <= poll_timeout_us <= 1000000U`, with one atomic condition per branch for MC/DC testability. | `SDD-QSPI-3.3` |
| `LLR-QSPI-023` | FIFO Byte Count Validation (`is_byte_count_ok`) | `is_byte_count_ok()` shall return `true` if and only if `1U <= byte_count <= 256U` (`QSPI_HAL_FIFO_DEPTH_BYTES`). | `SDD-QSPI-3.4` |
| `LLR-QSPI-024` | SPICR Control Word Construction (`build_control_word`) | `build_control_word()` shall combine `SPE | MASTER | MANUAL_SS | INHIBIT` (`0x186U`) with the `CPOL`/`CPHA` bits from `clock_mode_ctrl_bits[clock_mode]`, guarding out-of-range indices with a DO-178C defensive branch. | `SDD-QSPI-3.5` |
| `LLR-QSPI-025` | FIFO Flush Under Master Inhibit (`reset_fifos`) | `reset_fifos()` shall write `ctrl_inhibit | TX_RESET | RX_RESET` to `SPICR`, hold the reset bits asserted for `QSPI_RESET_SETTLE_READS` (32) AXI reads of `SPICR` so the pulse is at least the IP reset pulse (16 ACLK), then write `ctrl_inhibit`, while `SS0` remains deasserted. WCET bound: 34 AXI register accesses. | `SDD-QSPI-3.6` |
| `LLR-QSPI-026` | Framed Bus Cycle & Poll Loop (`run_bus_cycle`) | `run_bus_cycle()` shall assert `SS0` (`0xFFFFFFFEU`), clear Master Inhibit to start `SCK`, poll `read_rx_count(base_addr)` up to `timeout_us * 100U` iterations until `rx_count >= byte_count`, re-inhibit the master, deassert `SS0` (`0xFFFFFFFFU`), read back `SPISSR` and `SPISR`, and return `QSPI_HAL_ERR_HW` if `MODF` is set or `SS0` is stuck low, `QSPI_HAL_ERR_TIMEOUT` if `rx_count < byte_count`, or `QSPI_HAL_OK`. | `SDD-QSPI-3.7` |
| `LLR-QSPI-027` | RX FIFO Byte Drain (`drain_rx_bytes`) | `drain_rx_bytes()` shall pop `byte_count` bytes from `SPIDRR` (`0x6C`), verifying before each pop that `SPISR.RX_EMPTY == 0`; if `RX_EMPTY` asserts prematurely before the pop of byte `i`, it shall abort and return `QSPI_HAL_ERR_HW`, leaving `p_rx_bytes[0 .. i-1]` written with the bytes already popped and `p_rx_bytes[i .. byte_count-1]` unchanged (no roll-back). | `SDD-QSPI-3.8` |
| `LLR-QSPI-028` | Software Reset Settle Wait (`wait_reset_done`) | `wait_reset_done()` shall perform 32 consecutive AXI reads of `SPICR` (`>= 2 x 16 ACLK` reset pulse width) and return `true` if and only if the final read equals `QSPI_SPICR_RESET_WORD` (`0x00000180U`). | `SDD-QSPI-3.9` |
| `LLR-QSPI-029` | TX FIFO Array Loader (`load_tx_fifo_array`) | `load_tx_fifo_array()` shall write `byte_count` bytes from `p_tx_bytes[0..byte_count-1]` sequentially into `SPIDTR` (`0x68`). | `SDD-QSPI-3.10` |
| `LLR-QSPI-030` | TX FIFO Constant Fill Loader (`load_tx_fifo_fill`) | `load_tx_fifo_fill()` shall write `fill_byte` into `SPIDTR` (`0x68`) `byte_count` times. | `SDD-QSPI-3.11` |
| `LLR-QSPI-031` | RX FIFO Occupancy Reader (`read_rx_count`) | `read_rx_count()` shall read `SPISR`; if `RX_EMPTY == 0`, it shall read `RXOCY` (`0x78`) and return `(RXOCY & 0xFFU) + 1U`, otherwise `0U`. | `SDD-QSPI-3.12` |

---

## 2. AD7490 Driver Requirements (`LLR-AD7490-001` .. `LLR-AD7490-054`)

### 2.1 Public Types & State Encoding (`driver/ad7490_driver.h`)

| Requirement ID | Title | Specification | Design Trace |
|:---|:---|:---|:---|
| `LLR-AD7490-001` | AD7490 Driver Module Scope | The driver module shall manage the Analog Devices AD7490 16-channel 12-bit SAR ADC over 16-bit (`2-byte`) SPI Mode 2 frames via `axi_qspi_hal`. | `SDD-AD7490-1.0` |
| `LLR-AD7490-002` | AD7490 Driver Status Codes | `ad7490_status_t` shall enumerate `AD7490_OK = 0U`, `AD7490_ERR_PARAM = 1U`, `AD7490_ERR_SPI = 2U`, `AD7490_ERR_TIMEOUT = 3U`, `AD7490_ERR_CHAN_MISMATCH = 4U`, `AD7490_ERR_STATE = 5U`, and `AD7490_ERR_DEVICE_FAILED = 6U`. | `SDD-AD7490-1.1` |
| `LLR-AD7490-003` | Power Management Mode Enumeration | `ad7490_power_mode_t` shall map 1-to-1 to AD7490 Table III `PM1:PM0` bit patterns: `AUTO_STANDBY = 0U`, `AUTO_SHUTDOWN = 1U`, `FULL_SHUTDOWN = 2U`, `NORMAL = 3U`. | `SDD-AD7490-1.2` |
| `LLR-AD7490-004` | Analog Input Range Enumeration | `ad7490_range_t` shall map 1-to-1 to the `RANGE` bit: `AD7490_RANGE_2X_VREF = 0U` (`0..2 x REFIN`) and `AD7490_RANGE_1X_VREF = 1U` (`0..REFIN`). | `SDD-AD7490-1.3` |
| `LLR-AD7490-005` | Output Coding Enumeration | `ad7490_coding_t` shall map 1-to-1 to the `CODING` bit: `AD7490_CODING_TWOS_COMPLEMENT = 0U` and `AD7490_CODING_STRAIGHT_BINARY = 1U`. | `SDD-AD7490-1.4` |
| `LLR-AD7490-006` | DOUT Line Mode Enumeration | `ad7490_dout_mode_t` shall map 1-to-1 to the `WEAK/TRI` bit: `AD7490_DOUT_TRI_STATE = 0U` and `AD7490_DOUT_WEAK_ADD3 = 1U`. | `SDD-AD7490-1.5` |
| `LLR-AD7490-007` | Device Configuration Structure | `ad7490_config_t` shall aggregate `qspi_hal_config_t qspi_cfg`, `uint32_t vref_mv` (`2475..2525 mV`), and `ad7490_ctrl_cfg_t ctrl_cfg`. | `SDD-AD7490-1.6` |
| `LLR-AD7490-008` | Conversion Sample Structure | `ad7490_sample_t` shall hold `uint8_t channel_id` (`0..15`), `uint16_t raw_lsb` (`0..4095`), and `uint32_t voltage_mv` (`0..5050 mV`) from the same conversion frame. | `SDD-AD7490-1.7` |
| `LLR-AD7490-009` | Control Register Sub-Configuration | `ad7490_ctrl_cfg_t` shall encapsulate `power_mode`, `range_mode`, `coding`, and `dout_mode` to enforce minimal data coupling into frame-encoding helpers. | `SDD-AD7490-1.8` |
| `LLR-AD7490-040` | SEU-Robust Device State Encoding | `ad7490_dev_state_t` (`uint32_t`) shall encode `AD7490_STATE_READY = 0x5A3C96E1U`, `AD7490_STATE_SEQUENCING = 0x3CA5E196U`, and `AD7490_STATE_FAULT = 0xC35A693CU` with pairwise Hamming distance $\ge 20$ and $\ge 16$ from `0x00000000` and `0xFFFFFFFF`. | `SDD-AD7490-1.9` |
| `LLR-AD7490-041` | Device Handle Structure | `ad7490_dev_t` shall store a validated copy of `ad7490_config_t cfg`, `ad7490_dev_state_t state`, and `uint16_t channel_mask`. | `SDD-AD7490-1.10` |

### 2.2 Public AD7490 Driver Operations (`driver/ad7490_driver.h`, `driver/ad7490_driver.c`)

| Requirement ID | Title | Specification | Design Trace |
|:---|:---|:---|:---|
| `LLR-AD7490-010` | Device Power-Up & Initialisation (`ad7490_init`) | `ad7490_init()` shall default `p_dev->state = AD7490_STATE_FAULT`, validate `p_dev` and `p_cfg`, initialise the QSPI bus, send 2 dummy frames with `DIN = 0xFFFF` (AD7490 Fig. 16), write the initial Control Register (`SEQ=0, SHADOW=0, ADD=0`), and transition `p_dev->state` to `AD7490_STATE_READY` on success. | `SDD-AD7490-2.1` |
| `LLR-AD7490-011` | Single-Channel Conversion (`ad7490_read_channel`) | When `p_dev->state == AD7490_STATE_READY` and `power_mode != FULL_SHUTDOWN`, `ad7490_read_channel()` shall validate `channel_id < 16` and `p_sample != NULL`, send Frame 1 selecting `channel_id`, then convert `channel_id` in Frame 2 (`NORMAL`) or, in `AUTO_STANDBY` / `AUTO_SHUTDOWN`, send a power-up dummy Frame 2 (`WRITE = 0`, DOUT discarded) and convert in Frame 3 (`WRITE = 0`) per AD7490 Fig. 14 and 15, verify `DOUT[15:12] == channel_id` (`AD7490_ERR_CHAN_MISMATCH` otherwise), and write `*p_sample` only on `AD7490_OK`. | `SDD-AD7490-2.2` |
| `LLR-AD7490-012` | Shadow Register Sequencer Start (`ad7490_sequencer_start`) | When `p_dev->state == AD7490_STATE_READY` and `power_mode == AD7490_POWER_NORMAL`, `ad7490_sequencer_start()` shall require `channel_mask != 0U`, send 3 frames (CR with `SEQ=0, SHADOW=1`; bit-reversed Shadow Register word; priming frame with `WRITE=0`), and transition to `AD7490_STATE_SEQUENCING` (or `AD7490_STATE_FAULT` on bus error). | `SDD-AD7490-5.1` |
| `LLR-AD7490-013` | Full Sequenced Scan & CR Scrubbing (`ad7490_sequencer_read_scan`) | When `p_dev->state == AD7490_STATE_SEQUENCING`, `ad7490_sequencer_read_scan()` shall require `sample_count >= 16` and `channel_mask != 0U`, send a CR scrub word (`WRITE=1, SEQ=1, SHADOW=0`) on the first frame of the pass and `WRITE=0` (`0x0000U`) on remaining frames, verify every echoed channel belongs to `channel_mask` and appears exactly once per pass, store samples at `p_samples[channel_id]`, and latch `AD7490_STATE_FAULT` on any bus error or channel mismatch. | `SDD-AD7490-5.2` |
| `LLR-AD7490-014` | Power Mode Transition (`ad7490_set_power_mode`) | When `p_dev->state == AD7490_STATE_READY`, `ad7490_set_power_mode()` shall write the updated `PM1:PM0` bits to the Control Register and, for any target mode other than `FULL_SHUTDOWN`, clock one dummy frame (`WRITE=0`, `2.56 us > 1 us` $t_{\text{POWER-UP}}$). Any bus error shall transition `p_dev->state` to `AD7490_STATE_FAULT`. | `SDD-AD7490-2.5` |
| `LLR-AD7490-015` | Sequencer Stop (`ad7490_sequencer_stop`) | When `p_dev->state == AD7490_STATE_SEQUENCING`, `ad7490_sequencer_stop()` shall write the Control Register with `SEQ=0, SHADOW=0`, clear `channel_mask = 0U`, and transition to `AD7490_STATE_READY` (or `AD7490_STATE_FAULT` on bus error). | `SDD-AD7490-5.3` |

### 2.3 Internal Frame, Conversion & Sequencer Helpers (`driver/ad7490_driver.c`)

| Requirement ID | Title | Specification | Design Trace |
|:---|:---|:---|:---|
| `LLR-AD7490-020` | Control Config Enum Range Check (`is_ctrl_cfg_ok`) | `is_ctrl_cfg_ok()` shall verify `power_mode <= 3`, `range_mode <= 1`, `coding <= 1`, and `dout_mode <= 1` with one atomic condition per branch. | `SDD-AD7490-3.1` |
| `LLR-AD7490-021` | 16-Bit CR Frame Builder (`build_cr_frame`) | `build_cr_frame()` shall validate `p_ctrl_cfg`, encode the 12-bit CR word (`LLR-AD7490-030`), and shift it left by 4 bits into `DIN[15:4]`. | `SDD-AD7490-3.2` |
| `LLR-AD7490-022` | 16-Bit Write-Only Frame (`write_frame`) | `write_frame()` shall serialise `tx_frame` MSB-first into 2 bytes and transmit via `qspi_hal_write_bytes()`. | `SDD-AD7490-3.3` |
| `LLR-AD7490-023` | 16-Bit Full-Duplex Frame (`txrx_frame`) | `txrx_frame()` shall serialise `tx_frame` MSB-first, call `qspi_hal_transceive_bytes()`, and deserialise `*p_rx_frame` only on `AD7490_OK`. | `SDD-AD7490-3.4` |
| `LLR-AD7490-024` | 16-Bit NOP Read Frame (`read_frame`) | `read_frame()` shall call `qspi_hal_read_bytes()` with `dummy_tx_byte = 0x00U` (`WRITE = 0`) and deserialise `*p_rx_frame` only on `AD7490_OK`. | `SDD-AD7490-3.5` |
| `LLR-AD7490-025` | 12-Bit Code to Millivolt Conversion (`raw_to_mv`) | `raw_to_mv()` shall validate `vref_mv` (`LLR-AD7490-035`), convert `raw_lsb` to offset binary (`LLR-AD7490-036`), determine `span_mv` (`LLR-AD7490-037`), and compute `*p_voltage_mv = ((offset_code * span_mv) + 2048U) / 4096U` using 32-bit unsigned arithmetic without overflow. | `SDD-AD7490-4.1` |
| `LLR-AD7490-026` | DOUT Word Decoder (`decode_dout`) | `decode_dout()` shall extract `channel_id = (rx_frame >> 12U) & 0x0FU` and `raw_lsb = rx_frame & 0x0FFFU`. | `SDD-AD7490-3.7` |
| `LLR-AD7490-027` | 2-Byte Wire Frame Wrapper (`frame_bytes_t`) | `frame_bytes_t` shall wrap `uint8_t bytes[2]` in a struct for by-value passing without pointer aliasing. | `SDD-AD7490-3.8` |
| `LLR-AD7490-028` | MSB-First Frame Serialiser (`frame_to_bytes`) | `frame_to_bytes()` shall place `frame_word[15:8]` in `bytes[0]` and `frame_word[7:0]` in `bytes[1]`. | `SDD-AD7490-3.9` |
| `LLR-AD7490-029` | MSB-First Frame Deserialiser (`bytes_to_frame`) | `bytes_to_frame()` shall combine `(bytes[0] << 8U) | bytes[1]` into a `uint16_t`. | `SDD-AD7490-3.10` |
| `LLR-AD7490-030` | 12-Bit CR Word Encoder (`encode_cr_word`) | `encode_cr_word()` shall set `WRITE = 1` (`0x0800U`) and OR in `channel_id << 6U`, `power_mode << 4U`, `dout_mode << 2U`, `range_mode << 1U`, `coding`, and `seq_mode_cr_bits[seq_mode]`. | `SDD-AD7490-3.11` |
| `LLR-AD7490-031` | Sequencer Mode Bit Lookup Table (`seq_mode_cr_bits`) | `seq_mode_cr_bits[4]` shall map `ad7490_seq_mode_t` to `0U`, `SHADOW` (`0x008U`), `SEQ` (`0x400U`), and `SEQ | SHADOW` (`0x408U`). | `SDD-AD7490-3.12` |
| `LLR-AD7490-032` | Internal Sequencer Mode Enumeration (`ad7490_seq_mode_t`) | `ad7490_seq_mode_t` shall represent the 4 rows of AD7490 Table IV (`SINGLE`, `LOAD_SHADOW`, `KEEP_SEQ`, `CONSECUTIVE`). | `SDD-AD7490-3.13` |
| `LLR-AD7490-033` | Decoded DOUT Structure (`ad7490_dout_t`) | `ad7490_dout_t` shall hold decoded `channel_id` and `raw_lsb`. | `SDD-AD7490-3.14` |
| `LLR-AD7490-034` | HAL Status to Driver Status Mapper (`map_hal_status`) | `map_hal_status()` shall map `QSPI_HAL_OK -> AD7490_OK`, `ERR_PARAM -> ERR_PARAM`, `ERR_TIMEOUT -> ERR_TIMEOUT`, and `ERR_HW` (plus `default:`) `-> AD7490_ERR_SPI`. | `SDD-AD7490-3.15` |
| `LLR-AD7490-035` | Reference Voltage Range Validator (`is_vref_ok`) | `is_vref_ok()` shall return `true` if and only if `2475U <= vref_mv <= 2525U` (`2.5 V +/- 1%`). | `SDD-AD7490-4.2` |
| `LLR-AD7490-036` | Twos Complement to Offset Binary Mapper (`raw_to_offset_binary`) | `raw_to_offset_binary()` shall XOR `raw_lsb` with `0x0800U` (`DB11`) when `coding == AD7490_CODING_TWOS_COMPLEMENT`, and leave it unchanged for straight binary. | `SDD-AD7490-4.3` |
| `LLR-AD7490-037` | Full-Scale Span Calculator (`span_from_vref_mv`) | `span_from_vref_mv()` shall return `2U * vref_mv` for `AD7490_RANGE_2X_VREF` and `vref_mv` for `AD7490_RANGE_1X_VREF`. | `SDD-AD7490-4.4` |
| `LLR-AD7490-038` | QSPI Bus Initialiser Wrapper (`init_bus`) | `init_bus()` shall invoke `qspi_hal_init()` and translate its return code via `map_hal_status()`. | `SDD-AD7490-3.16` |
| `LLR-AD7490-042` | Device Configuration Validator (`is_dev_cfg_ok`) | `is_dev_cfg_ok()` shall check `p_cfg != NULL`, `is_vref_ok(vref_mv)`, `clock_mode == QSPI_CLOCK_MODE_2`, and `poll_timeout_us >= 3U` (`AD7490_MIN_TIMEOUT_US`). | `SDD-AD7490-2.6` |
| `LLR-AD7490-043` | Power-Up Frame Sequence Runner (`run_power_up`) | `run_power_up()` shall build the initial CR frame before touching hardware, initialize the QSPI bus, send 2 dummy `0xFFFFU` frames, and send the CR frame. | `SDD-AD7490-2.7` |
| `LLR-AD7490-044` | Single-Channel Pipeline Runner (`read_single`) | `read_single()` shall build the single-mode CR frame for `channel_id` and send Frame 1 (`write_frame`); then, when `is_auto_power_mode()` is `false`, send Frame 2 (`txrx_frame`), otherwise call `read_after_wakeup()` (Frames 2 and 3); and decode the result frame (`decode_sample`). | `SDD-AD7490-2.8` |
| `LLR-AD7490-045` | Echo Verification & Sample Decoder (`decode_sample`) | `decode_sample()` shall decode `rx_frame`, verify `dout.channel_id == channel_id` (`AD7490_ERR_CHAN_MISMATCH` otherwise), and convert `raw_lsb` to `voltage_mv`. | `SDD-AD7490-2.9` |
| `LLR-AD7490-046` | Power Mode Frame Writer (`write_power_mode`) | `write_power_mode()` shall write the new CR frame and, if `power_mode != FULL_SHUTDOWN`, clock one dummy read frame (`WRITE=0`) to cover $t_{\text{POWER-UP}}$. | `SDD-AD7490-2.10` |
| `LLR-AD7490-047` | AUTO Power Mode Classifier (`is_auto_power_mode`) | `is_auto_power_mode()` shall return `true` if and only if `power_mode` is `AD7490_POWER_AUTO_STANDBY` or `AD7490_POWER_AUTO_SHUTDOWN`, with one atomic condition per branch. | `SDD-AD7490-2.11` |
| `LLR-AD7490-048` | AUTO Mode Wake-Up Read (`read_after_wakeup`) | `read_after_wakeup()` shall send one power-up dummy frame (`read_frame`, `WRITE = 0`, DOUT discarded) and then one result frame (`read_frame`, `WRITE = 0`), returning the result frame DOUT in `*p_rx_frame` only on `AD7490_OK` and stopping at the first failed frame. | `SDD-AD7490-2.12` |
| `LLR-AD7490-050` | Channel Mask to Shadow Bit Reverser (`mask_to_shadow`) | `mask_to_shadow()` shall map API bit `n` (`VINn`) to Shadow Register bit `15 - n` (`VIN0 = MSB bit 15`, AD7490 Fig. 18). | `SDD-AD7490-5.4` |
| `LLR-AD7490-051` | Active Sequenced Channel Counter (`count_channels`) | `count_channels()` shall return the population count (`0..16`) of `channel_mask`. | `SDD-AD7490-5.5` |
| `LLR-AD7490-052` | 3-Frame Sequencer Start Sender (`send_start_frames`) | `send_start_frames()` shall send the `LOAD_SHADOW` CR frame, the 16-bit `shadow_word` frame, and one `WRITE=0` priming frame. | `SDD-AD7490-5.6` |
| `LLR-AD7490-053` | Sequencer Stop Frame Sender (`send_stop_frame`) | `send_stop_frame()` shall send a `SINGLE`-mode CR frame (`SEQ=0, SHADOW=0`). | `SDD-AD7490-5.7` |
| `LLR-AD7490-054` | Single Sequenced Frame Scanner (`scan_one`) | `scan_one()` shall transceive `tx_frame`, verify the echoed channel is set in `channel_mask` and not yet set in `*p_seen_mask`, convert to millivolts, store into `p_samples[dout.channel_id]`, and set the bit in `*p_seen_mask`. | `SDD-AD7490-5.8` |

---

## 3. AD7490 Periodic Monitor Requirements (`LLR-AD7490-060` .. `LLR-AD7490-081`)

### 3.1 Moving-Average Window (`driver/ad7490_monitor.h`, `driver/ad7490_monitor.c`)

| Requirement ID | Title | Specification | Design Trace |
|:---|:---|:---|:---|
| `LLR-AD7490-060` | Moving-Average Window Reset (`avg_reset`) | `avg_reset()` shall zero all 16 entries of `window_mv[]` and set `next_idx = 0U` and `fill_count = 0U`. | `SDD-AD7490-6.1` |
| `LLR-AD7490-061` | Moving-Average Sample Push (`avg_push`) | `avg_push()` shall verify `is_window_consistent(p_avg)`, store `sample_mv` at `window_mv[next_idx]`, advance `next_idx = (next_idx + 1U) % 16U`, and increment `fill_count` up to `16U`. | `SDD-AD7490-6.2` |
| `LLR-AD7490-062` | Moving-Average Rounded Mean Calculation (`avg_get_mv`) | `avg_get_mv()` shall verify `is_window_consistent(p_avg)` and `fill_count > 0U`, verify each active slot `window_mv[0..fill_count-1] <= 5050U` (`AD7490_VOLTAGE_MAX_MV`), sum the active slots on demand, and output `*p_avg_mv = (sum_mv + (fill_count / 2U)) / fill_count` and `*p_fill_count = fill_count`. | `SDD-AD7490-6.3` |
| `LLR-AD7490-063` | Window Index Consistency Check (`is_window_consistent`) | `is_window_consistent()` shall verify `next_idx < 16U`, `fill_count <= 16U`, and `(fill_count == 16U) || (next_idx == fill_count)`. | `SDD-AD7490-6.4` |
| `LLR-AD7490-064` | Moving-Average Window Structure (`ad7490_avg_t`) | `ad7490_avg_t` shall store `uint32_t window_mv[16]`, `size_t next_idx`, and `size_t fill_count` without a cached running sum. | `SDD-AD7490-6.5` |

### 3.2 Periodic Acquisition & Bounded Fault Recovery (`driver/ad7490_monitor.h`, `driver/ad7490_monitor.c`)

| Requirement ID | Title | Specification | Design Trace |
|:---|:---|:---|:---|
| `LLR-AD7490-070` | Monitor Initialisation (`ad7490_monitor_init`) | `ad7490_monitor_init()` shall validate `p_mon != NULL`, `p_cfg != NULL`, and `channel_mask != 0U`, clear all counters, set `health = AD7490_MON_HEALTH_ACTIVE` (`0x96E15A3CU`), and invoke `restart_adc()`. If `restart_adc()` fails with `AD7490_ERR_PARAM` or `AD7490_ERR_STATE`, the monitor shall disable itself (`channel_mask = 0U`); on a transient bus error it shall remain armed for recovery on the next `ad7490_monitor_step()`. | `SDD-AD7490-7.1` |
| `LLR-AD7490-071` | Periodic 1 ms Acquisition Step (`ad7490_monitor_step`) | `ad7490_monitor_step()` shall return `AD7490_ERR_PARAM` if `p_mon == NULL`, `AD7490_ERR_STATE` if `channel_mask == 0U`, and `AD7490_ERR_DEVICE_FAILED` if `health != AD7490_MON_HEALTH_ACTIVE`. If `dev.state != AD7490_STATE_SEQUENCING` or `dev.channel_mask != channel_mask`, it shall execute one `reconfig_attempt()`; otherwise it shall execute `scan_and_push()`. | `SDD-AD7490-7.2` |
| `LLR-AD7490-072` | Channel Moving-Average Reader (`ad7490_monitor_get_avg`) | `ad7490_monitor_get_avg()` shall validate `p_mon`, `p_avg_mv`, `p_fill_count`, `channel_id < 16`, and `(channel_mask & (1U << channel_id)) != 0U`, return `AD7490_ERR_DEVICE_FAILED` if `health != ACTIVE`, and delegate to `avg_get_mv(&p_mon->avg[channel_id], ...)`. | `SDD-AD7490-7.3` |
| `LLR-AD7490-073` | ADC & Sequencer Restart Helper (`restart_adc`) | `restart_adc()` shall empty all 16 channel windows (`reset_all_avg`), zero `good_scan_count`, call `ad7490_init()`, and on success call `ad7490_sequencer_start()`. | `SDD-AD7490-7.4` |
| `LLR-AD7490-074` | Monitored Channel Scan Pusher (`push_scan`) | `push_scan()` shall push `scan[chan_idx].voltage_mv` into `avg[chan_idx]` for every channel set in `channel_mask`. | `SDD-AD7490-7.5` |
| `LLR-AD7490-075` | All-Channel Window Reset Helper (`reset_all_avg`) | `reset_all_avg()` shall invoke `avg_reset(&p_mon->avg[chan_idx])` for all 16 channels. | `SDD-AD7490-7.6` |
| `LLR-AD7490-076` | Monitor State Structure (`ad7490_monitor_t`) | `ad7490_monitor_t` shall encapsulate `cfg`, `channel_mask`, `dev`, `scan[16]`, `avg[16]`, `health`, `reconfig_attempts`, `reconfig_total`, and `good_scan_count`. | `SDD-AD7490-7.7` |
| `LLR-AD7490-077` | Monitor Diagnostics Structure (`ad7490_mon_diag_t`) | `ad7490_mon_diag_t` shall report `bool is_failed`, `uint32_t reconfig_attempts`, and `uint32_t reconfig_total`. | `SDD-AD7490-7.8` |
| `LLR-AD7490-078` | Application-Triggered Re-Configuration (`ad7490_monitor_reconfig`) | `ad7490_monitor_reconfig()` shall validate `p_mon`, check `channel_mask != 0U` and `health == ACTIVE`, and execute one verified `reconfig_attempt()`. | `SDD-AD7490-7.9` |
| `LLR-AD7490-079` | Monitor Diagnostics Reader (`ad7490_monitor_get_diag`) | `ad7490_monitor_get_diag()` shall populate `p_diag->is_failed = (health != AD7490_MON_HEALTH_ACTIVE)`, `reconfig_attempts`, and `reconfig_total`. | `SDD-AD7490-7.10` |
| `LLR-AD7490-080` | Bounded Re-Configuration Attempt (`reconfig_attempt`) | `reconfig_attempt()` shall allow at most `AD7490_MON_MAX_RECONFIG = 3U` attempts per fault episode. If `reconfig_attempts >= 3U` on entry, or if the 3rd attempt fails `restart_adc()` or its verification `scan_and_push()`, it shall empty all windows, latch `health = AD7490_MON_HEALTH_FAILED` (`0x691EA5C3U`), and return `AD7490_ERR_DEVICE_FAILED`. `reconfig_total` shall increment with saturation at `0xFFFFFFFFU`. | `SDD-AD7490-7.11` |
| `LLR-AD7490-081` | Verified Scan & Stability Counter (`scan_and_push`) | `scan_and_push()` shall run `ad7490_sequencer_read_scan()` and `push_scan()`. On any error it shall empty all windows and reset `good_scan_count = 0U`. On success it shall increment `good_scan_count` up to `AD7490_MON_STABLE_SCANS = 16U`, clearing `reconfig_attempts = 0U` when `good_scan_count` reaches `16U`. | `SDD-AD7490-7.12` |

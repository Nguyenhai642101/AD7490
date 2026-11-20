# Bi-Directional Traceability Matrix (DO-178C §5.5, Table A-5)

This document provides complete bi-directional traceability between:
1. **Low-Level Requirements (`@req`)**
2. **Software Design Description items (`@design`)**
3. **Source Code Implementation (`hal/` and `driver/`)**
4. **Host Unit-Test Verification Cases (`test/`)**

Every function (public and `static`), file-scope lookup table, and public type in the production codebase traces to a unique `@req` and `@design` pair, with zero orphan code.

---

## 1. AXI Quad SPI HAL (`hal/axi_qspi_hal.h`, `hal/axi_qspi_hal.c`)

| Requirement ID (`@req`) | Design ID (`@design`) | Source Symbol / Artefact | File | Verification Test Case (`test/test_qspi_hal.c`) |
|:---|:---|:---|:---|:---|
| `LLR-QSPI-001` | `SDD-QSPI-1.0` | Module header & translation unit | `axi_qspi_hal.h`, `axi_qspi_hal.c` | `main()` (`test_qspi_hal`) |
| `LLR-QSPI-002` | `SDD-QSPI-1.1` | `qspi_hal_status_t` | `axi_qspi_hal.h` | `test_init_params()`, `test_transceive_bytes()` |
| `LLR-QSPI-003` | `SDD-QSPI-1.2` | `qspi_clock_mode_t` | `axi_qspi_hal.h` | `test_init_sequence_and_faults()` |
| `LLR-QSPI-004` | `SDD-QSPI-1.3` | `qspi_hal_config_t` | `axi_qspi_hal.h` | `test_init_params()` |
| `LLR-QSPI-005` | `SDD-QSPI-1.4` | `ASSERT(expr)` contract macro | `axi_qspi_hal.h` | All tests (mapped to `test_assert`) |
| `LLR-QSPI-010` | `SDD-QSPI-2.1` | `qspi_hal_init()` | `axi_qspi_hal.h`, `axi_qspi_hal.c` | `test_init_params()`, `test_init_sequence_and_faults()` |
| `LLR-QSPI-011` | `SDD-QSPI-2.2` | `qspi_hal_transceive_bytes()` | `axi_qspi_hal.h`, `axi_qspi_hal.c` | `test_transceive_bytes()` |
| `LLR-QSPI-012` | `SDD-QSPI-2.3` | `qspi_hal_write_bytes()` | `axi_qspi_hal.h`, `axi_qspi_hal.c` | `test_write_bytes()` |
| `LLR-QSPI-013` | `SDD-QSPI-2.4` | `qspi_hal_read_bytes()` | `axi_qspi_hal.h`, `axi_qspi_hal.c` | `test_read_bytes()` |
| `LLR-QSPI-020` | `SDD-QSPI-3.1` | `reg_write32()` | `axi_qspi_hal.c` | `test_init_sequence_and_faults()`, `test_write_bytes()` |
| `LLR-QSPI-021` | `SDD-QSPI-3.2` | `reg_read32()` | `axi_qspi_hal.c` | `test_init_sequence_and_faults()`, `test_read_bytes()` |
| `LLR-QSPI-022` | `SDD-QSPI-3.3` | `is_hal_cfg_ok()` | `axi_qspi_hal.c` | `test_init_params()` |
| `LLR-QSPI-023` | `SDD-QSPI-3.4` | `is_byte_count_ok()` | `axi_qspi_hal.c` | `test_transceive_bytes()`, `test_write_bytes()`, `test_read_bytes()` |
| `LLR-QSPI-024` | `SDD-QSPI-3.5` | `clock_mode_ctrl_bits[]`, `build_control_word()` | `axi_qspi_hal.c` | `test_init_sequence_and_faults()` |
| `LLR-QSPI-025` | `SDD-QSPI-3.6` | `reset_fifos()` | `axi_qspi_hal.c` | `test_transceive_bytes()`, `test_write_bytes()`, `test_read_bytes()` |
| `LLR-QSPI-026` | `SDD-QSPI-3.7` | `run_bus_cycle()` | `axi_qspi_hal.c` | `test_transceive_bytes()`, `test_write_bytes()`, `test_read_bytes()` |
| `LLR-QSPI-027` | `SDD-QSPI-3.8` | `drain_rx_bytes()` | `axi_qspi_hal.c` | `test_transceive_bytes()`, `test_read_bytes()`, `test_drain_rx_fault()` |
| `LLR-QSPI-028` | `SDD-QSPI-3.9` | `wait_reset_done()` | `axi_qspi_hal.c` | `test_init_sequence_and_faults()` |
| `LLR-QSPI-029` | `SDD-QSPI-3.10` | `load_tx_fifo_array()` | `axi_qspi_hal.c` | `test_transceive_bytes()`, `test_write_bytes()` |
| `LLR-QSPI-030` | `SDD-QSPI-3.11` | `load_tx_fifo_fill()` | `axi_qspi_hal.c` | `test_read_bytes()` |
| `LLR-QSPI-031` | `SDD-QSPI-3.12` | `read_rx_count()` | `axi_qspi_hal.c` | `test_transceive_bytes()`, `test_write_bytes()` |

---

## 2. AD7490 Driver (`driver/ad7490_driver.h`, `driver/ad7490_driver.c`)

| Requirement ID (`@req`) | Design ID (`@design`) | Source Symbol / Artefact | File | Verification Test Case (`test/test_*.c`) |
|:---|:---|:---|:---|:---|
| `LLR-AD7490-001` | `SDD-AD7490-1.0` | Module header & translation unit | `ad7490_driver.h`, `ad7490_driver.c` | `test_device`, `test_sequencer`, `test_frame_convert` |
| `LLR-AD7490-002` | `SDD-AD7490-1.1` | `ad7490_status_t` | `ad7490_driver.h` | All driver tests |
| `LLR-AD7490-003` | `SDD-AD7490-1.2` | `ad7490_power_mode_t` | `ad7490_driver.h` | `test_device.c:test_power_modes()` |
| `LLR-AD7490-004` | `SDD-AD7490-1.3` | `ad7490_range_t` | `ad7490_driver.h` | `test_frame_convert.c:test_conversion_table()` |
| `LLR-AD7490-005` | `SDD-AD7490-1.4` | `ad7490_coding_t` | `ad7490_driver.h` | `test_frame_convert.c:test_conversion_exhaustive()` |
| `LLR-AD7490-006` | `SDD-AD7490-1.5` | `ad7490_dout_mode_t` | `ad7490_driver.h` | `test_frame_convert.c:test_cr_encoding()` |
| `LLR-AD7490-007` | `SDD-AD7490-1.6` | `ad7490_config_t` | `ad7490_driver.h` | `test_device.c:test_init_rejects_bad_config()` |
| `LLR-AD7490-008` | `SDD-AD7490-1.7` | `ad7490_sample_t` | `ad7490_driver.h` | `test_device.c:test_read_channel()` |
| `LLR-AD7490-009` | `SDD-AD7490-1.8` | `ad7490_ctrl_cfg_t` | `ad7490_driver.h` | `test_frame_convert.c:test_cr_encoding()` |
| `LLR-AD7490-010` | `SDD-AD7490-2.1` | `ad7490_init()` | `ad7490_driver.h`, `ad7490_driver.c` | `test_device.c:test_init_sequence()`, `test_init_bus_errors()` |
| `LLR-AD7490-011` | `SDD-AD7490-2.2` | `ad7490_read_channel()` | `ad7490_driver.h`, `ad7490_driver.c` | `test_device.c:test_read_channel()`, `test_read_errors()`, `test_auto_mode_reads()` |
| `LLR-AD7490-012` | `SDD-AD7490-5.1` | `ad7490_sequencer_start()` | `ad7490_driver.h`, `ad7490_driver.c` | `test_sequencer.c:test_start_and_scan()`, `test_refused_calls()` |
| `LLR-AD7490-013` | `SDD-AD7490-5.2` | `ad7490_sequencer_read_scan()` | `ad7490_driver.h`, `ad7490_driver.c` | `test_sequencer.c:test_start_and_scan()`, `test_scan_mismatch()` |
| `LLR-AD7490-014` | `SDD-AD7490-2.5` | `ad7490_set_power_mode()` | `ad7490_driver.h`, `ad7490_driver.c` | `test_device.c:test_power_modes()`, `test_power_mode_errors()` |
| `LLR-AD7490-015` | `SDD-AD7490-5.3` | `ad7490_sequencer_stop()` | `ad7490_driver.h`, `ad7490_driver.c` | `test_sequencer.c:test_start_and_scan()`, `test_bus_errors()` |
| `LLR-AD7490-020` | `SDD-AD7490-3.1` | `is_ctrl_cfg_ok()` | `ad7490_driver.c` | `test_frame_convert.c:test_cr_rejects_bad_enum()` |
| `LLR-AD7490-021` | `SDD-AD7490-3.2` | `build_cr_frame()` | `ad7490_driver.c` | `test_frame_convert.c:test_cr_encoding()`, `test_cr_rejects_bad_enum()` |
| `LLR-AD7490-022` | `SDD-AD7490-3.3` | `write_frame()` | `ad7490_driver.c` | `test_frame_convert.c:test_frames()` |
| `LLR-AD7490-023` | `SDD-AD7490-3.4` | `txrx_frame()` | `ad7490_driver.c` | `test_frame_convert.c:test_frames()` |
| `LLR-AD7490-024` | `SDD-AD7490-3.5` | `read_frame()` | `ad7490_driver.c` | `test_frame_convert.c:test_frames()` |
| `LLR-AD7490-025` | `SDD-AD7490-4.1` | `raw_to_mv()` | `ad7490_driver.c` | `test_frame_convert.c:test_conversion_exhaustive()` |
| `LLR-AD7490-026` | `SDD-AD7490-3.7` | `decode_dout()` | `ad7490_driver.c` | `test_frame_convert.c:test_frames()` |
| `LLR-AD7490-027` | `SDD-AD7490-3.8` | `frame_bytes_t` | `ad7490_driver.c` | `test_frame_convert.c:test_frames()` |
| `LLR-AD7490-028` | `SDD-AD7490-3.9` | `frame_to_bytes()` | `ad7490_driver.c` | `test_frame_convert.c:test_frames()` |
| `LLR-AD7490-029` | `SDD-AD7490-3.10` | `bytes_to_frame()` | `ad7490_driver.c` | `test_frame_convert.c:test_frames()` |
| `LLR-AD7490-030` | `SDD-AD7490-3.11` | `encode_cr_word()` | `ad7490_driver.c` | `test_frame_convert.c:test_cr_encoding()`, `test_cr_rejects_bad_enum()` |
| `LLR-AD7490-031` | `SDD-AD7490-3.12` | `seq_mode_cr_bits[]` | `ad7490_driver.c` | `test_frame_convert.c:test_cr_encoding()` |
| `LLR-AD7490-032` | `SDD-AD7490-3.13` | `ad7490_seq_mode_t` | `ad7490_driver.c` | `test_frame_convert.c:test_cr_encoding()` |
| `LLR-AD7490-033` | `SDD-AD7490-3.14` | `ad7490_dout_t` | `ad7490_driver.c` | `test_frame_convert.c:test_frames()` |
| `LLR-AD7490-034` | `SDD-AD7490-3.15` | `map_hal_status()` | `ad7490_driver.c` | `test_frame_convert.c:test_hal_status_map()` |
| `LLR-AD7490-035` | `SDD-AD7490-4.2` | `is_vref_ok()` | `ad7490_driver.c` | `test_frame_convert.c:test_vref_limits()` |
| `LLR-AD7490-036` | `SDD-AD7490-4.3` | `raw_to_offset_binary()` | `ad7490_driver.c` | `test_frame_convert.c:test_conversion_table()` |
| `LLR-AD7490-037` | `SDD-AD7490-4.4` | `span_from_vref_mv()` | `ad7490_driver.c` | `test_frame_convert.c:test_conversion_table()` |
| `LLR-AD7490-038` | `SDD-AD7490-3.16` | `init_bus()` | `ad7490_driver.c` | `test_frame_convert.c:test_hal_status_map()` |
| `LLR-AD7490-040` | `SDD-AD7490-1.9` | `ad7490_dev_state_t` | `ad7490_driver.h` | `test_device.c:test_corrupted_handle()` |
| `LLR-AD7490-041` | `SDD-AD7490-1.10` | `ad7490_dev_t` | `ad7490_driver.h` | `test_device.c:test_uninitialised_handle()` |
| `LLR-AD7490-042` | `SDD-AD7490-2.6` | `is_dev_cfg_ok()` | `ad7490_driver.c` | `test_device.c:test_init_rejects_bad_config()` |
| `LLR-AD7490-043` | `SDD-AD7490-2.7` | `run_power_up()` | `ad7490_driver.c` | `test_device.c:test_init_sequence()`, `test_init_bus_errors()` |
| `LLR-AD7490-044` | `SDD-AD7490-2.8` | `read_single()` | `ad7490_driver.c` | `test_device.c:test_read_channel()`, `test_read_errors()`, `test_auto_mode_reads()` |
| `LLR-AD7490-045` | `SDD-AD7490-2.9` | `decode_sample()` | `ad7490_driver.c` | `test_device.c:test_read_channel()`, `test_read_errors()` |
| `LLR-AD7490-046` | `SDD-AD7490-2.10` | `write_power_mode()` | `ad7490_driver.c` | `test_device.c:test_power_modes()`, `test_power_mode_errors()` |
| `LLR-AD7490-047` | `SDD-AD7490-2.11` | `is_auto_power_mode()` | `ad7490_driver.c` | `test_device.c:test_read_channel()`, `test_auto_mode_reads()` |
| `LLR-AD7490-048` | `SDD-AD7490-2.12` | `read_after_wakeup()` | `ad7490_driver.c` | `test_device.c:test_auto_mode_reads()` |
| `LLR-AD7490-050` | `SDD-AD7490-5.4` | `mask_to_shadow()` | `ad7490_driver.c` | `test_sequencer.c:test_start_and_scan()` |
| `LLR-AD7490-051` | `SDD-AD7490-5.5` | `count_channels()` | `ad7490_driver.c` | `test_sequencer.c:test_start_and_scan()` |
| `LLR-AD7490-052` | `SDD-AD7490-5.6` | `send_start_frames()` | `ad7490_driver.c` | `test_sequencer.c:test_start_and_scan()`, `test_bus_errors()` |
| `LLR-AD7490-053` | `SDD-AD7490-5.7` | `send_stop_frame()` | `ad7490_driver.c` | `test_sequencer.c:test_start_and_scan()`, `test_bus_errors()` |
| `LLR-AD7490-054` | `SDD-AD7490-5.8` | `scan_one()` | `ad7490_driver.c` | `test_sequencer.c:test_start_and_scan()`, `test_scan_mismatch()` |

---

## 3. AD7490 Periodic Monitor (`driver/ad7490_monitor.h`, `driver/ad7490_monitor.c`)

| Requirement ID (`@req`) | Design ID (`@design`) | Source Symbol / Artefact | File | Verification Test Case (`test/test_monitor.c`) |
|:---|:---|:---|:---|:---|
| `LLR-AD7490-001` | `SDD-AD7490-7.0` | Module header & translation unit | `ad7490_monitor.h`, `ad7490_monitor.c` | `main()` (`test_monitor`) |
| `LLR-AD7490-060` | `SDD-AD7490-6.1` | `avg_reset()` | `ad7490_monitor.c` | `test_window()` |
| `LLR-AD7490-061` | `SDD-AD7490-6.2` | `avg_push()` | `ad7490_monitor.c` | `test_window()`, `test_window_corruption()` |
| `LLR-AD7490-062` | `SDD-AD7490-6.3` | `avg_get_mv()` | `ad7490_monitor.c` | `test_window()`, `test_window_corruption()` |
| `LLR-AD7490-063` | `SDD-AD7490-6.4` | `is_window_consistent()` | `ad7490_monitor.c` | `test_window_corruption()` |
| `LLR-AD7490-064` | `SDD-AD7490-6.5` | `ad7490_avg_t` | `ad7490_monitor.h` | `test_window()` |
| `LLR-AD7490-070` | `SDD-AD7490-7.1` | `ad7490_monitor_init()` | `ad7490_monitor.h`, `ad7490_monitor.c` | `test_parameters()`, `test_bad_config_disables()`, `test_init_bus_error_retried()` |
| `LLR-AD7490-071` | `SDD-AD7490-7.2` | `ad7490_monitor_step()` | `ad7490_monitor.h`, `ad7490_monitor.c` | `test_ramp_average()`, `test_scan_fault_and_recovery()`, `test_dead_adc_latches_failed()` |
| `LLR-AD7490-072` | `SDD-AD7490-7.3` | `ad7490_monitor_get_avg()` | `ad7490_monitor.h`, `ad7490_monitor.c` | `test_ramp_average()`, `test_parameters()`, `test_scan_fault_and_recovery()` |
| `LLR-AD7490-073` | `SDD-AD7490-7.4` | `restart_adc()` | `ad7490_monitor.c` | `test_init_bus_error_retried()`, `test_scan_fault_and_recovery()` |
| `LLR-AD7490-074` | `SDD-AD7490-7.5` | `push_scan()` | `ad7490_monitor.c` | `test_ramp_average()`, `test_corrupted_state()` |
| `LLR-AD7490-075` | `SDD-AD7490-7.6` | `reset_all_avg()` | `ad7490_monitor.c` | `test_scan_fault_and_recovery()`, `test_dead_adc_latches_failed()` |
| `LLR-AD7490-076` | `SDD-AD7490-7.7` | `ad7490_monitor_t` | `ad7490_monitor.h` | `test_ramp_average()`, `test_corrupted_state()` |
| `LLR-AD7490-077` | `SDD-AD7490-7.8` | `ad7490_mon_diag_t` | `ad7490_monitor.h` | `test_dead_adc_latches_failed()`, `test_rare_fault()` |
| `LLR-AD7490-078` | `SDD-AD7490-7.9` | `ad7490_monitor_reconfig()` | `ad7490_monitor.h`, `ad7490_monitor.c` | `test_app_requested_reconfig()`, `test_dead_adc_latches_failed()` |
| `LLR-AD7490-079` | `SDD-AD7490-7.10` | `ad7490_monitor_get_diag()` | `ad7490_monitor.h`, `ad7490_monitor.c` | `test_parameters()`, `test_dead_adc_latches_failed()`, `test_rare_fault()` |
| `LLR-AD7490-080` | `SDD-AD7490-7.11` | `reconfig_attempt()` | `ad7490_monitor.c` | `test_dead_adc_latches_failed()`, `test_intermittent_fault()`, `test_timeouts_inside_attempts()` |
| `LLR-AD7490-081` | `SDD-AD7490-7.12` | `scan_and_push()` | `ad7490_monitor.c` | `test_scan_fault_and_recovery()`, `test_coding_upset_scrubbed()` |

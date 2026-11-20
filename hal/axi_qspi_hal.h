/**
 * @file        axi_qspi_hal.h
 * @brief       HAL for Xilinx AXI Quad SPI v3.2 (PG153), polled, 8-bit.
 * @req         LLR-QSPI-001
 * @design      SDD-QSPI-1.0
 *
 * IP customisation this HAL is written for (must match Vivado):
 * - AXI ACLK          : 100 MHz
 * - Mode              : Standard SPI
 * - Transaction width : 8 bit
 * - Frequency ratio   : 16x1 -> SCK = 100 MHz / 16 = 6.25 MHz
 * - Number of slaves  : 1 (SS0)
 * - FIFO depth        : 256 bytes
 *
 * Target CPU: Arm Cortex-A53, bare-metal.
 * The register window of the IP MUST be mapped as Device-nGnRE (or
 * stronger) in the MMU translation table, never as Normal cacheable
 * memory. Device memory guarantees that accesses to this one peripheral
 * arrive in program order, are never merged and never speculated, so no
 * DMB is required between two register accesses of this HAL.
 */

#ifndef AXI_QSPI_HAL_H
#define AXI_QSPI_HAL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/**
 * @brief       Status codes for AXI Quad SPI HAL operations.
 * @req         LLR-QSPI-002
 * @design      SDD-QSPI-1.1
 */
typedef enum
{
    QSPI_HAL_OK          = 0U,
    QSPI_HAL_ERR_PARAM   = 1U,
    QSPI_HAL_ERR_TIMEOUT = 2U,
    QSPI_HAL_ERR_HW      = 3U
} qspi_hal_status_t;

/**
 * @brief       SPI clock polarity / phase modes (PG153 SPICR CPOL/CPHA).
 * @req         LLR-QSPI-003
 * @design      SDD-QSPI-1.2
 */
typedef enum
{
    QSPI_CLOCK_MODE_0 = 0U, /* CPOL = 0, CPHA = 0 */
    QSPI_CLOCK_MODE_1 = 1U, /* CPOL = 0, CPHA = 1 */
    QSPI_CLOCK_MODE_2 = 2U, /* CPOL = 1, CPHA = 0 */
    QSPI_CLOCK_MODE_3 = 3U  /* CPOL = 1, CPHA = 1 */
} qspi_clock_mode_t;

/**
 * @brief       Configuration of one AXI Quad SPI instance.
 * @req         LLR-QSPI-004
 * @design      SDD-QSPI-1.3
 * @note        poll_timeout_us is a guaranteed MINIMUM busy-wait per
 *              transfer. One byte shifts in 8 bit * 16 ACLK = 1.28 us, so
 *              a full 256-byte frame needs poll_timeout_us >= 328.
 */
typedef struct
{
    uintptr_t         base_addr;       /* MMIO base, != 0, 4-byte aligned */
    qspi_clock_mode_t clock_mode;      /* SPI CPOL/CPHA mode, 0..3        */
    uint32_t          poll_timeout_us; /* Microseconds, 1..1000000        */
} qspi_hal_config_t;

/* FIFO depth chosen in the IP customisation = max bytes per CS frame */
#define QSPI_HAL_FIFO_DEPTH_BYTES   (256U)

/* Upper bound of poll_timeout_us: 1 s (project choice, LLR-QSPI-004) */
#define QSPI_HAL_MAX_TIMEOUT_US     (1000000U)

/* Number of members in qspi_clock_mode_t */
#define QSPI_CLOCK_MODE_COUNT       (4U)

/**
 * @brief       Project contract assertion for internal static helpers.
 * @req         LLR-QSPI-005
 * @design      SDD-QSPI-1.4
 * @note        Guards programmer contracts only, never runtime input or
 *              hardware state [B 7.3]. Flight/release builds compile it
 *              out to ((void)0), so it adds no branch to MC/DC coverage
 *              (rule §10). A host unit-test build may supply its own
 *              definition on the compiler command line (-D) to trap
 *              contract violations.
 */
#if !defined(ASSERT)
#define ASSERT(expr)                ((void)0)
#endif /* !defined(ASSERT) */

/**
 * @brief       Reset the AXI Quad SPI IP and set 8-bit polled master mode.
 * @req         LLR-QSPI-010
 * @design      SDD-QSPI-2.1
 * @param[in]   p_cfg  Pointer to QSPI HAL configuration, must not be NULL.
 * @return      QSPI_HAL_OK, QSPI_HAL_ERR_PARAM, QSPI_HAL_ERR_HW.
 * @note        Not ISR-safe. Not reentrant on the same base_addr.
 *              Blocking. WCET bound: 70 AXI register accesses.
 *              QSPI_HAL_ERR_HW: core did not return to its PG153 reset
 *              state, or a mode fault (MODF) was flagged.
 */
qspi_hal_status_t qspi_hal_init(qspi_hal_config_t const * p_cfg);

/**
 * @brief       Full-duplex transfer of byte_count bytes under one SS0 frame.
 * @req         LLR-QSPI-011
 * @design      SDD-QSPI-2.2
 * @param[in]   p_cfg       Pointer to QSPI HAL config, must not be NULL.
 * @param[in]   p_tx_bytes  Bytes to shift out MSB-first, must not be NULL.
 * @param[out]  p_rx_bytes  Bytes shifted in, must not be NULL.
 * @param[in]   byte_count  Bytes in both arrays, 1..256.
 * @return      QSPI_HAL_OK, QSPI_HAL_ERR_PARAM, QSPI_HAL_ERR_TIMEOUT,
 *              QSPI_HAL_ERR_HW.
 * @note        Not ISR-safe. Not reentrant on the same base_addr: the
 *              caller serialises access (e.g. RTOS mutex). Blocking
 *              busy-wait. SS0 is released only after the last byte has
 *              completely shifted in. p_rx_bytes holds byte_count valid
 *              bytes only when the return value is QSPI_HAL_OK: on
 *              QSPI_HAL_ERR_HW the drain may already have stored the
 *              bytes popped before the fault (no roll-back), so the
 *              caller discards the whole array unless QSPI_HAL_OK.
 */
qspi_hal_status_t qspi_hal_transceive_bytes(qspi_hal_config_t const * p_cfg,
                                            uint8_t const * p_tx_bytes,
                                            uint8_t * p_rx_bytes,
                                            size_t byte_count);

/**
 * @brief       Transmit byte_count bytes under one SS0 frame, ignore MISO.
 * @req         LLR-QSPI-012
 * @design      SDD-QSPI-2.3
 * @param[in]   p_cfg       Pointer to QSPI HAL config, must not be NULL.
 * @param[in]   p_tx_bytes  Bytes to shift out MSB-first, must not be NULL.
 * @param[in]   byte_count  Number of bytes to write, 1..256.
 * @return      QSPI_HAL_OK, QSPI_HAL_ERR_PARAM, QSPI_HAL_ERR_TIMEOUT,
 *              QSPI_HAL_ERR_HW.
 * @note        Not ISR-safe. Not reentrant on the same base_addr.
 *              Blocking busy-wait. Received bytes stay in the RX FIFO
 *              and are flushed by the FIFO reset of the next transfer.
 */
qspi_hal_status_t qspi_hal_write_bytes(qspi_hal_config_t const * p_cfg,
                                       uint8_t const * p_tx_bytes,
                                       size_t byte_count);

/**
 * @brief       Read byte_count bytes while driving dummy_tx_byte on MOSI.
 * @req         LLR-QSPI-013
 * @design      SDD-QSPI-2.4
 * @param[in]   p_cfg          Pointer to QSPI HAL config, must not be NULL.
 * @param[in]   dummy_tx_byte  Byte repeated on MOSI for every byte read.
 * @param[out]  p_rx_bytes     Destination byte array, must not be NULL.
 * @param[in]   byte_count     Number of bytes to read, 1..256.
 * @return      QSPI_HAL_OK, QSPI_HAL_ERR_PARAM, QSPI_HAL_ERR_TIMEOUT,
 *              QSPI_HAL_ERR_HW.
 * @note        Not ISR-safe. Not reentrant on the same base_addr.
 *              Blocking busy-wait. p_rx_bytes holds byte_count valid
 *              bytes only when the return value is QSPI_HAL_OK: on
 *              QSPI_HAL_ERR_HW the drain may already have stored the
 *              bytes popped before the fault (no roll-back), so the
 *              caller discards the whole array unless QSPI_HAL_OK.
 */
qspi_hal_status_t qspi_hal_read_bytes(qspi_hal_config_t const * p_cfg,
                                      uint8_t dummy_tx_byte,
                                      uint8_t * p_rx_bytes,
                                      size_t byte_count);

#endif /* AXI_QSPI_HAL_H */

/**
 * @file        axi_qspi_hal.c
 * @brief       HAL implementation for Xilinx AXI Quad SPI (8-bit, polled).
 * @req         LLR-QSPI-001
 * @design      SDD-QSPI-1.0
 */

#include "axi_qspi_hal.h"

/* Register offsets, AXI Quad SPI v3.2 register space (PG153) */
#define QSPI_REG_GIER_OFFSET        (0x1CU) /* Global Interrupt Enable    */
#define QSPI_REG_SRR_OFFSET         (0x40U) /* Software Reset Register    */
#define QSPI_REG_SPICR_OFFSET       (0x60U) /* SPI Control Register       */
#define QSPI_REG_SPISR_OFFSET       (0x64U) /* SPI Status Register        */
#define QSPI_REG_SPIDTR_OFFSET      (0x68U) /* SPI Data Transmit Register */
#define QSPI_REG_SPIDRR_OFFSET      (0x6CU) /* SPI Data Receive Register  */
#define QSPI_REG_SPISSR_OFFSET      (0x70U) /* SPI Slave Select Register  */
#define QSPI_REG_RXOCY_OFFSET       (0x78U) /* RX FIFO Occupancy Register */

/* SRR key: resets the whole core for 16 ACLK cycles (PG153 SRR) */
#define QSPI_SRR_RESET_KEY          (0x0000000AU)

/* SPICR bits (PG153 SPICR). LSB_FIRST (bit 9) stays 0 = MSB first. */
#define QSPI_SPICR_SPE_MASK         (0x00000002U) /* Bit 1: SPI enable    */
#define QSPI_SPICR_MASTER_MASK      (0x00000004U) /* Bit 2: Master mode   */
#define QSPI_SPICR_CPOL_MASK        (0x00000008U) /* Bit 3: Clk polarity  */
#define QSPI_SPICR_CPHA_MASK        (0x00000010U) /* Bit 4: Clk phase     */
#define QSPI_SPICR_TX_RESET_MASK    (0x00000020U) /* Bit 5: TX FIFO reset */
#define QSPI_SPICR_RX_RESET_MASK    (0x00000040U) /* Bit 6: RX FIFO reset */
#define QSPI_SPICR_MANUAL_SS_MASK   (0x00000080U) /* Bit 7: Manual SS     */
#define QSPI_SPICR_INHIBIT_MASK     (0x00000100U) /* Bit 8: Master inhib. */

/* SPICR word after reset: Manual SS + Master inhibit set (PG153) */
#define QSPI_SPICR_RESET_WORD       (0x00000180U)

/* SPISR bits (PG153 SPISR) */
#define QSPI_SPISR_RX_EMPTY_MASK    (0x00000001U) /* Bit 0: RX FIFO empty */
#define QSPI_SPISR_MODF_MASK        (0x00000010U) /* Bit 4: Mode fault    */

/* SPISSR, one slave, active low; only bit 0 is implemented (NUM_SS=1) */
#define QSPI_SPISSR_DEASSERT_ALL    (0xFFFFFFFFU)
#define QSPI_SPISSR_ASSERT_SS0      (0xFFFFFFFEU)
#define QSPI_SPISSR_SS0_MASK        (0x00000001U)

/* RX occupancy register holds (bytes - 1); 8 bits for a 256-deep FIFO */
#define QSPI_RXOCY_MASK             (0x000000FFU)

/* 32-bit registers on Device memory: unaligned access = alignment fault */
#define QSPI_REG_ALIGN_MASK         (0x3U)

#define QSPI_BYTE_MASK              (0xFFU)

/*
 * Poll timing. AXI ACLK = 100 MHz. Every poll iteration performs at least
 * one AXI4-Lite read, which occupies the slave for >= 1 ACLK (10 ns).
 * Hence at most 100 iterations elapse per microsecond, and counting 100
 * iterations per microsecond makes poll_timeout_us a guaranteed MINIMUM
 * wait (the real wait may be longer, never shorter).
 * Worst-case tick count: 1000000 us * 100 = 1e8 < 2^32, no wrap.
 */
#define QSPI_AXI_ACLK_MHZ           (100U)
#define QSPI_POLL_ITERS_PER_US      (QSPI_AXI_ACLK_MHZ)

/* Reads issued after SRR: 2 x the 16-ACLK reset pulse, >= 1 ACLK each */
#define QSPI_RESET_SETTLE_READS     (32U)

static inline void reg_write32(uintptr_t base_addr,
                               uint32_t offset_bytes,
                               uint32_t reg_word);
static inline uint32_t reg_read32(uintptr_t base_addr,
                                  uint32_t offset_bytes);
static bool is_hal_cfg_ok(qspi_hal_config_t const * p_cfg);
static bool is_byte_count_ok(size_t byte_count);
static uint32_t build_control_word(qspi_clock_mode_t clock_mode);
static bool wait_reset_done(uintptr_t base_addr);
static void reset_fifos(uintptr_t base_addr,
                        uint32_t ctrl_inhibit);
static void load_tx_fifo_array(uintptr_t base_addr,
                               uint8_t const * p_tx_bytes,
                               size_t byte_count);
static void load_tx_fifo_fill(uintptr_t base_addr,
                              uint8_t fill_byte,
                              size_t byte_count);
static size_t read_rx_count(uintptr_t base_addr);
static qspi_hal_status_t run_bus_cycle(uintptr_t base_addr,
                                       uint32_t ctrl_inhibit,
                                       size_t byte_count,
                                       uint32_t timeout_us);
static qspi_hal_status_t drain_rx_bytes(uintptr_t base_addr,
                                        uint8_t * p_rx_bytes,
                                        size_t byte_count);

/**
 * @brief       SPICR CPOL/CPHA bits, indexed by qspi_clock_mode_t.
 * @req         LLR-QSPI-024
 * @design      SDD-QSPI-3.5
 */
static const uint32_t clock_mode_ctrl_bits[QSPI_CLOCK_MODE_COUNT] =
{
    [QSPI_CLOCK_MODE_0] = 0U,
    [QSPI_CLOCK_MODE_1] = QSPI_SPICR_CPHA_MASK,
    [QSPI_CLOCK_MODE_2] = QSPI_SPICR_CPOL_MASK,
    [QSPI_CLOCK_MODE_3] = (QSPI_SPICR_CPOL_MASK | QSPI_SPICR_CPHA_MASK)
};


/**
 * @brief       Reset the AXI Quad SPI IP and set 8-bit polled master mode.
 * @req         LLR-QSPI-010
 * @design      SDD-QSPI-2.1
 */
qspi_hal_status_t qspi_hal_init(qspi_hal_config_t const * p_cfg)
{
    qspi_hal_status_t result = QSPI_HAL_OK;
    bool is_cfg_ok = is_hal_cfg_ok(p_cfg);

    if (!is_cfg_ok)
    {
        result = QSPI_HAL_ERR_PARAM;
    }
    else
    {
        uintptr_t base_addr = p_cfg->base_addr;
        uint32_t ctrl_word = build_control_word(p_cfg->clock_mode);
        uint32_t init_ctrl = ctrl_word
                             | QSPI_SPICR_TX_RESET_MASK
                             | QSPI_SPICR_RX_RESET_MASK;
        uint32_t status_reg = 0U;
        bool is_reset_done = false;

        /* Software reset returns every register to its PG153 default */
        reg_write32(base_addr, QSPI_REG_SRR_OFFSET, QSPI_SRR_RESET_KEY);

        /* Writes landing inside the 16-ACLK reset pulse would be lost */
        is_reset_done = wait_reset_done(base_addr);

        if (!is_reset_done)
        {
            /* Core did not come out of reset: wrong base or IP fault */
            result = QSPI_HAL_ERR_HW;
        }
        else
        {
            /* Polled design: interrupts stay disabled at the IP */
            reg_write32(base_addr, QSPI_REG_GIER_OFFSET, 0U);

            /* SS0 high before the master output driver is enabled */
            reg_write32(base_addr,
                        QSPI_REG_SPISSR_OFFSET,
                        QSPI_SPISSR_DEASSERT_ALL);

            /* Master, manual SS, inhibited, FIFOs flushed */
            reg_write32(base_addr, QSPI_REG_SPICR_OFFSET, init_ctrl);
            reg_write32(base_addr, QSPI_REG_SPICR_OFFSET, ctrl_word);

            status_reg = reg_read32(base_addr, QSPI_REG_SPISR_OFFSET);

            if (0U != (status_reg & QSPI_SPISR_MODF_MASK))
            {
                result = QSPI_HAL_ERR_HW;
            }
            else
            {
                result = QSPI_HAL_OK;
            }
        }
    }

    return result;
}


/**
 * @brief       Full-duplex transfer of byte_count bytes under one SS0 frame.
 * @req         LLR-QSPI-011
 * @design      SDD-QSPI-2.2
 */
qspi_hal_status_t qspi_hal_transceive_bytes(qspi_hal_config_t const * p_cfg,
                                            uint8_t const * p_tx_bytes,
                                            uint8_t * p_rx_bytes,
                                            size_t byte_count)
{
    qspi_hal_status_t result = QSPI_HAL_OK;
    bool is_cfg_ok = is_hal_cfg_ok(p_cfg);
    bool is_cnt_ok = is_byte_count_ok(byte_count);

    if ((!is_cfg_ok) || (!is_cnt_ok))
    {
        result = QSPI_HAL_ERR_PARAM;
    }
    else if ((NULL == p_tx_bytes) || (NULL == p_rx_bytes))
    {
        result = QSPI_HAL_ERR_PARAM;
    }
    else
    {
        uint32_t ctrl_inhibit = build_control_word(p_cfg->clock_mode);

        reset_fifos(p_cfg->base_addr, ctrl_inhibit);
        load_tx_fifo_array(p_cfg->base_addr, p_tx_bytes, byte_count);

        result = run_bus_cycle(p_cfg->base_addr,
                               ctrl_inhibit,
                               byte_count,
                               p_cfg->poll_timeout_us);

        if (QSPI_HAL_OK == result)
        {
            result = drain_rx_bytes(p_cfg->base_addr,
                                    p_rx_bytes,
                                    byte_count);
        }
        else
        {
            /* Propagate bus timeout or hardware fault */
        }
    }

    return result;
}


/**
 * @brief       Transmit byte_count bytes under one SS0 frame, ignore MISO.
 * @req         LLR-QSPI-012
 * @design      SDD-QSPI-2.3
 */
qspi_hal_status_t qspi_hal_write_bytes(qspi_hal_config_t const * p_cfg,
                                       uint8_t const * p_tx_bytes,
                                       size_t byte_count)
{
    qspi_hal_status_t result = QSPI_HAL_OK;
    bool is_cfg_ok = is_hal_cfg_ok(p_cfg);
    bool is_cnt_ok = is_byte_count_ok(byte_count);

    if ((!is_cfg_ok) || (!is_cnt_ok) || (NULL == p_tx_bytes))
    {
        result = QSPI_HAL_ERR_PARAM;
    }
    else
    {
        uint32_t ctrl_inhibit = build_control_word(p_cfg->clock_mode);

        reset_fifos(p_cfg->base_addr, ctrl_inhibit);
        load_tx_fifo_array(p_cfg->base_addr, p_tx_bytes, byte_count);

        /* RX bytes are left in the FIFO; next reset_fifos() drops them */
        result = run_bus_cycle(p_cfg->base_addr,
                               ctrl_inhibit,
                               byte_count,
                               p_cfg->poll_timeout_us);
    }

    return result;
}


/**
 * @brief       Read byte_count bytes while driving dummy_tx_byte on MOSI.
 * @req         LLR-QSPI-013
 * @design      SDD-QSPI-2.4
 */
qspi_hal_status_t qspi_hal_read_bytes(qspi_hal_config_t const * p_cfg,
                                      uint8_t dummy_tx_byte,
                                      uint8_t * p_rx_bytes,
                                      size_t byte_count)
{
    qspi_hal_status_t result = QSPI_HAL_OK;
    bool is_cfg_ok = is_hal_cfg_ok(p_cfg);
    bool is_cnt_ok = is_byte_count_ok(byte_count);

    if ((!is_cfg_ok) || (!is_cnt_ok) || (NULL == p_rx_bytes))
    {
        result = QSPI_HAL_ERR_PARAM;
    }
    else
    {
        uint32_t ctrl_inhibit = build_control_word(p_cfg->clock_mode);

        reset_fifos(p_cfg->base_addr, ctrl_inhibit);
        load_tx_fifo_fill(p_cfg->base_addr, dummy_tx_byte, byte_count);

        result = run_bus_cycle(p_cfg->base_addr,
                               ctrl_inhibit,
                               byte_count,
                               p_cfg->poll_timeout_us);

        if (QSPI_HAL_OK == result)
        {
            result = drain_rx_bytes(p_cfg->base_addr,
                                    p_rx_bytes,
                                    byte_count);
        }
        else
        {
            /* Propagate bus timeout or hardware fault */
        }
    }

    return result;
}


/**
 * @brief       Write a 32-bit word to an AXI Quad SPI register.
 * @req         LLR-QSPI-020
 * @design      SDD-QSPI-3.1
 * @param[in]   base_addr     IP base address, 4-byte aligned, != 0.
 * @param[in]   offset_bytes  Register offset in bytes, 0x00..0x78.
 * @param[in]   reg_word      32-bit word to write.
 * @note        The only MMIO store of the project for this IP (rule §9).
 *              Region must be Device-nGnRE mapped (see header).
 */
static inline void reg_write32(uintptr_t base_addr,
                               uint32_t offset_bytes,
                               uint32_t reg_word)
{
    /* MISRA deviation: Rule 11.4 — MMIO register access — only here;
     * deviation log entry DEV-QSPI-001 */
    volatile uint32_t * p_reg =
        (volatile uint32_t *)(base_addr + (uintptr_t)offset_bytes);

    ASSERT(0U != base_addr);

    *p_reg = reg_word;
}


/**
 * @brief       Read a 32-bit word from an AXI Quad SPI register.
 * @req         LLR-QSPI-021
 * @design      SDD-QSPI-3.2
 * @param[in]   base_addr     IP base address, 4-byte aligned, != 0.
 * @param[in]   offset_bytes  Register offset in bytes, 0x00..0x78.
 * @return      32-bit register word read from hardware.
 * @note        The only MMIO load of the project for this IP (rule §9).
 */
static inline uint32_t reg_read32(uintptr_t base_addr,
                                  uint32_t offset_bytes)
{
    /* MISRA deviation: Rule 11.4 — MMIO register access — only here;
     * deviation log entry DEV-QSPI-002 */
    volatile uint32_t const * p_reg =
        (volatile uint32_t const *)(base_addr + (uintptr_t)offset_bytes);
    uint32_t reg_word = 0U;

    ASSERT(0U != base_addr);

    reg_word = *p_reg;

    return reg_word;
}


/**
 * @brief       Check a QSPI HAL configuration against its bounds.
 * @req         LLR-QSPI-022
 * @design      SDD-QSPI-3.3
 * @param[in]   p_cfg  Pointer to QSPI HAL configuration, may be NULL.
 * @return      true if every field is within bounds, false otherwise.
 * @note        One atomic condition per decision: no coupled conditions,
 *              MC/DC pairs exist for every check (rule §5).
 */
static bool is_hal_cfg_ok(qspi_hal_config_t const * p_cfg)
{
    bool is_ok = false;

    if (NULL == p_cfg)
    {
        is_ok = false;
    }
    else if (0U == p_cfg->base_addr)
    {
        is_ok = false;
    }
    else if (0U != (p_cfg->base_addr & QSPI_REG_ALIGN_MASK))
    {
        is_ok = false;
    }
    else if ((size_t)p_cfg->clock_mode >= QSPI_CLOCK_MODE_COUNT)
    {
        is_ok = false;
    }
    else if (0U == p_cfg->poll_timeout_us)
    {
        is_ok = false;
    }
    else if (p_cfg->poll_timeout_us > QSPI_HAL_MAX_TIMEOUT_US)
    {
        is_ok = false;
    }
    else
    {
        is_ok = true;
    }

    return is_ok;
}


/**
 * @brief       Check that a byte count fits the 256-byte hardware FIFO.
 * @req         LLR-QSPI-023
 * @design      SDD-QSPI-3.4
 * @param[in]   byte_count  Requested number of bytes.
 * @return      true if byte_count is in 1..256, false otherwise.
 * @note        Pure helper; cannot fail.
 */
static bool is_byte_count_ok(size_t byte_count)
{
    bool is_ok = false;

    if (0U == byte_count)
    {
        is_ok = false;
    }
    else if (byte_count > QSPI_HAL_FIFO_DEPTH_BYTES)
    {
        is_ok = false;
    }
    else
    {
        is_ok = true;
    }

    return is_ok;
}


/**
 * @brief       Build the SPICR word: enabled master, manual SS, inhibited.
 * @req         LLR-QSPI-024
 * @design      SDD-QSPI-3.5
 * @param[in]   clock_mode  Validated SPI CPOL/CPHA mode, 0..3.
 * @return      32-bit SPICR word with Master Transaction Inhibit set.
 * @note        Pure helper; cannot fail. Caller validated clock_mode.
 */
static uint32_t build_control_word(qspi_clock_mode_t clock_mode)
{
    uint32_t ctrl_word = QSPI_SPICR_SPE_MASK
                         | QSPI_SPICR_MASTER_MASK
                         | QSPI_SPICR_MANUAL_SS_MASK
                         | QSPI_SPICR_INHIBIT_MASK;
    size_t mode_idx = (size_t)clock_mode;

    if (mode_idx < QSPI_CLOCK_MODE_COUNT)
    {
        ctrl_word |= clock_mode_ctrl_bits[mode_idx];
    }
    else
    {
        /* DEFENSIVE [DO-178C]: RAM bit-flip — unreachable in normal
         * test; defensive log entry DEF-QSPI-001 */
    }

    return ctrl_word;
}


/**
 * @brief       Let the SRR reset pulse expire and confirm the reset state.
 * @req         LLR-QSPI-028
 * @design      SDD-QSPI-3.9
 * @param[in]   base_addr  IP base address, 4-byte aligned, != 0.
 * @return      true if SPICR reads its PG153 reset value afterwards.
 * @note        The AXI4-Lite slave serves one read at a time (>= 1 ACLK
 *              each) and Device memory keeps program order, so the next
 *              register write reaches the IP only after
 *              QSPI_RESET_SETTLE_READS reads. WCET: 32 AXI reads.
 */
static bool wait_reset_done(uintptr_t base_addr)
{
    bool is_done = false;
    uint32_t spicr_reg = 0U;
    uint32_t i = 0U;

    ASSERT(0U != base_addr);

    for (i = 0U; i < QSPI_RESET_SETTLE_READS; i++)
    {
        spicr_reg = reg_read32(base_addr, QSPI_REG_SPICR_OFFSET);
    }

    if (QSPI_SPICR_RESET_WORD == spicr_reg)
    {
        is_done = true;
    }
    else
    {
        is_done = false;
    }

    return is_done;
}


/**
 * @brief       Flush TX and RX FIFOs while the master stays inhibited.
 * @req         LLR-QSPI-025
 * @design      SDD-QSPI-3.6
 * @param[in]   base_addr     IP base address, 4-byte aligned, != 0.
 * @param[in]   ctrl_inhibit  SPICR word with Master Inhibit set.
 * @note        Leaves SS0 deasserted; it is asserted only in
 *              run_bus_cycle() so the CS-low window encloses SCK only.
 */
static void reset_fifos(uintptr_t base_addr,
                        uint32_t ctrl_inhibit)
{
    uint32_t ctrl_reset = ctrl_inhibit
                          | QSPI_SPICR_TX_RESET_MASK
                          | QSPI_SPICR_RX_RESET_MASK;

    ASSERT(0U != base_addr);

    reg_write32(base_addr, QSPI_REG_SPICR_OFFSET, ctrl_reset);
    reg_write32(base_addr, QSPI_REG_SPICR_OFFSET, ctrl_inhibit);
}


/**
 * @brief       Queue byte_count bytes from p_tx_bytes into the TX FIFO.
 * @req         LLR-QSPI-029
 * @design      SDD-QSPI-3.10
 * @param[in]   base_addr   IP base address, 4-byte aligned, != 0.
 * @param[in]   p_tx_bytes  Source bytes, not NULL (checked by caller).
 * @param[in]   byte_count  Number of bytes, 1..256 (checked by caller).
 * @note        Master is inhibited, so no SCK is generated yet.
 */
static void load_tx_fifo_array(uintptr_t base_addr,
                               uint8_t const * p_tx_bytes,
                               size_t byte_count)
{
    size_t i = 0U;

    ASSERT(NULL != p_tx_bytes);
    ASSERT(byte_count <= QSPI_HAL_FIFO_DEPTH_BYTES);

    for (i = 0U; i < byte_count; i++)
    {
        reg_write32(base_addr,
                    QSPI_REG_SPIDTR_OFFSET,
                    (uint32_t)p_tx_bytes[i]);
    }
}


/**
 * @brief       Queue fill_byte byte_count times into the TX FIFO.
 * @req         LLR-QSPI-030
 * @design      SDD-QSPI-3.11
 * @param[in]   base_addr   IP base address, 4-byte aligned, != 0.
 * @param[in]   fill_byte   Byte repeated on MOSI.
 * @param[in]   byte_count  Number of bytes, 1..256 (checked by caller).
 * @note        Master is inhibited, so no SCK is generated yet.
 */
static void load_tx_fifo_fill(uintptr_t base_addr,
                              uint8_t fill_byte,
                              size_t byte_count)
{
    size_t i = 0U;

    ASSERT(byte_count <= QSPI_HAL_FIFO_DEPTH_BYTES);

    for (i = 0U; i < byte_count; i++)
    {
        reg_write32(base_addr,
                    QSPI_REG_SPIDTR_OFFSET,
                    (uint32_t)fill_byte);
    }
}


/**
 * @brief       Return how many bytes the RX FIFO currently holds.
 * @req         LLR-QSPI-031
 * @design      SDD-QSPI-3.12
 * @param[in]   base_addr  IP base address, 4-byte aligned, != 0.
 * @return      Number of received bytes, 0..256.
 * @note        A byte enters the RX FIFO only after all 8 bits have been
 *              shifted, so this count is the true transfer progress.
 *              RXOCY is valid only when RX is not empty (PG153).
 */
static size_t read_rx_count(uintptr_t base_addr)
{
    size_t rx_count = 0U;
    uint32_t status_reg = reg_read32(base_addr, QSPI_REG_SPISR_OFFSET);

    if (0U == (status_reg & QSPI_SPISR_RX_EMPTY_MASK))
    {
        uint32_t occupancy_reg = reg_read32(base_addr,
                                            QSPI_REG_RXOCY_OFFSET);

        rx_count = (size_t)(occupancy_reg & QSPI_RXOCY_MASK) + 1U;
    }
    else
    {
        /* RX FIFO empty: no byte has finished shifting yet */
    }

    return rx_count;
}


/**
 * @brief       Assert SS0, clock out the TX FIFO, wait for the last bit.
 * @req         LLR-QSPI-026
 * @design      SDD-QSPI-3.7
 * @param[in]   base_addr     IP base address, 4-byte aligned, != 0.
 * @param[in]   ctrl_inhibit  SPICR word with Master Inhibit set.
 * @param[in]   byte_count    Bytes queued in TX FIFO, 1..256.
 * @param[in]   timeout_us    Minimum wait in microseconds, 1..1000000.
 * @return      QSPI_HAL_OK, QSPI_HAL_ERR_TIMEOUT, QSPI_HAL_ERR_HW.
 * @note        Completion = byte_count bytes received, NOT TX FIFO empty:
 *              Tx_Empty sets while the last byte is still shifting, and
 *              raising SS0 then would cut that byte. SS0 is always
 *              deasserted, also on timeout, and read back to flush the
 *              posted write and prove the line is high.
 *              WCET: 1e8 poll iterations (QSPI_HAL_MAX_TIMEOUT_US * 100).
 */
static qspi_hal_status_t run_bus_cycle(uintptr_t base_addr,
                                       uint32_t ctrl_inhibit,
                                       size_t byte_count,
                                       uint32_t timeout_us)
{
    qspi_hal_status_t result = QSPI_HAL_OK;
    uint32_t ctrl_active = ctrl_inhibit & (~QSPI_SPICR_INHIBIT_MASK);
    uint32_t timeout_ticks = 0U;
    size_t rx_count = 0U;
    uint32_t ss_reg = 0U;
    uint32_t status_reg = 0U;

    ASSERT(timeout_us <= QSPI_HAL_MAX_TIMEOUT_US);

    /* timeout_us <= 1e6 (validated), so the product <= 1e8: no wrap */
    timeout_ticks = timeout_us * QSPI_POLL_ITERS_PER_US;

    /* SS0 low only now: the CS window encloses SCK activity only */
    reg_write32(base_addr, QSPI_REG_SPISSR_OFFSET, QSPI_SPISSR_ASSERT_SS0);

    /* Releasing Master Transaction Inhibit starts SCK */
    reg_write32(base_addr, QSPI_REG_SPICR_OFFSET, ctrl_active);

    rx_count = read_rx_count(base_addr);
    while ((rx_count < byte_count) && (0U < timeout_ticks))
    {
        timeout_ticks--;
        rx_count = read_rx_count(base_addr);
    }

    reg_write32(base_addr, QSPI_REG_SPICR_OFFSET, ctrl_inhibit);
    reg_write32(base_addr,
                QSPI_REG_SPISSR_OFFSET,
                QSPI_SPISSR_DEASSERT_ALL);

    /* Same-peripheral read after write: returns only once SS0 is high */
    ss_reg = reg_read32(base_addr, QSPI_REG_SPISSR_OFFSET);
    status_reg = reg_read32(base_addr, QSPI_REG_SPISR_OFFSET);

    if (0U != (status_reg & QSPI_SPISR_MODF_MASK))
    {
        result = QSPI_HAL_ERR_HW;
    }
    else if (rx_count < byte_count)
    {
        result = QSPI_HAL_ERR_TIMEOUT;
    }
    else if (0U == (ss_reg & QSPI_SPISSR_SS0_MASK))
    {
        /* SS0 still reads asserted: slave select stuck low */
        result = QSPI_HAL_ERR_HW;
    }
    else
    {
        result = QSPI_HAL_OK;
    }

    return result;
}


/**
 * @brief       Copy byte_count received bytes out of the RX FIFO.
 * @req         LLR-QSPI-027
 * @design      SDD-QSPI-3.8
 * @param[in]   base_addr   IP base address, 4-byte aligned, != 0.
 * @param[out]  p_rx_bytes  Destination array, not NULL (checked by caller).
 * @param[in]   byte_count  Number of bytes to read, 1..256.
 * @return      QSPI_HAL_OK, QSPI_HAL_ERR_HW.
 * @note        run_bus_cycle() already saw byte_count bytes; RX empty here
 *              means a hardware fault, reported as QSPI_HAL_ERR_HW.
 *              The bytes popped before the fault stay written in
 *              p_rx_bytes (no roll-back): the caller discards the whole
 *              array unless QSPI_HAL_OK is returned.
 */
static qspi_hal_status_t drain_rx_bytes(uintptr_t base_addr,
                                        uint8_t * p_rx_bytes,
                                        size_t byte_count)
{
    qspi_hal_status_t result = QSPI_HAL_OK;
    size_t i = 0U;

    ASSERT(NULL != p_rx_bytes);
    ASSERT(byte_count <= QSPI_HAL_FIFO_DEPTH_BYTES);

    for (i = 0U; i < byte_count; i++)
    {
        uint32_t status_reg = reg_read32(base_addr, QSPI_REG_SPISR_OFFSET);

        if (0U != (status_reg & QSPI_SPISR_RX_EMPTY_MASK))
        {
            result = QSPI_HAL_ERR_HW;
            break;
        }
        else
        {
            uint32_t rx_reg = reg_read32(base_addr, QSPI_REG_SPIDRR_OFFSET);

            p_rx_bytes[i] = (uint8_t)(rx_reg & QSPI_BYTE_MASK);
        }
    }

    return result;
}

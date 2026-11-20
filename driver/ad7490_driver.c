/**
 * @file        ad7490_driver.c
 * @brief       AD7490 driver implementation.
 * @req         LLR-AD7490-001
 * @design      SDD-AD7490-1.0
 *
 * Sections (top to bottom, after the public functions):
 *   A. Device helpers     - power-up, single read, power mode
 *   B. Sequencer helpers  - Shadow Register, scans
 *   C. Frame layer        - Control Register encoding, 16-bit frames
 *   D. Conversion         - 12-bit code to millivolts
 */

#include "ad7490_driver.h"

/* ------------------------------------------------------------------ */
/* Serial frame and register map (AD7490 datasheet)                    */
/* ------------------------------------------------------------------ */

/* One conversion = one CS frame of 16 SCLK = 2 bytes (Figure 17) */
#define AD7490_FRAME_BYTES          (2U)

/* DIN all 1s for the two power-up dummy conversions (Figure 16) */
#define AD7490_DUMMY_DIN_WORD       (0xFFFFU)

/* DIN with WRITE = 0: Control Register left unchanged (Table I) */
#define AD7490_NOP_DIN_BYTE         (0x00U)
#define AD7490_NOP_DIN_WORD         (0x0000U)

/* 12-bit Control Register, bit 11 = MSB (Table I) */
#define AD7490_CR_WRITE_BIT         (0x0800U) /* Bit 11: WRITE         */
#define AD7490_CR_SEQ_BIT           (0x0400U) /* Bit 10: SEQ           */
#define AD7490_CR_ADD_SHIFT         (6U)      /* Bits 9..6: ADD3..ADD0 */
#define AD7490_CR_PM_SHIFT          (4U)      /* Bits 5..4: PM1..PM0   */
#define AD7490_CR_SHADOW_BIT        (0x0008U) /* Bit 3: SHADOW         */
#define AD7490_CR_WEAK_TRI_SHIFT    (2U)      /* Bit 2: WEAK/TRI       */
#define AD7490_CR_RANGE_SHIFT       (1U)      /* Bit 1: RANGE          */
#define AD7490_CR_CODING_SHIFT      (0U)      /* Bit 0: CODING         */

/* CR is clocked in on the first 12 SCLK: DIN bits 15..4 (Table I) */
#define AD7490_CR_FRAME_SHIFT       (4U)

/* DOUT word: ADD3..ADD0 in bits 15..12, DB11..DB0 in bits 11..0 */
#define AD7490_DOUT_CHAN_SHIFT      (12U)
#define AD7490_DOUT_CHAN_MASK       (0x000FU)
#define AD7490_DOUT_RAW_MASK        (0x0FFFU)

/* MSB-first byte (de)serialisation of a 16-bit frame */
#define AD7490_HIGH_BYTE_SHIFT      (8U)
#define AD7490_BYTE_MASK            (0x00FFU)
#define AD7490_FRAME_MASK           (0xFFFFU)

/* Shadow Register: VIN0 is bit 15 (MSB), VIN15 is bit 0 (Figure 18) */
#define AD7490_SHADOW_VIN0_BIT      (15U)

/* ------------------------------------------------------------------ */
/* Driver constants                                                    */
/* ------------------------------------------------------------------ */

/* Dummy conversions with DIN held high after power-on (Figure 16) */
#define AD7490_POWER_UP_DUMMY_FRAMES (2U)

/* ADD3..ADD0 written with every non-read CR write. Any channel is valid:
 * single reads rewrite ADD3..ADD0, the sequencer ignores it. */
#define AD7490_DEFAULT_CHANNEL      (0U)

/* Number of members in ad7490_seq_mode_t */
#define AD7490_SEQ_MODE_COUNT       (4U)

/* 12-bit converter: 2^12 codes per span, 1 LSB = span / 4096 (Fig 8, 9) */
#define AD7490_CODES_PER_SPAN       (4096U)

/* Half an LSB step of the division, for round-to-nearest */
#define AD7490_ROUND_HALF_CODES     (2048U)

/* Twos complement MSB (DB11); flipping it gives offset binary (Fig 9) */
#define AD7490_TWOS_COMP_MSB        (0x0800U)

/* RANGE = 0 doubles the input span to 2 x REFIN (Table I) */
#define AD7490_SPAN_FACTOR_2X       (2U)

/**
 * @brief       SEQ / SHADOW bit combinations (AD7490 Table IV).
 * @req         LLR-AD7490-032
 * @design      SDD-AD7490-3.13
 * @note        One named mode per row of the datasheet table instead of
 *              two bool flags (minimal control coupling, rule §3).
 */
typedef enum
{
    AD7490_SEQ_MODE_SINGLE      = 0U, /* SEQ 0 SHADOW 0: ADD3..0 = channel  */
    AD7490_SEQ_MODE_LOAD_SHADOW = 1U, /* SEQ 0 SHADOW 1: next frame = mask  */
    AD7490_SEQ_MODE_KEEP_SEQ    = 2U, /* SEQ 1 SHADOW 0: CR write, seq runs */
    AD7490_SEQ_MODE_CONSECUTIVE = 3U  /* SEQ 1 SHADOW 1: VIN0..ADD3..0 loop */
} ad7490_seq_mode_t;

/**
 * @brief       Fields decoded from one 16-bit DOUT word.
 * @req         LLR-AD7490-033
 * @design      SDD-AD7490-3.14
 */
typedef struct
{
    uint8_t  channel_id; /* ADD3..ADD0 from DOUT[15:12], 0..15 */
    uint16_t raw_lsb;    /* DB11..DB0 from DOUT[11:0], 0..4095 */
} ad7490_dout_t;

/**
 * @brief       One 16-bit frame as the 2 bytes on the wire, MSB first.
 * @req         LLR-AD7490-027
 * @design      SDD-AD7490-3.8
 * @note        Wrapping the array in a struct lets helpers pass it by
 *              value: no pointer, no length, no aliasing (rule §8).
 */
typedef struct
{
    uint8_t bytes[AD7490_FRAME_BYTES]; /* [0] = bits 15..8, [1] = 7..0 */
} frame_bytes_t;

/* A. Device helpers */
static bool is_dev_cfg_ok(ad7490_config_t const * p_cfg);
static ad7490_status_t run_power_up(ad7490_config_t const * p_cfg);
static ad7490_status_t read_single(ad7490_config_t const * p_cfg,
                                   uint8_t channel_id,
                                   ad7490_sample_t * p_sample);
static bool is_auto_power_mode(ad7490_power_mode_t power_mode);
static ad7490_status_t read_after_wakeup(qspi_hal_config_t const * p_qspi_cfg,
                                         uint16_t * p_rx_frame);
static ad7490_status_t decode_sample(ad7490_config_t const * p_cfg,
                                     uint8_t channel_id,
                                     uint16_t rx_frame,
                                     ad7490_sample_t * p_sample);
static ad7490_status_t write_power_mode(qspi_hal_config_t const * p_qspi_cfg,
                                        ad7490_ctrl_cfg_t const * p_ctrl_cfg);

/* B. Sequencer helpers */
static uint16_t mask_to_shadow(uint16_t channel_mask);
static uint32_t count_channels(uint16_t channel_mask);
static ad7490_status_t send_start_frames(ad7490_config_t const * p_cfg,
                                         uint16_t shadow_word);
static ad7490_status_t send_stop_frame(ad7490_config_t const * p_cfg);
static ad7490_status_t scan_one(ad7490_config_t const * p_cfg,
                                uint16_t channel_mask,
                                uint16_t tx_frame,
                                uint16_t * p_seen_mask,
                                ad7490_sample_t * p_samples);

/* C. Frame layer */
static ad7490_status_t init_bus(qspi_hal_config_t const * p_qspi_cfg);
static ad7490_status_t build_cr_frame(ad7490_ctrl_cfg_t const * p_ctrl_cfg,
                                      uint8_t channel_id,
                                      ad7490_seq_mode_t seq_mode,
                                      uint16_t * p_tx_frame);
static ad7490_status_t write_frame(qspi_hal_config_t const * p_qspi_cfg,
                                   uint16_t tx_frame);
static ad7490_status_t txrx_frame(qspi_hal_config_t const * p_qspi_cfg,
                                  uint16_t tx_frame,
                                  uint16_t * p_rx_frame);
static ad7490_status_t read_frame(qspi_hal_config_t const * p_qspi_cfg,
                                  uint16_t * p_rx_frame);
static ad7490_dout_t decode_dout(uint16_t rx_frame);
static bool is_ctrl_cfg_ok(ad7490_ctrl_cfg_t const * p_ctrl_cfg);
static uint32_t encode_cr_word(ad7490_ctrl_cfg_t const * p_ctrl_cfg,
                               uint8_t channel_id,
                               ad7490_seq_mode_t seq_mode);
static frame_bytes_t frame_to_bytes(uint16_t frame_word);
static uint16_t bytes_to_frame(frame_bytes_t frame_bytes);
static ad7490_status_t map_hal_status(qspi_hal_status_t hal_status);

/* D. Conversion */
static bool is_vref_ok(uint32_t vref_mv);
static ad7490_status_t raw_to_mv(uint16_t raw_lsb,
                                 uint32_t vref_mv,
                                 ad7490_range_t range_mode,
                                 ad7490_coding_t coding,
                                 uint32_t * p_voltage_mv);
static uint32_t raw_to_offset_binary(uint16_t raw_lsb,
                                     ad7490_coding_t coding);
static uint32_t span_from_vref_mv(uint32_t vref_mv,
                                  ad7490_range_t range_mode);

/**
 * @brief       SEQ / SHADOW Control Register bits per ad7490_seq_mode_t.
 * @req         LLR-AD7490-031
 * @design      SDD-AD7490-3.12
 */
static const uint32_t seq_mode_cr_bits[AD7490_SEQ_MODE_COUNT] =
{
    [AD7490_SEQ_MODE_SINGLE]      = 0U,
    [AD7490_SEQ_MODE_LOAD_SHADOW] = AD7490_CR_SHADOW_BIT,
    [AD7490_SEQ_MODE_KEEP_SEQ]    = AD7490_CR_SEQ_BIT,
    [AD7490_SEQ_MODE_CONSECUTIVE] = (AD7490_CR_SEQ_BIT | AD7490_CR_SHADOW_BIT)
};


/* ================================================================== */
/* Public functions                                                    */
/* ================================================================== */

/**
 * @brief       Power up the AD7490 and load its Control Register.
 * @req         LLR-AD7490-010
 * @design      SDD-AD7490-2.1
 */
ad7490_status_t ad7490_init(ad7490_dev_t * p_dev,
                            ad7490_config_t const * p_cfg)
{
    ad7490_status_t result = AD7490_OK;

    if (NULL == p_dev)
    {
        result = AD7490_ERR_PARAM;
    }
    else
    {
        bool is_cfg_ok = is_dev_cfg_ok(p_cfg);

        /* Pessimistic: the handle is unusable until power-up succeeds */
        p_dev->state = AD7490_STATE_FAULT;
        p_dev->channel_mask = 0U;

        if (!is_cfg_ok)
        {
            result = AD7490_ERR_PARAM;
        }
        else
        {
            result = run_power_up(p_cfg);

            if (AD7490_OK == result)
            {
                p_dev->cfg = *p_cfg;
                p_dev->state = AD7490_STATE_READY;
            }
            else
            {
                /* State stays FAULT: ADC content unknown */
            }
        }
    }

    return result;
}


/**
 * @brief       Convert one analog input channel (VIN0..VIN15).
 * @req         LLR-AD7490-011
 * @design      SDD-AD7490-2.2
 */
ad7490_status_t ad7490_read_channel(ad7490_dev_t const * p_dev,
                                    uint8_t channel_id,
                                    ad7490_sample_t * p_sample)
{
    ad7490_status_t result = AD7490_OK;

    if (NULL == p_dev)
    {
        result = AD7490_ERR_PARAM;
    }
    else if (NULL == p_sample)
    {
        result = AD7490_ERR_PARAM;
    }
    else if (channel_id >= AD7490_CHANNEL_COUNT)
    {
        result = AD7490_ERR_PARAM;
    }
    else if (AD7490_STATE_READY != p_dev->state)
    {
        result = AD7490_ERR_STATE;
    }
    else if (AD7490_POWER_FULL_SHUTDOWN == p_dev->cfg.ctrl_cfg.power_mode)
    {
        /* All analog circuitry is off: a conversion would be invalid */
        result = AD7490_ERR_STATE;
    }
    else
    {
        result = read_single(&p_dev->cfg, channel_id, p_sample);
    }

    return result;
}


/**
 * @brief       Change the AD7490 power management mode (PM1..PM0).
 * @req         LLR-AD7490-014
 * @design      SDD-AD7490-2.5
 */
ad7490_status_t ad7490_set_power_mode(ad7490_dev_t * p_dev,
                                      ad7490_power_mode_t power_mode)
{
    ad7490_status_t result = AD7490_OK;

    if (NULL == p_dev)
    {
        result = AD7490_ERR_PARAM;
    }
    else if ((uint32_t)power_mode > (uint32_t)AD7490_POWER_NORMAL)
    {
        result = AD7490_ERR_PARAM;
    }
    else if (AD7490_STATE_READY != p_dev->state)
    {
        result = AD7490_ERR_STATE;
    }
    else
    {
        ad7490_ctrl_cfg_t new_ctrl_cfg = p_dev->cfg.ctrl_cfg;

        new_ctrl_cfg.power_mode = power_mode;
        result = write_power_mode(&p_dev->cfg.qspi_cfg, &new_ctrl_cfg);

        if (AD7490_OK == result)
        {
            p_dev->cfg.ctrl_cfg.power_mode = power_mode;
        }
        else
        {
            /* PM1..PM0 inside the ADC are now unknown: force re-init */
            p_dev->state = AD7490_STATE_FAULT;
        }
    }

    return result;
}


/**
 * @brief       Program the Shadow Register and start the sequence.
 * @req         LLR-AD7490-012
 * @design      SDD-AD7490-5.1
 */
ad7490_status_t ad7490_sequencer_start(ad7490_dev_t * p_dev,
                                       uint16_t channel_mask)
{
    ad7490_status_t result = AD7490_OK;

    if (NULL == p_dev)
    {
        result = AD7490_ERR_PARAM;
    }
    else if (0U == channel_mask)
    {
        result = AD7490_ERR_PARAM;
    }
    else if (AD7490_STATE_READY != p_dev->state)
    {
        result = AD7490_ERR_STATE;
    }
    else if (AD7490_POWER_NORMAL != p_dev->cfg.ctrl_cfg.power_mode)
    {
        /* AUTO modes power down after each conversion: not sequenced */
        result = AD7490_ERR_STATE;
    }
    else
    {
        uint16_t shadow_word = mask_to_shadow(channel_mask);

        result = send_start_frames(&p_dev->cfg, shadow_word);

        if (AD7490_OK == result)
        {
            p_dev->channel_mask = channel_mask;
            p_dev->state = AD7490_STATE_SEQUENCING;
        }
        else
        {
            /* Sequencer state inside the ADC unknown: force re-init */
            p_dev->state = AD7490_STATE_FAULT;
        }
    }

    return result;
}


/**
 * @brief       Read one full pass: one sample of every sequenced channel.
 * @req         LLR-AD7490-013
 * @design      SDD-AD7490-5.2
 */
ad7490_status_t ad7490_sequencer_read_scan(ad7490_dev_t * p_dev,
                                           ad7490_sample_t * p_samples,
                                           size_t sample_count)
{
    ad7490_status_t result = AD7490_OK;

    if (NULL == p_dev)
    {
        result = AD7490_ERR_PARAM;
    }
    else if (NULL == p_samples)
    {
        result = AD7490_ERR_PARAM;
    }
    else if (sample_count < AD7490_CHANNEL_COUNT)
    {
        result = AD7490_ERR_PARAM;
    }
    else if (AD7490_STATE_SEQUENCING != p_dev->state)
    {
        result = AD7490_ERR_STATE;
    }
    else if (0U == p_dev->channel_mask)
    {
        /* Corrupted handle: SEQUENCING without channels */
        result = AD7490_ERR_STATE;
    }
    else
    {
        /* Bound: popcount of a 16-bit mask <= AD7490_CHANNEL_COUNT */
        uint32_t channel_total = count_channels(p_dev->channel_mask);
        uint16_t seen_mask = 0U;
        uint16_t tx_frame = AD7490_NOP_DIN_WORD;
        uint32_t k = 0U;

        /*
         * CR scrubbing: the first frame of every pass rewrites the
         * Control Register with SEQ = 1, SHADOW = 0, which updates the
         * CR without stopping the sequence (Table IV). A RANGE, CODING
         * or PM bit upset by EMI / ESD / supply glitch is repaired
         * within one pass; the other frames send WRITE = 0.
         */
        result = build_cr_frame(&p_dev->cfg.ctrl_cfg,
                                AD7490_DEFAULT_CHANNEL,
                                AD7490_SEQ_MODE_KEEP_SEQ,
                                &tx_frame);

        /*
         * The sequence is a ring over the sequenced channels, so any
         * channel_total consecutive frames hold each one exactly once,
         * whichever channel the pass starts on.
         */
        for (k = 0U; k < channel_total; k++)
        {
            if (AD7490_OK == result)
            {
                result = scan_one(&p_dev->cfg,
                                  p_dev->channel_mask,
                                  tx_frame,
                                  &seen_mask,
                                  p_samples);
                tx_frame = AD7490_NOP_DIN_WORD;
            }
            else
            {
                /* Earlier frame failed: skip the rest of the pass */
            }
        }

        if (AD7490_OK != result)
        {
            /* Lost frame or sync: sequence position unknown */
            p_dev->state = AD7490_STATE_FAULT;
        }
        else
        {
            /* Pass complete: every sequenced channel stored once */
        }
    }

    return result;
}


/**
 * @brief       Stop the sequence and return to single-channel mode.
 * @req         LLR-AD7490-015
 * @design      SDD-AD7490-5.3
 */
ad7490_status_t ad7490_sequencer_stop(ad7490_dev_t * p_dev)
{
    ad7490_status_t result = AD7490_OK;

    if (NULL == p_dev)
    {
        result = AD7490_ERR_PARAM;
    }
    else if (AD7490_STATE_SEQUENCING != p_dev->state)
    {
        result = AD7490_ERR_STATE;
    }
    else
    {
        result = send_stop_frame(&p_dev->cfg);

        if (AD7490_OK == result)
        {
            p_dev->channel_mask = 0U;
            p_dev->state = AD7490_STATE_READY;
        }
        else
        {
            p_dev->state = AD7490_STATE_FAULT;
        }
    }

    return result;
}


/* ================================================================== */
/* A. Device helpers                                                   */
/* ================================================================== */

/**
 * @brief       Check the device-level constraints of a configuration.
 * @req         LLR-AD7490-042
 * @design      SDD-AD7490-2.6
 * @param[in]   p_cfg  Configuration from the public API, may be NULL.
 * @return      true if REFIN, SPI mode and timeout suit the AD7490.
 * @note        One atomic condition per decision (MC/DC, rule §5).
 *              ctrl_cfg is checked by build_cr_frame() and the remaining
 *              qspi_cfg fields by the HAL, before any frame is sent.
 */
static bool is_dev_cfg_ok(ad7490_config_t const * p_cfg)
{
    bool is_ok = false;

    if (NULL == p_cfg)
    {
        is_ok = false;
    }
    else if (!is_vref_ok(p_cfg->vref_mv))
    {
        is_ok = false;
    }
    else if (AD7490_QSPI_CLOCK_MODE != p_cfg->qspi_cfg.clock_mode)
    {
        is_ok = false;
    }
    else if (p_cfg->qspi_cfg.poll_timeout_us < AD7490_MIN_TIMEOUT_US)
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
 * @brief       QSPI init, 2 dummy frames with DIN high, then the CR write.
 * @req         LLR-AD7490-043
 * @design      SDD-AD7490-2.7
 * @param[in]   p_cfg  Validated configuration.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_SPI,
 *              AD7490_ERR_TIMEOUT.
 * @note        The CR frame is built first, so an invalid ctrl_cfg is
 *              rejected before the QSPI IP is touched. DIN = 0xFFFF on
 *              the dummy frames keeps WRITE = 1 with a defined pattern,
 *              replacing whatever the CR held after power-on (Figure 16).
 *              WCET: QSPI init + 3 frames.
 */
static ad7490_status_t run_power_up(ad7490_config_t const * p_cfg)
{
    ad7490_status_t result = AD7490_OK;
    uint16_t cr_frame = 0U;
    uint32_t i = 0U;

    ASSERT(NULL != p_cfg);

    result = build_cr_frame(&p_cfg->ctrl_cfg,
                            AD7490_DEFAULT_CHANNEL,
                            AD7490_SEQ_MODE_SINGLE,
                            &cr_frame);

    if (AD7490_OK == result)
    {
        result = init_bus(&p_cfg->qspi_cfg);
    }
    else
    {
        /* Propagate invalid ctrl_cfg */
    }

    for (i = 0U; i < AD7490_POWER_UP_DUMMY_FRAMES; i++)
    {
        if (AD7490_OK == result)
        {
            result = write_frame(&p_cfg->qspi_cfg, AD7490_DUMMY_DIN_WORD);
        }
        else
        {
            /* Earlier step failed: skip remaining dummy frames */
        }
    }

    if (AD7490_OK == result)
    {
        /* Third frame loads the requested Control Register (Figure 16) */
        result = write_frame(&p_cfg->qspi_cfg, cr_frame);
    }
    else
    {
        /* Propagate error */
    }

    return result;
}


/**
 * @brief       Single-channel conversion (2 or 3 frames) and decoding.
 * @req         LLR-AD7490-044
 * @design      SDD-AD7490-2.8
 * @param[in]   p_cfg       Validated configuration of a READY device.
 * @param[in]   channel_id  Validated channel, 0..15.
 * @param[out]  p_sample    Result, written only on AD7490_OK.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_SPI,
 *              AD7490_ERR_TIMEOUT, AD7490_ERR_CHAN_MISMATCH.
 * @note        The AD7490 converts, in frame N, the channel written in
 *              frame N-1 (Table IV, Fig 2). Frame 1 selects channel_id.
 *              NORMAL: frame 2 converts it (2 frames).
 *              AUTO_STANDBY / AUTO_SHUTDOWN: the CR write of frame 1
 *              sends the part to its low-power state on the CS rising
 *              edge, so frame 2 is the power-up dummy (INVALID data) and
 *              frame 3 converts channel_id (3 frames, Fig 14, 15).
 */
static ad7490_status_t read_single(ad7490_config_t const * p_cfg,
                                   uint8_t channel_id,
                                   ad7490_sample_t * p_sample)
{
    ad7490_status_t result = AD7490_OK;
    uint16_t cr_frame = 0U;
    uint16_t rx_frame = 0U;
    bool is_auto_mode = false;

    ASSERT(NULL != p_cfg);
    ASSERT(NULL != p_sample);

    is_auto_mode = is_auto_power_mode(p_cfg->ctrl_cfg.power_mode);
    result = build_cr_frame(&p_cfg->ctrl_cfg,
                            channel_id,
                            AD7490_SEQ_MODE_SINGLE,
                            &cr_frame);

    if (AD7490_OK == result)
    {
        /* Frame 1: select channel_id; its own result is discarded */
        result = write_frame(&p_cfg->qspi_cfg, cr_frame);
    }
    else
    {
        /* Propagate invalid ctrl_cfg (handle corrupted) */
    }

    if (AD7490_OK != result)
    {
        /* Propagate error from frame 1 */
    }
    else if (is_auto_mode)
    {
        /* Frames 2 and 3: power-up dummy, then the valid result */
        result = read_after_wakeup(&p_cfg->qspi_cfg, &rx_frame);
    }
    else
    {
        /* Frame 2: converts channel_id; CR rewritten, ADD unchanged */
        result = txrx_frame(&p_cfg->qspi_cfg, cr_frame, &rx_frame);
    }

    if (AD7490_OK == result)
    {
        result = decode_sample(p_cfg, channel_id, rx_frame, p_sample);
    }
    else
    {
        /* Propagate error from the conversion frame(s) */
    }

    return result;
}


/**
 * @brief       Tell whether a power mode sleeps after each conversion.
 * @req         LLR-AD7490-047
 * @design      SDD-AD7490-2.11
 * @param[in]   power_mode  Validated power mode.
 * @return      true for AUTO_STANDBY and AUTO_SHUTDOWN, false otherwise.
 * @note        Pure helper; cannot fail. One condition per decision.
 */
static bool is_auto_power_mode(ad7490_power_mode_t power_mode)
{
    bool is_auto = false;

    if (AD7490_POWER_AUTO_STANDBY == power_mode)
    {
        is_auto = true;
    }
    else if (AD7490_POWER_AUTO_SHUTDOWN == power_mode)
    {
        is_auto = true;
    }
    else
    {
        is_auto = false;
    }

    return is_auto;
}


/**
 * @brief       AUTO modes: power-up dummy frame, then the result frame.
 * @req         LLR-AD7490-048
 * @design      SDD-AD7490-2.12
 * @param[in]   p_qspi_cfg  QSPI instance of a READY device.
 * @param[out]  p_rx_frame  DOUT of the valid conversion, only on OK.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_SPI,
 *              AD7490_ERR_TIMEOUT.
 * @note        Both frames send WRITE = 0, as the datasheet requires for
 *              the dummy cycle (Fig 14, 15): the CR (channel, PM) stays
 *              unchanged. The dummy frame (16 SCLK = 2.56 us) covers the
 *              1 us wake-up time t12; its DOUT is invalid and discarded.
 *              WCET: 2 frames.
 */
static ad7490_status_t read_after_wakeup(qspi_hal_config_t const * p_qspi_cfg,
                                         uint16_t * p_rx_frame)
{
    ad7490_status_t result = AD7490_OK;
    uint16_t dummy_rx_frame = 0U;

    ASSERT(NULL != p_rx_frame);

    result = read_frame(p_qspi_cfg, &dummy_rx_frame);

    if (AD7490_OK == result)
    {
        result = read_frame(p_qspi_cfg, p_rx_frame);
    }
    else
    {
        /* Propagate error from the dummy frame */
    }

    return result;
}


/**
 * @brief       Check the echoed channel and convert DOUT into a sample.
 * @req         LLR-AD7490-045
 * @design      SDD-AD7490-2.9
 * @param[in]   p_cfg       Configuration (vref, range, coding).
 * @param[in]   channel_id  Channel that was requested, 0..15.
 * @param[in]   rx_frame    DOUT word of the conversion frame.
 * @param[out]  p_sample    Result, written only on AD7490_OK.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_CHAN_MISMATCH.
 * @note        A wrong echoed channel means a lost frame or a broken
 *              DOUT line: the code is not converted and not returned.
 */
static ad7490_status_t decode_sample(ad7490_config_t const * p_cfg,
                                     uint8_t channel_id,
                                     uint16_t rx_frame,
                                     ad7490_sample_t * p_sample)
{
    ad7490_status_t result = AD7490_OK;
    ad7490_dout_t dout = decode_dout(rx_frame);
    uint32_t voltage_mv = 0U;

    ASSERT(NULL != p_cfg);
    ASSERT(NULL != p_sample);

    if (channel_id != dout.channel_id)
    {
        result = AD7490_ERR_CHAN_MISMATCH;
    }
    else
    {
        result = raw_to_mv(dout.raw_lsb,
                           p_cfg->vref_mv,
                           p_cfg->ctrl_cfg.range_mode,
                           p_cfg->ctrl_cfg.coding,
                           &voltage_mv);

        if (AD7490_OK == result)
        {
            p_sample->channel_id = dout.channel_id;
            p_sample->raw_lsb = dout.raw_lsb;
            p_sample->voltage_mv = voltage_mv;
        }
        else
        {
            /* Propagate invalid vref (handle corrupted) */
        }
    }

    return result;
}


/**
 * @brief       Write new PM1..PM0 and wait out the power-up time.
 * @req         LLR-AD7490-046
 * @design      SDD-AD7490-2.10
 * @param[in]   p_qspi_cfg  QSPI instance of a READY device.
 * @param[in]   p_ctrl_cfg  Control options carrying the new power mode.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_SPI,
 *              AD7490_ERR_TIMEOUT.
 * @note        Leaving full shutdown needs 1 us before a valid conversion
 *              (tPOWER-UP). One dummy frame (16 SCLK = 2.56 us) with
 *              WRITE = 0 covers it without touching the new CR; its DOUT
 *              is discarded on purpose. Not needed when entering
 *              FULL_SHUTDOWN. WCET: 2 frames.
 */
static ad7490_status_t write_power_mode(qspi_hal_config_t const * p_qspi_cfg,
                                        ad7490_ctrl_cfg_t const * p_ctrl_cfg)
{
    ad7490_status_t result = AD7490_OK;
    uint16_t cr_frame = 0U;
    uint16_t dummy_rx_frame = 0U;

    ASSERT(NULL != p_ctrl_cfg);

    result = build_cr_frame(p_ctrl_cfg,
                            AD7490_DEFAULT_CHANNEL,
                            AD7490_SEQ_MODE_SINGLE,
                            &cr_frame);

    if (AD7490_OK == result)
    {
        result = write_frame(p_qspi_cfg, cr_frame);
    }
    else
    {
        /* Propagate invalid ctrl_cfg */
    }

    if ((AD7490_OK == result)
        && (AD7490_POWER_FULL_SHUTDOWN != p_ctrl_cfg->power_mode))
    {
        result = read_frame(p_qspi_cfg, &dummy_rx_frame);
    }
    else
    {
        /* Error, or entering full shutdown: no power-up wait needed */
    }

    return result;
}


/* ================================================================== */
/* B. Sequencer helpers                                                */
/* ================================================================== */

/**
 * @brief       Re-order a VINn bit mask into Shadow Register bit order.
 * @req         LLR-AD7490-050
 * @design      SDD-AD7490-5.4
 * @param[in]   channel_mask  bit n = VINn.
 * @return      Shadow word, bit (15 - n) = VINn (Figure 18).
 * @note        Pure helper; cannot fail. WCET: 16 iterations.
 *              Example: 0x0003 (VIN0 + VIN1) -> 0xC000.
 */
static uint16_t mask_to_shadow(uint16_t channel_mask)
{
    uint32_t shadow_word = 0U;
    uint32_t chan_idx = 0U;

    for (chan_idx = 0U; chan_idx < AD7490_CHANNEL_COUNT; chan_idx++)
    {
        if (0U != ((uint32_t)channel_mask & ((uint32_t)1U << chan_idx)))
        {
            uint32_t bit_pos = AD7490_SHADOW_VIN0_BIT - chan_idx;

            shadow_word |= ((uint32_t)1U << bit_pos);
        }
        else
        {
            /* Channel not sequenced */
        }
    }

    return (uint16_t)shadow_word;
}


/**
 * @brief       Count the channels set in a VINn bit mask.
 * @req         LLR-AD7490-051
 * @design      SDD-AD7490-5.5
 * @param[in]   channel_mask  bit n = VINn.
 * @return      Number of set bits, 0..16.
 * @note        Pure helper; cannot fail. WCET: 16 iterations.
 */
static uint32_t count_channels(uint16_t channel_mask)
{
    uint32_t channel_total = 0U;
    uint32_t chan_idx = 0U;

    for (chan_idx = 0U; chan_idx < AD7490_CHANNEL_COUNT; chan_idx++)
    {
        if (0U != ((uint32_t)channel_mask & ((uint32_t)1U << chan_idx)))
        {
            channel_total++;
        }
        else
        {
            /* Channel not sequenced */
        }
    }

    return channel_total;
}


/**
 * @brief       Shadow-load CR write, Shadow Register write, priming frame.
 * @req         LLR-AD7490-052
 * @design      SDD-AD7490-5.6
 * @param[in]   p_cfg        Configuration of a READY device.
 * @param[in]   shadow_word  Shadow Register content, VIN0 = bit 15.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_SPI,
 *              AD7490_ERR_TIMEOUT.
 * @note        Frame 1: CR with SEQ = 0, SHADOW = 1 (Table IV).
 *              Frame 2: Shadow Register (Figure 18).
 *              Frame 3: priming frame, WRITE = 0, DOUT discarded. It
 *              flushes the conversion still selected by ADD3..ADD0, so
 *              the first scan sees only sequenced channels whatever the
 *              exact start latency of the sequencer. WCET: 3 frames.
 */
static ad7490_status_t send_start_frames(ad7490_config_t const * p_cfg,
                                         uint16_t shadow_word)
{
    ad7490_status_t result = AD7490_OK;
    uint16_t cr_frame = 0U;
    uint16_t prime_rx_frame = 0U;

    ASSERT(NULL != p_cfg);

    result = build_cr_frame(&p_cfg->ctrl_cfg,
                            AD7490_DEFAULT_CHANNEL,
                            AD7490_SEQ_MODE_LOAD_SHADOW,
                            &cr_frame);

    if (AD7490_OK == result)
    {
        result = write_frame(&p_cfg->qspi_cfg, cr_frame);
    }
    else
    {
        /* Propagate invalid ctrl_cfg (handle corrupted) */
    }

    if (AD7490_OK == result)
    {
        result = write_frame(&p_cfg->qspi_cfg, shadow_word);
    }
    else
    {
        /* Propagate error */
    }

    if (AD7490_OK == result)
    {
        result = read_frame(&p_cfg->qspi_cfg, &prime_rx_frame);
    }
    else
    {
        /* Propagate error */
    }

    return result;
}


/**
 * @brief       CR write with SEQ = 0, SHADOW = 0: ends the sequence.
 * @req         LLR-AD7490-053
 * @design      SDD-AD7490-5.7
 * @param[in]   p_cfg  Configuration of a SEQUENCING device.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_SPI,
 *              AD7490_ERR_TIMEOUT.
 * @note        WCET: 1 frame (Table IV, SEQ = 0 / SHADOW = 0 row).
 */
static ad7490_status_t send_stop_frame(ad7490_config_t const * p_cfg)
{
    ad7490_status_t result = AD7490_OK;
    uint16_t cr_frame = 0U;

    ASSERT(NULL != p_cfg);

    result = build_cr_frame(&p_cfg->ctrl_cfg,
                            AD7490_DEFAULT_CHANNEL,
                            AD7490_SEQ_MODE_SINGLE,
                            &cr_frame);

    if (AD7490_OK == result)
    {
        result = write_frame(&p_cfg->qspi_cfg, cr_frame);
    }
    else
    {
        /* Propagate invalid ctrl_cfg (handle corrupted) */
    }

    return result;
}


/**
 * @brief       Run one sequenced frame, check it, store it by channel.
 * @req         LLR-AD7490-054
 * @design      SDD-AD7490-5.8
 * @param[in]     p_cfg         Configuration of a SEQUENCING device.
 * @param[in]     channel_mask  Sequenced channels, bit n = VINn.
 * @param[in]     tx_frame      DIN: CR scrub word (SEQ = 1, SHADOW = 0)
 *                              or AD7490_NOP_DIN_WORD (WRITE = 0).
 * @param[in,out] p_seen_mask   Channels already stored in this pass.
 * @param[out]    p_samples     Array of >= 16 samples, by channel ID.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_SPI,
 *              AD7490_ERR_TIMEOUT, AD7490_ERR_CHAN_MISMATCH.
 * @note        Both DIN words keep the sequence running.
 *              Mismatch = echoed channel not sequenced, or seen twice in
 *              one pass (a frame was lost or the ADC lost sync).
 */
static ad7490_status_t scan_one(ad7490_config_t const * p_cfg,
                                uint16_t channel_mask,
                                uint16_t tx_frame,
                                uint16_t * p_seen_mask,
                                ad7490_sample_t * p_samples)
{
    ad7490_status_t result = AD7490_OK;
    uint16_t rx_frame = 0U;

    ASSERT(NULL != p_cfg);
    ASSERT(NULL != p_seen_mask);
    ASSERT(NULL != p_samples);

    result = txrx_frame(&p_cfg->qspi_cfg, tx_frame, &rx_frame);

    if (AD7490_OK == result)
    {
        ad7490_dout_t dout = decode_dout(rx_frame);
        uint32_t chan_bit = (uint32_t)1U << dout.channel_id;
        uint32_t voltage_mv = 0U;

        if (0U == ((uint32_t)channel_mask & chan_bit))
        {
            result = AD7490_ERR_CHAN_MISMATCH;
        }
        else if (0U != ((uint32_t)*p_seen_mask & chan_bit))
        {
            result = AD7490_ERR_CHAN_MISMATCH;
        }
        else
        {
            result = raw_to_mv(dout.raw_lsb,
                               p_cfg->vref_mv,
                               p_cfg->ctrl_cfg.range_mode,
                               p_cfg->ctrl_cfg.coding,
                               &voltage_mv);
        }

        if (AD7490_OK == result)
        {
            /* channel_id is 4 bits (0..15) and capacity >= 16 */
            ASSERT(dout.channel_id < AD7490_CHANNEL_COUNT);

            p_samples[dout.channel_id].channel_id = dout.channel_id;
            p_samples[dout.channel_id].raw_lsb = dout.raw_lsb;
            p_samples[dout.channel_id].voltage_mv = voltage_mv;
            *p_seen_mask = (uint16_t)((uint32_t)*p_seen_mask | chan_bit);
        }
        else
        {
            /* Propagate mismatch or invalid configuration */
        }
    }
    else
    {
        /* Propagate bus error */
    }

    return result;
}


/* ================================================================== */
/* C. Frame layer                                                      */
/* ================================================================== */

/**
 * @brief       Initialise the QSPI instance that the AD7490 is wired to.
 * @req         LLR-AD7490-038
 * @design      SDD-AD7490-3.16
 * @param[in]   p_qspi_cfg  QSPI instance, validated by the HAL.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_SPI.
 */
static ad7490_status_t init_bus(qspi_hal_config_t const * p_qspi_cfg)
{
    qspi_hal_status_t hal_status = qspi_hal_init(p_qspi_cfg);
    ad7490_status_t result = map_hal_status(hal_status);

    return result;
}


/**
 * @brief       Build the 16-bit DIN word that writes the Control Register.
 * @req         LLR-AD7490-021
 * @design      SDD-AD7490-3.2
 * @param[in]   p_ctrl_cfg  Control Register options (from a handle).
 * @param[in]   channel_id  Value for ADD3..ADD0, 0..15.
 * @param[in]   seq_mode    SEQ / SHADOW combination, 0..3.
 * @param[out]  p_tx_frame  DIN word, CR in bits 15..4.
 * @return      AD7490_OK, AD7490_ERR_PARAM.
 * @note        AD7490_ERR_PARAM only when ctrl_cfg holds a value outside
 *              its enum (bad configuration or RAM corruption); pointers,
 *              channel_id and seq_mode are guaranteed by the callers.
 *              WRITE bit is always 1. *p_tx_frame written only on OK.
 */
static ad7490_status_t build_cr_frame(ad7490_ctrl_cfg_t const * p_ctrl_cfg,
                                      uint8_t channel_id,
                                      ad7490_seq_mode_t seq_mode,
                                      uint16_t * p_tx_frame)
{
    ad7490_status_t result = AD7490_OK;
    bool is_ctrl_ok = is_ctrl_cfg_ok(p_ctrl_cfg);

    ASSERT(NULL != p_tx_frame);
    ASSERT(channel_id < AD7490_CHANNEL_COUNT);
    ASSERT((size_t)seq_mode < AD7490_SEQ_MODE_COUNT);

    if (!is_ctrl_ok)
    {
        result = AD7490_ERR_PARAM;
    }
    else
    {
        uint32_t cr_word = encode_cr_word(p_ctrl_cfg, channel_id, seq_mode);

        /* 12-bit CR rides in DIN bits 15..4; bits 3..0 are don't care */
        *p_tx_frame = (uint16_t)((cr_word << AD7490_CR_FRAME_SHIFT)
                                 & AD7490_FRAME_MASK);
    }

    return result;
}


/**
 * @brief       Send one 16-bit DIN word in one CS frame, ignore DOUT.
 * @req         LLR-AD7490-022
 * @design      SDD-AD7490-3.3
 * @param[in]   p_qspi_cfg  QSPI instance, validated by the HAL.
 * @param[in]   tx_frame    DIN word, sent MSB first.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_SPI,
 *              AD7490_ERR_TIMEOUT.
 */
static ad7490_status_t write_frame(qspi_hal_config_t const * p_qspi_cfg,
                                   uint16_t tx_frame)
{
    frame_bytes_t tx_bytes = frame_to_bytes(tx_frame);
    qspi_hal_status_t hal_status = qspi_hal_write_bytes(p_qspi_cfg,
                                                        tx_bytes.bytes,
                                                        AD7490_FRAME_BYTES);
    ad7490_status_t result = map_hal_status(hal_status);

    return result;
}


/**
 * @brief       Send one DIN word and receive the DOUT word of that frame.
 * @req         LLR-AD7490-023
 * @design      SDD-AD7490-3.4
 * @param[in]   p_qspi_cfg  QSPI instance, validated by the HAL.
 * @param[in]   tx_frame    DIN word, sent MSB first.
 * @param[out]  p_rx_frame  DOUT word, written only on AD7490_OK.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_SPI,
 *              AD7490_ERR_TIMEOUT.
 */
static ad7490_status_t txrx_frame(qspi_hal_config_t const * p_qspi_cfg,
                                  uint16_t tx_frame,
                                  uint16_t * p_rx_frame)
{
    ad7490_status_t result = AD7490_OK;
    frame_bytes_t tx_bytes = frame_to_bytes(tx_frame);
    frame_bytes_t rx_bytes = { .bytes = { 0U, 0U } };
    qspi_hal_status_t hal_status = QSPI_HAL_OK;

    ASSERT(NULL != p_rx_frame);

    hal_status = qspi_hal_transceive_bytes(p_qspi_cfg,
                                           tx_bytes.bytes,
                                           rx_bytes.bytes,
                                           AD7490_FRAME_BYTES);
    result = map_hal_status(hal_status);

    if (AD7490_OK == result)
    {
        *p_rx_frame = bytes_to_frame(rx_bytes);
    }
    else
    {
        /* Output left untouched: caller must not use a failed frame */
    }

    return result;
}


/**
 * @brief       Receive one DOUT word while DIN = 0 (WRITE = 0, no CR write).
 * @req         LLR-AD7490-024
 * @design      SDD-AD7490-3.5
 * @param[in]   p_qspi_cfg  QSPI instance, validated by the HAL.
 * @param[out]  p_rx_frame  DOUT word, written only on AD7490_OK.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_SPI,
 *              AD7490_ERR_TIMEOUT.
 * @note        Keeps a running sequence alive (CR not written).
 */
static ad7490_status_t read_frame(qspi_hal_config_t const * p_qspi_cfg,
                                  uint16_t * p_rx_frame)
{
    ad7490_status_t result = AD7490_OK;
    frame_bytes_t rx_bytes = { .bytes = { 0U, 0U } };
    qspi_hal_status_t hal_status = QSPI_HAL_OK;

    ASSERT(NULL != p_rx_frame);

    /* DIN = 0x00 both bytes: WRITE bit 0, Control Register kept */
    hal_status = qspi_hal_read_bytes(p_qspi_cfg,
                                     (uint8_t)AD7490_NOP_DIN_BYTE,
                                     rx_bytes.bytes,
                                     AD7490_FRAME_BYTES);
    result = map_hal_status(hal_status);

    if (AD7490_OK == result)
    {
        *p_rx_frame = bytes_to_frame(rx_bytes);
    }
    else
    {
        /* Output left untouched: caller must not use a failed frame */
    }

    return result;
}


/**
 * @brief       Split a DOUT word into channel ID and 12-bit code.
 * @req         LLR-AD7490-026
 * @design      SDD-AD7490-3.7
 * @param[in]   rx_frame  DOUT word as received, MSB first.
 * @return      Decoded channel_id (0..15) and raw_lsb (0..4095).
 * @note        Pure function; cannot fail.
 */
static ad7490_dout_t decode_dout(uint16_t rx_frame)
{
    ad7490_dout_t dout = { .channel_id = 0U, .raw_lsb = 0U };
    uint32_t rx_wide = (uint32_t)rx_frame;

    dout.channel_id = (uint8_t)((rx_wide >> AD7490_DOUT_CHAN_SHIFT)
                                & AD7490_DOUT_CHAN_MASK);
    dout.raw_lsb = (uint16_t)(rx_wide & AD7490_DOUT_RAW_MASK);

    return dout;
}


/**
 * @brief       Check every Control Register option against its enum range.
 * @req         LLR-AD7490-020
 * @design      SDD-AD7490-3.1
 * @param[in]   p_ctrl_cfg  Control Register options (from a handle).
 * @return      true if every field is a defined member.
 * @note        One atomic condition per decision (MC/DC, rule §5).
 *              Catches out-of-range values from RAM corruption too.
 */
static bool is_ctrl_cfg_ok(ad7490_ctrl_cfg_t const * p_ctrl_cfg)
{
    bool is_ok = false;

    ASSERT(NULL != p_ctrl_cfg);

    if ((uint32_t)p_ctrl_cfg->power_mode > (uint32_t)AD7490_POWER_NORMAL)
    {
        is_ok = false;
    }
    else if ((uint32_t)p_ctrl_cfg->range_mode
             > (uint32_t)AD7490_RANGE_1X_VREF)
    {
        is_ok = false;
    }
    else if ((uint32_t)p_ctrl_cfg->coding
             > (uint32_t)AD7490_CODING_STRAIGHT_BINARY)
    {
        is_ok = false;
    }
    else if ((uint32_t)p_ctrl_cfg->dout_mode
             > (uint32_t)AD7490_DOUT_WEAK_ADD3)
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
 * @brief       Assemble the 12-bit Control Register word (WRITE = 1).
 * @req         LLR-AD7490-030
 * @design      SDD-AD7490-3.11
 * @param[in]   p_ctrl_cfg  Validated Control Register options.
 * @param[in]   channel_id  Validated channel, 0..15.
 * @param[in]   seq_mode    Validated SEQ / SHADOW mode, 0..3.
 * @return      CR word in bits 11..0.
 * @note        Pure helper; cannot fail. Enum values equal their bit
 *              patterns, so they are shifted directly: no per-option
 *              if/else, fewer MC/DC test cases.
 */
static uint32_t encode_cr_word(ad7490_ctrl_cfg_t const * p_ctrl_cfg,
                               uint8_t channel_id,
                               ad7490_seq_mode_t seq_mode)
{
    uint32_t cr_word = AD7490_CR_WRITE_BIT;
    size_t seq_idx = (size_t)seq_mode;

    ASSERT(NULL != p_ctrl_cfg);
    ASSERT(channel_id < AD7490_CHANNEL_COUNT);

    cr_word |= ((uint32_t)channel_id << AD7490_CR_ADD_SHIFT);
    cr_word |= ((uint32_t)p_ctrl_cfg->power_mode << AD7490_CR_PM_SHIFT);
    cr_word |= ((uint32_t)p_ctrl_cfg->dout_mode << AD7490_CR_WEAK_TRI_SHIFT);
    cr_word |= ((uint32_t)p_ctrl_cfg->range_mode << AD7490_CR_RANGE_SHIFT);
    cr_word |= ((uint32_t)p_ctrl_cfg->coding << AD7490_CR_CODING_SHIFT);

    if (seq_idx < AD7490_SEQ_MODE_COUNT)
    {
        cr_word |= seq_mode_cr_bits[seq_idx];
    }
    else
    {
        /* DEFENSIVE [DO-178C]: RAM bit-flip — unreachable in normal
         * test; defensive log entry DEF-AD7490-001 */
    }

    return cr_word;
}


/**
 * @brief       Serialise a 16-bit frame into 2 bytes, MSB first.
 * @req         LLR-AD7490-028
 * @design      SDD-AD7490-3.9
 * @param[in]   frame_word  16-bit DIN word.
 * @return      bytes[0] = bits 15..8, bytes[1] = bits 7..0.
 * @note        Pure helper; explicit shifts, endian-independent (§8).
 */
static frame_bytes_t frame_to_bytes(uint16_t frame_word)
{
    frame_bytes_t frame_bytes = { .bytes = { 0U, 0U } };
    uint32_t frame_wide = (uint32_t)frame_word;

    frame_bytes.bytes[0U] = (uint8_t)((frame_wide >> AD7490_HIGH_BYTE_SHIFT)
                                      & AD7490_BYTE_MASK);
    frame_bytes.bytes[1U] = (uint8_t)(frame_wide & AD7490_BYTE_MASK);

    return frame_bytes;
}


/**
 * @brief       Deserialise 2 bytes received MSB first into a 16-bit word.
 * @req         LLR-AD7490-029
 * @design      SDD-AD7490-3.10
 * @param[in]   frame_bytes  bytes[0] = bits 15..8, bytes[1] = bits 7..0.
 * @return      16-bit DOUT word.
 * @note        Pure helper; explicit shifts, endian-independent (§8).
 */
static uint16_t bytes_to_frame(frame_bytes_t frame_bytes)
{
    uint32_t high_byte = (uint32_t)frame_bytes.bytes[0U];
    uint32_t low_byte = (uint32_t)frame_bytes.bytes[1U];
    uint16_t frame_word = (uint16_t)(((high_byte << AD7490_HIGH_BYTE_SHIFT)
                                      | low_byte)
                                     & AD7490_FRAME_MASK);

    return frame_word;
}


/**
 * @brief       Translate a QSPI HAL status into an AD7490 status.
 * @req         LLR-AD7490-034
 * @design      SDD-AD7490-3.15
 * @param[in]   hal_status  Status returned by a qspi_hal_* function.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_SPI,
 *              AD7490_ERR_TIMEOUT.
 * @note        Pure helper; cannot fail. Unknown values map to
 *              AD7490_ERR_SPI (fail safe).
 */
static ad7490_status_t map_hal_status(qspi_hal_status_t hal_status)
{
    ad7490_status_t result = AD7490_ERR_SPI;

    switch (hal_status)
    {
        case QSPI_HAL_OK:
            result = AD7490_OK;
            break;

        case QSPI_HAL_ERR_PARAM:
            result = AD7490_ERR_PARAM;
            break;

        case QSPI_HAL_ERR_TIMEOUT:
            result = AD7490_ERR_TIMEOUT;
            break;

        case QSPI_HAL_ERR_HW:
            result = AD7490_ERR_SPI;
            break;

        default:
            /* DEFENSIVE [DO-178C]: bit-flip — unreachable in normal
             * test; defensive log entry DEF-AD7490-002 */
            result = AD7490_ERR_SPI;
            break;
    }

    return result;
}


/* ================================================================== */
/* D. Conversion                                                       */
/* ================================================================== */

/**
 * @brief       Check REFIN against the datasheet-specified range.
 * @req         LLR-AD7490-035
 * @design      SDD-AD7490-4.2
 * @param[in]   vref_mv  REFIN voltage, millivolts.
 * @return      true if AD7490_VREF_MIN_MV <= vref_mv <= AD7490_VREF_MAX_MV.
 * @note        Pure; cannot fail. System requirement outside software:
 *              RANGE = 0 (2 x REFIN) also needs VDD = 4.75..5.25 V.
 */
static bool is_vref_ok(uint32_t vref_mv)
{
    bool is_ok = false;

    if (vref_mv < AD7490_VREF_MIN_MV)
    {
        is_ok = false;
    }
    else if (vref_mv > AD7490_VREF_MAX_MV)
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
 * @brief       Convert a 12-bit AD7490 code into the voltage at VINx.
 * @req         LLR-AD7490-025
 * @design      SDD-AD7490-4.1
 * @param[in]   raw_lsb       12-bit code from DOUT[11:0], 0..4095.
 * @param[in]   vref_mv       REFIN, millivolts (checked here).
 * @param[in]   range_mode    Validated RANGE setting.
 * @param[in]   coding        Validated CODING setting.
 * @param[out]  p_voltage_mv  Voltage at VINx pin, millivolts,
 *                            0..vref_mv (1x) or 0..2 x vref_mv (2x).
 * @return      AD7490_OK, AD7490_ERR_PARAM (vref out of range: the
 *              handle was corrupted after ad7490_init()).
 * @note        Rounded to the nearest millivolt (error <= 0.5 mV).
 *              range_mode / coding were validated by build_cr_frame() in
 *              the same call chain, raw_lsb is masked to 12 bits by
 *              decode_dout(). *p_voltage_mv written only on AD7490_OK.
 */
static ad7490_status_t raw_to_mv(uint16_t raw_lsb,
                                 uint32_t vref_mv,
                                 ad7490_range_t range_mode,
                                 ad7490_coding_t coding,
                                 uint32_t * p_voltage_mv)
{
    ad7490_status_t result = AD7490_OK;
    bool is_vref_in_range = is_vref_ok(vref_mv);

    ASSERT(NULL != p_voltage_mv);
    ASSERT(raw_lsb <= AD7490_RAW_LSB_MAX);
    ASSERT((uint32_t)range_mode <= (uint32_t)AD7490_RANGE_1X_VREF);
    ASSERT((uint32_t)coding <= (uint32_t)AD7490_CODING_STRAIGHT_BINARY);

    if (!is_vref_in_range)
    {
        result = AD7490_ERR_PARAM;
    }
    else
    {
        uint32_t offset_code = raw_to_offset_binary(raw_lsb, coding);
        uint32_t span_mv = span_from_vref_mv(vref_mv, range_mode);
        uint32_t scaled_num = 0U;

        /*
         * voltage_mv = round(offset_code * span_mv / 4096)
         * Scale: 1 LSB = span_mv / 4096. Widening type: uint32_t.
         * Worst case 4095 * 5050 + 2048 = 20682798 < 2^32: no wrap.
         * All terms are >= 0 (pin voltage), so unsigned math is exact
         * and avoids signed overflow UB.
         */
        scaled_num = (offset_code * span_mv) + AD7490_ROUND_HALF_CODES;
        *p_voltage_mv = scaled_num / AD7490_CODES_PER_SPAN;
    }

    return result;
}


/**
 * @brief       Map a code of either coding onto offset binary 0..4095.
 * @req         LLR-AD7490-036
 * @design      SDD-AD7490-4.3
 * @param[in]   raw_lsb  Validated 12-bit code, 0..4095.
 * @param[in]   coding   Validated output coding.
 * @return      Offset-binary code, 0 = 0 V, 4095 = full scale - 1 LSB.
 * @note        Pure helper; cannot fail. Twos complement codes are
 *              centred on mid-scale: 0x800 = 0 V, 0x000 = mid-scale,
 *              0x7FF = full scale. Flipping DB11 maps them to 0x000,
 *              0x800 and 0xFFF, which is exactly straight binary.
 */
static uint32_t raw_to_offset_binary(uint16_t raw_lsb,
                                     ad7490_coding_t coding)
{
    uint32_t offset_code = (uint32_t)raw_lsb;

    if (AD7490_CODING_TWOS_COMPLEMENT == coding)
    {
        offset_code = offset_code ^ AD7490_TWOS_COMP_MSB;
    }
    else
    {
        /* Straight binary is already offset binary */
    }

    return offset_code;
}


/**
 * @brief       Full input span selected by the RANGE bit.
 * @req         LLR-AD7490-037
 * @design      SDD-AD7490-4.4
 * @param[in]   vref_mv     Validated REFIN, millivolts, 2475..2525.
 * @param[in]   range_mode  Validated RANGE setting.
 * @return      Span in millivolts: vref_mv or 2 x vref_mv, 2475..5050.
 * @note        Pure helper; cannot fail. 2 x 2525 = 5050: no wrap.
 */
static uint32_t span_from_vref_mv(uint32_t vref_mv,
                                  ad7490_range_t range_mode)
{
    uint32_t span_mv = vref_mv;

    ASSERT(vref_mv <= AD7490_VREF_MAX_MV);

    if (AD7490_RANGE_2X_VREF == range_mode)
    {
        span_mv = vref_mv * AD7490_SPAN_FACTOR_2X;
    }
    else
    {
        /* RANGE = 1: span equals REFIN */
    }

    return span_mv;
}

/**
 * @file        fake_ad7490.c
 * @brief       Test double for the AXI Quad SPI HAL + AD7490 (see header).
 */

#include "fake_ad7490.h"

#define FAKE_CHANNELS               (16U)
#define FAKE_CHAN_MASK              (0x0FU)
#define FAKE_CODE_MASK              (0x0FFFU)
#define FAKE_TWOS_MSB               (0x0800U)
#define FAKE_DOUT_STUCK_HIGH        (0xFFFFU)
#define FAKE_POWER_ON_CHANNEL       (3U)

/* DIN bit positions of the CR fields (CR bits 11..0 = DIN bits 15..4) */
#define FAKE_DIN_WRITE_BIT          (0x8000U)
#define FAKE_DIN_SEQ_SHIFT          (14U)
#define FAKE_DIN_ADD_SHIFT          (10U)
#define FAKE_DIN_SHADOW_SHIFT       (7U)
#define FAKE_DIN_CODING_SHIFT       (4U)
#define FAKE_DIN_CR_SHIFT           (4U)
#define FAKE_SHADOW_VIN0_BIT        (0x8000U)

/* PM1..PM0 = CR bits 5..4 = DIN bits 9..8 (AD7490 Table III) */
#define FAKE_DIN_PM_SHIFT           (8U)
#define FAKE_PM_MASK                (0x3U)
#define FAKE_PM_NORMAL              (0x3U)
#define FAKE_PM_FULL_SHUTDOWN       (0x2U)

/* DOUT code of a frame clocked while the part is not powered up
 * ("INVALID DATA", Fig 13-15). The channel bits stay correct on
 * purpose: the worst case, which the channel echo check cannot catch. */
#define FAKE_INVALID_CODE           (0x0A5U)

/* Power state of the model at a CS falling edge */
typedef enum
{
    FAKE_POWER_OFF    = 0U, /* Shut down: this frame powers it up      */
    FAKE_POWER_WAKING = 1U, /* t_POWER-UP (1 us) not yet elapsed      */
    FAKE_POWER_ON     = 2U  /* Fully powered: conversion valid        */
} fake_power_t;

typedef struct
{
    fake_mode_t       mode;
    qspi_hal_status_t raw_status;
    qspi_hal_status_t init_status;
    uint16_t          raw_rx_word;
    uint32_t          frame_count;
    uint32_t          fail_frame_no;
    uint32_t          bad_echo_frame_no;
    uint16_t          din_log[FAKE_LOG_FRAMES];
    uint8_t           last_dummy_byte;
    size_t            last_byte_count;
    /* AD7490 model */
    uint16_t          codes[FAKE_CHANNELS];
    uint32_t          conv_ch;
    uint32_t          next_ch;
    bool              is_seq_on;
    bool              is_shadow_pending;
    uint16_t          shadow_word;
    uint32_t          coding_bit;
    uint32_t          cr_word;
    bool              is_dead;
    uint32_t          start_latency;
    uint32_t          delay_left;
    uint32_t          pm_bits;
    fake_power_t      power_state;
    bool              is_cr_written;
    uint32_t          invalid_frame_count;
} fake_state_t;

static fake_state_t g_fake;

static uint32_t next_in_shadow(uint32_t current_ch);
static uint32_t first_in_shadow(void);
static uint32_t next_seq_channel(void);
static void decode_cr_write(uint16_t din);
static fake_power_t next_power_state(fake_power_t start_state);
static uint16_t adc_frame(uint16_t din);
static qspi_hal_status_t run_frame(uint16_t din, uint8_t * p_rx_bytes);


void fake_reset(fake_mode_t mode)
{
    uint32_t i = 0U;

    g_fake.mode = mode;
    g_fake.raw_status = QSPI_HAL_OK;
    g_fake.init_status = QSPI_HAL_OK;
    g_fake.raw_rx_word = 0U;
    g_fake.frame_count = 0U;
    g_fake.fail_frame_no = 0U;
    g_fake.bad_echo_frame_no = 0U;
    g_fake.last_dummy_byte = 0xEEU;
    g_fake.last_byte_count = 0U;

    for (i = 0U; i < FAKE_LOG_FRAMES; i++)
    {
        g_fake.din_log[i] = 0U;
    }

    for (i = 0U; i < FAKE_CHANNELS; i++)
    {
        g_fake.codes[i] = 0U;
    }

    /* Power-on state of the model: no sequence, CODING = 1 */
    g_fake.conv_ch = FAKE_POWER_ON_CHANNEL;
    g_fake.next_ch = FAKE_POWER_ON_CHANNEL;
    g_fake.is_seq_on = false;
    g_fake.is_shadow_pending = false;
    g_fake.shadow_word = 0U;
    g_fake.coding_bit = 1U;
    g_fake.cr_word = 0U;
    g_fake.is_dead = false;
    g_fake.start_latency = 0U;
    g_fake.delay_left = 0U;

    /* Powered up in Normal mode */
    g_fake.pm_bits = FAKE_PM_NORMAL;
    g_fake.power_state = FAKE_POWER_ON;
    g_fake.is_cr_written = false;
    g_fake.invalid_frame_count = 0U;
}


void fake_set_hal_status(qspi_hal_status_t status)
{
    g_fake.raw_status = status;
}


void fake_set_raw_rx(uint16_t rx_word)
{
    g_fake.raw_rx_word = rx_word;
}


void fake_set_init_status(qspi_hal_status_t status)
{
    g_fake.init_status = status;
}


void fake_fail_frame(uint32_t frame_no)
{
    g_fake.fail_frame_no = frame_no;
}


void fake_bad_echo_frame(uint32_t frame_no)
{
    g_fake.bad_echo_frame_no = frame_no;
}


void fake_clear_frame_count(void)
{
    g_fake.frame_count = 0U;
}


uint32_t fake_frame_count(void)
{
    return g_fake.frame_count;
}


uint16_t fake_frame_din(uint32_t frame_no)
{
    uint16_t din = 0U;

    if ((0U < frame_no) && (frame_no <= FAKE_LOG_FRAMES))
    {
        din = g_fake.din_log[frame_no - 1U];
    }
    else
    {
        din = 0U;
    }

    return din;
}


uint8_t fake_last_dummy_byte(void)
{
    return g_fake.last_dummy_byte;
}


size_t fake_last_byte_count(void)
{
    return g_fake.last_byte_count;
}


void fake_adc_set_code(uint32_t channel, uint16_t code)
{
    g_fake.codes[channel & FAKE_CHAN_MASK] = (uint16_t)(code
                                                        & FAKE_CODE_MASK);
}


void fake_adc_set_start_latency(uint32_t extra_frames)
{
    g_fake.start_latency = extra_frames;
}


void fake_adc_set_dead(bool is_dead)
{
    g_fake.is_dead = is_dead;
}


void fake_adc_glitch(void)
{
    /* Brown-out: registers lost, part restarts on channel 0 */
    g_fake.is_seq_on = false;
    g_fake.is_shadow_pending = false;
    g_fake.conv_ch = 0U;
    g_fake.next_ch = 0U;
    g_fake.coding_bit = 0U;
}


void fake_adc_upset_coding(void)
{
    g_fake.coding_bit = g_fake.coding_bit ^ 1U;
}


uint32_t fake_adc_coding_bit(void)
{
    return g_fake.coding_bit;
}


uint32_t fake_adc_cr(void)
{
    return g_fake.cr_word;
}


uint32_t fake_adc_invalid_count(void)
{
    return g_fake.invalid_frame_count;
}


bool fake_adc_is_powered(void)
{
    return (FAKE_POWER_ON == g_fake.power_state);
}


/* ------------------------------------------------------------------ */
/* HAL API implemented by the double                                   */
/* ------------------------------------------------------------------ */

qspi_hal_status_t qspi_hal_init(qspi_hal_config_t const * p_cfg)
{
    (void)p_cfg;

    return g_fake.init_status;
}


qspi_hal_status_t qspi_hal_write_bytes(qspi_hal_config_t const * p_cfg,
                                       uint8_t const * p_tx_bytes,
                                       size_t byte_count)
{
    uint16_t din = (uint16_t)(((uint32_t)p_tx_bytes[0] << 8U)
                              | (uint32_t)p_tx_bytes[1]);

    (void)p_cfg;
    g_fake.last_byte_count = byte_count;

    return run_frame(din, NULL);
}


qspi_hal_status_t qspi_hal_transceive_bytes(qspi_hal_config_t const * p_cfg,
                                            uint8_t const * p_tx_bytes,
                                            uint8_t * p_rx_bytes,
                                            size_t byte_count)
{
    uint16_t din = (uint16_t)(((uint32_t)p_tx_bytes[0] << 8U)
                              | (uint32_t)p_tx_bytes[1]);

    (void)p_cfg;
    g_fake.last_byte_count = byte_count;

    return run_frame(din, p_rx_bytes);
}


qspi_hal_status_t qspi_hal_read_bytes(qspi_hal_config_t const * p_cfg,
                                      uint8_t dummy_tx_byte,
                                      uint8_t * p_rx_bytes,
                                      size_t byte_count)
{
    uint16_t din = (uint16_t)(((uint32_t)dummy_tx_byte << 8U)
                              | (uint32_t)dummy_tx_byte);

    (void)p_cfg;
    g_fake.last_dummy_byte = dummy_tx_byte;
    g_fake.last_byte_count = byte_count;

    return run_frame(din, p_rx_bytes);
}


/* ------------------------------------------------------------------ */
/* Model internals                                                     */
/* ------------------------------------------------------------------ */

static qspi_hal_status_t run_frame(uint16_t din, uint8_t * p_rx_bytes)
{
    qspi_hal_status_t status = QSPI_HAL_OK;
    uint16_t dout = 0U;

    g_fake.frame_count++;

    if (g_fake.frame_count <= FAKE_LOG_FRAMES)
    {
        g_fake.din_log[g_fake.frame_count - 1U] = din;
    }
    else
    {
        /* Log full */
    }

    if (g_fake.frame_count == g_fake.fail_frame_no)
    {
        /* Injected bus timeout: the ADC never sees this frame */
        status = QSPI_HAL_ERR_TIMEOUT;
    }
    else if (FAKE_MODE_RAW == g_fake.mode)
    {
        status = g_fake.raw_status;
        dout = g_fake.raw_rx_word;
    }
    else
    {
        dout = adc_frame(din);
    }

    if (NULL != p_rx_bytes)
    {
        p_rx_bytes[0] = (uint8_t)(dout >> 8U);
        p_rx_bytes[1] = (uint8_t)(dout & 0xFFU);
    }
    else
    {
        /* Write-only frame */
    }

    return status;
}


static uint16_t adc_frame(uint16_t din)
{
    uint32_t code = (uint32_t)g_fake.codes[g_fake.conv_ch];
    uint16_t dout = 0U;
    fake_power_t start_state = g_fake.power_state;

    g_fake.is_cr_written = false;

    /* Not powered at the CS falling edge: the conversion is invalid */
    if (FAKE_POWER_ON != start_state)
    {
        code = FAKE_INVALID_CODE;
        g_fake.invalid_frame_count++;
    }
    else
    {
        /* Powered: valid conversion */
    }

    /* Conversion uses the CR in force before this frame's write */
    if (0U == g_fake.coding_bit)
    {
        code = code ^ FAKE_TWOS_MSB;
    }
    else
    {
        /* Straight binary */
    }

    dout = (uint16_t)((g_fake.conv_ch << 12U) | (code & FAKE_CODE_MASK));

    if (g_fake.is_dead)
    {
        dout = FAKE_DOUT_STUCK_HIGH;
    }
    else if (g_fake.frame_count == g_fake.bad_echo_frame_no)
    {
        dout = (uint16_t)(((g_fake.conv_ch ^ 1U) & FAKE_CHAN_MASK) << 12U);
    }
    else
    {
        /* Normal DOUT */
    }

    if (g_fake.is_shadow_pending)
    {
        g_fake.shadow_word = din;
        g_fake.is_shadow_pending = false;
        g_fake.is_seq_on = true;
        g_fake.delay_left = g_fake.start_latency;

        if (0U == g_fake.delay_left)
        {
            g_fake.next_ch = first_in_shadow();
        }
        else
        {
            g_fake.next_ch = g_fake.conv_ch;
        }
    }
    else if (0U != (din & FAKE_DIN_WRITE_BIT))
    {
        decode_cr_write(din);
    }
    else if (g_fake.is_seq_on)
    {
        g_fake.next_ch = next_seq_channel();
    }
    else
    {
        /* WRITE = 0 outside a sequence: same channel again */
    }

    /* Power state at the CS rising edge that ends this frame */
    g_fake.power_state = next_power_state(start_state);
    g_fake.conv_ch = g_fake.next_ch;

    return dout;
}


/*
 * Power behaviour of AD7490 Figures 13 to 15:
 * - Normal: stays on; leaving shutdown takes t_POWER-UP after CS rising,
 *   so the next frame may still be invalid (WAKING) in the worst case.
 * - Full shutdown: off until PM bits change.
 * - Auto standby / auto shutdown: a CR write enters the low-power state
 *   on the CS rising edge; a powered (valid) conversion does too; a frame
 *   clocked while off is the dummy cycle that powers the part up.
 */
static fake_power_t next_power_state(fake_power_t start_state)
{
    fake_power_t next_state = FAKE_POWER_OFF;

    if (FAKE_PM_NORMAL == g_fake.pm_bits)
    {
        if (FAKE_POWER_OFF == start_state)
        {
            next_state = FAKE_POWER_WAKING;
        }
        else
        {
            next_state = FAKE_POWER_ON;
        }
    }
    else if (FAKE_PM_FULL_SHUTDOWN == g_fake.pm_bits)
    {
        next_state = FAKE_POWER_OFF;
    }
    else if (g_fake.is_cr_written)
    {
        next_state = FAKE_POWER_OFF;
    }
    else if (FAKE_POWER_ON == start_state)
    {
        next_state = FAKE_POWER_OFF;
    }
    else
    {
        next_state = FAKE_POWER_ON;
    }

    return next_state;
}


static void decode_cr_write(uint16_t din)
{
    uint32_t seq_bit = ((uint32_t)din >> FAKE_DIN_SEQ_SHIFT) & 1U;
    uint32_t shadow_bit = ((uint32_t)din >> FAKE_DIN_SHADOW_SHIFT) & 1U;
    uint32_t add = ((uint32_t)din >> FAKE_DIN_ADD_SHIFT) & FAKE_CHAN_MASK;

    g_fake.cr_word = (uint32_t)din >> FAKE_DIN_CR_SHIFT;
    g_fake.coding_bit = ((uint32_t)din >> FAKE_DIN_CODING_SHIFT) & 1U;
    g_fake.pm_bits = ((uint32_t)din >> FAKE_DIN_PM_SHIFT) & FAKE_PM_MASK;
    g_fake.is_cr_written = true;

    if ((0U == seq_bit) && (1U == shadow_bit))
    {
        g_fake.is_shadow_pending = true;
        g_fake.is_seq_on = false;
        g_fake.next_ch = add;
    }
    else if ((1U == seq_bit) && (0U == shadow_bit) && g_fake.is_seq_on)
    {
        /* CR updated, sequence keeps running */
        g_fake.next_ch = next_seq_channel();
    }
    else
    {
        /* Single mode (or unused CONSECUTIVE): ADD selects next */
        g_fake.is_seq_on = false;
        g_fake.next_ch = add;
    }
}


static uint32_t next_seq_channel(void)
{
    uint32_t next_ch = 0U;

    if (0U < g_fake.delay_left)
    {
        g_fake.delay_left--;
        next_ch = first_in_shadow();
    }
    else
    {
        next_ch = next_in_shadow(g_fake.conv_ch);
    }

    return next_ch;
}


static uint32_t next_in_shadow(uint32_t current_ch)
{
    uint32_t next_ch = current_ch;
    bool is_found = false;
    uint32_t k = 0U;

    for (k = 1U; k <= FAKE_CHANNELS; k++)
    {
        uint32_t chan_idx = (current_ch + k) & FAKE_CHAN_MASK;
        uint32_t bit_mask = FAKE_SHADOW_VIN0_BIT >> chan_idx;

        if ((!is_found) && (0U != (g_fake.shadow_word & bit_mask)))
        {
            next_ch = chan_idx;
            is_found = true;
        }
        else
        {
            /* Not selected, or already found */
        }
    }

    return next_ch;
}


static uint32_t first_in_shadow(void)
{
    return next_in_shadow(FAKE_CHANNELS - 1U);
}

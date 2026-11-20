/**
 * @file        ad7490_scan16_raw.c
 * @brief       Example: one scan of all 16 channels per period, with a
 *              16-sample FIFO of raw codes per channel.
 *
 * Application example, not certified driver code: it carries no @req /
 * @design tags and is not part of the traceability set, exactly like
 * the files under test/. It uses only the public driver API, so no
 * file of hal/ or driver/ changes because of it.
 *
 * What it keeps per channel:
 *   - last_raw[channel_id]        the newest raw code of that channel
 *   - window[channel_id]          the last 16 raw codes, oldest first;
 *                                 a new code pushes the oldest one out
 *
 * Raw codes (DOUT[11:0], 0..4095) are stored, not millivolts: the
 * caller monitors and scales them itself. One scan of 16 channels
 * takes 16 frames (~41 us at SCK 6.25 MHz), so calling scan16_step()
 * once per millisecond is comfortable.
 *
 * Usage:
 *   static scan16_state_t g_scan;           (1 kB: never on the stack)
 *   scan16_start(&g_scan, &cfg);            once at start-up
 *   scan16_step(&g_scan);                   once every period
 *   scan16_get_last_raw(&g_scan, raw, 16U);
 *   scan16_get_avg_raw(&g_scan, 5U, &avg_raw, &fill_count);
 *   scan16_get_window(&g_scan, 5U, window, 16U, &fill_count);
 */

#include <stdint.h>
#include <stddef.h>
#include "ad7490_driver.h"

/* Samples kept per channel: 16 scans x 1 ms = 16 ms of history */
#define SCAN16_WINDOW_SIZE          (16U)

/* Every analog input VIN0..VIN15 is scanned: bit n = VINn */
#define SCAN16_ALL_CHANNELS         (0xFFFFU)

/* Turns a sample count into its half, for rounding the mean */
#define SCAN16_ROUND_DIVISOR        (2U)

/** FIFO of the last SCAN16_WINDOW_SIZE raw codes of one channel. */
typedef struct
{
    uint16_t window_raw[SCAN16_WINDOW_SIZE]; /* raw codes, 0..4095     */
    size_t   next_idx;    /* Slot the next push overwrites, 0..15      */
    size_t   fill_count;  /* Codes stored since the last reset, 0..16  */
} scan16_window_t;

/** State of one 16-channel scanner. About 1 kB: use static storage. */
typedef struct
{
    ad7490_config_t cfg;                        /* Copy used to restart */
    ad7490_dev_t    dev;                        /* Device handle        */
    ad7490_sample_t scan[AD7490_CHANNEL_COUNT]; /* Last pass, by channel*/
    uint16_t        last_raw[AD7490_CHANNEL_COUNT]; /* Newest raw code  */
    scan16_window_t window[AD7490_CHANNEL_COUNT];   /* FIFO per channel */
    uint32_t        scan_total;                 /* Good scans, saturates*/
    uint32_t        error_total;                /* Failed scans, sat.   */
} scan16_state_t;

ad7490_status_t scan16_start(scan16_state_t * p_scan,
                             ad7490_config_t const * p_cfg);
ad7490_status_t scan16_step(scan16_state_t * p_scan);
ad7490_status_t scan16_get_last_raw(scan16_state_t const * p_scan,
                                    uint16_t * p_raw_lsb,
                                    size_t raw_count);
ad7490_status_t scan16_get_avg_raw(scan16_state_t const * p_scan,
                                   uint8_t channel_id,
                                   uint16_t * p_avg_raw,
                                   size_t * p_fill_count);
ad7490_status_t scan16_get_window(scan16_state_t const * p_scan,
                                  uint8_t channel_id,
                                  uint16_t * p_window_raw,
                                  size_t window_count,
                                  size_t * p_fill_count);

static void window_reset(scan16_window_t * p_window);
static void window_push(scan16_window_t * p_window, uint16_t raw_lsb);
static uint16_t window_mean_raw(scan16_window_t const * p_window);
static void reset_all_windows(scan16_state_t * p_scan);
static void push_raw_scan(scan16_state_t * p_scan);
static void count_up(uint32_t * p_counter);


/**
 * @brief   Initialise the ADC and start sequencing all 16 channels.
 * @return  AD7490_OK, or the driver status that failed.
 * @note    Empties every FIFO. Call it again to recover after a failed
 *          scan16_step(). Blocking: QSPI init + 6 frames.
 */
ad7490_status_t scan16_start(scan16_state_t * p_scan,
                             ad7490_config_t const * p_cfg)
{
    ad7490_status_t result = AD7490_OK;

    if ((NULL == p_scan) || (NULL == p_cfg))
    {
        result = AD7490_ERR_PARAM;
    }
    else
    {
        size_t i = 0U;

        p_scan->cfg = *p_cfg;
        p_scan->scan_total = 0U;
        p_scan->error_total = 0U;
        reset_all_windows(p_scan);

        for (i = 0U; i < AD7490_CHANNEL_COUNT; i++)
        {
            p_scan->last_raw[i] = 0U;
        }

        result = ad7490_init(&p_scan->dev, &p_scan->cfg);

        if (AD7490_OK == result)
        {
            result = ad7490_sequencer_start(&p_scan->dev,
                                            SCAN16_ALL_CHANNELS);
        }
        else
        {
            /* Propagate the power-up error; dev.state is FAULT */
        }
    }

    return result;
}


/**
 * @brief   One period: scan all 16 channels, push every raw code.
 * @return  AD7490_OK, or the driver status that failed.
 * @note    Call once per period (e.g. every 1 ms) from a task, NOT from
 *          an ISR. Blocking, 16 frames (~41 us at SCK 6.25 MHz). A
 *          failed scan empties every FIFO so a mean never mixes samples
 *          from before and after a fault; call scan16_start() again.
 */
ad7490_status_t scan16_step(scan16_state_t * p_scan)
{
    ad7490_status_t result = AD7490_OK;

    if (NULL == p_scan)
    {
        result = AD7490_ERR_PARAM;
    }
    else
    {
        result = ad7490_sequencer_read_scan(&p_scan->dev,
                                            p_scan->scan,
                                            AD7490_CHANNEL_COUNT);

        if (AD7490_OK == result)
        {
            push_raw_scan(p_scan);
            count_up(&p_scan->scan_total);
        }
        else
        {
            reset_all_windows(p_scan);
            count_up(&p_scan->error_total);
        }
    }

    return result;
}


/**
 * @brief   Copy the newest raw code of every channel, by channel ID.
 * @param   p_raw_lsb   Array of at least AD7490_CHANNEL_COUNT entries;
 *                      p_raw_lsb[n] becomes the newest code of VINn.
 * @return  AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_STATE (no scan yet).
 */
ad7490_status_t scan16_get_last_raw(scan16_state_t const * p_scan,
                                    uint16_t * p_raw_lsb,
                                    size_t raw_count)
{
    ad7490_status_t result = AD7490_OK;

    if ((NULL == p_scan) || (NULL == p_raw_lsb))
    {
        result = AD7490_ERR_PARAM;
    }
    else if (raw_count < AD7490_CHANNEL_COUNT)
    {
        result = AD7490_ERR_PARAM;
    }
    else if (0U == p_scan->window[0].fill_count)
    {
        /* Nothing measured yet: no fake zeroes */
        result = AD7490_ERR_STATE;
    }
    else
    {
        size_t i = 0U;

        for (i = 0U; i < AD7490_CHANNEL_COUNT; i++)
        {
            p_raw_lsb[i] = p_scan->last_raw[i];
        }
    }

    return result;
}


/**
 * @brief   Mean of the raw codes currently in one channel's FIFO.
 * @param   p_avg_raw     Rounded mean, 0..4095.
 * @param   p_fill_count  Codes the mean covers, 1..16.
 * @return  AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_STATE (no code yet).
 * @note    The mean covers only the codes actually stored, so the first
 *          15 scans are not diluted by empty slots.
 */
ad7490_status_t scan16_get_avg_raw(scan16_state_t const * p_scan,
                                   uint8_t channel_id,
                                   uint16_t * p_avg_raw,
                                   size_t * p_fill_count)
{
    ad7490_status_t result = AD7490_OK;

    if ((NULL == p_scan) || (NULL == p_avg_raw)
        || (NULL == p_fill_count))
    {
        result = AD7490_ERR_PARAM;
    }
    else if (channel_id >= AD7490_CHANNEL_COUNT)
    {
        result = AD7490_ERR_PARAM;
    }
    else if (0U == p_scan->window[channel_id].fill_count)
    {
        result = AD7490_ERR_STATE;
    }
    else
    {
        *p_avg_raw = window_mean_raw(&p_scan->window[channel_id]);
        *p_fill_count = p_scan->window[channel_id].fill_count;
    }

    return result;
}


/**
 * @brief   Copy one channel's FIFO, oldest code first.
 * @param   p_window_raw  Array of at least SCAN16_WINDOW_SIZE entries.
 * @param   p_fill_count  How many entries were written, 1..16.
 * @return  AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_STATE (no code yet).
 * @note    Entries beyond *p_fill_count are left untouched.
 */
ad7490_status_t scan16_get_window(scan16_state_t const * p_scan,
                                  uint8_t channel_id,
                                  uint16_t * p_window_raw,
                                  size_t window_count,
                                  size_t * p_fill_count)
{
    ad7490_status_t result = AD7490_OK;

    if ((NULL == p_scan) || (NULL == p_window_raw)
        || (NULL == p_fill_count))
    {
        result = AD7490_ERR_PARAM;
    }
    else if ((channel_id >= AD7490_CHANNEL_COUNT)
             || (window_count < SCAN16_WINDOW_SIZE))
    {
        result = AD7490_ERR_PARAM;
    }
    else if (0U == p_scan->window[channel_id].fill_count)
    {
        result = AD7490_ERR_STATE;
    }
    else
    {
        scan16_window_t const * p_window = &p_scan->window[channel_id];
        size_t oldest_idx = 0U;
        size_t i = 0U;

        /* While filling, slot 0 is the oldest; once full, the oldest
         * code sits in the slot the next push will overwrite. */
        if (p_window->fill_count >= SCAN16_WINDOW_SIZE)
        {
            oldest_idx = p_window->next_idx;
        }
        else
        {
            oldest_idx = 0U;
        }

        for (i = 0U; i < p_window->fill_count; i++)
        {
            size_t slot = (oldest_idx + i) % SCAN16_WINDOW_SIZE;

            p_window_raw[i] = p_window->window_raw[slot];
        }

        *p_fill_count = p_window->fill_count;
    }

    return result;
}


/* ----------------------------- private ------------------------------ */

static void window_reset(scan16_window_t * p_window)
{
    size_t i = 0U;

    for (i = 0U; i < SCAN16_WINDOW_SIZE; i++)
    {
        p_window->window_raw[i] = 0U;
    }

    p_window->next_idx = 0U;
    p_window->fill_count = 0U;
}


/* Store one code; when the FIFO is full the oldest one drops out */
static void window_push(scan16_window_t * p_window, uint16_t raw_lsb)
{
    p_window->window_raw[p_window->next_idx] = raw_lsb;
    p_window->next_idx = (p_window->next_idx + 1U) % SCAN16_WINDOW_SIZE;

    if (p_window->fill_count < SCAN16_WINDOW_SIZE)
    {
        p_window->fill_count++;
    }
    else
    {
        /* Full: the oldest code has just been overwritten */
    }
}


/* Rounded mean of the codes stored. Sum <= 16 x 4095 = 65520 */
static uint16_t window_mean_raw(scan16_window_t const * p_window)
{
    uint32_t sum_raw = 0U;
    uint32_t sample_total = (uint32_t)p_window->fill_count;
    uint32_t half_step = sample_total / SCAN16_ROUND_DIVISOR;
    size_t i = 0U;

    for (i = 0U; i < p_window->fill_count; i++)
    {
        sum_raw += (uint32_t)p_window->window_raw[i];
    }

    return (uint16_t)((sum_raw + half_step) / sample_total);
}


static void reset_all_windows(scan16_state_t * p_scan)
{
    size_t i = 0U;

    for (i = 0U; i < AD7490_CHANNEL_COUNT; i++)
    {
        window_reset(&p_scan->window[i]);
    }
}


/* The driver stores every sample at scan[channel_id] and has already
 * checked each echoed channel, so scan[i] belongs to channel i. */
static void push_raw_scan(scan16_state_t * p_scan)
{
    size_t i = 0U;

    for (i = 0U; i < AD7490_CHANNEL_COUNT; i++)
    {
        uint16_t raw_lsb = p_scan->scan[i].raw_lsb;

        p_scan->last_raw[i] = raw_lsb;
        window_push(&p_scan->window[i], raw_lsb);
    }
}


/* Saturate: a wrapped counter would read as "no errors so far" */
static void count_up(uint32_t * p_counter)
{
    if (*p_counter < UINT32_MAX)
    {
        (*p_counter)++;
    }
    else
    {
        /* Saturated */
    }
}

/**
 * @file        ad7490_monitor.c
 * @brief       Periodic AD7490 acquisition with per-channel moving
 *              average and bounded re-configuration.
 * @req         LLR-AD7490-001
 * @design      SDD-AD7490-7.0
 *
 * Sections (top to bottom, after the public functions):
 *   A. Recovery and scan helpers
 *   B. Moving-average window (16-sample FIFO)
 */

#include "ad7490_monitor.h"

/* Saturation value of reconfig_total (diagnostic counter) */
#define AD7490_MON_COUNTER_MAX      (0xFFFFFFFFU)

/* Divisor for half-step rounding in integer average calculation */
#define AD7490_MON_ROUND_DIVISOR    (2U)

/* A. Recovery and scan helpers */
static ad7490_status_t reconfig_attempt(ad7490_monitor_t * p_mon);
static ad7490_status_t restart_adc(ad7490_monitor_t * p_mon);
static ad7490_status_t scan_and_push(ad7490_monitor_t * p_mon);
static ad7490_status_t push_scan(ad7490_monitor_t * p_mon);
static void reset_all_avg(ad7490_monitor_t * p_mon);

/* B. Moving-average window */
static void avg_reset(ad7490_avg_t * p_avg);
static ad7490_status_t avg_push(ad7490_avg_t * p_avg,
                                uint32_t sample_mv);
static ad7490_status_t avg_get_mv(ad7490_avg_t const * p_avg,
                                  uint32_t * p_avg_mv,
                                  size_t * p_fill_count);
static bool is_window_consistent(ad7490_avg_t const * p_avg);


/* ================================================================== */
/* Public functions                                                    */
/* ================================================================== */

/**
 * @brief       Configure the monitor, initialise the ADC, start sequencing.
 * @req         LLR-AD7490-070
 * @design      SDD-AD7490-7.1
 */
ad7490_status_t ad7490_monitor_init(ad7490_monitor_t * p_mon,
                                    ad7490_config_t const * p_cfg,
                                    uint16_t channel_mask)
{
    ad7490_status_t result = AD7490_OK;

    if ((NULL == p_mon) || (NULL == p_cfg) || (0U == channel_mask))
    {
        result = AD7490_ERR_PARAM;
    }
    else
    {
        p_mon->cfg = *p_cfg;
        p_mon->channel_mask = channel_mask;
        p_mon->health = AD7490_MON_HEALTH_ACTIVE;
        p_mon->reconfig_attempts = 0U;
        p_mon->reconfig_total = 0U;
        p_mon->good_scan_count = 0U;

        result = restart_adc(p_mon);

        if (AD7490_ERR_PARAM == result)
        {
            /* Configuration rejected: retrying can never succeed */
            p_mon->channel_mask = 0U;
        }
        else if (AD7490_ERR_STATE == result)
        {
            /* e.g. power_mode not NORMAL: sequencing impossible */
            p_mon->channel_mask = 0U;
        }
        else
        {
            /* OK, or bus error that ad7490_monitor_step() recovers */
        }
    }

    return result;
}


/**
 * @brief       One acquisition period: scan, update averages, or recover.
 * @req         LLR-AD7490-071
 * @design      SDD-AD7490-7.2
 */
ad7490_status_t ad7490_monitor_step(ad7490_monitor_t * p_mon)
{
    ad7490_status_t result = AD7490_OK;

    if (NULL == p_mon)
    {
        result = AD7490_ERR_PARAM;
    }
    else if (0U == p_mon->channel_mask)
    {
        /* Not configured, or disabled by a bad configuration */
        result = AD7490_ERR_STATE;
    }
    else if (AD7490_MON_HEALTH_ACTIVE != p_mon->health)
    {
        /* FAILED latch (or corrupted health word: fail safe) */
        result = AD7490_ERR_DEVICE_FAILED;
    }
    else if ((AD7490_STATE_SEQUENCING != p_mon->dev.state)
             || (p_mon->channel_mask != p_mon->dev.channel_mask))
    {
        /* Previous cycle failed (or corrupted handle): one attempt */
        result = reconfig_attempt(p_mon);
    }
    else
    {
        result = scan_and_push(p_mon);
    }

    return result;
}


/**
 * @brief       One verified re-configuration attempt of the ADC.
 * @req         LLR-AD7490-078
 * @design      SDD-AD7490-7.9
 */
ad7490_status_t ad7490_monitor_reconfig(ad7490_monitor_t * p_mon)
{
    ad7490_status_t result = AD7490_OK;

    if (NULL == p_mon)
    {
        result = AD7490_ERR_PARAM;
    }
    else if (0U == p_mon->channel_mask)
    {
        result = AD7490_ERR_STATE;
    }
    else if (AD7490_MON_HEALTH_ACTIVE != p_mon->health)
    {
        result = AD7490_ERR_DEVICE_FAILED;
    }
    else
    {
        result = reconfig_attempt(p_mon);
    }

    return result;
}


/**
 * @brief       Moving average of one monitored channel.
 * @req         LLR-AD7490-072
 * @design      SDD-AD7490-7.3
 */
ad7490_status_t ad7490_monitor_get_avg(ad7490_monitor_t const * p_mon,
                                       uint8_t channel_id,
                                       uint32_t * p_avg_mv,
                                       size_t * p_fill_count)
{
    ad7490_status_t result = AD7490_OK;

    if ((NULL == p_mon) || (NULL == p_avg_mv) || (NULL == p_fill_count))
    {
        result = AD7490_ERR_PARAM;
    }
    else if (channel_id >= AD7490_CHANNEL_COUNT)
    {
        result = AD7490_ERR_PARAM;
    }
    else if (0U == ((uint32_t)p_mon->channel_mask
                    & ((uint32_t)1U << channel_id)))
    {
        /* Shift is safe: channel_id < 16 checked above */
        result = AD7490_ERR_PARAM;
    }
    else if (AD7490_MON_HEALTH_ACTIVE != p_mon->health)
    {
        result = AD7490_ERR_DEVICE_FAILED;
    }
    else
    {
        result = avg_get_mv(&p_mon->avg[channel_id],
                            p_avg_mv,
                            p_fill_count);
    }

    return result;
}


/**
 * @brief       Read the health flag and re-configuration counters.
 * @req         LLR-AD7490-079
 * @design      SDD-AD7490-7.10
 */
ad7490_status_t ad7490_monitor_get_diag(ad7490_monitor_t const * p_mon,
                                        ad7490_mon_diag_t * p_diag)
{
    ad7490_status_t result = AD7490_OK;

    if ((NULL == p_mon) || (NULL == p_diag))
    {
        result = AD7490_ERR_PARAM;
    }
    else
    {
        if (AD7490_MON_HEALTH_ACTIVE == p_mon->health)
        {
            p_diag->is_failed = false;
        }
        else
        {
            p_diag->is_failed = true;
        }

        p_diag->reconfig_attempts = p_mon->reconfig_attempts;
        p_diag->reconfig_total = p_mon->reconfig_total;
    }

    return result;
}


/* ================================================================== */
/* A. Recovery and scan helpers                                        */
/* ================================================================== */

/**
 * @brief       Count, run and verify one re-configuration attempt.
 * @req         LLR-AD7490-080
 * @design      SDD-AD7490-7.11
 * @param[in,out] p_mon  ACTIVE, configured monitor.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_STATE,
 *              AD7490_ERR_SPI, AD7490_ERR_TIMEOUT,
 *              AD7490_ERR_CHAN_MISMATCH, AD7490_ERR_DEVICE_FAILED.
 * @note        Latches FAILED (a) before a 4th attempt in one episode,
 *              or (b) when the 3rd attempt itself fails. The attempt
 *              counter is cleared only by AD7490_MON_STABLE_SCANS good
 *              scans in a row (scan_and_push), so a fault that keeps
 *              coming back cannot loop forever.
 */
static ad7490_status_t reconfig_attempt(ad7490_monitor_t * p_mon)
{
    ad7490_status_t result = AD7490_OK;

    ASSERT(NULL != p_mon);

    if (p_mon->reconfig_attempts >= AD7490_MON_MAX_RECONFIG)
    {
        /* Budget of this episode already spent: give up */
        reset_all_avg(p_mon);
        p_mon->health = AD7490_MON_HEALTH_FAILED;
        result = AD7490_ERR_DEVICE_FAILED;
    }
    else
    {
        p_mon->reconfig_attempts++;

        if (p_mon->reconfig_total < AD7490_MON_COUNTER_MAX)
        {
            p_mon->reconfig_total++;
        }
        else
        {
            /* Saturated diagnostic counter: keep the maximum */
        }

        result = restart_adc(p_mon);

        if (AD7490_OK == result)
        {
            /* Verification: a full scan with correct channel echoes */
            result = scan_and_push(p_mon);
        }
        else
        {
            /* Propagate restart error */
        }

        if ((AD7490_OK != result)
            && (p_mon->reconfig_attempts >= AD7490_MON_MAX_RECONFIG))
        {
            /* Last allowed attempt failed: declare the ADC failed */
            reset_all_avg(p_mon);
            p_mon->health = AD7490_MON_HEALTH_FAILED;
            result = AD7490_ERR_DEVICE_FAILED;
        }
        else
        {
            /* Success, or failure with attempts left (next cycle) */
        }
    }

    return result;
}


/**
 * @brief       Empty all windows, re-initialise the ADC, start sequencing.
 * @req         LLR-AD7490-073
 * @design      SDD-AD7490-7.4
 * @param[in,out] p_mon  Configured monitor.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_STATE,
 *              AD7490_ERR_SPI, AD7490_ERR_TIMEOUT.
 * @note        Resets the QSPI IP too (inside ad7490_init), so a glitch
 *              in the FPGA side is cleared as well. WCET: QSPI init +
 *              3 init frames + 3 sequencer frames.
 */
static ad7490_status_t restart_adc(ad7490_monitor_t * p_mon)
{
    ad7490_status_t result = AD7490_OK;

    ASSERT(NULL != p_mon);

    reset_all_avg(p_mon);
    p_mon->good_scan_count = 0U;

    result = ad7490_init(&p_mon->dev, &p_mon->cfg);

    if (AD7490_OK == result)
    {
        result = ad7490_sequencer_start(&p_mon->dev, p_mon->channel_mask);
    }
    else
    {
        /* Device stays FAULT; next cycle makes the next attempt */
    }

    return result;
}


/**
 * @brief       Scan all channels, push them, track the stable period.
 * @req         LLR-AD7490-081
 * @design      SDD-AD7490-7.12
 * @param[in,out] p_mon  Monitor with a SEQUENCING device.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_STATE,
 *              AD7490_ERR_SPI, AD7490_ERR_TIMEOUT,
 *              AD7490_ERR_CHAN_MISMATCH.
 * @note        Error: every window is emptied (never average across a
 *              fault) and the stable-period count restarts. After
 *              AD7490_MON_STABLE_SCANS good scans in a row the fault
 *              episode is closed: reconfig_attempts returns to 0.
 *              WCET: 16 frames + 16 pushes.
 */
static ad7490_status_t scan_and_push(ad7490_monitor_t * p_mon)
{
    ad7490_status_t result = AD7490_OK;

    ASSERT(NULL != p_mon);

    result = ad7490_sequencer_read_scan(&p_mon->dev,
                                        p_mon->scan,
                                        AD7490_CHANNEL_COUNT);

    if (AD7490_OK == result)
    {
        result = push_scan(p_mon);
    }
    else
    {
        /* Propagate scan error; device is now FAULT */
    }

    if (AD7490_OK != result)
    {
        reset_all_avg(p_mon);
        p_mon->good_scan_count = 0U;
    }
    else if (p_mon->good_scan_count < AD7490_MON_STABLE_SCANS)
    {
        p_mon->good_scan_count++;

        if (AD7490_MON_STABLE_SCANS == p_mon->good_scan_count)
        {
            /* Stable for one full window: close the fault episode */
            p_mon->reconfig_attempts = 0U;
        }
        else
        {
            /* Still inside the stable-period check */
        }
    }
    else
    {
        /* Already stable: count saturated, attempts already 0 */
    }

    return result;
}


/**
 * @brief       Push the last scan of every monitored channel into its window.
 * @req         LLR-AD7490-074
 * @design      SDD-AD7490-7.5
 * @param[in,out] p_mon  Monitor with a complete scan in p_mon->scan.
 * @return      AD7490_OK, AD7490_ERR_STATE (window corrupted).
 * @note        WCET: 16 pushes. Stops at the first failing push.
 */
static ad7490_status_t push_scan(ad7490_monitor_t * p_mon)
{
    ad7490_status_t result = AD7490_OK;
    uint32_t chan_idx = 0U;

    ASSERT(NULL != p_mon);

    for (chan_idx = 0U; chan_idx < AD7490_CHANNEL_COUNT; chan_idx++)
    {
        bool is_monitored = (0U != ((uint32_t)p_mon->channel_mask
                                    & ((uint32_t)1U << chan_idx)));

        if (!is_monitored)
        {
            /* Channel not monitored: its window stays empty */
        }
        else if (AD7490_OK != result)
        {
            /* Earlier push failed: skip the remaining channels */
        }
        else
        {
            result = avg_push(&p_mon->avg[chan_idx],
                              p_mon->scan[chan_idx].voltage_mv);
        }
    }

    return result;
}


/**
 * @brief       Empty the window of every channel.
 * @req         LLR-AD7490-075
 * @design      SDD-AD7490-7.6
 * @param[in,out] p_mon  Monitor.
 * @note        Cannot fail. WCET: 16 x 16 stores.
 */
static void reset_all_avg(ad7490_monitor_t * p_mon)
{
    uint32_t chan_idx = 0U;

    ASSERT(NULL != p_mon);

    for (chan_idx = 0U; chan_idx < AD7490_CHANNEL_COUNT; chan_idx++)
    {
        avg_reset(&p_mon->avg[chan_idx]);
    }
}


/* ================================================================== */
/* B. Moving-average window (16-sample FIFO)                           */
/* ================================================================== */

/**
 * @brief       Empty the window (all slots 0, no sample stored).
 * @req         LLR-AD7490-060
 * @design      SDD-AD7490-6.1
 * @param[out]  p_avg  Window to reset.
 * @note        Cannot fail. WCET: 16 stores.
 */
static void avg_reset(ad7490_avg_t * p_avg)
{
    size_t i = 0U;

    ASSERT(NULL != p_avg);

    for (i = 0U; i < AD7490_AVG_WINDOW_SIZE; i++)
    {
        p_avg->window_mv[i] = 0U;
    }

    p_avg->next_idx = 0U;
    p_avg->fill_count = 0U;
}


/**
 * @brief       Store a new sample, dropping the oldest one when full.
 * @req         LLR-AD7490-061
 * @design      SDD-AD7490-6.2
 * @param[in,out] p_avg      Window.
 * @param[in]     sample_mv  New sample from the driver, mV, 0..5050.
 * @return      AD7490_OK, AD7490_ERR_STATE (index fields or active slot
 *              corrupted: window unchanged, caller resets it).
 * @note        WCET: 16 iterations. Ring buffer: the slot after the
 *              newest sample always holds the oldest one.
 */
static ad7490_status_t avg_push(ad7490_avg_t * p_avg,
                                uint32_t sample_mv)
{
    ad7490_status_t result = AD7490_OK;

    ASSERT(NULL != p_avg);
    ASSERT(sample_mv <= AD7490_VOLTAGE_MAX_MV);

    if (!is_window_consistent(p_avg))
    {
        result = AD7490_ERR_STATE;
    }
    else
    {
        /* next_idx < AD7490_AVG_WINDOW_SIZE: checked just above */
        p_avg->window_mv[p_avg->next_idx] = sample_mv;
        p_avg->next_idx = (p_avg->next_idx + 1U) % AD7490_AVG_WINDOW_SIZE;

        if (p_avg->fill_count < AD7490_AVG_WINDOW_SIZE)
        {
            p_avg->fill_count++;
        }
        else
        {
            /* Window full: the oldest sample was just overwritten */
        }
    }

    return result;
}


/**
 * @brief       Rounded mean of the samples currently in the window.
 * @req         LLR-AD7490-062
 * @design      SDD-AD7490-6.3
 * @param[in]   p_avg         Window.
 * @param[out]  p_avg_mv      Mean, millivolts, 0..5050.
 * @param[out]  p_fill_count  Samples in the mean, 1..16.
 * @return      AD7490_OK, AD7490_ERR_STATE (no sample yet - no fake
 *              0 mV is returned - or window corrupted).
 * @note        WCET: 16 iterations. Outputs written only on AD7490_OK.
 */
static ad7490_status_t avg_get_mv(ad7490_avg_t const * p_avg,
                                  uint32_t * p_avg_mv,
                                  size_t * p_fill_count)
{
    ad7490_status_t result = AD7490_OK;

    ASSERT(NULL != p_avg);
    ASSERT(NULL != p_avg_mv);
    ASSERT(NULL != p_fill_count);

    if (!is_window_consistent(p_avg))
    {
        result = AD7490_ERR_STATE;
    }
    else if (0U == p_avg->fill_count)
    {
        /* Nothing measured yet: report "no data", not 0 mV */
        result = AD7490_ERR_STATE;
    }
    else
    {
        /*
         * Slots 0..fill_count-1 are valid and <= 5050 mV (verified by
         * is_window_consistent): sum <= 16 * 5050 = 80800 < 2^32.
         */
        uint32_t sum_mv = 0U;
        uint32_t sample_total = (uint32_t)p_avg->fill_count;
        uint32_t half_step = sample_total / AD7490_MON_ROUND_DIVISOR;
        size_t i = 0U;

        for (i = 0U; i < p_avg->fill_count; i++)
        {
            sum_mv += p_avg->window_mv[i];
        }

        /* Round to nearest; sample_total is 1..16, never 0 here */
        *p_avg_mv = (sum_mv + half_step) / sample_total;
        *p_fill_count = p_avg->fill_count;
    }

    return result;
}


/**
 * @brief       Check the index fields and active slots of a window.
 * @req         LLR-AD7490-063
 * @design      SDD-AD7490-6.4
 * @param[in]   p_avg  Window.
 * @return      true if indices and active slots satisfy all invariants.
 * @note        Invariants: next_idx < 16, fill_count <= 16, while not
 *              full next_idx == fill_count, and every active slot
 *              window_mv[0..fill_count-1] <= AD7490_VOLTAGE_MAX_MV.
 *              One condition per decision. WCET: 16 iterations.
 */
static bool is_window_consistent(ad7490_avg_t const * p_avg)
{
    bool is_ok = false;

    ASSERT(NULL != p_avg);

    if (p_avg->next_idx >= AD7490_AVG_WINDOW_SIZE)
    {
        is_ok = false;
    }
    else if (p_avg->fill_count > AD7490_AVG_WINDOW_SIZE)
    {
        is_ok = false;
    }
    else if (AD7490_AVG_WINDOW_SIZE == p_avg->fill_count)
    {
        /* Full window: any next_idx in range is valid */
        is_ok = true;
    }
    else if (p_avg->next_idx != p_avg->fill_count)
    {
        is_ok = false;
    }
    else
    {
        is_ok = true;
    }

    if (is_ok)
    {
        size_t i = 0U;

        for (i = 0U; i < p_avg->fill_count; i++)
        {
            if (p_avg->window_mv[i] > AD7490_VOLTAGE_MAX_MV)
            {
                /* Corrupted slot (SEU): reject before sum or push */
                is_ok = false;
            }
            else
            {
                /* Active slot within valid voltage range */
            }
        }
    }
    else
    {
        /* Index invariant already failed */
    }

    return is_ok;
}

/**
 * @file        test_monitor.c
 * @brief       Unit tests of ad7490_monitor.c: the static 16-sample
 *              window (section B), the 1 ms acquisition and the bounded
 *              re-configuration (max 3 attempts) against the fake AD7490.
 *
 * The monitor source is included so its static window helpers can be
 * called directly; the driver is linked as a normal object.
 */

#include "test_common.h"
#include "ad7490_monitor.c"
#include "fake_ad7490.h"

#define VIN0_CODE                   (1000U)
#define VIN1_CODE                   (3000U)
#define RAMP_CYCLES                 (40U)

static ad7490_monitor_t g_mon;

static ad7490_config_t make_cfg(void);
static void arm_monitor(uint32_t latency);
static void run_good_steps(uint32_t step_count);
static uint32_t code_to_mv(uint16_t code);
static ad7490_mon_diag_t read_diag(void);
static void test_window(void);
static void test_window_corruption(void);
static void test_ramp_average(void);
static void test_parameters(void);
static void test_bad_config_disables(void);
static void test_init_bus_error_retried(void);
static void test_scan_fault_and_recovery(void);
static void test_coding_upset_scrubbed(void);
static void test_dead_adc_latches_failed(void);
static void test_intermittent_fault(void);
static void test_rare_fault(void);
static void test_app_requested_reconfig(void);
static void test_timeouts_inside_attempts(void);
static void test_corrupted_state(void);


int main(void)
{
    test_window();
    test_window_corruption();
    test_ramp_average();
    test_parameters();
    test_bad_config_disables();
    test_init_bus_error_retried();
    test_scan_fault_and_recovery();
    test_coding_upset_scrubbed();
    test_dead_adc_latches_failed();
    test_intermittent_fault();
    test_rare_fault();
    test_app_requested_reconfig();
    test_timeouts_inside_attempts();
    test_corrupted_state();

    return test_summary("test_monitor");
}


static ad7490_config_t make_cfg(void)
{
    ad7490_config_t cfg = {
        .qspi_cfg = {
            .base_addr = 0x80000000U,
            .clock_mode = AD7490_QSPI_CLOCK_MODE,
            .poll_timeout_us = 10U
        },
        .vref_mv = 2500U,
        .ctrl_cfg = {
            .power_mode = AD7490_POWER_NORMAL,
            .range_mode = AD7490_RANGE_1X_VREF,
            .coding = AD7490_CODING_STRAIGHT_BINARY,
            .dout_mode = AD7490_DOUT_TRI_STATE
        }
    };

    return cfg;
}


/* Fresh model, VIN0 = current sense, VIN1 = voltage, monitor armed */
static void arm_monitor(uint32_t latency)
{
    ad7490_config_t cfg = make_cfg();

    fake_reset(FAKE_MODE_ADC);
    fake_adc_set_start_latency(latency);
    fake_adc_set_code(0U, VIN0_CODE);
    fake_adc_set_code(1U, VIN1_CODE);
    CHECK(AD7490_OK == ad7490_monitor_init(&g_mon, &cfg, 0x0003U));
}


static void run_good_steps(uint32_t step_count)
{
    uint32_t k = 0U;

    for (k = 0U; k < step_count; k++)
    {
        CHECK(AD7490_OK == ad7490_monitor_step(&g_mon));
    }
}


static uint32_t code_to_mv(uint16_t code)
{
    return (((uint32_t)code * 2500U) + 2048U) / 4096U;
}


static ad7490_mon_diag_t read_diag(void)
{
    ad7490_mon_diag_t diag = { false, 0U, 0U };

    CHECK(AD7490_OK == ad7490_monitor_get_diag(&g_mon, &diag));

    return diag;
}


static void test_window(void)
{
    ad7490_avg_t window;
    uint32_t avg_mv = 0U;
    size_t fill_count = 0U;
    uint32_t k = 0U;

    avg_reset(&window);
    CHECK(AD7490_ERR_STATE == avg_get_mv(&window, &avg_mv, &fill_count));

    /* First sample: the mean is that sample, not diluted by zeros */
    CHECK(AD7490_OK == avg_push(&window, 2500U));
    CHECK(AD7490_OK == avg_get_mv(&window, &avg_mv, &fill_count));
    CHECK(2500U == avg_mv);
    CHECK(1U == fill_count);

    /* 2500.5 rounds to 2501 */
    CHECK(AD7490_OK == avg_push(&window, 2501U));
    CHECK(AD7490_OK == avg_get_mv(&window, &avg_mv, &fill_count));
    CHECK(2501U == avg_mv);
    CHECK(2U == fill_count);

    /* FIFO: push 1..20, window keeps 5..20, mean 12.5 -> 13 */
    avg_reset(&window);
    for (k = 1U; k <= 20U; k++)
    {
        CHECK(AD7490_OK == avg_push(&window, k));
    }
    CHECK(AD7490_OK == avg_get_mv(&window, &avg_mv, &fill_count));
    CHECK(13U == avg_mv);
    CHECK(AD7490_AVG_WINDOW_SIZE == fill_count);
}


static void test_window_corruption(void)
{
    ad7490_avg_t window;
    uint32_t avg_mv = 7U;
    size_t fill_count = 0U;

    avg_reset(&window);
    window.next_idx = AD7490_AVG_WINDOW_SIZE;
    CHECK(AD7490_ERR_STATE == avg_push(&window, 1U));
    CHECK(AD7490_ERR_STATE == avg_get_mv(&window, &avg_mv, &fill_count));

    avg_reset(&window);
    (void)avg_push(&window, 1U);
    window.fill_count = AD7490_AVG_WINDOW_SIZE + 1U;
    CHECK(AD7490_ERR_STATE == avg_push(&window, 1U));

    avg_reset(&window);
    (void)avg_push(&window, 1U);
    window.next_idx = 3U;
    CHECK(AD7490_ERR_STATE == avg_get_mv(&window, &avg_mv, &fill_count));

    /* Corrupted slot: error on push and read, outputs untouched */
    avg_reset(&window);
    (void)avg_push(&window, 1U);
    window.window_mv[0] = 0xFFFFFFF0U;
    CHECK(AD7490_ERR_STATE == avg_push(&window, 1U));
    CHECK(AD7490_ERR_STATE == avg_get_mv(&window, &avg_mv, &fill_count));
    CHECK(7U == avg_mv);
}


static void test_ramp_average(void)
{
    uint32_t hist_vin0[RAMP_CYCLES];
    uint32_t hist_vin1[RAMP_CYCLES];
    uint32_t step_idx = 0U;

    arm_monitor(1U);

    for (step_idx = 0U; step_idx < RAMP_CYCLES; step_idx++)
    {
        uint16_t code0 = (uint16_t)(1000U + (37U * step_idx));
        uint16_t code1 = (uint16_t)(2000U + (100U * (step_idx & 1U)));
        uint32_t width = step_idx + 1U;
        uint32_t sum0 = 0U;
        uint32_t sum1 = 0U;
        uint32_t avg_mv = 0U;
        size_t fill_count = 0U;
        uint32_t i = 0U;

        fake_adc_set_code(0U, code0);
        fake_adc_set_code(1U, code1);
        hist_vin0[step_idx] = code_to_mv(code0);
        hist_vin1[step_idx] = code_to_mv(code1);
        CHECK(AD7490_OK == ad7490_monitor_step(&g_mon));

        if (width > AD7490_AVG_WINDOW_SIZE)
        {
            width = AD7490_AVG_WINDOW_SIZE;
        }
        else
        {
            /* Window not full yet */
        }

        for (i = (step_idx + 1U) - width; i <= step_idx; i++)
        {
            sum0 += hist_vin0[i];
            sum1 += hist_vin1[i];
        }

        CHECK(AD7490_OK == ad7490_monitor_get_avg(&g_mon, 0U, &avg_mv,
                                                  &fill_count));
        CHECK(width == fill_count);
        CHECK(((sum0 + (width / 2U)) / width) == avg_mv);
        CHECK(AD7490_OK == ad7490_monitor_get_avg(&g_mon, 1U, &avg_mv,
                                                  &fill_count));
        CHECK(((sum1 + (width / 2U)) / width) == avg_mv);
    }
}


static void test_parameters(void)
{
    ad7490_config_t cfg = make_cfg();
    ad7490_mon_diag_t diag = { false, 0U, 0U };
    uint32_t avg_mv = 0U;
    size_t fill_count = 0U;

    arm_monitor(0U);
    run_good_steps(1U);

    CHECK(AD7490_ERR_PARAM == ad7490_monitor_init(NULL, &cfg, 1U));
    CHECK(AD7490_ERR_PARAM == ad7490_monitor_init(&g_mon, NULL, 1U));
    CHECK(AD7490_ERR_PARAM == ad7490_monitor_step(NULL));
    CHECK(AD7490_ERR_PARAM == ad7490_monitor_reconfig(NULL));
    CHECK(AD7490_ERR_PARAM == ad7490_monitor_get_avg(NULL, 0U, &avg_mv,
                                                     &fill_count));
    CHECK(AD7490_ERR_PARAM == ad7490_monitor_get_avg(&g_mon, 0U, NULL,
                                                     &fill_count));
    CHECK(AD7490_ERR_PARAM == ad7490_monitor_get_avg(&g_mon, 0U, &avg_mv,
                                                     NULL));
    CHECK(AD7490_ERR_PARAM == ad7490_monitor_get_avg(&g_mon, 16U, &avg_mv,
                                                     &fill_count));
    CHECK(AD7490_ERR_PARAM == ad7490_monitor_get_avg(&g_mon, 2U, &avg_mv,
                                                     &fill_count));
    CHECK(AD7490_ERR_PARAM == ad7490_monitor_get_diag(NULL, &diag));
    CHECK(AD7490_ERR_PARAM == ad7490_monitor_get_diag(&g_mon, NULL));

    /* channel_mask 0 = not configured */
    CHECK(AD7490_ERR_PARAM == ad7490_monitor_init(&g_mon, &cfg, 0U));
    g_mon.channel_mask = 0U;
    CHECK(AD7490_ERR_STATE == ad7490_monitor_step(&g_mon));
    CHECK(AD7490_ERR_STATE == ad7490_monitor_reconfig(&g_mon));
}


static void test_bad_config_disables(void)
{
    ad7490_config_t cfg = make_cfg();

    fake_reset(FAKE_MODE_ADC);
    cfg.vref_mv = 3000U;
    CHECK(AD7490_ERR_PARAM == ad7490_monitor_init(&g_mon, &cfg, 0x0003U));
    CHECK(AD7490_ERR_STATE == ad7490_monitor_step(&g_mon));

    cfg = make_cfg();
    cfg.ctrl_cfg.power_mode = AD7490_POWER_AUTO_SHUTDOWN;
    CHECK(AD7490_ERR_STATE == ad7490_monitor_init(&g_mon, &cfg, 0x0003U));
    CHECK(AD7490_ERR_STATE == ad7490_monitor_step(&g_mon));
}


static void test_init_bus_error_retried(void)
{
    ad7490_config_t cfg = make_cfg();
    uint32_t avg_mv = 0U;
    size_t fill_count = 0U;

    fake_reset(FAKE_MODE_ADC);
    fake_fail_frame(2U);
    CHECK(AD7490_ERR_TIMEOUT == ad7490_monitor_init(&g_mon, &cfg, 0x0003U));
    fake_fail_frame(0U);

    /* Still configured: step re-configures, verify scan stores 1 sample */
    run_good_steps(2U);
    CHECK(AD7490_OK == ad7490_monitor_get_avg(&g_mon, 1U, &avg_mv,
                                              &fill_count));
    CHECK(2U == fill_count);
}


static void test_scan_fault_and_recovery(void)
{
    uint32_t avg_mv = 0U;
    size_t fill_count = 0U;

    /* Brown-out glitch: registers lost -> channel mismatch */
    arm_monitor(0U);
    run_good_steps(20U);
    fake_adc_glitch();
    CHECK(AD7490_ERR_CHAN_MISMATCH == ad7490_monitor_step(&g_mon));

    /* Windows emptied: no stale average */
    CHECK(AD7490_ERR_STATE == ad7490_monitor_get_avg(&g_mon, 0U, &avg_mv,
                                                     &fill_count));

    /* Attempt 1: re-config + verified scan, one fresh sample */
    CHECK(AD7490_OK == ad7490_monitor_step(&g_mon));
    CHECK(AD7490_OK == ad7490_monitor_get_avg(&g_mon, 1U, &avg_mv,
                                              &fill_count));
    CHECK(1U == fill_count);
    CHECK(code_to_mv(VIN1_CODE) == avg_mv);
    CHECK(1U == read_diag().reconfig_attempts);

    /* Episode closed after 16 good scans in a row (1 + 15) */
    run_good_steps(14U);
    CHECK(1U == read_diag().reconfig_attempts);
    run_good_steps(1U);
    CHECK(0U == read_diag().reconfig_attempts);
    CHECK(1U == read_diag().reconfig_total);
    run_good_steps(2U);
    CHECK(0U == read_diag().reconfig_attempts);
}


static void test_coding_upset_scrubbed(void)
{
    arm_monitor(0U);
    run_good_steps(20U);

    /* Silent CR upset: frames valid, scrub repairs it, no re-config */
    fake_adc_upset_coding();
    run_good_steps(1U);
    CHECK(1U == fake_adc_coding_bit());
    run_good_steps(1U);
    CHECK(VIN0_CODE == g_mon.scan[0].raw_lsb);
    CHECK(VIN1_CODE == g_mon.scan[1].raw_lsb);
    CHECK(0U == read_diag().reconfig_total);
}


static void test_dead_adc_latches_failed(void)
{
    ad7490_config_t cfg = make_cfg();
    ad7490_mon_diag_t diag = { false, 0U, 0U };
    uint32_t avg_mv = 0U;
    size_t fill_count = 0U;

    arm_monitor(0U);
    run_good_steps(5U);
    fake_adc_set_dead(true);

    CHECK(AD7490_ERR_CHAN_MISMATCH == ad7490_monitor_step(&g_mon));
    CHECK(AD7490_ERR_CHAN_MISMATCH == ad7490_monitor_step(&g_mon));
    CHECK(AD7490_ERR_CHAN_MISMATCH == ad7490_monitor_step(&g_mon));
    CHECK(AD7490_ERR_DEVICE_FAILED == ad7490_monitor_step(&g_mon));

    /* Latched: no more bus traffic, data refused */
    fake_clear_frame_count();
    CHECK(AD7490_ERR_DEVICE_FAILED == ad7490_monitor_step(&g_mon));
    CHECK(0U == fake_frame_count());
    CHECK(AD7490_ERR_DEVICE_FAILED == ad7490_monitor_get_avg(
                                          &g_mon, 0U, &avg_mv, &fill_count));
    CHECK(AD7490_ERR_DEVICE_FAILED == ad7490_monitor_reconfig(&g_mon));
    diag = read_diag();
    CHECK(diag.is_failed);
    CHECK(3U == diag.reconfig_attempts);
    CHECK(3U == diag.reconfig_total);

    /* Explicit system reset re-arms the monitor */
    fake_adc_set_dead(false);
    CHECK(AD7490_OK == ad7490_monitor_init(&g_mon, &cfg, 0x0003U));
    run_good_steps(1U);
    diag = read_diag();
    CHECK(!diag.is_failed);
    CHECK(0U == diag.reconfig_total);
}


static void test_intermittent_fault(void)
{
    ad7490_status_t status = AD7490_OK;
    uint32_t episode = 0U;

    /* Fault every few ms: never 16 good scans, attempts accumulate */
    arm_monitor(0U);
    run_good_steps(20U);

    for (episode = 0U; episode < 4U; episode++)
    {
        fake_adc_glitch();
        CHECK(AD7490_ERR_CHAN_MISMATCH == ad7490_monitor_step(&g_mon));
        status = ad7490_monitor_step(&g_mon);

        if (episode < 3U)
        {
            CHECK(AD7490_OK == status);
            run_good_steps(3U);
        }
        else
        {
            /* 4th episode would need a 4th attempt */
            CHECK(AD7490_ERR_DEVICE_FAILED == status);
        }
    }

    CHECK(read_diag().is_failed);
    CHECK(3U == read_diag().reconfig_total);
}


static void test_rare_fault(void)
{
    ad7490_mon_diag_t diag = { false, 0U, 0U };
    uint32_t episode = 0U;

    /* Fault every 30 ms: always recovered, never FAILED */
    arm_monitor(0U);

    for (episode = 0U; episode < 10U; episode++)
    {
        run_good_steps(28U);
        fake_adc_glitch();
        CHECK(AD7490_ERR_CHAN_MISMATCH == ad7490_monitor_step(&g_mon));
        CHECK(AD7490_OK == ad7490_monitor_step(&g_mon));
    }

    diag = read_diag();
    CHECK(!diag.is_failed);
    CHECK(1U == diag.reconfig_attempts);
    CHECK(10U == diag.reconfig_total);
}


static void test_app_requested_reconfig(void)
{
    uint32_t avg_mv = 0U;
    size_t fill_count = 0U;

    arm_monitor(0U);
    run_good_steps(20U);

    CHECK(AD7490_OK == ad7490_monitor_reconfig(&g_mon));
    CHECK(AD7490_OK == ad7490_monitor_get_avg(&g_mon, 0U, &avg_mv,
                                              &fill_count));
    CHECK(1U == fill_count);
    CHECK(1U == read_diag().reconfig_attempts);
}


static void test_timeouts_inside_attempts(void)
{
    arm_monitor(0U);
    run_good_steps(3U);
    fake_adc_glitch();
    CHECK(AD7490_ERR_CHAN_MISMATCH == ad7490_monitor_step(&g_mon));

    /* Attempt 1 fails in the init frames */
    fake_clear_frame_count();
    fake_fail_frame(1U);
    CHECK(AD7490_ERR_TIMEOUT == ad7490_monitor_step(&g_mon));

    /* Attempt 2 fails in the verification scan (frame 7) */
    fake_clear_frame_count();
    fake_fail_frame(7U);
    CHECK(AD7490_ERR_TIMEOUT == ad7490_monitor_step(&g_mon));

    /* Attempt 3 fails in the sequencer start: FAILED latched */
    fake_clear_frame_count();
    fake_fail_frame(4U);
    CHECK(AD7490_ERR_DEVICE_FAILED == ad7490_monitor_step(&g_mon));
    fake_fail_frame(0U);
}


static void test_corrupted_state(void)
{
    uint32_t avg_mv = 0U;
    size_t fill_count = 0U;

    /* Device mask corrupted: treated as a fault, re-configured */
    arm_monitor(0U);
    run_good_steps(2U);
    g_mon.dev.channel_mask = 0x0001U;
    CHECK(AD7490_OK == ad7490_monitor_step(&g_mon));
    CHECK(0x0003U == g_mon.dev.channel_mask);

    /* Window index or slot corrupted: push fails, every window resets */
    g_mon.avg[0].next_idx = 99U;
    CHECK(AD7490_ERR_STATE == ad7490_monitor_step(&g_mon));
    CHECK(AD7490_ERR_STATE == ad7490_monitor_get_avg(&g_mon, 1U, &avg_mv,
                                                     &fill_count));
    CHECK(AD7490_OK == ad7490_monitor_step(&g_mon));
    CHECK(AD7490_OK == ad7490_monitor_get_avg(&g_mon, 0U, &avg_mv,
                                              &fill_count));
    CHECK(1U == fill_count);
    g_mon.avg[0].window_mv[0] = 0xFFFFFFF0U;
    CHECK(AD7490_ERR_STATE == ad7490_monitor_step(&g_mon));
    CHECK(AD7490_OK == ad7490_monitor_step(&g_mon));

    /* Health word corrupted: fail safe */
    g_mon.health ^= 0x00010000U;
    CHECK(AD7490_ERR_DEVICE_FAILED == ad7490_monitor_step(&g_mon));
    CHECK(read_diag().is_failed);

    /* Saturating diagnostic counter */
    arm_monitor(0U);
    g_mon.reconfig_total = 0xFFFFFFFFU;
    CHECK(AD7490_OK == ad7490_monitor_reconfig(&g_mon));
    CHECK(0xFFFFFFFFU == g_mon.reconfig_total);
}

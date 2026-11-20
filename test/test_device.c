/**
 * @file        test_device.c
 * @brief       Unit tests of ad7490_init, ad7490_read_channel and
 *              ad7490_set_power_mode against the fake AD7490 model.
 */

#include <string.h>
#include "test_common.h"
#include "ad7490_driver.h"
#include "fake_ad7490.h"

static ad7490_config_t make_cfg(void);
static void reset_model(void);
static void test_uninitialised_handle(void);
static void test_init_rejects_bad_config(void);
static void test_init_sequence(void);
static void test_init_bus_errors(void);
static void test_read_channel(void);
static void test_read_errors(void);
static void test_power_modes(void);
static void check_auto_mode_reads(ad7490_power_mode_t power_mode,
                                  uint16_t cr_din_ch2);
static void test_auto_mode_reads(void);
static void test_power_mode_errors(void);
static void test_corrupted_handle(void);


int main(void)
{
    test_uninitialised_handle();
    test_init_rejects_bad_config();
    test_init_sequence();
    test_init_bus_errors();
    test_read_channel();
    test_read_errors();
    test_power_modes();
    test_auto_mode_reads();
    test_power_mode_errors();
    test_corrupted_handle();

    return test_summary("test_device");
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
            .range_mode = AD7490_RANGE_2X_VREF,
            .coding = AD7490_CODING_STRAIGHT_BINARY,
            .dout_mode = AD7490_DOUT_TRI_STATE
        }
    };

    return cfg;
}


/* Channel n returns code n * 256 + 0x10, e.g. VIN5 -> 0x510 */
static void reset_model(void)
{
    uint32_t chan_idx = 0U;

    fake_reset(FAKE_MODE_ADC);

    for (chan_idx = 0U; chan_idx < AD7490_CHANNEL_COUNT; chan_idx++)
    {
        fake_adc_set_code(chan_idx,
                          (uint16_t)((chan_idx * 256U) + 0x10U));
    }
}


static void test_uninitialised_handle(void)
{
    ad7490_dev_t dev;
    ad7490_sample_t sample = { 0U, 0U, 0U };

    (void)memset(&dev, 0, sizeof(dev));
    reset_model();

    CHECK(AD7490_ERR_STATE == ad7490_read_channel(&dev, 0U, &sample));
    CHECK(AD7490_ERR_STATE == ad7490_set_power_mode(&dev,
                                                    AD7490_POWER_NORMAL));
}


static void test_init_rejects_bad_config(void)
{
    ad7490_dev_t dev;
    ad7490_config_t cfg = make_cfg();
    ad7490_config_t bad = cfg;

    reset_model();

    CHECK(AD7490_ERR_PARAM == ad7490_init(NULL, &cfg));
    CHECK(AD7490_ERR_PARAM == ad7490_init(&dev, NULL));
    CHECK(AD7490_STATE_FAULT == dev.state);

    bad.vref_mv = 3300U;
    CHECK(AD7490_ERR_PARAM == ad7490_init(&dev, &bad));
    bad = cfg;
    bad.qspi_cfg.clock_mode = QSPI_CLOCK_MODE_3;
    CHECK(AD7490_ERR_PARAM == ad7490_init(&dev, &bad));
    bad = cfg;
    bad.qspi_cfg.poll_timeout_us = AD7490_MIN_TIMEOUT_US - 1U;
    CHECK(AD7490_ERR_PARAM == ad7490_init(&dev, &bad));
    bad = cfg;
    bad.ctrl_cfg.coding = (ad7490_coding_t)7U;
    CHECK(AD7490_ERR_PARAM == ad7490_init(&dev, &bad));

    /* Rejected before any frame reaches the bus */
    CHECK(0U == fake_frame_count());
}


static void test_init_sequence(void)
{
    ad7490_dev_t dev;
    ad7490_config_t cfg = make_cfg();

    reset_model();

    /* 0xFFFF, 0xFFFF, then CR (VIN0, normal, 2x, SB) = 0x831 */
    CHECK(AD7490_OK == ad7490_init(&dev, &cfg));
    CHECK(3U == fake_frame_count());
    CHECK(0xFFFFU == fake_frame_din(1U));
    CHECK(0xFFFFU == fake_frame_din(2U));
    CHECK(0x8310U == fake_frame_din(3U));
    CHECK(0x831U == fake_adc_cr());
    CHECK(AD7490_STATE_READY == dev.state);
}


static void test_init_bus_errors(void)
{
    ad7490_dev_t dev;
    ad7490_config_t cfg = make_cfg();

    reset_model();
    fake_set_init_status(QSPI_HAL_ERR_HW);
    CHECK(AD7490_ERR_SPI == ad7490_init(&dev, &cfg));
    CHECK(0U == fake_frame_count());
    CHECK(AD7490_STATE_FAULT == dev.state);

    /* Timeout on the 2nd dummy frame: no further frame, FAULT */
    reset_model();
    fake_fail_frame(2U);
    CHECK(AD7490_ERR_TIMEOUT == ad7490_init(&dev, &cfg));
    CHECK(2U == fake_frame_count());
    CHECK(AD7490_STATE_FAULT == dev.state);

    /* Timeout on the CR frame */
    reset_model();
    fake_fail_frame(3U);
    CHECK(AD7490_ERR_TIMEOUT == ad7490_init(&dev, &cfg));
    CHECK(AD7490_STATE_FAULT == dev.state);
}


static void test_read_channel(void)
{
    ad7490_dev_t dev;
    ad7490_config_t cfg = make_cfg();
    ad7490_sample_t sample = { 0U, 0U, 0U };

    reset_model();
    CHECK(AD7490_OK == ad7490_init(&dev, &cfg));
    fake_clear_frame_count();

    /* 2 frames, CR 0x971 twice; 0x510 = 1296 -> 1296*5000/4096 = 1582 */
    CHECK(AD7490_OK == ad7490_read_channel(&dev, 5U, &sample));
    CHECK(2U == fake_frame_count());
    CHECK(0x9710U == fake_frame_din(1U));
    CHECK(0x9710U == fake_frame_din(2U));
    CHECK(5U == sample.channel_id);
    CHECK(0x510U == sample.raw_lsb);
    CHECK(1582U == sample.voltage_mv);

    /* Back-to-back different channel: 1-frame pipeline handled */
    CHECK(AD7490_OK == ad7490_read_channel(&dev, 12U, &sample));
    CHECK(12U == sample.channel_id);
    CHECK(0xC10U == sample.raw_lsb);
}


static void test_read_errors(void)
{
    ad7490_dev_t dev;
    ad7490_config_t cfg = make_cfg();
    ad7490_sample_t sample = { 99U, 9999U, 9999U };

    reset_model();
    CHECK(AD7490_OK == ad7490_init(&dev, &cfg));

    CHECK(AD7490_ERR_PARAM == ad7490_read_channel(NULL, 0U, &sample));
    CHECK(AD7490_ERR_PARAM == ad7490_read_channel(&dev, 0U, NULL));
    CHECK(AD7490_ERR_PARAM == ad7490_read_channel(&dev, 16U, &sample));

    fake_clear_frame_count();
    fake_bad_echo_frame(2U);
    CHECK(AD7490_ERR_CHAN_MISMATCH == ad7490_read_channel(&dev, 3U,
                                                          &sample));
    fake_bad_echo_frame(0U);

    fake_clear_frame_count();
    fake_fail_frame(1U);
    CHECK(AD7490_ERR_TIMEOUT == ad7490_read_channel(&dev, 3U, &sample));
    CHECK(1U == fake_frame_count());

    fake_clear_frame_count();
    fake_fail_frame(2U);
    CHECK(AD7490_ERR_TIMEOUT == ad7490_read_channel(&dev, 3U, &sample));
    fake_fail_frame(0U);

    /* Output untouched by every failure; read errors keep READY */
    CHECK(99U == sample.channel_id);
    CHECK(9999U == sample.raw_lsb);
    CHECK(9999U == sample.voltage_mv);
    CHECK(AD7490_STATE_READY == dev.state);
}


static void test_power_modes(void)
{
    ad7490_dev_t dev;
    ad7490_config_t cfg = make_cfg();
    ad7490_sample_t sample = { 0U, 0U, 0U };

    reset_model();
    CHECK(AD7490_OK == ad7490_init(&dev, &cfg));

    /* Full shutdown: 1 frame, PM = 10 -> CR 0x821; reads refused */
    fake_clear_frame_count();
    CHECK(AD7490_OK == ad7490_set_power_mode(&dev,
                                             AD7490_POWER_FULL_SHUTDOWN));
    CHECK(1U == fake_frame_count());
    CHECK(0x8210U == fake_frame_din(1U));
    CHECK(AD7490_ERR_STATE == ad7490_read_channel(&dev, 0U, &sample));

    /* Wake to normal: CR write + power-up dummy frame with DIN 0x00 */
    fake_clear_frame_count();
    CHECK(AD7490_OK == ad7490_set_power_mode(&dev, AD7490_POWER_NORMAL));
    CHECK(2U == fake_frame_count());
    CHECK(0x8310U == fake_frame_din(1U));
    CHECK(0x00U == fake_last_dummy_byte());
    CHECK(AD7490_OK == ad7490_read_channel(&dev, 1U, &sample));
    CHECK(1U == sample.channel_id);
}


/*
 * AUTO modes (AD7490 Fig 14, 15): a CR write puts the part to sleep on
 * the CS rising edge, the next frame is a dummy that returns INVALID
 * data, the frame after it is valid. The model returns the right channel
 * bits with a wrong code on invalid frames, so the value is checked.
 * cr_din_ch2: expected DIN of the CR write selecting VIN2 in this mode.
 */
static void check_auto_mode_reads(ad7490_power_mode_t power_mode,
                                  uint16_t cr_din_ch2)
{
    ad7490_dev_t dev;
    ad7490_config_t cfg = make_cfg();
    ad7490_sample_t sample = { 99U, 9999U, 9999U };

    reset_model();
    CHECK(AD7490_OK == ad7490_init(&dev, &cfg));
    CHECK(AD7490_OK == ad7490_set_power_mode(&dev, power_mode));

    /* 3 frames: CR write (select), dummy WRITE=0, valid WRITE=0 */
    fake_clear_frame_count();
    CHECK(AD7490_OK == ad7490_read_channel(&dev, 2U, &sample));
    CHECK(3U == fake_frame_count());
    CHECK(cr_din_ch2 == fake_frame_din(1U));
    CHECK(0x0000U == fake_frame_din(2U));
    CHECK(0x0000U == fake_frame_din(3U));
    CHECK(2U == sample.channel_id);
    CHECK(0x210U == sample.raw_lsb);
    CHECK(645U == sample.voltage_mv);   /* 528 * 5000 / 4096 = 644.5 */

    /* Back-to-back: part asleep after the valid conversion */
    CHECK(!fake_adc_is_powered());
    CHECK(AD7490_OK == ad7490_read_channel(&dev, 7U, &sample));
    CHECK(7U == sample.channel_id);
    CHECK(0x710U == sample.raw_lsb);

    /* The model really produced invalid frames, none was returned */
    CHECK(0U < fake_adc_invalid_count());

    /* Bus error on the dummy frame or on the result frame */
    sample.raw_lsb = 9999U;
    fake_clear_frame_count();
    fake_fail_frame(2U);
    CHECK(AD7490_ERR_TIMEOUT == ad7490_read_channel(&dev, 3U, &sample));
    CHECK(2U == fake_frame_count());
    fake_clear_frame_count();
    fake_fail_frame(3U);
    CHECK(AD7490_ERR_TIMEOUT == ad7490_read_channel(&dev, 3U, &sample));
    fake_fail_frame(0U);
    CHECK(9999U == sample.raw_lsb);
    CHECK(AD7490_STATE_READY == dev.state);
}


static void test_auto_mode_reads(void)
{
    /* CR = WRITE | ADD=2 | PM | CODING=1: standby 0x881, shutdown 0x891 */
    check_auto_mode_reads(AD7490_POWER_AUTO_STANDBY, 0x8810U);
    check_auto_mode_reads(AD7490_POWER_AUTO_SHUTDOWN, 0x8910U);
}


static void test_power_mode_errors(void)
{
    ad7490_dev_t dev;
    ad7490_config_t cfg = make_cfg();
    ad7490_sample_t sample = { 0U, 0U, 0U };

    reset_model();
    CHECK(AD7490_OK == ad7490_init(&dev, &cfg));

    CHECK(AD7490_ERR_PARAM == ad7490_set_power_mode(
                                  &dev, (ad7490_power_mode_t)4U));
    CHECK(AD7490_ERR_PARAM == ad7490_set_power_mode(NULL,
                                                    AD7490_POWER_NORMAL));

    /* Dummy frame fails: PM state unknown -> FAULT, init recovers */
    fake_clear_frame_count();
    fake_fail_frame(2U);
    CHECK(AD7490_ERR_TIMEOUT == ad7490_set_power_mode(&dev,
                                                      AD7490_POWER_NORMAL));
    fake_fail_frame(0U);
    CHECK(AD7490_STATE_FAULT == dev.state);
    CHECK(AD7490_ERR_STATE == ad7490_read_channel(&dev, 0U, &sample));
    CHECK(AD7490_OK == ad7490_init(&dev, &cfg));
    CHECK(AD7490_STATE_READY == dev.state);
}


static void test_corrupted_handle(void)
{
    ad7490_dev_t dev;
    ad7490_config_t cfg = make_cfg();
    ad7490_sample_t sample = { 0U, 0U, 0U };

    reset_model();
    CHECK(AD7490_OK == ad7490_init(&dev, &cfg));

    /* One bit of the state pattern flipped: refused */
    dev.state ^= 0x00000100U;
    CHECK(AD7490_ERR_STATE == ad7490_read_channel(&dev, 0U, &sample));
    dev.state ^= 0x00000100U;

    /* Out-of-range enum inside a READY handle: PARAM, no frame */
    dev.cfg.ctrl_cfg.range_mode = (ad7490_range_t)5U;
    fake_clear_frame_count();
    CHECK(AD7490_ERR_PARAM == ad7490_read_channel(&dev, 0U, &sample));
    CHECK(0U == fake_frame_count());
    CHECK(AD7490_ERR_PARAM == ad7490_set_power_mode(&dev,
                                                    AD7490_POWER_NORMAL));
    CHECK(AD7490_STATE_FAULT == dev.state);

    /* Corrupted REFIN inside a READY handle: PARAM from conversion */
    reset_model();
    CHECK(AD7490_OK == ad7490_init(&dev, &cfg));
    dev.cfg.vref_mv = 1U;
    CHECK(AD7490_ERR_PARAM == ad7490_read_channel(&dev, 4U, &sample));
}

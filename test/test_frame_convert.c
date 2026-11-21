/**
 * @file        test_frame_convert.c
 * @brief       Unit tests of the static frame layer (section C) and code
 *              conversion (section D) of ad7490_driver.c.
 *
 * The driver source is included so its static helpers can be called
 * directly (test-build technique; the flight build compiles it alone).
 * HAL calls go to fake_ad7490 in RAW mode.
 */

#include <math.h>
#include "test_common.h"
#include "ad7490_driver.c"
#include "fake_ad7490.h"

#define VREF_TEST_MV                (2500U)
#define REF_TOLERANCE_MV            (0.5 + 1.0e-9)

static void test_cr_encoding(void);
static void test_cr_rejects_bad_enum(void);
static void test_frames(void);
static void test_hal_status_map(void);
static void test_conversion_table(void);
static void test_conversion_exhaustive(void);
static void test_vref_limits(void);
static uint32_t code_to_mv(uint16_t raw_lsb,
                           uint32_t vref_mv,
                           ad7490_range_t range_mode,
                           ad7490_coding_t coding);
static double reference_mv(uint32_t code,
                           uint32_t vref_mv,
                           ad7490_range_t range_mode,
                           ad7490_coding_t coding);


int main(void)
{
    test_cr_encoding();
    test_cr_rejects_bad_enum();
    test_frames();
    test_hal_status_map();
    test_conversion_table();
    test_conversion_exhaustive();
    test_vref_limits();

    return test_summary("test_frame_convert");
}


static void test_cr_encoding(void)
{
    ad7490_ctrl_cfg_t ctrl = {
        .power_mode = AD7490_POWER_NORMAL,
        .range_mode = AD7490_RANGE_2X_VREF,
        .coding = AD7490_CODING_STRAIGHT_BINARY,
        .dout_mode = AD7490_DOUT_TRI_STATE
    };
    uint16_t frame = 0U;

    /* WRITE | ADD=5 | PM=11 | CODING=1 -> CR 0x971 -> DIN 0x9710 */
    CHECK(AD7490_OK == build_cr_frame(&ctrl, 5U,
                                      AD7490_SEQ_MODE_SINGLE, &frame));
    CHECK(0x9710U == frame);

    /* LOAD_SHADOW, 1x, twos complement, weak: CR 0x83E */
    ctrl.range_mode = AD7490_RANGE_1X_VREF;
    ctrl.coding = AD7490_CODING_TWOS_COMPLEMENT;
    ctrl.dout_mode = AD7490_DOUT_WEAK_ADD3;
    CHECK(AD7490_OK == build_cr_frame(&ctrl, 0U,
                                      AD7490_SEQ_MODE_LOAD_SHADOW, &frame));
    CHECK(0x83E0U == frame);

    /* SINGLE on VIN15, auto standby: CR 0xBC0 -> DIN 0xBC00 */
    ctrl.power_mode = AD7490_POWER_AUTO_STANDBY;
    ctrl.range_mode = AD7490_RANGE_2X_VREF;
    ctrl.dout_mode = AD7490_DOUT_TRI_STATE;
    CHECK(AD7490_OK == build_cr_frame(&ctrl, 15U,
                                      AD7490_SEQ_MODE_SINGLE, &frame));
    CHECK(0xBC00U == frame);

    /* KEEP_SEQ: SEQ bit only */
    CHECK(AD7490_OK == build_cr_frame(&ctrl, 0U,
                                      AD7490_SEQ_MODE_KEEP_SEQ, &frame));
    CHECK(0xC000U == frame);
}


static void test_cr_rejects_bad_enum(void)
{
    ad7490_ctrl_cfg_t good = {
        .power_mode = AD7490_POWER_NORMAL,
        .range_mode = AD7490_RANGE_2X_VREF,
        .coding = AD7490_CODING_TWOS_COMPLEMENT,
        .dout_mode = AD7490_DOUT_TRI_STATE
    };
    ad7490_ctrl_cfg_t bad = good;
    uint16_t frame = 0x1234U;

    /* Values a corrupted handle could hold: PARAM, output untouched */
    bad.power_mode = (ad7490_power_mode_t)4U;
    CHECK(AD7490_ERR_PARAM == build_cr_frame(&bad, 0U,
                                             AD7490_SEQ_MODE_SINGLE, &frame));
    bad = good;
    bad.range_mode = (ad7490_range_t)2U;
    CHECK(AD7490_ERR_PARAM == build_cr_frame(&bad, 0U,
                                             AD7490_SEQ_MODE_SINGLE, &frame));
    bad = good;
    bad.coding = (ad7490_coding_t)2U;
    CHECK(AD7490_ERR_PARAM == build_cr_frame(&bad, 0U,
                                             AD7490_SEQ_MODE_SINGLE, &frame));
    bad = good;
    bad.dout_mode = (ad7490_dout_mode_t)2U;
    CHECK(AD7490_ERR_PARAM == build_cr_frame(&bad, 0U,
                                             AD7490_SEQ_MODE_SINGLE, &frame));
    CHECK(0x1234U == frame);

    /* Defensive branch in encode_cr_word when seq_mode >= 3 */
    CHECK(0x830U == encode_cr_word(&good,
                                   0U,
                                   (ad7490_seq_mode_t)AD7490_SEQ_MODE_COUNT));
}


static void test_frames(void)
{
    qspi_hal_config_t qspi = {
        .base_addr = 0x1000U,
        .clock_mode = QSPI_CLOCK_MODE_2,
        .poll_timeout_us = 100U
    };
    uint16_t rx_frame = 0U;
    ad7490_dout_t dout = { .channel_id = 0U, .raw_lsb = 0U };

    fake_reset(FAKE_MODE_RAW);

    /* write: 2 bytes, MSB first */
    CHECK(AD7490_OK == write_frame(&qspi, 0x9710U));
    CHECK(0x9710U == fake_frame_din(1U));
    CHECK(2U == fake_last_byte_count());

    /* txrx: DIN out, DOUT in */
    fake_set_raw_rx(0x5ABCU);
    CHECK(AD7490_OK == txrx_frame(&qspi, 0xFFFFU, &rx_frame));
    CHECK(0x5ABCU == rx_frame);
    CHECK(0xFFFFU == fake_frame_din(2U));

    /* read: DIN held at 0x00 (WRITE = 0) */
    rx_frame = 0U;
    CHECK(AD7490_OK == read_frame(&qspi, &rx_frame));
    CHECK(0x5ABCU == rx_frame);
    CHECK(0x00U == fake_last_dummy_byte());

    /* DOUT decode */
    dout = decode_dout(0x5ABCU);
    CHECK(5U == dout.channel_id);
    CHECK(0xABCU == dout.raw_lsb);
    dout = decode_dout(0xFFFFU);
    CHECK(15U == dout.channel_id);
    CHECK(0xFFFU == dout.raw_lsb);

    /* Failed frames leave the output untouched */
    rx_frame = 0x1111U;
    fake_set_hal_status(QSPI_HAL_ERR_TIMEOUT);
    CHECK(AD7490_ERR_TIMEOUT == txrx_frame(&qspi, 0U, &rx_frame));
    CHECK(0x1111U == rx_frame);
    fake_set_hal_status(QSPI_HAL_ERR_HW);
    CHECK(AD7490_ERR_SPI == read_frame(&qspi, &rx_frame));
    CHECK(0x1111U == rx_frame);
}


static void test_hal_status_map(void)
{
    qspi_hal_config_t qspi = {
        .base_addr = 0x1000U,
        .clock_mode = QSPI_CLOCK_MODE_2,
        .poll_timeout_us = 100U
    };

    fake_reset(FAKE_MODE_RAW);

    fake_set_hal_status(QSPI_HAL_ERR_PARAM);
    CHECK(AD7490_ERR_PARAM == write_frame(&qspi, 0U));

    /* Unknown HAL value (corruption) maps to ERR_SPI: fail safe */
    fake_set_hal_status((qspi_hal_status_t)9U);
    CHECK(AD7490_ERR_SPI == write_frame(&qspi, 0U));

    fake_set_init_status(QSPI_HAL_OK);
    CHECK(AD7490_OK == init_bus(&qspi));
    fake_set_init_status(QSPI_HAL_ERR_HW);
    CHECK(AD7490_ERR_SPI == init_bus(&qspi));
}


static void test_conversion_table(void)
{
    /* Review table: REFIN 2500, RANGE 2x, twos complement */
    CHECK(0U == code_to_mv(0x800U, VREF_TEST_MV, AD7490_RANGE_2X_VREF,
                           AD7490_CODING_TWOS_COMPLEMENT));
    CHECK(2500U == code_to_mv(0x000U, VREF_TEST_MV, AD7490_RANGE_2X_VREF,
                              AD7490_CODING_TWOS_COMPLEMENT));
    CHECK(4999U == code_to_mv(0x7FFU, VREF_TEST_MV, AD7490_RANGE_2X_VREF,
                              AD7490_CODING_TWOS_COMPLEMENT));

    /* Straight binary, rounding to nearest mV */
    CHECK(0U == code_to_mv(0U, VREF_TEST_MV, AD7490_RANGE_1X_VREF,
                           AD7490_CODING_STRAIGHT_BINARY));
    CHECK(1250U == code_to_mv(2048U, VREF_TEST_MV, AD7490_RANGE_1X_VREF,
                              AD7490_CODING_STRAIGHT_BINARY));
    CHECK(2499U == code_to_mv(4095U, VREF_TEST_MV, AD7490_RANGE_1X_VREF,
                              AD7490_CODING_STRAIGHT_BINARY));
    CHECK(5049U == code_to_mv(4095U, AD7490_VREF_MAX_MV,
                              AD7490_RANGE_2X_VREF,
                              AD7490_CODING_STRAIGHT_BINARY));
}


static void test_conversion_exhaustive(void)
{
    uint32_t vref_mv = 0U;
    uint32_t mismatch_count = 0U;
    uint32_t case_count = 0U;

    /* Every REFIN x RANGE x CODING x code vs an independent reference */
    for (vref_mv = AD7490_VREF_MIN_MV;
         vref_mv <= AD7490_VREF_MAX_MV;
         vref_mv++)
    {
        uint32_t combo = 0U;

        for (combo = 0U; combo < 4U; combo++)
        {
            ad7490_range_t range_mode = (ad7490_range_t)(combo & 1U);
            ad7490_coding_t coding = (ad7490_coding_t)(combo >> 1U);
            uint32_t code = 0U;

            for (code = 0U; code <= AD7490_RAW_LSB_MAX; code++)
            {
                double ref = reference_mv(code, vref_mv, range_mode, coding);
                uint32_t got = code_to_mv((uint16_t)code,
                                          vref_mv,
                                          range_mode,
                                          coding);

                if (fabs((double)got - ref) > REF_TOLERANCE_MV)
                {
                    mismatch_count++;
                }
                else
                {
                    /* Within 0.5 mV */
                }

                case_count++;
            }
        }
    }

    CHECK(0U == mismatch_count);
    CHECK(835584U == case_count);
}


static void test_vref_limits(void)
{
    uint32_t voltage_mv = 77U;

    CHECK(is_vref_ok(AD7490_VREF_MIN_MV));
    CHECK(is_vref_ok(AD7490_VREF_MAX_MV));
    CHECK(!is_vref_ok(AD7490_VREF_MIN_MV - 1U));
    CHECK(!is_vref_ok(AD7490_VREF_MAX_MV + 1U));

    /* Corrupted REFIN in a handle: PARAM, output untouched */
    CHECK(AD7490_ERR_PARAM == raw_to_mv(0U,
                                        AD7490_VREF_MIN_MV - 1U,
                                        AD7490_RANGE_1X_VREF,
                                        AD7490_CODING_STRAIGHT_BINARY,
                                        &voltage_mv));
    CHECK(AD7490_ERR_PARAM == raw_to_mv(0U,
                                        AD7490_VREF_MAX_MV + 1U,
                                        AD7490_RANGE_1X_VREF,
                                        AD7490_CODING_STRAIGHT_BINARY,
                                        &voltage_mv));
    CHECK(77U == voltage_mv);
}


static uint32_t code_to_mv(uint16_t raw_lsb,
                           uint32_t vref_mv,
                           ad7490_range_t range_mode,
                           ad7490_coding_t coding)
{
    uint32_t voltage_mv = 0xFFFFFFFFU;

    CHECK(AD7490_OK == raw_to_mv(raw_lsb,
                                 vref_mv,
                                 range_mode,
                                 coding,
                                 &voltage_mv));

    return voltage_mv;
}


/* Independent model: twos complement as a signed offset around mid-span */
static double reference_mv(uint32_t code,
                           uint32_t vref_mv,
                           ad7490_range_t range_mode,
                           ad7490_coding_t coding)
{
    double span_mv = (double)vref_mv;
    double ref_mv = 0.0;

    if (AD7490_RANGE_2X_VREF == range_mode)
    {
        span_mv = 2.0 * span_mv;
    }
    else
    {
        /* 1x span */
    }

    if (AD7490_CODING_TWOS_COMPLEMENT == coding)
    {
        double signed_code = (double)code;

        if (code >= 2048U)
        {
            signed_code = signed_code - 4096.0;
        }
        else
        {
            /* Positive half */
        }

        ref_mv = (span_mv / 2.0) + ((signed_code * span_mv) / 4096.0);
    }
    else
    {
        ref_mv = ((double)code * span_mv) / 4096.0;
    }

    return ref_mv;
}

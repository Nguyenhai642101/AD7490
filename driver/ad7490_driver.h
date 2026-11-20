/**
 * @file        ad7490_driver.h
 * @brief       Driver for the Analog Devices AD7490 (16-channel, 12-bit
 *              SAR ADC) behind a Xilinx AXI Quad SPI: public types,
 *              single-channel read, power modes and channel sequencer.
 * @req         LLR-AD7490-001
 * @design      SDD-AD7490-1.0
 *
 * Typical use:
 *   ad7490_dev_t adc;                           (one handle per ADC)
 *   ad7490_init(&adc, &adc_cfg);                (once, or to recover)
 *   ad7490_read_channel(&adc, 5U, &sample);     (single read)
 *   ad7490_sequencer_start(&adc, 0x0003U);      (or sequenced scans)
 *   ad7490_sequencer_read_scan(&adc, samples, 16U);
 */

#ifndef AD7490_DRIVER_H
#define AD7490_DRIVER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "axi_qspi_hal.h"

/**
 * @brief       Status codes of every AD7490 driver function.
 * @req         LLR-AD7490-002
 * @design      SDD-AD7490-1.1
 */
typedef enum
{
    AD7490_OK                = 0U,
    AD7490_ERR_PARAM         = 1U, /* Invalid argument or configuration */
    AD7490_ERR_SPI           = 2U, /* QSPI hardware fault (MODF, SS0)   */
    AD7490_ERR_TIMEOUT       = 3U, /* QSPI frame did not complete       */
    AD7490_ERR_CHAN_MISMATCH = 4U, /* DOUT channel ID != requested one  */
    AD7490_ERR_STATE         = 5U, /* Call not allowed in device state  */
    AD7490_ERR_DEVICE_FAILED = 6U  /* Re-config limit hit: ADC failed   */
} ad7490_status_t;

/**
 * @brief       Power management modes (AD7490 Table III, PM1..PM0).
 * @req         LLR-AD7490-003
 * @design      SDD-AD7490-1.2
 * @note        Each value equals the PM1:PM0 bit pattern; the driver
 *              shifts it straight into the Control Register.
 */
typedef enum
{
    AD7490_POWER_AUTO_STANDBY  = 0U, /* PM1 = 0, PM0 = 0 */
    AD7490_POWER_AUTO_SHUTDOWN = 1U, /* PM1 = 0, PM0 = 1 */
    AD7490_POWER_FULL_SHUTDOWN = 2U, /* PM1 = 1, PM0 = 0 */
    AD7490_POWER_NORMAL        = 3U  /* PM1 = 1, PM0 = 1 */
} ad7490_power_mode_t;

/**
 * @brief       Analog input range (AD7490 Table I, RANGE bit).
 * @req         LLR-AD7490-004
 * @design      SDD-AD7490-1.3
 * @note        Each value equals the RANGE bit.
 */
typedef enum
{
    AD7490_RANGE_2X_VREF = 0U, /* RANGE = 0: 0 V .. 2 x REFIN */
    AD7490_RANGE_1X_VREF = 1U  /* RANGE = 1: 0 V .. REFIN     */
} ad7490_range_t;

/**
 * @brief       Output coding (AD7490 Table I, CODING bit).
 * @req         LLR-AD7490-005
 * @design      SDD-AD7490-1.4
 * @note        Each value equals the CODING bit.
 */
typedef enum
{
    AD7490_CODING_TWOS_COMPLEMENT = 0U, /* CODING = 0 */
    AD7490_CODING_STRAIGHT_BINARY = 1U  /* CODING = 1 */
} ad7490_coding_t;

/**
 * @brief       DOUT state after the 16th SCLK (AD7490 Table I, WEAK/TRI).
 * @req         LLR-AD7490-006
 * @design      SDD-AD7490-1.5
 * @note        Each value equals the WEAK/TRI bit.
 */
typedef enum
{
    AD7490_DOUT_TRI_STATE = 0U, /* WEAK/TRI = 0: high-Z after frame      */
    AD7490_DOUT_WEAK_ADD3 = 1U  /* WEAK/TRI = 1: weakly drives next ADD3 */
} ad7490_dout_mode_t;

/**
 * @brief       Control Register options that stay fixed for a device.
 * @req         LLR-AD7490-009
 * @design      SDD-AD7490-1.8
 * @note        Separate sub-struct so the frame builder receives only the
 *              fields it encodes (minimal data coupling, rule §3).
 */
typedef struct
{
    ad7490_power_mode_t power_mode; /* PM1..PM0, 0..3        */
    ad7490_range_t      range_mode; /* RANGE bit, 0..1       */
    ad7490_coding_t     coding;     /* CODING bit, 0..1      */
    ad7490_dout_mode_t  dout_mode;  /* WEAK/TRI bit, 0..1    */
} ad7490_ctrl_cfg_t;

/**
 * @brief       Configuration of one AD7490 device.
 * @req         LLR-AD7490-007
 * @design      SDD-AD7490-1.6
 */
typedef struct
{
    qspi_hal_config_t qspi_cfg; /* AXI Quad SPI instance wired to AD7490 */
    uint32_t          vref_mv;  /* REFIN voltage, mV, 2475..2525         */
    ad7490_ctrl_cfg_t ctrl_cfg; /* Control Register options              */
} ad7490_config_t;

/**
 * @brief       One conversion result, raw and in physical units.
 * @req         LLR-AD7490-008
 * @design      SDD-AD7490-1.7
 * @note        raw_lsb and voltage_mv always come from the same frame.
 *              voltage_mv is the real voltage at the VINx pin for both
 *              codings: 0..REFIN (RANGE = 1) or 0..2 x REFIN (RANGE = 0),
 *              i.e. 0..5050 mV at the largest allowed REFIN.
 */
typedef struct
{
    uint8_t  channel_id; /* Channel echoed in DOUT[15:12], 0..15 */
    /* MISRA deviation: Project rule §11.1 — raw code is exported for
     * BIT and calibration logging — read-only copy taken from the same
     * frame as voltage_mv; mV conversion exists only inside the driver;
     * deviation log entry DEV-AD7490-001 */
    uint16_t raw_lsb;    /* 12-bit code DOUT[11:0], LSB, 0..4095 */
    uint32_t voltage_mv; /* Voltage at VINx pin, mV, 0..5050     */
} ad7490_sample_t;

/**
 * @brief       Driver state of one AD7490 device (SEU-robust encoding).
 * @req         LLR-AD7490-040
 * @design      SDD-AD7490-1.9
 * @note        uint32_t + #define instead of enum: C99 limits enumerator
 *              values to int, so 32-bit patterns fail -pedantic-errors.
 *              Pairwise Hamming distance >= 20, and >= 16 from 0x00000000
 *              and 0xFFFFFFFF (rule §9). Any other value, including a
 *              zero-filled handle, is treated as "not initialised".
 */
typedef uint32_t ad7490_dev_state_t;

/**
 * @brief       Handle of one AD7490 device, owned by the caller.
 * @req         LLR-AD7490-041
 * @design      SDD-AD7490-1.10
 * @note        Fields are private to the driver: read or write them only
 *              through ad7490_* functions. One handle per physical ADC.
 */
typedef struct
{
    ad7490_config_t    cfg;          /* Validated copy from ad7490_init() */
    ad7490_dev_state_t state;        /* AD7490_STATE_* pattern            */
    uint16_t           channel_mask; /* Sequenced channels, bit n = VINn  */
} ad7490_dev_t;

/* Analog inputs VIN0..VIN15 (AD7490 pin function descriptions) */
#define AD7490_CHANNEL_COUNT        (16U)

/* Largest 12-bit conversion code, 2^12 - 1 */
#define AD7490_RAW_LSB_MAX          (4095U)

/* REFIN for specified performance: 2.5 V +/- 1 % (AD7490 Specifications,
 * Reference Input). Widening it needs a datasheet-backed justification. */
#define AD7490_VREF_MIN_MV          (2475U)
#define AD7490_VREF_MAX_MV          (2525U)

/* Largest voltage_mv the driver can return: 2 x AD7490_VREF_MAX_MV */
#define AD7490_VOLTAGE_MAX_MV       (5050U)

/* ad7490_dev_state_t patterns (Hamming distances: see type @note) */
#define AD7490_STATE_READY          (0x5A3C96E1U) /* Init done, idle    */
#define AD7490_STATE_SEQUENCING     (0x3CA5E196U) /* Sequencer running  */
#define AD7490_STATE_FAULT          (0xC35A693CU) /* Re-init required   */

/*
 * SPI mode required by the AD7490 serial interface: DIN is latched and
 * DOUT is updated on SCLK falling edges, so the master must drive MOSI on
 * rising edges and sample MISO on falling edges, SCLK idle high:
 * CPOL = 1, CPHA = 0 (datasheet serial interface timing diagram).
 */
#define AD7490_QSPI_CLOCK_MODE      (QSPI_CLOCK_MODE_2)

/* Smallest poll_timeout_us that can finish one frame:
 * 16 SCLK x 160 ns (SCK 6.25 MHz) = 2.56 us, rounded up. */
#define AD7490_MIN_TIMEOUT_US       (3U)

/**
 * @brief       Power up the AD7490 and load its Control Register.
 * @req         LLR-AD7490-010
 * @design      SDD-AD7490-2.1
 * @param[out]  p_dev  Device handle to initialise, must not be NULL.
 * @param[in]   p_cfg  Configuration, must not be NULL; copied into p_dev.
 *                     vref_mv 2475..2525, qspi_cfg.clock_mode =
 *                     AD7490_QSPI_CLOCK_MODE, qspi_cfg.poll_timeout_us >=
 *                     AD7490_MIN_TIMEOUT_US.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_SPI,
 *              AD7490_ERR_TIMEOUT.
 * @note        Not ISR-safe. Not reentrant on the same p_dev or QSPI
 *              instance. Blocking, WCET: QSPI init + 3 frames. Always
 *              allowed, also to recover from AD7490_STATE_FAULT. On any
 *              error p_dev->state is AD7490_STATE_FAULT (when p_dev is
 *              not NULL). Sequence: 2 dummy frames with DIN = 0xFFFF,
 *              then the Control Register write (Figure 16).
 */
ad7490_status_t ad7490_init(ad7490_dev_t * p_dev,
                            ad7490_config_t const * p_cfg);

/**
 * @brief       Convert one analog input channel (VIN0..VIN15).
 * @req         LLR-AD7490-011
 * @design      SDD-AD7490-2.2
 * @param[in]   p_dev       Initialised device handle, must not be NULL.
 * @param[in]   channel_id  Channel to convert, 0..15.
 * @param[out]  p_sample    Result, must not be NULL; written only on
 *                          AD7490_OK. voltage_mv in mV at the VINx pin.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_STATE,
 *              AD7490_ERR_SPI, AD7490_ERR_TIMEOUT,
 *              AD7490_ERR_CHAN_MISMATCH.
 * @note        Not ISR-safe. Not reentrant on the same QSPI instance.
 *              Blocking. Frame 1 writes ADD3..ADD0. NORMAL mode: frame 2
 *              converts channel_id (WCET 2 frames). AUTO_STANDBY /
 *              AUTO_SHUTDOWN: the part sleeps after the CR write, so
 *              frame 2 is a power-up dummy (WRITE = 0, result discarded)
 *              and frame 3 converts channel_id (WCET 3 frames, Fig 14,
 *              15). The channel ID echoed in DOUT is checked against
 *              channel_id.
 *              AD7490_ERR_STATE: not initialised, sequencer active or
 *              FULL_SHUTDOWN power mode.
 */
ad7490_status_t ad7490_read_channel(ad7490_dev_t const * p_dev,
                                    uint8_t channel_id,
                                    ad7490_sample_t * p_sample);

/**
 * @brief       Change the AD7490 power management mode (PM1..PM0).
 * @req         LLR-AD7490-014
 * @design      SDD-AD7490-2.5
 * @param[in,out] p_dev       Initialised device handle, must not be NULL.
 * @param[in]     power_mode  New power mode, 0..3.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_STATE,
 *              AD7490_ERR_SPI, AD7490_ERR_TIMEOUT.
 * @note        Not ISR-safe. Not reentrant on the same p_dev. Blocking,
 *              WCET: 2 frames. Any mode other than FULL_SHUTDOWN is
 *              followed by one dummy frame (2.56 us) that covers the
 *              1 us power-up time. If a frame fails, the ADC power mode
 *              is unknown and p_dev->state becomes AD7490_STATE_FAULT:
 *              call ad7490_init() to recover.
 */
ad7490_status_t ad7490_set_power_mode(ad7490_dev_t * p_dev,
                                      ad7490_power_mode_t power_mode);

/**
 * @brief       Program the Shadow Register and start the sequence.
 * @req         LLR-AD7490-012
 * @design      SDD-AD7490-5.1
 * @param[in,out] p_dev         READY device handle, must not be NULL.
 * @param[in]     channel_mask  Channels to sequence, bit n = VINn,
 *                              0x0001..0xFFFF.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_STATE,
 *              AD7490_ERR_SPI, AD7490_ERR_TIMEOUT.
 * @note        Not ISR-safe. Not reentrant on the same p_dev. Blocking,
 *              WCET: 3 frames. Requires power mode NORMAL (ERR_STATE
 *              otherwise). Success: state SEQUENCING. Bus error: state
 *              FAULT, call ad7490_init() to recover.
 */
ad7490_status_t ad7490_sequencer_start(ad7490_dev_t * p_dev,
                                       uint16_t channel_mask);

/**
 * @brief       Read one full pass: one sample of every sequenced channel.
 * @req         LLR-AD7490-013
 * @design      SDD-AD7490-5.2
 * @param[in,out] p_dev         SEQUENCING device handle, not NULL.
 * @param[out]    p_samples     Array indexed by channel ID, not NULL.
 *                              Only entries of sequenced channels are
 *                              written, and only on AD7490_OK.
 * @param[in]     sample_count  Capacity of p_samples, >= 16.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_STATE,
 *              AD7490_ERR_SPI, AD7490_ERR_TIMEOUT,
 *              AD7490_ERR_CHAN_MISMATCH.
 * @note        Not ISR-safe. Not reentrant on the same p_dev. Blocking,
 *              WCET: 16 frames (one per sequenced channel). The first
 *              frame rewrites the Control Register (SEQ = 1, SHADOW = 0)
 *              so an upset RANGE / CODING / PM bit is repaired within
 *              one pass without stopping the sequence. Every frame
 *              must echo a sequenced channel not yet seen in this pass,
 *              otherwise AD7490_ERR_CHAN_MISMATCH (lost sync). Any error
 *              sets state FAULT; on error p_samples may hold part of
 *              the pass and must be discarded.
 */
ad7490_status_t ad7490_sequencer_read_scan(ad7490_dev_t * p_dev,
                                           ad7490_sample_t * p_samples,
                                           size_t sample_count);

/**
 * @brief       Stop the sequence and return to single-channel mode.
 * @req         LLR-AD7490-015
 * @design      SDD-AD7490-5.3
 * @param[in,out] p_dev  SEQUENCING device handle, must not be NULL.
 * @return      AD7490_OK, AD7490_ERR_PARAM, AD7490_ERR_STATE,
 *              AD7490_ERR_SPI, AD7490_ERR_TIMEOUT.
 * @note        Not ISR-safe. Blocking, WCET: 1 frame. Success: state
 *              READY. Bus error: state FAULT.
 */
ad7490_status_t ad7490_sequencer_stop(ad7490_dev_t * p_dev);

#endif /* AD7490_DRIVER_H */

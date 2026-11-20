/**
 * @file        fake_ad7490.h
 * @brief       Test double: implements the axi_qspi_hal.h API on the host,
 *              either as a byte recorder (RAW) or as a behavioural model
 *              of an AD7490 (ADC).
 *
 * ADC model: 1-frame pipeline (frame N converts the channel selected at
 * the end of frame N-1), Control Register decode (WRITE, SEQ, SHADOW,
 * ADD, CODING), Shadow Register sequencer ring, SEQ=1/SHADOW=0 CR update
 * without stopping the sequence, power states of Figures 13-15 (a frame
 * clocked while the part is not powered returns invalid data; AUTO modes
 * shut down after a CR write and after each valid conversion). Fault
 * injection: bus timeout on a
 * chosen frame, wrong channel echo, DOUT stuck high, register loss
 * (glitch), CODING bit upset, extra sequencer start latency.
 */

#ifndef FAKE_AD7490_H
#define FAKE_AD7490_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "axi_qspi_hal.h"

typedef enum
{
    FAKE_MODE_RAW = 0U, /* Fixed RX word and status, traffic recorded  */
    FAKE_MODE_ADC = 1U  /* Behavioural AD7490 model behind the HAL     */
} fake_mode_t;

/* Number of frames whose DIN word is kept in the log */
#define FAKE_LOG_FRAMES             (64U)

void fake_reset(fake_mode_t mode);

/* RAW mode */
void fake_set_hal_status(qspi_hal_status_t status);
void fake_set_raw_rx(uint16_t rx_word);

/* Both modes */
void fake_set_init_status(qspi_hal_status_t status);
void fake_fail_frame(uint32_t frame_no);
void fake_bad_echo_frame(uint32_t frame_no);
void fake_clear_frame_count(void);
uint32_t fake_frame_count(void);
uint16_t fake_frame_din(uint32_t frame_no);
uint8_t fake_last_dummy_byte(void);
size_t fake_last_byte_count(void);

/* ADC mode */
void fake_adc_set_code(uint32_t channel, uint16_t code);
void fake_adc_set_start_latency(uint32_t extra_frames);
void fake_adc_set_dead(bool is_dead);
void fake_adc_glitch(void);
void fake_adc_upset_coding(void);
uint32_t fake_adc_coding_bit(void);
uint32_t fake_adc_cr(void);
uint32_t fake_adc_invalid_count(void);
bool fake_adc_is_powered(void);

#endif /* FAKE_AD7490_H */

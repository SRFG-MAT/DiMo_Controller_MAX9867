/**************************************************************************//***
 *
 *
 * \file mtb_max9867.h
 *
 * Description: This file contains the function prototypes and constants used
 * in mtb_max9867.c. This driver is intended for the MAX9867 audio codec.
 *
 *******************************************************************************
 * \copyright
 * Copyright 2018-2020 Cypress Semiconductor Corporation
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#pragma once
#include <stdint.h>
#include "cy_pdl.h"
#include "cy_result.h"
//#include "cyhal_i2c.h"

#if defined(__cplusplus)
extern "C"
{
#endif


/**
 * MAX9867 register space
 *
 * The entries below marked as a "two-byte value" are intended for use with these functions:
 * * mtb_max9867_read_word()
 * * mtb_max9867_write_word()
 *
 */
typedef enum
{
	//---------------------------------------------------------------------------------------------
	// STATUS
	//---------------------------------------------------------------------------------------------
	MAX9867_REG_STATUS = 0x00, /* Status (Read Only) */
	MAX9867_REG_JACKSTATUS = 0x01, /* Jack Sense (Read Only) */
	MAX9867_REG_AUXHIGH = 0x02, /* AUX High (Read Only) */
	MAX9867_REG_AUXLOW = 0x03, /* AUX Low (Read Only) */
	MAX9867_REG_INTEN = 0x04, /* Interrupt Enable */

	//---------------------------------------------------------------------------------------------
	// CLOCK CONTROL
	//---------------------------------------------------------------------------------------------
	MAX9867_REG_SYSCLK = 0x05, /* System Clock */
	MAX9867_REG_AUDIOCLKHIGH = 0x06, /* Stereo Audio Clock Control High */
	MAX9867_REG_AUDIOCLKLOW = 0x07, /* Stereo Audio Clock Control Low */

	MAX9867_BITSET_PLL	= (1<<7), /* Set PLL bit in register "Stereo Audio Clock Control High" */

	MAX9867_VAL_RAPIDLOCK = 0x01, /* PLL’s rapid lock mode (Enabled: NI[0] = 1 / Disabled: NI[0] = 0) */
	MAX9867_VAL_PSCLK_10_20 = 0x1, /* System Clock's psclk value: 01 = Select if MCLK is between 10MHz and 20MHz. */
	MAX9867_VAL_PSCLK_20_40 = 0x2, /* System Clock's psclk value: 10 = Select if MCLK is between 20MHz and 40MHz. */
	MAX9867_VAL_PSCLK_40_60 = 0x3, /* System Clock's psclk value: 11 = Select if MCLK is between 40MHz and 60MHz. */
	MAX9867_VAL_PSCLK_SHIFT= 0x4, /* psclk Shift */
	MAX9867_VAL_PSCLK_WIDTH = 0x2, /* psclk Width */

	MAX9867_MASK_PSCLK = (0x03<<MAX9867_VAL_PSCLK_SHIFT), /* MASK for PSCLK */
	MAX9867_MASK_FREQ = 0xF, /* MASK for FREQ: register value = 00001111*/
	MAX9867_MASK_NI_HIGH = 0x7F, /* MASK for NI: register value = 01111111*/
	MAX9867_MASK_NI_LOW	= 0xFE, /* MASK for NI: register value = 11111110 */

	//---------------------------------------------------------------------------------------------
	// DIGITAL AUDIO INTERFACE
	//---------------------------------------------------------------------------------------------
	MAX9867_REG_IFC1A = 0x08, /* Interface Mode */
	MAX9867_REG_IFC1B = 0x09, /* Interface Mode */

	MAX9867_BITSET_MASTER = (1<<7), /* Set MAS bit in register "Interface Mode" */
	MAX9867_BITSET_WCI_MODE = (1<<6), /* Set WCI bit in register "Interface Mode" */
	MAX9867_BITSET_BCI_MODE = (1<<5), /* Set BCI bit in register "Interface Mode" */
	MAX9867_BITSET_I2S_DLY = (1<<4), /* Set DLY bit in register "Interface Mode" */
	MAX9867_BITSET_SDOUT_HIZ = (1<<3), /* Set HIZOFF bit in register "Interface Mode" */
	MAX9867_BITSET_TDM_MODE = (1<<2), /* Set TDM bit in register "Interface Mode" */

	MAX9867_VAL_IFC1B_64X = 0x01, /* BSEL-Value: 001 = 64x LRCLK (192x internal clock divided by 3) */
	MAX9867_VAL_IFC1B_48X = 0x02, /* BSEL-Value: 010 = 48x LRCLK (192x internal clock divided by 4) */
	MAX9867_VAL_IFC1B_PCLK_2 = 0x04, /* BSEL-Value: 100 = PCLK/2 */
	MAX9867_VAL_IFC1B_PCLK_4 = 0x05, /* BSEL-Value: 101 = PCLK/4 */
	MAX9867_VAL_IFC1B_PCLK_8 = 0x06, /* BSEL-Value: 110 = PCLK/8 */
	MAX9867_VAL_IFC1B_PCLK_16 = 0x07, /* BSEL-Value: 111 = PCLK/16 */

	MAX9867_MASK_IFC1B_BCLK = 7, /* MASK for BCLK */

	//---------------------------------------------------------------------------------------------
	// DIGITAL FILTERING
	//---------------------------------------------------------------------------------------------
	MAX9867_REG_CODECFLTR = 0x0a, /* Codec Filters */

	MAX9867_BITSET_CODECFLTR_MODE = (1<<7), /* Set MODE bit in register "Codec Filters" */

	//---------------------------------------------------------------------------------------------
	// LEVEL CONTROL
	//---------------------------------------------------------------------------------------------
	MAX9867_REG_SIDETONE = 0x0b, /* Sidetone */
	MAX9867_REG_DACLEVEL = 0x0c, /* DAC Level */
	MAX9867_REG_ADCLEVEL = 0x0d, /* ADC Level */
	MAX9867_REG_LEFTLINELVL = 0x0e, /* Left-Line Input Level */
	MAX9867_REG_RIGHTLINELVL = 0x0f, /* Right-Line Input Level */
	MAX9867_REG_LEFTVOL = 0x10, /* Left Volume Control */
	MAX9867_REG_RIGHTVOL = 0x11, /* Right Volume Control */
	MAX9867_REG_LEFTMICGAIN = 0x12, /* Left Microphone Gain */
	MAX9867_REG_RIGHTMICGAIN = 0x13, /* Right Microphone Gain */
	MAX9867_REG_INPUTCONFIG = 0x14, /* CONFIGURATION / ADC Input */
	MAX9867_REG_MICCONFIG = 0x15, /* Microphone */
	MAX9867_REG_MODECONFIG = 0x16, /* Mode */

	//---------------------------------------------------------------------------------------------
	// POWER MANAGEMENT
	//---------------------------------------------------------------------------------------------
	MAX9867_REG_PWRMAN = 0x17, /* System Shutdown */
	MAX9867_REG_REVISION = 0xff, /* Revision */

	MAX9867_BITSET_PWRMAN_SHDN = (1<<7), /* Set SHDN bit in register "System Shutdown" */

	//---------------------------------------------------------------------------------------------
	// OTHERS (unused)
	//---------------------------------------------------------------------------------------------
	MAX9867_VAL_CACHEREGNUM = 10 /**/

} mtb_max9867_reg_t;


/** Initialization failure error */
#define CY_RSLT_MAX9867_INIT_FAIL 0 // TODO

/**
 * Initialize the I2C communication with the audio codec and do basic configuration of
 * the codec.
 * @param[in] i2c_inst I2C instance to use for communicating with the MAX9867 audio codec.
 * @return CY_RSLT_SUCCESS if properly initialized, else an error indicating what went wrong.
 */
cy_rslt_t mtb_max9867_init(CySCB_Type *base, cy_stc_scb_i2c_context_t *context);

/**
 * Frees up any resources allocated by the display as part of \ref mtb_max9867_init().
 */
void mtb_max9867_free();

/**
 * This function updates the volume of both the left and right channels of the
 * headphone output.
 *
 * @param[in] volume - Steps of 0.5dB, where:
 *            Minimum volume: -65.5dB (0x8F)
 *            Maximum volume:  +6.0dB (0x00)
 *            Mute: (0x90~0xFF)
 *
 */
void mtb_max9867_adjust_volume(uint8_t volume);

/**
 * Activates the codec - This function is called in conjunction with 
 * max9867_deactivate API after successful configuration update of the codec.
 */
void mtb_max9867_activate(void);

/**
 * Deactivates the codec - the configuration is retained, just the codec 
 * input/outputs are disabled. The function should be called before changing 
 * any setting in the codec over I2C
 */
void mtb_max9867_deactivate(void);

/**
 * This function writes a data byte to an audio codec register
 *
 * @param[in] reg   The audio codec register to update
 * @param[in] data  The byte to be written to the audio codec register
 */
void mtb_max9867_write_byte(mtb_max9867_reg_t reg, uint8_t data);

/**
 * This function sets bits in a register.  This function can be used instead
 * of mtb_max9867_write_byte() if you want to change a single bit or select bits in
 * the register and preserve the value of other bits in the register. Only the bits
 * set to 1 in the mask are effected.
 *
 * @param[in] reg   The audio codec register to update
 * @param[in] mask  The mask used to set bits in the register
 */
void mtb_max9867_set(mtb_max9867_reg_t reg, uint8_t mask);

/**
 * This function clears bits in a register.  This function can be used instead
 * of mtb_max9867_write_byte() if you want to change a single bit or select bits in
 * the register and preserve the value of other bits in the register. Only the bits
 * set to 1 in the mask are effected.
 *
 * @param[in] reg   The audio codec register to update
 * @param[in] mask  The mask used to clear bits in the register
 */
void mtb_max9867_clear(mtb_max9867_reg_t reg, uint8_t mask);

/**
 * This function writes a data value that spans two register addresses
 *
 * @param[in] reg   The first of two audio codec registers to update
 * @param[in] data  The word (two-byte value) to be written to the audio codec register
 */
void mtb_max9867_write_word(mtb_max9867_reg_t reg, uint16_t data);

///**
// * This function writes multiple data bytes to the audio codec registers
// *
// * @param[in] reg       The first audio codec register to update
// * @param[in] pData     Pointer to the buffer that has data
// * @param[in] numBytes  Number of bytes to be written to the display controller
// */
//void mtb_max9867_write_data_stream(mtb_max9867_reg_t reg, uint8_t *pData, uint8_t numBytes);

/**
 * This function reads a data byte from an audio codec register
 *
 * @param[in] reg    The audio codec register read
 * @return data The byte read from the audio codec register
 */
uint8_t mtb_max9867_read_byte(mtb_max9867_reg_t reg);

/**
 * This function reads a data value that spans two register addresses
 *
 * @param[in] reg    The first of two audio codec registers to read
 * @return data The word (two-byte value) read from the audio codec register
 */
uint16_t mtb_max9867_read_word(mtb_max9867_reg_t reg);

///**
// * This function reads multiple data bytes from the audio codec registers
// *
// * @param[in] reg       The first audio codec register to read
// * @param[in] pData     Pointer to the location to store the data read
// * @param[in] numBytes  Number of bytes to read from the display controller
// */
//void mtb_max9867_read_data_stream(mtb_max9867_reg_t reg, uint8_t *pData, uint8_t numBytes);

#if defined(__cplusplus)
}
#endif

/** \} group_board_libs */

/**************************************************************************//***
 * \file ak4954a.c
 *
 * Description: This file contains the AK4954A codec control APIs.
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



/*
 * REMARK (2022 / DiMo Project):
 * (a) THIS DEMO-CODE (for the audio-chip "AK4954A") is originally based on the following similar "PSoC 62"-based example:
 * "CY8CKIT-062-BLE" --> Examples --> Peripherals --> "I2S Audio" see also: https://github.com/Infineon/mtb-example-psoc6-i2s/blob/master/README.md
 *
 * (b) However, in this code-example the underlying low-level libs I2S/I2C have been ported, so that the code is now using the *PDL*-based libraries (=configured by Modus-Toolbox GUI-"Device Configurator")
 * instead of the I2S *HAL*-library used in the original example (less recommended when targeting easier portable code for development-projects)
 *
 * GENERAL WARNING: Please *avoid* using/sharing the same HW-PIN-resources (here I2C-Master + I2S) in a mixed-setting with other *HAL*-based code-examples, this would collide (only share the same HW-ressources with PDL-based code libraries)
 *
 * IMPORTANT Remark: Currently the code only works when audio-chip is correctly connected, this proof-of-concept demo-code doesn't have perfect error-handling (see "//xxxx TODO" below)
 *
 */




#include "../release-v1.0.1-ported-to-pdl-lib/mtb_ak4954a.h"

#include <stdbool.h>


/* I2C-Address of the Audio-Chip AK4954A */
#define AK4954A_I2C_ADDR  (0x12u)

/* These 2 globals are used for accessing the I2C (base-pointer to hw-interface and context storing the state) */
static CySCB_Type *i2c_base_ptr;
static cy_stc_scb_i2c_context_t *i2c_context_ptr;


/*******************************************************************************
 * Initialize the Audio codec.
 *******************************************************************************/
cy_rslt_t mtb_ak4954a_init(CySCB_Type *base, cy_stc_scb_i2c_context_t *context)
{
    if (base == NULL)
        return CY_RSLT_AK4954A_INIT_FAIL;

	i2c_base_ptr = base;
	i2c_context_ptr = context;


    /* From the AK4954A spec:
     * Upon power-up, the AK4954A must be reset by bringing the PDN pin low. This reset is
     * released when a dummy command is input after the PDN pin is high. This ensures that all
     * internal registers reset to their initial value. This reset is released when the dummy
     * command (Actually, the rising edge of 16th SCL) is input after PDN pin is high. Dummy
     * command is executed by writing all 0's to the register address 00H. It is recommended
     * to set the PDN pin low before power up the AK4954A.
     *
     * We don't want to call mtb_ak4954a_write_byte() here because we will hang on the
     * CY_ASSERT() check.  The second call to cyhal_i2c_master_write() is needed to clean up
     * the error that occurs with the first one.
     * */
    {
    	cy_stc_scb_i2c_master_xfer_config_t transfer;
    	uint8_t writeBuffer[] = {AK4954A_REG_PWR_MGMT1, 0};

    	/* Configure write transaction */
    	transfer.slaveAddress = AK4954A_I2C_ADDR;
    	transfer.buffer       = writeBuffer;
    	transfer.bufferSize   = sizeof(writeBuffer);
    	transfer.xferPending  = false; /* Generate Stop condition at the end of transaction */

    	/* Initiate I2C-write transaction. */
    	Cy_SCB_I2C_MasterWrite(i2c_base_ptr, &transfer, i2c_context_ptr);
    	/* Blocking-Wait for transaction completion */
    	//xxxx TODO: Error handling (currently it would get stuck in case of "NACK"
    	while (0UL != (CY_SCB_I2C_MASTER_BUSY & Cy_SCB_I2C_MasterGetStatus(i2c_base_ptr, i2c_context_ptr))) {}

    	/* Initiate I2C-write transaction. */
    	Cy_SCB_I2C_MasterWrite(i2c_base_ptr, &transfer, i2c_context_ptr);

    	/* Blocking-Wait for transaction completion */
    	//xxxx TODO: Error handling (currently it would get stuck in case of "NACK"
    	while (0UL != (CY_SCB_I2C_MASTER_BUSY & Cy_SCB_I2C_MasterGetStatus(i2c_base_ptr, i2c_context_ptr))) {}
    }

    /* Clear Power Managament 1 register */
    mtb_ak4954a_write_byte(AK4954A_REG_PWR_MGMT1, 0x00);

    /* Set the data alignment */
    mtb_ak4954a_write_byte(AK4954A_REG_MODE_CTRL1, AK4954A_DEF_DATA_ALIGNMENT);

    /* Set sample rate */
    mtb_ak4954a_write_byte(AK4954A_REG_MODE_CTRL2, AK4954A_DEF_SAMPLING_RATE |
                                                   AK4954A_MODE_CTRL2_FS_48kHz);

    /* Clear Digital Filter Mode register */
    mtb_ak4954a_write_byte(AK4954A_REG_DIG_FLTR_MODE, 0x00);

    return CYRET_SUCCESS;
}


/*******************************************************************************
 * Free the resources used with the Audio codec.
 *******************************************************************************/
void mtb_ak4954a_free(void)
{
	i2c_base_ptr = NULL;
}


/*******************************************************************************
 * This function writes a data byte to an audio codec register
 *******************************************************************************/
void mtb_ak4954a_write_byte(mtb_ak4954a_reg_t reg, uint8_t data)
{
    cy_rslt_t rslt;
    cy_stc_scb_i2c_master_xfer_config_t transfer;

    uint8_t writeBuffer[] = {reg, data};

	/* Configure write transaction */
	transfer.slaveAddress = AK4954A_I2C_ADDR;
	transfer.buffer       = writeBuffer;
	transfer.bufferSize   = sizeof(writeBuffer);
	transfer.xferPending  = false; /* Generate Stop condition at the end of transaction */

	/* Initiate I2C-write transaction. */
	Cy_SCB_I2C_MasterWrite(i2c_base_ptr, &transfer, i2c_context_ptr);

	/* Blocking-Wait for transaction completion */
	//xxxx TODO: Error handling (currently it would get stuck in case of "NACK"
	while (0UL != (CY_SCB_I2C_MASTER_BUSY & Cy_SCB_I2C_MasterGetStatus(i2c_base_ptr, i2c_context_ptr))) {}

    rslt = CY_RSLT_SUCCESS;
}


/*******************************************************************************
 * This function sets bits in a register.
 *******************************************************************************/
void mtb_ak4954a_set(mtb_ak4954a_reg_t reg, uint8_t mask)
{
    uint8_t data = mtb_ak4954a_read_byte(reg) | mask;
    mtb_ak4954a_write_byte(reg, data);
}


/*******************************************************************************
 * This function clears bits in a register.
 *******************************************************************************/
void mtb_ak4954a_clear(mtb_ak4954a_reg_t reg, uint8_t mask)
{
    uint8_t data = mtb_ak4954a_read_byte(reg) & ~mask;
    mtb_ak4954a_write_byte(reg, data);
}


/*******************************************************************************
 * This function writes a data value that spans two register addresses
 *******************************************************************************/
void mtb_ak4954a_write_word(mtb_ak4954a_reg_t reg, uint16_t data)
{
    cy_rslt_t rslt;
    cy_stc_scb_i2c_master_xfer_config_t transfer;

    uint8_t writeBuffer[] = {reg, ((uint8_t*)&data)[0], ((uint8_t*)&data)[1]};

	/* Configure write transaction */
	transfer.slaveAddress = AK4954A_I2C_ADDR;
	transfer.buffer       = writeBuffer;
	transfer.bufferSize   = sizeof(writeBuffer);
	transfer.xferPending  = false; /* Generate Stop condition at the end of transaction */

	/* Initiate I2C-write transaction. */
	Cy_SCB_I2C_MasterWrite(i2c_base_ptr, &transfer, i2c_context_ptr);

	/* Blocking-Wait for transaction completion */
	//xxxx TODO: Error handling (currently it would get stuck in case of "NACK"
	while (0UL != (CY_SCB_I2C_MASTER_BUSY & Cy_SCB_I2C_MasterGetStatus(i2c_base_ptr, i2c_context_ptr))) {}

    rslt = CY_RSLT_SUCCESS;

}


/*******************************************************************************
 * This function reads a data byte from an audio codec register
 *******************************************************************************/
uint8_t mtb_ak4954a_read_byte(mtb_ak4954a_reg_t reg)
{
    cy_rslt_t rslt;
    cy_stc_scb_i2c_master_xfer_config_t transfer;

    uint8_t writeBuffer[] = {reg};
    uint8_t readBuffer1Byte;

	/* Configure write transaction */
	transfer.slaveAddress = AK4954A_I2C_ADDR;
	transfer.buffer       = writeBuffer;
	transfer.bufferSize   = sizeof(writeBuffer);
	transfer.xferPending  = true; /* Do not generate stop condition at the end of transaction (read after write) */

	/* Initiate I2C-write transaction. */
	Cy_SCB_I2C_MasterWrite(i2c_base_ptr, &transfer, i2c_context_ptr);

	/* Blocking-Wait for transaction completion */
	//xxxx TODO: Error handling (currently it would get stuck in case of "NACK"
	while (0UL != (CY_SCB_I2C_MASTER_BUSY & Cy_SCB_I2C_MasterGetStatus(i2c_base_ptr, i2c_context_ptr))) {}

    rslt = CY_RSLT_SUCCESS;

	/* Configure read transaction */
	transfer.slaveAddress = AK4954A_I2C_ADDR;
	transfer.buffer       = &readBuffer1Byte;
	transfer.bufferSize   = 1;
	transfer.xferPending  = false; /* Generate Stop condition the end of transaction (stop after read) */

	/* Initiate read transaction.
	* The ReStart condition is generated to begin this transaction because
	* previous transaction was completed without Stop.
	*/
	Cy_SCB_I2C_MasterRead(i2c_base_ptr, &transfer, i2c_context_ptr);

	/* Blocking-Wait for transaction completion */
	//xxxx TODO: Error handling (currently it would get stuck in case of "NACK"
	while (0UL != (CY_SCB_I2C_MASTER_BUSY & Cy_SCB_I2C_MasterGetStatus(i2c_base_ptr, i2c_context_ptr))) {}

    rslt = CY_RSLT_SUCCESS;

    return readBuffer1Byte;
}


/*******************************************************************************
 * This function reads a data value that spans two register addresses
 *******************************************************************************/
uint16_t mtb_ak4954a_read_word(mtb_ak4954a_reg_t reg)
{
    cy_rslt_t rslt;
    cy_stc_scb_i2c_master_xfer_config_t transfer;

    uint8_t writeBuffer[] = {reg};
    uint16_t readBuffer2Bytes;

	/* Configure write transaction */
	transfer.slaveAddress = AK4954A_I2C_ADDR;
	transfer.buffer       = writeBuffer;
	transfer.bufferSize   = sizeof(writeBuffer);
	transfer.xferPending  = true; /* Do not generate stop condition at the end of transaction (read after write) */

	/* Initiate I2C-write transaction. */
	Cy_SCB_I2C_MasterWrite(i2c_base_ptr, &transfer, i2c_context_ptr);

	/* Blocking-Wait for transaction completion */
	//xxxx TODO: Error handling (currently it would get stuck in case of "NACK"
	while (0UL != (CY_SCB_I2C_MASTER_BUSY & Cy_SCB_I2C_MasterGetStatus(i2c_base_ptr, i2c_context_ptr))) {}

    rslt = CY_RSLT_SUCCESS;

	/* Configure read transaction */
	transfer.slaveAddress = AK4954A_I2C_ADDR;
	transfer.buffer       = &readBuffer2Bytes;
	transfer.bufferSize   = 2;
	transfer.xferPending  = false; /* Generate Stop condition the end of transaction (stop after read) */

	/* Initiate read transaction.
	* The ReStart condition is generated to begin this transaction because
	* previous transaction was completed without Stop.
	*/
	Cy_SCB_I2C_MasterRead(i2c_base_ptr, &transfer, i2c_context_ptr);

	/* Blocking-Wait for transaction completion */
	//xxxx TODO: Error handling (currently it would get stuck in case of "NACK"
	while (0UL != (CY_SCB_I2C_MASTER_BUSY & Cy_SCB_I2C_MasterGetStatus(i2c_base_ptr, i2c_context_ptr))) {}

    rslt = CY_RSLT_SUCCESS;

    return readBuffer2Bytes;

}


/*******************************************************************************
 * This function updates the volume of both the left and right channels of the
 * headphone output.
 *******************************************************************************/
void mtb_ak4954a_adjust_volume(uint8_t volume)
{
    mtb_ak4954a_write_byte(AK4954A_REG_LCH_DIG_VOL_CTRL, volume);
    mtb_ak4954a_write_byte(AK4954A_REG_RCH_DIG_VOL_CTRL, volume);
}

/*******************************************************************************
 * Activates the codec - This function is called in conjunction with
 * ak4954A_deactivate API after successful configuration update of the codec.
 *******************************************************************************/
void mtb_ak4954a_activate(void)
{
    /* Enable Power Management DAC */
    mtb_ak4954a_write_byte(AK4954A_REG_PWR_MGMT1,
                           AK4954A_PWR_MGMT1_PMDAC | AK4954A_PWR_MGMT1_PMVCM);

    /* Enable Left/Right Channels */
    mtb_ak4954a_write_byte(AK4954A_REG_PWR_MGMT2,
                           AK4954A_PWR_MGMT2_PMHPL | AK4954A_PWR_MGMT2_PMHPR);
}

/*******************************************************************************
 * Deactivates the codec - the configuration is retained, just the codec
 * input/outputs are disabled. The function should be called before changing
 * any setting in the codec over I2C
 *******************************************************************************/
void mtb_ak4954a_deactivate(void)
{
    /* Disable Left/Right Channels */
    mtb_ak4954a_write_byte(AK4954A_REG_PWR_MGMT2, 0x00);

    /* Disable Power Management DAC */
    mtb_ak4954a_write_byte(AK4954A_REG_PWR_MGMT1, AK4954A_PWR_MGMT1_PMVCM);
}

/* [] END OF FILE */

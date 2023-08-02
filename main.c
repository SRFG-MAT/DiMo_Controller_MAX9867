/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty PSoC6 Application
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
 * Include header files
 ******************************************************************************/
//#include "audio-codec-ak4954a/release-v1.0.1-ported-to-pdl-lib/mtb_ak4954a.h"
#include "audio_codec/mtb_max9867.h"
#include "cybsp.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cycfg.h"
#include "cycfg_peripherals.h"
#include "led.h"

#include "MUS_Tribal_Kick_st_os_01.h"
#include "SFX_Tribal_Inhale_st_os_01.h"
#include "SFX_Tribal_Exhale_st_os_01.h"

/*******************************************************************************
* Macros
*******************************************************************************/


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void handle_error(void);

void I2C_Isr(void); // I2C IRQ callback (in this example used for I2C-Master)
void playKick(void); // Simple demo ported from PSoC62-demo, from playing 1x wav-file to the audio-chip
void playInhale(void); // Simple demo ported from PSoC62-demo, from playing 1x wav-file to the audio-chip
void playExhale(void); // Simple demo ported from PSoC62-demo, from playing 1x wav-file to the audio-chip


/*******************************************************************************
* Global Variables
*******************************************************************************/

/* I2C MASTER */
cy_stc_scb_i2c_context_t I2C_MASTER_Context; // Allocate context
#define I2C_MASTER_BUFFER_SIZE (2UL)
uint8_t I2C_MASTER_ReadBuffer[I2C_MASTER_BUFFER_SIZE]; // Allocate RX buffer
uint8_t I2C_MASTER_WriteBuffer[I2C_MASTER_BUFFER_SIZE]; // Allocate TX buffer

/* Assign I2C interrupt number and priority */
// REMARK: Check "Device Configurator" which SCB_x Number it is (different SCBs are hard-wired to different IO-Pins...)
#define I2C_INTR_NUM        scb_2_interrupt_IRQn
#define I2C_INTR_PRIORITY   (7UL)


/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(void)
{
    /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

void play_delay() {
	cyhal_system_delay_ms(1 * 50);
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function performs
*  - initial setup of device and configuration of external I2S-compatible audio-chip "AK4954A" (configured via I2C)
*  - endless loop: streaming 1x demo-audio wave-file and changing the volume in each loop
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    uint i;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq(); // Remark: Already must be enabled here, required for the audio configuration


    initLed();
    setStatus(1, 0);
    /* *********** AUDIO, Config-Part (1) = I2S HW-Interface ************************** */
    // Init I2S HW-interface (output of audio stream)
    if(CY_I2S_SUCCESS != Cy_I2S_Init(I2S0, &AUDIO_I2S_config))
    {
        /* Insert error handling */
        CY_ASSERT(0);
    }
    // Init PWM used as clock for I2C audio stream
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_PWM_Init(AUDIO_MCLK_HW, AUDIO_MCLK_NUM, &AUDIO_MCLK_config))
    {
        /* Handle possible errors */
    	CY_ASSERT(0);
    }
    // Enable and start PWM used as clock for I2C audio stream
    Cy_TCPWM_PWM_Enable(AUDIO_MCLK_HW, AUDIO_MCLK_NUM);
    Cy_TCPWM_TriggerReloadOrIndex_Single(AUDIO_MCLK_HW, AUDIO_MCLK_NUM); // Warning: Use "NUM", not MASK here!
    /* ******************************************************************************** */


    /* *********** AUDIO, Config-Part (2) = I2C HW-Interface ************************** */
    /* Wait for the MCLK to clock the audio codec */
    cyhal_system_delay_ms(1);

    /* Configure I2C to operate */
    Cy_SCB_I2C_Init(I2C_MASTER_HW, &I2C_MASTER_config, &I2C_MASTER_Context);

    /* Populate I2C ISR-configuration structure */
    const cy_stc_sysint_t i2cIntrConfig =
    {
        .intrSrc      = I2C_INTR_NUM,
        .intrPriority = I2C_INTR_PRIORITY,
    };
    /* Hook interrupt service routine and enable interrupt */
    Cy_SysInt_Init(&i2cIntrConfig, &I2C_Isr);
    NVIC_EnableIRQ(I2C_INTR_NUM);

    /* Enable I2C to operate */
    Cy_SCB_I2C_Enable(I2C_MASTER_HW);
    /* ******************************************************************************** */


    /** *********** AUDIO, Config-Part (3) = AK494A chip-specific configuration (via I2C) ************************** **/
	/* Configure the AK494A codec and enable it */
	result = mtb_max9867_init(I2C_MASTER_HW, &I2C_MASTER_Context);
	if (CYRET_SUCCESS != result)
	{
		/* Halt the CPU if AK494A initialization failed */
		CY_ASSERT(0);
	}
	mtb_max9867_activate();
	mtb_max9867_adjust_volume(MAX9867_VAL_VOLUME_DEFAULT);
    /* ******************************************************************************** */


    // Initialize LED: Turn off (remark: LED is inverse, turned off "by setting GPIO = 1")

    /* Enable global interrupts */
    __enable_irq();
    setStatus(0, 1);


    while(1) {

    		playKick();
    		play_delay();

    		playKick();
    		play_delay();

    		playInhale();
    		toggleRed();
    		play_delay();

    		playKick();
    		play_delay();

    		playKick();
    		play_delay();

    		playExhale();
    		play_delay();
    		toggleGreen();
    }
}



/*******************************************************************************
* I2C ISR: IRQ routine mandatory when using I2C based on the PDL-library "Cy_SCB_I2C" (see PDL-documentation)
*******************************************************************************/
void I2C_Isr(void)
{
	Cy_SCB_I2C_Interrupt(I2C_MASTER_HW, &I2C_MASTER_Context);
}



/*******************************************************************************
* audio_simple_demo: Just a simple proof-of-concept demo, showing how to stream audio (such as a "wave-file") to an I2S-standard compatible audio-chip "AK4954A"
*
* REMARK (2022 / DiMo Project):
* (a) THIS DEMO-CODE (for the audio-chip "AK4954A") is originally based on the following similar "PSoC 62"-based example:
* "CY8CKIT-062-BLE" --> Examples --> Peripherals --> "I2S Audio" see also: https://github.com/Infineon/mtb-example-psoc6-i2s/blob/master/README.md
*
* (b) However, in this code-example the underlying low-level libs I2S/I2C have been ported, so that the code is now using the *PDL*-based libraries (=configured by Modus-Toolbox GUI-"Device Configurator")
* instead of the I2S *HAL*-library used in the original example (less recommended when targeting easier portable code for development-projects)
*
* GENERAL WARNING: Please *avoid* using/sharing the same HW-PIN-resources (here I2C-Master + I2S) in a mixed-setting with other *HAL*-based code-examples, this would collide (only share the same HW-ressources with PDL-based code libraries)
*
*
*******************************************************************************/
void playKick()
{
	uint i;
	uint n;

    /* Clear both I2S FIFOs */

    //Cy_I2S_ClearRxFifo (I2S0); // RX not required in this example
    Cy_I2S_ClearTxFifo (I2S0);

    /* Put at least one frame (may be empty) into the Tx FIFO - two data words for I2S format */
    Cy_I2S_WriteTxData (I2S0, 0UL); Cy_I2S_WriteTxData (I2S0, 0UL);


    /* Clear possible pending interrupts */
    Cy_I2S_ClearInterrupt(I2S0, CY_I2S_INTR_TX_EMPTY);
    /* Enable interrupts */
    Cy_I2S_SetInterruptMask(I2S0, CY_I2S_INTR_TX_EMPTY);
    /* Enable I2S communication */
    Cy_I2S_EnableTx(I2S0);


    // START OF ACTUAL Audio-Streaming of a "wave-file"(array) to the Audio-Chip via I2S
    // By constantly filling the FIFO with wave-array, using BUSY-WAIT between FIFO-filling
    //
    // REMARK (DiMo): This is just a simple proof-of-concept example code. When using in a practical Free-RTOS based environment,
    //                this should be done in a worker-task (also mp3-decoding if necessary)
    for(i=0; i<13000; i++) {
    	Cy_I2S_WriteTxData (I2S0, kick_data[i] );

    	// BUSY-WAIT between FIFO-filling  (REMARK: Busy-waiting is just for this simple example, since this does not use any RTOS here)
    	n=1;
        while(n>0) {
        	n = Cy_I2S_GetNumInTxFifo(I2S0);
        }

    }

    // When audio-stream is finished, disable I2S IRQ and disable TX
    Cy_I2S_SetInterruptMask(I2S0, 0U);
    Cy_I2S_DisableTx(I2S0);

}

void playInhale()
{
	uint i;
	uint n;

    /* Clear both I2S FIFOs */

    //Cy_I2S_ClearRxFifo (I2S0); // RX not required in this example
    Cy_I2S_ClearTxFifo (I2S0);

    /* Put at least one frame (may be empty) into the Tx FIFO - two data words for I2S format */
    Cy_I2S_WriteTxData (I2S0, 0UL); Cy_I2S_WriteTxData (I2S0, 0UL);


    /* Clear possible pending interrupts */
    Cy_I2S_ClearInterrupt(I2S0, CY_I2S_INTR_TX_EMPTY);
    /* Enable interrupts */
    Cy_I2S_SetInterruptMask(I2S0, CY_I2S_INTR_TX_EMPTY);
    /* Enable I2S communication */
    Cy_I2S_EnableTx(I2S0);


    // START OF ACTUAL Audio-Streaming of a "wave-file"(array) to the Audio-Chip via I2S
    // By constantly filling the FIFO with wave-array, using BUSY-WAIT between FIFO-filling
    //
    // REMARK (DiMo): This is just a simple proof-of-concept example code. When using in a practical Free-RTOS based environment,
    //                this should be done in a worker-task (also mp3-decoding if necessary)
    for(i=0; i<13000-1; i++) {
    	Cy_I2S_WriteTxData (I2S0, inhale_data[i] );

    	// BUSY-WAIT between FIFO-filling  (REMARK: Busy-waiting is just for this simple example, since this does not use any RTOS here)
    	n=1;
        while(n>0) {
        	n = Cy_I2S_GetNumInTxFifo(I2S0);
        }

    }

    // When audio-stream is finished, disable I2S IRQ and disable TX
    Cy_I2S_SetInterruptMask(I2S0, 0U);
    Cy_I2S_DisableTx(I2S0);

}

void playExhale()
{
	uint i;
	uint n;

    /* Clear both I2S FIFOs */

    //Cy_I2S_ClearRxFifo (I2S0); // RX not required in this example
    Cy_I2S_ClearTxFifo (I2S0);

    /* Put at least one frame (may be empty) into the Tx FIFO - two data words for I2S format */
    Cy_I2S_WriteTxData (I2S0, 0UL); Cy_I2S_WriteTxData (I2S0, 0UL);


    /* Clear possible pending interrupts */
    Cy_I2S_ClearInterrupt(I2S0, CY_I2S_INTR_TX_EMPTY);
    /* Enable interrupts */
    Cy_I2S_SetInterruptMask(I2S0, CY_I2S_INTR_TX_EMPTY);
    /* Enable I2S communication */
    Cy_I2S_EnableTx(I2S0);


    // START OF ACTUAL Audio-Streaming of a "wave-file"(array) to the Audio-Chip via I2S
    // By constantly filling the FIFO with wave-array, using BUSY-WAIT between FIFO-filling
    //
    // REMARK (DiMo): This is just a simple proof-of-concept example code. When using in a practical Free-RTOS based environment,
    //                this should be done in a worker-task (also mp3-decoding if necessary)
    for(i=0; i<13000; i++) {
    	Cy_I2S_WriteTxData (I2S0, exhale_data[i] );

    	// BUSY-WAIT between FIFO-filling  (REMARK: Busy-waiting is just for this simple example, since this does not use any RTOS here)
    	n=1;
        while(n>0) {
        	n = Cy_I2S_GetNumInTxFifo(I2S0);
        }

    }

    // When audio-stream is finished, disable I2S IRQ and disable TX
    Cy_I2S_SetInterruptMask(I2S0, 0U);
    Cy_I2S_DisableTx(I2S0);

}

/* [] END OF FILE */

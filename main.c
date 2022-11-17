/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RutDevKit-PSoC62_Arduino_ADC_DMA_PDL
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2022-10-28
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
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
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/* DMA Interrupt Handler */
void dma_interrupt(void);

/* Buffer to store data from SAR using DMA */
int16_t sar_buffer[12] = {0};

/* This flag set in the DMA interrupt handler */
volatile bool dmaIntrTriggered = false;

/* DMA Transfer complete/error flags sent in DMA interrupt Handler*/
volatile uint8_t adc_dma_error;   /* ADCDma error flag */
volatile uint8_t adc_dma_done;    /* ADCDma done flag */

/* DMA interrupt configuration structure */
/* Source is set to DW 0 and Priority as 7 */
const cy_stc_sysint_t intRxDma_cfg =
{
        .intrSrc      = cpuss_interrupts_dw0_28_IRQn,
        .intrPriority = 7
};

void handle_error(void);
cy_rslt_t app_hw_init(void);

int main(void)
{
    cy_rslt_t result;
    int32_t arduino_analog[6] = {0};

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
    	handle_error();
    }

    /*Enable debug output via KitProg UART*/
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
    printf("\x1b[2J\x1b[;H");
    printf("RDK2 Arduino ADC DMA PDL Example.\r\n");

    /*Initialize LEDs*/
    result = cyhal_gpio_init( LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_gpio_init( LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Initialize voltage divider control */
    result = cyhal_gpio_init( CANFD_STB, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    __enable_irq();

    /*Configure the hardware*/
    cyhal_gpio_write(CANFD_STB, true); /*Enable battery voltage divider*/
    result = app_hw_init();
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }

    /* Start the TCPWM Timer */
    Cy_TCPWM_TriggerStart_Single(ADC_TIMER_HW, ADC_TIMER_NUM);

    for (;;)
    {
    	CyDelay(1000);
    	arduino_analog[0] = Cy_SAR_CountsTo_uVolts(SAR_ADC_HW, 0, sar_buffer[0])/1000;
    	arduino_analog[1] = Cy_SAR_CountsTo_uVolts(SAR_ADC_HW, 0, sar_buffer[1])/1000;
    	arduino_analog[2] = Cy_SAR_CountsTo_uVolts(SAR_ADC_HW, 0, sar_buffer[2])/1000;
    	arduino_analog[3] = Cy_SAR_CountsTo_uVolts(SAR_ADC_HW, 0, sar_buffer[3])/1000;
    	arduino_analog[4] = Cy_SAR_CountsTo_uVolts(SAR_ADC_HW, 0, sar_buffer[4])/1000;
    	arduino_analog[5] = Cy_SAR_CountsTo_uVolts(SAR_ADC_HW, 0, sar_buffer[5])/1000;
    	printf("A0: %4ldmV, A1: %4ldmV, A2: %4ldmV, A3: %4ldmV, A4: %4ldmV, A5: %4ldmV\r\n"
    												,(long int)arduino_analog[0]
													,(long int)arduino_analog[1]
													,(long int)arduino_analog[2]
													,(long int)arduino_analog[3]
													,(long int)arduino_analog[4]
													,(long int)arduino_analog[5]);
    	cyhal_gpio_toggle(LED1);
    }
}

void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

cy_rslt_t app_hw_init(void)
{
	cy_en_tcpwm_status_t tcpwm_res;
	cy_en_sysanalog_status_t aref_res;
	cy_en_sar_status_t sar_res;
	cy_en_dma_status_t dma_res;

	/*TCPWM*/
	tcpwm_res = Cy_TCPWM_Counter_Init(ADC_TIMER_HW, ADC_TIMER_NUM, &ADC_TIMER_config);
	if(tcpwm_res == CY_TCPWM_SUCCESS)
	{
		Cy_TCPWM_Counter_Enable(ADC_TIMER_HW, ADC_TIMER_NUM);
	}
	else {goto return_err;}

	/*VREF*/
	aref_res = Cy_SysAnalog_Init(&ADC_VREF_config);
	if(aref_res == CY_SYSANALOG_SUCCESS)
	{
		Cy_SysAnalog_Enable();
	}
	else {goto return_err;}

	/*SAR*/
	sar_res = Cy_SAR_Init(SAR_ADC_HW, &SAR_ADC_config);
	if(sar_res == CY_SAR_SUCCESS)
	{
		Cy_SAR_Enable(SAR_ADC_HW);
	}
	else {goto return_err;}

	/*DMA*/
	/* Initialize descriptor 0 */
	dma_res = Cy_DMA_Descriptor_Init(&cpuss_0_dw0_0_chan_28_Descriptor_0,&cpuss_0_dw0_0_chan_28_Descriptor_0_config);
	if(dma_res != CY_DMA_SUCCESS) {goto return_err;}

	/* Initialize the channel and associate the descriptor to it */
	dma_res = Cy_DMA_Channel_Init(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL,&cpuss_0_dw0_0_chan_28_channelConfig);
	if(dma_res != CY_DMA_SUCCESS) {goto return_err;}

	/*Get the pointer to the SAR measurement data*/
	uint32_t *sar_data_ptr = (uint32_t*)&SAR_CHAN_RESULT(SAR_ADC_HW, 0);

    /* Set DMA Source and Destination address */
    Cy_DMA_Descriptor_SetSrcAddress(&cpuss_0_dw0_0_chan_28_Descriptor_0, sar_data_ptr);
    Cy_DMA_Descriptor_SetDstAddress(&cpuss_0_dw0_0_chan_28_Descriptor_0, sar_buffer);

    /*Set DMA Descriptor */
    Cy_DMA_Channel_SetDescriptor(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL, &cpuss_0_dw0_0_chan_28_Descriptor_0);

    /* Initialize and enable the interrupt from SAR DMA */
    Cy_SysInt_Init(&intRxDma_cfg, &dma_interrupt);
    NVIC_EnableIRQ((IRQn_Type)intRxDma_cfg.intrSrc);

    /* Enable DMA interrupt source. */
    Cy_DMA_Channel_SetInterruptMask(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL, CY_DMA_INTR_MASK);

    /* Enable DMA Channel and DMA Block to start descriptor execution process */
    Cy_DMA_Channel_Enable(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL);
    Cy_DMA_Enable(cpuss_0_dw0_0_chan_28_HW);

	return CY_RSLT_SUCCESS;

	return_err:
	return CY_RSLT_TYPE_ERROR;
}

void dma_interrupt(void)
{
    dmaIntrTriggered = true;

    /* Check interrupt cause to capture errors. */
    if (CY_DMA_INTR_CAUSE_COMPLETION == Cy_DMA_Channel_GetStatus(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL))
    {
        adc_dma_done = 1;
    }
    else if((CY_DMA_INTR_CAUSE_COMPLETION != Cy_DMA_Channel_GetStatus(cpuss_0_dw0_0_chan_28_HW,cpuss_0_dw0_0_chan_28_CHANNEL)) &&
                                                (CY_DMA_INTR_CAUSE_CURR_PTR_NULL !=
                                                Cy_DMA_Channel_GetStatus(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL)))
    {
        /* DMA error occurred while ADC operations */
        adc_dma_error = Cy_DMA_Channel_GetStatus(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL);
    }

    /* Clear SAR DMA interrupt */
    Cy_DMA_Channel_ClearInterrupt(cpuss_0_dw0_0_chan_28_HW, cpuss_0_dw0_0_chan_28_CHANNEL);
 }

/* [] END OF FILE */

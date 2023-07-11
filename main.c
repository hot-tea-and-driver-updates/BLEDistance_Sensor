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

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cy_scb_uart.h"
#include "cycfg.h"
#include "cycfg_ble.h"
#include "cy_syslib.h"
#include "cy_sysint.h"
#include <stdio.h>

//global variables//
static uint8_t alertLevel = CY_BLE_NO_ALERT;
cy_stc_ble_conn_handle_t app_conn_handle;
/* The variables to initialize the BLE stack timer to get a 1-second interval */
static cy_stc_ble_timer_info_t  timerParam = { .timeout = 1u /* second */ };
static uint8_t                  mainTimer = 1u;
cy_stc_scb_i2c_context_t I2C_context;
uint32_t distance;
uint32_t count;
uint32_t pinStateZero = 0;
uint32_t pinStateOne = 1;
uint8_t distanceGlobal;
uint8_t printflag = 0;

/* BLESS interrupt configuration.
 * It is used when BLE middleware operates in BLE Single CM4 Core mode. */
const cy_stc_sysint_t blessIsrCfg =
{
    /* The BLESS interrupt */
    .intrSrc      = bless_interrupt_IRQn,

    /* The interrupt priority number */
    .intrPriority = 2u
};

cy_stc_ble_gatt_handle_value_pair_t BLEconst =
{
        .value.val = &distanceGlobal,
        .value.len = 1,
        .attrHandle = CY_BLE_CUSTOM_SERVICE_CUSTOM_CHARACTERISTIC_CHAR_HANDLE
};

cy_stc_sysint_t intrCfg =
{	/*refers to port 0, where the button connects to */
		.intrSrc = ioss_interrupts_gpio_9_IRQn,
		.intrPriority = 4u
};



/* Assign pins for UART on SCB5: P5[0], P5[1] */
#define UART_PORT       P5_0_PORT
#define UART_RX_NUM     P5_0_NUM
#define UART_TX_NUM     P5_1_NUM

/* Assign divider type and number for UART */
#define UART_CLK_DIV_TYPE     (CY_SYSCLK_DIV_16_BIT)
#define UART_CLK_DIV_NUMBER   (11U)

/*Function is called to setup UART at beginning of file*/
void UARTsetup(void);
void AppCallBack(uint32_t event, void *eventParam);
void IasEventHandler(uint32_t event, void *eventParam);
void BlessInterrupt(void);
void pwmInterrupt_Handler(void);

void pwmInterrupt_Handler(void){
	uint32_t distance;
	//printf("Currently in Interrupt..\r\n");
    //Cy_TCPWM_Counter_SetCounter(tcpwm_1_cnt_0_HW,tcpwm_1_cnt_0_NUM,0);
	if(Cy_GPIO_Read(ECHO_PORT, ECHO_NUM) == 1UL){
        Cy_TCPWM_TriggerStart(tcpwm_1_cnt_0_HW, tcpwm_1_cnt_0_MASK);
        Cy_GPIO_ClearInterrupt(ECHO_PORT, ECHO_PIN);
    }else{
        Cy_TCPWM_TriggerStopOrKill(tcpwm_1_cnt_0_HW,tcpwm_1_cnt_0_MASK);
        uint32_t count = Cy_TCPWM_Counter_GetCounter(tcpwm_1_cnt_0_HW, tcpwm_1_cnt_0_NUM);
        distance = ((0.0343 * count)/(2));
        distanceGlobal = distance;
        //printf("%lu\r\n",(unsigned int)distance);
        //char buf[30];
        //sprintf(buf, "%d cm      ",distance);
        Cy_TCPWM_Counter_SetCounter(tcpwm_1_cnt_0_HW,tcpwm_1_cnt_0_NUM,0);
        Cy_GPIO_ClearInterrupt(ECHO_PORT, ECHO_PIN);
        Cy_GPIO_Write(TRIGGER_PORT, TRIGGER_NUM, pinStateOne);
        Cy_SysLib_DelayUs(10);
        Cy_GPIO_Write(TRIGGER_PORT, TRIGGER_NUM, pinStateZero);
    }
    Cy_TCPWM_Counter_SetCounter(tcpwm_1_cnt_0_HW,tcpwm_1_cnt_0_NUM,0);
}

int main(void)
{
    cy_rslt_t result;
    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    init_cycfg_all();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();
    UARTsetup();
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                     CY_RETARGET_IO_BAUDRATE);

    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    if (CY_RSLT_SUCCESS != Cy_TCPWM_Counter_Init(tcpwm_1_cnt_0_HW, tcpwm_1_cnt_0_NUM, &tcpwm_1_cnt_0_config)){
    	CY_ASSERT(0);
    }
    Cy_TCPWM_Counter_Enable(tcpwm_1_cnt_0_HW, tcpwm_1_cnt_0_NUM);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("Initializing UART communication... \r\n\n");
    printf("Lab 4.4 BLE \r\n\n");

    (void) Cy_SysInt_Init(&blessIsrCfg, &BlessInterrupt);\

    cy_ble_config.hw->blessIsrConfig = &blessIsrCfg;

    Cy_BLE_RegisterEventCallback(AppCallBack);


    Cy_BLE_Init(&cy_ble_config);


    Cy_BLE_Enable();

    Cy_BLE_IAS_RegisterAttrCallback(IasEventHandler);



    //interrupts setup
    Cy_SysInt_Init(&intrCfg, pwmInterrupt_Handler);
    NVIC_EnableIRQ(intrCfg.intrSrc);
    NVIC_ClearPendingIRQ(intrCfg.intrSrc);

    Cy_GPIO_Write(TRIGGER_PORT, TRIGGER_NUM, 1UL);
    Cy_SysLib_DelayUs(10);
    Cy_GPIO_Write(TRIGGER_PORT, TRIGGER_NUM, 0UL);

    for (;;)
    {
    	Cy_BLE_ProcessEvents();
    	if(printflag == 0){
    		if(CY_BLE_CONN_STATE_CONNECTED == Cy_BLE_GetConnectionState(app_conn_handle)){
    			printf("Device connected..\r\n");
    			printflag = 1;
    		}
    	}
    	if(printflag == 1){
    		if(CY_BLE_CONN_STATE_CONNECTED != Cy_BLE_GetConnectionState(app_conn_handle)){
    		printf("Device disconnected..\r\n");
    		printflag = 0;
    		}
    	}
    }
}

void AppCallBack(uint32_t event, void *eventParam){
	//printf("\rGot to appcallback..\r\n");
    switch(event){
    case CY_BLE_EVT_STACK_ON:
        /* Enter into discoverable mode so that remote can find it */
        Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,
                                       CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
        //printf("Started advertising..\r\n");
        break;
    case  CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
         if (Cy_BLE_GetAdvertisementState() == CY_BLE_ADV_STATE_STOPPED)
         {
             /* The fast and slow advertising period is complete, go to Low-power
             * mode (Hibernate) and wait for an external user event to wake up
             * the device again */
             Cy_BLE_Disable();

             printf("Stopped advertising..\r\n");
         }
         break;
    case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
        /* Start BLE advertising for 30 seconds and update the link status on the LEDs */
        Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,
                                       CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);

        /* Set default value for alert level (CY_BLE_NO_ALERT) */
        alertLevel = CY_BLE_NO_ALERT;

        break;
    case CY_BLE_EVT_TIMEOUT:
        if( (((cy_stc_ble_timeout_param_t *)eventParam)->reasonCode == CY_BLE_GENERIC_APP_TO) &&
            (((cy_stc_ble_timeout_param_t *)eventParam)->timerHandle == timerParam.timerHandle) )
        {
            /* Indicate that the timer is raised to the main loop */
            mainTimer++;

            /* Update the LED state */
            if(Cy_BLE_GetAdvertisementState() == CY_BLE_ADV_STATE_ADVERTISING)
            {
                /* Advertising state */
            }
            else if(Cy_BLE_GetNumOfActiveConn() == 0u)
            {
                /* Disconnected state */
            }
            else
            {
                /* Connected state: update the alert level value LED8 */
            }
        }
        break;
    case CY_BLE_EVT_STACK_SHUTDOWN_COMPLETE:
        /* Set the device into Hibernate mode */
        Cy_SysPm_SystemEnterHibernate();
        break;
    case CY_BLE_EVT_GAP_AUTH_REQ:
        if(cy_ble_config.authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX].security ==
            (CY_BLE_GAP_SEC_MODE_1 | CY_BLE_GAP_SEC_LEVEL_1))
        {
            cy_ble_config.authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX].authErr =
                CY_BLE_GAP_AUTH_ERROR_PAIRING_NOT_SUPPORTED;
        }

        cy_ble_config.authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX].bdHandle =
            ((cy_stc_ble_gap_auth_info_t *)eventParam)->bdHandle;

        if (Cy_BLE_GAPP_AuthReqReply(&cy_ble_config.authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX]) !=
                CY_BLE_SUCCESS)
        {
            Cy_BLE_GAP_RemoveOldestDeviceFromBondedList();
            Cy_BLE_GAPP_AuthReqReply(&cy_ble_config.authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX]);
        }
        break;
     case CY_BLE_EVT_GATTS_READ_CHAR_VAL_ACCESS_REQ:
                 Cy_BLE_GATTS_WriteAttributeValueLocal(&BLEconst);
                 break;
    }
}

void BlessInterrupt(void)
{
    /* Call interrupt processing */
    Cy_BLE_BlessIsrHandler();
}

void UARTsetup(void){
	/* Allocate context for UART operation */
	cy_stc_scb_uart_context_t uartContext;

	/* Populate configuration structure */
	const cy_stc_scb_uart_config_t uartConfig =
	{
	    .uartMode                   = CY_SCB_UART_STANDARD,
	    .enableMutliProcessorMode   = false,
	    .smartCardRetryOnNack       = false,
	    .irdaInvertRx               = false,
	    .irdaEnableLowPowerReceiver = false,
	    .oversample                 = 8UL,
	    .enableMsbFirst             = false,
	    .dataWidth                  = 8UL,
	    .parity                     = CY_SCB_UART_PARITY_NONE,
	    .stopBits                   = CY_SCB_UART_STOP_BITS_1,
	    .enableInputFilter          = false,
	    .breakWidth                 = 11UL,
	    .dropOnFrameError           = false,
	    .dropOnParityError          = false,
	    .receiverAddress            = 0UL,
	    .receiverAddressMask        = 0UL,
	    .acceptAddrInFifo           = false,
	    .enableCts                  = false,
	    .ctsPolarity                = CY_SCB_UART_ACTIVE_LOW,
	    .rtsRxFifoLevel             = 0UL,
	    .rtsPolarity                = CY_SCB_UART_ACTIVE_LOW,
	    .rxFifoTriggerLevel = 63UL,
	    .rxFifoIntEnableMask = 0UL,
	    .txFifoTriggerLevel = 63UL,
	    .txFifoIntEnableMask = 0UL,
	};

	/* Connect SCB5 UART function to pins */
	Cy_GPIO_SetHSIOM(UART_PORT, UART_RX_NUM, P5_0_SCB5_UART_RX);
	Cy_GPIO_SetHSIOM(UART_PORT, UART_TX_NUM, P5_1_SCB5_UART_TX);

	/* Configure pins for UART operation */
	Cy_GPIO_SetDrivemode(UART_PORT, UART_RX_NUM, CY_GPIO_DM_HIGHZ);
	Cy_GPIO_SetDrivemode(UART_PORT, UART_TX_NUM, CY_GPIO_DM_STRONG_IN_OFF);


	/* Connect assigned divider to be a clock source for UART */
	Cy_SysClk_PeriphAssignDivider(PCLK_SCB5_CLOCK, UART_CLK_DIV_TYPE, UART_CLK_DIV_NUMBER);

	/* UART desired baud rate is 115200 bps (Standard mode).
	* The UART baud rate = (clk_scb / Oversample).
	* For clk_peri = 50 MHz, select divider value 36 and get SCB clock = (50 MHz / 36) = 1,389 MHz.
	* Select Oversample = 12. These setting results UART data rate = 1,389 MHz / 12 = 115750 bps.
	*/
	Cy_SysClk_PeriphSetDivider   (UART_CLK_DIV_TYPE, UART_CLK_DIV_NUMBER, 77U);
	Cy_SysClk_PeriphEnableDivider(UART_CLK_DIV_TYPE, UART_CLK_DIV_NUMBER);

	Cy_SCB_UART_Enable(SCB5);
}

void IasEventHandler(uint32_t event, void *eventParam)
{
    /* Alert Level Characteristic write event */
    if(event == CY_BLE_EVT_IASS_WRITE_CHAR_CMD)
    {
        /* Read the updated Alert Level value from the GATT database */
        Cy_BLE_IASS_GetCharacteristicValue(CY_BLE_IAS_ALERT_LEVEL,
                                           sizeof(alertLevel),
                                           &alertLevel);
    }

    (void) eventParam;
}



/* [] END OF FILE */

/*
Amazon FreeRTOS
Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
Copyright 2016-2017 NXP

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 http://aws.amazon.com/freertos
 http://www.FreeRTOS.org
*/

///////////////////////////////////////////////////////////////////////////////
//  Includes
///////////////////////////////////////////////////////////////////////////////

/* SDK Included Files */
#include "board.h"
#include "fsl_debug_console.h"
#include "ksdk_mbedtls.h"

#include "pin_mux.h"
#include <stdbool.h>

/* Amazon FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "aws_clientcredential.h"
#include "aws_logging_task.h"
//#include "aws_wifi.h"
#include "aws_system_init.h"
#include "aws_test_runner.h"
#include "aws_dev_mode_key_provisioning.h"

#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include "lwip/prot/dhcp.h"
#include "netif/ethernet.h"
#include "ethernetif.h"
#include "lwip/netifapi.h"

#define INIT_SUCCESS 0
#define INIT_FAIL 1

struct netif fsl_netif0;

#include "fsl_device_registers.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* MAC address configuration. */
#define configMAC_ADDR                     \
    {                                      \
        0x02, 0x12, 0x13, 0x10, 0x15, 0x11 \
    }

/* Address of PHY interface. */
#define EXAMPLE_PHY_ADDRESS BOARD_ENET0_PHY_ADDRESS

/* System clock name. */
#define EXAMPLE_CLOCK_NAME kCLOCK_CoreSysClk

/* The lpc54018 usb logging driver calls a blocking write function. Since this
 * task is the lowest priority, all of the test task priorities must be higher than
 * this to run. */
#define mainLOGGING_TASK_PRIORITY (tskIDLE_PRIORITY)
#define mainLOGGING_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 4)
#define mainLOGGING_QUEUE_LENGTH (16)

/* Test runner task defines. */
#define mainTEST_RUNNER_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 16)

/* The task delay for allowing the lower priority logging task to print out Wi-Fi
 * failure status before blocking indefinitely. */
#define mainLOGGING_WIFI_STATUS_DELAY pdMS_TO_TICKS(1000)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

// static void prvWifiConnect(void);

/*******************************************************************************
 * Code
 ******************************************************************************/


int main(void)
{
    SYSMPU_Type *base = SYSMPU;
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    /* Disable SYSMPU. */
    base->CESR &= ~SYSMPU_CESR_VLD_MASK;
    CRYPTO_InitHardware();

    xLoggingTaskInitialize(mainLOGGING_TASK_STACK_SIZE, mainLOGGING_TASK_PRIORITY, mainLOGGING_QUEUE_LENGTH);

    vTaskStartScheduler();
    for (;;)
        ;
}

#if defined(__GNUC__)
extern uint32_t __extra_bss_start__[];
extern uint32_t __extra_bss_end__[];
void SystemInitHook(void)
{
    /* 
       Newlib startup is not able to handle non-continuous space for .bss
       sections, so this routine handles extra .bss segment, defined by 
      (__extra_bss) symbols in linker script. Both adrresses expects 4B 
      alignment for performance reason. Set both addresses to zero to 
      skip the extra init code. Note: At this point standard global variables
      are not initialized.
    */
    /* Return if segment is unused */
    if (NULL == __extra_bss_start__ || NULL == __extra_bss_end__)
        return;
    /* Expects 4B alignment, copy 4B in single instruction in each loop */
    for(uint32_t * extra_cursor  = __extra_bss_start__;
        extra_cursor < __extra_bss_end__;
        extra_cursor++)
    {
        *extra_cursor = 0;
    }
}
#endif

int initNetwork(void)
{
    ip4_addr_t fsl_netif0_ipaddr, fsl_netif0_netmask, fsl_netif0_gw;
    ethernetif_config_t fsl_enet_config0 = {
        .phyAddress = EXAMPLE_PHY_ADDRESS, .clockName = EXAMPLE_CLOCK_NAME, .macAddress = configMAC_ADDR,
    };

    IP4_ADDR(&fsl_netif0_ipaddr, 0, 0, 0, 0);
    IP4_ADDR(&fsl_netif0_netmask, 0, 0, 0, 0);
    IP4_ADDR(&fsl_netif0_gw, 0, 0, 0, 0);

    tcpip_init(NULL, NULL);

    netifapi_netif_add(&fsl_netif0, &fsl_netif0_ipaddr, &fsl_netif0_netmask, &fsl_netif0_gw, &fsl_enet_config0,
                       ethernetif0_init, tcpip_input);
    netifapi_netif_set_default(&fsl_netif0);
    netifapi_netif_set_up(&fsl_netif0);

    configPRINTF(("Getting IP address from DHCP ...\r\n"));
    netifapi_dhcp_start(&fsl_netif0);

    struct dhcp *dhcp;
    dhcp = (struct dhcp *)netif_get_client_data(&fsl_netif0, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP);

    while (dhcp->state != DHCP_STATE_BOUND)
    {
        vTaskDelay(1000);
    }

    if (dhcp->state == DHCP_STATE_BOUND)
    {
        configPRINTF(("IPv4 Address: %u.%u.%u.%u\r\n", ((u8_t *)&fsl_netif0.ip_addr.addr)[0],
                      ((u8_t *)&fsl_netif0.ip_addr.addr)[1], ((u8_t *)&fsl_netif0.ip_addr.addr)[2],
                      ((u8_t *)&fsl_netif0.ip_addr.addr)[3]));
    }
    configPRINTF(("DHCP OK\r\n"));

    return INIT_SUCCESS;
}

void vApplicationDaemonTaskStartupHook(void)
{
    if (SYSTEM_Init() == pdPASS)
    {
        initNetwork();

        /* A simple example to demonstrate key and certificate provisioning in
         * microcontroller flash using PKCS#11 interface. This should be replaced
         * by production ready key provisioning mechanism. */
        vDevModeKeyProvisioning();

        /* Create the task to run tests. */
        xTaskCreate(TEST_RUNNER_RunTests_task, "TestRunner", mainTEST_RUNNER_TASK_STACK_SIZE, NULL,
                    tskIDLE_PRIORITY + 1, NULL);
    }
}

/*-----------------------------------------------------------*/

#if 0
static void prvWifiConnect(void)
{
    WIFINetworkParams_t xNetworkParams;
    WIFIReturnCode_t xWifiStatus;
    uint8_t ucTempIp[4];

    /* Initialize Network params. */
    xNetworkParams.pcSSID = clientcredentialWIFI_SSID;
    xNetworkParams.ucSSIDLength = sizeof(clientcredentialWIFI_SSID);
    xNetworkParams.pcPassword = clientcredentialWIFI_PASSWORD;
    xNetworkParams.ucPasswordLength = sizeof(clientcredentialWIFI_PASSWORD);
    xNetworkParams.xSecurity = clientcredentialWIFI_SECURITY;
    xNetworkParams.cChannel = 0;

    configPRINTF(("Starting Wi-Fi...\r\n"));

    xWifiStatus = WIFI_On();
    if (xWifiStatus == eWiFiSuccess)
    {
        configPRINTF(("Wi-Fi module initialized. Connecting to AP %s...\r\n", xNetworkParams.pcSSID));
    }
    else
    {
        configPRINTF(("Wi-Fi module failed to initialize.\r\n"));

        /* Delay to allow the lower priority logging task to print the above status. */
        vTaskDelay(pdMS_TO_TICKS(mainLOGGING_WIFI_STATUS_DELAY));

        while (1)
        {
        }
    }

    xWifiStatus = WIFI_ConnectAP(&xNetworkParams);
    if (xWifiStatus == eWiFiSuccess)
    {
        configPRINTF(("Wi-Fi connected to AP %s.\r\n", xNetworkParams.pcSSID));

        xWifiStatus = WIFI_GetIP(ucTempIp);
        if (eWiFiSuccess == xWifiStatus)
        {
            configPRINTF(("IP Address acquired %d.%d.%d.%d\r\n", ucTempIp[0], ucTempIp[1], ucTempIp[2], ucTempIp[3]));
        }
    }
    else
    {
        configPRINTF(("Wi-Fi failed to connect to AP %s.\r\n", xNetworkParams.pcSSID));

        /* Delay to allow the lower priority logging task to print the above status. */
        vTaskDelay(pdMS_TO_TICKS(mainLOGGING_WIFI_STATUS_DELAY));

        while (1)
        {
        }
    }
}
#endif

/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
 * implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
 * used by the Idle task. */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    /* If the buffers to be provided to the Idle task are declared inside this
     * function then they must be declared static - otherwise they will be allocated on
     * the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle
     * task's state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
 * application must provide an implementation of vApplicationGetTimerTaskMemory()
 * to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
    /* If the buffers to be provided to the Timer task are declared inside this
     * function then they must be declared static - otherwise they will be allocated on
     * the stack and so not exists after this function exits. */
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
     * task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

/*-----------------------------------------------------------*/

/**
 * @brief Warn user if pvPortMalloc fails.
 *
 * Called if a call to pvPortMalloc() fails because there is insufficient
 * free memory available in the FreeRTOS heap.  pvPortMalloc() is called
 * internally by FreeRTOS API functions that create tasks, queues, software
 * timers, and semaphores.  The size of the FreeRTOS heap is set by the
 * configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h.
 *
 */
void vApplicationMallocFailedHook()
{
    /* The TCP tests will test behavior when the entire heap is allocated. In
     * order to avoid interfering with those tests, this function does nothing. */
}

/*-----------------------------------------------------------*/

/**
 * @brief Loop forever if stack overflow is detected.
 *
 * If configCHECK_FOR_STACK_OVERFLOW is set to 1,
 * this hook provides a location for applications to
 * define a response to a stack overflow.
 *
 * Use this hook to help identify that a stack overflow
 * has occurred.
 *
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    portDISABLE_INTERRUPTS();

    /* Loop forever */
    for (;;)
        ;
}

/*-----------------------------------------------------------*/

void *pvPortCalloc(size_t xNum, size_t xSize)
{
    void *pvReturn;

    pvReturn = pvPortMalloc(xNum * xSize);
    if (pvReturn != NULL)
    {
        memset(pvReturn, 0x00, xNum * xSize);
    }

    return pvReturn;
}

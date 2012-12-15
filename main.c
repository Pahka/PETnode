/*
 * Copyright 2012 Pekka Nikander.  See NOTICE for licensing information.
 *
 * This is a temporary main file, for prototyping.
 */

#include <stm32f0xx.h>

#include <assert.h>
#include <stddef.h>

#include <uip.h>
#include <uip_arp.h>

#include <enc28j60.h>

#include "enc28j60-conf.h"
#include "contiki-conf.h"

#if CONTIKI
#include <core/sys/pt.h>
#include <core/sys/process.h>
#include <core/net/tcpip.h>
#include <core/sys/etimer.h>
#include <apps/webserver/webserver-nogui.h>
#include <core/sys/clock.h>
#endif

#include "init.h"

#include "debug.h"

void abort(void) {
    for (;;)
        ;
}

static __IO uint32_t TimingDelay;

void Delay(__IO uint32_t nTime)
{
#if CONTIKI
    static clock_time_t prevTime;
    if (prevTime == 0)
        prevTime = clock_time();
#endif
    uint32_t prevDelay;
    prevDelay = TimingDelay = nTime;

    while (TimingDelay > 0) {
        if (prevDelay != TimingDelay) {
            /*
             * Process Ethernet packets
             */
#define BUF ((struct uip_eth_hdr *)&uip_buf[0])
            DEBUG_SET_LED0(1);
            DEBUG_SET_LED1(1);
            uip_len = enc_packet_receive(uip_buf, sizeof(uip_buf));
            DEBUG_SET_LED1(0);
            if (uip_len > 0) {
                if (BUF->type == UIP_HTONS(UIP_ETHTYPE_IP)) {
                    uip_arp_ipin();
                    DEBUG_SET_LED2(1);
                    uip_input();
                    DEBUG_SET_LED2(0);
                    if (uip_len > 0) {
                        uip_arp_out();
                    }
                } else if (BUF->type == UIP_HTONS(UIP_ETHTYPE_ARP)) {
                    uip_arp_arpin();
                }
                if (uip_len > 0) {
                    DEBUG_SET_LED4(1);
                    enc_packet_send(uip_buf, uip_len);
                    DEBUG_SET_LED4(0);
                }
            }
            DEBUG_SET_LED0(0);

#ifdef CONTIKI
            /*
             * Process upper-layer processes
             */
            DEBUG_SET_LED2(1);  // Yellow
            while (process_run() > 0)
                ;
            DEBUG_SET_LED2(0);
            if (clock_time() > prevTime + 50) {
#endif
                /*
                 * Process TCP connections every 50ms
                 */
                DEBUG_SET_LED1(1);  // Green
                for (int i = 0; i < UIP_CONNS; ++i) {
                    GPIOC->ODR ^= GPIO_ODR_8;
                    uip_periodic(i);
                    GPIOC->ODR ^= GPIO_ODR_8;
                    if (uip_len > 0) {
                        uip_arp_out();
                        enc_packet_send(uip_buf, uip_len);
                    }
                }
                DEBUG_SET_LED1(0);  // Green
#if CONTIKI
                prevTime = clock_time();
            }
#endif
            prevDelay = TimingDelay;
        }
        /* XXX Sleep on semaphore. Simplest possible implementation. */
        // __WFI();
    }
    TimingDelay = 0;
}

void TimingDelay_Decrement(void)
{
    if (TimingDelay != 0x00) {
        TimingDelay--;
    } else {
        /* Wake up from semaphore. Automatic for WFI. */;
    }
}

#ifndef CONTIKI
void SysTick_Handler(void)
{
    TimingDelay_Decrement();
    // Could be more here
}
#endif

/*
 * The following derived from STM Copyrighted work.
 * If you remove that or modify it enough, please consider
 * removing the SMT copyright notice from the top of this file.
 */

extern __I uint8_t APBAHBPrescTable[16];

uint32_t
RCC_GetHCLKFreq() {
    /* Make sure we are running from PLL from HSI */
    assert((RCC->CFGR & RCC_CFGR_SWS)    == RCC_CFGR_SWS_PLL);
    assert((RCC->CFGR & RCC_CFGR_PLLSRC) == RCC_CFGR_PLLSRC_HSI_Div2);

    /* Get PLL clock multiplication factor ----------------------*/
    const uint32_t pllmull   = ((RCC->CFGR & RCC_CFGR_PLLMULL) >> 18) + 2;

    /* HSI oscillator clock divided by 2 as PLL clock entry */
    const uint32_t sysclk = (HSI_VALUE >> 1) * pllmull;

    /* HCLK clock frequency */
    return sysclk;
}

/*
 * End of STM derived work
 */
#if !CONTIKI
void tcpip_uipcall(void) {
}
#endif

int
main(void) {
    uint8_t BlinkSpeed = 0;

    /* SysTick end of count event each 1ms */
    SysTick_Config(RCC_GetHCLKFreq() / 1000); /* CMSIS */

    Peripheral_Init();

    GPIOC->ODR ^= GPIO_ODR_9;

    /* Enable ENC28J60 */
    GPIOA->BSRR = GPIO_BSRR_BS_3; /* Take out from reset */
    enc_init(uip_ethaddr.addr);

    uip_init();
    uip_ipaddr_t addr;
    uip_ipaddr(&addr, 10,0,0,2);
    uip_sethostaddr(&addr);

#if CONTIKI
    process_init();
    process_start(&etimer_process, NULL);
    process_start(&tcpip_process, NULL);
    process_start(&webserver_nogui_process, NULL);
#endif

    BlinkSpeed = 1;

    for (;;) {
        /* Check if the user button is pressed */
        if (GPIOA->IDR & GPIO_IDR_0) {

            /* BlinkSpeed: 1 -> 2 -> 3 -> 4 -> 1 then re-cycle */
            BlinkSpeed = BlinkSpeed % 4 + 1;

            DEBUG_SET_LED4(1);
            Delay(1000);
            DEBUG_SET_LED4(0);
        }
        GPIOC->ODR ^= GPIO_ODR_9;

        Delay(BlinkSpeed * 50);
    }
    /* NOTREACHED */
}

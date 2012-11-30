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

#include "init.h"

static __IO uint32_t TimingDelay;

#define DEBUG(data) (SPI1->DR = (uint16_t)(data))

void Delay(__IO uint32_t nTime)
{
    TimingDelay = nTime;

    while (TimingDelay != 0)
        /* Sleep on semaphore. Simplest possible implementation. */
        //__WFI()
        ;
}

void TimingDelay_Decrement(void)
{
    if (TimingDelay != 0x00) {
        TimingDelay--;
    } else {
        /* Wake up from semaphore. Automatic for WFI. */;
    }
}

void SysTick_Handler(void)
{
    TimingDelay_Decrement();
    // Could be more here
}

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

    /* Enable ENC28J60 */
    GPIOA->BSRR = GPIO_BSRR_BS_3; /* Take out from reset */
    enc_init(uip_ethaddr.addr);

    GPIOC->ODR ^= GPIO_ODR_9;

    uip_init();
    uip_ipaddr_t addr;
    uip_ipaddr(&addr, 10,0,0,2);
    uip_sethostaddr(&addr);

    {
        GPIOA->BSRR = GPIO_BSRR_BS_8;
        Delay(10);
        GPIOA->BSRR = GPIO_BSRR_BR_8;
    }

    /* XXX Replace with macro/inline */
    GPIOA->BSRR = GPIO_BSRR_BS_3; /* Enable ENC28J60 */

    BlinkSpeed = 1;

    for (;;) {
        /*
         * Process Ethernet packets
         */
 #define BUF ((struct uip_eth_hdr *)&uip_buf[0])
        uip_len = enc_packet_receive(uip_buf, sizeof(uip_buf));
        if (uip_len > 0) {
            if (BUF->type == UIP_HTONS(UIP_ETHTYPE_IP)) {
                uip_arp_ipin();
                uip_input();
                if (uip_len > 0) {
                    uip_arp_out();
                }
            } else if (BUF->type == UIP_HTONS(UIP_ETHTYPE_ARP)) {
                uip_arp_arpin();
            }
            if (uip_len > 0) {
                enc_packet_send(uip_buf, uip_len);
            }
        }

        /* Check if the user button is pressed */
        if (GPIOA->IDR & GPIO_IDR_0) {

            /* BlinkSpeed: 0 -> 1 -> 2 -> 3 -> 0, then re-cycle */
            BlinkSpeed = (BlinkSpeed + 1) % 4;

            GPIOC->BSRR = GPIO_BSRR_BS_8;
            Delay(1000);
            GPIOC->BSRR = GPIO_BSRR_BR_8;
        }
        GPIOC->ODR ^= GPIO_ODR_9;

        Delay(BlinkSpeed * 10);
    }
    /* NOTREACHED */
}

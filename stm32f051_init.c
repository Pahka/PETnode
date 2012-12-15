/*
 * Copyright 2012 Pekka Nikander.  See NOTICE for licensing information.
 */

/*
 * A data driven approach to initialise STM32F peripherals.
 */

#include <stm32f0xx.h>
#include <stm32f0xx_extra.h>

#include <stddef.h>
#include <assert.h>

#include "init.h"

#define D16(d, r, v)    DEVICE_REGISTER_INIT_STRUCT_VALUE16((d), r, (v))
#define D32(d, r, v)    DEVICE_REGISTER_INIT_STRUCT_VALUE32((d), r, (v))
#define M32(d, r, m, v) DEVICE_REGISTER_INIT_STRUCT_MASK_VALUE32((d), r, (m), (v))


/* XXX: Do we really need mask here?   */
const struct device_register_init_masked_32bit reset_and_clock_control[] = {
    M32(RCC, APB1ENR,
        RCC_APB1ENR_TIM2EN,
        RCC_APB1ENR_TIM2EN),
    M32(RCC, APB2ENR,
        RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_SPI1EN,
        RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_SPI1EN),
    M32(RCC, AHBENR,
        RCC_AHBENR_GPIOAEN   | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOFEN,
        RCC_AHBENR_GPIOAEN   | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOFEN),
};

/*
 * Assign a given priority for an interrupt, and enable the interrupt.
 *
 * See page 4-7 of ARM DUI 0497A, Cortex-M0 Peripherals and
 * NVIC_SetPriority implementation in CMSIS/Include/core_cm0.h
 */
#define _NVIC_PRIO_SHIFT(prio) ((prio) << (8 - __NVIC_PRIO_BITS))
#define ENABLE_INTERRUPT(irq, prio)                                 \
    M32(NVIC, IP[_IP_IDX((irq))],                                   \
        (                         0x000000FF) << _BIT_SHIFT((irq)), \
        (_NVIC_PRIO_SHIFT(prio) & 0x000000FF) << _BIT_SHIFT((irq))  \
        ),                                                          \
    M32(NVIC, ISER[0],                                              \
        1 << (irq), /* NOP */                                       \
        1 << (irq)                                                  \
       )

const struct device_register_init_masked_32bit nvic[] = {
    ENABLE_INTERRUPT(SPI1_IRQn, 3), /* Lowest priority */
};

/* Port A */
const struct device_register_init_static_32bit general_purpose_io_a[] = {
    D32(GPIOA, MODER,
        0
        | ! GPIO_MODER_MODER0    /* 00: PA0  Input  USER */
        | ! GPIO_MODER_MODER1    /* 00: PA1  Input  INT */
#if 1
        | ! GPIO_MODER_MODER2    /* 00: PA2  Input  CLKOUT */
#else
        |   GPIO_MODER_MODER2_1  /* 10: PA2  AF     TIM XXX */
#endif
        |   GPIO_MODER_MODER3_0  /* 01: PA3  Output RESET */
        |   GPIO_MODER_MODER4_1  /* 10: PA4  AF     SPI NSS */
        |   GPIO_MODER_MODER5_1  /* 10: PA5  AF     SPI CLK */
        |   GPIO_MODER_MODER6_1  /* 10: PA6  AF     SPI MISO */
        |   GPIO_MODER_MODER7_1  /* 10: PA7  AF     SPI MOSI */
        |   GPIO_MODER_MODER9_1  /* 10: PA9  AF     USART1 TX */
        |   GPIO_MODER_MODER10_1 /* 10: PA10 AF     USART1_RX */
        |   GPIO_MODER_MODER13_1 /* 10: PA13 AF     SWDAT, reset-time value */
        |   GPIO_MODER_MODER14_1 /* 10: PA14 AF     SWCLK, reset-time value */
        ),
    D32(GPIOA, OTYPER,
        0
        | ! GPIO_OTYPER_OT_3     /* PA3 GPIO output Push-pull */
        | ! GPIO_OTYPER_OT_4     /* PA4 SPI NSS     Push-pull */
        | ! GPIO_OTYPER_OT_5     /* PA5 SPI CLK     Push-pull */
        | ! GPIO_OTYPER_OT_7     /* PA7 SPI MOSI    Push-pull */
        | ! GPIO_OTYPER_OT_9     /* PA9 USART TX    Push-pull */
        ),
    /* See page 74 of DM39193 Doc ID 022265 Rev 3 STD32F051x data sheet */
    D32(GPIOA, OSPEEDR,
        0
        |   GPIO_OSPEEDER_OSPEEDR3 
        |   GPIO_OSPEEDER_OSPEEDR4
        |   GPIO_OSPEEDER_OSPEEDR5
        |   GPIO_OSPEEDER_OSPEEDR7
        |   GPIO_OSPEEDER_OSPEEDR9
        |   GPIO_OSPEEDER_OSPEEDR13 /* Reset-time value */
        | ! GPIO_OSPEEDER_OSPEEDR14 /* Reset-time value */
        ),
    D32(GPIOA, PUPDR,
        0
        |   GPIO_PUPDR_PUPDR0_1  /* PA0  <- USER    pull down */
        |   GPIO_PUPDR_PUPDR1_0  /* PA1  <- INT     pull up */
        |   GPIO_PUPDR_PUPDR2_1  /* PA2  <- CLKOUT  pull down */
        | ! GPIO_PUPDR_PUPDR3_0  /* PA3  -> RESET   no pull resistors */
        | ! GPIO_PUPDR_PUPDR3    /* PA4  -> CS      no pull resistors */
        | ! GPIO_PUPDR_PUPDR5    /* PA5  -> SCK     no pull resistors */
        |   GPIO_PUPDR_PUPDR6_1  /* PA6  <- MISO    pull down */
        | ! GPIO_PUPDR_PUPDR7    /* PA7  -> MOSI    no pull resistors */
        | ! GPIO_PUPDR_PUPDR9    /* PA9  -> TX      no pull resistors */
        |   GPIO_PUPDR_PUPDR10_1 /* PA10 <- RX      pull down */
        |   GPIO_PUPDR_PUPDR13_0 /* PA13 <> SWDAT   pull up,   reset-time value */
        |   GPIO_PUPDR_PUPDR14_1 /* PA10 <- SWCLK   pull down, reset-time value */
        ),
    D32(GPIOA, AFR[0],
        0
        | ! GPIO_AFRL_AFRL4      /* PA4  AF0 SPI */
        | ! GPIO_AFRL_AFRL5      /* PA5  AF0 SPI */
        | ! GPIO_AFRL_AFRL6      /* PA6  AF0 SPI */
        | ! GPIO_AFRL_AFRL7      /* PA7  AF0 SPI */
        ),
    D32(GPIOA, AFR[1],
        0
#define GPIO_AFRL_AFRL1_0            ((uint32_t)0x00000010)
#define GPIO_AFRL_AFRL2_0            ((uint32_t)0x00000100)
        |   GPIO_AFRL_AFRL1_0    /* PA9  AF1 TX */
        |   GPIO_AFRL_AFRL2_0    /* PA10 AF1 RX */
        | ! GPIO_AFRL_AFRL5      /* PA13 AF0 SWD, reset-time value */
        | ! GPIO_AFRL_AFRL6      /* PA14 AF0 SWC, reset-time value */
        ),
};


/* Port C -- Eval and debug functions */
const struct device_register_init_static_32bit general_purpose_io_eval[] = {
    D32(GPIOC, MODER,
        0
        |   GPIO_MODER_MODER6_0  /* 01: PC6  Output Led4 */
        |   GPIO_MODER_MODER7_0  /* 01: PC7  Output Led4 */
        |   GPIO_MODER_MODER8_0  /* 01: PC8  Output Led4 */
        |   GPIO_MODER_MODER9_0  /* 01: PC9  Output Led3 */
        |   GPIO_MODER_MODER10_0 /* 01: PC10 Output Led2 */
        |   GPIO_MODER_MODER11_0 /* 01: PC11 Output Led1 */
        |   GPIO_MODER_MODER12_0 /* 01: PC12 Output Led0 */
        | ! GPIO_MODER_MODER13   /* 00: PC13 Input  WOL  */
        ),
    D32(GPIOC, OTYPER,
        0
        | ! GPIO_OTYPER_OT_6     /* PC6  GPIO output Push-pull */
        | ! GPIO_OTYPER_OT_7     /* PC7  GPIO output Push-pull */
        | ! GPIO_OTYPER_OT_8     /* PC8  GPIO output Push-pull */
        | ! GPIO_OTYPER_OT_9     /* PC9  GPIO output Push-pull */
        | ! GPIO_OTYPER_OT_10    /* PC10 GPIO output Push-pull */
        | ! GPIO_OTYPER_OT_11    /* PC11 GPIO output Push-pull */
        | ! GPIO_OTYPER_OT_12    /* PC12 GPIO output Push-pull */
        ),
    /* See page 74 of DM39193 Doc ID 022265 Rev 3 STD32F051x data sheet */
    D32(GPIOC, OSPEEDR,
        0
        | ! GPIO_OSPEEDER_OSPEEDR6  /* 00: PC6 GPIO */
        | ! GPIO_OSPEEDER_OSPEEDR7  /* 00: PC7 GPIO */
        | ! GPIO_OSPEEDER_OSPEEDR8  /* 00: PC8 GPIO */
        | ! GPIO_OSPEEDER_OSPEEDR9  /* 00: PC9 GPIO */
        | ! GPIO_OSPEEDER_OSPEEDR10 /* 00: PC10 GPIO */
        | ! GPIO_OSPEEDER_OSPEEDR11 /* 00: PC11 GPIO */
        | ! GPIO_OSPEEDER_OSPEEDR12 /* 00: PC12 GPIO */
        ),
    D32(GPIOC, PUPDR,
        0
        | ! GPIO_PUPDR_PUPDR6    /* PC6  -> Led6    no pull resistors */
        | ! GPIO_PUPDR_PUPDR7    /* PC7  -> Led5    no pull resistors */
        | ! GPIO_PUPDR_PUPDR8    /* PC8  -> Led4    no pull resistors */
        | ! GPIO_PUPDR_PUPDR9    /* PC9  -> Led3    no pull resistors */
        | ! GPIO_PUPDR_PUPDR10   /* PC9  -> Led2    no pull resistors */
        | ! GPIO_PUPDR_PUPDR11   /* PC9  -> Led1    no pull resistors */
        | ! GPIO_PUPDR_PUPDR12   /* PC9  -> Led0    no pull resistors */
        | ! GPIO_PUPDR_PUPDR13_0 /* PC13 <- WOL     pull up*/
        ),
};

/*
 * Timer-counter for uIP and other timeouts.
 */
const struct device_register_init_static_16bit charger[] = {
#if 0
    XXX
    D16(TIM2, CR1, 0),        /* Disable the counter */
    D16(TIM2, CR2, 0),
    D16(TIM2, SMCR, 0),
    D16(TIM2, DIER, 0),
    D16(TIM2, CCMR1,
        0
        |   TIM_CCMR1_CC2S_0  /* Input TI2 = CH2 */
        | ! TIM_CCMR1_IC2PSC  /* No prescaler */
        | ! TIM_CCMR1_IC2F    /* No input filter */
        ),
    D16(TIM2, CCMR2, 0),
    D16(TIM2, CCER,
        0
        |   TIM_CCER_CC2E     /* Enable capture */
        |   TIM_CCER_CC2P_Falling
        ),

    /*
     * Note that TIM2 ARR and CCR registers are 32-bits long, but here we
     * initialize them with only 16-bits long values
     */
    D16(TIM2, ARR, 20000),    /* XXX */
    D16(TIM2, CCR4, 5000),    /* 25% */
    D16(TIM2, PSC, 0),        /* No prescaler */

    D16(TIM2, CR1,
        0
        | ! TIM_CR1_CKD       /* Clock divide by 1, i.e. zero value here */
        |   TIM_CR1_ARPE      /* Auto-reload preload enabled, ARR buffered */
        | ! TIM_CR1_CMS       /* Edge aligned mode */
        |   TIM_CR1_DIR       /* Down counter */
        | ! TIM_CR1_OPM       /* Continuous */
        |   TIM_CR1_CEN       /* Enable the counter */
        ),
#endif
};

/* SPI */
const struct device_register_init_static_16bit spi[] = {
    D16(SPI1, CR2,
        0
        | ! SPI_CR2_RXDMAEN    /* Disable RX DMA */
        | ! SPI_CR2_TXDMAEN    /* Disable TX DMA */
        |   SPI_CR2_SSOE       /* NSS output automatically managed by hardware */
        | ! SPI_CR2_NSSP       /* Default: NO NSS pulse */
        | ! SPI_CR2_FRF        /* SPI Motorola mode */
        | ! SPI_CR2_ERRIE      /* Error interrupt disabled */
        | ! SPI_CR2_RXNEIE     /* RX buffer not empty interrupt disabled */
        | ! SPI_CR2_TXEIE      /* TX buffer empty interrupt disabled */
        /* 0111: 8 bits data size */
        |   SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0
        | ! SPI_CR2_FRXTH      /* 0: RXNE goes high with two bytes */
        | ! SPI_CR2_LDMARX     /* N/A: RX DMA even */
        | ! SPI_CR2_LDMATX     /* N/A: TX DMA even */
        ),
    D16(SPI1, CR1,
        0
        | ! SPI_CR1_CPHA       /* Data at first edge */
        | ! SPI_CR1_CPOL       /* Clock low when idle */
        |   SPI_CR1_MSTR       /* Master mode */
        |   SPI_CR1_BR_1       /* Clock divider 8 */
        | ! SPI_CR1_SPE        /* SPI disabled until used */
        | ! SPI_CR1_LSBFIRST   /* MSB first */

        |   SPI_CR1_SSI        /* Internal NSS high, needed for master mode */
        | ! SPI_CR1_SSM        /* Hardware Slave management enabled */
        | ! SPI_CR1_RXONLY     /* 0: Full duplex */
        | ! SPI_CR1_CRCL       /* 0: N/A (8-bit CRC length) */
        | ! SPI_CR1_CRCNEXT    /* 0: Transmit TX buffer, not CERC */
        | ! SPI_CR1_CRCEN      /* 0: CRC disabled */
        |   SPI_CR1_BIDIOE     /* 1: Output enabled */
        | ! SPI_CR1_BIDIMODE   /* 0: 2-Line (uni)directional data */
        ),
};

/*
 * Master initialisation table
 */
const device_register_init_descriptor_t dri_tables[] = {
    DRI_DESCRIPTOR_MASKED_32BIT(RCC, reset_and_clock_control),
    DRI_DESCRIPTOR_MASKED_32BIT(NVIC, nvic),
    DRI_DESCRIPTOR_STATIC_32BIT(GPIOA, general_purpose_io_a),
    DRI_DESCRIPTOR_STATIC_32BIT(GPIOC, general_purpose_io_eval),
    DRI_DESCRIPTOR_STATIC_16BIT(SPI1, spi),
    DRI_DESCRIPTOR_STATIC_16BIT(TIM2, charger),
};

/*
 * The APB bus appears to be a 32-bit bus, always using 32-bit access.
 * There are four sets of evidence for this.  Firstly, using the
 * 32-bit writes below work even for 16-bit registers.  Secondly, all
 * the 16-bit registers are defined as the high-order 16 bits being
 * reserved.  Thirdly, at least some of the APB peripherial
 * descriptions say the following:
 *
 * RM0091, Doc ID 018940 Rev 2, page 332:
 *
 * The peripheral registers can be accessed by half-words (16-bit)
 * or words (32-bit).
 *
 * Finally, when describing the AHB2APB bridge and then the DMA
 * controller access to the APB devices, the manual says the following:
 *
 * RM0091, Doc ID 018940 Rev 2, page 36:
 *
 * Note: When a 16- or 8-bit access is performed on an APB register,
 * the access is transformed into a 32-bit access: the bridge
 * duplicates the 16- or 8-bit data to feed the 32-bit vector.
 *
 * RM0091, Doc ID 018940 Rev 2, page 149:
 *
 * Assuming that the AHB/APB bridge is an AHB 32-bit slave peripheral
 * that does not take the HSIZE data into account, it will transform
 * any AHB byte or halfword operation into a 32-bit APB operation in
 * the following manner:
 * - an AHB byte write operation of the data "0xB0" to 0x0 (or to 0x1,
 *   0x2 or 0x3) will be converted to an APB word write operation of
 *   the data "0xB0B0B0B0" to 0x0
 * - an AHB halfword write operation of the data "0xB1B0" to 0x0 (or
 *   to 0x2) will be converted to an APB word write operation of the
 *   data "0xB1B0B1B0" to 0x0
 *
 * Consequently, our current assumption is that it is safe to write
 * even the 16-bit APB registers with 32-bit writes, and we do so.
 */

static inline void
Config_Static16(void *device, const device_register_init_static_16bit_t *values, const int count) {
    const device_register_init_static_16bit_t *p;

    for (p = values; p < values + count; p++) {
        *((uint32_t *)(((char *)device) + p->offset)) = p->value;
    }
}

static inline void
Config_Static32(void *device, const device_register_init_static_32bit_t *values, const int count) {
    const device_register_init_static_32bit_t *p;

    for (p = values; p < values + count; p++) {
        *((uint32_t *)(((char *)device) + p->offset)) = p->value;
    }
}

static inline void
Config_Masked32(void *device, const device_register_init_masked_32bit_t *values, const int count) {
    const device_register_init_masked_32bit_t *p;

    for (p = values; p < values + count; p++) {
        register uint32_t *reg;              /* Pointer to the device register */
        register uint32_t tmp;

        reg = (uint32_t *)(((char *)device) + p->offset);

        // XXX: assert that no bits in p->value are outside of p->mask

        tmp  = *reg;
        tmp &= ~p->mask;
        tmp |=  p->value;
        *reg = tmp;
    }
}

void
Peripheral_Init(void) {
    for (unsigned int i = 0; i < COUNT_OF(dri_tables); i++) {
        const device_register_init_descriptor_t *dri = &dri_tables[i];
        switch (dri->dri_type) {
        case DRI_MASKED_32BIT:
            Config_Masked32(dri->dri_device, dri->dri_masked_32bit, dri->dri_count);
            break;
        case DRI_STATIC_32BIT:
            Config_Static32(dri->dri_device, dri->dri_static_32bit, dri->dri_count);
            break;
        case DRI_STATIC_16BIT:
            Config_Static16(dri->dri_device, dri->dri_static_16bit, dri->dri_count);
            break;
        default:
            abort();
            break;
        }
    }
}


/*
 * Copyright 2012 Pekka Nikander.  See NOTICE for licensing information.
 */

#include <stm32f0xx.h>

#include <stddef.h>

#include <enc28j60.h>
#include <enc28j60_spi.h>

#include "enc28j60-conf.h"

#include "init.h"

typedef struct enc28j60_register_init_static_8bit {
    enc_reg_t       reg;
    enc_reg_value_t value;
} device_register_init_static_8bit_t;

#define D8(r,    v) { .reg = (r),              .value = (v) }

#define D16(r, v) \
    D8(r ## _L, (v) & 0xff), \
    D8(r ## _H, (v) >> 8)


static const struct enc28j60_register_init_static_8bit enc28j60_init[] = {

    /**********************************
     * Bank0: RX and TX buffers, DMA
     **********************************/

    /* Buffer transfer */
    D16(E_RD_PTR,    RX_BUFFER_START),
    D16(E_WR_PTR,    TX_BUFFER_START),
    /* TX buffer */
    D16(E_TX_STA,    TX_BUFFER_START),
    D16(E_TX_END,    TX_BUFFER_END),
    /* RX buffer */
    D16(E_RX_STA,    RX_BUFFER_START),
    D16(E_RX_END,    RX_BUFFER_END),
    D16(E_RX_RD_PTR, RX_BUFFER_START),
    // E_RX_WR_PTR is read only
    /* DMA: not used in this application */

    /**********************************
     * Bank1: Packet filtering
     **********************************/

    /*
     * Hash-table not used
     */
    /*
     * Pattern matching broadcast packet filter for ARP
     */
    D8 (E_PM_M0, 0x3F),   /* Match destination Ethernet address (6 bytes) */
    D8 (E_PM_M1, 0x30),   /* Match ARP Ethertype 0x0608 */
    // All the rest of E_PM_M0 in their reset values */
    D16(E_PM_CHSUM, 0xf7f9), /* IP checksum for ff ff ff ff ff ff 06 08 */
    D16(E_PM_OFF,  0x0000),  /* Match from the beginning of the packet */
    /*
     * Wake-on-LAN
     */
#ifdef  XXX
    XXX
#endif
    /*
     * Filtering configuration.
     */
    D8(E_RX_FCOND,
       0
       |   E_RX_FCOND_UC_EN  /* 1: Unicast without our MAC discarded */
       | ! E_RX_FCOND_AND_OR /* 0: User OR filtering */
       |   E_RX_FCOND_CRC_EN /* 1: Enablve post-filter CRC check */
       |   E_RX_FCOND_PM_EN  /* 1: Pattern match filter enabled for ARP */
       | ! E_RX_FCOND_MP_EN  /* 0: Magic packet filter disabled */
       | ! E_RX_FCOND_HT_EN  /* 0: Hash-table filter disabled */
       | ! E_RX_FCOND_MC_EN  /* 0: Multicast disabled */
       |   E_RX_FCOND_BC_EN  /* 1: Broadcast enabled */
      ),

    /**********************************
     * Bank3: MAC & MII
     **********************************/
    D8(MAC_CON1,
       0
       | ! MAC_CON1_LOOP_BACK /* 0: No loopback */
       |   MAC_CON1_TX_PAUSE  /* 1: Allow sending pause control frames */
       |   MAC_CON1_RX_PAUSE  /* 1: Adhere received pause control frames */
       | ! MAC_CON1_PASS_ALL  /* 0: Don't pass control frames to host */
       |   MAC_CON1_RX_EN     /* 1: Enable the MAC */
      ),
    D8(MAC_CON2,
       0
       | ! MAC_CON2_MAC_RST    /* 0: Enable MAC */
       | ! MAC_CON2_RND_RST    /* 0: Enable MAC random number generator */
       | ! MAC_CON2_RX_RST     /* 0: Enable MAC receive logic */
       | ! MAC_CON2_RX_FUN_RST /* 0: Enable MAC packet receive */
       | ! MAC_CON2_TX_RST     /* 0: Enable MAC transmit logic */
       | ! MAC_CON2_TX_FUN_RST /* 0: Enable MAC packet transmit */
      ),
    D8(MAC_CON3,
       0
       | ! MAC_CON3_PAD_CRC_2   /* 001: Pad to 60 bytes and add CRC */
       | ! MAC_CON3_PAD_CRC_1
       |   MAC_CON3_PAD_CRC_0
       |   MAC_CON3_TX_CRC_EN   /* 1: Enable CRC for Transmit */
       | ! MAC_CON3_PHDR_LEN    /* 0: No proprietary header */
       | ! MAC_CON3_HUGE_FRAMES /* 0: No proprietary header */
       |   MAC_CON3_FR_LEN_CHK  /* 1: Check frame lengths */
       | ! MAC_CON3_FULL_DPX    /* 0: No full duplex (yet) XXX */
      ),
    D8(MAC_CON4,
       0
       | ! MAC_CON4_DEFER     /* 0: Abort after excessive wait */
       | ! MAC_CON4_BP_EN     /* 0: Use binary backoff algorithm */
       | ! MAC_CON4_NO_BKOFF  /* 0: Use binary backoff algorithm */
       | ! MAC_CON4_LONG_PRE  /* 0: Long preambles allowed */
       | ! MAC_CON4_PURE_PRE  /* 0: Don't check preable contents */
      ),
    D8(MAC_BBIP_GAP, 0x12),  /* Recommended back-to-back gap for half duplex XXX */
    D16(MAC_IP_GAP, 0x0C12), /* As recommended in the data sheet */
    // Collision control default values as per data sheet
    D16(MAC_MAX_FRAME, 1518),/* As recommended in the data shaat */
    // MAC-PHY Support not changed as per data sheet

    // MII Control register not changed as per data sheet
    // MII Command, status, MII reg address and data registers
    // used for MII communication, not initialised.

    /**********************************
     * Bank3: MAC address and misc
     **********************************/

#if 0
    MISTAT
    EREVID
    ECOCON
    EFLOCON
    EPAUS
#endif

    /**********************************
     * Cross-bank registers
     **********************************/
    D8(E_INT_ENA,
       0
       |   E_INT_ENA_INT      /* 1: Enable INT pin */
       |   E_INT_ENA_PKT      /* 1: Interrupt on pending packets */
       | ! E_INT_ENA_DMA      /* 0: Disable DMA interrupt */
       |   E_INT_ENA_LINK     /* 1: Interrupt on link status change */
       |   E_INT_ENA_TX       /* 1: Interrupt on transmit ready */
       |   E_INT_ENA_WOL      /* 1: Enable WOL pin */
       | ! E_INT_ENA_TX_ERR   /* 1: Disable transmit error interrupts */
       |   E_INT_ENA_RX_ERR   /* 1: Disable receive error interrupts */
      ),
#ifdef DONT_INIT_RESET_TIME_VALUE
    D8(E_CON2,
       0
       |   E_CON2_AUTOINC     /* 1: Autoinc; reset-time value */
       | ! E_CON2_PKTDEC      /* 0: Command, not used during initialisation */
       | ! E_CON2_PWRSV       /* 0: No MAC/PHY power save; reset-time value */
       | ! E_CON2_VRPS        /* 0: No voltage regu power save; reset-time value */
      ),
#endif
    D8(E_CON1,
       0
       | ! E_CON1_TX_RESET     /* 0: Transmit logic enabled */
       | ! E_CON1_RX_RESET     /* 0: Receive logic enabled */
       | ! E_CON1_DMA_START    /* 0: Don't use DMA now */
       | ! E_CON1_CSUM_ENABLE  /* 0: N/A, we are not using the DMA */
       | ! E_CON1_TX_REQUEST   /* 0: Don't transmit yet */
       |   E_CON1_RX_ENABLE    /* 1: Enable Receiving */
       |   E_CON1_B_SEL1       /* 11: Stay in bank 3 */
       |   E_CON1_B_SEL0
      ),
};

void enc_init(const uint8_t mac_address[ETH_ADDRESS_LEN]) {
    const device_register_init_static_8bit_t *p;

    /* Wait for the chip oscillator having started */
    while ((enc_reg_get(E_STAT) & E_STAT_CLOCK_READY) == 0)
        ; /* XXX: Enable other functionality */

    /* NB: MAC address in ENC28J60 is backwards
           and in very peculiar order. */
    enc_reg_set(MAC_ADR0, mac_address[5]);
    enc_reg_set(MAC_ADR1, mac_address[4]);
    enc_reg_set(MAC_ADR2, mac_address[3]);
    enc_reg_set(MAC_ADR3, mac_address[2]);
    enc_reg_set(MAC_ADR4, mac_address[1]);
    enc_reg_set(MAC_ADR5, mac_address[0]);

    /* Initialise the chip otherwise */
    for (p = enc28j60_init;
         p < enc28j60_init + COUNT_OF(enc28j60_init);
         p++) {
        enc_reg_set(p->reg, p->value);
    }

    /* Wait for the PHY to become active */
    while (enc_reg_get(PHY_CON1) & PHY_CON1_PRST)
        ; /* XXX: Enable other functionality */

    /* See Errata #9 */
    enc_reg_set(PHY_CON2, PHY_CON2_HDLDIS);

    /* Write the packet control byte to the beginning of the TX buffer.
     * As the enc_spi_xfer_buffer requires even number of bytes, we
     * write two zeros.  The second will get later overwritten.
     *
     * NB.  E_WR_PTR is initialized above to TX_BUFFER_START.
     */
    enc_buf_value_t b[2] = { 0, 0 };

    enc_spi_xfer_buffer(SPI_WBM, b, sizeof(b), 0);
}

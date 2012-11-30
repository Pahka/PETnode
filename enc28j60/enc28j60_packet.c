/*
 * Copyright 2012 Pekka Nikander.  See NOTICE for licensing information.
 */

#include <stm32f0xx.h>

#include <assert.h>
#include <stdlib.h>

#include <enc28j60.h>
#include <enc28j60_spi.h>

#include "enc28j60-conf.h"

static enc_rx_packet_header_t rx_header;

int
enc_packet_receive(enc_buf_value_t *buffer, enc_buf_len_t maxlen) {
#if 1
    /* Check if there are received packets; see also Errata #6 */
    if (enc_reg_get(E_PKT_CNT) == 0)
        return 0;
#else
    enc_reg_get(E_PKT_CNT);
#endif


    /*
     * We assume that the buffer transfer read pointer E_RD_PTR
     * is already pointing to the next packet in the circular
     * buffer.
     */

    /* Read the 6-byte packet header */
    enc_spi_xfer_buffer(SPI_RBM, (unsigned char *)&rx_header, sizeof(rx_header), 1);

    register int plen = rx_header.rx_length;
    register int next = rx_header.rx_next;

    /* Read (the beginning of) the packet */
    if (plen > maxlen)
        plen = maxlen;
    enc_spi_xfer_buffer(SPI_RBM, buffer, plen, 1);

    /*
     * Go to the beginning of the next packet.
     *
     * NB.  It might be possible to avoid this call if len ==
     *      rx_header.rx_length, but that would need to be tested, as
     *      it is not clear from the data sheet.
     */
    enc_reg_set(E_RD_PTR, next);

    /*
     * Free the ENC28J60 buffer memory for the next packets.
     * See Errata #14.
     */
    enc_reg_set(E_RX_RD_PTR, next == RX_BUFFER_START? RX_BUFFER_END: next-1);

    /* Decrement the packet count */
    enc_reg_bitop(SPI_BFS, E_CON2, E_CON2_PKT_DEC);

    return plen;
}

void
enc_packet_send(enc_buf_value_t *buffer, enc_buf_len_t len) {
    /*
     * Wait until the previous packet has been sent.
     */
    while ((enc_reg_get(E_CON1) & E_CON1_TX_REQUEST)) {
        /*
         * Reset the transmit logic if it has been stalled
         * as per Errata #12.
         */
        if ((enc_reg_get(E_INT_REQ) & E_INT_REQ_TX_ERR)) {
            /* Reset the transmit logic */
            enc_reg_bitop(SPI_BFS, E_CON1, E_CON1_TX_RESET);
            enc_reg_bitop(SPI_BFC, E_CON1, E_CON1_TX_RESET);
            enc_reg_bitop(SPI_BFC, E_INT_REQ, E_INT_REQ_TX_ERR);
        } else {
            ; // XXX: Allow other threads to run
        }
    }

    /*
     * Write the packet to the ENC28J60 packet buffer.
     *
     * NB.  The per-packet control byte is there already
     *      at TX_BUFFER_START. It was written there at
     *      enc_init().
     */
    enc_reg_set(E_WR_PTR, TX_BUFFER_START + 1);
    enc_spi_xfer_buffer(SPI_WBM, buffer, len, 0);

    /*
     * Set the packet start and end.
     */
    enc_reg_set(E_TX_STA, TX_BUFFER_START);
    enc_reg_set(E_TX_END, TX_BUFFER_START + 1 + len);

    /*
     * Request transmission.
     */
    enc_reg_bitop(SPI_BFS, E_CON1, E_CON1_TX_REQUEST);
}

#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

#include <stdint.h>

#define CCIF
#define CLIF

#define WITH_UIP 1

/* These names are deprecated, use C99 names. */
typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int8_t s8_t;
typedef int16_t s16_t;
typedef int32_t s32_t;

typedef unsigned int clock_time_t;
typedef unsigned int uip_stats_t;

#ifndef BV
#define BV(x) (1<<(x))
#endif

/* uIP configuration */
#define UIP_CONF_LLH_LEN         14 /* Ethernet header */
#define UIP_CONF_BROADCAST       1
#define UIP_CONF_LOGGING         0
#define UIP_CONF_BUFFER_SIZE     512

#define UIP_CONF_TCP_FORWARD     0

/*
 * Generated test MAC address: AE-68-2E-E2-BF-E0
 */

#define UIP_ETHADDR0 0xae
#define UIP_ETHADDR1 0x68
#define UIP_ETHADDR2 0x2e
#define UIP_ETHADDR3 0xe2
#define UIP_ETHADDR4 0xbf
#define UIP_ETHADDR5 0xe0

#endif /* __CONTIKI_CONF_H__ */

/*
 * Copyright 2012 Pekka Nikander.  See NOTICE for licensing information.
 *
 */

#include <sys/clock.h>
#include <sys/etimer.h>

static volatile clock_time_t count;

extern void TimingDelay_Decrement(void);

void
SysTick_Handler(void) {

    count++;

    if (etimer_pending()) {
        etimer_request_poll();
    }

    TimingDelay_Decrement();
}

clock_time_t
clock_time(void) {
    return count;
}

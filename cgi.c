#include <stm32f0xx.h>

#include <stdio.h>
#include <string.h>

#include "contiki-net.h"
#include "httpd.h"
#include "httpd-cgi.h"
#include "httpd-fs.h"

static struct {
    int value;
} leds[] = { 
    { .value = 0, },
    { .value = 100, },
    { .value = 100, },
    { .value = 100, },
 };

static unsigned short
generate_pahka_cgi(void *arg) {
    const char *str = (const char *)arg;

    const char *cmd = strchr(str, ' ');
    if (cmd == NULL)
        return 0;
    cmd += 1;

    const char *led = strchr(cmd, ' ');
    if (led == NULL)
        return 0;
    led += 1;

    int lednum = 0;
    if (*led == 'r') 
        lednum = 1;
    else if (*led == 'g') 
        lednum = 2;
    else if (*led == 'b') 
        lednum = 3;

    if (*cmd == 'i') {
        leds[lednum].value++;
        if (leds[lednum].value > 100)
            leds[lednum].value = 100;
    } else if (*cmd == 'd') {
        leds[lednum].value--;
        if (leds[lednum].value < 80)
            leds[lednum].value = 80;
    }
    switch (lednum) {
    case 1: 
        TIM16->CCR1 = leds[lednum].value;
        break;
    case 2: 
        TIM17->CCR1 = leds[lednum].value;
        break;
    case 3:
        TIM15->CCR1 = leds[lednum].value;
        break;
    }
    return snprintf((char *)uip_appdata, uip_mss(), "%d", leds[lednum].value);

}

PT_THREAD(pahka_cgi_thread(struct httpd_state *s, char *ptr)) {
    PSOCK_BEGIN(&s->sout);

    PSOCK_GENERATOR_SEND(&s->sout, generate_pahka_cgi, (void *)ptr);

    PSOCK_END(&s->sout);
}

HTTPD_CGI_CALL(pahka_cgi, "pahka", pahka_cgi_thread);

void
pahka_cgi_init(void) {
    httpd_cgi_add(&pahka_cgi);
}

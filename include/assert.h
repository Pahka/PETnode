/*
 * Copyright 2012 Pekka Nikander.  See NOTICE for licensing information.
 */

static inline void abort(void) {
    for (;;)
        ;
}

static inline void assert(int a) {
    if (!a) abort();
}



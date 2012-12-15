/*
 * Copyright 2012 Pekka Nikander.  See NOTICE for licensing information.
 */

void abort(void) __attribute__((noreturn));

static inline void assert(int a) {
    if (!a) abort();
}



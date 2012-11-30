
#if !CONTIKI
typedef unsigned int process_event_t;

#define PROCESS_NAME(name)

#else

#include <core/sys/process.h>
#include <core/sys/pt.h>

#endif


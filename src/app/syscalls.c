#include "FreeRTOS.h"
#include "task.h"
#include "watchdog.h"
#include <errno.h>
#include <sys/types.h>

__attribute__ ((noreturn))
static void halt_and_wait_forever(void)
{
    // block and wait forever
    vTaskSuspendAll();
    while (1) {
        watchdogReset();
    }
}

void __assert_func(const char *_file, int _line, const char *_func, const char *_expr )
{
    (void)_file;
    (void)_line;
    (void)_func;
    (void)_expr;

    halt_and_wait_forever();
}

void abort(void)
{
    halt_and_wait_forever();
}

__attribute__((used))
caddr_t _sbrk_r(struct _reent *r, int incr)
{
    halt_and_wait_forever();
    (void)incr;
    __errno_r(r) = ENOMEM;
    return (caddr_t)-1;
}

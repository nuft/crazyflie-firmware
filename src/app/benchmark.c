// inspired by
// http://embeddedb.blogspot.ch/2013/10/how-to-count-cycles-on-arm-cortex-m.html
#include <stdint.h>

// addresses of registers
volatile uint32_t *DWT_CONTROL = (uint32_t *)0xE0001000;
volatile uint32_t *DWT_CYCCNT = (uint32_t *)0xE0001004;
volatile uint32_t *DEMCR = (uint32_t *)0xE000EDFC;

void cycle_counter_reset(void)
{
    // enable the use DWT
    *DEMCR = *DEMCR | 0x01000000;
    // enable cycle counter
    *DWT_CONTROL = *DWT_CONTROL | 1;
    // Reset cycle counter
    *DWT_CYCCNT = 0;
}

uint32_t cycle_counter_get(void)
{
    // read number of cycles
    return *DWT_CYCCNT;
}

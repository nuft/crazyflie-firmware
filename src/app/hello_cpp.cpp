/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"

extern "C" {
#include "config.h"
#include "led.h"
}

namespace hello_cpp {

void main(void *param)
{
    (void) param;
    while (1) {
        ledSet(LED_GREEN_L, 1);
        vTaskDelay(M2T(500));
        ledSet(LED_GREEN_L, 0);
        vTaskDelay(M2T(500));
    }
}

} // namespace hello_cpp

extern "C"
void helloInit(void)
{
    xTaskCreate(hello_cpp::main, HELLO_TASK_NAME, HELLO_TASK_STACKSIZE, NULL,
                HELLO_TASK_PRI, NULL);
}

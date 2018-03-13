/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "led.h"

static void helloTask(void *param);

void helloInit(void)
{
    xTaskCreate(helloTask, HELLO_TASK_NAME, HELLO_TASK_STACKSIZE, NULL,
                HELLO_TASK_PRI, NULL);
}

void helloTask(void *param)
{
    (void) param;
    while (1) {
        ledSet(LED_GREEN_L, 1);
        vTaskDelay(M2T(500));
        ledSet(LED_GREEN_L, 0);
        vTaskDelay(M2T(500));
    }
}

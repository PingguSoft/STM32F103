//#include <wirish/wirish.h>
//#include "libraries/FreeRTOS/MapleFreeRTOS.h"
#include <MapleFreeRTOS821.h>

static void vLEDFlashTask(void *pvParameters) {
    int nCnt = 0;
    
    for (;;) {
        vTaskDelay(1000);
        digitalWrite(PC13, HIGH);
        vTaskDelay(50);
        digitalWrite(PC13, LOW);
        Serial1.print("CNT: ");
        Serial1.println(nCnt++, DEC);
    }
}

void setup() {
    // initialize the digital pin as an output:
    pinMode(PC13, OUTPUT);
    
    Serial1.begin(115200); // Ignored by Maple. But needed by boards using hardware serial via a USB to Serial adaptor
    Serial1.println("ASCII Table ~ Character Map");

    xTaskCreate(vLEDFlashTask,
                "Task1",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
    vTaskStartScheduler();
}

void loop() {
    // Insert background code here
}



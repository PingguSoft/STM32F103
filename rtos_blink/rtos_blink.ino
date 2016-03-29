#include <RTClock.h>

//#include <wirish/wirish.h>
//#include "libraries/FreeRTOS/MapleFreeRTOS.h"
#include <MapleFreeRTOS821.h>
#include "utils.h"

RTClock rt(RTCSEL_LSE);

static void vLEDFlashTask(void *pvParameters) {
    u32 tt, oldt;
    
    for (;;) {
        tt = rt.getTime();
        if (oldt != tt) {
            Utils::printf("time is: %d\n", tt);
            digitalWrite(PC13, !digitalRead(PC13));
            oldt = tt;
        }
    }
}

void setup() {
    // initialize the digital pin as an output:
    pinMode(PC13, OUTPUT);
    
    CONFIG_DBG_SERIAL.begin(115200); // Ignored by Maple. But needed by boards using hardware serial via a USB to Serial adaptor
    Utils::printf("HELLO !!!\n");

    xTaskCreate(vLEDFlashTask,
                "Task1",
                128,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
    vTaskStartScheduler();
}

void loop() {
    // Insert background code here
}


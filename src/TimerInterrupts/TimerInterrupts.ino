/*
 Timer Interrupts Example

 Demonstrates usage of the HardwareTimer classes by blinking the LED
 
 Created 22 April 2010, last updated 8 June 2010
 By Bryan Newbold for LeafLabs
 This code is released with no strings attached.
 
 */

#define LED_PIN PC13
#define BUTTON_PIN 38
#define LED_RATE 1000000    // in microseconds; should give 0.5Hz toggles

void handler_led(void);
void handler_count1(void);
void handler_count2(void);

int toggle = 0;

int count1 = 0;
int count2 = 0;
int led_rate = LED_RATE;
unsigned int ts1 = 0;

void setup()
{
	Serial.begin(115200); // Ignored by Maple. But needed by boards using hardware serial via a USB to Serial adaptor
    // Set up the LED to blink 
    pinMode(LED_PIN, OUTPUT);

    // Set up BUT for input
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Setup LED Timer
    Timer2.setChannel1Mode(TIMER_OUTPUTCOMPARE);
    Timer2.pause();
    Timer2.setCount(0);
    Timer2.setOverflow(65530);
    Timer2.setPrescaleFactor(CYCLES_PER_MICROSECOND);
    Timer2.refresh();
    
    //Timer2.setPeriod(LED_RATE); // in microseconds
    //Timer2.setCompare1(1);      // overflow might be small
    Timer2.setCompare(TIMER_CH1, 10000);
    Timer2.attachCompare1Interrupt(handler_led);
    ts1 = micros();
    Timer2.resume();

    // Setup Counting Timers
    Timer3.setChannel1Mode(TIMER_OUTPUTCOMPARE);
    Timer4.setChannel1Mode(TIMER_OUTPUTCOMPARE);
    Timer3.pause();
    Timer4.pause();
    Timer3.setCount(0);
    Timer4.setCount(0);
    Timer3.setOverflow(30000);
    Timer4.setOverflow(30000);
    Timer3.setCompare1(1000);   // somewhere in the middle
    Timer4.setCompare1(1000);   
    Timer3.attachCompare1Interrupt(handler1);
    Timer4.attachCompare1Interrupt(handler2);
    Timer3.resume();
    Timer4.resume();

}

int step = 0;
unsigned int ts2 = 0;
void loop() {

    // Display the running counts
    Serial.print("Count 1: "); 
    Serial.print(count1);
    Serial.print("\t\tCount 2: "); 
    Serial.println(count2);
    
    Serial.print("TS1: "); 
    Serial.print(ts1);
    Serial.print("\tTS2: "); 
    Serial.print(ts2);
    Serial.print("\tdiff: "); 
    Serial.print(ts2 - ts1);

    // Run... while BUT is held, pause Count2
    for(int i = 0; i<1000; i++) {
        if(digitalRead(BUTTON_PIN)) {
            Timer4.pause();
        } else {
            Timer4.resume();
        }
        delay(1);
    }
}

void handler_led(void) {
    toggle ^= 1;
    digitalWrite(LED_PIN, toggle);
    if (step == 0) {
           ts2 = micros();
           step = 1;
    }
} 

void handler1(void) {
    count1++;
} 
void handler2(void) {
    count2++;
} 

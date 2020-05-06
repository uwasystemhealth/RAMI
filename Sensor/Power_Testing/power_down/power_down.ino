#include <avr/sleep.h>
#include <avr/power.h>


//#define DEBUG // Comment out this line to turn off debug mode and save some power

#ifdef DEBUG
#define DEBUG_PRINT(msg) Serial.println(msg)
#else
#define DEBUG_PRINT(msg)
#endif

#define BAND            915E6  // Different RAMI devices have different bands
#define DEVICE_ID       0x01
#define TIMEOUT         200 //time to wait for a response from the server (x50ms)


// ADXL345 definitions
#define accIntPin             2
#define accCsPin              7
#define accRange              8 // valid values are 2g, 4g, 8g or 16g, (2, 4, 8, 16)
#define accThreshold          20 // The Scale Factor is 62.5mg/LSB, values are between 0 and 255.

//LoRa definitions
#define loraIntPin            6
#define loraRstPin            10
#define loraCsPin             9

//RTC Definitions
#define rtcIntPin             3
#define rtcRstPin             5
#define monitoringTime        5 // Time period to collect ADXL samples (seconds)
#define rtcActiveAlarmTime    10 // Time between wakes on active state (seconds)
                               // it is important that this time is greater than
                               // the monitoring time
#define rtcInactiveAlarmTime  10 // must be <86400 (one day) // Time between hearbeats (seconds)

// lets interrupts safely modify
volatile bool rtcInterruptFlag = false;
volatile bool accInterruptFlag = false;
volatile bool accInactiveInterruptFlag = false;
volatile bool stateFlag = false;

unsigned long nonce = 0;

unsigned int previous_magnitude = 0;

void setup() {
  //////////////// Configure ATMEGA328P ////////////////
  // turn off all non essential components
  power_adc_disable   ();
  power_timer0_disable();
  power_timer1_disable();
  power_usart0_disable(); // This disables serial communications
}

void loop() {
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  sleep_cpu ();
  sleep_enable();
}

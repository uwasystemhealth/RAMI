
#include <avr/sleep.h>
#include <avr/power.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFun_ADXL345.h>
#include <RtcDS3231.h>
#include <EEPROM.h>
#include <LoRa.h>

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
#define rtcActiveAlarmTime    120 // Time between wakes on active state (seconds)
                               // it is important that this time is greater than
                               // the monitoring time
#define rtcInactiveAlarmTime  240 // must be <86400 (one day) // Time between hearbeats (seconds)

// lets interrupts safely modify
volatile bool rtcInterruptFlag = false;
volatile bool accInterruptFlag = false;
volatile bool accInactiveInterruptFlag = false;
volatile bool stateFlag = false;

unsigned long nonce = 0;

unsigned int previous_magnitude = 0;

ADXL345 adxl;

RtcDS3231<TwoWire> Rtc(Wire);

void setup() {
  //////////////// Configure ATMEGA328P ////////////////
  SPI.begin();
  // turn off all non essential components
  power_adc_disable   ();
  #ifndef DEBUG
  power_timer1_disable();
  power_usart0_disable(); // This disables serial communications
  #else
  Serial.begin(9600);
  DEBUG_PRINT("DEBUG enabled");
  #endif

  ////////////////// Configure LoRa ////////////////
  LoRa.setPins(loraCsPin, loraRstPin, loraIntPin);
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(31.25E3);
  LoRa.setCodingRate4(8);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0xfa);

  //////////////// Configure ADXL345 ////////////////
  adxl = ADXL345(accCsPin);
  adxl.powerOn();                     // Power on the ADXL345
  adxl.setRangeSetting(16);           // Give the range settings
                                      // Accepted values are 2g, 4g, 8g or 16g
                                      // Higher Values = Wider Measurement Range
                                      // Lower Values = Greater Sensitivity

  adxl.setSpiBit(0);                  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1
                                      // Default: Set to 1
                                      // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library 
   
  adxl.setActivityXYZ(1, 1, 1);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setActivityThreshold(accThreshold);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)
 
  adxl.setInactivityXYZ(1, 1, 1);     // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setInactivityThreshold(accThreshold);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  adxl.setTimeInactivity(60);         // How many seconds of no activity is inactive 0-255

  adxl.setTapDetectionOnXYZ(1, 1, 1); // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)
 
  // Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
  adxl.setTapThreshold(50);           // 62.5 mg per increment
  adxl.setTapDuration(15);            // 625 μs per increment
  adxl.setDoubleTapLatency(80);       // 1.25 ms per increment
  adxl.setDoubleTapWindow(200);       // 1.25 ms per increment
 
  // Set values for what is considered FREE FALL (0-255)
  adxl.setFreeFallThreshold(7);       // (5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(30);       // (20 - 70) recommended - 5ms per increment
  
  // Turn on Interrupts for each mode (1 == ON, 0 == OFF)
  adxl.InactivityINT(0);
  adxl.ActivityINT(1);
  adxl.FreeFallINT(1);
  adxl.doubleTapINT(1);
  adxl.singleTapINT(1);

  // Make interrupts occur on pin 1
  adxl.setImportantInterruptMapping(2,2,2,2,2);
  adxl.setInterruptLevelBit(1); //Make the interrupt active low so we can use it to wake the ATMEGA328P
  
  // power minimisation for ADXL345
  adxl.setRate        (12.5);
  adxl.setLowPower    (1);
  
  //////////////// Configure DSM3231M ////////////////
  
  pinMode(rtcRstPin, OUTPUT);
  digitalWrite(rtcRstPin, HIGH);
  
  Rtc.Begin();
  
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  
  // Check if we have a vaild DateTime and if we can connect, if not print error or reset datetime to compiletime.
  if (!Rtc.IsDateTimeValid()) 
  {
      if (Rtc.LastError() != 0)
      {
          DEBUG_PRINT("RTC communications error = ");
          DEBUG_PRINT(Rtc.LastError());
      }
      else
      {
          DEBUG_PRINT("RTC lost confidence in the DateTime!");
          Rtc.SetDateTime(compiled);
      }
  }
  
  if (!Rtc.GetIsRunning()) {
    DEBUG_PRINT("RTC was not actively running, starting now");
    Rtc.SetIsRunning(true);
  }
  
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmOne);
  
  RtcDateTime now = Rtc.GetDateTime();
  if (now < compiled) 
  {
      DEBUG_PRINT("RTC is older than compile time!  (Updating DateTime)");
      Rtc.SetDateTime(compiled);
  }
  
  // set an alarm to trigger heartbeat
  // first signal sent ten seconds into future
  RtcDateTime alarmTime = now + 10;
  DS3231AlarmOne alarm1(
    alarmTime.Day(),
    alarmTime.Hour(),
    alarmTime.Minute(), 
    alarmTime.Second(),
    DS3231AlarmOneControl_HoursMinutesSecondsMatch);
  Rtc.SetAlarmOne(alarm1);
  Rtc.LatchAlarmsTriggeredFlags();

  if (!LoRa.begin(BAND)) {
    DEBUG_PRINT("Starting LoRa failed!");
    while (1);
  }

  // attachInterrupts 
  DS3231AlarmFlag flag = Rtc.LatchAlarmsTriggeredFlags();
  adxl.getInterruptSource();
  pinMode(accIntPin, INPUT_PULLUP);
  pinMode(rtcIntPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rtcIntPin), rtcInt, LOW);
  attachInterrupt(digitalPinToInterrupt(accIntPin), adxlInt, LOW);
}

void adxlInt() {
  sleep_disable();
  accInterruptFlag = true;  
  stateFlag = true;
  DEBUG_PRINT("ADXL int");
  detachInterrupt(digitalPinToInterrupt(accIntPin));
}

void adxlInactiveInt() {
  sleep_disable();
  accInactiveInterruptFlag = true;
  DEBUG_PRINT("ADXL inactive int");
  detachInterrupt(digitalPinToInterrupt(accIntPin));
}

void rtcInt () {
  sleep_disable();
  rtcInterruptFlag = true;
  detachInterrupt(digitalPinToInterrupt(rtcIntPin));
}

void inactiveRun(){
  DS3231AlarmFlag flag = Rtc.LatchAlarmsTriggeredFlags();
  byte adxl_int = adxl.getInterruptSource();
  if (flag & DS3231AlarmFlag_Alarm1) {
    //transmitStillAlive();
    DEBUG_PRINT("Device woken by RTC for Keepalive");
    RtcDateTime now = Rtc.GetDateTime();
    now = now + rtcInactiveAlarmTime;
    DS3231AlarmOne alarm1(
            now.Day(),
            now.Hour(),
            now.Minute(), 
            now.Second(),
            DS3231AlarmOneControl_HoursMinutesSecondsMatch); // longest time between heatbeats is 1 day in this configuration
                                                             // this can be extended to a maximum of one month if required
    Rtc.SetAlarmOne(alarm1);
    writeToLora(0x01, NULL);
  }
  if (accInterruptFlag) {
    stateFlag = true;
  }
  rtcInterruptFlag = false;
  accInterruptFlag = false;
  attachInterrupt(digitalPinToInterrupt(rtcIntPin), rtcInt, LOW);
  attachInterrupt(digitalPinToInterrupt(accIntPin), adxlInt, LOW);
  return;
}

void activeRun(){
  detachInterrupt(digitalPinToInterrupt(accIntPin));
  byte adxl_int = adxl.getInterruptSource();
  adxl.InactivityINT(1);
  adxl.ActivityINT(0);
  adxl.FreeFallINT(0);
  adxl.doubleTapINT(0);
  adxl.singleTapINT(0);
  detachInterrupt(digitalPinToInterrupt(rtcIntPin));
  DS3231AlarmFlag flag = Rtc.LatchAlarmsTriggeredFlags(); // Latch flags here incase inactivity & monitoriong time are equal
  if (accInactiveInterruptFlag) {
    accInactiveInterruptFlag = false;
    stateFlag = false;
    DEBUG_PRINT("Inactivity interrupt triggered. Setting state to inactive");
    RtcDateTime now = Rtc.GetDateTime();
    now = now + rtcInactiveAlarmTime;
    DS3231AlarmOne alarm1(
            now.Day(),
            now.Hour(),
            now.Minute(), 
            now.Second(),
            DS3231AlarmOneControl_HoursMinutesSecondsMatch); // longest time between heatbeats is 1 day in this configuration
                                                             // this can be extended to a maximum of one month if required
    Rtc.SetAlarmOne(alarm1);
    adxl.InactivityINT(0);
    adxl.ActivityINT(1);
    adxl.FreeFallINT(1);
    adxl.doubleTapINT(1);    
    adxl.singleTapINT(1);
    writeToLora(0x02, NULL);
    stateFlag = false;
    attachInterrupt(digitalPinToInterrupt(rtcIntPin), rtcInt, LOW);
    attachInterrupt(digitalPinToInterrupt(accIntPin), adxlInt, LOW);
    return;
  }
  attachInterrupt(digitalPinToInterrupt(accIntPin), adxlInactiveInt, LOW);
  unsigned int magnitude = 0;
  unsigned int maximum = 0;
  int x = 0;
  int y = 0;
  int z = 0;
  RtcDateTime now = Rtc.GetDateTime();
  now = now + rtcActiveAlarmTime + monitoringTime;
  DS3231AlarmOne alarm1(
          now.Day(),
          now.Hour(),
          now.Minute(), 
          now.Second(),
          DS3231AlarmOneControl_HoursMinutesSecondsMatch);
  Rtc.SetAlarmOne(alarm1);
  
  unsigned long elapsed = 0;
  unsigned long start = millis();
  while(elapsed < (monitoringTime*1000)){
    elapsed = millis() - start;
    adxl.readAccel(&x, &y, &z);
    magnitude = sqrt(x*x + y*y + z*z);
    if (magnitude > maximum){
      maximum = magnitude;
    }
  };
  DEBUG_PRINT(magnitude); 
  if (abs(magnitude - previous_magnitude) > accThreshold){
    previous_magnitude = magnitude;
    writeToLora(0x00, (byte*)(&previous_magnitude));
  }
  rtcInterruptFlag = false;
  accInterruptFlag = false;
  attachInterrupt(digitalPinToInterrupt(rtcIntPin), rtcInt, LOW);
  return;
}

void writeToLora(byte command, byte* dat){
  noInterrupts();
  LoRa.beginPacket();
  LoRa.write(command);
  LoRa.write(DEVICE_ID);
  LoRa.write(((byte*)&nonce)[0]);
  LoRa.write(((byte*)&nonce)[1]);
  if (dat != NULL) {
    LoRa.write(dat[1]);
    LoRa.write(dat[0]);
  }
  LoRa.endPacket();
  interrupts();
  nonce = nonce + 1;
  LoRa.sleep();
};

void loop() {
  #ifdef DEBUG
  Serial.print("RTC Interrupt Flag: ");
  DEBUG_PRINT(rtcInterruptFlag);
  Serial.print("ADXL Interrupt Flag: ");
  DEBUG_PRINT(accInterruptFlag);
  Serial.print("State Flag: ");
  DEBUG_PRINT(stateFlag);
  #else
  power_timer0_disable();
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  sleep_cpu ();
  sleep_enable();
  power_timer0_enable();
  #endif
  if(stateFlag && (rtcInterruptFlag || accInterruptFlag)) {
    activeRun();
  } else if(rtcInterruptFlag || accInterruptFlag) {
    inactiveRun();
  } else {
    inactiveRun();
  }
  #ifdef DEBUG
  delay(1000);
  #endif
}

//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>
#include <SparkFun_ADXL345.h>

//Libraries for OLED Display
#include <Wire.h>

//define SPI bus pins
#define SCK 13
#define MISO 12
#define MOSI 11

//define the pins used by the LoRa transceiver module
#define SS_LORA 9
#define RST 10
#define DIO0 6

//define the settings used by the ADXL345
#define SS_ADXL       7
#define INT_ADXL      2
#define RANGE_ADXL    8 // valid values are 2g, 4g, 8g or 16g, (2, 4, 8, 16)
#define ADXL_THRESH   10 // The Scale Factor is 62.5mg/LSB, values are between 0 and 255.

// Axis biases 
#define ADXL_X_BIAS   0
#define ADXL_Y_BIAS   0
#define ADXL_Z_BIAS   1

//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define BAND 915E6

//packet counter
int counter = 0;
int accIntPin       = 2;
int accCsPin        = 7;
int accRange        = 8; // valid values are 2g, 4g, 8g or 16g, (2, 4, 8, 16)
int accThreshold    = 10; // The Scale Factor is 62.5mg/LSB, values are between 0 and 255.

ADXL345 adxl;

void setup() {
  
  Serial.begin(9600);
  //SPI LoRa pins
  SPI.begin();
  //setup LoRa transceiver module
  LoRa.setPins(SS_LORA, RST, DIO0);
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(31.25E3);
  LoRa.setCodingRate4(8);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0xfa);

  adxl = ADXL345(SS_ADXL);

  adxl.powerOn                ();
  adxl.setRangeSetting        (RANGE_ADXL);
  adxl.setSpiBit              (0); // 4-wire SPI
  adxl.setActivityXYZ         (1, 1, 1); // Use all axis
  adxl.setActivityThreshold   (ADXL_THRESH); // Threshold for interrupt
  
  if (!LoRa.begin(BAND)) {
    while (1);
  }
  delay(2000);
}

void loop() {
   
  Serial.print("Sending packet: ");
  Serial.println(counter);

  //Send LoRa packet to receiver
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket();

  counter++;
  
  delay(10000);
}

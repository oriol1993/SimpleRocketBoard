///////////////////////////////////////////////////////////////////////////////
//                       PROGRAM HEADER
///////////////////////////////////////////////////////////////////////////////

// Libraries
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Buffer.h>

// Constants and sizes
#define LED 3
#define PB 5
#define force_serial 1
#define bytes_alt 4
#define bytes_timestamp 2

// Declarations
Adafruit_BMP280 bmp;
Buffer cbuffer;

byte altByte[bytes_alt];
byte timeByte[bytes_timestamp];

uint8_t state = 0;
uint32_t last_bttn = 0;
uint32_t led_flash = 0;
uint8_t led_state = 0;
uint32_t t_lastbmp = 0;
float alt = 0;
uint16_t timestamp = 0;
bool serial_init = 0;

///////////////////////////////////////////////////////////////////////////////
//                            FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

void setup(){
  // Input and output
  pinMode(LED, OUTPUT);
  pinMode(PB, INPUT_PULLUP);
  if(!digitalRead(PB) | force_serial){
    Serial.begin(9600);
    for(uint8_t i=0;i<7;i++)
    {
      digitalWrite(LED,HIGH);
      delay(100);
      digitalWrite(LED,LOW);
      delay(100);
    }
    serial_init = 1;
  }

  // BMP180 Initialization
  Serial.print(F("Initializing BMP280 ..."));
  if (!bmp.begin()) {
    Serial.println(F("Unable to connect!"));
    while (1);
  }

  Serial.println(F("Connection successful!"));
  bmp.osrs_p = 15; // 4.3.4 Table 21
  bmp.osrs_t = 1; // 4.3.4 Table 22
  bmp.t_sb = 0; // 3.6.3 Table 11, 4.3.5
  bmp.filter = 5; // 3.3.3
  cbuffer.reset(); // Buffer Initialization (head = 0; tail = 0)
}

void loop(){
 buttonCheck();
 checkLed();
 Serial.println("Writing to buffer...");
 for(uint8_t u=0;u<10;u++)
 {
    print_bmp();
 }
 Serial.println("Reading from buffer...");
 for(uint8_t u=0;u<10;u++)
 {
   buffer2serial();
 }
 while(1);
}

void buttonCheck(){
 if (millis() - last_bttn > 300) {
  if (digitalRead(PB) == 0){
   state = !state;
   last_bttn = millis();
   led_flash = millis();
   led_state = 1;
  }
 }
}

void checkLed(){
 if (state == true){
  if (millis() - led_flash > 500) {
   led_state = !led_state;
   led_flash = millis();
   digitalWrite(LED, led_state);
  }
 }
 else { digitalWrite(LED, LOW); }
}

void print_bmp(){
  if(millis()-t_lastbmp>2000  || 1)
  {
    // Acquire heigh measurement and time stamp
    alt = bmp.readAltitude(1013.25);
    timestamp = micros()%6553600;

    // Load data into buffer
    float2byte(alt, altByte);  // Convert float to byte array
    timeByte[0] = timestamp & 255;
    timeByte[1] = timestamp>>8;
    if (cbuffer.spaceAvailable(sizeof(altByte) + sizeof(timeByte))) {    //Check if there's enough space for all data
      cbuffer.CarregarBuffer(altByte, sizeof(altByte));
      cbuffer.CarregarBuffer(timeByte, sizeof(timeByte));
    }

    t_lastbmp = millis();

    // Debugging
    Serial.print("h="); Serial.print(alt);
    Serial.print(", ts="); Serial.print(timestamp);
    Serial.print(", tb0="); Serial.print(timeByte[0]);
    Serial.print(", tb1="); Serial.print(timeByte[1]);
    Serial.print(", al0="); Serial.print(altByte[0]);
    Serial.print(", al1="); Serial.print(altByte[1]);
    Serial.print(", al2="); Serial.print(altByte[2]);
    Serial.print(", al3="); Serial.print(altByte[3]);
    Serial.println();
  }
}

void buffer2serial(){
  cbuffer.DescarregarBuffer(altByte, sizeof(altByte));
  cbuffer.DescarregarBuffer(timeByte, sizeof(timeByte));
  timestamp = timeByte[0] | timeByte[1]<<8;
  alt = byte2float(altByte);
  // Debugging
  Serial.print("h="); Serial.print(alt);
  Serial.print(", ts="); Serial.print(timestamp);
  Serial.print(", tb0="); Serial.print(timeByte[0]);
  Serial.print(", tb1="); Serial.print(timeByte[1]);
  Serial.print(", al0="); Serial.print(altByte[0]);
  Serial.print(", al1="); Serial.print(altByte[1]);
  Serial.print(", al2="); Serial.print(altByte[2]);
  Serial.print(", al3="); Serial.print(altByte[3]);
  Serial.println();
}

void float2byte(float val,byte bytes_array[]){
  memcpy(bytes_array, &val, 4);
}

float byte2float(byte bytes_array[]){
  float out_float;
  memcpy(&out_float, bytes_array, 4);   // Assign bytes to input array
  return out_float;
}

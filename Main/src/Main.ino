///////////////////////////////////////////////////////////////////////////////
//                       PROGRAM HEADER
///////////////////////////////////////////////////////////////////////////////

// Libraries
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Buffer.h>
#include <SPIFlash.h>
#include <SPI.H>

// Constants and sizes
#define LED 3
#define PB 5
#define force_serial 1
#define bytes_alt 4
#define bytes_timestamp 2
#define baud_rate 115200
//#define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINT(x)     Serial.print (x)
 #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTLN(x)
#endif

// Declarations
//  Classes
Adafruit_BMP280 bmp;
Buffer cbuffer;
SPIFlash flash;
//  Data storage
byte altByte[bytes_alt], timeByte[bytes_timestamp];
uint16_t timestamp = 0, timestamp_ant = 0;
float timestamp_s = 0, alt;
uint32_t address = 0;
float ground_pressure = 1013.25;
//  Addresses
uint16_t pg = 0;
uint8_t bff[256];
//  Time
uint32_t last_bttn = 0, led_flash = 0, t_lastbmp = 0;
//  status
bool state = false;
uint8_t led_state = 0;
bool serial_init = 0;

uint8_t serialData = 0;
int x = 0;


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

  //DataInPage calculation + flash Initialization
  //dataInPage = (256)/(bytes_alt + bytes_timestamp);
  Serial.begin(baud_rate);
  while (!Serial) ; // Wait for Serial monitor to open
  flash.begin(MB64);

  // BMP180 Initialization
  DEBUG_PRINT(F("Initializing BMP280 ..."));
  if (!bmp.begin()) {
    DEBUG_PRINTLN(F("Unable to connect!"));
    while (1);
  }

  DEBUG_PRINTLN(F("Connection successful!"));
  bmp.osrs_p = 15; // 4.3.4 Table 21
  bmp.osrs_t = 1; // 4.3.4 Table 22
  bmp.t_sb = 0; // 3.6.3 Table 11, 4.3.5
  bmp.filter = 5; // 3.3.3
  get_gnd_pressure(ground_pressure);
  DEBUG_PRINT("p @ h=0m = "); DEBUG_PRINT(ground_pressure); DEBUG_PRINTLN(" Pa");
  cbuffer.reset(); // Buffer Initialization (head = 0; tail = 0)
}

void loop(){
  buttonCheck();
  checkLed();
  checkSerial();
  DEBUG_PRINTLN("Erasing chip...");
  if(flash.eraseChip()){
  DEBUG_PRINTLN("Erase chip success!");}else{DEBUG_PRINTLN("Erase chip failed!");}
  DEBUG_PRINTLN("Reading bmp280...");
  for(uint16_t ii=0;ii<50;ii++){
    read_bmp();
    buffer2flash();
  }
  DEBUG_PRINTLN("Reading flash...");
  printAllPages();
  while (1);
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

void checkSerial() {
  if (Serial.available() > 0) {
    serialData = Serial.parseInt();
    switchTask(serialData);
  }
}

void read_bmp(){

  if((millis()-t_lastbmp>2000 && state)  || 1)
  {
    // Acquire heigh measurement and time stamp
    alt = bmp.readAltitude(ground_pressure);
    timestamp = (micros()/100)%65536;

    // Load data into buffer
    float2byte(alt, altByte);  // Convert float to byte array
    timeByte[0] = timestamp & 255;
    timeByte[1] = timestamp>>8;
    if (cbuffer.spaceAvailable(sizeof(altByte) + sizeof(timeByte))) {    //Check if there's enough space for all data
      cbuffer.CarregarBuffer(altByte, sizeof(altByte));
      cbuffer.CarregarBuffer(timeByte, sizeof(timeByte));
    }else{
      DEBUG_PRINTLN("Not enough space in buffer");
    }
    t_lastbmp = millis();

    // Debugging

    DEBUG_PRINT("h="); DEBUG_PRINT(alt);
    DEBUG_PRINT(", ts="); DEBUG_PRINT(timestamp);
    /*
    DEBUG_PRINT(", tb0="); DEBUG_PRINT(timeByte[0]);
    DEBUG_PRINT(", tb1="); DEBUG_PRINT(timeByte[1]);
    DEBUG_PRINT(", al0="); DEBUG_PRINT(altByte[0]);
    DEBUG_PRINT(", al1="); DEBUG_PRINT(altByte[1]);
    DEBUG_PRINT(", al2="); DEBUG_PRINT(altByte[2]);
    DEBUG_PRINT(", al3="); DEBUG_PRINT(altByte[3]);
    */
    DEBUG_PRINTLN();
  }
}

void buffer2flash(){
  while(cbuffer.Check(256)) {
    DEBUG_PRINTLN("Writing page...");
    passDataToFlash();
  }
}

void buffer2serial(){
  cbuffer.DescarregarBuffer(altByte, sizeof(altByte));
  cbuffer.DescarregarBuffer(timeByte, sizeof(timeByte));

  timestamp = timeByte[0] | timeByte[1]<<8;
  timestamp_s+=((float) (timestamp-timestamp_ant))*0.0001;
  if(timestamp<timestamp_ant){ timestamp_s+=6.5536;}
  timestamp_ant = timestamp;

  alt = byte2float(altByte);

  Serial.print("h=");   Serial.print(alt);
  Serial.print(", ts=");   Serial.print(timestamp_s);

  // Debugging

  DEBUG_PRINT(", tb0="); DEBUG_PRINT(timeByte[0]);
  DEBUG_PRINT(", tb1="); DEBUG_PRINT(timeByte[1]);
  DEBUG_PRINT(", al0="); DEBUG_PRINT(altByte[0]);
  DEBUG_PRINT(", al1="); DEBUG_PRINT(altByte[1]);
  DEBUG_PRINT(", al2="); DEBUG_PRINT(altByte[2]);
  DEBUG_PRINT(", al3="); DEBUG_PRINT(altByte[3]);


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

void get_gnd_pressure(float &ground_pressure){
  uint8_t n=25;
  ground_pressure = 0;
  for(uint8_t i=0;i<25;i++){
      ground_pressure += bmp.readPressure();
      delay(50);
  }
  ground_pressure = ground_pressure/(float) (n*100);
}

void passDataToFlash(){
  DEBUG_PRINTLN("Downloading buffer...");
  cbuffer.DescarregarBuffer(bff,256);
  DEBUG_PRINTLN("Writing 2 flash...");
  flash.writeByteArray(pg++, 0, bff, 256, false);
  DEBUG_PRINTLN("Writing 2 flash finished...");
}

void printAllPages() {
  uint16_t i_pg = 0;
  DEBUG_PRINTLN("Reading all pages");
  cbuffer.reset();
  while(i_pg<pg){
    DEBUG_PRINT("Reading page "); DEBUG_PRINTLN(pg);
    flash.readByteArray(i_pg++, 0, bff, 256, false);
    cbuffer.CarregarBuffer(bff, sizeof(bff));
    while(cbuffer.Check(sizeof(altByte) + sizeof(timeByte))){
      buffer2serial();
    }
  }
}

void switchTask(uint8_t var){
  switch (var) {
    case 1:
      DEBUG_PRINTLN("Erasing chip...it can take a while");
      flash.eraseChip();
      DEBUG_PRINTLN("Chip erased!");
      break;
    case 9:
      printAllPages();
      break;
    default:
      break;
  }
}

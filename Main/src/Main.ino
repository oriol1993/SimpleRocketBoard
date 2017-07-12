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
#define initial_page 16
#define max_timestamp 65535
#define baud_rate 115200

// Declarations
Adafruit_BMP280 bmp;
Buffer cbuffer;
SPIFlash flash;

byte altByte[bytes_alt];
byte timeByte[bytes_timestamp];

bool state = false;
uint8_t dataInPage = 0;
uint8_t serialData = 0;
uint8_t led_state = 0;
uint16_t timestamp = 0;
uint16_t timestamp_ant = 0;
uint32_t timestamp_noOF = 0;
uint32_t last_bttn = 0;
uint32_t led_flash = 0;
uint32_t t_lastbmp = 0;
uint32_t repetitions = 0;
float alt = 0;
float timestamp_ms = 0;
float ground_pressure = 1013.25;
bool serial_init = 0;
unsigned long address = 0;


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
  dataInPage = (256)/(bytes_alt + bytes_timestamp);
  Serial.begin(baud_rate);
  while (!Serial) ; // Wait for Serial monitor to open
  flash.begin();

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
  get_gnd_pressure(ground_pressure);
  Serial.print("p @ h=0m = "); Serial.print(ground_pressure); Serial.println(" Pa");
  cbuffer.reset(); // Buffer Initialization (head = 0; tail = 0)
}

void loop(){
 buttonCheck();
 checkLed();
 checkSerial();
 if (state == true) {
   print_bmp();
 }
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

void print_bmp(){
  if(millis()-t_lastbmp>2000  || 1)
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
    }
    if(cbuffer.Check()) {
      Serial.println("Yepah");
      passDataToFlash();
    }

    t_lastbmp = millis();

    // Debugging
    /*Serial.print("h="); Serial.print(alt);
    Serial.print(", ts="); Serial.print(timestamp);
    Serial.print(", tb0="); Serial.print(timeByte[0]);
    Serial.print(", tb1="); Serial.print(timeByte[1]);
    Serial.print(", al0="); Serial.print(altByte[0]);
    Serial.print(", al1="); Serial.print(altByte[1]);
    Serial.print(", al2="); Serial.print(altByte[2]);
    Serial.print(", al3="); Serial.print(altByte[3]);
    Serial.println();*/
  }
}

void buffer2serial(){
  cbuffer.DescarregarBuffer(altByte, sizeof(altByte));
  cbuffer.DescarregarBuffer(timeByte, sizeof(timeByte));
  timestamp = timeByte[0] | timeByte[1]<<8;
  timestamp_ms = timestamp * 0.1;
  alt = byte2float(altByte);
  // Debugging
  Serial.print("h="); Serial.print(alt);
  Serial.print(", ts="); Serial.print(timestamp_ms);
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
  address = flash.readULong(0, 0, false);
  Serial.println("Adresa:");
  Serial.println(address);
  Serial.println("start a carregar a la flash");
  for (uint8_t i = 0; i < dataInPage; i++) {
    cbuffer.DescarregarBuffer(altByte, sizeof(altByte));
    cbuffer.DescarregarBuffer(timeByte, sizeof(timeByte));
    for(uint8_t j = 0; j < 4; j++) {
      flash.writeByte(address, (i*6)+j, altByte[j], true);
    }

    for (uint8_t k = 0; k < 2; k++) {
      flash.writeByte(address, 4+(6*i)+k, timeByte[k], true);
    }
  }
  flash.eraseSector(0, 0);
  flash.writeULong(0, 0, address + 1, false);
}

void printAllPages() {
  if (!Serial){
    Serial.begin(9600);
  }
  Serial.println("Reading all pages");
  uint8_t data_prova[256];
  uint8_t alt_buffer[4];
  uint8_t timestamp_buffer[2];
  uint32_t maxPage = flash.readULong(0, 0, false);
  for (int a = initial_page; a < maxPage; a++) {
    for (uint16_t j = 0; j < 256; j++) {
    data_prova[j] = flash.readByte(a, j, false);
  }

  for (uint8_t i = 0; i < dataInPage; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      alt_buffer[j] = data_prova[(6 * i) + j];
    }
    alt = byte2float(alt_buffer);
    Serial.print(alt);
    Serial.print(",");

    for (uint8_t j = 0; j < 2; j++) {
      timestamp_buffer[j] = data_prova[4 + (6 * i) + j];
    }
    timestamp = timestamp_buffer[0] | timestamp_buffer[1]<<8;
    if (timestamp < timestamp_ant) {
      repetitions ++;
      timestamp_noOF = (repetitions * max_timestamp) + timestamp;
    }
    else {timestamp_noOF = timestamp + (max_timestamp * repetitions);}
    timestamp_ant = timestamp;
    timestamp_ms = timestamp_noOF * 0.1;
    Serial.println(timestamp_noOF);
    }
  }
}

void switchTask(uint8_t var){
  switch (var) {
    case 1:
      Serial.println("Erasing chip...");
      for (uint32_t i = 0; i < flash.getMaxPage()/256; i++) {
        flash.eraseBlock64K(i*256, 0);
      }
      Serial.println("Chip erased!");
      if (flash.writeULong(0, 0, initial_page, false)) {
        Serial.print("Initial page = ");
        Serial.println(flash.readULong(0, 0, false));
      }
      break;
    case 9:
      printAllPages();
      break;
    default:
      break;
  }
}

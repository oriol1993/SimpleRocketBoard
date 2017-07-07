#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Buffer.h>
Adafruit_BMP280 bmp;
Buffer cbuffer;

#define LED 3
#define PB 5
#define force_serial 1

uint8_t state = 0;
uint32_t last_bttn = 0;
uint32_t led_flash = 0;
uint8_t led_state = 0;
uint32_t t_lastbmp = 0;
float alt = 0;
uint16_t timestamp = 0;
bool serial_init = 0;

void setup();
void loop();
void buttonCheck();
void checkLed();
void floatToByte();

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
  bmp.osrs_p = 15; // 4.3.4 Table 21
  bmp.osrs_t = 1; // 4.3.4 Table 22
  bmp.t_sb = 0; // 3.6.3 Table 11, 4.3.5
  bmp.filter = 5; // 3.3.3

  Serial.print(F("Initializing BMP280 ..."));
  if (!bmp.begin()) {
      Serial.println(F("Unable to connect!"));
      while (1);
    }else{
      Serial.println(F("Connection successful!"));
    }

  // Buffer Initialization (head = 0; tail = 0)
  cbuffer.reset();
  }

void loop(){
 buttonCheck();
 checkLed();
 print_bmp();
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
  if(millis()-t_lastbmp>2000)
  {
    alt = bmp.readAltitude(1013.25); // this should be adjusted to your local forcase
    timestamp ++;
    byte altByte[4];  //Creo un array de 4 bytes buit
    floatToByte(alt, altByte[0]);  //Converteixo el array buit en un array ple amb el float (alt)
    cbuffer.CarregarBuffer(altByte[0], 4); //Paso la altura al buffer
    byte timeByte[2]; //Creo un array de 2 bytes buit
    floatToByte(timestamp, timeByte[0]);  //Converteixo el array buit en un array amb el timestamp
    cbuffer.CarregarBuffer(timeByte[0], 2); //Paso el timestamp al buffer
    t_lastbmp = millis();
  }
}

void floatToByte(float val,byte &bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    byte temp_array[4];
  } u;
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
}

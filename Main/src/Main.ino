#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp;

#define LED 3
#define PB 5

uint8_t state = 0;
uint32_t last_bttn = 0;
uint32_t led_flash = 0;
uint8_t led_state = 0;
uint32_t t_lastbmp = 0;

void setup();
void loop();
void buttonCheck();
void checkLed();

void setup(){
 pinMode(LED, OUTPUT);
 pinMode(PB, INPUT_PULLUP);
 Serial.begin(9600);
 Serial.println(F("BMP280 test"));

 bmp.osrs_p = 15;
 bmp.osrs_t = 1;
 bmp.t_sb = 0;
 bmp.filter = 5;

 if (!bmp.begin()) {
   Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
   while (1);
 }else{
   Serial.println(F("BMP Initialized!"));
 }
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
  {  Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25)); // this should be adjusted to your local forcase
    Serial.println(" m");
    Serial.println();
    t_lastbmp = millis();
  }
}

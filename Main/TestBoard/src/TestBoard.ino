
#define LED 3
#define PB 5

void setup(){
  pinMode(LED, OUTPUT);
  pinMode(PB, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop(){

}

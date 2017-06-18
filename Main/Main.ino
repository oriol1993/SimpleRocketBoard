
#define LED 3
#define PB 5
uint8_t state = 0;
uint32_t last_bttn = 0;
uint32_t led_flash = 0;
uint8_t led_state = 0;

void setup()
{
	pinMode(LED, OUTPUT);
	pinMode(PB, INPUT_PULLUP);
	Serial.begin(9600);
}

void loop()
{
	buttonCheck();
	checkLed();
}

void buttonCheck()
{
	if (millis() - last_bttn > 300)
	{
		if (digitalRead(PB) == 0){ 
			state = !state;
			last_bttn = millis();
			led_flash = millis();
		}
	}
}

void checkLed()
{
	if (state == true)
	{
		if (millis() - led_flash > 500)
		{
			led_state = !led_state;
			led_flash = millis();
			digitalWrite(LED, led_state);
		}
	}
	else {
		digitalWrite(LED, LOW);
	}

}
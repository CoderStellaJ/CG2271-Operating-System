/**********************************************************************/
// Lab2 Q2
/***********************************************************************/
//#include <Arduino.h>
//#include <stdlib.h>
//#include <limits.h>
//
//int fun1(int x, int y)
//{
//	return x+y;
//}
//
//int fun2(int x, int y)
//{
//	return x*y;
//}
//
//void setup()
//{
//	Serial.begin(115200);
//}
//
//// Declare the function pointer
//int (*funcptr)(int, int);
//void loop()
//{
//	float turn=(float) rand() / INT_MAX;
//	int result;
//	if(turn>0.5)
//	funcptr=fun1;
//	else
//	funcptr=fun2;
//	// Invoke the function
//	result=funcptr(2,3);
//	Serial.print("Computation result:");
//	Serial.println(result);
//	//200ms pause
//	delay(200);
//}


/*******************************************************************/
// Lab1 part2 Q8
/******************************************************************/
//#include <Arduino.h>
//#include <math.h>
//#define PIN_LED1 6
//#define PIN_LED2 7
//#define PIN_PTTM 0
//#define PIN_TOUCH 1
//#define PIN_PB1 2
//#define PIN_PB2 3
//#define DEBOUNCE_DELAY 70
//int state_led6 = 1;
//int state_led7 = 1;
//int lastDebounce1 = 0;
//int lastDebounce2 = 0;
//int lastValue = 0;
//int lastTouch = 0;
//
//int remapTouch(int val) {
//	return (int)floor(75+(val-714)*375/(950.0-714.0)) ;
//}
//
//int remapPttm(int val) {
//	return (int)floor(val/4.0);
//}
//void isr1()
//{
//	int button = digitalRead(PIN_PB1);
//	if (millis() - lastDebounce1 > DEBOUNCE_DELAY) {
//		if (button == HIGH) {
//		state_led6 = 0;
//		lastDebounce1 = millis();
//		}
//	}
//}
//
//void isr2()
//{
//int button = digitalRead(PIN_PB2);
//	if (millis() - lastDebounce2 > DEBOUNCE_DELAY) {
//		if (button == HIGH) {
//		state_led7 = 0;
//		lastDebounce2 = millis();
//		}
//	}
//}
//
//void setup()
//{
//	attachInterrupt(0, isr1, RISING);
//	attachInterrupt(1, isr2, RISING);
//	pinMode(PIN_LED1, OUTPUT);
//	pinMode(PIN_LED2, OUTPUT);
//}
//
//void loop()
//{
//	int val,touch, inputVal, inputTouch;
//	val = analogRead(PIN_PTTM);
//	val = remapPttm(val);
//	touch = analogRead(PIN_TOUCH);
//	touch = remapTouch(touch);
//	if(state_led6 == 1) {
//		lastValue = val;
//	}
//	inputVal = lastValue;
//	analogWrite(PIN_LED1, inputVal);
//	if(state_led7 == 1) {
//		lastTouch = touch;
//	}
//	inputTouch = lastTouch;
//	analogWrite(PIN_LED2, 255);
//	delay(inputTouch);
//	analogWrite(PIN_LED2, 0);
//	delay(inputTouch);
//}


/*******************************************************************/
// Lab1 part2 Q7
/******************************************************************/
//#include <Arduino.h>
//
//#define PIN_LED 6
//#define PIN_PB 3
//#define DEBOUNCE_DELAY 70
//
//int state = LOW;
//int lastDebounce = 0;
//
//
//void isr()
//{
//	int button = digitalRead(PIN_PB);
//	if (millis() - lastDebounce > DEBOUNCE_DELAY) {
//		if (button == HIGH) {
//			state = !state;
//			lastDebounce = millis();
//		}
//	}
//}
//
//
//void setup()
//{
//	attachInterrupt(1, isr, RISING);
//// 	pinMode(PIN_PB , INPUT_PULLUP);
//	pinMode(PIN_LED, OUTPUT);
//}
//
//void loop()
//{
//	digitalWrite(PIN_LED, state);
//}

/*******************************************************************/
// Lab1 part2 Q6
/******************************************************************/
//#include <Arduino.h>
//#include <math.h>
//#include <stdio.h>
//#define PIN_LED_1 6
//#define PIN_LED_2 7
//#define PIN_PTTM 0
//#define PIN_TOUCH 1
//
//int remapTouch(int val) {
//	return (int)floor(75+(val-714)*375/(950.0-714.0));
//}
//
//int remapPttm(int val) {
//	return (int)floor(val/4.0);
//}
//
//void setup() {
//	pinMode(PIN_LED_1, OUTPUT);
//	pinMode(PIN_LED_2, OUTPUT);
//	Serial.begin(115200);
//}
//
//void loop() {
//	int val,touch;
//	val = analogRead(PIN_PTTM);
//	val = remapPttm(val);
//	touch = analogRead(PIN_TOUCH);
//	Serial.print(touch);
//	touch = remapTouch(touch);
//	Serial.print(" ");
//	analogWrite(PIN_LED_1, val);
//	analogWrite(PIN_LED_2, 255);
//	delay(touch);
//	analogWrite(PIN_LED_2, 0);
//	delay(touch);
//}

/*******************************************************************/
// Lab1 part2 Q4/5
/******************************************************************/
//#include <Arduino.h>
//#include <math.h>
//#define PIN_PTTM 0
//#define PIN_TOUCH 1
//
//int remap(int val) {
//	return (int)floor(75+(val-714)*375/(950.0-714.0));
//}
//
//void setup() {
//	// initialize serial communications at 115200 bps:
//	Serial.begin(115200);
//}
//void loop() {
//int val, touch,final;
//	// read potentiometer's value
//	val = analogRead(PIN_PTTM);
//	// read touch sensor's value
//	touch = analogRead(PIN_TOUCH);
//	final = remap(touch);
//	// dump them to serial port
//	Serial.print(val);
//	Serial.print(" ");
//	Serial.print(touch);
//	Serial.print(" ");
//	Serial.print(final);
//	Serial.print(remap(touch));
//	Serial.println();
//	// 200ms pause
//	delay(200);
//}

/*******************************************************************/
// Lab1 part2 Q3
/******************************************************************/
//#include <math.h>
//
//int remap(int val) {
//	return (int)floor(val/4.0);
//}

/*******************************************************************/
// Lab1 part2 Q2
/******************************************************************/
//#include <Arduino.h>
//#define PIN_LED 6
//#define PIN_PTTM 0
//
//void setup() {
//	pinMode(PIN_LED, OUTPUT);
//}
//
//void loop() {
//	int val;
//	val = analogRead(PIN_PTTM);
//	val = remap(val);
//	analogWrite(PIN_LED, val);
//	delay(500);
//}

/*******************************************************************/
// Lab1 part1
/******************************************************************/
//#include <avr/io.h> // Header file to access Atmega328 I/O registers
//#include <Arduino.h> // Header file for the Arduino library
//#define GREEN_PIN 12
//#define RED_PIN 13
//void blink_led(unsigned pinnum) {
//	digitalWrite(pinnum, HIGH); // Set digital I/O pin to a 1
//	delay(100); // Delay
//	digitalWrite(pinnum, LOW); // Set digital I/O pin to a 0
//	delay(1000); // Delay
//}
//void setup() {
//	pinMode(GREEN_PIN, OUTPUT); // Set digital I/O pins 12
//	pinMode(RED_PIN, OUTPUT); // and 13 to OUTPUT.
//}
//void loop () {
//	blink_led(RED_PIN);
//	blink_led(GREEN_PIN);
//}

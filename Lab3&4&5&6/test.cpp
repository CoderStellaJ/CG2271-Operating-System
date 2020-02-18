///////////// Lab 5 Try Version 1/////////////////////////////////////
/**
 * CG2271 Lab 6 - C.A.R.T.O.S. (Car And RTOS)
 * @author Justin Ng, Lim Hong Wei, CEG2 2017
 */

//#include <Arduino.h>
//#include <avr/io.h>
//#include <FreeRTOS.h>
//#include <task.h>
//#include <queue.h>
//
//#define STACK_SIZE 100
//#define BUZZER_PIN 9
//#define RED_PIN 10
//#define FAST_PIN 7
//#define MED_PIN 6
//#define SLOW_PIN 5
//#define PTTM_PIN 0
//#define DEBOUNCE_DELAY 200
//
//static unsigned long previousPressedTime0 = 0, previousPressedTime1 = 0;
//static unsigned int currentSpeed = 0;
//static unsigned int desiredSpeed = 0;
//static unsigned int emergencyBrake = 0;
//
//QueueHandle_t xMessageQueue;
//QueueHandle_t xSafeDistanceQueue;
//
//struct Message {
//    int mDistance;
//    int mCurrentSpeed;
//    int mDesiredSpeed;
//};
//
//void updateLedAndBuzzer(void *p) {
//	portTickType xRedLedBeginTime;
//	for (;;) {
//		switch (currentSpeed) {
//		case 3:
//			digitalWrite(FAST_PIN, HIGH);
//			digitalWrite(MED_PIN, HIGH);
//			digitalWrite(SLOW_PIN, HIGH);
//			break;
//		case 2:
//			digitalWrite(FAST_PIN, LOW);
//			digitalWrite(MED_PIN, HIGH);
//			digitalWrite(SLOW_PIN, HIGH);
//			break;
//		case 1:
//			digitalWrite(FAST_PIN, LOW);
//			digitalWrite(MED_PIN, LOW);
//			digitalWrite(SLOW_PIN, HIGH);
//			break;
//		case 0:
//			digitalWrite(FAST_PIN, LOW);
//			digitalWrite(MED_PIN, LOW);
//			digitalWrite(SLOW_PIN, LOW);
//		}
//		tone(BUZZER_PIN, currentSpeed*500+31); // 31Hz is base tone, other speeds vary step 500Hz.
//		if (emergencyBrake == 1) {
//			xRedLedBeginTime = xTaskGetTickCount();
//			digitalWrite(RED_PIN, HIGH);
//			emergencyBrake = 2;
//		} else if (emergencyBrake == 2 && xTaskGetTickCount() == xRedLedBeginTime+1000) {
//			digitalWrite(RED_PIN, LOW);
//			emergencyBrake = 0;
//		}
//		vTaskDelay(5);
//	}
//}
//
//void readAndSendSafeDistanceAhead(void *p) {
//	const portTickType xFrequency = 400;
//	portTickType xLastWakeTime = 0;
//	int safeDistance, distance;
//	for (;;) {
//		distance  = analogRead(PTTM_PIN);
//		if (distance < 256) {
//			safeDistance = 0;
//		} else if (distance < 512) {
//			safeDistance = 1;
//		} else if (distance < 768) {
//			safeDistance = 2;
//		} else if (distance < 1024) {
//			safeDistance = 3;
//		}
//		// Send safe distance to speed controller
//		xQueueSendToBack(xSafeDistanceQueue, &safeDistance, (TickType_t) 1);
//		vTaskDelayUntil(&xLastWakeTime, xFrequency);
//	}
//}
//
//void sendToUart(void *p) {
//	const portTickType xFrequency = 1000;
//	portTickType xLastWakeTime = 0;
//	for (;;) {
//		struct Message *pxRxedMessage;
//		if (xQueueReceive(xMessageQueue, &(pxRxedMessage), (TickType_t) 1) == pdTRUE) {
//			Serial.print("C");
//			Serial.print((int) pxRxedMessage->mCurrentSpeed);
//			Serial.print(" D");
//			Serial.print((int) pxRxedMessage->mDesiredSpeed);
//			Serial.print(" d");
//			Serial.println((int) pxRxedMessage->mDistance);
//		}
//		vTaskDelayUntil(&xLastWakeTime, xFrequency);
//	}
//}
//
//void adjustSpeed(void *p) {
//	for (;;) {
//		int oldSpeed = currentSpeed;
//		int safeDistance;
//		if (xQueueReceive(xSafeDistanceQueue, &safeDistance, (TickType_t) 1) == pdTRUE) {
//			currentSpeed = min(desiredSpeed, safeDistance);
//			// Latter part ensures that brakelight not activated by button.
//			if (currentSpeed < oldSpeed && safeDistance < desiredSpeed) {
//				emergencyBrake = 1;
//			}
//			// Send mesage to UART
//			Message message = {safeDistance+1, (int)currentSpeed, (int)desiredSpeed};
//			struct Message *pxMessage;
//			pxMessage = &message;
//			xQueueSendToBack(xMessageQueue, (void*)&pxMessage, (TickType_t) 1);
//		}
//		vTaskDelay(5);
//	}
//}
//
//static void increaseSpeed() {
//	if (desiredSpeed < 3) {
//		desiredSpeed++;
//	}
//}
//
//static void decreaseSpeed() {
//	if (desiredSpeed > 0) {
//		desiredSpeed--;
//	}
//}
//
//void int0ISR() {
//	unsigned long currentTime = millis();
//	if (currentTime - previousPressedTime0 > DEBOUNCE_DELAY) {
//		increaseSpeed();
//	}
//	previousPressedTime0 = currentTime;
//}
//
//void int1ISR() {
//	unsigned long currentTime = millis();
//	if (currentTime - previousPressedTime1 > DEBOUNCE_DELAY) {
//		decreaseSpeed();
//	}
//	previousPressedTime1 = currentTime;
//}
//
//void setup()
//{
//	Serial.begin(115200);
//	attachInterrupt(1, int0ISR, RISING);
//	attachInterrupt(0, int1ISR, RISING);
//	pinMode(FAST_PIN, OUTPUT);
//	pinMode(MED_PIN, OUTPUT);
//	pinMode(SLOW_PIN, OUTPUT);
//	pinMode(RED_PIN, OUTPUT);
//	pinMode(BUZZER_PIN, OUTPUT);
//	xMessageQueue = xQueueCreate(3, sizeof(struct Message *)); 	// Create message queue storing pointers to Message structs.
//	xSafeDistanceQueue = xQueueCreate(3, sizeof(int));
//}
//
//void loop() {
//	xTaskCreate(updateLedAndBuzzer,           // Pointer to the task entry function
//		     "UpdateSpeedLed",         // Task name
//		     STACK_SIZE,      // Stack size
//		     NULL,       // Pointer that will be used as parameter
//		     4,               // Task priority
//		    NULL);           // Used to pass back a handle by which the created task can be referenced.
//
//	xTaskCreate(readAndSendSafeDistanceAhead,           // Pointer to the task entry function
//		     "ReadDistanceAhead",         // Task name
//		     STACK_SIZE,      // Stack size
//		     NULL,       // Pointer that will be used as parameter
//		     3,               // Task priority
//		    NULL);           // Used to pass back a handle by which the created task can be referenced.
//
//	xTaskCreate(adjustSpeed,           // Pointer to the task entry function
//		     "AdjustSpeed",         // Task name
//		     STACK_SIZE,      // Stack size
//		     NULL,       // Pointer that will be used as parameter
//		     2,               // Task priority
//		    NULL);           // Used to pass back a handle by which the created task can be referenced.
//
//	xTaskCreate(sendToUart,           // Pointer to the task entry function
//		     "SendToUart",         // Task name
//		     STACK_SIZE,      // Stack size
//		     NULL,       // Pointer that will be used as parameter
//		     1,               // Task priority
//		    NULL);           // Used to pass back a handle by which the created task can be referenced.
//
//	vTaskStartScheduler();
//}

////////////   Lab 5 Version 2///////////////////////////////////////
//#include <Arduino.h>
//#include <avr/io.h>
//#include <FreeRTOS.h>
//#include <task.h>
//#include <semphr.h>
//#include <queue.h>
//
//#define STACK_SIZE 200
//#define ledBrake 10
//#define ledY1 6
//#define ledY2 7
//#define ledY3 8
//
//#define PUSH_B1 2
//#define PUSH_B2 3
//#define SPEAKER 9
//#define PTTM 0
//
//unsigned long previousTime = 0;
//volatile int brake_flag = 0;
//volatile int current_speed = 0;
//volatile int desired_speed = 0;
//
//unsigned long lastDebounceTime1 = 0;
//unsigned long lastDebounceTime2 = 0;
//unsigned long debounceDelay1 = 500;
//unsigned long debounceDelay2 = 500;
//
//QueueHandle_t xQueueUART = 0;
//QueueHandle_t xQueueSafeDist = 0;
//
////remap potentiometer from [0-1023] to [0-255]
//int remapDist(int value){
//	return (int) (value * 255.0 / 1023.0);
//}
//
////indicates safe brake
//void brakeFlag(void){
//	TickType_t xCurrTime = xTaskGetTickCount();
//	if (brake_flag == 1){
//
//		if (xCurrTime - previousTime >= 1000){
//			previousTime = xCurrTime;
//
//			digitalWrite(ledBrake, HIGH);
//
//		}
//		brake_flag = 0;
//	}
//}
//
//void int0ISR(void){
//
//	if((millis() - lastDebounceTime1) > debounceDelay1){
//		if(current_speed < 3){
//			current_speed++;
//		}
//		desired_speed = current_speed;
//		lastDebounceTime1 = millis();
//	}
//
//}
//
//void int1ISR(){
//
//	if((millis() - lastDebounceTime2) > debounceDelay2){
//		if(current_speed > 0){
//			current_speed--;
//		}
//		desired_speed = current_speed;
//		lastDebounceTime2 = millis();
//	}
//}
//
////decreases current speed after safe brake and get the minimum value in the pair of desired speed and safe speed
//void setSafeSpeed(int dist){
//	if( dist == 1){
//		current_speed = 0;
//	}
//	else if(dist == 2){
//		if(desired_speed > 0)
//			current_speed = 1;
//	}
//	else if(dist == 3){
//		if(desired_speed == 3)
//			current_speed = 2;
//	}
//	else {
//		if(desired_speed < 3){
//			current_speed = desired_speed;
//		}
//		else{
//			current_speed = 3;
//		}
//	}
//}
//
//void speed_task(void *p){
//
//	TickType_t xPrevTime;
//	const TickType_t xPeriod = 1000;
//
//	while(1){
//
//		int safeDistance;
//		xPrevTime = xTaskGetTickCount();
//
//
//		xQueueReceive(xQueueSafeDist, &safeDistance, portMAX_DELAY);
//		digitalWrite(ledBrake, LOW);
//		if((safeDistance < 2) && (current_speed == 1)){
//			brake_flag = 1;
//			digitalWrite(ledBrake, LOW);
//			brakeFlag();
//		}
//		else if((safeDistance < 3) && (current_speed == 2)){
//			brake_flag = 1;
//			digitalWrite(ledBrake, LOW);
//
//			brakeFlag();
//		}
//		else if((safeDistance < 4) && (current_speed == 3)){
//			brake_flag = 1;
//
//			digitalWrite(ledBrake, LOW);
//
//			brakeFlag();
//		}
//		setSafeSpeed(safeDistance);
//
//
//		if (current_speed == 0){
//			digitalWrite(ledY1, LOW);
//			digitalWrite(ledY2, LOW);
//			digitalWrite(ledY3, LOW);
//	//		digitalWrite(ledBrake, LOW);
//			tone(SPEAKER, 1519);
//		}
//
//		else if (current_speed == 1){
//
//			digitalWrite(ledY1, HIGH);
//			digitalWrite(ledY2, LOW);
//			digitalWrite(ledY3, LOW);
//		//	digitalWrite(ledBrake, LOW);
//			tone(SPEAKER, 1432);
//
//		}
//
//		else if (current_speed == 2){
//
//			digitalWrite(ledY1, HIGH);
//			digitalWrite(ledY2, HIGH);
//			digitalWrite(ledY3, LOW);
//			tone(SPEAKER, 1136 );
//	//		digitalWrite(ledBrake, LOW);
////			if(safeDistance < 3){
////				brake_flag = 1;
////
////				digitalWrite(ledBrake, LOW);
////				//setSafeSpeed(safeDistance);
////				brakeFlag();
////			}
//
//		}
//
//		else if (current_speed == 3){
//
//			digitalWrite(ledY1, HIGH);
//			digitalWrite(ledY2, HIGH);
//			digitalWrite(ledY3, HIGH);
//			tone(SPEAKER, 956);
//
////			digitalWrite(ledBrake, LOW);
////			if(safeDistance < 4){
////				brake_flag = 1;
////
////				digitalWrite(ledBrake, LOW);
////				//setSafeSpeed(safeDistance);
////				brakeFlag();
////			}
//
//		}
//
//		vTaskDelayUntil(&xPrevTime, xPeriod);
//	}
//
//}
//
//void distance_task(void *p) {
//	int distance = 0;
//	TickType_t xPrevTime;
//	const TickType_t xPeriod = 1000;
//	xPrevTime = xTaskGetTickCount();
//	while(1){
//		int value;
//		value = analogRead(PTTM);
//		value = remapDist(value);
//		if (value <= 64){
//			distance = 1;
//
//		}
//		else if ((value > 64) && (value <= 128)){
//			distance = 2;
//
//		}
//		else if ((value > 128) && (value <= 192)){
//			distance = 3;
//		}
//		else if (value > 192){
//			distance = 4;
//		}
//		//Sends safe distance to speed Task
//		xQueueSend(xQueueSafeDist, &distance, portMAX_DELAY);
//		//Sends safe distance to serialPrint
//		xQueueSend(xQueueUART, &distance, portMAX_DELAY);
//
//		vTaskDelayUntil(&xPrevTime, xPeriod);
//	}
//
//}
//
////Prints information on UART
//void serialPrint(void *p){
//	while(1){
//		int taskDistance;
//		xQueueReceive(xQueueUART, &taskDistance, portMAX_DELAY);
//		Serial.print("Desired Speed: ");
//		Serial.println(desired_speed);
//		Serial.print("Current Speed: ");
//		Serial.println(current_speed);
//		Serial.print("Distance: ");
//		if( taskDistance != 1)
//			Serial.print(taskDistance);
//		Serial.println("d");
//	}
//}
//
//
//void setup() {
//	Serial.begin(115200);
//	pinMode(ledBrake, OUTPUT);
//	pinMode(ledY1, OUTPUT);
//	pinMode(ledY2, OUTPUT);
//	pinMode(ledY3, OUTPUT);
//	pinMode(SPEAKER, OUTPUT);
//	pinMode(PUSH_B1, INPUT);
//	pinMode(PUSH_B2, INPUT);
//	attachInterrupt(0,int0ISR, RISING);
//	attachInterrupt(1,int1ISR, RISING);
//}
//
//void loop() {
//	//create message queues
//	xQueueUART = xQueueCreate(10, sizeof(unsigned long));
//	xQueueSafeDist = xQueueCreate(10, sizeof(unsigned long));
//
//	//create tasks
//	xTaskCreate(speed_task, "Task1", STACK_SIZE, NULL, 3, NULL);
//
//	xTaskCreate(distance_task, "Task2", STACK_SIZE,  NULL, 2, NULL);
//
//	xTaskCreate(serialPrint, "Task3", STACK_SIZE,  NULL, 1, NULL);
//
//	vTaskStartScheduler();
//}

/////////////////// Lab 5 Our Code/////////////////////////////////////////////
#include <Arduino.h>
#include <avr/io.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>


#define STACK_SIZE 200

#define LED_YELLOW1 6
#define LED_YELLOW2 7
#define LED_YELLOW3 8
#define LED_RED 9

#define PUSH_B1 2  //accelerator
#define PUSH_B2 3  //brake
#define PTTM 0	//distance

#define DEBOUNCE_DELAY 70

int lastTime = 0;
int brake_flag = 0;

int desired_speed = 0;
int current_speed = 0;

int lastDebounceTime1 = 0;
int lastDebounceTime2 = 0;

QueueHandle_t xQueueDistance = 0;
QueueHandle_t xQueueUART = 0;


//int0ISR for accelerating, set desired speed
void int0ISR(void){
	if((millis() - lastDebounceTime1) > DEBOUNCE_DELAY){
		desired_speed = min(3, current_speed+1);
//		Serial.print(desired_speed);
		lastDebounceTime1 = millis();
	}
}

//int1ISR for braking, set desired speed
void int1ISR(){
	if((millis() - lastDebounceTime2) > DEBOUNCE_DELAY){
		desired_speed = max(0, current_speed-1);
//		Serial.print(desired_speed);
		lastDebounceTime2 = millis();
	}
}

//set the speed based on safe speed and current speed
void set_speed(int dist){
	int safe_speed = dist - 1;
	current_speed = min(safe_speed, desired_speed);
}

//brake the car, show red LED on periods
void brake_func(void){
	TickType_t xCurrTime = xTaskGetTickCount();
	Serial.print(brake_flag);
	if (brake_flag == 1){
		if (xCurrTime - lastTime >= 1000){
			//turn on for a period
			lastTime = xCurrTime;
			digitalWrite(LED_RED, HIGH);
		}
	brake_flag = 0;
	}
}

//control the LEDs for speed
void speed_task(void *p){
	TickType_t xPrevTime;
	const TickType_t xPeriod = 1000;

	while(1){
		int distance;
		xPrevTime = xTaskGetTickCount();
		xQueueReceive(xQueueDistance, &distance, portMAX_DELAY);
		digitalWrite(LED_RED, LOW);

		//set red led for safety braking
		if((distance < 2) && (current_speed >= 1)){
			brake_flag = 1;
			digitalWrite(LED_RED, LOW);
			brake_func();
		}else if((distance < 3) && (current_speed >= 2)){
			brake_flag = 1;
			digitalWrite(LED_RED, LOW);
			brake_func();
		}else if((distance < 4) && (current_speed >= 3)){
			brake_flag = 1;
			digitalWrite(LED_RED, LOW);
			brake_func();
		}

		//set speed based on the current distance
		set_speed(distance);

		//LED signals for speed
		if (current_speed == 0){
			digitalWrite(LED_YELLOW1, LOW);
			digitalWrite(LED_YELLOW2, LOW);
			digitalWrite(LED_YELLOW3, LOW);
		}
		else if (current_speed == 1){
			digitalWrite(LED_YELLOW1, HIGH);
			digitalWrite(LED_YELLOW2, LOW);
			digitalWrite(LED_YELLOW3, LOW);
		}
		else if (current_speed == 2){
			digitalWrite(LED_YELLOW1, HIGH);
			digitalWrite(LED_YELLOW2, HIGH);
			digitalWrite(LED_YELLOW3, LOW);
		}

		else if (current_speed == 3){
			digitalWrite(LED_YELLOW1, HIGH);
			digitalWrite(LED_YELLOW2, HIGH);
			digitalWrite(LED_YELLOW3, HIGH);
		}

		vTaskDelayUntil(&xPrevTime, xPeriod);
	}

}

//potentiometer value mapped from [0-1023] to [0-255]
int map_pttm(int value){
	return (int)((value*1.0/1023.0)*255);
}

void distance_task(void *p) {
	int distance = 0;
	TickType_t xPrevTime;
	const TickType_t xPeriod = 1000;
	xPrevTime = xTaskGetTickCount();
	while(1){
		int value;
		value = analogRead(PTTM);
		value = map_pttm(value);
		if (value <= 64){
			distance = 1;
		}
		else if (value <= 128){
			distance = 2;
		}
		else if (value <= 192){
			distance = 3;
		}
		else{
			distance = 4;
		}
		//send distance from potentiometer to MQ
		xQueueSend(xQueueDistance, &distance, portMAX_DELAY);
		//send distance to UART to print out
		xQueueSend(xQueueUART, &distance, portMAX_DELAY);

		vTaskDelayUntil(&xPrevTime, xPeriod);
	}

}

//print information to UART terminal
void serialPrint(void *p){
	while(1){
		int distance;
		//receive distance from MQ
		xQueueReceive(xQueueUART, &distance, portMAX_DELAY);
		Serial.print("desired speed:");
		Serial.println(desired_speed);
		Serial.print("current speed:");
		Serial.println(current_speed);
		Serial.print("distance:");
		if(distance != 1){
			Serial.print(distance);
		}
		Serial.println("d");
	}
}


void setup() {
	Serial.begin(115200);

	pinMode(LED_YELLOW1, OUTPUT);
	pinMode(LED_YELLOW2, OUTPUT);
	pinMode(LED_YELLOW3, OUTPUT);
	pinMode(LED_RED, OUTPUT);

	pinMode(PUSH_B1, INPUT);
	pinMode(PUSH_B2, INPUT);

	attachInterrupt(0,int0ISR, FALLING);
	attachInterrupt(1,int1ISR, FALLING);
}


void loop() {
	//create message queues
	xQueueUART = xQueueCreate(10, sizeof(unsigned long));
	xQueueDistance = xQueueCreate(10, sizeof(unsigned long));

	//create tasks
	xTaskCreate(speed_task, "speed", STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(distance_task, "distance", STACK_SIZE,  NULL, 3, NULL);
	//UART print
	xTaskCreate(serialPrint, "print", STACK_SIZE,  NULL, 1, NULL);

	vTaskStartScheduler();
}


//////////////// Q7 /////////////////////////////////////////
//
//#include <Arduino.h>
//#include <avr/io.h>
//#include <FreeRTOS.h>
//#include <task.h>
//#include <semphr.h>
//
//#define STACK_SIZE 200
//#define LED_PIN1 6
//#define LED_PIN2 7
//#define LED_PIN3 8
//#define LED_PIN4 9
//
//SemaphoreHandle_t semaphore;
//TickType_t time1,time2,time3;
//
//void myDelay(int ms) {
//	for (int i = 0; i < ms; i++) {
//		delayMicroseconds(1000);
//	}
//}
//
//void task1Schedule(void *p) {
//	while(1) {
//		if (xSemaphoreTake(semaphore, 100000)) {
//			int count = 0;
//			while (count < 3000) {
//				digitalWrite(LED_PIN1, HIGH);
//				digitalWrite(LED_PIN4, HIGH);
//				myDelay(50);
//				digitalWrite(LED_PIN1, LOW);
//				myDelay(50);
//				count += 100;
//			}
//			digitalWrite(LED_PIN4, LOW);
//			xSemaphoreGive(semaphore);
//			vTaskDelayUntil(&time1, 10000);
//		}
//	}
//}
//void task2Schedule(void *p) {
//
//	while(1) {
//		int count = 0;
//		while (count < 4000) {
//			digitalWrite(LED_PIN2, HIGH);
//			myDelay(50);
//			digitalWrite(LED_PIN2, LOW);
//			myDelay(50);
//			count += 100;
//		}
//		vTaskDelayUntil(&time2, 15000);
//	}
//}
//void task3Schedule(void *p) {
//
//	while(1) {
//		if (xSemaphoreTake(semaphore, 100000)) {
//			int count = 0;
//			while (count < 10000) {
//				digitalWrite(LED_PIN3, HIGH);
//				digitalWrite(LED_PIN4, HIGH);
//				myDelay(50);
//				digitalWrite(LED_PIN3, LOW);
//				myDelay(50);
//				count += 100;
//			}
//			digitalWrite(LED_PIN4, LOW);
//			xSemaphoreGive(semaphore);
//			vTaskDelayUntil(&time3, 35000);
//		}
//	}
//}
//
//
//
//void setup() {
//	pinMode(LED_PIN1, OUTPUT);
//	pinMode(LED_PIN2, OUTPUT);
//	pinMode(LED_PIN3, OUTPUT);
//	pinMode(LED_PIN4, OUTPUT);
//
//	semaphore = xSemaphoreCreateMutex();
//}
//
//void loop() {
//
//	xTaskCreate(task1Schedule, "Task1",STACK_SIZE,NULL,4,NULL);
//	xTaskCreate(task2Schedule,"Task2",STACK_SIZE,NULL,3,NULL);
//	xTaskCreate(task3Schedule,"Task3",STACK_SIZE,NULL,2,NULL);
//
//	TickType_t time1 = xTaskGetTickCount();
//
//	time2 = time1;
//	time3 = time2;
//
//	vTaskStartScheduler();
//}


///////////// Q6 (A) ////////////////////////////////////

//#include <Arduino.h>
//#include <avr/io.h>
//#include <FreeRTOS.h>
//#include <task.h>
//#include <semphr.h>
//#define STACK_SIZE 200
//
//xQueueHandle q;
//SemaphoreHandle_t xSemaphore = NULL;
//
//
//void producer1(void *p) {
////	Initialize <counter> to 0;
//	int counter = 0;
//	while (1) {
//		counter = counter + 2;
//		if(xQueueSendToBack(q, (void *)&counter, 0) != pdPASS ) {
//			if (xSemaphoreTake(xSemaphore, (TickType_t)10)==pdTRUE) {
//				Serial.println("Message queue full");
//				xSemaphoreGive(xSemaphore);
//			}
//		}
////		increment <counter> by 2; // send only even values
////		send <counter> to message queue;
////		If (error in sending)
////		print ¡°Message queue full¡± on the serial port;
//
//	}
//}
//
//void producer2(void *p) {
//	int counter = 1;
//	while (1) {
//		counter = counter + 2;
//		if(xQueueSendToBack(q, (void *)&counter, 0) != pdPASS ) {
//			if (xSemaphoreTake(xSemaphore, (TickType_t)10)==pdTRUE) {
//				Serial.println("Message queue full");
//				xSemaphoreGive(xSemaphore);
//			}
//		}
//
////		increment <counter> by 2; // send only odd values
////		send <counter> to message queue;
////		If (error in sending)
////		print ¡°Message queue full¡± on the serial port;
//	}
//}
//
//void consumer(void *p) {
//	while (1) {
//		int counter;
//		if(xQueueReceive(q, &counter, portMAX_DELAY) == pdTRUE){
//			if (xSemaphoreTake(xSemaphore, (TickType_t)10)==pdTRUE) {
//				Serial.print("receiving = ");
//				Serial.println(counter);
//				xSemaphoreGive(xSemaphore);
//			}
//		}
//
//		int num = uxQueueMessagesWaiting(q);
//
//		if (xSemaphoreTake(xSemaphore, (TickType_t)10)==pdTRUE) {
//			Serial.print("number of items in queue = ");
//			Serial.println(num);
//			xSemaphoreGive(xSemaphore);
//		}
//
////		receive <counter> from message queue;
////		print ¡°receiving = <counter>¡± on the serial port;
////		print ¡°number of items in queue = ..¡± on the serial port;
//	}
//}
//
//void setup() {
//	Serial.begin(115200);
//	q = xQueueCreate(5, sizeof(unsigned long));
//}
//
//void loop() {
//	xSemaphore = xSemaphoreCreateMutex();
//	//same priority
//	xTaskCreate(producer1, "producer1", STACK_SIZE, (void * ) 1, 2, NULL);
//	xTaskCreate(producer2, "producer2", STACK_SIZE, (void * ) 2, 2, NULL);
//	xTaskCreate(consumer, "consumer", STACK_SIZE, (void * ) 3, 2, NULL);
////	xTaskCreate(serialPrint, "SerialPrint", STACK_SIZE, (void * ) 4, 3, NULL);
//	/* start scheduler */
//	vTaskStartScheduler();
//}
//

///////////////////////// Try //////////////////////////////

//#include <Arduino.h>
//#include <avr/io.h>
//#include <FreeRTOS.h>
//#include <task.h>
//#include <semphr.h>
//#define STACK_SIZE 200
//#define DELAY 200
//
//#define PIN_INT0  2
//#define PIN_INT1  3
//#define SIZE 5
//
//xQueueHandle q;
//SemaphoreHandle_t xSemaphore = NULL;
//int lastDebounceTime = 0;
//
//
//
//// Flashes LED at pin 7: 5 times a 4 Hz
//void int0task(void *p) {
//	while (1) {
//		if (xSemaphoreTake(bouncer, 500)) {
//			int reading;
//			reading = analogRead(0);
//			xQueueSendToBack(queue, &reading, 500);
//		}
//	}
//}
//
//// Flashes LED at pin 6: 5 times at 2HZ
//void int1task(void *p) {
//	TickType_t xLastWakeTime = xTaskGetTickCount();
//	while (1) {
//		int reading;
//		if (xQueueReceive(queue, &reading, 500)){
//			Serial.println(reading);
//		}
//		//Serial.print("\n");
//		vTaskDelayUntil(&xLastWakeTime, 5000);
//	}
//}
//
//
//
//void producer1(void *p) {
////	Initialize <counter> to 0;
//	int counter = 0;
//	while (1) {
//		counter = counter + 2;
//		if(xQueueSendToBack(q, (void *)&counter, 0) != pdPASS ) {
//			if (xSemaphoreTake(xSemaphore, (TickType_t)10)==pdTRUE) {
//				Serial.println("Message queue full");
//				xSemaphoreGive(xSemaphore);
//			}
//		}
////		increment <counter> by 2; // send only even values
////		send <counter> to message queue;
////		If (error in sending)
////		print ¡°Message queue full¡± on the serial port;
//
//	}
//}
//
//void producer2(void *p) {
//	int counter = 1;
//	while (1) {
//		counter = counter + 2;
//		if(xQueueSendToBack(q, (void *)&counter, 0) != pdPASS ) {
//			if (xSemaphoreTake(xSemaphore, (TickType_t)10)==pdTRUE) {
//				Serial.println("Message queue full");
//				xSemaphoreGive(xSemaphore);
//			}
//		}
//
////		increment <counter> by 2; // send only odd values
////		send <counter> to message queue;
////		If (error in sending)
////		print ¡°Message queue full¡± on the serial port;
//	}
//}
//
//void consumer(void *p) {
//	while (1) {
//		int counter;
//		if(xQueueReceive(q, &counter, portMAX_DELAY) == pdTRUE){
//			if (xSemaphoreTake(xSemaphore, (TickType_t)10)==pdTRUE) {
//				Serial.print("receiving = ");
//				Serial.println(counter);
//				xSemaphoreGive(xSemaphore);
//			}
//		}
//
//		int num = uxQueueMessagesWaiting(q);
//
//		if (xSemaphoreTake(xSemaphore, (TickType_t)10)==pdTRUE) {
//			Serial.print("number of items in queue = ");
//			Serial.println(num);
//			xSemaphoreGive(xSemaphore);
//		}
//
////		receive <counter> from message queue;
////		print ¡°receiving = <counter>¡± on the serial port;
////		print ¡°number of items in queue = ..¡± on the serial port;
//	}
//}
//
//void int0ISR() {
//	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//	int timer = millis();
//	if (timer - lastDebounceTime > DELAY) {
//		xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
//	}
//	lastDebounceTime = timer;
//	if (xHigherPriorityTaskWoken) {
//		taskYIELD();
//	}
//}
//
//void setup() {
//	Serial.begin(115200);
//	pinMode(PIN_INT0, INPUT);
//	pinMode(PIN_INT1, INPUT);
//
//	attachInterrupt(0, int0ISR, RISING);
//
//	q = xQueueCreate(5, sizeof(unsigned long));
//
//	vSemaphoreCreateBinary(xSemaphore);
//	xSemaphoreTake(xSemaphore, 500);
//}
//
//
//void loop() {
//	xSemaphore = xSemaphoreCreateMutex();
//	//same priority
//	xTaskCreate(producer1, "producer1", STACK_SIZE, (void * ) 1, 1, NULL);
//	xTaskCreate(producer2, "producer2", STACK_SIZE, (void * ) 2, 1, NULL);
//	xTaskCreate(consumer, "consumer", STACK_SIZE, (void * ) 3, 2, NULL);
////	xTaskCreate(serialPrint, "SerialPrint", STACK_SIZE, (void * ) 4, 3, NULL);
//	/* start scheduler */
//	vTaskStartScheduler();
//}



//////////////// Q5(A) //////////////////////////////////////
//#include <Arduino.h>
//#include <avr/io.h>
//#include <FreeRTOS.h>
//#include <task.h>
//#include <queue.h>
//#include <semphr.h>
//
//#define STACK_SIZE 200
//#define PIN_INT0  2
//#define PIN_INT1  3
//#define LED_INT0  7
//#define LED_INT1  6
//#define DELAY 200
//#define SIZE 5
//
//int lastDebounceTime = 0;
//xSemaphoreHandle semaphore, mutex, empty , full;
//
//int buf[SIZE];
//int in = 0, out = 0;
//
//void int0task(void *p) {
//	while (1) {
//		if (xSemaphoreTake(semaphore, 500)) {
//			if (xSemaphoreTake(empty, 500)) {
//				if (xSemaphoreTake(mutex, 500)) {
//					if ((in + 1 % SIZE == out)) ;
//					else {
//						buf[in] = analogRead(0);
//						in = (in + 1) % SIZE;
//					}
//					Serial.print("int0task\n");
//					xSemaphoreGive(mutex);
//				}
//				xSemaphoreGive(full);
//			}
//		}
//	}
//}
//
//void int1task(void *p) {
//	TickType_t xLastWakeTime = xTaskGetTickCount();
//	while (1) {
//		if (xSemaphoreTake(full, 500)) {
//			if (xSemaphoreTake(mutex, 500)) {
//				int value;
//				if (in == out) ;
//				else {
//					value = buf[out];
//					out = (out + 1) % SIZE;
//					Serial.print(value); Serial.print("\n");
//				}
//				xSemaphoreGive(mutex);
//			}
//			xSemaphoreGive(empty);
//		}
//		vTaskDelayUntil(&xLastWakeTime, 5000);
//	}
//}
//
//void int0ISR() {
//	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//	int temp = millis();
//	if (temp - lastDebounceTime > DELAY) {
//		xSemaphoreGiveFromISR(semaphore, &xHigherPriorityTaskWoken);
//	}
//	lastDebounceTime = temp;
//	if (xHigherPriorityTaskWoken) {
//		taskYIELD();
//	}
//}
//
//void setup() {
//	Serial.begin(115200);
//
//	pinMode(LED_INT0, OUTPUT);
//	pinMode(LED_INT1, OUTPUT);
//	pinMode(PIN_INT0, INPUT);
//	pinMode(PIN_INT1, INPUT);
//
//	attachInterrupt(0, int0ISR, RISING);
//
//	vSemaphoreCreateBinary(semaphore);
//	empty = xSemaphoreCreateCounting(SIZE-1, SIZE-1);
//	full = xSemaphoreCreateCounting(SIZE-1, 0);
//	mutex = xSemaphoreCreateMutex();
//	xSemaphoreTake(semaphore, 500);
//
//}
//void loop() {
//	xTaskCreate(int0task, "int0task", STACK_SIZE, NULL, 1, NULL);
//	xTaskCreate(int1task, "int1task", STACK_SIZE, NULL, 1, NULL);
//	/* start scheduler */
//	vTaskStartScheduler();
//}


///////////////// Q4(C) //////////////////////////////////////


//#include <Arduino.h>
//#include <avr/io.h>
//#include <FreeRTOS.h>
//#include <task.h>
//#include <semphr.h>
//
//#define STACK_SIZE 200
//
//#define PIN_INT0 2
//#define PIN_INT1 3
//
//#define LED_PIN0 6
//#define LED_PIN1 7
//
//#define DEBOUNCE_DELAY 200
//
//int checkDe0 = 0, checkDe1 = 0, button_0 = 0, last_button_0 = 0, button_1 = 0, last_button_1 = 0;
//
//SemaphoreHandle_t xSemaphore0 = NULL, xSemaphore1 = NULL;
//BaseType_t xHigherPriorityTaskWoken;
//
//void int0task(void *p) {
//	xSemaphore0 = xSemaphoreCreateBinary();
//	while(1) {
//		if(xSemaphoreTake(xSemaphore0, 10)) {
//			for(int i=0; i<5; i++) {
//				digitalWrite(LED_PIN0, HIGH);
//				delay(125);
//				digitalWrite(LED_PIN0, LOW);
//				delay(125);
//			}
//		}
//	}
//}
//
//void int1task(void *p) {
//	xSemaphore1 = xSemaphoreCreateBinary();
//	while(1) {
//		if(xSemaphoreTake(xSemaphore1, 10)) {
//			for(int i=0; i<5; i++) {
//				digitalWrite(LED_PIN1, HIGH);
//				delay(250);
//				digitalWrite(LED_PIN1, LOW);
//				delay(250);
//			}
//		}
//	}
//}
//
//void int0ISR() {
//	button_0 = millis();
//	if (button_0 - last_button_0 > DEBOUNCE_DELAY) {
//		checkDe0=1;
//	}
//	last_button_0 = button_0;
//	if (checkDe0 == 1) {
//		checkDe0 =0;
//		xSemaphoreGiveFromISR(xSemaphore0, &xHigherPriorityTaskWoken);
//	}
//	taskYIELD();
//}
//
//void int1ISR() {
//	button_1 = millis();
//	if (button_1 - last_button_1 > DEBOUNCE_DELAY) {
//		checkDe1=1;
//	}
//	last_button_1 = button_1;
//	if (checkDe1 == 1) {
//		checkDe1 =0;
//		xSemaphoreGiveFromISR(xSemaphore1, &xHigherPriorityTaskWoken);
//	}
//	taskYIELD();
//}
//
//
//
//void setup() {
//	Serial.begin(115200);
//
//	pinMode(LED_PIN0, OUTPUT);
//	pinMode(LED_PIN1, OUTPUT);
//
//	attachInterrupt(0, int0ISR, RISING);
//	attachInterrupt(1, int1ISR, RISING);
//}
//void loop() {
//	xHigherPriorityTaskWoken = pdFALSE;
//	xTaskCreate(int0task, "int0task", STACK_SIZE, NULL, 2, NULL);
//	xTaskCreate(int1task, "int1task", STACK_SIZE, NULL, 1, NULL);
//	vTaskStartScheduler();
//}


//////////////////// Q3(A) ////////////////////////////////////

//#include <Arduino.h>
//#include <avr/io.h>
//#include <FreeRTOS.h>
//#include <task.h>
//#include <semphr.h>
//#define STACK_SIZE 200
//
//xQueueHandle q;
//
//void serialPrint(void *p){
//	while(1){
//		int taskNum;
//		if(xQueueReceive(q, &taskNum, 10) == pdTRUE){
//			Serial.print("Task ");
//			Serial.println(taskNum);
//		}
//	}
//}
//
//
//void task1and2(void *p) {
//	while (1) {
//		int taskNum = (int) p;
//		xQueueSendToBack(q, (void *)&taskNum, 10);
//		vTaskDelay(1);
//	}
//}
//
//void setup() {
//	Serial.begin(115200);
//	q = xQueueCreate(10, sizeof( unsigned long));
//}
//
//void loop() {
//	/* create two tasks one with higher priority than the other */
//	xTaskCreate(task1and2, "Task1", STACK_SIZE, (void * ) 1, 1, NULL);
//	xTaskCreate(task1and2, "Task2", STACK_SIZE, (void * ) 2, 2, NULL);
//	xTaskCreate(serialPrint, "SerialPrint", STACK_SIZE, (void * ) 3, 3, NULL);
//	/* start scheduler */
//	vTaskStartScheduler();
//}



//////////////////// Q2 ///////////////////////////////////////
//#include <Arduino.h>
//#include <avr/io.h>
//#include <FreeRTOS.h>
//#include <task.h>
//#include <semphr.h>
//#define STACK_SIZE 200
//
//SemaphoreHandle_t xSemaphore = NULL;
//
//void task1and2(void *p) {
//	while (1) {
//		int taskNum = (int) p;
//		if (xSemaphoreTake(xSemaphore, (TickType_t)10)==pdTRUE) {
//			Serial.print("Task ");
//			Serial.println(taskNum);
//			xSemaphoreGive(xSemaphore);
//		}
//		vTaskDelay(1);
//	}
//}
//
//void setup() {
//	Serial.begin(115200);
//}
//
//void loop() {
//	xSemaphore = xSemaphoreCreateMutex();
//	/* create two tasks one with higher priority than the other */
//	xTaskCreate(task1and2, "Task1", STACK_SIZE, (void * ) 1, 1, NULL);
//	xTaskCreate(task1and2, "Task2", STACK_SIZE, (void * ) 2, 2, NULL);
//	/* start scheduler */
//	vTaskStartScheduler();
//}


//////////////////// Q1/////////////////////////////////////////
//#include <Arduino.h>
//#include <avr/io.h>
//#include <FreeRTOS.h>
//#include <task.h>
//#define STACK_SIZE 200
//void task1and2(void *p) {
//	while (1) {
//		int taskNum = (int) p;
//		Serial.print("Task ");
//		Serial.println(taskNum);
//		vTaskDelay(1);
//	}
//}
//void setup() {
//Serial.begin(115200);
//}
//void loop() {
//	/* create two tasks one with higher priority than the other */
//	xTaskCreate(task1and2, "Task1", STACK_SIZE, (void * ) 1, 1, NULL);
//	xTaskCreate(task1and2, "Task2", STACK_SIZE, (void * ) 2, 2, NULL);
//	/* start scheduler */
//	vTaskStartScheduler();
//}


/////////////////////// Lab 4//////////////////////////////////////

/////////////////////Q5 (B)/////////////////////////////////////////
//#include <Arduino.h>
//#include <avr/io.h>
//#include <FreeRTOS.h>
//#include <task.h>
//#define STACK_SIZE 200
//#define LED_PIN6 6
//#define LED_PIN7 7
//#define LED_PIN8 8
//#define LED_PIN9 9
//#define TASK_WCET_T1 1500
//#define TASK_WCET_T2 1700
//#define TASK_WCET_T3 2400
//#define TASK_WCET_T4 800
//
//void myDelay(int ms) {
//	for (int i = 0; i < ms; i++) {
//		delayMicroseconds(1000);
//	}
//}
//
//void task_instance(int pin, int WCET) {
//	int count = 0;
//	while (count < WCET) {
//		digitalWrite(pin, HIGH);
//		myDelay(50);
//		digitalWrite(pin, LOW);
//		myDelay(50);
//		count +=100;
//	}
//}
//
//void task1(void *p)
//{
//	task_instance(LED_PIN6, TASK_WCET_T1);
//}
//
//void task2(void *p)
//{
//	task_instance(LED_PIN7, TASK_WCET_T2);
//
//}
//
//void task3(void *p)
//{
//	task_instance(LED_PIN8, TASK_WCET_T3);
//
//}
//
//void task4(void *p)
//{
//	task_instance(LED_PIN9, TASK_WCET_T4);
//}
//
//void taskTotal(void *p)
//{
//	TickType_t xLastWakeTime;
//	const TickType_t xFrequency = 5000;
//	xLastWakeTime = xTaskGetTickCount();
//
//	while(1){
//		task_instance(LED_PIN6, TASK_WCET_T1);
//		task_instance(LED_PIN7, TASK_WCET_T2);
//		task_instance(LED_PIN9, TASK_WCET_T4);
//		vTaskDelayUntil( &xLastWakeTime, xFrequency );
//
//		task_instance(LED_PIN6, TASK_WCET_T1);
//		task_instance(LED_PIN8, TASK_WCET_T3);
//		vTaskDelayUntil( &xLastWakeTime, xFrequency );
//
//		task_instance(LED_PIN6, TASK_WCET_T1);
//		task_instance(LED_PIN7, TASK_WCET_T2);
//		vTaskDelayUntil( &xLastWakeTime, xFrequency );
//
//		task_instance(LED_PIN6, TASK_WCET_T1);
//		task_instance(LED_PIN8, TASK_WCET_T3);
//		vTaskDelayUntil( &xLastWakeTime, xFrequency );
//	}
//}
//
//void setup()
//{
//	pinMode(LED_PIN6, OUTPUT);
//	pinMode(LED_PIN7, OUTPUT);
//	pinMode(LED_PIN8, OUTPUT);
//	pinMode(LED_PIN9, OUTPUT);
//}
//void loop() {
//	xTaskCreate(taskTotal, // Pointer to the task entry function
//				"TaskTotal", // Task name
//				STACK_SIZE, // Stack size
//				NULL, // Pointer to the parameters
//				4, // Task priority
//				NULL); // A handle by which the created task can be referenced.
//	vTaskStartScheduler();
//}
//
////////////////////////Q4 (D)//////////////////////////////////////
//#include <Arduino.h>
//#include <avr/io.h>
//#include <FreeRTOS.h>
//#include <task.h>
//#define STACK_SIZE 200
//#define LED_PIN6 6
//#define LED_PIN7 7
//#define LED_PIN8 8
//#define LED_PIN9 9
//#define TASK_WCET_T1 1500
//#define TASK_WCET_T2 1500
//#define TASK_WCET_T3 2400
//#define TASK_WCET_T4 800
//
//void myDelay(int ms) {
//	for (int i = 0; i < ms; i++) {
//		delayMicroseconds(1000);
//	}
//}
//
//void task_instance(int pin, int WCET) {
//	int count = 0;
//	while (count < WCET) {
//		digitalWrite(pin, HIGH);
//		myDelay(50);
//		digitalWrite(pin, LOW);
//		myDelay(50);
//		count +=100;
//	}
//}
//
//void task1(void *p)
//{
//	TickType_t xLastWakeTime;
//	const TickType_t xFrequency = 5000;
//	xLastWakeTime = xTaskGetTickCount();
//
//	for( ;; ){
//		task_instance(LED_PIN6, TASK_WCET_T1);
//		vTaskDelayUntil( &xLastWakeTime, xFrequency );
//	}
//}
//
//void task2(void *p)
//{
//	TickType_t xLastWakeTime;
//	const TickType_t xFrequency = 10000;
//	xLastWakeTime = xTaskGetTickCount();
//
//	for( ;; ){
//		task_instance(LED_PIN7, TASK_WCET_T2);
//		vTaskDelayUntil( &xLastWakeTime, xFrequency );
//	}
//}
//
//void task3(void *p)
//{
//	TickType_t xLastWakeTime;
//	const TickType_t xFrequency = 10000;
//	xLastWakeTime = xTaskGetTickCount();
//
//	for( ;; ){
//		task_instance(LED_PIN8, TASK_WCET_T3);
//		vTaskDelayUntil( &xLastWakeTime, xFrequency );
//	}
//}
//
//void task4(void *p)
//{
//	TickType_t xLastWakeTime;
//	const TickType_t xFrequency = 20000;
//	xLastWakeTime = xTaskGetTickCount();
//
//	for( ;; ){
//		task_instance(LED_PIN9, TASK_WCET_T4);
//		vTaskDelayUntil( &xLastWakeTime, xFrequency );
//	}
//}
//
//void setup()
//{
//	pinMode(LED_PIN6, OUTPUT);
//	pinMode(LED_PIN7, OUTPUT);
//	pinMode(LED_PIN8, OUTPUT);
//	pinMode(LED_PIN9, OUTPUT);
//}
//void loop() {
//	xTaskCreate(task1, // Pointer to the task entry function
//				"Task1", // Task name
//				STACK_SIZE, // Stack size
//				NULL, // Pointer to the parameters
//				4, // Task priority
//				NULL); // A handle by which the created task can be referenced.
//	xTaskCreate(task2, // Pointer to the task entry function
//				"Task2", // Task name
//				STACK_SIZE, // Stack size
//				NULL, // Pointer to the parameters
//				3, // Task priority
//				NULL); // A handle by which the created task can be referenced.
//	xTaskCreate(task3, // Pointer to the task entry function
//				"Task3", // Task name
//				STACK_SIZE, // Stack size
//				NULL, // Pointer to the parameters
//				3, // Task priority
//				NULL); // A handle by which the created task can be referenced.
//	xTaskCreate(task4, // Pointer to the task entry function
//				"Task4", // Task name
//				STACK_SIZE, // Stack size
//				NULL, // Pointer to the parameters
//				1, // Task priority
//				NULL); // A handle by which the created task can be referenced.
//	vTaskStartScheduler();
//}
////////////////////Q4(B)/////////////////////////////////////////
//#include <Arduino.h>
//#include <avr/io.h>
//#include <FreeRTOS.h>
//#include <task.h>
//#define STACK_SIZE 200
//#define LED_PIN6 6
//#define LED_PIN7 7
//#define LED_PIN8 8
//#define LED_PIN9 9
//#define TASK_WCET_T1 1500
//#define TASK_WCET_T2 1700
//#define TASK_WCET_T3 2400
//#define TASK_WCET_T4 800
//
//void myDelay(int ms) {
//	for (int i = 0; i < ms; i++) {
//		delayMicroseconds(1000);
//	}
//}
//
//void task_instance(int pin, int WCET) {
//	int count = 0;
//	while (count < WCET) {
//		digitalWrite(pin, HIGH);
//		myDelay(50);
//		digitalWrite(pin, LOW);
//		myDelay(50);
//		count +=100;
//	}
//}
//
//void task1(void *p)
//{
//	TickType_t xLastWakeTime;
//	const TickType_t xFrequency = 5000;
//	xLastWakeTime = xTaskGetTickCount();
//
//	for( ;; ){
//		task_instance(LED_PIN6, TASK_WCET_T1);
//		vTaskDelayUntil( &xLastWakeTime, xFrequency );
//	}
//}
//
//void task2(void *p)
//{
//	TickType_t xLastWakeTime;
//	const TickType_t xFrequency = 10000;
//	xLastWakeTime = xTaskGetTickCount();
//
//	for( ;; ){
//		task_instance(LED_PIN7, TASK_WCET_T2);
//		vTaskDelayUntil( &xLastWakeTime, xFrequency );
//	}
//}
//
//void task3(void *p)
//{
//	TickType_t xLastWakeTime;
//	const TickType_t xFrequency = 10000;
//	xLastWakeTime = xTaskGetTickCount();
//
//	for( ;; ){
//		task_instance(LED_PIN8, TASK_WCET_T3);
//		vTaskDelayUntil( &xLastWakeTime, xFrequency );
//	}
//}
//
//void task4(void *p)
//{
//	TickType_t xLastWakeTime;
//	const TickType_t xFrequency = 20000;
//	xLastWakeTime = xTaskGetTickCount();
//
//	for( ;; ){
//		task_instance(LED_PIN9, TASK_WCET_T4);
//		vTaskDelayUntil( &xLastWakeTime, xFrequency );
//	}
//}
//
//void setup()
//{
//	pinMode(LED_PIN6, OUTPUT);
//	pinMode(LED_PIN7, OUTPUT);
//	pinMode(LED_PIN8, OUTPUT);
//	pinMode(LED_PIN9, OUTPUT);
//}
//void loop() {
//	xTaskCreate(task1, // Pointer to the task entry function
//				"Task1", // Task name
//				STACK_SIZE, // Stack size
//				NULL, // Pointer to the parameters
//				4, // Task priority
//				NULL); // A handle by which the created task can be referenced.
//	xTaskCreate(task2, // Pointer to the task entry function
//				"Task2", // Task name
//				STACK_SIZE, // Stack size
//				NULL, // Pointer to the parameters
//				3, // Task priority
//				NULL); // A handle by which the created task can be referenced.
//	xTaskCreate(task3, // Pointer to the task entry function
//				"Task3", // Task name
//				STACK_SIZE, // Stack size
//				NULL, // Pointer to the parameters
//				2, // Task priority
//				NULL); // A handle by which the created task can be referenced.
//	xTaskCreate(task4, // Pointer to the task entry function
//				"Task4", // Task name
//				STACK_SIZE, // Stack size
//				NULL, // Pointer to the parameters
//				1, // Task priority
//				NULL); // A handle by which the created task can be referenced.
//	vTaskStartScheduler();
//}
////////////////////Q3/////////////////////////////////////////
//#include <Arduino.h>
//#include <avr/io.h>
//#include <FreeRTOS.h>
//#include <task.h>
//#define STACK_SIZE 200
//#define LED_PIN6 6
//
//void task1(void *p)
//{
//	for (;;) {
//		for(int i = 0; i <= 20; i++) {
//			digitalWrite(LED_PIN6, HIGH);
//			delay(i);
//			digitalWrite(LED_PIN6, LOW);
//			delay(20-i);
//		}
//	}
//}
//
//void setup()
//{
//	pinMode(LED_PIN6, OUTPUT);
//}
//void loop() {
//	xTaskCreate(task1, // Pointer to the task entry function
//				"Task1", // Task name
//				STACK_SIZE, // Stack size
//				NULL, // Pointer to the parameters
//				1, // Task priority
//				NULL); // A handle by which the created task can be referenced.
//	vTaskStartScheduler();
//}

////////////////////Q2(F)/////////////////////////////////////////

//#include <Arduino.h>
//#include <avr/io.h>
//#include <FreeRTOS.h>
//#include <task.h>
//#define STACK_SIZE 200
//#define LED_PIN6 6
//#define LED_PIN7 7
//#define LED_PIN8 8
//
//void task1(void *p)
//{
//	TickType_t xLastWakeTime_PIN6;
//	const TickType_t xFrequency_PIN6 = 625;
//	for( ;; ){
//		digitalWrite(LED_PIN6, HIGH);
//		xLastWakeTime_PIN6 = xTaskGetTickCount ();
//		vTaskDelayUntil( &xLastWakeTime_PIN6, xFrequency_PIN6 );
//		digitalWrite(LED_PIN6, LOW);
//		xLastWakeTime_PIN6 = xTaskGetTickCount ();
//		vTaskDelayUntil( &xLastWakeTime_PIN6, xFrequency_PIN6 );
//	}
//
//}
//
//void task2(void *p)
//{
//	TickType_t xLastWakeTime_PIN7;
//	const TickType_t xFrequency_PIN7 = 1250;
//	for( ;; ){
//		digitalWrite(LED_PIN7, HIGH);
//		xLastWakeTime_PIN7 = xTaskGetTickCount ();
//		vTaskDelayUntil( &xLastWakeTime_PIN7, xFrequency_PIN7 );
//		digitalWrite(LED_PIN7, LOW);
//		xLastWakeTime_PIN7 = xTaskGetTickCount ();
//		vTaskDelayUntil( &xLastWakeTime_PIN7, xFrequency_PIN7 );
//	}
//
//}
//
//void task3(void *p)
//{
//	TickType_t xLastWakeTime_PIN8;
//	const TickType_t xFrequency_PIN8 = 2500;
//	for( ;; ){
//		digitalWrite(LED_PIN8, HIGH);
//		xLastWakeTime_PIN8 = xTaskGetTickCount ();
//		vTaskDelayUntil( &xLastWakeTime_PIN8, xFrequency_PIN8 );
//		digitalWrite(LED_PIN8, LOW);
//		xLastWakeTime_PIN8 = xTaskGetTickCount ();
//		vTaskDelayUntil( &xLastWakeTime_PIN8, xFrequency_PIN8 );
//	}
//
//}
//
//void setup()
//{
//	pinMode(LED_PIN6, OUTPUT);
//	pinMode(LED_PIN7, OUTPUT);
//	pinMode(LED_PIN8, OUTPUT);
//}
//void loop() {
//	xTaskCreate(task1, // Pointer to the task entry function
//				"Task1", // Task name
//				STACK_SIZE, // Stack size
//				NULL, // Pointer to the parameters
//				3, // Task priority
//				NULL); // A handle by which the created task can be referenced.
//	xTaskCreate(task2, // Pointer to the task entry function
//				"Task2", // Task name
//				STACK_SIZE, // Stack size
//				NULL, // Pointer to the parameters
//				2, // Task priority
//				NULL); // A handle by which the created task can be referenced.
//	xTaskCreate(task3, // Pointer to the task entry function
//				"Task3", // Task name
//				STACK_SIZE, // Stack size
//				NULL, // Pointer to the parameters
//				1, // Task priority
//				NULL); // A handle by which the created task can be referenced.
//	vTaskStartScheduler();
//}


////////////////////Q2(E)/////////////////////////////////////////
/*
#include <Arduino.h>
#include <avr/io.h>
#include <FreeRTOS.h>
#include <task.h>
#define STACK_SIZE 200
#define LED_PIN6 6

void task1(void *p)
{
	TickType_t xLastWakeTime_PIN6;
	const TickType_t xFrequency_PIN6 = 1000;
//	for (;;) {
//		digitalWrite(LED_PIN6, HIGH);
//		delay(1000);
//		digitalWrite(LED_PIN6, LOW);
//		delay(1000);
//	}
	for( ;; ){
		digitalWrite(LED_PIN6, HIGH);
		xLastWakeTime_PIN6 = xTaskGetTickCount ();
		vTaskDelayUntil( &xLastWakeTime_PIN6, xFrequency_PIN6 );
		digitalWrite(LED_PIN6, LOW);
		xLastWakeTime_PIN6 = xTaskGetTickCount ();
		vTaskDelayUntil( &xLastWakeTime_PIN6, xFrequency_PIN6 );
	}

}

void setup()
{
	pinMode(LED_PIN6, OUTPUT);
}
void loop() {
	xTaskCreate(task1, // Pointer to the task entry function
				"Task1", // Task name
				STACK_SIZE, // Stack size
				NULL, // Pointer to the parameters
				1, // Task priority
				NULL); // A handle by which the created task can be referenced.
	vTaskStartScheduler();
}
*/

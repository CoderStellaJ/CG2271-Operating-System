#include <Arduino.h>
#include "prioq.h"
#define PIN_PB1 2
#define PIN_PB2 3
#define PIN_LED1 6
#define PIN_LED2 7
#define DEBOUNCE_DELAY 70

int lastDebounce1 = 0;
int lastDebounce2 = 0;


// Declares a new type called "funcptr"

typedef void (*funcptr)(void);
TPrioQueue *queue;

// Flashes LED at pin 6: 7 times at 5Hz
void int0task()
{
    int i = 0;
    while (i < 7) {
        analogWrite(PIN_LED1, 255);
        delay(100);
        analogWrite(PIN_LED1, 0);
        delay(100);
        i ++;
    }
}

// Flashes LED at pin 7: 5 times at 1HZ
void int1task()
{
    int i = 0;
    while (i < 5) {
    	analogWrite(PIN_LED2, 255);
        delay(500);
        analogWrite(PIN_LED2, 0);
        delay(500);
        i ++;
    }
}

void int0ISR()
{
	int button = digitalRead(PIN_PB1);
	if (millis() - lastDebounce1 > DEBOUNCE_DELAY) {
		if (button == HIGH) {
			lastDebounce1 = millis();
			enq(queue, (void *)(&int0task), 1);	//low priority
		}
	}


}

void int1ISR()
{
	int button = digitalRead(PIN_PB2);
	if (millis() - lastDebounce2 > DEBOUNCE_DELAY) {
		if (button == HIGH) {
			lastDebounce2 = millis();
			enq(queue, (void *)(&int1task), 0);	//high priority
		}
	}
}

void setup()
{
	queue=makeQueue();
	attachInterrupt(0, int0ISR, RISING);
	attachInterrupt(1, int1ISR, RISING);
}

// Dequeues and calls functions if the queue is not empty
void loop()
{
	funcptr exeFunc;

	if (qlen(queue)>0)
	{
		// Dequeue it
		exeFunc = (funcptr)deq(queue);
		(*exeFunc)();
	}

}

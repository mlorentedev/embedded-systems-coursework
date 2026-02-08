//============================================================================
// Name        : P2_threads.cpp
// Author      : Manuel Lorente
// Version     : 0.1
// Copyright   : manloralm@outlook.com
// Description : Analog output using analog input in C++, Ansi-style
//============================================================================

#include <pthread.h>
#include <wiringPi.h>
#include <semaphore.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <mcp3004.h>
#define BASE 		100
#define SPI_CHAN 	0

#define LED			0
#define LED_PWM0 	1
#define LED_PWM1 	23

sem_t * sem;
pthread_t idHilo;
pthread_t idHilo2;
pthread_t idHilo3;

void *ledExterno(void * t){
	while(1)
	{
		digitalWrite(LED , HIGH); // turn the LED on (HIGH is the voltage level)
		usleep(1000000); // wait for a second
		digitalWrite(LED , LOW); // turn the LED off by making the voltage LOW
		usleep(1000000);
	}
}

void *ledPWM0(void * t){
	for (;;)
	{
		int x= analogRead (BASE + 2); //el potenciómetro está en el canal 2
		printf("%i\n",x);
		pwmWrite (LED_PWM0, x) ;
		delay (1) ;
	}
}

void *ledPWM1(void * t){
	int bright;
	for (;;)
	{
		for (bright = 0 ; bright < 1024 ; ++bright)
		{
			pwmWrite (LED_PWM1, bright) ;
			delay (1) ;
		}
	for (bright = 1023 ; bright >= 0 ; --bright)
		{
		pwmWrite (LED_PWM1, bright) ;
		delay (1) ;
		}
	}
}

void setup(){
	pinMode(LED , OUTPUT);
	pinMode (LED_PWM0, PWM_OUTPUT) ;
	pinMode (LED_PWM1, PWM_OUTPUT) ;
	mcp3004Setup (BASE, SPI_CHAN); // 3004 and 3008 are the same 4/8 channels

	pthread_create (&idHilo, NULL, ledExterno, NULL);
	pthread_create (&idHilo2, NULL, ledPWM0, NULL);
	pthread_create (&idHilo3, NULL, ledPWM1, NULL);
}

int main (void)
{
	if (wiringPiSetup () == -1)
		exit (1) ;
	setup();

	while(1) pause();
	return 0 ;
}

//============================================================================
// Name        : Blink_POSIX.cpp
// Author      : Manuel Lorente
// Version     :
// Copyright   : manloralm@outlook.com
// Description : Blink project with POSIX in C++, Ansi-style
//============================================================================
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <wiringPi.h>
#include <semaphore.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
// LED Pin - wiringPi pin 22 is Pin 31.

#define LED 0
sem_t * sem;
pthread_t idHilo;
pthread_t idHilo2;

void *test(void * t){
	while(1)
	{
		sem_wait(sem);
		digitalWrite(LED , HIGH); // turn the LED on (HIGH is the voltage level)
		usleep(100000); // wait for a second
		digitalWrite(LED , LOW); // turn the LED off by making the voltage LOW
		usleep(100000);
		sem_post(sem);
	}
}

void setup() {
	pinMode(LED , OUTPUT);
	sem = sem_open("test", O_CREAT, 0666, 1); /* Initial value is 1. */
	if (sem == SEM_FAILED) {
		perror("Sem fail");
		exit(1);
	}
	sem_unlink("test"); // borra el semaforo cuando termine el programa
	// put your setup code here, to run once:
	pthread_create (&idHilo, NULL, test, NULL);
}

int main() {
	wiringPiSetup();
	setup();
	// put your main code here, to run repeatedly:
	while (1) pause(); // stop loop

	return 0;
}

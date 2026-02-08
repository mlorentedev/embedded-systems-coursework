//============================================================================
// Name        : BlinkSwitch_POSIX.cpp
// Author      : Manuel Lorente
// Version     :
// Copyright   : manloralm@outlook.com
// Description : Hello World in C++, Ansi-style
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

#define LED 22
#define SWITCH 21

sem_t * sem;
pthread_t idHilo;

void *led(void * t){
	while(1)
	{
		sem_wait(sem);
		int switchState = digitalRead(SWITCH); //
		if (switchState == HIGH) { //
			digitalWrite (LED , HIGH);
		}
		else { //
			digitalWrite (LED , LOW);
		}
		sem_post(sem);
	}
}

void setup() {
	pinMode(LED , OUTPUT);
	pinMode (SWITCH, INPUT) ;

	sem = sem_open("test", O_CREAT, 0666, 1); /* Initial value is 1. */
	if (sem == SEM_FAILED) {
		perror("Sem fail");
		exit(1);
	}
	sem_unlink("test"); // borra el semaforo cuando termine el programa
	// put your setup code here, to run once:
	pthread_create (&idHilo, NULL, led, NULL);

}

int main() {
	wiringPiSetup();
	setup();
	// put your main code here, to run repeatedly:
	while (1) pause(); // stop loop

	return 0;
}

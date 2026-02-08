//============================================================================
// Name        : P3_timers.cpp
// Author      : Manuel Lorente
// Version     : 0.1
// Copyright   : manloralm@outlook.com
// Description : P3 timers, Ansi-style
//============================================================================

#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>
#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <string.h>

#define LED 2
#define PERIOD 1

using namespace std;

int estado = 0;
timer_t timerid;

void manejador(int sig) {
	if (estado)
		digitalWrite (LED, 0) ; // On
	else
		digitalWrite (LED, 1) ; // On
	estado = !estado;
}

void setup() {
	struct sigevent sev;
	struct itimerspec its;

	pinMode(LED, OUTPUT);

	signal(SIGRTMIN+1, manejador); // preparar el tratamiento de la señal, la señal a tratar va a ser SIGRTMIN

	// preparar el timer
	sev.sigev_notify = SIGEV_SIGNAL; // notificacion del timer por senal
	sev.sigev_signo = SIGRTMIN+1; // señal generada por el timer al vencer
	if (timer_create(CLOCK_REALTIME, &sev, &timerid) == -1) {
		perror("timer create");
		exit(1);
	}
	/* Start the timer */

	its.it_value.tv_sec = PERIOD;
	its.it_value.tv_nsec = 0;
	its.it_interval.tv_sec = its.it_value.tv_sec;
	its.it_interval.tv_nsec = its.it_value.tv_nsec;
	if (timer_settime(timerid, 0, &its, NULL) == -1) {
		perror(" timer_settime");
		exit(1);
	}
}

int main() {
	wiringPiSetup();
	setup();
	while (1)
		pause();
	return 0;
}

//============================================================================
// Name        : Led_PWM.cpp
// Author      : Manuel Lorente
// Version     :
// Copyright   : manloralm@outlook.com
// Description : Analog output using analog input in C++, Ansi-style
//============================================================================

#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <mcp3004.h>
#define BASE 100
#define SPI_CHAN 0

int main (void)
{
	printf ("Raspberry Pi wiringPi PWM test program\n") ;
	if (wiringPiSetup () == -1)
		exit (1) ;
	mcp3004Setup (BASE, SPI_CHAN); // 3004 and 3008 are the same 4/8 channels
	pinMode (1, PWM_OUTPUT) ;
	for (;;)
	{
		int bright = analogRead (BASE + 2); //el potenciómetro está en el canal 2
		printf("%i\n",bright);
		pwmWrite (1, bright) ;
		delay (1) ;
	}
	return 0 ;
}

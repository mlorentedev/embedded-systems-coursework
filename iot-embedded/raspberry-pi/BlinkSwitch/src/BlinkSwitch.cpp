//============================================================================
// Name        : BlinkSwitch.cpp
// Author      : Manuel Lorente
// Version     : 0.1
// Copyright   : manloralm@outlook.com
// Description : Led blink project in C++, Ansi-style
//============================================================================

#include <stdio.h>
#include <wiringPi.h>
// LED Pin - wiringPi pin 22 is Pin 31.
// SWITCH Pin - wiringPi pin 21 is Pin 29.
#define LED 22
#define SWITCH 21

int main (void)
{
	printf ("Raspberry Pi - Gertboard Led blinking with switch \n") ;
	wiringPiSetup();
	pinMode (LED, OUTPUT) ;
	pinMode (SWITCH, INPUT) ;
	for (;;)
	{
		int switchState = digitalRead(SWITCH); //
		if (switchState == HIGH) { //
			digitalWrite (LED , HIGH);
		}
		else { //
			digitalWrite (LED , LOW);
		}
	}
return 0 ;
}

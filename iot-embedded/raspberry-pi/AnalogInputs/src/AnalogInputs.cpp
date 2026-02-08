//============================================================================
// Name        : AnalogInputs.cpp
// Author      : Manuel Lorente
// Version     :
// Copyright   : manloralm@outlook.com
// Description : Analog inputs using MCP3008 in C++, Ansi-style
//============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringPi.h>
#include <mcp3004.h>
#define BASE 100
#define SPI_CHAN 0

int main() {
	wiringPiSetup() ;
	mcp3004Setup (BASE, SPI_CHAN); // 3004 and 3008 are the same 4/8 channels
	while(1) {
		int x = analogRead (BASE + 2); //el potenciómetro está en el canal 2
		printf("%i\n",x);
		float volts = ((float)x * 3.3) / 1023.0; // mapeamos el valor
		printf("Voltios = %f\n", volts);
		sleep(1);
	}
	return 0;
}

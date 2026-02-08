//============================================================================
// Name        : main.cpp
// Author      : Manuel Lorente
// Version     : 0.1
// Copyright   : manloralm@outlook.com
// Description : Practica final de la asignatura - Sensor IoT
//============================================================================

#include <iostream>
#include <wiringPi.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <strings.h>
#include <string.h>
#include <stdexcept>
#include <wiringPiI2C.h>
#include <signal.h>
#include <time.h>
#include <arpa/inet.h>
#include <semaphore.h>

// Commands
const unsigned char K_DS1621_START_CONVERT      = 0xEE;
const unsigned char K_DS1621_STOP_CONVERT       = 0x22;
const unsigned char K_DS1621_READ_TEMP          = 0xAA;
const unsigned char K_DS1621_READ_COUNTER       = 0xA8;
const unsigned char K_DS1621_READ_SLOPE         = 0xA9;
const unsigned char K_DS1621_ACCES_CONFIG       = 0xAC;
const unsigned char K_DS1621_ACCES_TH           = 0xA1;
const unsigned char K_DS1621_ACCES_TL           = 0xA2;

// Config
const unsigned char K_DS1621_DONE_CONFIG        = 0x80;
const unsigned char K_DS1621_THF_CONFIG         = 0x40;
const unsigned char K_DS1621_TLF_CONFIG         = 0x20;
const unsigned char K_DS1621_NVB_CONFIG         = 0x10;
const unsigned char K_DS1621_POL_CONFIG         = 0x02;
const unsigned char K_DS1621_1SHOT_CONFIG       = 0x01;

const float K_DS1621_MIN_THRESHOLD_TEMP         = -55.0;
const float K_DS1621_MAX_THRESHOLD_TEMP 		= -125.0;

#define LED 			1						// Led en WiringPi[1]
#define BUTTON 			21						// Pulsador en WiringPi[21]
#define DEV_ID 			0x4F 					// Direccion del sensor DS1621
#define DELAY			10						// Periodo lectura temperatura en segundos
#define TEMP_MIN		25						// Umbral inferior de temperatura para ajuste intensidad del led
#define TEMP_MAX		35						// Umbral superior de temperatura para ajuste intensidad del led
#define PORT_UDP 		8888

using namespace std;
extern float temp;

float temp;										// Variable global de temperatura a compartir entre hilos
int socketServer;
int descriptor = -1;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;	// Mutex para acceso al bus i2c
pthread_t idHilo_temperatura;
pthread_t idHilo_atenderPeticion;
pthread_t idHilo_ledPWM;
pthread_t idHilo_alarmaUDP;

void setupSensor()
{
	descriptor = wiringPiI2CSetup(DEV_ID);

	int result = wiringPiI2CWriteReg8(descriptor,K_DS1621_ACCES_CONFIG,K_DS1621_POL_CONFIG);

	if(0 != result){
		throw std::runtime_error("[Error] i2c 8 bits register write error");
	}

	result = wiringPiI2CWrite(descriptor,K_DS1621_START_CONVERT);

	if(0 != result){
		throw std::runtime_error("[Error] i2c 8 bits register write error");
	}
}

float readTemp(){
	float temperatura = 0;

	int nTemp = (signed int) wiringPiI2CReadReg16(descriptor, K_DS1621_READ_TEMP);
	if ((signed char) nTemp < 0) {
		// LSB is the temperature
		// Bit 7 of MSB is set in case of -0.5C
		temperatura= (signed char) nTemp - 0.5 * (nTemp >> 15);
	}
	else {
		// LSB is the temperature
		// Bit 7 of MSB is set in case of +0.5C
		temperatura = (unsigned char) nTemp + 0.5 * (nTemp >> 15);
	}
	return temperatura;
}

void *ledPWM(void *t){

	float step = 1024/(TEMP_MAX - TEMP_MIN);

	while(1){
		pthread_mutex_lock(&mutex);
		float temp_led = readTemp();
		pthread_mutex_unlock(&mutex);

		if (temp_led > TEMP_MAX){
			temp_led = TEMP_MAX;
		}
		if (temp_led < TEMP_MIN){
			temp_led = TEMP_MIN;
		}

		temp_led -= TEMP_MIN;
		int bright = int(step*temp_led);

		pwmWrite(LED, bright) ;
	}
}

void *updateTemp(void *t){
	while(1){
		pthread_mutex_lock(&mutex);
		temp = readTemp();
		pthread_mutex_unlock(&mutex);
		usleep(DELAY*1000000); // wait for a programmable delay
	}
}

void error(const char *msg){
	perror(msg);
	exit(1);
}

void initServer()
{
	struct sockaddr_in serv_addr;
	short portno;
	socketServer = socket(AF_INET, SOCK_STREAM, 0);
	if (socketServer < 0)
		error("ERROR opening socket");
	bzero((char *) &serv_addr, sizeof(serv_addr));
	portno = 8080; // puerto del servidor http, para poder ser consultado por el navegador
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(portno);
	if (bind(socketServer, (struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) // comprobar error
	{
		error("Error bind");
		exit(1);
	}
	listen(socketServer,5); //preparar la recepccion de peticiones
}

void *atenderPeticion(void *t)
{
	struct sockaddr_in cli_addr;
	int bytes;
	char buffer[256];
	socklen_t clilen;
	char mensaje[100];

	clilen = sizeof(cli_addr);

	while(1){
		int cliente = accept(socketServer, (struct sockaddr *) &cli_addr, &clilen); // esperar peticion de datos
		if (cliente < 0) // comprobar error
		{
			error("Error accept");
		}
		if (cliente >= 0) { //
			printf("new client \n");
			while (cliente >= 0) {
				bytes = read(cliente,buffer,255); // leer peticion
				if (bytes<=0)
				{
					cliente = -1;
					break;// socket cerrado.
				}
				printf(buffer); // preparar mensaje
				strcpy(mensaje,"HTTP/1.0 200 OK\n"); // Cabecera: identificacion version http
				bytes = write(cliente,mensaje,strlen(mensaje));
				if (bytes<=0) // error
				{
					cliente = -1;
					break;// socket cerrado.
				}
				strcpy(mensaje,"Content-Type: text/html\n"); // cabecera: tipo de fichero
				bytes = write(cliente,mensaje,strlen(mensaje));
				if (bytes<=0) // error
				{
					cliente = -1;
					break;// socket cerrado.
				}
				strcpy(mensaje,"Connection: close\n");
				bytes = write(cliente,mensaje,strlen(mensaje));
				if (bytes<=0) // error
				{
					cliente = -1;
					break;// socket cerrado.
				}
				strcpy(mensaje,"\n");
				bytes = write(cliente,mensaje,strlen(mensaje)); // retorno de carro
				if (bytes<=0) // error
				{
					cliente = -1;
					break;// socket cerrado.
				}
				strcpy(mensaje,"<!DOCTYPE HTML>\n"); // identifica que el documento es html
				bytes = write(cliente,mensaje,strlen(mensaje));
				if (bytes<=0) // error
				{
					cliente = -1;
					break;// socket cerrado.
				}
				strcpy(mensaje,"<html>\n"); // comienzo documento
				bytes = write(cliente,mensaje,strlen(mensaje));
				if (bytes<=0) // error
				{
					cliente = -1;
					break;// socket cerrado.
				}
				// read the analog in value:
				float outputValue = temp;

				// map it to the range of the analog out:
				sprintf(mensaje,"<h1>Valor del sensor = %f ºC</h1>\n",outputValue);
				bytes = write(cliente,mensaje,strlen(mensaje));
				if (bytes<=0) // error
				{
					cliente = -1;
					break;// socket cerrado.
				}
				sprintf(mensaje,"<h1>Estado del Pin de alarma = %i</h1>\n",digitalRead (BUTTON));
				bytes = write(cliente,mensaje,strlen(mensaje));
				if (bytes<=0) // error
				{
					cliente = -1;
					break;// socket cerrado.
				}
				strcpy(mensaje,"</html>\n"); // cerrando documento
				bytes = write(cliente,mensaje,strlen(mensaje));
				if (bytes<=0) // error
				{
					cliente = -1;
					break;// socket cerrado.
				}
				break;
			}
			shutdown(cliente, SHUT_RDWR);
		}
	}
}

void *alarmaUDP(void *) {
	int socketUdp;
	int value;
	struct sockaddr_in dest, serv_addr;
	// creamos socket UDP
	socketUdp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (socketUdp < 0) {
		perror("ERROR opening socket");
		exit(4);
	}
	// esto no es necesario si no se pretende leer nada de vuelta, solo enviar
	bzero((char *) &serv_addr, sizeof(serv_addr));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_addr.sin_port = htons(PORT_UDP);

	dest.sin_family = AF_INET;
	inet_pton(AF_INET, "192.168.137.2", &(dest.sin_addr)); // direccion del nodo al que quiero enviar el mensaje
	dest.sin_port = htons(PORT_UDP);

	value = digitalRead(BUTTON);
	while (1) {
		int tempvalue = digitalRead(BUTTON);
		if (value != tempvalue) {
			value = tempvalue;
			sendto(socketUdp, &value, sizeof(value), 0,
					(struct sockaddr*) &dest, sizeof dest);
			printf("Mensaje de alarma mandado por UDP\n");
		}
	}
}

void initsystem(){

	pinMode (BUTTON, INPUT) ;
	pinMode (LED, PWM_OUTPUT) ; 			// pin físico 12, logico 1, este soporta PWM

	setupSensor();

	initServer();

	pthread_create (&idHilo_temperatura, NULL, updateTemp, NULL);
	pthread_create (&idHilo_ledPWM, NULL, ledPWM, NULL);
	pthread_create (&idHilo_atenderPeticion, NULL, atenderPeticion, NULL);
	pthread_create (&idHilo_alarmaUDP, NULL, alarmaUDP, NULL);
}

int main() {
	if (wiringPiSetup () == -1)
		exit (1) ;
	initsystem();
	printf ("Initialization OK\n") ;
	while(1) pause();
	return 0;
}

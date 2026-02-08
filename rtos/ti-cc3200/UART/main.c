// ******************************************************************************
// PROGRAMA	: main.c
// TARGET	: CC3200
// DESCRIPCION	: Código ejemplo de UART y LEDs de la placa
// AUTOR	: José Manuel Cano García y Eva González Parada
// FECHA	: 10-12-16
// ESQUEMA  : Modo Activo-->Clock 80Mhz
// ******************************************************************************
// Includes de funciones estándar
#include <stdio.h>

// Includes de la biblioteca Driverlib para CC3200
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"
#include "uart.h"

// Otros includes de ficheros desarrollados
#include "pinmux.h"


//*****************************************************************************
//                VARIABLES GLOBALES -- Inicio
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif


//*****************************************************************************
//                  VARIABLES GLOBALES -- Fin
//*****************************************************************************


//*****************************************************************************
//                     PROTOTIPOS DE FUNCIONES LOCALES -- Inicio
//*****************************************************************************
static void BoardInit(void);
void UARTIntHandler(void);

//*****************************************************************************
//                     PROTOTIPOS DE FUNCIONES LOCALES -- Fin
//*****************************************************************************

//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//
//! Función main
//!
//! Parámentros: ninguno
//!
//! Retorno: ninguno
//
//*****************************************************************************
int
main()
{
    //
    // Inicialización y configuración de la placa
    //
    BoardInit();

    //
    // Configuración del modo, corriente y dirección de los pines implicados
    //
    PinMuxConfig();

    // Apagado de LED1, LED2 y LED3 (rojo, amarillo y verde)
    GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0);


    //Configuración de los parámetros UART
    UARTConfigSetExpClk(UARTA0_BASE,PRCMPeripheralClockGet(PRCM_UARTA0),
                      115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));


    UARTFIFOEnable(UARTA0_BASE);
    UARTFIFOLevelSet(UARTA0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);


	IntRegister(INT_UARTA0,UARTIntHandler); //Asignación en la Tabla de vectores del manejador de la interrupción de la UART
    UARTIntEnable(UARTA0_BASE, UART_INT_RX|UART_INT_RT); //Habilitación de las  fuentes de interrupción de la UART
    IntEnable(INT_UARTA0); // Habilitación de la interrupciones de la UART
    UARTEnable(UARTA0_BASE);//Habilitación del módulo receptor y transmisión de la UART

    IntMasterEnable();//Habilita las interrupciones del procesador (relacionado con el registro PRIMASK)

    //while(1){ PRCMSleepEnter();}; // Función para introducir al micro en BAJO CONSUMO
    while(1);
    return 0;
}

//*****************************************************************************
//*****************************************************************************



//*****************************************************************************
//         DEFINICIÓN DE FUNCIONES y MENEJADORES DE INTERRUPCIÓN -- Inicio
//*****************************************************************************

//*****************************************************************************
//
//! Inicialización de la placa
//!
//! \Parámentros: ninguno
//!
//! \Retorno: ninguno
//
//*****************************************************************************
static void
BoardInit(void)
{

#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif

    
    //
    // Configuración del microcontrolador tras el encendido, RESET o Hibernación
    //
    PRCMCC3200MCUInit();
}

//*****************************************************************************
//
//! Función de Interrupción de la UART0
//!
//! \Parámentros: ninguno
//!
//! \Retorno: ninguno
//
//*****************************************************************************


void UARTIntHandler(void)
{

	unsigned char rxChar;

	//Interrupción UART handler

	while(UARTCharsAvail(UARTA0_BASE))
	{
		rxChar=UARTCharGet(UARTA0_BASE);
		// Escritura de pins asociados a los leds verda, rojo y amarillo
		switch(rxChar)
		{
		case 'R':
		    GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_1,0);
		    break;
		case 'r':
		    GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_1,GPIO_PIN_1);
		    break;
		case 'A':
		    GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_2,0);
		    break;
		case 'a':
		    GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_2,GPIO_PIN_2);
		    break;
		case 'V':
		    GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_3,0);
		    break;
		case 'v':
		    GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_3,GPIO_PIN_3);
		    break;
		}
		// Echo local
		UARTCharPut(UARTA0_BASE, rxChar);
	}
	UARTIntClear(UARTA0_BASE,UART_INT_RX|UART_INT_RT);
}

//*****************************************************************************
//         DEFINICIÓN DE FUNCIONES y MENEJADORES DE INTERRUPCIÓN -- Fin
//*****************************************************************************



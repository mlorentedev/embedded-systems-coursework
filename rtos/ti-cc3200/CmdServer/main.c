//*****************************************************************************
// Copyright (C) 2014 Texas Instruments Incorporated
//
// All rights reserved. Property of Texas Instruments Incorporated.
// Restricted rights to use, duplicate or disclose this code are
// granted through contract.
// The program may not be used without the written permission of
// Texas Instruments Incorporated or against the terms and conditions
// stipulated in the agreement under which this program has been supplied,
// and under no circumstances can it be used with non-TI connectivity device.
//
//*****************************************************************************
// Modificado, adaptado y ampliado por Jose Manuel Cano Garcia y Eva Gonzalez
// Departamento de Tecnologia Electronica
// Universidad de Malaga
//*****************************************************************************
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup mqtt_client
//! @{
//
//*****************************************************************************

#include<stdint.h>
#include<stdbool.h>

// Standard includes
#include <stdlib.h>

// driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "interrupt.h"
#include "rom_map.h"
#include "prcm.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"
#include "utils.h"
#include "utils/uartstdio.h"
#include "utils/cpu_usage.h"
// application specific includes
#include "pinmux.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "portmacro.h"
//L34
#include "event_groups.h"



//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************

void BoardInit(void);
void ButtonInid(void);


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

extern void (* const g_pfnVectors[])(void);

uint32_t g_ui32CPUUsage=0;
static SemaphoreHandle_t semaforo1,semaforo2;

// Cola para comuniación entre tareas
QueueHandle_t cola1;

/*-----------------------------------------------------------*/

#define POSICIONES_COLA         5
#define ITERACIONES_ISR_BOTON   5
#define DELAY_ISR_BOTON         500

extern void vUARTTask( void *pvParameters );

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************



//****************************************************************************

//*****************************************************************************
// FreeRTOS User Hook Functions enabled in FreeRTOSConfig.h
//*****************************************************************************

//*****************************************************************************
//
//! \brief Application defined hook (or callback) function - assert
//!
//! \param[in]  pcFile - Pointer to the File Name
//! \param[in]  ulLine - Line Number
//!
//! \return none
//!
//*****************************************************************************
void
vAssertCalled( const char *pcFile, unsigned long ulLine )
{
    //Handle Assert here
    while(1)
    {
    }
}

//*****************************************************************************
//
//! \brief Application defined idle task hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
//void
//vApplicationIdleHook( void)
//{
//    //Handle Idle Hook for Profiling, Power Management etc
//	//
//	PRCMSleepEnter();
//
//}

//*****************************************************************************
//
//! \brief Application defined malloc failed hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vApplicationMallocFailedHook()
{
    //Handle Memory Allocation Errors
    while(1)
    {
    }
}

//*****************************************************************************
//
//! \brief Application defined stack overflow hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vApplicationStackOverflowHook( void *pxTask,
                                   signed char *pcTaskName)
{
    //Handle FreeRTOS Stack Overflow
    while(1)
    {
    }
}
//-------------- Fin ganchos -------------



//------------------------------------ Rutinas de interrupcion -----------------------
// ------------------------------------------------------------ -----------------------
void Button1_ISR (void)
{	//ISR boton izquierdo (SW3)
	uint32_t status;
	BaseType_t xHigherPriorityTaskWoken=pdFALSE;
    unsigned int parametros[3];
    parametros[0] = ITERACIONES_ISR_BOTON;
    parametros[1] = DELAY_ISR_BOTON;
    parametros[2] = 0;

    UtilsDelay(8000);	//antirrebote Sw "cutre", gasta tiempo de CPU ya que las interrupciones tienen prioridad sobre las tareas
    						//Por simplicidad lo dejamos así...
    status =  GPIOIntStatus(GPIOA1_BASE,1);

    if (status&GPIO_PIN_5)
    {
    	xSemaphoreGiveFromISR (semaforo1, &xHigherPriorityTaskWoken );
        xQueueSendFromISR(cola1,&parametros,&xHigherPriorityTaskWoken); //Envia parametros del led a la cola de mensajes
    }
    GPIOIntClear(GPIOA1_BASE,status);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void Button2_ISR (void)
{	//ISR boton derecho (SW2)
	uint32_t status;
	BaseType_t xHigherPriorityTaskWoken=pdFALSE;
    unsigned int parametros[3];
    parametros[0] = ITERACIONES_ISR_BOTON;
    parametros[1] = DELAY_ISR_BOTON;
    parametros[2] = 1;

	UtilsDelay(8000);	//antirrebote Sw "cutre", gasta tiempo de CPU ya que las interrupciones tienen prioridad sobre las tareas
	//Por simplicidad lo dejamos así...

	status =  GPIOIntStatus(GPIOA2_BASE,1);

	if (status&GPIO_PIN_6)
	{
		xSemaphoreGiveFromISR (semaforo2, &xHigherPriorityTaskWoken );
        xQueueSendFromISR(cola1,&parametros,&xHigherPriorityTaskWoken); //Envia parametros del led a la cola de mensajes
	}
	GPIOIntClear(GPIOA2_BASE,status);

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
// -------------------- FIN ISR ---------------------------------------- -----------------------

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
void BoardInit(void)
{

    //
    // Set vector table base
    //

    IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);

    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}
  
//Initialze Buttons and ISR
void ButtonsInit(void)
{
	//Los puertos GPIO1 y 2 ya han sido habilitados en la función PinMuxConfig, por lo que no tengo que hacerlo aqui
	// Aqui activamos las interrupciones e instalamos los manejadores de interrupción
	//
	GPIOIntTypeSet(GPIOA1_BASE,GPIO_PIN_5,GPIO_FALLING_EDGE); //Configura flanco de bajada en el PIN5
	IntPrioritySet(INT_GPIOA1, configMAX_SYSCALL_INTERRUPT_PRIORITY); //Configura la prioridad de la ISR
	GPIOIntRegister(GPIOA1_BASE, Button1_ISR); //Registra Button1_ISR como rutina de interrupción del puerto
	GPIOIntClear(GPIOA1_BASE,GPIO_PIN_5); //Borra el flag de interrupcion
	GPIOIntEnable(GPIOA1_BASE,GPIO_INT_PIN_5); //Habilita la interrupcion del PIN
	IntEnable(INT_GPIOA1); //Habilita la interrupcion del puerto

	//Igual configuracion para el otro boton (PIN6 del puerto 2).
	GPIOIntTypeSet(GPIOA2_BASE,GPIO_PIN_6,GPIO_FALLING_EDGE);
	IntPrioritySet(INT_GPIOA2, configMAX_SYSCALL_INTERRUPT_PRIORITY);
	GPIOIntRegister(GPIOA2_BASE, Button2_ISR);
	GPIOIntClear(GPIOA2_BASE,GPIO_PIN_6);
	GPIOIntEnable(GPIOA2_BASE,GPIO_INT_PIN_6);
	IntEnable(INT_GPIOA2);

}

/*--------------------- Tareas --------------------------------------*/
//Esta tarea parpadea el led verde
static void GreenLedBlink( void *pvParameters )
{
	//unsigned int iteraciones;

	//La tarea debe tener un bucle infinito...
	for( ;; )
	{

		//Parpadea el número de veces especificado por el numero
		vTaskDelay(0.2*configTICK_RATE_HZ);
		GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_3,GPIO_PIN_3);
		vTaskDelay(0.2*configTICK_RATE_HZ);
		GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_3,0);
	}
}

//Esta tarea parpadea el led rojo
static void LedBlink( void *pvParameters )
{
    unsigned int i = 0;
    unsigned int led = 0;
    unsigned int iteraciones = 0;
    unsigned int delay = 0;
    unsigned int ptr[3];

    for ( ;; )
    {
        // Leemos dato de la cola de mensajes

        while (xQueueReceive(cola1,&ptr,portMAX_DELAY))
        {
            iteraciones = ptr[0];
            delay = ptr[1];
            led = ptr[2]; // si led 0: rojo si led 1: naranja

            for (i = 0; i < iteraciones; i++)
            {
                vTaskDelay(delay);

                if(!led)
                    GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_1,GPIO_PIN_1);
                else
                    GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_2,GPIO_PIN_2);

                vTaskDelay(delay);

                if(!led)
                    GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_1,0);
                else
                    GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_2,0);
            }
        }
    }
}


//Hay una tarea mas, pero esta implementada en commands.c...

/*--------------------- Fin Tareas --------------------------------------*/


//*****************************************************************************
//
//! Main 
//!
//! \param  none
//!
//! This function
//!    1. Invokes the SLHost task (tarea asociada a SimpleLink)
//!    2. Invokes the TCPServer task
//!
//! \return None
//!
//*****************************************************************************
void main()
{ 
    //
    // Initialize the board configurations
    //
    BoardInit();

    //
    // Pinmux for LED, Buttons and UART
    //
    PinMuxConfig();

    //All LEDs off
    GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1,0);

     //
     // Configuring UART
     //
     UARTStdioConfig(0,115200);

     ButtonsInit();	//configure the buttons

     CPUUsageInit(configTICK_RATE_HZ/10, 3); //Cuidado que solo tiene 4 timers!!!

	/* luego creamos las tareas */
	if (xTaskCreate( GreenLedBlink, "GBli", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL )!=pdPASS)
		while (1);

	// tarea de parpadeo
    if (xTaskCreate( LedBlink, "Blink", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL )!=pdPASS)
        while (1);

	/* Crea la tarea que gestiona los comandos */
	if (xTaskCreate( vUARTTask, "command", 512, NULL, tskIDLE_PRIORITY+1, NULL )!=pdPASS)
			while (1);

    //Crea una cola de tamaño 3 posiciones de tipo entero.
    cola1 = xQueueCreate(POSICIONES_COLA,sizeof(unsigned int[3]));
    if (cola1==NULL)
    {
        while(1);
    }


	semaforo1 = xSemaphoreCreateBinary();
	if( semaforo1 == NULL )
	{
	        while(1);
	}

	semaforo2 = xSemaphoreCreateBinary();
	if( semaforo2 == NULL )
	{
		        while(1);
	}

	//
	// Start the task scheduler
	//
    vTaskStartScheduler();

}



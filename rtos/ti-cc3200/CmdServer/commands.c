//*****************************************************************************
//
// commands.c - FreeRTOS porting example on CCS4
//
//
//*****************************************************************************


#include <stdbool.h>
#include <stdint.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Standard TIVA includes */
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"

/* Other TIVA includes */
#include "utils/cpu_usage.h"
#include "utils/cmdline.h"
#include "utils/ustdlib.h"

#include "utils/uartstdio.h"

// ==============================================================================
// The CPU usage in percent, in 16.16 fixed point format.
// ==============================================================================
extern uint32_t g_ui32CPUUsage;

// Cola para comuniación entre tareas
extern QueueHandle_t cola1;

// ==============================================================================
// Implementa el comando cpu que muestra el uso de CPU
// ==============================================================================
int  Cmd_cpu(int argc, char *argv[])
{
    //
    // Print some header text.
    //
    UARTprintf("ARM Cortex-M4F %u MHz - ",configCPU_CLOCK_HZ / 1000000);
    UARTprintf("%2u%% de uso\n", ((g_ui32CPUUsage+32768) >> 16));

    // Return success.
    return(0);
}

// ==============================================================================
// Implementa el comando free, que muestra cuanta memoria "heap" le queda al FreeRTOS
// ==============================================================================
int Cmd_free(int argc, char *argv[])
{
    //
    // Print some header text.
    //
#ifdef HEAP_3
	UARTprintf("Estas usando HEAP3, no es posible saberlo\n");
#else
    UARTprintf("%d bytes libres\n", xPortGetFreeHeapSize());
#endif

    // Return success.
    return(0);
}

// ==============================================================================
// Implementa el comando task. Sï¿½lo es posible si la opciï¿½n configUSE_TRACE_FACILITY de FreeRTOS estï¿½ habilitada
// ==============================================================================
#if ( configUSE_TRACE_FACILITY == 1 )

extern char *__stack;
int Cmd_tasks(int argc, char *argv[])
{
	char*	pcBuffer;
	uint8_t*	pi8Stack;
	portBASE_TYPE	x;
	
	pcBuffer = pvPortMalloc(1024);
	vTaskList(pcBuffer);
	UARTprintf("\t\t\t\tUnused\nTaskName\tStatus\tPri\tStack\tTask ID\n");
	UARTprintf("=======================================================\n");
	UARTprintf("%s", pcBuffer);
	
	// Calculate kernel stack usage
	x = 0;
	pi8Stack = (uint8_t *) &__stack;
	while (*pi8Stack++ == 0xA5)
	{
		x++;	//Esto sólo funciona si hemos rellenado la pila del sistema con 0xA5 en el arranque (ResetISR en startup_ccs.c)
	}
	usprintf((char *) pcBuffer, "%%%us", configMAX_TASK_NAME_LEN);
	usprintf((char *) &pcBuffer[10], (const char *) pcBuffer, "kernel");
	UARTprintf("%s\t-\t*%u\t%u\t-\n", &pcBuffer[10], configKERNEL_INTERRUPT_PRIORITY, x/sizeof(portBASE_TYPE));
	vPortFree(pcBuffer);
	return 0;
}

#endif /* configUSE_TRACE_FACILITY */

#if configGENERATE_RUN_TIME_STATS
// ==============================================================================
// Implementa el comando "Stats"
// ==============================================================================
Cmd_stats(int argc, char *argv[])
{
	char*	pBuffer;

	pBuffer = pvPortMalloc(1024);
	if (pBuffer)
	{
		vTaskGetRunTimeStats(pBuffer); //Es un poco inseguro, pero por ahora nos vale...
		UARTprintf("TaskName\tCycles\t\tPercent\r\n");
		UARTprintf("===============================================\r\n");
		UARTprintf("%s", pBuffer);
		vPortFree(pBuffer);
	}
	return 0;
}
#endif

// ==============================================================================
// Implementa el comando help
// ==============================================================================
int Cmd_help(int argc, char *argv[])
{
    tCmdLineEntry *pEntry;

    //
    // Print some header text.
    //
    UARTprintf("Comandos disponibles\n");
    UARTprintf("------------------\n");

    //
    // Point at the beginning of the command table.
    //
    pEntry = &g_psCmdTable[0];

    //
    // Enter a loop to read each entry from the command table.  The end of the
    // table has been reached when the command name is NULL.
    //
    while(pEntry->pcCmd)
    {
        //
        // Print the command name and the brief description.
        //
        UARTprintf("%s%s\n", pEntry->pcCmd, pEntry->pcHelp);

        //
        // Advance to the next entry in the table.
        //
        pEntry++;
    }

    //
    // Return success.
    //
    return(0);
}

// ==============================================================================
// Implementa el comando "LED"
// ==============================================================================
int Cmd_led(int argc, char *argv[])
{
	if (argc != 3)
	{
		//Si los parametros no son suficientes o son demasiados, muestro la ayuda
		UARTprintf(" led [red|orange] [on|off]\n");
	}
	else
	{
		//seconds = ustrtoul(argv[1], NULL, 10);

		if (0==strncmp( argv[1], "red",3) && 0==strncmp( argv[2], "on",2))
		{
			UARTprintf("Enciendo el LED rojo\n");
			GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_1,GPIO_PIN_1);
		}
		else if (0==strncmp( argv[1], "red",3) && 0==strncmp( argv[2], "off",3))
		{
			UARTprintf("Apago el LED rojo\n");
			GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_1,0);
		}
		else if (0==strncmp( argv[1], "orange",6) && 0==strncmp( argv[2], "on",2))
        {
            UARTprintf("Enciendo el LED naranja\n");
            GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_2,GPIO_PIN_2);
        }
        else if (0==strncmp( argv[1], "orange",6) && 0==strncmp( argv[2], "off",3))
        {
            UARTprintf("Apago el LED naranja\n");
            GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_2,0);
        }
		else
		{
			//Si el parametro no es correcto, muestro la ayuda
			UARTprintf(" led [red|orange] [on|off]\n");
		}

	}

    return 0;
}

// ==============================================================================
// Implementa el comando "BLINK"
// ==============================================================================
int Cmd_blink(int argc, char *argv[])
{
    // Parámetros de la tarea blink
    unsigned int parametros[3];

    if (argc != 4)
    {
        //Si los parametros no son suficientes o son demasiados, muestro la ayuda
        UARTprintf(" blink [red|orange] [# de parpadeos] [semiperiodo]\n");
    }
    else
    {
        //seconds = ustrtoul(argv[1], NULL, 10);
        parametros[0] = atoi(argv[2]);
        parametros[1] = atoi(argv[3]);

        if (0==strncmp( argv[1], "red",3))
        {
            parametros[2] = 0; //indico que el led a parpadear es el rojo

            UARTprintf("Parpadeo del LED rojo %d iteraciones con semiperiodo de %d\n", parametros[0], parametros[1]);
            xQueueSend(cola1,&parametros,portMAX_DELAY); //Envia parametros del led a la cola de mensajes
        }
        else if (0==strncmp( argv[1], "orange",6))
        {
            parametros[2] = 1; //indico que el led a parpadear es el naranja

            UARTprintf("Parpadeo del LED naranja %d iteraciones con semiperiodo de %d\n", parametros[0], parametros[1]);

            xQueueSend(cola1,&parametros,portMAX_DELAY); //Envia parametros del led a la cola de mensajes
        }
        else
        {
            //Si el parametro no es correcto, muestro la ayuda
            UARTprintf(" blink [red|orange] [# de parpadeos] [semiperiodo]\n");
        }

    }


    return 0;
}

// ==============================================================================
// Tabla con los comandos y su descripcion. Si quiero anadir alguno, debo hacerlo aqui
// ==============================================================================
tCmdLineEntry g_psCmdTable[] =
{
    { "help",     Cmd_help,      "     : Lista de comandos" },
    { "?",        Cmd_help,      "        : lo mismo que help" },
    { "cpu",      Cmd_cpu,       "      : Muestra el uso de  CPU " },
    { "led",  	  Cmd_led,       "      : Apaga y enciende el led rojo o naranja" }, /* descomentar G3a.1 */
    { "blink",    Cmd_blink,     "    : Parpadeo de un led de la placa" },
    { "free",     Cmd_free,      "     : Muestra la memoria libre" },
#if ( configUSE_TRACE_FACILITY == 1 )
	{ "tasks",    Cmd_tasks,     "    : Muestra informacion de las tareas" },
#endif
#if (configGENERATE_RUN_TIME_STATS)
	{ "stats",    Cmd_stats,     "    : Muestra estadisticas de las tareas" },
#endif
    { 0, 0, 0 }
};

// ==============================================================================
// Tarea UARTTask.  Espera la llegada de comandos por el puerto serie y los ejecuta al recibirlos...
//					Por tanto, es la que implementa el interprete de comandos.
// ==============================================================================

void vUARTTask( void *pvParameters )
{
    char    pcCmdBuf[64];
    int32_t i32Status;

	UARTprintf("\n\n FreeRTOS %s \n",
		tskKERNEL_VERSION_NUMBER);
	UARTprintf("\n Teclee ? para ver la ayuda \n");
	UARTprintf("> ");

	/* Loop forever */
    while (1)
    {
	    	/* Read data from the UART and process the command line */
	        UARTgets(pcCmdBuf, sizeof(pcCmdBuf));	//UARTgets es bloqueante porque usa internamente una cola de mensaje
	        if (ustrlen(pcCmdBuf) == 0)
	        {
	            UARTprintf("> ");
	            continue;
	        }
	
	        //
	        // Pass the line from the user to the command processor.  It will be
	        // parsed and valid commands executed.
	        //
	        i32Status = CmdLineProcess(pcCmdBuf);
	
	        //
	        // Handle the case of bad command.
	        //
	        if(i32Status == CMDLINE_BAD_CMD)
	        {
	            UARTprintf("Comando erroneo!\n");	//No pongo acentos adrede
	        }
	
	        //
	        // Handle the case of too many arguments.
	        //
	        else if(i32Status == CMDLINE_TOO_MANY_ARGS)
	        {
	            UARTprintf("El interprete de comandos no admite tantos parametros\n");	//El maximo, CMDLINE_MAX_ARGS, esta definido en cmdline.c
	        }
	
	        //
	        // Otherwise the command was executed.  Print the error code if one was
	        // returned.
	        //
	        else if(i32Status != 0)
	        {
	            UARTprintf("El comando devolvio el error %d\n",i32Status);
	        }
	
	        UARTprintf("> ");
    }
}

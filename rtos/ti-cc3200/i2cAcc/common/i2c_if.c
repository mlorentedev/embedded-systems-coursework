//*****************************************************************************
// i2c_if.c
//
// I2C interface APIs. Operates in a polled mode.
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//Adaptada a FreeRTOS por J. M. Cano,  E Gonzalez e Ignacio Herrero
// Para mejorar la eficiencia gestion del bus I2C aprovechando el RTOS se utilizan los siguientes mecanismos:
// -> Las transacciones I2C las realiza una rutina de interrupción
// -> Las funciones I2C_IF_Read, I2C_IF_ReadFrom e I2C_IF_Write envian los datos de la transaccion a realizar mediante una cola de mensaje.
// -> La ISR va procesando las peticiones de transaccion/realizando nuevas transacciones. Cuando finaliza cada transaccion utiliza las DirectToTaskNotification para desbloquear la tarea.
// Se mantiene la compatibilidad hacia atras, por eso las funciones de las bibliotecas bma222drv.c y tmp000drv.c no hay que cambiarlas.


// Standard includes
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_ints.h"
#include "hw_i2c.h"
#include "i2c.h"
#include "pin.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "interrupt.h"

// Common interface include
#include "i2c_if.h"

//Include FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "portmacro.h"


typedef struct {
	TaskHandle_t OriginTask;	/* Tarea que origina la peticion */
	uint8_t *buffer;	/* puntero a los datos TX/RX */
	uint8_t rxlenght;	/* longitud a recibir */
	uint8_t txlenght;	/* longitud a transmitir */
	uint8_t command;	/* comando */
	uint8_t dev_address; /* direccion I2C */
} I2C_Transaction;



//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define I2C_BASE                I2CA0_BASE
#define SYS_CLK                 80000000
#define FAILURE                 -1
#define SUCCESS                 0
#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}

//ISR states....
#define STATE_IDLE              0
#define STATE_WRITE_NEXT        1
#define STATE_WRITE_FINAL       2
#define STATE_READ_NEXT         3
#define STATE_READ_FINAL        4

//Comandos (tipos de transaccion) que se pueden realizar
#define I2C_COMMAND_WRITE 0
#define I2C_COMMAND_READ 1
#define I2C_COMMAND_READ_FROM 2

//Numero maximo de ordenes de transaccion que se pueden acumular en la cola
#define MAX_I2C_TRANSACTIONS 16

//Flags para las DirectToTaskNotifications
#define I2C_NOTIFY_READ_COMPLETE (0x01)
#define I2C_NOTIFY_WRITE_COMPLETE (0x02)
#define I2C_NOTIFY_ERR (0x04)


//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                          
//****************************************************************************
static int I2CTransact(unsigned long ulCmd);


//Controla los estados que atraviesa la ISR mientras se va realizando la transaccion
//De esta forma cada vez que salta la ISR se ejecuta un comportamiento que depende de su estado
static volatile uint16_t g_i2cisrstate=STATE_IDLE;
static QueueHandle_t g_I2Cqueue;


//****************************************************************************
//
//! Invokes the transaction over I2C
//!
//! \param ulCmd is the command to be executed over I2C
//! 
//! This function works in a polling mode,
//!    1. Initiates the transfer of the command.
//!    2. Waits for the I2C transaction completion
//!    3. Check for any error in transaction
//!    4. Clears the master interrupt
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
static int 
I2CTransact(unsigned long ulCmd)
{
    //
    // Clear all interrupts
    //
    MAP_I2CMasterIntClearEx(I2C_BASE,MAP_I2CMasterIntStatusEx(I2C_BASE,false));
    //
    // Set the time-out. Not to be used with breakpoints.
    //
    MAP_I2CMasterTimeoutSet(I2C_BASE, I2C_TIMEOUT_VAL);
    //
    // Initiate the transfer.
    //
    MAP_I2CMasterControl(I2C_BASE, ulCmd);


    //
    // Check for any errors in transfer
    //
    if(MAP_I2CMasterErr(I2C_BASE) != I2C_MASTER_ERR_NONE)
    {
        switch(ulCmd)
        {
        case I2C_MASTER_CMD_BURST_SEND_START:
        case I2C_MASTER_CMD_BURST_SEND_CONT:
        case I2C_MASTER_CMD_BURST_SEND_STOP:
            MAP_I2CMasterControl(I2C_BASE,
                         I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
            break;
        case I2C_MASTER_CMD_BURST_RECEIVE_START:
        case I2C_MASTER_CMD_BURST_RECEIVE_CONT:
        case I2C_MASTER_CMD_BURST_RECEIVE_FINISH:
            MAP_I2CMasterControl(I2C_BASE,
                         I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP);
            break;
        default:
            break;
        }
        return FAILURE;
    }

    return SUCCESS;
}

//****************************************************************************
//
//! Invokes the I2C driver APIs to write to the specified address
//!
//! \param ucDevAddr is the 7-bit I2C slave address
//! \param pucData is the pointer to the data to be written
//! \param ucLen is the length of data to be written
//! \param ucStop should be set to 1. Mantained for compatibility
//! 
//! This function works in a polling mode,
//!    1. Writes the device register address to be written to.
//!    2. In a loop, writes all the bytes over I2C
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int 
I2C_IF_Write(unsigned char ucDevAddr,
		unsigned char *pucData,
		unsigned char ucLen,
		unsigned char ucStop)
{
	uint32_t notifVal=0;
	I2C_Transaction transaction;

	RETERR_IF_TRUE(pucData == NULL);
	RETERR_IF_TRUE(ucLen == 0);
	RETERR_IF_TRUE(ucStop == 0); //XXX quitar parametro ucStop!!


	transaction.OriginTask=xTaskGetCurrentTaskHandle();
	transaction.buffer=pucData;
	transaction.txlenght=ucLen;
	transaction.rxlenght=0;
	transaction.dev_address=ucDevAddr;
	transaction.command=I2C_COMMAND_WRITE;

	//Envia la transaccion a la cola de mensajes...
	xQueueSend(g_I2Cqueue,&transaction,portMAX_DELAY);

	if (g_i2cisrstate==STATE_IDLE)
	{
		IntPendSet(INT_I2CA0);	//Produce un disparo software de la ISR (comienza a transmitir)....
	}

	//Espera a que se produzca la transacción (o haya error)...
	while (!(notifVal&(I2C_NOTIFY_WRITE_COMPLETE|I2C_NOTIFY_ERR)))
	{
		xTaskNotifyWait( 0, I2C_NOTIFY_WRITE_COMPLETE|I2C_NOTIFY_ERR, &notifVal, portMAX_DELAY);
	}

	if (notifVal&I2C_NOTIFY_ERR) return FAILURE;

	return SUCCESS;

}

//****************************************************************************
//
//! Invokes the I2C driver APIs to read from the device. This assumes the 
//! device local address to read from is set using the I2CWrite API.
//!
//! \param ucDevAddr is the 7-bit I2C slave address
//! \param pucData is the pointer to the read data to be placed
//! \param ucLen is the length of data to be read
//! 
//! This function works in a polling mode,
//!    1. Writes the device register address to be written to.
//!    2. In a loop, reads all the bytes over I2C
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int 
I2C_IF_Read(unsigned char ucDevAddr,
		unsigned char *pucData,
		unsigned char ucLen)
{
	//unsigned long ulCmdID;
	uint32_t notifVal=0;
	I2C_Transaction transaction;

	RETERR_IF_TRUE(pucData == NULL);
	RETERR_IF_TRUE(ucLen == 0);


	transaction.OriginTask=xTaskGetCurrentTaskHandle();
	transaction.buffer=pucData;
	transaction.txlenght=0;
	transaction.rxlenght=ucLen;
	transaction.dev_address=ucDevAddr;
	transaction.command=I2C_COMMAND_READ;

	xQueueSend(g_I2Cqueue,&transaction,portMAX_DELAY);

	if (g_i2cisrstate==STATE_IDLE)
	{
		IntPendSet(INT_I2CA0);	//Produce un disparo software (comienza a transmitir)....
	}


	//Espera a que se produzca la transacción (o haya error)...
	while (!(notifVal&(I2C_NOTIFY_READ_COMPLETE|I2C_NOTIFY_ERR)))
	{
		xTaskNotifyWait( 0, I2C_NOTIFY_READ_COMPLETE|I2C_NOTIFY_ERR, &notifVal, portMAX_DELAY);
	}

	if (notifVal&I2C_NOTIFY_ERR) return FAILURE;

	return SUCCESS;
}

//****************************************************************************
//
//! Invokes the I2C driver APIs to read from a specified address the device. 
//! This assumes the device local address to be of 8-bit. For other 
//! combinations use I2CWrite followed by I2CRead.
//!
//! \param ucDevAddr is the 7-bit I2C slave address
//! \param pucWrDataBuf is the pointer to the data to be written (reg addr)
//! \param ucWrLen is the length of data to be written
//! \param pucRdDataBuf is the pointer to the read data to be placed
//! \param ucRdLen is the length of data to be read
//! 
//! This function works in a polling mode,
//!    1. Writes the data over I2C (device register address to be read from).
//!    2. In a loop, reads all the bytes over I2C
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int 
I2C_IF_ReadFrom(unsigned char ucDevAddr,
            unsigned char *pucWrDataBuf,
            unsigned char ucWrLen,
            unsigned char *pucRdDataBuf,
            unsigned char ucRdLen)
{
	I2C_Transaction transaction;
	uint32_t notifVal=0;

	    RETERR_IF_TRUE(pucRdDataBuf == NULL);
	    RETERR_IF_TRUE(pucWrDataBuf == NULL);
	    RETERR_IF_TRUE(ucWrLen == 0);
	    RETERR_IF_TRUE(ucWrLen > ucWrLen);

	    memcpy(pucRdDataBuf,pucWrDataBuf,ucWrLen);
	    transaction.OriginTask=xTaskGetCurrentTaskHandle();
	    transaction.buffer=pucRdDataBuf;
	    transaction.txlenght=ucWrLen;
	    transaction.rxlenght=ucRdLen;
	    transaction.dev_address=ucDevAddr;
	    transaction.command=I2C_COMMAND_READ_FROM;

	    xQueueSend(g_I2Cqueue,&transaction,portMAX_DELAY);

	    if (g_i2cisrstate==STATE_IDLE)
	    {
	    	IntPendSet(INT_I2CA0);	//Produce un disparo software....
	    }

	    //CEspera a que se complete la operacion de escritura/lectura o se produza error
	    while (!(notifVal&(I2C_NOTIFY_READ_COMPLETE|I2C_NOTIFY_ERR)))
	    {
	    	xTaskNotifyWait( 0, I2C_NOTIFY_WRITE_COMPLETE|I2C_NOTIFY_READ_COMPLETE|I2C_NOTIFY_ERR, &notifVal, portMAX_DELAY);
	    }

	    if (notifVal&I2C_NOTIFY_ERR) return FAILURE;

    return SUCCESS;
}


//Rutina de interrupcion.
//Esta rutina parece muy larga, pero sólo se ejecuta una parte u otra según el estado en el que estemos...
//Utiliza una máquina de estados para cambiar el comportamiento cuando se produce la interrupcion, ya que lo que se debe realizar depende de si estamos o no
// en una transacción, del tipo de transaccion (escritura, lectura o escritura-lectura, y de que punto de dicha transacción estamos.
// Para ello se utiliza la variable de estado g_i2cisrstate.

static void I2C_IF_ISR(void)
{
	BaseType_t xHigherPriorityTaskWoken=pdFALSE;

	//The following two variables should maintain values between calls to the ISR
	static I2C_Transaction transaction;
	static uint8_t *tmpptr;

	//Primero tratamos posible error...
	if (MAP_I2CMasterIntStatusEx(I2C_BASE,true)&(I2C_MASTER_INT_TIMEOUT|I2C_MASTER_INT_NACK))
	{

		MAP_I2CMasterIntDisableEx(I2C_BASE,(I2C_MASTER_INT_DATA|I2C_MASTER_INT_TIMEOUT|I2C_MASTER_INT_NACK));
		xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
		xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_ERR,eSetBits,&xHigherPriorityTaskWoken);
		MAP_I2CMasterIntClearEx(I2C_BASE,MAP_I2CMasterIntStatusEx(I2C_BASE,false)); //Transact already does this
		switch (g_i2cisrstate)
		{
			case STATE_READ_NEXT:
		 	case STATE_READ_FINAL:
				I2CMasterControl(I2C_BASE,I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
				break;
			case STATE_WRITE_NEXT:
			case STATE_WRITE_FINAL:
				I2CMasterControl(I2C_BASE,I2C_MASTER_CMD_BURST_SEND_FINISH);
				break;
		}
		g_i2cisrstate=STATE_IDLE;
		if (xQueuePeekFromISR(g_I2Cqueue,&transaction)) IntPendSet(INT_I2CA0);
		return;
	}

	//Ejecuta la maquina de estados para responder al evento. Miramos primero en qué estado estamos.
	switch(g_i2cisrstate)
	{
		case STATE_IDLE:
		{
			//Arranca la transaccion....
			if (xQueuePeekFromISR(g_I2Cqueue,&transaction)==pdTRUE)
			{
				//Hay algo en la cola... puedo comenzar
				switch (transaction.command)
				{
					case I2C_COMMAND_WRITE:
					case I2C_COMMAND_READ_FROM:
					{	//Disparo una escritura
						tmpptr=transaction.buffer;
						MAP_I2CMasterSlaveAddrSet(I2C_BASE, transaction.dev_address, false); // Set I2C codec slave address
						I2CMasterDataPut(I2C_BASE, *tmpptr);	 // Write the first byte to the controller.
						if (I2CTransact(I2C_MASTER_CMD_BURST_SEND_START)==SUCCESS)
						{
							transaction.txlenght--;
							tmpptr++;
							I2CMasterIntEnableEx(I2C_BASE,I2C_MASTER_INT_DATA|I2C_MASTER_INT_TIMEOUT|I2C_MASTER_INT_NACK);
							if (transaction.txlenght>0)
								g_i2cisrstate=STATE_WRITE_NEXT;	//Cambia de estado. La proxima interrupcion Escribe otro caracter
							else
								g_i2cisrstate=STATE_WRITE_FINAL; //Cambia de estado. La proxima interrupcion finaliza la transmision
						}
						else
						{	//Fallo de transmision. Elimino la transaccion en curso y aviso a la tarea
							//Luego borro los flags de interrupción y compruebo si hay mas transacciones pendientes
							xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
							xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_ERR,eSetBits,&xHigherPriorityTaskWoken);
							MAP_I2CMasterIntClearEx(I2C_BASE,MAP_I2CMasterIntStatusEx(I2C_BASE,false));
							if (xQueuePeekFromISR(g_I2Cqueue,&transaction)) IntPendSet(INT_I2CA0);
						}
					}
					break;
					case I2C_COMMAND_READ:
					{   //Disparo una lectura (simple o multiple, segun el caso)
						tmpptr=transaction.buffer;
						MAP_I2CMasterSlaveAddrSet(I2C_BASE, transaction.dev_address, true); // Set I2C codec slave address
						transaction.rxlenght--;
						if(transaction.rxlenght==0)
						{	//Lectura simple
							if (I2CTransact(I2C_MASTER_CMD_SINGLE_RECEIVE)==SUCCESS)
							{
								g_i2cisrstate=STATE_READ_FINAL; //La siguiente ISR sera el final de lectura
								I2CMasterIntEnableEx(I2C_BASE,I2C_MASTER_INT_DATA|I2C_MASTER_INT_TIMEOUT|I2C_MASTER_INT_NACK);
							}
							else
							{
								//Fallo de transmision. Elimino la transaccion en curso y aviso a la tarea
								//Luego borro los flags de interrupción y compruebo si hay mas transacciones pendientes
								xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
								xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_ERR,eSetBits,&xHigherPriorityTaskWoken);
								MAP_I2CMasterIntClearEx(I2C_BASE,MAP_I2CMasterIntStatusEx(I2C_BASE,false));
								if (xQueuePeekFromISR(g_I2Cqueue,&transaction)) IntPendSet(INT_I2CA0);

							}
						}
						else
						{	//lectura multiple
							if (I2CTransact(I2C_MASTER_CMD_BURST_RECEIVE_START)==SUCCESS)
							{
								g_i2cisrstate=STATE_READ_NEXT;	//La siguente ISR sera la recepcion de un dato
								I2CMasterIntEnableEx(I2C_BASE,I2C_MASTER_INT_DATA|I2C_MASTER_INT_TIMEOUT|I2C_MASTER_INT_NACK);
							}
							else
							{
								//Fallo de transmision. Elimino la transaccion en curso y aviso a la tarea
								//Luego borro los flags de interrupción y compruebo si hay mas transacciones pendientes
								xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
								xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_ERR,eSetBits,&xHigherPriorityTaskWoken);
								MAP_I2CMasterIntClearEx(I2C_BASE,MAP_I2CMasterIntStatusEx(I2C_BASE,false));
								if (xQueuePeekFromISR(g_I2Cqueue,&transaction)) IntPendSet(INT_I2CA0);
							}
						}
					}
					break;
				}
			}
			else
			{	//No habia nada en la cola de ordenes I2C
				//¿¿Que ha pasao??
			}
		}
		break; //FIN DEL CASO STATE_IDLE...

		case STATE_WRITE_NEXT:
		{	//Continuacion de escritura...Envio el siguiente byte
			MAP_I2CMasterDataPut(I2C_BASE, *tmpptr);
			if (I2CTransact(I2C_MASTER_CMD_BURST_SEND_CONT)==SUCCESS)
			{
				transaction.txlenght--;
				tmpptr++;
				if (transaction.txlenght>0)
					g_i2cisrstate=STATE_WRITE_NEXT; //Si hay mas que enviar, no cambio de estado (en realidad podia no hacer nada)
				else
					g_i2cisrstate=STATE_WRITE_FINAL; //Si no hay mas que enviar, la siguiente ISR finaliza la transmision (condicion de STOP)
			}
			else
			{
				//Fallo de transmision. Elimino la transaccion en curso y aviso a la tarea
				//Deshabilita las ISR y vuelve al estado inicial. De parar la transaccion ya se encarga internamente I2CTransact (creo)
				//Finalmente ompruebo si hay mas transacciones pendientes para volver a lanzar una
				I2CMasterIntDisableEx(I2C_BASE,I2C_MASTER_INT_DATA|I2C_MASTER_INT_TIMEOUT|I2C_MASTER_INT_NACK);
				g_i2cisrstate=STATE_IDLE; //Vuelve al estado IDLE
				xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
				xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_ERR,eSetBits,&xHigherPriorityTaskWoken);
				//MAP_I2CMasterIntClearEx(I2C_BASE,MAP_I2CMasterIntStatusEx(I2C_BASE,false)); //Transact already does this
				if (xQueuePeekFromISR(g_I2Cqueue,&transaction)) IntPendSet(INT_I2CA0);
			}
		}
		break; //FIN DEL ESTADO STATE_WRITE_NEXT
		case STATE_WRITE_FINAL:
		{	//Fin transmision. Si era una transmision simple, finalizo y paso a la siguiente
			if (transaction.command!=I2C_COMMAND_READ_FROM)
			{
				//Transaccion finalizada
				//Deshabilito las ISR, vuelvo al estado IDLE, elimino la transaccion de la cola
				//Finalmente compruebo si hay almacenada en la cola para dispararla
				I2CTransact(I2C_MASTER_CMD_BURST_SEND_STOP);
				I2CMasterIntDisableEx(I2C_BASE,I2C_MASTER_INT_DATA|I2C_MASTER_INT_TIMEOUT|I2C_MASTER_INT_NACK);
				g_i2cisrstate=STATE_IDLE; //Vuelve al estado IDLE
				xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
				xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_WRITE_COMPLETE,eSetBits,&xHigherPriorityTaskWoken);	//Transaccion correcta
				//MAP_I2CMasterIntClearEx(I2C_BASE,MAP_I2CMasterIntStatusEx(I2C_BASE,false)); //transact does this
				if (xQueuePeekFromISR(g_I2Cqueue,&transaction)) IntPendSet(INT_I2CA0);
			}
			else
			{
				//Operacion READ_FROM. Finaliza la parte de envio, ahora pasamos a recepcion
				//Comenzar una recepcion!!!
				tmpptr=transaction.buffer;
				MAP_I2CMasterSlaveAddrSet(I2C_BASE, transaction.dev_address, true); // Set I2C codec slave address
				transaction.rxlenght--;
				if(transaction.rxlenght==0)
				{	//Recepcion de un solo byte
					if (I2CTransact(I2C_MASTER_CMD_SINGLE_RECEIVE)==SUCCESS)
					{
						g_i2cisrstate=STATE_READ_FINAL;	//Cambia de estado a finalizar. La proxima ISR finaliza la RX
						I2CMasterIntEnableEx(I2C_BASE,I2C_MASTER_INT_DATA|I2C_MASTER_INT_TIMEOUT|I2C_MASTER_INT_NACK);
					}
					else
					{	//Error
						//Fallo de recepción. Elimino la transaccion en curso y aviso a la tarea
						//Deshabilita las ISR y vuelve al estado inicial. De parar la transaccion ya se encarga internamente I2CTransact (creo)
						//Finalmente ompruebo si hay mas transacciones pendientes para volver a lanzar una
						I2CMasterIntDisableEx(I2C_BASE,I2C_MASTER_INT_DATA|I2C_MASTER_INT_TIMEOUT|I2C_MASTER_INT_NACK);
						g_i2cisrstate=STATE_IDLE;	//Vuelve al estado IDLE
						xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
						xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_ERR,eSetBits,&xHigherPriorityTaskWoken);
						//MAP_I2CMasterIntClearEx(I2C_BASE,MAP_I2CMasterIntStatusEx(I2C_BASE,false)); // unnecesary. Transact does this
						if (xQueuePeekFromISR(g_I2Cqueue,&transaction)) IntPendSet(INT_I2CA0);
					}
				}
				else
				{	//Recepcion de varios bytes
					if (I2CTransact(I2C_MASTER_CMD_BURST_RECEIVE_START)==SUCCESS)
					{
						g_i2cisrstate=STATE_READ_NEXT;	//La proxima ISR continua la recepcion
						I2CMasterIntEnableEx(I2C_BASE,I2C_MASTER_INT_DATA|I2C_MASTER_INT_TIMEOUT|I2C_MASTER_INT_NACK);
					}
					else
					{
						//Fallo de recepción. Elimino la transaccion en curso y aviso a la tarea
						//Deshabilita las ISR y vuelve al estado inicial. De parar la transaccion ya se encarga internamente I2CTransact (creo)
						//Finalmente ompruebo si hay mas transacciones pendientes para volver a lanzar una
						I2CMasterIntDisableEx(I2C_BASE,I2C_MASTER_INT_DATA|I2C_MASTER_INT_TIMEOUT|I2C_MASTER_INT_NACK);
						g_i2cisrstate=STATE_IDLE; //Vuelve al estado IDLE
						xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
						xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_ERR,eSetBits,&xHigherPriorityTaskWoken);
						//MAP_I2CMasterIntClearEx(I2C_BASE,MAP_I2CMasterIntStatusEx(I2C_BASE,false)); //unnecesary. Transacts already check this
						if (xQueuePeekFromISR(g_I2Cqueue,&transaction)) IntPendSet(INT_I2CA0);
					}
				}
			}
		}
		break; //FIN DEL ESTADO STATE_WRITE_FINAL

		case STATE_READ_NEXT:
		{	//Lectura "larga" en curso... Intento leer datos  continuar...
			*tmpptr = MAP_I2CMasterDataGet(I2C_BASE);
			transaction.rxlenght--;
			tmpptr++;
			if (transaction.rxlenght==0)
			{	//Ya no tengo que recibir mas. Ordeno la recepcion del ultimo byte y la condicion de stop
				if (I2CTransact(I2C_MASTER_CMD_BURST_RECEIVE_FINISH)==SUCCESS)
				{
					g_i2cisrstate=STATE_READ_FINAL;	//Ultimo dato, la siguiente ISR finaliza la recepción
				}
				else
				{
					//Fallo de recepción. Elimino la transaccion en curso y aviso a la tarea
					//Deshabilita las ISR y vuelve al estado inicial. De parar la transaccion ya se encarga internamente I2CTransact (creo)
					//Finalmente ompruebo si hay mas transacciones pendientes para volver a lanzar una
					I2CMasterIntDisableEx(I2C_BASE,I2C_MASTER_INT_DATA|I2C_MASTER_INT_TIMEOUT|I2C_MASTER_INT_NACK);
					g_i2cisrstate=STATE_IDLE; //Vuelve al estado IDLE
					xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
					xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_ERR,eSetBits,&xHigherPriorityTaskWoken);
					//MAP_I2CMasterIntClearEx(I2C_BASE,MAP_I2CMasterIntStatusEx(I2C_BASE,false)); //Esto ya lo hace transact
					if (xQueuePeekFromISR(g_I2Cqueue,&transaction)) IntPendSet(INT_I2CA0);
				}
			}
			else
			{	//Tengo que continuar recibiendo
				if (I2CTransact(I2C_MASTER_CMD_BURST_RECEIVE_CONT)==SUCCESS)
				{
					//No state change
				}
				else
				{
					//Fallo de recepción. Elimino la transaccion en curso y aviso a la tarea
					//Deshabilita las ISR y vuelve al estado inicial. De parar la transaccion ya se encarga internamente I2CTransact (creo)
					//Finalmente ompruebo si hay mas transacciones pendientes para volver a lanzar una
					I2CMasterIntDisableEx(I2C_BASE,I2C_MASTER_INT_DATA|I2C_MASTER_INT_TIMEOUT|I2C_MASTER_INT_NACK);
					g_i2cisrstate=STATE_IDLE; //Vuelve al estado IDLE
					xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
					xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_ERR,eSetBits,&xHigherPriorityTaskWoken);
					//MAP_I2CMasterIntClearEx(I2C_BASE,MAP_I2CMasterIntStatusEx(I2C_BASE,false)); //Ya lo hace transact
					if (xQueuePeekFromISR(g_I2Cqueue,&transaction)) IntPendSet(INT_I2CA0);
				}
			}
		}
		break; //FIN DEL ESTADO STATE_READ_NEXT
		case STATE_READ_FINAL:
		{
			//Fin de la lectura/recepcion. Elimino la transaccion en curso y aviso a la tarea
			//Deshabilita las ISR y vuelve al estado inicial.
			//Finalmente ompruebo si hay mas transacciones pendientes para volver a lanzar una
			//Ademas borro los flags de interrupcion (aqui no se llama a transact)
			*tmpptr = MAP_I2CMasterDataGet(I2C_BASE);
			I2CMasterIntDisableEx(I2C_BASE,I2C_MASTER_INT_DATA|I2C_MASTER_INT_TIMEOUT|I2C_MASTER_INT_NACK);
			g_i2cisrstate=STATE_IDLE; //Vuelve al estado IDLE
			xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
			xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_READ_COMPLETE,eSetBits,&xHigherPriorityTaskWoken);
			MAP_I2CMasterIntClearEx(I2C_BASE,MAP_I2CMasterIntStatusEx(I2C_BASE,false));
			if (xQueuePeekFromISR(g_I2Cqueue,&transaction)) IntPendSet(INT_I2CA0);
		}
		break; //FIN DEL ESTADO STATE_READ_FINAL
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}


//****************************************************************************
//
//! Enables and configures the I2C peripheral
//!
//! \param ulMode is the mode configuration of I2C
//! The parameter \e ulMode is one of the following
//! - \b I2C_MASTER_MODE_STD for 100 Kbps standard mode.
//! - \b I2C_MASTER_MODE_FST for 400 Kbps fast mode.
//! 
//! This function works in a polling mode,
//!    1. Powers ON the I2C peripheral.
//!    2. Configures the I2C peripheral
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int 
I2C_IF_Open(unsigned long ulMode)
{
    //
    // Enable I2C Peripheral
    //           
    //MAP_HwSemaphoreLock(HWSEM_I2C, HWSEM_WAIT_FOR_EVER);
    MAP_PRCMPeripheralClkEnable(PRCM_I2CA0, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(PRCM_I2CA0);

    //
    // Configure I2C module in the specified mode
    //
    switch(ulMode)
    {
        case I2C_MASTER_MODE_STD:       /* 100000 */
            MAP_I2CMasterInitExpClk(I2C_BASE,SYS_CLK,false);
            break;

        case I2C_MASTER_MODE_FST:       /* 400000 */
            MAP_I2CMasterInitExpClk(I2C_BASE,SYS_CLK,true);
            break;

        default:
            MAP_I2CMasterInitExpClk(I2C_BASE,SYS_CLK,true);
            break;
    }

    
    //Empezamos por estado IDLE (no hay transaccion en marcha)
    g_i2cisrstate=STATE_IDLE;

    g_I2Cqueue=xQueueCreate(MAX_I2C_TRANSACTIONS,sizeof(I2C_Transaction));
    if (g_I2Cqueue==NULL)
    	while(1);

    //Registra, congigura y habilia la ISR
    IntRegister(INT_I2CA0,I2C_IF_ISR);
    IntPrioritySet(INT_I2CA0,configKERNEL_INTERRUPT_PRIORITY);
    IntEnable(INT_I2CA0);	//Habilita la ISR

    //This disconfigures PIN1 and 2 for LEDs...
    // Configure PIN_01 for I2C0 I2C_SCL
    //
    MAP_PinTypeI2C(PIN_01, PIN_MODE_1);

    //
    // Configure PIN_02 for I2C0 I2C_SDA
    //
    MAP_PinTypeI2C(PIN_02, PIN_MODE_1);


    return SUCCESS;
}

//****************************************************************************
//
//! Disables the I2C peripheral
//!
//! \param None
//! 
//! This function works in a polling mode,
//!    1. Powers OFF the I2C peripheral.
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int 
I2C_IF_Close()
{
    //
    // Power OFF the I2C peripheral
    //

	//xxx Debería comprobarse antes si no hay una transaccion en marcha!!!

	IntDisable(INT_I2CA0);	//Deshabilita la ISR
    MAP_PRCMPeripheralClkDisable(PRCM_I2CA0, PRCM_RUN_MODE_CLK);
    if (g_I2Cqueue!=NULL)
    {
    		vQueueDelete(g_I2Cqueue);
    		g_I2Cqueue=NULL;
    }

    return SUCCESS;
}




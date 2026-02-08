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
// Modificado, adaptado y ampliado por Jose Manuel Cano, Eva Gonzalez, Ignacio Herrero
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

// Standard includes
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

// simplelink includes
#include "simplelink.h"

// driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin.h"
#include "driverlib/prcm.h"
#include "driverlib/timer.h"

// common interface includes
#include "network_if.h"
#include "network_if.h"
#ifndef NOTERM
#include "uart_if.h"
#endif
#include "i2c_if.h"
#include "button_if.h"
#include "gpio_if.h"
#include "timer_if.h"
#include "common.h"
#include "utils.h"
#include "utils/uartstdio.h"

#include "pwm_if.h"

//Sensores
#include "tmp006drv.h"
#include "bma222drv.h"

// application specific includes
#include "pinmux.h"


//Definiciones del protocolo
#include "protocol.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/*Background receive task priority*/
#define TASK_PRIORITY           3


/*Spawn task priority and OSI Stack Size*/
#define OSI_STACK_SIZE          2048
#define UART_PRINT              Report

#define STATION_MODE    // Para que trabaje en modo estación con la WiFi de casa (192.168.1.135), quitar para modo AP


//JMCG Para los sockets...
// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    SOCKET_CREATE_ERROR = -0x7D0,
    BIND_ERROR = SOCKET_CREATE_ERROR - 1,
    LISTEN_ERROR = BIND_ERROR -1,
    SOCKET_OPT_ERROR = LISTEN_ERROR -1,
    CONNECT_ERROR = SOCKET_OPT_ERROR -1,
    ACCEPT_ERROR = CONNECT_ERROR - 1,
    SEND_ERROR = ACCEPT_ERROR -1,
    RECV_ERROR = SEND_ERROR -1,
    SOCKET_CLOSE_ERROR = RECV_ERROR -1,
    DEVICE_NOT_IN_STATION_MODE = SOCKET_CLOSE_ERROR - 1,
    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;



//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************

void ToggleLedState(ledEnum LedNum);
void TimerPeriodicIntHandler(void);
void LedTimerConfigNStart();
void LedTimerDeinitStop();
void BoardInit(void);
static void DisplayBanner(char * AppName);
void TCPServerTask(void *pvParameters);
void AsyncPollingTask(void *pvParameters);
void readTempTask(void *pvParameters);
void readAccTask(void *pvParameters);

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#ifdef USE_FREERTOS
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#endif

unsigned short g_usTimerInts;
/* AP Security Parameters */
SlSecParams_t SecurityParams = {0};

/*Message Queue*/
//OsiMsgQ_t g_PBQueue;

// Socket como variable global para ser utilizado por varios procesos
int iNewSockID;
// variable para capturar el boton de notificion asincrona
int async = 0;
// Variable global de frecuencia de lectura
float freqTemp;
float freqAcc;

//Variable para activar o no la lectura
int read_temp;
int read_acc;

// Variables utilizadas para gestion de tareas y recursos compartidos
static SemaphoreHandle_t semaforo_boton, semaforo_socket, semaforo_i2c, semaforo_temp, semaforo_acc;

#ifdef USE_FREERTOS
extern void vUARTTask( void *pvParameters );
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//*****************************************************************************
//
//! Buttons Interrupt Handler
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void Button_ISR (void)
{
    uint32_t status_sw1;
    uint32_t status_sw2;

    BaseType_t xHigherPriorityTaskWoken=pdFALSE;

    UtilsDelay(8000);   //antirrebote Sw "cutre", gasta tiempo de CPU ya que las interrupciones tienen prioridad sobre las tareas

    // Leemos el estado de ambos botones
    status_sw1 =  GPIOIntStatus(GPIOA1_BASE,1);
    status_sw2 =  GPIOIntStatus(GPIOA2_BASE,1);

    // Si cualquiera ha activado la interrupcion, actuamos
    if ((status_sw1&GPIO_PIN_5) || (status_sw2&GPIO_PIN_6))
    {
        xSemaphoreGiveFromISR( semaforo_boton, &xHigherPriorityTaskWoken );
    }
    GPIOIntClear(GPIOA1_BASE,status_sw1);
    GPIOIntClear(GPIOA2_BASE,status_sw2);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

//*****************************************************************************
//
//! Buttons initialization
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void ButtonsInit(void)
{
    //Los puertos GPIO1 y 2 ya han sido habilitados en la función PinMuxConfig, por lo que no tengo que hacerlo aqui
    // Aqui activamos las interrupciones e instalamos los manejadores de interrupción

    GPIOIntTypeSet(GPIOA1_BASE,GPIO_PIN_5,GPIO_FALLING_EDGE); //Configura flanco de bajada en el PIN5
    IntPrioritySet(INT_GPIOA1, configMAX_SYSCALL_INTERRUPT_PRIORITY); //Configura la prioridad de la ISR
    GPIOIntRegister(GPIOA1_BASE, Button_ISR); //Registra Button1_ISR como rutina de interrupción del puerto
    GPIOIntClear(GPIOA1_BASE,GPIO_PIN_5); //Borra el flag de interrupcion
    GPIOIntEnable(GPIOA1_BASE,GPIO_INT_PIN_5); //Habilita la interrupcion del PIN
    IntEnable(INT_GPIOA1); //Habilita la interrupcion del puerto

    //Igual configuracion para el otro boton (PIN6 del puerto 2).
    GPIOIntTypeSet(GPIOA2_BASE,GPIO_PIN_6,GPIO_FALLING_EDGE);
    IntPrioritySet(INT_GPIOA2, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    GPIOIntRegister(GPIOA2_BASE, Button_ISR);
    GPIOIntClear(GPIOA2_BASE,GPIO_PIN_6);
    GPIOIntEnable(GPIOA2_BASE,GPIO_INT_PIN_6);
    IntEnable(INT_GPIOA2);

}
//*****************************************************************************
//
//! Periodic Timer Interrupt Handler
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void
TimerPeriodicIntHandler(void)
{
    unsigned long ulInts;

    //
    // Clear all pending interrupts from the timer we are
    // currently using.
    //
    ulInts = MAP_TimerIntStatus(TIMERA2_BASE, true);
    MAP_TimerIntClear(TIMERA2_BASE, ulInts);

    //
    // Increment our interrupt counter.
    //
    g_usTimerInts++;
    if(!(g_usTimerInts & 0x1))
    {
        //
        // Off Led
        //
        GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    }
    else
    {
        //
        // On Led
        //
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
    }
}

//****************************************************************************
//
//! Function to configure and start timer to blink the LED while device is
//! trying to connect to an AP
//!
//! \param none
//!
//! return none
//
//****************************************************************************
void LedTimerConfigNStart()
{
    //
    // Configure Timer for blinking the LED for IP acquisition
    //
    Timer_IF_Init(PRCM_TIMERA2,TIMERA2_BASE,TIMER_CFG_PERIODIC,TIMER_A,0);
    Timer_IF_IntSetup(TIMERA2_BASE,TIMER_A,TimerPeriodicIntHandler);
    Timer_IF_Start(TIMERA2_BASE,TIMER_A,100);
}

//****************************************************************************
//
//! Disable the LED blinking Timer as Device is connected to AP
//!
//! \param none
//!
//! return none
//
//****************************************************************************
void LedTimerDeinitStop()
{
    //
    // Disable the LED blinking Timer as Device is connected to AP
    //
    Timer_IF_Stop(TIMERA2_BASE,TIMER_A);
    Timer_IF_DeInit(TIMERA2_BASE,TIMER_A);

}

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
    /* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void
DisplayBanner(char * AppName)
{

    UART_PRINT("\n\n\n\r");
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\t\t    CC3200 %s Application       \n\r", AppName);
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\n\n\n\r");
}


//Lee EXACTAMENDE size datos... para poder implementar el protocolo
int16_t ReadFrame(int16_t SockID, void *ptr,int16_t size)
{
    int16_t estado,i;

    i=size;
    while(i>0)
    {
        estado = sl_Recv(SockID, ptr,i, 0); //Esta funcion finaliza en cuanto llegan datos. Los tengo que ir acumulando hasta recibir la trama
        if (estado>0)
        {
            ptr+=estado;
            i-=estado;
        }
        else
        {
            return estado;
        }
    }
    return size;
}


//****************************************************************************
//
//! \brief Opening a TCP server side socket and receiving data
//!
//! This function opens a TCP socket in Listen mode and waits for an incoming
//!    TCP connection.
//! If a socket connection is established then the function will try to read
//!    TCP packets from the connected client and print them to the UART
//!
//! \param[in] port number on which the server will be listening on
//!
//! \return     0 on success, -1 on error.
//!
//! \note   This function will wait for an incoming connection till
//!                     one is established
//
//****************************************************************************
int BsdTcpServer(unsigned short usPort)
{
    SlSockAddrIn_t  sAddr;
    SlSockAddrIn_t  sLocalAddr;
    int             iCounter;
    int             iAddrSize;
    int             iSockID;
    int             iStatus;
    int             gpio_mode = 1;  // Variable para capturar el estado de los pines
    uint8_t         pwm_red = 0;
    uint8_t         pwm_green = 0;
    uint8_t         pwm_orange = 0;

    //    long            lLoopCount = 0;
    //    long            lNonBlocking = 1;
    //    int             iTestBufLen;

    TipoPaquete header;

    //filling the TCP server socket address
    sLocalAddr.sin_family = SL_AF_INET;
    sLocalAddr.sin_port = sl_Htons((unsigned short)usPort);
    sLocalAddr.sin_addr.s_addr = 0;

    // creating a TCP socket
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
    if( iSockID < 0 )
    {
        // error
        ASSERT_ON_ERROR(SOCKET_CREATE_ERROR);
    }

    iAddrSize = sizeof(SlSockAddrIn_t);

    // binding the TCP socket to the TCP server address
    iStatus = sl_Bind(iSockID, (SlSockAddr_t *)&sLocalAddr, iAddrSize);
    if( iStatus < 0 )
    {
        // error
        sl_Close(iSockID);
        ASSERT_ON_ERROR(BIND_ERROR);
    }

    // putting the socket for listening to the incoming TCP connection
    iStatus = sl_Listen(iSockID, 0);
    if( iStatus < 0 )
    {
        sl_Close(iSockID);
        ASSERT_ON_ERROR(LISTEN_ERROR);
    }

    // Opciones socket asincrono...
    //    // setting socket option to make the socket as non blocking
    //    iStatus = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_NONBLOCKING,
    //                            &lNonBlocking, sizeof(lNonBlocking));
    //    if( iStatus < 0 )
    //    {
    //        sl_Close(iSockID);
    //        ASSERT_ON_ERROR(SOCKET_OPT_ERROR);
    //    }

    iNewSockID = SL_EAGAIN;

    // waiting for an incoming TCP connection
    while( iNewSockID < 0 )
    {
        // accepts a connection form a TCP client, if there is any
        // otherwise returns SL_EAGAIN
        iNewSockID = sl_Accept(iSockID, ( struct SlSockAddr_t *)&sAddr,
                               (SlSocklen_t*)&iAddrSize);
        if( iNewSockID == SL_EAGAIN )
        {
            osi_Sleep(5); //Espera eficiente (RTOS)
        }
        else if( iNewSockID < 0 )
        {
            // error
            sl_Close(iNewSockID);
            sl_Close(iSockID);
            ASSERT_ON_ERROR(ACCEPT_ERROR);
        }
    }

    while (1)
    {
        //Recibe mientras pueda...
        iStatus = ReadFrame(iNewSockID,(void*)&header,sizeof(header));

        if( iStatus <= 0 )
        {
            // error. Finaliza la conexion
            sl_Close(iNewSockID);
            sl_Close(iSockID);
            ASSERT_ON_ERROR(RECV_ERROR);
        }
        else
        {	//Ha llegado un paquete con datos
            switch (header.comando)
            {
            case COMANDO_PING:
                //paquete ping, responde con el mismo comando
                xSemaphoreTake(semaforo_socket,portMAX_DELAY); //toma el mutex
                iStatus=sl_Send(iNewSockID,&header,sizeof(header),0);
                xSemaphoreGive(semaforo_socket); //devuelve el mutex
                if (iStatus<=0) return iStatus;
                break;
            case COMANDO_LED:
                //paquete LED. cambiamos el estado de los leds
                if (header.parametro.bits.bit0)
                {
                    GPIO_IF_LedOn(MCU_RED_LED_GPIO);
                }
                else
                {
                    GPIO_IF_LedOff(MCU_RED_LED_GPIO);

                }
                if (header.parametro.bits.bit1)
                {
                    GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
                }
                else
                {
                    GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
                }

                if (header.parametro.bits.bit2)
                {
                    GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
                }
                else
                {
                    GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
                }
                break;
            case COMANDO_SONDEA_BOTONES:
                //Comando sondea botones,miro el estado y
                //relleno el paquete de vuelta...
                if (async == 0)
                {
                    header.parametro.bits.bit0=GPIOPinRead(GPIOA1_BASE,GPIO_PIN_5)>>5;
                    header.parametro.bits.bit1=GPIOPinRead(GPIOA2_BASE,GPIO_PIN_6)>>6;
                    xSemaphoreTake(semaforo_socket,portMAX_DELAY); //toma el mutex
                    iStatus=sl_Send(iNewSockID,&header,sizeof(header),0);
                    xSemaphoreGive(semaforo_socket); //devuelve el mutex
                    if (iStatus<=0) return iStatus;
                }
                break;
            case COMANDO_MODO:
                switch (header.parametro.value)
                {
                case 0:
                    gpio_mode = 1;
                    PWM_IF_OutputDisable();
                    break;
                case 1:
                    gpio_mode = 0;
                    PWM_IF_OutputEnable();
                    break;
                case 2:
                    gpio_mode = 1;
                    MAP_PinTypeI2C(PIN_01, PIN_MODE_1); // Configure PIN_01 for I2C0 I2C_SCL
                    MAP_PinTypeI2C(PIN_02, PIN_MODE_1);  // Configure PIN_02 for I2C0 I2C_SDA
                    break;
                }
                break;
            case COMANDO_PWM_LEDS:
                // Protejo el caso en el que los pines estén modo GPIO y se modifique intensidad de los leds
                if (!gpio_mode)
                {
                    pwm_red = header.parametro.pwm.red;
                    pwm_green = header.parametro.pwm.green;
                    pwm_orange = header.parametro.pwm.orange;

                    // Mandamos intensidad del led con escala de 16b
                    PWM_IF_SetDutycycle((unsigned long)pwm_red*0xFF,
                                        (unsigned long)pwm_green*0xFF,
                                        (unsigned long)pwm_orange*0xFF);
                }
                break;
            case COMANDO_BOTONES_ASYNC:
                // Manejo de la notificacion asincrona del estado de los botones
                if (header.parametro.value)
                    async = 1;
                else
                    async = 0;
                break;
            case COMANDO_TEMP_START:
                // Envio comando lectura periodica de temperatura
                freqTemp = header.parametro.fvalue;
                read_temp = 1;
                xSemaphoreGive(semaforo_temp);
                break;
            case COMANDO_TEMP_STOP:
                // Envio comando stop lectura de temperatura
                read_temp = 0;
                break;
            case COMANDO_ACC_START:
                // Envio comando lectura periodica de acelerometro
                freqTemp = header.parametro.fvalue;
                read_acc = 1;
                xSemaphoreGive(semaforo_acc);
                break;
            case COMANDO_ACC_STOP:
                // Envio comando stop lectura de acelerometro
                read_acc = 0;
                break;
            default:
                //Comando hasta ahora no implementado. Respondo con COMANDO_RECHAZADO
                header.parametro.value=header.comando;
                header.comando=COMANDO_RECHAZADO;
                xSemaphoreTake(semaforo_socket,portMAX_DELAY); //toma el mutex
                iStatus=sl_Send(iNewSockID,&header,sizeof(header),0);
                xSemaphoreGive(semaforo_socket); //devuelve el mutex
                if (iStatus<=0) return iStatus;
                break;
            }
        }
    }

    return SUCCESS; //No deberia pasar nunca, por el momento...
}

//*****************************************************************************
//
//! Tarea que gestiona el envio de información asincrona cuando se pulsan los
//! botones. Primero captura interrupción de los botones y luego envía socket
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void AsyncPollingTask(void *pvParameters)
{
    int             iStatus;
    TipoPaquete     header;

    for ( ;; )
    {
        // Semáforo de acceso a los botones
        // No es necesario pero lo implemento por si en el futuro se añade
        // funcionalidad a los botones aparte de esta
        xSemaphoreTake(semaforo_boton,portMAX_DELAY);

        // Solo se ejecuta si esta el checkbox de envio asincrono marcado
        if (async)
        {
            // Formateamos la trama a enviar
            header.comando=COMANDO_SONDEA_BOTONES;
            header.parametro.bits.bit0=GPIOPinRead(GPIOA1_BASE,GPIO_PIN_5)>>5;
            header.parametro.bits.bit1=GPIOPinRead(GPIOA2_BASE,GPIO_PIN_6)>>6;

            // Mecanismo de exclusión mutua para el acceso al socket
            xSemaphoreTake(semaforo_socket,portMAX_DELAY); //toma el mutex
            iStatus=sl_Send(iNewSockID,&header,sizeof(header),0);
            xSemaphoreGive(semaforo_socket); //devuelve el mutex
            if (iStatus<=0) return iStatus;
        }
    }
}

//*****************************************************************************
//
//! Tarea que gestiona la lectura de temperatura
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void readTempTask(void *pvParameters)
{
    int             iStatus;
    TipoPaquete     header;
    float           tempIR;
    float           tempAmb;

    //Init Temperature Sensor
    if(TMP006DrvOpen() < 0)
    {
        UARTprintf("TMP006 open error\n");
    }

    for ( ;; )
    {
        // Semaforo para acceso a tarea de lectura temperatura
        xSemaphoreTake(semaforo_temp,portMAX_DELAY);

        while (read_temp == 1)
        {
            // Delay para muestreo segun frecuencia indicada
            vTaskDelay(freqTemp*configTICK_RATE_HZ);

            // Mutex de acceso i2c a la lectura
            xSemaphoreTake(semaforo_i2c,portMAX_DELAY);
            TMP006DrvGetTemp(&tempIR, &tempAmb);
            //devuelve el mutex
            xSemaphoreGive(semaforo_i2c);

            // Formateamos la trama a enviar
            header.comando=COMANDO_IRTEMP_DATA;
            header.parametro.fvalue=tempIR;

            // Mecanismo de exclusión mutua para el acceso al socket
            xSemaphoreTake(semaforo_socket,portMAX_DELAY); //toma el mutex
            iStatus=sl_Send(iNewSockID,&header,sizeof(header),0);
            xSemaphoreGive(semaforo_socket); //devuelve el mutex
            if (iStatus<=0) return iStatus;

            // Formateamos la trama a enviar
            header.comando=COMANDO_TEMP_DATA;
            header.parametro.fvalue=tempAmb;

            // Mecanismo de exclusión mutua para el acceso al socket
            xSemaphoreTake(semaforo_socket,portMAX_DELAY); //toma el mutex
            iStatus=sl_Send(iNewSockID,&header,sizeof(header),0);
            xSemaphoreGive(semaforo_socket); //devuelve el mutex
            if (iStatus<=0) return iStatus;
        }
    }
}

//*****************************************************************************
//
//! Tarea que gestiona la lectura del acelerómetro
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void readAccTask(void *pvParameters)
{
    int             iStatus;
    TipoPaquete     header;
    signed char     cAccXT2,cAccYT2,cAccZT2;

    //Init Temperature Sensor
    if(TMP006DrvOpen() < 0)
    {
        UARTprintf("TMP006 open error\n");
    }

    for ( ;; )
    {
        // Semaforo para acceso a tarea de lectura temperatura
        xSemaphoreTake(semaforo_acc,portMAX_DELAY);

        while (read_acc == 1)
        {
            // Delay para muestreo segun frecuencia indicada
            vTaskDelay(freqAcc*configTICK_RATE_HZ);

            // Mutex de acceso i2c a la lectura
            xSemaphoreTake(semaforo_i2c,portMAX_DELAY);
            BMA222ReadNew(&cAccXT2, &cAccYT2, &cAccZT2);
            //devuelve el mutex
            xSemaphoreGive(semaforo_i2c);

            // Formateamos la trama a enviar
            header.comando=COMANDO_ACC_DATA;
            header.parametro.acc.X= cAccXT2;
            header.parametro.acc.Y= cAccYT2;
            header.parametro.acc.Z= cAccZT2;

            // Mecanismo de exclusión mutua para el acceso al socket
            xSemaphoreTake(semaforo_socket,portMAX_DELAY); //toma el mutex
            iStatus=sl_Send(iNewSockID,&header,sizeof(header),0);
            xSemaphoreGive(semaforo_socket); //devuelve el mutex
            if (iStatus<=0) return iStatus;
        }
    }
}

//*****************************************************************************
//
//!Tarea que implementa el servidor TCP
//!    a broker
//!
//! \param  none
//!
//! Segun el modo:
//! -> Inicializa la red como AP o intenta conectarse a un AP como STATION
//! -> Inicializa el boton y sus callbacks...
//! -> Lanza la tarea del intï¿½rprete de comandos (commands.c)
//! -> Ejecuta el servidor TCP
//!
//! \return None
//!
//*****************************************************************************
void TCPServerTask(void *pvParameters)
{

    long lRetVal = -1;

    //
    // Configure LED
    //
    GPIO_IF_LedConfigure(LED1|LED2|LED3);

    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);

    //
    // Reset The state of the machine
    //
    Network_IF_ResetMCUStateMachine();

    //
    // Start the driver
    //
#ifdef STATION_MODE
    lRetVal = Network_IF_InitDriver(ROLE_STA);
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to start SimpleLink Device\n\r",lRetVal);
        LOOP_FOREVER();
    }

    // switch on Green LED to indicate Simplelink is properly up
    GPIO_IF_LedOn(MCU_ON_IND);

    // Start Timer to blink Red LED till AP connection
    LedTimerConfigNStart();		//Este timer es HARDWARE (solo se usa al principio, luego se libera) [TIMER2]

    // Initialize AP security params
    SecurityParams.Key = (signed char *)SECURITY_KEY;
    SecurityParams.KeyLen = strlen(SECURITY_KEY);
    SecurityParams.Type = SECURITY_TYPE;

    //
    // Connect to the Access Point
    //
    lRetVal = Network_IF_ConnectAP(SSID_NAME, SecurityParams);
    if(lRetVal < 0)
    {
        UART_PRINT("Connection to an AP failed\n\r");
        LOOP_FOREVER();
    }
#else
    lRetVal = Network_IF_InitDriver(ROLE_AP);
    if (lRetVal<0)
        LOOP_FOREVER();

    // switch on Green LED to indicate Simplelink is properly up
    GPIO_IF_LedOn(MCU_ON_IND);

    // Start Timer to blink Red LED
    LedTimerConfigNStart();	//Este timer es HARDWARE (solo se usa al principio, luego se libera [TIMER2])

    lRetVal=sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_SSID, strlen(SSID_NAME), SSID_NAME); //Configura el SSID en modo AP...
    if (lRetVal<0) ERR_PRINT( lRetVal);

    uint8_t secType=SECURITY_TYPE;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_AP_ID,
                         WLAN_AP_OPT_SECURITY_TYPE, 1,
                         (unsigned char *)&secType);
    if (lRetVal<0) ERR_PRINT( lRetVal);

    lRetVal = sl_WlanSet(SL_WLAN_CFG_AP_ID,
                         WLAN_AP_OPT_PASSWORD,
                         strlen((const char *)SECURITY_KEY),
                         (unsigned char *)SECURITY_KEY);
    if (lRetVal<0) ERR_PRINT( lRetVal);

    //Restart the network after changing access point name....
    lRetVal = sl_Stop(0xFF);
    if (lRetVal<0) ERR_PRINT( lRetVal);
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal<0) ERR_PRINT( lRetVal);

#endif
    //
    // Disable the LED blinking Timer as Device is connected to AP
    //
    LedTimerDeinitStop();	//Aqui queda liberado el timer hardware....

    //
    // Switch ON RED LED to indicate that Device acquired an IP
    //
    GPIO_IF_LedOn(MCU_IP_ALLOC_IND);

    osi_Sleep(1000); //Espera un poco

    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
    GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);

    //
    // Inicializa la biblioteca que gestiona el bus I2C. (movido desde main(...))
    //
    if(I2C_IF_Open(I2C_MASTER_MODE_FST) < 0)
    {
        while (1);
    }

    osi_Sleep(10); //Espera un poco

    //    Button_IF_Init(pushButtonInterruptHandler2,pushButtonInterruptHandler3); // Register Push Button Handlers

    //Esto tiene que ser realizado en una tarea. De momento lo pongo aqui
    //Init Accelerometer Sensor
    if(BMA222Open() < 0)
    {
        UARTprintf("BMA222 open error\n");
    }


    osi_Sleep(10); //Espera un poco

    //
    // Vuelve a poner los pines como salida GPIO
    // No deberia hacerlo mientras haya una transferencia I2C activa.
    MAP_PinTypeGPIO(PIN_01, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x4, GPIO_DIR_MODE_OUT);
    MAP_PinTypeGPIO(PIN_02, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x8, GPIO_DIR_MODE_OUT);

    //HAbilitamos los timers en modo PWM (pero NO habilitamos los pines como PWM)
    PWM_IF_Init(0);


    //
    // Display Application Banner
    //
    DisplayBanner("TCP_Server");

    //Lanza el interprete de comandos...(usa la API de la OSAL en lugar de la de FreeRTOS)
    lRetVal = osi_TaskCreate(vUARTTask,
                             (const signed char *)"CmdLine",
                             OSI_STACK_SIZE, NULL, 2, NULL );
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }


    //Este codigo lanza permanentemente el servidor TCP cada vez que se corta la conexion
    while(BsdTcpServer(5000)==RECV_ERROR)
    {

        UART_PRINT("Conexion cerrada. Relanzando el servidor\n\r");
        //Si se corta la conexion, vuelve a lanzarlo...

    }
    UART_PRINT("Fallo en el servidor\n\r");
    LOOP_FOREVER();
}


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
    long lRetVal = -1;
    //
    // Initialize the board configurations
    //
    BoardInit();

    //
    // Pinmux for UART
    //
    PinMuxConfig();

    //
    // Configuring UART
    //
    InitTerm();

    // Inicializamos los botones que harán de interrupciones
    ButtonsInit();

    //
    // Inicializa la biblioteca que gestiona el bus I2C.
    //
    if(I2C_IF_Open(I2C_MASTER_MODE_FST) < 0)
    {
        while (1);
    }

    //
    // Start the SimpleLink Host
    //
    lRetVal = VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY);
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }

    //
    // Start the TCP Server task
    //
    lRetVal = osi_TaskCreate(TCPServerTask,
                             (const signed char *)"TCPServ App",
                             OSI_STACK_SIZE, NULL, 2, NULL );

    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }

    // Tarea para sondeo asincrono de los botones. Le doy la misma prioridad que al servidor porque iba muy lento
    lRetVal = osi_TaskCreate(AsyncPollingTask,
                             (const signed char *)"AsyncPol",
                             OSI_STACK_SIZE, NULL, 2, NULL );
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }

    // Tarea para lectura de temperatura
    lRetVal = osi_TaskCreate(readTempTask,
                             (const signed char *)"readTemp",
                             OSI_STACK_SIZE, NULL, 2, NULL );
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }

    // Tarea para lectura de acelerometro
    lRetVal = osi_TaskCreate(readAccTask,
                             (const signed char *)"readAcc",
                             OSI_STACK_SIZE, NULL, 2, NULL );
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }

    // Semaforo para el acceso a botones por si fuese necesario más adelante
    semaforo_boton = xSemaphoreCreateBinary();
    if( semaforo_boton == NULL )
    {
        LOOP_FOREVER();
    }

    // Semaforo para el acceso a lectura de temperatura
    semaforo_temp = xSemaphoreCreateBinary();
    if( semaforo_temp == NULL )
    {
        LOOP_FOREVER();
    }

    // Semaforo para el acceso a lectura de acelerometro
    semaforo_acc = xSemaphoreCreateBinary();
    if( semaforo_temp == NULL )
    {
        LOOP_FOREVER();
    }
    // Mutex para gestionar el acceso a la conexion TCP
    semaforo_socket=xSemaphoreCreateMutex();
    if( semaforo_socket == NULL )
    {
        LOOP_FOREVER();
    }

    // Mutex para gestionar el acceso al bus i2c
    semaforo_i2c=xSemaphoreCreateMutex();
    if( semaforo_i2c == NULL )
    {
        LOOP_FOREVER();
    }

    //
    // Start the task scheduler
    //
    osi_start();
}


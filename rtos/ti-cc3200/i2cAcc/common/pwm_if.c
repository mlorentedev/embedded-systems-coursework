/*
 * pwm_if.c
 *
 *  Created on: 6 de feb. de 2017
 *      Author: cano
 */

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_ints.h"
#include "debug.h"
#include "interrupt.h"
#include "timer.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "pin.h"
#include "gpio.h"

//Inicializa el funcionamiento de los timers en modo PWM
void PWM_IF_Init(unsigned long ulValue)
{
    //
    // Initialize GPT A0 (in 32 bit mode) as periodic down counter.
    //
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA2, PRCM_RUN_MODE_CLK|PRCM_SLP_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA3, PRCM_RUN_MODE_CLK|PRCM_SLP_MODE_CLK);
    MAP_PRCMPeripheralReset(PRCM_TIMERA2);
    MAP_PRCMPeripheralReset(PRCM_TIMERA3);
    //
    TimerConfigure(TIMERA2_BASE,TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM);
    TimerConfigure(TIMERA3_BASE,TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM);
    TimerLoadSet(TIMERA2_BASE,TIMER_BOTH,0xFFFF);
    TimerMatchSet(TIMERA3_BASE,TIMER_BOTH,ulValue);
    TimerLoadSet(TIMERA3_BASE,TIMER_BOTH,0xFFFF);
    TimerMatchSet(TIMERA3_BASE,TIMER_BOTH,ulValue);

    TimerEnable(TIMERA2_BASE,TIMER_BOTH);
    TimerEnable(TIMERA3_BASE,TIMER_BOTH);
}

//Habilita la salida PWM
void PWM_IF_OutputEnable()
{

    //
    // Configure PIN_64 for GPIOOutput
    //
    MAP_PinTypeGPIO(PIN_64, PIN_MODE_3, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x2, GPIO_DIR_MODE_OUT);

    //
    // Configure PIN_01 for GPIOOutput
    //
    MAP_PinTypeGPIO(PIN_01, PIN_MODE_3, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x4, GPIO_DIR_MODE_OUT);

    //
    // Configure PIN_02 for GPIOOutput
    //
    MAP_PinTypeGPIO(PIN_02, PIN_MODE_3, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x8, GPIO_DIR_MODE_OUT);
}

//Deshabilita la salida PWM
void PWM_IF_OutputDisable()
{

    //
    // Configure PIN_64 for GPIOOutput
    //
    PinTypeGPIO(PIN_64, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x2, GPIO_DIR_MODE_OUT);

    //
    // Configure PIN_01 for GPIOOutput
    //
    MAP_PinTypeGPIO(PIN_01, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x4, GPIO_DIR_MODE_OUT);

    //
    // Configure PIN_02 for GPIOOutput
    //
    MAP_PinTypeGPIO(PIN_02, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x8, GPIO_DIR_MODE_OUT);
}

//Establece el nivel PWM
//Se le tiene que pasar un numero valorado entre 0 y 65535 (-->Cuidado!)
void PWM_IF_SetDutycycle(unsigned long red, unsigned long green, unsigned long orange)
{
	TimerMatchSet(TIMERA2_BASE,TIMER_B,(0xFFFE -red)); //PWM5
	TimerMatchSet(TIMERA3_BASE,TIMER_A,(0xFFFE -orange)); //PWM6
	TimerMatchSet(TIMERA3_BASE,TIMER_B,(0xFFFE -green)); //PWM7
}



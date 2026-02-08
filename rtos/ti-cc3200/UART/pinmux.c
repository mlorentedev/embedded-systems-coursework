// ******************************************************************************
// PROGRAMA	: pinmux.c
// TARGET	: CC3200
// DESCRIPCION	: Configuración de los pines de la UART y GPIO
// AUTOR	: José Manuel Cano García y Eva González Parada
// FECHA	: 10-12-16
// ESQUEMA  : Modo Activo-->Clock 80Mhz
// ******************************************************************************
#include "pinmux.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include "pin.h"
#include "rom.h"
#include "rom_map.h"
#include "gpio.h"
#include "prcm.h"

//*****************************************************************************

//*****************************************************************************
//
//! Configuración de los pines utilizados en el programa
//!
//! \Parámentros: ninguno
//!
//! \Retorno: ninguno
//
//*****************************************************************************


void PinMuxConfig(void)
{
    //
    // Habilitación de los relojes del GPIO.
	// Aunque el periférico GPIO no necesita reloj sí lo necesitan sus buses de interconexión
    //
    PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK|PRCM_SLP_MODE_CLK);

    //
    // Configuración del PIN_64 en modo GPIO y salida.
    // Es necesario programar el modo del pin entre  las distintas funcionalidades que tiene,
    // también es necesario la configuración eléctrica, así como especificar su dirección (entrada/salida)
    // El modo viene indicado en el Datasheet del CC3200 (página 20, tabla 3.1),  el modo 0 corresponde al GPIO
    //
    // El pin 64 corresponde al terminal 1 del puerto A1, que está conectado al LED Rojo de la placa CC3200
    // Esta última información se puede consultar los esquemáticos de la placa (Hojas 1 y 4) y en el documento "User´s Guide" de
    // la placa de desarrollo CC3200 SimpleLink Wi-Fi (página 14, tabla 9). En ambas documentaciones el pin 64 se identifica como GPIO_09
    //
   	PinTypeGPIO(PIN_64, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA1_BASE, GPIO_PIN_1, GPIO_DIR_MODE_OUT);

    //// PREGUNTA EC43
    // Configuración del pin correspondiente al LED Amarillo
    //
    PinTypeGPIO(PIN_01, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA1_BASE, GPIO_PIN_2, GPIO_DIR_MODE_OUT);

    //// PREGUNTA EC43
    // Configuración del pin correspondiente al LED verde
    //
    PinTypeGPIO(PIN_02, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA1_BASE, GPIO_PIN_3, GPIO_DIR_MODE_OUT);

    //
    //Habilitación de los relojes para la UART....
    //
    PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK|PRCM_SLP_MODE_CLK|PRCM_DSLP_MODE_CLK);

    //
    // Configuración del pin 55 correspondiente al transmisor de la UART0 UART0_TX
    // Datasheet del CC3200 (página 18, tabla 3.1), modo 3, correspondiente a UART
    //
    PinTypeUART(PIN_55, PIN_MODE_3);

    //
    // Configuración del pin 57 correspondiente al receptor de la UART0 UART0_RX
    // Datasheet del CC3200 (página 18, tabla 3.1), modo 3, correspondiente a UART
    //
    PinTypeUART(PIN_57, PIN_MODE_3);


}

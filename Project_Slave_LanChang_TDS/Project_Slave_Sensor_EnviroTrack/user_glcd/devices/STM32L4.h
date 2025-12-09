/**
	\file STM32L4.h
	\author Andy Gock (modified for STM32F4xx by Moreto)
	\brief Functions specific to STM32 F4 ARM Cortex-M4 devices.
 */

/*
	Copyright (c) 2012, Andy Gock

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
		* Redistributions of source code must retain the above copyright
		  notice, this list of conditions and the following disclaimer.
		* Redistributions in binary form must reproduce the above copyright
		  notice, this list of conditions and the following disclaimer in the
		  documentation and/or other materials provided with the distribution.
		* Neither the name of Andy Gock nor the
		  names of its contributors may be used to endorse or promote products
		  derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL ANDY GOCK BE LIABLE FOR ANY
	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef STM32L4XX_H_
#define STM32L4XX_H_

#if defined(GLCD_DEVICE_STM32L4XX)

#include <stm32l4xx.h>


#define CONTROLLER_SPI_SS_PORT  LCD_CS_GPIO_Port
#define CONTROLLER_SPI_SS_PIN   LCD_CS_Pin

#define CONTROLLER_SPI_DC_PORT  LCD_A0_GPIO_Port
#define CONTROLLER_SPI_DC_PIN   LCD_A0_Pin

#define CONTROLLER_SPI_RST_PORT LCD_RST_GPIO_Port
#define CONTROLLER_SPI_RST_PIN  LCD_RST_Pin


#define GPIO_ResetBits(port, pin)   HAL_GPIO_WritePin (port, pin, GPIO_PIN_RESET)
#define GPIO_SetBits(port, pin)     HAL_GPIO_WritePin (port, pin, GPIO_PIN_SET)

#define GLCD_SELECT()     GPIO_ResetBits(CONTROLLER_SPI_SS_PORT,CONTROLLER_SPI_SS_PIN)
#define GLCD_DESELECT()   GPIO_SetBits(CONTROLLER_SPI_SS_PORT,CONTROLLER_SPI_SS_PIN)
#define GLCD_A0_LOW()     GPIO_ResetBits(CONTROLLER_SPI_DC_PORT,CONTROLLER_SPI_DC_PIN)
#define GLCD_A0_HIGH()    GPIO_SetBits(CONTROLLER_SPI_DC_PORT,CONTROLLER_SPI_DC_PIN)
#define GLCD_RESET_LOW()  GPIO_ResetBits(CONTROLLER_SPI_RST_PORT,CONTROLLER_SPI_RST_PIN)
#define GLCD_RESET_HIGH() GPIO_SetBits(CONTROLLER_SPI_RST_PORT,CONTROLLER_SPI_RST_PIN)



/* Function prototypes: */
void glcd_enable_backlight(FunctionalState state);
#ifdef USE_TIMER_PWM
void glcd_change_backlight(uint8_t value);
#endif


#else
	#error "Controller not supported by STM32F4XX"
#endif


/*================ Function ===============*/

void glcd_init(void);
void glcd_spi_write(uint8_t c);
void glcd_reset(void);


#endif


/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */


/* USER CODE BEGIN Includes */
#define JULIAN_DATE_BASE    2440588

#define FLASH_KEY1 ((uint32_t)0x45670123)
#define FLASH_KEY2 ((uint32_t)0xCDEF89AB)
#define ST_ADR (uint32_t)(0x08007F00)  //????? ????????? ???????? ???? ??????
#define ST_ADRp (ST_ADR+8) ////((uint32_t)0x08008800)  //????? ????????? ???????? ???? ??????

#define RTC_TR_MASK 0x007F7F7F
#define RTC_DR_MASK 0x00FFFF3F
#define RTC_SET 0x01

#define NUMBER_ON 2
#define NUMBER_FULL 10

#define NUMBER_ON1 4
#define NUMBER_FULL1 28

#define SPEEDPLUS 8400

#define K1 1

const uint8_t state_tabl[43][2] ={
		{0x00,0x00}, //0x00
		{0x02,0x11}, //0x01
		{0x07,0x12}, //0x02
		{0x04,0x13}, //0x03
		{0x05,0x14}, //0x04
		{0x06,0x15}, //0x05
		{0x08,0x16}, //0x06
		{0x03,0x17}, //0x07
		{0x0A,0x18}, //0x08
		{0x01,0x01}, //0x09
		{0x01,0x1A}, //0x0A
		{0x07,0x07}, //0x0B
		{0x07,0x07}, //0x0C
		{0x07,0x07}, //0x0D
		{0x07,0x07}, //0x0E
		{0x07,0x07}, //0x0F
		{0x10,0x01}, //0x10
		{0x07,0x01}, //0x11
		{0x22,0x02}, //0x12
		{0x23,0x03}, //0x13
		{0x24,0x04}, //0x14
		{0x15,0x05}, //0x15
		{0x26,0x06}, //0x16
		{0x27,0x07}, //0x17
		{0x18,0x18}, //0x18
		{0x07,0x07}, //0x19
		{0x2A,0x0A}, //0x1A
		{0x07,0x07}, //0x1B
		{0x07,0x07}, //0x1C
		{0x07,0x07}, //0x1D
		{0x07,0x07}, //0x1E
		{0x07,0x07}, //0x1F
		{0x07,0x07}, //0x20
		{0x07,0x07}, //0x21
		{0x12,0x02}, //0x22
		{0x13,0x03}, //0x23
		{0x14,0x04}, //0x24
		{0x10,0x10}, //0x25
		{0x16,0x06}, //0x26
		{0x17,0x07}, //0x27
		{0x07,0x07}, //0x28
		{0x07,0x07}, //0x29
		{0x1A,0x0A}, //0x2A
};

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim22;
TIM_HandleTypeDef htim21;

const uint8_t CharSet[256][6] ={
//{00h}
	 {0x00,0x00,0x00,0x00,0x00,0x00},
     {0x00,0x00,0x00,0x00,0x00,0x00},
	 {0x00,0x00,0x00,0x00,0x00,0x00},
     {0x00,0x00,0x00,0x00,0x00,0x00},
	 {0x00,0x00,0x00,0x00,0x00,0x00},
     {0x00,0x00,0x00,0x00,0x00,0x00},
	 {0x00,0x00,0x00,0x00,0x00,0x00},
     {0x00,0x00,0x00,0x00,0x00,0x00},
	 {0x00,0x00,0x00,0x00,0x00,0x00},
     {0x00,0x00,0x00,0x00,0x00,0x00},
	 {0x00,0x00,0x00,0x00,0x00,0x00},
     {0x00,0x00,0x00,0x00,0x00,0x00},
	 {0x00,0x00,0x00,0x00,0x00,0x00},
     {0x00,0x00,0x00,0x00,0x00,0x00},
	 {0x00,0x00,0x00,0x00,0x00,0x00},
     {0x00,0x00,0x00,0x00,0x00,0x00},
//{10h-16d}
	 {0x00,0x00,0x00,0x00,0x00,0x00},
     {0x00,0x00,0x00,0x00,0x00,0x00},
	 {0x00,0x00,0x00,0x00,0x00,0x00},
     {0x00,0x00,0x00,0x00,0x00,0x00},
	 {0x00,0x00,0x00,0x00,0x00,0x00},
     {0x00,0x00,0x00,0x00,0x00,0x00},
	 {0x00,0x00,0x00,0x00,0x00,0x00},
     {0x00,0x00,0x00,0x00,0x00,0x00},
	 {0x00,0x00,0x00,0x00,0x00,0x00},
     {0x00,0x00,0x00,0x00,0x00,0x00},
	 {0x00,0x00,0x00,0x00,0x00,0x00},
     {0x00,0x00,0x00,0x00,0x00,0x00},
	 {0x00,0x00,0x00,0x00,0x00,0x00},
     {0x00,0x00,0x00,0x00,0x00,0x00},
	 {0x00,0x00,0x00,0x00,0x00,0x00},
     {0x00,0x00,0x00,0x00,0x00,0x00},
//{20h-32d}

     {0x00,0x00,0x00,0x00,0x00,0x00},//' '
     {0x00,0x07,0x00,0x07,0x00,0x00},
     {0x14,0x7f,0x14,0x7f,0x14,0x00},
     {0x24,0x6a,0x2a,0x2b,0x12,0x00},
     {0x23,0x13,0x08,0x64,0x62,0x00}, //%
     {0x36,0x49,0x55,0x22,0x50,0x00},
     {0x00,0x05,0x03,0x00,0x00,0x00},
     {0x00,0x1c,0x22,0x41,0x00,0x00},
     {0x00,0x41,0x22,0x1c,0x00,0x00},
     {0x14,0x08,0x3e,0x08,0x14,0x00},
     {0x08,0x08,0x3e,0x08,0x08,0x00},
     {0x00,0x50,0x30,0x00,0x00,0x00},
     {0x00,0x60,0x60,0x20,0x00,0x00},//,
     {0x00,0x60,0x60,0x00,0x00,0x00},
     {0x00,0x60,0x60,0x00,0x00,0x00},//.
     {0x00,0x00,0x00,0x00,0x00,0x00},//" "

//{30h}
     {0x3e,0x51,0x49,0x45,0x3e,00}, //{0}
     {0x04,0x42,0x7f,0x40,0x00,00}, //{1}
     {0x42,0x61,0x51,0x49,0x46,00}, //{2}
     {0x21,0x41,0x45,0x4b,0x31,00},// {3}
     {0x18,0x14,0x12,0x7f,0x10,00},// {4}
     {0x27,0x45,0x45,0x45,0x39,00},// {5}
     {0x3c,0x4a,0x49,0x49,0x30,00},// {6}
     {0x01,0x71,0x09,0x05,0x03,00},// {7}
     {0x36,0x49,0x49,0x49,0x36,00},// {8}
     {0x06,0x49,0x49,0x49,0x3e,00},// {9}
     {0x00,0x36,0x36,0x00,0x00,00},// {:}
     {0x00,0x56,0x36,0x00,0x00,00},// {;}
     {0x08,0x14,0x22,0x41,0x00,00},// {<}
     {0x14,0x14,0x14,0x14,0x14,00},// {=}
     {0x41,0x22,0x14,0x08,0x00,00},// {>}
     {0x02,0x01,0x51,0x09,0x06,00},// {?}
//{40h}
     {0x32,0x49,0x79,0x41,0x3e,00},// {@}
     {0x7e,0x11,0x11,0x11,0x7e,00},// {A}
     {0x7f,0x49,0x49,0x49,0x36,00},// {B}
     {0x3e,0x41,0x41,0x41,0x22,00},// {C}
     {0x7f,0x41,0x41,0x22,0x1c,00},// {D}
     {0x7f,0x49,0x49,0x49,0x49,00},// {E}
     {0x7f,0x09,0x09,0x09,0x01,00},// {F}
     {0x3e,0x41,0x49,0x49,0x72,00},// {G}
     {0x7f,0x08,0x08,0x08,0x7f,00},// {H}
     {0x00,0x41,0x7f,0x41,0x00,00},// {I}
     {0x20,0x40,0x41,0x3f,0x01,00},// {J}
     {0x7f,0x08,0x14,0x22,0x41,00},// {K}
     {0x7f,0x40,0x40,0x40,0x40,00},// {L}
     {0x7f,0x02,0x0c,0x02,0x7f,00},// {M}
     {0x7f,0x04,0x08,0x10,0x7f,00},// {N}
     {0x3e,0x41,0x41,0x41,0x3e,00},// {O}
//{50h}
     {0x7f,0x09,0x09,0x09,0x06,00},// {P}
     {0x3e,0x41,0x51,0x21,0x5e,00},// {Q}
     {0x7f,0x09,0x19,0x29,0x46,00},// {R}
     {0x46,0x49,0x49,0x49,0x31,00},// {S}
     {0x01,0x01,0x7f,0x01,0x01,00},// {T}
     {0x3f,0x40,0x40,0x40,0x3f,00},// {U}
     {0x1f,0x20,0x40,0x20,0x1f,00},// {V}
     {0x3f,0x40,0x38,0x40,0x3f,00},// {W}
     {0x63,0x14,0x08,0x14,0x63,00},// {X}
     {0x07,0x08,0x70,0x08,0x07,00},// {Y}
     {0x61,0x51,0x49,0x45,0x43,00},// {Z}
     {0x00,0x7f,0x41,0x41,0x00,00},// {[}
     {0x02,0x04,0x08,0x10,0x20,00},// {\}
     {0x00,0x41,0x41,0x7f,0x00,00},// {]}
     {0x04,0x02,0x01,0x02,0x04,00},// {^}
     {0x40,0x40,0x40,0x40,0x40,40},// {-}
//{60h}
     {0x00,0x01,0x02,0x04,0x00,00},// {}
     {0x20,0x54,0x54,0x54,0x78,00},// {a}
     {0x7f,0x48,0x44,0x44,0x38,00},// {b}
     {0x38,0x44,0x44,0x44,0x20,00},// {c}
     {0x38,0x44,0x44,0x48,0x7f,00},// {d}
     {0x38,0x54,0x54,0x54,0x18,00},// {e}
     {0x08,0x7f,0x01,0x01,0x02,00},// {f}
     {0x0c,0x52,0x52,0x52,0x3e,00},// {g}
     {0x7f,0x08,0x04,0x04,0x78,00},// {h}
     {0x00,0x44,0x7d,0x40,0x00,00},// {i}
     {0x20,0x20,0x44,0x3d,0x00,00},// {j}
     {0x7f,0x10,0x28,0x44,0x00,00},// {k}
     {0x00,0x00,0x7f,0x40,0x20,00},// {l}
     {0x7c,0x04,0x18,0x04,0x78,00},// {m}
     {0x7c,0x08,0x04,0x04,0x78,00},// {n}
     {0x38,0x44,0x44,0x44,0x38,00},// {o}

//{70h}

 	 {0x7c,0x14,0x14,0x14,0x08,00},//{p}
 	 {0x08,0x14,0x14,0x18,0x7c,00},//{q}
 	 {0x7c,0x08,0x04,0x04,0x08,00},//{r}
 	 {0x48,0x54,0x54,0x54,0x20,00},//{s}
  	 {0x04,0x3f,0x44,0x40,0x20,00},//{t}
     {0x3c,0x40,0x40,0x20,0x7c,00},//{u}
  	 {0x1c,0x20,0x40,0x20,0x1c,00},//{v}
 	 {0x3c,0x40,0x30,0x40,0x3c,00},//{w}
  	 {0x44,0x28,0x10,0x28,0x44,00},//{x}
 	 {0x0c,0x50,0x50,0x50,0x3c,00},//{y}
     {0x44,0x64,0x54,0x4c,0x44,00},//{z}
 	 {0x1f,0x00,0x1f,0x11,0x1f,00},//{10}
 	 {0x1f,0x00,0x1d,0x15,0x17,00},//{12}
	 {0x1f,0x00,0x17,0x15,0x1d,00},//{15}
 	 {0x38,0x04,0x08,0x10,0x0e,00},
 	 {0x2a,0x2a,0x7f,0x2a,0x2a,00},

//{80h}


	 {0x7c,0x12,0x11,0x12,0x7c,00},// {?}
	 {0x7f,0x49,0x49,0x49,0x31,00},// {?}
	 {0x7f,0x49,0x49,0x49,0x36,00},// {?}
	 {0x7f,0x01,0x01,0x01,0x03,00},// {?}
	 {0x60,0x3e,0x21,0x3f,0x60,00},// {?}
	 {0x7f,0x49,0x49,0x49,0x41,00},// {?}
	 {0x77,0x08,0x7f,0x08,0x77,00},// {?}
	 {0x22,0x49,0x49,0x49,0x36,00},// {?}
	 {0x7f,0x20,0x10,0x08,0x7f,00},// {?}
	 {0x7c,0x20,0x12,0x09,0x7c,00},// {?}
	 {0x7f,0x08,0x14,0x22,0x41,00},// {?}
     {0x40,0x3e,0x01,0x01,0x7f,00},
     {0x7f,0x02,0x0c,0x02,0x7f,00},
     {0x7f,0x08,0x08,0x08,0x7f,00},
     {0x3e,0x41,0x41,0x41,0x3e,00},
     {0x7f,0x01,0x01,0x01,0x7f,00},

//{90h}

     {0x7c,0x55,0x54,0x55,0x44,00},
     {0x38,0x55,0x54,0x55,0x18,00},
     {0x20,0x10,0x08,0x04,0x02,00},
     {0x02,0x04,0x08,0x10,0x20,00},
     {0x30,0x18,0x0c,0x06,0x03,00},
     {0x06,0x0c,0x18,0x30,0x60,00},
     {0x08,0x49,0x2a,0x1c,0x08,00},
     {0x08,0x1c,0x2a,0x49,0x08,00},
     {0x10,0x20,0x7f,0x20,0x10,00},
     {0x04,0x02,0x7f,0x02,0x04,00},
     {0x08,0x08,0x6b,0x08,0x08,00},
     {0x22,0x22,0x27,0x22,0x22,00},
     {0x78,0x10,0x20,0x7b,0x03,00},
     {0x5d,0x36,0x14,0x36,0x5d,00},
     {0x00,0x07,0x05,0x07,0x00,00},
     {0x30,0x48,0x48,0x03,0x03,00},

//{A0h}
//{- ?????????????}
  	 {0x08,0x0f,0x08,0x0f,0x08,0x08},
  	 {0x14,0x14,0xf4,0x14,0x14,0x14},
  	 {0x08,0xf8,0x08,0xf8,0x08,0x08},
  	 {0x00,0x0f,0x08,0x0f,0x08,0x08},
  	 {0x00,0x00,0x1f,0x14,0x14,0x14},
  	 {0x00,0x00,0xfc,0x14,0x14,0x14},
  	 {0x00,0xf8,0x08,0xf8,0x08,0x08},
  	 {0x08,0xff,0x08,0xff,0x08,0x08},
  	 {0x7c,0x55,0x54,0x45,0x00,0x00},// ?
  	 {0x14,0x14,0xff,0x14,0x14,0x14},
  	 {0x08,0x08,0x0f,0x00,0x00,0x00},
  	 {0x00,0x00,0xf8,0x08,0x08,0x08},
  	 {0xe7,0xe7,0xe7,0xe7,0xe7,0xe7},
  	 {0xe1,0xe1,0xe1,0xe1,0xe1,0xe1},
  	 {0xff,0xff,0xff,0x00,0x00,0x00},
     {0x00,0x00,0x00,0xff,0xff,0xff},
//   {0x07,0x07,0x07,0x07,0x07,0x07},

//{B0h}
  	 {0x49,0x00,0x49,0x00,0x49,00},
  	 {0x6b,0x14,0x6b,0x14,0x6b,00},
     {0x7f,0x49,0x7f,0x49,0x7f,00},
     {0x00,0x00,0xff,0x00,0x00,00},
     {0x08,0x08,0xff,0x00,0x00,00},
     {0x14,0x14,0xff,0x00,0x00,00},
     {0x08,0xff,0x00,0xff,0x00,00},
     {0x08,0xf8,0x08,0xf8,0x00,00},
     {0x38,0x55,0x54,0x55,0x58,00},//? --// ($14,$14,$fc,$00,00,00),
     {0x14,0xf7,0x00,0xff,00,00},
     {0x00,0xff,0x00,0xff,00,00},
     {0x14,0xf4,0x04,0xfc,00,00},
     {0x14,0x17,0x10,0x1f,00,00},
     {0x10,0x1f,0x10,0x1f,00,00},
     {0x14,0x14,0x1f,0x00,00,00},
     {0x08,0x08,0xf8,0x00,00,00},

//{C0h }

	 {0x7c,0x12,0x11,0x12,0x7c,00},// {?}
	 {0x7f,0x49,0x49,0x49,0x31,00},// {?}
	 {0x7f,0x49,0x49,0x49,0x36,00},// {?}
	 {0x7f,0x01,0x01,0x01,0x03,00},// {?}
	 {0x60,0x3e,0x21,0x3f,0x60,00},// {?}
	 {0x7f,0x49,0x49,0x49,0x41,00},// {?}
	 {0x77,0x08,0x7f,0x08,0x77,00},// {?}
	 {0x22,0x49,0x49,0x49,0x36,00},// {?}
	 {0x7f,0x20,0x10,0x08,0x7f,00},// {?}
	 {0x7c,0x20,0x12,0x09,0x7c,00},// {?}
	 {0x7f,0x08,0x14,0x22,0x41,00},// {?}
	 {0x40,0x3e,0x01,0x01,0x7f,00},// {?}
	 {0x7f,0x02,0x0c,0x02,0x7f,00},// {?}
	 {0x7f,0x08,0x08,0x08,0x7f,00},// {?}
	 {0x3e,0x41,0x41,0x41,0x3e,00},// {?}
	 {0x7f,0x01,0x01,0x01,0x7f,00},// {?}

//{D0h }


	{0x7f,0x09,0x09,0x09,0x06,00},// {?}
	{0x3e,0x41,0x41,0x41,0x22,00},// {?}
	{0x01,0x01,0x7f,0x01,0x01,00},// {?}
	{0x2f,0x50,0x50,0x50,0x3f,00},// {?}
	{0x0c,0x12,0x7f,0x12,0x0c,00},// {?}
	{0x63,0x14,0x08,0x14,0x63,00},// {?}
	{0x3f,0x20,0x20,0x3f,0x40,00},// {?}
	{0x0f,0x10,0x10,0x10,0x7f,00},// {?}
	{0x7f,0x40,0x7e,0x40,0x7f,00},// {?}
	{0x3f,0x20,0x3e,0x20,0x7f,00},// {?}
	{0x01,0x7f,0x48,0x48,0x30,00},// {?}
	{0x7f,0x48,0x30,0x00,0x7f,00},// {?}
	{0x7f,0x48,0x48,0x48,0x30,00},// {?}
	{0x22,0x41,0x49,0x49,0x3e,00},// {?}
	{0x7f,0x08,0x3e,0x41,0x3e,00},// {?}
	{0x46,0x29,0x19,0x09,0x7f,00},// {?}

 //{E0h} { ?????????????}
 //    {0x00,0x00,0x0f,0x08,0x08,0x08},
 //    {0x08,0x08,0x0f,0x08,0x08,0x08},
 //    {0x08,0x08,0xf8,0x08,0x08,0x08},
 //    {0x00,0x00,0xff,0x08,0x08,0x08},
 //    {0x08,0x08,0x08,0x08,0x08,0x08},
 //    {0x08,0x08,0xff,0x08,0x08,0x08},
 //    {0x00,0x00,0xff,0x14,0x14,0x14},
 //    {0x00,0xff,0x00,0xff,0x08,0x08},
 //    {0x00,0x1f,0x10,0x17,0x14,0x14},
 //    {0x00,0xfc,0x04,0xf4,0x14,0x14},
 //    {0x14,0x17,0x10,0x17,0x14,0x14},
 //    {0x14,0xf4,0x04,0xf4,0x14,0x14},
 //    {0x00,0xff,0x00,0xf7,0x14,0x14},
 //    {0x14,0x14,0x14,0x14,0x14,0x14},
 //    {0x14,0xf7,0x00,0xf7,0x14,0x14},
 //    {0x14,0x14,0x17,0x14,0x14,0x14},

//{E0h}


     {0x20,0x54,0x54,0x38,0x40,00},// {?}
     {0x38,0x54,0x54,0x54,0x22,00},// {?}
  	 {0x7c,0x54,0x54,0x54,0x28,00},// {?}
     {0x7c,0x04,0x04,0x04,0x04,00},// {?}
     {0x40,0x3c,0x24,0x3c,0x40,00},// {?}
     {0x38,0x54,0x54,0x54,0x58,00},// {?}
     {0x6c,0x10,0x7c,0x10,0x6c,00},// {?}
	 {0x28,0x44,0x54,0x54,0x28,00},// {?}
  	 {0x7c,0x20,0x10,0x08,0x7c,00},//{?}
  	 {0x7c,0x20,0x12,0x09,0x7c,00},//{?}
  	 {0x7c,0x10,0x10,0x28,0x44,00},//{?}
  	 {0x40,0x38,0x04,0x04,0x7c,00},//{?}
  	 {0x7c,0x08,0x10,0x08,0x7c,00},//{?}
  	 {0x7c,0x10,0x10,0x10,0x7c,00},//{?}
  	 {0x38,0x44,0x44,0x44,0x38,00},//{?}
  	 {0x7c,0x04,0x04,0x04,0x7c,00},//{?}

//{F0h}
  {0x7c,0x14,0x14,0x14,0x08,00},//{?}
  {0x38,0x44,0x44,0x44,0x28,00},//{?}
  {0x04,0x04,0x7c,0x04,0x04,00},//{?}
  {0x0c,0x50,0x50,0x50,0x3c,00},//{?}
  {0x38,0x28,0x7c,0x28,0x38,00},//{?}
  {0x44,0x28,0x10,0x28,0x44,00},//{?}
  {0x3c,0x20,0x20,0x3c,0x40,00},//{?}
  {0x0c,0x10,0x10,0x10,0x7c,00},//{?}
  {0x7c,0x40,0x78,0x40,0x7c,00},//{?}
  {0x3c,0x20,0x38,0x20,0x7c,00},//{?}
  {0x04,0x7c,0x50,0x50,0x20,00},//{?}
  {0x7c,0x50,0x20,0x00,0x7c,00},//{?}
  {0x7c,0x50,0x50,0x50,0x20,00},//{?}
  {0x44,0x44,0x54,0x54,0x38,00},//{?}
  {0x7c,0x10,0x38,0x44,0x38,00},//{?}
  {0x08,0x54,0x34,0x14,0x7c,00} //{?}
};

const unsigned char auchCRCHi[] =
{
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
};

const unsigned char auchCRCLo[] =
{
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
};




uint8_t reg_MB[30];
uint8_t reg_MB2[5];

const uint32_t USART_const [9] = {9600,4800,9600,14400,19200,38400,56000,57600,115200};
//const uint8_t  TIMER_const [9] = {4,8,4,3,2,2,2,2,2};
const uint8_t  TIMER_const [9] = {37,73,37,25,19,10,7,7,4};

uint16_t CRCCod;

uint8_t DEEPSLP=0;
uint8_t DEEPWK=0;

volatile uint8_t FlagModbGet=0; 
volatile uint8_t NeedChangeSpeed=0;

unsigned char res_buffer[20];					// приемный буфер
unsigned char write_buffer[60];					// буфер для передачи
volatile unsigned char res_wr_index;

uint8_t uu=0;




typedef struct
{
  uint8_t R_Hours;
  uint8_t R_Minutes;
  uint8_t R_Seconds;
}R_TimeTypeDef;


volatile uint8_t reScreen=0;


//uint16_t Korm[99];
R_TimeTypeDef Korm[99];
uint32_t PauseT,PT1;

uint32_t Korm_Now,Korm_St,Korm_St1,Korm_Fn,Korm_Fn1;

uint16_t DayUp;

int ik;

uint8_t ADC_b,ADC_old,ADC_cnt=0;
volatile uint8_t ADC_frst=1;
volatile uint8_t ADC_ready=0;
volatile uint8_t ADC_dr=0;

volatile uint8_t FirstStart=0;

volatile uint8_t rele=0x00;
volatile uint8_t SYN_OUT = 0x80;

uint8_t SecondCounter = 0;
uint8_t scr_st = 0;
uint8_t state = 0x09;
//uint8_t scr_st = 0;
uint8_t but_p,but_plus,but_minus,reset_t,double_but = 0;
uint8_t wait = 0;
volatile uint8_t sc_up = 1;
//volatile uint16_t m_timer = 0;
volatile uint8_t prev_min = 61;
volatile uint8_t prev_sec = 61;
//volatile uint8_t GlobalSec;

//volatile uint8_t adc_tim = 0;
volatile uint8_t MAYSLEEP = 0;
volatile uint8_t NOTSLEEP = 1;
volatile uint8_t LOWPOWER = 0;
volatile uint8_t POWERFAIL = 0;
volatile uint8_t blink = 0;
volatile uint8_t blon = 0;
volatile uint8_t start_blink = 0;
volatile uint8_t two_sec = 0;
volatile uint8_t twenty_sec = 0;
uint8_t ts_change=0;
volatile uint8_t power_not = 0;
volatile uint8_t MBpower_not = 0;
volatile uint8_t power = 0;
volatile uint8_t pow_test = 0;
volatile uint8_t power_ch = 0;
uint8_t button = 0;
volatile uint8_t FLSH_WRT_N=0;
volatile uint8_t TIME_CH=0;
volatile uint8_t VAR_CH=0;
volatile uint8_t JUST_FIN=0;
volatile uint8_t POWER_KORM=0;

volatile uint8_t flag_WT=1;

volatile uint8_t flag_FT=1;

volatile uint8_t NXT_TIME=100;

uint32_t FLASH_BUF;

RTC_TimeTypeDef RTC_DateTime;
RTC_DateTypeDef RTC_Date1;

#define MBPower reg_MB[0]

#define MBKorm reg_MB[1]
#define MBTest reg_MB[2]

#define MBMotorMin reg_MB[3]
#define MBMotorSec reg_MB[4]
#define MBError    reg_MB[5]

#define GlobalHr reg_MB[6]
#define GlobalMin reg_MB[7]

#define StartHr reg_MB[8]
#define StartMin reg_MB[9]

#define StopHr reg_MB[10]
#define StopMin reg_MB[11]

#define CntMin reg_MB[12]
#define CntSec reg_MB[13]

#define Per reg_MB[14]
#define CntFood reg_MB[15]

#define MBAdr reg_MB[16]
#define MBSpeed reg_MB[17]

#define MBCustomMin reg_MB[18]
#define MBCustomSec reg_MB[19]

const uint8_t MaxS[20]={0,0,0,0,0,0,24,60,24,60,24,60,100,60,100,100,247,9,100,60};

volatile uint8_t customTest = 0;
//uint8_t GlobalHr, GlobalMin = 0;
//uint8_t StartMin, StartHr = 0;
//uint8_t StopMin, StopHr = 0;
//uint8_t CntMin, CntSec = 0;

uint8_t CntMin1, CntSec1 = 0;

uint8_t UnHr, UnMin, UnSec = 0;
uint8_t UnMin1, UnSec1 = 0;

//uint8_t Per=0;
//uint8_t CntFood=0;

volatile unsigned char res_wr_index;

uint16_t MOTOR_TIME=0;

volatile uint16_t MOTOR_TIME1=0;

uint32_t Tick1,Tick2;

volatile uint8_t first_button_press=1;


volatile uint8_t pauseOff=0;

volatile uint8_t flag20=0;
volatile uint8_t sec20=0;
volatile uint8_t sec20cnt=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC_Init(void);
static void MX_IWDG_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static void MX_TIM22_Init(uint8_t bd)
{
	__HAL_RCC_TIM22_CLK_ENABLE();
  htim22.Instance = TIM22;
  htim22.Init.Prescaler = 800-1;
  htim22.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim22.Init.Period = TIMER_const[bd]; 
  htim22.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim22) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	
	TIM22->PSC = 800 - 1; 
	TIM22->ARR = TIMER_const[bd]; 
	TIM22->DIER |= TIM_DIER_UIE; 
	TIM22->CR1 |= TIM_CR1_OPM;
	NVIC_SetPriority(TIM22_IRQn, 0); 
	NVIC_EnableIRQ(TIM22_IRQn);
}


void timer1_init(void)  //2 ???????
{
	__HAL_RCC_TIM21_CLK_ENABLE();
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 8000-1;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 1500;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM21,ENABLE);
	TIM21->PSC = 8000 - 1; // ??????????? ???????? ??? ?????? ????? 1000 ??? ? ???????
	TIM21->ARR = 1500 ; // ???? ?????????? ????????? ??? ? 2 ???????
	TIM21->DIER |= TIM_DIER_UIE; //????????? ?????????? ?? ???????
	TIM21->CR1 |= TIM_CR1_OPM;
	TIM21->CR1 |= TIM_CR1_CEN; // ?????? ??????!
	NVIC_EnableIRQ(TIM21_IRQn);
}

void timer2_init(void)  //2 ???????
{
	__HAL_RCC_TIM2_CLK_ENABLE();
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM2,ENABLE);
	TIM2->PSC = 8000 - 1; // ??????????? ???????? ??? ?????? ????? 1000 ??? ? ???????
	TIM2->ARR = 1000; // ???? ?????????? ????????? ??? ? 2 ???????
	TIM2->DIER |= TIM_DIER_UIE; //????????? ?????????? ?? ???????
	TIM2->CR1 |= TIM_CR1_OPM;
	TIM2->CR1 |= TIM_CR1_CEN; // ?????? ??????!
	NVIC_EnableIRQ(TIM2_IRQn);
}

static void USART2_ReInit(uint8_t bd)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = USART_const[bd];
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

unsigned int CRC16 (unsigned char *pucFrame, unsigned int usLen)
													// pucFrame - указатель на начало буфера (адрес)
													// usLen - длина пакета
{
	volatile unsigned char   ucCRCHi = 0xFF;					// старший байт црц
	volatile unsigned char   ucCRCLo = 0xFF;					// младший байт црц
	volatile int             iIndex;
	int i=0;

	while (usLen--)									// цикл, usLen уменьшается на 1 при каждой итерации
	{
		iIndex = ucCRCLo ^ pucFrame[i];				// ксорим ucCRCLo  и байт, на который указывает pucFrame.
		i++;										// полученное значение будет индексом iIndex в таблицах. pucFrame инкрементируется.
		ucCRCLo = ucCRCHi ^ (auchCRCHi[iIndex] );	// ксорим ucCRCHi и значение из таблицы aucCRCHi с индексом iIndex.
		ucCRCHi = ( auchCRCLo[iIndex] );			// ucCRCHi равно значению из таблицы aucCRCLo с индексом iIndex
	}
	return ( ucCRCHi << 8 | ucCRCLo );				// Возвращаем старший и младший байт CRC в виде 16-разрядного слова
}

void res_in(void)
{
	/*GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);*/
}

void res_out(void)
{
	/*GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Pin = GPIO_PIN_3;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);*/
}

void pa7_out(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void pa7_spi(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}



void delay(uint32_t i0) {
 volatile uint32_t j0;
 for (j0=0; j0!= i0*10; j0++)
  ;
}

void delay_hz(uint32_t i00) {
 volatile uint32_t j00;
 for (j00=0; j00!= i00; j00++)
  ;
}





//????????? ?????? ????? ? ?????????
void Write_Byte(uint8_t b, uint8_t cd) {
	HAL_SPI_Transmit(&hspi1,&b,1,50);//SPI_SendData8(SPI1,b); //LCD.D=b;		//?????? ???? ?? ???? ?????? ??????????
	//HAL_SPI_DeInit(&hspi1);//(SPI1, DISABLE);
	pa7_out();
	if (cd) {HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);}
	else {HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET);
		}
	delay_hz(4);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14,GPIO_PIN_SET);
	delay_hz(20);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14,GPIO_PIN_RESET);
	//HAL_Delay(1);
	delay_hz(1000);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET);
		pa7_spi();
	//MX_SPI1_Init();//SPI_Cmd(SPI1, ENABLE);
}

void WriteCode(uint8_t b) { Write_Byte(b,0); }

void WriteData(uint8_t b) { Write_Byte(b,1); }

void screen_init()
{
	res_out();
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	//GPIO_fResetBits(GPIOA, GPIO_PIN_14); //LCD.E=0;//????????? ???????? ??????? ??????????
	//GPIO_fResetBits(GPIOA, GPIO_PIN_3); //LCD.RES=0;//?????? ?????? RES=0 ??????????
	delay_hz(100);//???????? ?? ????? ?????? 10 ???
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); //LCD.RES=1;//????? ?????? RES
	res_in();
	HAL_Delay(2);//???????? ?? ????? ?????? 1 ??
	WriteCode(0xE2);//Reset
	WriteCode(0xEE);//ReadModifyWrite off
	WriteCode(0xA4);//???????? ??????? ?????
	WriteCode(0xA8);//??????????? 1/16
	WriteCode(0xC0);//??????? ?????? ?? 0
	WriteCode(0xA0);//NonInvert scan RAM
	WriteCode(0xAF);//Display on
}

void clearscreen(int p)
{
	int c,i;
	for(i=p; i<p+2; i++) {//???? ?? ???? 2-? ????????? ??????????
		WriteCode(i|0xB8);//????????? ??????? ???????? ??? ????? ?????????? ??????????
		WriteCode(0x00);//????????? ???????? ?????? ??? ?????? ?????? ? 0
			for(c=0; c<61; c++) {//???????????? ???? ??????
			  	WriteData(0x00);
			}
	}
}

void SPI_syn_out(uint8_t data1)
{
	HAL_SPI_Transmit(&hspi1,&data1,1,50);//SPI_SendData8(SPI1,data1);
	//while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
	delay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);
}

void placedchar (char l1,int plc,int ln)
{
	uint8_t l; //??????? ???????
	WriteCode(ln|0xB8);//????????? ???????? ??? ????? ?????????? ??????????
	WriteCode(0x00|plc);//????????? ???????? ?????? ??? ?????? ?????? ? plc
		for(l=0; l<6; l++) {//?????????? ???? ??????
			WriteData(CharSet[l1][l]);
		}
}

void scr_time(uint8_t Hours, uint8_t Minutes, uint8_t lin, uint8_t clm, uint8_t shine)
{
		switch (shine)
      	{
  	  		case 0:
  	  		{
  	  			placedchar(48+Hours/10,0+clm,lin);
  	  			placedchar(48+Hours%10,6+clm,lin);
  	  			placedchar(58,11+clm,lin);
  	  			placedchar(48+Minutes/10,15+clm,lin);
  	  			placedchar(48+Minutes%10,21+clm,lin);
  	  			//prev_min=Minutes;
  	  			break;
  	  		}
  	  		case 1:
  	  		{
  	  			placedchar(0x01,0+clm,lin);
  	  			placedchar(0x01,6+clm,lin);
  	  			placedchar(58,11+clm,lin);
  	  		  	placedchar(48+Minutes/10,15+clm,lin);
  	  		  	placedchar(48+Minutes%10,21+clm,lin);
  	  			break;
  	  		}
  	  		case 2:
  	  		{
  	  			placedchar(48+Hours/10,0+clm,lin);
  	  			placedchar(48+Hours%10,6+clm,lin);
  	  			placedchar(58,11+clm,lin);
  	  			placedchar(0x01,15+clm,lin);
  	  			placedchar(0x01,21+clm,lin);
  	  			break;
  	  		}
		}
}

void scr_time_down (uint16_t Motor, uint8_t lin, uint8_t clm, uint8_t shine)
{
	uint8_t MIN,SEC;
	MIN=Motor/60;
	SEC=Motor%60;
	if (MIN>9) {placedchar(48+MIN/10,0+clm,lin);}
	else {placedchar(0x01,0+clm,lin);}
	placedchar(48+MIN%10,6+clm,lin);
	placedchar(58,11+clm,lin);
	placedchar(48+SEC/10,15+clm,lin);
	placedchar(48+SEC%10,21+clm,lin);
}


void dot_per(uint8_t High, uint8_t Low, uint8_t lin, uint8_t clm, uint8_t shine)
{
		switch (shine)
      	{
  	  		case 0:
  	  		{
  	  			placedchar(48+High,0+clm,lin);
  	  			placedchar(46,6+clm,lin);
  	  			placedchar(48+Low,8+clm,lin);
  	  			placedchar(36,14+clm,lin);
  	  			break;
  	  		}
  	  		case 1:
  	  		{
  	  			placedchar(0x01,0+clm,lin);
  	  			placedchar(46,6+clm,lin);
  	  			placedchar(48+Low,8+clm,lin);
  	  			placedchar(36,14+clm,lin);
  	  			break;
  	  		}
  	  		case 2:
  	  		{
  	  			placedchar(48+High,0+clm,lin);
  	  			placedchar(46,6+clm,lin);
  	  			placedchar(0x01+Low,8+clm,lin);
  	  			placedchar(36,14+clm,lin);
  	  			break;
  	  		}
		}
}

void scr_cnt (uint8_t CNT, uint8_t lin, uint8_t clm, uint8_t shine)
{
	switch (shine)
	{
		case 0:
		{
			if (CNT>99) {placedchar(48+CNT/100,clm-6,lin);}
			else {placedchar(0x01,clm-6,lin);}
			if (CNT>9) {placedchar(48+CNT%100/10,0+clm,lin);}
			else {placedchar(0x01,0+clm,lin);}
			placedchar(48+CNT%10,6+clm,lin);
			break;
		}
		case 1:
		{
			if (CNT>99) {placedchar(0x01,clm-6,lin);}
			placedchar(0x01,0+clm,lin);
			placedchar(0x01,6+clm,lin);
			break;
		}
	}
}

void oneline(uint8_t line,char l1[10])
{
	uint8_t p1; //????? ?????
	p1=strlen(l1);
	uint8_t c; //??????
	uint8_t l; //??????? ???????
	WriteCode(line|0xB8);//????????? ?????? ???????? ??? ????? ?????????? ??????????
	WriteCode(0x00);//????????? ???????? ?????? ??? ?????? ?????? ? 0
		for(c=0; c<p1; c++) {//???????????? ???? ??????
			for(l=0; l<6; l++) {//?????????? ???? ??????
				WriteData(CharSet[l1[c]][l]);
			}
		}
		for(c=p1; c<10; c++) {//???????????? ???? ??????
			for(l=0; l<6; l++) {//?????????? ???? ??????
				WriteData(CharSet[0x00][l]);
			}
		}
}

void twolines(char l1[10],char l2[10])
{
	uint8_t p1,p2;
	p1=strlen(l1);
	p2=strlen(l2);
	uint8_t c; 
	uint8_t l; 

		for(c=0; c<p1; c++) {
			placedchar(l1[c],c*6,0);
			/*for(l=0; l<6; l++) {
				WriteData(CharSet[l1[c]][l]);
			}*/
		}
		for(c=p1; c<10; c++) {
			for(l=0; l<6; l++) {
				WriteData(CharSet[0x00][l]);
			}
		}

		for(c=0; c<p2; c++) {
			placedchar(l2[c],c*6,1);
		}
		for(c=p2; c<10; c++) {
			for(l=0; l<6; l++) {
				WriteData(CharSet[0x00][l]);
			}
		}
}

void twolines2(char l1[10],char l2[10])
{
	uint8_t p1,p2; //????? ?????
	p1=strlen(l1);
	p2=strlen(l2);
	uint8_t c; //??????
	uint8_t l; //??????? ???????
	WriteCode(0|0xB8);//????????? ?????? ???????? ??? ????? ?????????? ??????????
	WriteCode(0x00);//????????? ???????? ?????? ??? ?????? ?????? ? 0
		for(c=0; c<p1; c++) {//???????????? ???? ??????
			for(l=0; l<6; l++) {//?????????? ???? ??????
				WriteData(CharSet[l1[c]][l]);
			}
		}
		for(c=p1; c<10; c++) {//???????????? ???? ??????
			for(l=0; l<6; l++) {//?????????? ???? ??????
				WriteData(CharSet[0x00][l]);
			}
		}
		WriteCode(1|0xB8);//????????? ?????? ???????? ??? ????? ?????????? ??????????
		WriteCode(0x00);//????????? ???????? ?????? ??? ?????? ?????? ? 0
		for(c=0; c<p2; c++) {//???????????? ???? ??????
			for(l=0; l<6; l++) {//?????????? ???? ??????
				WriteData(CharSet[l2[c]][l]);
			}
		}
		for(c=p2; c<10; c++) {//???????????? ???? ??????
			for(l=0; l<6; l++) {//?????????? ???? ??????
				WriteData(CharSet[0x00][l]);
			}
		}
}

uint8_t PlusOne(uint8_t pl,uint8_t high)
{
	pl++;
	if (pl>=high) {pl=0;}
	return pl;
}

uint8_t MinusOne(uint8_t pl,uint8_t high)
{
	if (pl<=0) {pl=high;}
	pl--;
	return pl;
}



void FlashRoyal(void)
{
	uint32_t Buf[7];
	uint32_t PgError = 0;
	Buf[0]=(uint32_t)((uint8_t)StartHr*0x1000000+(uint8_t)StartMin*0x10000+(uint8_t)StopHr*0x100+(uint8_t)StopMin);
	Buf[1]=(uint32_t)((uint8_t)CntMin*0x1000000+(uint8_t)CntSec*0x10000+(uint8_t)CntFood*0x100);//+(uint8_t)Per)
	Buf[2]=(uint32_t)(power);
	Buf[3]=(uint32_t)(Per);
	Buf[4]=(uint32_t)((uint8_t)MBAdr*0x100+(uint8_t)MBSpeed);
	HAL_FLASH_Unlock();
	
	FLASH_EraseInitTypeDef Flash_eraseInitStruct;
	Flash_eraseInitStruct.TypeErase     = FLASH_TYPEERASE_PAGES;
	Flash_eraseInitStruct.PageAddress  = ST_ADR;
	Flash_eraseInitStruct.NbPages        = 2;

	if(HAL_FLASHEx_Erase(&Flash_eraseInitStruct, &PgError) != HAL_OK)
	{
		 HAL_FLASH_Lock();
	}

	HAL_FLASH_Program(TYPEPROGRAM_WORD, ST_ADR,Buf[0]);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, ST_ADR+4,Buf[1]);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, ST_ADR+8,Buf[2]);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, ST_ADR+12,Buf[3]);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, ST_ADR+16,Buf[4]);
	HAL_FLASH_Lock();
}

uint8_t find_next(uint16_t START, uint16_t STOP, uint32_t NOW)
{
	uint8_t nxt=100;
	if (START<STOP)
	{
		if((NOW*60+RTC_DateTime.Seconds<=START*60)||(NOW>STOP))
		{
			nxt=0;
		}
		else if(NOW==STOP)
		{
			nxt=CntFood-1;
		}
		else
		{
			nxt=1;
			NOW=NOW*60+RTC_DateTime.Seconds;
			while(Korm[nxt].R_Hours*3600+Korm[nxt].R_Minutes*60+Korm[nxt].R_Seconds<NOW)
			{
				nxt++;
			}
		}
	}
	else
	{
		power=0;
		FlashRoyal();
	}
	return nxt;
}



void WakeRestart2 (void)
{

    //HAL_Delay(1000);
		SystemClock_Config();

    screen_init();
    /* Disable Wakeup Counter */
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
		NOTSLEEP=1;
	if ((state==0x09)||(MOTOR_TIME1==0))
	{
		sc_up=1;
	}
	if (state==0x00)
	{
		twolines("КОРМЛЕНИЕ","");
		scr_time(GlobalHr,GlobalMin,1,0,0);
		prev_min=RTC_DateTime.Minutes;

	}
	if ((state!=0x00)&&(state!=0x18)&&(state!=0x25))
	{
		sc_up=1;
		state=0x09;
	}
}

void WakeRestart (void)
{

    HAL_Delay(1000);
		SystemClock_Config();

    screen_init();
		clearscreen(0);
		clearscreen(2);
    /* Disable Wakeup Counter */
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	
		if ((state==0x09)||(MOTOR_TIME1==0))
		{
			sc_up=1;
		}
		if (state==0x00)
		{
			twolines("КОРМЛЕНИЕ","");
			scr_time(GlobalHr,GlobalMin,1,0,0);
			prev_min=RTC_DateTime.Minutes;

		}
		if ((state!=0x00)&&(state!=0x18)&&(state!=0x25))
		{
			sc_up=1;
			state=0x01;
		}
}

void KormPlacement(void)
{
	if (CntFood>0)
	{
		MOTOR_TIME=((uint16_t)CntMin*60+CntSec)/(CntFood*K1);


		Korm[0].R_Hours=StartHr;
		Korm[0].R_Minutes=StartMin;
		Korm[0].R_Seconds=0;
		Korm[CntFood-1].R_Hours=StopHr;
		Korm[CntFood-1].R_Minutes=StopMin;
		Korm[CntFood-1].R_Seconds=0;
		if(Korm[0].R_Hours*60+Korm[0].R_Minutes<Korm[CntFood-1].R_Hours*60+Korm[CntFood-1].R_Minutes)
		{
			PauseT=((Korm[CntFood-1].R_Hours*3600+Korm[CntFood-1].R_Minutes*60)-(Korm[0].R_Hours*3600+Korm[0].R_Minutes*60))/(CntFood-1);
		}
		for(ik=1;ik<CntFood-1;ik++)
		{
			PT1=Korm[ik-1].R_Hours*3600+Korm[ik-1].R_Minutes*60+Korm[ik-1].R_Seconds+PauseT;
			Korm[ik].R_Hours=PT1/3600;
			Korm[ik].R_Minutes=PT1%3600/60;
			Korm[ik].R_Seconds=PT1%60;
		}
	}
	else {power=0;
  FlashRoyal();
	}
}

void Flash_Jump_Adress(uint32_t adress)															// Переход в область другой программы во флеш памяти
{
	__set_PRIMASK(1);																							// Отключаем глобальные прерывания(обязательно перед переходом)																						 					
	
	typedef 	void (*pFunction)(void);														// Объявляем тип функции-ссылки
	pFunction Jump_To_Application;																// Объявляем функцию-ссылку

  uint32_t JumpAddress = *(__IO uint32_t*) (adress + 4); 						// Адрес перехода на вектор (Reset_Handler) 		
  Jump_To_Application = (pFunction) JumpAddress;  							// Указатель на функцию перехода
	__set_MSP(*(volatile uint32_t*) adress);														// Указываем адрес вектора стека(Stack Pointer)	
  Jump_To_Application();                          							// Переходим на основную программу
}	

static void SystemPower_Config2(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();
  
  /* Enable the fast wake up from Ultra low power mode */
  HAL_PWREx_EnableFastWakeUp();

  /* Select MSI as system clock source after Wake Up from Stop mode */
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG (RCC_STOP_WAKEUPCLOCK_MSI);
  
  /* Enable GPIOs clock */
	
	
  /*__HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStructure.Pin = GPIO_PIN_All;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.Pin = GPIO_PIN_1;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
	
	
	GPIO_InitStructure.Pin = GPIO_PIN_1;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  //
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_USART2_CLK_DISABLE();
	__HAL_RCC_TIM2_CLK_DISABLE();
	__HAL_RCC_SPI1_CLK_DISABLE();
	__HAL_RCC_ADC1_CLK_DISABLE();*/
	
}

static void SystemPower_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();
  
  /* Enable the fast wake up from Ultra low power mode */
  HAL_PWREx_EnableFastWakeUp();

  /* Select MSI as system clock source after Wake Up from Stop mode */
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG (RCC_STOP_WAKEUPCLOCK_MSI);
  
  /* Enable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();


  /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
  /* Note: Debug using ST-Link is not possible during the execution of this   */
  /*       example because communication between ST-link and the device       */
  /*       under test is done through UART. All GPIO pins are disabled (set   */
  /*       to analog input mode) including  UART I/O pins.           */
  GPIO_InitStructure.Pin = GPIO_PIN_All;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Disable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();

}

uint32_t FLASH_Read(uint32_t address)
{
    return (*(__IO uint32_t*)address);
}

uint16_t errorsGet(void)
{
	uint16_t result=0;
	if(CntFood==0)
	{
		result|=0x04;
	}
	if(StartHr*60+StartMin>=StopHr*60+StopMin)
	{
		result|=0x01;
	}
	if((CntMin+CntSec)==0)
	{
		result|=0x02;
	}
	if((CntMin*60+CntSec)>=(StopHr*3600+StopMin*60+MOTOR_TIME)-(StartHr*3600+StartMin*60))
	{
		result|=0x10;
	}
	if(PauseT<60)
	{
		result|=0x08;
	}
	return result;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
	
		

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	SystemPower_Config();

  /* USER CODE END SysInit */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Pin = GPIO_PIN_All;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_ADC_Init();
	MX_IWDG_Init();

	rele&=0xDF;
  SPI_syn_out(rele);
	
	uint8_t a[5]={1,2,3,4,5};
	res_in();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14 | GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_IWDG_Refresh(&hiwdg);
	//SPI_syn_out(0x00);
	HAL_Delay(500);
	res_in();

	/*rele=0x80;
	rele|=0x40;
	SPI_syn_out(rele);
	delay(1000);
	rele&=0xBF;
	SPI_syn_out(rele);*/

	timer1_init();
	timer2_init();
	
	/*timer3_init();
	timer4_init();*/

	
	FLASH_BUF = FLASH_Read(ST_ADR);


	if (FLASH_BUF==0xFFFFFFFF) //?????? ????????? -> ????????? ??????????? ??????????
	{
		StartHr=0;
		StartMin=0;
		StopHr=0;
		StopMin=0;

		CntMin=0;
		CntSec=0;
		CntFood=0;
		Per=0;
		MBSpeed=8;
		MBAdr=1;
		FlashRoyal();
	}

	StartHr=FLASH_BUF/0x1000000;
	StartMin=FLASH_BUF/0x10000%0x100;
	StopHr=FLASH_BUF%0x10000/0x100;
	StopMin=FLASH_BUF%0x1000000;
	
	FLASH_BUF = FLASH_Read(ST_ADR+4);
	CntMin=FLASH_BUF/0x1000000;
	CntSec=FLASH_BUF/0x10000%0x100;
	CntFood=FLASH_BUF%0x10000/0x100;
	//Per=FLASH_BUF%0x1000000;
	
	Per = FLASH_Read(ST_ADR+12);
	
	FLASH_BUF = FLASH_Read(ST_ADR+16);
	
	MBSpeed=FLASH_BUF%0x10;
	MX_TIM22_Init(MBSpeed);
	USART2_ReInit(MBSpeed);
	MBAdr=FLASH_BUF%0x10000/0x100;

	KormPlacement();
	HAL_IWDG_Refresh(&hiwdg);
	

	power=0;
	if(!FirstStart)
	{
		if ((CntFood>0)&&(StartHr*60+StartMin<StopHr*60+StopMin)&&((CntMin+CntSec)>0)&&
		((CntMin*60+CntSec)<(StopHr*3600+StopMin*60+MOTOR_TIME)-(StartHr*3600+StartMin*60))&&(PauseT>=60)&&
		(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1)<20))
		{
			power=(uint8_t)FLASH_Read(ST_ADRp);
			
		}
		state=0x01;
		
	}
	else
	{
		FlashRoyal();	
		state=0x02;
	}

	screen_init();
	clearscreen(0);
	clearscreen(2);
	sc_up=1;
	ts_change=0;



	//SPI_syn_out(0xFF);
	//HAL_Delay(1000);
	//SPI_syn_out(0x80);
	
	rele&=0xDF;
  SPI_syn_out(rele);
	
	HAL_Delay(500);
	
	rele&=0xDF;
  SPI_syn_out(rele);
	
	int i;

	
	HAL_NVIC_SetPriority(RTC_IRQn, 0x0, 0);

HAL_NVIC_EnableIRQ(RTC_IRQn);
	HAL_ADC_Start(&hadc);
	
	uint16_t ADC_b;
	
	NVIC_SetPriority(USART2_IRQn, 0); 
  NVIC_EnableIRQ(USART2_IRQn);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE); 

	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	DEEPSLP=0;
	HAL_UART_Transmit(&huart2,write_buffer,5,100);
	HAL_UART_Receive_IT(&huart2, write_buffer, 15);
	screen_init();
  while (1)
  {
		HAL_IWDG_Refresh(&hiwdg);
		MBPower = power+MBpower_not;
		
		NVIC_SetPriority(USART2_IRQn, 0); 
		NVIC_EnableIRQ(USART2_IRQn);
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE); 
		
		DEEPSLP=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3);	
		uint16_t WakePower=0;
		//DEEPWK=0;
		//HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,WakePower);
			//__HAL_RCC_WAKEUPSTOP_CLK_CONFIG (RCC_STOP_WAKEUPCLOCK_HSI);
			while(DEEPSLP<1)
			{
				HAL_IWDG_Refresh(&hiwdg);
				if(flag_FT)
				{
					flag_FT=0;
					WakePower=0;
				}
				else
				{
					WakePower=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR1);
				}
				if(WakePower<30)
				{
					WakePower++;
					HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,WakePower);
				}
				HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0x07D0, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
				//TIM2->CR1 |= TIM_CR1_CEN;
				//SystemPower_Config();
				//HAL_ADC_Stop(&hadc);
				// Configure the system Power 
				__HAL_RCC_PWR_CLK_ENABLE();



				// Enter Stop Mode 
				HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
			
			
				//uu++;
			
			
				DEEPSLP=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3);
				DEEPWK=1;
			}
			if(DEEPWK)
			{
				DEEPWK=0;
				HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

				WakeRestart();
				flag_FT=1;
				if(HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR1)>25)
				{
					power=0;
					FlashRoyal();
				}
				HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,0);

			}
			HAL_IWDG_Refresh(&hiwdg);
			
			if(flag20)
			{
				if(sec20!=RTC_DateTime.Seconds)
				{
					sec20=RTC_DateTime.Seconds;
					sec20cnt++;
					if(sec20cnt>19)
					{
						twenty_sec=1;
					}
				}
			}

    	TIM2->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
			/*if(
				(rele>128)&&!((state==0x00)||(state==0x18)||(state==0x25)))
			{
				rele&=0xDF;
    		SPI_syn_out(rele);
			}*/
    	if((LOWPOWER==0)||(state==0x00)||(state==0x18)||(state==0x25)){
    	//while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
				
 
			if(state==0x00)
			{
				MBKorm=1;
			}
			else
			{
				MBKorm=0;
			}
			if(state==0x18)
			{
				MBTest=1;
			}
			else
			{
				MBTest=0;
			}
			HAL_ADC_PollForConversion(&hadc, 100);
    	ADC_b = HAL_ADC_GetValue(&hadc);
				
			if((state!=0x00)&&(state!=0x18)&&(state!=0x25))
			{
				rele&=0xDF;
    		SPI_syn_out(rele);
			}	
				
    	/*if(ADC_b>4)
    	{
    		NOTSLEEP=1;
				ADC_ready=1;
    	}*/

    	if(ADC_b!=ADC_old)
    	{
    		ADC_dr=1;
    	}
    	if(ADC_dr)
    	{
    		NOTSLEEP=1;
    		MAYSLEEP=0;
    		if(ADC_frst)
    		{
    			ADC_old=ADC_b;
    			ADC_frst=0;
    			ADC_ready=0;
    		}
    		else if ((ADC_b<ADC_old+2)&&((int)ADC_b>=(int)(ADC_old-2)))
    		{
    			ADC_cnt++;
    		}
    		else
    		{
    			ADC_cnt=0;
    			ADC_frst=1;
    		}
    		if(ADC_cnt>10)
    		{
					
    			ADC_cnt=0;
    			ADC_old=ADC_b;
    			ADC_ready=1;
    			ADC_dr=0;
    			ADC_frst=1;
    		}
    	}
    	}

    	if (!(TIME_CH))
    	{
				HAL_RTC_GetDate(&hrtc, &RTC_Date1, RTC_FORMAT_BIN);
				HAL_RTC_GetTime(&hrtc, &RTC_DateTime, RTC_FORMAT_BIN);
    		GlobalHr=RTC_DateTime.Hours;
    		GlobalMin=RTC_DateTime.Minutes;
    		//GlobalSec=RTC_DateTime.Seconds;
    	}

    	if (power&&!VAR_CH&&!TIME_CH)
    	{
    		if (NXT_TIME>=100)
    		{
    			NXT_TIME=find_next(StartHr*60+StartMin,StopHr*60+StopMin,GlobalHr*60+GlobalMin);
    		}
    		else
    		{
					if(POWER_KORM)//?????????? ???? ?????? ?? ???
					{
						POWER_KORM=0;
						Korm_Now=RTC_DateTime.Hours*3600+RTC_DateTime.Minutes*60+RTC_DateTime.Seconds;
						Korm_St=Korm[NXT_TIME].R_Hours*3600+Korm[NXT_TIME].R_Minutes*60+Korm[NXT_TIME].R_Seconds;
						Korm_Fn=Korm_St+MOTOR_TIME;
						Korm_St1=Korm[NXT_TIME-1].R_Hours*3600+Korm[NXT_TIME-1].R_Minutes*60+Korm[NXT_TIME-1].R_Seconds;
						Korm_Fn1=Korm_St1+MOTOR_TIME;
						if((Korm_Now>=Korm_St)&&(Korm_Now<Korm_Fn))
						{
							MOTOR_TIME1=Korm_Fn-Korm_Now;
							NOTSLEEP=1;
							state=0x00;
							sc_up=1;
							prev_min = 61;
							prev_sec=61;
							clearscreen(0);
						}
						else if ((Korm_Now>=Korm_St1)&&(Korm_Now<Korm_Fn1))
						{
							NXT_TIME--;
							MOTOR_TIME1=Korm_Fn-Korm_Now;
							NOTSLEEP=1;
							state=0x00;
							sc_up=1;
							prev_min = 61;
							prev_sec=61;
							clearscreen(0);
						}
						else 
						{
							NXT_TIME=100;
							
						}
					}
					else
					
						if ((RTC_DateTime.Hours==Korm[NXT_TIME].R_Hours)&&(RTC_DateTime.Minutes==Korm[NXT_TIME].R_Minutes)&&(RTC_DateTime.Seconds==Korm[NXT_TIME].R_Seconds))
						{
							state=0x00;
							sc_up=1;
							prev_min = 61;
							prev_sec=61;
							clearscreen(0);
							MOTOR_TIME1=MOTOR_TIME+1;
							NOTSLEEP=1;
						}
				}
    	}


    //if(NOTSLEEP)
    //{
    	if (ADC_ready)
    	{
    	if ((state==0)||(state==0x18))
    	//if (state==0)
    	{
    		if ((ADC_b>=24)&&(ADC_b<27))
        	{
        		//delay(700);
        		if(!reset_t)
        		{
        			reset_t=1;
        			TIM21->CNT = 0;//TIM_Set/Counter(TIM21, 0);
        			//TIM21->CR1 |= TIM_CR1_CEN;
							HAL_TIM_Base_Start_IT(&htim21);
							//placedchar(46,56,0);
        		}
        		else if(two_sec)
        		{
        			//TIM21->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
							HAL_TIM_Base_Stop_IT(&htim21);
        			MOTOR_TIME1=0;
        			power=0;
								FlashRoyal();					
        			reset_t=0;
							blink=0;
							placedchar(49,50,0);
							/*if (state==0x18)
							{
								placedchar(49,56,0);
	      			  rele&=0xDF;
	      			  SPI_syn_out(rele);
								twenty_sec=1;
								
							}*/
        		}
        	}
    		else if (reset_t)
    		{
    			reset_t=0;
    			//TIM21->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
					HAL_TIM_Base_Stop_IT(&htim21);
    		}
    	}
    	else
    	{
    		/*while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    		ADC_b = ADC_GetConversionValue(ADC1);*/

    	if ((ADC_b>=24)&&(ADC_b<27)&&(state==0x08))
    	{
    		delay(700);
    		if (!double_but)
    		{
    			double_but=1;
    		  	TIM21->CNT = 0;
        		//TIM21->CR1 |= TIM_CR1_CEN;
						HAL_TIM_Base_Start_IT(&htim21);
    		  	//TIM22->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
						//HAL_TIM_Base_Stop_IT(&htim22);
						flag20=0;
    		  	wait=0;
    		}
    	}

    	else if /*((ADC_b>=11)&&(ADC_b<13))*/((ADC_b>21)&&(ADC_b<24))
    	{
    		delay(700);
    		if (!but_p)
    		{
    			but_p=1;
    		  	TIM21->CNT = 0;
        		//TIM21->CR1 |= TIM_CR1_CEN;
						HAL_TIM_Base_Start_IT(&htim21);
    		  	//TIM22->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
						//HAL_TIM_Base_Stop_IT(&htim22);
						flag20=0;
    		  	wait=0;
    		}
    	}
    	else if ((ADC_b>=5)&&(ADC_b<7))
    	{
  			delay(700);
  			if (!but_plus)
  			{
  				but_plus=1;
    		  TIM21->CNT = 0;
        	//TIM21->CR1 |= TIM_CR1_CEN;
					HAL_TIM_Base_Start_IT(&htim21);
    		  //TIM22->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
					//HAL_TIM_Base_Stop_IT(&htim22);
					flag20=0;
  				wait=0;
  			}
  			else if (two_sec)
  			{
    			switch (state)
    		    {
    				case 0x12:
    			    {
    			    	GlobalHr=PlusOne(GlobalHr,24);
    			    	break;
    			    }
    				case 0x22:
    			    {
    			    	GlobalMin=PlusOne(GlobalMin,60);
    			    	break;
    			    }
    				case 0x13:
    			    {
    			    	StartHr=PlusOne(StartHr,24);
    			    	break;
    			    }
    				case 0x23:
    			    {
    			    	StartMin=PlusOne(StartMin,60);
    			    	break;
    			    }
    				case 0x14:
    			    {
    			    	StopHr=PlusOne(StopHr,24);
    			    	break;
    			    }
    				case 0x15:
    			    {
    			    	CntFood=PlusOne(CntFood,100);
    			    	break;
    			    }
    				case 0x24:
    			    {
    			    	StopMin=PlusOne(StopMin,60);
    			    	break;
    			    }
    				case 0x16:
    			    {
    			    	Per=(Per%10)+PlusOne(Per/10,10)*10;
    			    	break;
    			    }
    				case 0x26:
    			    {
    			    	Per=(Per/10*10)+PlusOne(Per%10,10);
    			    	break;
    			    }
    				case 0x17:
    			    {
    			    	CntMin=PlusOne(CntMin,100);
    			    	break;
    			    }
    				case 0x27:
    			    {
    			    	CntSec=PlusOne(CntSec,60);
    			    	break;
    			    }
						case 0x1A:
    			    {
    			    	MBAdr=PlusOne(MBAdr,247);
								if(MBAdr==0)
								{
									MBAdr=1;
								}
    			    	break;
    			    }
    				case 0x2A:
    			    {
    			    	MBSpeed=PlusOne(MBSpeed,9);
    			    	break;
    			    }
    		    }
    			delay(SPEEDPLUS);
			}
    	}
    	else if /*((ADC_b>21)&&(ADC_b<24))*/((ADC_b>=11)&&(ADC_b<13))
    	{
  			delay(700);
  			if (!but_minus)
  			{
  				but_minus=1;
    		  TIM21->CNT = 0;
        	//TIM21->CR1 |= TIM_CR1_CEN;
					HAL_TIM_Base_Start_IT(&htim21);
    		  //TIM22->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
					//HAL_TIM_Base_Stop_IT(&htim22);
					flag20=0;
  				wait=0;

  			}
  			else if (two_sec)
  			{
    			switch (state)
    		    {
    				case 0x12:
    			    {
    			    	GlobalHr=MinusOne(GlobalHr,24);
    			    	break;
    			    }
    				case 0x22:
    			    {
    			    	GlobalMin=MinusOne(GlobalMin,60);
    			    	break;
    			    }
    				case 0x13:
    			    {
    			    	StartHr=MinusOne(StartHr,24);
    			    	break;
    			    }
    				case 0x23:
    			    {
    			    	StartMin=MinusOne(StartMin,60);
    			    	break;
    			    }
    				case 0x14:
    			    {
    			    	StopHr=MinusOne(StopHr,24);
    			    	break;
    			    }
    				case 0x15:
    			    {
    			    	CntFood=MinusOne(CntFood,100);
    			    	break;
    			    }
    				case 0x24:
    			    {
    			    	StopMin=MinusOne(StopMin,60);
    			    	break;
    			    }
    				case 0x16:
    			    {
    			    	Per=(Per%10)+MinusOne(Per/10,10)*10;
    			    	break;
    			    }
    				case 0x26:
    			    {
    			    	Per=(Per/10*10)+MinusOne(Per%10,10);
    			    	break;
    			    }
    				case 0x17:
    			    {
    			    	CntMin=MinusOne(CntMin,100);
    			    	break;
    			    }
    				case 0x27:
    			    {
    			    	CntSec=MinusOne(CntSec,60);
    			    	break;
    			    }
						case 0x1A:
    			    {
    			    	MBAdr=MinusOne(MBAdr,247);
								if(MBAdr==0)
								{
									MBAdr=246;
								}
    			    	break;
    			    }
    				case 0x2A:
    			    {
    			    	MBSpeed=MinusOne(MBSpeed,9);
    			    	break;
    			    }
    		    }
    			delay(SPEEDPLUS);
			}
    	}
    	else if ((but_plus)||(but_minus))
    	{
    		//TIM21->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
				HAL_TIM_Base_Stop_IT(&htim21);
    		//blink=0;
    		prev_min = 61;
    		wait=1;
    		//m_timer=0;
    		if (two_sec)
    		{
    			two_sec=0;
    		}
    		else
    		{
    			switch (state)
    		    {
    				case 0x12:
    			    {
    			    	if (but_plus){GlobalHr=PlusOne(GlobalHr,24);}
    			    	else {GlobalHr=MinusOne(GlobalHr,24);}
    			    	break;
    			    }
    				case 0x22:
    			    {
    			    	if (but_plus){GlobalMin=PlusOne(GlobalMin,60);}
    			    	else {GlobalMin=MinusOne(GlobalMin,60);}
    			    	break;
    			    }
    				case 0x13:
    			    {
    			    	if (but_plus){StartHr=PlusOne(StartHr,24);}
    			    	else {StartHr=MinusOne(StartHr,24);}
    			    	break;
    			    }
    				case 0x23:
    			    {
    			    	if (but_plus){StartMin=PlusOne(StartMin,60);}
    			    	else {StartMin=MinusOne(StartMin,60);}
    			    	break;
    			    }
    				case 0x14:
    			    {
    			    	if (but_plus){StopHr=PlusOne(StopHr,24);}
    			    	else {StopHr=MinusOne(StopHr,24);}
    			    	break;
    			    }
    				case 0x15:
    			    {
    			    	if (but_plus){CntFood=PlusOne(CntFood,100);}
    			    	else {CntFood=MinusOne(CntFood,100);}
    			    	break;
    			    }
    				case 0x24:
    			    {
    			    	if (but_plus){StopMin=PlusOne(StopMin,60);}
    			    	else {StopMin=MinusOne(StopMin,60);}
    			    	break;
    			    }
    				case 0x16:
    			    {
    			    	if (but_plus){Per=(Per%10)+PlusOne(Per/10,10)*10;}
    			    	else {Per=(Per%10)+MinusOne(Per/10,10)*10;}
    			    	break;
    			    }
    				case 0x26:
    			    {
    			    	if (but_plus){Per=(Per/10*10)+PlusOne(Per%10,10);}
    			    	else {Per=(Per/10*10)+MinusOne(Per%10,10);}
    			    	break;
    			    }
    				case 0x17:
    			    {
    			    	if (but_plus){CntMin=PlusOne(CntMin,100);}
    			    	else {CntMin=MinusOne(CntMin,100);}
    			    	break;
    			    }
    				case 0x27:
    			    {
    			    	if (but_plus){CntSec=PlusOne(CntSec,60);}
    			    	else {CntSec=MinusOne(CntSec,60);}
    			    	break;
    			    }
						case 0x1A:
    			    {
								if (but_plus)
								{
									MBAdr=PlusOne(MBAdr,247);
									if(MBAdr==0)
									{
										MBAdr=1;
									}
								}
    			    	else 
								{
									MBAdr=MinusOne(MBAdr,247);
									if(MBAdr==0)
									{
										MBAdr=246;
									}
								}
    			    	break;
    			    }
    				case 0x2A:
    			    {
    			    	if (but_plus){MBSpeed=PlusOne(MBSpeed,9);}
    			    	else {MBSpeed=MinusOne(MBSpeed,9);}
    			    	break;
    			    }
    		    }
    		}
    		VAR_CH=1;
    		but_plus=0;but_minus=0;
    	}
    	else if (but_p)
			{
				//TIM21->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
				HAL_TIM_Base_Stop_IT(&htim21);
				POWERFAIL=0;
				SecondCounter=0;
				but_p=0;
				//blink=0;
				clearscreen(0);
				sc_up=1;
				//m_timer=0;
				prev_min = 61;
				if (state!=0x18)
				{
					prev_sec=61;
				}
				if (!ts_change)
	{
					state=state_tabl[state][0];
	}
				if ((VAR_CH)&&(ts_change)) {FLSH_WRT_N=1; VAR_CH=0;power=0;
			FlashRoyal();
				}
				if ((TIME_CH)&&(ts_change))
				{
					TIME_CH=0;
					RTC_DateTime.Hours = GlobalHr;
					RTC_DateTime.Minutes = GlobalMin;
					RTC_DateTime.Seconds = 00;
					HAL_RTC_SetTime(&hrtc, &RTC_DateTime, RTC_FORMAT_BIN);
					power=0;
					FlashRoyal();
				}
				if ((power_ch)&&(ts_change))
				{
					NXT_TIME=100;
					if (power_not) {power_not=0;}
					else 
					{
						power=!power;
						FlashRoyal();
				  }
					power_ch=0;
				}
				else if (power_ch) {power_ch=0;}
				if(ts_change) {ts_change=0;}
				ADC_b=0;
				//if (state!=0x07)
				//{
					wait=1;
				//}
				if(first_button_press)
				{
					first_button_press=0;
					state=0x02;
				}
			}
    	}
    	}
    	    	switch (state)
    		      	{
    		  	  		case 0x00:
    		  	  	    {
											if (sc_up)
											{
												twolines("КОРМЛЕНИЕ","");
												sc_up=0;
												rele|=0x20;
												SPI_syn_out(rele);
												//TIM22->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
												//HAL_TIM_Base_Stop_IT(&htim22);
												flag20=0;
												wait=0;
												NXT_TIME++;
												//while(!RTC_GetSubSecond()) {}
											}
											if (RTC_DateTime.Minutes!=prev_min)
											{
												if(reScreen==0)
													{
														reScreen=1;
													}
													else if (reScreen==1)
													{
														reScreen=2;
														screen_init();
														twolines("КОРМЛЕНИЕ","");
														sc_up=0;
														rele|=0x20;
														delay(1);
														SPI_syn_out(rele);
														flag20=0;
														wait=0;
													}
												scr_time(GlobalHr,GlobalMin,1,0,0);
												prev_min=RTC_DateTime.Minutes;
											}
											if (RTC_DateTime.Seconds!=prev_sec)
											{
												rele|=0x20;
												delay(1);
												SPI_syn_out(rele);
												scr_time_down(MOTOR_TIME1-1,1,31,0);
												MOTOR_TIME1--;
												MBMotorMin=MOTOR_TIME1/60;
												MBMotorSec=MOTOR_TIME1%60;
												prev_sec=RTC_DateTime.Seconds;
											}
    		  	  	    	if (MOTOR_TIME1==0)
    		  	  	    	{
    		  	  	    	      twenty_sec=1;
    		  	  	    	      rele&=0xDF;
    		  	  	    	      SPI_syn_out(rele);
    		  	  	    	      if (NXT_TIME>=100)
    		  	  	    	      {}
    		  	  	    	      else if (NXT_TIME>CntFood-1)
    		  	  	    	      {

    		  	  	    	    	  NXT_TIME=0;
    		  	  	    	    	  if(Per>0)
    		  	  	    	    	  {
																DayUp=(CntMin*60+CntSec)*(1000+Per)/1000;
																if(DayUp>5999){DayUp=5999;}
																CntMin1=DayUp/60;
																CntSec1=DayUp%60;
																
																if ((PauseT)>((CntMin1*60+CntSec1)/(CntFood*K1)))
																{
																	FLSH_WRT_N=1;
																	JUST_FIN=1;
																	CntMin=CntMin1;
																	CntSec=CntSec1;
																}
    		  	  	    	    	  }
    		  	  	    	      }
    		  	  	    	}
    		  	  	      	break;
    		  	  	    }
    		  	  		case 0x10:
    		  	  		{
    		  	  			if (sc_up)
    		  	  		    {
    		  	  				rele&=0xDF;
    		  	  				SPI_syn_out(rele);
    		  	  				//power=pow_test;
    		  	  				placedchar(48+UnHr/10,11,1);
    		  	  				placedchar(48+UnHr%10,17,1);
    		  	  				placedchar(58,22,1);
    		  	  				placedchar(48+UnMin/10,26,1);
    		  	  				placedchar(48+UnMin%10,32,1);
    		  	  				placedchar(58,37,1);
    		  	  				placedchar(48+UnSec/10,41,1);
    		  	  				placedchar(48+UnSec%10-1,47,1);
    		  	  		    }
    		  	  			break;
    		  	  		}
    		      		case 0x01:
    		      		{
    		      			if (sc_up)
    		      		  {
											MBError=errorsGet();
												
											if (MBError==0)
											{				
													power_not=0;
													MBpower_not=0;
													if (power) 
													{
														twolines("ПРОГРАММА","ВКЛ");
														placedchar(0,46,1);
														placedchar(48+GlobalMin/10,50,1);
														scr_time(GlobalHr,GlobalMin,1,35,state/0x10);
													}
													else {twolines("ПРОГРАММА","ВЫКЛ");}
											}
											else
											{
													twolines("УСТАНОВИТЕ","ПАРАМЕТРЫ");
													MBpower_not=10;
													power_not=1;
													
											}
											sc_up=0;
    		      		  }
										if (power) 
										{
											Tick1=HAL_GetTick();
											if((Tick1-Tick2)>1000)
											{
												blink=1;
												Tick2=Tick1;
											}
											if(blink)
											{
												blink=0;
												if(blon)
												{
													blon=0;
													scr_time(GlobalHr,GlobalMin,1,35,0);
													placedchar(0,46,1);
													placedchar(48+GlobalMin/10,50,1);
												}
												else
												{
													blon=1;
													scr_time(GlobalHr,GlobalMin,1,35,0);
												}
											}	
										}
										if (ts_change)
										{
												if (power) {twolines("ПРОГРАММА","ВЫКЛ");}
												else {placedchar(0x01,56,0);twolines("ПРОГРАММА","ОБРАБОТКА");}
										}
    		      			break;
    		      		}
    		      		case 0x11:
    		      		{
    		      			power_ch=1;
										state=0x01;
    		      			break;
    		      		}
    		      		case 0x02:
    		      		{
    		      			if (sc_up)
    		      		    {
    		      				twolines("УСТАНОВКА","ВРЕМЯ");
    		      				sc_up=0;
    		      				scr_time(GlobalHr,GlobalMin,1,35,state/0x10);
    		      		    }
    		      			break;
    		      		}
    		      		case 0x12:
    		      		{
    		      			if (sc_up)
    		      		    {
    		      				twolines("УСТАНОВКА","ВРЕМЯ");
    		      				sc_up=0;
    		      		    }
    		      			TIME_CH=1;
										if((but_p||but_minus||but_plus)&&two_sec){scr_time(GlobalHr,GlobalMin,1,35,0);}
										Tick1=HAL_GetTick();
										if((Tick1-Tick2)>250)
										{
											blink=1;
											Tick2=Tick1;
										}
										if(blink)
										{
											blink=0;
											if(blon)
											{
												blon=0;
												scr_time(GlobalHr,GlobalMin,1,35,state/0x10);
											}
											else
											{
												blon=1;
												scr_time(GlobalHr,GlobalMin,1,35,0);
											}
										}													
    		      			break;
    		      		}
    		      		case 0x22:
    		      		{
    		      			if (sc_up)
    		      		    {
    		      				twolines("УСТАНОВКА","ВРЕМЯ");
    		      				sc_up=0;
    		      		    }
    		      			TIME_CH=1;
										if((but_p||but_minus||but_plus)&&two_sec){scr_time(GlobalHr,GlobalMin,1,35,0);}
										
										Tick1=HAL_GetTick();
										if((Tick1-Tick2)>250)
										{
											blink=1;
											Tick2=Tick1;
										}
										if(blink)
										{
											blink=0;
											if(blon)
											{
												blon=0;
												scr_time(GlobalHr,GlobalMin,1,35,state/0x10);
											}
											else
											{
												blon=1;
												scr_time(GlobalHr,GlobalMin,1,35,0);
											}
										}													
    		      			break;
    		      		}
    		      		case 0x03:
    		      		{
    		      			if (sc_up)
    		      		    {
    		      				twolines("УСТАНОВКА","СТАРТ");
    		      				sc_up=0;
        		      			scr_time(StartHr,StartMin,1,35,state/0x10);
    		      		    }
    		      			break;
    		      		}
    		      		case 0x13:
    		      		{
    		      			if (sc_up)
    		      		    {
    		      				twolines("УСТАНОВКА","СТАРТ");
    		      				sc_up=0;
    		      		    }
										if((but_p||but_minus||but_plus)&&two_sec){scr_time(StartHr,StartMin,1,35,0);}
										Tick1=HAL_GetTick();
										if((Tick1-Tick2)>250)
										{
											blink=1;
											Tick2=Tick1;
										}
										if(blink)
										{
											blink=0;
											if(blon)
											{
												blon=0;
												scr_time(StartHr,StartMin,1,35,state/0x10);
											}
											else
											{
												blon=1;
												scr_time(StartHr,StartMin,1,35,0);
											}
										}													
    		      			break;
    		      		}
    		      		case 0x23:
    		      		{
    		      			if (sc_up)
    		      		    {
    		      				twolines("УСТАНОВКА","СТАРТ");
    		      				sc_up=0;
    		      		    }
										if((but_p||but_minus||but_plus)&&two_sec){scr_time(StartHr,StartMin,1,35,0);}	
										Tick1=HAL_GetTick();
										if((Tick1-Tick2)>250)
										{
											blink=1;
											Tick2=Tick1;
										}
										if(blink)
										{
											blink=0;
											if(blon)
											{
												blon=0;
												scr_time(StartHr,StartMin,1,35,state/0x10);
											}
											else
											{
												blon=1;
												scr_time(StartHr,StartMin,1,35,0);
											}
										}													
    		      			break;
    		      		}
    		      		case 0x04:
    		      		{
    		      			if (sc_up)
    		      		    {
    		      				twolines("УСТАНОВКА","СТОП");
    		      				sc_up=0;
        		      			scr_time(StopHr,StopMin,1,35,state/0x10);
    		      		    }
    		      			break;
    		      		}
    		      		case 0x14:
    		      		{
    		      			if (sc_up)
    		      		    {
    		      				twolines("УСТАНОВКА","СТОП");
    		      				sc_up=0;
    		      		    }
										if((but_p||but_minus||but_plus)&&two_sec){scr_time(StopHr,StopMin,1,35,0);}
										Tick1=HAL_GetTick();
										if((Tick1-Tick2)>250)
										{
											blink=1;
											Tick2=Tick1;
										}
										if(blink)
										{
											blink=0;
											if(blon)
											{
												blon=0;
												scr_time(StopHr,StopMin,1,35,state/0x10);
											}
											else
											{
												blon=1;
												scr_time(StopHr,StopMin,1,35,0);
											}
										}													

    		      			break;
    		      		}
    		      		case 0x24:
    		      		{
    		      			if (sc_up)
    		      		    {
    		      				twolines("УСТАНОВКА","СТОП");
    		      				sc_up=0;
    		      		    }
										if((but_p||but_minus||but_plus)&&two_sec){scr_time(StopHr,StopMin,1,35,0);}	
										Tick1=HAL_GetTick();
										if((Tick1-Tick2)>250)
										{
											blink=1;
											Tick2=Tick1;
										}
										if(blink)
										{
											blink=0;
											if(blon)
											{
												blon=0;
												scr_time(StopHr,StopMin,1,35,state/0x10);
											}
											else
											{
												blon=1;
												scr_time(StopHr,StopMin,1,35,0);
											}
										}		

    		      			break;
    		      		}
    		      		case 0x05:
    		      		{
    		      			if (sc_up)
    		      		    {
    		      				twolines("КОРМЛЕНИЙ","ВСЕГО");
    		      				sc_up=0;
        		      			scr_cnt(CntFood,1,42,state/0x10);
    		      		    }
    		      			break;
    		      		}
    		      		case 0x15:
    		      		{
    		      			if (sc_up)
    		      		    {
    		      				twolines("КОРМЛЕНИЙ","ВСЕГО");
    		      				sc_up=0;
    		      		    }
										if((but_p||but_minus||but_plus)&&two_sec){scr_cnt(CntFood,1,42,0);}
										Tick1=HAL_GetTick();
										if((Tick1-Tick2)>250)
										{
											blink=1;
											Tick2=Tick1;
										}
										if(blink)
										{
											blink=0;
											if(blon)
											{
												blon=0;
												scr_cnt(CntFood,1,42,state/0x10);
											}
											else
											{
												blon=1;
												scr_cnt(CntFood,1,42,0);
											}
										}												

    		      			break;
    		      		}
    		      		case 0x25:
    		      		{
    		      			if (sc_up)
										{
											twolines("НЕПРЕРЫВНО","");
											rele|=0x20;
											SPI_syn_out(rele);

											wait=0;
											sc_up=0;
											UnHr = 0;
											UnMin = 0;
											UnSec = 0;
											prev_sec=61;
											//pow_test=power;
											power=0;
											FlashRoyal();
											ts_change=0;
											//while(!RTC_GetSubSecond()) {}
										}
    		      			wait=0;
    		      			

	      			    	if (RTC_DateTime.Seconds!=prev_sec)
	      			    	{
											placedchar(48+UnHr/10,11,1);
											placedchar(48+UnHr%10,17,1);
											placedchar(58,22,1);
											placedchar(48+UnMin/10,26,1);
											placedchar(48+UnMin%10,32,1);
											placedchar(58,37,1);
											placedchar(48+UnSec/10,41,1);
											placedchar(48+UnSec%10,47,1);
	      			    		UnSec1=UnSec;
	      			    		UnSec=PlusOne(UnSec,60);
	      			    		if(UnSec1==59)
	      			    		{
	      			    			UnMin1=UnMin;
	      			    			UnMin=PlusOne(UnMin,60);
	      			    			if(UnMin1==59) {UnHr=PlusOne(UnHr,24);}
	      			    		}
		      			    	prev_sec=RTC_DateTime.Seconds;
											rele|=0x20;
											SPI_syn_out(rele);
	      			    	}


    		      			break;
    		      		}
    		      		case 0x06:
    		      		{
    		      			if (sc_up)
    		      		    {
    		      				twolines("СУТОЧНЫЙ","РОСТ НА");
    		      				sc_up=0;
    		      				dot_per(Per/10,Per%10,1,42,state/0x10);
    		      		    }
    		      			break;
    		      		}
    		      		case 0x16:
    		      		{
    		      			if (sc_up)
    		      		    {
    		      				twolines("СУТОЧНЫЙ","РОСТ НА");
    		      				sc_up=0;
    		      		    }
										if((but_p||but_minus||but_plus)&&two_sec){dot_per(Per/10,Per%10,1,42,0);}
										Tick1=HAL_GetTick();
										if((Tick1-Tick2)>250)
										{
											blink=1;
											Tick2=Tick1;
										}
										if(blink)
										{
											blink=0;
											if(blon)
											{
												blon=0;
												dot_per(Per/10,Per%10,1,42,state/0x10);
											}
											else
											{
												blon=1;
												dot_per(Per/10,Per%10,1,42,0);
											}
										}												
    		      			break;
    		      		}
    		      		case 0x26:
    		      		{
    		      		    if (sc_up)
    		      		    {
    		      		    	twolines("СУТОЧНЫЙ","РОСТ НА");
    		      		    	sc_up=0;
    		      		    }
										if((but_p||but_minus||but_plus)&&two_sec){dot_per(Per/10,Per%10,1,42,0);}
										Tick1=HAL_GetTick();
										if((Tick1-Tick2)>250)
										{
											blink=1;
											Tick2=Tick1;
										}
										if(blink)
										{
											blink=0;
											if(blon)
											{
												blon=0;
												dot_per(Per/10,Per%10,1,42,state/0x10);
											}
											else
											{
												blon=1;
												dot_per(Per/10,Per%10,1,42,0);
											}
										}											
    		      		    break;
    		      		}
    		      		case 0x07:
    		      		{
    		      			if (sc_up)
    		      		    {
											rele&=0xDF;
											SPI_syn_out(rele);
    		      				twolines("КОРМ ОБЩЕЕ","ВРЕМЯ");
    		      				scr_time(CntMin,CntSec,1,35,state/0x10);
    		      				sc_up=0;
    		      		    }
    		      			break;
    		      		}
    		      		case 0x17:
    		      		{
    		      			if (sc_up)
    		      		  {
    		      				twolines("КОРМ ОБЩЕЕ","ВРЕМЯ");
    		      				sc_up=0;
    		      		  }
										if((but_p||but_minus||but_plus)&&two_sec){scr_time(CntMin,CntSec,1,35,0);}
										Tick1=HAL_GetTick();
										if((Tick1-Tick2)>250)
										{
											blink=1;
											Tick2=Tick1;
										}
										if(blink)
										{
											blink=0;
											if(blon)
											{
												blon=0;
												scr_time(CntMin,CntSec,1,35,state/0x10);
											}
											else
											{
												blon=1;
												scr_time(CntMin,CntSec,1,35,0);
											}
										}
    		      			break;
    		      		}
    		      		case 0x27:
    		      		{
    		      			if (sc_up)
    		      		    {
    		      				twolines("КОРМ ОБЩЕЕ","ВРЕМЯ");
    		      				sc_up=0;
    		      		    }
										if((but_p||but_minus||but_plus)  &&two_sec){scr_time(CntMin,CntSec,1,35,0);}
										Tick1=HAL_GetTick();
										if((Tick1-Tick2)>250)
										{
											blink=1;
											Tick2=Tick1;
										}
										if(blink)
										{
											blink=0;
											if(blon)
											{
												blon=0;
												scr_time(CntMin,CntSec,1,35,state/0x10);
											}
											else
											{
												blon=1;
												scr_time(CntMin,CntSec,1,35,0);
											}
										}
    		      			break;
    		      		}
    		      		case 0x08:
    		      		{
    		      			if (sc_up)
    		      		  {
    		      				twolines("ВКЛ ТЕСТ","");
    		      				sc_up=0;
    		      		  }
    		      			MOTOR_TIME1=MOTOR_TIME+1;
    		      			break;
    		      		}
    		      		case 0x18:
    		      		{
    		      			if (sc_up||blink)
    		      		  {
    		      				twolines("ВКЛ ТЕСТ","М:С");
    		      				sc_up=0;
    		      				blink=0;
    		      				rele|=0x20;
    		  	  	      SPI_syn_out(rele);
											flag20=0;
											wait=0;
											power=0;
											FlashRoyal();

    		      		  }
	      			    	if (RTC_DateTime.Seconds!=prev_sec)
	      			    	{
											rele|=0x20;
											SPI_syn_out(rele);
	      			    		scr_time_down(MOTOR_TIME1-1,1,31,0);
	      			    		MOTOR_TIME1--;
											MBMotorMin=MOTOR_TIME1/60;
											MBMotorSec=MOTOR_TIME1%60;
		      			    	prev_sec=RTC_DateTime.Seconds;
											
	      			    	}
	      			    	if ((MOTOR_TIME1==0)||(two_sec))
	      			    	{
	      			    		//power=pow_test;
	      			    		twenty_sec=1;
	      			    		rele&=0xDF;
	      			    		SPI_syn_out(rele);
	      			    	}
    		      			break;
    		      		}
    		      		case 0x09:
    		      		{
    		      			if (sc_up)
    		      			{
    		      				sc_up=0;
    		      				clearscreen(0);

											twolines("AVTOKOR","FEED v1.3L");
											placedchar(46,46,0);
											placedchar(77,42,0);
											placedchar(82,50,0);
											placedchar(85,56,0);

    		      			}
    		      			twenty_sec=0;
    		      			break;
    		      		}
									case 0x0A:
    		      		{
    		      			if (sc_up)
    		      			{
    		      				sc_up=0;
    		      				clearscreen(0);

											twolines("ПАРАМЕТРЫ","MODBUS");
    		      			}
    		      			break;
    		      		}
									case 0x1A:
    		      		{
    		      			if (sc_up)
    		      			{
    		      				sc_up=0;
    		      				clearscreen(0);
											oneline(0,"адрес");
											switch(MBSpeed)
											{
												case 0: {oneline(1,"скр   2400");break;}
												case 1: {oneline(1,"скр   4800");break;}
												case 2: {oneline(1,"скр   9600");break;}
												case 3: {oneline(1,"скр  14400");break;}
												case 4: {oneline(1,"скр  19200");break;}
												case 5: {oneline(1,"скр  38400");break;}
												case 6: {oneline(1,"скр  56000");break;}
												case 7: {oneline(1,"скр  57600");break;}
												case 8: {oneline(1,"скр 115200");break;}
												default: {oneline(1,"скрo115200");break;}
											}
										}
										Tick1=HAL_GetTick();
										if((Tick1-Tick2)>250)
										{
											blink=1;
											Tick2=Tick1;
										}
										if(blink)
										{
											blink=0;
											if(blon)
											{
												blon=0;
												scr_cnt(MBAdr,0,36,state/0x10);
											}
											else
											{
												blon=1;
												scr_cnt(MBAdr,0,36,0);
											}
										}
    		      			break;
    		      		}
									case 0x2A:
    		      		{
    		      			if (sc_up)
    		      			{
											NeedChangeSpeed=1;
    		      				sc_up=0;
    		      				clearscreen(0);
											oneline(0,"адрес");
											scr_cnt(MBAdr,0,36,0);
											switch(MBSpeed)
											{
												case 0: {oneline(1,"скр   2400");break;}
												case 1: {oneline(1,"скр   4800");break;}
												case 2: {oneline(1,"скр   9600");break;}
												case 3: {oneline(1,"скр  14400");break;}
												case 4: {oneline(1,"скр  19200");break;}
												case 5: {oneline(1,"скр  38400");break;}
												case 6: {oneline(1,"скр  56000");break;}
												case 7: {oneline(1,"скр  57600");break;}
												case 8: {oneline(1,"скр 115200");break;}
											}
										}
										Tick1=HAL_GetTick();
										if((Tick1-Tick2)>250)
										{
											blink=1;
											Tick2=Tick1;
										}
										if(blink)
										{
											blink=0;
											if(blon)
											{
												blon=0;
												oneline(1,"скр       ");
											}
											else
											{
												blon=1;
												switch(MBSpeed)
												{
													case 0: {oneline(1,"скр   2400");break;}
													case 1: {oneline(1,"скр   4800");break;}
													case 2: {oneline(1,"скр   9600");break;}
													case 3: {oneline(1,"скр  14400");break;}
													case 4: {oneline(1,"скр  19200");break;}
													case 5: {oneline(1,"скр  38400");break;}
													case 6: {oneline(1,"скр  56000");break;}
													case 7: {oneline(1,"скр  57600");break;}
													case 8: {oneline(1,"скр 115200");break;}
												}
											}
    		      			}
    		      			break;
    		      		}
    		      	}
    		  	  	if(wait)
    		  	  	{
    		  	  		//TIM22->CNT = 0;
								  //TIM22->CR1 |= TIM_CR1_CEN;
									//HAL_TIM_Base_Start_IT(&htim22);
									sec20=RTC_DateTime.Seconds;
									flag20=1;
									sec20cnt=0;
    		  	  		wait=0;
    		  	  	}
    		  	  	if (two_sec&&but_p)
    		  	  	{
    		  	  		ts_change=1;
    		  	  		state=state_tabl[state][two_sec];
    		  	  		if (state/10>0) 
									{
    		  	  			blink=1;
										if(state!=0x18&&state!=0x25&&state!=0x11)
										{blon=1;Tick2=HAL_GetTick();}
										if(state==0x1A||state==0x2A) {sc_up=1;}
									}
									else {blink=0;}
    		  	  		two_sec=0;
    		  	  	}
    		  	  	if (two_sec&&double_but)
    		  	  	{
    		  	  		ts_change=1;
    		  	  		state=0x25;
    		  	  		two_sec=0;
    		  	  		sc_up=1;
    		  	  		double_but=0;
    		  	  	}
    		  	  	if(MAYSLEEP)
    		  	  	{
    		  	  		MAYSLEEP=0;
    		  	  		NOTSLEEP=0;
    		  	  	}
    		  	  	if(twenty_sec)
    		  	  	{
									rele&=0xDF;
	      			    SPI_syn_out(rele);
									////TIM_Cmd(TIM3, DISABLE);
    		  	  		if(POWERFAIL)
    		  	  		{state=0x09;}
    		  	  		else {clearscreen(0);state=0x01;}
    		  	  		sc_up=1;
									flag20=0;
									sec20cnt=0;
    		  	  		twenty_sec=0;
    		  	  		two_sec=0;
    		  	  		prev_min=61;
    		  	  		prev_sec=61;
    		  	  		if (TIME_CH)
    		  	  		{
    		  	  			TIME_CH=0;
    		  	  			RTC_DateTime.Hours = GlobalHr;
    		  	  			RTC_DateTime.Minutes = GlobalMin;
    		  	  			RTC_DateTime.Seconds = 00;
    		  	  			HAL_RTC_SetTime(&hrtc, &RTC_DateTime, RTC_FORMAT_BIN);
    		  	  		}
    		  	  		if (VAR_CH)
    		  	  		{
    		  	  			VAR_CH=0;
    		  	  			FLSH_WRT_N=1;
    		  	  		}
    		  	  		
									//TIM22->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
									//HAL_TIM_Base_Stop_IT(&htim22);
									flag20=0;
    		  	  		MAYSLEEP=1;
    		  	  	}
    		  	  	if (FLSH_WRT_N)
    		  	  	{
    		  	  		FLSH_WRT_N=0;
    		  	  		if(!JUST_FIN)
    		  	  		{
    		  	  			NXT_TIME=100;
    		  	  		}
    		  	  		else
    		  	  		{
    		  	  			JUST_FIN=0;
									}
    		  	  		//?????? ?? ????
									FlashRoyal();

    		  	  		//?????? ??????????
									KormPlacement();
									sc_up=1;

    		  	  	}

    /*  }
    else
    {
    	TIM2->CNT = 0;
      TIM2->CR1 |= TIM_CR1_CEN;
			//HAL_TIM_Base_Start_IT(&htim2);
			HAL_SuspendTick();

			// Enter Sleep Mode , wake up is done once jumper is put between PA.12 (Arduino D2) and GND 
			HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

			//Resume Tick interrupt if disabled prior to SLEEP mode entry 
			HAL_ResumeTick();
    	//__WFI();
    }*/
		
	if(FlagModbGet)
	{
		uint8_t snd_cnt=0;
		uint8_t dt=0;
		FlagModbGet=0;
		if (res_buffer[0]==MBAdr)

		{

	  CRCCod=CRC16(res_buffer, (res_wr_index));	// Расчет СRC
	  if (CRCCod==0)								// Проверка CRC в посылке
	  {											// Если верно - выполняем действие
			
		  switch (res_buffer[1]) 
			{
				case 0x03:							// Чтение регистров
				{
					if ((res_buffer[2]<1) && ((res_buffer[3]+res_buffer[4]*256+res_buffer[5]-1)<20))
					{
						write_buffer[0]=res_buffer[0];					// Адрес устройства
						write_buffer[1]=0x03;						// Та-же функция
						write_buffer[2]=res_buffer[5]*2;			// Счетчик байт

						for (i=0; i<res_buffer[5]; i++)				// Значения регистров
						{
							write_buffer[4+(2*i)]=(reg_MB[res_buffer[2]*0x100+res_buffer[3]+i])%256;		// Младший байт (2-ой)
							write_buffer[3+(2*i)]=0;//(reg_MB[res_buffer[2]*0x100+res_buffer[3]+i])/256;	// Старший байт (1-ый)
						}

						snd_cnt=write_buffer[2]+3;			
					}
					else if ((res_buffer[2]==1) && ((res_buffer[3]+res_buffer[4]*256+res_buffer[5]-1)<3))
					{
						write_buffer[0]=res_buffer[0];					// Адрес устройства
						write_buffer[1]=0x03;						// Та-же функция
						write_buffer[2]=res_buffer[5]*2;			// Счетчик байт
						
						reg_MB2[0]=reg_MB[0];
						reg_MB2[1]=reg_MB[2];
						reg_MB2[2]=reg_MB[2];
						for (i=0; i<res_buffer[5]; i++)				// Значения регистров
						{
							write_buffer[4+(2*i)]=(reg_MB2[res_buffer[3]+i]);		// Младший байт (2-ой)
							write_buffer[3+(2*i)]=0;//(reg_MB[res_buffer[2]*0x100+res_buffer[3]+i])/256;	// Старший байт (1-ый)
						}

						snd_cnt=write_buffer[2]+3;		
					}
					else
					{
						write_buffer[0]=res_buffer[0];					// адрес блока
						write_buffer[1]=0x83;						// та-же функция + взведенный бит ошибки
						write_buffer[2]=0x02;				// код ошибки - недопустимый адрес
						snd_cnt=3;
					}
					break;
				}
				case 0x06:						//запись регистра
				{
						//if ((res_buffer[2]*0x100+res_buffer[3]==1)||(res_buffer[2]*0x100+res_buffer[3]==2))//если возможна запись регистра
						if ((res_buffer[2]==0)&&(res_buffer[3]>5)&&(res_buffer[3]<20))
						{
							if (state != 0x00 || state != 0x18)
							{
								if((res_buffer[4]==0)&&(res_buffer[5]<MaxS[res_buffer[3]]))
								{
									reg_MB[res_buffer[3]]=res_buffer[5];
									if(res_buffer[3]==0x11)
									{
										NeedChangeSpeed=1;
										//dt=res_buffer[5];
									}
									if(res_buffer[3]==0x06 || res_buffer[3]==0x07)
									{
										RTC_DateTime.Hours = GlobalHr;
										RTC_DateTime.Minutes = GlobalMin;
										RTC_DateTime.Seconds = 00;
										HAL_RTC_SetTime(&hrtc, &RTC_DateTime, RTC_FORMAT_BIN);
									}
									
									power=0;
									FLSH_WRT_N=1;
									//FlashRoyal();
									sc_up=1;
									write_buffer[0]=res_buffer[0];					// адрес блока
									write_buffer[1]=0x06;						// та-же функция
									write_buffer[2]=res_buffer[2];				// те же данные
									write_buffer[3]=res_buffer[3];
									write_buffer[4]=res_buffer[4];
									write_buffer[5]=res_buffer[5];
									snd_cnt=6;
								}
								else
								{
									write_buffer[0]=res_buffer[0];					// адрес устройства
									write_buffer[1]=0x16;						// та-же функция
									write_buffer[2]=0x03;				// код ошибки - недопустимое значение
									snd_cnt=3;						
								}
							}
							else
							{
								write_buffer[0]=res_buffer[0];					// адрес устройства
								write_buffer[1]=0x16;						// та-же функция
								write_buffer[2]=0x06;				// код ошибки - недопустимое значение
								snd_cnt=3;						
							}
						}
						else if ((res_buffer[2]==1)&&(res_buffer[3]==0)&&(res_buffer[4]==0)&&(res_buffer[5]<2))
						{
							MBError=errorsGet();
							if (MBError==0)
							{		
								power=res_buffer[5];
								FLSH_WRT_N=1;
								sc_up=1;
								write_buffer[0]=res_buffer[0];					// адрес блока
								write_buffer[1]=0x06;						// та-же функция
								write_buffer[2]=res_buffer[2];				// те же данные
								write_buffer[3]=res_buffer[3];
								write_buffer[4]=res_buffer[4];
								write_buffer[5]=res_buffer[5];
								snd_cnt=6;
							}
							else
							{
								power=0;
								FLSH_WRT_N=1;
								sc_up=1;
								write_buffer[0]=res_buffer[0];					// адрес устройства
								write_buffer[1]=0x16;						// та-же функция
								write_buffer[2]=0x03;				// код ошибки
								snd_cnt=3;
							}
						}
						else if ((res_buffer[2]==1)&&(res_buffer[3]==1)&&(res_buffer[4]==0)&&(res_buffer[5]<2))
						{
							write_buffer[0]=res_buffer[0];					// адрес блока
							write_buffer[1]=0x06;						// та-же функция
							write_buffer[2]=res_buffer[2];				// те же данные
							write_buffer[3]=res_buffer[3];
							write_buffer[4]=res_buffer[4];
							write_buffer[5]=res_buffer[5];
							snd_cnt=6;
							sc_up=1;
							if(res_buffer[5])
							{
								MOTOR_TIME1=MOTOR_TIME+1;
							}
							else
							{
								MOTOR_TIME1=0;
							}
							state=0x18;
						}
						else if ((res_buffer[2]==1)&&(res_buffer[3]==2)&&(res_buffer[4]==0)&&(res_buffer[5]<2))
						{
							write_buffer[0]=res_buffer[0];					// адрес блока
							write_buffer[1]=0x06;						// та-же функция
							write_buffer[2]=res_buffer[2];				// те же данные
							write_buffer[3]=res_buffer[3];
							write_buffer[4]=res_buffer[4];
							write_buffer[5]=res_buffer[5];
							snd_cnt=6;
							if(res_buffer[5])
							{
								MOTOR_TIME1=MBCustomMin*60+MBCustomSec;
						
							}
							else
							{
								MOTOR_TIME1=0;
							}
							sc_up=1;
							state=0x18;
						}
						else if ((res_buffer[2]==2)&&(res_buffer[3]==0)&&(res_buffer[4]==0)&&(res_buffer[5]==0))
						{
							write_buffer[0]=res_buffer[0];					// адрес блока
							write_buffer[1]=0x06;						// та-же функция
							write_buffer[2]=res_buffer[2];				// те же данные
							write_buffer[3]=res_buffer[3];
							write_buffer[4]=res_buffer[4];
							write_buffer[5]=res_buffer[5];
							two_sec=1;
							double_but=1;
							snd_cnt=6;
							sc_up=1;
							//state=0x18;
						}
						else
						{
							write_buffer[0]=res_buffer[0];					// адрес устройства
							write_buffer[1]=0x16;						// та-же функция
							write_buffer[2]=0x02;				// код ошибки - недопустимый адрес
							snd_cnt=3;
						}
					break;
				}
				default:
				{
					write_buffer[0]=res_buffer[0];					// адрес блока
					write_buffer[1]=res_buffer[1]+0x80;						// та-же функция + взведенный бит ошибки
					write_buffer[2]=0x01;				// код ошибки - недопустимая функция

					snd_cnt=3;

					break;
				}
			}
			CRCCod=CRC16(write_buffer, snd_cnt);				// расчет CRC

			write_buffer[snd_cnt] = CRCCod & 0x00FF;			// мл. байт CRC
			write_buffer[snd_cnt+1] = CRCCod >> 8;				// ст. байт CRC
			HAL_UART_Transmit(&huart2,write_buffer,snd_cnt+2,100);
	  }
	}
	res_wr_index=0;
  }
	if(NeedChangeSpeed && state!=0x2A)
	{
		NeedChangeSpeed=0;
		USART2_ReInit(MBSpeed);
		MX_TIM22_Init(MBSpeed);
	}

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */

void SystemClock_Config(void) 
{ 

RCC_OscInitTypeDef RCC_OscInitStruct; 
RCC_ClkInitTypeDef RCC_ClkInitStruct; 
RCC_PeriphCLKInitTypeDef PeriphClkInit; 

/**Configure the main internal regulator output voltage 
*/ 
__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1); 

/**Configure LSE Drive Capability 
*/ 
HAL_PWR_EnableBkUpAccess(); 

__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW); 

/**Initializes the CPU, AHB and APB busses clocks 
*/ 
RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE; 
RCC_OscInitStruct.LSEState = RCC_LSE_ON; 
RCC_OscInitStruct.HSIState = RCC_HSI_DIV4; 
RCC_OscInitStruct.HSICalibrationValue = 16; 
RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; 
RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI; 
RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4; 
RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2; 
if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) 
{ 
_Error_Handler(__FILE__, __LINE__); 
} 

/**Initializes the CPU, AHB and APB busses clocks 
*/ 
RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK 
|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2; 
RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; 
RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; 
RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; 

if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) 
{ 
_Error_Handler(__FILE__, __LINE__); 
} 

PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_RTC; 
PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1; 
PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE; 
if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) 
{ 
_Error_Handler(__FILE__, __LINE__); 
} 

/**Configure the Systick interrupt time 
*/ 
HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000); 

/**Configure the Systick 
*/ 
HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK); 

/* SysTick_IRQn interrupt configuration */ 
HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0); 
}

void SystemClock_Config2(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Configure LSE Drive Capability 
    */
  HAL_PWR_EnableBkUpAccess();

  //op__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  //opRCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
	//opRCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  //opPeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_6B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

    /**Initialize RTC Only 
    */

  hrtc.Instance = RTC;
	if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
		
		FirstStart=1;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 3 */

  /* USER CODE END RTC_Init 3 */

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 4 */
    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
  }
  /* USER CODE END RTC_Init 4 */

    /**Enable the WakeUp 
    */
  /*if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 0x00C3, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }*/
  /* USER CODE BEGIN RTC_Init 5 */

  /* USER CODE END RTC_Init 5 */

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA13 PA14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  /*HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);*/

}

/* USER CODE BEGIN 4 */

/*void USART2_IRQHandler(void)
{

scr_cnt (res_wr_index, 0, 0, 0);

  HAL_UART_IRQHandler(&huart2);

}*/


void USART2_IRQHandler(void)
{
	if((USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
	{	
		
        TIM22->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
        TIM22->CNT=0;
        res_buffer[res_wr_index]=(uint8_t)(USART2->RDR);
        if(res_wr_index<19)
        {
            res_wr_index++;			
        }
        TIM22->CR1 |= TIM_CR1_CEN; 
	}
	HAL_UART_IRQHandler(&huart2);
}


void TIM22_IRQHandler(void)
{
	
	TIM22->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
	HAL_TIM_IRQHandler(&htim22);
	FlagModbGet=1;
}


void TIM21_IRQHandler(void)
{
	TIM21->SR &= ~TIM_SR_UIF;
	if(flag_WT)
	{
		flag_WT=0;
		TIM21->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
	}
  else
	{
    two_sec=1;
	}
}


void TIM2_IRQHandler(void)
{
    TIM2->SR &= ~TIM_SR_UIF;
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    TIM2->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
}


/*void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	
  
  
}*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */

  /* USER CODE END EXTI0_1_IRQn 0 */
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */
				//__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

	/*if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET)
		{
			__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

			uu++;
	scr_cnt (uu, 1, 50, 0);
		}
	else
		{
		__HAL_RCC_PWR_CLK_ENABLE();
    // Enter Stop Mode 
								uu++;
	scr_cnt (uu, 0, 50, 0);
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
						//__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

		WakeRestart();


		}*/
	
  /* USER CODE END EXTI0_1_IRQn 1 */
}

void RTC_IRQHandler(void)
{
	HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
	//__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	/*WakeRestart();
	uu++;
	scr_cnt (uu, 1, 50, 0);
	HAL_ADC_Start(&hadc);*/
	
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc){
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

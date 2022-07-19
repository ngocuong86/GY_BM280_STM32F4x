/*******************************************************************************
 *				 _ _                                             _ _
				|   |                                           (_ _)
				|   |        _ _     _ _   _ _ _ _ _ _ _ _ _ _   _ _
				|   |       |   |   |   | |    _ _     _ _    | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |_ _ _  |   |_ _|   | |   |   |   |   |   | |   |
				|_ _ _ _ _| |_ _ _ _ _ _| |_ _|   |_ _|   |_ _| |_ _|
								(C)2022 Lumi
 * Copyright (c) 2022
 * Lumi, JSC.
 * All Rights Reserved
 *
 * File name: I2C-Interface.c
 *
 * Description:
 * file header for I2C1 Interface
 * Author: CuongNV
 *
 * Last Changed By:  $Author: cuongnv $
 * Revision:         $Revision: $
 * Last Changed:     $Date: $Jul 15, 2022
 ******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "stm32f401re_gpio.h"
#include "stm32f401re_spi.h"
#include "stm32f401re_rcc.h"
#include "SPI1-Interface.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define SPI1_CS_PORT			GPIOB
#define SPI1_CS_PIN				GPIO_Pin_6
#define SPI1_RST_PORT			GPIOC
#define SPI1_RST_PIN			GPIO_Pin_7
#define SPI1_MOSI_PORT			GPIOA
#define SPI1_MOSI_PIN			GPIO_Pin_7
#define SPI1_SCK_PORT			GPIOA
#define SPI1_SCK_PIN			GPIO_Pin_5
#define SPI1_RS_PORT			GPIOA
#define SPI1_RS_PIN				GPIO_Pin_9
#define SPI1_ENABLE_PORT		GPIOB
#define SPI1_ENABLE_PIN			GPIO_Pin_10
#define SPI1_MODE_PORT			GPIOA
#define SPI1_MODE_PIN			GPIO_Pin_8
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/******************************************************************************
* @func					SPI1_Init
* @brief				This func Init SPI1 mode master
* @param				none
* @return				none
* @Note					none
*/
void_t SPI1Init(void_t){
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	// Enable clock for GPIOA - GPIOB - GPIOC
	/* GPIOA, GPIOB and GPIOC Clocks enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN | SPI1_MOSI_PIN | SPI1_RS_PIN | SPI1_MODE_PIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI1_CS_PIN | SPI1_ENABLE_PIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI1_RST_PIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	 //enable peripheral clock SPI1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	// set to hafl deplex mode, seperate MOSI Lines
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;

	// Use SPI1 as slave mode
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;

	// One packet of data is 8 bits wide
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;

	// Clock is low when idle
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;

	// Data sampled at first edge
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;

	//SPI frequency is APB2 frequency
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;

	// Set NSS us software
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;

	// data is trasmitted MSB first
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;

	SPI_Init(SPI1, &SPI_InitStructure);

	// Enable SPI1
	SPI_Cmd(SPI1, ENABLE);
}


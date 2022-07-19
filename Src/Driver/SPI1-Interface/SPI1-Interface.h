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
 * File name: I2C-Interface.h
 *
 * Description:
 * file header for I2C1 Interface
 * Author: CuongNV
 *
 * Last Changed By:  $Author: cuongnv $
 * Revision:         $Revision: $
 * Last Changed:     $Date: $Jul 15, 2022
 ******************************************************************************/

#ifndef DRIVER_SPI1_INTERFACE_SPI1_INTERFACE_H_
#define DRIVER_SPI1_INTERFACE_SPI1_INTERFACE_H_
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "typedefs.h"
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
void_t SPI1Init(void_t);

#endif /* DRIVER_SPI1_INTERFACE_SPI1_INTERFACE_H_ */

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

#ifndef I2C_INTERFACE_I2C_INTERFACE_H_
#define I2C_INTERFACE_I2C_INTERFACE_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "typedefs.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define I2Cx_RCC				RCC_APB1Periph_I2C1
#define I2C_GPIO_RCC		    RCC_AHB1Periph_GPIOB
#define I2Cx_SENSOR				I2C1
#define I2C_GPIO				GPIOB
#define I2C_PIN_SCL				GPIO_Pin_8
#define I2C_PIN_SDA				GPIO_Pin_9
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
* @func					I2C1_Init
* @brief				This func Init I2C1_Init mode master
* @param				none
* @return				none
* @Note					none
*/
void_t I2C1Init(void_t);
/**
  ******************************************************************************
  *	@brief	Generate I2C start condition
  * @param	None
  * @retval	None
  ******************************************************************************
  */
void_t I2C1Start(void_t);
/**
  ******************************************************************************
  *	@brief	Generate I2C stop condition
  * @param	None
  * @retval	None
  ******************************************************************************
  */
void_t I2C1Stop(void_t);
/**
  ******************************************************************************
  *	@brief		Write slave address to I2C bus
	* @param	Slave address
	* @param	I2C direction (transmitter or receiver)
  * @retval	None
  ******************************************************************************
  */
void_t I2C1AddressDirection(u8_t byAddress, u8_t byDirection);
/**
  ******************************************************************************
  *	@brief	Transmit one byte to I2C bus
  * @param	Data byte to transmit
  * @retval	None
  ******************************************************************************
  */
void_t I2C1Transmit(u8_t byData);
/**
  ******************************************************************************
  *	@brief	Receive data byte from I2C bus, then return ACK
  * @param	None
  * @retval	Received data byte
  ******************************************************************************
  */
u8_t I2C1ReceiveAck(void_t);
/**
  ******************************************************************************
  *	@brief	Receive data byte from I2C bus, then return NACK
  * @param	None
  * @retval	Received data byte
  ******************************************************************************
  */
u8_t I2C1ReceiveNack(void_t);
#endif /* I2C_INTERFACE_I2C_INTERFACE_H_ */

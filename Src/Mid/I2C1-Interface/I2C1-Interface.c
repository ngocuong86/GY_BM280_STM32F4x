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
 * File name: temperature-pressure-altitude.h
 *
 * Description:
 *
 * Author: CuongNV
 *
 * Last Changed By:  $Author: cuongnv $
 * Revision:         $Revision: $
 * Last Changed:     $Date: $Jul 15, 2022
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "stm32f401re_rcc.h"
#include "stm32f401re_gpio.h"
#include "stm32f401re_i2c.h"
#include "../I2C1-Interface/I2C1-Interface.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define I2Cx_RCC				RCC_APB1Periph_I2C1
#define I2Cx_SENSOR				I2C1
#define I2C_GPIO_RCC		    RCC_AHB1Periph_GPIOB
#define I2C_GPIO				GPIOB
#define I2C_PIN_SDA				GPIO_Pin_8
#define I2C_PIN_SCL				GPIO_Pin_9
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
/******************************************************************************/

/******************************************************************************
* @func					I2C1_Init
* @brief				This func Init I2C1_Init mode master
* @param				none
* @return				none
* @Note					none
*/
void_t I2C1Init(void_t){
	I2C_InitTypeDef  I2C_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_APB1PeriphClockCmd(I2Cx_RCC, ENABLE);
	RCC_AHB1PeriphClockCmd(I2C_GPIO_RCC, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_InitStruct.GPIO_Pin = I2C_PIN_SCL | I2C_PIN_SDA;
	GPIO_Init(I2C_GPIO, &GPIO_InitStruct);

	// Connect PA8 to I2C1 SCL
	GPIO_PinAFConfig(I2C_GPIO, GPIO_PinSource8, GPIO_AF_I2C1);

	// Connect PC9 to I2C1 SDA
	GPIO_PinAFConfig(I2C_GPIO, GPIO_PinSource9, GPIO_AF_I2C1);

	I2C_InitStruct.I2C_ClockSpeed = 400000;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

	I2C_Init(I2Cx_SENSOR, &I2C_InitStruct);
	I2C_Cmd(I2Cx_SENSOR, ENABLE);
}
/**
  ******************************************************************************
  *	@brief	Generate I2C stop condition
  * @param	None
  * @retval	None
  ******************************************************************************
  */
void_t I2C1Stop(void)
{
	// Generate I2C stop condition
	I2C_GenerateSTOP(I2Cx_SENSOR, ENABLE);
}
/**
  ******************************************************************************
  *	@brief	Generate I2C start condition
  * @param	None
  * @retval	None
  ******************************************************************************
  */
void_t I2C1Start(void)
{
	// Wait until I2Cx is not busy anymore
	while (I2C_GetFlagStatus(I2Cx_SENSOR, I2C_FLAG_BUSY));

	// Generate start condition
	I2C_GenerateSTART(I2Cx_SENSOR, ENABLE);

	// Wait for I2C EV5.
	// It means that the start condition has been correctly released
	// on the I2C bus (the bus is free, no other devices is communicating))
	while (!I2C_CheckEvent(I2Cx_SENSOR, I2C_EVENT_MASTER_MODE_SELECT));
}
/**
  ******************************************************************************
  *	@brief	Write slave address to I2C bus
	* @param	Slave address
	* @param	I2C direction (transmitter or receiver)
  * @retval	None
  ******************************************************************************
  */
void_t I2C1AddressDirection(u8_t byAddress, u8_t byDirection)
{
	// Send slave address
	I2C_Send7bitAddress(I2Cx_SENSOR, byAddress, byDirection);

	// Wait for I2C EV6
	// It means that a slave acknowledges his address
	if (byDirection == I2C_Direction_Transmitter)
	{
		while (!I2C_CheckEvent(I2Cx_SENSOR, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if (byDirection == I2C_Direction_Receiver)
	{
		while (!I2C_CheckEvent(I2Cx_SENSOR, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}
/**
  ******************************************************************************
  *	@brief	Transmit one byte to I2C bus
  * @param	Data byte to transmit
  * @retval	None
  ******************************************************************************
  */
void_t I2C1Transmit(u8_t byData)
{
	// Send data byte
	I2C_SendData(I2Cx_SENSOR, byData);
	// Wait for I2C EV8_2.
	// It means that the data has been physically shifted out and
	// output on the bus)
	while (!I2C_CheckEvent(I2Cx_SENSOR, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}
/**
  ******************************************************************************
  *	@brief	Receive data byte from I2C bus, then return ACK
  * @param	None
  * @retval	Received data byte
  ******************************************************************************
  */
u8_t I2C1ReceiveAck(void_t)
{
	// Enable ACK of received data
	I2C_AcknowledgeConfig(I2Cx_SENSOR, ENABLE);
	// Wait for I2C EV7
	// It means that the data has been received in I2C data register
	while (!I2C_CheckEvent(I2Cx_SENSOR, I2C_EVENT_MASTER_BYTE_RECEIVED));

	// Read and return data byte from I2C data register
	return I2C_ReceiveData(I2Cx_SENSOR);
}

/**
  ******************************************************************************
  *	@brief	Receive data byte from I2C bus, then return NACK
  * @param	None
  * @retval	Received data byte
  ******************************************************************************
  */
u8_t I2C1ReceiveNack(void_t)
{
	// Disable ACK of received data
	I2C_AcknowledgeConfig(I2Cx_SENSOR, DISABLE);
	// Wait for I2C EV7
	// It means that the data has been received in I2C data register
	while (!I2C_CheckEvent(I2Cx_SENSOR, I2C_EVENT_MASTER_BYTE_RECEIVED));

	// Read and return data byte from I2C data register
	return I2C_ReceiveData(I2Cx_SENSOR);
}

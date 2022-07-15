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
#include <math.h>
#include <stdio.h>
#include "stm32f401re_gpio.h"
#include "stm32f401re_rcc.h"
#include "stm32f401re_i2c.h"
#include "stm32f401re_spi.h"
#include "Src/Mid/temperature-pressure-altitude/temperature-pressure-altitude.h"
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

#define I2Cx_RCC				RCC_APB1Periph_I2C1
#define I2Cx_SENSOR				I2C1
#define I2C_GPIO_RCC		    RCC_AHB1Periph_GPIOB
#define I2C_GPIO				GPIOB
#define I2C_PIN_SDA				GPIO_Pin_9
#define I2C_PIN_SCL				GPIO_Pin_8

#define I2C_MODE 0
#define SPI_MODE 1

//Register names:
#define BME280_DIGT1_LSB_REG			0x88
#define BME280_DIGT1_MSB_REG			0x89
#define BME280_DIGT2_LSB_REG			0x8A
#define BME280_DIGT2_MSB_REG			0x8B
#define BME280_DIGT3_LSB_REG			0x8C
#define BME280_DIGT3_MSB_REG			0x8D
#define BME280_DIGP1_LSB_REG			0x8E
#define BME280_DIGP1_MSB_REG			0x8F
#define BME280_DIGP2_LSB_REG			0x90
#define BME280_DIGP2_MSB_REG			0x91
#define BME280_DIGP3_LSB_REG			0x92
#define BME280_DIGP3_MSB_REG			0x93
#define BME280_DIGP4_LSB_REG			0x94
#define BME280_DIGP4_MSB_REG			0x95
#define BME280_DIGP5_LSB_REG			0x96
#define BME280_DIGP5_MSB_REG			0x97
#define BME280_DIGP6_LSB_REG			0x98
#define BME280_DIGP6_MSB_REG			0x99
#define BME280_DIGP7_LSB_REG			0x9A
#define BME280_DIGP7_MSB_REG			0x9B
#define BME280_DIGP8_LSB_REG			0x9C
#define BME280_DIGP8_MSB_REG			0x9D
#define BME280_DIGP9_LSB_REG			0x9E
#define BME280_DIGP9_MSB_REG			0x9F
#define BME280_DIGH1_REG				0xA1
#define BME280_CHIP_ID_REG				0xD0 //Chip ID
#define BME280_RST_REG					0xE0 //Softreset Reg
#define BME280_DIGH2_LSB_REG			0xE1
#define BME280_DIGH2_MSB_REG			0xE2
#define BME280_DIGH3_REG				0xE3
#define BME280_DIGH4_MSB_REG			0xE4
#define BME280_DIGH4_LSB_REG			0xE5
#define BME280_DIGH5_MSB_REG			0xE6
#define BME280_DIGH6_REG				0xE7
#define BME280_CTRL_HUMIDITY_REG		0xF2 //Ctrl Humidity Reg
#define BME280_STAT_REG					0xF3 //Status Reg
#define BME280_CTRL_MEAS_REG			0xF4 //Ctrl Measure Reg
#define BME280_CONFIG_REG				0xF5 //Configuration Reg
#define BME280_MEASUREMENTS_REG			0xF7 //Measurements register start
#define BME280_PRESSURE_MSB_REG			0xF7 //Pressure MSB
#define BME280_PRESSURE_LSB_REG			0xF8 //Pressure LSB
#define BME280_PRESSURE_XLSB_REG		0xF9 //Pressure XLSB
#define BME280_TEMPERATURE_MSB_REG		0xFA //Temperature MSB
#define BME280_TEMPERATURE_LSB_REG		0xFB //Temperature LSB
#define BME280_TEMPERATURE_XLSB_REG		0xFC //Temperature XLSB
#define BME280_HUMIDITY_MSB_REG			0xFD //Humidity MSB
#define BME280_HUMIDITY_LSB_REG			0xFE //Humidity LSB

typedef struct
{


	//Main ierface and mode g_SensorSettings
    u8_t byComInterface;
    u8_t byI2CAddress;
    u8_t byChipSelectPin;
	//Deprecated g_SensorSettings
	u8_t byRunMode;
	u8_t byTimeStandby;
	u8_t byFilter;
	u8_t byTempOverSample;
	u8_t byPressOverSample;
	u8_t byHumidOverSample;
    float_t fTempCorrection; // correction of temperature - added to the result
}BME280SensorSetting_t;
typedef struct
{
	u16_t byDigT1;
	i16_t ibDigT2;
	i16_t ibDigT3;

	u16_t byDigP1;
	i16_t ibDigP2;
	i16_t ibDigP3;
	i16_t ibDigP4;
	i16_t ibDigP5;
	i16_t ibDigP6;
	i16_t ibDigP7;
	i16_t ibDigP8;
	i16_t ibDigP9;

	u8_t byDigH1;
	i16_t ibDigH2;
	u8_t byDigH3;
	i16_t ibDigH4;
	i16_t ibDigH5;
	i8_t ibDigH6;

}SensorCalibration_t;
typedef struct
{
	float_t fTemperature;
	float_t fPressure;
	float_t fHumidity;
}BME280SensorMeasurements_t;

enum sensor_sampling {
    SAMPLING_NONE = 0b000,
    SAMPLING_X1 = 0b001,
    SAMPLING_X2 = 0b010,
    SAMPLING_X4 = 0b011,
    SAMPLING_X8 = 0b100,
    SAMPLING_X16 = 0b101
  };

  /**************************************************************************/
  /*!
      @brief  power modes
  */
  /**************************************************************************/
  enum{
    MODE_SLEEP = 0b00,
    MODE_FORCED = 0b01,
    MODE_NORMAL = 0b11
  }SensorMode;

  /**************************************************************************/
  /*!
      @brief  filter values
  */
  /**************************************************************************/
  enum {
    FILTER_OFF = 0b000,
    FILTER_X2 = 0b001,
    FILTER_X4 = 0b010,
    FILTER_X8 = 0b011,
    FILTER_X16 = 0b100
  }SensorFilter;

  /**************************************************************************/
  /*!
      @brief  standby duration in ms
  */
  /**************************************************************************/
  enum {
    STANDBY_MS_0_5 = 0b000,
    STANDBY_MS_10 = 0b110,
    STANDBY_MS_20 = 0b111,
    STANDBY_MS_62_5 = 0b001,
    STANDBY_MS_125 = 0b010,
    STANDBY_MS_250 = 0b011,
    STANDBY_MS_500 = 0b100,
    STANDBY_MS_1000 = 0b101
  }StandbyDuration;
  enum{
	  ZERO_SAMPLE_VALUE = 0,
	  ONE_SAMPLE_VALUE = 1,
	  TWO_SAMPLE_VALUE = 2,
	  FOUR_SAMPLE_VALUE = 4,
	  EIGHT_SAMPLE_VALUE = 8,
	  SIXTEEN_SAMPLE_VALUE = 16
  };
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void_t I2C1Init(void_t);
static void_t SPI1Init(void_t);
static u8_t getMode(static void_t); //Get the current mode: sleep, forced, or normal
static void_t setMode(u8_t byMode); //Set the current mode



static void_t setI2CAddress(u8_t byI2CAddress); //Set the address the library should use to communicate. Use if address jumper is closed (0x76).

static void_t setReferencePressure(float_t byRefPressure); //Allows user to set local sea level reference pressure
static float_t getReferencePressure();

static bool_t isMeasuring(void_t); //Returns true while the device is taking measurement
//Temperature related methods
static void_t setTemperatureCorrection(float_t fCorr);
static float_t readTempC(void_t );
static float_t readTempF(void_t );
static float_t readTempFromBurst(u8_t byBuffer[]);
//ReadRegisterRegion takes a u8 array address as input and reads
//a chunk of memory io that array.
static void_t readRegisterRegion(u8_t*, u8_t, u8_t );
//readRegister reads one register
static u8_t readRegister(u8_t);
//Reads two regs, LSByte then MSByte order, and concatenates them
//Used for two-byte reads
static u16_t readRegisteri16( u8_t byOffset );
//Writes a byte;
static void_t writeRegister(u8_t, u8_t);

static u8_t checkSampleValue(u8_t byUserValue); //Checks for valid over sample values
static void readTempCFromBurst(u8_t byBuffer[], BME280SensorMeasurements_t *pmeasurements);
static void readTempFFromBurst(u8_t byBuffer[], BME280SensorMeasurements_t *pmeasurements);

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/******************************************************************************/
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
float_t g_fReferencePressure = 101325.0;
BME280SensorSetting_t g_SensorSettings;
SensorCalibration_t g_Calibration;
i32_t g_ibTempfine;
/******************************************************************************
* @func					I2C1_Init
* @brief				This func Init I2C1_Init mode master
* @param				none
* @return				none
* @Note					none
*/
static void_t I2C_Init(static void_t){
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
/******************************************************************************
* @func					SPI1_Init
* @brief				This func Init SPI1 mode master
* @param				none
* @return				none
* @Note					none
*/
static void_t SPI1_Init(static void_t){
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
/******************************************************************************
* @func					BMP280_Init
* @brief				This func Init BMP280_Init
* @param				none
* @return				none
* @Note					none
*/
static u8_t BMP280_Init(void_t){
	g_SensorSettings.byComInterface = I2C_MODE; //Default to I2C
	g_SensorSettings.byI2CAddress = 0x77; //Default, jumper open is 0x77

//	g_SensorSettings.chipSelectPin = 10; //Select CS pin for SPI

	//These are deprecated g_SensorSettings
	g_SensorSettings.byRunMode = 3; //Normal/Run
	g_SensorSettings.byTimeStandby = 0; //0.5ms
	g_SensorSettings.byFilter = 0; //Filter off
	g_SensorSettings.byTempOverSample = 1;
	g_SensorSettings.byPressOverSample = 1;
	g_SensorSettings.byHumidOverSample = 1;
	g_SensorSettings.fTempCorrection = 0.f; // correction of temperature - added to the result

	u8_t chipID = readRegister(BME280_CHIP_ID_REG); //Should return 0x60 or 0x58
	if(chipID != 0x58 && chipID != 0x60) // Is this BMP or BME?
	return(chipID); //This is not BMP nor BME!

	//Reading all compensation data, range 0x88:A1, 0xE1:E7
	g_Calibration.byDigT1 = ((u16_t)((readRegister(BME280_DIG_T1_MSB_REG) << 8) + readRegister(BME280_DIG_T1_LSB_REG)));
	g_Calibration.ibDigT2 = ((i16_t)((readRegister(BME280_DIG_T2_MSB_REG) << 8) + readRegister(BME280_DIG_T2_LSB_REG)));
	g_Calibration.ibDigT3 = ((i16_t)((readRegister(BME280_DIG_T3_MSB_REG) << 8) + readRegister(BME280_DIG_T3_LSB_REG)));

	g_Calibration.byDigP1 = ((u16_t)((readRegister(BME280_DIG_P1_MSB_REG) << 8) + readRegister(BME280_DIG_P1_LSB_REG)));
	g_Calibration.ibDigP2 = ((i16_t)((readRegister(BME280_DIG_P2_MSB_REG) << 8) + readRegister(BME280_DIG_P2_LSB_REG)));
	g_Calibration.ibDigP3 = ((i16_t)((readRegister(BME280_DIG_P3_MSB_REG) << 8) + readRegister(BME280_DIG_P3_LSB_REG)));
	g_Calibration.ibDigP4 = ((i16_t)((readRegister(BME280_DIG_P4_MSB_REG) << 8) + readRegister(BME280_DIG_P4_LSB_REG)));
	g_Calibration.ibDigP5 = ((i16_t)((readRegister(BME280_DIG_P5_MSB_REG) << 8) + readRegister(BME280_DIG_P5_LSB_REG)));
	g_Calibration.ibDigP6 = ((i16_t)((readRegister(BME280_DIG_P6_MSB_REG) << 8) + readRegister(BME280_DIG_P6_LSB_REG)));
	g_Calibration.ibDigP7 = ((i16_t)((readRegister(BME280_DIG_P7_MSB_REG) << 8) + readRegister(BME280_DIG_P7_LSB_REG)));
	g_Calibration.ibDigP8 = ((i16_t)((readRegister(BME280_DIG_P8_MSB_REG) << 8) + readRegister(BME280_DIG_P8_LSB_REG)));
	g_Calibration.ibDigP9 = ((i16_t)((readRegister(BME280_DIG_P9_MSB_REG) << 8) + readRegister(BME280_DIG_P9_LSB_REG)));

	g_Calibration.byDigH1 = ((u8_t)(readRegister(BME280_DIG_H1_REG)));
	g_Calibration.ibDigH2 = ((i16_t)((readRegister(BME280_DIG_H2_MSB_REG) << 8) + readRegister(BME280_DIG_H2_LSB_REG)));
	g_Calibration.byDigH3 = ((u8_t)(readRegister(BME280_DIG_H3_REG)));
	g_Calibration.ibDigH4 = ((i16_t)((readRegister(BME280_DIG_H4_MSB_REG) << 4) + (readRegister(BME280_DIG_H4_LSB_REG) & 0x0F)));
	g_Calibration.ibDigH5 = ((i16_t)((readRegister(BME280_DIG_H5_MSB_REG) << 4) + ((readRegister(BME280_DIG_H4_LSB_REG) >> 4) & 0x0F)));
	g_Calibration.ibDigH6 = ((i8_t)readRegister(BME280_DIG_H6_REG));

	//Most of the time the sensor will be init with default values
	//But in case user has old/deprecated code, use the settings.x values
	setStandbyTime(g_SensorSettings.byTimeStandby);
	setFilter(g_SensorSettings.byFilter);
	setPressureOverSample(g_SensorSettings.byPressOverSample); //Default of 1x oversample
	setHumidityOverSample(g_SensorSettings.byHumidOverSample); //Default of 1x oversample
	setTempOverSample(g_SensorSettings.byTempOverSample); //Default of 1x oversample

	setMode(MODE_NORMAL); //Go!

	return(readRegister(BME280_CHIP_ID_REG)); //Should return 0x60
}
/******************************************************************************
* @func					beginWithI2C
* @brief				This func begin with I2C interface
* @param				none
* @return				none
* @Note					none
*/
static bool_t beginWithI2C(){
	I2C1Init();
	u8_t byChipID = BMP280_Init();
	if(byChipID == 0x58) return(TRUE); //Begin normal init with these settings. Should return chip ID of 0x58 for BMP
	if(byChipID == 0x60) return(TRUE); //Begin normal init with these settings. Should return chip ID of 0x60 for BME
	return(FALSE);
}
/******************************************************************************
* @func					beginWithSPI
* @brief				This func begin with SPI interface
* @param				none
* @return				none
* @Note					none
*/
static bool_t beginWithSPI(){
	SPI1Init();
	u8_t byChipID = BMP280_Init();
	if(byChipID == 0x58) return(TRUE); //Begin normal init with these settings. Should return chip ID of 0x58 for BMP
	if(byChipID == 0x60) return(TRUE); //Begin normal init with these settings. Should return chip ID of 0x60 for BME
	return(FALSE);
}
/******************************************************************************
* @func					setMode
* @brief				This func setmode
* @param				u8_t byMode( sleep, force, normal)
* @return				none
* @Note					none
*/
static void_t setMode(u8_t byMode){
	if(byMode > MODE_NORMAL) byMode = 0; //Error check. Default to sleep mode

	u8_t byControlData = readRegister(BME280_CTRL_MEAS_REG);
	byControlData &= ~( (1<<1) | (1<<0) ); //Clear the mode[1:0] bits
	byControlData |= byMode; //Set
	writeRegister(BME280_CTRL_MEAS_REG, byControlData);
}
/******************************************************************************
* @func					beginWithSPI
* @brief				This func get Mode (slepp, force, normal)
* @param				none
* @return				none
* @Note					none
*/
static u8_t getMode(void_t){
	u8_t byControlData = readRegister(BME280_CTRL_MEAS_REG);
	return(byControlData & 0b00000011); //Clear bits 7 through 2
}
/******************************************************************************
* @func					setStandbyTime
* @brief				This func set stand time
* @param				u8_t byTimeSetting
//  0, 0.5ms
//  1, 62.5ms
//  2, 125ms
//  3, 250ms
//  4, 500ms
//  5, 1000ms
//  6, 10ms
//  7, 20ms
* @return				none
* @Note					none
*/
static void_t setStandbyTime(u8_t byTimeSetting){
	if(byTimeSetting > STANDBY_MS_20) byTimeSetting = 0; //Error check. Default to 0.5ms

	u8_t byControlData = readRegister(BME280_CONFIG_REG);
	byControlData &= ~( (1<<7) | (1<<6) | (1<<5) ); //Clear the 7/6/5 bits
	byControlData |= (byTimeSetting << 5); //Align with bits 7/6/5
	writeRegister(BME280_CONFIG_REG, byControlData);
}
/******************************************************************************
* @func					setFilter
* @brief				This func set value filter
* @param				u8_t byFilterSetting
/  0, filter off
//  1, coefficients = 2
//  2, coefficients = 4
//  3, coefficients = 8
//  4, coefficients = 16
* @return				none
* @Note					none
*/
static void_t setFilter(u8_t byFilterSetting){
	if(byFilterSetting > FILTER_X16) byFilterSetting = 0; //Error check. Default to filter off

	u8_t byControlData = readRegister(BME280_CONFIG_REG);
	byControlData &= ~( (1<<4) | (1<<3) | (1<<2) ); //Clear the 4/3/2 bits
	byControlData |= (byFilterSetting << 2); //Align with bits 4/3/2
	writeRegister(BME280_CONFIG_REG, byControlData);
}
/******************************************************************************
* @func					setTempOverSample
* @brief				This func swt Oversamping for Meas Temperature
* @param				u8_t byOverSampleAmount
* @return				none
* @Note					none
*/
static void_t setTempOverSample(u8_t byOverSampleAmount){
	byOverSampleAmount = checkSampleValue(byOverSampleAmount); //Error check

	u8_t byOriginalMode = getMode(); //Get the current mode so we can go back to it at the end

	setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_t bits (7, 6, 5) to overSampleAmount
	u8_t byControlData = readRegister(BME280_CTRL_MEAS_REG);
	byControlData &= ~( (1<<7) | (1<<6) | (1<<5) ); //Clear bits 765
	byControlData |= byOverSampleAmount << 5; //Align overSampleAmount to bits 7/6/5
	writeRegister(BME280_CTRL_MEAS_REG, controlData);

	setMode(byOriginalMode); //Return to the original user's choice
}
/******************************************************************************
* @func					setPressureOverSample
* @brief				This func swt Oversamping for Meas Pressure
* @param				u8_t byOverSampleAmount
* @return				none
* @Note					none
*/
static void_t setPressureOverSample(u8_t byOverSampleAmount)
{
	byOverSampleAmount = checkSampleValue(byOverSampleAmount); //Error check

	u8_t byOriginalMode = getMode(); //Get the current mode so we can go back to it at the end

	setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_p bits (4, 3, 2) to overSampleAmount
	u8_t byControlData = readRegister(BME280_CTRL_MEAS_REG);
	byControlData &= ~( (1<<4) | (1<<3) | (1<<2) ); //Clear bits 432
	byControlData |= byOverSampleAmount << 2; //Align overSampleAmount to bits 4/3/2
	writeRegister(BME280_CTRL_MEAS_REG, byControlData);

	setMode(byOriginalMode); //Return to the original user's choice
}
/******************************************************************************
* @func					setPressureOverSample
* @brief				This func swt Oversamping for Meas Humi
* @param				u8_t byOverSampleAmount
* @return				none
* @Note					none
*/
static void_t setHumidityOverSample(u8_t overSampleAmount)
{
	byOverSampleAmount = checkSampleValue(overSampleAmount); //Error check

	u8_t byOriginalMode = getMode(); //Get the current mode so we can go back to it at the end

	setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_h bits (2, 1, 0) to overSampleAmount
	u8_t byControlData = readRegister(BME280_CTRL_HUMIDITY_REG);
	byControlData &= ~( (1<<2) | (1<<1) | (1<<0) ); //Clear bits 2/1/0
	byControlData |= byOverSampleAmount << 0; //Align overSampleAmount to bits 2/1/0
	writeRegister(BME280_CTRL_HUMIDITY_REG, byControlData);

	setMode(byOriginalMode); //Return to the original user's choice
}

/******************************************************************************
* @func					checkSampleValue
* @brief				This func check Validates an over sample value
* 						These are used in the humidty, pressure, and temp oversample functions
* @param				u8_t byUserValue (Allowed values are 0 to 16)
* @return				none
* @Note					none
*/

static u8_t checkSampleValue(u8_t byUserValue)
{
	switch(byUserValue)
	{
		case(ZERO_SAMPLE_VALUE):
			return 0;
			break; //Valid
		case(ONE_SAMPLE_VALUE):
			return 1;
			break; //Valid
		case(TWO_SAMPLE_VALUE):
			return 2;
			break; //Valid
		case(FOUR_SAMPLE_VALUE):
			return 3;
			break; //Valid
		case(EIGHT_SAMPLE_VALUE):
			return 4;
			break; //Valid
		case(SIXTEEN_SAMPLE_VALUE):
			return 5;
			break; //Valid
		default:
			return 1; //Default to 1x
			break; //Good
	}
}
/******************************************************************************
* @func					setI2CAddress
* @brief				Set the global setting for the I2C address we want to communicate with
						Default is 0x77
* @param				u8_t byI2CAddress
* @return				none
* @Note					none
*/

static void_t setI2CAddress(u8_t byI2CAddress)
{
	g_SensorSettings.byI2CAddress = byAddress; //Set the I2C address for this device
}
/******************************************************************************
* @func					isMeasuring
* @brief				this func determine state meas
* @param				none
* @return				none
* @Note					none
*/
static bool_t isMeasuring(void_t)
{
	ut8_t byStat = readRegister(BME280_STAT_REG);
	return(byStat & (1<<3)); //If the measuring bit (3) is set, return true
}
/******************************************************************************
* @func					reset
* @brief				this func reset
* @param				none
* @return				none
* @Note					none
*/
static void_t reset( void_t )
{
	writeRegister(BME280_RST_REG, 0xB6);

}
/******************************************************************************
* @func					readAllMeasurements
* @brief				this func read all value ( press, temp, humi)
* @param				BME280SensorMeasurements_t *pmeasurements (press, temp, humi)
* 						u8_t byTempScale (C, F)
* @return				none
* @Note					none
*/
static void_t readAllMeasurements(BME280SensorMeasurements_t *pmeasurements, u8_t byTempScale){

	u8_t byDataBurst[8];
	readRegisterRegion(byDataBurst, BME280_MEASUREMENTS_REG, 8);

	if(byTempScale == 0){
		readTempCFromBurst(byDataBurst, pmeasurements);
	}else{
		readTempFFromBurst(byDataBurst, pmeasurements);
	}
	readFloatPressureFromBurst(byDataBurst, pmeasurements);
	readFloatHumidityFromBurst(byDataBurst, pmeasurements);
}
/******************************************************************************
* @func					readFloatPressure
* @brief				this func read value pressure with data type float
* @param				none
* @return				none
* @Note					none
*/
static float_t readFloatPressure(void_t )
{

	// Returns pressure in Pa as unsigned 32 bit ieger in Q24.8 format (24 ieger bits and 8 fractional bits).
	// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
    u8_t byBuffer[3];
	readRegisterRegion(byBuffer, BME280_PRESSURE_MSB_REG, 3);
    i32_t adc_P = ((u32_t)byBuffer[0] << 12) | ((u32_t)byBuffer[1] << 4) | ((byBuffer[2] >> 4) & 0x0F);

	i64_t ibVar1, ibVar2, ibPressADC;
	ibVar1 = ((i64_t)g_ibTempfine) - 128000;
	ibVar2 = ibVar1 * ibVar1 * (i64_t)g_Calibration.ibDigP6;
	ibVar2 = ibVar2 + ((ibVar1 * (i64_t)g_Calibration.ibDigP5)<<17);
	ibVar2 = ibVar2 + (((i64_t)g_Calibration.ibDigP4)<<35);
	ibVar1 = ((ibVar1 * ibVar1 * (i64_t)g_Calibration.ibDigP3)>>8) + ((ibVar1 * (i64_t)g_Calibration.ibDigP2)<<12);
	ibVar1 = (((((i64_t)1)<<47)+ibVar1))*((i64_t)g_Calibration.byDigP1)>>33;
	if (ibVar1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	ibPressADC = 1048576 - adc_P;
	ibPressADC = (((ibPressADC<<31) - ibVar2)*3125)/ibVar1;
	ibVar1 = (((i64_t)g_Calibration.ibDigP9) * (ibPressADC>>13) * (ibPressADC>>13)) >> 25;
	ibVar2 = (((i64_t)g_Calibration.ibDigP8) * ibPressADC) >> 19;
	ibPressADC = ((ibPressADC + ibVar1 + ibVar2) >> 8) + (((i64_t)g_Calibration.ibDigP7)<<4);

	return (float_t)ibPressADC / 256.0;

}
/******************************************************************************
* @func					readFloatPressureFromBurst
* @brief				this func read value pressure burst with data type float
* @param				u8_t byBuffer[],
* 						BME280SensorMeasurements_t *pmeasurements
* @return				none
* @Note					none
*/
static void_t readFloatPressureFromBurst(u8_t byBuffer[], BME280SensorMeasurements_t *pmeasurements)
{

	// Set pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
	// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa

  i32_t adc_P = ((u32_t)byBuffer[0] << 12) | ((u32_t)byBuffer[1] << 4) | ((byBuffer[2] >> 4) & 0x0F);

	i64_t byVar1, byVar2, byPressADC;
	byVar1 = ((i64_t)g_ibTempfine) - 128000;
	byVar2 = byVar1 * byVar1 * (i64_t)g_Calibration.ibDigP6;
	byVar2 = byVar2 + ((byVar1 * (i64_t)g_Calibration.ibDigP5)<<17);
	byVar2 = byVar2 + (((i64_t)g_Calibration.ibDigP4)<<35);
	byVar1 = ((byVar1 * byVar1 * (i64_t)g_Calibration.ibDigP3)>>8) + ((byVar1 * (i64_t)g_Calibration.ibDigP2)<<12);
	byVar1 = (((((i64_t)1)<<47)+byVar1))*((i64_t)g_Calibration.byDigP1)>>33;
	if (byVar1 == 0)
	{
		pmeasurements->fPressure = 0; // avoid exception caused by division by zero
	}
	else
	{
		byPressADC = 1048576 - adc_P;
		byPressADC = (((byPressADC<<31) - byVar2)*3125)/byVar1;
		byVar1 = (((i64_t)g_Calibration.ibDigP9) * (byPressADC>>13) * (byPressADC>>13)) >> 25;
		byVar2 = (((i64_t)g_Calibration.ibDigP8) * byPressADC) >> 19;
		byPressADC = ((byPressADC + byVar1 + byVar2) >> 8) + (((i64_t)g_Calibration.ibDigP7)<<4);

		pmeasurements->fPressure = (float_t)byPressADC / 256.0;
	}
}
/******************************************************************************
* @func					setReferencePressure
* @brief				this func set reference Pressure Value
* @param				float_t fRefPressure
* @return				none
* @Note					none
*/
static void_t setReferencePressure(float_t fRefPressure)
{
	g_fReferencePressure = fRefPressure;
}
/******************************************************************************
* @func					getReferencePressure
* @brief				this func get reference pressure
* @param				none
* @return				none
* @Note					none
*/
static float_t getReferencePressure()
{
	return g_fReferencePressure;
}
/******************************************************************************
* @func					readFloatAltitudeMeters
* @brief				this func read value altitude with unit of meas is meters
* @param				none
* @return				none
* @Note					none
*/
static float_t readFloatAltitudeMeters( void_t )
{
	float_t fHeightOutput = 0;

  // Getting height from a pressure reading is called the "international barometric height formula".
  // The magic value of 44330.77 was adjusted in issue #30.
  // There's also some discussion of it here: https://www.sparkfun.com/tutorials/253
  // This calculation is NOT designed to work on non-Earthlike planets such as Mars or Venus;
  // see NRLMSISE-00. That's why it is the "international" formula, not "interplanetary".
  // Sparkfun is not liable for incorrect altitude calculations from this
  // code on those planets. Interplanetary selfies are welcome, however.
	fHeightOutput = ((float)-44330.77)*(pow(((float_t)readFloatPressure()/(float_t)g_fReferencePressure), 0.190263) - (float_t)1); //Corrected, see issue 30
	return fHeightOutput;

}
/******************************************************************************
* @func					readFloatAltitudeFeet
* @brief				this func read value altitude with unit of meas is feet
* @param				none
* @return				none
* @Note					none
*/
static float_t readFloatAltitudeFeet( void_t )
{
	float_t fHeightOutput = 0;

	fHeightOutput = readFloatAltitudeMeters() * 3.28084;
	return fHeightOutput;
}
/******************************************************************************
* @func					readFloatHumidity
* @brief				this func read value humidity
* @param				none
* @return				none
* @Note					none
*/
static float_t readFloatHumidity( void_t )
{

	// Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
	// Output value of “47445” represents 47445/1024 = 46. 333 %RH
    u8_t byBuffer[2];
	readRegisterRegion(byBuffer, BME280_HUMIDITY_MSB_REG, 2);
    i32_t ibHumADC = ((u32_t)byBuffer[0] << 8) | ((u32_t)byBuffer[1]);

	i32_t byVar1;
	byVar1 = (g_ibTempfine - ((i32_t)76800));
	byVar1 = (((((ibHumADC << 14) - (((i32_t)g_Calibration.ibDigH4) << 20) - (((i32_t)g_Calibration.ibDigH5) * byVar1)) +
	((i32_t)16384)) >> 15) * (((((((byVar1 * ((i32_t)g_Calibration.ibDigH6)) >> 10) * (((byVar1 * ((i32_t)g_Calibration.byDigH3)) >> 11) + ((i32_t)32768))) >> 10) + ((i32_t)2097152)) *
	((i32_t)g_Calibration.ibDigH2) + 8192) >> 14));
	byVar1 = (byVar1 - (((((byVar1 >> 15) * (byVar1 >> 15)) >> 7) * ((i32_t)g_Calibration.byDigH1)) >> 4));
	byVar1 = (byVar1 < 0 ? 0 : byVar1);
	byVar1 = (byVar1 > 419430400 ? 419430400 : byVar1);

	return (float_t)(byVar1>>12) / 1024.0;
}
/******************************************************************************
* @func					readFloatHumidityFromBurst
* @brief				this func read value humidity from burst
* @param				u8_t byBuffer[],
* 						BME280SensorMeasurements_t *pmeasurements
* @return				none
* @Note					none
*/
static void_t readFloatHumidityFromBurst(u8_t byBuffer[], BME280SensorMeasurements_t *pmeasurements)
{

	// Set humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
	// Output value of “47445” represents 47445/1024 = 46. 333 %RH
	i32_t ibHumADC = ((ui32_t)byBuffer[6] << 8) | ((ui32_t)byBuffer[7]);

	i32_t byVar1;
	byVar1 = (g_ibTempfine - ((i32_t)76800));
	byVar1 = (((((ibHumADC << 14) - (((i32_t)g_Calibration.ibDigH4) << 20) - (((i32_t)g_Calibration.ibDigH5) * byVar1)) +
	((i32_t)16384)) >> 15) * (((((((byVar1 * ((i32_t)g_Calibration.ibDigH6)) >> 10) * (((byVar1 * ((i32_t)g_Calibration.byDigH3)) >> 11) + ((i32_t)32768))) >> 10) + ((i32_t)2097152)) *
	((i32_t)g_Calibration.ibDigH2) + 8192) >> 14));
	byVar1 = (byVar1 - (((((byVar1 >> 15) * (byVar1 >> 15)) >> 7) * ((i32_t)g_Calibration.byDigH1)) >> 4));
	byVar1 = (byVar1 < 0 ? 0 : byVar1);
	byVar1 = (byVar1 > 419430400 ? 419430400 : byVar1);

	pmeasurements->fHumidity = (float_t)(byVar1>>12) / 1024.0;
}
/******************************************************************************
* @func					setTemperatureCorrection
* @brief				this func set temp correct
* @param				float_t fCorr
* @return				none
* @Note					none
*/
static void_t setTemperatureCorrection(float_t fCorr)
{
	g_SensorSettings.fTempCorrection = fCorr;
}
/******************************************************************************
* @func					readTempC
* @brief				this func read temp with unit of meas is C
* @param				none
* @return				none
* @Note					none
*/
static float_t readTempC( void_t )
{
	// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
	// t_fine carries fine temperature as global value

	//get the reading (adc_T);
    u8_t byBuffer[3];
	readRegisterRegion(byBuffer, BME280_TEMPERATURE_MSB_REG, 3);
    i32_t adc_T = ((ui32_t)byBuffer[0] << 12) | ((ui32_t)byBuffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);

	//By datasheet, calibrate
	i64_t ibVar1, ibVar2;

	ibVar1 = ((((adc_T>>3) - ((i32_t)g_Calibration.byDigT1<<1))) * ((i32_t)g_Calibration.ibDigT2)) >> 11;
	ibVar2 = (((((adc_T>>4) - ((i32_t)g_Calibration.byDigT1)) * ((adc_T>>4) - ((i32_t)g_Calibration.byDigT1))) >> 12) *
	((i32_t)g_Calibration.dig_T3)) >> 14;
	g_ibTempfine = ibVar1 + ibVar2;
	float_t fOutput = (g_ibTempfine * 5 + 128) >> 8;

	fOutput = fOutput / 100 + g_SensorSettings.fTempCorrection;

	return output;
}
/******************************************************************************
* @func					readTempFromBurst
* @brief				this func read temp F from Burst
* @param				u8_t byBuffer[]
* @return				none
* @Note					none
*/
static float_t readTempFromBurst(u8_t byBuffer[])
{
  i32_t ibTempADC = ((u32_t)byBuffer[3] << 12) | ((u32_t)byBuffer[4] << 4) | ((byBuffer[5] >> 4) & 0x0F);

	//By datasheet, calibrate
	i64_t ibVar1, ibVar2;

	ibVar1 = ((((ibTempADC>>3) - ((i32_t)g_Calibration.byDigT1<<1))) * ((i32_t)g_Calibration.byDigT1)) >> 11;
	ibVar2 = (((((ibTempADC>>4) - ((i32_t)g_Calibration.byDigT1)) * ((ibTempADC>>4) - ((i32_t)g_Calibration.byDigT1))) >> 12) *
	((i32_t)calibration.dig_T3)) >> 14;
	g_ibTempfine = ibVar1 + ibVar2;
	float_t fOutput = (g_ibTempfine * 5 + 128) >> 8;

	fOutput = fOutput / 100 + g_SensorSettings.fTempCorrection;

 	return fOutput;
}
/******************************************************************************
* @func					readTempCFromBurst
* @brief				this func read temp C from Burst
* @param				u8_t byBuffer[]
* @return				none
* @Note					none
*/
static void_t readTempCFromBurst(u8_t byBuffer[], BME280SensorMeasurements_t *pmeasurements)
{
	pmeasurements->fTemperature = readTempFromBurst(byBuffer);
}
/******************************************************************************
* @func					readTempF
* @brief				this func read temp with unit of meas F
* @param				u8_t byBuffer[]
* @return				none
* @Note					none
*/
static float_t readTempF( void )
{
	float_t fOutput = readTempC();
	fOutput = (fOutput * 9) / 5 + 32;

	return fOutput;
}
/******************************************************************************
* @func					readTempFFromBurst
* @brief				this func read temp F from burst
* @param				u8_t byBuffer[],
* 						BME280SensorMeasurements_t *pmeasurements
* @return				none
* @Note					none
*/
static void_t readTempFFromBurst(u8_t byBuffer[], BME280SensorMeasurements_t *pmeasurements)
{
  float_t fOutput = readTempFromBurst(byBuffer);
  fOutput = (fOutput * 9) / 5 + 32;

  pmeasurements->fTemperature = fOutput;
}

////****************************************************************************//
////
////  Utility
////
////****************************************************************************//
static void readRegisterRegion(u8_t *pOutputPointer , u8_t byOffset, u8_t byLength)
{
}

static u8_t readRegister(u8_t offset)
{

}
static u16_t readRegisterInt16( u8_t byOffset )
{
	u8_t byBuffer[2];
	readRegisterRegion(byBuffer, byOffset, 2);  //Does memory transfer
	u16_t byOutput = (i16_t)byBuffer[0] | (i16_t)(byBuffer[1] << 8);

	return byOutput;
}

static void_t writeRegister(u8_t offset, u8_t dataToWrite)
{
}


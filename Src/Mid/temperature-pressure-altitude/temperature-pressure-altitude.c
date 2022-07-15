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
#include "Src/utils/typedefs.h"
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

#define BME280_ADDRESS (0x77)
#define BME280_ADDRESS_ALTERNATE (0x76) // Alternate Address

#define BME280_SENSOR_ID (0x60)
enum {
  BME280_REGISTER_ibDigT1 = 0x88,
  BME280_REGISTER_ibDigT2 = 0x8A,
  BME280_REGISTER_ibDigT3 = 0x8C,

  BME280_REGISTER_ibDigP1 = 0x8E,
  BME280_REGISTER_ibDigP2 = 0x90,
  BME280_REGISTER_ibDigP3 = 0x92,
  BME280_REGISTER_ibDigP4 = 0x94,
  BME280_REGISTER_ibDigP5 = 0x96,
  BME280_REGISTER_ibDigP6 = 0x98,
  BME280_REGISTER_ibDigP7 = 0x9A,
  BME280_REGISTER_ibDigP8 = 0x9C,
  BME280_REGISTER_ibDigP9 = 0x9E,

  BME280_REGISTER_ibDigH1 = 0xA1,
  BME280_REGISTER_ibDigH2 = 0xE1,
  BME280_REGISTER_ibDigH3 = 0xE3,
  BME280_REGISTER_ibDigH4 = 0xE4,
  BME280_REGISTER_ibDigH5 = 0xE5,
  BME280_REGISTER_ibDigH6 = 0xE7,

  BME280_REGISTER_CHIPID = 0xD0,
  BME280_REGISTER_VERSION = 0xD1,
  BME280_REGISTER_SOFTRESET = 0xE0,

  BME280_REGISTER_CAL26 = 0xE1, // R calibration stored in 0xE1-0xF0

  BME280_REGISTER_CONTROLHUMID = 0xF2,
  BME280_REGISTER_STATUS = 0XF3,
  BME280_REGISTER_CONTROL = 0xF4,
  BME280_REGISTER_CONFIG = 0xF5,
  BME280_REGISTER_PRESSUREDATA = 0xF7,
  BME280_REGISTER_TEMPDATA = 0xFA,
  BME280_REGISTER_HUMIDDATA = 0xFD,
  BME280_REGISTER_IRR = 0xB6
};

typedef struct {
  u16_t byDigT1;  ///< temperature compensation value
  i16_t ibDigT2;  ///< temperature compensation value
  i16_t ibDigT3;  ///< temperature compensation value

  u16_t byDigP1;  ///< pressure compensation value
  i16_t ibDigP2;  ///< pressure compensation value
  i16_t ibDigP3;  ///< pressure compensation value
  i16_t ibDigP4;  ///< pressure compensation value
  i16_t ibDigP5;  ///< pressure compensation value
  i16_t ibDigP6;  ///< pressure compensation value
  i16_t ibDigP7;  ///< pressure compensation value
  i16_t ibDigP8;  ///< pressure compensation value
  i16_t ibDigP9;  ///< pressure compensation value
} Bmp280CalibData_t;

enum  {
   SAMPLING_NONE = 0b000,
   SAMPLING_X1 = 0b001,
   SAMPLING_X2 = 0b010,
   SAMPLING_X4 = 0b011,
   SAMPLING_X8 = 0b100,
   SAMPLING_X16 = 0b101
 }SensorSampling;

enum  {
  MODE_SLEEP = 0b00,
  MODE_FORCED = 0b01,
  MODE_NORMAL = 0b11
}SensorMode;

enum  {
  FILTER_OFF = 0b000,
  FILTER_X2 = 0b001,
  FILTER_X4 = 0b010,
  FILTER_X8 = 0b011,
  FILTER_X16 = 0b100
}SensorFilter;

enum  {
  STANDBY_MS_0_5 = 0b000,
  STANDBY_MS_10 = 0b110,
  STANDBY_MS_20 = 0b111,
  STANDBY_MS_62_5 = 0b001,
  STANDBY_MS_125 = 0b010,
  STANDBY_MS_250 = 0b011,
  STANDBY_MS_500 = 0b100,
  STANDBY_MS_1000 = 0b101
}StandDuration;
typedef enum
{
  SENSOR_TYPE_ACCELEROMETER         = (1),
  SENSOR_TYPE_MAGNETIC_FIELD        = (2),
  SENSOR_TYPE_ORIENTATION           = (3),
  SENSOR_TYPE_GYROSCOPE             = (4),
  SENSOR_TYPE_LIGHT                 = (5),
  SENSOR_TYPE_PRESSURE              = (6),
  SENSOR_TYPE_PROXIMITY             = (8),
  SENSOR_TYPE_GRAVITY               = (9),
  SENSOR_TYPE_LINEAR_ACCELERATION   = (10),
  SENSOR_TYPE_ROTATION_VECTOR       = (11),
  SENSOR_TYPE_RELATIVE_HUMIDITY     = (12),
  SENSOR_TYPE_AMBIENT_TEMPERATURE   = (13),
  SENSOR_TYPE_VOLTAGE               = (15),
  SENSOR_TYPE_CURRENT               = (16),
  SENSOR_TYPE_COLOR                 = (17)
} SensorType_t;
/* Sensor details (40 bytes) */
/** struct sensor_s is used to describe basic information about a specific sensor. */
typedef struct
{
    i8_t    ibName[12];
    i32_t  	ibVersion;
    i32_t  	ibSensor_id;
    i32_t  	ibType;
    float_t   flMax_value;
    float_t   flMin_value;
    float_t   flResolution;
    i32_t  	ibMinDelay;
} Sensor_t;
typedef struct {
    // inactive duration (standby time) in normal mode
    // 000 = 0.5 ms
    // 001 = 62.5 ms
    // 010 = 125 ms
    // 011 = 250 ms
    // 100 = 500 ms
    // 101 = 1000 ms
    // 110 = 10 ms
    // 111 = 20 ms
    u8_t byStandbyTime : 3; ///< inactive duration (standby time) in normal mode

    // filter settings
    // 000 = filter off
    // 001 = 2x filter
    // 010 = 4x filter
    // 011 = 8x filter
    // 100 and above = 16x filter
    u8_t byFilter : 3; ///< filter settings

    // unused - don't set
    u8_t byNone : 1;     ///< unused - don't set
    u8_t bySpi3w_en : 1; ///< unused - don't set

    /// @return combined config register
    u8_t get() {
    	return (byStandbyTime << 5) | (byFilter << 2) | bySpi3w_en;
    }
  }Config_t;

 typedef struct {
      // temperature oversampling
      // 000 = skipped
      // 001 = x1
      // 010 = x2
      // 011 = x4
      // 100 = x8
      // 101 and above = x16
      u8_t byOsrs_t : 3; ///< temperature oversampling

      // pressure oversampling
      // 000 = skipped
      // 001 = x1
      // 010 = x2
      // 011 = x4
      // 100 = x8
      // 101 and above = x16
      u8_t byOsrs_p : 3; ///< pressure oversampling

      // device mode
      // 00       = sleep
      // 01 or 10 = forced
      // 11       = normal
      u8_t byMode : 2; ///< device mode

      /// @return combined ctrl register
      u8_t get() {
    	  return (byOsrs_t << 5) | (byOsrs_p << 2) | byMode;
      }
    }ControlMeas_t;
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
static void_t SPI1_Init(void_t);
static bool_t BME280Init(void_t);
static void_t delayMs(u16_t byTimeDelay);
static void_t setSampling(SensorMode sensorMode,
				   SensorSampling tempSampling,
				   SensorSampling pressureSampling,
				   SensorFilter sensorFilter,
				   StandDuration standDuration);
static bool_t takeForcedMeasurement(void_t);
static float_t getTemperatureCompensation(void_t);
static void_t setTemperatureCompensation(float_t);
static void_t readCoefficients(void_t);
static bool_t isReadingCalibration(void_t);
static void_t write8(u8_t byReg, u8_t byValue);
static u8_t  read8(u8_t byReg);
static u16_t read16(u8_t byReg);
static u32_t read24(u8_t byReg);
static i16_t readS16(u8_t byReg);
static u16_t read16LE(u8_t byReg); // little endian
static i16_t readS16LE(u8_t byReg); // little endian
static u32_t bySensorID(void_t);
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

u8_t g_byI2CAddress;   //!< I2C addr for the TwoWire interface
i32_t g_ibSensorID;  //!< ID of the BME Sensor
i32_t g_ibTfine; 		//!< temperature with high resolution, stored as an attribute
                    //!< as this is used for temperature compensation reading
                    //!< humidity and pressure

i32_t g_ibTfineAdjust = 0; //!< add to compensate temp readings and in turn
                             //!< to pressure and humidity readings

Bmp280CalibData_t g_Bmp280Calib; //!< here calibration data is stored
Config_t g_ConfigReg;
ControlMeas_t g_ControlMeas;
/******************************************************************************/
/******************************************************************************
* @func					I2C1_Init
* @brief				This func Init I2C1_Init mode master
* @param				none
* @return				none
* @Note					none
*/
void_t delayMs(u16_t byTimeDelays){
	for(int i = 0; i < byTimeDelays; i++){
		for(int j = 0; j < 5000; j++){
		}
	}
}

/******************************************************************************
* @func					I2C1_Init
* @brief				This func Init I2C1_Init mode master
* @param				none
* @return				none
* @Note					none
*/
static void_t I2C1Init(void_t){
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
static void_t SPI1_Init(void_t){
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
static bool_t BME280Init(void_t){
	g_bySensorID = read8(BME280_REGISTER_CHIPID);
	if(g_bySensorID != BME280_SENSOR_ID){
		return FALSE;
	}
	write8(BME280_REGISTER_SOFTRESET, BME280_REGISTER_IRR);
	delayMs(10);
	while(isReadingCalibration())
	delayMs(10);
	readCoefficients();
	setSampling(sensorMode, tempSampling, pressSampling, sensorFilter, standDuration);
	delayMs(100);
	return TRUE;
}
/*!
 *   @brief  setup sensor with given parameters / settings
 *
 *   This is simply a overload to the normal begin()-function, so SPI users
 *   don't get confused about the library requiring an address.
 *   @param mode the power mode to use for the sensor
 *   @param tempSampling the temp samping rate to use
 *   @param pressSampling the pressure sampling rate to use
 *   @param humSampling the humidity sampling rate to use
 *   @param filter the filter mode to use
 *   @param duration the standby duration to use
 */
static void_t setSampling(SensorMode sensorMode,
				   	   	  SensorSampling tempSampling,
						  SensorSampling pressureSampling,
						  SensorFilter sensorFilter,
						  StandDuration standDuration){
	g_ControlMeas.byMode = sensorMode;
	g_ControlMeas.byOsrs_t = tempSampling;
	g_ControlMeas.byOsrs_p = pressureSampling;

	g_ConfigReg.byFilter = sensorFilter;
	g_ConfigReg.byStandbyTime = standDuration;
	g_ConfigReg.bySpi3w_en = 0;
	 // making sure sensor is in sleep mode before setting configuration
	 // as it otherwise may be ignored
	write8(BME280_REGISTER_CONTROL, MODE_SLEEP);
	// you must make sure to also set REGISTER_CONTROL after setting the
	// CONTROLHUMID register, otherwise the values won't be applied (see
	// DS 5.4.3)
	write8(BME280_REGISTER_CONFIG, g_ConfigReg.get());
	write8(BME280_REGISTER_CONTROL, g_ControlMeas.get());
}
static bool_t takeForcedMeasurement(void_t);
static float_t getTemperatureCompensation(void_t);
static void_t setTemperatureCompensation(float_t);
static void_t readCoefficients(void_t);
static bool_t isReadingCalibration(void_t);
/*!
 *   @brief  Writes an 8 bit value over I2C or SPI
 *   @param reg the register address to write to
 *   @param value the value to write to the register
 */
static void_t write8(u8_t byReg, u8_t byValue){
	  u8_t bybuffer[2];
	  bybuffer[1] = byValue;
	  if (i2c_dev) {
		  bybuffer[0] = byReg;
	    i2c_dev->write(bybuffer, 2);
	  } else {
		  bybuffer[0] = byReg & ~0x80;
	    spi_dev->write(buffer, 2);
	  }
}
static u8_t  read8(u8_t byReg);
static u16_t read16(u8_t byReg);
static u32_t read24(u8_t byReg);
static i16_t readS16(u8_t byReg);
static u16_t read16LE(u8_t byReg); // little endian
static i16_t readS16LE(u8_t byReg); // little endian
static u32_t bySensorID(void_t);

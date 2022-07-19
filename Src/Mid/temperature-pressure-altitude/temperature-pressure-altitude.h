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
#ifndef MID_TEMPERATURE_PRESSURE_ALTITUDE_H_
#define MID_TEMPERATURE_PRESSURE_ALTITUDE_H_
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "typedefs.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
typedef struct
{
	float_t fTemperature;
	float_t fPressure;
	float_t fHumidity;
}BME280SensorMeasurements_t;
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
* @func					beginWithI2C
* @brief				This func begin with I2C interface
* @param				none
* @return				none
* @Note					none
*/
bool_t beginWithI2C(void_t);
/******************************************************************************
* @func					beginWithSPI
* @brief				This func begin with SPI interface
* @param				none
* @return				none
* @Note					none
*/
bool_t beginWithSPI(void_t);
/******************************************************************************
* @func					BMP280_Init
* @brief				This func Init BMP280_Init
* @param				none
* @return				none
* @Note					none
*/
u8_t  BMP280_Init(void_t);
/******************************************************************************
* @func					setTempOverSample
* @brief				This func swt Oversamping for Meas Temperature
* @param				u8_t byOverSampleAmount
* @return				none
* @Note					none
*/
void_t setTempOverSample(u8_t byOverSampleAmount); //Set the temperature sample mode
/******************************************************************************
* @func					setPressureOverSample
* @brief				This func swt Oversamping for Meas Pressure
* @param				u8_t byOverSampleAmount
* @return				none
* @Note					none
*/
void_t setPressureOverSample(u8_t byOverSampleAmount); //Set the pressure sample mode
/******************************************************************************
* @func					setPressureOverSample
* @brief				This func swt Oversamping for Meas Humi
* @param				u8_t byOverSampleAmount
* @return				none
* @Note					none
*/
void_t setHumidityOverSample(u8_t byOverSampleAmount); //Set the humidity sample mode
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
void_t setStandbyTime(u8_t byTimeSetting); //Set the standby time between measurements
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
void_t setFilter(u8_t byFilterSetting); //Set the filter

//Software reset routine
/******************************************************************************
* @func					reset
* @brief				this func reset
* @param				none
* @return				none
* @Note					none
*/
void_t reset(void_t );
/******************************************************************************
* @func					readAllMeasurements
* @brief				this func read all value ( press, temp, humi)
* @param				BME280SensorMeasurements_t *pmeasurements (press, temp, humi)
* 						u8_t byTempScale (C, F)
* @return				none
* @Note					none
*/
void_t readAllMeasurements(BME280SensorMeasurements_t *pmeasurements, u8_t byTempScale );

//Returns the values as floats.
/******************************************************************************
* @func					readFloatPressure
* @brief				this func read value pressure with data type float
* @param				none
* @return				none
* @Note					none
*/
u32_t readFloatPressure(void_t );
/******************************************************************************
* @func					readFloatAltitudeMeters
* @brief				this func read value altitude with unit of meas is meters
* @param				none
* @return				none
* @Note					none
*/
u32_t readFloatAltitudeMeters(void_t);
/******************************************************************************
* @func					readFloatAltitudeFeet
* @brief				this func read value altitude with unit of meas is feet
* @param				none
* @return				none
* @Note					none
*/
u32_t readFloatAltitudeFeet( void_t );
/******************************************************************************
* @func					readFloatPressureFromBurst
* @brief				this func read value pressure burst with data type float
* @param				u8_t byBuffer[],
* 						BME280SensorMeasurements_t *pmeasurements
* @return				none
* @Note					none
*/
void_t readFloatPressureFromBurst(u8_t byBuffer[], BME280SensorMeasurements_t *pmeasurements);
/******************************************************************************
* @func					readFloatHumidity
* @brief				this func read value humidity
* @param				none
* @return				none
* @Note					none
*/
float_t readFloatHumidity(void_t );
/******************************************************************************
* @func					readFloatHumidityFromBurst
* @brief				this func read value humidity from burst
* @param				u8_t byBuffer[],
* 						BME280SensorMeasurements_t *pmeasurements
* @return				none
* @Note					none
*/
void_t readFloatHumidityFromBurst(u8_t byBuffer[], BME280SensorMeasurements_t *pmeasurements);
/******************************************************************************
* @func					readTempC
* @brief				this func read temp with unit of meas is C
* @param				none
* @return				none
* @Note					none
*/
u8_t readTempC( void_t);
/******************************************************************************
* @func					readTempF
* @brief				this func read temp with unit of meas F
* @param				u8_t byBuffer[]
* @return				none
* @Note					none
*/
u32_t readTempF( void_t );
/******************************************************************************
* @func					setReferencePressure
* @brief				this func set reference Pressure Value
* @param				float_t fRefPressure
* @return				none
* @Note					none
*/
void_t setReferencePressure(float_t fRefPressure);

/******************************************************************************
* @func					getReferencePressure
* @brief				this func get reference pressure
* @param				none
* @return				none
* @Note					none
*/
float_t getReferencePressure();

#endif /* MID_TEMPERATURE_PRESSURE_ALTITUDE_H_ */

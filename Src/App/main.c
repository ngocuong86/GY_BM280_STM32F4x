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
 * Description:main.c
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
#include <stdio.h>
#include "ucg.h"
#include "Ucglib.h"
#include "typedefs.h"
#include "system_stm32f4xx.h"
#include "stm32f401re_usart.h"
#include "stm32f401re_gpio.h"
#include "stm32f401re_rcc.h"
#include "../Driver/I2C1-Interface/I2C1-Interface.h"
#include "../Driver/SPI1-Interface/SPI1-Interface.h"
#include "../Mid/BME280-pressure-temperature-sensor/BME280-pressure-temperature-sensor.h"
/******************************************************************************/
/*                     PRIVETA TYPES and DEFINITIONS                         */
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
 static ucg_t g_Ucg;
 u32_t g_byTimerInit = 0, g_byTimerCurrent = 0;
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
 * @func					displayTempPresAlti
 * @brief				this func display value sensor to LCD
 * @param				none
 * @return				none
 * @Note				none
 */
void_t displayTempPresAlti(void_t){
	static char strPressure[20] = "";
	static char strTemperature[20] = "";
	static char strAltitude[20] = "";
	static char strDewPointC[20] = "";
	memset(strTemperature, 0, sizeof(strTemperature));
	memset(strPressure, 0, sizeof(strPressure));
	memset(strAltitude, 0, sizeof(strAltitude));
	memset(strDewPointC, 0, sizeof(strDewPointC));
	float_t fTeamp = (u64_t)readTempC();
	fTeamp = (float_t)fTeamp/100;
	float_t fPres = (u64_t)readIntPressure();
	fPres = (float_t)fPres/100;
	float_t fAlti = (u64_t)readIntAltitudeMeters();
	fAlti = (float_t)fAlti/100;
	sprintf(strTemperature, "Temp = %.2f oC", (float_t)(fTeamp));
	sprintf(strPressure, "Pres = %.2f Pa ",(float_t)(fPres));
	sprintf(strAltitude, "Alti = %.2f m", (float_t)(fAlti));
	ucg_DrawString(&g_Ucg, 0, 32, 0, strTemperature);
	ucg_DrawString(&g_Ucg, 0, 62, 0, strPressure);
	ucg_DrawString(&g_Ucg, 0, 92, 0, strAltitude);
}
/******************************************************************************
* @func					processGetValueSensor
* @brief				this func process Time getValue
* @param				none
* @return				none
* @Note					none
*/
void_t processGetValueSensor(void_t){
	g_byTimerCurrent = GetMilSecTick();
	if(g_byTimerCurrent - g_byTimerInit >= 1000){
		displayTempPresAlti();
		g_byTimerInit = g_byTimerCurrent;
	}
}
/******************************************************************************
* @func					main
* @brief				this func main
* @param				none
* @return				none
* @Note					none
*/
int main(){
	SystemCoreClockUpdate();
	TimerInit();
	Ucglib4WireSWSPI_begin(&g_Ucg, UCG_FONT_MODE_SOLID);
	ucg_ClearScreen(&g_Ucg);
	ucg_SetFont(&g_Ucg, ucg_font_ncenR12_hr);
	ucg_SetColor(&g_Ucg, 0, 255, 255, 255);
	ucg_SetColor(&g_Ucg, 1, 0, 0, 0);
	ucg_SetRotate180(&g_Ucg);
	g_byTimerInit = GetMilSecTick();
	if(beginWithI2C() == TRUE){ // use beginWithSPI to use SPI interface
		while(1){
			setReferencePressure(100700); // Set value Pressure in Ha Noi (Value Pressure in sea surface is 101330);
			processTimerScheduler();
			processGetValueSensor();
		}
	}
	return 0;
}


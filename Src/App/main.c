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
#include "../Src/Mid/I2C1-Interface/I2C1-Interface.h"
#include "../Src/Mid/SPI1-Interface/SPI1-Interface.h"
#include "ucg.h"
#include "Ucglib.h"
#include "typedefs.h"
#include "system_stm32f4xx.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
 static ucg_t ucg;
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

/******************************************************************************/
void Display_TemHum(void){
	static char strPres[20] = "";
	static char strTemp[20] = "";
	static char strAlti[20] = "";
	memset(strPres, 0, sizeof(strPres));
	memset(strTemp, 0, sizeof(strTemp));
	memset(strAlti, 0, sizeof(strAlti));
	u8_t fTeamp = (u8_t)readTempC();
	u32_t fPres = (u32_t)readFloatPressure();
	u32_t fAlti = (u32_t)readFloatAltitudeMeters();
	sprintf(strTemp, "Temp = %d oC", (u8_t)fTeamp);
	sprintf(strPres, "Pres = %d Pa ",(u32_t)(fPres*100));
	sprintf(strAlti, "Alti = %d m", (u32_t)(fAlti));
	ucg_DrawString(&ucg, 0, 32, 0, strTemp);
	ucg_DrawString(&ucg, 0, 72, 0, strPres);
	ucg_DrawString(&ucg, 0, 102, 0, strAlti);
}
void processGetValueSensor(void){
	g_byTimerCurrent = GetMilSecTick();
	if(g_byTimerCurrent - g_byTimerInit >= 1000){
		Display_TemHum();
		g_byTimerInit = g_byTimerCurrent;
	}
}
int main(){
	SystemCoreClockUpdate();
	TimerInit();
	Ucglib4WireSWSPI_begin(&ucg, UCG_FONT_MODE_SOLID);
	ucg_ClearScreen(&ucg);
	ucg_SetFont(&ucg, ucg_font_ncenR12_hr);
	ucg_SetColor(&ucg, 0, 255, 255, 255);
	ucg_SetColor(&ucg, 1, 0, 0, 0);
	ucg_SetRotate180(&ucg);
	g_byTimerInit = GetMilSecTick();
	if(beginWithI2C() == TRUE){ // use beginWithSPI to use SPI interface
		while(1){
			setReferencePressure(100460); // Set value Pressure in Ha Noi (Value Pressure in sea surface is 101330);
			processTimerScheduler();
			processGetValueSensor();
		}
	}
	return 0;
}


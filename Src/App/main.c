#include <stdio.h>
#include "../Src/Mid/I2C1-Interface/I2C1-Interface.h"
#include "../Src/Mid/SPI1-Interface/SPI1-Interface.h"
#include "ucg.h"
#include "Ucglib.h"
#include "typedefs.h"
static ucg_t ucg;
int main(){
	Ucglib4WireSWSPI_begin(&ucg, UCG_FONT_MODE_SOLID);
	ucg_ClearScreen(&ucg);
	ucg_SetFont(&ucg, ucg_font_ncenR12_hr);
	ucg_SetColor(&ucg, 0, 255, 255, 255);
	ucg_SetColor(&ucg, 1, 0, 0, 0);
	ucg_SetRotate180(&ucg);
	ucg_ClearScreen(&ucg);
	static char text[20] = "hello";
	ucg_DrawString(&ucg, 0, 92, 0, text);
	beginWithI2C();
	char strTemp[20] = "";
	char strPres[20] = "";
	char strAlti[20] = "";
	memset(strPres, 0, sizeof(strPres));
	memset(strTemp, 0, sizeof(strTemp));
	memset(strAlti, 0, sizeof(strAlti));
	float_t fTeamp = readTempC();
	float_t fPres = readFloatPressure();
	float_t fAlti = readFloatAltitudeMeters();
	sprintf(strTemp, "Temperature = %d oC", fTeamp);
	sprintf(strPres, "Pressure = %d hPa ", fPres);
	sprintf(strAlti, "Temperature = %d m", fAlti);
	ucg_DrawString(&ucg, 0, 72, 0, strTemp);
	ucg_DrawString(&ucg, 0, 92, 0, strPres);
	ucg_DrawString(&ucg, 0, 102, 0, strAlti);
	return 0;

}


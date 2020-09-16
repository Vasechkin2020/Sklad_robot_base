
//I2C ADDRESS / BITS
//---------------------------------------------------------------------- - */

#define BME280_ADDRESS 0x76			  // Адрес по которому на шине I2C находится датчик
int32_t   t_fine;  // Глобальная переменная используется в разных расчетах температуры, давления, влажности

float TemperatureBME280, PressureBME280, PressureBME280_mm, HumidityBME280;

//   REGISTERS   ---------------------------------------------------------------------- - */
enum
{
	BME280_REGISTER_DIG_T1 = 0x88,
	BME280_REGISTER_DIG_T2 = 0x8A,
	BME280_REGISTER_DIG_T3 = 0x8C,

	BME280_REGISTER_DIG_P1 = 0x8E,
	BME280_REGISTER_DIG_P2 = 0x90,
	BME280_REGISTER_DIG_P3 = 0x92,
	BME280_REGISTER_DIG_P4 = 0x94,
	BME280_REGISTER_DIG_P5 = 0x96,
	BME280_REGISTER_DIG_P6 = 0x98,
	BME280_REGISTER_DIG_P7 = 0x9A,
	BME280_REGISTER_DIG_P8 = 0x9C,
	BME280_REGISTER_DIG_P9 = 0x9E,

	BME280_REGISTER_DIG_H1 = 0xA1,
	BME280_REGISTER_DIG_H2 = 0xE1,
	BME280_REGISTER_DIG_H3 = 0xE3,
	BME280_REGISTER_DIG_H4 = 0xE4,
	BME280_REGISTER_DIG_H5 = 0xE5,
	BME280_REGISTER_DIG_H6 = 0xE7,

	BME280_REGISTER_CHIPID = 0xD0,
	BME280_REGISTER_VERSION = 0xD1,
	BME280_REGISTER_SOFTRESET = 0xE0,

	BME280_REGISTER_CAL26 = 0xE1,  // R calibration stored in 0xE1-0xF0

	BME280_REGISTER_CONTROLHUMID = 0xF2,
	BME280_REGISTER_STATUS = 0XF3,
	BME280_REGISTER_CONTROL = 0xF4,
	BME280_REGISTER_CONFIG = 0xF5,
	BME280_REGISTER_PRESSUREDATA = 0xF7,
	BME280_REGISTER_TEMPDATA = 0xFA,
	BME280_REGISTER_HUMIDDATA = 0xFD
};
enum sensor_sampling {
	SAMPLING_NONE = 0b000,
	SAMPLING_X1 = 0b001,
	SAMPLING_X2 = 0b010,
	SAMPLING_X4 = 0b011,
	SAMPLING_X8 = 0b100,
	SAMPLING_X16 = 0b101
};

enum sensor_mode {
	MODE_SLEEP = 0b00,
	MODE_FORCED = 0b01,
	MODE_NORMAL = 0b11
};

enum sensor_filter {
	FILTER_OFF = 0b000,
	FILTER_X2 = 0b001,
	FILTER_X4 = 0b010,
	FILTER_X8 = 0b011,
	FILTER_X16 = 0b100
};

// standby durations in ms 
enum standby_duration {
	STANDBY_MS_0_5 = 0b000,
	STANDBY_MS_10 = 0b110,
	STANDBY_MS_20 = 0b111,
	STANDBY_MS_62_5 = 0b001,
	STANDBY_MS_125 = 0b010,
	STANDBY_MS_250 = 0b011,
	STANDBY_MS_500 = 0b100,
	STANDBY_MS_1000 = 0b101
};

/*=========================================================================
	CALIBRATION DATA
	-----------------------------------------------------------------------*/
struct	bme280_calib_data
{
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;

	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;

	uint8_t  dig_H1;
	int16_t  dig_H2;
	uint8_t  dig_H3;
	int16_t  dig_H4;
	int16_t  dig_H5;
	int8_t   dig_H6;
};
/*=========================================================================*/

bme280_calib_data _bme280_calib;

void ReadSensor()
{
	//sensor.read(1) ; //  Считываем данные и выводим: температуру в °С, давление в мм. рт. ст., изменение высоты относительно указанной в функции begin(по умолчанию 0 метров).
	//bmp280Temp = (bmp280Temp + sensor.temperature)/2;        // Температура с барометра 2 msec
	//bmp280Pressure = (bmp280Pressure + sensor.pressure)/2;
	//bmp280Altitude = (bmp280Altitude + sensor.altitude)/2;  
 //   siHumd = (siHumd + myHumidity.readHumidity())/2;
  //  siTemp = (siTemp + myHumidity.readTemperature())/2;  // Температура с датчика   72 msec ОЧЕНЬ долго
	//lux = (lux + lightMeter.readLightLevel()) / 2;   // Освещенность с датчика
}
void readCoefficients(void)
{
	_bme280_calib.dig_T1 = ReadWord_I2C_LE(BME280_ADDRESS, BME280_REGISTER_DIG_T1);	// unsigned short
	_bme280_calib.dig_T2 = (int16_t)ReadWord_I2C_LE(BME280_ADDRESS, BME280_REGISTER_DIG_T2);	//  signed short
	_bme280_calib.dig_T3 = (int16_t)ReadWord_I2C_LE(BME280_ADDRESS, BME280_REGISTER_DIG_T3);

	_bme280_calib.dig_P1 = ReadWord_I2C_LE(BME280_ADDRESS, BME280_REGISTER_DIG_P1);
	_bme280_calib.dig_P2 = (int16_t)ReadWord_I2C_LE(BME280_ADDRESS, BME280_REGISTER_DIG_P2);
	_bme280_calib.dig_P3 = (int16_t)ReadWord_I2C_LE(BME280_ADDRESS, BME280_REGISTER_DIG_P3);
	_bme280_calib.dig_P4 = (int16_t)ReadWord_I2C_LE(BME280_ADDRESS, BME280_REGISTER_DIG_P4);
	_bme280_calib.dig_P5 = (int16_t)ReadWord_I2C_LE(BME280_ADDRESS, BME280_REGISTER_DIG_P5);
	_bme280_calib.dig_P6 = (int16_t)ReadWord_I2C_LE(BME280_ADDRESS, BME280_REGISTER_DIG_P6);
	_bme280_calib.dig_P7 = (int16_t)ReadWord_I2C_LE(BME280_ADDRESS, BME280_REGISTER_DIG_P7);
	_bme280_calib.dig_P8 = (int16_t)ReadWord_I2C_LE(BME280_ADDRESS, BME280_REGISTER_DIG_P8);
	_bme280_calib.dig_P9 = (int16_t)ReadWord_I2C_LE(BME280_ADDRESS, BME280_REGISTER_DIG_P9);

	_bme280_calib.dig_H1 = ReadByte_I2C(BME280_ADDRESS, BME280_REGISTER_DIG_H1);
	_bme280_calib.dig_H2 = (int16_t)ReadWord_I2C_LE(BME280_ADDRESS, BME280_REGISTER_DIG_H2);
	_bme280_calib.dig_H3 = ReadByte_I2C(BME280_ADDRESS, BME280_REGISTER_DIG_H3);


	/***************** ЭТО РАСПИСАНЫЙ СПОСОБ ПРЕОБРАЗОВАНИЯ ПО ШАГАМ */

	//uint8_t h = ReadByte_I2C(BME280_ADDRESS, BME280_REGISTER_DIG_H4);	 // Читаем 8 бит
	//uint16_t h16 = (uint16_t) h << 4;								// В 16 битном числе сдвигаем влево для получения 12 битного чила
	//uint8_t l = ReadByte_I2C(BME280_ADDRESS, BME280_REGISTER_DIG_H4+1);	 // Читаем 8 бит
	//l = l & 0xF;     // Оставляем только 4 млладших бита с 0 по 3
	//h16 = (int16_t)h16 | l; // Склеиваем и получаем 12 битное значение в 16 битной переменной
	//_bme280_calib.dig_H4 = h16;

	_bme280_calib.dig_H4 = (int16_t)(ReadByte_I2C(BME280_ADDRESS, BME280_REGISTER_DIG_H4) << 4) | (ReadByte_I2C(BME280_ADDRESS, BME280_REGISTER_DIG_H4 + 1) & 0xF);

	//h = ReadByte_I2C(BME280_ADDRESS, BME280_REGISTER_DIG_H5 + 1);	 // Читаем 8 бит
	//h16 = (uint16_t)h << 4;								// В 16 битном числе сдвигаем влево для получения 12 битного чила
	//l = ReadByte_I2C(BME280_ADDRESS, BME280_REGISTER_DIG_H5);	 // Читаем 8 бит
	//l = l >> 4;     // Оставляем только 4 старших бита с 7 по 4 сдвигая их влево на место младших
	//h16 = (int16_t)h16 | l; // Склеиваем и получаем 12 битное значение в 16 битной переменной
	//_bme280_calib.dig_H5 = h16;

	_bme280_calib.dig_H5 = (ReadByte_I2C(BME280_ADDRESS, BME280_REGISTER_DIG_H5 + 1) << 4) | (ReadByte_I2C(BME280_ADDRESS, BME280_REGISTER_DIG_H5) >> 4);

	_bme280_calib.dig_H6 = (int8_t)ReadByte_I2C(BME280_ADDRESS, BME280_REGISTER_DIG_H6);

	Serial.print("dig_T1="); Serial.println(_bme280_calib.dig_T1);
	Serial.print("dig_T2="); Serial.println(_bme280_calib.dig_T2);
	Serial.print("dig_T3="); Serial.println(_bme280_calib.dig_T3);
	Serial.print("dig_P1="); Serial.println(_bme280_calib.dig_P1);
	Serial.print("dig_P2="); Serial.println(_bme280_calib.dig_P2);
	Serial.print("dig_P3="); Serial.println(_bme280_calib.dig_P3);
	Serial.print("dig_P4="); Serial.println(_bme280_calib.dig_P4);
	Serial.print("dig_P5="); Serial.println(_bme280_calib.dig_P5);
	Serial.print("dig_P6="); Serial.println(_bme280_calib.dig_P6);
	Serial.print("dig_P7="); Serial.println(_bme280_calib.dig_P7);
	Serial.print("dig_P8="); Serial.println(_bme280_calib.dig_P8);
	Serial.print("dig_P9="); Serial.println(_bme280_calib.dig_P9);
}

void setSampling(						 //Установка режима работы
	sensor_mode       mode,
	sensor_sampling   tempSampling,
	sensor_sampling   pressSampling,
	sensor_sampling   humSampling,
	sensor_filter     filter,
	standby_duration  duration)
{
	Serial.println("setSampling... ");
	// you must make sure to also set REGISTER_CONTROL after setting the	CONTROLHUMID register, otherwise the values won't be applied (see DS 5.4.3)
	WriteByte_I2C(BME280_ADDRESS, BME280_REGISTER_CONTROLHUMID, humSampling);
	delay(100);
	Serial.print("BME280_REGISTER_CONTROLHUMID ");
	Serial.println(ReadByte_I2C(BME280_ADDRESS, BME280_REGISTER_CONTROLHUMID), BIN);


	//Формируем байт с настроуками температуры и давления и устанавливаем режим sleep чтобы записать настройки 
  //  в нормальном режиме они могут не записаться 
  /*The “config” register sets the rate, filter and interface options of the device.Writes to the “config”
	  register in normal mode may be ignored.In sleep mode writes are not ignored. */

	byte ctrl_meas = tempSampling << 5 | pressSampling << 2 | 0b00000000;
	WriteByte_I2C(BME280_ADDRESS, BME280_REGISTER_CONTROL, ctrl_meas);
	delay(100);
	Serial.print("BME280_REGISTER_CONTROL ");
	Serial.println(ReadByte_I2C(BME280_ADDRESS, BME280_REGISTER_CONTROL), BIN);


	byte config = duration << 5 | filter << 2 | 00;	  // Формируем байт с настройками задержки между измерениями в нормальном режиме и настройки фильтра и последние байты для SPI тут не используем
	WriteByte_I2C(BME280_ADDRESS, BME280_REGISTER_CONFIG, config);
	delay(100);
	Serial.print("BME280_REGISTER_CONFIG ");
	Serial.println(ReadByte_I2C(BME280_ADDRESS, BME280_REGISTER_CONFIG), BIN);


	ctrl_meas = ctrl_meas | 0b00000011;				   //Те же настрйки и включаем нормальный режим.
	WriteByte_I2C(BME280_ADDRESS, BME280_REGISTER_CONTROL, ctrl_meas);
	delay(100);
	Serial.print("BME280_REGISTER_CONTROL ");
	Serial.println(ReadByte_I2C(BME280_ADDRESS, BME280_REGISTER_CONTROL), BIN);

}

void Init_BME280()
{
	Serial.println("***********************************************************");

	Serial.println(F("BMP280/BME280 Init"));
	Serial2.println(" Init_BME280 ");


	// check if sensor, i.e. the chip ID is correct
	byte b280_ID = ReadByte_I2C(BME280_ADDRESS, BME280_REGISTER_CHIPID);
	if (b280_ID == 0x58)
	{
		Serial.print("BMP280_ID = ");  Serial.println(b280_ID, HEX);
	}
	if (b280_ID == 0x60)
	{
		Serial.print("BME280_ID = ");  Serial.println(b280_ID, HEX);
	}

	// reset the device using soft-reset    this makes sure the IIR is off, etc.
	WriteByte_I2C(BME280_ADDRESS, BME280_REGISTER_SOFTRESET, 0xB6);
	// wait for chip to wake up.
	delay(100);
	while (ReadByte_I2C(BME280_ADDRESS, BME280_REGISTER_STATUS) != 0); {	delay(10);	}	//Ждем когда колибровычные данные скопируются в регистры

	readCoefficients(); // read trimming parameters, see DS 4.2.2

	setSampling(
		MODE_NORMAL,   //Режим работы
		SAMPLING_X4,  // temperature
		SAMPLING_X2, // pressure
		SAMPLING_X2,  // humidity
		FILTER_X2,	   //Фильтр
		STANDBY_MS_500);   //Задержка
//	// suggested rate is 25Hz
//	// 1 + (2 * T_ovs) + (2 * P_ovs + 0.5) + (2 * H_ovs + 0.5)
//	// T_ovs = 2
//	// P_ovs = 16
//	// H_ovs = 2
//	// = 42ms (25Hz)
//	// with standby time that should really be 24.16913... Hz
//	//delayTime = 50;

	//while (1);

}


uint32_t read24(byte reg)		//	Reads a 24 bit value over I2C
{
	uint32_t value;
	Wire.beginTransmission((uint8_t)BME280_ADDRESS);
	Wire.write((uint8_t)reg);
	int reza = Wire.endTransmission();
	if (reza != 0)
	{
		Serial.print("!!! BME280 read24 Mistake reza = ");Serial.println(reza);
		Serial2.print("!!! BME280 read24 Mistake reza = ");Serial2.println(reza);

	};
	int rezb = Wire.requestFrom((uint8_t)BME280_ADDRESS, (byte)3);
	if (rezb == 3)         // Если вернулось  столько сколько просили
	{
		value = Wire.read();
		value <<= 8;
		value |= Wire.read();
		value <<= 8;
		value |= Wire.read();
		return value;
	}
	else
	{
		Serial.print("!!! BME280 read24 Mistake rezb = ");Serial.println(rezb);
		Serial2.print("!!! BME280 read24 Mistake rezb = ");Serial2.println(rezb);
		return 0;

	}
}
uint32_t read16(byte reg)		//	Reads a 16 bit value over I2C
{
	uint32_t value;
	Wire.beginTransmission((uint8_t)BME280_ADDRESS);
	Wire.write((uint8_t)reg);
	int reza = Wire.endTransmission();
	if (reza != 0)
	{
		Serial.print("!!! BME280 read16 Mistake reza = ");Serial.println(reza);
		Serial2.print("!!! BME280 read16 Mistake reza = ");Serial2.println(reza);

	};

	int rezb = Wire.requestFrom((uint8_t)BME280_ADDRESS, (byte)2);
	if (rezb == 2)         // Если вернулось  столько сколько просили
	{
		value = Wire.read();
		value <<= 8;
		value |= Wire.read();
		return value;
	}
	else
	{
		Serial.print("!!! BME280 read16 Mistake rezb = ");Serial.println(rezb);
		Serial2.print("!!! BME280 read16 Mistake rezb = ");Serial2.println(rezb);
		return 0;
	}

}


float readTemperature(void)		//	   Returns the temperature from the sensor
{
	int32_t var1, var2;

	int32_t adc_T = read24(BME280_REGISTER_TEMPDATA);
	if (adc_T == 0x800000) // value in case temp measurement was disabled
		return NAN;
	adc_T >>= 4;

	var1 = ((((adc_T >> 3) - ((int32_t)_bme280_calib.dig_T1 << 1))) *
		((int32_t)_bme280_calib.dig_T2)) >> 11;

	var2 = (((((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1)) *
		((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1))) >> 12) *
		((int32_t)_bme280_calib.dig_T3)) >> 14;

	t_fine = var1 + var2;

	float T = (t_fine * 5 + 128) >> 8;
	return T / 100;
}

float readHumidity(void) 	   //	   Returns the humidity from the sensor
{
	//readTemperature(); // must be done first to get t_fine

	int32_t adc_H = read16(BME280_REGISTER_HUMIDDATA);
	if (adc_H == 0x8000) // value in case humidity measurement was disabled
		return NAN;

	int32_t v_x1_u32r;

	v_x1_u32r = (t_fine - ((int32_t)76800));

	v_x1_u32r = (((((adc_H << 14) - (((int32_t)_bme280_calib.dig_H4) << 20) -
		(((int32_t)_bme280_calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
		(((((((v_x1_u32r * ((int32_t)_bme280_calib.dig_H6)) >> 10) *
		(((v_x1_u32r * ((int32_t)_bme280_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
			((int32_t)2097152)) * ((int32_t)_bme280_calib.dig_H2) + 8192) >> 14));

	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
		((int32_t)_bme280_calib.dig_H1)) >> 4));

	v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
	v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
	float h = (v_x1_u32r >> 12);
	return  h / 1024.0;
}
/**************************************************************************/
float readPressure(void) {
	int64_t var1, var2, p;

	//readTemperature(); // must be done first to get t_fine

	int32_t adc_P = read24(BME280_REGISTER_PRESSUREDATA);
	if (adc_P == 0x800000) // value in case pressure measurement was disabled
		return NAN;
	adc_P >>= 4;

	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)_bme280_calib.dig_P6;
	var2 = var2 + ((var1*(int64_t)_bme280_calib.dig_P5) << 17);
	var2 = var2 + (((int64_t)_bme280_calib.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)_bme280_calib.dig_P3) >> 8) +
		((var1 * (int64_t)_bme280_calib.dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1))*((int64_t)_bme280_calib.dig_P1) >> 33;

	if (var1 == 0) {
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)_bme280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)_bme280_calib.dig_P8) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((int64_t)_bme280_calib.dig_P7) << 4);
	return (float)p / 256;
}





void Read_BME280_TemperHumi()    // Примерно 3 милисекунды
{
	//long a = micros();

	TemperatureBME280 = readTemperature();	//Всегда вызыввается первой так как от туда переменна яиспользуется в других расчетах
	//long b = micros();
	HumidityBME280 = readHumidity();   // Внутри МОЖЕТ!!! вызывается расчет температуры и специально можно не вызывать
	//long c = micros();
	//PressureBME280 = readPressure() / 100.0F;
	//long d = micros();

	//PressureBME280_mm = PressureBME280 / 1.3333;        // В миллиметрах ртутного столба
	//Serial.println(HumidityBME280);
	// Расчет высоты над уровнем моря

	//Serial.print("Approx. Altitude = ");
	//Serial.print(myBME280.readAltitude(SEALEVELPRESSURE_HPA));
	//Serial.println(" m");
	//Serial.print(" Humidity_BME280= "); Serial.println(b - a);
	//Serial.print(" Temperature_BME280= "); Serial.println(c - b);
	//Serial.print(" PressureBME280= "); Serial.println(d - c);

	//Serial.print(" All_BME280= "); Serial.println(d - a);

}
void Read_BME280_Pressure()    // Примерно 3 милисекунды
{
	PressureBME280 = readPressure() / 100.0F;
	PressureBME280_mm = PressureBME280 / 1.3333;        // В миллиметрах ртутного столба
	//Serial.print(" PressureBME280= "); Serial.println(d - c);
}




//*************************** SI7021 ------------------------------------

/*					    Страница 18 даташида
Table 11. I2C Command Table
		Command Description Command Code
		Measure Relative Humidity, Hold Master Mode 0xE5
		Measure Relative Humidity, No Hold Master Mode 0xF5
		Measure Temperature, Hold Master Mode 0xE3
		Measure Temperature, No Hold Master Mode 0xF3
		Read Temperature Value from Previous RH Measurement 0xE0
		Reset 0xFE
		Write RH/T User Register 1 0xE6
		Read RH/T User Register 1 0xE7
		Write Heater Control Register 0x51
		Read Heater Control Register 0x11
		Read Electronic ID 1st Byte 0xFA 0x0F
		Read Electronic ID 2nd Byte 0xFC 0xC9
		Read Firmware Revision 0x84 0xB8
*/
//
//#define Adress_Si7021 0x40      // Адрес по которому на шине I2C находится датчик
//
//
//unsigned int readSi7021(uint8_t command)
//{
//	Wire.beginTransmission(Adress_Si7021);
//	Wire.write(command);
//	Wire.endTransmission();                   // difference to normal read
//	Wire.requestFrom((uint8_t)Adress_Si7021, (uint8_t)3);        //Запрашиваем 3 байта. 3 байт чексумма
//	unsigned int msb = Wire.read();
//	unsigned int lsb = Wire.read();
//	//byte chek = Wire.read();
//
//	// Clear the last to bits of LSB to 00.
//	// According to datasheet LSB of RH is always xxxxxx10
//	lsb &= 0xFC;
//	unsigned int mesurment = msb << 8 | lsb;
//	Wire.endTransmission();
//	//Serial.print(" = "); Serial.print(chek,BIN);
//
//
//	return    mesurment;
//};

//float readHumiditySi7021(uint8_t command)
//{
//	return  (readSi7021(command) * 125.0 / 65536) - 6;;
//};
//
//float readTempSi7021(uint8_t command)
//{
//	return  (readSi7021(command) * 175.72 / 65536) - 46.85;
//};
//
//void ResetSi7021()
//{
//	Wire.beginTransmission(Adress_Si7021);
//	Wire.write(0xFE);
//	Wire.endTransmission();
//};
//
//void setResolutionSi7021(uint8_t Resolution)
//{
//	Wire.beginTransmission(Adress_Si7021);
//	Wire.write(0xE6);
//	Wire.write(Resolution);
//	Wire.endTransmission();
//};
//
//uint8_t checkIDSi7021()
//{
//	uint8_t ID_1;
//	// Check device ID
//	Wire.beginTransmission(Adress_Si7021);
//	Wire.write(0xFC);
//	Wire.write(0xC9);
//	Wire.endTransmission();
//
//	Wire.requestFrom(Adress_Si7021, 1);
//	ID_1 = Wire.read();
//	Wire.endTransmission();
//	return(ID_1);
//}

//void Init_Si7021()
//{
//	Serial.print("Check ID: "); Serial.println(checkIDSi7021(), HEX);
//
//	ResetSi7021();                 // Reset
//	//setResolutionSi7021(0b00000000);   // Устанавливаем точность измерения. 
//	setResolutionSi7021(0b10000001);   // Устанавливаем точность измерения. Это быстрее но менее точно
//									   //От нее зависит длительность измерения от 2 до 16 милисекунд см даташид 5 и 26 страница
//}

//void Read_Si7021()
//{
//	long a = micros();
//	Humidity_Si7021 = readHumiditySi7021(0xE5);   //0xE5	0xF5	Это режим без ожидания НЕ РАБОТАЕТ !!!! смотри даташид  18 страница режимы чтения
//	long b = micros();
//	Temperature_Si7021 = readTempSi7021(0xE3);	   //0xE3   0xF3
//	long c = micros();
//	Serial.print(" HUMI= "); Serial.print(Humidity_Si7021);
//	Serial.print(" Temp= "); Serial.println(Temperature_Si7021);
//	Serial.print(" Humidity_Si7021= "); Serial.println(b - a);
//	Serial.print(" Temperature_Si7021= "); Serial.println(c - b);
//	//Serial.print(" All_Si7021= "); Serial.println(c - a);
//	Serial.println("-");
//
//}

void Print_BME280()
{
	//------------------------------------------------------------------------------------
	Serial.print(" Temperature = ");
	Serial.print(TemperatureBME280);
	Serial.print(" *C");
	//------------------------------------------------------------------------------------
	Serial.print(" Pressure = ");
	Serial.print((float)(PressureBME280*0.75006375f), 2);
	Serial.print(" mm");
	Serial2.print(" Pressure = ");
	Serial2.print((float)(PressureBME280*0.75006375f), 2);
	Serial2.println(" mm");

	//------------------------------------------------------------------------------------
	Serial.print(" Humidity = ");
	Serial.print(HumidityBME280);
	Serial.println(" %");
	//------------------------------------------------------------------------------------

}


void Loop_BME280()
{
	//==========================================================
	if (flag_BME280 == true)	 // Примерно 0.7 милисекунды  !!!
	{
		flag_BME280 = false;
		//long a = micros();
		static byte porcia_BME280 = 1;

		switch (porcia_BME280)
		{
		case 1:  Read_BME280_TemperHumi();   porcia_BME280 = 2;	break;
		case 2:  Read_BME280_Pressure();     porcia_BME280 = 1;	break;
		}
		Print_BME280();
		//long b = micros();
		//Serial.print("Time flag_BME280 = "); 	 Serial.println(b - a);

	}
}
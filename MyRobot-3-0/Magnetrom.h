uint8_t MAG_ADR = 0x0c;
int16_t mx, my, mz;
bool  StatusColibrMag = false;   // Статут был ли откалиброван магнетрометр
float intervalMag, preintervalMag;
uint8_t ASAX, ASAY, ASAZ;
float mx_sensitivity, my_sensitivity, mz_sensitivity;
float magXOffset, magYOffset, magZOffset;
float magXMin, magYMin, magZMin;
float magXMax, magYMax, magZMax;
float magX, magY, magZ;
float magXScale, magYScale, magZScale, magScaleAvg;




void Init_AK8963()
{
	Serial.println("--------------------------------------");
	Serial.println(" Start connected to COMPASS(AK8963)...");
	Serial2.println(" Init_AK8963 ");


	// Connect to COMPASS(AK8963) via AUX_i2c 

  //Wire.beginTransmission(0x0C);         // 20 micro sec
 // Wire.write(0x00);               // 100 micro sec
 // Wire.endTransmission();                // 120 micro sec
 // Wire.requestFrom(0x0C, 1);      //700 micro sec
 // uint8_t data = Wire.read();//read one byte of data
 // Serial.print("Data2: "); Serial.println(data,HEX);

	uint8_t WIA_MAG = ReadByte_I2C(0x0C, 0x00);
	Serial.print("WIA_MAG: "); Serial.print(WIA_MAG, HEX);

	if (WIA_MAG == 0b01001000)
	{
		Serial.println(" Successfully connected to COMPASS(AK8963)");
		// CONTROL(0x0A) -> 0x00
		// Reset Magnet Compass
	  //  I2Cdev::writeByte(MAG_ADR, 0x0a, 0x00);

		WriteByte_I2C(0x0C, 0x0A, 0x00);// Reset Magnet Compass
		delay(10);

		// CONTROL(0x0A) -> 0x16
		// BIT       = 1  -> 16 bit output
		// MODE[3:0] = 0110b -> continuous measurement mode 2(100Hz);
		//I2Cdev::writeByte(MAG_ADR, 0x0a, 0x16);

		WriteByte_I2C(0x0C, 0x0A, 0b00010110);

		uint8_t Aregistr = ReadByte_I2C(0x0C, 0x0A);        //Проверяем что записалось в регистр
		// Serial.print(" Aregistr "); Serial.print( Aregistr,BIN );

	// Считываем коефицианты учета чувствительности
		ASAX = ReadByte_I2C(0x0C, 0x10);
		ASAY = ReadByte_I2C(0x0C, 0x11);
		ASAZ = ReadByte_I2C(0x0C, 0x12);
		//  Serial.print(" ASAX "); Serial.print( ASAX );
		//  Serial.print(" ASAY "); Serial.print( ASAY);
	   //   Serial.print(" ASAZ "); Serial.println( ASAZ);

	 // По формуле из даташита переволим в микро Теслу
		mx_sensitivity = ((((float)ASAX) - 128.0f) / (256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
		my_sensitivity = ((((float)ASAY) - 128.0f) / (256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
		mz_sensitivity = ((((float)ASAZ) - 128.0f) / (256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla

	  //  Serial.print(" mx_sensitivity "); Serial.print( mx_sensitivity,4 );
	 //   Serial.print(" my_sensitivity "); Serial.print( my_sensitivity,4);
	 //   Serial.print(" mz_sensitivity "); Serial.println( mz_sensitivity,4);

	  // Записываем OffSet и magScale измеренный в другой прграмме по многим измерениям и вращениям датчика. 
	  // Приминимо только примерно в одной местности. В других условиях нужно колибровать заново.

	   //9050 - колибровка без значений по умолчанию пустая	   Начальные значения. потом или колибкуются или считываются
		magXOffset = 0;      magYOffset = 0;      magZOffset = 0;
		magXScale = 1.0;      magYScale = 1.0;     magZScale = 1.0;
	}
	else
	{
		Serial.println("Failed to Connect to COMPASS(AK8963) !!!!!!!!!!!!!!!!!!!");
		delay(10000);
	}

}

void Read_AK8963()    // Чтение данных с магнетрометра	 ВРЕМЯ ИСПОЛНЕНИЯ 700 микросекунд
{
	//Serial.print("+");
	//Serial.print("AK");
	//Serial2.print("*");
	//Serial2.print("WIA_MAG: "); Serial2.println(ReadByte_I2C(0x0C, 0x00), HEX);
	uint8_t buffer_gyro[7];
	
	Wire.beginTransmission(0x0C);
	Wire.write(0x03);
	int reza = Wire.endTransmission();
	if (reza != 0)
	{
		Serial.print("!!! Read_AK8963 WriteMistake reza = "); Serial.println(reza);
		Serial2.print("!!! Read_AK8963 WriteMistake reza = "); Serial2.println(reza);

	};
	byte rezb = Wire.requestFrom(0x0C, (int)7);			   // Чмтаем 7 байт. Последний байт читаем как признак что считали- это обязательно
	if (rezb == 7)
	{
		for (int i = 0; i < 7; i++)
		{
			buffer_gyro[i] = Wire.read();             //read one byte of data
		}
	}
	else
	{
		Serial.print("!!! Read_AK8963 WriteMistake rezb = ");	Serial.println(rezb);
		Serial2.print("!!! Read_AK8963 WriteMistake rezb = ");	Serial2.println(rezb);

	}

	
	mx = (((int16_t)buffer_gyro[1]) << 8) | buffer_gyro[0];
	my = (((int16_t)buffer_gyro[3]) << 8) | buffer_gyro[2];
	mz = (((int16_t)buffer_gyro[5]) << 8) | buffer_gyro[4];
	//int reg2 =  buffer_acc_gyro[6];
	//Serial.print(" mx ");	Serial.print(mx);
	//Serial.print(" my ");	Serial.print(my);
	//Serial.print(" mz ");	Serial.println(mz);


	magX = ((mx * mx_sensitivity) - magXOffset) * magXScale;
	magY = ((my * my_sensitivity) - magYOffset) * magYScale;
	magZ = ((mz * mz_sensitivity) - magZOffset) * magZScale;
	magD = atan2(magX, magY) * RAD_TO_DEG;

	//Serial.print(" magD ");	Serial.print(magD);
	magD = 180 - magD;	   // Перевод в 360 градусов такак как стандартно идет от -180 до +180
	//Serial.print(" magD ");	Serial.println(magD);

	//magD = magD + 180;    // Погоняем под положение второго датчика так как они развернуты на 180 градусов

	//if (magD < 0) 
	//	{ magD = 180 - magD; } 
	//else 
	//	{ magD = 180 - magD; }  // Перевод в 360 градусов

	//Serial.print(" magX ");	Serial.print(magX);
	//Serial.print(" magY ");	Serial.print(magY);
	//Serial.print(" magZ ");	Serial.print(magZ);
	//Serial.print(" magD ");	Serial.println(magD);


	//Serial.print("-");
	//Serial.println("AK+");

}
void Test_Colibrovka_Magnetrometr()			//Функция что-бы покрутиться кругом и вывести на экран значения
{
	Serial.println(" Test_Colibrovka_Magnetrometr ");
	byte speed = 50;

	Run_MotorL(speed);
	Run_MotorR(-speed);

	int i = 0;
	while (i < 1000)
	{
		i++;
		delay(11);
		Read_AK8963();  //Запрашиваем данные с магнетрометра

		Serial.print(i);
		Serial.print(" ");  Serial.print(magX);
		Serial.print(" ");  Serial.print(magY);
		Serial.print(" ");  Serial.print(magZ);
		Serial.print(" ");  Serial.println(magD);
	}

	Run_MotorL(0);
	Run_MotorR(0);

	delay(1000);

	Run_MotorL(-speed);
	Run_MotorR(speed);
	i = 0;
	while (i < 1000)
	{
		i++;
		delay(11);
		Read_AK8963();  //Запрашиваем данные с магнетрометра

		Serial.print(i);
		Serial.print(" ");  Serial.print(magX);
		Serial.print(" ");  Serial.print(magY);
		Serial.print(" ");  Serial.println(magZ);
	}

	Run_MotorL(0);
	Run_MotorR(0);
}

void ReadCalibrovka_Mag()
{
	if (eeprom_read_byte(64) == 1)   // Если есть поправочные значения то считываем их
	{
		Serial.println(" Collibrovka coefficient EEPROM  64 adress ColibrMag EEPROM Ok");
		magXOffset = eeprom_read_float(66);
		magYOffset = eeprom_read_float(70);
		magZOffset = eeprom_read_float(74);
		magXScale = eeprom_read_float(78);
		magYScale = eeprom_read_float(82);
		magZScale = eeprom_read_float(86);
		
		//Colibrovka_krugom();		 //Если надо то запускаем тест кругом и смотрим значения

		Serial.print("MX : ");Serial.println(magXOffset, 6);
		Serial.print("MY : ");Serial.println(magYOffset, 6);
		Serial.print("MZ : ");Serial.println(magZOffset, 6);
		Serial.print("MSX : ");Serial.println(magXScale, 6);
		Serial.print("MSY : ");Serial.println(magYScale, 6);
		Serial.print("MSZ : ");Serial.println(magZScale, 6);
		Serial.println(" ----------------------------------------------");
		Read_AK8963();  //Запрашиваем данные с магнетрометра для присвоения начального значения положения углу гироскопа
		Serial.print(" magX ");  Serial.print(magX);
		Serial.print(" magY ");  Serial.print(magY);
		Serial.print(" magZ ");  Serial.println(magZ);
		Serial.print("magD ");  Serial.println(magD);
		angleGyroZ = magD;                                  // Присваиваем начальное значение
		angleCompZ = magD;									 // Присваиваем начальное значение
		Z_angle_comp = magD;								 // Присваиваем начальное значение
		BNO055_EulerAngles.my_z = magD;						 // Присваиваем начальное значение
		Serial.println(" Starting position gyroskop set.");
		StatusColibrMag = true;      // Меняем статус на истину
	}
	else
	{
		Serial.println(" NOT coefficient EPROM 64 adress ColibrMag EEPROM Error");

	}
}

void Calibrovka_Mag()
{
	// Обнуляем значения офсетов и шкалы
	magXOffset = 0;      magYOffset = 0;      magZOffset = 0;
	magXScale = 1;       magYScale = 1;       magZScale = 1;


	Read_AK8963();      // Первое измерение что-бы было какие минимальные и максимальные значения установить в начальные значения
	magXMin = magXMax = magX;
	magYMin = magYMax = magY;
	magZMin = magZMax = magZ;

	int speed = 125;

	//StatusMotor = -1;   // Меняем текущий статус мотора что-бы не мешал командам управления
	Serial.println(" LeftMotor ");

	Run_MotorL( 50);
	Run_MotorR(-50);

	int i = 0;
	while (i < 1000)
	{
		i++;
		delay(11);
		Read_AK8963();  //Запрашиваем данные с магнетрометра

		// защита от случайных значений которые проскакивают
		//if (magX > 0) magX = 0;if (magX < -50) magX = -50;
		//if (magY > 80) magY = 80;if (magY < -0) magY = 0;

		Serial.print(i);
		Serial.print(" ");  Serial.print(magX);
		Serial.print(" ");  Serial.print(magY);
		Serial.print(" ");  Serial.println(magZ);


		if (magX > magXMax) magXMax = magX;
		if (magY > magYMax) magYMax = magY;
		if (magZ > magZMax) magZMax = magZ;
		if (magX < magXMin) magXMin = magX;
		if (magY < magYMin) magYMin = magY;
		if (magZ < magZMin) magZMin = magZ;
	}

	Run_MotorL(0);
	Run_MotorR(0);

	delay(1000);

	Run_MotorL(-50);
	Run_MotorR( 50);
	i = 0;
	while (i < 1000)
	{
		i++;
		delay(11);
		Read_AK8963();  //Запрашиваем данные с магнетрометра

		// защита от случайных значений которые проскакивают
		//if (magX > 0) magX = 0;if (magX < -50) magX = -50;
		//if (magY > 80) magY = 80;if (magY < -0) magY = 0;

		Serial.print(i);
		Serial.print(" ");  Serial.print(magX);
		Serial.print(" ");  Serial.print(magY);
		Serial.print(" ");  Serial.println(magZ);


		if (magX > magXMax) magXMax = magX;
		if (magY > magYMax) magYMax = magY;
		if (magZ > magZMax) magZMax = magZ;
		if (magX < magXMin) magXMin = magX;
		if (magY < magYMin) magYMin = magY;
		if (magZ < magZMin) magZMin = magZ;
	}

	Run_MotorL(0);
	Run_MotorR(0);

	magXOffset = (magXMax + magXMin) / 2.0f;    // find the magnetometer bias
	magYOffset = (magYMax + magYMin) / 2.0f;
	magZOffset = (magZMax + magZMin) / 2.0f;

	magXScale = (magXMax - magXMin) / 2.0f;    // find the magnetometer scale factor
	magYScale = (magYMax - magYMin) / 2.0f;
	magZScale = (magZMax - magZMin) / 2.0f;
	magScaleAvg = (magXScale + magYScale + magZScale) / 3.0f;

	magXScale = magScaleAvg / magXScale;
	magYScale = magScaleAvg / magYScale;
	magZScale = magScaleAvg / magZScale;


	delay(1000);
	Run_MotorL(50);
	Run_MotorR(-50);

	i = 0;
	while (i < 1000)
	{
		i++;
		delay(11);
		Read_AK8963();  //Запрашиваем данные с магнетрометра

		// защита от случайных значений которые проскакивают
		//if (magX > 0) magX = 0;if (magX < -50) magX = -50;
		//if (magY > 80) magY = 80;if (magY < -0) magY = 0;

		Serial.print(i);
		Serial.print(" ");  Serial.print(magX);
		Serial.print(" ");  Serial.print(magY);
		Serial.print(" ");  Serial.println(magZ);


		if (magX > magXMax) magXMax = magX;
		if (magY > magYMax) magYMax = magY;
		if (magZ > magZMax) magZMax = magZ;
		if (magX < magXMin) magXMin = magX;
		if (magY < magYMin) magYMin = magY;
		if (magZ < magZMin) magZMin = magZ;
	}

	Run_MotorL(0);
	Run_MotorR(0);

	delay(1000);

	Run_MotorL(-50);
	Run_MotorR( 50);


	i = 0;
	while (i < 1000)
	{
		i++;
		delay(11);
		Read_AK8963();  //Запрашиваем данные с магнетрометра

		// защита от случайных значений которые проскакивают
		//if (magX > 0) magX = 0;if (magX < -50) magX = -50;
		//if (magY > 80) magY = 80;if (magY < -0) magY = 0;

		Serial.print(i);
		Serial.print(" ");  Serial.print(magX);
		Serial.print(" ");  Serial.print(magY);
		Serial.print(" ");  Serial.println(magZ);


		if (magX > magXMax) magXMax = magX;
		if (magY > magYMax) magYMax = magY;
		if (magZ > magZMax) magZMax = magZ;
		if (magX < magXMin) magXMin = magX;
		if (magY < magYMin) magYMin = magY;
		if (magZ < magZMin) magZMin = magZ;
	}

	Run_MotorL(0);
	Run_MotorR(0);


	//delay(1000);
	//Синхронизируем углы один раз
	Read_AK8963();  //Запрашиваем данные с магнетрометра
	delay(1000);
	Serial.print(" magD ");  Serial.print (magD, 2);
	//Serial.print(" angleGyroZ ");  Serial.println(angleGyroZ, 2);
	angleGyroZ = magD;
	Serial.print(" angleGyroZ ");  Serial.println(angleGyroZ);

	//Serial.print(" magD ");  Serial.println(magD, 2);

	delay(1000);

	StatusColibrMag = true;      // Меняем статус на истину


	if (1)
	{
		Serial.print(" ============================START COLIBRATION MAGNEROMETR ======================== ");
		Serial.println("");

		Serial.print(" magScaleAvg ");  Serial.println(magScaleAvg, 2);

		Serial.print(" magXMin ");  Serial.print(magXMin, 3);
		Serial.print(" magXMax ");  Serial.print(magXMax, 3);
		Serial.print(" magXOffset ");  Serial.print(magXOffset, 3);
		Serial.print(" magX-Centre ");  Serial.println(magXMax - magXOffset);
		Serial.print(" magXScale ");  Serial.println(magXScale, 3);

		Serial.print(" magYMin ");  Serial.print(magYMin, 3);
		Serial.print(" magYMax ");  Serial.print(magYMax, 3);
		Serial.print(" magYOffset ");  Serial.print(magYOffset, 3);
		Serial.print(" magY-Centre ");  Serial.println(magYMax - magYOffset);
		Serial.print(" magYScale ");  Serial.println(magYScale, 3);

		Serial.print(" magZMin ");  Serial.print(magZMin, 3);
		Serial.print(" magZMax ");  Serial.print(magZMax, 3);
		Serial.print(" magZOffset ");  Serial.print(magZOffset, 3);
		Serial.print(" magZ-Centre ");  Serial.println(magZMax - magZOffset);
		Serial.print(" magZScale ");  Serial.println(magZScale, 3);

		Serial.println(" ============================END COLIBRATION MAGNEROMETR ======================== ");

		// --------------------- Запись в ячейки памяти ----------------------------------------
		Serial.println(" ============================START EEPROM COLIBRATION MAGNEROMETR ======================== ");
		eeprom_write_byte(64, 1);     // записываем признак что есть данные колибровки в 40 адрес
		Serial.print("Flag Mag: "); Serial.println(eeprom_read_byte(40));

		eeprom_write_float(66, magXOffset);
		Serial.print("magX Offset : "); Serial.println(eeprom_read_float(66), 6);
		eeprom_write_float(70, magYOffset);
		Serial.print("magY Offset : "); Serial.println(eeprom_read_float(70), 6);
		eeprom_write_float(74, magZOffset);
		Serial.print("magZ Offset : "); Serial.println(eeprom_read_float(74), 6);

		eeprom_write_float(78, magXScale);
		Serial.print("magX Scale : "); Serial.println(eeprom_read_float(78), 6);
		eeprom_write_float(82, magYScale);
		Serial.print("magY Scale : "); Serial.println(eeprom_read_float(82), 6);
		eeprom_write_float(86, magZScale);
		Serial.print("magZ Scale : "); Serial.println(eeprom_read_float(86), 6);
		Serial.println(" ============================END EEPROM COLIBRATION MAGNEROMETR ======================== ");
		// --------------------- Запись в ячейки памяти ----------------------------------------

	}
	delay(1000);
}

void Loop_AK8963()
{
	//==========================================================
	if (flag_magnetrom == true)
	{
		flag_magnetrom = false;
		Read_AK8963();					  // ВРЕМЯ ИСПОЛНЕНИЯ 1 МИЛИСЕКУНДА
		//Serial.print("magD ");  Serial.println(magD);
		//PrintDirection();	
		//Serial.println(" ");
		//Serial.print(" R_PWM : "); Serial.print(MotorR.motor_pwm);
		//Serial.print(" R_motor_speed : "); Serial.println(MotorR.motor_speed, 4);
	}


}
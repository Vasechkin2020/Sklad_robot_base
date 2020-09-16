
const int16_t Adr_DS3231 = 0x68; // I2C адрес таймера DS3231
byte dateTime[7]; // 7 байтов дл€ хранени€ даты и времени
static uint8_t bcd2bin(uint8_t val) { return val - 6 * (val >> 4); }  // ѕеревод из формата числа когда оно в 16 разр€дном виде записано как 10 значное
static uint8_t bin2bcd(uint8_t val) { return val + 6 * (val / 10); }  // ѕеревод дес€тизначного числа в формат 16 разр€дного записанного как дес€тичное
void printDateTime(byte *dateTime)
{
	if (dateTime[4] < 10) Serial.print("0");
	Serial.print(dateTime[4], HEX); // выводим дату
	Serial.print(".");
	if (dateTime[5] < 10) Serial.print("0");
	Serial.print(dateTime[5], HEX); // выводим мес€ц
	Serial.print(".20");
	Serial.print(dateTime[6], HEX); // выводим год
	Serial.print(" ");
	if (dateTime[2] < 10) Serial.print("0");
	Serial.print(dateTime[2], HEX); // выводим час
	Serial.print(":");
	if (dateTime[1] < 10) Serial.print("0");
	Serial.print(dateTime[1], HEX); // выводим минуты
	Serial.print(":");
	if (dateTime[0] < 10) Serial.print("0");
	Serial.println(dateTime[0], HEX); // выводим секунды
}

void Read_DS3231()
{
	//Serial.print("dT");
	Wire.beginTransmission(Adr_DS3231); // начинаем обмен с DS3231
	Wire.write(byte(0x00)); // записываем адрес регистра, с которого начинаютс€ данные даты и времени
	Wire.endTransmission(); // завершаем передачу

	int i = 0; // индекс текущего элемента массива

	//Wire.beginTransmission(Adr_DS3231); // начинаем обмен с DS3231
	Wire.requestFrom(Adr_DS3231, (int16_t)7); // запрашиваем 7 байтов у DS3231
	while (Wire.available()) // пока есть данные от Adr_DS3231
	{
		dateTime[i] = Wire.read(); // читаем 1 байт и сохран€ем в массив dateTime
		i += 1; // инкрементируем индекс элемента массива
	}
	Wire.endTransmission(); // завершаем передачу
	//Serial.println("dT");
	//printDateTime(dateTime); // выводим дату и врем€
}





//void Set_Adr_DS3231() 
//{
//	//ѕолучаем текущую дату врем€ с локального компьютера.
//	time_t rawtime;
//	struct tm * timeinfo;
//
//	time(&rawtime);                               // получить текущую дату, выраженную в секундах
//	timeinfo = localtime(&rawtime);
//	int a = timeinfo->tm_hour;
//	Serial.print(" aaaa = ");	  	Serial.println(a);
//	
//	Wire.beginTransmission(0x68); // начинаем обмен с Adr_DS3231 с i2c адресом 0x68
//	byte arr[] = { 0x00, 0x02, 0x30, 0x17, 0x03, 0x02, 0x01, 0x19 };
//	Wire.write(arr, 8); // записываем 8 байтов массива arr
//	Wire.endTransmission(); // завершение передачи
//}

void Loop_DataTime()
{
	//==========================================================
	if (flag_datatime == true)
	{
		//digitalWrite(49, 1);

		flag_datatime = false;
		Read_DS3231();				  // ¬–≈ћя »—ѕќЋЌ≈Ќ»я 0.5 ћ»Ћ»—≈ ”Ќƒ

		//Write32F_I2C(0x09, 0xF1, -2519.9333);
		//printDateTime(dateTime); // выводим дату и врем€
		//Serial.print(" MotorL.motor_speed: "); Serial.print(MotorL.motor_speed, 4);
	//	Serial.print(" MotorR.motor_pwm: "); Serial.print(MotorR.motor_pwm);
	//	Serial.print(" MotorR.motor_speed: "); Serial.print(MotorR.motor_speed, 4);
		//Serial.print(" MotorR.motor_way: "); Serial.print(MotorR.motor_way, 4);
	//	Serial.println("");
		//printDateTime(dateTime); // выводим дату и врем€
		//Serial.print("  Datchik_L_Pered.Distancia "); 		Serial.println(Datchik_L_Pered.Distancia);
		//Serial.print("  Datchik_R_Pered.Distancia "); 		Serial.println(Datchik_R_Pered.Distancia);
		//Serial.print("  Datchik_L_Zad.Distancia "); 		Serial.println(Datchik_L_Zad.Distancia);
		//Serial.print("  Datchik_R_Zad.Distancia "); 		Serial.println(Datchik_R_Zad.Distancia);

		//Serial.print(" x: "); 	 Serial.print(BNO055_EulerAngles.x, 2);
		//Serial.print(" y: "); 	 Serial.print(BNO055_EulerAngles.y, 2);
		//Serial.print(" z: "); 	 Serial.println(BNO055_EulerAngles.z, 2);
		//PrintDirection();	
		//PrintAngle();
		//PrintAngleCompFiltr();
		//PrintAngleTest();
		//PrintDirection();
		//Serial.println("");
	}
	//==========================================================
}
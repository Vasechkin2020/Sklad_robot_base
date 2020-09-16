
const int16_t Adr_DS3231 = 0x68; // I2C ����� ������� DS3231
byte dateTime[7]; // 7 ������ ��� �������� ���� � �������
static uint8_t bcd2bin(uint8_t val) { return val - 6 * (val >> 4); }  // ������� �� ������� ����� ����� ��� � 16 ��������� ���� �������� ��� 10 �������
static uint8_t bin2bcd(uint8_t val) { return val + 6 * (val / 10); }  // ������� �������������� ����� � ������ 16 ���������� ����������� ��� ����������
void printDateTime(byte *dateTime)
{
	if (dateTime[4] < 10) Serial.print("0");
	Serial.print(dateTime[4], HEX); // ������� ����
	Serial.print(".");
	if (dateTime[5] < 10) Serial.print("0");
	Serial.print(dateTime[5], HEX); // ������� �����
	Serial.print(".20");
	Serial.print(dateTime[6], HEX); // ������� ���
	Serial.print(" ");
	if (dateTime[2] < 10) Serial.print("0");
	Serial.print(dateTime[2], HEX); // ������� ���
	Serial.print(":");
	if (dateTime[1] < 10) Serial.print("0");
	Serial.print(dateTime[1], HEX); // ������� ������
	Serial.print(":");
	if (dateTime[0] < 10) Serial.print("0");
	Serial.println(dateTime[0], HEX); // ������� �������
}

void Read_DS3231()
{
	//Serial.print("dT");
	Wire.beginTransmission(Adr_DS3231); // �������� ����� � DS3231
	Wire.write(byte(0x00)); // ���������� ����� ��������, � �������� ���������� ������ ���� � �������
	Wire.endTransmission(); // ��������� ��������

	int i = 0; // ������ �������� �������� �������

	//Wire.beginTransmission(Adr_DS3231); // �������� ����� � DS3231
	Wire.requestFrom(Adr_DS3231, (int16_t)7); // ����������� 7 ������ � DS3231
	while (Wire.available()) // ���� ���� ������ �� Adr_DS3231
	{
		dateTime[i] = Wire.read(); // ������ 1 ���� � ��������� � ������ dateTime
		i += 1; // �������������� ������ �������� �������
	}
	Wire.endTransmission(); // ��������� ��������
	//Serial.println("dT");
	//printDateTime(dateTime); // ������� ���� � �����
}





//void Set_Adr_DS3231() 
//{
//	//�������� ������� ���� ����� � ���������� ����������.
//	time_t rawtime;
//	struct tm * timeinfo;
//
//	time(&rawtime);                               // �������� ������� ����, ���������� � ��������
//	timeinfo = localtime(&rawtime);
//	int a = timeinfo->tm_hour;
//	Serial.print(" aaaa = ");	  	Serial.println(a);
//	
//	Wire.beginTransmission(0x68); // �������� ����� � Adr_DS3231 � i2c ������� 0x68
//	byte arr[] = { 0x00, 0x02, 0x30, 0x17, 0x03, 0x02, 0x01, 0x19 };
//	Wire.write(arr, 8); // ���������� 8 ������ ������� arr
//	Wire.endTransmission(); // ���������� ��������
//}

void Loop_DataTime()
{
	//==========================================================
	if (flag_datatime == true)
	{
		//digitalWrite(49, 1);

		flag_datatime = false;
		Read_DS3231();				  // ����� ���������� 0.5 ����������

		//Write32F_I2C(0x09, 0xF1, -2519.9333);
		//printDateTime(dateTime); // ������� ���� � �����
		//Serial.print(" MotorL.motor_speed: "); Serial.print(MotorL.motor_speed, 4);
	//	Serial.print(" MotorR.motor_pwm: "); Serial.print(MotorR.motor_pwm);
	//	Serial.print(" MotorR.motor_speed: "); Serial.print(MotorR.motor_speed, 4);
		//Serial.print(" MotorR.motor_way: "); Serial.print(MotorR.motor_way, 4);
	//	Serial.println("");
		//printDateTime(dateTime); // ������� ���� � �����
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
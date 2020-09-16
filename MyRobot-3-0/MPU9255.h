
uint8_t buffer_acc_gyro[14];			 // ����� ��� ���������� ������ �������������

//#define MPU9255_CS   19  //cs		  ��� ������ ��� SPI ��� 9255
#define MPU9255_Address   0x69  //cs		 ����� ��� I2C



int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t temp9255;
uint16_t acc_delitel, gyro_delitel;

float accX, accY, accZ;
float accXoffset, accYoffset, accZoffset;
float angleAccX, angleAccY;
float angleX, angleY, angleZ;
float Temperatura9255;
bool  StatusColibrAccGyro = false;
float gyroX, gyroY, gyroZ;
float gyroXoffset, gyroYoffset, gyroZoffset;
float angleGyroX, angleGyroY, angleGyroZ, angleGyro_Delta_Z;
float intervalAccGyro, pre_intervalAccGyro;	 // ��� ������� ��������� ������� ��������� ����� ������������

float angleCompX;		// ������ ����� ���������������� �������
float angleCompY;
float angleCompZ;
float X_angle_comp; // ���������� � ������� ������ ���������� �������� ���������������� �������
float Y_angle_comp; // ���������� � ������� ������ ���������� �������� ���������������� �������
float Z_angle_comp; // ���������� � ������� ������ ���������� �������� ���������������� �������



void Init_9255()
{
	Serial.println(" ----------------------------------------------------------");
	Serial.println(" Start MPU9250...");
	Serial2.println(" Start Init_9255 ");

	//pinMode(MPU9255_CS, OUTPUT);    // MPU9255 ChipSelect SPI
	//digitalWrite(MPU9255_CS, HIGH);

	uint8_t WIA_MPU;
	//WIA_MPU = ReadByte_SPI(MPU9255_CS, 0x75);     // ��������� �������� �������� "��� �"
	WIA_MPU = ReadByte_I2C(MPU9255_Address, 0x75);     // ��������� �������� �������� "��� �"
	Serial.print("WIA_MPU: "); Serial.print(WIA_MPU, HEX);

	angleGyroX = 0;           // �������� ��������� �������� ����
	angleGyroY = 0;
	angleGyroZ = 0;

	//digitalWrite(MPU9255_CS,LOW);
	//uint8_t dump;
	//dump = SPI.transfer(0xF5); // READ(MSB=1) 0x80 or 0x75 -> 0xF5
	//Serial.print("dump: "); Serial.println(dump);
	//who_am_i = SPI.transfer(0);
	//digitalWrite(MPU9255_CS, HIGH);



	if (WIA_MPU == 0x71 | WIA_MPU == 0x73)          //71- 9050       73-9255
	{
		if (WIA_MPU == 0x71)  Serial.println(" Successfully connected to MPU9250");
		if (WIA_MPU == 0x73)  Serial.println(" Successfully connected to MPU9255");

	//	SPI.setClockDivider(SPI_CLOCK_DIV16);  // ������ SPI_CLOCK_DIV2  ������������� �������� ������ ��������� SPI ������� �� ������������ 9255

		WriteByte_I2C(MPU9255_Address, 0x6B, 0b10000000);                         // 107 ������� Reset MPU9255 
		delay(10);  // �����  ���� 1 ������������

		//WriteByte_SPI(MPU9255_CS, 0x6A, 0b00010000);                         // 106 ������� ���������� ������ �� I2C � ����������� ������ SPI
		//delay(1);  // �����  ���� 1 ������������

		WriteByte_I2C(MPU9255_Address, 0x6B, 0b00000001);                         // 107 ������� Clock Source  Auto selects the best available clock source � PLL if ready, else use the Internal oscillator
		delay(1);  // �����  ���� 1 ������������

		WriteByte_I2C(MPU9255_Address, 0x6C, 0b00000000);                         // 108 ������� �������� �������� � ������������
		delay(1);  // �����  ���� 1 ������������
																		// 25 ������� SMPLRT_DIV ������������� �������� ���������� ������� 
//		WriteByte_SPI(MPU9255_CS, 0x19, 0b00000011);                         // ������������� HEX ������� 3 � BIN (11) � �������� ������� 1 ��� � 4 ����������� ��� 250 ���� 
		WriteByte_I2C(MPU9255_Address, 0x19, 0b00000111);                         // ������������� HEX ������� 7 � BIN (111) � �������� ������� 1 ��� � 8 ����������� ��� 125 ���� 

		delay(1);  // �����  ���� 1 ������������ 

		//WriteByte_SPI(MPU9255_CS, 0x1C, 0b00000000);   acc_delitel = 16384;   //  +-2G      // 28 ������� ACCEL_CONFIG ������������� ���������������� ������������
		WriteByte_I2C(MPU9255_Address,0x1C,0b00001000);   acc_delitel = 8192;    //  +-4G      // 28 ������� ACCEL_CONFIG ������������� ���������������� ������������    
	    //WriteByte_SPI(MPU9255_CS,0x1C,0b00010000);   acc_delitel = 4096;    //  +-8G      // 28 ������� ACCEL_CONFIG ������������� ���������������� ������������   
	    //WriteByte_SPI(MPU9255_CS,0x1C,0b00011000);     acc_delitel = 2048;  //  +-16G     // 28 ������� ACCEL_CONFIG ������������� ���������������� ������������

		delay(1);  // �����  ���� 1 ������������

	//  WriteByte_SPI(MPU9255_CS, 0x1D, 0b00001000);    //   460  Bandwidth(Hz);     // 29 ������� ACCEL_CONFIG2 ������������� ������ ����������� ������������
	//	WriteByte_SPI(MPU9255_CS, 0x1D, 0b00001001);    //   184  Bandwidth(Hz);     // 29 ������� ACCEL_CONFIG2 ������������� ������ ����������� ������������      
	//	WriteByte_SPI(MPU9255_CS, 0x1D, 0b00001010);    //   92  Bandwidth(Hz);     // 29 ������� ACCEL_CONFIG2 ������������� ������ ����������� ������������      
		WriteByte_I2C(MPU9255_Address, 0x1D, 0b00001011);    //   41  Bandwidth(Hz);     // 29 ������� ACCEL_CONFIG2 ������������� ������ ����������� ������������      

		delay(1);  // �����  ���� 1 ������������

	//	WriteByte_SPI(MPU9255_CS, 0x1B, 0b00000000);   gyro_delitel = 131;    //  250dps     // 27 ������� GYRO_CONFIG ������������� ���������������� ���������
	  //WriteByte_SPI(MPU9255_CS, 0x1B, 0b00001000);   gyro_delitel = 65.5;   //  500dps     // 27 ������� GYRO_CONFIG ������������� ���������������� ���������
		WriteByte_I2C(MPU9255_Address, 0x1B, 0b00010000);   gyro_delitel = 32.8;   //  1000dps    // 27 ������� GYRO_CONFIG ������������� ���������������� ���������
	 // WriteByte_SPI(MPU9255_CS, 0x1B, 0b00011000);   gyro_delitel = 16.4;     //  2000dps      // 27 ������� GYRO_CONFIG ������������� ���������������� ���������      
		delay(1);

	  //WriteByte_SPI(MPU9255_CS, 0x1A, 0b00000000);    //   250  Bandwidth(Hz);     // 26 ������� CONFIG ������������� ������ ����������� ���������      
	//	WriteByte_SPI(MPU9255_CS, 0x1A, 0b00000001);    //   184  Bandwidth(Hz);     // 26 ������� CONFIG ������������� ������ ����������� ���������
	//	WriteByte_SPI(MPU9255_CS, 0x1A, 0b00000010);    //   92  Bandwidth(Hz);     // 26 ������� CONFIG ������������� ������ ����������� ���������
		WriteByte_I2C(MPU9255_Address, 0x1A, 0b00000011);    //   41  Bandwidth(Hz);     // 26 ������� CONFIG ������������� ������ ����������� ���������
		delay(1);

		WriteByte_I2C(MPU9255_Address, 0x37, 0b00010000);         // 55 ������� INT_PIN_CFG ������������� �������� ������� ���������� ��� ����� ������ �� �������� ������
//		WriteByte_SPI(MPU9255_CS, 0x37, 0b00000000);         // 55 ������� INT_PIN_CFG ������������� �������� ������� ���������� ��� ����� ������ �� �������� ������

		delay(1);

		WriteByte_I2C(MPU9255_Address, 0x38, 0b00000001);         // 56 ������� INT_PIN_CFG �������� ���������� �� ����
		delay(1);

		// Set by pass mode for the magnetometers
		WriteByte_I2C(MPU9255_Address, 0x37, 0x02);		 //�������� ������ � ������������� �� I2C �� ���� �� ���������. ���� ��������� ���������� ���� � ������� .


		//SPI.setClockDivider(SPI_CLOCK_DIV2);  // SPI_CLOCK_DIV2       ������������� �������� SPI	������ SPI_CLOCK_DIV2 �������� 2 � ������� ������� 16 = 8 �����

	}
	else
	{
		Serial.println("Failed to Connect to MPU9250 !!!!!!!!!!!!!!");
		delay(100000);
	}


}

double getCompAngleX(float comp_AngleX, float comp_GyroY, float dt)       //���������� ��������������� ������
{
	//a(t) = (1-K) * (a(t-1) + gx*dt) + K * acc 
	float coefficient = 0.03;
	X_angle_comp = ((1 - coefficient) * (X_angle_comp + comp_GyroY * dt)) + (coefficient * comp_AngleX);
	return X_angle_comp;
}
double getCompAngleY(float comp_AngleY, float comp_GyroX, float dt)       //���������� ��������������� ������
{
	//a(t) = (1-K) * (a(t-1) + gx*dt) + K * acc 
	float coefficient = 0.05;
	Y_angle_comp = ((1 - coefficient) * (Y_angle_comp + comp_GyroX * dt)) + (coefficient * comp_AngleY);
	return Y_angle_comp;
}

double getCompAngleZ(float magD, float comp_GyroZ, float dt)       //���������� ��������������� ������
{
	//a(t) = (1-K) * (a(t-1) + gx*dt) + K * acc 
	float coefficient = 0.001;

	//Serial.print(" magD= ");	Serial.print(magD);
	float gyroD = Z_angle_comp + (comp_GyroZ * dt);

	if (gyroD < 0) { gyroD = gyroD + 360; }
	if (gyroD > 360) { gyroD = gyroD - 360; }

	//Serial.print(" g1= ");	Serial.print(gyroD);

	//������� ����� 360 � ����������� � ����� ����� magD

	if (gyroD < 10 && magD > 350)      // ���� � ������ �������� �� 360 ��������
	{
		gyroD = gyroD + 360;       // ����� ��������� 360 � ��� ���������� � ����� �������
	}
	if (gyroD > 350 && magD < 10)      // ���� � ������ �������� �� 360 ��������
	{
		gyroD = gyroD - 360;       // ����� ��������� 360 � ��� ���������� � ����� �������
	}

	//Serial.print(" g2= ");	Serial.print(gyroD);


	Z_angle_comp = ((1 - coefficient) * gyroD) + (coefficient * magD);

	//Serial.print(" Z1= ");	Serial.print(Z_angle_comp);

	if (Z_angle_comp < 0) { Z_angle_comp = Z_angle_comp + 360; }
	if (Z_angle_comp > 360) { Z_angle_comp = Z_angle_comp - 360; }

	//Serial.print(" Z_angle_comp= ");	Serial.println(Z_angle_comp);

	return Z_angle_comp;
}


//void Read_9255(uint8_t CS_PIN)  // ��������� �� ����� ���� ���������� � � ������ �������� ����� ��������� ������
//{
//	//Serial.print("+");
//
////	SPI.setClockDivider(SPI_CLOCK_DIV2);  // ������ SPI_CLOCK_DIV2 �������� 2 � ������� ������� 16 = 8 �����
//
//	//SPI.beginTransaction(settings2);
//	digitalWrite(CS_PIN, LOW);
//	int8_t registr = 0x3B;			  // ������ ��������� ��������������� 14 ����	������� � �����
//	registr = registr | (1 << 7);   //   READ(MSB=1) 0x80 or 0x75 -> 0xF5   // ��������� ������� ��� ��� ��� ��� ������
//	SPI.transfer(registr);
//	//Serial.print("= ");Serial.println(micros());
//	for (uint8_t i = 0; i < 14; i++)   // ��������� 14 ���� uint16_t - ��� 7 ����������� �������� uint16_t
//	{
//		buffer_acc_gyro[i] = SPI.transfer(0x00);
//	}
//	digitalWrite(CS_PIN, HIGH);
//	//SPI.endTransaction();
//
//
//	//Serial.print("T");Serial.println(micros());
//	//Serial.print(".");
//}

void Read_9255_I2C()  // ��������� �� ����� ���� ���������� � � ������ �������� ����� ��������� ������
{
	//Serial2.print("/");

	//Serial2.print("WIA_MPU: "); Serial2.print(ReadByte_I2C(MPU9255_Address, 0x75), HEX);
	Wire.beginTransmission(MPU9255_Address);
	int8_t registr = 0x3B;			  // ������ ��������� ��������������� 14 ����	������� � �����
	Wire.write(registr);
	byte reza = Wire.endTransmission();
	if (reza != 0)
	{
		Serial.print("!!! 9255 Write Mistake reza = ");Serial.println(reza);
		Serial2.print("!!! 9255 Write Mistake reza = ");Serial2.println(reza);

	};

	uint8_t buffer_acc_gyroTemp[14];			 // ����� ��� ���������� ������ �������������  ���������
	byte count_byte = 14;			  // ����� ������������� ����
	
	int rezb = Wire.requestFrom(MPU9255_Address, count_byte);
	//Serial2.print(rezb);
	if (rezb == count_byte)         // ���� ���������  ������� ������� �������
	{
		for (byte i = 0 ;i < count_byte; i++)   // ��������� 14 ���� uint16_t - ��� 7 ����������� �������� uint16_t ���� ����� ������ ����
		{
			buffer_acc_gyro[i] = Wire.read();
		}
	}
	else
	{
		Serial.print("!!! 9255 Write Mistake rezb = ");Serial.println(rezb);
		Serial2.print("!!! 9255 Write Mistake rezb = ");Serial2.println(rezb);

	}
	//uint8_t i = 0;

	//while (Wire.available())    // ������� ���������� ����� ������� ������, ��� ���������
	//{
	//	buffer_acc_gyroTemp[i] = Wire.read();
	//	i++;
	//}

	//for (;i < 14 && (Wire.available() > 0); i++)   // ��������� 14 ���� uint16_t - ��� 7 ����������� �������� uint16_t ���� ����� ������ ����
	//{
	//	buffer_acc_gyroTemp[i] = Wire.read();
	//}
	//Serial2.println("\ "); 
//	Serial2.print(i);

	//if (i == 14)       // ���� ��������� ��� 14 ���� ��������� � ������ �� ������� ����� �� ��������� ������� ������������ � ����������
	//{
	//	for (int y = 0; y < 14; y++) {	buffer_acc_gyro[y] = buffer_acc_gyroTemp[y]; }
	//}
	//else
	//{
	//	Serial2.println(" BLAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
	//	for (int y = 0; y < 14; y++) 
	//	{ 
	//		Serial2.print(buffer_acc_gyro[y]);
	//		Serial2.print("new->");
	//		Serial2.println(buffer_acc_gyroTemp[y]);
	//		//buffer_acc_gyro[y] = 0;
	//	}

	//}

	//Serial.print("T");Serial.println(micros());
	//Serial.print(".");
}


void Update_9255()
{

	//Serial.print("F");Serial.println(micros()); Serial.println("");

	ax = (((int16_t)buffer_acc_gyro[0]) << 8) | buffer_acc_gyro[1];
	ay = (((int16_t)buffer_acc_gyro[2]) << 8) | buffer_acc_gyro[3];
	az = (((int16_t)buffer_acc_gyro[4]) << 8) | buffer_acc_gyro[5];
	temp9255 = (((int16_t)buffer_acc_gyro[6]) << 8) | buffer_acc_gyro[7];
	gx = (((int16_t)buffer_acc_gyro[8]) << 8) | buffer_acc_gyro[9];
	gy = (((int16_t)buffer_acc_gyro[10]) << 8) | buffer_acc_gyro[11];
	gz = (((int16_t)buffer_acc_gyro[12]) << 8) | buffer_acc_gyro[13];



	Temperatura9255 = (temp9255 - 0) / 333.87 + 21;     //������ ����������� ������ ���� �� �������, � �� ����������� �������

	accX = ((float)ax) / acc_delitel;
	accY = ((float)ay) / acc_delitel;
	accZ = ((float)az) / acc_delitel;
	//��������� ���������� 

	accX -= accXoffset;
	accY -= accYoffset;
	accZ -= accZoffset;

	if ( accZoffset != 0 )		// ���� ���� ����������
	{
//	   if (accZ < -1) accZ = -1;		if (accZ > 1) accZ = 1;    //���-�� �� ������� �� �������  
	}

	gyroX = ((float)gx) / gyro_delitel;
	gyroY = ((float)gy) / gyro_delitel;
	gyroZ = ((float)gz) / gyro_delitel;									   	//��������� ���������� 

	gyroX -= gyroXoffset;
	gyroY -= gyroYoffset;
	gyroZ -= gyroZoffset;

	//��������� ����

	angleAccX = atan2(accX, accZ) * RAD_TO_DEG;
	angleAccY = atan2(accY, accZ) * RAD_TO_DEG;

	//angleAccX = atan2(accX, accZ + abs(accY)) * RAD_TO_DEG;		   //������ ������� ������� ��������� �������������� ����������
	//angleAccY = atan2(accY, accZ + abs(accX)) * RAD_TO_DEG;		   // ����� ������� ��� �� �� ����������


	//if (angleAccX < 0) { angleAccX = angleAccX + 180; }
	//else { angleAccX = -180 + angleAccX; }  // ������� ���� ������� � ������ ���� ��� ����������� ���������� �� �������
	//if (angleAccY < 0) { angleAccY = -angleAccY - 180; }
	//else { angleAccY = 180 - angleAccY; }  // ������� ���� ������� � ������ ���� ��� ����������� ���������� �� �������

	//�������� �� ��������� �������� �� ��������� ����� � �������� ���� �� ������� �� ��� ����� ����������
	angleGyroX += gyroX * intervalAccGyro;
	angleGyroY += -gyroY * intervalAccGyro;
	angleGyroZ +=  gyroZ * intervalAccGyro;
	angleGyro_Delta_Z = gyroZ * intervalAccGyro;

	if (angleGyroZ > 360) { angleGyroZ = angleGyroZ - 360; }	  // �������������� � 360 ��������
	if (angleGyroZ < 0) { angleGyroZ = angleGyroZ + 360; }


	// ��������� ���� �� ������������� � ���������
	//angleCompX = getCompAngleX(angleAccX, -gyroY, intervalAccGyro);		  //����� ������� ��� ��� ��� �� ������������� � ����� �� ���������
	//angleCompY = getCompAngleY(angleAccY, gyroX, intervalAccGyro);

	// ��������� ���� �� ������������� � ���������
	angleCompX = getCompAngleX(angleAccX, -gyroY, intervalAccGyro);
	angleCompY = getCompAngleY(angleAccY, -gyroX, intervalAccGyro);
	angleCompZ = getCompAngleZ(magD, gyroZ, intervalAccGyro);

	//��������� ���� �������� ��������� ������������ � ��������
	//myAHRS.compAngleZ = myAHRS.getCompAngleZ(magD, -gyroZ, intervalAccGyro);
	//myAHRS.compAngleZ = myAHRS.getCompAngleZ(magD, angleGyroZ, intervalAccGyro);


		//myAHRS.kalmAngleX = myAHRS.getkalmAngleX(angleAccX, gyroY, intervalAccGyro); // Calculate the angle using a Kalman filter X
		//myAHRS.kalmAngleY = myAHRS.getkalmAngleY(angleAccY, gyroX, intervalAccGyro); // Calculate the angle using a Kalman filter Y
	//	Filter_Complementary();            // ��������� ��������

	//Serial2.println("UpGood");
}

void ReadCalibrovka_AccGyro()
{
	if (eeprom_read_byte(32) == 1)   // ���� ���� ����������� �������� �� ��������� ��
	{
		Serial.println(" Read Colibrovka from EEPROM ");
		accXoffset = eeprom_read_float(34);
		accYoffset = eeprom_read_float(38);
		accZoffset = eeprom_read_float(42);
		gyroXoffset = eeprom_read_float(46);
		gyroYoffset = eeprom_read_float(50);
		gyroZoffset = eeprom_read_float(54);
		Serial.print("AX : ");Serial.println(accXoffset, 6);
		Serial.print("AY : ");Serial.println(accYoffset, 6);
		Serial.print("AZ : ");Serial.println(accZoffset, 6);
		Serial.print("GX : ");Serial.println(gyroXoffset, 6);
		Serial.print("GY : ");Serial.println(gyroYoffset, 6);
		Serial.print("GZ : ");Serial.println(gyroZoffset, 6);
		Serial.println(" ----------------------------------------------");
		StatusColibrAccGyro = true;

	}
}

void Calibrovka_AccGyro()
{

	float ax = 0, ay = 0, az = 0;
	float gx = 0, gy = 0, gz = 0;
	int ii = 300;       // ������������ ����������
	accXoffset = 0;
	accYoffset = 0;
	accZoffset = 0;

	gyroXoffset = 0;
	gyroYoffset = 0;
	gyroZoffset = 0;

	Serial.println("======================= calculate  offsets =================");

	for (int i = 0; i < ii; i++)
	{
		// ���������� ��������, ������������ � ������������
		Read_9255_I2C();            //��������� ��� 7 �������� �� ������-���������
		Update_9255();
		//Serial.print(".");
		//Serial.print(" i = ");    Serial.print(i);
		ax += accX; //Serial.print(" AX+ = ");    Serial.print(ax);
		ay += accY;// Serial.print(" AY+ = ");    Serial.print(ay);
		az += accZ; //Serial.print(" AZ+ = ");    Serial.print(az);

		gx += gyroX;// Serial.print(" GX+ = ");    Serial.print(gx);
		gy += gyroY; //Serial.print(" GY+ = ");    Serial.print(gy);
		gz += gyroZ;// Serial.print(" GZ+ = ");    Serial.println(gz);
		delay(10);
	}
	Serial.println("End calculate offsets. ");

	accXoffset = ax / ii;
	accYoffset = ay / ii;
	//accZoffset = (az / ii ) + 1;  // �������� �� � ����, � � ����� �������, 1g ������ ���������� ���������� ������� �� ���� ��� ����������� ���������� 
	accZoffset = (az / ii) - 1;  // �������� �� � ����, � � �������, 1g ������ ���������� ���������� ������� �� ���� ��� ����������� ���������� 

	gyroXoffset = gx / ii + 0.00;
	gyroYoffset = gy / ii +-0.00; // ��� ����� ������� �������� �������� � ��������������� �����������
	gyroZoffset = gz / ii + 0.00;

	angleGyroX = 0;           // �������� ��������� �������� ����
	angleGyroY = 0;
	angleGyroZ = 0;

	//eeprom_write_byte(32, 1);     // ���������� ������� ��� ���� ������ ���������� � 10 �����
	//Serial.print("Flag ACC_GYRO: "); Serial.println(Eeprom_ReadByte(32));

	//Serial.println("==================ACC======================");

	//Serial.print("X : ");Serial.println(accXoffset, 6);
	//Eeprom_WriteFloat(34, accXoffset);
	//Serial.print("accX offset : "); Serial.println(Eeprom_ReadFloat(34), 6);

	//Serial.print("Y : ");Serial.println(accYoffset, 6);
	//Eeprom_WriteFloat(38, accYoffset);
	//Serial.print("accY offset : "); Serial.println(Eeprom_ReadFloat(38), 6);

	//Serial.print("Z : ");Serial.println(accZoffset, 6);
	//Eeprom_WriteFloat(42, accZoffset);
	//Serial.print("accZ offset : "); Serial.println(Eeprom_ReadFloat(42), 6);

	//Serial.println("==================GYRO=====================");
	//Serial.print("X : ");Serial.println(gyroXoffset, 6);
	//Eeprom_WriteFloat(46, gyroXoffset);
	//Serial.print("gyroX offset : "); Serial.println(Eeprom_ReadFloat(46), 6);

	//Serial.print("Y : ");Serial.println(gyroYoffset, 6);
	//Eeprom_WriteFloat(50, gyroYoffset);
	//Serial.print("gyroY offset : "); Serial.println(Eeprom_ReadFloat(50), 6);

	//Serial.print("Z : ");Serial.println(gyroZoffset, 6);
	//Eeprom_WriteFloat(54, gyroZoffset);
	//Serial.print("gyroZ offset : "); Serial.println(Eeprom_ReadFloat(54), 6);

	//Serial.println("========================================");

	StatusColibrAccGyro = true;
}


void  int5_func()         // MPU9255 ������� ���������� 9255  5 ���������� 18 ���  
{
	if (StatusColibrAccGyro == true)    // ���� ��� ��������� ����������
	{
		//digitalWrite(43, 1);
		now_time = micros();
		intervalAccGyro = (now_time - pre_intervalAccGyro)*0.000001;			// �������� ����� ��������� � ����������� ���������� ������ ��������� � ��������
		pre_intervalAccGyro = now_time;
		flag_DataReady_AccGyro = true;     // ������� ���� ���������� ������ ��� ��������� ������

	//	Serial.print("T");Serial.println(micros());
		//Serial.print(".");
		//digitalWrite(43, 0);
	}

}
void PrintGyroTest()
{
	Serial.print(angleGyroX);   Serial.print("  ");
	Serial.print(angleGyroY);   Serial.print("  ");
	Serial.print(angleGyroZ);   Serial.print("  ");
	Serial.println(" ");
}


void Loop_9255()
{

	//==========================================================
	if (flag_DataReady_AccGyro == true)	// ������ 8 ����������� ���� ����� ������ ������ ������	   	  // ����� ����������  0.3 �����������  ��� ��������
	{
		flag_DataReady_AccGyro = false;

		// ��������� ������ ��������� �� ��������
		Read_9255_I2C();            //��������� ��� 7 �������� �� ������-���������
		Update_9255();     // ������ ��������� ��� ������ ������� �� ����� ���������� // ����������� Raw ������  ��������� ������������� 
		//Serial.print(" Update_9255 --- : ");  	Serial.println(micros());
		//PrintRawData();	  
		//PrintData();
		//PrintAngle();
		//PrintAngleTest();
		//PrintGyroTest();


		//PrintAngleCompFiltr();	

	}

}




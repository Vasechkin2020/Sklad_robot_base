

// Датчики 4 штуки все с одним адресом 0х29
#include <VL53L0X.h>

VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;
VL53L0X sensor4;

void i2c_scan()
{

	Serial.println("-------------------------------------------------------------");

	byte error, address;
	int nDevices;

	nDevices = 0;
	for (address = 8; address < 127; address++) {
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0) {
			Serial.print("Address 0x");
			if (address < 16)
				Serial.print("0");
			Serial.print(address, HEX);
			Serial.println(" !");

			nDevices++;
		}
		else if (error == 4) {
			Serial.print("Unknow error at address 0x");
			if (address < 16)
				Serial.print("0");
			Serial.println(address, HEX);
		}
	}
		Serial.print("done = ");
		Serial.println(nDevices);
	Serial.println("-------------------------------------------------------------");


}

int Read_VL53L0X(VL53L0X sensor)
{
	int rez = 0;
		//float rez = sensor.readRangeContinuousMillimeters();
		if ((sensor.readReg(0x13) & 0x07) != 0)
		{
			//long a = micros();
			rez = sensor.readReg16Bit(0x1E);
			sensor.writeReg(0x0B, 0x01);
			//long b = micros();
			//Serial.print(rez);     Serial.print("   ");     Serial.print(b - a);
		}
		else
		{
			Serial.print(" TIMEOUT");
		}

		//Serial.println();
	return 	rez;
}

void Start_VL53L0X(VL53L0X &sensor, byte new_address, byte pin)		 //  С помощью тильды не пропадают значения переменной 
                                                                     //класса что меняем внутри функции для этого берем не копию переменной а саму переменную с помощью тильды.
{
	delay(10);
	digitalWrite(pin, HIGH);					 //Включаем датчик
	Serial.print("PIN HIGH ... "); Serial.println(pin);
	Serial2.print("PIN HIGH ... "); Serial2.println(pin);

	delay(100);
	i2c_scan();
	sensor.setTimeout(0);
	sensor.setAddress(new_address);
	delay(25);
	i2c_scan();	
	
	byte address = sensor.readReg(0x8A);
	Serial.print(" address = "); Serial.print(address,HEX);
	byte WiA = sensor.readReg(0xC0);
	Serial.print(" WiA = "); Serial.println(WiA);



	if (sensor.init())
	{
		Serial.println("Init_VL53L0X OK !!!");
		Serial2.println("Init_VL53L0X OK !!!");

		delay(500);
	}
	else
	{
		Serial.println(" !!!!!!!!!!!!!!!!!!Failed to detect and initialize sensor!");
		Serial2.println(" !!!!!!!!!!!!!!!!!!Failed to detect and initialize sensor!");

		delay(2000);
	}

	// Start continuous back-to-back mode (take readings as	 fast as possible).  To use continuous timed mode  instead, provide a desired inter-measurement period in  ms (e.g. sensor.startContinuous(100)).
	sensor.startContinuous();
	sensor.setMeasurementTimingBudget(25000);
	delay(25);
	Serial.println("End Init_VL53L0X ===");
}

#define VL53_pin1  51
#define VL53_pin2  53
#define VL53_pin3  49
#define VL53_pin4  47

void Init_VL53L0X()
{
	Serial.println("-------------------------------------------------------------");
	Serial.println("Start Init_VL53L0X... ");
	Serial2.println("Start Init_VL53L0X...");

	Serial.println("PIN LOW ... ");

	pinMode(VL53_pin1, OUTPUT); 	//Обьявляем пины и они переходят в состояние LOW  
	pinMode(VL53_pin2, OUTPUT);
	pinMode(VL53_pin3, OUTPUT); 		
	pinMode(VL53_pin4, OUTPUT); 

	//digitalWrite(VL53_pin1, LOW);  delay(10);	   // Пины все опускаем и тем самым выключаем датчики и включаем потом по одному меняя адрес I2C
	//digitalWrite(VL53_pin2, LOW);  delay(10);
	//digitalWrite(VL53_pin3, LOW);  delay(10);
	//digitalWrite(VL53_pin4, LOW);  delay(10);

	digitalWrite(VL53_pin1, HIGH); delay(10);		  // Включаем и выключаем датчики что-бы они перезагрузились
	digitalWrite(VL53_pin1, LOW); delay(10);

	digitalWrite(VL53_pin2, HIGH); delay(10);
	digitalWrite(VL53_pin2, LOW); delay(10);

	digitalWrite(VL53_pin3, HIGH); delay(10);
	digitalWrite(VL53_pin3, LOW); delay(10);	
	
	digitalWrite(VL53_pin4, HIGH); delay(10);
	digitalWrite(VL53_pin4, LOW); delay(10);

	
	i2c_scan();

	Start_VL53L0X(sensor1, 0x30, VL53_pin1);
	Start_VL53L0X(sensor2, 0x31, VL53_pin2);
	Start_VL53L0X(sensor3, 0x32, VL53_pin3);
	Start_VL53L0X(sensor4, 0x33, VL53_pin4);

	//delay(99999);

}



// Gy_56.h

				   /*
GY56-----------MINI
VCC ---------- 5V
GND  --------- GND
Pin 5 -------- RC
Pin 4 -------- TD
Pin PS-------GND
*/
//The Arduino Wire library uses the 7-bit version of the address, so the code example uses 0x70 instead of the 8-bit 0xE0
#define SensorAddress1 byte(0x71)     //Передний левый
#define SensorAddress2 byte(0x73)	  //Передний Правый
#define SensorAddress3 byte(0x70)	  //Задний Левый
#define SensorAddress4 byte(0x72)	  //Задний Правый

//The sensors ranging command has a value of 0x51
#define RangeCommand byte(0x51)
//These are the two commands that need to be sent in sequence to change the sensor address
#define ChangeAddressCommand1 byte(0xAA)
#define ChangeAddressCommand2 byte(0xA5)
#define ChangModeCommand1  byte(0x5A)
#define ChangModeCommand2  byte(0x55)
#define measure_time_55ms 0
#define measure_time_110ms 1
#define measure_time_200ms 2
#define measure_time_300ms 3
#define DISTANCEMODE_SHORT  1
#define DISTANCEMODE_MEDIUM 2
#define DISTANCEMODE_LONG   3
uint16_t delay_time = 0;

void change_mode(byte Address, byte mode, byte  time)
{
	Wire.beginTransmission(Address); //Begin addressing
	Wire.write(ChangModeCommand1); //Send first  command
	Wire.write(ChangModeCommand2); //Send second  command
	byte temp;
	temp = (mode << 4) | time;
	Wire.write(temp); //Send the new address to change to
	Wire.endTransmission();
}

/* Commands a sensor at oldAddress to change its address to newAddress
oldAddress must be the 7-bit form of the address that is used by Wire
7BitHuh determines whether newAddress is given as the new 7 bit version or the 8 bit version of the address
If true, if is the 7 bit version, if false, it is the 8 bit version
*/
//changeAddress(0x70,0x71,true);  Serial.print("New address:0x71");
//delay(5000);

void changeAddress(byte oldAddress, byte newAddress, boolean SevenBitHuh)
{

	Wire.beginTransmission(oldAddress); //Begin addressing
	Wire.write(ChangeAddressCommand1); //Send first change address command
	Wire.write(ChangeAddressCommand2); //Send second change address command
	byte temp;
	if (SevenBitHuh) { temp = newAddress << 1; } //The new address must be written to the sensor
	else { temp = newAddress; } //in the 8bit form, so this handles automatic shifting
	Wire.write(temp); //Send the new address to change to
	Wire.endTransmission();
}
//Returns the last range that the sensor determined in its last ranging cycle in centimeters. Returns 0 if there is no communication.
word requestRange(byte SensorAddress)
{
	//Serial.print("rR");

	Wire.requestFrom(SensorAddress, byte(3));
	if (Wire.available() >= 2)
	{ //Sensor responded with the two bytes
		//Serial.print("/");

		byte HighByte = Wire.read(); //Read the high byte back
		byte LowByte = Wire.read(); //Read the low byte back
		byte State = Wire.read(); //Read the low byte back
		//Serial.print("/");

		word range = word(HighByte, LowByte); //Make a 16-bit word out of the two bytes for the range
	//	Serial.println("rR+");
		byte RangeStatus = (State >> 4) & 0x0f;
		byte Time = (State >> 2) & 0x03;
		byte Mode = State & 0x03;
									/*
									Текущее состояние, возвращаемое модулем.Значение: 0 ~ 14.
									0: означает надежное значение расстояния?
									1: указывает на влияние окружающего света?
									2: указывает, что возвращенный сигнал слабый?
									4: означает вне диапазона измерения?
									5: указывает на аппаратный сбой?
									7: указывает, что окружающая среда имеет помехи и шум?
									8: переполнение или переполнение внутреннего алгоритма?
									14: недопустимое измерение.
									*/
		if (RangeStatus != 0)		 // Если статус любой кроме хорошего то считаем что 160 см до преграды
		{
			range = 160;
		}

		//Serial.print(" State: ");Serial.print(State,BIN);
		//Serial.print(" RangeStatus: ");Serial.print(RangeStatus);
		//Serial.print(" Time: ");Serial.print(Time);
		//Serial.print(" Mode: ");Serial.println(Mode);

		return range;
	}
	else
	{
		Serial.print("requestRange Mistake = ");Serial.print(Wire.available());
		Serial.print(" SensorAddress = ");Serial.println(SensorAddress,HEX);

		return word(0); //Else nothing was received, return 0
	}
}

//Commands the sensor to take a range reading
void takeRangeReading(byte SensorAddress)
{
	//Serial.print("rS");

	Wire.beginTransmission(SensorAddress); //Start addressing
	Wire.write(RangeCommand); //send range command
	int reza = Wire.endTransmission(); //Stop and do something else now
	if (reza != 0)
	{
		Serial.print(" takeRangeReading reza= ");	Serial.print(reza);  
		Serial2.print(" takeRangeReading reza= ");	Serial2.println(reza);
		Serial.print(" SensorAddress = ");Serial.println(SensorAddress,HEX);

	}
	//Serial.println("rS+");

}

void Init_Gy56()
{
	uint16_t measure_time = measure_time_55ms;
	change_mode(SensorAddress1, DISTANCEMODE_MEDIUM, measure_time);
	change_mode(SensorAddress2, DISTANCEMODE_MEDIUM, measure_time);
	change_mode(SensorAddress3, DISTANCEMODE_MEDIUM, measure_time);
	change_mode(SensorAddress4, DISTANCEMODE_MEDIUM, measure_time);

	switch (measure_time)
	{
	case 0:delay_time = 55;break;
	case 1:delay_time = 110;break;
	case 2:delay_time = 200;break;
	case 3:delay_time = 300;break;
	}
	//changeAddress(0x70,0x71,true);  Serial.print("New address:0x71");
	//delay(5000);

	Datchik_L_Pered.Angle_Ustanovki_in_0 = +45;                //Установлен с наклоном влево
	Datchik_L_Pered.X_koordinata = 245;
	Datchik_L_Pered.Y_koordinata = 275;
	Datchik_L_Pered.id_servo = 0;
	Datchik_L_Pered.napravlenie = 1;

	Datchik_R_Pered.Angle_Ustanovki_in_0 = +135;                //Установлен с наклоном вправо
	Datchik_R_Pered.X_koordinata = 265;
	Datchik_R_Pered.Y_koordinata = 275;
	Datchik_R_Pered.id_servo = 1;
	Datchik_R_Pered.napravlenie = 1;

	Datchik_L_Zad.Angle_Ustanovki_in_0 = +135;                //Установлен с наклоном вправо
	Datchik_L_Zad.X_koordinata = 245;
	Datchik_L_Zad.Y_koordinata = 225;
	Datchik_L_Zad.id_servo = 4;
	Datchik_L_Zad.napravlenie = -1;

	Datchik_R_Zad.Angle_Ustanovki_in_0 = +45;                //Установлен с наклоном вправо
	Datchik_R_Zad.X_koordinata = 265;
	Datchik_R_Zad.Y_koordinata = 225;
	Datchik_R_Zad.id_servo = 5;
	Datchik_R_Zad.napravlenie = -1;
	
	Serial.print(" End Init_Gy56 : ");  Serial.println(millis()); 
}

void Gy56_Start()
{
	//Serial.print(" Start Gy56 : ");  Serial.println(millis());
	takeRangeReading(SensorAddress1); //Tell the sensor to perform a ranging cycle
	takeRangeReading(SensorAddress2); //Tell the sensor to perform a ranging cycle
	takeRangeReading(SensorAddress3); //Tell the sensor to perform a ranging cycle
	takeRangeReading(SensorAddress4); //Tell the sensor to perform a ranging cycle
}

void Gy56_Get()
{
	//Serial.print(" Start Get_Gy56 : ");  Serial.println(millis());

	static int predel_lucha = 200;

	Datchik_L_Pered.Distancia = requestRange(SensorAddress1); //Get the range from the sensor		   Считываем значения после измерения
	Datchik_R_Pered.Distancia = requestRange(SensorAddress2); //Get the range from the sensor
	Datchik_L_Zad.Distancia = requestRange(SensorAddress3); //Get the range from the sensor
	Datchik_R_Zad.Distancia = requestRange(SensorAddress4); //Get the range from the sensor

	//Datchik_L_Pered.Distancia = predel_lucha; //Get the range from the sensor
	//Datchik_R_Pered.Distancia = predel_lucha; //Get the range from the sensor
	//Datchik_L_Zad.Distancia = predel_lucha; //Get the range from the sensor
	//Datchik_R_Zad.Distancia = predel_lucha; //Get the range from the sensor

	//Serial.print("Start_Rashet : ");  Serial.println(millis());
	//Serial.print(" End Range : ");  Serial.print(millis()); 


	Raschet_SinusCosinus(Datchik_L_Pered);						   //Вычисляем синусы косинусы и какой алгоритм расчета
	Raschet_SinusCosinus(Datchik_R_Pered);
	Raschet_SinusCosinus(Datchik_L_Zad);
	Raschet_SinusCosinus(Datchik_R_Zad);

	Serial.print(" RP= "); 	Serial.print(Datchik_R_Pered.Distancia);	
	Serial.print(" LP= ");  	Serial.print(Datchik_L_Pered.Distancia);	
	Serial.print(" RZ= "); 	Serial.print(Datchik_R_Zad.Distancia);	
	Serial.print(" LZ= ");  	Serial.println(Datchik_L_Zad.Distancia);	

	flag_raschet_slam = true;     // Взыодим флаг что можно расчитывать окружающее пространство
}

void Loop_VL53L0X()
{
	if (flag_VL53L0X == true)		  // ВРЕМЯ ИСПОЛНЕНИЯ 0.5 МИЛИСЕКУНДА
	{
		flag_VL53L0X = false;
		static byte porcia_VL53L0X = 1;
		//long a = micros();

		switch (porcia_VL53L0X)
		{
		case 1:Datchik_L_Pered_Verh = Read_VL53L0X(sensor4); porcia_VL53L0X = 2; break;
		case 2:Datchik_R_Pered_Verh = Read_VL53L0X(sensor3); porcia_VL53L0X = 3; break;
		case 3:Datchik_L_Zad_Verh = Read_VL53L0X(sensor2); porcia_VL53L0X = 4; break;
		case 4:Datchik_R_Zad_Verh = Read_VL53L0X(sensor1); porcia_VL53L0X = 1; break;
		}

		//long b = micros();
		//Serial.print("Time flag_VL53L0X = "); 	 Serial.println(b - a);
	}

}

void Loop_GY56()
{
	//==========================================================
	if (flag_Gy56_Start == true)
	{
		flag_Gy56_Start = false;
		Gy56_Start();				 // Запускаем процес измерения на всех датчиках. Они кэтому моменту должны уже занять свое положение для измерений
	}
	//==========================================================
	if (flag_Gy56_End == true)
	{
		flag_Gy56_End = false;
		//Serial.println("D");
		Gy56_Get();				    // Запускаем процессчитывания результатов измерения на всех датчиках. Они кэтому моменту должны уже закончить измерение (55 мисекунд по даташиту)
		//Serial.print("  time millis "); 		Serial.println(millis());
	}

}
void Loop_SLAM()
{
	//==========================================================

//if (flag_raschet_slam == true)            // Если флаг что нужно расчитывать окружающее пространство
//{
//	if (radius_rasheta <= glubina_rascheta)
//		{
//		//Serial.print("rr: "); 		Serial.println(radius_rasheta);

//			Raschet_Okrujenie(Datchik_L_Pered, radius_rasheta);						   //Вычисляем свободное пространство по последним вычислениям
//			Raschet_Okrujenie(Datchik_R_Pered, radius_rasheta);
//			Raschet_Okrujenie(Datchik_L_Zad, radius_rasheta);
//			Raschet_Okrujenie(Datchik_R_Zad, radius_rasheta);
//			
//			radius_rasheta = radius_rasheta + shag_rascheta;
//		}
//	else								  //Если досчитали до конца
//		{
//			//Serial.print("     End_Raschet: ");  Serial.println(millis());
//			radius_rasheta = 0;		    // Сбрасываем в ноль для следующего расчета
//			flag_raschet_slam = false;	 // меняем флаг и прекращаем расчет
//			//Print_Lidar();
//			//Serial.print("EndLidar: ");  Serial.println(millis());
//			//Serial.println("");
//	}
//}

}
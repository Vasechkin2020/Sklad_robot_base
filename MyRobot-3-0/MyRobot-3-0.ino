/*
 Name:		MyRobot_3_0.ino
 Created:	24.09.2019 20:43:24
 Author:	Vasiliy
*/
#define arduino_SLAVE 0x09          // ����� �������������� �����

//#define motor  yes
//#define servomotor  yes
//#define imu9250 yes
//#define AK8963 yes
#define manipul_def yes

//#define imubno055 yes
//#define datchikGy56 yes
//#define bme yes
//#define to_Slave yes
//#define datchikLine yes

//#define datchikVL530 yes

//#define datatime yes

#include <Wire.h>
#include <SPI.h>
#include "time.h"	   
#include "SPI_I2C.h"


#ifdef manipul_def;
	#include "Manipul.h"
#endif 



// ----------------- ���������� ���������� --------------
struct Struct_Motor
{
	volatile uint8_t EncoderPulseA = 0, EncoderPulseB = 0; //, EncoderPulseC = 0;	// ��� ������������ �������� �������� ������� �� ������� �� ������� � 255. ���� �������� ������� �� ����� �� ������� 1 �����
	int motor_pwm = 0;	   // ���������� ����� ��� ���������� �� ������
	float motor_rpm = 0;             // ������� �������� � �������� � ������ 
	float motor_speed = 0;             // �������� �������� �� ������ �� ������� �� �������� � ������� ������ � 62 �� 
	byte motor_napravlenie;		   //����������� �������� �� ��������
	float motor_way = 0;             // ���� ���������� � ������ 

};

Struct_Motor MotorL, MotorR;

struct Struct_Datchik
{
	int Distancia;   // ��������� ��� ���������� ���������
	int Angle;   // ���� ��� ���������� ���������   ������������ ������������� ��������� �������
	int Angle_Ustanovki_in_0;   // ���� ��� ������� ������ ���������� ������������ ������������� ��������� ����������� ���� ��������� ���� �� �����������
	int X_koordinata;     // ����������������� ������� ������������ ��� ��������� 
	int Y_koordinata;	   // ����������������� ������� ������������ ��� ��������� 
	int napravlenie;      // ����������� ���� ��������� ������ �� ������� ���������
	float sinus;		   //�������� ������������ ��� ������� ����� ����, ��������������� ����� ������� ���������� ���������
	float cosinus;		    //�������� ������������ ��� ������� ����� ����, ��������������� ����� ������� ���������� ���������
	int id_servo;		    //����� ����������� �� ������� ���������� ������
	int variant_rascheta;    // �� ������ �������� ����������� ������

};

Struct_Datchik Datchik_L_Pered;		   //��������� ��� ������� ����������� �������
Struct_Datchik Datchik_R_Pered;		   //��������� ��� ������� ����������� �������
Struct_Datchik Datchik_L_Zad;		   //��������� ��� ������� ����������� �������
Struct_Datchik Datchik_R_Zad;		   //��������� ��� ������� ����������� �������

int Datchik_L_Pered_Verh;
int Datchik_R_Pered_Verh;
int Datchik_L_Zad_Verh;
int Datchik_R_Zad_Verh;


char komandaSerial2[127];		// ���������� ���� �������� ������� ���������� ����� ������ � Serial2
long StartTime;


struct Struct_Car
{
	float speed_jelaemaya;           // �������� ��������	� ������� ���� ����� ����������� �� ������ �������� � �������� ��������
	float speed;           // �������� ��������	� ������� ���� ����� ����������� �� ������ �������� � �������� ��������
	float speed_L;		   //�������� �� ������ � ������� ���� ����� � �������� ������ �������	�� ������� ������� ��� ����������
	float speed_R;		   //�������� �� ������ � ������� ���� ����� � �������� ������ �������	�� ������� ������� ��� ����������
	float speed_L_kurs;		   //�������� �� ������ � ������� ���� ����� � �������� ������ �������	� ������ ���������� ��������� �����
	float speed_R_kurs;		   //�������� �� ������ � ������� ���� ����� � �������� ������ �������	� ������ ���������� ��������� �����
	float kurs;			   // ���� �������� ���� ���������
	int napravl;           // ����������� ��������. 1 ������, -1 �����, 2 ������, 3 �����

	float accelerationRun = 0.5;     // ��������� ������� ��������� � ������ � �������
	float accelerationStop = -0.3;     // ���������� ������� ��������� � ������ � �������
	float timeRun ;       // ����� ����������� ���-�� ������� �������� ��������
	float timeStop ;       // ����� ����������� ���-�� ����������� � �������� ��������

	float way;             // ���� ������� ���� ��������
	float way_start;             // ��������� ���� � ������ ������ ������� ����������� ����
	int   way_etap;         // ���� ���� ������ -1 �������� 2 ���������� 3

	float way_1_etap ; // ���� ������� ������� ������� � ������� ���������� �������� ��������
	float way_2_etap ; // ���� ������� ������� ������� � �������� ���������
	float way_3_etap ; // ���� ������� ������� ������� � ������� ��������� ���� ��������

	float angle;         // ���� �� ������� ������������
	float angle_start;
	float angle_start_BNO055;
	float angle_start_MyZ;


	float angle_end;
	float diametrR = 0.24;	   // ��������� ����� �������� ��� ������� ����   � ������
	float diametrL = 0.24;	   // ��������� ����� �������� ��� ������� ����   � ������
};

Struct_Car Car;    

bool flag_gy56; 				// ���� ������� ���������

bool flag_line = false;         // ���� ������� ���������� �������� �����
bool flag_servo = false;         // ���� ������� ������� �������� ����� ����������
bool flag_magnetrom = false;         // ���� ������� ���������� �������������
bool flag_datatime = false;         // ���� ������� ���������� ������� �� DS3231
bool flag_BME280 = false;         // ���� ������� ���������� ����������� �������� ���������
bool flag_VL53L0X = false;         // ���� ������� ���������� �������� ������


bool flag_DataReady_AccGyro = false;  // ���� ������� ���������� ������ ���������-�������������
bool flag_Encoder = false;              // ���� ������� ������������� ������� �������� ���������
bool flag_Gy56_Start = false;          // ���� ������������ ��������� ������ ���������
bool flag_Gy56_End = false;          // ���� ������������ ������� ���������� ���������
bool flag_Platforma = false;          // ���� �������� ��������� �����
bool flag_raschet_slam = false;       // ���� ������� ����������� ������������
bool flag_print = false;       // ���� ��� ������
bool flag_BNO055 = false;       // ���� ��� ���������� ������ �� �������
bool flag_Arduino_Slave = false;       // ���� ��� �������� �� ������ �������
bool flag_Line = false;               // ���� ��� �������� �����



bool flag_PWM = false;                   // ���� ������� ������������� ������������� ���
bool flag_GyroKurs = false;              // ���� ������� ������������� ������������  �������� �� �������� ���������� ��� ����� ��� ������� �� ����
//bool flag_AngleKurs = false;              // ���� ������� ������������� ������������  �������� �� �������� ���������� ��� ����� ��� ������� �� ����
//
//
//bool flag_StopWay = false;              // ���� ������� ������������� ���������� ������ ��� ���������� ������� ���������
//bool flag_StopAngle = false;              // ���� ������� ������������� ���������� ������ ��� ���������� ������� �����

float way_start;
float stopWay_way_now;
float stopAngle_otklonenie_gyro;

float dlinna_dugi;
float dlinna_angle;	   // ������ ���� � 1 ������

float pred_otklonenie_L, now_otklonenie_L, sum_otklonenie_L, speed_otklonenie_L;
float pred_otklonenie_R, now_otklonenie_R, sum_otklonenie_R, speed_otklonenie_R;
float pred_otklonenie_gyro, now_otklonenie_gyro, sum_otklonenie_gyro, speed_otklonenie_gyro;

float changeSpeed_L, changeSpeed_R;
float new_Speed_L, new_Speed_R;

float changeSpeed_gyro;

bool flag_a = true,flag_b = true, flag_c = true;      // ���� ��� ������
bool flag_komanda = false;

int switch_program = 0;  // ����� ����� ��������� ��� ����������
double timeStop;             // ����� �� ������� ����� ����� ������ ���������

int radius_rasheta = 0;           // ��� ������� ����������� ������������


float intervalEncoder = 0;	// ��� ������� ��������� ����� �������� ��������	  � ��������
long now_time;			   //���������� ��� ����������� �������� �������
unsigned long second;      // ������� � ������ ������� ���������

int   magD;                // ����������� �� �������������
int chislo_shagov = 0;


float l, r;
double time_servoUp = 0;
long time_comanda = 0;

//#include "EEPROM_My.h"
#include "BME280.h"
#include <EEPROM.h>

//#include "AHRS.h"

//AHRS myAHRS;          // ��� ���������� ������ ��� �������� ���� ������ �� 9255



#include "MPU9255.h"
#include "BNO055.h"
#include "Motor.h"
#include "PWM.h"
#include "Magnetrom.h"
#include "MyServo.h"
#include "Timer.h"
#include "Clock_DS3231.h"
#include "Line.h"
#include "Lidar.h"
#include "Gy_56.h"

#include "Output.h"

void setup() 
{
	pinMode(8, OUTPUT);		   //��� �������
 	digitalWrite(8, LOW);	delay(100);		digitalWrite(8, HIGH);	   //��������� �������. �� LOW ��� ����� �� HIGH ���

																	   //---------------------------------
	SPI.begin();
	SPI.setDataMode(SPI_MODE3);
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockDivider(SPI_CLOCK_DIV2);  // ������ SPI_CLOCK_DIV2 �������� 2 � ������� ������� 16 = 8 �����
	//---------------------------------

	Wire.begin(); // ����������� � ���� I2C � ���� ��������.
	Wire.setClock(400000); // �������� �������� ������ 400 ����/�.
	//Wire.setClock(100000); // �������� �������� ������ 100 ����/�.

	Serial.begin(115200);
	Serial2.begin(115200);   // ��� ������ ����� ������

	delay(3000);      // ���� 3 ������� ���� ������ �����������
	
	i2c_scan();
	Serial2.println(" ");
	Serial2.println(" -------------------------------------------------");
	Serial2.println(" START 111 SetUp !!! ");
	Serial2.println(" -------------------------------------------------");

	Timer5_Init();				 //������ ��� ��� �������


	//Serial.print(" Init_PIN for Analizator... Time: ");  Serial.println(millis());

	//pinMode(41, OUTPUT);     //    ��� ����������� ����� 
	//pinMode(43, OUTPUT);     //    ��� ����������� ����� 
	//pinMode(45, OUTPUT);     //    ��� ����������� ����� 
	//pinMode(47, OUTPUT);     //    ��� ����������� ����� 

#ifdef datchikLine
	Init_Line();			 // ������������� � ���� �������� �����
#endif 

#ifdef manipul_def;
	pwm.begin();				//��� ������������ � ���������� �������������� ������� �� i2c 0x70
	pwm.setPWMFreq(SERVO_FREQ);  // ������� ���������� ��������� 60 ��
	Init_Manipul();			 // ������������� � ���� �������� �����
#endif 

//	Test_EEPROM();		// ��������� ������ ���������� ������
	

#ifdef datchikGy56
	Init_Gy56();          // ������������� ��������		����������� ���� ������������ �������� ������� �� 0�70
#endif 

#ifdef Lidar
	Init_Lidar();

	//Print_Lidar();
//
//	for (float i = -135; i <= 135; i = i + 5)
//	{
//		long a = micros();
//		Raschet_Kordinat(307.5, 322.5, i, 300, 1);
//
////		Raschet_Kordinat(290, 325, i, 300, 1);
//		//Raschet_Kordinat(310, 325, i, 300, 1);
//
//		long b = micros();
//		Serial.print("Time : ");Serial.println(b-a);
//	}
//
//	Print_Lidar();
//
//	delay(999999);

#endif 

#ifdef datchikVL530
	Init_VL53L0X();		 // ������������� �������� ������

#endif 

#ifdef bme
	Init_BME280();
#endif 

#ifdef motor
	ChangePWM();   // ��������� ���� ��� ������ ��� ������ ������, ����� �� ������ 
	MotorInit();   //������������� ������� � ��������		
	//Serial.print("Run_TEST_Motor "); // Serial.println(100);
	//Run_MotorL(50);
	//Run_MotorR(50);
	//delay(3000);
	//Motor_Stop();
	//Serial.print("End_TEST_Motor... ");	 
	//delay(1000);
#endif 

#ifdef servomotor

	Timer3_Init();               //������ ��� �����������	
	ServoMotor_Init();			 // ������������� � ��������� �������� �������������
	//TIMSK3 = 0;   //��������� ������ ��� ������������ �� 3 �������	  ��� �� ��������
#endif 
#ifdef imubno055
	Init_BNO055();
	//Calibrovka_BNO055();
	ReadCalibrovka_BNO055();
	BNO055_Start();               // ������ ������� � �������� ������
#endif

#ifdef imu9250
	Init_9255();
	Calibrovka_AccGyro();		  // ���� ����� �� ��������� ������������-��������
	//ReadCalibrovka_AccGyro();	  // ���� ��� ������������ �� ������ ��������� ��������
	//   �������� ���������� ����������
	attachInterrupt(5, int5_func, RISING);   // ��������� ���������� ��� 9255 ����� 5 �� 18 ����. �� ��� ��� ������� � �������
#endif

#ifdef AK8963
	Init_AK8963();
	//Calibrovka_Mag();			 // ���� ����� �� ��������� ������������
	ReadCalibrovka_Mag();		 // ���� ��� ������������ �� ������ ��������� ��������	
	//Test_Colibrovka_Magnetrometr();
	//delay(10000);

#endif
	Serial.println (" ///////////////////  START PROGRAMM /////////////////////////////");
	Serial2.println(" ///////////////////  START PROGRAMM /////////////////////////////");
	//set_time_ManipulServo(0, 90, 5);
	//set_time_ManipulServo(1, 90, 5);
	//set_time_ManipulServo(2, 0, 5);
	//set_time_ManipulServo(3, 0, 5);
	//set_time_ManipulServo(4, 45, 5);
	//delay(500000000);

	digitalWrite(8, LOW);	delay(50);		digitalWrite(8, HIGH);	   //��������� �������. �� LOW ��� ����� �� HIGH ���
	delay(50);
	digitalWrite(8, LOW);	delay(50);  	digitalWrite(8, HIGH);	   //��������� �������. �� LOW ��� ����� �� HIGH ���
	StartTime = millis();

	
}

int flagservo = 0;
// the loop function runs over and over again until power down or reset
void loop()
{
	while (true)
	{	
		Serial2.println("+");
		if ((millis() - StartTime > 5000) && (flagservo == 0))
			{
			Serial.println("!!!!!!!!!!!!!!");
			set_time_ManipulServo(0, 90, 3);
			set_time_ManipulServo(1, -80, 3);
			set_time_ManipulServo(2, -45, 3);
			set_time_ManipulServo(3, 45, 3);
			set_time_ManipulServo(4, 0, 3);


			flagservo = 1;
			StartTime = millis();

		}
		//if ((millis() - StartTime > 11000) && (flagservo == 1)) 
		//{
		//	Serial.println("!!!!!!SJAT!!!!!!!");
		//	//set_time_ManipulServo(4, 45, 2);

		//}

		if ((millis() - StartTime > 5000) && (flagservo == 1))
		{
			Serial.println("!!!!!!2222!!!!!!!");
			//set_time_ManipulServo(0, 0, 3);
			//set_time_ManipulServo(1, -90, 3);
			//set_time_ManipulServo(2, -90, 3);
			//set_time_ManipulServo(3, 45, 3);
			set_time_ManipulServo(4, 90, 3);


			flagservo = 2;
			StartTime = millis();
		}
		if ((millis() - StartTime > 5000) && (flagservo == 2))
		{
			Serial.println("!!!!!!2222!!!!!!!");
			set_time_ManipulServo(0, 90, 3);
			set_time_ManipulServo(1, -30, 3);
			set_time_ManipulServo(2, 0, 3);
			set_time_ManipulServo(3, -45, 3);
			set_time_ManipulServo(4, 90, 3);


			flagservo = 3;
			StartTime = millis();
		}
		if ((millis() - StartTime > 5000) && (flagservo == 3))
		{
			Serial.println("!!!!!!2222!!!!!!!");
			set_time_ManipulServo(0, 90, 3);
			set_time_ManipulServo(1, -15, 3);
			set_time_ManipulServo(2, 0, 3);
			set_time_ManipulServo(3, -60, 3);
			set_time_ManipulServo(4, 100, 3);


			flagservo = 0;
			StartTime = millis();
		}

		char komanda = GetCharSerial2();
		if (komanda == '0')
		{
			Motor_Stop();
			flag_komanda = false;
			chislo_shagov = 0;
		}

		if (komanda == '1')
		{
			switch_program = 1;
			flag_komanda = true;
			Serial2.println("switch_program = 1");
		}
		if (komanda == '2')
		{
			Serial2.print("Car.kurs "); Serial2.print(Car.kurs);
			Car.kurs = Car.kurs + 45;
			Serial2.print(" Car.kursNew "); Serial2.println(Car.kurs);

		}
		if (komanda == '3')
		{
			//Serial2.print("Car.kurs "); Serial2.print(Car.kurs);
			Car.kurs = Car.kurs - 45;
			//Serial2.print(" Car.kursNew "); Serial2.println(Car.kurs);
		}
		if (komanda == '4')
		{
			Serial2.println("komanda == 4 ");
			flag_komanda = true;
		}									

		if (komanda == '5')
		{
			Run_MotorL(PWM_from_Speed(0.1));
			Run_MotorR(PWM_from_Speed(0.3));
			Serial2.println("komanda == 5 ");
			flag_komanda = true;
		}


		//if (flag_komanda == true && (millis() - time_comanda > 200) && chislo_shagov < 18 )
		//{
		//	Car.kurs = Car.kurs - 5;
		//	if (Car.kurs > 360) Car.kurs = Car.kurs - 360;
		//	if (Car.kurs < 0) Car.kurs = Car.kurs + 360;
		//	chislo_shagov ++;
		//	
		//	Serial2.print(" + ");  Serial2.print(Car.kurs);
		//	Serial2.print(" + ");  Serial2.println(angleGyroZ);
		//	
		//	time_comanda = millis();
		//}
		if (flag_komanda == true && (millis() - time_comanda > 200))
		{
			Car.kurs = Car.kurs - 5;
			if (Car.kurs > 360) Car.kurs = Car.kurs - 360;
			if (Car.kurs < 0) Car.kurs = Car.kurs + 360;
			chislo_shagov++;

			Serial2.print(" + ");  Serial2.print(Car.kurs);
			Serial2.print(" + ");  Serial2.println(angleGyroZ);

			time_comanda = millis();
		}

#ifdef motor
		Loop_Encoder();
		Loop_PWN();
		Loop_Switch();
		//MotorR(0, 0);
		//MotorL(0, 0);
		//delay(3000);			  

#endif

#ifdef manipul_def
		Loop_Manipul();
#endif

#ifdef imu9250
		Loop_9255();
#endif

#ifdef AK8963
		Loop_AK8963();
#endif

#ifdef imubno055
		Loop_BNO055();
#endif

#ifdef servomotor
		Loop_Servo();	
		PodemServo();					   // �������� ������ � ��������� ������������ �������
#endif

#ifdef datchikGy56
		Loop_GY56();
#endif

#ifdef datchikVL530
		Loop_VL53L0X();
#endif

#ifdef bme
		Loop_BME280();
#endif

#ifdef datchikLine
		Loop_Line();
#endif	

#ifdef to_Slave
		Loop_to_Slave();
#endif


		//Loop_SLAM();
		//Loop_DataTime();




		//if (flag_print == true)
		//{
		//	digitalWrite(49, 1);
			//Print_to_Serial2();		      // ������� ������ ������ � ���� ����� ������	��������


		//	//Serial.print("Seconds: ");  Serial.println(second);
		//	//Print_Lidar();
		//	//PrintGyroTest();
		//}

		//digitalWrite(49, 0);
	}
}
/*
 Name:		MyRobot_3_0.ino
 Created:	24.09.2019 20:43:24
 Author:	Vasiliy
*/
#define arduino_SLAVE 0x09          // Адрес дополнительной платы

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



// ----------------- Глобальные переменные --------------
struct Struct_Motor
{
	volatile uint8_t EncoderPulseA = 0, EncoderPulseB = 0; //, EncoderPulseC = 0;	// При максимальной скорости вращения энкодер не выходит за границу в 255. если моторчик быстрее то может не хватить 1 байта
	int motor_pwm = 0;	   // Запоминаем какой ШИМ установлен на моторе
	float motor_rpm = 0;             // Частота вращения в оборотах в минуту 
	float motor_speed = 0;             // Скорость движения по колесу по формуле из оборотов и радиуса колеса в 62 мм 
	byte motor_napravlenie;		   //Направление вращения по энкодеру
	float motor_way = 0;             // Путь пройденный в метрах 

};

Struct_Motor MotorL, MotorR;

struct Struct_Datchik
{
	int Distancia;   // Растояние при последенем измерении
	int Angle;   // Угол при последенем измерении   относительно диаметральной плоскости машинки
	int Angle_Ustanovki_in_0;   // Угол под которым датчик установлен относительно диаметральной плоскости машинкиугол куда направлен ноль на сервомоторе
	int X_koordinata;     // Месторасположение датчика относительно оси координат 
	int Y_koordinata;	   // Месторасположение датчика относительно оси координат 
	int napravlenie;      // Направление куда направлен датчик по системе координат
	float sinus;		   //Значения используемые для расчета точек луча, пересчитываются после каждого завершения измерения
	float cosinus;		    //Значения используемые для расчета точек луча, пересчитываются после каждого завершения измерения
	int id_servo;		    //Номер сервомотора на который установлен датчик
	int variant_rascheta;    // По какому варианту расчитываем катеты

};

Struct_Datchik Datchik_L_Pered;		   //Структура для хранени результатов датчика
Struct_Datchik Datchik_R_Pered;		   //Структура для хранени результатов датчика
Struct_Datchik Datchik_L_Zad;		   //Структура для хранени результатов датчика
Struct_Datchik Datchik_R_Zad;		   //Структура для хранени результатов датчика

int Datchik_L_Pered_Verh;
int Datchik_R_Pered_Verh;
int Datchik_L_Zad_Verh;
int Datchik_R_Zad_Verh;


char komandaSerial2[127];		// Переменная куда помещвем копанду полученную через блютуз в Serial2
long StartTime;


struct Struct_Car
{
	float speed_jelaemaya;           // Заданная скорость	с которой надо ехать максимально на участе движения с заданной скоротью
	float speed;           // Заданная скорость	с которой надо ехать максимально на участе движения с заданной скоротью
	float speed_L;		   //Скорость на моторе с которой надо ехать в заданный момент времени	на участке разгона или торможения
	float speed_R;		   //Скорость на моторе с которой надо ехать в заданный момент времени	на участке разгона или торможения
	float speed_L_kurs;		   //Скорость на моторе с которой надо ехать в заданный момент времени	с учетом иподдержки заданного курса
	float speed_R_kurs;		   //Скорость на моторе с которой надо ехать в заданный момент времени	с учетом иподдержки заданного курса
	float kurs;			   // Курс которому надо следовать
	int napravl;           // Направление движения. 1 вперед, -1 назад, 2 вправо, 3 влево

	float accelerationRun = 0.5;     // Ускорение машинки константа к метрах в секунду
	float accelerationStop = -0.3;     // Торможение машинки константа к метрах в секунду
	float timeRun ;       // Время необходимое что-бы достичь заданной скорости
	float timeStop ;       // Время необходимое что-бы затормозить с заданной скорости

	float way;             // Путь который надо проехать
	float way_start;             // Начальный путь в момент старта отсчета пройденного пути
	int   way_etap;         // Этап пути разгон -1 движение 2 торможение 3

	float way_1_etap ; // Путь который пройдет машинка к моменту достижения заданной скорости
	float way_2_etap ; // Путь который пройдет машинка с заданной скоростью
	float way_3_etap ; // Путь который пройдет машинка к моменту остановки пока тормозит

	float angle;         // Угол на который поворачиваем
	float angle_start;
	float angle_start_BNO055;
	float angle_start_MyZ;


	float angle_end;
	float diametrR = 0.24;	   // Растояние между колесами для расчета дуги   в метрах
	float diametrL = 0.24;	   // Растояние между колесами для расчета дуги   в метрах
};

Struct_Car Car;    

bool flag_gy56; 				// Флаг датчика измерений

bool flag_line = false;         // Флаг статуса считывания датчиков линии
bool flag_servo = false;         // Флаг статуса времени поворота серво моторчиков
bool flag_magnetrom = false;         // Флаг статуса считывания магнетрометра
bool flag_datatime = false;         // Флаг статуса считывания времени из DS3231
bool flag_BME280 = false;         // Флаг статуса считывания температуры давления влажности
bool flag_VL53L0X = false;         // Флаг статуса считывания датчиков сверху


bool flag_DataReady_AccGyro = false;  // Флаг статуса готовности данных гироскопа-Аксельрометра
bool flag_Encoder = false;              // Флаг статуса необходимости считать хначения эннкодера
bool flag_Gy56_Start = false;          // Флаг неоходимости запустить начало измерений
bool flag_Gy56_End = false;          // Флаг неоходимости считать результаты измерений
bool flag_Platforma = false;          // Флаг поднятия платформы вверх
bool flag_raschet_slam = false;       // Флаг расчета окружающего пространства
bool flag_print = false;       // Флаг для печати
bool flag_BNO055 = false;       // Флаг для считывания данных по датчику
bool flag_Arduino_Slave = false;       // Флаг для передачи на вторую ардуино
bool flag_Line = false;               // Флаг для датчиков линии



bool flag_PWM = false;                   // Флаг статуса необходимости регулирования ШИМ
bool flag_GyroKurs = false;              // Флаг статуса необходимости поддерживать  движение по заданный траектории или прямо или поворот на угол
//bool flag_AngleKurs = false;              // Флаг статуса необходимости поддерживать  движение по заданный траектории или прямо или поворот на угол
//
//
//bool flag_StopWay = false;              // Флаг статуса необходимости остановить моторы при достижении нужного растояния
//bool flag_StopAngle = false;              // Флаг статуса необходимости остановить моторы при достижении нужного курса

float way_start;
float stopWay_way_now;
float stopAngle_otklonenie_gyro;

float dlinna_dugi;
float dlinna_angle;	   // Длинна дуги в 1 градус

float pred_otklonenie_L, now_otklonenie_L, sum_otklonenie_L, speed_otklonenie_L;
float pred_otklonenie_R, now_otklonenie_R, sum_otklonenie_R, speed_otklonenie_R;
float pred_otklonenie_gyro, now_otklonenie_gyro, sum_otklonenie_gyro, speed_otklonenie_gyro;

float changeSpeed_L, changeSpeed_R;
float new_Speed_L, new_Speed_R;

float changeSpeed_gyro;

bool flag_a = true,flag_b = true, flag_c = true;      // Флаг для тестов
bool flag_komanda = false;

int switch_program = 0;  // Выбор шагов программы для исполнения
double timeStop;             // Время дл яначала паузы между шагами программы

int radius_rasheta = 0;           // Для расчета окружающего пространства


float intervalEncoder = 0;	// Для расчета интревала между опросами энкодера	  в секундах
long now_time;			   //Переменная для запоминания текущего времени
unsigned long second;      // секунды с начала запуска программы

int   magD;                // Направление по магнетрометру
int chislo_shagov = 0;


float l, r;
double time_servoUp = 0;
long time_comanda = 0;

//#include "EEPROM_My.h"
#include "BME280.h"
#include <EEPROM.h>

//#include "AHRS.h"

//AHRS myAHRS;          // Моя переменная класса для хранения всех данных по 9255



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
	pinMode(8, OUTPUT);		   //Для пищалки
 	digitalWrite(8, LOW);	delay(100);		digitalWrite(8, HIGH);	   //Выключаем пищалку. На LOW она пищит на HIGH нет

																	   //---------------------------------
	SPI.begin();
	SPI.setDataMode(SPI_MODE3);
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockDivider(SPI_CLOCK_DIV2);  // чтение SPI_CLOCK_DIV2 делитель 2 к частоте адруино 16 = 8 Мгерц
	//---------------------------------

	Wire.begin(); // подключение к шине I2C в роли ведущего.
	Wire.setClock(400000); // скорость передачи данных 400 кБит/с.
	//Wire.setClock(100000); // скорость передачи данных 100 кБит/с.

	Serial.begin(115200);
	Serial2.begin(115200);   // Для вывода через блютуз

	delay(3000);      // Ждем 3 секунды пока блютуз соедениться
	
	i2c_scan();
	Serial2.println(" ");
	Serial2.println(" -------------------------------------------------");
	Serial2.println(" START 111 SetUp !!! ");
	Serial2.println(" -------------------------------------------------");

	Timer5_Init();				 //Таймер под мои отсчеты


	//Serial.print(" Init_PIN for Analizator... Time: ");  Serial.println(millis());

	//pinMode(41, OUTPUT);     //    для анализатора выход 
	//pinMode(43, OUTPUT);     //    для анализатора выход 
	//pinMode(45, OUTPUT);     //    для анализатора выход 
	//pinMode(47, OUTPUT);     //    для анализатора выход 

#ifdef datchikLine
	Init_Line();			 // Инициализация и тест датчиков линии
#endif 

#ifdef manipul_def;
	pwm.begin();				//ТУт перезагрузка и отключение дополнительных ардесов по i2c 0x70
	pwm.setPWMFreq(SERVO_FREQ);  // Частота следования импульсов 60 Гц
	Init_Manipul();			 // Инициализация и тест датчиков линии
#endif 

//	Test_EEPROM();		// Проверяем работу микросхемы памяти
	

#ifdef datchikGy56
	Init_Gy56();          // Инициализация датчиков		обязательно ниже манипулятора конфликт адресов по 0х70
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
	Init_VL53L0X();		 // Инициализация датчиков сверху

#endif 

#ifdef bme
	Init_BME280();
#endif 

#ifdef motor
	ChangePWM();   // изменение ШИМа под моторы для луйшей работы, можно не менять 
	MotorInit();   //инициализация моторов и энкодера		
	//Serial.print("Run_TEST_Motor "); // Serial.println(100);
	//Run_MotorL(50);
	//Run_MotorR(50);
	//delay(3000);
	//Motor_Stop();
	//Serial.print("End_TEST_Motor... ");	 
	//delay(1000);
#endif 

#ifdef servomotor

	Timer3_Init();               //Таймер под сервомоторы	
	ServoMotor_Init();			 // Инициализация в начальное состоние сервоприводов
	//TIMSK3 = 0;   //выключаем таймер для сервомоторов на 3 таймере	  они не работают
#endif 
#ifdef imubno055
	Init_BNO055();
	//Calibrovka_BNO055();
	ReadCalibrovka_BNO055();
	BNO055_Start();               // Запуск датчика в заданном режиме
#endif

#ifdef imu9250
	Init_9255();
	Calibrovka_AccGyro();		  // Если нужно то колибруем аксельрометр-гироскоп
	//ReadCalibrovka_AccGyro();	  // Если уже откалиброано то просто считываем значения
	//   Включаем прерывания последними
	attachInterrupt(5, int5_func, RISING);   // Запускаем Прерывание для 9255 номер 5 на 18 пине. Не так как описано в пинауте
#endif

#ifdef AK8963
	Init_AK8963();
	//Calibrovka_Mag();			 // Если нужно то колибруем магнетрометр
	ReadCalibrovka_Mag();		 // Если уже откалиброано то просто считываем значения	
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

	digitalWrite(8, LOW);	delay(50);		digitalWrite(8, HIGH);	   //Выключаем пищалку. На LOW она пищит на HIGH нет
	delay(50);
	digitalWrite(8, LOW);	delay(50);  	digitalWrite(8, HIGH);	   //Выключаем пищалку. На LOW она пищит на HIGH нет
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
		PodemServo();					   // Тестовый подьем и опускание сервомоторов силовых
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
			//Print_to_Serial2();		      // Функция вывода данных в порт чреез блютуз	порциями


		//	//Serial.print("Seconds: ");  Serial.println(second);
		//	//Print_Lidar();
		//	//PrintGyroTest();
		//}

		//digitalWrite(49, 0);
	}
}

#include "Adafruit_PWMServoDriver.h"
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates

struct Struct_Manipul_ServoMotor
{
	byte port;	 //Порт платы драйвера управления сервомоторами
	byte pin;	 //Аналоговый Пин на котороый обратная связь от потенциометра
	int ugol;    //Угол на который поворачивается мотор
	int min_impuls;				 //Минимальный имульс по дадашиту примерно 500 
	int max_impuls;				 //Максимальный импульс	 по дадашиту  примерно 2500
	float tek_polojenie;			//Текущее положение		оно же 	Начальное положение при инициализации (запуске)  в градусах	
	float zadanoe_polojenie;		//Положение в которое надо перемститься	  в градусах
	float time_pulse = 0;		   //Время дня поворота на 1 градус

	int SERVOMIN;                 // Минимальное положение 	 считаем как	 500 / (1000000/60/4096)  в имульсах драйвера
	int SERVOMAX;	              // Максимальное положение 	 считаем как	 1000000/60/4096*2500	 в имульсах драйвера
	int SERVOSER;	              // Среднее  положение 	0 градусов  	 в имульсах драйвера
	int SERVOTEK;	              // Текущее  положение 	 в имульсах драйвера
	int SERVOZAD;	              // Заданное положение 	 в имульсах драйвера
	float SERVOTIME;		      //изменение положения для дня поворота на 1 градус  в имульсах драйвера

	long time_izmen = 0;                // время когда менали последний раз угол
};

byte const col_mot = 5;   // Кол-во моторов на манипуляторе

Struct_Manipul_ServoMotor Manipul[col_mot];		  // Массив на 6 моторчиков

void set_ManipulServo(byte num_servo, float gradus)
{
	Manipul[num_servo].SERVOZAD = Manipul[num_servo].SERVOSER - (gradus * Manipul[num_servo].SERVOTIME);
	pwm.setPWM(Manipul[num_servo].port, 0, Manipul[num_servo].SERVOZAD);

	Manipul[num_servo].SERVOTEK = Manipul[num_servo].SERVOZAD;
	Manipul[num_servo].tek_polojenie = gradus;
	Manipul[num_servo].zadanoe_polojenie = gradus;
	Serial.print(" polojenie= ");	Serial.println(Manipul[num_servo].SERVOZAD);
}

void set_time_ManipulServo(byte num_servo, float gradus, float time) // время в секундах
{
	//Serial.print(" num_servo= ");	Serial.println(num_servo);
	//Serial.print(" SERVOTIME= ");	Serial.println(Manipul[num_servo].SERVOTIME);
	//Serial.print(" SERVOTEK= ");	Serial.println(Manipul[num_servo].SERVOTEK);
	float dif_pulse = abs((gradus - Manipul[num_servo].tek_polojenie) * Manipul[num_servo].SERVOTIME );
	//Serial.print(" dif_pulse= ");	Serial.println(dif_pulse);
	Manipul[num_servo].time_pulse = (time * 1000000) / dif_pulse;		   //Время межу импульсами при увеличении уменьшении на 1 импульс
	//Serial.print(" time_pulse= ");	Serial.println(Manipul[num_servo].time_pulse);
	
	Manipul[num_servo].zadanoe_polojenie = gradus;			  //Задаем новое положение	  в градусах
	Manipul[num_servo].SERVOZAD = Manipul[num_servo].SERVOSER - (gradus * Manipul[num_servo].SERVOTIME); 
}


void Init_Manipul()
{

	pinMode(A12, INPUT);
	pinMode(A13, INPUT);
	pinMode(A14, INPUT);
	pinMode(A15, INPUT);

	 // Моторчик основание вертеть
	byte port = 0;
	Manipul[port].port = port;
	Manipul[port].pin = A15;

	Manipul[port].ugol = 270;
	Manipul[port].min_impuls = 530;
	Manipul[port].max_impuls = Manipul[port].min_impuls + 2000;
	Manipul[port].SERVOMIN = Manipul[port].min_impuls / ((float)1000000 / SERVO_FREQ / 4096) ;	//   Serial.print (" SERVOMIN= ");	Serial.print(Manipul[0].SERVOMIN);
	Manipul[port].SERVOMAX = Manipul[port].max_impuls / ((float)1000000 / SERVO_FREQ / 4096);	 //  Serial.print (" SERVOMAX= ");	Serial.print(Manipul[0].SERVOMAX);
	Manipul[port].SERVOTIME = (Manipul[port].SERVOMAX - Manipul[port].SERVOMIN) / (float) Manipul[port].ugol;	 // Serial.print (" SERVOTIME= ");	Serial.print(Manipul[0].SERVOTIME,4);
	Manipul[port].SERVOSER = Manipul[port].SERVOMIN + (Manipul[port].SERVOTIME * (Manipul[port].ugol / 2)); //  Serial.print (" SERVOSER= ");	Serial.println(Manipul[0].SERVOSER);

	int zn = analogRead(Manipul[port].pin);
	Serial.print("port");Serial.print(port);Serial.print(" = "); Serial.print(zn);    // Аналоговый вход для считывания положения сервомотора
	int zd = map(zn, 63, 650, Manipul[port].ugol / 2, -Manipul[port].ugol / 2);
	Serial.print(" Start Angle = "); Serial.println(zd);    // Аналоговый вход для считывания положения сервомотора

	Manipul[port].tek_polojenie = zd;			  //Присваиваем считанное положение при запуске
	Manipul[port].SERVOTEK = Manipul[port].SERVOSER - (Manipul[port].tek_polojenie * Manipul[port].SERVOTIME);

	Manipul[port].zadanoe_polojenie = 0;		 //Задаем положение при запуске

	set_time_ManipulServo(port, Manipul[port].zadanoe_polojenie, 2);	 //Команда встать в заданное начальное положение за нужное время в лупе
	//set_ManipulServo(port, Manipul[port].tek_polojenie);   // 	Встаем в начальное положение  с максимальной скоростью

	 // Моторчик  1 ступени
	port = 1;
	Manipul[port].port = port;
	Manipul[port].pin = A14;

	Manipul[port].ugol = 270;
	Manipul[port].min_impuls = 520;
	Manipul[port].max_impuls = Manipul[port].min_impuls+2000;
	Manipul[port].SERVOMIN = Manipul[port].min_impuls / ((float)1000000 / SERVO_FREQ / 4096);	//   Serial.print (" SERVOMIN= ");	Serial.print(Manipul[0].SERVOMIN);
	Manipul[port].SERVOMAX = Manipul[port].max_impuls / ((float)1000000 / SERVO_FREQ / 4096);	 //  Serial.print (" SERVOMAX= ");	Serial.print(Manipul[0].SERVOMAX);
	Manipul[port].SERVOTIME = (Manipul[port].SERVOMAX - Manipul[port].SERVOMIN) / (float)Manipul[port].ugol;	 // Serial.print (" SERVOTIME= ");	Serial.print(Manipul[0].SERVOTIME,4);
	Manipul[port].SERVOSER = Manipul[port].SERVOMIN + (Manipul[port].SERVOTIME * (Manipul[port].ugol / 2)); //  Serial.print (" SERVOSER= ");	Serial.println(Manipul[0].SERVOSER);

	zn = analogRead(Manipul[port].pin);
	Serial.print("port");Serial.print(port);Serial.print(" = "); Serial.print(zn);    // Аналоговый вход для считывания положения сервомотора
	zd = map(zn, 50, 600, Manipul[port].ugol / 2, -Manipul[port].ugol / 2);
	Serial.print(" Start Angle = "); Serial.println(zd);    // Аналоговый вход для считывания положения сервомотора

	Manipul[port].tek_polojenie = -zd;			  //Присваиваем считанное положение при запуске
	Manipul[port].SERVOTEK = Manipul[port].SERVOSER - (Manipul[port].tek_polojenie * Manipul[port].SERVOTIME);

	Manipul[port].zadanoe_polojenie = 0;		 //Задаем положение при запуске

	set_time_ManipulServo(port, Manipul[port].zadanoe_polojenie, 5);	 //Команда встать в заданное начальное положение за нужное время в лупе

//	set_ManipulServo(port, Manipul[port].tek_polojenie);   // 	Встаем в начальное положение  с максимальной скоростью

	 // Моторчик  2 ступени
	port = 2;
	Manipul[port].port = port;
	Manipul[port].pin = A13;

	Manipul[port].ugol = 270;
	Manipul[port].min_impuls = 540;
	Manipul[port].max_impuls = Manipul[port].min_impuls + 2000;
	Manipul[port].SERVOMIN = Manipul[port].min_impuls / ((float)1000000 / SERVO_FREQ / 4096);	//   Serial.print (" SERVOMIN= ");	Serial.print(Manipul[0].SERVOMIN);
	Manipul[port].SERVOMAX = Manipul[port].max_impuls / ((float)1000000 / SERVO_FREQ / 4096);	 //  Serial.print (" SERVOMAX= ");	Serial.print(Manipul[0].SERVOMAX);
	Manipul[port].SERVOTIME = (Manipul[port].SERVOMAX - Manipul[port].SERVOMIN) / (float)Manipul[port].ugol;	 // Serial.print (" SERVOTIME= ");	Serial.print(Manipul[0].SERVOTIME,4);
	Manipul[port].SERVOSER = Manipul[port].SERVOMIN + (Manipul[port].SERVOTIME * (Manipul[port].ugol / 2)); //  Serial.print (" SERVOSER= ");	Serial.println(Manipul[0].SERVOSER);

	zn = analogRead(Manipul[port].pin);
	Serial.print("port");Serial.print(port);Serial.print(" = "); Serial.print(zn);    // Аналоговый вход для считывания положения сервомотора
	zd = map(zn, 65, 630, Manipul[port].ugol / 2, -Manipul[port].ugol / 2);
	Serial.print(" Start Angle = "); Serial.println(zd);    // Аналоговый вход для считывания положения сервомотора

	Manipul[port].tek_polojenie = zd;			  //Присваиваем считанное положение при запуске
	Manipul[port].SERVOTEK = Manipul[port].SERVOSER - (Manipul[port].tek_polojenie * Manipul[port].SERVOTIME);

	Manipul[port].zadanoe_polojenie = 0;		 //Задаем положение при запуске

	set_time_ManipulServo(port, Manipul[port].zadanoe_polojenie, 2);	 //Команда встать в заданное начальное положение за нужное время в лупе

//	set_ManipulServo(port, Manipul[port].tek_polojenie);   // 	Встаем в начальное положение  с максимальной скоростью

		 // Моторчик  3 ступени
	port = 3;
	Manipul[port].port = port;
	Manipul[port].pin = A12;

	Manipul[port].ugol = 270;
	Manipul[port].min_impuls = 515;
	Manipul[port].max_impuls = Manipul[port].min_impuls + 2000;
	Manipul[port].SERVOMIN = Manipul[port].min_impuls / ((float)1000000 / SERVO_FREQ / 4096);	//   Serial.print (" SERVOMIN= ");	Serial.print(Manipul[0].SERVOMIN);
	Manipul[port].SERVOMAX = Manipul[port].max_impuls / ((float)1000000 / SERVO_FREQ / 4096);	 //  Serial.print (" SERVOMAX= ");	Serial.print(Manipul[0].SERVOMAX);
	Manipul[port].SERVOTIME = (Manipul[port].SERVOMAX - Manipul[port].SERVOMIN) / (float)Manipul[port].ugol;	 // Serial.print (" SERVOTIME= ");	Serial.print(Manipul[0].SERVOTIME,4);
	Manipul[port].SERVOSER = Manipul[port].SERVOMIN + (Manipul[port].SERVOTIME * (Manipul[port].ugol / 2)); //  Serial.print (" SERVOSER= ");	Serial.println(Manipul[0].SERVOSER);


	zn = analogRead(Manipul[port].pin);
	Serial.print("port");Serial.print(port);Serial.print(" = "); Serial.print(zn);    // Аналоговый вход для считывания положения сервомотора
	zd = map(zn, 65, 630, Manipul[port].ugol / 2, -Manipul[port].ugol / 2);
	Serial.print(" Start Angle = "); Serial.println(zd);    // Аналоговый вход для считывания положения сервомотора

	Manipul[port].tek_polojenie = zd;			  //Присваиваем считанное положение при запуске
	Manipul[port].SERVOTEK = Manipul[port].SERVOSER - (Manipul[port].tek_polojenie * Manipul[port].SERVOTIME);

	Manipul[port].zadanoe_polojenie = 0;		 //Задаем положение при запуске

	set_time_ManipulServo(port, Manipul[port].zadanoe_polojenie, 2);	 //Команда встать в заданное начальное положение за нужное время в лупе

//	set_ManipulServo(port, Manipul[port].tek_polojenie);   // 	Встаем в начальное положение  с максимальной скоростью

		 // Моторчик  4 ступени
	port = 4;
	Manipul[port].port = port;
	Manipul[port].ugol = 270;
	Manipul[port].min_impuls = 500;
	Manipul[port].max_impuls = Manipul[port].min_impuls + 2000;
	Manipul[port].tek_polojenie = Manipul[port].zadanoe_polojenie = -90;		 //Задаем положение при запуске

	Manipul[port].SERVOMIN = Manipul[port].min_impuls / ((float)1000000 / SERVO_FREQ / 4096);	//   Serial.print (" SERVOMIN= ");	Serial.print(Manipul[0].SERVOMIN);
	Manipul[port].SERVOMAX = Manipul[port].max_impuls / ((float)1000000 / SERVO_FREQ / 4096);	 //  Serial.print (" SERVOMAX= ");	Serial.print(Manipul[0].SERVOMAX);
	Manipul[port].SERVOTIME = (Manipul[port].SERVOMAX - Manipul[port].SERVOMIN) / (float)Manipul[port].ugol;	 // Serial.print (" SERVOTIME= ");	Serial.print(Manipul[0].SERVOTIME,4);
	Manipul[port].SERVOSER = Manipul[port].SERVOMIN + (Manipul[port].SERVOTIME * (Manipul[port].ugol / 2)); //  Serial.print (" SERVOSER= ");	Serial.println(Manipul[0].SERVOSER);

	set_ManipulServo(port, Manipul[port].zadanoe_polojenie);   // 	Встаем в начальное положение  с максимальной скоростью

	delay(2000);
}

void Loop_Manipul()
{
	for (byte i = 0; i < col_mot; i++)
	{
		if (Manipul[i].SERVOTEK != Manipul[i].SERVOZAD)	   //Если положение не совпадает, тогда
		{
			if (micros()- Manipul[i].time_izmen > Manipul[i].time_pulse)			 // если время с прошлого изменения угла больше чем время шага
			{
				Manipul[i].time_izmen = micros();					 //Запоминаем время изменения угла
				//Serial.print(" time= ");Serial.print(i); Serial.print("  ");	Serial.println(Manipul[i].time_izmen);
				if (Manipul[i].SERVOZAD > Manipul[i].SERVOTEK)
				{
					Manipul[i].SERVOTEK++;      // увеличиваем на 1 градус
				}
				else
				{
					Manipul[i].SERVOTEK--;      // уменьшаем на 1 градус
				}
				pwm.setPWM(i, 0, Manipul[i].SERVOTEK);			  // даем команду  на поворот на 1 импульс
				Manipul[i].tek_polojenie = (Manipul[i].SERVOSER - Manipul[i].SERVOTEK) / Manipul[i].SERVOTIME;	//вычисляем положение в градусах
				float zn = analogRead(Manipul[i].pin);
				Serial.print("port");Serial.print(i);Serial.print(" = "); Serial.print(zn);    // Аналоговый вход для считывания положения сервомотора
				float zd = map(zn, 50, 600, 135, -135);
				Serial.print(" Angle = "); Serial.print(zd);    // Аналоговый вход для считывания положения сервомотора

				Serial.print(" tek_polojenie= ");	Serial.println(Manipul[i].tek_polojenie);

			}
		}
	}
}
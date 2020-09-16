//------------------------------- 5 таймер для СЕРВОМОТОРОВ --------------------------------------------
struct Struct_Servo
{
	byte pinId;					// Номер пина через который управляем моторчиком
	bool active;               // флаг используется или нет
	int angle_position;			   // текущая позиция в градусах
	int pulse_position;			   // позиция в длине импульса
	int angle_min, angle_max;	 // ограничения по вращению угол в градусах
	int pulse_min, pulse_max;	 // пределы импульса, можно подбирать индивидуально под каждый серво-моторчик
	float max_angle;               // максимальный угол возможного поворота моторчика в градусах 180 или 270 		
	bool flag_pulse = false; 	 // Флаг подачи импульса
	int napravlenie_povorota;    // Направление куда вращать мотор 1 или -1
};
const byte num_motor = 8;               // ЧИсло серво-моторов под управлением на одном прерывании
Struct_Servo Servo_Array[num_motor];		// Массив с данными сервомоторчиков

static void Timer3_Init(void)     // Таймер на по совпадению А
{
	TCCR3A = 0;
	TCCR3B = 0;
	TCCR3B |= (1 << WGM32);                    // Режим CTC (сброс по совпадению) 
   // TCCR3B |= (1<<CS30);                       // Тактирование от CLK. 
															  // Если нужен предделитель : 
	TCCR3B |= (1 << CS31);                   // CLK/8 		   // Каждые 0,5 микросекунды
   //TCCR3B |= (1 << CS30) | (1 << CS31); // CLK/64 //          Через 4 микросекунды считает 1/(16 000 000/64)
  // TCCR3B |= (1<<CS32);                   // CLK/256 
  // TCCR3B |= (1<<CS30)|(1<<CS32); // CLK/1024 
														   // Верхняя граница счета. Диапазон от 0 до 65535. 
	OCR3A = 2500 * 2;    // 1 раз в  милисекунд                           // Частота прерываний A будет = Fclk/(N*(1+OCR5A))  На 1 меньше так как считает от 0
  //  OCR3B = 15624;                                // Частота прерываний B будет = Fclk/(N*(1+OCR5B)) 
															 // где N - коэф. предделителя (1, 8, 64, 256 или 1024) 
	TIMSK3 = (1 << OCIE3A);                   // Разрешить прерывание по совпадению A 
   // TIMSK5 |= (1<<OCIE5B);                   // Разрешить прерывание по совпадению B 
  //  TIMSK5 |= (1<<TOIE5);                     // Разрешить прерывание по переполнению 
}


ISR(TIMER3_COMPA_vect)    // Обработчик прерывания таймера 3 по совпадению A   //Время выполнения 10 или 20 микросекунд в зависимости от того нулевой моторчик включаем или ищем активный и потом включаем-выключаем    
{
	//digitalWrite(47, 1);
	//long a = micros();
	static byte number_motor_pulse = 0;			  // Номер моторчика которому передаем импулс
	static long time_all = 0;						  // Общее время работы за цикл для всех моторчиков

	if (Servo_Array[number_motor_pulse].active == true)				 // Если моторчик активный
	{
		if (Servo_Array[number_motor_pulse].flag_pulse == false)
		{
			Servo_Array[number_motor_pulse].flag_pulse = true;					// меняем статус
			OCR3A = Servo_Array[number_motor_pulse].pulse_position * 2;			// Считаем сколько тактов до следующего прерывания 0,5 микросекунды на такт
 			digitalWrite(Servo_Array[number_motor_pulse].pinId, HIGH);	   // Включаем импульс
			digitalWrite(48, 1);
			//Serial.print(" OCR13=1= ");	Serial.println(OCR3A);
		}
		else
		{
			digitalWrite(Servo_Array[number_motor_pulse].pinId, LOW);	   // Выключаем импульс
			digitalWrite(48, 0);
			Servo_Array[number_motor_pulse].flag_pulse = false;				// меняем статус
			time_all = time_all + Servo_Array[number_motor_pulse].pulse_position;     // Запоминаем время на импульс на этом моторчике

			int temp_num_motor = number_motor_pulse;  // номер текущего моторчика для сравнения

			for (byte i = number_motor_pulse + 1; i < num_motor; i++)
			{
				//Serial.print(",");	Serial.println(Servo_Array[i].active);
				if (Servo_Array[i].active == true)   // ищем активный мотор
				{
					//Serial.print(" number_motor_pulse ");	Serial.println(i);
					number_motor_pulse = i;  //   как только нашли запоминаем его номер и прерываем поиск
					Servo_Array[number_motor_pulse].flag_pulse = true;					// меняем статус
					OCR3A = Servo_Array[number_motor_pulse].pulse_position * 2;			// Считаем сколько тактов до следующего прерывания 0,5 микросекунды на такт
					digitalWrite(Servo_Array[number_motor_pulse].pinId, HIGH);	   // Включаем импульс	
					digitalWrite(48, 1);
					break;
				}
			}
			if (number_motor_pulse == temp_num_motor)       // Если после пеербора не поменялся номер моторчика значит мы не нашли больше активных
			{
				number_motor_pulse = 0;    // если до конца массива не нашли активных то начинаем с нулевого
				OCR3A = (22000 - time_all) * 2;		   // Определяем сколько ждать до начала следующего цикла	 Тут задается частота работы 50 герц или ниже или выше у меня 45 Гц така как 22 милисекунды на 9 моторов
				time_all = 0;						   // обнулем время общее
				//Serial.print(" OCR3A-2 ");	Serial.print(OCR3A/2);
				//Serial.print(" time_all ");	Serial.println(time_all);
			}
			//Serial.print(" =");	Serial.println(OCR3A);
		}
	}
	//long b = micros();
	//Serial.print(" Time TIMER3_COMPA_vect : "); Serial.println(b - a);

	//digitalWrite(47, 0);
}

long Get_Pulse_from_Gradus(byte number_servo, int gradus)
{
	float koef = (Servo_Array[number_servo].pulse_max - Servo_Array[number_servo].pulse_min) / Servo_Array[number_servo].max_angle;
	long pulse_servo = Servo_Array[number_servo].pulse_min + (gradus * koef);
	if (pulse_servo > Servo_Array[number_servo].pulse_max) pulse_servo = Servo_Array[number_servo].pulse_max;
	if (pulse_servo < Servo_Array[number_servo].pulse_min) pulse_servo = Servo_Array[number_servo].pulse_min;
	
	//Serial.print(" Get_Pulse_from_Gradus pulse_servo= ");	  	Serial.println(pulse_servo);
	return pulse_servo;

}

//void ServoMotor_Stop(int num_)
//{
//	Servo_Array[num_].active = false;								// Отключаем моторчик
//
//}
//void ServoMotor_Start(int num_)
//{
//	Servo_Array[num_].active = true;								// Включаем моторчик
//
//}
void InitServoMotor(byte id_servo, int pinId, int angle_position, int angle_min, int angle_max, int pulse_min, int pulse_max, int max_angle)
{
	pinMode(pinId, OUTPUT);                                             // Пин для серво мотора определяем на выход
	Servo_Array[id_servo].pinId = pinId;
	Servo_Array[id_servo].active = true;								// Активизируем моторчик
	Servo_Array[id_servo].napravlenie_povorota = 1;								// Прибавляем градусы при вращении
	Servo_Array[id_servo].angle_min = angle_min;
	Servo_Array[id_servo].angle_max = angle_max;
	Servo_Array[id_servo].pulse_min = pulse_min;
	Servo_Array[id_servo].pulse_max = pulse_max;	
	Servo_Array[id_servo].max_angle = max_angle;
	Servo_Array[id_servo].angle_position = angle_position;
	Servo_Array[id_servo].pulse_position = Get_Pulse_from_Gradus(id_servo, angle_position);				// Устанавливаем начальную позицию
	//Serial.print(" Get_Pulse_from_Gradus : ");	  	Serial.println(Servo_Array[id_servo].pulse_position);

}

void Set_Angle_ServoMotor(byte id_servo, int angle_position)	  // время выполнения 72 микросекунды
{
	if (angle_position > Servo_Array[id_servo].angle_max) angle_position = Servo_Array[id_servo].angle_max;
	if (angle_position < Servo_Array[id_servo].angle_min) angle_position = Servo_Array[id_servo].angle_min;
	Servo_Array[id_servo].angle_position = angle_position;												  // Устанавливаем  позицию  в градусах чтобы знать куда ходим переместиться
	Servo_Array[id_servo].pulse_position = Get_Pulse_from_Gradus(id_servo, angle_position);				// Устанавливаем  позицию

	//Serial.print(" Servo_Array[id_servo].pulse_position= ");	  	Serial.println(Servo_Array[id_servo].pulse_position);
}


void ServoMotor_Init()       // Начальная установка параметров шагового мотора
{
	Serial.println(" InitServoMotor...");
	Serial2.println(" InitServoMotor...");


	for (byte i = 0; i < 9; i++)
	{										     
		Servo_Array[i].active = false;								// отключаем все моторы	для правильного перебора при поиске активных
	}
														     // Устанавливаем 0 мотор на 22 пине с начальным положением в 90 градусов с пределами вращения от 0 до 180 и импульсами управления от 544 до 2400 и сам мотор 180 градусный

	InitServoMotor(0, 24, 0, 0, 180, 500, 2500, 180);		 //Передний  левый начальное положение
	InitServoMotor(1, 22, 0, 0, 180, 500, 2500, 180);		   //Передний правый    начальное положение

	InitServoMotor(2, 28, 130, 0, 180, 500, 2500, 180);		 //Передний левый начальное положение	    //Силовые
	InitServoMotor(3, 26, 50, 0, 180, 500, 2500, 180);		 //Передний правый	начальное положение		  //Силовые

	InitServoMotor(4, 32, 0, 0, 180, 500, 2500, 180);		 //Задний правый начальное положение	
	InitServoMotor(5, 34, 0, 0, 180, 500, 2500, 180);		 //Задний левый начальное положение	
	
	InitServoMotor(6, 36, 60, 0, 180, 500, 2500, 180);		 //Задний правый начальное положение	    //Силовые
	InitServoMotor(7, 38, 120, 0, 180, 500, 2500, 180);		 //Задний левый начальное положение		    //Силовые
	
	delay(2000);

	Set_Angle_ServoMotor(0, 180);
	Set_Angle_ServoMotor(1, 180);
	Set_Angle_ServoMotor(2, 0);					 //Силовые
	Set_Angle_ServoMotor(3, 180);				  //Силовые
	Set_Angle_ServoMotor(4, 180);
	Set_Angle_ServoMotor(5, 180);
	Set_Angle_ServoMotor(6, 180);				   //Силовые
	Set_Angle_ServoMotor(7, 0);					    //Силовые
	
	delay(2000);						//Для серво с датчиками начальное положение
	Set_Angle_ServoMotor(0, 45);
	Set_Angle_ServoMotor(1, 135);
	Set_Angle_ServoMotor(4, 135);
	Set_Angle_ServoMotor(5, 45);

	Set_Angle_ServoMotor(2, 130);	    //Для силовых серво моторов опускание вниз
	Set_Angle_ServoMotor(3, 50);
	Set_Angle_ServoMotor(6, 50);
	Set_Angle_ServoMotor(7, 130);
	delay(2000);


	for (byte i = 0; i < 9; i++)
	{
		//Servo_Array[i].active = false;								// отключаем все моторы	
	}


	Serial.println(" END InitServoMotor");
	Serial2.println(" END InitServoMotor");


	//delay(1000);



	//TIMSK3 = 0;   //выключаем таймер

}


void Change_angle_servo(int id_servo, int delta)
{
	int temp_angle = Servo_Array[id_servo].angle_position + ( delta * Servo_Array[id_servo].napravlenie_povorota );   // меняем угол на величину
	if (temp_angle >= Servo_Array[id_servo].angle_max)
	{
		temp_angle = Servo_Array[id_servo].angle_max; //  устанавливаем предельный угол
		Servo_Array[id_servo].napravlenie_povorota = -1;								  //  меняем напраление движения
	}
	if (temp_angle <= Servo_Array[id_servo].angle_min)
	{
		temp_angle = Servo_Array[id_servo].angle_min; //  устанавливаем предельный угол
		Servo_Array[id_servo].napravlenie_povorota = 1;								  //  меняем напраление движения
	}
	Servo_Array[id_servo].angle_position = temp_angle; // Запоминаем новый угол	 в который будем поворачиваться
	//Serial.print(" Servo angle_position= ");	  	Serial.println(temp_angle);
	//Serial.print(" millis= ");	  	Serial.println(millis());
	Set_Angle_ServoMotor(id_servo, temp_angle);		  // Передаем угол для поворота
}

void Change_angle_servo_diapazon(int id_servo, int delta, int min_, int max_)
{
	int temp_angle = Servo_Array[id_servo].angle_position + (delta * Servo_Array[id_servo].napravlenie_povorota);   // меняем угол на величину
	if (temp_angle >= max_)
	{
		temp_angle = max_; //  устанавливаем предельный угол
		Servo_Array[id_servo].napravlenie_povorota = -1;								  //  меняем напраление движения
	}
	if (temp_angle <= min_)
	{
		temp_angle = min_; //  устанавливаем предельный угол
		Servo_Array[id_servo].napravlenie_povorota = 1;								  //  меняем напраление движения
	}
	//Servo_Array[id_servo].angle_position = temp_angle; // Запоминаем новый угол
	//Serial.print(" Servo angle_position= ");	  	Serial.println(temp_angle);
	//Serial.print(" millis= ");	  	Serial.println(millis());
	Set_Angle_ServoMotor(id_servo, temp_angle);		  // Передаем угол для поворота
}


void Run_servo()	// Функция запускает поворот серво моторчиков на нужный угол
{
	//Serial.print(" Start Run_servo : ");  Serial.println(millis());
	Change_angle_servo_diapazon(0, 5, 15, 75);		//  ВРЕМЯ ИСПОЛНЕНИЯ 100 микросекунд
	Change_angle_servo_diapazon(1, 5, 105, 165);
	Change_angle_servo_diapazon(4, 5, 105, 165);
	Change_angle_servo_diapazon(5, 5, 15, 75);		//  ВРЕМЯ ИСПОЛНЕНИЯ 100 микросекунд

	//Serial.print(" millis= ");  Serial.println(millis());
}

void PodemServo()
{
	if (millis() - time_servoUp > 10000 && flag_Platforma == false)		   //Поднимаем серво моторы
{
	Serial.println("Platfotma_UP.");
	flag_Platforma = true;
	Set_Angle_ServoMotor(2, 0);
	Set_Angle_ServoMotor(3, 180);
	Set_Angle_ServoMotor(6, 180);
	Set_Angle_ServoMotor(7, 0);		
	
	time_servoUp = millis();	 //Время когда подняли платформу
}
if (millis() - time_servoUp > 2000 && flag_Platforma == true)      // Если платформа поднята уже 2 секунжы то опускаем
{
	Serial.println("Platfotma_DOWN.");
	flag_Platforma = false;
	Set_Angle_ServoMotor(2, 120);
	Set_Angle_ServoMotor(3, 60);
	Set_Angle_ServoMotor(6, 60);
	Set_Angle_ServoMotor(7, 120);
	time_servoUp = millis();	 //Время когда опустили платформу
}

}

void Loop_Servo()
{
	//==========================================================
	if (flag_servo == true)
	{
		flag_servo = false;
		Run_servo();  				  // ВРЕМЯ ИСПОЛНЕНИЯ 0.2 МИЛИСЕКУНДА  измеряно для 2 моторчиков, потом перемерить
		//Serial.print(" Run_servo --- : ");
	}
}
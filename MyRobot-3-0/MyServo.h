//------------------------------- 5 ������ ��� ������������ --------------------------------------------
struct Struct_Servo
{
	byte pinId;					// ����� ���� ����� ������� ��������� ����������
	bool active;               // ���� ������������ ��� ���
	int angle_position;			   // ������� ������� � ��������
	int pulse_position;			   // ������� � ����� ��������
	int angle_min, angle_max;	 // ����������� �� �������� ���� � ��������
	int pulse_min, pulse_max;	 // ������� ��������, ����� ��������� ������������� ��� ������ �����-��������
	float max_angle;               // ������������ ���� ���������� �������� ��������� � �������� 180 ��� 270 		
	bool flag_pulse = false; 	 // ���� ������ ��������
	int napravlenie_povorota;    // ����������� ���� ������� ����� 1 ��� -1
};
const byte num_motor = 8;               // ����� �����-������� ��� ����������� �� ����� ����������
Struct_Servo Servo_Array[num_motor];		// ������ � ������� ���������������

static void Timer3_Init(void)     // ������ �� �� ���������� �
{
	TCCR3A = 0;
	TCCR3B = 0;
	TCCR3B |= (1 << WGM32);                    // ����� CTC (����� �� ����������) 
   // TCCR3B |= (1<<CS30);                       // ������������ �� CLK. 
															  // ���� ����� ������������ : 
	TCCR3B |= (1 << CS31);                   // CLK/8 		   // ������ 0,5 ������������
   //TCCR3B |= (1 << CS30) | (1 << CS31); // CLK/64 //          ����� 4 ������������ ������� 1/(16 000 000/64)
  // TCCR3B |= (1<<CS32);                   // CLK/256 
  // TCCR3B |= (1<<CS30)|(1<<CS32); // CLK/1024 
														   // ������� ������� �����. �������� �� 0 �� 65535. 
	OCR3A = 2500 * 2;    // 1 ��� �  ����������                           // ������� ���������� A ����� = Fclk/(N*(1+OCR5A))  �� 1 ������ ��� ��� ������� �� 0
  //  OCR3B = 15624;                                // ������� ���������� B ����� = Fclk/(N*(1+OCR5B)) 
															 // ��� N - ����. ������������ (1, 8, 64, 256 ��� 1024) 
	TIMSK3 = (1 << OCIE3A);                   // ��������� ���������� �� ���������� A 
   // TIMSK5 |= (1<<OCIE5B);                   // ��������� ���������� �� ���������� B 
  //  TIMSK5 |= (1<<TOIE5);                     // ��������� ���������� �� ������������ 
}


ISR(TIMER3_COMPA_vect)    // ���������� ���������� ������� 3 �� ���������� A   //����� ���������� 10 ��� 20 ����������� � ����������� �� ���� ������� �������� �������� ��� ���� �������� � ����� ��������-���������    
{
	//digitalWrite(47, 1);
	//long a = micros();
	static byte number_motor_pulse = 0;			  // ����� ��������� �������� �������� ������
	static long time_all = 0;						  // ����� ����� ������ �� ���� ��� ���� ����������

	if (Servo_Array[number_motor_pulse].active == true)				 // ���� �������� ��������
	{
		if (Servo_Array[number_motor_pulse].flag_pulse == false)
		{
			Servo_Array[number_motor_pulse].flag_pulse = true;					// ������ ������
			OCR3A = Servo_Array[number_motor_pulse].pulse_position * 2;			// ������� ������� ������ �� ���������� ���������� 0,5 ������������ �� ����
 			digitalWrite(Servo_Array[number_motor_pulse].pinId, HIGH);	   // �������� �������
			digitalWrite(48, 1);
			//Serial.print(" OCR13=1= ");	Serial.println(OCR3A);
		}
		else
		{
			digitalWrite(Servo_Array[number_motor_pulse].pinId, LOW);	   // ��������� �������
			digitalWrite(48, 0);
			Servo_Array[number_motor_pulse].flag_pulse = false;				// ������ ������
			time_all = time_all + Servo_Array[number_motor_pulse].pulse_position;     // ���������� ����� �� ������� �� ���� ���������

			int temp_num_motor = number_motor_pulse;  // ����� �������� ��������� ��� ���������

			for (byte i = number_motor_pulse + 1; i < num_motor; i++)
			{
				//Serial.print(",");	Serial.println(Servo_Array[i].active);
				if (Servo_Array[i].active == true)   // ���� �������� �����
				{
					//Serial.print(" number_motor_pulse ");	Serial.println(i);
					number_motor_pulse = i;  //   ��� ������ ����� ���������� ��� ����� � ��������� �����
					Servo_Array[number_motor_pulse].flag_pulse = true;					// ������ ������
					OCR3A = Servo_Array[number_motor_pulse].pulse_position * 2;			// ������� ������� ������ �� ���������� ���������� 0,5 ������������ �� ����
					digitalWrite(Servo_Array[number_motor_pulse].pinId, HIGH);	   // �������� �������	
					digitalWrite(48, 1);
					break;
				}
			}
			if (number_motor_pulse == temp_num_motor)       // ���� ����� �������� �� ��������� ����� ��������� ������ �� �� ����� ������ ��������
			{
				number_motor_pulse = 0;    // ���� �� ����� ������� �� ����� �������� �� �������� � ��������
				OCR3A = (22000 - time_all) * 2;		   // ���������� ������� ����� �� ������ ���������� �����	 ��� �������� ������� ������ 50 ���� ��� ���� ��� ���� � ���� 45 �� ���� ��� 22 ����������� �� 9 �������
				time_all = 0;						   // ������� ����� �����
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
//	Servo_Array[num_].active = false;								// ��������� ��������
//
//}
//void ServoMotor_Start(int num_)
//{
//	Servo_Array[num_].active = true;								// �������� ��������
//
//}
void InitServoMotor(byte id_servo, int pinId, int angle_position, int angle_min, int angle_max, int pulse_min, int pulse_max, int max_angle)
{
	pinMode(pinId, OUTPUT);                                             // ��� ��� ����� ������ ���������� �� �����
	Servo_Array[id_servo].pinId = pinId;
	Servo_Array[id_servo].active = true;								// ������������ ��������
	Servo_Array[id_servo].napravlenie_povorota = 1;								// ���������� ������� ��� ��������
	Servo_Array[id_servo].angle_min = angle_min;
	Servo_Array[id_servo].angle_max = angle_max;
	Servo_Array[id_servo].pulse_min = pulse_min;
	Servo_Array[id_servo].pulse_max = pulse_max;	
	Servo_Array[id_servo].max_angle = max_angle;
	Servo_Array[id_servo].angle_position = angle_position;
	Servo_Array[id_servo].pulse_position = Get_Pulse_from_Gradus(id_servo, angle_position);				// ������������� ��������� �������
	//Serial.print(" Get_Pulse_from_Gradus : ");	  	Serial.println(Servo_Array[id_servo].pulse_position);

}

void Set_Angle_ServoMotor(byte id_servo, int angle_position)	  // ����� ���������� 72 ������������
{
	if (angle_position > Servo_Array[id_servo].angle_max) angle_position = Servo_Array[id_servo].angle_max;
	if (angle_position < Servo_Array[id_servo].angle_min) angle_position = Servo_Array[id_servo].angle_min;
	Servo_Array[id_servo].angle_position = angle_position;												  // �������������  �������  � �������� ����� ����� ���� ����� �������������
	Servo_Array[id_servo].pulse_position = Get_Pulse_from_Gradus(id_servo, angle_position);				// �������������  �������

	//Serial.print(" Servo_Array[id_servo].pulse_position= ");	  	Serial.println(Servo_Array[id_servo].pulse_position);
}


void ServoMotor_Init()       // ��������� ��������� ���������� �������� ������
{
	Serial.println(" InitServoMotor...");
	Serial2.println(" InitServoMotor...");


	for (byte i = 0; i < 9; i++)
	{										     
		Servo_Array[i].active = false;								// ��������� ��� ������	��� ����������� �������� ��� ������ ��������
	}
														     // ������������� 0 ����� �� 22 ���� � ��������� ���������� � 90 �������� � ��������� �������� �� 0 �� 180 � ���������� ���������� �� 544 �� 2400 � ��� ����� 180 ���������

	InitServoMotor(0, 24, 0, 0, 180, 500, 2500, 180);		 //��������  ����� ��������� ���������
	InitServoMotor(1, 22, 0, 0, 180, 500, 2500, 180);		   //�������� ������    ��������� ���������

	InitServoMotor(2, 28, 130, 0, 180, 500, 2500, 180);		 //�������� ����� ��������� ���������	    //�������
	InitServoMotor(3, 26, 50, 0, 180, 500, 2500, 180);		 //�������� ������	��������� ���������		  //�������

	InitServoMotor(4, 32, 0, 0, 180, 500, 2500, 180);		 //������ ������ ��������� ���������	
	InitServoMotor(5, 34, 0, 0, 180, 500, 2500, 180);		 //������ ����� ��������� ���������	
	
	InitServoMotor(6, 36, 60, 0, 180, 500, 2500, 180);		 //������ ������ ��������� ���������	    //�������
	InitServoMotor(7, 38, 120, 0, 180, 500, 2500, 180);		 //������ ����� ��������� ���������		    //�������
	
	delay(2000);

	Set_Angle_ServoMotor(0, 180);
	Set_Angle_ServoMotor(1, 180);
	Set_Angle_ServoMotor(2, 0);					 //�������
	Set_Angle_ServoMotor(3, 180);				  //�������
	Set_Angle_ServoMotor(4, 180);
	Set_Angle_ServoMotor(5, 180);
	Set_Angle_ServoMotor(6, 180);				   //�������
	Set_Angle_ServoMotor(7, 0);					    //�������
	
	delay(2000);						//��� ����� � ��������� ��������� ���������
	Set_Angle_ServoMotor(0, 45);
	Set_Angle_ServoMotor(1, 135);
	Set_Angle_ServoMotor(4, 135);
	Set_Angle_ServoMotor(5, 45);

	Set_Angle_ServoMotor(2, 130);	    //��� ������� ����� ������� ��������� ����
	Set_Angle_ServoMotor(3, 50);
	Set_Angle_ServoMotor(6, 50);
	Set_Angle_ServoMotor(7, 130);
	delay(2000);


	for (byte i = 0; i < 9; i++)
	{
		//Servo_Array[i].active = false;								// ��������� ��� ������	
	}


	Serial.println(" END InitServoMotor");
	Serial2.println(" END InitServoMotor");


	//delay(1000);



	//TIMSK3 = 0;   //��������� ������

}


void Change_angle_servo(int id_servo, int delta)
{
	int temp_angle = Servo_Array[id_servo].angle_position + ( delta * Servo_Array[id_servo].napravlenie_povorota );   // ������ ���� �� ��������
	if (temp_angle >= Servo_Array[id_servo].angle_max)
	{
		temp_angle = Servo_Array[id_servo].angle_max; //  ������������� ���������� ����
		Servo_Array[id_servo].napravlenie_povorota = -1;								  //  ������ ���������� ��������
	}
	if (temp_angle <= Servo_Array[id_servo].angle_min)
	{
		temp_angle = Servo_Array[id_servo].angle_min; //  ������������� ���������� ����
		Servo_Array[id_servo].napravlenie_povorota = 1;								  //  ������ ���������� ��������
	}
	Servo_Array[id_servo].angle_position = temp_angle; // ���������� ����� ����	 � ������� ����� ��������������
	//Serial.print(" Servo angle_position= ");	  	Serial.println(temp_angle);
	//Serial.print(" millis= ");	  	Serial.println(millis());
	Set_Angle_ServoMotor(id_servo, temp_angle);		  // �������� ���� ��� ��������
}

void Change_angle_servo_diapazon(int id_servo, int delta, int min_, int max_)
{
	int temp_angle = Servo_Array[id_servo].angle_position + (delta * Servo_Array[id_servo].napravlenie_povorota);   // ������ ���� �� ��������
	if (temp_angle >= max_)
	{
		temp_angle = max_; //  ������������� ���������� ����
		Servo_Array[id_servo].napravlenie_povorota = -1;								  //  ������ ���������� ��������
	}
	if (temp_angle <= min_)
	{
		temp_angle = min_; //  ������������� ���������� ����
		Servo_Array[id_servo].napravlenie_povorota = 1;								  //  ������ ���������� ��������
	}
	//Servo_Array[id_servo].angle_position = temp_angle; // ���������� ����� ����
	//Serial.print(" Servo angle_position= ");	  	Serial.println(temp_angle);
	//Serial.print(" millis= ");	  	Serial.println(millis());
	Set_Angle_ServoMotor(id_servo, temp_angle);		  // �������� ���� ��� ��������
}


void Run_servo()	// ������� ��������� ������� ����� ���������� �� ������ ����
{
	//Serial.print(" Start Run_servo : ");  Serial.println(millis());
	Change_angle_servo_diapazon(0, 5, 15, 75);		//  ����� ���������� 100 �����������
	Change_angle_servo_diapazon(1, 5, 105, 165);
	Change_angle_servo_diapazon(4, 5, 105, 165);
	Change_angle_servo_diapazon(5, 5, 15, 75);		//  ����� ���������� 100 �����������

	//Serial.print(" millis= ");  Serial.println(millis());
}

void PodemServo()
{
	if (millis() - time_servoUp > 10000 && flag_Platforma == false)		   //��������� ����� ������
{
	Serial.println("Platfotma_UP.");
	flag_Platforma = true;
	Set_Angle_ServoMotor(2, 0);
	Set_Angle_ServoMotor(3, 180);
	Set_Angle_ServoMotor(6, 180);
	Set_Angle_ServoMotor(7, 0);		
	
	time_servoUp = millis();	 //����� ����� ������� ���������
}
if (millis() - time_servoUp > 2000 && flag_Platforma == true)      // ���� ��������� ������� ��� 2 ������� �� ��������
{
	Serial.println("Platfotma_DOWN.");
	flag_Platforma = false;
	Set_Angle_ServoMotor(2, 120);
	Set_Angle_ServoMotor(3, 60);
	Set_Angle_ServoMotor(6, 60);
	Set_Angle_ServoMotor(7, 120);
	time_servoUp = millis();	 //����� ����� �������� ���������
}

}

void Loop_Servo()
{
	//==========================================================
	if (flag_servo == true)
	{
		flag_servo = false;
		Run_servo();  				  // ����� ���������� 0.2 �����������  �������� ��� 2 ����������, ����� ����������
		//Serial.print(" Run_servo --- : ");
	}
}
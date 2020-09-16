

int Pin_MotorL1 = 10;   int Pin_MotorL2 = 11;	 // ���� �� ������� ���������� ���������
int Pin_MotorR1 = 9;  int Pin_MotorR2 = 12;
int Pin_Encoder_L_A = A8; int Pin_Encoder_L_B = A9;
int Pin_Encoder_R_A = A10; int Pin_Encoder_R_B = A11;

//int minPWM = 15;
float minSpeed = 0.05;

void MotorInit()       // ��������� ��������� ����������� �������
{
	pinMode(Pin_MotorL1, OUTPUT); 	analogWrite(Pin_MotorL1, 0);   //���� ��� ���������� ��������� � ����� ������ � ���������
	pinMode(Pin_MotorL2, OUTPUT);	analogWrite(Pin_MotorL2, 0);

   //���������� ������ 5 ����� �� ������� ������ �� ������ ���� ���  R_EN L_EN
	pinMode(23, OUTPUT);   digitalWrite(23, 1);
	//pinMode(35, OUTPUT);   analogWrite(Pin_MotorR2, 1);	  // ��� ������ � �������� ����� � ��� ������ 5 ����� � ������ ������ �����
	//------------------------------------------------

	pinMode(Pin_MotorR1, OUTPUT);  	analogWrite(Pin_MotorR1, 0);
	pinMode(Pin_MotorR2, OUTPUT);	analogWrite(Pin_MotorR2, 0);

	//���������� ������ 5 ����� �� ������� ������ �� ������ ���� ���  R_EN L_EN

	pinMode(33, OUTPUT);   digitalWrite(33, 1);
	pinMode(35, OUTPUT);   digitalWrite(35, 1);

	//---------------------------------------------------

	pinMode(Pin_Encoder_L_A, INPUT);  	digitalWrite(Pin_Encoder_L_A, HIGH);  // �������� ������������� ��������
	pinMode(Pin_Encoder_L_B, INPUT);  	digitalWrite(Pin_Encoder_L_B, HIGH);  // �������� ������������� ��������

	pinMode(Pin_Encoder_R_A, INPUT);  	digitalWrite(Pin_Encoder_R_A, HIGH);  // �������� ������������� ��������
	pinMode(Pin_Encoder_R_B, INPUT);  	digitalWrite(Pin_Encoder_R_B, HIGH);  // �������� ������������� ��������

	
	// ��� ��������
	// ��������� ���������� � ����� � �������:
	/*� Bit 2 � PCIE2 : Pin Change Interrupt Enable 1
		When the PCIE2 bit is set(one) and the I - bit in the Status Register(SREG) is set(one), pin
		change interrupt 2 is enabled.Any change on any enabled PCINT23 : 16 pin will cause an interrupt.
		The corresponding interrupt of Pin Change Interrupt Request is executed from the PCI2
		Interrupt Vector.PCINT23 : 16 pins are enabled individually by the PCMSK2 Register.		 */	 
		//��� 0 � �������� PCICR �������� �� ������ 0, ��� 1 �� ������ 1, ��� 2 �� ������ 2.
	PCICR = 0b00000100;  		  // �������� ������ PCIE2 �� 2 ����


  // ������ �������� ���������� �� � ����� ������, ������ ������ � �������� PCMSK0...PCMSK2:
																	  
	/*PCMSK2 � Pin Change Mask Register 2
		� Bit 7:0 � PCINT23 : 16 : Pin Change Enable Mask 23 : 16
		Each PCINT23 : 16 - bit selects whether pin change interrupt is enabled on the corresponding I / O
		pin.If PCINT23 : 16 is set and the PCIE2 bit in PCICR is set, pin change interrupt is enabled on
		the corresponding I / O pin.If PCINT23 : 16 is cleared, pin change interrupt on the corresponding
		I / O pin is disabled.			*/
	PCMSK2 = 0b00001111;	   // ��� ��������� �� ����� PK0 PK1 PK2 PK3


	//Serial.print("-End MotorInit- "); 
}

void Run_MotorL(int PWM)       // �������� ������ ������ � �������� �����	  // ����� ���������� 50 �����������
{
	if (PWM == 0)
	{ 
		analogWrite(Pin_MotorL1, 0);
		analogWrite(Pin_MotorL2, 0); 
	}
	if (PWM > 0)
	{ 
		if (PWM > 255) PWM = 255;
		if (PWM < 15) PWM = 15;
		analogWrite(Pin_MotorL1, 0);
		analogWrite(Pin_MotorL2, PWM);
	}	
	if (PWM < 0)
	{
		if (PWM < -255) PWM = -255;
		if (PWM > -15) PWM = -15;
		analogWrite(Pin_MotorL1, -PWM);
		analogWrite(Pin_MotorL2, 0);
	}
	MotorL.motor_pwm = PWM;  //���������� ����� ��� ���������� �� �����
}

void Run_MotorR(int PWM)       // �������� ������� ������ � �������� �����
{
	if (PWM == 0)
	{
		analogWrite(Pin_MotorR1, 0);
		analogWrite(Pin_MotorR2, 0);
	}
	if (PWM > 0)
	{
		if (PWM > 255) PWM = 255;
		if (PWM < 15) PWM = 15;
		analogWrite(Pin_MotorR1, 0);
		analogWrite(Pin_MotorR2, PWM);
	}
	if (PWM < 0)
	{
		if (PWM < -255) PWM = -255;
		if (PWM > -15) PWM = -15;
		analogWrite(Pin_MotorR1, -PWM);
		analogWrite(Pin_MotorR2, 0);
	}
	MotorR.motor_pwm = PWM;  //���������� ����� ��� ���������� �� �����
}


volatile byte st_PK0 = 0, st_PK1 = 0, st_PK2 = 0, st_PK3 = 0;		 // ��������� ���� ��� ��������
volatile byte tek_st_PK0 = 0, tek_st_PK1 = 0, tek_st_PK2 = 0, tek_st_PK3 = 0;		 // ��������� ���� ��� ��������
//volatile byte st_C_L = 0, st_C_R = 0;	 //��������� ���������� ��������
//volatile bool tek_st_C_L = 0, tek_st_C_R = 0;	 //��������� ������� ���������� ��������
volatile byte napr_L = 0, napr_R = 0;	 //����������� ��������

//ISR(PCINT2_vect)
//{
//	//PORTJ |= (1 << 1);
//	if (!(PINK & (1 << PK0)) && st_PK0 == 1)   // ���� ��� ���� ����� 0 � � ���������� ������ ��� 1
//	{
//		st_PK0 = 0;
//		MotorR.EncoderPulseA++;
//	}
//	if ((PINK & (1 << PK0)) && st_PK0 == 0)   // ���� ��� ���� ����� 1 � � ���������� ������ ��� 0
//	{
//		st_PK0 = 1;
//		MotorR.EncoderPulseA++;
//	}
//	if (!(PINK & (1 << PK1)) && st_PK1 == 1)   // ���� ��� ���� ����� 0 � � ���������� ������ ��� 1
//	{
//		st_PK1 = 0;
//		MotorR.EncoderPulseB++;
//	}
//	if ((PINK & (1 << PK1)) && st_PK1 == 0)   // ���� ��� ���� ����� 1 � � ���������� ������ ��� 0
//	{
//		st_PK1 = 1;
//		MotorR.EncoderPulseB++;
//	}
//	//--------------------------------------------------------------------------------------------
//	if (!(PINK & (1 << PK2)) && st_PK2 == 1)   // ���� ��� ���� ����� 0 � � ���������� ������ ��� 1
//	{
//		st_PK2 = 0;
//		MotorL.EncoderPulseA++;
//	}
//	if ((PINK & (1 << PK2)) && st_PK2 == 0)   // ���� ��� ���� ����� 1 � � ���������� ������ ��� 0
//	{
//		st_PK2 = 1;
//		MotorL.EncoderPulseA++;
//	}
//	if (!(PINK & (1 << PK3)) && st_PK3 == 1)   // ���� ��� ���� ����� 0 � � ���������� ������ ��� 1
//	{
//		st_PK3 = 0;
//		MotorL.EncoderPulseB++;
//	}
//	if ((PINK & (1 << PK3)) && st_PK3 == 0)   // ���� ��� ���� ����� 1 � � ���������� ������ ��� 0
//	{
//		st_PK3 = 1;
//		MotorL.EncoderPulseB++;
//	}
//	//Serial.print(" LA= ");Serial.print(MotorL.EncoderPulseA);
//	//Serial.print(" LB= ");Serial.print(MotorL.EncoderPulseA);
//	Serial.print(" ");Serial.print(MotorR.EncoderPulseA);
//	Serial.print(" ");Serial.println(MotorR.EncoderPulseA);
//
//	//PORTJ &= ~(1 << 1);
//}

ISR(PCINT2_vect)
{
	//PORTJ |= (1 << 1);
	//long a = micros();
	tek_st_PK0 = PINK & (1 << PK0);		// ����������� �������� ���� 
	if (tek_st_PK0 != st_PK0)
	{
		MotorL.EncoderPulseA++;
		st_PK0 = tek_st_PK0;
		//���� �������� ������ �� ������ � � ��� ���� � �� ����� ����� � ������ ������, ���� ��� � ������ ���������� ����� ������ ���������
		//napr_R = tek_st_PK0 != tek_st_PK1; //- ����� ��� �������� ���� �������. ��� � ������ ��������
		if (bool(tek_st_PK0) != bool(tek_st_PK1)) { napr_R = 1; } // �������� ������ �������
		else { napr_R = 0; } // �������� �� �������
	}
	tek_st_PK1 = PINK & (1 << PK1);		// ����������� �������� ���� 
	if (tek_st_PK1 != st_PK1)
	{
		MotorL.EncoderPulseB++;
		st_PK1 = tek_st_PK1;
		//���� �������� ������ �� ������ � � ��� ���� � �� ����� ����� � ����� ������, ���� ��� � ������ ���������� ����� ����� ���������
		//napr_R = tek_st_PK0 == tek_st_PK1;		 //- ����� ��� �������� ���� �������. ��� � ������ ��������
		if (bool(tek_st_PK1) != bool(tek_st_PK0)) { napr_R = 0; }   // �������� �� �������
		else { napr_R = 1; }   // �������� ������ �������
	}

	//tek_st_C_R = tek_st_PK0 ^ tek_st_PK1;	   // ���������� ���������� ��� � �������� �������� � ��� ���� �����. ��� ����� ���������� �������� �� ������ � � � � ��������� � ������� ��� �������
	//if (tek_st_C_R != st_C_R)
	//{
	//	MotorR.EncoderPulseC++;
	//	st_C_R = tek_st_C_R;
	//}

	//--------------------------------------------------------------------------------------------
	tek_st_PK2 = PINK & (1 << PK2);		// ����������� �������� ���� 
	if (tek_st_PK2 != st_PK2)
	{
		MotorR.EncoderPulseA++;
		st_PK2 = tek_st_PK2;
		//���� �������� ������ �� ������ � � ��� ���� � �� ����� ����� � ������ ������, ���� ��� � ������ ���������� ����� ������ ���������
		//napr_R = tek_st_PK2 != tek_st_PK3; //- ����� ��� �������� ���� �������. ��� � ������ ��������
		if (bool(tek_st_PK2) != bool(tek_st_PK3)) { napr_L = 1; } // �������� ������ �������
		else { napr_L = 0; } // �������� �� �������
	}
	tek_st_PK3 = PINK & (1 << PK3);		// ����������� �������� ���� 
	if (tek_st_PK3 != st_PK3)
	{
		MotorR.EncoderPulseB++;
		st_PK3 = tek_st_PK3;
		//���� �������� ������ �� ������ � � ��� ���� � �� ����� ����� � ����� ������, ���� ��� � ������ ���������� ����� ����� ���������
		//napr_R = tek_st_PK2 == tek_st_PK3;		 //- ����� ��� �������� ���� �������. ��� � ������ ��������
		if (bool(tek_st_PK3) != bool(tek_st_PK2)) { napr_L = 0; }   // �������� �� �������
		else { napr_L = 1; }   // �������� ������ �������
	}

	//tek_st_C_L = tek_st_PK2 ^ tek_st_PK3;	   // ���������� ���������� ��� � �������� �������� � ��� ���� �����. ��� ����� ���������� �������� �� ������ � � � � ��������� � ������� ��� �������
	//if (tek_st_C_L != st_C_L)
	//{
	//	MotorL.EncoderPulseC++;
	//	st_C_L = tek_st_C_L;
	//}
	//long d = micros();
	//Serial.print(" LA= ");Serial.print(MotorL.EncoderPulseA);
	//Serial.print(" LB= ");Serial.print(MotorL.EncoderPulseA);
	//Serial.print(" ");Serial.println(MotorR.EncoderPulseA);
	//Serial.print(" ");Serial.print(MotorR.EncoderPulseB);
	//Serial.print(" ");Serial.print(MotorL.EncoderPulseC);
	//Serial.print(" ");Serial.print(MotorR.EncoderPulseC);

	//Serial.print(" ");Serial.print(napr_L); Serial.print(" ");
	//Serial.print(" ");Serial.print(napr_R); Serial.print(" ");


	//Serial.print(" "); Serial.println(d - a);


	//PORTJ &= ~(1 << 1);
}

void Read_Encoder()
{
	//Serial.print(MotorL.EncoderPulseA);
	//Serial.print(MotorL.EncoderPulseB);
	//Serial.print(MotorR.EncoderPulseA);
	//Serial.print(MotorR.EncoderPulseB);

	//Serial.println(" ");

	static float pre_intervalEncoder = micros();
	now_time = micros();               // ��������� ������� �����
	intervalEncoder = (now_time - pre_intervalEncoder)*0.000001;            // ������� ������� �� ������ � ���������� ����������� � ��������
	if (intervalEncoder < 0.001) intervalEncoder = 0.01;	  // ��� ������� ������� ���� ��� ����������� ��������

	//Serial.print(" intervalEncoder0 ");Serial.println(intervalEncoder);

	pre_intervalEncoder = now_time;										 // ��������� ����� ��� ��������� ����
	//Serial.println(intervalEncoder);
	//Serial.println("///");
	MotorL.motor_rpm = (MotorL.EncoderPulseA + MotorL.EncoderPulseB)/2 * (1 / intervalEncoder) * 60 / 22 / 50;      // ��������� �������� �������� �������� � ������. ����� �� 50 ��������, ����� �� 22 ����� ��������, �������� �� 60 ������
	MotorR.motor_rpm = (MotorR.EncoderPulseA + MotorR.EncoderPulseB)/2 * (1 / intervalEncoder) * 60 / 22 / 50;
	//MotorR.motor_rpm = MotorR.EncoderPulseC * (1 / intervalEncoder) * 60 / 44 / 50;							   // ��������� ���������� ����� � ��� ��������� ������� �� ����� �� 44 � �� �� 22
	//Serial.println(MotorL.motor_rpm);
	//Serial.println(MotorR.motor_rpm);
	//Serial.println("===");


	// !!!!!!!!!!!!!!!!!!!!!!!! ������� ������� ������ 67 �� ��������� ��� ���-�� ������ ����� ����� � ���-�� ���������� ��������� !!!!!!!!!!!!!!!!!!!!!!!!!!!!

	MotorL.motor_speed = (float)MotorL.motor_rpm / 60 * PI * 67 / 1000; // ������� �������� � ������� �������� �� �� �� ������� ������ 67 �� � ��������� � �����
	MotorR.motor_speed = (float)MotorR.motor_rpm / 60 * PI * 67 / 1000; // ������� �������� � ������� �������� �� �� �� ������� ������ 67 �� � ��������� � �����
	MotorL.motor_napravlenie = napr_L;
	MotorR.motor_napravlenie = napr_R;

	MotorL.EncoderPulseA = MotorL.EncoderPulseB = 0; //MotorL.EncoderPulseC = 0;
	MotorR.EncoderPulseA = MotorR.EncoderPulseB = 0; //MotorR.EncoderPulseC = 0;
	//Serial.println("-");

	MotorL.motor_way += (float)MotorL.motor_speed * intervalEncoder;
	MotorR.motor_way += (float)MotorR.motor_speed * intervalEncoder;
}

void Raschet_Way(float way, float speed)
{
	Car.way_etap = 1;						// ������������� ��������� ���� ������
	Car.way_start = 0;
	MotorL.motor_way = MotorR.motor_way = 0;		//���������� �������� ���������� ����
 	Car.way = way; 								   //���������� ����� ���� ��� ���� ��������
	//Car.speed_L = 0;						   // ������ ��������� 0 ��������
	//Car.speed_R = 0;
    Serial.print(" speedL: "); Serial.print(Car.speed_L);
	Serial.print(" speedR: "); Serial.print(Car.speed_R);

	float deltaSpeed = speed - ((Car.speed_L + Car.speed_R) / 2);			  // ������� ������� ����� ������� ��������� � ���������
	if (deltaSpeed <= 0)
	{
		deltaSpeed = 0;									  // ���� ���� ������� ��� ���� �� ������� ��� ���� � ������
		Car.speed_L = Car.speed_R = speed;
	}
	Serial.print(" deltaSpeed: "); Serial.println(deltaSpeed);

	Car.timeRun = deltaSpeed / Car.accelerationRun;       // ����� ����������� ���-�� ������� �������� ��������
	Car.timeStop = speed / -Car.accelerationStop;       // ����� ����������� ���-�� ����������� � �������� ��������
//	Serial.print(" timeRun: "); Serial.println(Car.timeRun,4);
//	Serial.print(" timeStop: "); Serial.println(Car.timeStop, 4);

	float teor_way_1_etap = way / (Car.accelerationRun / -Car.accelerationStop + 1);     // ������� ����������� ��������� 
	float teor_way_3_etap = teor_way_1_etap * (Car.accelerationRun / -Car.accelerationStop);

	//Serial.print(" way: "); Serial.println(way, 4);
	//Serial.print(" speed: "); Serial.println(speed, 4);
	//Serial.print(" teor_way_1_etap: "); Serial.println(teor_way_1_etap, 4);
	//Serial.print(" teor_way_3_etap: "); Serial.println(teor_way_3_etap, 4);

	Car.way_1_etap = Car.accelerationRun *  Car.timeRun * Car.timeRun / 2; // ���� ������� ������� ������� � ������� ���������� �������� ��������

	if (Car.way_1_etap / teor_way_1_etap < 1)	    // ���� ��� ������� �� ����� �������� ���� ������ ��� ������������ ��������� �� ��������� ��������� �� ��������� �������� �� ���������
	{
		Car.way_1_etap = Car.accelerationRun *  Car.timeRun * Car.timeRun / 2; // ���� ������� ������� ������� � ������� ���������� �������� ��������
		Car.way_3_etap = -Car.accelerationStop *  Car.timeStop * Car.timeStop / 2; // ���� ������� ������� ������� ��� ���������
		Car.way_2_etap = way - Car.way_1_etap - Car.way_3_etap;        // ���� ������� ������� ������� � �������� ���������
		Car.speed = speed;
	}
	else
	{
		Car.way_1_etap = teor_way_1_etap; // ���� ������� ������� ������� � ������� ���������� �������� ��������
		Car.way_3_etap = teor_way_3_etap; // ���� ������� ������� ������� ��� ���������
		Car.way_2_etap = 0;        // ���� ������� ������� ������� � �������� ���������
		Car.speed = sqrt(2 * Car.accelerationRun * teor_way_1_etap);			   // ������ �� 2 �������� �� ��������� �������� �� ��������
	}

	//Serial.print(" way_1_etap: "); Serial.println(Car.way_1_etap, 4);
	//Serial.print(" way_2_etap: "); Serial.println(Car.way_2_etap, 4);
	//Serial.print(" way_3_etap: "); Serial.println(Car.way_3_etap, 4);
	//Serial.print(" Car.speedNEW: "); Serial.println(Car.speed, 4);
}

int PWM_from_Speed(float speed_)			  // �������������� ��������� �������� � ������ ��� ���������������
{
	return (255 * speed_ / 0.65);	  // ������������ ��� �������� �� ��������� �������� � ����� �� ������������ �������� ��� 255 ���	  ���������
}

void Car_Go(int napravl, float way, float speed)					// ������� �������� �������� ���� � �������� ���������
{
	Car.napravl = napravl;							 //���������� ����������� ���� ����
	Raschet_Way(way, speed);
	flag_GyroKurs = true;				   // ���� ��� ����� ����������� ���� � ����� ��� ������
	//flag_GyroKurs = false;				   // ���� ��� ����� ����������� ���� � ����� ��� ������
	Car.kurs = angleGyroZ;						   // ���������� ������� ���� ������� ����� ����������� ��� ��������

	Serial.print(" ");
	Serial.print(angleCompZ); Serial.print(" , ");
	Serial.print(angleGyroZ); Serial.print(" , ");
	Serial.println(BNO055_EulerAngles.z);


	//delay(2500000);
	//�������

//	int pwm = 255 * minSpeed / 0.65;	   // ��������� ����� ��������� !!! ��� ����� ���� ������ �� ������������� ������	 	// ������������ ��� �������� �� ��������� �������� � ����� �� ������������ �������� ��� 255 ���	  ���������
//	
//	if (Car.napravl == 1)		  // ���� ������
//	{
//		Run_MotorR(pwm);
//		Run_MotorL(pwm);
//	}
//	if (Car.napravl == -1)		  //���� �����
//	{
//		Run_MotorR(-pwm);
//		Run_MotorL(-pwm);
////		Car.speed = -Car.speed;
//	}


	//flag_StopWay = true;				  // ���� ��� ����� ����������� ���������� ���� � ������������ ����� ��� ��������

	//flag_AngleKurs = false;				  // ���� ��� ����� ����������� ���� � ������������ �� ��� ����������
	//flag_StopAngle = false;				  // ���� ��� ����� ����������� ����� ������������ ����� ��� ��������


}

void Car_Go_Angle(int napravl, int angle, float speed)
{
	if (napravl == 1)		  //���������� ����������� ���� ����
		{ 						  // �������
			Car.napravl = 2;
			if (angle <= 45)
			{
				dlinna_dugi  = PI * Car.diametrR / 2 * angle / 180;           //������� ������ ���� �� ������� ��� ����� ����������� ��� ����� ���� ��������	   //���������� ����� ���� ��� ���� ��������
				dlinna_angle = PI * Car.diametrR / 2 * 1 / 180;           //������� ������ ���� � 1 ������ ��� ���������� ����������� ���� �� ���������
			}
			if (angle > 45 && angle <= 90)
			{
				dlinna_dugi  = PI * (Car.diametrR + 0.007) / 2 * angle / 180;           //������� ������ ���� �� ������� ��� ����� ����������� ��� ����� ���� ��������	   //���������� ����� ���� ��� ���� ��������
				dlinna_angle = PI * (Car.diametrR + 0.007) / 2 *  1    / 180;           //������� ������ ���� � 1 ������ ��� ���������� ����������� ���� �� ���������
			}
			if (angle > 90 )
			{
				dlinna_dugi  = PI * (Car.diametrR + 0.01) / 2 * angle / 180;           //������� ������ ���� �� ������� ��� ����� ����������� ��� ����� ���� ��������	   //���������� ����� ���� ��� ���� ��������
				dlinna_angle = PI * (Car.diametrR + 0.01) / 2 * 1 / 180;           //������� ������ ���� � 1 ������ ��� ���������� ����������� ���� �� ���������
			}
	}
	else						     // ������
		{ 
			Car.napravl = 3;
			if (angle <= 45)
			{
				dlinna_dugi = PI * Car.diametrL / 2 * angle / 180;           //������� ������ ���� �� ������� ��� ����� ����������� ��� ����� ���� ��������	   //���������� ����� ���� ��� ���� ��������
				dlinna_angle = PI * Car.diametrL / 2 * 1 / 180;           //������� ������ ���� � 1 ������ ��� ���������� ����������� ���� �� ���������
			}
			if (angle > 45 && angle <= 90)
			{
				dlinna_dugi = PI * (Car.diametrL + 0.005) / 2 * angle / 180;           //������� ������ ���� �� ������� ��� ����� ����������� ��� ����� ���� ��������	   //���������� ����� ���� ��� ���� ��������
				dlinna_angle = PI * (Car.diametrL + 0.005) / 2 * 1 / 180;           //������� ������ ���� � 1 ������ ��� ���������� ����������� ���� �� ���������
			}
			if (angle > 90)
			{
				dlinna_dugi = PI * (Car.diametrL + 0.01) / 2 * angle / 180;           //������� ������ ���� �� ������� ��� ����� ����������� ��� ����� ���� ��������	   //���������� ����� ���� ��� ���� ��������
				dlinna_angle = PI * (Car.diametrL + 0.01) / 2 * 1 / 180;           //������� ������ ���� � 1 ������ ��� ���������� ����������� ���� �� ���������
			}

		}    
						
	Car.angle = angle;
	Car.speed_jelaemaya = speed;

	//Serial.println("");
	//Serial.print("Start Car_Go_Angle: ");
	//Serial.print(angleGyroZ); Serial.print(" , ");
	//Serial.println(BNO055_EulerAngles.z);

	Raschet_Way(dlinna_dugi, speed);
	flag_GyroKurs = false;				   // ���� ��� ����� �� ����� ����������� ���� � ��������

	Car.angle_start = angleGyroZ;		//���������� �������� ���������� ����
	Car.angle_start_BNO055 = BNO055_EulerAngles.z;	 //���������� �������� ���������� ����
	Car.angle_start_MyZ = BNO055_EulerAngles.my_z;	  	 //���������� �������� ���������� ����

	if (napravl == 2)
	{
		Car.angle_end = Car.angle_start + angle;		//���������� �������� ��������� ����
		if (Car.angle_end > 360) { Car.angle_end = Car.angle_end - 360; }
	}
	else
	{
		Car.angle_end = Car.angle_start - angle;		//���������� �������� ��������� ����
		if (Car.angle_end < 0) { Car.angle_end = Car.angle_end + 360; }
	}

	//Serial.print(" Car.angle_start : "); Serial.print(Car.angle_start, 1);
	//Serial.print(" Car.angle_end : "); Serial.println(Car.angle_end, 1);


	//Car.angle_end = Car.angle_start + angle;        // ���������� ���� �� ������� ��� ����� ������
	//if (Car.angle_end > 360) { Car.angle_end = Car.angle_end - 360; }

//	Car.way_start = (MotorL.motor_way + MotorR.motor_way) / 2;		//���������� �������� ���������� ����
//
//
//	Car.timeRun =  speed   /  Car.accelerationRun;       // ����� ����������� ���-�� ������� �������� ��������	  
//	Car.timeStop =  speed  / -Car.accelerationStop;       // ����� ����������� ���-�� ����������� � �������� ��������
////	Serial.print(" timeRun: "); Serial.println(Car.timeRun,4);
////	Serial.print(" timeStop: "); Serial.println(Car.timeStop, 4);
//	
//
//	float teor_way_1_etap = Car.way / (Car.accelerationRun / -Car.accelerationStop + 1);     // ������� ����������� ��������� 
//	float teor_way_3_etap = teor_way_1_etap * (Car.accelerationRun / -Car.accelerationStop);
//
//	Serial.print(" way: "); Serial.println(Car.way, 4);
//	Serial.print(" speed: "); Serial.println(speed, 4);
//	Serial.print(" teor_way_1_etap: "); Serial.println(teor_way_1_etap, 4);
//	Serial.print(" teor_way_3_etap: "); Serial.println(teor_way_3_etap, 4);
//
//	Car.way_1_etap = Car.accelerationRun *  Car.timeRun * Car.timeRun / 2; // ���� ������� ������� ������� � ������� ���������� �������� ��������
//
//
//	if (Car.way_1_etap / teor_way_1_etap < 1)	    // ���� ��� ������� �� ����� �������� ���� ������ ��� ������������ ��������� �� ��������� ��������� �� ��������� �������� �� ���������
//	{
//		Car.way_1_etap = Car.accelerationRun *  Car.timeRun * Car.timeRun / 2; // ���� ������� ������� ������� � ������� ���������� �������� ��������
//		Car.way_3_etap = -Car.accelerationStop *  Car.timeStop * Car.timeStop / 2; // ���� ������� ������� ������� ��� ���������
//		Car.way_2_etap = Car.way - Car.way_1_etap - Car.way_3_etap;        // ���� ������� ������� ������� � �������� ���������
//		Car.speed = speed;
//	}
//	else
//	{
//		Car.way_1_etap = teor_way_1_etap; // ���� ������� ������� ������� � ������� ���������� �������� ��������
//		Car.way_3_etap = teor_way_3_etap; // ���� ������� ������� ������� ��� ���������
//		Car.way_2_etap = 0;        // ���� ������� ������� ������� � �������� ���������
//		Car.speed = sqrt(2 * Car.accelerationRun * teor_way_1_etap);			   // ������ �� 2 �������� �� ��������� �������� �� ��������
//
//	}
//
//	Serial.print(" way_1_etap: "); Serial.println(Car.way_1_etap, 4);
//	Serial.print(" way_2_etap: "); Serial.println(Car.way_2_etap, 4);
//	Serial.print(" way_3_etap: "); Serial.println(Car.way_3_etap, 4);
//	Serial.print(" Car.speedNEW: "); Serial.println(Car.speed, 4);


	//�������
	//Car.way_etap = 1;						// ������������� ��������� ���� ������
	//Car.speed_L = 0;						   // ������ ��������� 0 ��������
	//Car.speed_R = 0;
 	//int pwm = 255 * minSpeed / 0.65;	   // ��������� ����� ��������� !!! ��� ����� ���� ������ �� ������������� ������ � ������� ������	 	// ������������ ��� �������� �� ��������� �������� � ����� �� ������������ �������� ��� 255 ���	  ���������

	//if (Car.napravl == 2)		  // ���� ������
	//{
	//	Run_MotorR(-pwm);
	//	Run_MotorL(pwm);
	//}
	//if (Car.napravl == 3)		  //���� ������
	//{
	//	Run_MotorR(pwm);
	//	Run_MotorL(-pwm);
	//}

	//delay(2500000);

	//flag_GyroKurs = true;				   // ���� ��� ����� ����������� ���� � ����� ��� ������
	//flag_StopWay = true;				  // ���� ��� ����� ����������� ���������� ���� � ������������ ����� ��� ��������

	//flag_AngleKurs = false;				  // ���� ��� ����� ����������� ���� � ������������ �� ��� ����������
	//flag_StopAngle = false;				  // ���� ��� ����� ����������� ����� ������������ ����� ��� ��������




	//Car.speed_L = minSpeed;
	//Car.speed_R = 0;
	//int pwm = 255 * minSpeed / 0.65;	   // ��������� ����� ��������� !!! ��� ����� ���� ������ �� ������������� ������ � ������� ������	 	// ������������ ��� �������� �� ��������� �������� � ����� �� ������������ �������� ��� 255 ���	  ���������
	//Run_MotorR(0);
	//Run_MotorL(pwm);
	//
	//flag_AngleKurs = true;				  // ���� ��� ����� ����������� ���� � ������������ �� ��� ����������
	//flag_StopAngle = true;				  // ���� ��� ����� ����������� ����� ������������ ����� ��� ��������

	//flag_GyroKurs = false;				  // ���� ��� ����� ����������� ���� � ������������ �� ��� ����������
	//flag_StopWay = false;				  // ���� ��� ����� ����������� ���������� ���� � ������������ ����� ��� ��������

}
//void Car_Go_Left(int kurs, float speed)
//{
//	//kurs = kurs * 0.991667;          // �������� ����� ����� �����������. ����� �������
//	Car.speed_L = 0;
//	Car.speed_R = minSpeed;
//	int pwm = 255 * minSpeed / 0.65;	   // ��������� ����� ��������� !!! ��� ����� ���� ������ �� ������������� ������ � ������� ������	 	// ������������ ��� �������� �� ��������� �������� � ����� �� ������������ �������� ��� 255 ���	  ���������
//	Serial.print(" PWM : "); Serial.print(pwm,4);
//
//	Run_MotorR(pwm);
//	Serial.print(" MotorR.motor_pwm : "); Serial.print(MotorR.motor_pwm,4);
//
//	Run_MotorL(0);
//	Car.kurs = angleGyroZ - kurs;        // ���������� ������� ���� ������� ����� ����������� ��� ��������
//	if (Car.kurs < 0) { Car.kurs = Car.kurs + 360; }
//
//	flag_AngleKurs = true;				  // ���� ��� ����� ����������� ���� � ������������ �� ��� ����������
//	flag_StopAngle = true;				  // ���� ��� ����� ����������� ����� ������������ ����� ��� ��������
//
//	flag_GyroKurs = false;				  // ���� ��� ����� ����������� ���� � ������������ �� ��� ����������
//	flag_StopWay = false;				  // ���� ��� ����� ����������� ���������� ���� � ������������ ����� ��� ��������
//
//}



void Motor_Stop()
{
	//Serial.println("Motor_Stop:");
	Car.speed_L = 0;
	Car.speed_R = 0;
	Run_MotorR(0);
	Run_MotorL(0);
	flag_GyroKurs = false;
	Car.kurs = angleGyroZ; 
	now_otklonenie_gyro = sum_otklonenie_gyro = speed_otklonenie_gyro = 0;
	now_otklonenie_L = sum_otklonenie_L = speed_otklonenie_L = 0;
	//flag_AngleKurs = false;
	//flag_StopAngle = false;
	//flag_StopWay = false;

	//Serial.println("= STOP MOTOR =");
}

void Motor_PWM()
{
	int speedL_temp = PWM_from_Speed(Car.speed_L_kurs);
	int speedR_temp = PWM_from_Speed(Car.speed_R_kurs);

	Run_MotorL(speedL_temp);          // ��������� �������� �������� ������ � �������� ���������. �������� �������� � ������� Dvijenie_po_Etapam
	Run_MotorR(speedR_temp);          // ��������� �������� �������� ������ � �������� ���������. �������� �������� � ������� Dvijenie_po_Etapam


	//Serial.print(" Car.speed_L_kurs : "); Serial.print(Car.speed_L_kurs, 3);
	//Serial.print(" MotorL.motor_speed : "); Serial.print(MotorL.motor_speed, 3);
	//Serial.print(" PWM_L : "); Serial.print(speedL_temp);
	//Serial.print(" MotorL.motor_way : "); Serial.println(MotorL.motor_way);

	//Serial.print(" Car.speed_R_kurs : "); Serial.print(Car.speed_R_kurs, 3);
	//Serial.print(" MotorR.motor_speed : "); Serial.print(MotorR.motor_speed, 3);
	//Serial.print(" PWM_R : "); Serial.print(speedR_temp);
	//Serial.print(" MotorR.motor_way : "); Serial.println(MotorR.motor_way);
	//Serial.println("");

}
void Motor_PWM_PID()		  // ������� ������� ������������ ���������� � ���������� �������� �������� �������� 
{
	static float pid_P = 2;
	static float pid_I = 0.01;
	static float pid_D = 0;

//		if (MotorL.motor_speed != 0)
		if (Car.speed_L_kurs > 0)
		{
			pred_otklonenie_L = now_otklonenie_L;					 //���������� ����� ���� ���������� ��� ���������� ���������
		   // Serial.print(" MotorL.motor_speed : "); Serial.println(MotorL.motor_speed);
			now_otklonenie_L = Car.speed_L_kurs - MotorL.motor_speed;	 // ��������� ���������� � ������� ������
			//Serial.print(" now_otklonenie_L : "); Serial.println(now_otklonenie_L);
			sum_otklonenie_L += now_otklonenie_L;					 // ������������ ����� ���� ����������
			speed_otklonenie_L = now_otklonenie_L - pred_otklonenie_L;	// ��������� �������� ��������� ����������
											// ��� ��������� 												  �������� ���������������� ����� ����� �������� ��� � ��� ��� ��������� �������� �������� ������ �� ������ �����
			changeSpeed_L = (now_otklonenie_L * pid_P) + (sum_otklonenie_L * pid_I) + (speed_otklonenie_L * pid_D); 	// ��������� ��� ���������
			//Serial.print(" changeSpeed_L : "); Serial.println(changeSpeed_L);
			new_Speed_L = Car.speed_L_kurs + changeSpeed_L;		  //��������� � �������� ������� ������ ���� �������� �� ��� ����������

			//Serial.print(" =new_Speed_L : "); Serial.println(new_Speed_L);


			//��� �������� � ���� ������������ ��������, ������ ����� ������			
			//Run_MotorL(MotorL.motor_pwm + changeSpeed_L);
		}
		if (Car.speed_L_kurs < 0)
		{
			pred_otklonenie_L = now_otklonenie_L;					 //���������� ����� ���� ���������� ��� ���������� ���������
			now_otklonenie_L = -Car.speed_L_kurs - MotorL.motor_speed;	 // ��������� ���������� � ������� ������	 
																	 //	�������� � ������ ������ ������������� � �� ������� ����� ���� ������������� ���� ���� �����
			sum_otklonenie_L += now_otklonenie_L;					 // ������������ ����� ���� ����������
			speed_otklonenie_L = now_otklonenie_L - pred_otklonenie_L;	// ��������� �������� ��������� ����������
											// ��� ��������� 												  �������� ���������������� ����� ����� �������� ��� � ��� ��� ��������� �������� �������� ������ �� ������ �����
			changeSpeed_L = (now_otklonenie_L * pid_P) + (sum_otklonenie_L * pid_I) + (speed_otklonenie_L * pid_D); 	// ��������� ��� ���������
			new_Speed_L = Car.speed_L_kurs - changeSpeed_L;		  //��������� � �������� ������� ������ ���� �������� �� ��� ���������� ��� �������� 
			//Run_MotorL(MotorL.motor_pwm - changeSpeed_L);
		}

			//if (MotorR.motor_speed != 0)
		if (Car.speed_R_kurs > 0)
		{
			pred_otklonenie_R = now_otklonenie_R;					 //���������� ����� ���� ���������� ��� ���������� ���������
			now_otklonenie_R = Car.speed_R_kurs - MotorR.motor_speed;	 // ��������� ���������� � ������� ������
		//	Serial.print(" MotorR.motor_speed : "); Serial.println(MotorR.motor_speed);

			sum_otklonenie_R += now_otklonenie_R;					 // ������������ ����� ���� ����������
			speed_otklonenie_R = now_otklonenie_R - pred_otklonenie_R;	// ��������� �������� ��������� ����������
											// ��� ��������� 
			changeSpeed_R = (now_otklonenie_R * pid_P) + (sum_otklonenie_R * pid_I) + (speed_otklonenie_R * pid_D); 	// ��������� ��� ���������
		//	Serial.print(" changeSpeed_R : "); Serial.println(changeSpeed_R);

			new_Speed_R = Car.speed_R_kurs + changeSpeed_R;		  //��������� � �������� ������� ������ ���� �������� �� ��� ����������
		//	Serial.print(" new_Speed_R : "); Serial.println(new_Speed_R);


			//��� �������� � ���� ������������ ��������, ������ ����� ������			
			//Run_MotorR(MotorR.motor_pwm + changeSpeed_R);
		}
		if (Car.speed_R_kurs < 0)
		{
			pred_otklonenie_R = now_otklonenie_R;					 //���������� ����� ���� ���������� ��� ���������� ���������
			now_otklonenie_R = -Car.speed_R_kurs - MotorR.motor_speed;	 // ��������� ���������� � ������� ������
			sum_otklonenie_R += now_otklonenie_R;					 // ������������ ����� ���� ����������
			speed_otklonenie_R = now_otklonenie_R - pred_otklonenie_R;	// ��������� �������� ��������� ����������
											// ��� ��������� 
			changeSpeed_R = (now_otklonenie_R * pid_P) + (sum_otklonenie_R * pid_I) + (speed_otklonenie_R * pid_D); 	// ��������� ��� ���������
			new_Speed_R = Car.speed_R_kurs - changeSpeed_R;		  //��������� � �������� ������� ������ ���� �������� �� ��� ����������

			//��� �������� � ���� ������������ ��������, ������ ����� ������			
			//Run_MotorR(MotorR.motor_pwm + changeSpeed_R);
		}
		int speedL_temp = PWM_from_Speed(new_Speed_L);
		int speedR_temp = PWM_from_Speed(new_Speed_R);
		new_Speed_L = new_Speed_R = 0;          // �������� ���-�� �� ������ ���� �� ��������� ��� ��������� �������
		Run_MotorL(speedL_temp);
		Run_MotorR(speedR_temp);
		//Serial.print(" PWM speedL_temp : "); Serial.print(speedL_temp);
		//Serial.print(" PWM speedR_temp : "); Serial.println(speedR_temp);





	//		Serial.print(" Time --- : "); Serial.println(millis());

	//Serial.print(" changeSpeed_L : "); Serial.print(changeSpeed_L);
	//Serial.print(" now_otklonenie_L : "); Serial.print(now_otklonenie_L);
	//Serial.print(" sum_otklonenie_L : "); Serial.print(sum_otklonenie_L);
	//Serial.print(" speed_otklonenie_L : "); Serial.print(speed_otklonenie_L);


	//Serial.print(" Car.speed_L : "); Serial.print(Car.speed_L, 3);
	//Serial.print(" Car.speed_L_kurs : "); Serial.print(Car.speed_L_kurs, 3);
	//Serial.print(" MotorL.motor_speed : "); Serial.print(MotorL.motor_speed, 3);
	//Serial.print(" changeSpeed_L : "); Serial.print(changeSpeed_L, 3);
	//Serial.print(" new_Speed_L : "); Serial.print(new_Speed_L, 3);
	//Serial.print(" PWM_L : "); Serial.print(speedL_temp);
	//Serial.print(" MotorL.motor_way : "); Serial.println(MotorL.motor_way);

	//Serial.print(" Car.speed_R : "); Serial.print(Car.speed_R, 3);
	//Serial.print(" Car.speed_R_kurs : "); Serial.print(Car.speed_R_kurs, 3);
	//Serial.print(" MotorR.motor_speed : "); Serial.print(MotorR.motor_speed, 3);
	//Serial.print(" changeSpeed_R : "); Serial.print(changeSpeed_R, 3);
	//Serial.print(" new_Speed_R : "); Serial.print(new_Speed_R, 3);
	//Serial.print(" PWM_R : "); Serial.print(speedR_temp);
	//Serial.print(" MotorR.motor_way : "); Serial.println(MotorR.motor_way);





	//Serial.print(" changeSpeed_R : "); Serial.print(changeSpeed_R);
	//Serial.print(" now_otklonenie_R : "); Serial.print(now_otklonenie_R);
	//Serial.print(" sum_otklonenie_R : "); Serial.print(sum_otklonenie_R);
	//Serial.print(" speed_otklonenie_R : "); Serial.println(speed_otklonenie_R);

	//l = l + MotorL.motor_rpm;
	//r = r + MotorR.motor_rpm;
	//Serial.print(" MotorL.motor_rpm : "); Serial.println(l);
	//Serial.print(" MotorR.motor_rpm : "); Serial.println(r);
 }




void Motor_Gyro_PID()		   
{
	float pid_P; float pid_I; float pid_D;
	pred_otklonenie_gyro = now_otklonenie_gyro;								 //���������� ����� ���� ���������� ��� ���������� ���������
	now_otklonenie_gyro = Car.kurs - angleGyroZ;							 // ��������� ���������� � ������� ������
											   	//���� ���������� ������ 180 �������� �� ����� ������ ��� ��������� 360		  // ������ ��� ���� ������ 360 ��������
	if (now_otklonenie_gyro < -180 ) 	{		now_otklonenie_gyro = now_otklonenie_gyro + 360;	}
	if (now_otklonenie_gyro >  180 )	{		now_otklonenie_gyro = now_otklonenie_gyro - 360;	}

	sum_otklonenie_gyro += now_otklonenie_gyro;								 // ������������ ����� ���� ����������
	speed_otklonenie_gyro = now_otklonenie_gyro - pred_otklonenie_gyro;	  	 // ��������� �������� ��������� ����������
																			 
		pid_P = 0.07;		pid_I = 0;		pid_D = 0.0; 		   // ��� ��������� ��� ������
																			
	changeSpeed_gyro = (now_otklonenie_gyro * pid_P) + (sum_otklonenie_gyro * pid_I) - (speed_otklonenie_gyro * pid_D); 	// ��������� ��� ���������
	changeSpeed_gyro = changeSpeed_gyro / 2;			//����� �������
	Car.speed_L_kurs = Car.speed_L - changeSpeed_gyro;  //�������� ����� �������� � ������ ���������� �� �����
	Car.speed_R_kurs = Car.speed_R + changeSpeed_gyro;	//�������� ����� �������� � ������ ���������� �� �����

	//Serial.print(" speed_L_kurs 2: "); Serial.print(Car.speed_L_kurs);
	//Serial.print(" speed_R_kurs 2: "); Serial.println(Car.speed_R_kurs);

	if (Car.speed_L_kurs > 0.65) Car.speed_L_kurs = 0.65;
	if (Car.speed_L_kurs < 0.1) Car.speed_L_kurs = 0.1;

	if (Car.speed_R_kurs > 0.65) Car.speed_R_kurs = 0.65;
	if (Car.speed_R_kurs < 0.1) Car.speed_R_kurs = 0.1;

	//Serial.print(" speed_L_kurs 3: "); Serial.println(Car.speed_L_kurs);
	//Serial.print(" speed_R_kurs 3: "); Serial.println(Car.speed_R_kurs);

	//Serial.println(" - ");
	//Serial2.print(" Car.kurs : "); Serial2.print(Car.kurs,4);
	//Serial2.print(" angleGyroZ : "); Serial2.print(angleGyroZ,4);
	//Serial.print(" BNO055_EulerAngles.z : "); Serial.print(BNO055_EulerAngles.z, 4);
	//Serial2.print(" now_otklonenie_gyro : "); Serial2.print(now_otklonenie_gyro,4);
	//Serial2.print(" speed_otklonenie_gyro : "); Serial2.print(speed_otklonenie_gyro, 4);

	//Serial2.print(" changeSpeed_gyro : "); Serial2.println(changeSpeed_gyro,4);
}

//void Motor_Angle_PID()
//{
//	float pid_P = 0.01;
//	float pid_I = 0;
//	float pid_D = 0;
//
//	pred_otklonenie_gyro = now_otklonenie_gyro;								 //���������� ����� ���� ���������� ��� ���������� ���������
//	now_otklonenie_gyro = Car.kurs - angleGyroZ;							 // ��������� ���������� � ������� ������
//												//���� ���������� ������ 180 �������� �� ����� ������ ��� ��������� 360
//	if (now_otklonenie_gyro < -180) { now_otklonenie_gyro = now_otklonenie_gyro + 360; }
//	if (now_otklonenie_gyro > 180) { now_otklonenie_gyro = now_otklonenie_gyro - 360; }
//
//	sum_otklonenie_gyro += now_otklonenie_gyro;								 // ������������ ����� ���� ����������
//	speed_otklonenie_gyro = now_otklonenie_gyro - pred_otklonenie_gyro;	  	 // ��������� �������� ��������� ����������
//																			 // ��� ��������� ��� ������
//	changeSpeed_gyro = (now_otklonenie_gyro * pid_P) + (sum_otklonenie_gyro * pid_I) + (speed_otklonenie_gyro * pid_D); 	// ��������� ��� ���������
//	if (Car.speed_L == 0 )	{ Car.speed_R = minSpeed - changeSpeed_gyro; }						// ������ ���� ������������ �� ����� � ������
//	if (Car.speed_R == 0 )  { Car.speed_L = minSpeed - changeSpeed_gyro; }
//
//	//Serial.print(" - ");
//	//Serial.print(" Car.kurs : "); Serial.println(Car.kurs, 4);
//	//Serial.print(" angleGyroZ : "); Serial.println(angleGyroZ, 4);
//	Serial.print(" now_otklonenie_gyro : "); Serial.print(now_otklonenie_gyro, 4);
//	Serial.print(" changeSpeed_gyro : "); Serial.print(changeSpeed_gyro, 4);
//	Serial.print(" Car.speed_L : "); Serial.print(Car.speed_L, 4);
//	Serial.print(" Car.speed_R : "); Serial.print(Car.speed_R, 4);
//
//	Serial.println(" ");
//
//}

float Povorot_Angle(float nacal, float konech)
{
	float povernut_angle;

	if (Car.napravl == 2)     // ���� �������� ������
	{
		povernut_angle = konech - nacal;				   // ��� ����� ����� ��� ����
	}
	if (Car.napravl == 3)     // ���� �������� �����
	{
		povernut_angle = nacal - konech;                 // ��� ���� ����� ��� �����
	}
	//Serial.print(" p_w1: "); Serial.print(povernut_angle,4);
	if (povernut_angle < 0)   povernut_angle += 360;

	return 	povernut_angle;
}

void Dvijenie_po_Etapam()
{
	float accelerationRun = Car.accelerationRun * intervalEncoder;
	float accelerationStop = Car.accelerationStop * intervalEncoder;    // ���������� ������������� ���������

	//Serial.print(" accelerationRun ");Serial.print(accelerationRun);
	//Serial.print(" accelerationStop ");Serial.println(accelerationStop);
	//Serial.print(" intervalEncoder ");Serial.println(intervalEncoder);



	float way_now = (MotorL.motor_way + MotorR.motor_way) / 2 - Car.way_start;			   // ������� ����� ��������� 
 	//Serial.print(" way_now : "); Serial.print(way_now, 4);

	switch (Car.way_etap)
	{
	case 1:    // ���� �������
	{

		if (way_now < Car.way_1_etap)
		{
			if (Car.napravl == 1)	   //���� ������
			{
				Car.speed_L += accelerationRun;	 // ��������� � ����������� �� ��������� ��������� � ��������
				Car.speed_R += accelerationRun;	 // ��������� � ����������� �� ��������� ��������� � ��������
			}
			if (Car.napravl == -1)		//���� �����
			{
				Car.speed_L -= accelerationRun;	 // ��������� � ����������� �� ��������� ��������� � ��������
				Car.speed_R -= accelerationRun;	 // ��������� � ����������� �� ��������� ��������� � ��������
			}
			if (Car.napravl == 2)		//���� �������
			{
				Car.speed_L += accelerationRun;	 // ��������� � ����������� �� ��������� ��������� � ��������
				Car.speed_R -= accelerationRun;	 // ��������� � ����������� �� ��������� ��������� � ��������
			}
			if (Car.napravl == 3)		//���� ������
			{
				Car.speed_L -= accelerationRun;	 // ��������� � ����������� �� ��������� ��������� � ��������
				Car.speed_R += accelerationRun;	 // ��������� � ����������� �� ��������� ��������� � ��������
			}
		}
		else
		{
			if (Car.napravl == 1)	   //���� ������
			{
				Car.speed_L = Car.speed;
				Car.speed_R = Car.speed;
			}
			if (Car.napravl == -1)		//���� �����
			{
				Car.speed_L = -Car.speed;
				Car.speed_R = -Car.speed;
			}
			if (Car.napravl == 2)		//���� �������
			{
				Car.speed_L = Car.speed;
				Car.speed_R = -Car.speed;
			}
			if (Car.napravl == 3)		//���� ������
			{
				Car.speed_L = -Car.speed;
				Car.speed_R = Car.speed;
			}
			Car.way_etap = 2;      // ��������� �� 2 ���� �������� � �������� ���������
			//Serial.println(" 1 ETAP END ");
		}
	}
	break;
	case 2:    // ���� ��������
	{
		if (way_now >= Car.way_1_etap + Car.way_2_etap)
		{
			Car.way_etap = 3;      // ��������� �� 3 ���� ����������
			//Serial.println(" 2 ETAP END ");
		}
	}
	break;
	case 3:    // ���� ����������
	{
		if (way_now < Car.way_1_etap + Car.way_2_etap + Car.way_3_etap)
		{
			if (Car.napravl == 1)	   //���� ������
			{
				Car.speed_L += accelerationStop;	 // ��������� � ����������� �� ��������� ��������� � ��������
				Car.speed_R += accelerationStop;	 // ��������� � ����������� �� ��������� ��������� � ��������
			}
			if (Car.napravl == -1)	   //���� �����
			{
				Car.speed_L -= accelerationStop;	 // ��������� � ����������� �� ��������� ��������� � ��������
				Car.speed_R -= accelerationStop;	 // ��������� � ����������� �� ��������� ��������� � ��������
			}
			if (Car.napravl == 2)		//���� �������
			{
				Car.speed_L += accelerationStop;	 // ��������� � ����������� �� ��������� ��������� � ��������
				Car.speed_R -= accelerationStop;	 // ��������� � ����������� �� ��������� ��������� � ��������
			}
			if (Car.napravl == 3)		//���� ������
			{
				Car.speed_L -= accelerationStop;	 // ��������� � ����������� �� ��������� ��������� � ��������
				Car.speed_R += accelerationStop;	 // ��������� � ����������� �� ��������� ��������� � ��������
			}
		}
		else
		{
			Motor_Stop();
			Car.way_etap = 4;     // ��������� �� 4 ���� 
			timeStop = millis();
			//Serial.println(" Start Pause ");
		}
	}
	break;
	case 4:    // ���� �����
	{
		if (timeStop + 500 < millis())       //���� ������ 100 ����������
		{
			Car.way_etap = 0;      // ��������� �� 0 ���� 

			//Serial.print(" Z = ");	Serial.print(angleGyroZ);
			//Serial.print(" .z = ");			Serial.print(BNO055_EulerAngles.z);	
			//Serial.print(" .my_z =  ");			Serial.print(BNO055_EulerAngles.my_z);
			Serial.print(" AngleGyro: "); Serial.print(Povorot_Angle(Car.angle_start, angleGyroZ));
			Serial.print(" AngleBNO055: "); Serial.print(Povorot_Angle(Car.angle_start_BNO055,BNO055_EulerAngles.z));
			Serial.print(" AngleMy: "); Serial.println(Povorot_Angle(Car.angle_start_MyZ, BNO055_EulerAngles.my_z));


			//Serial.print(" Delta2 "); Serial.print((Car.angle_start - angleGyroZ) - 90 );
			//Serial.print(" Delta2 "); Serial.print((Car.angle_start - angleGyroZ) - (Car.angle_start_BNO055 - BNO055_EulerAngles.z));
			//Serial.print(" Delta3: "); Serial.println((Car.angle_start - angleGyroZ) - (Car.angle_start_MyZ - BNO055_EulerAngles.my_z));
			//Serial.println("");

			//Serial.print(" angleGyroZ : "); Serial.print(angleGyroZ, 4);
			//Serial.print(" way_now : "); Serial.print(way_now, 4);
			switch_program = -switch_program + 1;  // ��������� � ��������� ���� ���������
			//Serial.print(" OSHIBKA: ");Serial.print((angleGyroZ - Car.angle_end) / Car.angle * 100, 2);Serial.println("%");
			//Serial.println(" End Pause ");
		}
	}
	break;
	}
	//Serial.print(" SpeedL ");Serial.print(Car.speed_L);
	//Serial.print(" SpeedR ");Serial.println (Car.speed_R);

}

void Raschet_Angle()
{

	float way_now = (MotorL.motor_way + MotorR.motor_way) / 2 - Car.way_start;			   // ������� ����� ��������� 
	float angle_way = way_now / dlinna_angle;                                         // �� ������ ���� �����������
	Serial.print(" a_way: "); Serial.print(angle_way);
	//Serial.print("  a_start: "); Serial.print(Car.angle_start);
	//Serial.print("  a_GyroZ: "); Serial.print(angleGyroZ);
	float povernut_angle;

	if (Car.napravl == 2)     // ���� �������� ������
	{
		povernut_angle = angleGyroZ - Car.angle_start;				   // ��� ����� ����� ��� ����
	}
	if (Car.napravl == 3)     // ���� �������� �����
	{
		povernut_angle = Car.angle_start - angleGyroZ ;                 // ��� ���� ����� ��� �����
	}
		//Serial.print(" p_w1: "); Serial.print(povernut_angle,4);
		if (povernut_angle < 0)   povernut_angle += 360;
		Serial.print(" p_2: "); Serial.print(povernut_angle);


		float otkl = angle_way - povernut_angle;	  // ������� ���������� �� ���� ������� �� ������ ���� ��������� � ��� ��� ���� �� ����� ����
		Serial.print(" D: "); Serial.print(otkl); Serial.print("   ");
		float proc_otkl = otkl / angle_way  ;			// ������� ����������
		Serial.print(proc_otkl*100);	Serial.println (" %");
}


char GetCharSerial2()
{
	byte len = Serial2.available();
	if (len > 0)
	{
		Serial2.print(" available: ");  Serial2.print(len);

		Serial2.readBytes(komandaSerial2,len); // ������ ����������
		//Serial.print(" komandaSerial2: ");  Serial.println(komandaSerial2[0]);
		//Serial2.print(" komanda: ");  Serial2.println(komandaSerial2[0]);
		return komandaSerial2[0];
	}
	return 0;

}
void Loop_Encoder()
{
	//==========================================================
	if (flag_Encoder == true)
	{
		flag_Encoder = false;
		Read_Encoder();						  // ����� ����������  0,2 ����������� // ������� ������ �������� � ���������� ������� ��������
		//Serial.print("Read_Encoder "); 
	}

}
void Loop_PWN()
{
	//==========================================================
	if (flag_PWM == true)				  // ����� ���������� 0.4 ����������
	{
		//digitalWrite(49, 1);

		flag_PWM = false;
		if (Car.way_etap != 0)			   //���� ���� ���� �����-�� ���� ��������
		{

			//Raschet_Angle();				  //�������� ������� ��� ����� ��� �������� ��������� �� ���������� ����

			Dvijenie_po_Etapam();			  //������� ������� ����������� �� �� ����� ����� ������ �������� � ���������� ��������	 // ����� ���������� 0.05 ����������

		//	Serial.print(" Car.speed_L : "); Serial.print(Car.speed_L, 4);
		//	Serial.print(" Car.speed_R : "); Serial.println(Car.speed_R, 4);

			if (flag_GyroKurs == true)
			{
				Motor_Gyro_PID();			 // ���� ��������� ��������� ���� ��� ��������  �� � ������� ��� ���������� �������� ���������� �������� �� ����� ������
			}
			else
			{
				Car.speed_L_kurs = Car.speed_L;    // ���� �� ����� �������������� �������� �� ����������� ����� �� ���������� ��� ����
				Car.speed_R_kurs = Car.speed_R;	   // ���� �� ����� �������������� �������� �� ����������� ����� �� ���������� ��� ����
			}
			//Serial.println(" --- ");
			//Serial.print(" Car.speed_L_kurs : "); Serial.print(Car.speed_L_kurs, 4);
			//Serial.print(" Car.speed_R_kurs : "); Serial.println(Car.speed_R_kurs, 4);
			//Serial.println(" --- ");


			//Motor_PWM();
			Motor_PWM_PID();				  // ������������ �������� ���������� �������� �����   // ����� ���������� 0.2 ����������


			//Serial.print(" Angle: "); Serial.print(Car.angle_start- angleGyroZ);
			//Serial.print(" Angle2: "); Serial.println(Car.angle_start_BNO055 - BNO055_EulerAngles.z);

			//Serial.print(" time: "); Serial.print(millis());
			//Serial.print(" Car.speed: "); Serial.println(Car.speed, 4);
			//Serial.print(" Car.kurs : "); Serial.print(Car.kurs, 4);
			//Serial.print(" angleGyroZ : "); Serial.println(angleGyroZ, 1);
			//PrintDirection();

			//Serial.print(" MotorL.motor_pwm: "); Serial.print(MotorL.motor_pwm);
			//Serial.print(" MotorL.motor_speed: "); Serial.print(MotorL.motor_speed, 4);
			//Serial.print(" MotorL.motor_way: "); Serial.print(MotorL.motor_way, 4);

			//Serial.print(" MotorR.motor_pwm: "); Serial.print(MotorR.motor_pwm);
			//Serial.print(" MotorR.motor_speed: "); Serial.print(MotorR.motor_speed, 4);
			//Serial.print(" MotorR.motor_way: "); Serial.println(MotorR.motor_way, 4);


		}

	}
	//==========================================================

}

void Loop_Switch()
{
	//Serial.print(" switch_program: "); Serial.println(switch_program);
	if (switch_program > 0)
	{

		switch (switch_program)
		{
		case 1:
		{
			//Car_Go_Angle(1, 180, 0.25);			// ������� ������� �� 90 ��������
			Car_Go(1, 300, 0.3);					 // �������� ����� �� 1 ����
			switch_program = -switch_program;
		}	break;
		//case 2:
		//{
		//	Car_Go_Angle(1, 90, 0.25);			// ������� ������� �� 90 ��������
		//	//Car_Go(-1, 1.5, 0.25);					 // �������� ����� �� 1 ����
		//	switch_program = -switch_program;
		//}	break;
		//case 3:
		//{
		//	//Car_Go_Angle(-1, 180, 0.25);			// ������� ������� �� 90 ��������
		//	//Car_Go(1, 1.5, 0.25);					 // �������� ����� �� 1 ����
		//	//switch_program = -switch_program;
		//	switch_program = 1;
		//}	break;
		//case 4:
		//{
		//	Car_Go_Angle(1, 90, 0.25);			// ������� ������� �� 90 ��������
		//	//Car_Go_Angle(-1, 180, 0.25);			// ������� ������� �� 90 ��������
		//	switch_program = 1; //-switch_program;
		//}	break;
	//	case 5:
	//	{
	//		//Car_Go_Angle(3, 360, 0.25);			// ������� ������� �� 90 ��������

	//		//Car_Go(1, 0.5, 0.2);					 // �������� ����� �� 1 ����
	//		//switch_program = -switch_program;
//				switch_program = 1;			
	  //	}	break;
	  //	
	  //	//case 6:
	  //	//{
	  //	//	Car_Go_Angle(2, 90, 0.2);			// ������� ������� �� 90 ��������
	  //	//	switch_program = -switch_program;
	  //	//}
	  //	//break;
	  //	//case 7:
	  //	//{
	  //	//	Car_Go(1, 0.5, 0.2);					 // �������� ����� �� 1 ����
	  //	//	switch_program = -switch_program;
	  //	//}
	  //	//break;
	  //	//case 8:
	  //	//{
	  //	//	Car_Go_Angle(2, 90, 0.2);			// ������� ������� �� 90 ��������
	  //	//	switch_program = -switch_program;
	  //	//}
	  //	//break;



		}


	}

}
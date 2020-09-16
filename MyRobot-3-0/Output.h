
void PrintRawData()
{
	Serial.print("RAW -> ");
	 Serial.print(ax);   Serial.print(" , ");
     Serial.print(ay);   Serial.print(" , ");
     Serial.print(az);   Serial.print(" ;  !!  ;  ");
	//Serial.print(gx);   Serial.print(" , ");
	//Serial.print(gy);   Serial.print(" , ");
	//Serial.print(gz);   Serial.print(" ,  !!  ;  ");
	//Serial.print(temp9255);   Serial.print(" , ");
	Serial.println(" ");

}
void PrintData()
{
	Serial.print("Data -> ");
	  Serial.print(accX, 2);   Serial.print(" , ");
      Serial.print(accY, 2);   Serial.print(" , ");
	  Serial.print(accZ, 2);   Serial.print(" ;  !!  ");
	//Serial.print(gyroX, 2);   Serial.print(" , ");
	//Serial.print(gyroY, 2);   Serial.print(" , ");
	//Serial.print(gyroZ, 2);   Serial.print(" , ");
	Serial.println(" ");

}
void PrintAngle()
{
	//Serial.print("Angle -> ");
	Serial.print(angleAccX, 1);   Serial.print(" , ");
	Serial.print(angleGyroY, 1);   Serial.print(" , ");
	Serial.print(" / ");
	Serial.print(angleAccY, 1);   Serial.print(" , ");
	Serial.print(angleGyroX, 1);   Serial.print(" , ");
	//Serial.print(angleGyroZ, 1);   Serial.print(" , ");
	Serial.println(" ");
}
void PrintAngleCompFiltr()
{
	//Serial.print("AngleCompFilter -> ");
	Serial.print(angleCompX,2);   Serial.print(" , ");
	Serial.print(BNO055_EulerAngles.x,2);   Serial.print(" , ");

	Serial.print(angleCompY,2);   Serial.print(" , ");
	Serial.print(BNO055_EulerAngles.y,2);   Serial.print(" , ");
	Serial.println(" ");

}
void PrintAngleTest()
{
	//Serial.print(angleAccX);   Serial.print(" , ");
	//Serial.print(angleGyroY);   Serial.print(" , ");
	//Serial.print(angleCompX);   Serial.print(" , ");
	//Serial.print(BNO055_EulerAngles.x);   Serial.print(" , ");

	//Serial.print(angleAccY);   Serial.print(" , ");
	//Serial.print(angleGyroX);   Serial.print(" , ");
	//Serial.print(angleCompY);   Serial.print(" , ");
	//Serial.print(BNO055_EulerAngles.y);   Serial.print(" , ");


	Serial.print(magD);   Serial.print(" , ");
	Serial.print(angleGyroZ);   Serial.print(" , ");
	Serial.print(angleCompZ);   Serial.print(" , ");
	Serial.print(BNO055_EulerAngles.z);   Serial.print(" , ");
	Serial.print(BNO055_EulerAngles.my_z);   Serial.print(" , ");
	Serial.print(BNO055_EulerAngles.delta_z);   Serial.print(" , ");



	Serial.println(" , ");

}


void PrintMag()
{
	Serial.print("Magnetrometr -> ");
	Serial.print(magX);   Serial.print(" , ");
	Serial.print(magY);   Serial.print(" , ");
	Serial.print(magZ);   Serial.print(" , ");
	Serial.print(magD);   Serial.print(" , ");
	Serial.println(" , ");
}

void PrintDirection()
{
	Serial.print(" Dir-> ");
	Serial.print(magD);   	  Serial.print(" ");

	//Serial.print(gyroZ);   	  Serial.print(" , ");
	Serial.print(angleGyroZ, 4);   Serial.print(" ");

	//Serial.print(myAHRS.compAngleZ, 0);

	Serial.println(" ");
}

void PrintZ()
{
	Serial.print(magD, 0);   Serial.print(" , ");
	Serial.print(angleGyroZ, 0);   Serial.print(" , ");
	Serial.println(" , ");
}

void PrintMotor()
{
	Serial.print(" L_PWM : "); Serial.print(MotorL.motor_pwm);	
	Serial.print(" L_RPM : "); Serial.print(MotorL.motor_rpm);
	Serial.print(" L_SPEED : "); Serial.print(MotorL.motor_speed,4);
	Serial.print(" L_WAY : "); Serial.print(MotorL.motor_way, 4);

	Serial.println(" ");
	Serial.print(" R_PWM : "); Serial.print(MotorR.motor_pwm);
	Serial.print(" R_RPM : "); Serial.print(MotorR.motor_rpm);
	Serial.print(" R_SPEED : "); Serial.print(MotorR.motor_speed, 4);
	Serial.print(" R_WAY : "); Serial.print(MotorR.motor_way, 4);


	Serial.print(" angleGyroZ : "); Serial.print(angleGyroZ, 4);
	//Serial.print(" myAHRS.compAngleZ : "); Serial.print(myAHRS.compAngleZ, 0);

	//Serial.println("");
	Serial.println("**********************************************");

	//Serial.println("---");

	//Serial.print(MotorL.motor_speed, 4);Serial.print(" "); Serial.println(MotorR.motor_speed, 4);

	
	//delay(20);
}

void Output_to_Slave()		   // Функция вывода данных на вторую ардуино порциями
{
	static byte porcia = 0;
	switch (porcia)
	{
		case 0:
		{
			Write16_I2C(arduino_SLAVE, 0xB4, Datchik_L_Pered.Distancia);	//Передаем во во вторую ардуину данны для вывода на экранчики
			Write16_I2C(arduino_SLAVE, 0xB5, Datchik_R_Pered.Distancia);
			porcia = 1;
		}	break;
		case 1:
		{
			Write16_I2C(arduino_SLAVE, 0xB6, Datchik_L_Zad.Distancia);
			Write16_I2C(arduino_SLAVE, 0xB7, Datchik_R_Zad.Distancia);
			porcia = 2;
		}	break;
		case 2:
		{
			if (MotorL.motor_napravlenie == 1)				  // Учитывается как установлены
			{
				Write32F_I2C(arduino_SLAVE, 0xF1, MotorL.motor_speed);
			}
			else
			{
				Write32F_I2C(arduino_SLAVE, 0xF1, -MotorL.motor_speed);
			}

			if (MotorR.motor_napravlenie == 0)				  // Учитывается как установлены
			{
				Write32F_I2C(arduino_SLAVE, 0xF2, MotorR.motor_speed);
			}
			else
			{
				Write32F_I2C(arduino_SLAVE, 0xF2, -MotorR.motor_speed);
			}
			porcia = 3;

		}	break;
		case 3:
		{
			Write32F_I2C(arduino_SLAVE, 0xF3, MotorL.motor_way);
			Write32F_I2C(arduino_SLAVE, 0xF4, MotorR.motor_way);
			porcia = 4;
		}	break;
		case 4:
		{
			Write32F_I2C(arduino_SLAVE, 0xF5, TemperatureBME280);
			Write32F_I2C(arduino_SLAVE, 0xF6, PressureBME280_mm);

			porcia = 5;
		}	break;
		case 5:
		{
			Write16_I2C(arduino_SLAVE, 0xB8, Datchik_L_Pered_Verh);	//Передаем во во вторую ардуину данны для вывода на экранчики
			Write16_I2C(arduino_SLAVE, 0xB9, Datchik_R_Pered_Verh);
			porcia = 6;
		}	break;
		case 6:
		{
			Write16_I2C(arduino_SLAVE, 0xBA, Datchik_L_Zad_Verh);	//Передаем во во вторую ардуину данны для вывода на экранчики
			Write16_I2C(arduino_SLAVE, 0xBB, Datchik_R_Zad_Verh);
			porcia = 7;
		}	break;
		case 7:
		{
			Write8_I2C(arduino_SLAVE, 0xA1, StatusLineTekuFront);	//Передаем во во вторую ардуину данны для вывода на экранчики
			Write8_I2C(arduino_SLAVE, 0xA2, StatusLineTekuRear);
			porcia = 8;
		}	break;
		case 8:
		{
			porcia = 9;
		}	break;
		case 9:
		{
			flag_Arduino_Slave = false;
			porcia = 0;
		}	break;
	}
}

void Print_to_Serial2()		   // Функция вывода данных в порт чреез блютуз	порциями
{
	static byte porcia = 0;
	switch (porcia)
	{
	case 0:
	{
		Serial2.print(" MLs "); 		Serial2.print(MotorL.motor_speed);
		porcia = 1;
	}	break;
	case 1:
	{
		Serial2.print(" MRs "); 		Serial2.print(MotorR.motor_speed);
		porcia = 2;
	}	break;
	case 2:
	{
		Serial2.print(" MLw "); 	    Serial2.print(MotorL.motor_way);
		porcia = 3;
	}	break;
	case 3:
	{
		Serial2.print(" MRw "); 	   	Serial2.print(MotorR.motor_way);
		porcia = 4;
	}	break;
	case 4:
	{
		Serial2.print(" LPd "); 		Serial2.print(Datchik_L_Pered.Distancia);
		porcia = 5;
	}	break;
	case 5:
	{
		Serial2.print(" RPd "); 		Serial2.print(Datchik_R_Pered.Distancia);
		porcia = 6;
	}	break;
	case 6:
	{
		Serial2.print(" LZd "); 		Serial2.print(Datchik_L_Zad.Distancia);
		porcia = 7;
	}	break;
	case 7:
	{
		Serial2.print(" RZd "); 		Serial2.print(Datchik_R_Zad.Distancia);
		porcia = 8;
	}	break;
	case 8:
	{
		Serial2.print("  MLn "); 		Serial2.print(MotorL.motor_napravlenie);
		porcia = 9;
	}	break;
	case 9:
	{
		Serial2.print(" MRn "); 		Serial2.print(MotorR.motor_napravlenie);
		Serial2.println(" ");
		flag_print = false;
		porcia = 0;
	}	break;
	}
}


void Loop_to_Slave()
{
	if (flag_Arduino_Slave == true)
	{
		//digitalWrite(49, 1);
		//long a = micros();
		Output_to_Slave();		 //Вывод данных на вторую ардуино порциями	 0.5 МИЛЛИСЕКУНДЫ
		//long b = micros();
		//Serial.print("Time Output_to_Slave = "); 	 Serial.println(b - a);

	}
}
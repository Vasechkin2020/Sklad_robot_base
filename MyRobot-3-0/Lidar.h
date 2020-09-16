
const byte X_koordin = 50;     // Размерность масисива окружающего пространства
const byte Y_koordin = 50;     // Размерность масисива окружающего пространства
byte Lidar[X_koordin][Y_koordin];		 // Карта окружающего пространства

int glubina_rascheta = 250;        // На какую длинну по оси Y расчитываем координаты
byte shag_rascheta = 6;                // С каким шагом считаем 
float shag_setki = 1./10.;                   // шаг сетки с квадратами

void Init_Lidar()				   // Заполняем весь массив единичками, типа все занято. Нулями будем стирать свободное пространство
{
	Serial.println("Init_Lidar:");
	for (byte x = 0; x < X_koordin; x++)
	{
		for (byte y = 0; y < Y_koordin; y++)
		{
			Lidar[x][y] = 1;
		}
	}
}

void Print_Lidar()
{
	for (byte y = Y_koordin; y > 0 ; y--)
	{
		for (byte x = 0; x < X_koordin; x++)
		{
			Serial.print(Lidar[x][y]);
			//Serial.print(" ");
		}
		Serial.println("");
	}
}
void Raschet_SinusCosinus(Struct_Datchik& Datchik)
{
	float sinus = 0, cosinus = 0;
	byte variant_rascheta = 0;

	Datchik.Angle = Datchik.Angle_Ustanovki_in_0 - Servo_Array[Datchik.id_servo].angle_position;	// Вычисляем угол по которому делали измерение
	//Datchik.Angle = 15;

	//Serial.print(" Servo_Angel: ");	 Serial.print(Servo_Array[Datchik.id_servo].angle_position);
	//Serial.print(" Datchik.Angle: ");	 Serial.print(Datchik.Angle);
	//Serial.print(" Datchik.Distancia: ");	 Serial.println(Datchik.Distancia);

	float angle_ = Datchik.Angle;
	//==========================================================================
	if (angle_ >= 0 && angle_ <= 45)	//Считаем по оси Y
	{
		variant_rascheta = 1;
	}
	if (angle_ > 45 && angle_ < 90)		 // Меняем угол и считаем наоборот по оси Х
	{
		variant_rascheta = 2;
		angle_ = 90 - angle_;
	}
	if (angle_ > 90 && angle_ <= 135)		 // Меняем угол и считаем наоборот по оси Х
	{
		variant_rascheta = 7;
		angle_ = angle_ - 90;
	}
	if (angle_ == 90)		 // 
	{
		variant_rascheta = 3;
	}
	//==========================================================================
	if (angle_ >= -45 && angle_ < 0)	//Считаем по оси Y
	{
		variant_rascheta = 5;
	}
	if (angle_ > -90 && angle_ < -45)	//Считаем по оси x
	{
		variant_rascheta = 6;
		angle_ = 90 + angle_;
	}
	if (angle_ == -90)		 // 
	{
		variant_rascheta = 4;
	}
	if (angle_ >= -135 && angle_ < -90)	//Считаем по оси x
	{
		variant_rascheta = 8;
		angle_ = -angle_ - 90;
	}

	//Serial.print(" angle_: ");	 Serial.println(angle_);
	int angle_abs = abs(angle_);
	switch (angle_abs)
	{
		case 0: { sinus = 0;                       cosinus = 1; }	                        	break;
		case 3: { sinus = sin(3 * DEG_TO_RAD);     cosinus = 1. / cos(3 * DEG_TO_RAD); }		break;
		case 5: { sinus = sin(5 * DEG_TO_RAD);     cosinus = 1. / cos(5 * DEG_TO_RAD); }		break;
		case 6: { sinus = sin(6 * DEG_TO_RAD);     cosinus = 1. / cos(6 * DEG_TO_RAD); }		break;
		case 9: { sinus = sin(9 * DEG_TO_RAD);     cosinus = 1. / cos(9 * DEG_TO_RAD); }		break;
		case 10: { sinus = sin(10 * DEG_TO_RAD);   cosinus = 1. / cos(10 * DEG_TO_RAD); }   	break;
		case 12: { sinus = sin(12 * DEG_TO_RAD);   cosinus = 1. / cos(12 * DEG_TO_RAD); }		break;
		case 15: { sinus = sin(15 * DEG_TO_RAD);   cosinus = 1. / cos(15 * DEG_TO_RAD); }		break;
		case 18: { sinus = sin(18 * DEG_TO_RAD);   cosinus = 1. / cos(18 * DEG_TO_RAD); }		break;
		case 20: { sinus = sin(20 * DEG_TO_RAD);   cosinus = 1. / cos(20 * DEG_TO_RAD); }		break;
		case 21: { sinus = sin(21 * DEG_TO_RAD);   cosinus = 1. / cos(21 * DEG_TO_RAD); }		break;
		case 24: { sinus = sin(24 * DEG_TO_RAD);   cosinus = 1. / cos(24 * DEG_TO_RAD); }		break;
		case 25: { sinus = sin(25 * DEG_TO_RAD);   cosinus = 1. / cos(25 * DEG_TO_RAD); }		break;
		case 27: { sinus = sin(27 * DEG_TO_RAD);   cosinus = 1. / cos(27 * DEG_TO_RAD); }		break;
		case 30: { sinus = sin(30 * DEG_TO_RAD);   cosinus = 1. / cos(30 * DEG_TO_RAD); }	    break;
		case 33: { sinus = sin(33 * DEG_TO_RAD);   cosinus = 1. / cos(33 * DEG_TO_RAD); }		break;
		case 35: { sinus = sin(35 * DEG_TO_RAD);   cosinus = 1. / cos(35 * DEG_TO_RAD); }		break;
		case 36: { sinus = sin(36 * DEG_TO_RAD);   cosinus = 1. / cos(36 * DEG_TO_RAD); }		break;
		case 39: { sinus = sin(39 * DEG_TO_RAD);   cosinus = 1. / cos(39 * DEG_TO_RAD); }		break;
		case 40: { sinus = sin(40 * DEG_TO_RAD);   cosinus = 1. / cos(40 * DEG_TO_RAD); }		break;
		case 42: { sinus = sin(42 * DEG_TO_RAD);   cosinus = 1. / cos(42 * DEG_TO_RAD); }		break;
		case 45: { sinus = sin(45 * DEG_TO_RAD);   cosinus = 1. / cos(45 * DEG_TO_RAD); }		break;
	}


	//if (angle_abs == 0) { sinus = 0;        cosinus = 1; }
	//if (angle_abs == 3) { sinus = sin(3 * DEG_TO_RAD);   cosinus = 1. / cos(3 * DEG_TO_RAD); }
	//if (angle_abs == 5) { sinus = sin(5 * DEG_TO_RAD);   cosinus = 1. / cos(5 * DEG_TO_RAD); }
	//if (angle_abs == 6) { sinus = sin(6 * DEG_TO_RAD);   cosinus = 1. / cos(6 * DEG_TO_RAD); }
	//if (angle_abs == 9) { sinus = sin(9 * DEG_TO_RAD);   cosinus = 1. / cos(9 * DEG_TO_RAD); }
	//if (angle_abs == 10) { sinus = sin(10 * DEG_TO_RAD);  cosinus = 1. / cos(10 * DEG_TO_RAD); }
	//if (angle_abs == 12) { sinus = sin(12 * DEG_TO_RAD);   cosinus = 1. / cos(12 * DEG_TO_RAD); }
	//if (angle_abs == 15) { sinus = sin(15 * DEG_TO_RAD);  cosinus = 1. / cos(15 * DEG_TO_RAD); }
	//if (angle_abs == 18) { sinus = sin(18 * DEG_TO_RAD);   cosinus = 1. / cos(18 * DEG_TO_RAD); }
	//if (angle_abs == 20) { sinus = sin(20 * DEG_TO_RAD);  cosinus = 1. / cos(20 * DEG_TO_RAD); }
	//if (angle_abs == 21) { sinus = sin(21 * DEG_TO_RAD);   cosinus = 1. / cos(21 * DEG_TO_RAD); }
	//if (angle_abs == 24) { sinus = sin(24 * DEG_TO_RAD);   cosinus = 1. / cos(24 * DEG_TO_RAD); }
	//if (angle_abs == 25) { sinus = sin(25 * DEG_TO_RAD);  cosinus = 1. / cos(25 * DEG_TO_RAD); }
	//if (angle_abs == 27) { sinus = sin(27 * DEG_TO_RAD);   cosinus = 1. / cos(27 * DEG_TO_RAD); }
	//if (angle_abs == 30) { sinus = sin(30 * DEG_TO_RAD);  cosinus = 1. / cos(30 * DEG_TO_RAD); }
	//if (angle_abs == 33) { sinus = sin(33 * DEG_TO_RAD);   cosinus = 1. / cos(33 * DEG_TO_RAD); }
	//if (angle_abs == 35) { sinus = sin(35 * DEG_TO_RAD);  cosinus = 1. / cos(35 * DEG_TO_RAD); }
	//if (angle_abs == 36) { sinus = sin(36 * DEG_TO_RAD);   cosinus = 1. / cos(36 * DEG_TO_RAD); }
	//if (angle_abs == 39) { sinus = sin(39 * DEG_TO_RAD);   cosinus = 1. / cos(39 * DEG_TO_RAD); }
	//if (angle_abs == 40) { sinus = sin(40 * DEG_TO_RAD);  cosinus = 1. / cos(40 * DEG_TO_RAD); }
	//if (angle_abs == 42) { sinus = sin(42 * DEG_TO_RAD);   cosinus = 1. / cos(42 * DEG_TO_RAD); }
	//if (angle_abs == 45) { sinus = sin(45 * DEG_TO_RAD);  cosinus = 1. / cos(45 * DEG_TO_RAD); }

	Datchik.sinus = sinus;
	Datchik.cosinus = cosinus;
	Datchik.variant_rascheta = variant_rascheta;

	//Serial.print(" Datchik.sinus: ");	 Serial.println(Datchik.sinus);
	//Serial.print(" Datchik.cosinus: ");	 Serial.println(1/Datchik.cosinus);
	//Serial.print(" Datchik.variant_rascheta: ");	 Serial.println(Datchik.variant_rascheta);

}

void Raschet_Okrujenie(Struct_Datchik& Datchik_, int dlinna_kateta_)
{
	byte x2 = 0, y2 = 0;
	float x1 = 0, y1 = 0, gipotenuza = 0;   // Временные координаты для расчета

	//Serial.print(" Datchik.variant_rascheta2: ");	 Serial.println(Datchik_.variant_rascheta);

	switch (Datchik_.variant_rascheta)
	{
	case 1:
	{
		y1 = dlinna_kateta_;
		gipotenuza = y1 * Datchik_.cosinus;   // Находим длинну гипотенузы
		x1 = gipotenuza * Datchik_.sinus;     // Находим длинну катета	

	}	break;
	case 2:
	{
		x1 = dlinna_kateta_;
		gipotenuza = x1 * Datchik_.cosinus;   // Находим длинну гипотенузы
		y1 = gipotenuza * Datchik_.sinus;     // Находим длинну катета	
	}	break;
	case 3:
	{
		x1 = dlinna_kateta_;
		gipotenuza = dlinna_kateta_;
		y1 = 0;
	}	break;
	case 4:
	{
		x1 = -dlinna_kateta_;
		gipotenuza = dlinna_kateta_;
		y1 = 0;
	}	break;
	case 5:
	{
		y1 = dlinna_kateta_;
		gipotenuza = y1 * Datchik_.cosinus;   // Находим длинну гипотенузы
		x1 = gipotenuza * -Datchik_.sinus;     // Находим длинну катета	
	}	break;
	case 6:
	{
		x1 = -dlinna_kateta_;
		gipotenuza = dlinna_kateta_ * Datchik_.cosinus;   // Находим длинну гипотенузы
		y1 = gipotenuza * Datchik_.sinus;     // Находим длинну катета	
	}	break;
	case 7:
	{
		x1 = dlinna_kateta_;
		gipotenuza = x1 * Datchik_.cosinus;   // Находим длинну гипотенузы
		y1 = gipotenuza * -Datchik_.sinus;     // Находим длинну катета	
	}	break;
	case 8:
	{
		x1 = -dlinna_kateta_;
		gipotenuza = dlinna_kateta_ * Datchik_.cosinus;   // Находим длинну гипотенузы
		y1 = gipotenuza * -Datchik_.sinus;     // Находим длинну катета	
	}	break;
	}


	//Serial.print(" x1: ");	 Serial.print(x1);
	//Serial.print(" y1: ");	 Serial.print(y1);
	//Serial.print(" gipotenuza: ");	 Serial.print(gipotenuza);
	//Serial.print(" Datchik_.napravlenie: ");	 Serial.print(Datchik_.napravlenie);

	if (Datchik_.napravlenie == 1)		 // Это измеряем назад и поэтому значения уменьшаем
		{
			x2 = ((x1 + Datchik_.X_koordinata) * shag_setki);	  //Рассчитываем в какой квадрат попадаем с учетом положения датчика
			y2 = ((y1 + Datchik_.Y_koordinata) * shag_setki);	  //Рассчитываем в какой квадрат попадаем с учетом положения датчика
		}

	if (Datchik_.napravlenie == -1)		 // Это измеряем назад и поэтому значения уменьшаем
		{
			x2 = ((Datchik_.X_koordinata - x1) * shag_setki);	  //Рассчитываем в какой квадрат попадаем с учетом положения датчика
			y2 = ((Datchik_.Y_koordinata - y1) * shag_setki);	  //Рассчитываем в какой квадрат попадаем с учетом положения датчика
		}
	//Serial.print(" x0: ");	 Serial.print(Datchik_.X_koordinata - x1);
	//Serial.print(" y0: ");	 Serial.println(Datchik_.Y_koordinata - y1);

	//Serial.print(" x2: ");	 Serial.print(x2);
	//Serial.print(" y2: ");	 Serial.println(y2);
		
	if (x2 < X_koordin && y2 < Y_koordin)		   // Если не выходим за границы массива
	{
			//Serial.print(" x2: ");	 Serial.print(x2);
			//Serial.print(" y2: ");	 Serial.println(y2);
		if (gipotenuza < Datchik_.Distancia)	   // Если длина гипотенузы для этого треугольника меньше длинны измеренного растояния то присваиваем 0 - типа свободно 
		{
			Lidar[x2][y2] = 0;
				//Serial.println(" 0: ");
		}
		else							//   иначе присваиваиваем 1 - типа препятствие
		{
			Lidar[x2][y2] = 1;
				//Serial.println(" 1: ");
		}
	}
}

void Raschet_Kordinat(float X_kord_, float Y_kord_, float angle_, int distance_, byte napravlenie_)
{
	//long a = micros();
	//Serial.print(" Raschet_Kordinat: "); Serial.println(angle_);
	float sinus = 0, cosinus = 0;
	byte x2 = 0, y2 = 0;
	float x1 = 0 , y1 = 0, gipotenuza = 0;   // Временные координаты для расчета
	byte variant_rascheta = 0;
	//==========================================================================
	if (angle_ >= 0 && angle_ <= 45)	//Считаем по оси Y
	{
		variant_rascheta = 1;
	}
	if (angle_ > 45 && angle_ < 90)		 // Меняем угол и считаем наоборот по оси Х
	{
		variant_rascheta = 2;
		angle_ = 90 - angle_;	
	}
	if (angle_ > 90 && angle_ <= 135)		 // Меняем угол и считаем наоборот по оси Х
	{
		variant_rascheta = 7;
		angle_ = angle_ - 90;
	}
	if (angle_ == 90)		 // 
	{
		variant_rascheta = 3;
	}
	
	//==========================================================================

	if (angle_ >= -45 && angle_ < 0)	//Считаем по оси Y
	{
		variant_rascheta = 5;
	}
	if (angle_ > -90 && angle_ < -45)	//Считаем по оси x
	{
		variant_rascheta = 6;
		angle_ = 90 + angle_;
	}
 	if (angle_ == -90)		 // 
	{
		variant_rascheta = 4;
	}	
	if (angle_ >= -135 && angle_ < -90)	//Считаем по оси x
	{
		variant_rascheta = 8;
		angle_ = 135 + angle_;
	}

	//Serial.print(" angle_: ");	 Serial.println(angle_);
	if (abs(angle_) == 0)    { sinus = 0;        cosinus = 1;}
	if (abs(angle_) == 5)   { sinus = sin(5 * DEG_TO_RAD);     cosinus = 1./cos(5 * DEG_TO_RAD); }
	if (abs(angle_) == 7.5) { sinus = sin(7.5 * DEG_TO_RAD);   cosinus = 1./cos(7.5 * DEG_TO_RAD); }
	if (abs(angle_) == 10)  { sinus = sin(10 * DEG_TO_RAD);    cosinus = 1./cos(10 * DEG_TO_RAD); }
	if (abs(angle_) == 15)  { sinus = sin(15 * DEG_TO_RAD);    cosinus = 1./cos(15 * DEG_TO_RAD); }
	if (abs(angle_) == 20)  { sinus = sin(20 * DEG_TO_RAD);    cosinus = 1./cos(20 * DEG_TO_RAD); }
	if (abs(angle_) == 22.5) { sinus = sin(22.5 * DEG_TO_RAD); cosinus = 1./cos(22.5 * DEG_TO_RAD); }
	if (abs(angle_) == 25)  { sinus = sin(25 * DEG_TO_RAD);    cosinus = 1./cos(25 * DEG_TO_RAD); }
	if (abs(angle_) == 30)  { sinus = sin(30 * DEG_TO_RAD);    cosinus = 1./cos(30 * DEG_TO_RAD); }
	if (abs(angle_) == 35)  { sinus = sin(35 * DEG_TO_RAD);    cosinus = 1./cos(35 * DEG_TO_RAD); }
	if (abs(angle_) == 37.5) { sinus = sin(37.5 * DEG_TO_RAD); cosinus = 1./cos(37.5 * DEG_TO_RAD); }
	if (abs(angle_) == 40)  { sinus = sin(40 * DEG_TO_RAD);    cosinus = 1./cos(40 * DEG_TO_RAD); }
	if (abs(angle_) == 45)  { sinus = sin(45 * DEG_TO_RAD);    cosinus = 1./cos(45 * DEG_TO_RAD); }

	//Serial.print(" variant_rascheta: ");	 Serial.println(variant_rascheta);

	//long b = micros();
	//Serial.print(" Time variant_rascheta: ");	 Serial.println(b-a);

	
	for (int i = 0; i <= glubina_rascheta; i = i + shag_rascheta)			 // Расчитываем для каждой точки координаты с заданым шагом по оси Y
	{
		switch (variant_rascheta)
		{
			case 1:
			{
				y1 = i;
				gipotenuza = y1 * cosinus;   // Находим длинну гипотенузы
				x1 = gipotenuza * sinus;     // Находим длинну катета	
			}	break;
			case 2:
			{
				x1 = i;
				gipotenuza = x1 * cosinus;   // Находим длинну гипотенузы
				y1 = gipotenuza * sinus;     // Находим длинну катета	
			}	break;
			case 3:
			{
				x1 = i;
				y1 = 0;
			}	break;
			case 4:
			{
				x1 = -i;
				y1 = 0;
			}	break;
			case 5:
			{
				y1 = i;
				gipotenuza = y1 * cosinus;   // Находим длинну гипотенузы
				x1 = gipotenuza * -sinus;     // Находим длинну катета	
			}	break;
			case 6:
			{
				x1 = -i;
				gipotenuza = i * cosinus;   // Находим длинну гипотенузы
				y1 = gipotenuza * sinus;     // Находим длинну катета	
			}	break;
			case 7:
			{
				x1 = i;
				gipotenuza = x1 * cosinus;   // Находим длинну гипотенузы
				y1 = gipotenuza * -sinus;     // Находим длинну катета	
			}	break;
			case 8:
			{
				x1 = -i;
				gipotenuza = i * cosinus;   // Находим длинну гипотенузы
				y1 = gipotenuza * -sinus;     // Находим длинну катета	
			}	break;
		}


		//Serial.print(" x: ");	 Serial.print(x1);
		//Serial.print(" y: ");	 Serial.print(y1);
		//Serial.print(" gipotenuza: ");	 Serial.print(gipotenuza);

		if (napravlenie_ == 1)			// Это измеряем вперед и поэтому значения увеличиваем
		{
			x2 = ( (x1 + X_kord_) * shag_setki) ;	  //Рассчитываем в какой квадрат попадаем с учетом положения датчика
			y2 = ( (y1 + Y_kord_) * shag_setki) ;	  //Рассчитываем в какой квадрат попадаем с учетом положения датчика
		}
		//if (napravlenie_ == -1)		 // Это измеряем назад и поэтому значения уменьшаем
		//{
		//	x2 = ( (x1 - X_kord_) * shag_setki);	  //Рассчитываем в какой квадрат попадаем с учетом положения датчика
		//	y2 = ( (y1 - Y_kord_) * shag_setki);	  //Рассчитываем в какой квадрат попадаем с учетом положения датчика
		//}

		//Serial.print(" x2: ");	 Serial.print(x2);
		//Serial.print(" y2: ");	 Serial.println(y2);


		if (x2 < X_koordin && y2 < Y_koordin)		   // Если не выходим за границы массива
		{
		//	Serial.print(" x2: ");	 Serial.print(x2);
		//	Serial.print(" y2: ");	 Serial.print(y2);
			if (gipotenuza < distance_)	   // Если длина гипотенузы для этого треугольника меньше длинны измеренного растояния то присваиваем 0 - типа свободно 
			{
				Lidar[x2][y2] = 0;
			//	Serial.println(" 0: ");
			}
			else							//   иначе присваиваиваем 1 - типа препятствие
			{
				Lidar[x2][y2] = 1;
			//	Serial.println(" 1: ");
			}
		}
	}


}


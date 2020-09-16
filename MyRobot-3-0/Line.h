
  //   -----------------  Переменные для датчика линии переднего и заднего
byte LineFront_1, LineFront_2, LineFront_3, LineFront_4;
byte LineRear_1, LineRear_2, LineRear_3, LineRear_4;

byte Pin_LineFront_1 = A4;      // Пин 27 для первого датчика линии
byte Pin_LineFront_2 = A5;
byte Pin_LineFront_3 = A6;
byte Pin_LineFront_4 = A7;
byte Pin_LineRear_1 = A3;
byte Pin_LineRear_2 = A2;
byte Pin_LineRear_3 = A1;
byte Pin_LineRear_4 = A0;

int StatusLine = 0;   // Статус в текущий момент
byte StatusLinePredFront, StatusLinePredRear = 0;       // Предыдущий статус. 
byte StatusLineTekuFront, StatusLineTekuRear = 0;		// Текущий статус. 
int porog_line = 500;     // Пороговое значение для аналогового переключения 

void Init_Line()
{
	pinMode(Pin_LineFront_1, INPUT);
	pinMode(Pin_LineFront_2, INPUT);
	pinMode(Pin_LineFront_3, INPUT);
	pinMode(Pin_LineFront_4, INPUT);
	pinMode(Pin_LineRear_1, INPUT);
	pinMode(Pin_LineRear_2, INPUT);
	pinMode(Pin_LineRear_3, INPUT);
	pinMode(Pin_LineRear_4, INPUT);


	for (byte i = 0; i < 10; i++)
	{
		Serial.print(analogRead(Pin_LineFront_1)); Serial.print(" / ");
		Serial.print(analogRead(Pin_LineFront_2)); Serial.print(" / ");
		Serial.print(analogRead(Pin_LineFront_3)); Serial.print(" / ");
		Serial.print(analogRead(Pin_LineFront_4)); Serial.print(" //  ");
		Serial.print(analogRead(Pin_LineRear_1)); Serial.print(" / ");
		Serial.print(analogRead(Pin_LineRear_2)); Serial.print(" / ");
		Serial.print(analogRead(Pin_LineRear_3)); Serial.print(" / ");
		Serial.print(analogRead(Pin_LineRear_4)); Serial.print(" / ");
		Serial.println(" ");
		delay(50);

	}

	Serial.print(" End InitLine. Time:  ");  Serial.println(millis());

}


void ReadDataLinia()              //   Считываем данные с пинов	   ВРЕМЯ ИСПОЛНЕНИЯ 1 МИЛИСЕКУНДА
{
	if (analogRead(Pin_LineFront_1) > 600)  LineFront_1 = 1; 	else  LineFront_1 = 0; 
	if (analogRead(Pin_LineFront_2) > 600)  LineFront_2 = 1;  	else  LineFront_2 = 0; 
	if (analogRead(Pin_LineFront_3) > 600)  LineFront_3 = 1; 	else  LineFront_3 = 0; 
	if (analogRead(Pin_LineFront_4) > 600)  LineFront_4 = 1; 	else  LineFront_4 = 0; 
	if (analogRead(Pin_LineRear_1) > 600)   LineRear_1 = 1; 	else  LineRear_1 = 0; 
	if (analogRead(Pin_LineRear_2) > 600)   LineRear_2 = 1; 	else  LineRear_2 = 0; 
	if (analogRead(Pin_LineRear_3) > 600)   LineRear_3 = 1; 	else  LineRear_3 = 0; 
	if (analogRead(Pin_LineRear_4) > 600)   LineRear_4 = 1; 	else  LineRear_4 = 0; 


	//Serial.print(" Time LineFront_1 : "); Serial.print(LineFront_1);  Serial.print(" Time : "); Serial.println(analogRead(Pin_LineFront_1));
	//Serial.print(" Time LineFront_2 : "); Serial.print(LineFront_2);  Serial.print(" Time : "); Serial.println(analogRead(Pin_LineFront_2));
	//Serial.print(" Time LineFront_3 : "); Serial.print(LineFront_3);  Serial.print(" Time : "); Serial.println(analogRead(Pin_LineFront_3));
	//Serial.print(" Time LineFront_4 : "); Serial.print(LineFront_4);  Serial.print(" Time : "); Serial.println(analogRead(Pin_LineFront_4));

	if (LineFront_1 == 0 && LineFront_2 == 0 && LineFront_3 == 0 && LineFront_4 == 0) StatusLineTekuFront = 0b0000;   //Статус 0 "Нет линии"
	if (LineFront_1 == 0 && LineFront_2 == 1 && LineFront_3 == 1 && LineFront_4 == 0) StatusLineTekuFront = 0b0110;   //Статус 0 "Линия посередине"
	if (LineFront_1 == 1 && LineFront_2 == 1 && LineFront_3 == 1 && LineFront_4 == 1) StatusLineTekuFront = 0b1111;   //Статус 0 "Пересекаем линию"
	if (LineFront_1 == 1 && LineFront_2 == 1 && LineFront_3 == 1 && LineFront_4 == 0) StatusLineTekuFront = 0b1110;   //Статус	  Поворот НАЛЕВО
	if (LineFront_1 == 0 && LineFront_2 == 1 && LineFront_3 == 1 && LineFront_4 == 1) StatusLineTekuFront = 0b0111;   //Статус 	  Поворот НАПРАВО

	if (LineFront_1 == 0 && LineFront_2 == 1 && LineFront_3 == 0 && LineFront_4 == 0) StatusLineTekuFront = 0b0100;   //Статус 0 "Смещаемся ВПРАВО"
	if (LineFront_1 == 1 && LineFront_2 == 1 && LineFront_3 == 0 && LineFront_4 == 0) StatusLineTekuFront = 0b1100;   //Статус 0 "Смещаемся ВПРАВО СИЛЬНО"
	if (LineFront_1 == 1 && LineFront_2 == 0 && LineFront_3 == 0 && LineFront_4 == 0) StatusLineTekuFront = 0b1000;   //Статус 0 "Смещаемся ВПРАВО СИЛЬНО СИЛЬНО"


	if (LineFront_1 == 0 && LineFront_2 == 0 && LineFront_3 == 1 && LineFront_4 == 0) StatusLineTekuFront = 0b0010;   //Статус 0 "Смещаемся ВЛЕВО"
	if (LineFront_1 == 0 && LineFront_2 == 0 && LineFront_3 == 1 && LineFront_4 == 1) StatusLineTekuFront = 0b0011;   //Статус 0 "Смещаемся ВЛЕВО СИЛЬНО"
	if (LineFront_1 == 0 && LineFront_2 == 0 && LineFront_3 == 0 && LineFront_4 == 1) StatusLineTekuFront = 0b0001;   //Статус 0 "Смещаемся ВЛЕВО СИЛЬНО СИЛЬНО"

	//---------------------------------------------------------------------------------------------------------------------------------------------

	if (LineRear_1 == 0 && LineRear_2 == 0 && LineRear_3 == 0 && LineRear_4 == 0) StatusLineTekuRear = 0b0000;   //Статус 0 "Нет линии"
	if (LineRear_1 == 0 && LineRear_2 == 1 && LineRear_3 == 1 && LineRear_4 == 0) StatusLineTekuRear = 0b0110;   //Статус 0 "Линия посередине"
	if (LineRear_1 == 1 && LineRear_2 == 1 && LineRear_3 == 1 && LineRear_4 == 1) StatusLineTekuRear = 0b1111;   //Статус 0 "Пересекаем линию"
	if (LineRear_1 == 1 && LineRear_2 == 1 && LineRear_3 == 1 && LineRear_4 == 0) StatusLineTekuRear = 0b1110;   //Статус Поворот НАЛЕВО
	if (LineRear_1 == 0 && LineRear_2 == 1 && LineRear_3 == 1 && LineRear_4 == 1) StatusLineTekuRear = 0b0111;   //Статус Поворот НАПРАВО


	if (LineRear_1 == 0 && LineRear_2 == 1 && LineRear_3 == 0 && LineRear_4 == 0) StatusLineTekuRear = 0b0100;   //Статус 0 "Смещаемся ВПРАВО"
	if (LineRear_1 == 1 && LineRear_2 == 1 && LineRear_3 == 0 && LineRear_4 == 0) StatusLineTekuRear = 0b1100;   //Статус 0 "Смещаемся ВПРАВО СИЛЬНО"
	if (LineRear_1 == 1 && LineRear_2 == 0 && LineRear_3 == 0 && LineRear_4 == 0) StatusLineTekuRear = 0b1000;   //Статус 0 "Смещаемся ВПРАВО СИЛЬНО"

	if (LineRear_1 == 0 && LineRear_2 == 0 && LineRear_3 == 1 && LineRear_4 == 0) StatusLineTekuRear = 0b0010;   //Статус 0 "Смещаемся ВЛЕВО"
	if (LineRear_1 == 0 && LineRear_2 == 0 && LineRear_3 == 1 && LineRear_4 == 1) StatusLineTekuRear = 0b0011;   //Статус 0 "Смещаемся ВЛЕВО СИЛЬНО"
	if (LineRear_1 == 0 && LineRear_2 == 0 && LineRear_3 == 0 && LineRear_4 == 1) StatusLineTekuRear = 0b0001;   //Статус 0 "Смещаемся ВЛЕВО СИЛЬНО СИЛЬНО"


}
void PrintLine()
{
	Serial2.print(" LineFront = ");Serial2.print (LineFront_1 );
	Serial2.print(" ");Serial2.print(LineFront_2);
	Serial2.print(" ");Serial2.print(LineFront_3);
	Serial2.print(" ");Serial2.print(LineFront_4);
	Serial2.print(" / ");Serial2.print(StatusLineTekuFront, BIN);


	Serial2.print("   LineRear = ");Serial2.print(LineRear_1);
	Serial2.print(" ");Serial2.print(LineRear_2);
	Serial2.print(" ");Serial2.print(LineRear_3);
	Serial2.print(" ");Serial2.print(LineRear_4);
	Serial2.print(" / ");Serial2.println(StatusLineTekuRear,BIN);

}

//void LineStatus()      // Определяем в каком статусе относительно линии находится машинка
//{
//
//
//	//if (StatusLineTeku != StatusLine)   //Если произошла смена статуса
//	//{
//	//	StatusLinePred = StatusLine;
//	//	StatusLine = StatusLineTeku;
//	//	StatusKursa = 0;
//
//	//}
//
//	//Serial.print(" LineStatusPred "); Serial.print(StatusLinePred);
//	//Serial.print(" LineStatus "); Serial.print(StatusLine);
//
//
//}
void Loop_Line()
{

	//==========================================================
	if (flag_Line == true)
	{
		flag_Line = false;
		ReadDataLinia();						  // ВРЕМЯ ИСПОЛНЕНИЯ  1  МИЛИСЕКУНДА // Функция опроса датчиков линии
		//PrintLine();
		//Serial.print("Read_Encoder "); 
	}
}
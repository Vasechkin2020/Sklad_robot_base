
const int16_t Adr_EEPROM = 0x57; // I2C адрес микросхемы EEPROM




// Карта регистров     EEPROM
// internally organized as 256 pages of 32 bytes !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// 1 page memory
//  0 - Who i Am   должно быть 14
//  4-5  Значение положения шагового двигателя

// 2 page memory

//	32 - Признак наличия поправочных коэффициентов Аксерометра и гироскопа
//  34-37 - ax offset  - 4 байта под float
//  38-41 - ay offset  - 4 байта под float
//  42-45 - az offset  - 4 байта под float
//  46-49 - gx offset  - 4 байта под float
//  50-53 - gy offset  - 4 байта под float
//  54-57 - gz offset  - 4 байта под float


// 3 page memory

//	64 - Признак наличия поравочных коэффициентов магнетрометра
//  66-69 - mx offset  - 4 байта под float
//  70-73 - my offset  - 4 байта под float
//  74-77 - mz offset  - 4 байта под float
//  78-81 - msx offset  - 4 байта под float
//  82-85 - msy offset  - 4 байта под float
//  86-89 - msz offset  - 4 байта под float

// 4 page memory Offset BNO055

//	96 - Признак наличия поравочных коэффициентов магнетрометра 
//  98-120 Пишем 22 байта всех поправочных коэфициентов для BNO055




void Eeprom_WriteByte(uint16_t registr, uint8_t data)
{
	Wire.beginTransmission(Adr_EEPROM);
	Wire.write((int)(registr >> 8)); // MSB
	Wire.write((int)(registr & 0xFF)); // LSB
	Wire.write(data);   //      Serial.print("byte: "); Serial.println(data,BIN); 
	Wire.endTransmission();
	delay(3);

}

void Eeprom_WriteWord(uint16_t registr, int data)
{
	Wire.beginTransmission(Adr_EEPROM);
	Wire.write((int)(registr >> 8)); // MSB
	Wire.write((int)(registr & 0xFF)); // LSB
	uint8_t *pB = (uint8_t *)(&data);
	uint8_t L_byte = pB[0];
	uint8_t H_byte = pB[1];
	Wire.write(L_byte);   //  Serial.print("L_byte: "); Serial.println(L_byte,BIN); 
	Wire.write(H_byte);   //  Serial.print("H_byte: "); Serial.println(H_byte,BIN); 
	Wire.endTransmission();	   
	delay(3);
}

void Eeprom_WriteLong(uint16_t registr, uint32_t data)
{
	Wire.beginTransmission(Adr_EEPROM);
	Wire.write((int)(registr >> 8)); // MSB
	Wire.write((int)(registr & 0xFF)); // LSB
	uint8_t *pB = (uint8_t *)(&data);
	uint8_t LL_byte = pB[0];
	uint8_t L_byte = pB[1];
	uint8_t H_byte = pB[2];
	uint8_t HH_byte = pB[3];
	Wire.write(LL_byte);     // Serial.print("LL_byte: "); Serial.println(LL_byte,BIN); 
	Wire.write(L_byte);    //  Serial.print("L_byte: "); Serial.println(L_byte,BIN); 
	Wire.write(H_byte);     // Serial.print("H_byte: "); Serial.println(H_byte,BIN); 
	Wire.write(HH_byte);   // Serial.print("HH_byte: "); Serial.println(HH_byte,BIN); 
	Wire.endTransmission();
	delay(3);
}

void Eeprom_WriteFloat(uint16_t registr, float data)
{
	Wire.beginTransmission(Adr_EEPROM);
	Wire.write((int)(registr >> 8)); // MSB
	Wire.write((int)(registr & 0xFF)); // LSB
	uint8_t *pB = (uint8_t *)(&data);
	uint8_t LL_byte = pB[0];
	uint8_t L_byte = pB[1];
	uint8_t H_byte = pB[2];
	uint8_t HH_byte = pB[3];
	Wire.write(LL_byte);   //   Serial.print("LL_byte: "); Serial.println(LL_byte,BIN); 
	Wire.write(L_byte);   //   Serial.print("L_byte: "); Serial.println(L_byte,BIN); 
	Wire.write(H_byte);   //   Serial.print("H_byte: "); Serial.println(H_byte,BIN); 
	Wire.write(HH_byte);  //  Serial.print("HH_byte: "); Serial.println(HH_byte,BIN); 
	Wire.endTransmission();
	delay(3);
}


//-------------------------------------------------------------------------------------------

uint8_t  Eeprom_ReadByte(uint16_t registr)
{
	Wire.beginTransmission(Adr_EEPROM);
	Wire.write((int)(registr >> 8)); // MSB
	Wire.write((int)(registr & 0xFF)); // LSB
	delay(1);
	Wire.endTransmission();
	Wire.requestFrom(Adr_EEPROM, 1);
	uint8_t data = Wire.read();             //read one byte of data
	//delay(1);
	//Serial.print("byte: "); Serial.println(data, BIN);
	return data;
}

int Eeprom_ReadWord(int16_t registr)
{
	uint8_t buffer[2];
	Wire.beginTransmission(Adr_EEPROM);
	Wire.write((int)(registr >> 8)); // MSB
	Wire.write((int)(registr & 0xFF)); // LSB
	delay(1);
	Wire.endTransmission();
	Wire.requestFrom(Adr_EEPROM, 2);
	buffer[0] = Wire.read();   //Serial.print("L_byte: "); Serial.println(buffer[0],BIN); 
	buffer[1] = Wire.read();   //Serial.print("H_byte: "); Serial.println(buffer[1],BIN); 
	//delay(1);
	int *pI = (int*)(&buffer);         // Serial.print("buffer: "); Serial.println(*pI); 
	return *pI;

	// uint8_t L_byte = Wire.read();           Serial.print("L_byte: "); Serial.println(L_byte,BIN);       //read one byte of data
	// uint8_t H_byte = Wire.read();           Serial.print("H_byte: "); Serial.println(H_byte,BIN);       //read one byte of data
	//return ((int16_t)H_byte << 8) | L_byte;;             // Сдвигаем старший байт влево и добавляем младший байт  // Возвращаем значение
}

uint32_t Eeprom_ReadLong(int16_t registr)
{
	uint8_t buffer[4];
	Wire.beginTransmission(Adr_EEPROM);
	Wire.write((int)(registr >> 8)); // MSB
	Wire.write((int)(registr & 0xFF)); // LSB
	delay(1);
	Wire.endTransmission();
	Wire.requestFrom(Adr_EEPROM, 4);
	buffer[0] = Wire.read();   //Serial.print("LL_byte: "); Serial.println(buffer[0],BIN); 
	buffer[1] = Wire.read();   //Serial.print("L_byte: "); Serial.println(buffer[1],BIN); 
	buffer[2] = Wire.read();   //Serial.print("H_byte: "); Serial.println(buffer[0],BIN); 
	buffer[3] = Wire.read();   //Serial.print("HH_byte: "); Serial.println(buffer[1],BIN); 
	//delay(1);
	uint32_t *pL = (uint32_t*)(&buffer);
	return *pL;

	//uint8_t LL_byte = Wire.read();         // Serial.print("LL_byte: "); Serial.println(LL_byte,BIN);      
	//uint8_t L_byte = Wire.read();          // Serial.print("L_byte: ");  Serial.println(L_byte,BIN); 
	//uint8_t H_byte = Wire.read();          // Serial.print("H_byte: ");  Serial.println(H_byte,BIN);
	//uint8_t HH_byte = Wire.read();         // Serial.print("HH_byte: "); Serial.println(HH_byte,BIN);     
	//return ((uint32_t)HH_byte<<24) | ((uint32_t)H_byte<<16) | ((uint32_t)L_byte<<8) | (uint32_t)LL_byte ;    // Сдвигаем старший байт влево и добавляем младший байт  // Возвращаем значение

}

float Eeprom_ReadFloat(int16_t registr)
{
	uint8_t buffer[4];
	Wire.beginTransmission(Adr_EEPROM);
	Wire.write((int)(registr >> 8)); // MSB
	Wire.write((int)(registr & 0xFF)); // LSB
	delay(1);
	Wire.endTransmission();
	Wire.requestFrom(Adr_EEPROM, 4);
	buffer[0] = Wire.read(); //  Serial.print("LL_byte: "); Serial.println(buffer[0],BIN); 
	buffer[1] = Wire.read(); //  Serial.print("L_byte: "); Serial.println(buffer[1],BIN); 
	buffer[2] = Wire.read(); //  Serial.print("H_byte: "); Serial.println(buffer[0],BIN); 
	buffer[3] = Wire.read(); //  Serial.print("HH_byte: "); Serial.println(buffer[1],BIN); 
	//delay(1);
	float *pL = (float*)(&buffer);
	return *pL;

	//uint8_t LL_byte = Wire.read();         // Serial.print("LL_byte: "); Serial.println(LL_byte,BIN);      
	//uint8_t L_byte = Wire.read();          // Serial.print("L_byte: ");  Serial.println(L_byte,BIN); 
	//uint8_t H_byte = Wire.read();          // Serial.print("H_byte: ");  Serial.println(H_byte,BIN);
	//uint8_t HH_byte = Wire.read();         // Serial.print("HH_byte: "); Serial.println(HH_byte,BIN);     
	//return ((uint32_t)HH_byte<<24) | ((uint32_t)H_byte<<16) | ((uint32_t)L_byte<<8) | (uint32_t)LL_byte ;    // Сдвигаем старший байт влево и добавляем младший байт  // Возвращаем значение

}


void Test_EEPROM()
{
	Serial.println(" EEPROM Start...");
	Serial2.println(" Test_EEPROM ");


	//Eeprom_WriteByte(0, 222);			   // Записыываем какое-нибудь уникальное значение для идентификации
	byte aaa = Eeprom_ReadByte(0);

	Serial.print("BIN: "); Serial.print(aaa, BIN); Serial.print(" DEC: ");  Serial.print(aaa, DEC);
	if (Eeprom_ReadByte(0) == 222)
	{
		Serial.println(" EEPROM Ok");
		Serial.println(" --------------------------------");

	}
	else
	{
		Serial.println(" EEPROM PROBLEM !!!");
	}
}

//Eeprom_WriteByte(8, 9);

//Eeprom_WriteByte(1, 13);   
//byte t = Eeprom_ReadByte(1);  
//Serial.print(" Eeprom_ReadByte  = ");	  	Serial.println(t);

//Eeprom_WriteWord(4, 61230);
//word tt = Eeprom_ReadWord(4);
//Serial.print(" Eeprom_ReadWord  = ");	  	Serial.println(tt);

//Eeprom_WriteLong(8, 999999);
//long ttt = Eeprom_ReadLong(8);
//Serial.print(" Eeprom_ReadLong  = ");	  	Serial.println(ttt);

//Eeprom_WriteFloat(16, 777.55);
//float tttt = Eeprom_ReadFloat(16);
//Serial.print(" Eeprom_ReadFloat  = ");	  	Serial.println(tttt);



// Ќастроки скорости чтени€ и запиши по шине SPI  set up the speed, data order and data mode
//SPISettings settings16(1000000, MSBFIRST, SPI_MODE3);
//SPISettings settings2(8000000, MSBFIRST, SPI_MODE3);


int16_t uint8ToUint16(uint8_t Hbyte, uint8_t Lbyte)   // »з двух байт получаем двухбайтное число. не перепутать старший и младший байт. так как бывает разна€ последовательность регистров
{
	return ((int16_t)Hbyte << 8) | Lbyte;
}


uint8_t ReadByte_SPI(uint8_t CS_PIN, int8_t registr)  // ”казываем на каком пине устройство и с какого регистра нужно прочитать данные
{
	//SPI.beginTransaction(settings2);		   // читаем со скоростью 8 MHz делитель 2
	digitalWrite(CS_PIN, LOW);
	registr = registr | (1 << 7);   //   READ(MSB=1) 0x80 or 0x75 -> 0xF5   // ƒобавл€ем старший бит так как это чтение
	SPI.transfer(registr);                    // ѕередаем из какого регистра будем читать
	uint8_t data = SPI.transfer(0);            //—читываем данные из регистра   // ¬озвращаем значение
	digitalWrite(CS_PIN, HIGH);
	SPI.endTransaction();
	return data;
}
uint16_t ReadWord_SPI(uint8_t CS_PIN, int8_t registr)  // ”казываем на каком пине устройство и с какого регистра нужно прочитать данные
{
	//SPI.beginTransaction(settings2);           // читаем со скоростью 8 MHz делитель 2
	digitalWrite(CS_PIN, LOW);
	registr = registr | (1 << 7);   //   READ(MSB=1) 0x80 or 0x75 -> 0xF5   // ƒобавл€ем старший бит так как это чтение
	SPI.transfer(registr);                    // ѕередаем из какого регистра будем читать
	uint8_t Hbyte = SPI.transfer(0);            //—читываем High byte данные из регистра
	uint8_t Lbyte = SPI.transfer(0);            //—читываем Low byte данные из регистра
	digitalWrite(CS_PIN, HIGH);
	SPI.endTransaction();
	return ((int16_t)Hbyte << 8) | Lbyte;;             // —двигаем старший байт влево и добавл€ем младший байт  // ¬озвращаем значение
}

void  WriteByte_I2C(uint8_t address, int8_t registr, uint8_t data)
{
	Wire.beginTransmission(address);
	Wire.write(registr);
	Wire.write(data);
	Wire.endTransmission();
}


void WriteByte_SPI(uint8_t CS_PIN, int8_t registr, uint8_t data)  // ”казываем на каком пине устройство и с какого регистра нужно прочитать данные
{
	//SPI.beginTransaction(settings16);       // записываем со скоростью 1 MHz делитель 16
	digitalWrite(CS_PIN, LOW);
	SPI.transfer(registr); //  Write(MSB=0)
	SPI.transfer(data);
	digitalWrite(CS_PIN, HIGH);
	SPI.endTransaction();
}

//==============================================================================================================================
uint8_t  ReadByte_I2C(uint8_t address, int8_t registr)
{
	Wire.beginTransmission(address);
	Wire.write(registr);
	byte reza = Wire.endTransmission();
	if (reza != 0)
	{
		Serial.print("!!! ReadByte_I2C_WriteMistake reza = ");Serial.println(reza);
		Serial2.print("!!! ReadByte_I2C_WriteMistake reza = ");Serial2.println(reza);

	};
	byte rezb = Wire.requestFrom(address, (uint8_t)1);
	if (rezb == 1)
	{
		uint8_t data = Wire.read();             //read one byte of data
		return data;
	}
	else
	{
		Serial.print("!!! ReadByte_I2C_WriteMistake rezb = ");Serial.println(rezb);
		Serial2.print("!!! ReadByte_I2C_WriteMistake rezb = ");Serial2.println(rezb);

		return 0;
	}
}
uint16_t  ReadWord_I2C(uint8_t address, int8_t registr)
{
	Wire.beginTransmission(address);
	Wire.write(registr);
	Wire.endTransmission();
	Wire.requestFrom(address, (uint8_t)2);
	uint8_t Hbyte = Wire.read();             //read High byte of data
	uint8_t Lbyte = Wire.read();             //read Low byte of data
	return ((int16_t)Hbyte << 8) | Lbyte;;             // —двигаем старший байт влево и добавл€ем младший байт
}

void scaner_i2c()       // ---------------- ќпределение адресов устройств -----------------
{
	// put your setup code here, to run once:
	while (!Serial) {}   Serial.println();   Serial.println("I2C scanner. Scanning ...");
	byte count = 0;
	//Wire.begin();
	for (byte i = 8; i < 120; i++)
	{
		Wire.beginTransmission(i);
		if (Wire.endTransmission() == 0)
		{
			Serial.print("Found address: ");
			Serial.print(i, DEC);
			Serial.print(" (0x");
			Serial.print(i, HEX);
			Serial.println(")");
			count++;
			delay(1);  // maybe unneeded?
		} // end of good response
	} // end of for loop
	Serial.println("Done.");
	Serial.print("Found ");
	Serial.print(count, DEC);
	Serial.println(" device(s).");
}

void  Write8_I2C(uint8_t address, int8_t registr, uint8_t data)
{
	Wire.beginTransmission(address);
	Wire.write(registr);
	Wire.write(data);
	int reza = Wire.endTransmission();
	if (reza != 0)
	{
		Serial.print(" Write8_I2C Mistake reza= ");Serial.println(reza);
	}
}
void  Write16_I2C(uint8_t address, int8_t registr, int16_t data)
{
	Wire.beginTransmission(address);
	Wire.write(registr);
	Wire.write(data >> 8);
	Wire.write(data & 0xFF);
	int reza = Wire.endTransmission();
	if (reza != 0)
	{
		Serial.print(" Write16_I2C Mistake reza= ");Serial.println(reza);
	}
}
void  Write32_I2C(uint8_t address, int8_t registr, int32_t data)
{
	Wire.beginTransmission(address);
	Wire.write(registr);
	Wire.write(data >> 24);
	Wire.write(data >> 16);
	Wire.write(data >> 8);
	Wire.write(data & 0xFF);
	int reza = Wire.endTransmission();
	if (reza != 0)
	{
		Serial.print(" Write32F_I2C Mistake reza= ");Serial.println(reza);
	}
}
void  Write32F_I2C(uint8_t address, int8_t registr, float data)
{
	//Serial.print(" float = "); Serial.println(data, BIN);

	uint8_t *pB = (uint8_t *)(&data);
	uint8_t LL_byte = pB[0];
	uint8_t L_byte = pB[1];
	uint8_t H_byte = pB[2];
	uint8_t HH_byte = pB[3];

	Wire.beginTransmission(address);
	Wire.write(registr);
	Wire.write(LL_byte);  //    Serial.print("LL_byte: "); Serial.println(LL_byte,BIN); 
	Wire.write(L_byte);  //    Serial.print("L_byte: "); Serial.println(L_byte,BIN); 
	Wire.write(H_byte);  //    Serial.print("H_byte: "); Serial.println(H_byte,BIN); 
	Wire.write(HH_byte);  //  Serial.print("HH_byte: "); Serial.println(HH_byte,BIN); 
	int reza = Wire.endTransmission();
	if (reza != 0)
	{
		Serial.print(" Write32F_I2C float Mistake reza= ");Serial.println(reza);
	}
}
uint16_t  ReadWord_I2C_LE(uint8_t address, int8_t registr)  // ќбратный пор€док байт !!!!!!!!!!!!!!
{
	Wire.beginTransmission(address);
	Wire.write(registr);
	int reza = Wire.endTransmission();
	if (reza != 0)
	{
		Serial.print(" ReadWord_I2C_LE Mistake reza= ");Serial.println(reza);
	}
	int rezb = Wire.requestFrom(address, (uint8_t)2);
	if (rezb == 2)
	{
		uint8_t Lbyte = Wire.read();             //read Low  byte of data
		uint8_t Hbyte = Wire.read();             //read High byte of data
		return ((int16_t)Hbyte << 8) | Lbyte;;             // —двигаем старший байт влево и добавл€ем младший байт
	}
	else
	{
		Serial.print(" ReadWord_I2C_LE Mistake rezb= ");Serial.println(rezb);
	}
}

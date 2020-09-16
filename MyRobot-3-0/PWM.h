
void ChangePWM()       // ��������� ������� ��� 
{
	//  - 976 ��		16 ���������   ������ ����� 1
	TCCR1A = 0b00000001; // 8bit
	TCCR1B = 0b00001011; // x64 fast pwm

//  - 976 �� -  		 8 ���������   ������	 ����� 2
	TCCR2B = 0b00000011; // x64
	TCCR2A = 0b00000011; // fast pwm
}


/* ��� 8 ������� �������   ���������� ��� 8 ��� �� 255

	//  - 62.5 ���
TCCR0B = 0b00000001; // x1
TCCR0A = 0b00000011; // fast pwm
//  - 31.4 ���
TCCR0B = 0b00000001; // x1
TCCR0A = 0b00000001; // phase correct
//  - 7.8 ���
TCCR0B = 0b00000010; // x8
TCCR0A = 0b00000011; // fast pwm
//  - 4 ���
TCCR0B = 0b00000010; // x8
TCCR0A = 0b00000001; // phase correct
//  - 976 �� - �� ���������
TCCR0B = 0b00000011; // x64
TCCR0A = 0b00000011; // fast pwm
//  - 490 ��
TCCR0B = 0b00000011; // x64
TCCR0A = 0b00000001; // phase correct
//  - 244 ��
TCCR0B = 0b00000100; // x256
TCCR0A = 0b00000011; // fast pwm
//  - 122 ��
TCCR0B = 0b00000100; // x256
TCCR0A = 0b00000001; // phase correct
//  - 61 ��
TCCR0B = 0b00000101; // x1024
TCCR0A = 0b00000011; // fast pwm
//  - 30 ��
TCCR0B = 0b00000101; // x1024
TCCR0A = 0b00000001; // phase correct
*/

/*	��� 16 ������� �������		���������� ��� 8 ��� �� 255

   //  - 62.5 ���
TCCR1A = 0b00000001; // 8bit
TCCR1B = 0b00001001; // x1 fast pwm
//  - 31.4 ���
TCCR1A = 0b00000001; // 8bit
TCCR1B = 0b00000001; // x1 phase correct
//  - 7.8 ���
TCCR1A = 0b00000001; // 8bit
TCCR1B = 0b00001010; // x8 fast pwm
//  - 4 ���
TCCR1A = 0b00000001; // 8bit
TCCR1B = 0b00000010; // x8 phase correct
//  - 976 ��
TCCR1A = 0b00000001; // 8bit
TCCR1B = 0b00001011; // x64 fast pwm
//  - 490 �� - �� ���������
TCCR1A = 0b00000001; // 8bit
TCCR1B = 0b00000011; // x64 phase correct
//  - 244 ��
TCCR1A = 0b00000001; // 8bit
TCCR1B = 0b00001100; // x256 fast pwm
//  - 122 ��
TCCR1A = 0b00000001; // 8bit
TCCR1B = 0b00000100; // x256 phase correct
//  - 61 ��
TCCR1A = 0b00000001; // 8bit
TCCR1B = 0b00001101; // x1024 fast pwm
//  - 30 ��
TCCR1A = 0b00000001; // 8bit
TCCR1B = 0b00000101; // x1024 phase correct
*/
/*

������� �������� ATmega328p
������	�����������	�������	�������	������	��� Arduino	��� ��
Timer0	8 ���	31 �� � 1 ���	32 258 � 1 ���	CHANNEL_A	D6	PD6
												CHANNEL_B	D5	PD5
Timer1	16 ���	0.11 �� � 1 ���	9 000 000 � 1 ���	CHANNEL_A	D9	PB1
													CHANNEL_B	D10	PB2
Timer2	8 ���	31 �� � 1 ���	32 258 � 1 ���	CHANNEL_A	D11	PB3
												CHANNEL_B	D3	PD3
������� �������� ATmega2560
������	�����������	�������	�������	������	��� Arduino	��� ��
Timer0	8 ���	31 �� � 1 ���	32 258 � 1 ���	CHANNEL_A	13	PB7
CHANNEL_B	4	PG5
Timer1	16 ���	0.11 �� � 1 ���	9 000 000 � 1 ���	CHANNEL_A	11	PB5
CHANNEL_B	12	PB6
CHANNEL_C	13	PB7
Timer2	8 ���	31 �� � 1 ���	32 258 � 1 ���	CHANNEL_A	10	PB4
CHANNEL_B	9	PH6
Timer3	16 ���	0.11 �� � 1 ���	9 000 000 � 1 ���	CHANNEL_A	5	PE3
CHANNEL_B	2	PE4
CHANNEL_C	3	PE5
Timer4	16 ���	0.11 �� � 1 ���	9 000 000 � 1 ���	CHANNEL_A	6	PH3
CHANNEL_B	7	PH4
CHANNEL_C	8	PH5
Timer5	16 ���	0.11 �� � 1 ���	9 000 000 � 1 ���	CHANNEL_A	46	PL3
CHANNEL_B	45	PL4
CHANNEL_C	44	PL5

*/
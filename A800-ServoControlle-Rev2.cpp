#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>

#define PWM_SP_CH		0
#define PWM_PGain_CH	1
#define PWM_DGain_CH	2


#define LED_PIN			13
#define Poten_PIN		0
#define STF_PIN			10
#define STR_PIN			11

#define MAX_ANGLE		45
#define MIN_ANGLE		-45
#define MAX_SUM_ERROR	30000
#define MAX_Freq		60
#define MIN_Freq		0

int readPoten(byte pin);
void SetFreqrOut(double FreqOut);
unsigned char LED_STATUS;

double Cvt_PWMtoAngle(unsigned int PWM_Period, double Min, double Max);
double Cvt_PotentoAngle(int PotVal,double Min, double Max);
double Cvt_PWMtoGain(int PWM,double Min, double Max);

void runPID();

void ISP_PWM_SP(void);
//void ISP_PWM_PGain(void);

uint8_t channel_pin[] = {2,3,4/*, 5, 6*/};
unsigned int PWM_Rising_Time[] = {0, 0, 0, 0};
unsigned int PWM_period[] = {0, 0, 0, 0};
unsigned int Last_PWM_period[] = {0, 0, 0, 0};
unsigned char PWM_SP_ReadComplete = 0;
//unsigned char PWM_PGain_ReadComplete = 0;
//unsigned char PWM_DGain_ReadComplete = 0;

double Feedback_Angle;
double Setpoint_Angle;
double PGain,DGain;
double FreqOut;
double Error, Last_Error, Sum_Error;

Adafruit_MCP4725 dac;

void setup() {
	Serial.begin(19200);
	Wire.begin();
	dac.begin(0x62);

	pinMode(0,INPUT);
	pinMode(1,OUTPUT);

	pinMode(LED_PIN,OUTPUT);

	pinMode(channel_pin[PWM_SP_CH], INPUT);
	//pinMode(channel_pin[PWM_PGain_CH], INPUT);
	//pinMode(channel_pin[PWM_DGain_CH], INPUT);

	pinMode(STF_PIN, OUTPUT);
	pinMode(STR_PIN, OUTPUT);

	attachInterrupt(digitalPinToInterrupt(channel_pin[PWM_SP_CH]),ISP_PWM_SP,RISING);
	//attachInterrupt(digitalPinToInterrupt(channel_pin[PWM_PGain_CH]), ISP_PWM_PGain,RISING);
	//attachInterrupt(digitalPinToInterrupt(channel_pin[PWM_DGain_CH]), ISP_PWM_DGain,RISING);

	digitalWrite(STF_PIN,HIGH);
	digitalWrite(STR_PIN,HIGH);

	Error = 0;
	Last_Error = 0;
	Sum_Error = 0;
	Feedback_Angle = 0;
	Setpoint_Angle = 0;
	FreqOut = 0;
	LED_STATUS = LOW;
}

void loop() {

	Feedback_Angle = Cvt_PotentoAngle(readPoten(Poten_PIN),MIN_ANGLE,MAX_ANGLE);
	Setpoint_Angle = Cvt_PWMtoAngle(PWM_period[PWM_SP_CH], MIN_ANGLE,MAX_ANGLE);
	Serial.println(Feedback_Angle);
	//PGain = Cvt_PWMtoGain(PWM_period[PWM_PGain_CH],0,5);
	//DGain = Cvt_PWMtoGain(PWM_period[PWM_DGain_CH],0,5);
	PGain = 1.5;
	DGain = 3;
	runPID();

	//----------- For Debug -----------

/*	Serial.print("Set Point = ");
	Serial.print(Setpoint_Angle,2);
	Serial.print(" Feedback = ");
	Serial.print(Feedback_Angle,2);
	Serial.print(" P Gain = ");
	Serial.println(PGain,2);
	Serial.print(" D Gain = ");
	Serial.print(DGain,2);
	Serial.print(" Out Freq = ");
	Serial.println(FreqOut);*/

	//---------------------------------

	SetFreqrOut(FreqOut);
	LED_STATUS = !LED_STATUS;
	digitalWrite(LED_PIN,LED_STATUS);
	delay(20);
}

void runPID(){
	Error = Setpoint_Angle - Feedback_Angle;
	FreqOut = PGain*Error + DGain*(Error-Last_Error);
	Last_Error = Error;
}

void SetFreqrOut(double FreqOut){
	if(FreqOut<0){
		if(-1*FreqOut>MAX_Freq) FreqOut = -1*MAX_Freq;
		dac.setVoltage((unsigned short int)map((int)(-1*FreqOut),MIN_Freq,MAX_Freq,0,4095), false);
		digitalWrite(STF_PIN,HIGH);
		digitalWrite(STR_PIN,LOW);
	}else{
		if(FreqOut>MAX_Freq) FreqOut = MAX_Freq;
		dac.setVoltage((unsigned short int)map((int)FreqOut,MIN_Freq,MAX_Freq,0,4095), false);
		digitalWrite(STF_PIN,LOW);
		digitalWrite(STR_PIN,HIGH);
	}
}

int readPoten(byte pin) {
	return analogRead(pin);
}

double Cvt_PWMtoAngle(unsigned int PWM_Period, double Min, double Max){
	int IntAngle;
	IntAngle = map((int)PWM_Period,1000,2000,(int)(Min*10),(int)(Max*10));
	return (double)IntAngle/10;
}

double Cvt_PotentoAngle(int PotVal,double Min, double Max){
	int IntAngle;
	IntAngle = map(PotVal,362,662,(int)(Min*10),(int)(Max*10));
	return (double)IntAngle/10;
}

double Cvt_PWMtoGain(int PWM,double Min, double Max){
	int Gain;
	Gain = map(PWM,1000,2000,(int)(Min*10),(int)(Max*10));
	return (double)Gain/10;
}

void ISP_PWM_SP(void) {
	if(PWM_SP_ReadComplete == 0){
		PWM_Rising_Time[PWM_SP_CH] = micros();
		attachInterrupt(digitalPinToInterrupt(channel_pin[PWM_SP_CH]),ISP_PWM_SP,FALLING);
		PWM_SP_ReadComplete = 1;
	}else{
		PWM_period[PWM_SP_CH] = micros() - PWM_Rising_Time[PWM_SP_CH];
		attachInterrupt(digitalPinToInterrupt(channel_pin[PWM_SP_CH]),ISP_PWM_SP,RISING);
		if(PWM_period[PWM_SP_CH]>=2100 || PWM_period[PWM_SP_CH] <= 900) PWM_period[PWM_SP_CH] = Last_PWM_period[PWM_SP_CH];
		Last_PWM_period[PWM_SP_CH] = PWM_period[PWM_SP_CH];
		PWM_SP_ReadComplete = 0;
	}
}
/*
void ISP_PWM_PGain(void) {
	if(PWM_PGain_ReadComplete == 0){
		PWM_Rising_Time[PWM_PGain_CH] = micros();
		attachInterrupt(digitalPinToInterrupt(channel_pin[PWM_PGain_CH]),ISP_PWM_PGain,FALLING);
		PWM_PGain_ReadComplete = 1;
	}else{
		PWM_period[PWM_PGain_CH] = micros() - PWM_Rising_Time[PWM_PGain_CH];
		attachInterrupt(digitalPinToInterrupt(channel_pin[PWM_PGain_CH]),ISP_PWM_PGain,RISING);
		PWM_PGain_ReadComplete = 0;
	}
}*/

/*
void ISP_PWM_DGain(void) {
		if(PWM_DGain_ReadComplete == 0){
		PWM_Rising_Time[PWM_DGain_CH] = micros();
		attachInterrupt(digitalPinToInterrupt(channel_pin[PWM_DGain_CH]),ISP_PWM_DGain,FALLING);
		PWM_DGain_ReadComplete = 1;
	}else{
		PWM_period[PWM_DGain_CH] = micros() - PWM_Rising_Time[PWM_DGain_CH];
		attachInterrupt(digitalPinToInterrupt(channel_pin[PWM_DGain_CH]),ISP_PWM_DGain,RISING);
		PWM_DGain_ReadComplete = 0;;
	}
	Serial.println(PWM_period[PWM_DGain_CH]);
}*/

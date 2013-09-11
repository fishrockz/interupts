// PinChangeIntExample, version 1.1 Sun Jan 15 06:24:19 CST 2012
// See the Wiki at http://code.google.com/p/arduino-pinchangeint/wiki for more information.

#include <PinChangeInt.h>



uint8_t latest_interrupted_pin;
volatile uint16_t interrupt_count[100]={0}; // 100 possible arduino pins
volatile uint16_t interrupt_start[100]={0}; // 100 possible arduino pins
unsigned long timestart=0;
//int16_t 
// unThrottleInStart = TCNT1;



#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
double Setpoint3, Input3, Output3;
double Setpoint4, Input4, Output4;

//Specify the links and initial tuning parameters
PID myPIDA(&Input1, &Output1, &Setpoint1,1,0,0.1, DIRECT);
PID myPIDB(&Input2, &Output2, &Setpoint2,1,0,0.1, DIRECT);
PID myPIDC(&Input3, &Output3, &Setpoint3,1,0,0.1, DIRECT);
PID myPIDD(&Input4, &Output4, &Setpoint4,1,0,0.1, DIRECT);

#include <RCArduinoFastLib.h>

// Assign your channel out pins
#define chanel1_OUT_PIN PIN6
#define chanel2_OUT_PIN PIN5
#define chanel3_OUT_PIN PIN4
#define chanel4_OUT_PIN PIN3
#define chanel5_OUT_PIN PIN2

// Assign servo indexes
#define chanel1_INDEX 0
#define chanel2_INDEX 1
#define chanel3_INDEX 2
#define chanel4_INDEX 3
#define SERVO_FRAME_SPACE 4




void quicfunc() {
	uint8_t sreg = SREG;
	cli();
	//latest_interrupted_pin=PCintPort::arduinoPin;
	if(1==PCintPort::pinState){
	  	interrupt_start[PCintPort::arduinoPin]=TCNT1;
	 }else{
		interrupt_count[PCintPort::arduinoPin]=(TCNT1-interrupt_start[PCintPort::arduinoPin])>>1;
	}
	SREG = sreg;
}

void PCpin(int pin){
	pinMode(pin,INPUT);
	digitalWrite(pin, LOW);
	PCintPort::attachInterrupt(pin, &quicfunc, CHANGE);
}


void setup() {
	pinMode(13,OUTPUT);
	digitalWrite(13, LOW);
	//pinMode(PIN2, INPUT);	
	
	//sensors
	PCpin(A8);//62
	/*PCpin(A9);
	PCpin(A10);
	PCpin(A11);
	PCpin(A12);
	PCpin(A13);
	PCpin(A14);
	PCpin(A15);//69
	
	//inputs
	PCpin(50);
	PCpin(51);
	PCpin(52);
	PCpin(53);
	PCpin(10);
	PCpin(11);
	*/
	
	Serial.begin(115200);
	Serial.println("---------------------------------------");


	pinMode(chanel1_OUT_PIN,OUTPUT);
	pinMode(chanel2_OUT_PIN,OUTPUT);
	pinMode(chanel3_OUT_PIN,OUTPUT);
	pinMode(chanel4_OUT_PIN,OUTPUT);



	CRCArduinoFastServos::attach(chanel1_INDEX,chanel1_OUT_PIN);
	CRCArduinoFastServos::attach(chanel2_INDEX,chanel2_OUT_PIN);
	CRCArduinoFastServos::attach(chanel3_INDEX,chanel3_OUT_PIN);
	CRCArduinoFastServos::attach(chanel4_INDEX,chanel4_OUT_PIN);


	// lets set a standard rate of 50 Hz by setting a frame space of 10 * 2000 = 3 Servos + 7 times 2000
	CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE,6*2000);

	CRCArduinoFastServos::begin();



	//initialize the variables we're linked to
	Input1 = Input2 = Input3 = Input4 = 10;
	Setpoint1 = Setpoint2 = Setpoint3 = Setpoint4 = 150;
	//turn the PID's on
	myPIDA.SetMode(AUTOMATIC);
	myPIDB.SetMode(AUTOMATIC);
	myPIDC.SetMode(AUTOMATIC);
	myPIDD.SetMode(AUTOMATIC);
}

uint8_t i;
void loop() {
	if(myPIDA.Compute())
	{
		Serial.print("pided ");
		Serial.print(Output1);
		Serial.print(" ");
		Serial.println(Input1);
	}
	myPIDB.Compute();
	myPIDC.Compute();
	myPIDD.Compute();
	
	uint16_t  count;
	Serial.print(".");
	delay(100);
	Serial.print("Count for pin ");
	for (i=0; i < 100; i++) {
		count=interrupt_count[i];
		if (count != 0) {
			

			Serial.print("{Setpoint(V ");
			Serial.print(i, DEC);
		
			Serial.print("),T, ");
			Serial.print(count);
			Serial.print("} ");
			
		}
		
	}
	Input1 = interrupt_count[62]/10;
	Input2 = interrupt_count[63]/10;
	Input3 = interrupt_count[64]/10;
	Input4 = interrupt_count[65]/10;
	
	CRCArduinoFastServos::writeMicroseconds(chanel1_INDEX,int(Output1*4 + 1000.0));
	CRCArduinoFastServos::writeMicroseconds(chanel2_INDEX,int(Output2*4 + 1000.0));
	//CRCArduinoFastServos::writeMicroseconds(chanel3_INDEX,int(Output3*4 + 1000.0));
	//CRCArduinoFastServos::writeMicroseconds(chanel4_INDEX,int(Output4*4 + 1000.0));

	CRCArduinoFastServos::writeMicroseconds(chanel3_INDEX,interrupt_count[62]);
	CRCArduinoFastServos::writeMicroseconds(chanel4_INDEX,interrupt_count[63]);


	Serial.print(int(Output1*4 + 1000.0));Serial.print(" ");
	Serial.print(int(Output2*4 + 1000.0));Serial.print(" ");
	Serial.print(int(Output3*4 + 1000.0));Serial.print(" ");
	Serial.print(int(Output4*4 + 1000.0));Serial.print(" ");

	Serial.print("{Setpoint(O 1),T, ");
	Serial.print(int(Output1*4 + 1000.0));
	Serial.print("} ");
	Serial.print("{Setpoint(O 2),T, ");
	Serial.print(int(Output2*4 + 1000.0));
	Serial.print("} ");


	Serial.print(" \n");
}


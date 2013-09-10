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
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPIDA(&Input, &Output, &Setpoint,2,5,1, DIRECT);
PID myPIDB(&Input, &Output, &Setpoint,2,5,1, DIRECT);
PID myPIDC(&Input, &Output, &Setpoint,2,5,1, DIRECT);
PID myPIDD(&Input, &Output, &Setpoint,2,5,1, DIRECT);

#include <RCArduinoFastLib.h>

// Assign your channel out pins
#define THROTTLE_OUT_PIN 5
#define STEERING_OUT_PIN 4
#define AUX_OUT_PIN 3

// Assign servo indexes
#define SERVO_THROTTLE 0
#define SERVO_STEERING 1
#define SERVO_AUX 2
#define SERVO_FRAME_SPACE 3




void quicfunc() {
	  uint8_t sreg = SREG;
  cli();
	//latest_interrupted_pin=PCintPort::arduinoPin;
	if(1==PCintPort::pinState){
	  	interrupt_start[PCintPort::arduinoPin]=TCNT1;
	 }else{
		interrupt_count[PCintPort::arduinoPin]=(TCNT1-interrupt_start[PCintPort::arduinoPin])>>1;
	}
	//interrupts();
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
	pinMode(PIN2, INPUT);	
	
	//sensors
	PCpin(A8);//62
	PCpin(A9);
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
	
	
	Serial.begin(9600);
	Serial.println("---------------------------------------");
	
	/************ not needed as FastServos do this tooo
	TCNT1 = 0;              // clear the timer count  

    // Initilialise Timer1
    TCCR1A = 0;             // normal counting mode 
    TCCR1B = 2;     // set prescaler of 64 = 1 tick = 4us */
    
    
   //CRCArduinoFastServos::setup();
  pinMode(THROTTLE_OUT_PIN,OUTPUT);
  pinMode(STEERING_OUT_PIN,OUTPUT);
  pinMode(AUX_OUT_PIN,OUTPUT);
  CRCArduinoFastServos::attach(SERVO_THROTTLE,THROTTLE_OUT_PIN);
  CRCArduinoFastServos::attach(SERVO_STEERING,STEERING_OUT_PIN);
  CRCArduinoFastServos::attach(SERVO_AUX,AUX_OUT_PIN);
  
  // lets set a standard rate of 50 Hz by setting a frame space of 10 * 2000 = 3 Servos + 7 times 2000
  CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE,7*2000);

  CRCArduinoFastServos::begin();



  //initialize the variables we're linked to
  Input = 10;
  Setpoint = 150;

  //turn the PID on
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
		Serial.print(Output);
		Serial.print(" ");
		Serial.println(Input);
	}
	myPIDB.Compute();
	myPIDC.Compute();
	myPIDD.Compute();
	
	uint16_t  count;
	Serial.print(".");
	delay(300);
	Serial.print("Count for pin ");
	for (i=0; i < 100; i++) {
	
		if (interrupt_count[i] != 0) {
			count=interrupt_count[i];
			//interrupt_count[i]=0;
			CRCArduinoFastServos::writeMicroseconds(SERVO_THROTTLE,1001);
			CRCArduinoFastServos::writeMicroseconds(SERVO_AUX,1999);
			Serial.print("<< ");
			Serial.print(i, DEC);
			
			Serial.print(" is ");
			Serial.print(interrupt_count[i]);//Serial.print(" ");Serial.print(interrupt_start[i]);
			Serial.print(" >>");
			
		}
		
	}
	Input = interrupt_count[52]/10;
	
	CRCArduinoFastServos::writeMicroseconds(SERVO_STEERING,interrupt_start[62]);
	Serial.print(" \n");
}


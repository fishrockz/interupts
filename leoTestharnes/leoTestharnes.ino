// PinChangeIntExample, version 1.1 Sun Jan 15 06:24:19 CST 2012
// See the Wiki at http://code.google.com/p/arduino-pinchangeint/wiki for more information.

#include <PinChangeInt.h>



uint8_t latest_interrupted_pin;
volatile uint16_t interrupt_count[100]={0}; // 100 possible arduino pins
volatile uint16_t interrupt_start[100]={0}; // 100 possible arduino pins
unsigned long timestart=0;
//int16_t 
// unThrottleInStart = TCNT1;






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

	PCpin(11);//pin11
	PCpin(10);
	PCpin(9);
	PCpin(8);//pin8
	
	
	Serial.begin(115200);
	Serial.println("---------------------------------------");

	/************ not needed as FastServos do this tooo*/
	TCNT1 = 0;              // clear the timer count  

	// Initilialise Timer1
	TCCR1A = 0;             // normal counting mode 
	TCCR1B = 2;     // set prescaler of 64 = 1 tick = 4us */
    


	
}

uint8_t i;
void loop() {

	
	uint16_t  count;
	Serial.print(".");
	delay(20);//50hz
	Serial.print("Count for pin ");
	for (i=0; i < 100; i++) {
		count=interrupt_count[i];
		if (count != 0) {
			

			Serial.print("{Setpoint(T ");// "T" for test
			Serial.print(i, DEC);
		
			Serial.print("),T, ");
			Serial.print(count);
			Serial.print("} ");
			
		}
		
	}


	Serial.print(" \n");
}


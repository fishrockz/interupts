// PinChangeIntExample, version 1.1 Sun Jan 15 06:24:19 CST 2012
// See the Wiki at http://code.google.com/p/arduino-pinchangeint/wiki for more information.

#include <PinChangeInt.h>



uint8_t latest_interrupted_pin;
uint16_t report_time;
uint16_t work_time;
uint16_t tmp_time;


volatile uint16_t interrupt_count[100]={0}; // 100 possible arduino pins
volatile uint16_t interrupt_start[100]={0}; // 100 possible arduino pins
unsigned long timestart=0;
//int16_t 
// unThrottleInStart = TCNT1;



#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint_right, Input_right, Output_right;
double Setpoint_front, Input_front, Output_front;
double Setpoint_left, Input_left, Output_left;
double Setpoint_rear, Input_rear, Output_rear;

//Specify the links and initial tuning parameters
PID myPID_right(&Input_right, &Output_right, &Setpoint_right,0.9,0.1,1.5, DIRECT);
PID myPID_front(&Input_front, &Output_front, &Setpoint_front,1,0,0.1, DIRECT);
PID myPID_left(&Input_left, &Output_left, &Setpoint_left,1,0,0.1, DIRECT);
PID myPID_rear(&Input_rear, &Output_rear, &Setpoint_rear,1,0,0.1, DIRECT);

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


//
double safe_distance=130; //value in cm
double right_sonar, front_sonar, left_sonar, rear_sonar, bottom_sonar, top_sonar;

double pitch_in, roll_in, throttle_in, mode_switch;
int compd_pitch, compd_roll, compd_throttle;



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
	*/PCpin(50);
	PCpin(51);
	PCpin(52);
	PCpin(53);
	/*PCpin(10);
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
	Input_right = right_sonar;      
        Input_front = front_sonar;
        Input_left = left_sonar;
        Input_rear = rear_sonar;
        
	Setpoint_right= Setpoint_front = Setpoint_left = Setpoint_rear = safe_distance;

	//turn the PID's on
	myPID_right.SetMode(AUTOMATIC);
	myPID_front.SetMode(AUTOMATIC);
	myPID_left.SetMode(AUTOMATIC);
	myPID_rear.SetMode(AUTOMATIC);
	report_time	= millis();
	work_time	= millis();
}





void report(){
	report_time	= millis();
	uint16_t  count;
	uint8_t i;
	/*Serial.print("Count for pin ");
	for (i=0; i < 100; i++) {
		count=interrupt_count[i];
		if (count != 0) {
			

			Serial.print("{Setpoint(V ");
			Serial.print(i, DEC);
		
			Serial.print("),T, ");
			Serial.print(count);
			Serial.print("} ");
			
		}
		}*/

		Serial.print("{Output_right,T, ");
		Serial.print(Output_right);
		Serial.print("} ");
/*
                Serial.print("{Output_front,T, ");
		Serial.print(Output_front);
		Serial.print("} ");
                  
                Serial.print("{Output_left,T, ");
		Serial.print(Output_left);
		Serial.print("} ");

                Serial.print("{Output_rear,T, ");
		Serial.print(Output_rear);
		Serial.print("} ");

		Serial.print("{pitch_in,T, ");
		Serial.print(pitch_in);
		Serial.print("} ");
*/
                Serial.print("{roll_in,T, ");
		Serial.print(roll_in/10);
		Serial.print("} ");
/*
                Serial.print("{throttle_in,T, ");
		Serial.print(throttle_in);
		Serial.print("} ");
*/
                Serial.print("{right_sonar,T, ");
		Serial.print(right_sonar);
		Serial.print("} ");
/*
                Serial.print("{front_sonar,T, ");
		Serial.print(front_sonar);
		Serial.print("} ");

                Serial.print("{left_sonar,T, ");
		Serial.print(left_sonar);
		Serial.print("} ");

                Serial.print("{rear_sonar,T, ");
		Serial.print(rear_sonar);
		Serial.print("} ");
*/
                Serial.print("{Setpoint_right,T, ");
		Serial.print(Setpoint_right);
		Serial.print("} ");
/*
                Serial.print("{compd_pitch,T, ");
		Serial.print(compd_pitch);
		Serial.print("} ");
*/
                Serial.print("{compd_roll,T, ");
		Serial.print(compd_roll/10);
		Serial.print("} ");
/*
                Serial.print("{compd_throttle,T, ");
		Serial.print(compd_throttle;
		Serial.print("} ");

*/


		Serial.print(" \n");

}

void workloop(){

	work_time	= millis();

	//right_sonar= (interrupt_count[62])/58; //value in cm
        right_sonar= (interrupt_count[62])/10; //RC emulation sonar
	front_sonar= (interrupt_count[63])/58; //value in cm
	left_sonar= (interrupt_count[64])/58; //value in cm
	rear_sonar= (interrupt_count[65])/58; //value in cm
        pitch_in= (interrupt_count[50]);
        roll_in= (interrupt_count[51]);
        throttle_in= (interrupt_count[52]);
        mode_switch= (interrupt_count[53]);
        
        
	Input_right=right_sonar;
	Input_front=front_sonar;
	Input_left=left_sonar;
	Input_rear=rear_sonar;

// i think this should have the latist input data so have moved it to after the seting of the input vars
	myPID_right.Compute();
	myPID_front.Compute();
	myPID_left.Compute();
	myPID_rear.Compute();

	//compd_pitch=constrain(pitch_in + int(map(Output_front,0,30,0,1000)-map(Output_rear,0,30,0,1000),1000,2000);
	//compd_roll=constrain(roll_in + int(map(Output_right,0,30,0,1000) - map(Output_left,0,30,0,1000)),1000,2000);
	compd_roll=constrain(roll_in-int(map(Output_right,0,30,0,1000)),1000,2000);

	//this one might have to be a bit difrent in the final cut.
	//compd_throttle=constrain(throttle_in-map(Output_top,0,30,0,1000)+map(Output_bottom,0,30,0,1000),1000,2000);

	CRCArduinoFastServos::writeMicroseconds(chanel1_INDEX,compd_pitch);
	CRCArduinoFastServos::writeMicroseconds(chanel2_INDEX,compd_roll);
        //CRCArduinoFastServos::writeMicroseconds(chanel3_INDEX,compd_throttle);
	
	//CRCArduinoFastServos::writeMicroseconds(chanel4_INDEX,interrupt_count[63]);

}


void loop() {
	tmp_time=millis();
	
	if (tmp_time  >report_time+ 50){
		report();
	}
	
	if (tmp_time  >work_time+ 5){
		workloop();
	}
}






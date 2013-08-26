/*****************************************************************************************************************************
// RCArduinoFastLib by DuaneB is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
//
// http://rcarduino.blogspot.com
//
*****************************************************************************************************************************/

#include "Arduino.h"
#include "pins_arduino.h"
#include "RCArduinoFastLib.h"

/*----------------------------------------------------------------------------------------

This is essentially a derivative of the Arduino Servo Library created by Michael Margolis

As the technique is very similar to the Servo class, it can be useful to study in order
to understand the servo class.

What does the library do ? It uses a very inexpensive and common 4017 Counter IC
To generate pulses to independently drive up to 10 servos from two Arduino Pins

As previously mentioned, the library is based on the techniques used in the Arduino Servo
library created by Michael Margolis. This means that the library uses Timer1 and Timer1 output
compare register A.

OCR1A is linked to digital pin 9 and so we use digital pin 9 to generate the clock signal
for the 4017 counter.

Pin 12 is used as the reset pin.

*/

void CRCArduinoFastServos::setup()
{
 m_sCurrentOutputChannelA = 0;
 while(m_sCurrentOutputChannelA < RC_CHANNEL_OUT_COUNT)
 {
  m_ChannelOutA[m_sCurrentOutputChannelA].m_unPulseWidth = microsecondsToTicks(RCARDUINO_SERIAL_SERVO_MAX); 


   m_sCurrentOutputChannelA++;
  }
}


// Timer1 Output Compare A interrupt service routine
// call out class member function OCR1A_ISR so that we can
// access out member variables
ISR(TIMER1_COMPA_vect)
{
    CRCArduinoFastServos::OCR1A_ISR();
}

void CRCArduinoFastServos::OCR1A_ISR()
{

  if(m_sCurrentOutputChannelA >= RC_CHANNEL_OUT_COUNT)
  {
    // reset our current servo/output channel to 0
    m_sCurrentOutputChannelA = 0;
    
    CRCArduinoFastServos::setChannelPinLowA(RC_CHANNEL_OUT_COUNT-1);
  }
  else
  {   
    CRCArduinoFastServos::setChannelPinLowA(m_sCurrentOutputChannelA-1);
  }
   
  CRCArduinoFastServos::setCurrentChannelPinHighA();
    
    // set the duration of the output pulse
    CRCArduinoFastServos::setOutputTimerForPulseDurationA();

    // done with this channel so move on.
    m_sCurrentOutputChannelA++;
}

void CRCArduinoFastServos::setChannelPinLowA(uint8_t sChannel)
{
    volatile CPortPin *pPortPin = m_ChannelOutA + sChannel;

    if(pPortPin->m_sPinMask)
     *pPortPin->m_pPort ^= pPortPin->m_sPinMask;
}

void CRCArduinoFastServos::setCurrentChannelPinHighA()
{
    volatile CPortPin *pPortPin = m_ChannelOutA + m_sCurrentOutputChannelA;

    if(pPortPin->m_sPinMask)
     *pPortPin->m_pPort |= pPortPin->m_sPinMask;
}

// After we set an output pin high, we need to set the timer to comeback for the end of the pulse 
void CRCArduinoFastServos::setOutputTimerForPulseDurationA()
{
  OCR1A = TCNT1 + m_ChannelOutA[m_sCurrentOutputChannelA].m_unPulseWidth; 
}



// updates a channel to a new value, the class will continue to pulse the channel
// with this value for the lifetime of the sketch or until writeChannel is called
// again to update the value
void CRCArduinoFastServos::writeMicroseconds(uint8_t nChannel,uint16_t unMicroseconds)
{
    // dont allow a write to a non existent channel
    if(nChannel > RCARDUINO_MAX_SERVOS)
        return;

  // constraint the value just in case
  unMicroseconds = constrain(unMicroseconds,RCARDUINO_SERIAL_SERVO_MIN,RCARDUINO_SERIAL_SERVO_MAX);


  
  unMicroseconds = microsecondsToTicks(unMicroseconds);
  
  // disable interrupts while we update the multi byte value output value
  uint8_t sreg = SREG;
  cli();
  
  m_ChannelOutA[nChannel].m_unPulseWidth = unMicroseconds; 

  // enable interrupts
  SREG = sreg;
}

uint16_t CRCArduinoFastServos::ticksToMicroseconds(uint16_t unTicks)
{
    return unTicks / 2;
}

uint16_t CRCArduinoFastServos::microsecondsToTicks(uint16_t unMicroseconds)
{
 return unMicroseconds * 2;
}

void CRCArduinoFastServos::attach(uint8_t sChannel,uint8_t sPin)
{
    if(sChannel >= RCARDUINO_MAX_SERVOS)
        return;


  
  // disable interrupts while we update the multi byte value output value
  uint8_t sreg = SREG;
  cli();
  
  m_ChannelOutA[sChannel].m_unPulseWidth = microsecondsToTicks(RCARDUINO_SERIAL_SERVO_DEFAULT);
  m_ChannelOutA[sChannel].m_pPort = getPortFromPin(sPin);
  m_ChannelOutA[sChannel].m_sPinMask = getPortPinMaskFromPin(sPin);

  // enable interrupts
  SREG = sreg;
  Serial.print("sPortPinMask ");
  Serial.println( m_ChannelOutA[sChannel].m_sPinMask );
  pinMode(sPin,OUTPUT);
}

// this allows us to run different refresh frequencies on channel A and B
// for example servos at a 70Hz rate on A and ESCs at 250Hz on B
void CRCArduinoFastServos::setFrameSpaceA(uint8_t sChannel,uint16_t unMicroseconds)
{
  // disable interrupts while we update the multi byte value output value
  uint8_t sreg = SREG;
  cli();
  
  m_ChannelOutA[sChannel].m_unPulseWidth = microsecondsToTicks(unMicroseconds);
  m_ChannelOutA[sChannel].m_pPort = 0;
  m_ChannelOutA[sChannel].m_sPinMask = 0;

  // enable interrupts
  SREG = sreg;
}


// Easy to optimise this, but lets keep it readable instead, its short enough.
volatile uint8_t* CRCArduinoFastServos::getPortFromPin(uint8_t sPin)
{
    volatile uint8_t* pPort;

    //*pPort = digitalPinToPort(sPin);
    
    uint8_t port = digitalPinToPort(sPin);
    pPort = portOutputRegister(port);
    
    return pPort;
}

// Easy to optimise this, but lets keep it readable instead, its short enough.
uint8_t CRCArduinoFastServos::getPortPinMaskFromPin(uint8_t sPin)
{

    uint8_t sPortPinMask ;

    sPortPinMask = digitalPinToBitMask(sPin);
    
    return sPortPinMask;
}   

void CRCArduinoFastServos::begin()
{
    TCNT1 = 0;              // clear the timer count  

    // Initilialise Timer1
    TCCR1A = 0;             // normal counting mode 
    TCCR1B = 2;     // set prescaler of 64 = 1 tick = 4us 

    // ENABLE TIMER1 OCR1A INTERRUPT to enabled the first bank (A) of ten servos
    TIFR1 |= _BV(OCF1A);     // clear any pending interrupts; 
    TIMSK1 |=  _BV(OCIE1A) ; // enable the output compare interrupt 



    OCR1A = TCNT1 + 4000; // Start in two milli seconds
    
    for(uint8_t sServo = 0;sServo<RC_CHANNEL_OUT_COUNT;sServo++)
    {
        Serial.println(m_ChannelOutA[sServo].m_unPulseWidth);


        }
}

volatile CRCArduinoFastServos::CPortPin CRCArduinoFastServos::m_ChannelOutA[RC_CHANNEL_OUT_COUNT]; 
uint8_t CRCArduinoFastServos::m_sCurrentOutputChannelA;





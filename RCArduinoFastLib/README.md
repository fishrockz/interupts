interupts
=========

	volatile uint8_t* CRCArduinoFastServos::getPortFromPin(uint8_t sPin)

	uint8_t CRCArduinoFastServos::getPortPinMaskFromPin(uint8_t sPin)

had been "optermised" even tho they are called seldomly

the "optermisation" had maded the script work only on one board i cant remeber is i made any other modifications to get it to run on the mega... cant remeber if i tested it on the UNO too, i think i did...

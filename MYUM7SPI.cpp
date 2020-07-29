/*

Custom UM7 Library for the Dynamic Knee Bracing Project
Ben Milligan, 2020
Compilation code from several libraries

*/

#include "MYUM7SPI.h"

//////////////////////////////////////////
//		READ FUNCTIONS FOR THE UM7		//
//////////////////////////////////////////

/*
	Default constructor. Initializes cs pin
*/
MYUM7SPI::MYUM7SPI(uint16_t cs_) {
	cs = cs_;
}

/*
	Initializes the UM7, include the desired SPI settings.
*/
void MYUM7SPI::begin() {
	SPI.begin();
	pinMode(cs, OUTPUT);
}

/*
	Read a register that carries 2 datasets (euler data). Uses a user defined bool to determine which dataset to return
*/
int16_t MYUM7SPI::read_register(uint16_t address, bool first_half) {
	byte inByte = 0;
	int16_t result;
	
	digitalWrite(cs, LOW);
	
	SPI.transfer(READ);
	delayMicroseconds(5);
	
	SPI.transfer(address);
	delayMicroseconds(5);
	
	if(!first_half) {
		SPI.transfer(0x00);
		delayMicroseconds(5);

		SPI.transfer(0x00);
		delayMicroseconds(5);
	}
	result = SPI.transfer(0x00);
	delayMicroseconds(5);

	result = result << 8;

	inByte = SPI.transfer(0x00);
	delayMicroseconds(5);

	result = result | inByte;

	digitalWrite(cs, HIGH);
	return(result);
}

/*
	Read from a register. Assume register takes an entire 4 Bytes and is a float point type.
*/
int32_t MYUM7SPI::read_register(uint16_t address) {
	byte inByte = 0;
	int32_t result;

	digitalWrite(cs, LOW);

	SPI.transfer(READ);
	delayMicroseconds(5);

	SPI.transfer(address);
	delayMicroseconds(5);

	result = SPI.transfer(0x00);
	delayMicroseconds(5);

	for (int i = 0; i < 3; i++) {
		result = result << 8;

		inByte = SPI.transfer(0x00);
		delayMicroseconds(5);

		result = result | inByte;
	}

	digitalWrite(cs, HIGH);
	return(result);
}

/*

*/
typedef union {
	float val;
	uint8_t bytes[4];
} floatval;

/*
	Writes to a register. 
*/
void MYUM7SPI::write_config_register(uint16_t address, float contents_) {
	floatval contents;
	contents.val = contents_;

	digitalWrite(cs, LOW);

	SPI.transfer(WRITE);
	delayMicroseconds(5);

	SPI.transfer(address);
	delayMicroseconds(5);

	for (int i = 3; i >= 0; i--) {
		SPI.transfer(contents.bytes[i]);
		delayMicroseconds(5);
	}

	digitalWrite(cs, HIGH);
}

/*

*/
void MYUM7SPI::calibrate() {

}

/*

*/
void MYUM7SPI::reset() {

}
/*

Custom UM7 Library for the Dynamic Knee Bracing Project
Ben Milligan, 2020

Compilation code from several libraries. Credit to:
 - Michael Hoyer, https://github.com/mikehoyer/UM7-Arduino
 - crazyFrg, https://forum.arduino.cc/index.php?topic=625401.0
 - 

 Adaptation to the SPI library to allow for synchronous capabilities.
 Could add:
 - SPISettings
 - health packet/error decode
 - baud rate configs
*/

#include "MYUM7SPI.h"



/*
	Default constructor. Initializes cs pin and sets it as an output.
*/
MYUM7SPI::MYUM7SPI(uint16_t cs_) {
	cs = cs_;
	pinMode(cs, OUTPUT);
}

/*

*/
typedef union {
	uint32_t val;
	uint8_t bytes[4];
} intval;

//////////////////////////////////
//		CONFIG FUNCTIONS		//
//////////////////////////////////

/*
	Sets the rate for all raw datasets. rate will vary from 0-255
*/
void MYUM7SPI::set_all_raw_rate(byte rate_) {
	intval rate;
	rate.bytes[0] = 0;
	rate.bytes[1] = 0;
	rate.bytes[2] = 0;
	rate.bytes[3] = rate_;
	
	write_register(CREG_COM_RATES2, rate.val);
}

/*
	Sets the rate for all processed datasets. rate will vary from 0-255
*/
void MYUM7SPI::set_all_processed_rate(byte rate_) {
	intval rate;
	rate.bytes[0] = 0;
	rate.bytes[1] = 0;
	rate.bytes[2] = 0;
	rate.bytes[3] = rate_;

	write_register(CREG_COM_RATES4, rate.val);
}

/*
	Overloaded function for multiple rate config capabilities. includes the:
	- quaternion rate
	- euler rate
	- position rate
	- velocity rate
*/
void MYUM7SPI::set_orientation_rate(byte quat_rate, byte euler_rate, byte pos_rate, byte vel_rate) {
	intval rate;
	rate.bytes[0] = quat_rate;
	rate.bytes[1] = euler_rate;
	rate.bytes[2] = pos_rate;
	rate.bytes[3] = vel_rate;

	write_register(CREG_COM_RATES4, rate.val);
}

/*
	Overloaded function for multiple rate config capabilities. includes the:
	- quaternion rate
	- euler rate
	- position rate
	- velocity rate
*/
void MYUM7SPI::set_orientation_rate(byte quat_rate, byte euler_rate, byte pos_rate) {
	intval rate;
	rate.bytes[0] = quat_rate;
	rate.bytes[1] = euler_rate;
	rate.bytes[2] = pos_rate;
	rate.bytes[3] = 0;

	write_register(CREG_COM_RATES4, rate.val);
}

/*
	Overloaded function for multiple rate config capabilities. includes the:
	- quaternion rate
	- euler rate
	- position rate
	- velocity rate
*/
void MYUM7SPI::set_orientation_rate(byte quat_rate, byte euler_rate) {
	intval rate;
	rate.bytes[0] = quat_rate;
	rate.bytes[1] = euler_rate;
	rate.bytes[2] = 0;
	rate.bytes[3] = 0;

	write_register(CREG_COM_RATES4, rate.val);
}

/*
	Overloaded function for multiple rate config capabilities. includes the:
	- quaternion rate
	- euler rate
	- position rate
	- velocity rate
*/
void MYUM7SPI::set_orientation_rate(byte quat_rate) {
	intval rate;
	rate.bytes[0] = quat_rate;
	rate.bytes[1] = 0;
	rate.bytes[2] = 0;
	rate.bytes[3] = 0;

	write_register(CREG_COM_RATES4, rate.val);
}

/*
	Miscellaneous settings for filter and sensor control options. Send a 0 if you don't wish to configure a specific setting
	Ex. set_misc_settings(0, 1, 1, 0)

	PPS bit = Causes the TX2/RX2 pin to be used with an external GPS
	ZG bit = Causes UM7 to measure gyro bias at setup
	Q bit = Sensor will run in Quternion mode instead of Euler mode. Fixes pitch error in the Gimbal lock position
	MAG bit = Magnetometer will be used in state updates
*/
void MYUM7SPI::set_misc_ssettings(bool pps, bool zg, bool q, bool mag) {
	byte b1 = 0, b0 = 0;

	if (pps) b1 = 0b00000001;

	if (zg) {
		b0 = 00000100;
		if (q) {
			b0 = 00000110;
			if (mag) b0 = 00000111;
		}
		if (mag) b0 = 00000101;
	}
	if (q) {
		b0 = 00000010;
		if (mag) b0 = 00000011;
	}
	if (mag) b0 = 00000001;

	intval temp;
	temp.bytes[0] = 0;
	temp.bytes[1] = 0;
	temp.bytes[2] = b1;
	temp.bytes[3] = b0;

	write_register(CREG_MISC_SETTINGS, temp.val);
}

//////////////////////////////
//		DATA FUNCTIONS		//
//////////////////////////////

void MYUM7SPI::get_all_raw_data() {
	gyro_raw_x = read_register(DREG_GYRO_RAW_XY, 1);
	gyro_raw_y = read_register(DREG_GYRO_RAW_XY, 0);
	gyro_raw_z = read_register(DREG_GYRO_RAW_Z, 1);
	gyro_raw_time = read_register(DREG_GYRO_RAW_TIME);

	accel_raw_x = read_register(DREG_ACCEL_RAW_XY, 1);
	accel_raw_y = read_register(DREG_ACCEL_RAW_XY, 0);
	accel_raw_z = read_register(DREG_ACCEL_RAW_Z, 1);
	accel_raw_time = read_register(DREG_ACCEL_RAW_TIME);

	mag_raw_x = read_register(DREG_MAG_RAW_XY, 1);
	mag_raw_y = read_register(DREG_MAG_RAW_XY, 0);
	mag_raw_z = read_register(DREG_MAG_RAW_Z, 1);
	mag_raw_time = read_register(DREG_MAG_RAW_TIME);

	temp = read_register(DREG_TEMPERATURE);
	temp_time = read_register(DREG_TEMPERATURE_TIME);
}

void MYUM7SPI::get_all_processed_data() {
	gyro_x = read_register(DREG_GYRO_PROC_X);
	gyro_y = read_register(DREG_GYRO_PROC_Y);
	gyro_z = read_register(DREG_GYRO_PROC_Z);
	gyro_time = read_register(DREG_GYRO_PROC_TIME);

	accel_x = read_register(DREG_ACCEL_PROC_X);
	accel_y = read_register(DREG_ACCEL_PROC_Y);
	accel_z = read_register(DREG_ACCEL_PROC_Z);
	accel_time = read_register(DREG_ACCEL_PROC_TIME);

	mag_x = read_register(DREG_MAG_PROC_X);
	mag_y = read_register(DREG_MAG_PROC_Y);
	mag_z = read_register(DREG_MAG_PROC_Z);
	mag_time = read_register(DREG_MAG_PROC_TIME);
}

void MYUM7SPI::get_all_orientation_data() {
	quat_a = read_register(DREG_QUAT_AB, 1) / 29789.09091;
	quat_b = read_register(DREG_QUAT_AB, 0) / 29789.09091;
	quat_c = read_register(DREG_QUAT_CD, 1) / 29789.09091;
	quat_d = read_register(DREG_QUAT_CD, 0) / 29789.09091;
	quat_time = read_register(DREG_QUAT_TIME, 0);

	roll = read_register(DREG_EULER_PHI_THETA, 1) / 91.02222;
	pitch = read_register(DREG_EULER_PHI_THETA, 0) / 91.02222;
	yaw = read_register(DREG_EULER_PSI, 1) / 91.02222;
	roll_rate = read_register(DREG_EULER_PHI_THETA_DOT, 1) / 16.0;
	pitch_rate = read_register(DREG_EULER_PHI_THETA_DOT, 0) / 16.0;
	yaw_rate = read_register(DREG_EULER_PSI_DOT, 1) / 16.0;
	euler_time = read_register(DREG_EULER_TIME);

	north_pos = read_register(DREG_POSITION_N);
	east_pos = read_register(DREG_POSITION_E);
	up_pos = read_register(DREG_POSITION_UP);
	pos_time = read_register(DREG_POSITION_TIME);

	north_vel = read_register(DREG_VELOCITY_N);
	east_vel = read_register(DREG_VELOCITY_E);
	up_vel = read_register(DREG_VELOCITY_UP);
	vel_time = read_register(DREG_VELOCITY_TIME);
}

void MYUM7SPI::get_vals_data() {
	gyro_x = read_register(DREG_GYRO_PROC_X);
	gyro_y = read_register(DREG_GYRO_PROC_Y);
	gyro_z = read_register(DREG_GYRO_PROC_Z);

	accel_x = read_register(DREG_ACCEL_PROC_X);
	accel_y = read_register(DREG_ACCEL_PROC_Y);
	accel_z = read_register(DREG_ACCEL_PROC_Z);

	roll = read_register(DREG_EULER_PHI_THETA, 1) / 91.02222;
	pitch = read_register(DREG_EULER_PHI_THETA, 0) / 91.02222;
	yaw = read_register(DREG_EULER_PSI, 1) / 91.02222;

	// Include these datasets to get to 128 B/transfer
	roll_rate = read_register(DREG_EULER_PHI_THETA_DOT, 1) / 16.0;
	pitch_rate = read_register(DREG_EULER_PHI_THETA_DOT, 0) / 16.0;
	yaw_rate = read_register(DREG_EULER_PSI_DOT, 1) / 16.0;
}

//////////////////////////////////
//		COMMAND FUNCTIONS		//
//////////////////////////////////

/*
	Causes the UM7 to load default factory settings.
*/
int32_t MYUM7SPI::get_firmware() {
	return read_register(GET_FW_REVISION);
}

/*
	Causes the UM7 to load default factory settings.
*/
void MYUM7SPI::flash_commit() {
	write_register(FLASH_COMMIT);
}

/*
	Causes the UM7 to load default factory settings.
*/
void MYUM7SPI::factory_reset() {
	write_register(RESET_TO_FACTORY);
}

/*
	Causes the UM7 to measure the gyro outputs and set the output trim registers to compensate for any non-zero bias. 
	The UM7 should be kept stationary while the zero operation is underway.
*/
void MYUM7SPI::zero_gyros() {
	write_register(ZERO_GYROS);
}

/*
	Sets the current GPS latitude, longitude, and altitude as the home position. 
	All future positions will be referenced to the current GPS position.
*/
void MYUM7SPI::set_home_position() {
	write_register(SET_HOME_POSITION);
}

/*
	Sets the current yaw heading position as north.
*/
void MYUM7SPI::set_mag_reference() {
	write_register(SET_MAG_REFERENCE);
}

/*
	Reboots the UM7 and performs a crude calibration on the accelerometers. Best performed on a flat surface.
*/
void MYUM7SPI::calibrate_accelerometers() {
	write_register(CALIBRATE_ACCELEROMETERS);
}

/*
	Resets the Extended Kalman Filter (EKF)
*/
void MYUM7SPI::reset_ekf() {
	write_register(RESET_EKF);
}

//////////////////////////////////
//		INTERNAL FUNCTIONS		//
//////////////////////////////////

/*
	Read a register that carries 2 datasets (euler data). Uses a user defined bool to determine which dataset to return
*/
int16_t MYUM7SPI::read_register(byte address, bool first_half) {
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
int32_t MYUM7SPI::read_register(byte address) {
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


// Used for the SD example in order to write binary data directly,
// conversion to a csv file is done after data collection. 
// This is an overloaded function to fit the various sizes of datasets from the UM7
// This function is for 32bit registers
void MYUM7SPI::read_binary_data(byte address, byte b0, byte b1, byte b2, byte b3) {
	byte inByte = 0;

	digitalWrite(cs, LOW);

	SPI.transfer(READ);
	delayMicroseconds(5);

	SPI.transfer(address);
	delayMicroseconds(5);

	b3 = SPI.transfer(0x00);
	delayMicroseconds(5);

	b2 = SPI.transfer(0x00);
	delayMicroseconds(5);

	b1 = SPI.transfer(0x00);
	delayMicroseconds(5);

	b0 = SPI.transfer(0x00);
	delayMicroseconds(5);

	digitalWrite(cs, HIGH);
	return(result);
}

// Used for the SD example in order to write binary data directly,
// conversion to a csv file is done after data collection. 
// This is an overloaded function to fit the various sizes of datasets from the UM7
// This function is for 16bit registers
void MYUM7SPI::read_binary_data(byte address, byte b0, byte b1, bool first_half) {
	byte inByte = 0;

	digitalWrite(cs, LOW);

	SPI.transfer(READ);
	delayMicroseconds(5);

	SPI.transfer(address);
	delayMicroseconds(5);

	if (!first_half) {
		SPI.transfer(0x00);
		delayMicroseconds(5);

		SPI.transfer(0x00);
		delayMicroseconds(5);
	}
	b1 = SPI.transfer(0x00);
	delayMicroseconds(5);

	b0 = SPI.transfer(0x00);
	delayMicroseconds(5);

	digitalWrite(cs, HIGH);
	return(result);
}

// Writes to a configuration register, sends the contents of "contents_" to the proper bytes.
void MYUM7SPI::write_register(byte address, uint32_t contents_) {
	intval contents;
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

// Writes to a command register. Since no contents are required, the SPI bus passes 0x00 over the MOSI line.
void MYUM7SPI::write_register(byte address) {
	digitalWrite(cs, LOW);

	SPI.transfer(WRITE);
	delayMicroseconds(5);

	SPI.transfer(address);
	delayMicroseconds(5);

	for (int i = 0; i < 4; i++) {
		SPI.transfer(0x00);
		delayMicroseconds(5);
	}

	digitalWrite(cs, HIGH);
}

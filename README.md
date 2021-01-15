MYUM7SPI Library for Arduino IDE

Revision 2: Sep 15, 2020

- Allows for configurable communication with x UM7 units on x cs pin (SPI bus)
- Contains the useful configuration and command registers
- Can read from all useful data registers
- Working on sending firmware
- Can configure the SPI r/w rate 

		CONFIGURATION REGISTERS			

CREG_COM_SETTINGS
CREG_COM_RATES1
CREG_COM_RATES2
CREG_COM_RATES3
CREG_COM_RATES4
CREG_COM_RATES5
CREG_COM_RATES6
CREG_COM_RATES7
CREG_MISC_SETTING

CREG_HOME_NORTH
CREG_HOME_EAST
CREG_HOME_UP

CREG_GYRO_TRIM_X 
CREG_GYRO_TRIM_Y 
CREG_GYRO_TRIM_Z 

CREG_MAG_CAL1_1
CREG_MAG_CAL1_2
CREG_MAG_CAL1_3
CREG_MAG_CAL2_1
CREG_MAG_CAL2_2
CREG_MAG_CAL2_3
CREG_MAG_CAL3_1
CREG_MAG_CAL3_2
CREG_MAG_CAL3_3

CREG_MAG_BIAS_X
CREG_MAG_BIAS_Y
CREG_MAG_BIAS_Z

CREG_ACCEL_CAL1_1
CREG_ACCEL_CAL1_2
CREG_ACCEL_CAL1_3
CREG_ACCEL_CAL2_1
CREG_ACCEL_CAL2_2
CREG_ACCEL_CAL2_3
CREG_ACCEL_CAL3_1
CREG_ACCEL_CAL3_2
CREG_ACCEL_CAL3_3

CREG_ACCEL_BIAS_X
CREG_ACCEL_BIAS_Y
CREG_ACCEL_BIAS_Z

		    DATA REGISTERS			

DREG_HEALTH
DREG_GYRO_RAW_XY
DREG_GYRO_RAW_Z
DREG_GYRO_RAW_TIME
DREG_ACCEL_RAW_XY
DREG_ACCEL_RAW_Z
DREG_ACCEL_RAW_TIME
DREG_MAG_RAW_XY
DREG_MAG_RAW_Z
DREG_MAG_RAW_TIME
DREG_TEMPERATURE
DREG_TEMPERATURE_TIME

DREG_GYRO_PROC_X
DREG_GYRO_PROC_Y
DREG_GYRO_PROC_Z
DREG_GYRO_PROC_TIME
DREG_ACCEL_PROC_X
DREG_ACCEL_PROC_Y
DREG_ACCEL_PROC_Z
DREG_ACCEL_PROC_TIME
DREG_MAG_PROC_X
DREG_MAG_PROC_Y
DREG_MAG_PROC_Z
DREG_MAG_PROC_TIME

DREG_QUAT_AB
DREG_QUAT_CD
DREG_QUAT_TIME
DREG_EULER_PHI_THETA
DREG_EULER_PSI
DREG_EULER_PHI_THETA_DOT
DREG_EULER_PSI_DOT
DREG_EULER_TIME
DREG_POSITION_N
DREG_POSITION_E
DREG_POSITION_UP
DREG_POSITION_TIME
DREG_VELOCITY_N
DREG_VELOCITY_E
DREG_VELOCITY_UP
DREG_VELOCITY_TIME

DREG_GPS_LATITUDE
DREG_GPS_LONGITUDE
DREG_GPS_ALTITUDE
DREG_GPS_COURSE
DREG_GPS_SPEED
DREG_GPS_TIME
DREG_GPS_SAT_1_2
DREG_GPS_SAT_3_4
DREG_GPS_SAT_5_6
DREG_GPS_SAT_7_8
DREG_GPS_SAT_9_10
DREG_GPS_SAT_11_12

DREG_GYRO_BIAS_X
DREG_GYRO_BIAS_Y
DREG_GYRO_BIAS_Z

		    COMMAND REGISTERS			

GET_FW_REVISION
FLASH_COMMIT
RESET_TO_FACTORY
ZERO_GYROS
SET_HOME_POSITION
SET_MAG_REFERENCE
CALIBRATE_ACCELEROMETERS
RESET_EKF

		    ACCESIBLE VARIABLES			

*** RAW VARIABLES ***
int16_t 	gyro_raw_x, gyro_raw_y, gyro_raw_z;
int16_t 	accel_raw_x, accel_raw_y, accel_raw_z;
int16_t 	mag_raw_x, mag_raw_y, mag_raw_z;
float 		temp, temp_time;
float 		gyro_raw_time, accel_raw_time, mag_raw_time;

*** PROCESSED VARIABLES ***
float 		gyro_x, gyro_y, gyro_z, gyro_time;
float 		accel_x, accel_y, accel_z, accel_time;
float 		mag_x, mag_y, mag_z, mag_time;

*** ORIENTATION VARIABLES ***
int16_t 	quat_a, quat_b, quat_c, quat_d, quat_time;
int16_t 	roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate;
float 		euler_time;

*** POSITION/VELOCITY VARIABLES ***
float 		north_pos, east_pos, up_pos, pos_time;
float 		north_vel, east_vel, up_vel, vel_time;

		    INTERNAL VARIABLES			

int		cs;

		    ACCESIBLE FUNCTIONS			

// Default constructor. Initializes cs pin and sets it as an output. Also inits the SPI rate for r/w transfer
MYUM7SPI(uint16_t cs_, uint32_t rate_)

// Sets the rate for all raw datasets to the same desired rate
set_all_raw_rate(uint8_t rate)

// Sets the rate for all processed datasets to the same desired rate
set_all_processed_rate(uint8_t rate)

// Sets the rate for all quaternion, euler, position, and velocity datasets. This is an overloaded function.
set_orientation_rate(byte quat_rate, byte euler_rate, byte pos_rate, byte vel_rate)

// Sets the rate for all quaternion, euler, and position datasets. This is an overloaded function.
set_orientation_rate(byte quat_rate, byte euler_rate, byte pos_rate)

// Sets the rate for all quaternion and euler datasets. This is an overloaded function.
set_orientation_rate(byte quat_rate, byte euler_rate)

// Sets the rate for quaternion datasets. This is an overloaded function.
set_orientation_rate(byte quat_rate)

// Miscellaneous settings for filter and sensor control options. Send a 0 if you don't wish to configure a specific setting
// Ex. set_misc_settings(0, 1, 1, 0)
//
// PPS bit = Causes the TX2/RX2 pin to be used with an external GPS
// ZG bit = Causes UM7 to measure gyro bias at setup
// Q bit = Sensor will run in Quternion mode instead of Euler mode. Fixes pitch error in the Gimbal lock position
// MAG bit = Magnetometer will be used in state updates
set_misc_settings(bool pps, bool zg, bool q, bool mag)

// Causes UM7 to transmit a packet containing the firmware revision string (a 4B char sequence)
get_firmware()

// Causes the UM7 to write all configuration settings to FLASH so that they will remain when the power is cycled.
save_configs_to_flash()

// Causes the UM7 to load default factory settings.
factory_reset()

// Causes the UM7 to measure the gyro outputs and set the output trim registers to compensate for any non-zero bias.
// The UM7 should be kept stationary while the zero operation is underway.
zero_gyros()

// Sets the current GPS latitude, longitude, and altitude as the home position.
// All future positions will be referenced to the current GPS position.
set_home_position()

// Sets the current yaw heading position as north.
set_mag_reference()

// Reboots the UM7 and performs a crude calibration on the accelerometers. Best performed on a flat surface.
calibrate_accelerometers()

// Resets the EKF. Extended Kalman Filter (EKF)
reset_kalman_filter()

		    INTERNAL FUNCTIONS

// Read a register that carries 2 datasets (euler data). Uses a user defined bool to determine which dataset to return.
read_register(uint16_t address, bool first_half)

// Read from a register. Assume register takes an entire 4 Bytes and is a float point type.
read_register(uint16_t address)

// Writes to a configuration register, sends the contents of "contents_" to the proper bytes.
write_register(byte address, uint32_t contents_)

// Writes to a command register. Since no contents are required, the SPI bus passes 0x00 over the MOSI line.
write_register(byte address)

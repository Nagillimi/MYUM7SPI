// Avoid IDE problems by defining struct in septate .h file.
// Pad record so size is a power of two for best write performance.
/*
  Size of the total logged dataset in bits:

 | PACKET # | TIME | FSR_HEEL | FSR_TOE | IMU_1 | IMU_2 | IMU_3 |
 |    32    |  32  |    16    |    16   |  240  |  240  |  240  |

 = 816 bits = 102 Bytes

 Note:
 - Should pad until 128 Bytes for best logging performance.
*/
#ifndef ExFatLogger_h
#define ExFatLogger_h

#include "MYUM7SPI.h"

// Init um7s at 10MHz (max)
MYUM7SPI imu1(6, 10000000); // cs pin 1
MYUM7SPI imu2(9, 10000000); // cs pin 2
MYUM7SPI imu3(4, 10000000); // cs pin 3

#define UM7_MOSI_PIN 11
#define UM7_MISO_PIN 12
#define UM7_SCK_PIN 13

// FSR analog pins
// Make sure they aren't any SPI bus pins
int fsr_heel_pin = A8, fsr_toe_pin = A9;

// Collection of data custom for application
// Note: delta is NOT part of data_t, it's computed during conversion based on "t"
struct data_t {
	// 98 Byte data transfer:
	uint32_t t;
	uint16_t fsr_heel;
	uint16_t fsr_toe;
	float gx_1;
	float gy_1;
	float gz_1;
	float ax_1;
	float ay_1;
	float az_1;
	int16_t roll_1;
	int16_t pitch_1;
	int16_t yaw_1;
	float gx_2;
	float gy_2;
	float gz_2;
	float ax_2;
	float ay_2;
	float az_2;
	int16_t roll_2;
	int16_t pitch_2;
	int16_t yaw_2;
	float gx_3;
	float gy_3;
	float gz_3;
	float ax_3;
	float ay_3;
	float az_3;
	int16_t roll_3;
	int16_t pitch_3;
	int16_t yaw_3;

	// Variable to fill in the transfer to 128 Bytes (30B difference).
	// 16b * 15 = 240b = 30 Bytes
	uint16_t whitespace[15];
};
#endif  // ExFatLogger_h

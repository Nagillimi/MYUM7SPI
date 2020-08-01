// Avoid IDE problems by defining struct in septate .h file.
// Pad record so size is a power of two for best write performance.
#ifndef ExFatLogger_h
#define ExFatLogger_h

#include "MYUM7SPI.h"

MYUM7SPI imu1(6); // cs pin 1
MYUM7SPI imu2(9); // cs pin 2

// FSR analog pins
// Make sure they aren't any SPI bus pins
int fsr_heel_pin = A8, fsr_toe_pin = A9;

// Collection of data custom for application
struct data_t {
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
};
#endif  // ExFatLogger_h

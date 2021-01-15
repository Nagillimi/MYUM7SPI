#include "MYUM7SPI.h"

// Init 3 UM7's at 10MHz
MYUM7SPI imu1(6, 10000000); // cs pin 1
MYUM7SPI imu2(9, 10000000); // cs pin 2
MYUM7SPI imu3(4, 10000000); // cs pin 3

#define UM7_MOSI_PIN 11
#define UM7_MISO_PIN 12
#define UM7_SCK_PIN 13

void setup() {
  imu1.factory_reset();
  imu2.factory_reset();
  imu3.factory_reset();
}

void loop() {}

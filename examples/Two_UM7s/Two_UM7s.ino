/* Arduino Example for reading from two UM7 sensors over the SPI bus
 * Adaptation from the <MYUM7.h> library
 */
#include <MYUM7SPI.h>

// Init the um7's at 10MHz
MYUM7SPI imu1(37, 10000000); // chip select pin for UM7 #1
MYUM7SPI imu2(36, 10000000); // chip select pin for UM7 #2

void setup() {
  Serial.begin(115200);
  while (!Serial); // Serial acts as a on switch

  SPI.begin();

  imu1.set_all_processed_rate(255);
  delay(100);
  imu2.set_all_processed_rate(255);
  delay(100);

  imu1.set_orientation_rate(255, 255);
  delay(100);
  imu2.set_orientation_rate(255, 255);
  delay(100);
  
//   imu1.calibrate_accelerometers();
//   delay(100);
//   imu2.calibrate_accelerometers();
//   delay(100);

//   imu1.zero_gyros();
//   delay(100);
//   imu2.zero_gyros();
//   delay(100);
}

void loop() {
  /* Will overload Serial, even at max 115200 buad.
   *  (3*4 + 3*4 + 3*2)bytes * 2sensors * 255Hz = 15,300 B/s
   *  
   *  115200 baud = 11,520 B/s (with start/stop bits)
   *              = 14400 B/s (without start/stop bits)
   *              
   *  You'll need an SD card to see all these values properly at these rates. Otherwise, comment out undesired data.
   */
  imu1.get_all_orientation_data();
  imu1.get_all_processed_data();
  Serial.print(imu1.gyro_x); Serial.print(",");
  Serial.print(imu1.gyro_y); Serial.print(",");
  Serial.print(imu1.gyro_z); Serial.print(",");

  Serial.print(imu1.accel_x); Serial.print(",");
  Serial.print(imu1.accel_y); Serial.print(",");
  Serial.print(imu1.accel_z); Serial.print(",");
  
  Serial.print(imu1.roll); Serial.print(",");
  Serial.print(imu1.pitch); Serial.print(",");
  Serial.print(imu1.yaw); Serial.print(",");

  imu2.get_all_orientation_data();
  Serial.print(imu2.gyro_x); Serial.print(",");
  Serial.print(imu2.gyro_y); Serial.print(",");
  Serial.print(imu2.gyro_z); Serial.print(",");

  Serial.print(imu2.accel_x); Serial.print(",");
  Serial.print(imu2.accel_y); Serial.print(",");
  Serial.print(imu2.accel_z); Serial.print(",");
  
  Serial.print(imu2.roll); Serial.print(",");
  Serial.print(imu2.pitch); Serial.print(",");
  Serial.println(imu2.yaw);
}

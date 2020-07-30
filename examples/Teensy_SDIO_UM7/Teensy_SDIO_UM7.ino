/* Arduino example for a Teensy Low Latency Logger using SDIO
   Project wouldn't be possible without:
   - Bill Greiman's TeensySdioDemo
     (https://github.com/greiman/SdFat-beta)
   - Phillip Johnston's Flexible Logging Library for Arduino
     (https://b9f1cb8b0e.nxcli.net/blog/2020/01/13/building-a-flexible-logging-library-for-arduino-part-3/)
   - https://www.youtube.com/watch?v=zA5sr0qXsaU
   - Matthew Ford's High Frequency, Long Duration Datalogging ...
     (https://www.forward.com.au/pfod/ArduinoProgramming/DataRate/SD_logging/HighFreqLongDur.html)

   An implementation of the UM7's SPI mode to allow for logging of:
   - 3 UM7's @ 250 Hz
   - 2 FSR's @ 500 Hz

*/

#include "SdFat.h"
#include "MYUM7SPI.h"
#include "Parameters.h"

byte fsr_heel_pin = A0, fsr_heel_pin = A1;
uint16_t fsr_heel, fsr_toe;

MYUM7SPI thigh_imu(36);
MYUM7SPI shank_imu(37);
MYUM7SPI foot_imu(38);

char file_name[] = "SdioLogger.txt";

void setup() {
	Serial.begin(9600);
	while (!Serial); // Serial acts as an on switch

	// Setup sensors
	pinMode(A0, INPUT);
	pinMode(A1, INPUT);
	setup_imus(250);

	// Start SD card
	if (!sd.begin(SdioConfig(FIFO_SDIO))) {
		errorHalt("SD begin failed");
	}
}

char c, d;
void loop() {
	// First message
	Serial.println("\nType '1' to begin logging in FIFO SDIO mode.");
	// Wait for Serial input
	while (!Serial.available()) {
	}
	// Entering logging mode
	c = Serial.read();
	if (c == '1') {
		Serial.println(
			"\nFIFO SDIO mode."
			"\nPress '2' to stop logging."
		);

		// Open file as WRITE ONLY
		if (!file.open(file_name, O_WRITE | O_CREAT)) {
			errorHalt("file open failed");
		}

		// Format file
		if (!file.truncate()) {
			errorHalt("truncate failed")
		}

		do {
			while (!Serial.available()) {
			}
			d = Serial.read();
			// Entering logging mode
			log_data();
		} while (d != '2');
		file.close();

	} else {
		Serial.println("Invalid input");
		return;
	}
}

void setup_imus(byte rate_) {
	SPI.begin();

	thigh_imu.set_all_processed_rate(rate_);
	shank_imu.set_all_processed_rate(rate_);
	foot_imu.set_all_processed_rate(rate_);

	thigh_imu.set_orientation_rate(rate_);
	shank_imu.set_orientation_rate(rate_);
	foot_imu.set_orientation_rate(rate_);

	thigh_imu.calibrate_accelerometers();
	shank_imu.calibrate_accelerometers();
	foot_imu.calibrate_accelerometers();

	thigh_imu.zero_gyros();
	shank_imu.zero_gyros();
	foot_imu.zero_gyros();
}

void errorHalt(const char* msg) {
	Serial.print("Error: ");
	Serial.println(msg);
	if (sd.sdErrorCode()) {
		if (sd.sdErrorCode() == SD_CARD_ERROR_ACMD41) {
			Serial.println("Try power cycling the SD card.");
		}
		printSdErrorSymbol(&Serial, sd.sdErrorCode());
		Serial.print(", ErrorData: 0X");
		Serial.println(sd.sdErrorData(), HEX);
	}
	while (true) {}
}

// UM7's will just duplicate the data, since this will operate at 500 Hz
// Comes out to be 122 B/transfer = 61KB/s
void log_data() {

}

void flush() {
	// Make sure file is open
	if (!file.open(file_name, O_WRITE | O_CREAT)) {
		errorHalt("file open failed");
	}

	// Find the size of the file
	unsigned long count = file.size();
	// Write the buffer contents to the SD card
	if (count != file.write(buf, count)) {
		errorHalt("Failed to write to log file");
	}
}
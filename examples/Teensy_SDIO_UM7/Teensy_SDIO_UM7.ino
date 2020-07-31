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
   
   Notes
   1. A txt file is used instead of a bin file since datasets have three sizes
      and the highest dataset value is used for each variable sent to the buffer (4B).
	  Shown:
		  comma = 1B
		  euler data & analog data = 2B
		  gyro data & accel data = 4B
	2. 
*/

#include "SdFat.h"
#include "MYUM7SPI.h"
#include "Parameters.h"

byte fsr_heel_pin = A0, fsr_heel_pin = A1;
uint16_t fsr_heel, fsr_toe;

MYUM7SPI thigh_imu(36);
MYUM7SPI shank_imu(37);
MYUM7SPI foot_imu(38);

char bin_file_name[] = "SdioLogger.bin";
char csv_file_name[] = "Log.csv";

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
	Serial.println(
		"\nType '1' to begin logging in FIFO SDIO mode."
		"\n     '2' to convert binary file to csv file."
		);
	// Wait for Serial input
	while (!Serial.available()) {
	}
	// Entering logging mode
	c = Serial.read();
	if (c == '1') {
		Serial.println(
			"\nFIFO SDIO mode."
			"\nType any character to stop logging."
		);

		// Open file as WRITE ONLY
		if (!binFile.open(bin_file_name, O_WRITE | O_CREAT)) {
			errorHalt("file open failed");
		}

		// Format file
		if (!file.truncate()) {
			errorHalt("truncate failed")
		}

		log_data();
		file.close();
	} else if (c == '2') {
		bin_to_csv();
	} else {
		Serial.println("Invalid input");
		return;
	}
	delay(2000);
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
	int32_t delta;
	
	// Wait until SD is not busy. Included in SdFat library
	while (sd.card()->isBusy()) {}

	// Start time for log file.
	uint32_t m = millis();

	// Time to log next record.
	uint32_t logTime = micros();
	while (1) {
		// Time for next data record.
		logTime += LOG_INTERVAL_USEC;

		delta = micros() - logTime;
		if (delta > 0) {
			Serial.print(F("delta: "));
			Serial.println(delta);
			error("Rate too fast");
		}
		// Wait until time to log data.
		while (delta < 0) {
			delta = micros() - logTime;
		}

		for (int i = 0; i < FIFO_DIM; i += DATA_BYTE_WRITE_SIZE) {
			// Capture the imu data from the UM7s
			thigh_imu.get_vals_data();
			shank_imu.get_vals_data();
			foot_imu.get_vals_data();

			// 29 * 2 * 32 bits / 8 bits per Byte = 232 Bytes/transfer without
			// parsing. It's 464 Bytes/transfer with commas & newlines.
			// This leaves 48 Bytes in the 512 Byte buffer...
			// Get better!!

			// FSR analog data
			buf32[i] = analogRead(heel_fsr); buf32[i++] = ',';
			byf32[i++] = analogRead(toe_fsr); buf32[i++] = ',';
			// thigh data
			buf32[i++] = thigh_imu.gyro_x; buf32[i++] = ',';
			buf32[i++] = thigh_imu.gyro_y; buf32[i++] = ',';
			buf32[i++] = thigh_imu.gyro_z; buf32[i++] = ',';
			buf32[i++] = thigh_imu.accel_x; buf32[i++] = ',';
			buf32[i++] = thigh_imu.accel_y; buf32[i++] = ',';
			buf32[i++] = thigh_imu.accel_z; buf32[i++] = ',';
			buf32[i++] = thigh_imu.roll; buf32[i++] = ',';
			buf32[i++] = thigh_imu.pitch; buf32[i++] = ',';
			buf32[i++] = thigh_imu.yaw; buf32[i++] = ',';
			// shank data
			buf32[i++] = shank_imu.gyro_x; buf32[i++] = ',';
			buf32[i++] = shank_imu.gyro_y; buf32[i++] = ',';
			buf32[i++] = shank_imu.gyro_z; buf32[i++] = ',';
			buf32[i++] = shank_imu.accel_x; buf32[i++] = ',';
			buf32[i++] = shank_imu.accel_y; buf32[i++] = ',';
			buf32[i++] = shank_imu.accel_z; buf32[i++] = ',';
			buf32[i++] = shank_imu.roll; buf32[i++] = ',';
			buf32[i++] = shank_imu.pitch; buf32[i++] = ',';
			buf32[i++] = shank_imu.yaw; buf32[i++] = ',';
			// foot data
			buf32[i++] = foot_imu.gyro_x; buf32[i++] = ',';
			buf32[i++] = foot_imu.gyro_y; buf32[i++] = ',';
			buf32[i++] = foot_imu.gyro_z; buf32[i++] = ',';
			buf32[i++] = foot_imu.accel_x; buf32[i++] = ',';
			buf32[i++] = foot_imu.accel_y; buf32[i++] = ',';
			buf32[i++] = foot_imu.accel_z; buf32[i++] = ',';
			buf32[i++] = foot_imu.roll; buf32[i++] = ',';
			buf32[i++] = foot_imu.pitch; buf32[i++] = ',';
			buf32[i++] = foot_imu.yaw; buf32[i++] = '\n';
		}
		flush();

		// Character typed over Serial triggers program stop
		if (Serial.available()) {
			break;
		}
	}
	Serial.print(F("\nLog time: "));
	Serial.print(0.001*(millis() - m));
	Serial.println(F(" Seconds"));

	Serial.print(("File size: "));
	// Warning cast used for print since fileSize is uint64_t.
	Serial.print((uint32_t)binFile.fileSize());
}

void flush() {
	// Write data if SD is not busy.
	if (!sd.card()->isBusy()) {
		// Find the size of the file
		unsigned long count = file.size();
		// Write the buffer contents to the SD card
		if (count != binfile.write(buf, count)) {
			errorHalt("Failed to write to log file");
		}
	}
}

// Similar to printRecord() function in ExFatLogger from SdFat library
void bin_to_csv() {
	// Check to see if a binary file is open
	if (!binFile.isOpen()) {
		Serial.println(F("No current binary file"));
		return false;
	}
	
	// Open the csv file as WRONLY
	if (!csvFile.open(csvName, O_WRONLY | O_CREAT | O_TRUNC)) {
		error("open csvFile failed");
	}
	
	// Set the seek to 512 Bytes
	if (!binFile.seekSet(512)) {
		error("binFile.seek faile");
	}

	while (!Serial.available() && binFile.available()) {
		int nb = binFile.read(binData, sizeof(binData));
		if (nb <= 0 ) {
			error("read binFile failed");
		}
		size_t nr = nb/sizeof(data_t);
		for (size_t i = 0; i < nr; i++) {
			printRecord(&csvFile, &binData[i]);
		}

		// Printing the % over Serial
		if ((millis() - tPct) > 1000) {
		uint8_t pct = binFile.curPosition()/(binFile.fileSize()/100);
		if (pct != lastPct) {
			tPct = millis();
			lastPct = pct;
			Serial.print(pct, DEC);
			Serial.println('%');
			csvFile.sync();
		}
    }

	csvFile.close();
}
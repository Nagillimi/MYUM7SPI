/* Arduino example for a Teensy Low Latency Logger using SDIO
   Project wouldn't be possible without:
   - Bill Greiman's TeensySdioDemo
     (https://github.com/greiman/SdFat-beta)
   - Phillip Johnston's Flexible Logging Library for Arduino
     (https://b9f1cb8b0e.nxcli.net/blog/2020/01/13/building-a-flexible-logging-library-for-arduino-part-3/)
   - https://www.youtube.com/watch?v=zA5sr0qXsaU
   - Matthew Ford's High Frequency, Long Duration Datalogging project
     (https://www.forward.com.au/pfod/ArduinoProgramming/DataRate/SD_logging/HighFreqLongDur.html)

   An implementation of the UM7's SPI mode to allow for logging of:
   - 3 UM7's @ 250 Hz
   - 2 FSR's @ 500 Hz
   
   Notes
   1. A text file was used originally instead of a binary file since the datasets had three sizes.
      The highest dataset value was used for each variable sent to the buffer (4B), which resulted
	  in a massive read function (464 Bytes with commas for one transfer).
	  
	  A binary file is now being used to save memory space in the program and to allow for more
	  efficient SD writes. An entire transfer now only takes up 95 Bytes
	2. There's no overrun check in this example, so missed packets aren't tracked
	3. 
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

struct data_t {
	// Temporary storage variables for converion
	int16_t 
			fsr_heel_, fsr_toe_, 
			thigh_roll, thigh_pitch, thigh_yaw,
			shank_roll, shank_pitch, shank_yaw,
			foot_roll, foot_pitch, foot_yaw;
		int32_t
			thigh_gx, thigh_gy, thigh_gz, thigh_ax, thigh_ay, thigh_az,
			shank_gx, shank_gy, shank_gz, shank_ax, shank_ay, shank_az,
			foot_gx, foot_gy, foot_gz, foot_ax, foot_ay, foot_az;
};

void setup() {
	Serial.begin(9600);
	while (!Serial); // Serial acts as an on switch

	// Setup sensors
	pinMode(A0, INPUT);
	pinMode(A1, INPUT);
	setup_imus(250);

	// Init SD card
	if (!sd.begin(SdioConfig(FIFO_SDIO))) {
		errorHalt("SD begin failed");
	}
}

void loop() {
	char c, d;
	// Reoccuring message
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
		if (!binFile.open(bin_file_name, O_RDWR | O_CREAT)) {
			errorHalt("binary file open failed");
		}

		// Format file
		if (!file.truncate()) {
			errorHalt("truncate failed")
		}

		log_data();
		file.close();
	} else if (c == '2') {
		Serial.println(
			"\nConverting binary file to csv file."
			"\nType any character to stop conversion."
		);
		bin_to_csv();
	} else {
		Serial.println("Invalid input");
		return;
	}
	delay(2000);
}

// Sets up the 3 UM7s processed and euler rates
// Calibrates all the accels and zeroes all the gyros
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

// Generic error function, searches the SdFat lib for error codes and prints them along 
// with the passed message (msg)
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
	
	// Loop runs forever unless any character is typed or max file size is hit
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
			// Capture FSR analog data			
			heel_fsr = analogRead(heel_fsr_pin);
			toe_fsr = analogRead(toe_fsr_pin);

			// FSR and euler data are only 16bits, but they take up 32 to sync with the buf32 array
			// Total data size is 94 Bytes

			// FSR data
			buf[i] = (byte)(heel_fsr & 0xFF);
			buf[i++] = (byte)((heel_fsr >> 8) & 0xFF);
			buf[i++] = (byte)(toe_fsr & 0xFF);
			buf[i++] = (byte)((toe_fsr >> 8) & 0xFF);
			//  Capture thigh imu data
			thigh_imu.read_binary_data(DREG_GYRO_PROC_X, buf[i++], buf[i++], buf[i++], buf[i++]);
			thigh_imu.read_binary_data(DREG_GYRO_PROC_Y, buf[i++], buf[i++], buf[i++], buf[i++]);
			thigh_imu.read_binary_data(DREG_GYRO_PROC_Z, buf[i++], buf[i++], buf[i++], buf[i++]);
			thigh_imu.read_binary_data(DREG_ACCEL_PROC_X, buf[i++], buf[i++], buf[i++], buf[i++]);
			thigh_imu.read_binary_data(DREG_ACCEL_PROC_Y, buf[i++], buf[i++], buf[i++], buf[i++]);
			thigh_imu.read_binary_data(DREG_ACCEL_PROC_Z, buf[i++], buf[i++], buf[i++], buf[i++]);
			thigh_imu.read_binary_data(DREG_EULER_PHI_THETA, buf[i++], buf[i++], buf[i++], buf[i++]);
			thigh_imu.read_binary_data(DREG_EULER_PSI, buf[i++], buf[i++], true);
			//  Capture shank imu data
			shank_imu.read_binary_data(DREG_GYRO_PROC_X, buf[i++], buf[i++], buf[i++], buf[i++]);
			shank_imu.read_binary_data(DREG_GYRO_PROC_Y, buf[i++], buf[i++], buf[i++], buf[i++]);
			shank_imu.read_binary_data(DREG_GYRO_PROC_Z, buf[i++], buf[i++], buf[i++], buf[i++]);
			shank_imu.read_binary_data(DREG_ACCEL_PROC_X, buf[i++], buf[i++], buf[i++], buf[i++]);
			shank_imu.read_binary_data(DREG_ACCEL_PROC_Y, buf[i++], buf[i++], buf[i++], buf[i++]);
			shank_imu.read_binary_data(DREG_ACCEL_PROC_Z, buf[i++], buf[i++], buf[i++], buf[i++]);
			shank_imu.read_binary_data(DREG_EULER_PHI_THETA, buf[i++], buf[i++], buf[i++], buf[i++]);
			shank_imu.read_binary_data(DREG_EULER_PSI, buf[i++], buf[i++], true);
			//  Capture foot imu data
			foot_imu.read_binary_data(DREG_GYRO_PROC_X, buf[i++], buf[i++], buf[i++], buf[i++]);
			foot_imu.read_binary_data(DREG_GYRO_PROC_Y, buf[i++], buf[i++], buf[i++], buf[i++]);
			foot_imu.read_binary_data(DREG_GYRO_PROC_Z, buf[i++], buf[i++], buf[i++], buf[i++]);
			foot_imu.read_binary_data(DREG_ACCEL_PROC_X, buf[i++], buf[i++], buf[i++], buf[i++]);
			foot_imu.read_binary_data(DREG_ACCEL_PROC_Y, buf[i++], buf[i++], buf[i++], buf[i++]);
			foot_imu.read_binary_data(DREG_ACCEL_PROC_Z, buf[i++], buf[i++], buf[i++], buf[i++]);
			foot_imu.read_binary_data(DREG_EULER_PHI_THETA, buf[i++], buf[i++], buf[i++], buf[i++]);
			foot_imu.read_binary_data(DREG_EULER_PSI, buf[i++], buf[i++], true);
			// Print a newline character for parsing. Try without it first, can just say it's 94 Bytes
			// buf[i++] = ',';
		}
		// Flush the 512 Byte buffer to the SD card
		flush();

		// Character typed over Serial triggers function stop
		if (Serial.available()) {
			break;
		}
	}
	Serial.print(F("\nLog time: "));
	Serial.print(0.001*(millis() - m));
	Serial.println(F(" Seconds"));

	Serial.print(("File size: "));
	// Warning cast used for print since fileSize is uint64_t.
	// Could use sizeof()
	Serial.print((uint32_t)binFile.fileSize());
}

// Flushes the filled buffer to the SD card
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
// Converts bin files to csv files custom for this application
// May need to use a console application for large files
void bin_to_csv() {
	// Use the hard-coded data_t type to track and parse bytes from binary file
	data_t binData[FIFO_DIM];
	bool header = true;

	// Check to see if a binary file is open
	if (!binFile.isOpen()) {
		Serial.println(F("No current binary file"));
		return false;
	}
	
	// Open the csv file as WRONLY
	if (!csvFile.open(csvName, O_WRONLY | O_CREAT | O_TRUNC)) {
		errorHalt("csv file open failed");
	}
	
	// Set the seek to 512 Bytes
	if (!binFile.seekSet(512)) {
		errorHalt("binFile.seek failed");
	}
	
	// overhead timestamp for conversion progress
	uint32_t tPct = millis();

	// Loop runs unless csv converion is complete or a character is typed
	while (true) {		
		// Read from binary file. nb is the total size of the logging session in bytes (I think)
		int nb = binFile.read(binData, sizeof(binData));

		// Return error if 0 or -1 is read (no data in binary)
		if (nb <= 0 ) {
			errorHalt("read binFile failed");
		}
		
		// nr is the number of instances found for binData, should be big
		size_t nr = nb/sizeof(data_t);
		// Write to csv file
		for (size_t i = 0; i < nr; i++) {
			// Prints the header for the csv file
			if(header) {
				csvFile.print(F("LOG_INTERVAL_USEC,"));
				csvFile.println(LOG_INTERVAL_USEC);
				csvFile.print(F(
					"TIME,FSR_HEEL,FSR_TOE,"
					"THIGH_GX,THIGH_GY,THIGH_GZ,THIGH_AX,THIGH_AY,THIGH_AZ,THIGH_ROLL,THIGH_PITCH,THIGH_YAW,"
					"SHANK_GX,SHANK_GY,SHANK_GZ,SHANK_AX,SHANK_AY,SHANK_AZ,SHANK_ROLL,SHANK_PITCH,SHANK_YAW,"
					"FOOT_GX,FOOT_GY,FOOT_GZ,FOOT_AX,FOOT_AY,FOOT_AZ,FOOT_ROLL,FOOT_PITCH,FOOT_YAW"
				));
				csvFile.println();
				header = false;
			}
			// Print contents of binData to csv file one block at a time
			// Data sizes should line up accordingly to the data_t variables
			csvFile.print(binData.heel_fsr_); csvFile.write(',');
			csvFile.print(binData.toe_fsr_); csvFile.write(',');

			csvFile.print(binData.thigh_gx); csvFile.write(',');
			csvFile.print(binData.thigh_gy); csvFile.write(',');
			csvFile.print(binData.thigh_gz); csvFile.write(',');
			csvFile.print(binData.thigh_ax); csvFile.write(',');
			csvFile.print(binData.thigh_ay); csvFile.write(',');
			csvFile.print(binData.thigh_az); csvFile.write(',');
			csvFile.print(binData.thigh_roll); csvFile.write(',');
			csvFile.print(binData.thigh_pitch); csvFile.write(',');
			csvFile.print(binData.thigh_yaw); csvFile.write(',');

			csvFile.print(binData.shank_gx); csvFile.write(',');
			csvFile.print(binData.shank_gy); csvFile.write(',');
			csvFile.print(binData.shank_gz); csvFile.write(',');
			csvFile.print(binData.shank_ax); csvFile.write(',');
			csvFile.print(binData.shank_ay); csvFile.write(',');
			csvFile.print(binData.shank_az); csvFile.write(',');
			csvFile.print(binData.shank_roll); csvFile.write(',');
			csvFile.print(binData.shank_pitch); csvFile.write(',');
			csvFile.print(binData.shank_yaw); csvFile.write(',');

			csvFile.print(binData.foot_gx); csvFile.write(',');
			csvFile.print(binData.foot_gy); csvFile.write(',');
			csvFile.print(binData.foot_gz); csvFile.write(',');
			csvFile.print(binData.foot_ax); csvFile.write(',');
			csvFile.print(binData.foot_ay); csvFile.write(',');
			csvFile.print(binData.foot_az); csvFile.write(',');
			csvFile.print(binData.foot_roll); csvFile.write(',');
			csvFile.print(binData.foot_pitch); csvFile.write(',');
			csvFile.print(binData.foot_yaw);
			csvFile.println();
		}

		// Printing the converion progress over Serial,
		if ((millis() - tPct) > 1000) {
		uint8_t pct = binFile.curPosition()/(binFile.fileSize()/100);
		if (pct != lastPct) {
			tPct = millis();
			lastPct = pct;
			Serial.print(pct, DEC);
			Serial.println('%');
			csvFile.sync();
		}

		// Once bin file is completely read, it triggers function stop
		if (!binFile.available()) {
			Serial.println("\nBinary file conversion complete.");
			break;
		}

		// Character typed over Serial triggers function stop
		if (Serial.available()) {
			Serial.println("\ncsv file conversion aborted.");
			break;
		}
    }
	// Close csv file after write is complete
	csvFile.close();
}
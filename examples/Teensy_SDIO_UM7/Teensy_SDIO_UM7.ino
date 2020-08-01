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
	  efficient SD writes. An entire transfer now only takes up 98 Bytes.
	2. There's no overrun check in this example, so missed packets aren't tracked
	3. Alter parameters.h to change functionality
	4. 
*/

#include "SdFat.h"
#include "MYUM7SPI.h"
#include "Parameters.h"

byte fsr_heel_pin = A0, fsr_toe_pin = A1;
uint16_t fsr_heel, fsr_toe;

// Init UM7's on the associated SS/CS pin
MYUM7SPI thigh_imu(36);
MYUM7SPI shank_imu(37);
MYUM7SPI foot_imu(38);

char bin_file_name[] = "SdioLogger.bin";
char csv_file_name[] = "Log.csv";

struct data_t {
	uint32_t timestamp;
	// FSRs
	uint16_t fsr_heel_;
	uint16_t fsr_toe_;
	// Thigh
	float thigh_gx;
	float thigh_gy;
	float thigh_gz;
	float thigh_ax;
	float thigh_ay;
	float thigh_az;
	int16_t thigh_roll;
	int16_t thigh_pitch;
	int16_t thigh_yaw;
	// Shank
	float shank_gx;
	float shank_gy;
	float shank_gz;
	float shank_ax;
	float shank_ay;
	float shank_az;
	int16_t shank_roll;
	int16_t shank_pitch;
	int16_t shank_yaw;
	// Foot
	float foot_gx;
	float foot_gy;
	float foot_gz;
	float foot_ax;
	float foot_ay;
	float foot_az;
	int16_t foot_roll;
	int16_t foot_pitch;
	int16_t foot_yaw;
};

void setup() {
	Serial.begin(9600);
	while (!Serial); // Serial acts as an on switch

	// Setup sensors
	pinMode(A0, INPUT);
	pinMode(A1, INPUT);
	setup_imus(250);

	// Init SD card in FIFO_SDIO mode
	if (!sd.begin(SdioConfig(FIFO_SDIO))) {
		errorHalt("SD begin failed");
	}
}

void loop() {
	// Reoccuring message
	Serial.println(
		"\nType '1' to begin logging in FIFO SDIO mode."
		"\n     '2' to convert binary file to csv file."
		);
	// Wait for Serial input
	while (!Serial.available()) {
	}
	// Entering logging mode
	char c = Serial.read();

	if (c == '1') {
		Serial.println(
			"\nNow logging."
			"\nType any character to stop."
		);

		// Open binary file as read/write
		if (!binFile.open(bin_file_name, O_RDWR | O_CREAT)) {
			errorHalt("binary file open failed");
		}

		// Optimize file
		if (!binFile.truncate()) {
			errorHalt("truncate failed");
		}

		log_data();
		binFile.close();
	} else if (c == '2') {
		Serial.println(
			"\nConverting binary log file to csv file."
			"\nType any character to stop conversion."
		);
		bin_to_csv();
	} else {
		Serial.println("Invalid input");
		return;
	}
	delay(2000);
}
// ---------------------------------------------------------------------------------------------------------
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
// ---------------------------------------------------------------------------------------------------------
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
	// Halts program
	while (true) {}
}
// ---------------------------------------------------------------------------------------------------------
// UM7's will just duplicate the data, since this will operate at 500 Hz
// Comes out to be 98 B/transfer = 49KB/s written to the SD card
void log_data() {
	int32_t delta;
	
	// Wait until SD is not busy. Included in SdFat library
	while (sd.card()->isBusy());

	// Start time in msec for log file
	// Used to disply total logging duration
	uint32_t m = millis();

	// Time in usec to log next data transfer.
	// Separated by the preset interval in usec: LOG_INTERVAL_USEC
	uint32_t logTime = micros();
	
	// First timestamp for synchronized logging
	uint32_t t0 = logTime;

	// Loop runs forever unless any character is typed or max file size is hit
	while (1) {
		// Time for next data record.
		logTime += LOG_INTERVAL_USEC;

		// logging timestamp
		uint32_t t = micros() - t0;

		// If program can't keep up with low latency
		delta = micros() - logTime;
		if (delta > 0) {
			Serial.print(F("delta: "));
			Serial.println(delta);
			errorHalt("Rate too fast for controller, lower LOG_INTERVAL_USEC");
		}

		// Wait until time to log data.
		while (delta < 0) {
			delta = micros() - logTime;
		}

		// Loop fills up buffer with exact amount of data required to fill 512 bytes.
		// Iterates in complete dataset sized chunks (DATA_BYTE_WRITE_SIZE)
		for (uint i = 0; i < BUF_DIM; i += DATA_BYTE_WRITE_SIZE) {
			// Capture FSR analog data			
			fsr_heel = analogRead(fsr_heel_pin);
			fsr_toe = analogRead(fsr_toe_pin);

			// Data is read in as binary directly from FSRs and UM7s over analog and SPI, respectively.
			// Converts unsigned Bytes to their associated signed datasets in bin_to_csv()

			// Timestamp data, uint32_t parsed into bytes
			buf[i] = (byte)(t & 0xFF);
			buf[i+=1] = (byte)((t >> 8) & 0xFF);
			buf[i+=2] = (byte)((t >> 16) & 0xFF);
			buf[i+=3] = (byte)((t >> 24) & 0xFF);
			// FSR data, 2x uint16_t parsed into bytes
			buf[i+=4] = (byte)(fsr_heel & 0xFF);
			buf[i+=5] = (byte)((fsr_heel >> 8) & 0xFF);
			buf[i+=6] = (byte)(fsr_toe & 0xFF);
			buf[i+=7] = (byte)((fsr_toe >> 8) & 0xFF);
			//  Capture thigh imu data
			thigh_imu.read_binary_data(DREG_GYRO_PROC_X, buf[i+=8], buf[i+=9], buf[i+=10], buf[i+=11]);
			thigh_imu.read_binary_data(DREG_GYRO_PROC_Y, buf[i+=12], buf[i+=13], buf[i+=14], buf[i+=15]);
			thigh_imu.read_binary_data(DREG_GYRO_PROC_Z, buf[i+=16], buf[i+=17], buf[i+=18], buf[i+=19]);
			thigh_imu.read_binary_data(DREG_ACCEL_PROC_X, buf[i+=20], buf[i+=21], buf[i+=22], buf[i+=23]);
			thigh_imu.read_binary_data(DREG_ACCEL_PROC_Y, buf[i+=24], buf[i+=25], buf[i+=26], buf[i+=27]);
			thigh_imu.read_binary_data(DREG_ACCEL_PROC_Z, buf[i+=28], buf[i+=29], buf[i+=30], buf[i+=31]);
			thigh_imu.read_binary_data(DREG_EULER_PHI_THETA, buf[i+=32], buf[i+=33], buf[i+=34], buf[i+=35]);
			thigh_imu.read_binary_data(DREG_EULER_PSI, buf[i+=36], buf[i+=37], true);
			//  Capture shank imu data
			shank_imu.read_binary_data(DREG_GYRO_PROC_X, buf[i+=38], buf[i+=39], buf[i+=40], buf[i+=41]);
			shank_imu.read_binary_data(DREG_GYRO_PROC_Y, buf[i+=42], buf[i+=43], buf[i+=44], buf[i+=45]);
			shank_imu.read_binary_data(DREG_GYRO_PROC_Z, buf[i+=46], buf[i+=47], buf[i+=48], buf[i+=49]);
			shank_imu.read_binary_data(DREG_ACCEL_PROC_X, buf[i+=50], buf[i+=51], buf[i+=52], buf[i+=53]);
			shank_imu.read_binary_data(DREG_ACCEL_PROC_Y, buf[i+=54], buf[i+=55], buf[i+=56], buf[i+=57]);
			shank_imu.read_binary_data(DREG_ACCEL_PROC_Z, buf[i+=58], buf[i+=59], buf[i+=60], buf[i+=61]);
			shank_imu.read_binary_data(DREG_EULER_PHI_THETA, buf[i+=62], buf[i+=63], buf[i+=64], buf[i+=65]);
			shank_imu.read_binary_data(DREG_EULER_PSI, buf[i+=66], buf[i+=67], true);
			//  Capture foot imu data
			foot_imu.read_binary_data(DREG_GYRO_PROC_X, buf[i+=68], buf[i+=69], buf[i+=70], buf[i+=71]);
			foot_imu.read_binary_data(DREG_GYRO_PROC_Y, buf[i+=72], buf[i+=73], buf[i+=74], buf[i+=75]);
			foot_imu.read_binary_data(DREG_GYRO_PROC_Z, buf[i+=76], buf[i+=77], buf[i+=78], buf[i+=79]);
			foot_imu.read_binary_data(DREG_ACCEL_PROC_X, buf[i+=80], buf[i+=81], buf[i+=82], buf[i+=83]);
			foot_imu.read_binary_data(DREG_ACCEL_PROC_Y, buf[i+=84], buf[i+=85], buf[i+=86], buf[i+=87]);
			foot_imu.read_binary_data(DREG_ACCEL_PROC_Z, buf[i+=88], buf[i+=89], buf[i+=90], buf[i+=91]);
			foot_imu.read_binary_data(DREG_EULER_PHI_THETA, buf[i+=92], buf[i+=93], buf[i+=94], buf[i+=95]);
			foot_imu.read_binary_data(DREG_EULER_PSI, buf[i+=96], buf[i+=97], true);
			// Print a newline character for parsing. Try without it first, can just say it's 98 Bytes
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
	// Could use this value for bin_to_csv()?
	Serial.print((uint32_t)binFile.fileSize());
}
// ---------------------------------------------------------------------------------------------------------
// Flushes the filled buffer to the SD card
// Buffer is then overwritten instead of cleared
void flush() {
	// Write data if SD is not busy.
	if (!sd.card()->isBusy()) {
		// Find the size of the file
		unsigned long count = binFile.size();
		// Write the buffer contents to the SD card
		if (count != binFile.write(buf, count)) {
			errorHalt("Failed to write to log file");
		}
	}
}
// ---------------------------------------------------------------------------------------------------------
// Similar to printRecord() function in ExFatLogger from SdFat library
// Converts bin files to csv files custom for this application
// May need to use a console application for large files, found here:
// (https://www.forward.com.au/pfod/ArduinoProgramming/DataRate/SD_logging/SDToCSV.jar)
void bin_to_csv() {
	// Use the hard-coded data_t type to track and parse bytes from binary file
	// This should be a huge array
	data_t binData[FIFO_DIM];
	// header trace
	bool header = true;
	byte lastPct = 0;

	// Check to see if a binary file is open
	if (!binFile.isOpen()) {
		errorHalt("No current binary file");
	}
	
	// Open the csv file as write only
	if (!csvFile.open(csv_file_name, O_WRONLY | O_CREAT | O_TRUNC)) {
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
		
		// nr is the number of transfers found for binData, should be big
		// This value determines how many lines the loop below needs to run
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
			// ',' is used for csv delaminating between cells
			csvFile.print(binData[i].timestamp); csvFile.write(',');
			// FSRs
			csvFile.print(binData[i].heel_fsr_); csvFile.write(',');
			csvFile.print(binData[i].toe_fsr_); csvFile.write(',');
			// Thigh
			csvFile.print(binData[i].thigh_gx); csvFile.write(',');
			csvFile.print(binData[i].thigh_gy); csvFile.write(',');
			csvFile.print(binData[i].thigh_gz); csvFile.write(',');
			csvFile.print(binData[i].thigh_ax); csvFile.write(',');
			csvFile.print(binData[i].thigh_ay); csvFile.write(',');
			csvFile.print(binData[i].thigh_az); csvFile.write(',');
			csvFile.print(binData[i].thigh_roll); csvFile.write(',');
			csvFile.print(binData[i].thigh_pitch); csvFile.write(',');
			csvFile.print(binData[i].thigh_yaw); csvFile.write(',');
			// Shank
			csvFile.print(binData[i].shank_gx); csvFile.write(',');
			csvFile.print(binData[i].shank_gy); csvFile.write(',');
			csvFile.print(binData[i].shank_gz); csvFile.write(',');
			csvFile.print(binData[i].shank_ax); csvFile.write(',');
			csvFile.print(binData[i].shank_ay); csvFile.write(',');
			csvFile.print(binData[i].shank_az); csvFile.write(',');
			csvFile.print(binData[i].shank_roll); csvFile.write(',');
			csvFile.print(binData[i].shank_pitch); csvFile.write(',');
			csvFile.print(binData[i].shank_yaw); csvFile.write(',');
			// Foot
			csvFile.print(binData[i].foot_gx); csvFile.write(',');
			csvFile.print(binData[i].foot_gy); csvFile.write(',');
			csvFile.print(binData[i].foot_gz); csvFile.write(',');
			csvFile.print(binData[i].foot_ax); csvFile.write(',');
			csvFile.print(binData[i].foot_ay); csvFile.write(',');
			csvFile.print(binData[i].foot_az); csvFile.write(',');
			csvFile.print(binData[i].foot_roll); csvFile.write(',');
			csvFile.print(binData[i].foot_pitch); csvFile.write(',');
			csvFile.print(binData[i].foot_yaw);
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
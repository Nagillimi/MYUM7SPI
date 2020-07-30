/* Arduino Example for storing UM7 data on a SD card in a 
   low latency setting. This example uses Bill Greiman's SdFat-beta
   library, specifically the TeensySdioDemo example
   (https://github.com/greiman/SdFat-beta)

   The sensors required for this project are:
   - 3 UM7s @ 250Hz
      {gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,roll,pitch,yaw}
   - 2 FSRs @ 500Hz
      {analog data}

*/

// Include the SdFat Library
#include "SdFat.h"
#include "Logger.h"
#include "MYUM7SPI.h"

// Use built-in SD for SPI modes on Teensy 3.5/3.6.
// Teensy 4.0 use first SPI port.
// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN

// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
// Format the SD card in exFat before 2 or 3.
#define SD_FAT_TYPE 3

// 32 KiB buffer.
const size_t BUF_DIM = 32768;

// 8 MiB file.
const uint32_t FILE_SIZE = 256UL * BUF_DIM;

#if SD_FAT_TYPE == 0
SdFat sd;
File file;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 file;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile file;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile file;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

uint8_t buf[BUF_DIM];

// buffer as uint32_t
uint32_t* buf32 = (uint32_t*)buf;

//------------------------------------------------------------------------------
void logRecord(data_t* data, uint16_t overrun) {
	if (overrun) {
		// Add one since this record has no adc data. Could add overrun field.
		overrun++;
		data->adc[0] = 0X8000 | overrun;
	}
	else {
		for (size_t i = 0; i < ADC_COUNT; i++) {
			data->adc[i] = analogRead(i);
		}
	}
}
//------------------------------------------------------------------------------
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
bool ready = false;
//------------------------------------------------------------------------------
bool sdBusy() {
	return ready ? sd.card()->isBusy() : false;
}
//------------------------------------------------------------------------------
// Replace "weak" system yield() function.
void yield() {
	// Only count cardBusy time.
	if (!sdBusy()) {
		return;
	}
	uint32_t m = micros();
	yieldCalls++;
	while (sdBusy()) {
		// Do something here.
	}
	m = micros() - m;
	if (m > yieldMaxUsec) {
		yieldMaxUsec = m;
	}
	yieldMicros += m;
}
//------------------------------------------------------------------------------
void logData() {
	/*// Opens binary file in RDWR or CREATE mode
	if (!file.open("TeensyDemo.bin", O_RDWR | O_CREAT)) {
		errorHalt("open failed");
	}
	// Write to file in 512 Byte chunks
	for (size_t nb = 512; nb <= BUF_DIM; nb *= 2) {
		uint32_t nRdWr = FILE_SIZE / nb;
		if (!file.truncate(0)) {
			errorHalt("truncate failed");
		}

		Serial.print(nb);
		Serial.print(',');
		uint32_t t = micros();
		for (uint32_t n = 0; n < nRdWr; n++) {
			// Store data to buffer here

			// Write buffer to file
			if (nb != file.write(buf, nb)) {
				errorHalt("write failed");
			}
		}
		Serial.println(1000.0*FILE_SIZE / t);
	}
	file.close();
	Serial.println("Done");*/

	int32_t delta;  // Jitter in log time.
	int32_t maxDelta = 0;
	uint32_t maxLogMicros = 0;
	uint32_t maxWriteMicros = 0;
	size_t maxFifoUse = 0;
	size_t fifoCount = 0;
	size_t fifoHead = 0;
	size_t fifoTail = 0;
	uint16_t overrun = 0;
	uint16_t maxOverrun = 0;
	uint32_t totalOverrun = 0;
	uint32_t fifoBuf[128 * FIFO_SIZE_SECTORS];
	data_t* fifoData = (data_t*)fifoBuf;

	// Write dummy sector to start multi-block write.
	dbgAssert(sizeof(fifoBuf) >= 512);
	memset(fifoBuf, 0, sizeof(fifoBuf));
	if (binFile.write(fifoBuf, 512) != 512) {
		error("write first sector failed");
	}
	serialClearInput();
	Serial.println(F("Type any character to stop"));

	// Wait until SD is not busy.
	while (sd.card()->isBusy()) {}

	// Start time for log file.
	uint32_t m = millis();

	// Time to log next record.
	uint32_t logTime = micros();
	while (true) {
		// Time for next data record.
		logTime += LOG_INTERVAL_USEC;

		// Wait until time to log data.
		delta = micros() - logTime;
		if (delta > 0) {
			Serial.print(F("delta: "));
			Serial.println(delta);
			error("Rate too fast");
		}
		while (delta < 0) {
			delta = micros() - logTime;
		}

		if (fifoCount < FIFO_DIM) {
			uint32_t m = micros();
			logRecord(fifoData + fifoHead, overrun);
			m = micros() - m;
			if (m > maxLogMicros) {
				maxLogMicros = m;
			}
			fifoHead = fifoHead < (FIFO_DIM - 1) ? fifoHead + 1 : 0;
			fifoCount++;
			if (overrun) {
				if (overrun > maxOverrun) {
					maxOverrun = overrun;
				}
				overrun = 0;
			}
		}
		else {
			totalOverrun++;
			overrun++;
			if (overrun > 0XFFF) {
				error("too many overruns");
			}
			if (ERROR_LED_PIN >= 0) {
				digitalWrite(ERROR_LED_PIN, HIGH);
			}
		}
		// Save max jitter.
		if (delta > maxDelta) {
			maxDelta = delta;
		}
		// Write data if SD is not busy.
		if (!sd.card()->isBusy()) {
			size_t nw = fifoHead > fifoTail ? fifoCount : FIFO_DIM - fifoTail;
			// Limit write time by not writing more than 512 bytes.
			const size_t MAX_WRITE = 512 / sizeof(data_t);
			if (nw > MAX_WRITE) nw = MAX_WRITE;
			size_t nb = nw * sizeof(data_t);
			uint32_t usec = micros();
			if (nb != binFile.write(fifoData + fifoTail, nb)) {
				error("write binFile failed");
			}
			usec = micros() - usec;
			if (usec > maxWriteMicros) {
				maxWriteMicros = usec;
			}
			fifoTail = (fifoTail + nw) < FIFO_DIM ? fifoTail + nw : 0;
			if (fifoCount > maxFifoUse) {
				maxFifoUse = fifoCount;
			}
			fifoCount -= nw;
			if (Serial.available()) {
				break;
			}
		}
	}
	Serial.print(F("\nLog time: "));
	Serial.print(0.001*(millis() - m));
	Serial.println(F(" Seconds"));
	binFile.truncate();
	binFile.sync();
	Serial.print(("File size: "));
	// Warning cast used for print since fileSize is uint64_t.
	Serial.print((uint32_t)binFile.fileSize());
	Serial.println(F(" bytes"));
	Serial.print(F("totalOverrun: "));
	Serial.println(totalOverrun);
	Serial.print(F("FIFO_DIM: "));
	Serial.println(FIFO_DIM);
	Serial.print(F("maxFifoUse: "));
	Serial.println(maxFifoUse);
	Serial.print(F("maxLogMicros: "));
	Serial.println(maxLogMicros);
	Serial.print(F("maxWriteMicros: "));
	Serial.println(maxWriteMicros);
	Serial.print(F("Log interval: "));
	Serial.print(LOG_INTERVAL_USEC);
	Serial.print(F(" micros\nmaxDelta: "));
	Serial.print(maxDelta);
	Serial.println(F(" micros"));
}
//------------------------------------------------------------------------------
void setup() {
	Serial.begin(9600);
	while (!Serial); // Serial acts as an on switch
}
//------------------------------------------------------------------------------
void loop() {
	static bool warn = true;
	if (warn) {
		warn = false;
		Serial.println(
			"SD cards must be power cycled to leave, so\n"
			"\ncycle power on the card if an error occurs.");
	}
	do {
		delay(10);
	} while (Serial.available() && Serial.read());

	Serial.println(
		"\nType '1' to log in FIFO SDIO mode");
	while (!Serial.available()) {
	}
	char c = Serial.read();

	if (c == '1') {
		if (!sd.begin(SdioConfig(FIFO_SDIO))) {
			errorHalt("begin failed");
		}
		Serial.println("\nFIFO SDIO mode.");
		ready = true;
		logData();
		ready = false;
	}
	else {
		Serial.println("Invalid input");
		return;
	}
}

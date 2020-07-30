#include "SdFat.h"
#include "MYUM7SPI.h"
#include "Parameters.h"

byte fsr_heel_pin = A0, fsr_heel_pin = A1;
uint16_t fsr_heel, fsr_toe;

char file_name[] = "SdioLogger.txt";

void setup() {
	Serial.begin(9600);
	while (!Serial); // Serial acts as an on switch

	pinMode(A0, INPUT);
	pinMode(A1, INPUT);

	// Start SD card
	if (!sd.begin(SdioConfig(FIFO_SDIO))) {
		errorHalt("begin failed");
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

		// Open file
		if (!file.open(file_name, O_WRITE | O_CREAT)) {
			errorHalt("file open failed");
		}

		// Format file
		file.truncate();

		do {
			while (!Serial.available()) {
			}
			// Entering logging mode
			d = Serial.read();
			log_data();
		} while (d != '2')
	} else {
		Serial.println("Invalid input");
		return;
	}
}

log_data() {

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
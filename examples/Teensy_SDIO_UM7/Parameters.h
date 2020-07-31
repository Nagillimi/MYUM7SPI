#ifndef Parameters_h
#define Parameters_h

#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN

// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 3

// Huge 32 KiB buffer.
const size_t BUF_DIM = 32768;

// 4 GiB file. Maybe make a little bigger?
const uint32_t FILE_SIZE = 131072UL * BUF_DIM;

// Init the sd and binFile parameters
#if SD_FAT_TYPE == 0
SdFat sd;
File binFile;
File csvFile;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 binFile;
File32 csvFile;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile binFile;
ExFile csvFile;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile binFile;
FsFile csvFile;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

// BYTE BUFFER
uint8_t buf[BUF_DIM];

// Pointer to the buffer as a uint32_t,
// size is 1024 B for a 32 KiB uint8_t buffer
uint32_t* buf32 = (uint32_t*)buf;

// Latency
const uint32_t LOG_INTERVAL_USEC = 2000;

// Size of data required to be sent per one transfer. 
// Val's 29 datasets come in to 58 4Byte usigned integers.
// (because of commas parsing the data)
const uint32_t DATA_BYTE_WRITE_SIZE = 29*2;

// Set USE_RTC nonzero for file timestamps.
// RAM use will be marginal on Uno with RTClib.
#define USE_RTC 0
#if USE_RTC
#include "RTClib.h"
#endif  // USE_RTC

// FIFO SIZE - 512 byte sectors.  Modify for the RAM on your board.
#ifdef __AVR_ATmega328P__
// Use 512 bytes for 328 boards.
#define FIFO_SIZE_SECTORS 1
#elif defined(__AVR__)
// Use 2 KiB for other AVR boards.
#define FIFO_SIZE_SECTORS 4
#else  // __AVR_ATmega328P__
// Use 8 KiB for non-AVR boards.
#define FIFO_SIZE_SECTORS 16
#endif  // __AVR_ATmega328P__

// Max number of records to buffer while SD is busy.
const size_t FIFO_DIM = 512 * FIFO_SIZE_SECTORS / DATA_BYTE_WRITE_SIZE;

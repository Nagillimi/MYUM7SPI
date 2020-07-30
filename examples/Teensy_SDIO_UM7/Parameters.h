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

// BYTE BUFFER
uint8_t buf[BUF_DIM];

// buffer as uint32_t
uint32_t* buf32 = (uint32_t*)buf;

// Set USE_RTC nonzero for file timestamps.
// RAM use will be marginal on Uno with RTClib.
#define USE_RTC 0
#if USE_RTC
#include "RTClib.h"
#endif  // USE_RTC


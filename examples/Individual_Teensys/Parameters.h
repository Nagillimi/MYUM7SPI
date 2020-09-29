/*Bus schematic:
  | UM7 |-----(SPI0)---->| Teensy |-----(SPI1)---->| SD CARD |
  | FSR |----(Analog)--->|  3.5   |
  
 Note:
 - Should pad data_t to 2^i for best performance
*/
#ifndef Parameters_h
#define Parameters_h
#include "MYUM7SPI.h"
//---------------------------------APPARATUS FREQUENCIES---------------------------------
// Freq for SPI0
// Should be evenly divisible by 60,000,000 Hz and no more than 10,000,000 Hz
#define UM7_SPI_FREQ 1500000 // Frequency for UM7's

// Freq for SPI1
// Should be kept at 50MHz UOS
#define SD_SPI_FREQ_MHZ 50

// Interval between data records in microseconds.
// 100Hz = 10000usec
// 250Hz = 4000usec
// 500Hz = 2000usec
const uint16_t LOG_INTERVAL_USEC = 2000;
// Use to compare timestamps for missed packets
const uint16_t MAX_INTERVAL_USEC = 3000;
//---------------------------------SENSOR INITIALIZATION---------------------------------
#define UM7_CS_PIN 9
#define UM7_MOSI_PIN 11
#define UM7_MISO_PIN 12
#define UM7_SCK_PIN 13

// Led pin for done flag
#define LED_PIN 31;

// FSR analog pins
// Make sure they aren't any SPI bus pins
int fsr_heel_pin = A8, fsr_toe_pin = A9;

// Start Button pin
const int start_button_pin = 32;
//------------------------------------------------------------------------------
// Initialize the UM7 (cs_pin, rate)
MYUM7SPI imu1(UM7_CS_PIN, UM7_SPI_FREQ); // cs pin 1

// This is where you set the SPI1 bus. Enter your according pins for MOSI, MISO, and SCK. 
// The CS/SS pin is already defined through the argument in the default constructor.
// Call this function with a integer for your desired rate. Assuming all the same rate for 
// each dataset
void setup_imus(byte rate_) {
  // Init the analog sensors
  pinMode(fsr_heel_pin, INPUT);
  pinMode(fsr_toe_pin, INPUT);
  // Init the SPI bus used for UM7s at default rate
  SPI.begin();
  // Default SPI0 pins:
  SPI.setMOSI(UM7_MOSI_PIN);
  SPI.setMISO(UM7_MISO_PIN);
  SPI.setSCK(UM7_SCK_PIN);
  // Init UM7 1
  imu1.set_all_processed_rate(rate_);
  imu1.set_orientation_rate(rate_, rate_);
  imu1.calibrate_accelerometers();
  imu1.zero_gyros();
}
//-----------------------------------DATA PACKET-----------------------------------------
// Collection of data custom for application
// Note: delta is NOT part of data_t, it's computed during conversion based on "t"
struct data_t {
  // 98 Byte data transfer:
  uint32_t t;
  uint16_t fsr_heel;
  uint16_t fsr_toe;
  float gx_1;
  float gy_1;
  float gz_1;
  float ax_1;
  float ay_1;
  float az_1;
  int16_t roll_1;
  int16_t pitch_1;
  int16_t yaw_1;

  // Variable to fill in the transfer to 128 Bytes (30B difference).
  // 16b * 15 = 240b = 30 Bytes
//  uint16_t whitespace[15];
};
//-----------------------------------PARAMETERS-----------------------------------------
// You may modify the log file name up to 40 characters.
// Digits before the dot are file versions, don't edit them!
char binName[] = "DataLogParticipant00.bin";
//------------------------------------------------------------------------------
// This example was designed for exFAT but will support FAT16/FAT32.
// Note: Uno will not support SD_FAT_TYPE = 3.
// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 2

#if SD_FAT_TYPE == 0
typedef SdFat sd_t;
typedef File file_t;
#elif SD_FAT_TYPE == 1
typedef SdFat32 sd_t;
typedef File32 file_t;
#elif SD_FAT_TYPE == 2
typedef SdExFat sd_t;
typedef ExFile file_t;
#elif SD_FAT_TYPE == 3
typedef SdFs sd_t;
typedef FsFile file_t;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE
//------------------------------------------------------------------------------
// Initial time before logging starts, set once logging has begun
// And total log time of session, used to print to csv file once
// converted.
uint32_t t0, log_time;
// Init the time delta to track missed packets
uint32_t delta = 0;
//------------------------------------------------------------------------------
// SDCARD_SS_PIN is defined for the built-in SD on some boards.
// Teensy boards have pre-defined SS = BUILTIN_SDCARD = 254
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN
//------------------------------------------------------------------------------
// FIFO SIZE - 512 byte sectors.  Modify for your board.
#ifdef __AVR_ATmega328P__
// Use 512 bytes for 328 boards.
#define FIFO_SIZE_SECTORS 1
#elif defined(__AVR__)
// Use 2 KiB for other AVR boards.
#define FIFO_SIZE_SECTORS 4
#else  // __AVR_ATmega328P__
// Use 8 KiB for non-AVR boards, ex: Teensy 3.5/6
#define FIFO_SIZE_SECTORS 16
// Use 2 KiB for Teensy LC
//#define FIFO_SIZE_SECTORS 4
#endif  // __AVR_ATmega328P__
//------------------------------------------------------------------------------
// Preallocate 1GiB file.
const uint32_t PREALLOCATE_SIZE_MiB = 1024UL;
// Conversion to 64b variable to match the library param
const uint64_t PREALLOCATE_SIZE = (uint64_t)PREALLOCATE_SIZE_MiB << 20;
//------------------------------------------------------------------------------
// Select the appropriate SPI configuration. 
// ENABLE_DEDICATED_SPI default is true for Teensy boards, change this in
// SdFatConfig.h to zero if you want a (slower) shared SPI bus.
#if ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(SD_SPI_FREQ_MHZ))
#else  // ENABLE_DEDICATED_SPI
// Shared SPI bus, MAY need to alter the 50MHz depending on if it's already declared
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(SD_SPI_FREQ_MHZ))
#endif  // ENABLE_DEDICATED_SPI
//------------------------------------------------------------------------------
// Max length of file name including zero byte.
#define FILE_NAME_DIM 40

// Max number of records to buffer while SD is busy. Should alter factors to result
// in an integer! (Faster writes)
const size_t FIFO_DIM = 512 * FIFO_SIZE_SECTORS / sizeof(data_t);

// Create single sd type
sd_t sd;

// Create two filetypes
file_t binFile;
file_t csvFile;

// Boolean used to track whether or not you're just testing the sensors. Won't print
// the "Missed packet(s)" everytime when testing, otherwise printed in data logging.
bool test = false;
//------------------------------------------------------------------------------
#endif  // Parameters_h

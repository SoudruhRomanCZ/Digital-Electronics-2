// do some anotation, maybe doxygen
#include <stdint.h>
#include <twi.h>
#include <avr/eeprom.h>

// RTC
#define RTC_ADR  0x68
#define EEPROM_ADR 0x57
#define SECONDS_REG 0x00
#define MINUTES_REG 0x01
#define HOURS_REG 0x02
#define DAY_REG 0x03
#define DATE_REG 0x04
#define MONTH_REG 0x05
#define YEAR_REG 0x06
/*
struct Data {
    uint8_t hum_int;
    uint8_t hum_dec;
    uint8_t temp_int;
    uint8_t temp_dec; // 4 bytes
    uint8_t checksum; // 5 bytes

    uint8_t secs;
    uint8_t mins;
    uint8_t hours;
    uint8_t days;
    uint8_t date;
    uint8_t months;
    uint8_t years;  //7 bytes

    uint16_t mois_int; // 2 bytes
}; // 14 or 13 bytes 

struct Values_structure {
    uint8_t hum_int;
    uint8_t hum_dec;
    uint8_t temp_int;
    uint8_t temp_dec; // 4 bytes
    uint8_t checksum;
} dht12;

struct RTC_values_structure {
    uint8_t secs;
    uint8_t mins;
    uint8_t hours;
    uint8_t days;
    uint8_t date;
    uint8_t months;
    uint8_t years;  // 7 bytes
} rtc;
*/
uint16_t mois_int;
// Function to write time to DS3231 RTC module
void writeTimeToDS3231(uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t day, uint8_t date, uint8_t month, uint8_t year) {
    // Start I2C communication
    twi_start();

    // Check ACK from RTC 
    if (twi_write((RTC_ADR << 1) | TWI_WRITE) == 0) {
        // Set internal memory location to seconds register
        twi_write(SECONDS_REG);
        twi_write(seconds); // Write seconds to the register
        twi_stop();

        // Start I2C communication again
        twi_start();

        // Set internal memory location to minutes register
        twi_write((RTC_ADR << 1) | TWI_WRITE);
        twi_write(MINUTES_REG);
        twi_write(minutes); // Write minutes to the register
        twi_stop();

        // Start I2C communication again
        twi_start();

        // Set internal memory location to hours register
        twi_write((RTC_ADR << 1) | TWI_WRITE);
        twi_write(HOURS_REG);
        //twi_write(0b0); set 24h format 0 0/1 0 0 0000
        twi_write(hours); // Write hours to the register
        twi_stop();

        // Start I2C communication again
        twi_start();

        // Set internal memory location to hours register
        twi_write((RTC_ADR << 1) | TWI_WRITE);
        twi_write(DAY_REG);
        twi_write(day); // Write day of the week to the register
        twi_stop();

        // Start I2C communication again
        twi_start();

        // Set internal memory location to hours register
        twi_write((RTC_ADR << 1) | TWI_WRITE);
        twi_write(DATE_REG);
        twi_write(date); // Write day in the month to the register
        twi_stop();

        // Start I2C communication again
        twi_start();

        // Set internal memory location to hours register
        twi_write((RTC_ADR << 1) | TWI_WRITE);
        twi_write(MONTH_REG);
        twi_write(month); // Write month to the register
        twi_stop();

        // Start I2C communication again
        twi_start();

        // Set internal memory location to hours register
        twi_write((RTC_ADR << 1) | TWI_WRITE);
        twi_write(YEAR_REG);
        twi_write(year); // Write year to the register
        twi_stop();
    }

    // Stop I2C communication
    twi_stop();
}


/*
// Function to save data to RTC EEPROM memory
void saveDataToRtcEeprom() { //have to save 13 bytes 104 bits of data
    // Read previous data from Arduino's EEPROM memory
    struct RTC_values_structure prevData;
    eeprom_read_block(&prevData, EEPROM_ADR, sizeof(struct RTC_values_structure));

    // Compare if the data has changed
    if (memcmp(&prevData, &rtc, sizeof(struct RTC_values_structure)) != 0) {
        // Jump to the EEPROM memory address and save new data
        eeprom_write_block(&rtc, EEPROM_ADR, sizeof(struct RTC_values_structure));
    }
}
*/
/*
void writeDataToEEPROM(struct Data* data) {
    eeprom_write_block((const void*)data, (void*)0, sizeof(struct Data));
}
void readDataFromEEPROM(struct Data* data) {
    eeprom_read_block((void*)data, (const void*)0, sizeof(struct Data));
}
*/
/*
void writeDataToEEPROM() {
    struct Values_structure dht12Data;
    struct RTC_values_structure rtcData;

    // Copy data from dht12 and rtc structures to separate variables
    dht12Data.hum_int = dht12.hum_int;
    dht12Data.hum_dec = dht12.hum_dec;
    dht12Data.temp_int = dht12.temp_int;
    dht12Data.temp_dec = dht12.temp_dec;
    dht12Data.checksum = dht12.checksum;

    rtcData.secs = rtc.secs;
    rtcData.mins = rtc.mins;
    rtcData.hours = rtc.hours;
    rtcData.days = rtc.days;
    rtcData.date = rtc.date;
    rtcData.months = rtc.months;
    rtcData.years = rtc.years;

    // Write the data structures to EEPROM
    eeprom_write_block(&dht12Data, (void*)0, sizeof(struct Values_structure));
    eeprom_write_block(&rtcData, (void*)sizeof(struct Values_structure), sizeof(struct RTC_values_structure));

    // Write the mois_int variable to EEPROM
    eeprom_write_word((uint16_t*) (sizeof(struct Values_structure) + sizeof(struct RTC_values_structure)), mois_int);
}
void readDataFromEEPROM() {
    struct Values_structure dht12Data;
    struct RTC_values_structure rtcData;

    // Read the data structures from EEPROM
    eeprom_read_block(&dht12Data, (const void*) 0, sizeof(struct Values_structure));
    eeprom_read_block(&rtcData, (const void*) sizeof(struct Values_structure), sizeof(struct RTC_values_structure));

    // Copy the data from the separate variables to dht12 and rtc structures
    dht12.hum_int = dht12Data.hum_int;
    dht12.hum_dec = dht12Data.hum_dec;
    dht12.temp_int = dht12Data.temp_int;
    dht12.temp_dec = dht12Data.temp_dec;
    dht12.checksum = dht12Data.checksum;

    rtc.secs = rtcData.secs;
    rtc.mins = rtcData.mins;
    rtc.hours = rtcData.hours;
    rtc.days = rtcData.days;
    rtc.date = rtcData.date;
    rtc.months = rtcData.months;
    rtc.years = rtcData.years;

    // Read the mois_int variable from EEPROM
    mois_int = eeprom_read_word((const uint16_t*)(sizeof(struct Values_structure) + sizeof(struct RTC_values_structure)));
}
*/
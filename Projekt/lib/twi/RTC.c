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


/* Global variables --------------------------------------------------*/
// Declaration of "dht12" variable with structure "DHT_values_structure"
struct Values_structure {
   uint8_t hum_int;
   uint8_t hum_dec;
   uint8_t temp_int;
   uint8_t temp_dec;
   uint8_t checksum;
} dht12;
// Declaration of "rtc" variable with structure "RTC_values_structure"
struct RTC_values_structure {
    uint8_t secs;
    uint8_t mins;
    uint8_t hours;
    uint8_t days;
    uint8_t date;
    uint8_t months;
    uint8_t years;
} rtc; 
uint16_t mois_int;

// Function to write time to <link>DS3231</link>
void writeTimeToDS3231(uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t day, uint8_t date, uint8_t month, uint8_t year) {
    // Start I2C communication
    twi_start();

    // Check ACK from <link>RTC</link>
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



// Function to save data to RTC EEPROM memory
void saveDataToRtcEeprom() { //have to save 13 bytes 104 bits of data
    // Read previous data from Arduino's EEPROM memory
    struct RTC_values_structure prevData;
    struct RTC_values_structure newData;
    eeprom_read_block(&prevData, EEPROM_ADR, sizeof(struct RTC_values_structure));

    // Compare if the data has changed
    if (memcmp(&prevData, &newData, sizeof(struct RTC_values_structure)) != 0) {
        // Jump to the EEPROM memory address and save new data
        eeprom_write_block(&rtc, EEPROM_ADR, sizeof(struct RTC_values_structure));
    }
}
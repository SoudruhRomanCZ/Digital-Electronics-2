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

// Define the data structure for DHT12 and RTC
struct Values_structure {
    uint8_t hum_int;
    uint8_t hum_dec;
    uint8_t temp_int;
    uint8_t temp_dec;
    uint8_t checksum;
} dht12;

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
uint64_t address;

// Function to write time to DS3231 RTC module
void writeTimeToDS3231(uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t day, uint8_t date, uint8_t month, uint16_t year) {
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

void writeDataToEEPROM() {
    // Calculate the total size of the data structure
    uint8_t totalSize = sizeof(struct Values_structure) + sizeof(struct RTC_values_structure) + sizeof(uint16_t);

    // Start EEPROM write operation
    eeprom_write_block(&dht12, (void*)0, sizeof(dht12)); // Write DHT12 data
    eeprom_write_block(&rtc, (void*)sizeof(dht12), sizeof(rtc)); // Write RTC data
    eeprom_write_word((uint16_t*)(sizeof(dht12) + sizeof(rtc)), mois_int); // Write mois_int

    // Increment the EEPROM address for the next write operation
    // You might need to store and manage the EEPROM address elsewhere in your code
    address += totalSize;
}

void readDataFromEEPROM() {
    // Read DHT12 data from EEPROM
    eeprom_read_block(&dht12, (void*)0, sizeof(dht12));

    // Read RTC data from EEPROM
    eeprom_read_block(&rtc, (void*)sizeof(dht12), sizeof(rtc));

    // Read mois_int from EEPROM
    mois_int = eeprom_read_word((uint16_t*)(sizeof(dht12) + sizeof(rtc)));

    // Now you have the data stored in the respective variables
    // You can use these variables to print the data in the terminal or render a graph on an OLED display
}
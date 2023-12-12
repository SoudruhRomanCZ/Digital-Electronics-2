// do some anotation, maybe doxygen
#include <stdint.h>
#include <twi.h>
#include <avr/eeprom.h>
#include <uart.h>

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

<<<<<<< HEAD:Projekt/lib/twi/RTC.c
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

=======
>>>>>>> 63f49e027e5667c24dc5e96762ee9be6ef1096a6:lib/twi/RTC.c
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
        twi_write(seconds);// Write seconds to the register
        twi_stop();

        // Start I2C communication again
        twi_start();

        // Set internal memory location to minutes register
        twi_write((RTC_ADR << 1) | TWI_WRITE);
        twi_write(MINUTES_REG);
        twi_write(minutes);// Write seconds to the register
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
    //twi_stop();
}

uint16_t address =0;
// Function to save data to RTC EEPROM memory
uint16_t saveDataToRtcEeprom(uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t temp_int,uint8_t temp_dec, uint8_t moist) { //have to save 13 bytes 104 bits of data
    eeprom_write_byte(address,hours);
    address++;
    eeprom_write_byte(address,minutes);
    address++;
    eeprom_write_byte(address,seconds);
    address++;
    eeprom_write_byte(address,temp_int);
    address++;
    eeprom_write_byte(address,temp_dec);
    address++;
    eeprom_write_byte(address,moist);
    address++;
    if(address>=32768){
    address=0;
    }
    return address;
}

void writeDataToEEPROM() {
    // Calculate the total size of the data structure
    uint8_t totalSize = sizeof(struct Values_structure) + sizeof(struct RTC_values_structure) + sizeof(uint16_t);

<<<<<<< HEAD:Projekt/lib/twi/RTC.c
    // Start EEPROM write operation
    eeprom_write_block(&dht12, (void*)0, sizeof(dht12)); // Write DHT12 data
    eeprom_write_block(&rtc, (void*)sizeof(dht12), sizeof(rtc)); // Write RTC data
    eeprom_write_word((uint16_t*)(sizeof(dht12) + sizeof(rtc)), mois_int); // Write mois_int

    // Increment the EEPROM address for the next write operation
    // You might need to store and manage the EEPROM address elsewhere in your code
    address += totalSize;
=======
/*
uint8_t read=0;
void readDatafromRtcEeprom() { 
    char string[4];
    for(uint16_t i;i<=address;i++){
        itoa(i, string, 10);
        uart_puts(string);
        read=eeprom_read_byte(i);   
        writeDataToUART(read, "data from EEPROM ", 1, 1);
    }
    itoa(address-1, string, 10);
    uart_puts(string);
    uart_puts(" adresa");
    writeDataToUART(eeprom_read_byte(address-1), "percenta from EEPROM ", 0, 1);
    uart_puts("\r\n");
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
>>>>>>> 63f49e027e5667c24dc5e96762ee9be6ef1096a6:lib/twi/RTC.c
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
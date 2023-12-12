#include <stdint.h>
#include <twi.h>
#include <avr/eeprom.h>
#include <uart.h>

// TWI adresses and REG address
#define RTC_ADR  0x68
#define EEPROM_ADR 0x57
#define SECONDS_REG 0x00

uint16_t mois_int;
// Function to write time to DS3231 RTC module
void writeTimeToDS3231(uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t day, uint8_t date, uint8_t month, uint16_t year) {
    // Start I2C communication
    twi_start();
    // Check ACK from RTC 
    if (twi_write((RTC_ADR << 1) | TWI_WRITE) == 0) {
        // Write each time unit to the RTC module
        for (uint8_t i = 0; i < 8; i++) {
            twi_write(SECONDS_REG + i);
            twi_write(i == 6 ? year >> 8 : i == 7 ? year & 0xFF : i == 0 ? seconds : i == 1 ? minutes : i == 2 ? hours : i == 3 ? day : i == 4 ? date : month);
        }
    }
    // Stop I2C communication
    twi_stop();
}
uint16_t address =0;
// Function to save data to RTC EEPROM memory
uint16_t saveDataToRtcEeprom(uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t temp_int,uint8_t temp_dec, uint8_t moist) { //have to save 13 bytes 104 bits of data
    eeprom_write_byte(address,hours);       address++;
    eeprom_write_byte(address,minutes);     address++;
    eeprom_write_byte(address,seconds);     address++;
    eeprom_write_byte(address,temp_int);    address++;
    eeprom_write_byte(address,temp_dec);    address++;
    eeprom_write_byte(address,moist);       address++;
    if(address>=32768){
    address=0;
    }
    return address;
}
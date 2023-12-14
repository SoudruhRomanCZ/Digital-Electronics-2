/* Defines -----------------------------------------------------------*/
#ifndef F_CPU
#define F_CPU 16000000  // CPU frequency in Hz required for UART_BAUD_SELECT
#endif

// Including needed libraries
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <avr/eeprom.h>     // AVR library for EEPROM read and write operations
#include <stdlib.h>         // C library. Needed for number conversions
#include <timer.h>
#include <oled.h>
#include <gpio.h>
#include <twi.h>
#include <uart.h>
#include <RTC.c>

/* Global variables --------------------------------------------------*/
// Declaration of "dht12" variable with structure "DHT_values_structure"
struct Values_structure {
   uint8_t hum_int;
   uint8_t hum_dec;
   uint8_t temp_int;
   uint8_t temp_dec;
} dht12;
// Declaration of "rtc" variable with structure "RTC_values_structure"
struct RTC_values_structure {
    uint8_t secs;
    uint8_t mins;
    uint8_t hours;
} rtc;


// Flag for printing new data from sensor
volatile uint8_t new_sensor_data = 0;

// Slave and internal addresses of temperature/humidity sensor DHT12
#define SENSOR_ADR 0x5c
#define SENSOR_TEMP_MEM 2
// RTC
#define RTC_ADR  0x68
#define EEPROM_ADR 0x57
#define SECONDS_REG 0x00
// TWI OLED adress
#define OLED_ADR 0x3c
// variable for voltage conversion of moisture humidity
uint16_t mois_int; 
// Normal values of moisture sensor
const uint16_t AirValue = 950;   
const uint16_t WaterValue = 650; 
// Variable for conversion of moisture sensor data to percentual value
const int onePercent = 3;
uint8_t percentualValue;
uint16_t calculus;
#define soil PC0
// defining pins of GPIO
#define RELAY PB0
#define LED_BLUE PB1
#define LED_GREEN PB2   
#define LED_RED PB3
#define BUTTON PB4

int main(void)
{   
    twi_init(); // Initialize TWI interface
    //checking if OLED display is connected to TWI interface
    if(twi_test_address(OLED_ADR) == 0){
    oled_init(OLED_DISP_ON);
    oled_clrscr();
    // Boot screen
    oled_charMode(DOUBLESIZE);
    oled_puts("SMART");
    oled_gotoxy(0, 2);
    oled_puts("PLANT");
    oled_gotoxy(0, 4);
    oled_puts("WATERING");
    oled_gotoxy(0, 6);
    oled_puts("SYSTEM");
    oled_display();
    oled_charMode(NORMALSIZE);
    }
    // Initialize USART to asynchronous, 8-N-1, 115200 Bd
    uart_init(UART_BAUD_SELECT(115200, F_CPU));
    sei();  // Needed for UART
    // Configure Analog-to-Digital Convertion unit
    // Select input channel ADC0 (voltage divider pin)
    ADMUX = ADMUX & ~(1<<MUX3 | 1<<MUX2 | 1<<MUX1 | 1<<MUX0);
    // Enable ADC module
    ADCSRA = ADCSRA | (1<<ADEN);
    // Enable conversion complete interrupt
    ADCSRA = ADCSRA | (1<<ADIE);
    // Set clock prescaler to 128
    ADCSRA = ADCSRA | (1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0);
    // Timer1
    TIM1_OVF_1SEC
    TIM1_OVF_ENABLE
    // GPIO pin mode configuration
    GPIO_mode_output(&DDRB, LED_GREEN);  
    GPIO_mode_output(&DDRB, LED_RED);  
    GPIO_mode_output(&DDRB, LED_BLUE);  
    GPIO_mode_output(&DDRB, RELAY);  
    GPIO_mode_input_pullup(&DDRB, BUTTON);
    // Load time into RTC DS3231 
    writeTimeToDS3231(0x00,0x00,0x23,4,30,11,2023);
    uint16_t currentAddress = 0 ;
    uint8_t button_was_pressed = 0;
    while (1) {
        if (new_sensor_data == 1) {
            oled_clrscr();
            // Conversion of the value from the plant's moisture sensor into percentages
            calculus = (mois_int - WaterValue);
            calculus = calculus/onePercent;
            percentualValue = 100-calculus;
            // Save current data to RTC's EEPROM
            currentAddress = saveDataToRtcEeprom(rtc.hours, rtc.mins, rtc.secs, dht12.temp_int,dht12.temp_dec,mois_int);
            uint16_t numoflogs = currentAddress/6; // devided by number of saved bytes per 1 saving
            // Determine if the watering system is running or not based on percentual moisture
            // error handling, if sensor is not connected
            if (percentualValue < 10) { 
            GPIO_write_high(&PORTB, LED_RED);
            GPIO_write_high(&PORTB, LED_GREEN);
            GPIO_write_high(&PORTB, LED_BLUE);
            GPIO_write_high(&PORTB, RELAY);
            }
            //low moisture, turning on the pump 
            else if (percentualValue<30){ 
                GPIO_write_high(&PORTB, LED_RED);
                GPIO_write_low(&PORTB, LED_GREEN);
                GPIO_write_low(&PORTB, LED_BLUE);
                GPIO_write_low(&PORTB, RELAY); // starting pump
            }
            //moisture in range, will not change pump status, so it can water from 30% to 60% or dry from 60% to 30%
            else if(percentualValue<60){ 
                GPIO_write_high(&PORTB, LED_BLUE);
                GPIO_write_low(&PORTB, LED_RED);
                GPIO_write_low(&PORTB, LED_GREEN);
            }
            //moisture is high, turning off the pump 
            else if(percentualValue>=60){
                GPIO_write_high(&PORTB, LED_GREEN);
                GPIO_write_low(&PORTB, LED_RED);
                GPIO_write_low(&PORTB, LED_BLUE);
                GPIO_write_high(&PORTB, RELAY); //turning off pump
            }
            // Set all obtained data to be displayed on OLED
            if(twi_test_address(OLED_ADR) == 0){
                // Display time from RTC
                // mask and diveded by 16 for making it the upper half of byte and only 2 bits used for tens of hours 10-60
                writeDataToOLED((rtc.hours& 0b00110000)/16,0,0);
                // mask for counting 0-16, 4 bits used for hours 0-9 
                writeDataToOLED(rtc.hours& 0b00001111,1,0); 
                oled_gotoxy(2, 0);
                oled_puts(":");
                writeDataToOLED((rtc.mins& 0b01110000)/16,3,0); 
                writeDataToOLED(rtc.mins& 0b00001111,4,0);
                oled_gotoxy(5, 0);
                oled_puts(":");
                writeDataToOLED((rtc.secs& 0b01110000)/16,6,0);
                writeDataToOLED(rtc.secs& 0b00001111,7,0);
                // Display temperature from temperature sensor
                if (twi_test_address(SENSOR_ADR) == 0){
                    writeDataToOLED(dht12.temp_int,0,2);
                    oled_gotoxy(2, 2);
                    oled_puts(".");
                    writeDataToOLED(dht12.temp_dec,3,2);
                    oled_gotoxy(4, 2);
                    oled_puts(" ");
                    oled_putc((char)247); // degree symbol
                    oled_puts("C");
                }
                // Display plant's moisture percentage
                writeDataToOLED(percentualValue,0,4);
                oled_gotoxy(3, 4);
                oled_puts("%");
                oled_gotoxy(0, 6);
                if (percentualValue<30){
                    oled_puts("Dry");
                }
                else if(percentualValue<60){
                    oled_puts("Wet");   
                }
                else if(percentualValue>=60){
                    oled_puts("Watered");
                }
            }                      
            // If the button was pressed then display the stored data in serial monitor (data EXPORT)
            if(button_was_pressed == 1){
                writeDataToUART(numoflogs, " total number of logs", 0,1);
                for (uint16_t i = 0; i < numoflogs; i++)
                {
                    writeDataToUART(i+1, ". log Time : ",0,0);
                    writeDataToUART((eeprom_read_byte(0+i*6) & 0b00110000)/16, "", 0, 0);
                    writeDataToUART(eeprom_read_byte(0+i*6) & 0b00001111, ":" , 0, 0);
                    writeDataToUART((eeprom_read_byte(1+i*6) & 0b01110000)/16, "" , 0, 0);
                    writeDataToUART(eeprom_read_byte(1+i*6) & 0b00001111, ":" , 0, 0);
                    writeDataToUART((eeprom_read_byte(2+i*6) & 0b01110000)/16, "" , 0, 0);
                    writeDataToUART(eeprom_read_byte(2+i*6) & 0b00001111, " Temperature : " , 0, 0);
                    writeDataToUART(eeprom_read_byte(3+i*6), "." , 0, 0);  //temperature
                    writeDataToUART(eeprom_read_byte(4+i*6), " °C Moisture : " , 0, 0);
                    writeDataToUART(eeprom_read_byte(5+i*6), " ", 0, 1);
                }
                uart_puts("That's all the data !");
                uart_puts("\r\n");
                button_was_pressed = 0;
            }
            // Display all set data on OLED
            oled_display();
            // Do not print it again and wait for the new data
            new_sensor_data = 0;
        }

        if(!(PINB & (1 << BUTTON))) {
            button_was_pressed = 1;       
        }
    }
    return 0;
} // End of main function

/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
* Function: Timer/Counter1 overflow interrupt
* Purpose:  Read temperature and humidity from DHT12, SLA = 0x5c.
**********************************************************************/
ISR(TIMER1_OVF_vect)
{   // Activating of reading Voltage value of pin PC0- arduinos A0
    ADCSRA = ADCSRA | (1<<ADSC);
    // Test ACK from sensor
    twi_start();
    // Temperature and humidity sensor
    if (twi_write((SENSOR_ADR<<1) | TWI_WRITE) == 0) {
        // Set internal memory location
        twi_write(SENSOR_TEMP_MEM);
        twi_stop();
        // Read data from internal memory
        twi_start();
        twi_write((SENSOR_ADR<<1) | TWI_READ);
        dht12.temp_int = twi_read(TWI_ACK);
        dht12.temp_dec = twi_read(TWI_NACK);
    }
    twi_stop();
    // Read Time from RTC DS3231; SLA = 0x68
    // FYI: MPU-6050; SLA = 0x68
        // Test ACK from RTC
        twi_start();
        if (twi_write((RTC_ADR<<1) | TWI_WRITE) == 0) {
            // Set internal memory location
            twi_write(SECONDS_REG);
            twi_stop();
            // Read data from internal memory
            twi_start();
            twi_write((RTC_ADR<<1) | TWI_READ);
            rtc.secs = twi_read(TWI_ACK);
            rtc.mins = twi_read(TWI_ACK);
            rtc.hours = twi_read(TWI_NACK);
            new_sensor_data = 1;
        }
        twi_stop();
}
ISR(ADC_vect)
{
    // Note that, register pair ADCH and ADCL can be read as a 16-bit value ADC
    mois_int = ADC;
}
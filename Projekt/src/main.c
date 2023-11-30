/* Defines -----------------------------------------------------------*/
#ifndef F_CPU
# define F_CPU 16000000  // CPU frequency in Hz required for UART_BAUD_SELECT
#endif

// Including of needed libraries
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <avr/eeprom.h>
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
   uint8_t temp_dec; // 32 bit == 4 byte
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
    uint8_t years;  //56 bit == 7 byte
} rtc;

uint16_t mois_int; //total of 112b bits to store every reading at 32kb storage it is 286 readings == 1 reading a second 4 min 46 sec of saved data
// Normal values of moisture sensor
const uint16_t AirValue = 950;   //you need to replace this value with Value_1
const uint16_t WaterValue = 650;  //you need to replace this value with Value_2 
// Variable for conversion of moisture sensor data to percentual value
const int onePercent = 3;
int percentualValue;

// Flag for printing new data from sensor
volatile uint8_t new_sensor_data = 0;

// Slave and internal addresses of temperature/humidity sensor DHT12
#define SENSOR_ADR 0x5c
#define SENSOR_HUM_MEM 0
#define SENSOR_TEMP_MEM 2
#define SENSOR_CHECKSUM 4 
// RTC
#define RTC_ADR  0x68
#define EEPROM_ADR 0x57
#define SECONDS_REG 0x00
//OLED
#define OLED_ADR 0x3c

// defining needed pins

// #define soil PC0
#define BUTTON PD5
#define PUMP PD6
#define LED_RED PD2
#define LED_GREEN PD3
#define LED_BLUE PD4

int main(void)
{ 
    oled_init(OLED_DISP_ON);
    oled_clrscr();
    //boot screen
    oled_charMode(DOUBLESIZE);  //would rather write it on 1 line on the top of the screen
    oled_puts("SMART");
    oled_gotoxy(0, 2);
    oled_puts("PLANT");
    oled_gotoxy(0, 4);
    oled_puts("WATERING");
    oled_gotoxy(0, 6);
    oled_puts("SYSTEM");
    oled_display();
    oled_charMode(NORMALSIZE);

    twi_init();
    // Initialize USART to asynchronous, 8-N-1, 115200 Bd
    uart_init(UART_BAUD_SELECT(115200, F_CPU));
    sei();  // Needed for UART
    // Configure Analog-to-Digital Convertion unit
    // Select ADC voltage reference to "AVcc with external capacitor at AREF pin"
    ADMUX = ADMUX | (1<<REFS0);
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
    //ver 3
    // Examples of various function calls
    GPIO_mode_output(&DDRB, LED_RED);
    GPIO_mode_output(&DDRB, LED_GREEN); // Set output mode in DDRB reg
    GPIO_mode_output(&DDRB, LED_BLUE);
    GPIO_mode_input_pullup(&DDRB, BUTTON);
    //function to load time to DS3231 
    writeTimeToDS3231(0,0,9,4,30,11,2023); //loading this time to DS3231 after startup so it is not starting from 0 0 0

    while (1) {
        if (new_sensor_data == 1) {
            oled_clrscr(); 
        
            if (twi_test_address(RTC_ADR) == 0){
                writeDataToUART(rtc.hours & 0b111111, ":", 0, 0); // which position i want to send
                writeDataToUART(rtc.mins, ":" , 0, 0);
                writeDataToUART(rtc.secs, " : " , 0, 0);
                if(twi_test_address(OLED_ADR) == 0){ // edit the x y position to make it nice
                    writeDataToOLED(rtc.hours & 0b111111 ,0,0);
                    oled_gotoxy(2, 0);
                    oled_puts(":");
                    writeDataToOLED(rtc.mins,3,0);
                    oled_gotoxy(5, 0);
                    oled_puts(":");
                    writeDataToOLED(rtc.secs,6,0);
                }                
            }
            
            if (twi_test_address(SENSOR_ADR) == 0){
                //writing an integer value of temperature

                writeDataToUART(dht12.temp_int, "." , 0, 0);
                writeDataToUART(dht12.temp_dec, " °C " , 0, 0);
                if(twi_test_address(OLED_ADR) == 0){
                writeDataToOLED(dht12.temp_int,0,2);
                oled_gotoxy(2, 2);
                oled_puts(".");
                writeDataToOLED(dht12.temp_dec,3,2);
                oled_gotoxy(4, 2);
                oled_puts(" °C ");
                }
            }

            if(mois_int != 0){
                //percentualValue = 100-((mois_int - WaterValue)/onePercent);
                percentualValue = (mois_int - WaterValue);
                percentualValue = percentualValue/onePercent;
                percentualValue = 100-percentualValue;

                //add RGB LED for each state?
                if (mois_int>850){
                    //uart_puts('\x1b[1;31m'); // Set style to bold, red foreground
                    writeDataToUART(percentualValue, " % : Plant is thirsty, turning on the pump\r\n" , 0, 1);
                    // write code for Relay enabled
                    GPIO_write_low(&PORTB, LED_RED);
                }
                else if(mois_int>750){
                    //uart_puts('\x1b[4;32m'); // 4: underline style; 32: green foreground
                    writeDataToUART(percentualValue, " % : Plant is watered enough, pump is off\r\n" , 0, 1);  
                    GPIO_write_low(&PORTB, LED_GREEN);
                }
                else if(mois_int>700){
                    writeDataToUART(percentualValue, " % : Plant is watered enough, turning off the pump\r\n" , 0, 1);
                    // write code for Relay disabled
                    GPIO_write_low(&PORTB, LED_BLUE);
                }
                else {
                    writeDataToUART(mois_int, " % : Value out of range, there is problem with moisture sensor\r\n" , 0, 1);
                }
                //uart_puts('\x1b[0m'); // 0: reset all attributes
                // writeDataToUART(mois_int, " " , false, true);
                if(twi_test_address(OLED_ADR) == 0){
                    writeDataToOLED(percentualValue,0,4);
                    writeDataToOLED(mois_int,8,4);
                    oled_gotoxy(3, 4);
                    oled_puts("%");
                    oled_gotoxy(0, 6);
                    if (mois_int>850){
                        oled_puts("DRY");
                        // write code for Relay enabled
                    }
                    else if(mois_int>750){
                        oled_puts("Wet");   
                    }
                    else if(mois_int>700){
                        oled_puts("Watered");
                        // write code for Relay disabled 
                    }
                    else {
                        oled_puts("[ERROR] Value out");
                    }
                }
            }
        oled_display();

        // saving data to RTC EEPROM memory
        // if read previus data from Arduinos EEPROM memory and compare if changed then jump in EEPROM memory and save new data, bcose there is time stamp on the data 
        //saveDataToRtcEeprom();
        writedatatoEEPROM(rtc);
        writedatatoEEPROM(dht12);
        writedatatoEEPROM(mois_int);
        // Do not print it again and wait for the new data
        new_sensor_data = 0;
        }
        if (GPIO_read(&DDRB,BUTTON)){ //button menu to load old data? like show graph from saved data
            //ver 3
            // GPIO_write_low(&PORTB, LED_GREEN); // Set output low in PORTB reg
            // GPIO_write_low(&PORTB, LED_RED);
        }
    }return 0;
} //end of main loop

/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
* Function: Timer/Counter1 overflow interrupt
* Purpose:  Read temperature and humidity from DHT12, SLA = 0x5c.
**********************************************************************/
ISR(TIMER1_OVF_vect)
{       // add some conditions for I2C sensors, to work even if 1 is not connected, if hum sensor is not connected read temperature from RTC
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
        twi_stop();
        // reading humidity of air
        twi_start();
        // Set internal memory location
        twi_write(SENSOR_HUM_MEM);
        twi_stop();
        // Read data from internal memory
        twi_start();
        twi_write((SENSOR_ADR<<1) | TWI_READ);
        dht12.hum_int = twi_read(TWI_ACK);
        dht12.hum_dec = twi_read(TWI_NACK);
        new_sensor_data = 1;
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
        }
        twi_stop();  
}

ISR(ADC_vect)
{    // Read converted value
    // Note that, register pair ADCH and ADCL can be read as a 16-bit value ADC
    mois_int = ADC;
}
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
const int AirValue = 620;   //you need to replace this value with Value_1
const int WaterValue = 310;  //you need to replace this value with Value_2
//const float onePercent = 2.5;

// Variable for conversion of moisture sensor data to percentual value
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
#define MINUTES_REG 0x01
#define HOURS_REG 0x02
#define DAY_REG 0x03
#define DATE_REG 0x04
#define MONTH_REG 0x05
#define YEAR_REG 0x06
//OLED
#define OLED_ADR 0x3c

// defining needed pins (do i really need this?)

#define soil PC0

int main(void)
{
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

    //char string[3];  // For converting numbers by itoa() 2 or 3 or more?

    twi_init();

    // Initialize USART to asynchronous, 8-N-1, 115200 Bd
    uart_init(UART_BAUD_SELECT(115200, F_CPU));
    sei();  // Needed for UART
    /*
    // Test if sensor is ready
    if (twi_test_address(SENSOR_ADR) == 0){
        oled_gotoxy(0, 3);
        oled_puts("TEMPERATURE sensor detected");
        uart_puts("TEMPERATURE sensor detected\r\n"); //
        }
    else {
        oled_gotoxy(0, 4);
        oled_puts("[ERROR] TEMPERATURE device not detected");
        uart_puts("[ERROR] TEMPERATURE device not detected\r\n");
        while (1);
    }
    */
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

    //function to load time to DS3231 
    writeTimeToDS3231(0,0,9,4,30,11,2023); //loading this time to DS3231 after startup so it is not starting from 0 0 0

    float onePercent =(AirValue - WaterValue)/100; 

    while (1) {
        if (new_sensor_data == 1) {
            if (twi_test_address(RTC_ADR) == 0){
                writeDataToUART(rtc.hours & 0b111111, ":", 0, 0); //& 0b111111 which position i want to send
                writeDataToUART(rtc.mins, ":" , 0, 0);
                writeDataToUART(rtc.secs, " : " , 1, 0);
                if(twi_test_address(OLED_ADR) == 0){ // edit the x y position to make it nice
                    writeDataToOLED(rtc.hours & 0b111111,0,4);
                    oled_gotoxy(2, 4);
                    oled_puts(":");
                    writeDataToOLED(rtc.mins,3,4);
                    oled_gotoxy(5, 4);
                    oled_puts(":");
                    writeDataToOLED(rtc.secs,6,4);
                }

                /*
        	    // Display RTC data
                itoa(rtc.hours, string, 16);
                uart_puts(string);
                uart_puts(":");
                itoa(rtc.mins, string, 16);
                uart_puts(string);
                uart_puts(":");
                itoa(rtc.secs, string, 16);
                uart_puts(string);
                uart_puts(":\t");
                */
            }
            if (twi_test_address(SENSOR_ADR) == 0){ // how to add checking if OLED is connected should be 2 functions that writes into OLEd and into UART(console)
                oled_clrscr(); //it will clear the first message wouldnt the message be there all the time? so load it again
                //writing an integer value of temperature
                //itoa(dht12.temp_int, string, 10);

                writeDataToUART(dht12.temp_int, "." , 0, 0);
                writeDataToUART(dht12.temp_dec, " °C " , 1, 0);
                if(twi_test_address(OLED_ADR) == 0){
                writeDataToOLED(dht12.temp_int,0,2);
                oled_gotoxy(2, 2);
                oled_puts(".");
                writeDataToOLED(dht12.temp_dec,3,2);
                oled_gotoxy(4, 2);
                oled_puts(" °C ");
                }
                /*
                //writing an decimal value of temperature
                itoa(dht12.temp_dec, string, 10);
                uart_puts(string);
                uart_puts(" °C ");
                oled_gotoxy(2, 2);
                oled_puts(".");
                oled_gotoxy(3, 2);
                oled_puts(string);
                oled_gotoxy(4, 2);
                oled_puts(" °C ");
                */
            }
            
            if(mois_int != 0){
                //add RGB LED for each state?
                if (mois_int>850){
                     uart_puts( "\x1b[1;31m"); // Set style to bold, red foreground
                    writeDataToUART(mois_int, " : Plant is thirsty, turning on the pump" , 0, 1);
                    // write code for Relay enabled
                }
                else if(mois_int>750){
                    uart_puts("\x1b[4;32m"); // 4: underline style; 32: green foreground
                    writeDataToUART(mois_int, " : Plant is watered enough, pump is off" , 0, 1);  
                }
                else if(mois_int>700){
                    writeDataToUART(mois_int, " : Plant is watered enough, turning off the pump" , 0, 1);
                    // write code for Relay disabled
                }
                else {
                    writeDataToUART(mois_int, " : Value out of range, there is problem with moisture sensor" , 0, 1);
                }
                uart_puts("\x1b[0m"); // 0: reset all attributes
                // writeDataToUART(mois_int, " " , false, true);
                if(twi_test_address(OLED_ADR) == 0){
                    writeDataToOLED(mois_int*onePercent,0,6);
                    oled_gotoxy(3, 6);
                    oled_puts("%"); //should work
                    oled_gotoxy(5, 6);
                    if (mois_int>850){
                        oled_puts(" : Plant is thirsty, turning on the pump");
                        // write code for Relay enabled
                    }
                    else if(mois_int>750){
                        oled_puts(" : Plant is watered enough, pump is off");   
                    }
                    else if(mois_int>700){
                        oled_puts(" : Plant is watered enough, turning off the pump");
                        // write code for Relay disabled 
                    }
                    else {
                        oled_puts(" [ERROR] Value out of range");
                    }
                }
                /*  
                // writing an procentual value of moisture 
                itoa(mois_int, string, 10);
                uart_puts(string);
                oled_gotoxy(0, 3);
                oled_puts(string);
                oled_gotoxy(3, 3);
                oled_puts("percent");
                //is it really in %?
                if (mois_int>850){
                    uart_puts(": Rostlina má nedostatek vláhy, zapínám zalévání\r\n");
                    oled_gotoxy(0, 4);// can we create some function of animated image on oled?
                    oled_puts("Watering");
                }
                else if(mois_int>750){
                    uart_puts(": Rostlina má dostatek vláhy, zalévání nejede\r\n");
                    oled_gotoxy(0, 4);
                    oled_puts("adekvatna vlaha");    
                }
                else if(mois_int>700){
                    uart_puts(": Rostlina má dostatek vláhy, vypínám zalévání\r\n");
                    oled_gotoxy(0, 4);
                    oled_puts("preliata");
                }
                else {uart_puts(": Chyba snímače, hodnota mimo rozsah měření\r\n");
                }
                */
            }
        oled_display();

        // saving data to RTC EEPROM memory
        // if read previus data from Arduinos EEPROM memory and compare if changed then jump in EEPROM memory and save new data, bcose there is time stamp on the data 
        //saveDataToRtcEeprom();
        
        // Do not print it again and wait for the new data
        new_sensor_data = 0;
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
        // Test ACK from RTC
        twi_start();
        if(twi_write((RTC_ADR<<1) | TWI_WRITE) == 0) { //can it be rewritten to for loop and with array indexed by sec min hour etc.
            // Set internal memory location
            twi_write(SECONDS_REG);
            twi_stop();
            // Read data from internal memory
            twi_start();
            twi_write((RTC_ADR<<1) | TWI_READ);
            rtc.secs = twi_read(TWI_ACK);
            twi_start();
            // Set internal memory location
            twi_write(MINUTES_REG);
            twi_stop();
            rtc.mins = twi_read(TWI_ACK);
            twi_start();
            // Set internal memory location
            twi_write(HOURS_REG);
            twi_stop();
            rtc.hours = twi_read(TWI_ACK);
            twi_start();
            // Set internal memory location
            twi_write(DAY_REG);
            twi_stop();
            rtc.days = twi_read(TWI_ACK);
            twi_start();
            // Set internal memory location
            twi_write(DATE_REG);
            twi_stop();
            rtc.date = twi_read(TWI_ACK);
            twi_start();
            // Set internal memory location
            twi_write(MONTH_REG);
            twi_stop();
            rtc.months = twi_read(TWI_ACK);
            twi_start();
            // Set internal memory location
            twi_write(YEAR_REG);
            twi_stop();
            rtc.years = twi_read(TWI_ACK);
        }
        twi_stop();
}

ISR(ADC_vect)
{
    // char string[4];  // String for converted numbers by itoa()

    // Read converted value
    // Note that, register pair ADCH and ADCL can be read as a 16-bit value ADC
    mois_int = ADC;
    //was there itoa of mois_int to string
}
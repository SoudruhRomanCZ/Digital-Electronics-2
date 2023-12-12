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
uint8_t percentualValue;
uint16_t calculus;
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
#define LED_GREEN PB2   // PB5 is AVR pin where green on-board LED 
#define LED_RED PB3
#define LED_BLUE PB1
#define RELAY PB0
#define BUTTON PB4

int main(void)
{ 
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

    // 
    GPIO_mode_output(&DDRB, LED_GREEN);  // Set output mode in DDRB reg
    GPIO_mode_output(&DDRB, LED_RED);  // Set output mode in DDRB reg
    GPIO_mode_output(&DDRB, LED_BLUE);  // Set output mode in DDRB reg
    GPIO_mode_output(&DDRB, RELAY);  // Set output mode in DDRB reg
    GPIO_mode_input_pullup(&DDRB, BUTTON);

    // Load time into DS3231 
    writeTimeToDS3231(0x00,0x00,0x23,4,30,11,2023); //loading this time to DS3231 after startup so it is not starting from 0 0 0
    
    uint16_t currentAddress = 0 ;
    uint8_t button_was_pressed = 0;
    
    while (1) {
        
        if (new_sensor_data == 1) {
            oled_clrscr();
            
            // Conversion of the value from the plant's moisture sensor into percentages
            calculus = (mois_int - WaterValue);
            calculus = calculus/onePercent;
            percentualValue = 100-calculus;
           
            // Save current data to RTC EEPROM
            currentAddress = saveDataToRtcEeprom(rtc.hours, rtc.mins, rtc.secs, dht12.temp_int,dht12.temp_dec,mois_int );
            uint16_t numoflogs = currentAddress/6;



            // Out of range ERROR handeling
            if(percentualValue > 100){
                percentualValue = 100;
            }


            

            if (percentualValue<30){
                //writeDataToUART(percentualValue, " % : Plant is thirsty, turning on the pump\r\n" , 0, 1);
                GPIO_write_high(&PORTB, LED_RED);
                GPIO_write_low(&PORTB, LED_GREEN);
                GPIO_write_low(&PORTB, LED_BLUE);
                GPIO_write_low(&PORTB, RELAY);
            }
            else if(percentualValue<60){
                //writeDataToUART(percentualValue, " % : Plant is watered enough, pump is off\r\n" , 0, 1);  
                GPIO_write_high(&PORTB, LED_BLUE);
                GPIO_write_low(&PORTB, LED_RED);
                GPIO_write_low(&PORTB, LED_GREEN);
            }
            else if(percentualValue>=60){
                //writeDataToUART(percentualValue, " % : Plant is watered enough, turning off the pump\r\n" , 0, 1);
                GPIO_write_high(&PORTB, LED_GREEN);
                GPIO_write_low(&PORTB, LED_RED);
                GPIO_write_low(&PORTB, LED_BLUE);
                GPIO_write_high(&PORTB, RELAY);
            }

            // Set all obtained data to be displayed on OLED
            if(twi_test_address(OLED_ADR) == 0){

                // Display time from RTC
                if (twi_test_address(RTC_ADR) == 0){
                    writeDataToOLED((rtc.hours& 0b00110000)/16,0,0);
                    writeDataToOLED(rtc.hours& 0b00001111,1,0);
                    oled_gotoxy(2, 0);
                    oled_puts(":");
                    writeDataToOLED((rtc.mins& 0b01110000)/16,3,0);
                    writeDataToOLED(rtc.mins& 0b00001111,4,0);
                    oled_gotoxy(5, 0);
                    oled_puts(":");
                    writeDataToOLED((rtc.secs& 0b01110000)/16,6,0);
                    writeDataToOLED(rtc.secs& 0b00001111,7,0);
                }

                // Display temperature from temperature sensor
                if (twi_test_address(SENSOR_ADR) == 0){
                    writeDataToOLED(dht12.temp_int,0,2);
                    oled_gotoxy(2, 2);
                    oled_puts(".");
                    writeDataToOLED(dht12.temp_dec,3,2);
                    oled_gotoxy(4, 2);
                    //oled_puts(" °C ");
                    // Hope this will display the ° symbol
                    oled_puts(" ");
                    oled_putc((char)247); // degree symbol
                    oled_puts("C ");
                }

                // Display plant's moisture percentage
                writeDataToOLED(percentualValue,0,4);
                //writeDataToOLED(mois_int,8,4);
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

        // Display all set data on OLED
        oled_display();
        // Do not print it again and wait for the new data
        new_sensor_data = 0;
                // If the button was pressed then display the stored data in serial monitor (data EXPORT)


        if(button_was_pressed == 1){
            writeDataToUART(numoflogs, " total number of logs", 0,1); //total number of logs
            for (uint16_t i = 0; i < numoflogs; i++)
            {
                writeDataToUART(i+1, ". log Time : ",0,0); // index of each log
                writeDataToUART((eeprom_read_byte(0+i*6) & 0b00110000)/16, "", 0, 0);//time from hours to seconds
                writeDataToUART(eeprom_read_byte(0+i*6) & 0b00001111, ":" , 0, 0);
                writeDataToUART((eeprom_read_byte(1+i*6) & 0b01110000)/16, "" , 0, 0);
                writeDataToUART(eeprom_read_byte(1+i*6) & 0b00001111, ":" , 0, 0);
                writeDataToUART((eeprom_read_byte(2+i*6) & 0b01110000)/16, "" , 0, 0);
                writeDataToUART(eeprom_read_byte(2+i*6) & 0b00001111, " Temperature : " , 0, 0);
                writeDataToUART(eeprom_read_byte(3+i*6), "." , 0, 0);  //temperature
                writeDataToUART(eeprom_read_byte(4+i*6), " °C Moisture : " , 0, 0);
                writeDataToUART(eeprom_read_byte(5+i*6), " ", 0, 1); // moisture
                uart_puts("\r\n");
            }
            uart_puts("That's all the data !");
            uart_puts("\r\n");
            button_was_pressed = 0;
        }
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
* Purpose:  Read data from sensors.
**********************************************************************/
ISR(TIMER1_OVF_vect)
{   
    ADCSRA = ADCSRA | (1<<ADSC);
    // Test ACK from sensor
    twi_start();
    // Read Temperature and humidity sensor from DTH12; SLA = 0x5c 
    if (twi_write((SENSOR_ADR<<1) | TWI_WRITE) == 0) {
        // Set internal memory location
        twi_write(SENSOR_TEMP_MEM);
        twi_stop();
        // Read data from internal memory
        twi_start();
        twi_write((SENSOR_ADR<<1) | TWI_READ);
        dht12.temp_int = twi_read(TWI_ACK);
        dht12.temp_dec = twi_read(TWI_NACK);
        new_sensor_data = 1;
    }
    twi_stop();

    // Read Time from RTC DS3231; SLA = 0x68
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
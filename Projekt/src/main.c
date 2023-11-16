/* Defines -----------------------------------------------------------*/
#ifndef F_CPU
# define F_CPU 16000000  // CPU frequency in Hz required for UART_BAUD_SELECT
#endif

// Including of needed libraries
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <stdlib.h>         // C library. Needed for number conversions
#include <timer.h>
#include <oled.h>
#include <gpio.h>
#include <twi.h>
#include <uart.h>

/* Global variables --------------------------------------------------*/
// Declaration of "dht12" variable with structure "DHT_values_structure"
struct Values_structure {
   uint8_t hum_int;
   uint8_t hum_dec;
   uint8_t temp_int;
   uint8_t temp_dec;
   uint16_t mois_int;
   uint8_t time_int;
   uint8_t checksum;
} dht12;
// Normal values of moisture sensor
const int AirValue = 620;   //you need to replace this value with Value_1
const int WaterValue = 310;  //you need to replace this value with Value_2

// Flag for printing new data from sensor
volatile uint8_t new_sensor_data = 0;

// Slave and internal addresses of temperature/humidity sensor DHT12
#define SENSOR_ADR 0x5c
#define SENSOR_HUM_MEM 0
#define SENSOR_TEMP_MEM 2
#define SENSOR_CHECKSUM 4
// RTC
#define RTC_ADR  0x68
//OLED
#define OLED_ADR 0x3c

// defining needed variables and pins

#define soil PC0

int main(void)
{/*
    oled_init(OLED_DISP_ON);
    oled_clrscr();

    // oled_gotoxy(x, y)
    oled_gotoxy(0, 2);
    oled_puts("128x64, SHH1106");
*/
    char string[2];  // For converting numbers by itoa()

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
    
    while (1) {
        if (new_sensor_data == 1) {
            itoa(dht12.temp_int, string, 10);
            uart_puts(string);
            uart_puts(".");
            
            itoa(dht12.temp_dec, string, 10);
            uart_puts(string);
            uart_puts(" °C ");
            
            // Convert "value" to "string" and display it
            itoa(dht12.mois_int, string, 10);
            uart_puts(string);
            
            if (dht12.mois_int>850){
                uart_puts(": Rostlina má nedostatek vláhy, zapínám zalévání\r\n");
                }
            else if(dht12.mois_int>750){
                uart_puts(": Rostlina má dostatek vláhy, zalévání nejede\r\n");
                    }
            else if(dht12.mois_int>700){
                uart_puts(": Rostlina má dostatek vláhy, vypínám zalévání\r\n");
                    }
            else {uart_puts(": Chyba snímače, hodnota mimo rozsah měření\r\n");
                    }
            
            // Do not print it again and wait for the new data
            new_sensor_data = 0;
            }
        
    }return 0;
 }
/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
* Function: Timer/Counter1 overflow interrupt
* Purpose:  Read temperature and humidity from DHT12, SLA = 0x5c.
**********************************************************************/
ISR(TIMER1_OVF_vect)
{
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
        new_sensor_data = 1;
    }
    twi_stop();
}
ISR(ADC_vect)
{
    char string[4];  // String for converted numbers by itoa()

    // Read converted value
    // Note that, register pair ADCH and ADCL can be read as a 16-bit value ADC
    dht12.mois_int = ADC;
    // Convert "value" to "string" and display it
    itoa(dht12.mois_int, string, 10);
}
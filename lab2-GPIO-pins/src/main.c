/***********************************************************************
 * 
 * Blink a LED in Arduino-style and use function from the delay library.
 * ATmega328P (Arduino Uno), 16 MHz, PlatformIO
 *
 * Copyright (c) 2022 Tomas Fryza
 * Dept. of Radio Electronics, Brno University of Technology, Czechia
 * This work is licensed under the terms of the MIT license.
 * 
 **********************************************************************/


/* Defines -----------------------------------------------------------*/
#define LED_GREEN PB5   // PB5 is AVR pin where green on-board LED 
                        // is connected
#define LED_RED PB0
#define BUTTON PD2
#define SHORT_DELAY 1000 // Delay in milliseconds
#ifndef F_CPU
# define F_CPU 16000000 // CPU frequency in Hz required for delay funcs
#endif

/* Includes ----------------------------------------------------------*/
#include <avr/io.h>     // AVR device-specific IO definitions
#include <util/delay.h> // Functions for busy-wait delay loops
#include <gpio.h>

// -----
// This part is needed to use Arduino functions but also physical pin
// names. We are using Arduino-style just to simplify the first lab.
// #include "Arduino.h"
// #define PB5 13          // In Arduino world, PB5 is called "13"
// #define PB0 8
// -----


/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Toggle one LED and use delay library.
 * Returns:  none
 **********************************************************************/
int main(void)
{
    uint8_t led_value = 0;  // Local variable to keep LED status
    // uint8_t button_pressed = 1;
    //ver 1
    // Set pin where on-board LED is connected as output
    // pinMode(LED_GREEN, OUTPUT);
    // pinMode(LED_RED, OUTPUT);


    //ver 2
    // DDRB= DDRB | (1<<LED_GREEN);
    // DDRB= DDRB | (1<<LED_RED);

    //ver 3
      // Examples of various function calls
    GPIO_mode_output(&DDRB, LED_GREEN);  // Set output mode in DDRB reg
    GPIO_mode_output(&DDRB, LED_RED);  // Set output mode in DDRB reg
    GPIO_mode_input_pullup(&DDRB, BUTTON);

    // Infinite loop
    while (1)
    {
      if (GPIO_read(&DDRB,BUTTON)){
        // Pause several milliseconds
        _delay_ms(SHORT_DELAY);

        // Change LED value
        if (led_value == 0) {
            led_value = 1;
            // Set pin to HIGH
            //ver 1
            // digitalWrite(LED_GREEN, HIGH);
            // digitalWrite(LED_RED,HIGH);
            //ver 2
            // PORTB=PORTB | (1<<LED_GREEN);
            // PORTB=PORTB | (1<<LED_RED);
            //ver 3
            GPIO_write_high(&PORTB, LED_GREEN);   // Set output low in PORTB reg
            GPIO_write_high(&PORTB, LED_RED);   // Set output low in PORTB reg
        }
        else {
            led_value = 0;
            //ver 1
            // Clear pin to LOW
            // digitalWrite(LED_GREEN, LOW);
            // digitalWrite(LED_RED,LOW);
            //ver 2
            // PORTB=PORTB & ~(1<<LED_GREEN);
            // PORTB=PORTB & ~(1<<LED_RED);
            //ver 3
            GPIO_write_low(&PORTB, LED_GREEN);   // Set output low in PORTB reg
            GPIO_write_low(&PORTB, LED_RED);   // Set output low in PORTB reg
        }
    }
}
    // Will never reach this
    return 0;
}


//VER 1 = 508B easy arduino style
//VER 2 = 182B Low-level (register) style
//VER 3 = 182B 
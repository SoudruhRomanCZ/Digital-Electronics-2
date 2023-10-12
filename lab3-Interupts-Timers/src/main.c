/***********************************************************************
 * 
 * Blink two LEDs using functions from GPIO and Timer libraries. Do not 
 * use delay library any more.
 * 
 * ATmega328P (Arduino Uno), 16 MHz, PlatformIO
 *
 * Copyright (c) 2018 Tomas Fryza
 * Dept. of Radio Electronics, Brno University of Technology, Czechia
 * This work is licensed under the terms of the MIT license.
 * 
 **********************************************************************/


/* Defines -----------------------------------------------------------*/
#define LED_GREEN PB5  // Arduino Uno on-board LED
#define LED_RED PB0    // External active-low LED
#define LED_GREEN2 PB1

/* Includes ----------------------------------------------------------*/
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <gpio.h>           // GPIO library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC


/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Toggle two LEDs using the internal 8- and 16-bit 
 *           Timer/Counter.
 * Returns:  none
 **********************************************************************/
int main(void)
{
    // Set pins where LEDs are connected as output
    GPIO_mode_output(&DDRB, LED_GREEN);
    GPIO_mode_output(&DDRB, LED_GREEN2);
    GPIO_mode_output(&DDRB, LED_RED);
  
    // Configuration of 16-bit Timer/Counter1 for LED blinking
    // Set the overflow prescaler to 262 ms and enable interrupt
    TIM1_OVF_33MS
    TIM1_OVF_ENABLE

    //TIM0_OVF_16MS
    //TIM0_OVF_ENABLE

    TIM2_OVF_16MS
    TIM2_OVF_ENABLE
    // Enables interrupts by setting the global interrupt mask
    sei();

    // Infinite loop
    while (1)
    {
        /* Empty loop. All subsequent operations are performed exclusively 
         * inside interrupt service routines, ISRs */
    }

    // Will never reach this
    return 0;
}


/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
 * Function: Timer/Counter1 overflow interrupt
 * Purpose:  Toggle on-board LED.
 **********************************************************************/
ISR(TIMER1_OVF_vect)
{
    PORTB = PORTB ^ (1<<LED_GREEN);
}
/*
ISR(TIMER0_OVF_vect)
{
      static uint8_t no_of_overflows = 0;

    no_of_overflows++;
    if (no_of_overflows >= 60)
    {
        // Do this every 6 x 16 ms = 100 ms
        no_of_overflows = 0;
        PORTB = PORTB ^ (1<<LED_GREEN2);
    }
    // Else do nothing and exit the ISR
    
}*/
ISR(TIMER2_OVF_vect)
{        
  static uint8_t no_of_overflows = 0;

    no_of_overflows++;
    if (no_of_overflows >= 60)
    {
        
        PORTB = PORTB ^ (1<<LED_RED);
    }
    if (no_of_overflows >= 85)
    {
        no_of_overflows = 0;
        PORTB = PORTB ^ (1<<LED_GREEN2);
    }
    // Change 8-bit timer value anytime it overflows
    TCNT0 = 0;
    // Overflow time: t_ovf = 1/f_cpu * (2^bit-init) * prescaler
    // Normal counting:
    // TCNT0 = 0, 1, 2, ...., 128, 129, ...., 254, 255, 0, 1
    //        |---------------------------------------|
    //                         16 ms
    // t_ovf = 1/16e6 * 256 * 1024 = 16 ms
    //
    // Shortened counting:
    // TCNT0 = 0, 128, 129, ...., 254, 255, 0, 128, ....
    //        |---------------------------|
    //                     8 ms
    // t_ovf = 1/16e6 * (256-128) * 1024 = 8 ms

    
}
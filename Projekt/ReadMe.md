# Smart Plant Watering System Firmware Documentation

## Overview
The Smart Plant Watering System firmware facilitates automated plant care by monitoring soil moisture levels, displaying real-time data on an OLED screen, and controlling watering based on predefined thresholds. This documentation aims to explain the code structure, functionalities, and hardware interactions.

## Members:
- **Roman Křivánek** ID: 240642
- **Filip Frey** ID: 240233
- **Michal Papaj** ID: 240408

## Hardware Used
- **Arduino Uno R3**
- **Capacitive Moisture Sensor V1.2** (or replaceable with a resistance moisture sensor; initial values and one percent value must be recalibrated accordingly)
- **DHT12 Humidity and Temperature Sensor**
- **DS3231 RTC Module** with attached EEPROM memory chip
- **5V Relay Module** or 5V DC Pump (max 20mA current)

## Software Used
- **Visual Studio Code** with PlatformIO extension
- **C Programming Language**
- **Headers and Example Code** sourced from DE2 lectures ([DE2 lectures](https://github.com/tomas-fryza/digital-electronics-2/tree/master/solutions))

## File Structure
### Source Files
- `main.c`: Main functionality of the Smart Plant Watering System.
### Library Files
- `gpio.c`: GPIO pin handling functions.
- `gpio.h`: Header file for GPIO pin handling functions.
- `font.h`: Header file for OLED font-related functionalities.
- `oled.c`: Functions for OLED display control.
- `oled.h`: Header file for OLED display control functions.
- `RTC.c`: Implementation file for Real-Time Clock functionalities.
- `twi.c`: TWI (Two-Wire Interface) functionalities.
- `twi.h`: Header file for TWI functionalities.
- `timer.h`: Custom library for timer configurations.
- `uart.c`: UART (Universal Asynchronous Receiver-Transmitter) operations.
- `uart.h`: Header file for UART operations.


## Functionalities
### Initialization
- Initializes TWI, UART, ADC, and GPIO pins.
- Checks connectivity with the OLED display and presents a boot screen if connected.
### Sensor Readings
- Utilizes TWI to acquire temperature and humidity data from the DHT12 sensor.
- Retrieves time data from the Real-Time Clock (RTC, DS3231) using TWI.
### Moisture Monitoring
- Utilizes ADC for reading soil moisture and computes the percentage of moisture.
### Plant Watering Logic
- Controls the relay based on preset moisture thresholds.
- Indication LEDs (Red, Green, Blue) reflect the current soil moisture level.
### Data Storage
- Saves sensor data (time, temperature, moisture) into the RTC's EEPROM memory.
### User Interaction
- Displays real-time data (time, temperature, moisture) on the OLED screen.
- Allows exporting stored data via UART to a serial monitor upon button press.

## Hardware Configuration
### I/O Pin Configuration
- Soil Moisture Sensor: Connected to ADC0 - arduino pin A0.
- LEDs: Red (PB1), Green (PB2), Blue (PB3) - arduino pins D9,D10,D11.
- Water Pump Relay: Connected to PB0 - arduino pin D8.
- Button for Data Export: Connected to PB4 - arduino pin D12.
### Communication Interfaces
- TWI (Two-Wire Interface): Used for communication with OLED, DHT12 sensor, and RTC.
- UART (Universal Asynchronous Receiver-Transmitter): Configured for serial communication.

## Interrupts
- `TIMER1` Overflow Interrupt: Triggers periodic sensor readings (temperature, humidity, time) and sets flags for new data.
- `ADC` Conversion Complete Interrupt: Captures soil moisture data for percentage calculation.

## Usage
### Setup
- Connect hardware components based on the provided pin configurations.
- Flash the firmware onto the microcontroller.
### Operation
- Power on the system.
- The OLED displays real-time information about temperature, time, and moisture levels.
- LEDs indicate the current moisture status.
- Press the button to export stored data via the UART interface.
### Maintenance
- Adjust moisture threshold values (WaterValue, AirValue) for accurate watering control.
- Ensure correct connections and power supply to all components.

## Source Code
### Main Functionality (`main.c`)
- The `main()` function orchestrates system initialization, sensor readings, moisture calculations, control logic, and user interactions.

#### Soil moisture reading
```
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
// Configure Analog-to-Digital Convertion unit
// Select input channel ADC0 (voltage divider pin)
ADMUX = ADMUX & ~(1<<MUX3 | 1<<MUX2 | 1<<MUX1 | 1<<MUX0);
// Enable ADC module
ADCSRA = ADCSRA | (1<<ADEN);
// Enable conversion complete interrupt
ADCSRA = ADCSRA | (1<<ADIE);
// Set clock prescaler to 128
ADCSRA = ADCSRA | (1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0);
// Determine if the watering system is running or not based on percentual moisture
//low moisture, turning on the pump
if (percentualValue<30){ 
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
// Conversion of the value from the plant's moisture sensor into percentages
calculus = (mois_int - WaterValue);
calculus = calculus/onePercent;
percentualValue = 100-calculus;
// Activating of reading Voltage value of pin PC0- arduinos A0
ADCSRA = ADCSRA | (1<<ADSC);
ISR(ADC_vect)
{
    // Note that, register pair ADCH and ADCL can be read as a 16-bit value ADC
    mois_int = ADC;
}

```

#### Temperature reading
```
code
```

#### Time reading
```
code
```

#### OLED printing
```
code
```

#### Data logging to EEPROM
```
code
```

#### Data printing from EEPROM
```
code
```

## Notes
- Ensure accurate connections and a stable power supply to prevent sensor malfunction.
- Modify moisture threshold values carefully for optimal plant care.

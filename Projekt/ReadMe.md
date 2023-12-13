# Smart Watering Plant project
In this project we will use xxx soil moisture sensor alongside DHT12 humidity sensor and D3231 RTC module.
## Members:
- Roman Křivánek ID: 240642 (resposible for putting together the code + writeTimetoRTC function)
- Filip Frey ID: 240233 (responsible for EEPROM read and write functions)
- Michal Papaj ID: 240408 (responsible for OLED layout)
## Hardware used: 
- Arduino uno R3
- Capacitave moisture sensor - can be used resistance moisture sensor, but must be measured and replaced initial values and onepercentvalue
- DTH12 humodity and temperature sensor
- D3231 RTC module with xxx EEPROM memory chip
- 5V relay module or 5V DC pump (max 20 mA current)
## Software used:
- Visual studio Code with PlatformIO extension
- C programing language
- headers and example code from DE2 lectures (source: [DE2 lectures](https://github.com/tomas-fryza/digital-electronics-2/tree/master/solutions))
## Structure of the project
This project was proggramed in VisualStudio Code with addon named PlatformIO. After creating project folder via PlatformIO interface, it generates basic folder structure where we added needed .h and .c library files from previus lectures of Digital Electronics 2. Picture 1 shows the hierarchy of such project folder.

![Project file hierarchy](pics/hierarchy.jpg)

## Main.c program functions explained
In this section we will look at functions from [Main.c](src/main.c). It is devided by functionality
### Soil moisture reading

![Variables and declaration of used pin](pics/soilVar.jpg)

![Voltage devider preparation](pics/soilPrep.jpg)

![Voltage reading in Interuption](pics/soilRead.jpg)

![LED status and Relay turn on/off](pics/soilStatus.jpg)

### Temperature reading

![Project file hierarchy](pics/hierarchy.jpg)

![Project file hierarchy](pics/hierarchy.jpg)

![Project file hierarchy](pics/hierarchy.jpg)

![Project file hierarchy](pics/hierarchy.jpg)

### Time reading

![Project file hierarchy](pics/hierarchy.jpg)

![Project file hierarchy](pics/hierarchy.jpg)

![Project file hierarchy](pics/hierarchy.jpg)

![Project file hierarchy](pics/hierarchy.jpg)

### OLED printing


### Data logging to EEPROM

![Project file hierarchy](pics/hierarchy.jpg)

### Data printing from EEPROM

![Project file hierarchy](pics/hierarchy.jpg)

## RTC.c program functions explained
In this section we will look at functions in [RTC.c](lib/TWI/RTC.c).
### WriteTimetoRTC
### WritedatatoEEPROM
### ReaddatafromEEPROM

## New functions in old libraries
In this section we will look at new functions placed in libraries UART and OLED, that consist of old functions from that libraries in new coat that suits this program better
### writeToUART

### writeToOLED

###

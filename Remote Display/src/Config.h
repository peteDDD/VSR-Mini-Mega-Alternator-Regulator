#ifndef _CONFIG_H_
#define _CONFIG_H_

//#define LOWVOLTAGETEST  // use to test with only 5 volts on the BatVolt input

//#define BENCHTEST  // use to speed up graph updates while testing

//#define DIAGS_PRINTOUTS

#define BAUD_RATE 9600UL
uint8_t ina226_address = 0x40;  // Default INA226 address is 0x40

const unsigned long SampleInterval = 1500UL;
const unsigned long Screen1DelayForRevCode = 1000UL;
const unsigned long Screen1Duration = 4000UL;

// TFT Hardware Definitions#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
//#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin
#define LCD_RESET 10 // Modified Display to move reset line away from A4 for I2C
#define LCD_DRIVER 0x8357 //  0x9341

#endif //_CONFIG_H_

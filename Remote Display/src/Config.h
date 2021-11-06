#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "Arduino.h"

//****************************************************
// #define BENCHTEST
//****************************************************
#ifdef BENCHTEST
const unsigned long ScreenToggleInterval = 6 * 1000UL;
const unsigned long xAxisTimeInterval = 5 * 1000UL;
#else
const unsigned long ScreenToggleInterval = 15 * 1000UL;
const unsigned long xAxisTimeInterval = 30 * 1000UL; // 30 seconds
#endif
//****************************************************

const unsigned long Screen1DelayForRevCode = 1000UL;
const unsigned long Screen1Duration = 4000UL;

// TFT Hardware Definitions#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin
#define LCD_DRIVER 0x9341

// String Parsing Variables
#define DATABUFFERSIZE 40
#define DISPLAYSTRINGHEADER "$D" 
char    delimiters[2] {':',','};
const char    startChar = '$';
const char    endChar   = '\r';

char    dataBuffer[DATABUFFERSIZE+1] = {'\0'}; // Make sure there's always a null character at end of buffer

#endif //_CONFIG_H_

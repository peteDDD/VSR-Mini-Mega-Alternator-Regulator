#ifndef _MAIN_H_
#define _MAIN_H_

//SOFTWARE REVISION
char Revision [] = "Display rev 10c";

// includes
#include <Arduino.h>
#include "Config.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <Adafruit_I2CDevice.h> // Needed to fix problem with PIO not walking the library folders
#include "LCDfield.h"

// forward declarations

bool getSerialString(void);
void DrawScreen1(void);
bool ParseString(void);
void DrawStaticScreen2(void);
void RefreshScreen2(void);
void RefreshScreen3(void);
void DrawGraphLines(int);
void DrawYGrid(void);
void DrawXGrid(void);
void LabelAxes(void);
void UpdateGraphDataArray(void);
void ToggleScreens(void);

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

uint8_t Volts[GridBottomRightX - GridBottomLeftX];//(volts comes in as a string float, is multiplied by 10, and saved as an integer)
uint8_t Amps[GridBottomRightX - GridBottomLeftX];// (amps come in as a string integer, saves as an integer)
uint8_t xAxisTime = 0;

unsigned long currentTime, startTime;
unsigned long screenToggleStartTime;

uint8_t objectNumber;
uint8_t screenState;

//Display States
enum States {
  Screen1New     = 1,
  Screen1Active  = 2,
  Screen2New     = 3,
  Screen2Active  = 4,
  Screen3New     = 5,
  Screen3Active  = 6
};

#endif //_MAIN_H_
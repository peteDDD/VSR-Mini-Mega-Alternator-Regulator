#ifndef _MAIN_H_
#define _MAIN_H_

//SOFTWARE REVISION
char Revision [] = "1/3/21";

// includes
#include <Arduino.h>
#include <wire.h>
#include <INA226.h>
#include <Config.h>

//Class initialization
INA226 ina;
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <Adafruit_I2CDevice.h> // Needed to fix problem with PIO not walking the library folders
#include "LCDfield.h"

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

// forward declarations
void getNewData(void);
void checkForChanges(void);
//bool getSerialString(void);
void DrawScreen1(void);
//bool ParseString(void);
void DrawStaticScreen2(void);
void RefreshScreen2(void);
void RefreshScreen3(void);
void DrawGraphLines(int);
void DrawYGrid(void);
void DrawXGrid(void);
void LabelAxes(void);
void UpdateGraphDataArray(void);
void ToggleScreens(void);
char *float2string(float v, uint8_t decimals);
float roundoff(float num, int precision);

uint8_t Volts[GridRightX - GridLeftX];//(volts comes in as a string float, is multiplied by 10, and saved as an integer)
int16_t Amps[GridRightX - GridLeftX];// (amps come in as a string integer, saves as an integer)
uint8_t xAxisTime = 0;

unsigned long currentTime, startTime, sampleTime;
unsigned long screenToggleStartTime;

//uint8_t objectNumber;
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
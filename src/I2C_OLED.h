//      I2C_OLED.h                                                           http://smartdcgenerator.blogspot.com/
//
//      Copyright (c) 2021 by Pete Dubler
//
//              This program is free software: you can redistribute it and/or modify
//              it under the terms of the GNU General Public License as published by
//              the Free Software Foundation, either version 3 of the License, or
//              (at your option) any later version.
//      
//              This program is distributed in the hope that it will be useful,
//              but WITHOUT ANY WARRANTY; without even the implied warranty of
//              MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//              GNU General Public License for more details.
//      
//              You should have received a copy of the GNU General Public License
//              along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//
#ifndef _I2C_OLED_H_
#define _I2C_OLED_H_

#define LCD_ADDRESS 0x03C          // I2C address of the Geekcreit SSD1306
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

SSD1306AsciiWire oled;

extern const char *  chargingStateString;
extern int    inChargingStateCount;

// LCD SCREEN FORMAT CONSTANTS AND VARIABLES
#define screen2PixelsPerChar 7

//FieldNumber  (this enum table must match the one in Remote Display)
enum FieldNumber {
  fnAV = 0,  // Alternator Volts
  fnBV = 1,  // Battery Volts
  fnAA = 2,  // Alternator Amps
  fnBA = 3,  // Battery Amps
  fnAT = 4,  // Alternator Temperature
  fnBT = 5,  // Battery Temperature
  fnPW = 6,  // PWM
  fnCD = 7,  // CountDown
  fnCS = 8,  // ChargeState  -- update display after CountDown to overwrite any dangling countdown number
  fnSB = 9,  // Scuba
  fnFL = 10, // FAULT
  fnCR = 11, // Alternator Controler code revision
  fnTM = 12, // Time HH:MM:SS in current chargingState
  //TODO Need to add fnTM to remote display code
  fnSCREEN2 = 92 // Clear screen, write Screen2
};


//Top row is 0
#define screen2RowVolt      1
#define screen2RowAmp       2
#define screen2RowTemp      3
#define screen2RowPWM       5
#define screen2RowState     7
#define screen2RowCount     screen2RowState
#define screen2RowScuba     screen2RowState

//Column variable is number of pixels
#define screen2ColAlt       6 *screen2PixelsPerChar
#define screen2ColBat       13*screen2PixelsPerChar
#define screen2ColPWM       9 *screen2PixelsPerChar
#define screen2ColCount     7 *screen2PixelsPerChar // for seconds count down
#define screen2ColCount2    11*screen2PixelsPerChar // for hr:mm:ss count up
#define screen2ColState     0
#define screen2ColScuba     13*screen2PixelsPerChar

//Erase variable is number of characters
#define screen2EraseVolt        6
#define screen2EraseAmp         6
#define screen2EraseTemp        7
#define screen2ErasePWM         4 
#define screen2EraseCount       3
#define screen2EraseCount2      4
#define screen2EraseState       8 //11
#define screen2EraseScuba       5

//Format is single character to follow output
#define screen2FormatVolt       ' '
#define screen2FormatAmp        ' '

#ifdef OLED_DISPLAY_DEG_IN_F
  #define screen2FormatTemp     'F'
#else
  #define screen2FormatTemp     'C'
#endif

#define screen2FormatPWM        '%'
#define screen2FormatState      ' '
#define screen2FormatCount      ' '
#define screen2FormatScuba      ' '

// END LCD SCREEN FORMAT CONSTANTS AND VARIABLES

//Template Class

template <class T>
class LCDfield {
  public:
    T  m_lastValue;
    uint8_t    m_ObjectNumber;
    uint8_t    m_column;
    uint8_t    m_row;
    uint8_t    m_eraseWidth;
    uint8_t    m_Digits;  //number of decimal places
    char       m_Format;
  public:
    LCDfield(uint8_t object, int column,  uint8_t row,  uint8_t eraseWidth,  uint8_t Digits, char Format)
    {
      m_ObjectNumber = object;
      m_column = column;
      m_row = row;
      m_eraseWidth = eraseWidth;
      m_Digits = Digits;
      m_Format = Format;
    }

    //Overload option to provide initial value for LastValue
    //Used to preload PWM so 0% is displayed during ramping state
    LCDfield(T lastValue, uint8_t object, int column,  uint8_t row,  uint8_t eraseWidth,  uint8_t Digits, char Format)
    {
      m_lastValue = lastValue;
      m_ObjectNumber = object;
      m_column = column;
      m_row = row;
      m_eraseWidth = eraseWidth;
      m_Digits = Digits;
      m_Format = Format;
    }

    void Write(T);
    void Update(T);
}; // don't forget the semicolon at the end of the class

//                      (optional)
// construct instances  LastValue), ObjectNumber, column,           row,              EraseWidth,      Digits,  Format
// construct instances
LCDfield <float> LCDaltVolts(         fnAV,       screen2ColAlt,    screen2RowVolt,   screen2EraseVolt,   2, screen2FormatVolt);
LCDfield <float> LCDbatVolts(         fnBV,       screen2ColBat,    screen2RowVolt,   screen2EraseVolt,   2, screen2FormatVolt);
LCDfield <int>   LCDaltAmps(          fnAA,       screen2ColAlt,    screen2RowAmp,    screen2EraseAmp,    0, screen2FormatAmp);
LCDfield <int>   LCDbatAmps(          fnBA,       screen2ColBat,    screen2RowAmp,    screen2EraseAmp,    0, screen2FormatAmp);
LCDfield <int>   LCDaltTemp(          fnAT,       screen2ColAlt,    screen2RowTemp,   screen2EraseTemp,   0, screen2FormatTemp);
LCDfield <int>   LCDbatTemp(          fnBT,       screen2ColBat,    screen2RowTemp,   screen2EraseTemp,   0, screen2FormatTemp);
// pre-load lastValue for PWM so 0% is displayed during ramping state
LCDfield <int>   LCDPWM(    0,        fnPW,       screen2ColPWM,    screen2RowPWM,    screen2ErasePWM,    0, screen2FormatPWM);
LCDfield <const char *> LCDState(     fnCS,       screen2ColState,  screen2RowState,  screen2EraseState,  0, screen2FormatState);
LCDfield <int>   LCDCount(            fnCD,       screen2ColCount,  screen2RowCount,  screen2EraseCount,  0, screen2FormatCount);
//LCDfield <int>    LCDCount2(          fnCD,       screen2ColCount2, screen2RowCount,  screen2EraseCount2, 0, screen2FormatCount);
LCDfield <char *> LCDCount2(          fnTM,       screen2ColCount2, screen2RowCount,  screen2EraseCount2, 0, screen2FormatCount);
//LCDfield <const char *> LCDCountString(fnCD,      screen2ColCount3, screen2RowCount,  screen2EraseCount2, 0, screen2FormatCount);
LCDfield <const char *> LCDScuba(     fnSB,       screen2ColScuba,  screen2RowPWM,  screen2EraseScuba,  0, screen2FormatScuba);


// Utility to round floating number precision to a specified number of decimal places
float roundoff(float num,int precision)
{
      int temp=(int )(num*pow(10,precision));
      int num1=num*pow(10,precision+1);
      temp*=10;
      temp+=5;
      if(num1>=temp)
              num1+=10;
      num1/=10;
      num1*=10;
      num=num1/pow(10,precision+1);
      return num;
}//float roundoff(float num,int precision)

//Template Functions

template <>  //forced write to the field.  No checking for changed field
void LCDfield<char *>::Write(char *newValue) {
    oled.clearField(m_column, m_row, m_eraseWidth);
    oled.print(newValue);
    oled.println(m_Format);
#ifdef USE_SERIAL_DISPLAY
    char buffer[40];

    //sprintf_P(buffer, PSTR("DSP:%d,%s%c"),  // alternative to use program memory instead of RAM
    sprintf(buffer, "$D:%d,%s%c",
            m_ObjectNumber,
            newValue, m_Format);
    SERIAL_DISPLAY_PORT.println(buffer);
#endif
  }

template <>  //overload function for const char* strings
void LCDfield<const char *>::Update(const char* newValue) {
  if (newValue != m_lastValue) {
    m_lastValue = newValue;
    oled.clearField(m_column, m_row, m_eraseWidth);
    oled.print(newValue);
    oled.println(m_Format);
#ifdef USE_SERIAL_DISPLAY
    char buffer[40];

    //sprintf_P(buffer, PSTR("DSP:%d,%s%c"),  // alternative to use program memory instead of RAM
    sprintf(buffer, "$D:%d,%s%c",
            m_ObjectNumber,
            newValue, m_Format);
    SERIAL_DISPLAY_PORT.println(buffer);
#endif
  }
}

template <>  //catches floats
void LCDfield<float>::Update(float newValue) {
  newValue = roundoff(newValue, 2); // set precision to two decimal places
  if (newValue != m_lastValue) {
    m_lastValue = newValue;

    oled.clearField(m_column, m_row, m_eraseWidth);
    oled.print(newValue, m_Digits);  // this funtion call format will not work with const char* or String
    oled.println(m_Format);
#ifdef USE_SERIAL_DISPLAY
    char buffer[40];
    extern char *float2string(float v, uint8_t decimals);
    //sprintf_P(buffer, PSTR("DSP:%d,%s%c"),  // alternative to use program memory instead of RAM
    sprintf(buffer, "$D:%d,%s%c",
            m_ObjectNumber,
            float2string(newValue, m_Digits), m_Format);
    SERIAL_DISPLAY_PORT.println(buffer);
#endif
  }//if (newValue != m_lastValue) 
}//void LCDfield<float>::Update(T newValue) 

template <typename T>  //catches all other types  ... integers
void LCDfield<T>::Update(T newValue) {
 if (newValue != m_lastValue) 
 {
 /*    // Here we have a patch to prevent a "%%" from appearing on the OLED
    if(this==&LCDPWM)  // only do this for the PWM field
    {
      if((m_lastValue == 100) && (newValue < 100)) // transition down from 100%
      {
        oled.clearField(m_column, m_row, m_eraseWidth + screen2PixelsPerChar); //erase one character wider
        //TODO do we also need to update the scubastring here so we don't erase part of that?
      }
    } //if(this==&LCDPWM)
*/
  m_lastValue = newValue;
  oled.clearField(m_column, m_row, m_eraseWidth);  // does not matter if it clears the field twice...
  oled.print(newValue);
  oled.println(m_Format);

#ifdef USE_SERIAL_DISPLAY
    char buffer[40];
    // TRAP TO FIX DANGLING COUNTDOWN DIGIT ON REMOTE SERIAL DISPLAY
    if ((m_ObjectNumber == fnCD) && (newValue == 0)) 
    { //Trap to catch dangling countdown digit on remote display
      sprintf(buffer, "$D:%d,%s%c",
              m_ObjectNumber,
              "  ", m_Format);
    }//if (m_ObjectNumber == fnCD)
    // END OF TRAP for dangling countdown digit
    else {
      //sprintf_P(buffer, PSTR("DSP:%d,%d%c"), // alternative to use program memory instead of RAM
      sprintf(buffer, "$D:%d,%d%c",
              m_ObjectNumber,
              newValue, m_Format);
    }//else
    SERIAL_DISPLAY_PORT.println(buffer);
#endif
  }//if (newValue != m_lastValue) 
}//void LCDfield<T>::Update(T newValue) 


#endif
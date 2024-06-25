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

extern SSD1306AsciiWire oled;

void OLEDPrintlnCentered(const char *text, uint8_t row = 0);
void OLEDWriteFeatureAssignments (void);
void OLEDWriteSerialPortAssignments(void);
float roundoff(float num,int precision);

// LCD SCREEN FORMAT CONSTANTS AND VARIABLES
#define screen2PixelsPerChar 7

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
};

//Template Function Definitions
template <>  //forced write to the field.  No checking for changed field
void LCDfield<char *>::Write(char *newValue);

template <>  //overload function for const char* strings
void LCDfield<const char *>::Update(const char* newValue);

template <>  //catches floats
void LCDfield<float>::Update(float newValue);

template <typename T>  //catches all other types  ... integers
void LCDfield<T>::Update(T newValue) {
    if (newValue != m_lastValue) {
        m_lastValue = newValue;
        oled.clearField(m_column, m_row, m_eraseWidth);
        oled.print(newValue);
        oled.println(m_Format);
#ifdef USE_SERIAL_DISPLAY
        char buffer[40];
        if ((m_ObjectNumber == fnCD) && (newValue == 0)) {
            sprintf(buffer, "$D:%d,%s%c",
                    m_ObjectNumber,
                    "  ", m_Format);
        } else {
            sprintf(buffer, "$D:%d,%d%c",
                    m_ObjectNumber,
                    newValue, m_Format);
        }
        SERIAL_DISPLAY_PORT.println(buffer);
#endif
    }
}

extern LCDfield<float> LCDaltVolts;
extern LCDfield<float> LCDbatVolts;
extern LCDfield<int> LCDaltAmps;
extern LCDfield<int> LCDbatAmps;
extern LCDfield<int> LCDaltTemp;
extern LCDfield<int> LCDbatTemp;
// pre-load lastValue for PWM so 0% is displayed during ramping state
extern LCDfield<int> LCDPWM;
extern LCDfield<const char *> LCDState;
extern LCDfield<int> LCDCount;
extern LCDfield<char *> LCDCount2;
extern LCDfield<const char *> LCDScuba;

#endif
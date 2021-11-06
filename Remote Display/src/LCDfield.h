#ifndef _LCDFIELD_H_
#define _LCDFIELD_H_

#include <Arduino.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library

#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

extern Adafruit_TFTLCD tft;
extern char Buffer[80];

//Fonts
#include <Fonts/FreeSans12pt7b.h> // best font
#include <Fonts/FreeSans18pt7b.h> // best font

// Assign human-readable names to some common 16-bit color values:
#define BLACK       0x0000
#define BLUE        0x001F
#define RED         0xF800
#define GREEN       0x07E0
#define CYAN        0x07FF
#define MAGENTA     0xF81F
#define YELLOW      0xFFE0
#define WHITE       0xFFFF
#define ORANGE      0xFD20
#define GREENYELLOW 0xAFE5
#define NAVY        0x000F
#define DARKGREEN   0x03E0
#define DARKCYAN    0x03EF
#define MAROON      0x7800
#define PURPLE      0x780F
#define OLIVE       0x7BE0
#define LIGHTGREY   0xC618
#define DARKGREY    0x7BEF
#define GOLD        0x6ABC

//Screen1
const int     Screen1FillColor = DARKGREEN;
const int     Screen1TextColor = WHITE;

//Screen2
const int     Screen2FillColor = YELLOW;
const int     Screen2TextColor = BLUE;
#define       Screen2Font &FreeSans18pt7b

//Screen3
const int     Screen3FillColor = WHITE;
const int     Screen3GridLineColor = YELLOW;
#define       Screen3Font &FreeSans12pt7b
const int     VoltColor = BLUE;
const int     AmpColor  = RED;

//FieldNumber
enum FieldNumber {
  fnAV = 0,  // Alternator Volts
  fnBV = 1,  // Battery Volts
  fnAA = 2,  // Alternator Amps
  fnBA = 3,  // Battery Amps
  fnAT = 4,  // Alternator Temperature
  fnBT = 5,  // Battery Temperature
  fnPW = 6,  // PWM
  fnCD = 7,  // CountDown
  fnCS = 8,  // ChargeState
  fnSB = 9,  // Scuba
  fnFL = 10, // FAULT
  fnCR = 11, // Alternator Controler code revision
  fnSCREEN2 = 92 // Clear screen, write Screen2
};

//TEST DATA
/*
  $DSP:1,12.40
  $DSP:2,12.50
  $DSP:3,100
  $DSP:4,76
  $DSP:5,132
  $DSP:6,200
  $DSP:7,88%
  $DSP:8,Ramping
  $DSP:9,32
  $DSP:10,SCUBA
  $DSP:10,     ,
  $DSP:92,0
  $DSP:11,FAULT 788
*/

//Justify
const uint8_t RJ = 1; //Right
const uint8_t LJ = 2; //Left
const uint8_t CJ = 3; //Center

//Column (X) variable is number of pixels
const uint16_t screen2ColAlt     = 120;
const uint16_t screen2ColBat     = 220;
const uint16_t screen2ColPWM     = 220;
const uint16_t screen2ColCount   = 180;
const uint16_t screen2ColState   = 1;
const uint16_t screen2ColScuba   = 310;

//Top row (Y) is 0
const uint16_t Y_Pixels_12pt  = 21;  //without descenders, Theight is 18, with descenders (used in lower case letters), tHeight is 23
const uint16_t Y_Pixels_18pt  = 31;  //without descenders, Theight is 26, with descenders (used in lower case letters), tHeight is 34

const uint16_t screen2RowTop      = 1 * Y_Pixels_18pt;
const uint16_t screen2RowVolt     = 2 * Y_Pixels_18pt;
const uint16_t screen2RowAmp      = 3 * Y_Pixels_18pt;
const uint16_t screen2RowTemp     = 4 * Y_Pixels_18pt;
const uint16_t screen2RowPWM      = 5 * Y_Pixels_18pt + 5;
const uint16_t screen2RowState    = 6 * Y_Pixels_18pt + 8;
const uint16_t screen2RowCount    = screen2RowState;
const uint16_t screen2RowScuba    = 235;


struct ScreenData
{
  uint16_t  column;
  uint16_t  row;
  const char * text;
};

const uint8_t Screen2StaticLines = 6;
struct ScreenData Screen2StaticData[Screen2StaticLines] = {
  {screen2ColAlt   + 15, screen2RowTop,  "ALT"},
  {screen2ColBat   + 15, screen2RowTop,  "BAT"},
  {screen2ColState +  6, screen2RowVolt, "VOLT:"},
  {screen2ColState + 22, screen2RowAmp,  "AMP:"},
  {screen2ColState,      screen2RowTemp, "TEMP:"},
  {screen2ColState,      screen2RowPWM,  "Field Drive:"}
};

const uint8_t UpperLeftX   = 2;
const uint8_t UpperLeftY   = 2;
const uint8_t BottomLeftX  = 2;
const int GridBottomLeftX  = 40; // must be int for array sizing to match BottomRightX type
const uint8_t BottomY      = 238;
const uint8_t GridBottomY  = 228;
const int BottomRightX     = 318; 
const int GridBottomRightX = 265; // must be int for array sizing
const uint8_t GridYZero    = 28;
const uint8_t DisplayMaxY  = 240;
const uint8_t LabelXOffset = 50;
const uint8_t LabelYOffset = 3;


const uint8_t GraphLowerVoltage    = 120; // (12V * 10)
const uint8_t GraphUpperVoltage    = 145; // (14.5V * 10)
const float GraphLowerVoltageValue = 12.0;
const float GraphVoltageStep       = 0.5;
const uint8_t GraphLowerAmpValue   = 0;
const uint8_t GraphAmpStep         = 25;

const uint8_t YGridStep = 40;
const uint8_t XGridStep = 30;  // with xAxisTimeInterval = 30, 30 here = 15 minutes



// Class
class IScreens {
  public:
    virtual void SetNewValue(const char* newValue) = 0;
    virtual void UpdateDisplayField() = 0;
    virtual const char * GetNewValue();
}; // don't forget the semicolon at the end of the class

template <size_t arraysize>
class LCDfield : public IScreens {
  public:
    //constructor
    LCDfield( uint16_t x,  uint16_t y, const GFXfont *Font, uint8_t just = LJ)
    {
      m_X = x;
      m_Y = y;
      m_Font = Font;
      m_Just = just;
    }

    virtual void SetNewValue(const char* newValue);
    virtual void UpdateDisplayField();
    virtual const char * GetNewValue();

  private:
    char          m_newValue[arraysize + 1] = {'\0'}; // Make sure there is always a null character at the end
    int           m_X, m_Y; // This is the cursor reference position (for example, the center of the string if using Center Justify)
    int           m_X1, m_Y1;  // Used for position of fillRect to write over last string printed
    const         GFXfont *m_Font;
    uint8_t       m_Just; // Specifies the text Justification (position relative to m_X, m_Y, as Right, Left, or Center)
    int           m_LLX, m_LLY; //corrected cursor position for the start of the string - Proportional Fonts use Lower Left for origin of string
    unsigned int  m_Twidth, m_Theight;
}; // don't forget the semicolon at the end of the class

template <size_t arraysize>
void LCDfield<arraysize>::SetNewValue(const char* newValue) {
  strncpy(m_newValue, newValue, arraysize);
}

template <size_t arraysize>
const char * LCDfield<arraysize>::GetNewValue() {
 return(m_newValue);
}

template <size_t arraysize>
void LCDfield<arraysize>::UpdateDisplayField(void) {
  // calculate size and position  newValue data will have on the display

  tft.setFont(m_Font);

  // Draw over existing text by using values previously calculated
  tft.fillRect(m_X1, m_Y1, m_Twidth, m_Theight, Screen2FillColor);  
  
  // calculate the size of the new text
  // by using 0,0 for first two variables, we just measure the width and height
  tft.getTextBounds(m_newValue, 0, 0, &m_X1, &m_Y1, &m_Twidth, &m_Theight); 

  //** Justify the text **
  // Right Justify  move cursor left by Twidth pixels
  if (m_Just == RJ)  m_LLX = m_X - m_Twidth;  

  // Center Justify   move cursor left by Twidth/2 pixels
  else if (m_Just == CJ)  m_LLX = m_X - (m_Twidth >> 1);  // (shift to right by one place divides by two with no remainder)
  
  // default to Left Justify
  else m_LLX = m_X;  

  // set Y position
  m_LLY = m_Y;  

  // calculate specs of rectangle to draw over existing text next time Update is called
  tft.getTextBounds(m_newValue, m_LLX, m_LLY, &m_X1, &m_Y1, &m_Twidth, &m_Theight); 

  //Now display
  tft.setCursor(m_LLX, m_LLY);
  tft.println(m_newValue);
}//Update()

//LCDfield Screen 1 Objects                                                   (Optional)
//construct instances     column,           row,               Font          Justification
LCDfield <20> fS1L1 {     160,              40,               &FreeSans18pt7b,  CJ  };
LCDfield <20> fS1L2 {     160,              80,               &FreeSans18pt7b,  CJ  };
LCDfield <20> fS1L3 {     160,              120,              &FreeSans18pt7b,  CJ  };
LCDfield <20> fS1L4 {     160,              160,              &FreeSans12pt7b,  CJ  };

//List of Screen1 objects
IScreens* const Screen1Objects[] = {&fS1L1, &fS1L2, &fS1L3, &fS1L4};


//LCDfield Screen 2 Objects                                             (Optional)
//construct instances   column,           row,               Font       Justification
LCDfield <5>  fAV  {  screen2ColAlt,    screen2RowVolt,   &FreeSans18pt7b,  LJ  };
LCDfield <5>  fBV  {  screen2ColBat,    screen2RowVolt,   &FreeSans18pt7b,  LJ  };
LCDfield <3>  fAA  {  screen2ColAlt,    screen2RowAmp,    &FreeSans18pt7b,  LJ  };
LCDfield <3>  fBA  {  screen2ColBat,    screen2RowAmp,    &FreeSans18pt7b,  LJ  };
LCDfield <4>  fAT  {  screen2ColAlt,    screen2RowTemp,   &FreeSans18pt7b,  LJ  };
LCDfield <4>  fBT  {  screen2ColBat,    screen2RowTemp,   &FreeSans18pt7b,  LJ  };
LCDfield <2>  fCD  {  screen2ColCount,  screen2RowCount,  &FreeSans18pt7b,  LJ  };
LCDfield <13> fCS  {  screen2ColState,  screen2RowState,  &FreeSans18pt7b,  LJ  };
LCDfield <4>  fPW  {  screen2ColPWM,    screen2RowPWM,    &FreeSans18pt7b,  LJ  };
LCDfield <5>  fSB  {  screen2ColScuba,  screen2RowScuba,  &FreeSans18pt7b,  RJ  };
LCDfield <30> fCR  {  160,              200,              &FreeSans12pt7b,  CJ  };
LCDfield <20> fFL  {  screen2ColState,  screen2RowScuba,  &FreeSans18pt7b,  LJ  };

//List of Screen2 objects - does not include rev code field or fault code field 
// the order of these must follow the order of the FieldNumber enum
IScreens* const Screen2Objects[] = {&fAV, &fBV, &fAA, &fBA, &fAT, &fBT, &fPW, &fCD, &fCS, &fSB};
#endif //_LCDFIELD_H_

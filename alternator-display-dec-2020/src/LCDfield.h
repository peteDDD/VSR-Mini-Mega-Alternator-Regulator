#ifndef _LCDFIELD_H_
#define _LCDFIELD_H_

//Driver Libraries
#include <Arduino.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library

//Fonts
#include <Fonts/FreeSans12pt7b.h> // best font
//#include <Fonts/FreeSans18pt7b.h> // best font
//#include <Fonts/SansSerifBIDIGS64GFX.h> //huge digits only
#include <Fonts/SansSerifBIdigits84.h> //extra huge digits only


extern Adafruit_TFTLCD tft;

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

//****************************************************************
//Screen1
const int     Screen1FillColor = DARKGREEN;
const int     Screen1TextColor = WHITE;
//****************************************************************

//****************************************************************
//Screen2
//****************************************************************
const int     Screen2FillColor = YELLOW;
const int     Screen2TextColor = BLUE;
//#define       Screen2Font &FreeSans18pt7b
#define       Screen2Font &FreeSans12pt7b

//const uint16_t Y_Pixels_12pt  = 21;  //without descenders, Theight is 18, with descenders (used in lower case letters), tHeight is 23
const uint16_t Y_Pixels_18pt  = 31;  //without descenders, Theight is 26, with descenders (used in lower case letters), tHeight is 34

// Screen2 Column (X) - variable is number of pixels
const uint16_t screen2ColAlt     = 120;
const uint16_t screen2ColBat     = 220;
const uint16_t screen2ColState   = 1;

// Screen2 Row (Y) - variable is number of pixels
// Top row (Y) is 0
const uint16_t screen2RowVolt     = 2 * Y_Pixels_18pt * 2 - 10;
const uint16_t screen2RowAmp      = 3 * Y_Pixels_18pt * 2 + 20;
const uint16_t screen2ColNum      = 140;



//****************************************************************
//Screen3
//****************************************************************
const int     Screen3FillColor = WHITE;
const int     Screen3GridLineColor  = YELLOW;
const int     Screen3GridLineColor2 = GREENYELLOW;
#define       Screen3Font &FreeSans12pt7b

// Screen 3 Voltage bars on graph
const int     VoltColor = BLUE;
const uint8_t GraphVoltageScaleFactor = 10;  // voltage values are scaled by 10 to plot at finer resolution
const float   GraphLowerVoltageValue  = 12.0;
const uint8_t GraphLowerVoltageScaled = int(GraphLowerVoltageValue * GraphVoltageScaleFactor); // 120 (12V * 10)
const float   GraphUpperVoltageValue  = 14.5;
const uint8_t GraphUpperVoltageScaled = int(GraphUpperVoltageValue * GraphVoltageScaleFactor); // 145 (14.5V * 10)
const float   GraphVoltageStep        = 0.5;
const uint8_t GraphVoltBarWidth       = 1;

// Screen 3 Amp line on graph
// Top row (Y) is 0
const int     AmpColor  = RED;
const uint8_t GraphLowerAmpValue   = 0;
const uint8_t GraphUpperAmpValue   = 125;
const uint8_t GraphAmpStep         = 25;
const uint8_t GraphAmpLineWidth    = 1;
const uint8_t GraphAmpLineHeight   = 4;
const uint8_t GraphAmpLineMarginHeight = 1;

// Screen 3 Coordinates
const uint8_t LeftX        = 2;
const int     RightX       = 318;
const uint8_t UpperY       = 2;
const uint8_t BottomY      = 238;
const uint8_t DisplayMaxY  = 240;

const int GridLeftX  = 40; // must be int for array sizing to match BottomRightX type
const int GridRightX = 265; // must be int for array sizing
const uint8_t GridYTop     = 28;
const uint8_t GridBottomY  = 228;

// Screen3 Graph Steps and Offsets
const uint8_t GridYStep = 40;
//const uint8_t GridXStep = 30;  // with xAxisTimeInterval = 30, 30 here = 15 minutes
const uint8_t LabelXOffset = 50;
const uint8_t LabelYOffset = 3;

#ifdef BENCHTEST
const unsigned long ScreenToggleInterval = 6 * 1000UL;
const unsigned long xAxisTimeInterval = 5 * 1000UL;
const uint8_t       PixelsPer15Min = 30;  // not really accurate here, but used for vertical grid on graph
#else
const unsigned long ScreenToggleInterval = 10 * 1000UL;
// Calculate seconds per pixel of X-axis of graph
//   two hours = 120 minutes * 60 seconds  divided by 
//   number of pixels on X-axis of graph, all times 1000ms/sec
const unsigned long xAxisTimeInterval = ((120 * 60)/(GridRightX-GridLeftX)) * 1000UL; // 32 sec/pixl
// Make GridXStep equal to 15 minutes
const int       GridXStep         = int(15 * 60UL)/(xAxisTimeInterval / 1000UL); // (15min * 60sec/min) / (pixelsPerInterval/1000ms/sec)
#endif

//****************************************************************

//Justify
const uint8_t RJ = 1; //Right
const uint8_t LJ = 2; //Left
const uint8_t CJ = 3; //Center

struct ScreenData
{
  uint16_t  column;
  uint16_t  row;
  const char * text;
};

const uint8_t Screen2StaticLines = 6;
struct ScreenData Screen2StaticData[Screen2StaticLines] = {
  {290, screen2RowVolt, "V"},
  {290, screen2RowAmp,  "A"},
};

// Class for display field parameters and data
class IScreens {
  public:
    virtual void SetNewValueChar(const char* newValue) = 0;
    virtual void SetNewValueFloat(float newValue) = 0;
    virtual void UpdateDisplayField() = 0;
    virtual const char * GetNewValueChar() = 0;
    virtual float GetNewValueFloat() = 0;
    virtual bool CheckForChanges() = 0;
    virtual void MoveFloat2Char() = 0;
}; // don't forget the semicolon at the end of the class

template <size_t arraysize>
class LCDfield : public IScreens {
  public:
    //constructor
    LCDfield( uint16_t x,  uint16_t y, const GFXfont *Font, uint8_t just = LJ, uint8_t textSize = 1, int decimalPlaces = 0)
    {
      m_X = x;
      m_Y = y;
      m_Font = Font;
      m_Just = just;
      m_precision = decimalPlaces;
      m_textSize = textSize;
    }

    virtual void SetNewValueChar(const char* newValue);
    virtual void SetNewValueFloat(float newValue);
    virtual void UpdateDisplayField();
    virtual const char * GetNewValueChar();
    virtual float GetNewValueFloat();
    virtual bool CheckForChanges();
    virtual void MoveFloat2Char();

  private:
    char          m_newValueChar[arraysize + 1] = {'\0'}; // Make sure there is always a null character at the end
    float         m_oldValueFloat = .5; // seed with small value so displayed first time
    float         m_newValueFloat;
    uint8_t       m_precision;  // number of decimal places when converting float to string
    uint8_t       m_textSize; // multiplier of text size
    int           m_X, m_Y; // This is the cursor reference position (for example, the center of the string if using Center Justify)
    int           m_X1, m_Y1;  // Used for position of fillRect to write over last string printed
    const         GFXfont *m_Font;
    uint8_t       m_Just; // Specifies the text Justification (position relative to m_X, m_Y, as Right, Left, or Center)
    int           m_LLX, m_LLY; //corrected cursor position for the start of the string - Proportional Fonts use Lower Left for origin of string
    unsigned int  m_Twidth, m_Theight;
}; // don't forget the semicolon at the end of the class

template <size_t arraysize>
void LCDfield<arraysize>::SetNewValueChar(const char* newValue) {
  strncpy(m_newValueChar, newValue, arraysize);
}

template <size_t arraysize>
void LCDfield<arraysize>::SetNewValueFloat(float newValue) {
  m_newValueFloat = newValue;
}

template <size_t arraysize>
const char * LCDfield<arraysize>::GetNewValueChar() {
 return(m_newValueChar);
}

template <size_t arraysize>
float LCDfield<arraysize>::GetNewValueFloat() {
 return(m_newValueFloat);
}

template <size_t arraysize>
bool LCDfield<arraysize>::CheckForChanges() {
 if(m_newValueFloat != m_oldValueFloat) {
   m_oldValueFloat = m_newValueFloat;
   return(true);
 }
 else
 return(false);
}

template <size_t arraysize>
void LCDfield<arraysize>::MoveFloat2Char() {
  extern char *float2string(float v, uint8_t decimals);
  if(m_precision <= 0) {
    sprintf(m_newValueChar,"%d",int(m_newValueFloat));  // print withou decimal point
  }
  else {
  strncpy(m_newValueChar, (float2string(m_newValueFloat, m_precision)), arraysize);
  }
}

template <size_t arraysize>
void LCDfield<arraysize>::UpdateDisplayField(void) {

  tft.setTextSize(m_textSize);
  // calculate size and position  newValue data will have on the display
  tft.setFont(m_Font);

  // Draw over existing text by using values previously calculated
  tft.fillRect(m_X1, m_Y1, m_Twidth, m_Theight, Screen2FillColor);  
  
  // calculate the size of the new text
  // by using 0,0 for first two variables, we just measure the width and height
  tft.getTextBounds(m_newValueChar, 0, 0, &m_X1, &m_Y1, &m_Twidth, &m_Theight); 

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
  tft.getTextBounds(m_newValueChar, m_LLX, m_LLY, &m_X1, &m_Y1, &m_Twidth, &m_Theight); 

  //Now display
  tft.setCursor(m_LLX, m_LLY);
  tft.println(m_newValueChar);

  tft.setTextSize(1); // reset the text size
}//Update()

//LCDfield Screen 1 Objects                                                   (Optional)      
//construct instances     column,           row,               Font          Justification
LCDfield <20> fS1L1 {     160,              40,               &FreeSans12pt7b,  CJ  };
LCDfield <20> fS1L2 {     160,              80,               &FreeSans12pt7b,  CJ  };
LCDfield <20> fS1L3 {     160,              120,              &FreeSans12pt7b,  CJ  };
LCDfield <20> fS1L4 {     160,              160,              &FreeSans12pt7b,  CJ  };

//List of Screen1 objects
IScreens* const Screen1Objects[] = {&fS1L1, &fS1L2, &fS1L3, &fS1L4};


//LCDfield Screen 2 Objects                                             (Optional)      (Optional)   (Optional) 
//                                                                      (default LJ)    (default=1)  (default=0))
//construct instances   column,           row,               Font       Justification   TextSize     Precision
LCDfield <5>  fBV  {  screen2ColNum,    screen2RowVolt,   &SansSerifBIdigits84,   CJ  ,         1,       2 };
LCDfield <3>  fAA  {  screen2ColNum,    screen2RowAmp,    &SansSerifBIdigits84,   CJ  ,         1,       0 };

//List of Screen2 objects - does not include rev code field or fault code field 
// the order of these must follow the order of the FieldNumber enum
//IScreens* const Screen2Objects[] = {&fAV, &fBV, &fAA, &fBA, &fAT, &fBT, &fPW, &fCD, &fCS, &fSB};
IScreens* const Screen2Objects[] = {&fBV, &fAA};
#endif //_LCDFIELD_H_

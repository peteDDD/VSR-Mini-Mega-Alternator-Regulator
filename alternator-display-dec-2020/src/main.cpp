//  Alternator Amps and Voltage Display
//
//  12/14/20
//


#include "main.h"

void setup(void)
{
  #ifdef DIAGS_PRINTOUTS
    Serial.begin(9600);
  #endif

  tft.reset();
  tft.begin(LCD_DRIVER);  
  tft.setRotation(3);  //was 3

  ina.begin(ina226_address);
  // Configure INA226
  ina.configure(INA226_AVERAGES_64, INA226_BUS_CONV_TIME_4156US, INA226_SHUNT_CONV_TIME_4156US, INA226_MODE_SHUNT_BUS_CONT);
  // Calibrate INA226. // rShunt 200Amp 50mv = 0.00025 ohm, Max excepted current = 200A
  ina.calibrate(0.00025, 200);

  DrawScreen1();
  screenState = Screen1New;

  startTime = millis();
  sampleTime = startTime;
  
} //void setup(void)



void loop(void)
{
   currentTime = millis();

  if((currentTime-sampleTime) > SampleInterval) {
    getNewData();
    checkForChanges();
    sampleTime = millis();
  }

  //State Machine manages screens
  switch (screenState) {
    case Screen1New:
      /*  Can do something while waiting for the first screen to timeout.  
          Here is an example...
      if ((currentTime - startTime) > Screen1DelayForRevCode) {
        tft.setTextColor(Screen1TextColor);
        fCR.UpdateDisplayField(); //dislay the alt controller rev code
        
      }
      */
     screenState = Screen1Active;
      
    case Screen1Active:
      if ((currentTime - startTime) > Screen1Duration) { // when start screen is complete, set up to load Screen2
        UpdateGraphDataArray();  //put first set of data into GraphDataArray
        screenState = Screen2New;
        startTime = currentTime;  //startTime is reused in managing UpdateGraphDataArray timing
      }
      break;

    case Screen2New:
      RefreshScreen2();
      screenState = Screen2Active;
      screenToggleStartTime = currentTime;  // reset timer for switching screens
      break;

    case Screen2Active:
      if ((currentTime - startTime) > xAxisTimeInterval) UpdateGraphDataArray();
      break;

    case Screen3New:
      RefreshScreen3();  // if drawing first time, xAxisTime = 0 so no data drawn, just grid and labels
      screenState = Screen3Active;
      screenToggleStartTime = currentTime;  // reset timer for switching screens
      break;

    case Screen3Active:
      if ((currentTime - startTime) > xAxisTimeInterval) UpdateGraphDataArray();
      break;

  } //switch (screenState)
  if ((currentTime - screenToggleStartTime) > ScreenToggleInterval) ToggleScreens();

  } // loop

void getNewData(void) {
  #ifdef LOWVOLTAGETEST
   fBV.SetNewValueFloat(roundoff((2.8 * ina.readBusVoltage()),2));
  #else
   fBV.SetNewValueFloat(roundoff(ina.readBusVoltage(),2));
  #endif
  fAA.SetNewValueFloat(roundoff(ina.readShuntCurrent(),0));
  #ifdef DIAGS_PRINTOUTS
   Serial.print("measuredBatVolts = "); Serial.println(fBV.GetNewValueFloat());
   Serial.print("measuredAltAmps = "); Serial.println(fAA.GetNewValueFloat());
  #endif
}

void checkForChanges(void) {
   for (size_t i = 0; i < sizeof Screen2Objects / sizeof Screen2Objects[0]; i++) {
    if(Screen2Objects[i]->CheckForChanges()) {  // If the incoming data has changed, 
      Screen2Objects[i]->MoveFloat2Char();      // Move the new data into m_newValueChar_
      if(screenState == Screen2Active){
        Screen2Objects[i]->UpdateDisplayField(); // And, if displaying screen2, update that data on the screen
      }
   }  
  }
}

void DrawScreen1(void) {
  tft.fillScreen(Screen1FillColor);
  tft.setTextColor(Screen1TextColor);
  fS1L1.SetNewValueChar("Regina Oceani");
  fS1L2.SetNewValueChar("Alternator Display");
  fS1L3.SetNewValueChar("rev");
  fS1L4.SetNewValueChar(Revision);
  for (size_t i = 0; i < sizeof Screen1Objects / sizeof Screen1Objects[0]; i++) {
    Screen1Objects[i]->UpdateDisplayField();
  }
}//void DrawScreen1(void) {


void DrawStaticScreen2(void) {
  tft.fillScreen(Screen2FillColor);
  tft.setTextColor(Screen2TextColor);
  tft.setFont(Screen2Font);
  for (int i = 0; i < Screen2StaticLines; i++) {
    tft.setCursor(Screen2StaticData[i].column,  Screen2StaticData[i].row);
    tft.print(Screen2StaticData[i].text);
  }
}//DrawStaticScreen2(void)


void RefreshScreen2(void) {
  DrawStaticScreen2();
  for (size_t i = 0; i < sizeof Screen2Objects / sizeof Screen2Objects[0]; i++) {
    Screen2Objects[i]->UpdateDisplayField();  //refresh using existing m_newValue data
  }
}


void RefreshScreen3(void) {
  tft.fillScreen(Screen3FillColor);
  DrawYGrid();
  LabelAxes();
  for (uint8_t index = 0; index < xAxisTime; index++) {
    DrawGraphLines(index);
  }
}//void RefreshScreen3(void)


void DrawGraphLines(int Time) {
  int Height = map(Volts[Time], GraphLowerVoltageScaled, GraphUpperVoltageScaled, GridBottomY, GridYTop);
  DrawXGrid(); // draw X grid first so these lines stay below voltage bars
  //  fillRect(top left x,             top left y, width,            height,               color)
  tft.fillRect(Time + GridLeftX, Height,    GraphVoltBarWidth, DisplayMaxY - Height, VoltColor);
  DrawYGrid(); // draw Y grid last so these lines are in front of voltage bars
  

  //now calc and update Amps line
  Height = map(Amps[Time], GraphLowerAmpValue, GraphUpperAmpValue, GridBottomY, GridYTop);
  tft.drawPixel(Time + GridLeftX, Height - GraphAmpLineMarginHeight, Screen3FillColor); // create white margin below Amps line
  tft.fillRect(Time + GridLeftX, Height, GraphAmpLineWidth, GraphAmpLineHeight, AmpColor);
  tft.drawPixel(Time + GridLeftX, Height + GraphAmpLineHeight + GraphAmpLineMarginHeight, Screen3FillColor); // create white margin above Amps line
}//DrawGraphLines


void DrawYGrid(void) { //draw horizontal grid lines
  for (int Y = 0; (Y * GridYStep) < DisplayMaxY; Y++ ) {
    tft.drawLine(GridLeftX, GridBottomY - Y * GridYStep, GridRightX, GridBottomY - Y * GridYStep, Screen3GridLineColor); //x-axis
  }
}//void DrawYGrid(void) 

void DrawXGrid(void) { //draw vertical grid lines}
  int localColor;
  for (int X = 1; GridLeftX + (((X+1) * GridXStep)) < GridRightX; X++ ){  // start at 1, so 15 minutes
    if((X % 2) == 0) localColor = Screen3GridLineColor2;
       else localColor = Screen3GridLineColor;
    tft.drawLine(GridLeftX + (GridXStep * X), GridBottomY, GridLeftX + (GridXStep * X), UpperY, localColor);
  }
}//void DrawXGrid(void)

void LabelAxes(void) {
  tft.setFont(Screen3Font);
  for (int Y = 0; (Y * GridYStep) < (GridRightX-GridLeftX); Y++ ) {
    tft.setCursor(RightX - LabelXOffset, BottomY - LabelYOffset - (Y * GridYStep));
    tft.setTextColor(VoltColor);
    tft.println(GraphLowerVoltageValue + (GraphVoltageStep * Y), 1);

    tft.setCursor(LeftX, BottomY - LabelYOffset - (Y * GridYStep));
    tft.setTextColor(AmpColor);
    tft.println(GraphLowerAmpValue + (GraphAmpStep * Y));
  }
} //LabelAxes


void UpdateGraphDataArray(void) {
  //Puts current data into Volt and Amps arrays,
  //If currently displaying Screen3 (the graph), draw the new data
  if (xAxisTime < (GridRightX - GridLeftX)) {  // Stop if we are on the right edge of the graph
    Volts[xAxisTime] = uint8_t(10 * (atof(fBV.GetNewValueChar())));
    Amps[xAxisTime++] = uint8_t(atoi(fAA.GetNewValueChar()));
    #ifdef DIAGS_PRINTOUTS
    Serial.print("Amps[xAxisTime] = "); Serial.println(Amps[xAxisTime-1]);
    #endif
    if (screenState == Screen3Active) {  // if currently displaying graph, draw next bar
      DrawGraphLines(xAxisTime - 1);
    }
  }//if (xAxisTime < (GridBottomRightX ...
  startTime = currentTime;
}//void UpdateGraphDataArray(void)


void ToggleScreens(void) {
  if (screenState == Screen3Active) {
    screenState = Screen2New;
    return;
  }
  if (screenState == Screen2Active) screenState = Screen3New;
} //void ToggleScreens(void)

char *float2string(float v, uint8_t decimals) {
// Utility for converting floats to strings with defined number of decimal places
    const int OUTPUT_BUFS = 1; //7;                              // maximum number of floats in a single sprintf
    const int MAX_OUTPUT = 6; //13;                              // largest possible output string
    float absV;
    static char outputBuffers[OUTPUT_BUFS][MAX_OUTPUT+1];
    static uint8_t callCount; 
    char   formatter[] = "%d.%0_d";

   
    char *pos = outputBuffers[++callCount % OUTPUT_BUFS];   // Select buffer to use this time (Round-robben)
    //__ return (dtostrf(v,MAX_OUTPUT , decimals, pos));    //<<-- This works GREAT, but is about 1.2K in code size larger then what is here.

    formatter[5] = decimals+'0';                            // create the number for the # of decimal characters we ultimatly want.
    
    unsigned int mult = 1;                                  // Calc multplier for fractional portion of number
    uint8_t multleft = decimals;
    while(multleft--) {
      mult *= 10;
    }
    
    absV = v;
    if (absV<0) 
        absV *= -1;
 
    snprintf(pos, MAX_OUTPUT, formatter,
             (int)v,                                        // Whole number, with its sign
             (unsigned int) ((absV * mult)) % mult);      // Fraction, w/o a sign.
 
    return pos;

}       
   
float roundoff(float num, int precision)
// Utility to round floats to determined precision
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


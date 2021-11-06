//  Remote Serial Display for Alternator Controller rev 10c
//
//  works with SmartRegulator rev 1.3.1.12
//


#include "Arduino.h"
#include "main.h"


void setup(void)
{
  Serial.begin(115200);

  tft.reset();
  tft.begin(LCD_DRIVER);  
  tft.setRotation(3);

  DrawScreen1();
  screenState = Screen1New;

  startTime = millis();
} //void setup(void)


void loop(void)
{
  if (getSerialString()) {
    if (ParseString()) {  // unpack string and SetNewValue
      if (screenState == Screen2Active) { //Update Screen2 if being shown
        Screen2Objects[objectNumber]->UpdateDisplayField();
      }
    }
  } //if (getSerialString()) {

  //State Machine manages screens
  currentTime = millis();
  switch (screenState) {
    case Screen1New:
      if ((currentTime - startTime) > Screen1DelayForRevCode) {
        tft.setTextColor(Screen1TextColor);
        fCR.UpdateDisplayField(); //dislay the alt controller rev code
        screenState = Screen1Active;
      }
      
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

bool getSerialString() {
  static byte dataBufferIndex = 0;
  static bool storeString;
 
  while (Serial.available() > 0) {
    char incomingbyte = Serial.read();
    if (incomingbyte == startChar) {
      dataBufferIndex = 0;  //Initialize our dataBufferIndex variable
      storeString = true;
    }
    if (storeString) {
      //Let's check our index here, and abort if we're outside our buffer size
      //We use our define here so our buffer size can be easily modified
      if (dataBufferIndex == DATABUFFERSIZE) {
        //Oops, our index is pointing to an array element outside our buffer.
        dataBufferIndex = 0;
        break;
      }
      if (incomingbyte == endChar) {
        storeString = false;
        dataBuffer[dataBufferIndex] = 0; //null terminate the C string
        //Our data string is complete.
        return (true);
      }
      else {
        dataBuffer[dataBufferIndex++] = incomingbyte;
        dataBuffer[dataBufferIndex] = 0; //null terminate the C string
      }
    //}
   // else {
      
    }  
    //fall out the bottom with flag_StringReady still false;
  }//while (Serial.available() > 0) {
  return (false);
}//getSerialString()


bool ParseString() {
  char* valPosition;
  uint8_t i = 0;
  //This initializes strtok with our string to tokenize
  valPosition = strtok(dataBuffer, delimiters);

  while (valPosition != NULL) {
    if (i == 0) {
      if (strcmp(valPosition, DISPLAYSTRINGHEADER) != 0) {
        valPosition = NULL;
        return false;  //abort because this is not the string we are looking for
      }
    }
    if (i == 1) {
      objectNumber = atoi(valPosition);
    }
    if (i++ == 2) {
     //stringField = valPosition;
      // put any incoming data into correct object m_NewValue
      if (objectNumber == fnCR) { // alt controller rev code
       // fCR.SetNewValue(stringField);
       fCR.SetNewValue(valPosition);
      }
      else
        Screen2Objects[objectNumber]->SetNewValue(valPosition);
    }

    //Here we pass in a NULL value, which tells strtok to continue working with the previous string
    valPosition = strtok(NULL, delimiters);
  }//while
  return true; //complete parsing
}//ParseString()


void DrawScreen1(void) {
  tft.fillScreen(Screen1FillColor);
  tft.setTextColor(Screen1TextColor);
  fS1L1.SetNewValue("Regina Oceani");
  fS1L2.SetNewValue("Genset Alternator");
  fS1L3.SetNewValue("Controller");
  fS1L4.SetNewValue(Revision);
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
  int Height = map(Volts[Time], GraphLowerVoltage, GraphUpperVoltage, GridBottomY, GridYZero);
  //  fillRect(top left x,             top left y, width, height,       color)
  tft.fillRect(Time + GridBottomLeftX, Height,       1,   240 - Height, VoltColor);
  DrawYGrid();
  DrawXGrid();

  //now calc and update Amps line
  Height = map(Amps[Time], 0, 125, 228, 28);
  tft.drawPixel(Time + GridBottomLeftX, Height - 1, Screen3FillColor); // create white margin on Amps line
  tft.fillRect(Time + GridBottomLeftX, Height, 1, 4, AmpColor);
  tft.drawPixel(Time + GridBottomLeftX, Height + 5, Screen3FillColor);
}


void DrawYGrid(void) { //draw horizontal grid lines
  for (int Y = 0; (Y * YGridStep) < DisplayMaxY; Y++ ) {
    tft.drawLine(GridBottomLeftX, GridBottomY - Y * YGridStep, GridBottomRightX, GridBottomY - Y * YGridStep, Screen3GridLineColor); //x-axis
  }
}//DrawGrid

void DrawXGrid(void) { //draw vertical grid lines}
  for (int X = 1; X < 8; X++ ){  // start at 1, so 15 minutes
    tft.drawLine(GridBottomLeftX + XGridStep * X, GridBottomY, GridBottomLeftX + XGridStep * X, UpperLeftY, Screen3GridLineColor);
  }
}

void LabelAxes(void) {
  // Serial.println("Label Axes");
  tft.setFont(Screen3Font);
  for (int Y = 0; (Y * YGridStep) < DisplayMaxY; Y++ ) {
    tft.setCursor(BottomRightX - LabelXOffset, BottomY - LabelYOffset - (Y * YGridStep));
    tft.setTextColor(VoltColor);
    tft.println(GraphLowerVoltageValue + (GraphVoltageStep * Y), 1);

    tft.setCursor(BottomLeftX, BottomY - LabelYOffset - (Y * YGridStep));
    tft.setTextColor(AmpColor);
    tft.println(GraphLowerAmpValue + (GraphAmpStep * Y));
  }
} //LabelAxes


void UpdateGraphDataArray(void) {
  //Puts current data into Volt and Amps arrays,
  //If currently displaying Screen3 (the graph), draw the new data
  if (xAxisTime < (GridBottomRightX - GridBottomLeftX)) {  // Stop if we are on the right edge of the graph
    Volts[xAxisTime] = uint8_t(10 * (atof(fBV.GetNewValue())));
    Amps[xAxisTime++] = uint8_t(atoi(fAA.GetNewValue()));
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




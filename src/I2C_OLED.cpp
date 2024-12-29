//      I2C_OLED.cpp                                                           http://smartdcgenerator.blogspot.com/
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

#include <Arduino.h>
#include "I2C_OLED.h"
#include "Config.h"


SSD1306AsciiWire oled;

extern const char *chargingStateString;
extern int inChargingStateCount;

template <> // forced write to the field.  No checking for changed field
void LCDfield<char *>::Write(char *newValue)
{
    oled.clearField(m_column, m_row, m_eraseWidth);
    oled.print(newValue);
    oled.println(m_Format);
#ifdef USE_SERIAL_DISPLAY
    char buffer[40];
    sprintf(buffer, "$D:%d,%s%c",
            m_ObjectNumber,
            newValue, m_Format);
    SERIAL_DISPLAY_PORT.println(buffer);
#endif
}

template <> // overload function for const char* strings
void LCDfield<const char *>::Update(const char *newValue)
{
    if (newValue != m_lastValue)
    {
        m_lastValue = newValue;
        oled.clearField(m_column, m_row, m_eraseWidth);
        oled.print(newValue);
        oled.println(m_Format);
#ifdef USE_SERIAL_DISPLAY
        char buffer[40];
        sprintf(buffer, "$D:%d,%s%c",
                m_ObjectNumber,
                newValue, m_Format);
        SERIAL_DISPLAY_PORT.println(buffer);
#endif
    }
}

template <> // catches floats
void LCDfield<float>::Update(float newValue)
{
    newValue = roundoff(newValue, 2); // set precision to two decimal places
    if (newValue != m_lastValue)
    {
        m_lastValue = newValue;
        oled.clearField(m_column, m_row, m_eraseWidth);
        oled.print(newValue, m_Digits); // this funtion call format will not work with const char* or String
        oled.println(m_Format);
#ifdef USE_SERIAL_DISPLAY
        char buffer[40];
        extern char *float2string(float v, uint8_t decimals);
        sprintf(buffer, "$D:%d,%s%c",
                m_ObjectNumber,
                float2string(newValue, m_Digits), m_Format);
        SERIAL_DISPLAY_PORT.println(buffer);
#endif
    }
}

//                      (optional)
// construct instances  LastValue), ObjectNumber, column,           row,              EraseWidth,      Digits,  Format
// construct instances
LCDfield<float> LCDaltVolts(fnAV, screen2ColAlt, screen2RowVolt, screen2EraseVolt, 2, screen2FormatVolt);
LCDfield<float> LCDbatVolts(fnBV, screen2ColBat, screen2RowVolt, screen2EraseVolt, 2, screen2FormatVolt);
LCDfield<int> LCDaltAmps(fnAA, screen2ColAlt, screen2RowAmp, screen2EraseAmp, 0, screen2FormatAmp);
LCDfield<int> LCDbatAmps(fnBA, screen2ColBat, screen2RowAmp, screen2EraseAmp, 0, screen2FormatAmp);
LCDfield<int> LCDaltTemp(fnAT, screen2ColAlt, screen2RowTemp, screen2EraseTemp, 0, screen2FormatTemp);
LCDfield<int> LCDbatTemp(fnBT, screen2ColBat, screen2RowTemp, screen2EraseTemp, 0, screen2FormatTemp);
// pre-load lastValue for PWM so 0% is displayed during ramping state
LCDfield<int> LCDPWM(0, fnPW, screen2ColPWM, screen2RowPWM, screen2ErasePWM, 0, screen2FormatPWM);
LCDfield<const char *> LCDState(fnCS, screen2ColState, screen2RowState, screen2EraseState, 0, screen2FormatState);
LCDfield<int> LCDCount(fnCD, screen2ColCount, screen2RowCount, screen2EraseCount, 0, screen2FormatCount);
// LCDfield <int>    LCDCount2(          fnCD,       screen2ColCount2, screen2RowCount,  screen2EraseCount2, 0, screen2FormatCount);
LCDfield<char *> LCDCount2(fnTM, screen2ColCount2, screen2RowCount, screen2EraseCount2, 0, screen2FormatCount);
// LCDfield <const char *> LCDCountString(fnCD,      screen2ColCount3, screen2RowCount,  screen2EraseCount2, 0, screen2FormatCount);
LCDfield<const char *> LCDScuba(fnSB, screen2ColScuba, screen2RowPWM, screen2EraseScuba, 0, screen2FormatScuba);

// Utility to round floating number precision to a specified number of decimal places
float roundoff(float num, int precision)
{
    int temp = (int)(num * pow(10, precision));
    int num1 = num * pow(10, precision + 1);
    temp *= 10;
    temp += 5;
    if (num1 >= temp)
        num1 += 10;
    num1 /= 10;
    num1 *= 10;
    num = num1 / pow(10, precision + 1);
    return num;
} // float roundoff(float num,int precision)

void OLEDPrintlnCentered(const char *text, uint8_t row)
{
    size_t size = oled.strWidth(text);
    oled.setCursor((oled.displayWidth() - size) / 2, row);
    oled.println(text);
}

String BuildOutLampFeatures(const char *FeatureInString)
{
    String result = FeatureInString;
#if OUT_LAMP_MIRROR_EQUALIZE == true
    {
        result += "/EQ";
    }
#endif
#if OUT_LAMP_MIRROR_RESETTING == true
    {
        result += "/RESET";
    }
#endif
#if OUT_LAMP_MIRROR_FAULT == true
    {
        result += "/FAULT";
    }
#endif
    return result;
}

void OLEDprintWrappedText(const String &text1, const String &text2)
{
    // function to combine two strings and print them on the OLED in a wrapped text
    String line = "";
    String fullText = text1 + text2;
    bool firstLine = true;

    for (size_t i = 0; i < fullText.length(); i++)
    {
        if (oled.strWidth(line.c_str()) > OLED_PX_WIDTH)
        {
            oled.println(line);
            line = "";
            if (firstLine)
            {
                line += ' '; // Indent the second line by one character
                firstLine = false;
            }
        }
        line += fullText[i];
    }
    if (line.length() > 0)
    {
        oled.println(line);
    }
    oled.setCursor(0, oled.row()); // Move back to the beginning of the row
}

void OLEDprintWrappedTextBreakAtSpace(const String &text1, const String &text2)
{
    // function to combine two strings and print them on the OLED in a wrapped text
    // but break only at spaces
    String line = "";
    String fullText = text1 + text2;
    bool firstLine = true;
    size_t lastBlank = 0;

    for (size_t i = 0; i < fullText.length(); i++)
    {
        line += fullText[i];
        if (fullText[i] == ' ')
        {
            lastBlank = i;
        }

        if (oled.strWidth(line.c_str()) > OLED_PX_WIDTH)
        {
            if (lastBlank > 0)
            {
                line = line.substring(0, lastBlank);
                i = lastBlank; // Move the index to the last blank character
                lastBlank = 0; // Reset lastBlank
            }
            oled.println(line);
            line = "";
            if (firstLine)
            {
                line += ' '; // Indent the second line by one character
                firstLine = false;
            }
        }
    }
    if (line.length() > 0)
    {
        oled.println(line);
    }
    oled.setCursor(0, oled.row()); // Move back to the beginning of the row
}

void WriteOLEDFeatureAssignments(void)
{
    /*
    const char *FeatureIn1String = "None";
    const char *FeatureIn2String = "None";
    const char *FeatureIn3String = "None";

    const char *FeatureOut1String = "None";
    const char *FeatureOut2String = "None";
    const char *FeatureOut3String = "None";
*/

    String FeatureIn1String = "None";
    String FeatureIn2String = "None";
    String FeatureIn3String = "None";

    String FeatureOut1String = "None";
    String FeatureOut2String = "None";
    String FeatureOut3String = "None";

#ifdef FEATURE_IN_EQUALIZE
    {
        switch (FEATURE_IN_EQUALIZE_PORT)
        {
        case FEATURE_IN_PORT1:
            FeatureIn1String = "EQUALIZE";
            break;
        case FEATURE_IN_PORT2:
            FeatureIn2String = "EQUALIZE";
            break;
        case FEATURE_IN_PORT3:
            FeatureIn3String = "EQUALIZE";
            break;
        }
    }
#endif

#ifdef ENABLE_FEATURE_IN_SCUBA
    {
        switch (ENABLE_FEATURE_IN_SCUBA_PORT)
        {
        case FEATURE_IN_PORT1:
            FeatureIn1String = "SCUBA";
            break;
        case FEATURE_IN_PORT2:
            FeatureIn2String = "SCUBA";
            break;
        case FEATURE_IN_PORT3:
            FeatureIn3String = "SCUBA";
            break;
        }
    }
#endif

#ifdef FEATURE_IN_FORCE_TO_FLOAT
    {
        switch (FEATURE_IN_FORCE_TO_FLOAT_PORT)
        {
        case FEATURE_IN_PORT1:
            FeatureIn1String = "Force to Float";
            break;
        case FEATURE_IN_PORT2:
            FeatureIn2String = "Force to Float";
            break;
        case FEATURE_IN_PORT3:
            FeatureIn3String = "Force to Float";
            break;
        }
    }
#endif

#ifdef FEATURE_OUT_GREEN_LED
    {
        switch (FEATURE_OUT_GREEN_LED_PORT)
        {
        case FEATURE_OUT_PORT1:
            FeatureOut1String = "Green LED";
            break;
        case FEATURE_OUT_PORT2:
            FeatureOut2String = "Green LED";
            break;
        case FEATURE_OUT_PORT3:
            FeatureOut3String = "Green LED";
            break;
        }
    }
#endif

#ifdef FEATURE_OUT_RED_LED
    {
        switch (FEATURE_OUT_RED_LED_PORT)
        {
        case FEATURE_OUT_PORT1:
            FeatureOut1String = "Red LED";
            break;
        case FEATURE_OUT_PORT2:
            FeatureOut2String = "Red LED";
            break;
        case FEATURE_OUT_PORT3:
            FeatureOut3String = "Red LED";
            break;
        }
    }
#endif

#ifdef FEATURE_OUT_LAMP
    {
        switch (FEATURE_OUT_LAMP_PORT)
        {
        case FEATURE_OUT_PORT1:
            FeatureOut1String = "Lamp";
            FeatureOut1String = BuildOutLampFeatures(FeatureOut1String);
            break;
        case FEATURE_OUT_PORT2:
            FeatureOut2String = "Lamp";
            BuildOutLampFeatures(FeatureOut2String);
            break;
        case FEATURE_OUT_PORT3:
            FeatureOut3String = "Lamp";
            FeatureOut3String = BuildOutLampFeatures(FeatureOut3String);
            break;
        }
    }
#endif

#ifdef FEATURE_OUT_ENGINE_STOP
    {
        switch (FEATURE_OUT_ENGINE_STOP_PORT)
        {
        case FEATURE_OUT_PORT1:
            FeatureOut1String = "Engine Stop" + "/" + ENGINE_STOP_PULSE_DURATION;
            break;
        case FEATURE_OUT_PORT2:
            FeatureOut2String = "Engine Stop" + "/" + ENGINE_STOP_PULSE_DURATION;
            break;
        case FEATURE_OUT_PORT3:
            FeatureOut3String = "Engine Stop" + "/" + ENGINE_STOP_PULSE_DURATION;
            break;
        }
    }
#endif

#ifdef FEATURE_OUT_LIFEPO_SHUTDOWN_ALARM
    {
        switch (FEATURE_OUT_LIFEPO_SHUTDOWN_ALARM_PORT)
        {
        case FEATURE_OUT_PORT1:
            FeatureOut1String = "LifePO Shutdown Alarm";
            break;
        case FEATURE_OUT_PORT2:
            FeatureOut2String = "LifePO Shutdown Alarm";
            break;
        case FEATURE_OUT_PORT3:
            FeatureOut3String = "LifePO Shutdown Alarm";
            break;
        }
    }
#endif

#ifdef FEATURE_OUT_COMBINER
    {
        String AcceptCarryover = String(COMBINE_ACCEPT_CARRYOVER / 3600000UL);

        // Build Combiner String
        String combinerString = String("Combiner ") + String(COMBINE_CUTIN_VOLTS) + "/" + String(COMBINE_HOLD_VOLTS) + "/" + String(COMBINE_CUTOUT_VOLTS);
        Serial.println("COMBINER STRING = " + combinerString);

        switch (FEATURE_OUT_COMBINER_PORT)
        {
        case FEATURE_OUT_PORT1:
            FeatureOut1String = combinerString;
            break;
        case FEATURE_OUT_PORT2:
            FeatureOut2String = combinerString;
            break;
        case FEATURE_OUT_PORT3:
            FeatureOut3String = combinerString;
            break;
        }
    }
#endif
    oled.clear();
    oled.setFont(Callibri15);
    oled.setCursor(0, 0);
    OLEDprintWrappedTextBreakAtSpace("FIn1: ", FeatureIn1String);
    OLEDprintWrappedTextBreakAtSpace("FIn2: ", FeatureIn2String);
    OLEDprintWrappedTextBreakAtSpace("FIn3: ", FeatureIn3String);
    delay(TIME_BETWEEN_OLED_SCREENS);

    oled.clear();
    oled.setCursor(0, 0);
    OLEDprintWrappedTextBreakAtSpace("FOut1: ", FeatureOut1String);
    OLEDprintWrappedTextBreakAtSpace("FOut2: ", FeatureOut2String);
    OLEDprintWrappedTextBreakAtSpace("FOut3: ", FeatureOut3String);
    delay(TIME_BETWEEN_OLED_SCREENS);
}

void WriteOLEDSerialPortAssignments(void)
{
    String SerialPort1String = "None";
    String SerialPort2String = "None";

#ifdef USE_SERIAL_DISPLAY
{
    #if SERIAL_DISPLAY_PORT_NUM == 1
        SerialPort1String = "Display";
    #elif SERIAL_DISPLAY_PORT_NUM == 2
        SerialPort2String = "Display";
    #endif
}
#endif
#ifdef USE_BMS_SERIAL_IN
{
    #if BMS_SERIAL_PORT_NUM == 1
        SerialPort1String = "BMS";
    #elif BMS_SERIAL_PORT_NUM == 2
        SerialPort2String = "BMS";
    #endif
}
#endif

    oled.clear();
    oled.setFont(Callibri15);
    //oled.setFont(TimesNewRoman16);
    oled.setCursor(0, 0);
    oled.print("Serial Port 1: ");
    oled.println(SerialPort1String);
    oled.print("Serial Port 2: ");
    oled.println(SerialPort2String);
    delay(TIME_BETWEEN_OLED_SCREENS);
}
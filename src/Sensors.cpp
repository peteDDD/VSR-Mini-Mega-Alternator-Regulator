//      Sensors.cpp
//
//      Copyright (c) 2016, 2017 by William A. Thomason.      http://arduinoalternatorregulator.blogspot.com/
//
//      Refactored 2020/2021 derivative work (c) 2021 by Pete Dubler
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

#include "Config.h"
#include "Sensors.h"
#include "Alternator.h"
#include "Flash.h"
#include <Wire.h>

#include <I2Cx.h> // Newer I2C lib with improved reliability and error checking.
                  // Only use with AVR processors

#ifdef USE_OLED
  #include "I2C_OLED.h"
  char buffer2[20];
#else
  #include <SoftI2CMaster.h> // http://homepage.hispeed.ch/peterfleury/avr-software.html
#endif                     //USE_OLED   // DUBLER 061420 new URL https://github.com/felias-fogg/SoftI2CMaster 
                           // It is too bad there are so many of these with the same name.  Be sure to get the correct one
//----  Public  veriables
float measuredAltVolts = 0; // The regulator takes local measurements, and to start with we ASSUME these sensors are connected to the Alternator.  So, we locally measure
float measuredAltAmps = 0;  // ALTERNATOR voltage and current.
float measuredBatVolts = 0; // However, we need to regulate based on Battery voltage and current.  In stand-along installations, we have to assume that what we measure via
float measuredBatAmps = 0;  // the regulator is what we are to 'regulate' to, so the code will copy over the 'alternator' values into the 'battery' values.
// On the Mini Mega version, we have two INA226 voltage and current sensor chips on the I2C bus, one for the battery and one for the alternator.
// TODO Add config setting to not use the alternator INA226

bool shuntBatAmpsMeasured = false; 
bool shuntAltAmpsMeasured = false;
//  shuntBatAmpsMeasured and shuntAltAmpsMeasured are signals that we have ever been able to measure Amps via the respective shunt - used to disable amp based decisions in
//  manageAlt() when we are not even able to measure Amps....
int measuredAltWatts = 0;
int measuredFETTemp = -99;  // -99 indicated not present.  Temperature of Field FETs, in degrees C.
int measuredAltTemp = -99;  // -99 indicated not present.  -100 indicates user has shorted the Alt probe and we should run in half-power mode.
//int measuredAlt2Temp = -99; // -99 indicated not present.  Value derived from 2nd NTC port if battery temperature is delivered by an external source via the CAN.
int measuredBatTemp = -99;  // -99 indicates we have not measured this yet, or the sender has failed and we need to use Defaults.
// Battery Temperature typically will be measured by the 2nd NTC, the B-NTC port.  However, if the temperature is provided by an external source
//  (Specifically, via the CAN bus), then the 2nd NTC port will be considered a 2nd alternator sensor for alternator temperature regulation.
bool batTempExternal = false; // Have we received the battery temperature from an external source via the CAN?
// Defaults to false for compatibility with non-CAN enabled regulators.

int measuredFieldAmps = -99; // What is the current being delivered to the field?  -99 indicated we are not able to measure it.
bool updatingBatVAs = false; // Are we in the process of updating the Volts and Amps?  (Meaning, hold off doing anything critical until we get new data..)
bool updatingAltVAs = false; 
//----- Calibration buffer
//      Future releases may allow for calibration of individual boards.  Either by user, or by some external manufacturing process.
//      Would allow for use of lower-cost resistors in all the dividers.  This structure is saved in the FLASH of each device.
//      For now, will simple prime these with default values.
tCAL ADCCal = {false, // locked   == Let user do calibration if they wish to.
               1.0,   // .VBAT_GAIN_ERROR  == Error of VBat ADC + resister dividers
               1.0,   // .AMP_GAIN_ERROR
               0,     // .VBAT_OFFSET
               -522}; // TODO Do we need this???   .AMP_OFFSET == ADC Offset error of shunt circuit measured @ 0A

//****************************************************************************************************************************
//i += 522;                                 // VERY BAD!  Original PCB needed to add in a manual offset to accommodate a hardware design error with regard to
// Default offset is now contained          // how common-mode noise is divided by R22/R24, and causes -14.2A to be displayed when no current
// in this  'default' ADCCal structure      // is present in amp shunt.
// IF YOU DO BUILD OPTION OF NOT INSTALLING INA282 LEVEL SHIFTER, CHANGE DEFAULT TO = 0
//****************************************************************************************************************************

//----  Internal veriables and prototypes
uint32_t sensorsLastSampled; // Used in the main loop to force an sensor (INA226, NTC) sample cycle if alternator isn't running.

uint32_t accumulatedNTC_A = 0; // Accumulated RAW A/D readings from sample_ADCs();  Used to calculate temperatures after averaging NTC_AVERAGING A/D readings.
uint32_t accumulatedNTC_B = 0;
uint32_t accumulatedNTC_FET = 0;
uint8_t accumulatedADCSamples = 0; // How many NTC A/D samples have been accumulated?  Not that in actuality, we ping-pong between the two NTC sensors each 'sample' cycle, to
// allow for the Atmel A/D to settle after selecting a given A/D port.

uint32_t accumulateUpdated;  // Time the Last Run Accumulators were last updated.
uint32_t generatorLrStarted; // At what time (mills) did the Generator start producing power?
uint32_t generatorLrRunTime; // Accumulated time for the last Alternator run (in mills)
int32_t accumulatedASecs;    // Accumulated Amp-Seconds of current charge cycle.   This actually holds Amps @ ACCUMULATED_SAMPLING rate.  Need to divide to get true value.
int32_t accumulatedWSecs;    // Accumulated Watt-Seconds of current charge cycle.  This actually holds Watts @ ACCUMULATED_SAMPLING rate. Need to divide to get true value.

int16_t savedShuntRawADC; // Place holder for the last raw Shunt ADC reading during read_INA().  Used by calibrate_ADCs() to determine offset error of board

int normalizeNTCAverage(uint32_t accumalatedSample, int beta, bool hasRG);
int read_Bat_INA226(void);
int read_Alt_INA226(void);

void sample_ADCs_for_Temperatures(void);
void resolve_ADCs_for_Temperatures(void);
void calibrate_ADCs(void);

//------------------------------------------------------------------------------------------------------
//
//  Initalize Sensor
//
//      This function will configure the board for locally attached sensor readings.
//      It will also iniitate a sampling cycle of the local ADCs - for later polling and reading.
//
//
//------------------------------------------------------------------------------------------------------
bool initialize_sensors(void)
{ // ALSO INITIALIZE LCD HERE, below

  pinMode(NTC_ALT_PORT, INPUT); // And A/D input pins
  pinMode(NTC_BAT_PORT, INPUT);
  pinMode(NTC_FET_PORT, INPUT);

  // Startup the stand-alone regulators Vbat and Amps sensor.
  I2c.begin();              // new I2C library
  I2c.timeOut(I2C_TIMEOUT); // Enable fault timeouts on the I2C bus.
  I2c.pullup(0);            // Disable internal pull-ups, there are external pull-up resisters.

#ifdef USE_OLED
  Wire.begin();                             // wire.begin will also do an i2c_init since we are using the wire.h shell of I2CMaster
  oled.begin(&Adafruit128x64, LCD_ADDRESS); // start up the SDD1306 OLED
#else
  i2c_init();
#endif //USE_OLED

  sensorsLastSampled = millis(); // Prime all the loop counters;
  reset_run_summary();
  sample_ALT_and_BAT_VoltAmps(); // Let's get these guys doing a round of sampling for use to decide system voltage.

  return (true);
} //initialize_sensors

//------------------------------------------------------------------------------------------------------
// Calibrate ADCs
//
//      Self calibration attempts to calibrate most of the ADCs / DAC to the tolerance of the band-gap reference in the CPU.
//      User can select this mode during startup if they connect VBat to the +5v connector on the ICSP, and also short out
//      the amperage shunt.  Device will calibrate voltage and amperage offset values and save them to EEPROM.
//      Doing a Master Clear (via Feature-in at power-on, or by using $MSR:  ASCII command).
//      Calibration is based on self-measuring of Vcc using the Vcc/4 ADC channel internal to the CPU.
//
//------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------
//
//  read_sensors()
//
//      This function will gather information from all the sensors and update
//      global variables.
//
//      If there is a fatal error it will return FALSE
//         Currently, Faulting conditions are read errors on the INA226 sensor  (Volts and Amps)
//
//------------------------------------------------------------------------------------------------------
bool read_sensors(void)
{
  // Start a sample run if the INAs are ready and the stator just signaled,
  // OR too much time has elapsed and it seems
  // either the INA226 has stalled, or we are not receiving Stator interrupts.

  if ((statorIRQflag && !updatingBatVAs && !updatingAltVAs) || // Synchronized with stator
      ((millis() - sensorsLastSampled) >= SENSOR_SAMPLE_RATE))
  { // Or just go get them it is has been too long. . . .  (e.g., alt stopped)

    sample_ALT_and_BAT_VoltAmps();  // Start the INA226 sample cycle.
    sample_ADCs_for_Temperatures(); // Use the same pacing rate for the local NTC ADC sampling conversions.

    sensorsLastSampled = millis();
    statorIRQflag = false; // Clear the Stator Flag for next time.
  }

  resolve_ADCs_for_Temperatures();
  return (read_ALT_and_BAT_VoltAmps()); // here we update the actual values, having already started the read earlier, and return the status
} //read_sensors

//------------------------------------------------------------------------------------------------------
// Sample ALT Volts & Amps
//      This function will instruct the sensors used for local reading of Volts and Amps to begin a sample cycle.
//        Example:  instruct both of the INA226's to each begin a sample cycle (triggered mode).
//        As a result of the Config reg being written to -  each INA-226 will begin a new sample / averaging cycle.
//        (Note, this is also called during Setup to initially program the INA226s with its configuration values)
//
//------------------------------------------------------------------------------------------------------
bool sample_ALT_and_BAT_VoltAmps(void)
{
  uint8_t ptr[2];

  ptr[0] = highByte(INA226_CONFIG); // Config current & Vbat INA226
  ptr[1] = lowByte(INA226_CONFIG);
  I2c.write(INA226_Bat_I2C_ADDR, CONFIG_REG, ptr, 2); // Writing the Config reg also 'triggers' a INA226 sample cycle.
  updatingBatVAs = true; // Let the world know we are working on getting a new Battery Volts and Amps reading
  I2c.write(INA226_Alt_I2C_ADDR, CONFIG_REG, ptr, 2);
  updatingAltVAs = true; // Let the world know we are working on getting a new Alternator Volts and Amps reading

  return (true);
} //bool  read_sensors(void) {

//------------------------------------------------------------------------------------------------------
//
//  read_ALT_and_BAT_VoltAmps()
//
//      This function will read the Volts and Amps from both INA226 chips and update global alternator variables for Volts, Amps, and Watts.
//      After doing this, the function resolve_BAT_voltsAmps() should be called.
//
//      Much of the simulation code is here, enabled with the SIMULATION flag.  Scroll down to get to the real code!
//
//      If there is a fatal error it will set the appropriate chargingState and faultCode and return FALSE
//      Fatal error meaning we are not able to determine the battery voltage, and as such need to not be running the alternator.
//
//------------------------------------------------------------------------------------------------------
bool read_ALT_and_BAT_VoltAmps(void)
{
  unsigned u = read_Bat_INA226(); // Get Alt Volts, Alt Amps, and update global variables.
  if (u != 0)
  {
    chargingState = FAULTED;              // A non-zero return contains the I2C error code.
    faultCode = FC_BAT_INA226_READ_ERROR + u; // Add in I2C returned error code.
    return (false);                       // And loop back to allow fault handler to stop everything!
  }

//Now read the alternator INA226 values

  u = read_Alt_INA226(); // Get Bat Volts, Bat Shunt Amps, and update global variables.

  if (u != 0)
  {
    chargingState = FAULTED;                       // A non-zero return contains the I2C error code.
    faultCode = FC_ALT_INA226_READ_ERROR + u; // Add in I2C returned error code.
    return (false);                                // And loop back to allow fault handler to stop everything!
  }

  measuredAltWatts = (int)(measuredAltVolts * measuredAltAmps);

  if (measuredAltAmps >= USE_AMPS_THRESHOLD) // Set flag if it looks like we are able to read current via local shunt.
    shuntAltAmpsMeasured = true;
  if (measuredBatAmps >= USE_AMPS_THRESHOLD) // Set flag if it looks like we are able to read current via local shunt.
    shuntBatAmpsMeasured = true;

  return (true);
} //read_ALT_VoltAmps


//------------------------------------------------------------------------------------------------------
// Read INA-226
//      This function will check to see if the values in the INA-226 is ready to be read.
//      If so, this will read them and update the global variables for measured ALTERNATOR voltage and amps;
//      an external function needs to decide how these should be used with regards to BATTERY voltage and amps.
//      The Global Variable INA226_ready will also be set to TRUE to indicate they are ready for another
//      sampling cycle to begin.
//
//      Will return '0' if all is OK, else will return the I2C lib error code.
//
//------------------------------------------------------------------------------------------------------
int read_Bat_INA226(void)
{ //BATTERY VOLTS AND AMPS
  int16_t i;

  //--- Do we have anything to read?   Check the VBat+, Alternator AMPs sensor.
  if ((i = I2c.read(INA226_Bat_I2C_ADDR, STATUS_REG, 2)) != 0) // Read in the Status Register.
    return (i);                                                // If I2C read error (non zero return), skip the rest.

  i = I2c.receive() << 8;
  i |= I2c.receive();

  if (i & 0x0008)
  { // Conversion is completed!   Go get them!
    if ((i = I2c.read(INA226_Bat_I2C_ADDR, VOLTAGE_REG, 2)) != 0) // Check Valt
      return (i);  // If I2C read error (non zero return), return error and skip the rest.

    i = I2c.receive() << 8;
    i |= I2c.receive();

    measuredBatVolts = i * INA226_VOLTS_PER_BIT * ADCCal.VBAT_GAIN_ERROR; 

    if ((i = I2c.read(INA226_Bat_I2C_ADDR, SHUNT_V_REG, 2)) != 0) // Now read the Amps, read the raw shunt voltage.
      return (i);  // If I2C read error (non zero return), return error and skip the rest.

    i = I2c.receive() << 8;
    i |= I2c.receive();

    measuredBatAmps = (i - ADCCal.AMP_OFFSET) * INA226_VOLTS_PER_AMP * (float)systemConfig.BAT_AMP_SHUNT_RATIO;  
    if (systemConfig.REVERSED_BAT_SHUNT == true)
      measuredBatAmps *= -1.0; // If shunt is wired backwards, reverse measured value.

    updatingBatVAs = false; // All done, ready to do another synchronized sample session anytime.
  }
  return (0);
} //int read_Bat_INA226(void) {


int read_Alt_INA226(void)
{ // ALTERNATOR VOLTS AND AMPS
  int16_t i;

  //--- Do we have anything to read?   Check the VBat+, Alternator AMPs sensor.

  if ((i = I2c.read(INA226_Alt_I2C_ADDR, STATUS_REG, 2)) != 0) // Read in the Status Register.
    return (i);  // If I2C read error (non zero return), skip the rest.

  i = I2c.receive() << 8;
  i |= I2c.receive();

  if (i & 0x0008)
  {  // Conversion is completed!   Go get them!
    if ((i = I2c.read(INA226_Alt_I2C_ADDR, VOLTAGE_REG, 2)) != 0) // Check Valt
      return (i);  // If I2C read error (non zero return), return error and skip the rest.

    i = I2c.receive() << 8;
    i |= I2c.receive();

    measuredAltVolts = i * INA226_VOLTS_PER_BIT * ADCCal.VBAT_GAIN_ERROR; 

    if ((i = I2c.read(INA226_Alt_I2C_ADDR, SHUNT_V_REG, 2)) != 0) // Now read the Amps, read the raw shunt voltage.
      return (i);  // If I2C read error (non zero return), return error and skip the rest.

    i = I2c.receive() << 8;
    i |= I2c.receive();

    measuredAltAmps = (i - ADCCal.AMP_OFFSET) * INA226_VOLTS_PER_AMP * (float)systemConfig.ALT_AMP_SHUNT_RATIO; // Each bit = 2.5uV Shunt Voltage.  Adjust by Shunt ratio. 
    if (systemConfig.REVERSED_ALT_SHUNT == true)
      measuredAltAmps *= -1.0; // If shunt is wired backwards, reverse measured value.

    updatingAltVAs = false; // All done, ready to do another synchronized sample session anytime.
  }
  return (0);
} // END read_Alt_INA226

//------------------------------------------------------------------------------------------------------
// Sample NTC's
//      This function will sample the NTC's A/D ports for temperatures and update the accumulated A/D value.
//
//
//------------------------------------------------------------------------------------------------------

void sample_ADCs_for_Temperatures(void)
{

  switch (accumulatedADCSamples % 3)
  { // Which NTC port do we need to do?
  case 0:
    accumulatedNTC_A += (uint32_t)analogRead(NTC_ALT_PORT); // Read the 'A' (Alternator) port
    break;

  case 1:
    accumulatedNTC_B += (uint32_t)analogRead(NTC_BAT_PORT); // Read the 'B' ((Battery, or 2nd alternator) port
    break;

#ifdef NTC_FET_PORT
  case 2:
    accumulatedNTC_FET += (uint32_t)analogRead(NTC_FET_PORT); // Read the Field FET's NTC,
    break;
#endif

  default:
    break;
  }
  accumulatedADCSamples++;
} //void sample_ADCs(void) {

//------------------------------------------------------------------------------------------------------
// Resolve ADCs
//      This function will check to see if it is time to calculate the temps based on the accumulated NTC Samples.
//
//------------------------------------------------------------------------------------------------------

void resolve_ADCs_for_Temperatures(void)
{
  if ((accumulatedADCSamples < NTC_AVERAGING) || ((accumulatedADCSamples % 3) != 0))
    return; // Not ready to do calculation yet, or counter  in mid-cycle through the NTC ports (messes up average)!

  measuredAltTemp = normalizeNTCAverage(accumulatedNTC_A, NTC_BETA_ALT_AND_BAT, true); // Convert the A NTC sensor for the alternator.

  measuredBatTemp = normalizeNTCAverage(accumulatedNTC_B, NTC_BETA_ALT_AND_BAT, true); // Convert the A NTC sensor for the battery
    //measuredAlt2Temp = -99;
  

#ifdef NTC_FET_PORT
  measuredFETTemp = normalizeNTCAverage(accumulatedNTC_FET, NTC_BETA_FETs, false); // And also convert the FET sensor (Onboard FET NTC does not have a Ground Isolation Resistor)
#endif

  if ((measuredBatTemp > NTC_OUT_OF_RANGE_HIGH) || (measuredBatTemp < NTC_OUT_OF_RANGE_LOW))
    measuredBatTemp = -99; // Out of bound A/D reading, indicates something is wrong...
 // if ((measuredAlt2Temp > 120) || (measuredAlt2Temp < -40))
 //   measuredAlt2Temp = -99;

  if (measuredAltTemp > NTC_SHORTED)
    measuredAltTemp = -100; // If user has shorted out the Alt sensor -
  else                      // that indicates they want to run in 1/2 power mode
      if ((measuredAltTemp > NTC_OUT_OF_RANGE_HIGH) || (measuredAltTemp < NTC_OUT_OF_RANGE_LOW))
    measuredAltTemp = -99;

#ifdef NTC_FET_PORT
  if ((measuredFETTemp > NTC_OUT_OF_RANGE_HIGH) || (measuredFETTemp < NTC_OUT_OF_RANGE_LOW))
    measuredFETTemp = -99;
#endif

  accumulatedNTC_A = 0; // Reset the accumulators and counter
  accumulatedNTC_B = 0;
  accumulatedNTC_FET = 0;
  accumulatedADCSamples = 0;

} //void resolve_ADCs(void) {

int normalizeNTCAverage(uint32_t accumulatedSample, int beta, bool hasRG)
{ // Helper function, will convert the passed oversampled ADC value into a temperature
  float resistanceNTC;
  int adcNTC;

  adcNTC = (accumulatedSample / (accumulatedADCSamples / 3)); // Calculate NTC resistance
  if (adcNTC > 1000)   // There must not be any NTC probe attached to this port.
    return (-1000);    // Signal that by sending back a very very cold temp...
  resistanceNTC = (1023.0 / (float)adcNTC) - 1.0;
  resistanceNTC = (float)NTC_RF / resistanceNTC;

  if (hasRG)
    resistanceNTC -= (float)NTC_RG; // Adjust for the Ground Isolation Resistor, if this sensor has one.
  // Use Beta method for calculating dec-C.
  return ((int)(1 / (log(resistanceNTC / NTC_RO) / beta + 1 / (25.0 + 273.15)) - 273.15));

} //int normalizeNTCAverage(uint32_t accumulatedSample,...

//------------------------------------------------------------------------------------------------------
//
//  update_run_summary()
//
//      This function will update the global accumulate variables Ah and Wh, as well as Run Time.
//      Used to drive Last Run Summary display screen, and also provide values for exiting Float mode via Ahs.
//
//------------------------------------------------------------------------------------------------------
void update_run_summary(void)
{
  if ((millis() - accumulateUpdated) < ACCUMULATE_SAMPLING_RATE) // Update the runtime accumulators?
    return;                                                      //  Not yet.  (Do it every 1 second)

  accumulateUpdated = millis();

  if ((chargingState >= warm_up) && (chargingState <= equalize))
  { //  If the Alternator is running, update the last-run vars.
    generatorLrRunTime = millis() - generatorLrStarted;
    accumulatedASecs += measuredAltAmps;
    accumulatedWSecs += measuredAltWatts;
  }

} //void update_run_summary(void) {

//------------------------------------------------------------------------------------------------------
//
//  reset_run_summary()
//
//      This function will reset the global accumulate variables Ah and Wh, as well as Run Time.
//
//------------------------------------------------------------------------------------------------------
void reset_run_summary(void)
{ // Zero out the accumulated AHs counters as we start this new 'charge cycle'
  accumulateUpdated = millis();
  generatorLrStarted = millis(); // Reset the 'last ran' counters.
  generatorLrRunTime = 0;
  accumulatedASecs = 0;
  accumulatedWSecs = 0;
}

#ifdef USE_OLED

void WriteOLEDTitlePage(void)
{
  // output version/fork and then the static parts of the LCD display
  oled.clear();
  oled.setFont(Callibri15);
  oled.setCursor(10, 1);
  oled.println("VSR MINI MEGA");
  oled.setCursor(18, 3);
  oled.println(REV_FORK);
  oled.setCursor(30,6);
  oled.println(DATE_CODE);
  delay(2000);
}
  
void WriteOLEDBatteryType(void)
{
  oled.clear();
  oled.setFont(Callibri15);
  oled.setCursor(20, 0);
  oled.println("Battery Type");
  oled.setCursor(30, 2);
  oled.println(chargingParms.BATTERY_TYPE);

  oled.setCursor(20, 5);
  switch(int(systemAmpMult *2 )) // systemAmpMult scales to 500AH, so have to multiply by 2
                               //   to get integer steps for the four levels of bat bank size
  {
    case 1:
      oled.println("< 250AH");
      break;

    case 2:
      oled.println("250 - 500AH");
      break;
    
    case 3:
      oled.println("500AH - 750AH");
      break;

    case 4:
      oled.println("> 750AH");
      break;
  }
  
  delay(2000);
}

void WriteOLEDDIPSettings(void)
{
  oled.clear();
  oled.setFont(Callibri15);
  oled.setCursor(8, 1);
  oled.print("SMALL ALT:  ");
  if(smallAltMode) oled.println("YES");
  else oled.println("NO");
  
  oled.setCursor(8, 4);
  oled.print("TACH MODE:  ");
  if(tachMode) oled.println("ON");
  else oled.println("OFF");
  delay(2000);
}

void WriteOLEDFactoryReset(void)
{
  oled.clear();
  oled.setFont(Callibri15);
  oled.setCursor(30, 1);
  oled.println("FACTORY");
  oled.setCursor(30, 4);
  oled.println(" RESET");
  delay(2000);
}

void WriteOLEDResetting(void)
{
  oled.clear();
  oled.setFont(Callibri15);
  oled.setCursor(32, 1);
  oled.println("RESETTING");
  WriteOLEDFaultString();
}

void WriteOLEDNonResetFault(void)
{
  // output version/fork and then the static parts of the LCD display
  oled.clear();
  oled.setFont(Callibri15);
  oled.setCursor(15, 1);
  oled.println("NON-RESETTING");
  WriteOLEDFaultString();
}

void WriteOLEDFault(void)
{
  // output version/fork and then the static parts of the LCD display
  oled.clear();
  oled.setFont(Callibri15);
  //delay(300);
  //oled.clear();
  WriteOLEDFaultString();
}

void WriteOLEDFaultString(void)
{
  oled.setCursor(35, 3);
  oled.print("FAULT  ");
  oled.println(faultCode);
  oled.setCursor(25,5);
  switch(faultCode & 0x7FFFU) // mask out reset bit 0x8000
  {   // fault code list is in System.h
    case 12:
      oled.println("Battery Temp");
      break;

    case 13:
      oled.println("Battery Volts");
      break;
    
    case 14:
      oled.println("Bat Low Volts");
      break;

    case 21:
      oled.println("  Alt Temp   ");
      break;
        
    case 22:
      oled.println("  Alt RPMs   ");
      break;
    
    case 24:
      oled.println("Alt Temp Ramp");
      break;

    case 31:
      oled.println("UnSup Chrg St ");
      break;

    case 32:
      oled.println("UnSup Chrg St1");
      break;
    
    case 33:
      oled.println("UnSup CPIndex ");
      break;
    
    case 34:
      oled.println("UnSup CPIndex1");
      break;
    
    case 35:
      oled.println("UnSup CPI St  ");
      break;
    
    case 36:
      oled.println("UnSup CPI St1 ");
      break;
  
    case 41:
      oled.println("   FET Temp   ");
      break;
    
    case 42:
      oled.println("Missing Sensor");
      break;
    
    case 72:
      oled.println("ADC Read Error");
      break;
    
    case 100:
      oled.println(" I2C 1 Error  ");
      break;
    
    case 200:
      oled.println(" I2C 2 Error  ");
      break;

    default:
      oled.println("Unknown Error ");
      break;
  }

}

void WriteOLEDDataScreenStaticData(void)
{
  oled.clear();
  oled.setFont(Callibri15);
  oled.setCursor(3, 2);
  oled.println(chargingStateString);
  delay(2000);

  oled.setFont(font5x7);
  oled.clear();
  oled.println("        ALT     BAT");
  oled.println("VOLTS: ");
  oled.println(" AMPS: ");
  oled.println(" TEMP: ");

  oled.setCursor(0, 5);
  oled.println("FIELD PWM: ");
} //void WriteOLEDDataScreenStaticData(void) {

void WriteOLEDDynamicData(void)
{  //********************************************************************************
  //   OUTPUT LCD DYNAMIC DATA
  //********************************************************************************
  extern const char *chargingStateString;
  extern int inChargingStateCount;
 
  #ifdef ENABLE_FEATURE_IN_SCUBA
    extern const char *scubaModeString;
    static String lcdLastScubaModeString;
    char buffer[40];
  #endif

  LCDaltVolts.Update(measuredAltVolts); //Check for change and if change, save value into lastValue and print to OLED
  LCDbatVolts.Update(measuredBatVolts);
  LCDaltAmps.Update(measuredAltAmps);
  LCDbatAmps.Update(measuredBatAmps);
  //LCDbatAmps.Update(measuredBatAmps);  // don't know why this was duplicated...
  
  #ifdef OLED_DISPLAY_DEG_IN_F
  LCDaltTemp.Update(measuredAltTemp * 9 / 5 + 32); // Temp is stored in deg C.  Convert to def F.
  LCDbatTemp.Update(measuredBatTemp * 9 / 5 + 32);
  #else
  LCDaltTemp.Update(measuredAltTemp); // Display in degrees C
  LCDbatTemp.Update(measuredBatTemp);
  #endif
  
  LCDPWM.Update((100 * fieldPWMvalue) / FIELD_PWM_MAX);

  // add countdown time field data
  if ((chargingState == warm_up) || (chargingState == ramping))
  {
    LCDCount.Update(inChargingStateCount);
  }
  else 
  { 
  // not Warmup or Ramping so make sure the CountDown is zero - sets up for trap in the LCDfield Update method
    LCDCount.Update(0);
    
    if ((chargingState == acceptance_charge) || (chargingState == bulk_charge))
    {
      // convert inChargingStateTime to HH:MM:SS format
      unsigned long val = inChargingStateTime/1000UL;
      int hours = numberOfHours(val);
      int minutes = numberOfMinutes(val);
      int seconds = numberOfSeconds(val);
          
      sprintf(buffer2, "%02d:%02d:%02d" , hours, minutes, seconds);
      
    }
    else // in other chargingStates, erase the LCDCount2 field
    {
      sprintf(buffer2, "        ");
    }
    LCDCount2.Write(buffer2);
  }
  LCDState.Update(chargingStateString);  // write the chargingStateString after the inChargingStateCount
                                         //   so it overwrites the numbers if state changes during WarmUp or ramping

#ifdef ENABLE_FEATURE_IN_SCUBA
  // add scuba mode label - write this on every pass since it can change at random times and can create times when it is not displayed
  LCDScuba.Update(scubaModeString);
  //LCDScuba.Update(scubaModeString); does not work when charging state changes while scubaMode is true.  Direct print each pass is simple solution
  #ifdef USE_SERIAL_DISPLAY
    sprintf(buffer, "$D:%d,%s",
            fnSB,
            scubaModeString);
    SERIAL_DISPLAY_PORT.println(buffer);
  #endif //USE_SERIAL_DISPLAY
#endif //ENABLE_FEATURE_IN_SCUBA
} //WriteOLEDDynamicData

#endif // USE_OLED

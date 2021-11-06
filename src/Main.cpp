//------------------------------------------------------------------------------------------------------
//
//
//
// Alternator Regulator based on the Arduino IDE
//
//      Copyright (c) 2016, 2017, 2018 by William A. Thomason.      http://arduinoalternatorregulator.blogspot.com/
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
//
//
//
//    This software is a 3/4-stage alternator regulator.  A fundamental difference between this and currently available
//      regulators is the ability to monitor Amps being delivered to the battery as well as Voltage.  By doing so more intelligent
//      charging transitions can be made, resulting in shorter charging times and longer battery life.
//
//      This work is derived from the Smart Engine Control and Alternator Regulator project, and takes the same approach
//      for alternator regulation, but removes all the engine control and integration. http://smartdcgenerator.blogspot.com/
//
//
//    Note special feature, independent of Feature-in selections: 
//        - Shorting NTC sensor the ALTERNATOR will place regulator in 1/2 Alternator amps mode.
//          (Except if Charge Profile #8 LiFePO4 is selected, in that case a HIGH feature-in will cause regulator to stop charging (force to float))
//
//      December 2020 Pete Dubler refactored William A. Thomason's code to remove CAN and Bluetooth related code and move the hardware to a 
//        board with a Mini Mega Pro 2560 daughter card as the processor. "VSR MINI MEGA"
/*
 *      New hardware features include:
 *       - Dual INA226 current voltage sensors (for Alternator and Battery)    
 *       - Three Feature-in Ports and Three Feature-out Ports (functionality and port assignments selectable in Config.h)
 *       - Two serial port screw headers, separate from port used to USB programming, one can support external full function display
 *       - Two I2C pin headers to add OLED display or other features
 */
//
//                ***********************************************
//                * BE SURE TO UPDATE REV_FORK and/or DATE_CODE * 
//                * IN SmartRegulator.h after making changes    *
//                ***********************************************
/*
 *      10/13/20    LIST OF NEW DUBLER FEATURES (Turned on/off with #define statements)
 *                         ENABLE_FEATURE_IN_SCUBA, 
 *                         FEATURE_OUT_GREEN_LED, FEATURE_OUT_RED_LED,
 *                         FEATURE_OUT_LIFEPO_SHUTDOWN_ALARM,
 *                         USE_OLED, USE_SERIAL_DISPLAY ,
 *                         BENCHTEST
 *                          
 *                         
 *                  
*/

#include "Config.h"
#include "SmartRegulator.h"
#include <Wire.h>
#include "Flash.h"
#include "CPE.h"
#include "OSEnergy_Serial.h"
#include "Alternator.h"
#include "System.h"
#include "Sensors.h"
#include "LED.h"
#include "BMS_SERIAL.h"

/***************************************************************************************
****************************************************************************************
*                                                                                      *
                               STARTUP FUNCTIONS
*                                                                                      *
*                                                                                      *
       This is called one time at controller startup.
*                                                                                      *
*                                                                                      *
****************************************************************************************
****************************************************************************************/

//
//------------------------------------------------------------------------------------------------------
// Start-up routine.
//
//      For now both cold start and Watchdog restart will come through here
//
//
//
//------------------------------------------------------------------------------------------------------

void setup()
{

  wdt_enable(WDT_PERIOD); // Because we do SOOO much during setup use the Watchdog during startup...

  Serial.begin(SYSTEM_BAUD); // Start the Serial port.
  while(!Serial){};  // wait for Serial to start successfully

  #ifdef USE_SERIAL_DISPLAY
    if (SERIAL_DISPLAY_PORT != Serial)
    {
      SERIAL_DISPLAY_PORT.begin(SERIAL_DISPLAY_BAUD); // Start the SERIAL_DISPLAY_PORT.
      while(!SERIAL_DISPLAY_PORT){};  // wait for SERIAL_DISPLAY_PORT to start successfully
    }
    // output revision of this code to serial display
    delay(1000); // let remote display boot
    char buffer[40];
    sprintf(buffer, "$D:%d,%s",
         11, // fnCR (which is not in scope here)
         REV_FORK);
    SERIAL_DISPLAY_PORT.println(buffer);
  #endif

  #ifdef USE_BMS_SERIAL_IN
    BMS_SERIAL_PORT.begin(BMS_SERIAL_BAUD); // Start the BMS_SERIAL_PORT.
    while(!BMS_SERIAL_PORT){};  // wait for BMS_SERIAL_PORT to start successfully
  #endif
  
  wdt_reset(); // pat the watchdog timer

  //---  Assign Atmel pins and their functions.
  //         Am using Arduino calls as opposed to Atmel - to make things easier to read.
  //         Make sure to also see the Initialze_xxx(); functions, as they will also assign port values.
  pinMode(LED_RED_PCB, OUTPUT);
  pinMode(LED_GREEN_PCB, OUTPUT);
  pinMode(LED_RED_EXT, OUTPUT);
  pinMode(LED_GREEN_EXT, OUTPUT);

  pinMode(FEATURE_OUT_PORT1, OUTPUT);
  pinMode(FEATURE_OUT_PORT2, OUTPUT);
  pinMode(FEATURE_OUT_PORT3, OUTPUT);

  pinMode(FEATURE_IN_PORT1, INPUT);
  pinMode(FEATURE_IN_PORT2, INPUT);
  pinMode(FEATURE_IN_PORT3, INPUT);

  //----  DIP switch ports.
  pinMode(DIP_BIT0, INPUT_PULLUP); // We will use the weak internal Pull-ups for the DIP switch.
  pinMode(DIP_BIT1, INPUT_PULLUP); 
  pinMode(DIP_BIT2, INPUT_PULLUP);
  pinMode(DIP_BIT3, INPUT_PULLUP);
  pinMode(DIP_BIT4, INPUT_PULLUP);
  pinMode(DIP_BIT5, INPUT_PULLUP); 
  pinMode(DIP_BIT6, INPUT_PULLUP); 
  pinMode(DIP_BIT7, INPUT_PULLUP); 

  wdt_reset(); // pat the watchdog timer

//** good place for TEST CODE

/* 
  for(int i=0;i<8;i++)
  {
    digitalWrite(FEATURE_OUT_PORT1, (i & 1));
    digitalWrite(FEATURE_OUT_PORT2, ((i>>1) & 1));
    digitalWrite(FEATURE_OUT_PORT3, ((i>>2) & 1));
    delay(500);
  }
  
  
  for(;;){};
*/

//**END TEST CODE

  //--------      Adjust System Configuration
  //              Now that things are initialized, we need to adjust the system configuration based on the Users settings of the DIP switches.
  //              We also need to fetch the EEPROM to see if there was a saved 'custom' configuration the user had entered via the GIU.
  //
  //              Note the DIP switches (and hence any selections / adjustments) are sampled ONLY at Power up,
  //
  //------  Read the DIP Switches, and make adjustments to system based on them
  uint8_t dipSwitch;
  dipSwitch = readDipSwitch(); // Read the DIP Switch value

  cpIndex = dipSwitch & 0x07;  //  Mask off the bits indicating the Battery Type - Charge Profile - (CPE) selection
  systemAmpMult = ((float)(((dipSwitch >> 3) & 0x03) + 1)) / 2.0; //  Mask off the bits indicating the Battery Capacity selection and adjust in 250A increments 

  smallAltMode = (dipSwitch >> 5) & 0x01; //  Mask off the bit indicating Small Alternator Capability
 
  tachMode = (dipSwitch >> 6) & 0x01; // Shift and mask off bit for tachMode
  //------  Fetch Configuration files from EEPROM.  The structures already contain their heir 'default' values compiled FLASH.  But we check to see if there are
  //        validated user-saved overrides in EEPROM memory.

  read_SCS_EEPROM(&systemConfig); // See if there are valid structures that have been saved in the EEPROM to overwrite the default (as-compiled) values
  read_CAL_EEPROM(&ADCCal);       // See if there is an existing Calibration structure contained in the EEPROM.

  thresholdPWMvalue = systemConfig.FIELD_TACH_PWM; // Transfer over the users desire into the working variable.  If -1, we will do Auto determine.  If anything else we will just use that
  // value as the MIN PWM drive.  Note if user sets this = 0, they have in effect disabled Tach mode independent DIP switch.
  if (systemConfig.FORCED_TM == true)
    tachMode = true; // If user has set a specific value, or asked for auto-size, then we must assume they want Tach-mode enabled, independent of the DIP

  //---  Initialize library functions and hardware modules
  //
  initialize_sensors();
  initialize_alternator();

  //-----  Sample the System Voltage and adjust system to accommodate different VBats
  //
  delay(100); // It should have only take 17mS for the INA226 to complete a sample, but let's add a bit of padding..
  // Read the voltage to determine the voltage range for the regulator, 12V or 24V
  read_ALT_and_BAT_VoltAmps(); // Sample the voltage the alternator is connected to
   if (measuredAltVolts < 17.0)
    systemVoltMult = 1; //  Likely 12v 'system'
   else
    systemVoltMult = 2; //  must be a 24v 'system'

  //------ Now that we have done all the above work, let's see if the user has chosen to override any of the features!

  if (systemConfig.CP_INDEX_OVERRIDE != 0)
    cpIndex = systemConfig.CP_INDEX_OVERRIDE - 1;
  if (systemConfig.BC_MULT_OVERRIDE != 0.0)
    systemAmpMult = systemConfig.BC_MULT_OVERRIDE;
  if (systemConfig.SV_OVERRIDE != 0.0)
    systemVoltMult = systemConfig.SV_OVERRIDE;
  if (systemConfig.ALT_AMPS_LIMIT != -1)
    altCapAmps = systemConfig.ALT_AMPS_LIMIT; // User has declared alternator size - make sure it is recognized before getting into things.
  // (Needed here in case user has forced regulator to float via feature_in out the door
  //    bypassing RAMP: mode where this variable would normally get checked)

  if (read_CPS_EEPROM(cpIndex, &chargingParms) != true) // See if there is a valid user modified CPE in EEPROM we should be using.
    transfer_default_CPS(cpIndex, &chargingParms);      // No, so prime the working Charge profile tables with default values from the FLASH (PROGMEM) store.

    //---- And after all that, is the user requesting a Master Reset?


#ifdef FEATURE_IN_RESTORE
  if ((digitalRead(FEATURE_IN_RESTORE_PORT) == true) && // Check feature_in port, and do all the 'debounce' when doing so.
        (dipSwitch & MASTER_RESTORE_DIP_CHECK) && //  ONLY do Master Restore if CPE#5  is selected, and all the rest are in defined state.
      (systemConfig.CONFIG_LOCKOUT != 2))
  { //       And the regulator has not been really locked out!
    // feature_in is active, so do a restore all now.
    systemConfig.CONFIG_LOCKOUT = 0; // As this is a HARDWARE restore-all, override any lockout.
    #ifdef USE_OLED
      WriteOLEDFactoryReset();
    #endif
    restore_all();                   // We will not come back from this, as it will reboot after restoring all.
    
  }
#endif


#ifdef USE_OLED 
  WriteOLEDTitlePage();
  WriteOLEDBatteryType();
  wdt_reset(); // pat the watchdog timer
  WriteOLEDDIPSettings();
  WriteOLEDDataScreenStaticData();
#endif
} // End of the Setup() function.

/****************************************************************************************
 ****************************************************************************************
 *                                                                                      *
                                SUPPORTIVE FUNCTIONS
 *                                                                                      *
 *                                                                                      *
        This routines are called from within the Mainloop functions or Startup.
        These are mostly low-level functions to read sensors, keyboards, refresh
           displays, etc...
 *                                                                                      *
 *                                                                                      *
 ****************************************************************************************
 ****************************************************************************************/

//------------------------------------------------------------------------------------------------------
// Stack / Heap stamping
//
//      These functions will place a know pattern (stamp) in un-used memeory between the bottom of the stack
//      and the top of the heap.   Other functions can call later and check to see how much of this
//      'stamping' is retained in order to get an estimate of how much stack space is used during runtime.
//
//      Not quite as good as hardware enforced fences, but better then nothing.
//
//------------------------------------------------------------------------------------------------------
#ifdef DEBUG

void stampFreeStack(void) {
  uint8_t *stmpPtr;
  uint8_t *stkPtr;
  extern unsigned int __heap_start;
  extern void *__brkval;

  if ((int)__brkval == 0)
    stmpPtr = (uint8_t*)&__heap_start;
  else
    stmpPtr = (uint8_t*)__brkval;

  stmpPtr += 8;                                       // Just move a little above the top of the the heap.
  stkPtr  = (unsigned char*) &stkPtr - 8;             // And a little below the current stack pointer.

  //Serial.print("Stamping: ");
  //Serial.print((unsigned int)stmpPtr,HEX);
  //Serial.print(", to: ");
  //Serial.println((unsigned int)stkPtr,HEX);

  while (stmpPtr < stkPtr)
    *stmpPtr++ = 0xA5;

}

int checkStampStack(void) {
  unsigned int unusedCnt;
  uint8_t *stmpPtr;
  uint8_t *stkPtr;
  extern unsigned int __heap_start;
  extern void *__brkval;

  if ((int)__brkval == 0)
    stmpPtr = (uint8_t*)&__heap_start;
  else
    stmpPtr = (uint8_t*)__brkval;

  stmpPtr += 8;                      // Just move a little above the top of the the heap.
  stkPtr  = (unsigned char*) &stkPtr - 8;             // And a little below the current stack pointer.

  //Serial.print("Free memory=");
  //Serial.println(freeMemory());

  //Serial.print("Looking from: 0x");
  //Serial.print((unsigned int)stmpPtr,HEX);
  //Serial.print(", to: 0x");
  //Serial.print((unsigned int)stkPtr,HEX);


  unusedCnt = 0;

  while ((stmpPtr < stkPtr) && (*stmpPtr++ == 0xA5))
    unusedCnt++;

  //Serial.print("      Untouched is now: 0x");
  //Serial.println(unusedCnt,HEX);

  return (unusedCnt);
}
#else
int checkStampStack(void) {
  return (-1);  // Need dummy function when not compiling debug..
}
#endif

//------------------------------------------------------------------------------------------------------
// Read DIP Switch
//      This function is called from Startup and will read the DIP switch.  It will return all 7 usable bits
//      assembled into one returned value.  It is also called during the $SCN: command to make sure the DIP switch
//      is set to all-on before allowing initial updating of the Name and Password.
//
//
// For MiniMega, DIP switch mapping is:
 
  //   Bit 0 = SW1 = Charge Profile 0 - Used to select which entry in chargeParms[] array we should be using.
  //   Bit 1 = SW2 = Charge Profile 1
  //   Bit 2 = SW3 = Charge Profile 2
  //   Bit 3 = SW4 = Battery Capacity 0 - Used to indicate 'size' of battery, to adjust exit amp values
  //   Bit 4 = SW5 = Battery Capacity 1 -  SW4  SW5
  //                                        0    0 =  < 250AmpHr
  //                                        1    0 =  250AH to 500AH 
  //                                        0    1 =  500AH to 750AH
  //                                        1    1 =  >750AH
  //   Bit 5 = SW6 = Small Alt Mode     - Is the Alternator a small capability, and hence the amps should be limited to prevent overheating?
  //   Bit 6 = SW7 = Tachometer Mode    - Turn on to make sure some small level of field PWM is always sent, even during Float and Post-Float modes..
  //   Bit 7 = SW8 = Factory Reset (with FEATURE_IN_RESTORE asserted at startup)
//------------------------------------------------------------------------------------------------------

uint8_t readDipSwitch()
{

  uint8_t dipSwitch;

  dipSwitch = digitalRead(DIP_BIT0);         // Start with the LSB switch,
  dipSwitch |= (digitalRead(DIP_BIT1) << 1); // Then read the DIP switches one switch at a time, ORing them at their appropriate bit location,
  dipSwitch |= (digitalRead(DIP_BIT2) << 2); // This approach allows the DIP switch pins to be randomly assigned instead of on one port.
  dipSwitch |= (digitalRead(DIP_BIT3) << 3);
  dipSwitch |= (digitalRead(DIP_BIT4) << 4);
  dipSwitch |= (digitalRead(DIP_BIT5) << 5);
  dipSwitch |= (digitalRead(DIP_BIT6) << 6);
  dipSwitch |= (digitalRead(DIP_BIT7) << 7);

  return (~dipSwitch); // Invert the returned value, as the DIP switches are really pull-downs.  So, a HIGH means switch is NOT active.
} //read_Dip_Switch

//------------------------------------------------------------------------------------------------------
// Manage System State
//      This function look at the system and decide what changes should occur.
//
//      For the stand-alone alternator, it is much simpler - if the Alternator is not running, and not Faulted, start it up!
//
//
//------------------------------------------------------------------------------------------------------

void manage_system_state()
{

  // --------  Do we need to start the Alternator up?

  if ((chargingState == unknown) || (chargingState == disabled)) // || (chargingState == LIFEPO_FORCED_SHUTDOWN))
  {

    set_ALT_PWM(FIELD_PWM_MIN); // Make sure Field is at starting point.
    set_charging_mode(warm_up); // Prepare for Alternator, let manage_ALT() change state into Ramping
    reset_run_summary();        // Zero out the accumulated AHs counters as we start this new 'charge cycle'
  }
} //manage_system_state

//------------------------------------------------------------------------------------------------------
// Reboot
//
//      This function will disable the Alternator and then forces the Atmel CPU to reboot via a watchdog timeout.
//      It is typically called making a change to the EEPROM information to allow the controller to re-initialize
//
//
//
//------------------------------------------------------------------------------------------------------

void reboot()
{

  set_ALT_PWM(0); // 1st, turn OFF the alternator, All the way OFF as we will no longer be managing it...

  wdt_enable(WDT_PERIOD); // JUST IN CASE:  Make sure the Watchdog is enabled!
  //  (And in any case, let it be the shorter reboot watchdog timeout.

  blink_LED(LED_RESETTING, LED_RATE_FAST, -1, OUT_LAMP_MIRROR_RESETTING); // Show that something different is going on...

  ASCII_write("RST;\r\n");  // tell the USB serial world that we are resetting
  Serial_flush(); // And then make sure the output buffer clears before proceeding with the actual reset.

#ifdef USE_OLED
  WriteOLEDResetting();
#endif

#ifdef USE_SERIAL_DISPLAY // Clear fault display line
  char buffer[40];
  sprintf(buffer, "$D:%d,%s",
          11, // CAUTION hard-coded for fnFL
          "          ");
  SERIAL_DISPLAY_PORT.println(buffer);
#endif

#ifdef OPTIBOOT
  while (true)     // Optiboot!
    refresh_LED(); // Sit around, blinking the LED, until the Watchdog barks.

#else // The standard Arduino 3.3v / atMega328 has a bug in it that will not allow the watchdog to
  // work - instead the system hangs.  Optiboot fixes this.  (Note the standard bootload for the
  // Arduino UNO card is actually an Optiboot bootloader..)
  // If we are not being placed into an Optiboot target, we need to do something kludge to cause the
  // system to reboot, as opposed to leverage the watchdog timer.  So:

  void (*resetFunc)(void) = 0; // Kludge - just jump the CPU to address 0000 in the EEPROM.
  // declare reset function @ address 0.  Work around for non-support in watchdog on Arduino 3.3v
  resetFunc(); // Then jump to it to 'restart' things.
#endif

} //reboot()

/****************************************************************************************
 ****************************************************************************************
 *                                                                                      *
                                MAIN LOOP
 *                                                                                      *
 *                                                                                      *
        This gets called repeatedly by the Arduino environment
 *                                                                                      *
 *                                                                                      *
 *                                                                                      *
 ****************************************************************************************
 ****************************************************************************************/

void loop()
{
  // first things first, check if BMS is getting ready to shutdown
  #ifdef USE_BMS_SERIAL_IN
    if(checkBMS_Serial_In())
    {
       chargingState = LIFEPO_FORCED_SHUTDOWN;
       //TODO add code for fault state (and new fault code)
    }
  #endif

  if (chargingState == FAULTED)
  {
    wdt_disable();            // Turn off the Watch Dog so we do not 'restart' things.
    handle_fault_condition(); // Take steps to protect system

    if (faultCode & 0x8000U) // If the Restart flag is set in the fault code,
    {
      reboot();              //    then do a forced software restart of the regulator
    }
    //else

    for(;;){} // ain't going nowhere... return; // Else bail out, we will need a full hardware reset from the user.
  }

  //
  //
  //-------   OK, we are NOT in a fault condition.  Let's get to business, read Sensors and calculate how the machine should be behaving
  //
  //

  if (read_sensors() == false) return;    // If there was an error in reading a critical sensor we have FAULTED, restart the loop.
                                          // Treat volts and amps sensed directly by the regulator as the ALTERNATOR values. .                                                               

  calculate_RPMs();        // What speed is the engine spinning?
  calculate_ALT_targets(); // With all that known, update the target charging Volts, Amps, Watts, RPMs...  global variables.

  //
  //
  //-------     Now lets adjust the system.  But first, lets make sure we have not exceeded some threshold and hence faulted.
  //
  //

  if (check_for_faults() == true)
    return; // Check for FAULT conditions
  // If we found one, bail out now and enter holding pattern when we re-enter main loop.

  manage_ALT();          // OK we are not faulted, we have made all our calculations. . . let's set the Alternator Field.
  manage_system_state(); // See if the overall System State needs changing.

  //
  //
  //-------     Tell the world what we are doing, and see if they want us to do something else!
  //
  //
  handle_feature_in();
  check_inbound(); // See if any communication is coming in via the Bluetooth (or DEBUG terminal), or Feature-in port.

  update_run_summary(); // Update the Run Summary variables
  send_outbound(false); // And send the status via serial port - pacing the strings out.
  update_LED();         // Set the blinking pattern and refresh it. (Will also blink the FEATURE_OUT if so configured via #defines
  update_feature_out(); // Handle any other FEATURE_OUT mode (as defined by #defines) other then Blinking.

  wdt_reset(); // Pet the Dog so he does not bit us!

} // loop      // All done this time around.  

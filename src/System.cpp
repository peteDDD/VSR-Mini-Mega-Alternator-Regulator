//      System.cpp
//
//      Copyright (c) 2018 by William A. Thomason.                  http://arduinoalternatorregulator.blogspot.com/
//                                                                  http://smartdcgenerator.blogspot.com/
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
//              Note:  in an effort to have as many common files as possible between the different projects,
//              system.cpp will contain kind of a catch-all for loosely defined global functions and variables.
//              Many of these came from 'SmartRegulator.ino' and 'Alternator.cpp'
//

#include "Config.h"
#include "Sensors.h"
#include "Flash.h"
#include "CPE.h"
#include "OSEnergy_Serial.h"
#include "System.h"
#include "LED.h"
#include "Alternator.h"

//
//------------------------------------------------------------------------------------------------------
//
//      Global Variables
//
//              These control major functions within the program
//
//
//

unsigned faultCode;              // If chargingState is set to FAULTED, this holds a # that indicates what faulted.
uint8_t requiredSensorsFlag = 0; // Are there any required sensors missing?

char const firmwareVersion[] = FIRMWARE_VERSION; // Sent out with SST; status string and CAN initialization for product ID.

//---   Default System Config variable is defined here.
tSCS systemConfig = {
    false, // .REVERSED_BAT_SHUNT          --> Assume shunt is not reversed.
    false, // .REVERSED_ALT_SHUNT          --> Assume shunt is not reversed.
    90,    // .ALT_TEMP_SETPOINT           --> Default Alternator temp - 90c  (Approx 195f)
    1.00,  // .ALT_AMP_DERATE_NORMAL       --> Normal cap Alternator at 100% of demonstrated max Amp capability,
    0.75,  // .ALT_AMP_DERATE_SMALL_MODE   --> Unless user has selected Small Alt Mode via DIP switch, then do 75% of its capability
    0.50,  // .ALT_AMP_DERATE_HALF_POWER   --> User has shorted out the Alternator Temp NTC probe, indicating they want
           //                                    to do 1/2 power mode.
    -1,    // .ALT_PULLBACK_FACTOR         --> Used to pull-back Field Drive as we move towards Idle.
    0,     // .ALT_IDLE_RPM                --> Used to pull-back Field Drive as we move towards idle.  
    //                                          Set = 0 causes RPMs to be determined automatically during operation.
   //!! NOTE THAT I SET THIS TO 125 AMPS instead of 0 for my genset
    125,   // .ALT_AMPS_LIMIT --> The regulator may OPTIONALLY be configured to limit the size of the alternator output
    //                               Set = 0 to disable Amps capping.  
    //                               Set = -1 to auto-size Alternator during Ramp. (required Shunt on Alt, not Bat)
    0,     // .ALT_WATTS_LIMIT  --> The regulator may OPTIONALLY be configured to limit the load placed on the engine via the Alternator.
    //                                 Set = 0 to disable, -1 to use auto-calc based on Alternator size. (Required Shunt on Alt)
    12,    // .ALTERNATOR_POLES            --> # of poles on alternator (Leece Neville 4800/4900 series are 12 pole alts)
    ((6.7 / 2.8) * 1.00),        // .ENGINE_ALT_DRIVE_RATIO      --> Engine pulley diameter / alternator diameter &  fine tuning calibration ratio
    (int)((500 / 0.050) * 1.00), // .BAT_AMP_SHUNT_RATIO **Spec of amp shunt,  500A / 50mV shunt (Link10 default) and % calibrating error
    //                                 CAUTION:  Do NOT exceed 80mV on the AMP Shunt input
    (int)((300 / 0.075) * 1.00), // .ALT_AMP_SHUNT_RATIO  -->
    //                                 DUBLER 061720 CHANGED TO 300A/75mV for alternator shunt 
    //                                 CAUTION:  Do NOT exceed 80mV on the AMP Shunt input
    
    
    -1, // .FIELD_TACH_PWM  --> If user has selected Tach Mode, use this for MIN Field PWM.
    //                      Set = -1 to 'auto determine' the this value during RAMP phase
    //                      Set =  0 to in effect 'disable' tach mode, independent of the DIP switch.
    0, // .FORCED_TM   --> User can FORCE tach mode independent of DIP switch using $SCT command.
    //                      0=DIP/off, 1=Force-on

    0,   // .CP_INDEX_OVERRIDE  --> Use the DIP switch selected indexes
    0.0, // .BC_MULT_OVERRIDE   --> Use the DIP switch selected multiplier
    0.0, // .SV_OVERRIDE        --> Enable Auto System voltage detection
    0,   // .CONFIG_LOCKOUT     --> No lockouts at this time.
    #ifdef BENCHTEST
    2,   // .ENGINE_WARMUP_DURATION  **Shortened for bench testing**
    #else
    15,  // .ENGINE_WARMUP_DURATION  --> Allow engine X seconds to start and 'warm up' before placing a load on it.  
    //                                   DUBLER MOD 080320 changed from 60
    #endif
    0};  // .REQURED_SENSORS  --> Force check and fault if some sensors are not present (eg alt temp sensor)

tModes chargingState = unknown; // What is the current state of the alternator regulators?  (Ramping, bulk, float, faulted, etc...)
//tModes systemState = unknown;   // For Alternator, this is just a dummy placeholder to allow for common code.

//#define _SET_FO_PORT1(x) digitalWrite(FEATURE_OUT_PORT1, x)
//#define _GET_FO_PORT1() digitalRead(FEATURE_IN_PORT1)

#define FOP_ON HIGH
#define FOP_OFF LOW

void blink_out(int digd);

/****************************************************************************************
 ****************************************************************************************
 *                                                                                      *
                                MAINLOOP SUPPORT FUNCTIONS
 *                                                                                      *
 *                                                                                      *
        These routines are called from the Mainloop (as opposed to being inline)
        They are placed here to help readability of the mainloop.
 *                                                                                      *
 *                                                                                      *
 ****************************************************************************************
 ****************************************************************************************/

//------------------------------------------------------------------------------------------------------
// Feature in - Multiple Feature-In Port Implementation
//
//      A class is created for the Feature-In_Ports so we can have as many as needed and leverage the code.
//
//      The functionality of each Feature-In port is configurand in Config.h
//      Each of the Feature_In instances are sampled and will return TRUE if the port is currently being held High.
// 
//      Check_Port is passed a flag will tell us if we should just check the Feature_in port, doing all the de-bounce
//      checking in this one call, or if we should just check the debounce and return the last known state
//      if we have not completed the required number of samples (to be checked again at a later time).
//      (e.g:  Should this function behave as a Blocking or non-blocking function?)
//
//      Returns True only if the feature_in pin has been held active during the entire debouncing time.
//
//
//------------------------------------------------------------------------------------------------------

class Feature_In_Ports
{
private:
  bool m_lastKnownState = false; // Used to retain the featureIn statue during 'next' debounce cycle.
  bool m_proposedState = false;  // Used to see if we have a constant state during entire debounce time period.
  bool m_readState;
  bool m_have_seen_low = false;    // Used when looking for a low-to-high transition, where needed
  int8_t m_debounceCounter = DEBOUNCE_COUNT;  // Used to count-down sampling for de-bouncing.
  int8_t m_stuckCounter = 2 * DEBOUNCE_COUNT; // OK, if there is a LOT of noise on the feature-in pin, only stay in this routing for so int32_t.
  uint8_t m_port_number;

public:
  // constructor
  Feature_In_Ports(uint8_t feature_in_port) : m_port_number(feature_in_port)
  {
  }

  uint8_t Get_Port_Number()
  {
    return m_port_number;
  }

  bool Get_Have_Seen_Low()  // tells us if this port has been low
  {                         
    return m_have_seen_low;
  }

  void Set_Have_Seen_Low(bool have_seen_low)
  {
    m_have_seen_low = have_seen_low;
  }

  bool Check_Port(bool waitForDebounce)
  {
    if(cpIndex==LIFEPO4){};
    while (--m_stuckCounter > 0)
    {
      // A bit of safety, this will prevent us from being stuck here forever if there is a noisy feature_in...

      m_readState = digitalRead(m_port_number);

      if (m_readState == m_proposedState)
      { // feature-in looks to be stable.
        if (--m_debounceCounter <= 0)
        {                                     // And it appears it has been for some time as well!
          m_lastKnownState = m_proposedState; //   Recognize the new 'state'
          m_debounceCounter = DEBOUNCE_COUNT; //   Reset the debounce counter to take a new go at the feature_in port
          break;                              //   And return the new state!
        }
      }
      else // Nope, not stable.
      {
        m_proposedState = m_readState;      // Looks like we have a new candidate for the feature_in() port.
        m_debounceCounter = DEBOUNCE_COUNT; // Reset the counter and let’s keep looking to see if it remains this way for a while.
      }

      if (waitForDebounce == true)
      {
        delay(DEBOUNCE_TIME); // They want us to do all the debouncing, etc..
      }
      else // Don't want to stick around,
      {
        break; // so break out of this While() loop.
      }
    } // while
    return (m_lastKnownState);
  } //Check_Port

}; //end of class

// create Feature-In port instances
Feature_In_Ports FI_Port_1(FEATURE_IN_PORT1);
Feature_In_Ports FI_Port_2(FEATURE_IN_PORT2);
Feature_In_Ports FI_Port_3(FEATURE_IN_PORT3);

// create list of Feature-In Ports to step through
Feature_In_Ports *All_FI_Ports[] = {&FI_Port_1, &FI_Port_2, &FI_Port_3};



void handle_feature_in(void)
{
  for (size_t i = 0; i < sizeof All_FI_Ports / sizeof All_FI_Ports[0]; i++)
  {
    // NOTE: if none of the feature-in ports and features are enabled, the following three declarations
    //       will generate an "unused variable" error at compile time.  Just ignore these errors and 
    //       DO NOT REMOVE THESE or you enabling the feature in features later will not compile.
    bool port_state = All_FI_Ports[i]->Check_Port(false);  // Check the feature-in port, but do NOT wait around for debouncing.. (We will catch it next time)
    uint8_t port_number = All_FI_Ports[i]->Get_Port_Number();
    bool have_seen_low = All_FI_Ports[i]->Get_Have_Seen_Low();  // used in handling Equalization and LIFEPO Shutdown features-in

#ifdef ENABLE_FEATURE_IN_SCUBA
    if (port_number == ENABLE_FEATURE_IN_SCUBA_PORT)
    {
      if (port_state == true)
      {
        scubaMode = true;
        scubaModeString = "SCUBA";
      }
      else
      {
        scubaMode = false;
        scubaModeString = "     ";
      }
    }
#endif // ENABLE_FEATURE_IN_SCUBA

#ifdef FEATURE_IN_EQUALIZE 
  if (cpIndex != LIFEPO4) // never equalize a LiFePO4 battery
    if (port_number == FEATURE_IN_EQUALIZE_PORT)
      {
        if (port_state == true)
        {                                                                                  
          if (((chargingState == acceptance_charge) || (chargingState == float_charge)) && // True, user is asking for Equalize.  Can they have it?
              (have_seen_low == true))
          { // Only if the port has toggled from low to high - safety to prevent stuck (or left on) feature-in switch.
            set_charging_mode(equalize);
          } // OK, they want it - it seems like the battery is ready for it - so:  let them have it.
        }
        else
        {
          All_FI_Ports[i]->Set_Have_Seen_Low(true);
          if (chargingState == equalize)     // We are equalizing, and user dropped Feature-in line.
            set_charging_mode(float_charge); // So stop equalizing, and go into Float...
        }
      } //if (port_number == FEATURE_IN_EQUALIZE_PORT)
        
#endif //FEATURE_IN_EQUALIZE

#ifdef FEATURE_IN_FORCE_TO_FLOAT
        //if ((cpIndex == 7) && (chargingState > pending_R))  // in MiniMega version, we allow Force-To_Float with any CPE
         // Check to see if we are being asked to force the mode into float via the feature-in port.
          if (port_number == FEATURE_IN_FORCE_TO_FLOAT_PORT)
          {
            if (port_state == true)
            {                                           // An external device is telling us to keep out of bulk/accept/OC.  (e.g, a LiFoPO4 BMC)
              if (chargingState != forced_float_charge) // If not already in forced_float, change the state.
              { 
                set_charging_mode(forced_float_charge);
                #ifdef FEATURE_OUT_LIFEPO_SHUTDOWN_ALARM
                  digitalWrite(FEATURE_OUT_LIFEPO_SHUTDOWN_ALARM_PORT,FOP_ON);
                #endif
              }
            }
            else
            {  // Feature_in() is no longer being held active.
              #ifdef FEATURE_OUT_LIFEPO_SHUTDOWN_ALARM
                  digitalWrite(FEATURE_OUT_LIFEPO_SHUTDOWN_ALARM_PORT,FOP_OFF); 
              #endif
              if ((chargingState == forced_float_charge) && ((requiredSensorsFlag & RQBatTempSen) == 0)) // All right, Feature-in port is NOT asking use to force float mode.   By chance, did it?
              { set_charging_mode(ramping);  
                
              }                                                           // Yes it had, and now it is no longer doing so.  So we are being asked to restart charging.
              //  (Unless it also happens to be that the required battery temperature sensor is missing...)
            }
          }
#endif  // FEATURE_IN_FORCE_TO_FLOAT

      } // for (size_t i = 0; i < sizeof All_FI_Ports / sizeof All_FI_Ports[0]; i++)
} //handle_feature_in

//------------------------------------------------------------------------------------------------------
//
// Update FEATURE_OUT port
//
//      This routine will manage the Feature Out port (other than Blinking) depending on how #defined are
//      at the beginning of the program.  Note that 'communications' type usage of the Feature_out port
//      (Specifically, blinking of the light to mirror the LED in error conditions) is not handled here, but
//      instead as part of the BLINK_XXX() functions above.
//
//
//
//
//------------------------------------------------------------------------------------------------------

void update_feature_out(void)
{
// NOTE: FEATURE_OUT_LAMP is handled in LED.cppv... refresh_LED()
// NOTE: FEATURE_OUT_GREEN_LED and FEATURE_OUT_RED_LED are handled in LED.cpp ... outputGreenLED(x) and outputRedLED(x)
// NOTE: FEATURE_OUT_LIFEPO_SHUTDOWN_ALARM is handled in handle_feature_in() above as part of handling FEATURE_IN_FORCE_TO_FLOAT
#ifdef FEATURE_OUT_COMBINER // Enable FEATURE_OUT port to go active when VBat exceeds ????, useful for connecting external relay to join 2nd battery
#define _SET_FO_COMBINER(x) digitalWrite(FEATURE_OUT_COMBINER_PORT, x)
  static bool combinerEnabled = false; // Is the combiner currently enabled?  Used in part to allow a hysteresis check before disabling on low voltage.
  switch (chargingState)
  {
  case acceptance_charge: // During the Acceptance Phase, see if we are configured for a time-out...

    if ((millis() - altModeChanged) > COMBINE_ACCEPT_CARRYOVER)
      combinerEnabled = false; //  ...disable combiner.

    break;

  case bulk_charge:
    // Go through a nested tree of decisions to see if we should enable the combiner
    if (measuredBatVolts >= (COMBINE_CUTIN_VOLTS * systemVoltMult))
      combinerEnabled = true; // 1st check:  is VBat above the cut-in voltage?   Yes, enable the combiner.
    // Continue the remaining nested checks outside of the SWITCH, in that way
    // the boundary tests are always preformed.  (e.g., during carryover in acceptance mode)
    break;

  default:
    combinerEnabled = false; // All other modes, disable combiner

  } // switch

  if (measuredBatVolts >= (COMBINE_CUTOUT_VOLTS * systemVoltMult))
    combinerEnabled = false; // 2nd check:  if VBat is too high, disable combiner
  if (measuredBatVolts < (COMBINE_HOLD_VOLTS * systemVoltMult))
    combinerEnabled = false; // 3rd check:  if VBat is below the cut-out, disable combiner.
  // All other cases, just leave it in the state it already is in.  This will provide for
  // a level of hysteresis between the cutin volts and the hold-volts levels

  if (combinerEnabled == true) // Now that is known which state we want, set the feature_out port.
    _SET_FO_COMBINER(FOP_ON);
  else
    _SET_FO_COMBINER(FOP_OFF);
#endif

// Enable FEATURE_OUT port to go active when we enter FLOAT mode.  Useful to auto-stop a DC generator when charging has finished.
#ifdef FEATURE_OUT_ENGINE_STOP 
  #define _SET_FO_ENGINE_STOP(x) digitalWrite(FEATURE_OUT_ENGINE_STOP_PORT, x) 
  switch (chargingState)
  {
  case float_charge:
  case forced_float_charge:
  case post_float:
  case FAULTED:
    //_SET_FO_ENGINE_STOP(FOP_ON); // If we are in Float, or Post_float:  Enable Feature-out to indicate battery is all charged.
    single_engine_stop_pulse();  // send out a single pulse, once and only once.
    break;

  default:
    _SET_FO_ENGINE_STOP(FOP_OFF); // All other modes, Disable Feature-out port
  }

#endif
} //update_feature_out

#ifdef FEATURE_OUT_ENGINE_STOP 
void single_engine_stop_pulse()
{
  static bool f_sent_single_pulse = false;
  #define _SET_FO_ENGINE_STOP(x) digitalWrite(FEATURE_OUT_ENGINE_STOP_PORT, x) 
  if(f_sent_single_pulse == false)
  {
    wdt_reset();
     _SET_FO_ENGINE_STOP(FOP_ON);
     delay(ENGINE_STOP_PULSE_DURATION);
     _SET_FO_ENGINE_STOP(FOP_OFF); 
     f_sent_single_pulse = true;
  }
}
#endif
//------------------------------------------------------------------------------------------------------
// Check for Fault conditions
//
//      This function will check to see if something has faulted on the engine.
//
//      If a fault condition is found, the appropriate state will be set of FAULTED, the global variable
//      faultCode will be set, and this function will return TRUE indicating a fault.
//
//      If no fault is found, this function will return FALSE.
//
//------------------------------------------------------------------------------------------------------

bool check_for_faults()
{

  unsigned u;

  //----  Alternator doing OK?
  //
  //      Take note how this Switch statement is structured, with many fall-thoughs to allow testing at approperate stages.
  //

  u = 0; // Assume there is no fault present.
  switch (chargingState)
  {
  case unknown:
  case disabled:
  case warm_up:
  case post_float:
    break;

  case ramping:
  //case post_ramp:
    if (measuredAltTemp >= systemConfig.ALT_TEMP_SETPOINT)
      u = FC_LOOP_ALT_TEMP_RAMP;
    // If we reach alternator temp limit while ramping, it means something is wrong.
    // Alt is already too hot before we really even start to do anything...

    //----   And at this point we also check to see if some of the REQUIRED sensors are missing.
    //       If so, we set approperte flags to notify the rest of the code that different behaivious is exptected.
    //
    if ((systemConfig.REQURED_SENSORS & RQAltTempSen) && (measuredAltTemp == -99))
      requiredSensorsFlag |= RQAltTempSen;
    if ((systemConfig.REQURED_SENSORS & RQBatTempSen) && (measuredBatTemp == -99))
    {
      requiredSensorsFlag |= RQBatTempSen;
      set_charging_mode(forced_float_charge);
    }

  case determine_ALT_cap:
  case acceptance_charge:
  case overcharge_charge:
    if ((systemConfig.REQURED_SENSORS & RQAmpShunt) && (shuntAltAmpsMeasured != true))
    {
      requiredSensorsFlag |= RQAmpShunt;
      u = FC_SYS_REQIRED_SENSOR;
    }
    // By this time we SHOULD have seen some indication of the amps present..

  case bulk_charge: //  Do some more checks if we are running.
    if (measuredBatVolts > (FAULT_BAT_VOLTS_CHARGE * systemVoltMult))
      u = FC_LOOP_BAT_VOLTS;
    //  Slightly lower limit when not equalizing.
    break;

  case equalize:
  case float_charge:
  case forced_float_charge:
  case LIFEPO_FORCED_SHUTDOWN:
    if (measuredBatVolts > (FAULT_BAT_VOLTS_EQUALIZE * systemVoltMult))
      u = FC_LOOP_BAT_VOLTS;
    // We check for Float overvolt using the higher Equalize level, because when we
    // leave Equalize we will go into Float mode.  This prevents a false-fault, though it
    // does leave us a bit less protected while in Float mode...
    break; // If we make it to here, all is well with the Alternator!

  default:
    u = FC_LOG_ALT_STATE; //  Some odd, unsupported mode.   Logic error!
  }                       //switch

  if (measuredAltTemp > (systemConfig.ALT_TEMP_SETPOINT * FAULT_ALT_TEMP))
    u = FC_LOOP_ALT_TEMP;
  
  if ((measuredBatVolts < (FAULT_BAT_VOLTS_LOW * systemVoltMult)) && (fieldPWMvalue > (FIELD_PWM_MAX - FIELD_PWM_MIN) / 3))
    u = FC_LOOP_BAT_LOWV;
  // Check for low battery voltage, but hold off until we have applied at least 1/3 field drive.
  // In this way, we CAN start charging a very low battery, but will not go wild driving the alternator
  // if nothing is happening.  (ala, the ignition is turned on, but the engine not started.)

  if (measuredBatTemp > FAULT_BAT_TEMP)
    u = FC_LOOP_BAT_TEMP;

  if ((requiredSensorsFlag != 0) && (systemConfig.REQURED_SENSORS & RQFault))
    u = FC_SYS_REQIRED_SENSOR; // A required sensor is missing, and we are configured to FAULT out on that condition.

  if (u != 0)
  {
    chargingState = FAULTED;
    faultCode = u;
  }

  //----  System doing OK?
  u = 0; //  Again, use 'u' to receive fault-code if a fault is found.  For not assume no fault.
  if (cpIndex != (cpIndex & 0x07))
    u = FC_LOG_CPINDEX; // cpIndex pointer out of range?
  if (systemAmpMult != constrain(systemAmpMult, 0.0, 10.0))
    u = FC_LOG_SYSAMPMULT; // systemAmpMult out of range?

  if (measuredFETTemp > FAULT_FET_TEMP)
    u = FC_SYS_FET_TEMP; // Driver FETs overheating?

  if (u != 0)
  {
    chargingState = FAULTED; //   Just use the chargingState to indicate these two system faults,
    faultCode = u;
  }

  if (chargingState == FAULTED)
    return (true); //  Did something fault?
  else
    return (false);

} //check_for_faults

//------------------------------------------------------------------------------------------------------
//
//      Handle FAULT condition
//
//
//      This function is called when the system is in a FAULTED condition.  It will make sure things are
//      shut down, and blink out the appropriate error code on the LED.
//
//      If the Fault Code has the 'restart flag' set (by containing 0x8000), after blinking out the fault code
//      the regulator will be restarted.  Else the regulator will continue to blink out the fault code.
//
//
//------------------------------------------------------------------------------------------------------

void handle_fault_condition()
{

  unsigned j;
  char buffer[80];

  //-----  Make sure the alternator, etc. is stopped.
  set_ALT_PWM(0); // Field PWM Drive to Low (Off)

  j = faultCode & 0x7FFFU; // Masking off the restart bit.

  #ifdef USE_OLED
    if(faultCode & 0x8000U) // check for restart bit
      WriteOLEDFault();
    else
      WriteOLEDNonResetFault();
  #endif

#ifdef USE_SERIAL_DISPLAY // output fault code to serial display
  sprintf(buffer, "$D:%d,FAULT %d",
          11, // CAUTION hard-coded for fnFL
          j);
  SERIAL_DISPLAY_PORT.println(buffer);
#endif

  snprintf_P(buffer, sizeof(buffer) - 1, PSTR("FLT;,%d,%d\r\n"), // Send out the Fault Code number and the Required Sensor Flag.
             j,
             requiredSensorsFlag);

  ASCII_write(buffer);
  send_outbound(true); // And follow it with all the rest of the status information.
  while (send_outbound(false))
    ; //  (Keep calling until all the strings are pushed out)

  blink_LED(LED_FAULTED, LED_RATE_FAST, 2, OUT_LAMP_MIRROR_FAULT); // Blink out 'Faulted' pattern to both LED and LAMP
  while (refresh_LED() == true)
    ; // Send the pattern out to the LED.

  delay(1000); // 1 second pause

  if (j >= 100)
    blink_out(((j / 100) % 10)); // Blink out Fault number 100's 1st

  blink_out(((j / 10) % 10)); // Blink out Fault number 10's 2nd
  blink_out((j % 10));        // Blink out Fault number units last

  delay(5000); // Pause 5 seconds so flashing LED can be observed
} //handle_fault_condition

void blink_out(int digd)
{
  static const PROGMEM unsigned num2blinkTBL[9] = {
      // Table used convert a number into a blink-pattern
      0x0000, // Blink digit '0'  (Note, Arduino pre-processor cannot process binary decs larger than 8 bits..   So, 0x____)
      0x0002, // Blink digit '1'
      0x000A, // Blink digit '2'
      0x002A, // Blink digit '3'
      0x00AA, // Blink digit '4'
      0x02AA, // Blink digit '5'
      0x0AAA, // Blink digit '6'
      0x2AAA, // Blink digit '7'
      0xAAAA  // Blink digit '8'      For 9, you will need to blink out "1"+8 :-)
  };          //   (BTW: This 1+8 is why the other digits appear to be 'shifted left' one bit, to make a smooth LED blink when doing a 9

  if (digd == 9)
  {
    blink_LED(0x0001, LED_RATE_NORMAL, 1, OUT_LAMP_MIRROR_FAULT); // 9 has to be sent as a "1"+8
    while (refresh_LED() == true);  // wait for led to complete the refresh
    digd--;
  }

  blink_LED(pgm_read_dword_near(num2blinkTBL + digd), LED_RATE_NORMAL, 1, OUT_LAMP_MIRROR_FAULT);
  while (refresh_LED() == true)
    ; // Send the pattern out to the LED.
  delay(750);
} //blink_out

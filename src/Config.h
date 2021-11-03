//
//      Config.h
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


#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "SmartRegulator.h" 
                                                       
/****************************************************************************************
 ****************************************************************************************
 *                                                                                      *
 *                              CUSTOMIZING PARAMETERS                                  *
 *                                                                                      *
 *                                                                                      *
 *      Change the following parameters to meet the exact needs of your installation.   *
 *                                                                                      *
 *      These are general settings which most likely need to be changed to meet your    *
 *         needs.   For more advanced settings, see SmartRegulator.h                    *
 *                                                                                      *
 *      See references at the end of this file for DIP switch settings                  *
 *                                                                                      *
 * **************************************************************************************
 ****************************************************************************************/


//*************************************************************************************************************************************
//*************************************************************************************************************************************
// You can turn off the following features by commenting out the #define statements
// Display Options - both OLED and SERIAL_DISPLAY can be used
#define USE_OLED     // to use the Geekcreit SSD1306 I2C OLED which also requires the SoftWire shell to the I2CMaster library
#define OLED_DISPLAY_DEG_IN_F // display temperatures in Degrees F instead of C (comment out for Deg C)

#define USE_SERIAL_DISPLAY  //output LCD change info on serial line
#define SERIAL_DISPLAY_PORT Serial1
#define SERIAL_DISPLAY_BAUD 9600UL

#define USE_BMS_SERIAL_IN  // Serial input for BMS commands 
#define BMS_SERIAL_PORT Serial2
#define BMS_SERIAL_BAUD 9600UL

//Note: for faster bench testing, turn on BENCHTEST in SmartRegulator.h

//*************************************************************************************************************************************
//*************************************************************************************************************************************

//FEATURE_IN and FEATURE_OUT are configured below
//*********************************************************************************************
//*********      SELECT FEATURE_IN TO USE     SELECT UP TO THREE    ***************************
//*********      **BUT BE SURE TO ASSIGN THEM TO DIFFERENT FEATURE_IN_PORTS**      ************
//*********************************************************************************************
//** INSTRUCTIONS:
//*  To turn on a feature-in functionality, first uncomment the feature 
//*     example: to activate the Equalize feature-in functionality:
//*       //#define FEATURE_IN_EQUALIZE   
//*     would be changed to
//*       #define FEATURE_IN_EQUALIZE
//*  Next assign a feature-in port number by uncommenting the next line and changing the last 
//*   character of that line.
//*     example: to assign FEATURE_IN_EQUALIZE to FEATURE_IN_PORT3: 
//*       #define FEATURE_IN_EQUALIZE_PORT FEATURE_IN_PORT3 
//*     would be changed to
//*       define FEATURE_IN_EQUALIZE_PORT FEATURE_IN_PORT2
//* 
//*   NOTE: The order of the Feature-In features below is important because the last one encountered
//*           and active will set the system behavior.
//*********************************************************************************************

//#define FEATURE_IN_EQUALIZE     // Enable FEATURE_IN port to select EQUALIZE mode while regulator is operating
//#define FEATURE_IN_EQUALIZE_PORT FEATURE_IN_PORT3

#define ENABLE_FEATURE_IN_SCUBA
#define ENABLE_FEATURE_IN_SCUBA_PORT FEATURE_IN_PORT1

#ifdef ENABLE_FEATURE_IN_SCUBA
#define FIELD_PWM_SCUBA 23 //some small percent of maximum value for PWM  23 = 9%   32 = 12%
                           //the equation is value = int((target percentage * 255)/100)S
#endif

#define FEATURE_IN_FORCE_TO_FLOAT     // Enable FEATURE_IN port to prevent entering active charge modes (Bulk, Acceptance, Overcharge) and only allow Float or Post_Float with any CPE.                                                              //   This capability will ONLY be active if CPE #8 is selected, and it will also prevent other feature_in options from being usable
#define FEATURE_IN_FORCE_TO_FLOAT_PORT FEATURE_IN_PORT2  //   
  //NOTE: You can add FEATURE_OUT_LIFEPO_SHUTDOWN_ALARM below in the Feature-Out section to provide
  //        a BMS shutdown alarm

//#define FEATURE_IN_RESTORE      // Enable FEATURE_IN port to be used during Startup to restore the Regulator to as-shipped (compiled) state if actived at inintal power-on
//  #define  FEATURE_IN_RESTORE_PORT  FEATURE_IN_PORT3
//     NOTE:  "restore" may be combined with other Feature_in defines, as it impacts only startup operation, not running.
//            "Restore" is only effective if user selects Charge Profile #6

//*************************************************************************************************************************************
//*********************************************************************************************
//*********      SELECT FEATURE_OUT TO USE    SELECT UP TO THREE    ***************************
//*********      **BUT BE SURE TO ASSIGN THEM TO DIFFERENT FEATURE_OUT_PORTS**     ************
//*********************************************************************************************
//** INSTRUCTIONS:
//*  To turn on a feature-out functionality, first uncomment the feature 
//*     example: to activate the Green LED feature-out functionality:
//*       //#define FEATURE_OUT_GREEN_LED   
//*     would be changed to
//*       #define FEATURE_OUT_GREEN_LED
//*  Next assign a feature-out port number by uncommenting the next line and changing the last 
//*   character of that line.
//*     example: to assign FEATURE_OUT_GREEN_LED to FEATURE_IN_PORT3: 
//*       #define FEATURE_OUT_GREEN_LED_PORT FEATURE_OUT_PORT1
//*     would be changed to
//*       define FEATURE_OUT_GREEN_LED_PORT FEATURE_OUT_PORT3
//* 
//#define FEATURE_OUT_GREEN_LED // Add new feature to echo the green LED to the feature out port
//  #define FEATURE_OUT_GREEN_LED_PORT FEATURE_OUT_PORT1

//#define FEATURE_OUT_RED_LED // Add new feature to echo the green LED to the feature out port
//  #define FEATURE_OUT_RED_LED_PORT FEATURE_OUT_PORT2

//#define FEATURE_OUT_LAMP // Enable FEATURE_OUT port as a LAMP driver, to show rough status as well as blink out the FAULT code
//  #define FEATURE_OUT_LAMP_PORT FEATURE_OUT_PORT1
// set as many of the following mirror options to true for the "Lamp" as you wish
//   each must be set to either true or false and should not be commented out
  #define OUT_LAMP_MIRROR_EQUALIZE true  // reflect equalization status on LAMP if FEATURE_OUT_LAMP is defined
  #define OUT_LAMP_MIRROR_RESETTING true  // flash resetting code to LAMP if FEATURE_OUT_LAMP is defined
  #define OUT_LAMP_MIRROR_FAULT true  // flash fault codes to LAMP if FEATURE_OUT_LAMP is defined


#define FEATURE_OUT_ENGINE_STOP // Enable FEATURE_OUT port to go active when we enter FLOAT mode.  Useful to auto-stop a DC generator when charging has finished.
  #define FEATURE_OUT_ENGINE_STOP_PORT FEATURE_OUT_PORT3
#ifdef FEATURE_OUT_ENGINE_STOP
  #define ENGINE_STOP_PULSE_DURATION 1000 // milliseconds
#endif

#define FEATURE_OUT_LIFEPO_SHUTDOWN_ALARM // Enable FEATURE_OUT port to go active when Alternator Controller receives an active Force_To_Float Feature-in
#define FEATURE_OUT_LIFEPO_SHUTDOWN_ALARM_PORT FEATURE_OUT_PORT2

//#define FEATURE_OUT_COMBINER    // Enable FEATURE_OUT port to go active when alternator is in Accept or Float phase.  That is when the main battery had been 'bulked up'.
  //#define FEATURE_OUT_COMBINER_PORT FEATURE_OUT_PORT1
  // uncomment one of the following two profiles
    #define COMBINER_PROFILE_LIFEPO // for LiFePO4 batteries
    //#define COMBINER_PROFILE_OTHER // for other batteries
//   Useful for connecting external relay to join 2nd battery, but you need to make sure to set the Capacity DIP switches to reflect the
//   total capacity of the TWO (or more) batteries when combined.
//*********************************************************************************************
//----   COMBINER VALUES
//       Used to define how the FEATURE_OUT_COMBINER will work.
//       note:  all voltages are in 'nominal' 12v form, and will be adjusted at runtime by the systemVoltMult, but not battery temperature...
#ifdef FEATURE_OUT_COMBINER
  #ifdef COMBINER_PROFILE_LIFEPO // combiner settings for LiFePO4 batteries
    #define COMBINE_CUTIN_VOLTS 13.6              // Enable to combiner once OUR battery voltage reaches this level.
    #define COMBINE_HOLD_VOLTS 13.3               // Once enabled remain so on even if voltage sags to this value - a hysteresis to reduce relay chatter/cycling.
    #define COMBINE_CUTOUT_VOLTS 13.9            // But then disable the combiner once OUR battery voltage reaches this level.
                                                  // This is for cases where we are looking to get a boost from the other battery during
                                                  // bulk phase, but do not want to run the risk of 'back charging' the other battery
                                                  // once voltage raises a bit.   --HOWEVER--
    #define COMBINE_ACCEPT_CARRYOVER 0.05 * 3600000UL // If we indeed want to continue on and back charge the other battery (say, in the case the other
                                                  // battery has no other charging source, ala a bow thruster battery), then this is a brute-force
                                                  // carry over into the Accept phase, keep the combiner on for this many hours. (0.75, or 45 minutes)
                                                  // If you want this to actually WORK, then you will need to raise the dropout voltage above to a higher
                                                  // threshold, perhaps 15v or so??
                                                  // See documentation for more details around these parameters and how to configure them..
  #endif

  #ifdef COMBINER_PROFILE_OTHER // combiner settings for other battery types
    #define COMBINE_CUTIN_VOLTS 13.2              // Enable to combiner once OUR battery voltage reaches this level.
    #define COMBINE_HOLD_VOLTS 13.0               // Once enabled remain so on even if voltage sags to this value - a hysteresis to reduce relay chatter/cycling.
    #define COMBINE_CUTOUT_VOLTS 14.2            // But then disable the combiner once OUR battery voltage reaches this level.
                                                  // This is for cases where we are looking to get a boost from the other battery during
                                                  // bulk phase, but do not want to run the risk of 'back charging' the other battery
                                                  // once voltage raises a bit.   --HOWEVER--
    #define COMBINE_ACCEPT_CARRYOVER 0.75 * 3600000UL // If we indeed want to continue on and back charge the other battery (say, in the case the other
                                                  // battery has no other charging source, ala a bow thruster battery), then this is a brute-force
                                                  // carry over into the Accept phase, keep the combiner on for this many hours. (0.75, or 45 minutes)
                                                  // If you want this to actually WORK, then you will need to raise the dropout voltage above to a higher
                                                  // threshold, perhaps 15v or so??
                                                  // See documentation for more details around these parameters and how to configure them..
  #endif
#endif

//*************************************************************************************************************************************

// Here is a reference to the DIP switch settings
// For MiniMega, DIP switch mapping is:
 
  //   Bit 0 = SW1 = Charge Profile 0 - aka "Battery type" - Used to select which entry in chargeParms[] array we should be using.
  //   Bit 1 = SW2 = Charge Profile 1 - see charge profile reference below
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

// Here is a reference to the Charge Profiles (battery types)
//   Details are found in CPE.cpp
//
// Charge Profile      DIP SWITCH
//  Number    Name      1    2    3
//  0       LIFELINE   off  off  off
//  1       STD FLA    off  off  on
//  2       HD FLA     off  on   off
//  3       AGM #2     off  on   on
//  4       GEL        on   off  off
//  5       FIREFLY    on   off  on
//  6       CUSTOM     on   on   off
//  7       LiFePO4    on   on   on

#endif  // _CONFIG_H_

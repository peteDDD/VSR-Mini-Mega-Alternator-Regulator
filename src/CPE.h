//      CPE.h
//
//      Copyright (c) 2016, 2017, 2018 by William A. Thomason.      http://arduinoalternatorregulator.blogspot.com/
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

#ifndef _CPE_H_
#define _CPE_H_

#include "Config.h"

//----- This structure defines a 'profile' for battery charging.  Each stage consist of 'modes', primarily:  Bulk, Acceptance,
//      Overcharge, and Float.  Each mode has a max voltage set point, and criteria for exiting that phase (Exceeding a time limit,
//      or Amps dropping below a given value).   Of special note is the entry Float and Post Float, which have additional criteria
//      resuming charging.
//
#define MAX_CPES 8    //  There are 8 different Charge profile Entries
#define CUSTOM_CPES 2 //  The last two of which are set aside as 'customizable' and are changeable via the ASCII string commands.
// create human-readable names for charge profiles  
// *** BE CERTAIN THESE CORRESPOND TO ANY CHANGES MADE IN CPE.cpp ***
enum ChargeProfiles {LIFELINE, STD_FLA, HD_FLA, AGM_2, GEL, FIREFLY, FOUR_STAGE_FLA, LIFEPO4};

typedef struct
{ // Charging Profile Structure
   
   const char *BATTERY_TYPE;    // String for the battery type
   float ACPT_BAT_V_SETPOINT;   // Set point for Ramp, Bulk and Acceptance battery voltage.
                                // Alternator will transition from BULK mode into Accept Mode when this voltage is reached, and then start the Accept Duration counter.
   uint32_t EXIT_ACPT_DURATION; // Stay in Accept mode no longer then duration in mS  (Set = 0 to disable Acceptance phase and move directly to OC or Float mode)
   int EXIT_ACPT_AMPS;          // If Amps being delivered falls to this level or below, exit Accept mode and go to next
                                //      Set ExitAcptAmps =  0 to disable Amps based transition and only rely on EXIT_ACPT_DURATION timeout.
                                //      Set ExitAcptAmps = -1 to disable Amps based transition and rely on EXIT_ACPT_DURATION timeout
                                //                            or ADPT_ACPT_TIME_FACTOR adaptive duration.
                                //      Set ExitAcptAmps = Same value used for LIMIT_OC_AMPS if Overcharge mode is to be used.
                                // Note:  If both Time and Amps are set = 0, Acceptance will be bypassed.
                                // FUTURE:  EXIT_ACPT_DVDT  Add dV/dt exit criteria for Acceptance mode, need to decide what it is :-)

   // Overcharge mode is sometimes used with AGM batteries and occurs between Acceptance and Float phase.
   int LIMIT_OC_AMPS;         // During Overcharge phase, Amps are capped at this low value.  (Set this = 0 to disable OC mode.)
   float EXIT_OC_VOLTS;       // Overcharge will continue until the battery voltage reaches this level.
   uint32_t EXIT_OC_DURATION; // Over Charge mode duration in mS.
                              // ( as a safety step, setting OC_VOLTS or DURATION = 0 will also disable OC mode..)
                              // FUTURE:  EXIT_OC_DVDT  Add dV/dt exit criteria for Overcharge mode, need to decide what it is :-)

   float FLOAT_BAT_V_SETPOINT;   // Set point for Float battery voltage, do not exceed this voltage.
   int LIMIT_FLOAT_AMPS;         // During Float, manage system to keep Amps into Battery at or under this value.  May = 0, set = -1 to disable limit.
   uint32_t EXIT_FLOAT_DURATION; // Alternator will stay in Float mode this int32_t (in mS) before entering Post-Float (no charging) mode.  Set = 0UL disable transition to Post-float mode.
   int FLOAT_TO_BULK_AMPS;       // If Amps being delivered exceeds this value, we will assume a LARGE load has been placed on the battery and we need to re-enter
                                 // BULK phase.   Set this = 0 to disable re-entering BULK phase feature
   int FLOAT_TO_BULK_AHS;        // If the number of Ahs removed from the battery after 1st entering Float mode exceed this value, revert back to BULK.
                                 // Note this will ONLY be usable if the Amp shunt is at the battery.  Set = 0 to disable this feature.
   float FLOAT_TO_BULK_VOLTS;    // As with Amps, if the voltage drops below this threshold we will revert to Bulk.  Set = 0 to disable.

   uint32_t EXIT_PF_DURATION; // Only stay in Post_float mode (no charging) this amount of time.  Set = 0UL to disable times based Post-float exiting and exit only on Voltage.
   float PF_TO_BULK_VOLTS;    // If during Post-Float mode VBat drops below this voltage, re-enter FLOAT mode.
                              // Set = 0.0 to disable exiting of post-float mode based on voltage.
                              // Config note:  IF you configure the system to enter post-float mode from float-mode (by setting a time value EXIT_FLOAT_DURATION), AND you
                              //               set both EXIT_PT_DURATION and PF_TO_BULK_VOLTS = 0, the regulator will in effect turn off the alternator once charging is completed
                              //               and not restart a charge cycle until powered down and up again.  This can be useful if you truly want a one-time only charge.
                              //               You could also config the FEATURE-OUT port to indicate the complete charge cycle has finished, to say power-off the driving engine?
   int PF_TO_BULK_AHS;        // If the number of Ahs removed from the battery after 1st entering Post Float mode exceed this value, revert back to BULK.
                              // Note this will ONLY be usable if the Amp shunt is at the battery.  Set = 0 to disable this feature.

   float EQUAL_BAT_V_SETPOINT;   // If Equalize mode is selected, this is the target voltage.  Set = 0 to prevent user from entering Equalization mode.
   int LIMIT_EQUAL_AMPS;         // During equalization, system will limit Amps to this value.   Set = 0 to disable amp limits during Equalization Mode.
   uint32_t EXIT_EQUAL_DURATION; // Regulator will not stay in Equalization any longer then this (in mS).  If set = 0, then Equalization mode will be disabled.
   int EXIT_EQUAL_AMPS;          // If Amps fall below this value during Equalization while at VBat setpoint -- exit equalization.  Set = 0 to disable exit by Amps and use only time.

   float BAT_TEMP_1C_COMP;  // Battery Temperature is compensated by this factor for every 1C temp change.  Note this is based off of BAT_TEMP_NOMINAL (25c)
   int MIN_TEMP_COMP_LIMIT; // If battery temperature falls below this value (in deg-c), limit temp compensation voltage rise to prevent overvoltage in very very cold places.
   int BAT_MIN_CHARGE_TEMP; // If Battery is below this temp (in deg-c), stop charging and force into Float Mode to protect it from under-temperature damage.
   int BAT_MAX_CHARGE_TEMP; // If Battery exceeds this temp (in deg-c),  stop charging and force into Float Mode to protect it from over-temperature damage.
   uint8_t CPSPLACEHOLDER[8]; // Room for future expansion
} tCPS;

#define BAT_TEMP_NOMINAL 25    // Nominal temp which .BAT_TEMP_1C_COMP is based around (in deg-C).
#define PID_VOLTAGE_SENS 0.020 // When looking at charger mode transitions, if we come within 20mV of the target voltage (for rep 12v battery), consider we have 'met' that voltage condtion. 
                               //  (This is to help with smaller charging sources driving a LARGE battery, where the charging soruce is not really able to push VBat slightly over the limit..)

extern const tCPS defaultCPS[MAX_CPES] PROGMEM;

#endif /*  _CPE_H_ */

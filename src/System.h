//      System.h
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

#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include "Config.h"

typedef enum tModes {unknown   =0, disabled, FAULTED, FAULTED_REDUCED_LOAD, sleeping,                                           // Used by most or all  (0..9)
                     warm_up=10, ramping, determine_ALT_cap,   //post_ramp=15,  
                     bulk_charge=20, acceptance_charge, overcharge_charge,  float_charge=30, forced_float_charge, LIFEPO_FORCED_SHUTDOWN, post_float=36, equalize=38, //RBM_CVCC,
                                                                                                                                // Alternator / Charger Mode specific (10..39)
                                                                                                                                //  (BMS uses a subset for chargerMode, keeping order to simplify external ASCII apps.)
                    // discharge=40, recharge, LTStore, holdSOC,                                                                  // BatMon specific  (4x)
                    // stopped  =50, stopping, starting, pre_warming, warming, running,    priming_oil=59,                        // Generator engine specific  (5x)
                    // pending_WM=60, WM_enabled, post_WM                                                                         // Watermaker (Aug load for DC Gen)  (6x)
                    } tModes;                                                                                                   // Take care not to change the order of these, some tests use
                                                                                                                                // things like <= pending_R
                    
//----- The tSCS structure is used to hold many global 'system' configurations values.  By placing them into a Structure (as opposed to
//      using #define statements) there is the ability to modify them during run-time, and even save them into FLASH.

typedef struct  {                                               // System Configuration Structure used by Alternator and DC Generator

   bool         REVERSED_BAT_SHUNT;         // Is the BAT AMP Shunt wired up backwards?  If so, this Flag will invert the readings via Software.  
   bool         REVERSED_ALT_SHUNT;         // Is the ALT AMP Shunt wired up backwards?  If so, this Flag will invert the readings via Software. 
                                            //------  Default values for Alternator Goals (Amps, temp, etc..)
   uint8_t      ALT_TEMP_SETPOINT;          // Temp range we want the Alternator to run at (max). Anything above it, start backing off Alternator Field drive.
   float        ALT_AMP_DERATE_NORMAL;      // In normal operation, run the Alternator this % of it  demonstrated max capability
   float        ALT_AMP_DERATE_SMALL_MODE;  // But if user has selected Small Alternator Mode, then scale back to a lower %
   float        ALT_AMP_DERATE_HALF_POWER;  // If user shorts out the Alt Temp probe, we will then use this value to derate.
   int          ALT_PULLBACK_FACTOR;        // Exponential factor in RPM based pull-back factor; to protect Alts by reducing max field PWM at low RPMs
                                            // How many RPMs are needed to increase idle-pull-back 1% above 1/4 power at idle.
                                            // Range 0..10, set = 0 to disable pullback feature.  See set_VAWL() in manage_ALT();
                                            // Set = -1 to trigger fixed 70% cap when stator signal is not seen.
   int          ALT_IDLE_RPM;               // Used in conjunction with PBF to manage Field Drive at lower RPMs.  Establishes the 'starting point' for 1/4 field drive.
                                            // Range 0..1500, set = 0 to enable auto determination of Idle RPMs. 
   
   int          ALT_AMPS_LIMIT;             // The regulator may OPTIONALLY be configured to limit the maximum number of AMPs being drawn from the alternator.
                                            // For example if you have small wire sizes in the alternators wiring harness. Place the size of the Alternator here. 
                                            // If this is set = -1, the regulator will enable a auto-size-determining feature and during Bulk phase it will for
                                            // short periods of time drive the alternator very hard to measure its Amps viability.  (this is triggered any time an
                                            // increase in noted Amps exceeds what was noted before, OR if an increase in RPMS is noted..)
                                            // CAUTION:  During Auto-size phase Full-Drive will occur EVEN if the half-power mode has need selected (by shorting the Alt NTC sender wires)
                                            // Set this = 0 to disable AMPS based management of the alternator, the regulator will simply drive the Alternator as hard as it can.
                                            // CAUTION:  If Amps based management is disabled, then features such as Small Alternator, and Half Power will no longer function.
                                            //          You also run the risk of driving the FIELD at Full Field, with no increase in Alternator output - causes excessive heating in the field.



   int          ALT_WATTS_LIMIT;            // The regulator also has the ability to limit the max number of Watts load being placed on the engine.  For example if you
                                            // are using a large alternator on a small engine.  To enable this feature, place the max # of WATTS you want the alternator 
                                            // to deliver here and the regulator will cap total engine load to this value.  Note this is loosely related to ALT_AMPS_LIMIT,
                                            // but Watts is a true measurement when talking about loads being placed on engines, as Amps do not take into account the 
                                            // current battery voltage..
                                            // Set this to 0 to disable this feature let the regulator drive to the max capability of the Alternator (or the defined 
                                            // ALT_AMPS_LIMIT above)
                                            // Set = -1 to auto-calc the Watts limit based on the determined (or defined) size of the Alternator and the current target charge voltage.

                                            // Note that ALT_WATTS_LIMIT and ALT_AMPS_LIMIT values are NOT adjusted by the sensing of the battery nominal voltage, If you enter 200A and
                                            // 5000W, the regulator will cap at 200A or 5000W independent of the battery voltage...

               // ------ Parameters used to calibrate RPMs as measured from Alternator Stator Pulses
               //        IF you wish to read 'engine RPMs' from the regulator, you will need to adjust these.  
               //        But w/o any adjustment the regulator will function correctly, for its purpose.
   uint8_t      ALTERNATOR_POLES;           // # of poles on alternator 
   float        ENGINE_ALT_DRIVE_RATIO;     // engine pulley diameter / alternator diameter



                //----   Controller Hardware configuration parameters   Here hardware options are noted.  See Blog for more details
   int          BAT_AMP_SHUNT_RATIO;        // Spec of battery amp shunt, Amp/mV * % Calibration error (e.g:   250A / 75mV shunt * 100% -->  250 / 0.075 * 1.00 = 3,333) 
                                            //  Note if Shunt voltage will exceed 80mV, then the INA-226 calibration will need to be adjusted

   int          ALT_AMP_SHUNT_RATIO;        // Spec alternator of amp shunt, Amp/mV * % Calibration error (e.g:   250A / 75mV shunt * 100% -->  250 / 0.075 * 1.00 = 3,333) 
                                            //  Note if Shunt voltage will exceed 80mV, then the INA-226 calibration will need to be adjusted.

   int          FIELD_TACH_PWM;             // If the user has selected Alternator Tach mode via the DIP switch, then this value will be used for the minimum Field PWM
                                            // (As opposed to 0).  If this is set = -1, then during initial RAMP the field value to use for Tach mode will be auto-determined
                                            // and set to a value just before Amps are produced by the alternator.  Set = 0 to disable Tach mode, independent of the DIP switch.

   bool         FORCED_TM;                  // Has user Forced on Tach-mode via the $SCT command

   uint8_t      CP_INDEX_OVERRIDE;          // User has used issues command to override the DIP switches for the cpIndex.      -1 = use DIP switches.
   float        BC_MULT_OVERRIDE;           // User has used issues command to override the DIP switches for the bcMultiplier.  0 = use DIP switches.
   float        SV_OVERRIDE;                // User forced Voltage Multiplier (1..4) associated with 12v..48v.  Use 0 to enable auto-detect feature.
   uint8_t      CONFIG_LOCKOUT;             // 0=no lockout, 1=no config change, 2=no change, no clearing via FEATURE-IN. 
   int          ENGINE_WARMUP_DURATION;     // Duration in seconds alternator is held off at initial power-on before starting to apply load to engine (Start the RAMP phase)
   uint8_t      REQURED_SENSORS;            // Flags to indicate the regulator should check if some sensors are not present, ala Alt Temp, battery shunt, etc..
   
   uint8_t    SCSPLACEHOLDER[15];         // Room for future expansion   
   } tSCS;
                  
   // Bit fields for REQUIRED_SENSORS above and global variable: requiredSensorsFlag
const uint8_t RQAltTempSen = 0x01;    // Alternator Temp Sensor Required  -- Go to half-power mode if missing
const uint8_t RQBatTempSen = 0x02;    // Battery Temp Sensor Required     -- Force into float if missing
const uint8_t RQAmpShunt   = 0x04;    // Amp shunt Required               -- Fault if missing
const uint8_t RQFault      = 0x80;    // If any of the required sensors are missing, just force a FAULT vs. taking the 'default' action.

         
   //----    Error codes. If there is a FAULTED status, the variable errorCode will contain one of these...
   //              Note at this time, only one error code is retained.  Multi-faults will only show the last one in the checking tree.
   //              Errors with + 0x8000 on them will cause the regulator to re-start, others will freeze the regulator.
   //              (Note combinations like 10, and 11 are not used.  Because one cannot flash out 0's, and kind of hard to 
   //               tell if 11 is a 1+1, or a real slow 2+0)
#define FC_LOOP_BAT_TEMP                12              // Battery temp exceeded limit
#define FC_LOOP_BAT_VOLTS               13              // Battery Volts exceeded upper limit (measured via INA226)
#define FC_LOOP_BAT_LOWV                14  + 0x8000U   // Battery Volts exceeded lower limit, either damaged or sensing wire missing. (or engine not started!)

#define FC_LOOP_ALT_TEMP                21              // Alternator temp exceeded limit
#define FC_LOOP_ALT_RPMs                22              // Alternator seems to be spinning way to fast!
#define FC_LOOP_ALT_TEMP_RAMP           24              // Alternator temp reached / exceeded while ramping - this can NOT be right, to reach target while ramping means way too risky.


#define FC_LOG_ALT_STATE                31              // Global Variable chargingState has some unsupported value in check_for_faults() 
#define FC_LOG_ALT_STATE1               32              // Global Variable chargingState has some unsupported value in manage_ALT() 
#define FC_LOG_CPI_STATE                33              // Global Variable cpIndex has some unsupported value in caculate_ALT_Targets() 
#define FC_LOG_CPINDEX                  34              // Global Variable cpIndex has some unsupported value in check_for_faults() 
#define FC_LOG_SYSAMPMULT               35              // Global Variable systemAmpMult has some unsupported value in check_for_faults() 
#define FC_LOG_BAT_STATE1               36              // Global Variable chargingState has some unsupported value in manage_BAT()    
        
#define FC_SYS_FET_TEMP                 41              // Internal Field FET temperature exceed limit.
#define FC_SYS_REQIRED_SENSOR           42              // A 'Required' sensor is missing, and we are configured to FAULT out.
#define FC_ADC_READ_ERROR               71              // Internal error - unable to use ADC subsystem

#define FC_BAT_INA226_READ_ERROR        100 + 0x8000U   // Returned I2C error code is added to this, see I2C lib for error codes.
#define FC_ALT_INA226_READ_ERROR        200 + 0x8000U   // Returned I2C error code is added to this, see I2C lib for error codes.


//---  Global vars and prototypes

extern tSCS     systemConfig;

extern tModes   chargingState;
//extern tModes   systemState;
//extern tModes   batteryState;

extern unsigned faultCode; 
extern uint8_t  requiredSensorsFlag;
extern char const firmwareVersion[];

bool    feature_in(bool waitForDebounce);
void    handle_feature_in(void);
void    update_feature_out(void);
bool    check_for_faults(void) ;
void    handle_fault_condition(void);
void    single_engine_stop_pulse(void);

#endif   /*  _SYSTEM_H_  */

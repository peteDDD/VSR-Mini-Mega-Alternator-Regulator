//
//      SmartRegulator.h
//
//      Copyright (c) 2018 by William A. Thomason.      http://arduinoalternatorregulator.blogspot.com/
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

#ifndef _SMARTREG_H_
#define _SMARTREG_H_

/****************************************************************************************
 ****************************************************************************************
 *                                                                                      *
 *                              CUSTOMIZING PARAMETERS                                  *
 *                                                                                      *
 *                                                                                      *
 *      Change the following parameters to meet the exact needs of your installation.   *
 *                                                                                      *
 *      These are more advanced settings than those in Config.h                                                                               *
 ****************************************************************************************
 ****************************************************************************************/

//#define BENCHTEST  // shortens the warm-up and ramping time to make testing faster

//#define FIRMWARE_VERSION "AREG1.3.1" // from original source code
#define  FIRMWARE_VERSION  "AREG1.3.M"     // Sent out with SST; status string (as well as the CAN product ID if implemented)
                                                // Use format of xxxxyyyyyy  where xxxx= 4 chars of device type, and yyy= varying chars of version number.
                                                // Note the yyy's are to follow the Semantiic Version spec guidelines:  https://semver.org/
                                                // Make sure to use `-' for any trailing modifiers after the final version number 'number'
#define REV_FORK "MiniMega v1.0"
#define DATE_CODE "10/31/21"

#include "Config.h"
// --------  BOARD SPECIFIC SELECTIONS
//          This source is optimized for the Arduino Mini Mega Alternator Controller board using
//          the Robotdyn Mini Mega 2560 Pro 5V processor board as designed by Pete Dubler in 2021.
//          The processor board is available from:
//             DIYMore: https://www.diymore.cc/products/mini-mega-2560-pro-micro-usb-ch340g-atmega2560-16au-for-arduino-mega-2560-r3
//             or from RobotDyn https://robotdyn.com/mega-2560-pro-embed-ch340g-atmega2560-16au.html
//         
//          NOTE THAT THE platformio.ini file must have the following lines:
//                ; repoint variant & variant folder to the project-local one.
//                board_build.variants_dir = variants
//                board_build.variant = mcupro
//
//          AND THAT A variants directory must be added to the root of the project code and
//            that the MCUPRO directory must be put under teh variants directory.  
//          REFER TO THE BUILD ENVIRONMENT INSTRUCTIONS IN THE README DIRECTORY AT
//!! NEED TO ADD GITHUB DIRECTORY HERE 
//
//          The #ifdef directly below confirms that the compiler is set for the correct processor
//          
//          If one wanted to adapt this code for another processor or alternator controller board
//          additional #ifdefs could be added to make the appropriate port selections
//          for I/O ports and additional include files as needed.
//
//          Note that ALL I/O PORTS are to be defined here, in one common location.
//          This will help avoid issues with duplicate errors...


#if defined(__AVR_ATmega2560__)
#include "Portability.h" // Pick up all the IDE specifics, ala PROGMEM

#define OPTIBOOT // Compile this code for use with Optiboot bootloader?  If so, fully enable the Watchdog timer.         
                 // If Optiboot is not used, and the 'default' Arduino IDE bootloader is used, there are some workarounds 
                 // needed to make reboot() function, and if the watchdog is ever triggered the regulator will hang vs.                    // restart.  This is a fault / bug in the basic Arduino bootloader for 3.3v + atMega328.  See blog for more details.


//---- start of MINI MEGA 2560 EMBED VERSION pin config
#define FEATURE_OUT_PORT1 27 // Drives Feature-out open-collector port.
#define FEATURE_OUT_PORT2 29 // Drives Feature-out open-collector port.
#define FEATURE_OUT_PORT3 31 // Drives Feature-out open-collector port.
#define FEATURE_IN_PORT1 12  // PB6 is also PCINT6
#define FEATURE_IN_PORT2 13  // PB7 is also PCINT7
#define FEATURE_IN_PORT3 14  // PJ1 is also PCINT10

#define LED_ON_PCB_REVERSED  // In case our friends at PCBWay.com got it wrong
#ifndef LED_ON_PCB_REVERSED
  #define LED_GREEN_PCB A7      // Connected to status LED on the PCB.
  #define LED_RED_PCB A5        // Connected to status LED on the PCB.
#else
  // Red and Green reversed on first proto
  #define LED_GREEN_PCB A5      
  #define LED_RED_PCB A7 
#endif       
//
#define LED_GREEN_EXT A3    // Connected to status LED external connector.
#define LED_RED_EXT A1      // Connected to status LED external connector.
                              ////    #define CHARGE_PUMP_PORT                 6              // PWM port that drives the FET boost-voltage Charge Pump
#define SDA_PIN SDA
#define SCL_PIN SCL
#define I2C_TIMEOUT 20 // timeout after 20 msec -- do not wait for clock stretching longer than this time

#define RX1 RXD1
#define TX1 TXD1
#define RX2 RXD2
#define TX2 TXD2

// DIP Switch bits directly wired to digital port
#define DIP_BIT0 4  //PG5 SWITCH DIP-1
#define DIP_BIT1 5  //PE3 SWITCH DIP-2
#define DIP_BIT2 6  //PH3 SWITCH DIP-3
#define DIP_BIT3 7  //PH4 SWITCH DIP-4
#define DIP_BIT4 8  //PH5 SWITCH DIP-5
#define DIP_BIT5 9  //PH6 SWITCH DIP-6
#define DIP_BIT6 10 //PB4 SWITCH DIP-7
#define DIP_BIT7 11 //PB5 SWITCH DIP-8

#define DIP_MASK 0xFF // There are 8-bits usable in this version.
#define MASTER_RESTORE_DIP_CHECK 0b10000000 //  ONLY do Master Restore with FEATURE_IN_RESTORE asserted at startup
                                            //  if DIP 8 is selected

#define NTC_FET_PORT A0 // Onboard FET temperature sensor
#define NTC_ALT_PORT A2 // Alternator NTC port
#define NTC_BAT_PORT A8 // Battery NTC port
#define NTC_AVERAGING 15 // There are up to 3 NTC sensors, and we will average 5 A/D samples each before converting to a temperature, to smooth things over. (Max 254) 
                         // Note that this, combined with SENSOR_SAMPLE_RATE will define how often the temperatures get updated...

#define STATOR_IRQ_NUMBER 4 // Stator IRQ is attached to arduino pin 2, INT-4, OSC3B, PE4

#define FIELD_PWM_PORT 3    // Field PWM is connected to arduino pin 3, PE5, OSC3C
#define set_PWM_frequency() TCCR3B = (TCCR3B & 0b11111000) | 0x04; // Set Timer 3 (Pin 2/3/5 PWM) to 122Hz (from default 488hz).  This more matches 
                                                                   // optimal Alternator Field requirements, as frequencies above 400Hz seem to send
                                                                   // ref: https://arduinoinfo.mywikis.net/wiki/Arduino-PWM-Frequency
#define FIELD_PWM_MAX 0xFF // Maximum alternator PWM value.


//end of MINIMEGA pin config


#define SYSTEM_BAUD 9600UL             // Nice and slow Baud, to assure reliable comms with local RN-41 Bluetooth module + future expansion.



#else
#error Unsupported Smart Regulator CPU
#endif

//
//
//------------------------------------------------------------------------------------
//
//      The Following are internal parameters and typically will not need to be changed.
//
//

// -----   Critical fault values: exceeding these causes system fault and fault handler.
//         Note that these values are set for a 'normalized - 12v / 500Ah battery', and will be automatically scaled during runtime
//         by the regulator after sampling the battery voltage and setting the variable "systemVoltMult"
//
#define FAULT_BAT_VOLTS_CHARGE 16.5   // This is where we will fault.  Make sure you have sufficient headroom for Over Charge voltage (if being used)
#define FAULT_BAT_VOLTS_EQUALIZE 18.0 // When doing Equalization, allow a higher limit.
#define FAULT_BAT_VOLTS_LOW 8.0       // Anything below this means battery is damaged, or sensing wire is missing / failing.
#define FAULT_BAT_TEMP 60             //  At or above this, fault out. (Approx 140F)
#define FAULT_ALT_TEMP 1.1            //  Fault if Alt Temp exceeds desired set point by 10%
#define FAULT_FET_TEMP 70             // If Field driver FETs are over 80c (Approx 160f), something is wrong with the FETs - fault.

#define ADPT_ACPT_TIME_FACTOR 5 // If the regulators is operating in Adaptive Acceptance Duration mode (either because EXIT_ACPT_AMPS was set = -1, or                   
                                // if we are unable to measure any amps), the amount of time we spend in Bulk phase will be multiplied by this factor, and               
                                // we will remain in Acceptance phase no longer then this, or EXIT_ACPT_DURATION - whichever is less.  This is in reality a backup       
                                // to Amp based decisions, in case the shunt is not installed, or fails.  Or in the case where the install never uses the Amp shunt, and 
                                // charging is started with a full battery - do not want to boil off the battery.

// ------ Parameters used to calibrate RPMs as measured from Alternator Stator Pulses
#define RPM_IRQ_AVERAGING_FACTOR 100 // # sector pulses we count between RPM calculations, to smooth things out.  100 should give 3-6 updates / second as speed.
#define IRQ_TIMEOUT 10               // If we do not see pulses every 10mS on average, figure things have stopped.
#define IDLE_SETTLE_PERIOD 10000UL   // While looking for a potential new low for idleRPMs, the engine must maintain this new 'idle' period for at least 10 seconds.

//----  PWM Field Control values
#define FIELD_PWM_MIN 0x00 // Minimum alternator PWM value - It is unlikely you will need to change this.
// #define FIELD_PWM_MAX           -------              // Maximum alternator PWM value - (This is now defined in the CPU specific section of xxxx.h, SmartRegulator.h, SmartGen.h, etc...)
#define MAX_TACH_PWM 75  // Do not allow MIN PWM (held in 'thresholdPWMvalue') to go above this value when Tach Mode is enabled.  Safety, esp in case Auto tech mode is enabled. 
                         //  This same value is used to limit the highest value the user is allowed to enter using the $SCT ASCII command.
#define PWM_CHANGE_CAP 2 // Limits how much UP we will change the Field PWM in each time 'adjusting' it.

#define PWM_CHANGE_RATE 100UL // Time (in mS) between the 'adjustments' of the PWM.  Allows a settling period before making another move.

#ifdef BENCHTEST
#define PWM_RAMP_RATE 40UL // use fast ramp rate for bench testing
#else
#define PWM_RAMP_RATE 400UL   // When ramping, slow the rate of change down to update only this often (in mS) 
                              //    This combined with PWM_CHANGE_CAP will define the ramping time.            
                              //    (for PWM to reach the FIELD_PWM_MAX value and exit Ramp).
#endif
// These are used to count down how many PWM_CHANGE_RATE cycles must pass before we look at the
// these slow changing temperature values.  (TA = Alt)
// If any of these are over value, we will adjust DOWN the PWM once every xx times through adjusting PWM.

//---- The following scaling values are used in the PID calculations used in manage_ALT()
//     Factors are adjusted in manage_alt() by systemMult as needed.
//     Values are represented in 'Gain' format, and MUST be defined as floating values (e.g.  30.0  vs. 30) in order to keep each component of the
//      PID engine working with fractions (each small fractional value is significant when summing up the PID components)
#define KpPWM_V 20.0
#define KiPWM_V 10.0
#define KdPWM_V 75.0

#define KpPWM_A 0.6
#define KiPWM_A 0.3
#define KdPWM_A 0.7

#define KpPWM_W 0.05
#define KiPWM_W 0.0
#define KdPWM_W 0.02

#define KpPWM_AT 2          // (These are OK as int consts, as the AT error calcs are done using ints)
#define KdPWM_AT 20         // D for temps are only applied to pullback as we approch target.
#define TAM_SENSITIVITY 100 // Pace the updating of the dT to every 100x PWM_CHANGE_RATE (or ~every 10 second)  (Max 126) 
                            // Note that the D factor IS applied each manage_alt() loop, it is only updated ever 10x -    
                            // Which also means once updated it will continue to be applied for all the  loops until the next update.

#define PID_I_WINDUP_CAP 0.9 // Capping value for the 'I' factor in the PID engines.  I is not allowed to influence the PWM any more then this limit 
                             // to prevent 'integrator Runaway' .

//---- Load Dump / Raw Overvoltage - over temp -  detection thresholds and actions.
//     (These action occur asynchronous to the PID engine -- handled in real time linked to the ADCs sampling rate.)

#define LD1_THRESHOLD 0.040 // During a Load Dump situation, VBat can start to rise VERY QUICKLY.   Too quick for the PID engine
#define LD2_THRESHOLD 0.080 // in the case of large (200A+) alternators.  If measured voltage exceeds target voltage by these thresholds
#define LD3_THRESHOLD 0.100 // the PWM is pulled back.   Once these brute force changes are made, the normal PID engine can start 
                            // adjusting things back to the new situation.  PROTECT THE BATTERY IS #1 CRITERIA!!!!

#define LD1_PULLBACK 0.95 // On 1st sign of overvoltage, pull back the field a little.
#define LD2_PULLBACK 0.85 // On 2nd sign of overvoltage, pull back harder 
                          // LD 3 overvoltage holds the Field at 0 until the overvoltage situation clears.

#define AOT_PULLBACK_THRESHOLD 1.03 // If we find we are 3% over alternator temperature goal, the PID engine is not keeping up.
#define AOT_PULLBACK_RESUME 0.90    //  (And do not resume normal operation until we are at 90% of goal temperature or less)
#define AOT_PULLBACK_FACTOR 0.50    // Do something dramatic, cut field drive by 50% before it continues to rise and we fault out.

// Any time we go over-temp, it seems we have a condition
// of a fight between that temp value and the amp/watts target - creating an osculation situation.
#define OT_PULLBACK_FACTOR 0.95       // When triggered, we will pull down the Watts target and max PWM limit this ratio to try and self correct.
#define OT_PULLBACK_FACTOR_LIMIT 0.60 // Don't pull back more then this, if we are unable to correct with this much pullback - let the system fault out 
                                      // as install is very very wrong.

#define USE_AMPS_THRESHOLD 5 // If at some time we do not measure AT LEAST this amount of Amps, then we will ASSUME the Amp Shunt is not    
                             // connected - this will cause Manage_ALT() to ignore any Amps based thresholds when deciding if it is time to 
                             // transition out of a given charging phase.  It is a way for the regulator to act in Voltage Only mode - just 
                             // do not connect up the shunt!  (Actually, would be better to put a wire jumper across the Shunt terminals)

// When deciding to change Alternator charge states, and adjust the throttle, we use persistent Watts and Amps.
// These are averaged values over X periods.  These are used to smooth changes in
// Alternator State modes - to allow for short term bumps and dips.
#define AMPS_PERSISTENCE_FACTOR 256
// Set = 1L to disable  (Used to exit Acceptance and Float modes)
#define VOLTS_PERSISTENCE_FACTOR 300 // Volts will be averaged over this number of samples at "PWM_CHANGE_RATE". (300 = ~1/2 min look-back) 
                                     // Set = 1 to disable  (Used to exit post_float mode)

// Desensitizing parameters for deciding when to initiate a new Alternator Capacity Measurement cycle,
#define SAMPLE_ALT_CAP_RPM_THRESH 250         // Initiate a new Capacity Sample if we have seen in increase in RPMs / Amps from the prior reading.
#define SAMPLE_ALT_CAP_AMPS_THRESH_RATIO 1.05 // To keep from being too twitchy ..

// --------  INA226 Registers & configuration

#define INA226_Bat_I2C_ADDR 0x40 // I2C addresses of INA226 device. Measures Amp shunt, and Vbat+   DUBLER: 0x40 = decimal 64 = A0 and A1 tied to ground
#define INA226_Alt_I2C_ADDR 0x45 // I2C address for second INA226 board to measure ALTERNATOR SHUNT AND Valt+.    0x45 = decimal 69 = A0 and A1 tied to Vs

#define CONFIG_REG 0x00 // Register Pointers in the INA-226
#define SHUNT_V_REG 0x01
#define VOLTAGE_REG 0x02
//#define POWER_REG                0x03 // Because I am doing raw voltage reads of the Shunt, no need to access any of the INA-226 calculated Amps/power
//#define SHUNT_A_REG              0x04
//#define CAL_REG                  0x05 // Nor the Cal reg
#define STATUS_REG 0x06
#define LIMIT_REG 0x07
#define DIE_ID_REG 0xFF // Unique 16-bit ID for the part.

#define INA226_CONFIG 0x4523                          // Configuration: Average 16 samples of 1.1mS A/Ds (17mS conversion time), mode=shunt&volt:triggered
#define INA226_PD_CONFIG (INA226_CONFIG & 0xFFF8)     // Mask out the power-down bits.
#define I2C_TIMEOUTms 100                             // If any given I2C transaction takes more then 100mS, fault out.
#define INA226_TIMEOUTms 100                          // If it takes more then 100mS for the INA226 to complete a sample cycle, fault out.
#define INA226_SAMPLE_TIMEOUTms 2 * INA_SAMPLE_PERIOD // If something prevents us from initating a voltas/amps (ina226) sample cycle, fault out.
#define INA226_VOLTS_PER_BIT 0.00125              // 125mV per bit
#define INA226_VOLTS_PER_AMP 0.0000025            // 2.5uV per amp (before shunt scaling factor)
// ------ Values use to read the Feature-in port to handle debouncing.
#define DEBOUNCE_COUNT 5 // We will do 5 samples of the Feature_in port to handling any de-bouncing.
#define DEBOUNCE_TIME 1  // And if asked for, we will block the system for 1mS between each of those samples in order to complete a 
                         // debounce cycle when feature_IN() is called.  Do not make this too large, as it is a raw delay.          
                         // And remember, there is an external R/C to help filter any bouncing before we even see it.

// ------ Values use for the NTC temp sensors.
#define NTC_RO 10000       // Nominal resistance of NTC sensors at 25c
#define NTC_BETA_ALT_AND_BAT 3950      // Most common Beta for probes on EBay is 3950.
#define NTC_BETA_FETs 3380 // Beta of NTC chip used for FET sensing.
#define NTC_RF 10000       // Value of 'Feed' resister
#define NTC_RG 100         // Value of 'Ground Isolation' resister - used only on the external NTC probes.
#define NTC_OUT_OF_RANGE_HIGH 120 // If over this temperature, something is wrong with the sensor
#define NTC_OUT_OF_RANGE_LOW  -40 // If under this temperature, something is wrong with the sensor
#define NTC_SHORTED 160 // If temperature is this high, the sensor is shorted.  (Used as method for user to select alternator half load)

// ----- Mainloop timing values, how often do we update the PWM, check for key pressed, etc.
//         All times are in mS
#define SENSOR_SAMPLE_RATE 50UL         // If we are not able to synchronize with the stator, force a sample of Volts, Amps, Temperatures, every 50mS min.
#define ACCUMULATE_SAMPLING_RATE 1000UL // Update the accumulated AHs and WHs  every 1 second.
#define SAMPLE_ALT_CAP_DURATION 10000UL // When we have decided it is time to sample the Alternators capability, run it hard for 10 seconds.
#define SAMPLE_ALT_CAP_REST 30000UL     // and give a 30 second minute rest period between Sampling Cycles.
//#define OA_HOLD_DURATION          60000UL               // Hold on to external offset amps (received via $EOA command) for only 60 seconds MAX.
// REDACTED 2-26-2018

//----  RTOS controls and flags  (If RTOS is being used)
#define NTC_SAMPLE_PERIOD 500 // Sample the NTC sensors every 0.5 seconds
#define INA_SAMPLE_PERIOD 50  // Sample the INA226 (Volts/AMps) sensors every 50mS

// --------  Global Timouts
#define WDT_PERIOD WDTO_8S   // Set the Watchdog Timer for 8 seconds (Lock step this with CHECK_WDT_mS below)
#define CHECK_WDT_mS 8000 // Used by validation in pre-compile checks for errors in critical timing.  Make this the # mS the WDT is set for.
#define I2C_TIMEOUT 20    // Bail on faulted I2C functions after 20mS

#define RVC_CHARGER_TYPE RVCDCct_Alternator // This OSEnergy device is an Alternator

/* Useful Constants */
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)
 
/* Useful Macros for getting elapsed time */
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN) 
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)

//---  Functions and global vars exported from SmartRegulator.ino
uint8_t readDipSwitch(void);
int checkStampStack(void);
void reboot(void);

extern uint8_t requiredSensorsFlag;

#endif // _SMARTREG_H_

//      Alternator.cpp
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

#include "Config.h"
#include "System.h"
#include "LED.h"
#include "Alternator.h"
#include "OSEnergy_Serial.h"
#include "Sensors.h"
#include <math.h>

int inChargingStateCount; // seconds left in warmup
uint32_t inChargingStateTime;  // count up milliseconds in current state
const char *chargingStateString;

#ifdef ENABLE_FEATURE_IN_SCUBA
bool scubaMode = false;
const char *scubaModeString = "     ";  // Start with blank string
#endif


//---  The core function of the regulator is to manage the alternators field in response to several criteria, e.g.:  battery voltage, system load, temperature, engine RPMs., etc..
//      (Simple regulators typically only look at voltage).  For this there are three key variables:  The current field drive value, as well as an upper limit, or cap which can be set
//      to limit alternator output for some reason (e.g, we are at idle RPMs and need to lessen the load to the engine).  There is also a 'floor' value typically used to assure the field
//      is being driven hard anough to allow for stator pulses to be produced and recognized by the stator IRQ ckt.  Here are the critical variables to accomplish all this:
int fieldPWMvalue = FIELD_PWM_MIN;     // How hard are we driving the Alternator Field?  Start not driving it.
int fieldPWMLimit = FIELD_PWM_MAX;     // The upper limit of PWM we should use, after adjusting for reduced power modes..  Note that during auto-sizing, this will be
                                       // reset to FIELD_PWM_MAX, as opposed to constrained values from user selected reduced power modes.
int thresholdPWMvalue = FIELD_PWM_MIN; // This contains the PWM level needed to provide for a Stator IRQ sufficient to produce a stable RPM measurement.
                                       // It may be pre-defined by the user (systemConfig.FIELD_TACH_PWM).  Or if the user has asked for auto-determination -
                                       // while operating (typically during RAMP phase) once there is sufficient stability in the Stator IRQ signal to allow
                                       // calculation the then current PWM value is noted. If TACH mode is enabled, this becomes the floor for the lower limit of
                                       // PWM drive.  Hopefully keeping the tach alive at all times.
                                       // Will contain -1 if we have not seen a stator IRQ signal (and the user wants to look for one.)

int APUCapPWM;                           // Stashed snap-shot of fieldPWMLmit after most calcs but before CAN stuff is added in.  Used by ALT_Per_Util()
int APUAdjAT;                            // And stached snap-shot of any PWM adjustments beign done via Alternator Tempeture PID. Used by ALT_Per_Util()
bool static otPullbackTriggered = false; // Awk, ALSO used by ALT_Per_Util(), so had to move them here.....
float static otPullbackFactor = 1.0;     // If we detecte an Over Temp condition in this charge cycle, we want to reduce the load on the engine and
                                         // try and help cool things off.  THIS variable is used to accomplish this
                                         // by adjusting the targetAltWatts before calculating the watts error in manage_alt.
                                         // fieldPWM  is also adjsuted by this pull-back factor.
                                         // Initialize with no pull-back factor (= 1.0) each time a new charging cycle is started.

//---  Timing variables and smoothed voltages and current values - used to pace and control state changes
uint32_t lastPWMChanged;      // Used to give a settling time when the PWM is altered.
uint32_t altModeChanged;      // When was the charging mode was last changed.
uint32_t rampModeEntered = 0; // When did we enter Ramp - to allow check in BULK in case ramping for short-changed we will contiue the soft ramp up.

uint32_t adptExitAcceptDuration = 4 * 3600000UL; // If we will be doing 'adaptive time' based exiting of Acceptance mode, this holds the amount of time we should remain in
                                                 // acceptance mode.  It is calculated in manage_alt() and is set at ADPT_ACPT_TIME_FACTOR * the duration we were in BULK phase.
                                                 // Initializing it for 4 hrs in case we have some condition where we never were in Bulk mode and went directly into Accept
                                                 //  (Example via an ASCII command, this is a safety fall-back)

float persistentBatAmps = 0;    // Used to smooth alternator mode changes.  Usually the same as measuredAltAmps, unless user has overridden this via $EOR command.
float persistentBatVolts = 0.0; // Used in post-float to decide if we need to restart a charging cycle.
uint32_t modeChangeASecs = 0;   // Snapshot of value in accumulatedASecs when we Charger_status is changed.  Used to calc AHs withdrawn from the battery to check
                                // against AHs exits in the CPEs.

//---   Tachometer veriables.  Driven from the Stator IRQ ckt.
volatile uint32_t IRQDuration = 0;   // Time (in uS) for RPM_IRQ_AVERAGING_FACTOR IRQs to occure.  Used to calc RPMs.
volatile bool statorIRQflag = false; // Used by read_sensors() and IQR_vector() to lock-step INA226 sampling with stator pulses Stator
int measuredRPMs = 0;                // Current measured RPM of Engine (via the alternator, after converting for belt diameter).  Contains 0 = if the RPMs cannot be measured.
                                     //    Note these are incremented in the IRQ handler, hence Volatile directive.
bool tachMode = false;               // Has the user indicated (via the DIP Switch) that they are driving a Tachometer via the Alternator, and hence
                                     // we should always give some small level of Field PWM??

//---   Targets which are 'regulated' towards:
int altCapAmps = 0;     // This will contain the capacity of the Alternator, either determined by auto-sizing or as declared to use by the user.
int altCapRPMs = 0;     // If we did an auto-sizing cycle, this will be the high-water mark RPMs  (= 0 indicates we have not yet measured the capacity)
                        // Default deployment is assumed to be Battery Centric; hence these two limits are disabled.  Note that reduced power modes will
                        // directly adjust the PWM duty-cycle, to approx things like Half-Power mode, etc.
int targetAltWatts = 0; // Where do we want to be.  Will me adjusted for charger mode and battery temp.
float targetBatVolts = 0;
float targetAltAmps = 0;
float targetBatAmps = 1000; // (This is a place-holder, to help simplify a common OSEnergy_CAN.cpp file.
                            // We do not know how much the battery can take, so just use this 1,000A max value we have used elsewere

//---   States and Modes -  defines or modifies various functions behaviors
bool smallAltMode = false; // Has the user indicated (vita the DIP Switch) they want us to run in the Small Alternator mode?  If so, we will assume the
                           //   alternator has limited heat dispersion capability and will as such reduce its output to a max of ALT_AMP_DERATE_SMALL_MODE
                           //   of its capability (as auto-measured, or defined by the user).
tCPS chargingParms;  // Charge Parameters we are currently working with, appropriate entry is copied from EEPROM or FLASH during startup();
uint8_t cpIndex = 0; // Which entry in the chargeParms structure should we be using for battery setpoints?  (Default = 1st one)

float systemAmpMult = 1; // Multiplier used to adjust Amp targets for the charging setpoints?
                         //      i.e.:   systemBatMult = 1 for <500Ah          battery
                         //      i.e.:   systemBatMult = 2 for  500   - 1000Ah battery
                         //      i.e.:   systemBatMult = 3 for 1000Ah - 1500Ah battery
                         //      i.e.:   systemBatMult = 4 for        > 1500Ah battery
                         //
                         //  using Floating point allows fine tuning of battery size via ASCII commands.

float systemVoltMult = 1; // The actually system battery is this mutable of a "12v battery".   It is set during startup by reading the
                          //  at-rest value of the system battery via a raw A/D on the Atmel CPU and calculating the multiplier value. And is used                                                                                                //  to adjust the target battery voltages contained within the chargeParm table.
                          //      i.e.:   systemBatMult = 1 for 12v battery
                          //      i.e.:   systemBatMult = 2 for 24v battery
                          //      i.e.:   systemBatMult = 3 for 36v battery
                          //      i.e.:   systemBatMult = 4 for 48v battery
                          //
                          //  Using a Float allows better support for other voltages, ala 2.666 --> 32v batteries. 3.5 --> 42v ones.

int8_t SDMCounter = SDM_SENSITIVITY; // Use this to moderate the number of times the Serial port is updated with debug info.
bool sendDebugString = false;        // By default, lets assume we are NOT sending out the debug string.  Unless overridden by $EDB:, or
                                     // if #define DEBUG is present (look in startup() )

// Internal function prototypes.
void set_VAWL(float passedV);

//------------------------------------------------------------------------------------------------------
// Initialize Alternator
//      This function is called once during Startup to establish the ports and global variables needed
//      to manage the Alternators PWM field.
//
//------------------------------------------------------------------------------------------------------

bool initialize_alternator(void)
{

    pinMode(FIELD_PWM_PORT, OUTPUT); // Set up the FET Driver pins

    set_PWM_frequency(); // this was defined in SmartRegulator.h when we set the PWM pin

    attachInterrupt(STATOR_IRQ_NUMBER, stator_IRQ, RISING); // Setup the Interrupt from the Stator.

    set_ALT_PWM(0);             // When starting up, make sure to turn off the Field
    set_charging_mode(unknown); // We are just starting out...

    return (true);
} //initialize_alternator

//------------------------------------------------------------------------------------------------------
// Stator IRQ Handler
//      This function is called at on every spike from the Stator sensor.  It is used to estimate RPMs of
//      the engine, and well as to synchronize the current and voltage sampling of the alternator
//      via the INA-226 chip.
//
//------------------------------------------------------------------------------------------------------

void stator_IRQ()
{
    uint32_t static priorInterupt_uS = 0L; // uSec timer of prior pass through
    int static interuptCounter = 0;        // Used for smoothing function in interrupt handler when calculating RPMs.

    if (++interuptCounter >= RPM_IRQ_AVERAGING_FACTOR)
    {
        IRQDuration = micros() - priorInterupt_uS;
        priorInterupt_uS = micros();
        interuptCounter = 0;
    }
    statorIRQflag = true; // Signal to the main loop that an the stator voltage has started to rise.
}

//------------------------------------------------------------------------------------------------------
// Calculate RPMs
//      This function will calculate the RPMs based in the current interrupt counter and time between last calculation
//
//------------------------------------------------------------------------------------------------------

void calculate_RPMs()
{

    uint32_t static lastRPMCalc = 0UL;
    uint32_t mills;
    int workRPMs;

    mills = millis(); // We use millis() a lot, hold a local copy.

    // Calculate RPMs via the Stator signal
    if ((mills - lastRPMCalc) > (IRQ_TIMEOUT * RPM_IRQ_AVERAGING_FACTOR))
    {                     // If we have waited too long for the needed number of IRQs, we are either just
                          //     starting, (priorInterupt_uS == 0), or we are not getting any stator IRQs at the moment.
                          //  Or if the Field is turned off - and hence IRQs are unstable . . .
        measuredRPMs = 0; //  Let the world know we have no idea what the RPMs are...
        lastRPMCalc = mills;
        return;
    }

    if (IRQDuration != 0)
    { // Wait for several interrupts before doing anything, a smoothing function.
        workRPMs = (int)((60 * 1000000 / IRQDuration * RPM_IRQ_AVERAGING_FACTOR) / ((systemConfig.ALTERNATOR_POLES * systemConfig.ENGINE_ALT_DRIVE_RATIO) / 2));
        // Calculate RPMs based on time, adjusting for # of interrupts we have received,
        // 60 seconds in a minute, number of poles on the alternator,
        // and the engine/alternator belt drive ratio.
        IRQDuration = 0; // Reset signal flag / counter
        lastRPMCalc = mills;

        if (workRPMs > 0)
        {                            // Do we have a valid RPMs measurement?
            measuredRPMs = workRPMs; // Yes, take note of it.  If we had calculated a negative number (due to micros() wrapping),
                                     // just ignore it this time around and update the value next cycle.

            //     Now - let's see if we need to be looking for any tests or levels we need to save.

            if ((systemConfig.FIELD_TACH_PWM == -1) &&                              // Did user set this to Auto Determine Field PWM min for tech mode dive?
                ((fieldPWMvalue < thresholdPWMvalue) || (thresholdPWMvalue == -1))) //  And is this either a new PWM drive 'low', or have we never even see a low value before ( == -1)
                thresholdPWMvalue = fieldPWMvalue;                                  //  Yes, Yes, and/or Yes:  So, lets take note of this PWM value.

            if (thresholdPWMvalue > MAX_TACH_PWM) // Range check:  Do not allow the 'floor' PWM value to exceed this limit, a safety in case something goes wrong with auto-detect code...
                thresholdPWMvalue = MAX_TACH_PWM;
        }
    }
} //calculate_RPMs

//------------------------------------------------------------------------------------------------------
//
//      Calculate Alternator Targets
//
//
//      This function calculates target or goals values for Alternator Volts, Amps and Watts limits.
//      After calculating, it will update appropriate target Global Variables.
//
//------------------------------------------------------------------------------------------------------

void calculate_ALT_targets(void)
{

    int i;
    float f;
    if (cpIndex >= MAX_CPES)
    { // As this is a rather critical function, do a range check on the current index
        chargingState = FAULTED;
        faultCode = FC_LOG_CPI_STATE; // cpIndex is out of array bound....
    }                                 // Fall through and the case(chargingState) = default will set values = 0
                                      // in effect turning off the regulator.

    if ((measuredBatTemp != -99) && 
        ((measuredBatTemp >= chargingParms.BAT_MAX_CHARGE_TEMP) || (measuredBatTemp <= chargingParms.BAT_MIN_CHARGE_TEMP)))
        set_charging_mode(float_charge); // If we are too warm or too cold - force charger to Float Charge safety voltage.

    switch (chargingState)  // to find correct BAT_V_SETPOINT and set targetAltAmps and targetAltWatts
    {
    case warm_up:
    case ramping:
    //case post_ramp:
        f = chargingParms.ACPT_BAT_V_SETPOINT;  // When Ramping up, we want to target the lower of Acpt or Float set points
        if ((chargingParms.FLOAT_BAT_V_SETPOINT != 0) && (chargingParms.FLOAT_BAT_V_SETPOINT < f)) 
            f = chargingParms.FLOAT_BAT_V_SETPOINT;                                                

        set_VAWL(f); // Set the Volts/Amps/Watts limits (See helper function just below)
        break;

    case bulk_charge:
    case determine_ALT_cap:
    case acceptance_charge:
        set_VAWL(chargingParms.ACPT_BAT_V_SETPOINT); // Set the Volts/Amps/Watts limits (See helper function just below)
        break;

    case overcharge_charge:
        set_VAWL(chargingParms.EXIT_OC_VOLTS);  // Set the Volts taking into account comp factors (system voltage, bat temp..)
        targetAltAmps = min(targetAltAmps, (chargingParms.LIMIT_OC_AMPS * systemAmpMult)); // Need to override the default Amps / Watts calc, as we do things a bit different in OC mode.
        targetAltWatts = min(targetAltWatts, (chargingParms.EXIT_OC_VOLTS * systemVoltMult * targetAltAmps));
        break;

    case float_charge:
    case forced_float_charge:
        set_VAWL(chargingParms.FLOAT_BAT_V_SETPOINT); // Set the Volts/Amps/Watts limits (See helper function just below)

        if ((chargingParms.LIMIT_FLOAT_AMPS != -1) && (shuntAltAmpsMeasured == true))
        {   // User wants active current regulation during FLOAT, AND it seems the shunt it working.
            targetAltAmps = min(targetAltAmps, (chargingParms.LIMIT_FLOAT_AMPS * systemAmpMult)); // Need to override the commonly set target Amps / Watts calcs.
            targetAltWatts = min(targetAltWatts, (chargingParms.FLOAT_BAT_V_SETPOINT * systemVoltMult * targetAltAmps));
        }

        break;

    case equalize:
        set_VAWL(chargingParms.EQUAL_BAT_V_SETPOINT); // Set the Volts/Amps/Watts limits (See helper function just below)

        if (chargingParms.LIMIT_EQUAL_AMPS != 0)
        {
            targetAltAmps = min(targetAltAmps, (chargingParms.LIMIT_EQUAL_AMPS * systemAmpMult));
            targetAltWatts = min(targetAltWatts, (chargingParms.EQUAL_BAT_V_SETPOINT * systemVoltMult * targetAltAmps));
        }
        break; // In equalization mode need to re-calc the Watts limits, as one of the

    case LIFEPO_FORCED_SHUTDOWN:
    default:
        targetBatVolts = 0.0; // We are shut down, faulted, or in an undefined state.
        targetAltWatts = 0;
        targetAltAmps = 0;
        fieldPWMLimit = 0;  // no need to call set_VAWL() and should not to avoid any overrides that might get set there
        break; //return;
    } //switch (chargingState)

    if ((targetBatVolts != 0.0) && (measuredBatTemp != -99))
    {  // If we can read the Bat Temp probe, do the Battery Temp Comp Calcs.
        if (measuredBatTemp < chargingParms.MIN_TEMP_COMP_LIMIT) // 1st check to see if it is really cold out, if so only compensate up to the
            i = chargingParms.MIN_TEMP_COMP_LIMIT;               // 'limit' - else we risk overvolting the battery in very cold climates.
        else
            i = measuredBatTemp;

        targetBatVolts += (BAT_TEMP_NOMINAL - i) * chargingParms.BAT_TEMP_1C_COMP * systemVoltMult;
    }
} //calculate_ALT_targets

//-------       'helper' process called by a LOT of places in calculate_ALT_targets();
//              Pass in the targetBatVolts you want to use and this will set the global variable and may use it to calc the target system watts.
//              This function will also 'adjust' the passed-in set voltage parameter based in the current systemVoltMult as well as initialy set the max
//              field PWM value we should be using.
void set_VAWL(float passedV)
{

    targetBatVolts = passedV * systemVoltMult; // Set global regulate-to voltage.

    // Set the 'high water limits' (Alt Amps, Watts, and max PWM to use) applying various de-rating values.
    //   Note that we ALWAYS apply the de-rating values, even if the user has told us the Alternator size.
    //   Note also that in addition to de-rating via measured Amps, we will also 'de-rate' based in the max
    //   PWM value to be sent out.
    //   Some of these 'global' limits will be over-written after calling set_VAWL(), example, when determining alternators
    //   capacity, in some of the float modes and at times when we are in-sync with a remote battery master via the CAN bus.

    // Start with the biggie, the overall de-rating % based on user's selection
    if ((measuredAltTemp == -100) || ((requiredSensorsFlag & RQAltTempSen) != 0))
    {                                                                           // User has indicated they want 'half power' mode by shorting out the Alt Temp NTC sender.
        targetAltAmps = systemConfig.ALT_AMP_DERATE_HALF_POWER * altCapAmps;    //   (Or is the required Alt Temp sensor missing)
        fieldPWMLimit = systemConfig.ALT_AMP_DERATE_HALF_POWER * FIELD_PWM_MAX; // Prime the 'max' allowed PWM using the appropriate de-rating factor.  (Will adjust later for idle)
    }
    else if (smallAltMode == true)
    { //  User selected Small Alternator mode, lets treat it gently.
        targetAltAmps = systemConfig.ALT_AMP_DERATE_SMALL_MODE * altCapAmps;
        fieldPWMLimit = systemConfig.ALT_AMP_DERATE_SMALL_MODE * FIELD_PWM_MAX;
    }
    else
    { //  No de-ratings, Full Power (Well, maybe just a little back from that if user configured DERATE_NORMAL ..)
        targetAltAmps = systemConfig.ALT_AMP_DERATE_NORMAL * altCapAmps;
        fieldPWMLimit = systemConfig.ALT_AMP_DERATE_NORMAL * FIELD_PWM_MAX;
    }

    // Now we will see if we need to further adjust down the max PWM allowed based in the Alternator RPMs.
    //   (Trying to prevent frying the alternator with Full Field if the engine is turning slowly - or perhaps stopped)

    if ((thresholdPWMvalue > 0) && (measuredRPMs == 0) && (systemConfig.ALT_PULLBACK_FACTOR == -1)) // At one time have we seen RPMS?  But not now?  And has user selected 70% cap for this situation?
        fieldPWMLimit = min(fieldPWMLimit, FIELD_PWM_MAX * 0.70);                                   // Yes, cap PWM to 70% max field -- reduce stress on field.

    if ((measuredRPMs != 0) && (systemConfig.ALT_PULLBACK_FACTOR > 0))
    { // Well, we still can measure RPMs - Are we configured to do a PWM pull-back based on the current RPMs?
        fieldPWMLimit = constrain((60 + (3 * (measuredRPMs - systemConfig.ALT_IDLE_RPM) / systemConfig.ALT_PULLBACK_FACTOR)), FIELD_PWM_MIN, fieldPWMLimit);
        // This actually comes to around 0.8% additional PWM for every APBF RPMs, but let's call it close enough..
        // User configurable ALT_PULLBACK_FACTOR  (via PBF in $SCA command) determine how quickly this pull-back is phased out.
        fieldPWMLimit = max(fieldPWMLimit, thresholdPWMvalue); // However, do not pull down too much so that we lose the Tach sync.
    }

    // Finally, do we need to adjust things out for some other 'special cases'

    if ((targetAltAmps == 0) || (chargingState == determine_ALT_cap))
        targetAltAmps = 1000; // If we need to do a new auto-size cycle (or user has told us to disable all
                              // AMP capacity limits for the Alternator), set Amps to a LARGE number
                              // (Ok you, yes YOU!  If you are here to change this 1000A value it must mean you have a serious system.
                              //  Which is why I put in the FIXED 1000 value - to get you HERE to consciously make a change, showing you
                              //  understand what you are doing!!!)
    if (chargingState == determine_ALT_cap)
        fieldPWMLimit = FIELD_PWM_MAX; // And if we are indeed determining the Alt Capacity, we need to be able to drive the Field PWM full bore!

    if (systemConfig.ALT_WATTS_LIMIT == -1)
        targetAltWatts = targetBatVolts * targetAltAmps; // User has selected Auto-calculation for Watts limit
    else
        targetAltWatts = systemConfig.ALT_WATTS_LIMIT; //  Or they have specified a fixed value (or disabled watts capping by specifying 0).

    if ((targetAltWatts == 0) || (chargingState == determine_ALT_cap))
        targetAltWatts = 15000; // If we have NOT measured the capacity (or user has told us to disable watts capacity limits
                                // for the System Wattage), set Watts to a LARGE number  (Ahem, see 1000A comment above  :-)
} //set_VAWL

//------------------------------------------------------------------------------------------------------
//
//  Set Alternator Mode
//              This function is used to change the Alternator Mode.
//              If the mode is changed, it will also reset global counters and timers as appropriate.
//              Note that is the alternator is already in the requested mode, no global variables will be changes -
//              nor will the timers be reset.
//
//
//------------------------------------------------------------------------------------------------------

void set_charging_mode(tModes requestedMode)
{

    if (chargingState != requestedMode)
    {

        if ((chargingState != ramping) && (requestedMode == ramping))
            rampModeEntered = millis(); // Just take note of this time, in case some how RAMP gets cut short
                                        // we will continue it in the 1st part of Bulk.

        chargingState = requestedMode;

        altModeChanged = millis();
        LEDRepeat = 0;                                 // Force a resetting of the LED blinking pattern
        if (abs(measuredAltAmps) < USE_AMPS_THRESHOLD) // If we are seeing low battery current (+ or -) ..
            shuntAltAmpsMeasured = false;              //   .. reset the amp shunt flag, as it may be that the shunt has failed during operation.  (Not a total fail-safe, but this adds a little more reliability)
        modeChangeASecs = accumulatedASecs;            // Noting Amp-Seconds at this point in case we have been asked to enter a Float, or post-float mode (one of the exits is AH based)
    }
} //set_charging_mode

//------------------------------------------------------------------------------------------------------
//
//  Set Alternator PWM
//              This function is used to change the Alternator PWM field.
//
//
//------------------------------------------------------------------------------------------------------

void set_ALT_PWM(int PWM)
{

    if ((chargingState == unknown) || (chargingState == disabled) || (chargingState == FAULTED))
        fieldPWMvalue = 0; // NO PWM if we are disabled!
    else
        fieldPWMvalue = PWM;

    if ((measuredBatVolts -  targetBatVolts)  <= (LD1_THRESHOLD * systemVoltMult))  {                                        // Yes, we are AT LEAST over the 1st line...
        analogWrite(FIELD_PWM_PORT,PWM);   // Only set the PWM active value if we are at or below Vbat target.
        lastPWMChanged = millis();
    } else {
        analogWrite(FIELD_PWM_PORT,max(0, thresholdPWMvalue)); // If at ANY time we are over-target, turn off the actual field PWM drive until Vbat lowers itself.
    }                                                          // Note we do not change the PID engine working value (fieldPWMValue), just turn off the field drive until 
                                                               // the next cycle.        
                                                               // Note also that we do not go below the TACH drive value - either measured or set by user.

} //set_ALT_PWM

//------------------------------------------------------------------------------------------------------
//
//  Manage the Alternator.
//              This function will check and change the Alternator state as well as make adjustments to the PWM Field Value
//              If the system is so configured, it will also manage the auto-sampling of the Alternators AMPs capacity.
//
//              -- Consider Ramping
//              -- Consider how far from target we are, further away = hit change it harder.
//              --  And if overvoltage, drop it down even faster!
//
//
//
//
//      Note:  Make sure to also look at the ASCII command $FRM:, as it ALSO can change alternator modes...
//
//
//------------------------------------------------------------------------------------------------------

void manage_ALT()
{

    //----   Working variable used each time through, to hold calcs for the PID engine.
    float errorV; // Calc the real-time delta error (P value of PID) Measured - target:  Note the order, over target will result in positive number!
    float errorA;
    float errorW;
    int errorAT;
    bool atTargVoltage; // Have we reached the target voltage?  Used when checking to see if we are ready to transation to the next Mode.

    float VdErr; //  Calculate 1st order derivative of VBat error  (Rate of Change, D value of PID)
    float AdErr; //  Calculate 1st order derivative of Alt Amps error
    float WdErr; //  Calculate 1st order derivative of Alt Watts error
    int ATdErr;  //  Calculate 1st order derivative of Alt Temp error (convert to Float in calc, leave INT here for smaller code size)

    //----  Once the PID values are calculated, we then use the PID formula to calculate the PWM adjustments.
    //      Note that many things are 'regulated', Battery Voltage, but also current, alternator watts (engine load), and alternator temperature.
    int PWMErrorW;             // Watts delta (Engine  limit)                          // Calculated PWM correction factors
    int PWMErrorV;             // Volts delta (Battery limited)                        //   +  --> Drive the PWM harder
    int PWMErrorA;             // Alt Amps delta (Alternator limited)
    int static PWMErrorAT = 0; // Alt Temp delta (Alternator limited)  -- we remember this value between PID calcs, as if we are over-temp we do not want anyone else to raise things.

    int PWMError; // Holds final PWM modification value.

    //-----  Working variables that must RETAIN their values between calls for mange_alt().  Some are for the PID, others for load-dumps management and temperature pull-backs.
    float static ViErr = 0; // Accumulated integral error of VBat errors - retained between calls to manage_alt();
    float static AiErr = 0; // Accumulated integral error of Alt Amps errors  (I values of PID)
    float static WiErr = 0; // Accumulated integral error of Alt Watts errors

    float static priorBatVolts = 0; // These is used in manage_ALT() to implement PID type logic, it holds the battery voltage of the prior
    float static priorAltAmps = 0;  // to be used to derived the derivative of VBat.
    int static priorAltWatts = 0;
    int static priorAltTemp = 0;

    int8_t static TAMCounter = TAM_SENSITIVITY; // We will make temperature based adjustmetns only every x cycles through adjusting PWM.

    bool static LD3Triggered = false;
    bool static AOTTriggered = false; // Has the Alternator Overtemp been triggred?  If so, do not let PWM rise until it cools off some.

    uint32_t enteredMills;                   // Time in millis() managed_alt() was entered.  Used throughout function and saves 300 bytes of code vs. repeated millis() calls
        
    char charBuffer[OUTBOUND_BUFF_SIZE + 1]; // Used to assemble Debug ASCII String (if needed)

    //------ NOW we can start the code!!
    //

    if (updatingBatVAs || updatingAltVAs)
        return; // If Volts/Amps measurements are being refreshed just skip checking things this time around until they are ready.

    errorV = measuredBatVolts - targetBatVolts;                      // Calc the error values, as they are used a lot down the road.
    errorA = measuredAltAmps - targetAltAmps;                        // + = over target, - = under target.
    errorW = measuredAltWatts - (targetAltWatts * otPullbackFactor); // (Adjust down Target Alt Watts for any overtemp condition...)
    errorAT = measuredAltTemp - systemConfig.ALT_TEMP_SETPOINT;

    atTargVoltage = (errorV >= (-PID_VOLTAGE_SENS * systemVoltMult)); // We only need to be within 'shooting range' of the target voltage to consider we have met the conditions for a phase transition.
                                                                      //  (Helpful with small alternators which may not be able to push over the target voltage on low-impedance batteries)

    enteredMills = millis(); // Remember this as we are going to use it a lot..
                             // Using the working variable saves code size, and also assures we have consistency with all
                             // the time-stamping that will happen inside of manage_alt()

    //----  1st, seeing as we have a new valid voltage reading, let's do the quick over-voltage check as well check to see if there is if there seems to be load-dump situation...
    //

    if (errorV > (LD1_THRESHOLD * systemVoltMult)) // Yes, we are AT LEAST over the 1st line...
        set_ALT_PWM(fieldPWMvalue);                // If we are over-voltage call set_ALT_PWM() - let it turn off the field drive if we are.

    //---   OK then, let's get on with the rest of things.  But 1st, is it even time for us to adjust the PWM yet?
    //       Aside from the Load Dump checks we did above (which happen every time we get a new Vbat reading) we need to take some time to let the alternator and system
    //       settle in to changes.   Alts seem to take anywhere from 100-300mS to 'respond' to a change in PWM up, a bit less for down.  By controlling how often we
    //       try to adjust the PWM, we give the system time to respond to a prior change.
    //       A 2nd benefit is that by using a fixed time between adjustments  the PID calculations are somewhat simplified, specifically in the Derivative and Integral factors.

    if ((enteredMills - lastPWMChanged) <= PWM_CHANGE_RATE) //   Is it too soon to make a change?
        return;                                             //     Yes - skip doing anything this time around.

    //--- Calculate the values the 1st order Derivative (D) of the PID engine.
    //
    VdErr = measuredBatVolts - priorBatVolts; // 'D's 1st ! Note we are using the D of the 'input' to the PID engine, this avoids the
    priorBatVolts = measuredBatVolts;         //  issue knows as the 'Derivative Kick'

    AdErr = measuredAltAmps - priorAltAmps;
    priorAltAmps = measuredAltAmps;

    WdErr = measuredAltWatts - priorAltWatts;
    priorAltWatts = measuredAltWatts;

    ATdErr = measuredAltTemp - priorAltTemp; // Prior AT is only updated every once and a while, just below.
    ATdErr = constrain(ATdErr, 0, KdPWM_AT); // And we only want the D to pull-down as we are approching target temp.
                                             //    (Never prevent an OT from pulling down)

    //--- Calculate the values for the Integral (I) values
    //
    ViErr += (errorV * KiPWM_V / systemVoltMult); // Calc the I values.
    AiErr += (errorA * KiPWM_A);                  // Note also that the scaling factors are figured in here, as opposed to during the PID formula below.
    WiErr += (errorW * KiPWM_W / systemVoltMult); // Doing so helps avoid issues down the road if we ever implement an auto-tuning capability, and change the I

    ViErr = constrain(ViErr, 0, PID_I_WINDUP_CAP); // Keep the accumulated errors from getting out of hand, their impact is meant to be a soft refinement, not a
    AiErr = constrain(AiErr, 0, PID_I_WINDUP_CAP); // sledge hammer!
    WiErr = constrain(WiErr, 0, PID_I_WINDUP_CAP); // Also - ONLY use 'I' to pull-back the PWM, never to allow it to be driven stronger.

    //--  And calc the final PID correction factors
    //
    PWMErrorV = (int)((errorV * -KpPWM_V / systemVoltMult) - ViErr - (VdErr * KdPWM_V / systemVoltMult));
    PWMErrorA = (int)((errorA * -KpPWM_A) - AiErr - (AdErr * KdPWM_A));
    PWMErrorW = (int)((errorW * -KpPWM_W / systemVoltMult) - WiErr - (WdErr * KdPWM_W / systemVoltMult));

    //--  Temperature adjustments are handled a little different, in that the calcs are paced out and applied to better match the slow responcee time of temperature changes.
    //

    if (PWMErrorAT <= 0)
        PWMErrorAT = 0; // If the last cycle of temp control did a pull-down do not allow any raise until we recalc.

    if (--TAMCounter <= 0)
    { // We will make ADJUSTMENTS based on Temp Error only every x times through.
        TAMCounter = TAM_SENSITIVITY;

        priorAltTemp = measuredAltTemp;

        PWMErrorAT = (int)((errorAT * -KpPWM_AT) - (ATdErr * KdPWM_AT));
        APUAdjAT = PWMErrorAT; // Snap-shot of any PWM adjustments beign done via Alternator Tempeture PID for use in .
    };

    //---   Next, we want to do a special check to help reduce a tug-of-war between the system overheating (Ta, Te, Tx) which will cause
    //      PWMs to be reduced - and once things have cooled off some having the SAME load
    //      that caused the over-temp situation in the 1st place.  Thus creating an oscillation.
    //      So, to reduce that, we will take down the Watts Target (targetAltWatts) each time we get into a large overtemp
    //      situation.

    if (errorAT > 0)
    { // Are we overtemp on something?
        if (otPullbackTriggered == false)
        {
            otPullbackFactor *= OT_PULLBACK_FACTOR;                             // Yes, pull back the target watts and PWM limit some %
            otPullbackFactor = max(otPullbackFactor, OT_PULLBACK_FACTOR_LIMIT); // Only pull back so much..
            otPullbackTriggered = true;                                         // Only apply the pull-back factor each time we exceed a limit
        }
    }
    else
        otPullbackTriggered = false; // All is well temperaturewise, reset the trigger


    if (measuredAltTemp <= -99)      // Yet another Special Case for alt temp:  if we are not able to measure an alternator temp...
        PWMErrorAT = PWM_CHANGE_CAP; //  .. make no effort to do any adjustments up based on Alt Temp.

    //-- Finaly, do a kind of 'load-dump' check on alternator temperature, to see if it is growing so fast we have a hard time catching up with it.
    //
    if (measuredAltTemp > (systemConfig.ALT_TEMP_SETPOINT * AOT_PULLBACK_THRESHOLD) && (AOTTriggered == false))
    {
        AOTTriggered = true;                  // Only do it once per 'event'.  (This flag will also hold off any attempt to increase PWM till temp lowers)
        fieldPWMvalue *= AOT_PULLBACK_FACTOR; // Do something dramatic if we are very much over timp!
    }

    if (measuredAltTemp < (systemConfig.ALT_TEMP_SETPOINT * AOT_PULLBACK_RESUME))
        AOTTriggered = false; // Check to see if the alternator is cool enough to release the Alt OverTemp hold.

    //---   Now lets calculate how much total error we have accumulated
    //      We will let the 'littlest kid' win.  Meaning, we will adjust the PWM up only as much as the smallest one calculated for above will call for.
    //      while also letting the largest pull-down control things when needed.

    PWMError = min(PWMErrorV, PWMErrorA); // If there is ANYONE who thinks PWM should be pulled down (or left as-is), let them have the 1st say.
    PWMError = min(PWMError, PWMErrorW);
    PWMError = min(PWMError, PWMErrorAT);
    PWMError = min(PWMError, PWM_CHANGE_CAP); // While we are at it, make sure we do not ramp up too fast!  (Note that ramping down is not capped)

    //----  Check to see if it is time to transition charging phases
    //
    //        Ramp   --> Bulk:    Total time allocated for Ramping exceeded, or we are driving alternator full bore.
    //        Bulk   --> Accept:  VBatt reached max
    //        Accept --> Float:   Been in Accept phase for EXIT_ACPT_DURATION
    //        Float  --> ????
    //

    if (measuredBatAmps >= persistentBatAmps) // Adjust the smoothing Amps variable now.
        persistentBatAmps = measuredBatAmps;  // We want to track increases quickly,
    else if (measuredBatAmps > 0.0)           // but decreases slowly...  This will prevent us from changing state too soon. (And don't count discharging)
        persistentBatAmps = (((persistentBatAmps * (float)(AMPS_PERSISTENCE_FACTOR - 1L)) + measuredBatAmps) / AMPS_PERSISTENCE_FACTOR);

    if (measuredBatVolts >= persistentBatVolts) // Adjust the smoothing Volts variable now.
        persistentBatVolts = measuredBatVolts;  // We want to track increases quickly,
    else                                        // but slowly decreases...  This will prevent us from changing state too soon.
        persistentBatVolts = (((persistentBatVolts * (float)(VOLTS_PERSISTENCE_FACTOR - 1)) + measuredBatVolts) / VOLTS_PERSISTENCE_FACTOR);



    switch (chargingState)  // manage the transitions between charging states
    {
    case warm_up:
        chargingStateString = "WARMUP     ";

        if ((tachMode) && (systemConfig.FIELD_TACH_PWM > 0)) // If user has configured system to have a min PWM value for Tach mode
            fieldPWMvalue = systemConfig.FIELD_TACH_PWM;     // send that out, even during engine warm-up period.
        else
            fieldPWMvalue = FIELD_PWM_MIN; //  All other cases, Alternator should still be turned off.

        PWMError = 0; // we will not be making ANY adjustments to the PWM for now.

        // countdown for warm-up
        inChargingStateCount = (uint32_t)systemConfig.ENGINE_WARMUP_DURATION - int((enteredMills - altModeChanged) / 1000UL); // capture the number of second remaining in the warmup.
      
        if (((enteredMills - altModeChanged) > ((uint32_t)systemConfig.ENGINE_WARMUP_DURATION * 1000UL)) &&  
            !((tachMode) && (systemConfig.FIELD_TACH_PWM > 0) && (measuredRPMs == 0))) //  (But 1st - if we expect to be able to measure RPMs, don't leave pending until we do  (Engine might be stopped))
            set_charging_mode(ramping);                                                // It is time to start Ramping!

        break; // WarmUp

    case ramping:
        chargingStateString = "RAMPING    ";

        persistentBatAmps = measuredBatAmps;   //  While ramping, just track the actually measured amps and Watts.
        persistentBatVolts = measuredBatVolts; //  Overwriting the persistence calculation above.
        reset_run_summary();                   // Starting a new Charge Cycle - reset the accumulators.

        // countdown for ramping
        inChargingStateCount = int(((PWM_RAMP_RATE * FIELD_PWM_MAX / PWM_CHANGE_CAP) - (enteredMills - altModeChanged)) / 1000UL); // capture the number of second remaining in the ramp.

        if (systemConfig.ALT_AMPS_LIMIT != -1)        // Starting a new 'charge cycle' (1st time or restart from float).  Re-sample Alt limits
            altCapAmps = systemConfig.ALT_AMPS_LIMIT; // User is telling us the capacity of the alternator (Or disabled Amps by setting this = 0)
        else
            altCapAmps = 0; // User has selected Auto-determine mode for Alternator size, so reset the high water
                            // mark for the next auto-sizing cycle.
        altCapRPMs = 0;

        if ((fieldPWMvalue >= fieldPWMLimit) ||  // Driving alternator full bore?
            (atTargVoltage) ||                   // Reached terminal voltage?
            (errorA >= 0) ||                     // Reached terminal Amps?
            (errorW >= 0) ||                     // Reached terminal Watts?  (Reaching ANY of these limits should cause exit of RAMP mode)
            ((enteredMills - altModeChanged) >= PWM_RAMP_RATE * FIELD_PWM_MAX / PWM_CHANGE_CAP) // Or, have we been ramping long enough?
#ifdef ENABLE_FEATURE_IN_SCUBA
            ||
            ((scubaMode) && (fieldPWMvalue >= FIELD_PWM_SCUBA)) // or, if in scubaMode AND PWM is at Scuba level
#endif
        )
        {
          if (systemConfig.ALT_AMPS_LIMIT == -1)
              set_charging_mode(determine_ALT_cap); //  Yes or Yes!  Time to go into Bulk Phase
          else
              set_charging_mode(bulk_charge); // (was post_ramp)  But 1st see if we need to measure the Alternators Capacity..
        }
        else
        {
          if ((enteredMills - lastPWMChanged) <= PWM_RAMP_RATE) // Still under limits, while ramping wait longer between changes..
            return;
        }

        break;  // ramping

    case determine_ALT_cap:
        chargingStateString = "DET ALT CAP";

        if ((systemConfig.ALT_AMPS_LIMIT != -1) || // If we are NOT configured to auto-determining the Alt Capacity,   --OR--
            (fieldPWMvalue == FIELD_PWM_MAX) ||    // we have Maxed Out the Field (PWM capping should have been removed during alt_cap mode) --OR--
            (atTargVoltage) ||                     // Reached terminal voltage?  --OR-- (meaning, we really can not finish determine the Alt cap as the battery is kind of full.....)
            (measuredRPMs == 0) ||                 // If we are not able to see RPMs, can not reliably do alt-cap setting
            (shuntAltAmpsMeasured == false) ||     //  And of course, if we are not even able to measure current - not a chance to size the alt!
            ((enteredMills - altModeChanged) >= SAMPLE_ALT_CAP_DURATION))
        { // Finally -- have we been doing an Alt Cap Sampling Cycle long enough..

            set_charging_mode(bulk_charge); // (was post_ramp) Stop this cycle - go back to Bulk Charge mode.
        }

        if (measuredAltAmps > altCapAmps)
        {                                 // Still pushing the alternator hard.
            altCapAmps = measuredAltAmps; // Take note if we have a new High Amp Value . . .
            altCapRPMs = measuredRPMs;
        }

        break; // determine_ALT_cap

         
        //---  BULK CHARGE MODE
        //      The purpose of bulk mode is to drive as much energy into the battery as fast as it will take it.
        //      Bulk is easy, we simply drive the alternator hard until the battery voltage reaches the terminal voltage as defined by .ACPT_BAT_V_SETPOINT in the CPE.
        //      While in Bulk Mode, we will also see if there is reason for us to re-sample the alternator capacity (if configured to do so), ala the RPM have increased
        //      indicating the engine has sped up.

    case bulk_charge:
        chargingStateString = "BULK       ";
        inChargingStateTime = (enteredMills - altModeChanged);

        if (atTargVoltage)
        {   // Have we reached terminal voltage during Bulk?
            adptExitAcceptDuration = (enteredMills - altModeChanged) * ADPT_ACPT_TIME_FACTOR; // Calculate a time-only based acceptance duration based on how long we had been in Bulk mode,
                                                                                              //    (In case we cannot see Amps.)
            set_charging_mode(acceptance_charge);                                             // Bulk is easy - got the volts so go into Acceptance Phase!
        }

        persistentBatAmps = measuredBatAmps; //  While in Bulk as well, just track the actually measured amps and Watts.
                                             // (Prevents any initial low-amp numbers from clouding the issue once we get into Acceptance)

        if (((enteredMills - rampModeEntered) <= PWM_RAMP_RATE * FIELD_PWM_MAX / PWM_CHANGE_CAP) &&
            ((enteredMills - lastPWMChanged) <= PWM_RAMP_RATE))
            return; // If somehow Ramp-mode got short-changed, continue the nice soft ramping until we get to the target voltage.

        // Not at VBat target yet.  See if we need to start a auto-capacity sample cycle on the Alternator                                                                                                     // Not yet.  See if we need to start a auto-capacity sample cycle on the Alternator
        if (systemConfig.ALT_AMPS_LIMIT != -1) // Are we configured to auto-capacity sample the alternator?
            break;                             //    Nope - they told us how big it is - so done for now.

        if ((enteredMills - altModeChanged) <= SAMPLE_ALT_CAP_REST) // Did we just do a capacity sample cycle, and if so have we 'rested' the alternator long enough?
            break;                                                  // Yes we did a cycle, and no we have not rested sufficient.

        // OK, we are configured to do auto Alt Sampling, we have not just done one, so . . .
        if ((measuredRPMs > (altCapRPMs + SAMPLE_ALT_CAP_RPM_THRESH)) || // IF we are spinning the alternator faster, -OR-
            (measuredAltAmps > (altCapAmps * SAMPLE_ALT_CAP_AMPS_THRESH_RATIO)))
        { //    we have seen a new High Amp Value .

            set_charging_mode(determine_ALT_cap); // Start a new 'capacity determining' cycle (If we are not already on one)
                                                  // This last step is kind of the key.  During a new cycle we will not artificially reduce the output
                                                  // of the alternator, but instead run it as hard as we can to see if we get a new High Water Mark.
                                                  // After a short period of time we will then re-enable the reduced current modes (ala smallAltMode)
        }

        break; // bulk_charge

        //---  ACCEPTANCE CHARGE MODE
        //      In acceptance phase we are packing in extra energy into the battery until it is fully charged.  We cap the voltage to .ACPT_BAT_V_SETPOINT to keep from overheating
        //      the battery, and the battery itself controls how many Amps it will 'accept'.  The handling of accept phase is one of the key benefits of using this regulator, as
        //      by monitoring the accepted amps we can MEASURE the batteries state of charge.  Once the acceptance amps fall below about 1-2% of the batteries Ah capacity, we know
        //      the battery is fully charged.  The Amp trigger is defined by .EXIT_ACPT_AMPS, once we fall below this level we move on.  We can also move on by staying too long in
        //      acceptance phase.  CPE entry .EXIT_ACPT_DURATION allows for a time limit to be defined, and if we exceed this time limit we move on.  This can be used when the AMP
        //      shunt is not connected, but also provides a level of protection increase something goes wrong - protection from boiling the battery dry.
        //      Finally, the regulator can be configured to disable Amp-based determination of Exit-Acceptance and instead to a time-based exit criteria based on some factor
        //      of the amount of timer we spent in Bulk.  This will happen if the user has entered -1 in the  EXIT_ACPT_AMPS, OR we do not seem to be able to measure any Amps
        //      (ala, the user has not connected up the shunt).  Note that even with Adaptive Acceptance duration, we will never exceed the configured EXIT_ACPT_DURATION value.
    case acceptance_charge:
        chargingStateString = "ACCEPTANCE ";
        inChargingStateTime= uint32_t(enteredMills - altModeChanged);

        if (((chargingParms.EXIT_ACPT_DURATION > 0) &&
             ((enteredMills - altModeChanged) >= chargingParms.EXIT_ACPT_DURATION)) || // 4 ways to exit.  Have we have been in Acceptance Phase long enough?  --OR--

            ((chargingParms.EXIT_ACPT_AMPS == -1) &&                         // Have we been configured to do Adaptive Acceptance?
             ((enteredMills - altModeChanged) >= adptExitAcceptDuration)) || // ..  Yes, early exit Acceptance Phase if we have exceeded the amount of time in Bulk by x-factor.
                                                                             //                                                                      --OR--

            ((chargingParms.EXIT_ACPT_AMPS > 0) &&  // Is exiting by Amps enabled, and we have reached that threshold?
             ((shuntAltAmpsMeasured == true)) &&    //  ... and does it look like we are even measuring Amps?
             (atTargVoltage) &&    //  ... Also, make sure the low amps are not because the engine is idling, or perhaps a large external load
             (persistentBatAmps <= (chargingParms.EXIT_ACPT_AMPS * systemAmpMult))) || //  has been applied.  We need to see low amps at the appropriate full voltage!

            ((chargingParms.EXIT_ACPT_DURATION == 0) && (chargingParms.EXIT_ACPT_AMPS == 0)))
        {   //  if user has set BOTH time and amps = 0, they do not want to do any Acceptance...
            

            if (chargingParms.LIMIT_OC_AMPS == 0)
                set_charging_mode(float_charge); //      Yes -- time to float  -- OR --. . . .
            else
                set_charging_mode(overcharge_charge); // . . .into OC mode, if it is configured (Limit Amps != 0)
        }
        break; // acceptance_charge

        //---  OVERCHARGE MODE
        //      Overcharge is used by some batteries to pack just-a-little-more in after completing the acceptance phase.  (ala, some AMG batteries like an Overcharge).
        //      Overcharge holds the charge current at a low level while allowing the voltage to rise.  Once the voltage reaches a defined point, the battery is considered
        //      fully charged.  Note that to make full use of this capability, the Amp Shunt should be installed on the BATTERY, not the ALTERNATOR.
        //      The regulator will allow for overcharge to be optionally configured, compete with its own target voltage .EXIT_OC_BAT_VOLTS, Amp limit via .LIMIT_OC_AMPS, and
        //      a time limit .EXIT_OC_DURATION.    To disable OC mode, set LIMIT_OC_AMPS = 0.  (for safety, Time or Volts = 0 will also disable OC mode)

    case overcharge_charge:
        chargingStateString = "OVER CHARGE";

        if (((enteredMills - altModeChanged) >= chargingParms.EXIT_OC_DURATION) || // Have we have been in Overcharge Phase long enough?  --OR--
            (chargingParms.LIMIT_OC_AMPS == 0) ||                                  // Are we even configured to do OC mode? --OR--
            (chargingParms.EXIT_OC_VOLTS == 0) ||
            (atTargVoltage))
        { // Did we reach the terminal voltage for Overcharge mode?

            set_charging_mode(float_charge); // Yes to one of the conditions.  NOW we can go into Acceptance Phase
        }

        break;  //overcharge_charge

        //---  FLOAT MODE
        //      Float is not really a Charge mode, it is more intended to just hold station.  To keep the battery at a point where it will neither continue to charge,
        //      nor discharge.  You can think of it as putting in just enough energy to make up for any self-discharge of the battery.  Exiting Float will happen in several
        //      ways.  A normal exit will be by time; .EXIT_FLOAT_DURATION in the CPE will tell us how long to stay in float.  Setting this to '0' will cause
        //      us to never move out of float on to the next charge state (Post Float).  In normal operations, just hanging around in Float once a battery is fully charged is
        //      the right choice: supplying sufficient energy to keep the battery happy, and also providing any additional amps as needed to drive house loads - so those loads
        //      do not try and take energy from the battery.
        //
        //      But what happens if that house loads gets really large, too large for the Alternator to keep up?  Energy will start to be sapped out of the battery and as some time
        //      we will have to recognize the battery is no longer fully charged.  The 1st (and preferred) way is if we start to see negative Amps on the shunt.  Configuring the
        //      Exit Amps value for a neg number will let the regulator watch the battery and then go out of float when it starts to see too much of a discharge.
        //
        //      If the Amp Shunt is attached to the Alternator, one COULD also look to see if a large number of Amps is being asked for, perhaps near the full capacity
        //      of the Alternator.
        //
        //      An indirect way to recognize the battery is being drawn upon is if the battery voltage drops below .FLOAT_TO_BULK_VOLTS, the alternator will revert to Bulk mode.
        //      This is an indirect way of telling of the battery is losing energy.
        //
        //      Starting with revision 0.1.3, two new capabilities were added:
        //         1) Reevaluation of Amps while in Float (See set_alt_targets(), allowed 'managing' battery amps that flow into battery, even down to 0A
        //         2) Exit criteria based in number of Amp-Hours that have been withdrawn from the battery after first entering Float mode.

    case float_charge:
        chargingStateString = "FLOAT      ";

        if ((((int32_t)chargingParms.EXIT_FLOAT_DURATION) != 0) && // Has max time for Float been configured?
            ((enteredMills - altModeChanged) >= chargingParms.EXIT_FLOAT_DURATION))
        { // And have we been in Float long enough?

            set_charging_mode(post_float); //  Yes, go into Post Float mode for now
            fieldPWMvalue = FIELD_PWM_MIN; //  Turn off alternator
            PWMError = 0;
        }

        if (((chargingParms.FLOAT_TO_BULK_VOLTS != 0) && (persistentBatVolts <= (chargingParms.FLOAT_TO_BULK_VOLTS * systemVoltMult))) ||

            (((shuntAltAmpsMeasured == true)) && // VBat too low, or we are able to measure Amps AND one of the current triggers tripped
             (((chargingParms.FLOAT_TO_BULK_AMPS != 0) && (persistentBatAmps <= (chargingParms.FLOAT_TO_BULK_AMPS * systemAmpMult))) ||

              ((chargingParms.FLOAT_TO_BULK_AHS != 0) &&
               ((int)(((accumulatedASecs - modeChangeASecs) / 3600UL) * (ACCUMULATE_SAMPLING_RATE / 1000UL)) <= (chargingParms.FLOAT_TO_BULK_AHS * systemAmpMult))))))
        {

            //  then we need to go back into Bulk mode?
            set_charging_mode(ramping); //  (Via ramping, so as to soften shock to fan belts)
            
        }
        break; // float_charge

    case forced_float_charge:                  //  If we have been 'forced' into Float mode via the Feature-in or a missing  'required' Bat temp sensor, we just stay there.
        chargingStateString = "FORCE FLOAT"; //  (check_inbound() will note when the feature-in signal is removed and take us out of Float
        break;

    case LIFEPO_FORCED_SHUTDOWN:
        chargingStateString = "LIFEPO STDN";
        fieldPWMvalue = 0; //  Turn off alternator.
        break;

    case post_float: //  During Post-Float the alternator is turned off, but voltage is monitors to see if a large load is placed
                     //  on the system and we need to restart charging.
        chargingStateString = "POST FLOAT ";

        if ((((int32_t)chargingParms.EXIT_PF_DURATION) != 0) && // Has max time for Post-Float been configured?
            ((enteredMills - altModeChanged) >= chargingParms.EXIT_PF_DURATION))
        { // And have we been in Post-Float long enough?

            set_charging_mode(float_charge); //  Yes, switch back to Float for a while.

            break;
        }

        if (((chargingParms.PF_TO_BULK_VOLTS != 0.0) && (persistentBatVolts < chargingParms.PF_TO_BULK_VOLTS * systemVoltMult)) ||
            ((chargingParms.PF_TO_BULK_AHS != 0) && ((shuntAltAmpsMeasured == true)) && // Able to measure current - so do Ah check.
             ((int)(((accumulatedASecs - modeChangeASecs) / 3600UL) * (ACCUMULATE_SAMPLING_RATE / 1000UL)) <= (chargingParms.PF_TO_BULK_AHS * systemAmpMult))))
        {
            //  Do we need to go back into Bulk mode?
            set_charging_mode(ramping); //      Yes or Yes!  Time to go into Bulk Phase  (Via ramping, so as to soften shock to fan belts)
                                        //      In this case I go directly back into recharging, as opposed to float.  This is because the battery
                                        //      has shown some sign of discharging, so no need to flip to float to only then flip to bulk.
            break;
        }

        fieldPWMvalue = FIELD_PWM_MIN; //  If still in post Float mode, make sure to turn off alternator.
        PWMError = 0;

        break;

    case equalize:
        chargingStateString = "EQUALIZE   ";

        if (((enteredMills - altModeChanged) >= chargingParms.EXIT_EQUAL_DURATION) ||
            ((chargingParms.EXIT_EQUAL_AMPS != 0) &&
             ((shuntAltAmpsMeasured == true)) && //  ... and does it look like we are even measuring Amps?
             (atTargVoltage) &&                                         //
             (measuredBatAmps <= (chargingParms.EXIT_EQUAL_AMPS * systemAmpMult))))
        {
            // Have we have been in Equalize mode long enough?  --OR--
            //   Is exiting by Amps enabled, and we have reached that threshold while at target voltage?
            set_charging_mode(float_charge); //      Yes -- time to float  - let the main loop take care of adjusting the PWM.
        }

        break;
/*
    case RBM_CVCC: // Running in true slave mode, doing what the Remote Battery Master is telling us to do.
        chargingStateString = "RBM-CVCC     ";

        break;
*/
    case disabled:
        if(chargingState == disabled)  chargingStateString = "DISABLED   ";
    case unknown:
        if(chargingState == unknown) chargingStateString = "UNKNOWN    ";
        PWMError = 0;
        LEDRepeat = 0; // And force a resetting of the LED blinking pattern
        fieldPWMvalue = 0; //  Turn off alternator.
        break;

    case FAULTED: // If we got called here while in FAULT condition, do not change anything.
        chargingStateString = "FAULTED    ";
        return;

    default:
        chargingStateString = "FAULTED    ";
        chargingState = FAULTED; // We should never have gotten here.   Something is wrong. . .
        faultCode = FC_LOG_ALT_STATE1;
        return;

    } //switch(chargingState)                                                                                             //  End of Switch/case

    //-----   Put out the PWM value to the Field control, making sure the adjusted value is within bounds.
    //        (And if the Tach mode is enabled, make sure we have SOME PWM)
    //
    fieldPWMvalue += PWMError; // Adjust the field value based on above calculations.
    if (tachMode)
        fieldPWMvalue = max(fieldPWMvalue, thresholdPWMvalue); // But if Tach mode, do not let PWM drop too low - else tach will stop working.

#ifdef ENABLE_FEATURE_IN_SCUBA
   // old code...
     //  if (scubaMode && chargingState != forced_float_charge)  // if in forced_float_charge, do not let scubaMode create a higher PWM
     //   fieldPWMvalue = FIELD_PWM_SCUBA;

    if (scubaMode)
    {
        //do not exceed scubaMode field PWM.   This keeps the forced_float_charge below FIELD_PWM_SCUBA
        fieldPWMvalue = min(fieldPWMvalue, FIELD_PWM_SCUBA);
    }
#endif

    fieldPWMvalue = constrain(fieldPWMvalue, FIELD_PWM_MIN, (fieldPWMLimit * otPullbackFactor)); // And in any case, always make sure we have not fallen out of bounds.

    if (!LD3Triggered)
        set_ALT_PWM(fieldPWMvalue); // Ok, after all that DO IT!  Update the PWM
                                    // But NOT if we are in a LD3, as PWM has been forced to 0, let that ride out until
                                    // we get another VBat reading..
    LD3Triggered = false;           // Reset trap (flag) for next time in.

    //-----    Before we leave, does the user want to see detailed Debug information of what just happened?
    //

    if ((sendDebugString == true) && (--SDMCounter <= 0))
    {

        snprintf_P(charBuffer, OUTBOUND_BUFF_SIZE, PSTR("DBG;,%d.%03d, ,%d,%d, ,%d,%d,%d, ,%d,%d,%d,%d,%d, ,%d,%d, ,%c,%s,%s, ,%d,%d,%s, ,%d\r\n"),

                   (int)(enteredMills / 1000UL), // Timestamp - Seconds
                   (int)(enteredMills % 1000),   // time-stamp - 1000th of seconds

                   (int)chargingState,
                   fieldPWMvalue,

                   sample_feature_IN_port1(),
                   sample_feature_IN_port2(),
                   sample_feature_IN_port3(),

                   PWMErrorV,
                   PWMErrorA,
                   PWMErrorW,
                   PWMErrorAT,
                   PWMError,

                   (int)errorAT,
                   

                   'A',
                   float2string(measuredAltVolts, 3),
                   float2string(measuredAltAmps, 1),

                   //usingEXTAmps,
                   thresholdPWMvalue,
                   fieldPWMLimit,
                   float2string(otPullbackFactor, 2),

                   measuredAltTemp,
                   checkStampStack() // How much of the stack has been used?

        );

        ASCII_write(charBuffer);

        SDMCounter = SDM_SENSITIVITY;
    }
} //manage_ALT()

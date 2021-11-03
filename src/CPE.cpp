//      CPE.cpp
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

#include "CPE.h"

//  Default parameters.  The selected row (as indicted by the DIP switches) is copied from the FLASH into the working "chargeParms" variable
//      during Startup().   Also, the EEPROM is checked to see if there is a replacement entry for the default values and if so, those are
//      copied.
//
//      CAUTION:   ALL ENTRIES IN THIS TABLE ASSUMES A NOMINAL 12V / 500Ah - SMALL SIZED BATTERY SYSTEM.
//                 THEY WILL BE AUTOMATICALLY ADJUSTED FOR HIGHER VOLTAGE / CAPACITY BATTERIES VIA THE AUTO-SENSING CAPABILITY.
//                 When populating this table, if you have a higher voltage battery (ala 48v), then make sure to scale
//                 these appropriately back to a '12v' value (ala, divide volts / 4, mult Amps by 4)
//
//      The CPEs below have been defined assuming the Amp shunt is INSTALLED AT THE BATTERY vs. at the Alternator.  If the regulator is configured
//      with the Amp shunt on the Alternator, exit_amp values need to be adjusted to accommodate house loads.  A suggestion is to add 5A to each
//       'threshold' value, with assumption that ship running loads will be around 5A for a small boat.
//      These will scale with the Battery Size scaling switches, so that a larger boat with say a 1500AH battery, that 5A will turn into 15A.
//

const tCPS PROGMEM defaultCPS[MAX_CPES] = {
    
    // eliminated decimal places to fix conversion narrowing problem.  Note that, for example, 6 hours uses 60 here and 4.5 hours uses 45
    // Name      Bulk/Accpt                     Overcharge                    Float                              Post Float                    Post Float                           Equalization                      Temp Comp
    //           Target  <--- Exit Accpt --->   Limit  <-- Exit OC ------->   Target  Limit     Exit Float    Float to   Float to  Float to    Exit PF       PF to       PF to      Target  Limit  Exit         Exit   1 Deg C      Min       Bat Temp
    //           Volts      Duration    Amps    Amps   Volts   Duration       Volts    Amps      Duration     Bulk Amps  Bulk AHrs Bulk Volts  Duration      Bulk Volts  Bulk AHrs  Volts   Amps   Duration     Amps   Compensation Temp Comp Min  Max
    {"LIFELINE", 14.3,   60 * 360000UL,  15,     0,     0.0,  0 * 360000UL,    13.3,    -1,    0 * 360000UL,    -10,       0,       12.8,      0 * 360000UL,  0.0,       0,          0.0,    0,    0 * 360000UL, 0,    0.0234,      -9,       -45, 45}, // LIFELINE BATTERY #1 Default (safe) profile & AGM #1 (Low Voltage AGM).
    {"STD FLA",  14.8,   30 * 360000UL,   5,     0,     0.0,  0 * 360000UL,    13.5,    -1,    0 * 360000UL,    -10,       0,       12.8,      0 * 360000UL,  0.0,       0,          0.0,    0,    0 * 360000UL, 0,    0.005 * 6,   -9,       -45, 45}, // #2 Standard FLA (e.g. Starter Battery, small storage)
    {"HD FLA",   14.6,   45 * 360000UL,   5,     0,     0.0,  0 * 360000UL,    13.2,    -1,    0 * 360000UL,    -10,       0,       12.8,      0 * 360000UL,  0.0,       0,         15.3,   25,   30 * 360000UL, 0,    0.005 * 6,   -9,       -45, 45}, // #3 HD FLA (GC, L16, larger)
    {"AGM #2",   14.7,   45 * 360000UL,   3,     0,     0.0,  0 * 360000UL,    13.4,    -1,    0 * 360000UL,    -10,       0,       12.8,      0 * 360000UL,  0.0,       0,          0.0,    0,    0 * 360000UL, 0,    0.004 * 6,   -9,       -45, 45}, // #4 AGM #2 (Higher Voltage AGM)
    {"GEL",      14.1,   60 * 360000UL,   5,     0,     0.0,  0 * 360000UL,    13.5,    -1,    0 * 360000UL,    -10,       0,       12.8,      0 * 360000UL,  0.0,       0,          0.0,    0,    0 * 360000UL, 0,    0.005 * 6,   -9,       -45, 45}, // #5 GEL
    {"FIREFLY",  14.4,   60 * 360000UL,   7,     0,     0.0,  0 * 360000UL,    13.4,    -1,    0 * 360000UL,    -20,       0,       12.0,      0 * 360000UL,  0.0,       0,         14.4,    0,   30 * 360000UL, 3,    0.024,      -20,       -20, 50}, // #6 Firefly (Carbon Foam)
    {"CUSTOM ",  14.4,   60 * 360000UL,  15,    15,     5.3, 30 * 360000UL,    13.1,    -1,    0 * 360000UL,    -10,       0,       12.8,      0 * 360000UL,  0.0,       0,         15.3,   25,   30 * 360000UL, 0,    0.005 * 6,   -9,       -45, 45}, // #7 4-stage HD FLA (& Custom #1 changeable profile)
    {"LiFePO4 ", 13.8,   10 * 360000UL,  15,     0,     0.0,  0 * 360000UL,    13.6,     0,    0 * 360000UL,      0,     -50,       13.3,      0 * 360000UL,  0.0,       0,          0.0,    0,    0 * 360000UL, 0,    0.000 * 6,    0,         0, 40}  // #8 LiFeP04        (& Custom #2 changeable profile)
}; //*** be certain to change the ENUM of the CPE names in CPE.h if the profiles above are changed ***


// Side note:  The Arduino programming environment will place the above populated table into EPROM during compile time.
//              Upon power on, the Startup code will copy the selected CPE entry (table row) into RAM for use.
//
//              These values are normalized for a 500Ah/12v battery, and when used are adjusted as follows:
//                      - Volts:   Increased (multiplied) by battery voltage scaling factor  - "systemVoltMult"
//                                 As determined by auto-sensing the battery voltage at startup (or forced override via ASCII command).
//                                 e.g.:  at 24v system would multiply all voltages above by 2
//
//                      - Amps:    Increased (multiplied) by battery capacity scaling factor  - "systemAmpMult".
//                                 As selected by the System Size DIP switches (or forced override via ASCII command).
//                                 e.g:  A 1000 Ah battery was selected -- a capacity of 2x --  and Amp values will be increased by 2x before being used.
//
//                      - Temps:   BAT_TEMP_1C_COMP is increased (multiplied) by battery scaling factor.
//                                 As determined by auto-sensing the battery voltage via the RAW Atmel A/D port.
//                                 e.g.:  at 24v system would multiply BAT_TEMP_1C_COMP above by 2
//
//              Entries that are NOT modified are:
//                      - Temp Limits:  MIN_TEMP_COMP_LIMIT, BAT_MIN_CHARGE_TEMP  and  BAT_MAX_CHARGE_TEMP are not scaled.
//
//                      - Time values:  No time value is modified
//                                        

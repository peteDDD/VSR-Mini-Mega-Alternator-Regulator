//      OSEnergy_Serial.cpp
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
#include "OSEnergy_Serial.h"
#include "Sensors.h"
#include "Flash.h"
#include "Alternator.h"

extern void WriteOLEDDynamicData(void);

char ibBuf[INBOUND_BUFF_SIZE + 1]; // Static buffer used to assemble inbound data in background. (+1 to allow for NULL terminator)
bool ibBufFilling = false;         // Static flag used to manage filling of ibBuf.  If this is set = true, we have indentified the start of
                                   // a new command 'string' and are working to get the rest of it.
                                   // During this time period, all normal 'status updates' outputs from the regulator will be suspended.
                                   // This is to make a more direct linkage between a command that asks for a response and the actual response.

//---- Local only helper function prototypes for Check_inbound() & Send_outbound()
bool getInt(char *buffer, int *dest, int LLim, int HLim);
bool getByte(char *buffer, uint8_t *dest, uint8_t LLim, uint8_t HLim);
bool getDurationMin(char *buffer, uint32_t *dest, int HLim);
bool getDurationS(char *buffer, uint32_t *dest, int HLim);
bool getDurationTh(char *buffer, uint32_t *dest, int HLim);
bool getFloat(char *buffer, float *dest, float LLim, float HLim);
bool getBool(char *buffer, bool *dest);
void append_string(char *dest, const char *src, int n);
void send_AOK(void);

// NOT NEEDED IN SHIELD VERSION -- bool CCx_handler(char *StrPtr);  //$CCN: - Change parameters in the CAN Configuration table
                                 //$CCR: - RESTORES CAN Configuration table to defaul
bool CPx_handler(char *StrPtr);  //$CPA:n - Change ACCEPT parameters in CPE user entry n (n = 7 or 8)
                                 //$CPO:n - Change OVERCHARGE parameters in CPE user entry n (n = 7 or 8)
                                 //$CPF:n - Change FLOAT parameters in CPE user entry n (n = 7 or 8) 
                                 //$CPP:n - Change POST-FLOAT parameters in CPE user entry n (n = 7 or 8) 
                                 //$CPE:n - Change EQUALIZE parameters in CPE user entry n (n = 7 or 8)
                                 //$CPB:n - Change BATTERY parameters in CPE user entry n (n = 7 or 8) 
                                 //$CPR:n - RESTORES Charge Profile ‘n’ to default values
//bool EBA_handler(char* StrPtr);  REDACTED  2-26-2018
bool EDB_handler(char *StrPtr);  //$EDB: - Enable DeBug serial strings
bool FRM_handler(char *StrPtr);  //$FRM: - Force Regulator Mode
bool MSR_handler(char *StrPtr);  //$MSR: - RESTORE all parameters (to as defined at program compile time)
bool RAS_handler(char *StrPtr);  //$RAS: - Request All Status back
bool RBT_handler(char *StrPtr);  //$RBT: - ReBooT system
bool RCP_handler(char *StrPtr);  //$RCP:n -Request to send back CPE entry #N (n=1..8)
                                 //$RCP:0 - Request to send back current selected CPE
bool SCx_handler(char *StrPtr);  //$SCA: - Changes ALTERNATOR parameters in System Configuration table
                                 //$SCT: - Changes TACHOMETER parameters in System Configuration table
                                 //$SCO: - Override features
                                 //$SCN: - Changes NAME (and PASSWORD)
                                 //$SCR: - RESTORES System Configuration table (+ Bluetooth) to default

void prep_AST(char *buffer);     //AST; -- ALTERNATOR STATUS
void prep_CPE(char *buffer, tCPS *cpsPtr, int Index); //CPE; -- CHARGE PROFILE ENTRY
void prep_default_CPE(char *buffer); //CPE; -- CHARGE PROFILE ENTRY
void prep_SST(char *buffer);     //SST; -- SYSTEM STATUS 
void prep_SCV(char *buffer);     //SCV; -- SYSTEM CONFIGURATION

typedef struct
{
    char command[3];
    bool (*handler)(char *strPtr); // Return TRUE and 'AOK' will be sent to serial port.
} tIBHandlers;

const tIBHandlers IBHandlers[] = {   // Lookup table used by check_inbound()
   // NOT NEEDED IN SHIELD VERSION (CAN) {{'C', 'C', '*'}, &CCx_handler}, //!! HEY!!!  Can we get this into PROGMEM sometime?
    {{'C', 'P', '*'}, &CPx_handler},
    // {{'E','B','A'},  &EBA_handler},  REDACTED 2-18-2018
    {{'E', 'D', 'B'}, &EDB_handler},
    {{'F', 'R', 'M'}, &FRM_handler},
    {{'M', 'S', 'R'}, &MSR_handler},
    {{'R', 'A', 'S'}, &RAS_handler},
    {{'R', 'B', 'T'}, &RBT_handler},
    {{'R', 'C', 'P'}, &RCP_handler},
    {{'S', 'C', '*'}, &SCx_handler},
    {{0, 0, 0}, NULL}};

typedef struct
{
    void (*preper)(char *strPtr);
    bool HP;
} tOBPrepers;

const tOBPrepers OBPrepers[] = {
    {&prep_AST, true},
    {&prep_default_CPE, false},
    {&prep_SST, false},
    {&prep_SCV, false},
    {NULL, false}};

//------------------------------------------------------------------------------------------------------
// Fill Inbound Buffer
//
//      This helper function will look to the Serial Port (TTL and Bluetooth) and assemble a complete command string
//      starting after the $ and up to the terminating CR/LF, or '@' char (the latter used to work around bug in Arduino IDE)
//      It is non-blocking and will return FALSE if the string entire string is not ready, or TRUE
//      once the string has been assembled.
//
//      If this is a CAN enabled target, the CAN terminal buffer will also be looked at.
//
//
//
//------------------------------------------------------------------------------------------------------
bool fill_ib_buffer()
{
    static uint8_t ibIndex = 0;       // This pointer retains its value between function calls.
    static uint32_t ibBufFillStarted; // Will contain the time the last 'start' of a string happened
    char c;

    while (Serial.available() > 0)
    {
        c = Serial.read();

        if (c == '$')
        {                        // "$" (start of command) character?
            ibIndex = 0;         //   Yes!  Initialize ibIndex variable
            ibBufFilling = true; //   And start storing the remaining characters into the buffer.
            ibBufFillStarted = millis();
            break; //   (The '$' character is not saved in the buffer...)
        }

        if (ibBufFilling == true)
        { // We are actively filling the inbound buffer, as we found the start '$' character.
            if (ibIndex >= (INBOUND_BUFF_SIZE - 1))
            {                         // Are we about to overflow the buffer array?  (Need to leave room for null terminator)
                ibBufFilling = false; //  Yes, abort and start looking again for a shorter one - something must have gone wrong....
                break;
            }

            if ((c == '\n') || (c == '@'))
            { // Did we find the termination character? ('@' for BUG in Arduino IDE, will not send CR\LF, so 'fake it' using @)
                if (ibIndex < 3)
                {                         // If terminator is found before the command string is AT LEAST 4 characters in length, something is wrong.
                    ibBufFilling = false; // ALL valid commands are in form of xxx:, so must be at least 4 characters long.  So, abandon this and start over.
                    break;
                }

                ibBufFilling = false; // All done storing strings his time around - start looking again for a $
                return (true);        // Return that we have a valid command string!
            }
            else
            {
                ibBuf[ibIndex++] = c; // Just a plane character, put it into the buffer
                ibBuf[ibIndex] = 0;   // And add a null terminate just-in-case
            }
        }
    } // while

    if ((ibBufFilling == true) && (IB_BUFF_FILL_TIMEOUT != 0UL) && //  Have we timed-out waiting for a complete command string to be send to us?
        ((millis() - ibBufFillStarted) > IB_BUFF_FILL_TIMEOUT))
    {
        ibBufFilling = false; //  Yes, abort this and start looking for a new command initiator character ('$')
    }

    return (false); // We have read all there is in the Serial queue, so go back and try to get the rest later.
} //fill_ib_buffer

//------------------------------------------------------------------------------------------------------
// Check Inbound
//
//      This function will check the serial terminal for incoming communications, either via the Bluetooth or
//      an attached Debug terminal.  It is used not only during debug, but also during operation to allow for
//      simple user configuration of the device (ala adjusting custom charging profiles - to be stored in FLASH)
//
//------------------------------------------------------------------------------------------------------
void check_inbound()
{
    int i;

    //----- Serial Port (TTL) receiving code.  Read the 'packet', verify it, and make changes as needed.
    //      Store those changes in FLASH if needed.

    if (fill_ib_buffer() == true)
    { // Assemble a string, and see if it is "completed" yet..

        if (ibBuf[3] != ':')
            return; // ALL valid commands have a ":" in the 4th position.

        if (chargingState == warm_up)
            set_charging_mode(warm_up); // User is trying to bench-conf the regulator, let them do all they need to do before we
                                          // move on and start ramping.

        for (i = 0; IBHandlers[i].command[0] != 0; i++)
        { // Scan the table, looking for match

            if ((IBHandlers[i].command[0] != ibBuf[0]) ||
                (IBHandlers[i].command[1] != ibBuf[1]))
                continue; // No match on 1st two characters, keep looking.

            if ((IBHandlers[i].command[2] != '*') &&
                (IBHandlers[i].command[2] != ibBuf[2]))
                continue; // Last table entery might be a wildcard, if not it must also match

            if (IBHandlers[i].handler(ibBuf))
            {               // Got a match!  Call the matching command procedure
                send_AOK(); //   -- If command procedure was accepted, send user acknowledgment.
                return;     //   And we are all done now.
            }
        }
    }
} //check_inbound

//------------------------------------------------------------------------------------------------------
// Inbound command string handlers
//
//      The following functions are called via check_inbound() to execute their matching command.
//      If the called functions wishes 'AOK' to be sent to the user, return TRUE.
//
//------------------------------------------------------------------------------------------------------

//--------- CC*:  Something to do with the CAN Configuration . . .  (This one is a wild-card)
//
bool CCx_handler(char *StrPtr)
{
    return (false); // This command not supported in non-CAN enabled devices
}

//--------- CP*:  Something about changing a Charge Profile  (This one is a wild-card)
bool CPx_handler(char *StrPtr)
{
    int8_t index;
    int j;
    tCPS buffCP;

    //------  Something about changing a Charge Profile
    //        Note that I only allow changes to CPE #7 or 8 (index = 6 or 7), if you want to change any other ones you
    //        will need to modify the source code.  This is to help prevent accidents.

    index = ibBuf[4] - '1';                   //   Which one?  There SHOULD be a number following, go get it.
    if ((index < (MAX_CPES - CUSTOM_CPES)) || //   Make sure it is within range (7 or 8), abort if not..
        (index >= MAX_CPES))                  //   (Only the last two Charge Profiles are allowed to be modified)
        return (false);

    if (read_CPS_EEPROM(index, &buffCP) != true) // Go get requested index entry, and check to see if it is valid.
        transfer_default_CPS(index, &buffCP);    //   Nope, not a good one in EEPROM, so fetch the default entry from FLASH

    if (systemConfig.CONFIG_LOCKOUT != 0)
        return (false); // If system is locked-out, do not allow any changes...

    //  Now, let's move on and see what they wish to change...

    switch (ibBuf[2])
    {
    case 'A': //   Change  ACCEPT parameters in CPE user entry n
              //   $CPA:n  <VBat Set Point>, <Exit Duration>, <Exit Amps>
        if (!getFloat((ibBuf + 5), &buffCP.ACPT_BAT_V_SETPOINT, 0.0, 20.0))
            return (false); //  20 volts MAX  - If any problems, just abort this command.
        if (!getDurationMin(NULL, &buffCP.EXIT_ACPT_DURATION, (60 * 10)))
            return (false); //  10 Hours MAX, converted into mS for use
        if (!getInt(NULL, &buffCP.EXIT_ACPT_AMPS, -1, 200))
            return (false); // 200 Amps MAX
        if (!getInt(NULL, &j, 0, 0))
            return (false); // Place holder for future dV/dt
        break;              // Got it all, drop down and finish storing it into EEPROM

    case 'O': // Change OVERCHARGE  parameters in CPE user entry n
              //   $CPO:n    <Exit Amps>, <Exit Duration>, <Exit  VBat>

        if (!getInt((ibBuf + 5), &buffCP.LIMIT_OC_AMPS, 0, 50))
            return (false); // 50 Amps MAX
        if (!getDurationMin(NULL, &buffCP.EXIT_OC_DURATION, (60 * 10)))
            return (false); // 10 Hours MAX, converted into mS for use
        if (!getFloat(NULL, &buffCP.EXIT_OC_VOLTS, 0.0, 20.0))
            return (false); // 20 volts MAX
        if (!getInt(NULL, &j, 0, 0))
            return (false); // Place holder for future dV/dt
        break;

    case 'F': // Change FLOAT  parameters in CPE user entry n
              //   $CPF:n    <VBat Set Point>, <Exit Duration>, <Revert Amps> , <Revert Volts>

        if (!getFloat((ibBuf + 5), &buffCP.FLOAT_BAT_V_SETPOINT, 0.0, 20.0))
            return (false); //  20 volts MAX
        if (!getInt(NULL, &buffCP.LIMIT_FLOAT_AMPS, -1, 50))
            return (false);
        if (!getDurationMin(NULL, &buffCP.EXIT_FLOAT_DURATION, (60 * 500)))
            return (false); // 500 Hours MAX, converted into mS for use
        if (!getInt(NULL, &buffCP.FLOAT_TO_BULK_AMPS, -300, 0))
            return (false); // 300 Amps MAX
        if (!getInt(NULL, &buffCP.FLOAT_TO_BULK_AHS, -250, 0))
            return (false);
        if (!getFloat(NULL, &buffCP.FLOAT_TO_BULK_VOLTS, 0.0, 20.0))
            return (false); //  20 volts MAX
        break;

    case 'P': // Change POST-FLOAT parameters in CPE user entry n
              //   $CPP:n     <Exit Duration>, <Revert VBat>

        if (!getDurationMin((ibBuf + 5), &buffCP.EXIT_PF_DURATION, (60 * 500)))
            return (false); // 500 Hours MAX, converted into mS for use
        if (!getFloat(NULL, &buffCP.PF_TO_BULK_VOLTS, 0.0, 20.0))
            return (false); //  20 volts MAX to trigger moving back to Bulk mode (via Ramp)
        if (!getInt(NULL, &buffCP.PF_TO_BULK_AHS, -250, 0))
            return (false);
        break;

    case 'E': // Change EQUALIZE  parameters in CPE user entry n
              //   $CPE:n     <VBat Set Point>, < Max Amps >, <Exit Duration>, <Exit Amps>

        if (!getFloat((ibBuf + 5), &buffCP.EQUAL_BAT_V_SETPOINT, 0.0, 25.0))
            return (false); //   25 volts MAX for Equalization.
        if (!getInt(NULL, &buffCP.LIMIT_EQUAL_AMPS, 0, 50))
            return (false); //  50A MAX while equalizing...
        if (!getDurationMin(NULL, &buffCP.EXIT_EQUAL_DURATION, 240))
            return (false); //  240 MINUTES MAX, converted into mS for use
        if (!getInt(NULL, &buffCP.EXIT_EQUAL_AMPS, 0, 50))
            return (false); //   50 Amps MAX for Equalization
        break;

    case 'B': // Change BATTERY parameters in CPE user entry n
              //   $CPB:n     <VBat Comp per 1c>, < Min Comp Temp >, <Max Charge Temp>

        if (!getFloat((ibBuf + 5), &buffCP.BAT_TEMP_1C_COMP, 0.0, 0.1))
            return (false); //   0.1v / deg C MAX.  (Note, this is for a normalized 12v battery)
        if (!getInt(NULL, &buffCP.MIN_TEMP_COMP_LIMIT, -30, 40))
            return (false); //  -30c to 40c range should be good???
        if (!getInt(NULL, &buffCP.BAT_MIN_CHARGE_TEMP, -50, 10))
            return (false);
        if (!getInt(NULL, &buffCP.BAT_MAX_CHARGE_TEMP, 20, 95))
            return (false); //  Cap at 95 to protect NTC sensor? (Esp Epoxy filling?)
        break;

    case 'R':                          // $CPR:n  RESTORES Charge Profile table entry 'n' to default
        write_CPS_EEPROM(index, NULL); // Erase selected saved systemConfig structure in the EEPROM (if present)
        return (true);                 // Let user know we understand.

    default:
        return (false); // Not a know Charge Profile xx command
    }                   //switch

    write_CPS_EEPROM(index, &buffCP); // Save back the new entry
    return (true);                    // Let user know we understand.  User must issue $RBT command for changes                                                                     //   to be loaded into regulators working memory.
} //CPx_handler

//--------- $EDB:  Enable DeBug ASCII string
bool EDB_handler(char *StrPtr)
{

    SDMCounter = SDM_SENSITIVITY; // Set up the counters and turn on the DBG string switch!
    sendDebugString = true;
    return (true);
} //EDB_handler

//--------- $FRM:  Force Regulator Mode ASCII string
bool FRM_handler(char *StrPtr)
{

    switch (ibBuf[4])
    { // SHOULD be an ASCII character following the ':', if note, will hold NULL
    case 'B':
        set_charging_mode(bulk_charge); //  B   = Force into BULK mode.
        break;

    case 'A':
        set_charging_mode(acceptance_charge); //  A   = Force into ACCEPTANCE mode.
        break;

    case 'O':
        set_charging_mode(overcharge_charge); //  O   = Force into OVER-CHARGE mode.
        break;

    case 'F':
        set_charging_mode(float_charge); //  F   = Force into FLOAT mode.
        break;

    case 'P':
        set_charging_mode(post_float); //  P   = Force into POST-FLOAT mode.
        break;

    case 'E':
        set_charging_mode(equalize); //  E   = Force into EQUALIZE mode.
        break;

    default:
        return (false); // Not something we understand..
    }

    return (true);
} //FRM_handler

//--------- $MSR:  Master System Restore
bool MSR_handler(char *StrPtr)
{
    if (systemConfig.CONFIG_LOCKOUT != 0)
        return (false); // If system is locked-out, do not allow restore.

    send_AOK();    //  (Need to send this string now, as we will not return from restore_all() function.
    restore_all(); // Erase all the EEPROM and reset. (we will not come back from here)

    return (true); // Keep compiler from complaining, even if we will never get here.
} //MSR_handler

//--------- $RAS:  They want a copy of all the status strings
bool RAS_handler(char *StrPtr)
{
    send_outbound(true); //  Send them all!
    return (false);      //    Send-outbound() will finish things up with an 'AOK' message.
} //RAS_handler

//--------- $RBT:  ReBooT?  Yea, they are 'requesting' a reboot!
bool RBT_handler(char *StrPtr)
{
    if (systemConfig.CONFIG_LOCKOUT != 0)
        return (false); // If system is locked-out, do not allow
    reboot();           //  Else just reboot (will not return from there...)

    return (true); // Keep compiler from complaining, even if we will never get here.
} //RBT_handler

//--------- $RCP:n  Request Charge Profile
bool RCP_handler(char *StrPtr)
{
    char charBuffer[OUTBOUND_BUFF_SIZE + 1];
    tCPS buffCP;
    int8_t index;

    index = ibBuf[4] - '1'; //   Which one?  There SHOULD be a number following, go get it.
                            //     adjusting for "0-origin" of Arrays.
                            //     (Note that if user did not type a number in, ala sent "$RCP:",  ibBuf[4] will contain the NULL or 0.
    if (index == -1)
        index = cpIndex; //   If user sent '0' (i will contain -1), this means they want the 'current' one.

    if ((index >= MAX_CPES) || (index < 0))
        return (false); //   And make sure it is in bounds of the array (This will also trap-out any non-numeric character --or no character-- following the ':')

    if (read_CPS_EEPROM(index, &buffCP) != true) //   See if there is a valid user modified CPE in EEPROM we should be using.
        transfer_default_CPS(index, &buffCP);    //   No, so get the correct entry from the values in the FLASH (PROGMEM) store.

    prep_CPE(charBuffer, &buffCP, index); //   And Finally,  assemble the string to send out requested information.
    ASCII_write(charBuffer);              //   Send it out via Serial port

    return (true);
} //RCP_handler

//--------- SC*:  Something about changing a System Config  (This one is a wild-card)
bool SCx_handler(char *StrPtr)
{
    bool dummy;
    tSCS buffSC;

    if (systemConfig.CONFIG_LOCKOUT != 0)
        return (false); // If system is locked-out, do not allow any changes...

    if (read_SCS_EEPROM(&buffSC) != true) // Prime the work buffer with anything already in FLASH
        buffSC = systemConfig;            //   (or the default values if FLASH is empty / invalid)

    switch (ibBuf[2])
    {

    case 'A': // Changes ALTERNATOR parameters in System Configuration table
              // $SCA: <32v?>, < Alt Target Temp >, <Alt Derate (norm) >,
              //       <Alt Derate (small) >,<Alt Derate (half) >,<PBF>,
              //       <Alt Amp Cap.>, <System Watt Cap. >, <BAT Amp Shunt Ratio>, <ALT Amp Shunt Ratio>

        if (!getBool((ibBuf + 4), &dummy))
            return (false); // Was 'Favor32v' has been redacted, ignor it.
        if (!getByte(NULL, &buffSC.ALT_TEMP_SETPOINT, 15, 120))
            return (false);
        if (!getFloat(NULL, &buffSC.ALT_AMP_DERATE_NORMAL, 0.1, 1.0))
            return (false);
        if (!getFloat(NULL, &buffSC.ALT_AMP_DERATE_SMALL_MODE, 0.1, 1.0))
            return (false);
        if (!getFloat(NULL, &buffSC.ALT_AMP_DERATE_HALF_POWER, 0.1, 1.0))
            return (false);
        if (!getInt(NULL, &buffSC.ALT_PULLBACK_FACTOR, -1, 10))
            return (false);
        if (!getInt(NULL, &buffSC.ALT_AMPS_LIMIT, -1, 500))
            return (false);
        if (!getInt(NULL, &buffSC.ALT_WATTS_LIMIT, -1, 20000))
            return (false);
        if (!getInt(NULL, &buffSC.BAT_AMP_SHUNT_RATIO, 500, 20000))
            return (false);
        if (!getInt(NULL, &buffSC.ALT_AMP_SHUNT_RATIO, 500, 20000))
            return (false);
        if (!getBool(NULL, &buffSC.REVERSED_BAT_SHUNT))
            return (false);
        if (!getBool(NULL, &buffSC.REVERSED_ALT_SHUNT))
            return (false);
        if (!getInt(NULL, &buffSC.ALT_IDLE_RPM, 0, 1500))
            buffSC.ALT_IDLE_RPM = 0;
        if (!getInt(NULL, &buffSC.ENGINE_WARMUP_DURATION, 15, 600))
            buffSC.ENGINE_WARMUP_DURATION = 15; //DUBLER MOD 080320 changed to 15 from 60
        if (!getByte(NULL, &buffSC.REQURED_SENSORS, 0, 255))
            buffSC.REQURED_SENSORS = 0;

        break;

    case 'T': // Changes TACHOMETER parameters in System Configuration table
              // $SCT: <Alt poles>, < Eng/Alt drive ratio >, <Field Tach Min>, <Forced TachMode>

        if (!getByte((ibBuf + 4), &buffSC.ALTERNATOR_POLES, 2, 25))
            return (false);
        if (!getFloat(NULL, &buffSC.ENGINE_ALT_DRIVE_RATIO, 0.5, 20.0))
            return (false);
        if (!getInt(NULL, &buffSC.FIELD_TACH_PWM, -1, 30))
            return (false);
        if (!getBool(NULL, &buffSC.FORCED_TM))
            return (false);

        if (buffSC.FIELD_TACH_PWM > 0)
            buffSC.FIELD_TACH_PWM = min(((buffSC.FIELD_TACH_PWM * FIELD_PWM_MAX) / 100), MAX_TACH_PWM);
        // Convert any entered % of max field into a raw PWM number.
        break;

   
    case 'O': // OVERRIDE DIP Switch settings for CP_Index and BC_Index.
              // $SCO:  <CP_Index>, <BC_Index >, <SV_Override>, <Lockout>

        if (!getByte((ibBuf + 4), &buffSC.CP_INDEX_OVERRIDE, 0, 8))
            return (false);
        if (!getFloat(NULL, &buffSC.BC_MULT_OVERRIDE, 0, 10.0))
            return (false);
        if (!getFloat(NULL, &buffSC.SV_OVERRIDE, 0, 4.0))
            return (false);
        if (!getByte(NULL, &buffSC.CONFIG_LOCKOUT, 0, 2))
            return (false);

        break;

    case 'R':                   // RESTORES System Configuration table to default
        write_SCS_EEPROM(NULL); // Erase any saved systemConfig structure in the EEPROM
        return (true);          // Let user know we understand.

    default:
        return (false);
    } //switch

    write_SCS_EEPROM(&buffSC); // Save back the new Charge profile
    return (true);             // Let user know we understand.  User must issue $RBT command for changes
                               //   to be loaded into regulators working memory.
} //SCx_handler

//---- Helper functions for Check_inbound()
bool getInt(char *buffer, int *dest, int LLim, int HLim)
{
    char *cp;
    cp = strtok(buffer, ",");
    if (cp == NULL)
        return (false);
    *dest = constrain(atoi(cp), LLim, HLim);
    return (true);
}

bool getByte(char *buffer, uint8_t *dest, uint8_t LLim, uint8_t HLim)
{
    char *cp;
    cp = strtok(buffer, ",");
    if (cp == NULL)
        return (false);
    *dest = (uint8_t)constrain(atoi(cp), LLim, HLim);
    return (true);
}

bool getDurationMin(char *buffer, uint32_t *dest, int HLim)
{ // Time in Minutes converted to mS
    char *cp;
    cp = strtok(buffer, ",");
    if (cp == NULL)
        return (false);
    *dest = (uint32_t)(constrain(atoi(cp), 0, HLim)) * 60000UL; // Noticed, EVERY duration has this 60000UL multiplier and 0 LLim, so no need to pass as separate parameter...
    return (true);
}

bool getDurationS(char *buffer, uint32_t *dest, int HLim)
{ // Time in Seconds converted to mS
    char *cp;
    cp = strtok(buffer, ",");
    if (cp == NULL)
        return (false);
    *dest = (uint32_t)(constrain(atoi(cp), 0, HLim)) * 1000UL;
    return (true);
}

bool getDurationTh(char *buffer, uint32_t *dest, int HLim)
{ // Time in Tenths of a second converted to mS
    char *cp;
    cp = strtok(buffer, ",");
    if (cp == NULL)
        return (false);
    *dest = (uint32_t)(constrain(atoi(cp), 0, HLim)) * 100UL;
    return (true);
}

bool getFloat(char *buffer, float *dest, float LLim, float HLim)
{
    char *cp;
    cp = strtok(buffer, ",");
    if (cp == NULL)
        return (false);
    *dest = constrain(atof(cp), LLim, HLim);
    return (true);
}

bool getBool(char *buffer, bool *dest)
{
    char *cp;
    cp = strtok(buffer, ",");
    if (cp == NULL)
        return (false);
    *dest = (atoi(cp) == 1);
    return (true);
}

void send_AOK(void)
{
    ASCII_write("AOK;\r\n");
}

//------------------------------------------------------------------------------------------------------
// Send Outbound
//
//      This function will send to the Serial Terminal the current system status.  It is used to send
//      information primarily via the Bluetooth to an external HUI program.
//
//      If pushAll was requested, all the satus strings will be sent out in order.  This is usefull in the case of FAULTED condition.
//      as well as the $RAS: command.
//
//      FALSE is retuned if send_outbound has no more strings it wants to send out, else TRUE is returned indicating there are
//      more strings to be sent as a result of a prior call with pushAll
//
//
//------------------------------------------------------------------------------------------------------
bool send_outbound(bool pushAll)
{
    char charBuffer[OUTBOUND_BUFF_SIZE + 1]; // Large working buffer to assemble strings before sending to the serial port.
    uint8_t static indexMajor = 0;           // Index into OBPrepers[] array of which message to send next.
    uint8_t static indexMinor = 0;
    uint8_t static minorCounter = 0; // Used to see if it is time to send out a MINOR message this time around.
    uint8_t index;
    uint32_t static lastStatusSent = 0U; // When was the Status last sent?
    int8_t static pushingAllIndex = -1;  // If we have been asked to push-all, this will contain the index to the next 'message' we should push out.
                                         //  -1 = not pushing all.

    if (pushAll)
        pushingAllIndex = 0; // We are being asked to push-all, start with the AST and work up.

    if (pushingAllIndex >= 0)
    { // Doing  'Push-all' block?
        if (OBPrepers[pushingAllIndex].preper == NULL)
        {                         // Are we at the end of the list?
            send_AOK();           // Yup, send out the AOK now and be done.
            pushingAllIndex = -1; // And the next one is the end of the list, so we are all done after the one.
            return (false);
        }

        index = pushingAllIndex++; // Yes, and this is the one we will do this time.
    }
    else
    { // Not pushing all
        //--    OK, we will be sending out a normal message.  See if it is time  and if so which one  (Minor or Major).
        if ((millis() - lastStatusSent) < UPDATE_STATUS_RATE)
            return (false); // Time to send Update packet?  No, not just yet.
        if (ibBufFilling == true)
            return (false); // Also, we suspend the sending of status updates while a new command is being assembled.
                            //   (This way there is no confusion over data received from the regulator as to if it)

        if (++minorCounter >= 10)
        { // Time for a Minor message, work the Minor index.
            do
            { // Looking for a non-high prioity message
                indexMinor++;
                if (OBPrepers[indexMinor].preper == NULL) // At end of list...
                    indexMinor = 0;
            } while (OBPrepers[indexMinor].HP != false);

            index = indexMinor; // Got one!  This is the one we will use.
            minorCounter = 0;
        }
        else
        { // Doing a Major this time.
            do
            { // Looking for a high prioity message
                indexMajor++;
                if (OBPrepers[indexMajor].preper == NULL) // At end of list...
                    indexMajor = 0;
            } while (OBPrepers[indexMajor].HP != true);

            index = indexMajor; // Got one!  This is the one we will use.
        }
    }

    OBPrepers[index].preper(charBuffer); // Envoke the selected message creation function
    ASCII_write(charBuffer);             // And send it out.  (Even if Null - will not hurt anything.)

#ifdef USE_OLED
  if(chargingState!= FAULTED)
  {
    WriteOLEDDynamicData(); // OUTPUT UPDATE TO THE I2C LCD
  }  
#endif

    lastStatusSent = millis();
    return (pushingAllIndex != -1); // Let caller know if there are more push-all messages taht need to go.
} //send_outbound

//------------------------------------------------------------------------------------------------------
// Prep Outbound strings
//
//      These functions will assemble the outbound strings into the passed buffer.
//      Note that the passed buffer MUST BE AT LEAST 'OUTBOUND_BUFF_SIZE' in size.
//      If the called message is not supported by the target compiled device, no change will be made
//      to the passed buffer.
//
//
//------------------------------------------------------------------------------------------------------

void prep_AST(char *buffer)
{ // AST: Alternator Status ASCII string.
    snprintf_P(buffer, OUTBOUND_BUFF_SIZE, PSTR("AST;,%d.%02d, ,%s,%s,%s,%d, ,%s,%d,%d,%d, ,%d,%d, ,%d, ,%s,%d,%d,%d\r\n"),
               (int)(generatorLrRunTime / (3600UL * 1000UL)),       // Runtime Hours
               (int)((generatorLrRunTime / (3600UL * 10UL)) % 100), // Runtime 1/100th

               float2string(measuredBatVolts, 2),
               float2string(measuredAltAmps, 1),
               float2string(measuredBatAmps, 1),
               measuredAltWatts,

               float2string(targetBatVolts, 2),
               (int)targetAltAmps,
               targetAltWatts,
               chargingState,

               measuredBatTemp, // In deg C
               //max(measuredAltTemp, measuredAlt2Temp),
               measuredAltTemp,

               measuredRPMs,

               float2string(measuredAltVolts, 2),
               measuredFETTemp,
               measuredFieldAmps,
               ((100 * fieldPWMvalue) / FIELD_PWM_MAX));
} //prep_AST

//void prep_GST(char *buffer) {                                                           // Assembled the Generator Status ASCII string.
//}

//void prep_BST(char *buffer) {                                                           // BST:  Battery Status ASCII string.
//}

void prep_default_CPE(char *buffer)
{
    prep_CPE(buffer, &chargingParms, cpIndex);
}

void prep_CPE(char *buffer, tCPS *cpsPtr, int index)
{
    snprintf_P(buffer, OUTBOUND_BUFF_SIZE, PSTR("CPE;,%d,%s,%d,%d,%d, ,%d,%d,%s,%d, ,%s,%d,%d,%d,%d,%s, ,%d,%s,%d, ,%s,%d,%d,%d, ,%s,%d,%d,%d\r\n"),
               (index + 1), // Convert from "0-origin" of array indexes for display
               float2string(cpsPtr->ACPT_BAT_V_SETPOINT, 2),
               (unsigned int)(cpsPtr->EXIT_ACPT_DURATION / 60000UL), // Show time running in Minutes, as opposed to mS
               cpsPtr->EXIT_ACPT_AMPS,
               (int)0, // Place holder for future dV/dT exit parameter, hard coded = 0 for now.

               cpsPtr->LIMIT_OC_AMPS,
               (unsigned int)(cpsPtr->EXIT_OC_DURATION / 60000UL),
               float2string(cpsPtr->EXIT_OC_VOLTS, 2),
               (int)0, // Place holder for future dV/dT exit parameter, hard coded = 0 for now.

               float2string(cpsPtr->FLOAT_BAT_V_SETPOINT, 2),
               cpsPtr->LIMIT_FLOAT_AMPS,
               (unsigned int)(cpsPtr->EXIT_FLOAT_DURATION / 60000UL),
               cpsPtr->FLOAT_TO_BULK_AMPS,
               cpsPtr->FLOAT_TO_BULK_AHS,
               float2string(cpsPtr->FLOAT_TO_BULK_VOLTS, 2),

               (unsigned int)(cpsPtr->EXIT_PF_DURATION / 60000UL), //  Show in Minutes, as opposed to mS,
               float2string(cpsPtr->PF_TO_BULK_VOLTS, 2),
               cpsPtr->PF_TO_BULK_AHS,

               float2string(cpsPtr->EQUAL_BAT_V_SETPOINT, 2),
               cpsPtr->LIMIT_EQUAL_AMPS,
               (unsigned int)(cpsPtr->EXIT_EQUAL_DURATION / 60000UL), //  Show in Minutes, as opposed to mS,
               cpsPtr->EXIT_EQUAL_AMPS,

               float2string(cpsPtr->BAT_TEMP_1C_COMP, 3),
               cpsPtr->MIN_TEMP_COMP_LIMIT,
               cpsPtr->BAT_MIN_CHARGE_TEMP,
               cpsPtr->BAT_MAX_CHARGE_TEMP);
} //prep_CPE

//void prep_CST(char *buffer) {   // Prep  the CAN Control Variable string. (Only on CAN enabled regulator)
//}

//void prep_GNC(char *buffer) {                                                           // Assembled the Generator Configuration  ASCII string.
//}

//void prep_GSC(char *buffer) {                                                           // Assembled the Generator Starter Configuration ASCII string.
//}

//void prep_GTC(char *buffer) {                                                           // Assembled the Generator Throttle Configuraiton ASCII string.
//}

// Comment out prep_NPC.  Only used in ver 2 controller with Bluetooth
/*
void prep_NPC(char *buffer)
{                                                                          // Prep the System Control Variable string.  Pass in working buffer of OUTBOUND_BUFF_SIZE
    snprintf_P(buffer, OUTBOUND_BUFF_SIZE - 3, PSTR("NPC;,%1u,%s,%s\r\n"), // Prime things with the initial ASCII string..
               systemConfig.USE_BT,
               systemConfig.DEVICE_NAME,
               systemConfig.DEVICE_PSWD);
}
*/

void prep_SCV(char *buffer)
{ // Prep the System Control Variables.
    snprintf_P(buffer, OUTBOUND_BUFF_SIZE, PSTR("SCV;,%1u,%1u,%1u,%s,%s,%1u, ,%d,%s,%s,%s,%d, ,%d,%d, ,%d,%s,%d,%d, ,%d,%d,%d,%u\r\n"),
               systemConfig.CONFIG_LOCKOUT,
               systemConfig.REVERSED_BAT_SHUNT,
               systemConfig.REVERSED_ALT_SHUNT,
               float2string(systemConfig.SV_OVERRIDE, 2),
               float2string(systemConfig.BC_MULT_OVERRIDE, 2),
               systemConfig.CP_INDEX_OVERRIDE,

               systemConfig.ALT_TEMP_SETPOINT,
               float2string(systemConfig.ALT_AMP_DERATE_NORMAL, 2),
               float2string(systemConfig.ALT_AMP_DERATE_SMALL_MODE, 2),
               float2string(systemConfig.ALT_AMP_DERATE_HALF_POWER, 2),
               systemConfig.ALT_PULLBACK_FACTOR,

               systemConfig.ALT_AMPS_LIMIT,
               systemConfig.ALT_WATTS_LIMIT,

               systemConfig.ALTERNATOR_POLES,
               float2string(systemConfig.ENGINE_ALT_DRIVE_RATIO, 3),
               systemConfig.BAT_AMP_SHUNT_RATIO,
               systemConfig.ALT_AMP_SHUNT_RATIO,

               systemConfig.ALT_IDLE_RPM,
               ((systemConfig.FIELD_TACH_PWM > 0) ? ((100 * systemConfig.FIELD_TACH_PWM) / FIELD_PWM_MAX) : systemConfig.FIELD_TACH_PWM),
               systemConfig.ENGINE_WARMUP_DURATION,
               systemConfig.REQURED_SENSORS);
} //prep_SCV

void prep_SST(char *buffer)
{
    snprintf_P(buffer, OUTBOUND_BUFF_SIZE - 3, PSTR("SST;,%s, ,%1u,%1u, ,%d,%s,%s, ,%d,%d, ,%d,%d, ,%1u\r\n"), //  System Status
               firmwareVersion,

               smallAltMode,
               tachMode,

               (int)(cpIndex + 1),
               float2string(systemAmpMult, 2),
               float2string(systemVoltMult, 2),

               altCapAmps,
               altCapRPMs,

               (int)((accumulatedASecs / 3600UL) * (ACCUMULATE_SAMPLING_RATE / 1000UL)), // Convert into actual AHs
               (int)((accumulatedWSecs / 3600UL) * (ACCUMULATE_SAMPLING_RATE / 1000UL)), // Convert into actual WHs

               systemConfig.FORCED_TM);
} //prep_SST

//----  Helper function for Send_outbound();
//
// floatString converts a float to a string
// We return one of a set of static buffers
//  (Thank-you to Ron K. for this concept / code stub)

char *float2string(float v, uint8_t decimals)
{
    const int OUTPUT_BUFS = 7; // maximum number of floats in a single sprintf
    const int MAX_OUTPUT = 13; // largest possible output string
    float absV;
    static char outputBuffers[OUTPUT_BUFS][MAX_OUTPUT + 1];
    static uint8_t callCount;
    char formatter[] = "%d.%0_d";

    char *pos = outputBuffers[++callCount % OUTPUT_BUFS]; // Select buffer to use this time (Round-robben)
    //__ return (dtostrf(v,MAX_OUTPUT , decimals, pos));    //<<-- This works GREAT, but is about 1.2K in code size larger then what is here.

    formatter[5] = decimals + '0'; // create the number for the # of decimal characters we ultimatly want.

    unsigned int mult = 1; // Calc multplier for fractional portion of number
    uint8_t multleft = decimals;
    while (multleft--)
    {
        mult *= 10;
    }

    absV = v;
    if (absV < 0)
        absV *= -1;

    snprintf(pos, MAX_OUTPUT, formatter,
             (int)v,                                // Whole number, with its sign
             (unsigned int)((absV * mult)) % mult); // Fraction, w/o a sign.

    return pos;
} //float2string

void append_string(char *dest, const char *src, int n)
{
    int i;
    int offset;
    int limit;

    offset = strlen(dest); // Where should we start appending?
    limit = n - offset;    // Figure out how much room is left in the destination string

    for (i = 0; (i < limit) && *(src + i); i++)
    {                                      // As long as we have not exceeded the remaining room (leaving room for a final \o), and have not reached the end of the 'src' string
        *(dest + i + offset) = *(src + i); // copy over the character from the Source to the Destination.
    }

    *(dest + i + offset) = '\0'; // NULL terminator goes at the end.
} //append_string

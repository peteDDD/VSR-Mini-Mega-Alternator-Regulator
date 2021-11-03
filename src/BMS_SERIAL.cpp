// BMS_SERIAL.cpp
//
//      Copyright (c) 2021 by Pete Dubler
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
//*****************************************************************************************
//
//    Monitors the BMS_SERIAL_PORT for input indicating forced shutdown
//
//*****************************************************************************************

//TODO complete code here

bool checkBMS_Serial_In(void)
{
// check BMS_SERIAL_IN port for input
//TODO -- TO BE DEVELOPED
// parse the input

// if BMS shutdown command received, turn on FEATURE_OUT_LIFEPO_SHUTDOWN_ALARM, and return true

/*
#ifdef FEATURE_OUT_LIFEPO_SHUTDOWN_ALARM
   digitalWrite(FEATURE_OUT_LIFEPO_SHUTDOWN_ALARM_PORT,FOP_ON);
#endif
return(true);
*/

// else, turn off LIFEPO_SHUTDOWN_ALARM_PORT, if feature-out is active, and return false
/*#ifdef FEATURE_OUT_LIFEPO_SHUTDOWN_ALARM
   digitalWrite(FEATURE_OUT_LIFEPO_SHUTDOWN_ALARM_PORT,FOP_OFF);
#endif
*/
return(false);  // do not do forced float


}
//      Flash.h
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

#ifndef _FLASH_H_
#define _FLASH_H_

#include "Config.h" // Pick up the specific structures and their sizes for this program.
#include "Sensors.h"
#include "CPE.h"

void transfer_default_CPS(uint8_t index, tCPS *cpsPtr);
void write_CPS_EEPROM(uint8_t index, tCPS *cpsPtr);
void write_SCS_EEPROM(tSCS *scsPtr);
void write_CAL_EEPROM(tCAL *calPtr);

bool read_CPS_EEPROM(uint8_t index, tCPS *cpsPtr);
bool read_SCS_EEPROM(tSCS *scsPtr);
bool read_CAL_EEPROM(tCAL *calPtr);

void restore_all(void);
void commit_EEPROM(void);

//----- The following are used during the management of the EEPROM, to validate saved data
//      If a structure format is changed, change these keys to help invalidate prior saved data in the CPU.
//      Note:  All CRC values changed in v1.2.0, all structeures expanded to include reserve space.

#define SCS_ID1_K 0xF3CA // Key value that should be contained in the EEPROM EKEY structure to indicate a valid sytemConfig structure has been saved
#define SCS_ID2_K 0x73D3
#define CPS_ID1_K 0x6B9C // Key value that should be continued in the EEPROM EKEY structure to indicate a valid CPE structure has been saved
#define CPS_ID2_K 0x0A47 // (Changed in 0.2.0 - as CPS was expanded)
#define CAL_ID1_K 0xF9AC // Calibration Structure
#define CAL_ID2_K 0x0A97

//-----  EEPROM is laid out in this way:  (I was not able to get #defines to work, as the preprocessor seems to not be able to handle sizeof() )
//       CAL is placed 1st in hopes it will not be invalidated as revs change.
//          Note the addition of a 32 byte 'reserved' space after the CAL structure, for future use.
//
//      Order in EEPROM:
//          CAL
//          Reserved / expansion space (oritionaly 32 bytes)
//          CPS
//          SCS
//          CCS

#define EKEY_FLASH_LOCATION 0
#define CAL_FLASH_LOCATION (sizeof(tEKEY))
#define CPS_FLASH_LOCATION (sizeof(tEKEY) + sizeof(tCAL) + 32 + (sizeof(tCPS) * index))
#define SCS_FLASH_LOCATION (sizeof(tEKEY) + sizeof(tCAL) + 32 + (sizeof(tCPS) * MAX_CPES))

#

typedef struct
{ // EEPROM Key Structure - used to validate storage of CPS and SCV structures in the EEPROM

   unsigned SCS_ID1;           // Unique bit pattern that must match to indicate an systemConfig structure has been placed
   unsigned SCS_ID2;           // into the EEPROM at one time.
   unsigned CPS_ID1[MAX_CPES]; // Unique bit pattern that must match to indicate an ChargeParms structure has been placed
   unsigned CPS_ID2[MAX_CPES]; // into the EEPROM at one time.
   unsigned CAL_ID1;           // Calibration structure
   unsigned CAL_ID2;

   uint32_t SCS_CRC32;           //  CRC-32 of last stored SystemConfig structure
   uint32_t CPS_CRC32[MAX_CPES]; //  CRC-32 of last stored ChargeParms structure
   uint32_t CAL_CRC32;           //  CRC-32 of last stored ADCCal structure

} tEKEY;

#endif //_FLASH_H_

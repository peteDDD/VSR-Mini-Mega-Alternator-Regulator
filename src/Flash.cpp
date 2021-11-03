
//      Flash.cpp
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


#include "Flash.h"

//------------------------------------------------------------------------------------------------------
// CRC-32
//
//      This function will calculate a CRC-32 for the passed array and return it as an unsigned LONG value
//      They are used in the EEPROM reads and writes
//      
// 
//------------------------------------------------------------------------------------------------------

uint32_t crc_update(uint32_t crc, uint8_t data)                               // Helper function, used to read CRC value out of PROGMEN.
{
    static const PROGMEM uint32_t crc_table[16] = {                                     // Table to simplify calculations of CRCs
      0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
      0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
      0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
      0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
      };
 
    uint8_t tbl_idx;
    tbl_idx = crc ^ (data >> (0 * 4));
    crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
    tbl_idx = crc ^ (data >> (1 * 4));
    crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
    return crc;
    }



uint32_t calc_crc (uint8_t*d, int sizeD) {

 uint32_t crc = ~0L;
  int i;

  for (i = 0; i < sizeD; i++)
    crc = crc_update(crc, *d++);
  crc = ~crc;
  return crc;
  }


//------------------------------------------------------------------------------------------------------
// Read SCS EEPROM
//
//      This function will see if there is are valid structures for the  sytemConfig saved EEPROM. 
//      If so, it will copy those from the EEPROM into the passed buffer and return TRUE.
//
//
//
//------------------------------------------------------------------------------------------------------

bool read_SCS_EEPROM(tSCS *scsPtr) {
  
   tSCS buff;
   tEKEY  key;                                                                          // Structure used to see validate the presence of saved data.

   eeprom_read_block((void *)&key, (void *) EKEY_FLASH_LOCATION, sizeof(tEKEY));        // Fetch the EKEY structure from EEPROM to update appropriate portions


   if  ((key.SCS_ID1 == SCS_ID1_K) && (key.SCS_ID2 == SCS_ID2_K)) {                     // We may have a valid systemConfig structure . . 
        eeprom_read_block((void*)&buff, (void *)SCS_FLASH_LOCATION, sizeof(tSCS));     
                                                                                        //  . .  let's fetch it from EEPROM and see if the CRCs check out..
        if (calc_crc ((uint8_t*)&buff, sizeof(tSCS)) == key.SCS_CRC32) {
            *scsPtr = buff;                                                             //  Looks valid, copy the working buffer into RAM
             return(true);
             }
        }

 
    return(false);                                                                      // Did not make it through all the checks...
}


//------------------------------------------------------------------------------------------------------
// Write SCS EEPROM
//
//      If a pointer is passed into this function it will write the current systemConfig and ChargeParms tables into the EEPROM and update the
//      appropriate EEPROM Key variables.   If NULL is passed in for the data pointer, the saved SCS will be invalidated in the EEPROM
//
//
//
//------------------------------------------------------------------------------------------------------

void write_SCS_EEPROM(tSCS *scsPtr) {

   tEKEY  key;                                                                          // Structure used to see validate the presence of saved data.

 
   eeprom_read_block((void *)&key, (void *) EKEY_FLASH_LOCATION  , sizeof(tEKEY));      // Fetch the EKEY structure from EEPROM to update appropriate portions

   
  if (scsPtr != NULL) {
        key.SCS_ID1 = SCS_ID1_K;                                                        // User wants to save the current systemConfig structure to EEPROM
        key.SCS_ID2 = SCS_ID2_K;                                                        // Put in validation tokens
        key.SCS_CRC32 = calc_crc ((uint8_t*)scsPtr, sizeof(tSCS));

        
      /*  #if !defined DEBUG && !defined SIMULATION
           if ((systemConfig.BT_CONFIG_CHANGED == false) ||                             // Wait a minute: Before we do any actual changes. . if the Bluetooth 
               (systemConfig.CONFIG_LOCKOUT   != 0))                                    // has been made a bit more secure, or if we are locked out --> prevent any update.
              return;
              #endif                                                                    // (But do this check only if not in 'testing' mode!
      */

        eeprom_write_block((void*)scsPtr, (void *)SCS_FLASH_LOCATION, sizeof(tSCS));
        }                                                                               // And write out the current structure
        
  else  {
        key.SCS_ID1 = 0;                                                                // User wants to invalidate the EEPROM saved info.  
        key.SCS_ID2 = 0;                                                                // So just zero out the validation tokens
        key.SCS_CRC32 = 0;                                                              // And the CRC-32 to make dbl sure.
        }
        
  eeprom_write_block((void *)&key, (void *)EKEY_FLASH_LOCATION  , sizeof(tEKEY));       // Save back the updated EKEY structure

}


//------------------------------------------------------------------------------------------------------
// Read CPS EEPROM
//
//      This function will see if there is are valid structures saved in EEPROM for the entry 'index' of the chargeParm table.
//      If so, it will copy that from the EEPROM into the passed CPS structure and TRUE is returned.
//
//      index is CPS entry from the EEPROM to be copied into the working CPE structure 
//
//
//------------------------------------------------------------------------------------------------------

bool read_CPS_EEPROM(uint8_t index, tCPS *cpsPtr) {
  
   tCPS   buff;
   tEKEY  key;                                                                          // Structure used to see validate the presence of saved data.


   eeprom_read_block((void *)&key, (const void *)EKEY_FLASH_LOCATION  , sizeof(tEKEY)); // Fetch the EKEY structure from EEPROM

   if  ((key.CPS_ID1[index] == CPS_ID1_K) && (key.CPS_ID2[index] == CPS_ID2_K)) {       // How about the Charge Profile tables? . . 

        eeprom_read_block((void*)&buff, (void *)CPS_FLASH_LOCATION, sizeof(tCPS));
                                                                                        //  . .  let's fetch it from EEPROM and see if the CRCs check out..
        
        if (calc_crc ((uint8_t*)&buff, sizeof(tCPS)) == key.CPS_CRC32[index]) {
                *cpsPtr = buff;                                                         //  Looks valid, copy the working buffer into RAM
                return(true);
                }
        }

   return(false);                                                                       // Did not make it through all the checks...
  
}



//------------------------------------------------------------------------------------------------------
// Write CPS EEPROM
//
//      Will write the passed CPS structure into the saved entry 'N' of the EEPROM.  If NULL is passed in for the data pointer, 
//      Entry 'N' will be invalidated in the EEPROM
//
//
//
//------------------------------------------------------------------------------------------------------
void write_CPS_EEPROM(uint8_t index, tCPS *cpsPtr) {                                    // Save/flush entry 'index'

   tEKEY  key;                                                                          // Structure used to see validate the presence of saved data.

 
   eeprom_read_block((void *)&key, (void *) EKEY_FLASH_LOCATION  , sizeof(tEKEY));      // Fetch the EKEY structure from EEPROM to update appropriate portions

   
  if (cpsPtr != NULL) {
        key.CPS_ID1[index] = CPS_ID1_K;                                                 // User wants to save the current systemConfig structure to EEPROM
        key.CPS_ID2[index] = CPS_ID2_K;                                                 // Put in validation tokens
        key.CPS_CRC32[index] = calc_crc ((uint8_t*)cpsPtr, sizeof(tCPS));

       /*
        #if !defined DEBUG && !defined SIMULATION
           if ((systemConfig.BT_CONFIG_CHANGED == false) ||                             // Wait a minute: Before we do any actual changes. . if the Bluetooth 
               (systemConfig.CONFIG_LOCKOUT   != 0))                                    // has not been made a bit more secure, or if we are locked out, prevent any update.
              return;
              #endif                                                                    // (But do this check only if not in 'testing' mode!
       */

        eeprom_write_block((void*)cpsPtr, (void *)CPS_FLASH_LOCATION, sizeof(tCPS));
        }                                                                               // And write out the indexed CPE entry
        
  else  {
        key.CPS_ID1[index]   = 0;                                                       // User wants to invalidate the EEPROM saved info.  
        key.CPS_ID2[index]   = 0;                                                       // So just zero out the validation tokens
        key.CPS_CRC32[index] = 0;                                                       // And the CRC-32 to make dbl sure.
        }

     eeprom_write_block((void *)&key, (void *) EKEY_FLASH_LOCATION  , sizeof(tEKEY));   // Save back the updated EKEY structure

}


//------------------------------------------------------------------------------------------------------
// Transfer Default CPS from FLASH
//
//      This function copy the default entry from the PROGMEM CPS structure in FLASH to the passed buffer.
//
//      index is CPS entry from the PROGMEM to be copied into the passed CPE structure 
//
//
//------------------------------------------------------------------------------------------------------
void transfer_default_CPS(uint8_t index, tCPS  *cpsPtr) {


   uint8_t*wp = (uint8_t*) cpsPtr;
   uint8_t*ep = (uint8_t*) &defaultCPS[index];                                               //----- Copy the appropriate default Charge Parameters entry from FLASH into the working structure   

   for (unsigned int u=0; u < sizeof(tCPS); u++)                                 
           *wp++ = (uint8_t) pgm_read_byte_near(ep++);

}


//------------------------------------------------------------------------------------------------------
// Read CAL EEPROM
//
//      This function will see if there is are valid structures for the  sytemConfig saved EEPROM. 
//      If so, it will copy those from the EEPROM into the passed buffer and return TRUE.
//
//
//
//------------------------------------------------------------------------------------------------------
bool read_CAL_EEPROM(tCAL *calPtr) {
  
   tCAL buff;
   tEKEY  key;                                                                          // Structure used to see validate the presence of saved data.


   eeprom_read_block((void *)&key, (void *) EKEY_FLASH_LOCATION, sizeof(tEKEY));        // Fetch the EKEY structure from EEPROM to update appropriate portions

  
   if  ((key.CAL_ID1 == CAL_ID1_K) && (key.CAL_ID2 == CAL_ID2_K)) {                     // We may have a valid systemConfig structure . . 

        eeprom_read_block((void*)&buff, (void *)CAL_FLASH_LOCATION, sizeof(tCAL));     
                                                                                        //  . .  lets fetch it from EEPROM and see if the CRCs check out..
        if (calc_crc ((uint8_t*)&buff, sizeof(tCAL)) == key.CAL_CRC32) {
            *calPtr = buff;                                                             //  Looks valid, copy the working buffer into RAM
             return(true);
             }
        }

    return(false);                                                                      // Did not make it through all the checks...
}


//------------------------------------------------------------------------------------------------------
// Write CAL EEPROM
//
//      If a pointer is passed into this function it will write the current ADCCal and ChargeParms tables into the EEPROM and update the
//      appropriate EEPROM Key variables.   If NULL is passed in for the data pointer, the saved CAL structure will be invalidated in the EEPROM
//
//
//
//------------------------------------------------------------------------------------------------------
void write_CAL_EEPROM(tCAL *calPtr) {

   tEKEY  key;                                                                          // Structure used to see validate the presence of saved data.

 
   eeprom_read_block((void *)&key, (void *) EKEY_FLASH_LOCATION  , sizeof(tEKEY));      // Fetch the EKEY structure from EEPROM to update appropriate portions

   
  if (calPtr != NULL) {
        key.CAL_ID1 = CAL_ID1_K;                                                        // User wants to save the current systemConfig structure to EEPROM
        key.CAL_ID2 = CAL_ID2_K;                                                        // Put in validation tokens
        key.CAL_CRC32 = calc_crc ((uint8_t*)calPtr, sizeof(tCAL));

        
          eeprom_write_block((void*)calPtr, (void *)CAL_FLASH_LOCATION, sizeof(tCAL));
        }                                                                               // And write out the current structure
        
  else  {
        key.CAL_ID1 = 0;                                                                // User wants to invalidate the EEPROM saved info.  
        key.CAL_ID2 = 0;                                                                // So just zero out the validation tokens
        key.CAL_CRC32 = 0;                                                              // And the CRC-32 to make dbl sure.
        }
        
  eeprom_write_block((void *)&key, (void *)EKEY_FLASH_LOCATION  , sizeof(tEKEY));       // Save back the updated EKEY structure

}


//------------------------------------------------------------------------------------------------------
// Restore All
//
//      This function will restore all EEPROM based configuration values (system and all the Charge profile tables
//      to their default (as compiled) values.  It does this by erasing clearing out each table entry.
//      Note that this function then will reboot the machine, so it will not return...
// 
//
//------------------------------------------------------------------------------------------------------
void    restore_all(){

     uint8_t b;

     write_SCS_EEPROM(NULL);                              // Erase any saved systemConfig structure in the EEPROM
     
     if (ADCCal.LOCKED != true)   write_CAL_EEPROM(NULL); // If not factory locked, reset the CAL structure.

     for (b=0; b<MAX_CPES; b++) 
        write_CPS_EEPROM(b, NULL);                        // Erase any saved charge profiles structures in the EEPROM
    
     reboot();                                            // And FORCE the system to reset - we will never come back from here!
   }

//
//      Portability.h
//
//      Copyright (c) 2018 by William A. Thomason.      http://arduinoalternatorregulator.blogspot.com/
//
//
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

#ifndef _PORTABILITY_H_
#define _PORTABILITY_H_
                                                       
/****************************************************************************************
 ****************************************************************************************
 *                                                                                      *
 *                              PORTABILITY                                             *
 *                                                                                      *
 *                                                                                      *
 *      This include file contains all the 'ugly' definitions and conversions           *
 *      used to allow much of the VSR projects source to be used with the Arduino       * 
 *      IDE for use wiht AVR CPUs, or the STm32 cubeMX / Kiel uVsion environment        *
 *      when targeting the STM32F07 CPU.                                                *
 *                                                                                      *
 *       Note that projecct/board specifics (e.g. port assigments) are NOT contained    *
 *       in this file.  Only compiler / linker stuff                                    *
 *                                                                                      *
 *                                                                                      *
 *                                                                                      *
 ****************************************************************************************
 ****************************************************************************************/
                                  
    // Arduino IDE target
    
    #include <Arduino.h>     // Pick up Arduino specifics, ala PROGMEM
    #include <avr/wdt.h>     
    #include <MemoryFree.h>
    #include "System.h"
                                                        
    //-- Some #defines to allow for more common code across different CPUs, w/o a TON of #ifdefs
    #define sample_feature_IN_port1()   digitalRead(FEATURE_IN_PORT1)    
    #define sample_feature_IN_port2()   digitalRead(FEATURE_IN_PORT2) 
    #define sample_feature_IN_port3()   digitalRead(FEATURE_IN_PORT3)                                                   

    #define ASCII_read(v)          Serial.read()
    #define ASCII_RxAvailable(v)  (Serial.available() > 0)
    #define ASCII_write(v)         Serial.write(v)
    #define Serial_flush();        Serial.flush();
    
 #endif  // _PORTABILITY_H_

//      LED.h
//
//      Copyright (c) 2018 by William A. Thomason.      http://arduinoalternatorregulator.blogspot.com/
//                                                      http://smartdcgenerator.blogspot.com/
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
#ifndef _LED_H_
#define _LED_H_

                //----- LED blinking controls and conversion tables
                //

#define LED_RATE_NORMAL         300                             // Normally flash the LED patterns changing the LED every 300mS
#define LED_RATE_SLOW           600                             // And slow down to once per second for dramatic effect
#define LED_RATE_FAST           100                             //  Even more drama!

#define LED_FAULTED             0xAA04                          // FAULTED LED pattern  - Use FAST and repeat 2x times.
#define LED_BULK                0xAAAA                          // Blink out Normal while in Ramp, or Bulk mode
#define LED_ACCEPT              0xA0A0                          // Blink out Normal while in Acceptance mode
#define LED_OC                  0xAAAA                          // Blink out Slow while in Overcharge
#define LED_FLOAT               0xFF00                          // Blink out Normal while in Float / post-float mode
#define LED_EQUALIZE            0xA000                          // Blink out Fast while in Equalize mode
#define LED_IDLE                0x0100                          // Blink out Normal while idle.
#define LED_RESETTING           0xAAAA                          // Show that we are about to reset, blink out FAST

void update_LED(void);
void blink_LED (unsigned pattern, unsigned led_time, int8_t led_repeat,  bool mirror);
bool refresh_LED(void);
void outputGreenLED(int LED_state); 
void outputRedLED(int LED_state); 

extern int8_t   LEDRepeat;

#endif /** _LED_H_ **/

//      Alternator.h
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

#ifndef _ALTERNATOR_H_
#define _ALTERNATOR_H_

#include "Config.h"
#include "System.h"
#include "CPE.h"

extern int inChargingStateCount; // seconds left in warmup
extern uint32_t inChargingStateTime;  // count up milliseconds in current state

extern volatile bool statorIRQflag;
extern uint32_t lastPWMChanged;
extern uint32_t altModeChanged;

extern int altCapAmps;
extern int altCapRPMs;

extern int targetAltWatts;
extern float targetBatVolts;
extern float targetBatAmps;
extern float targetAltAmps;
extern int measuredRPMs;
extern bool smallAltMode;
extern bool tachMode;
#ifdef ENABLE_FEATURE_IN_SCUBA
  extern bool scubaMode;
  extern const char *scubaModeString;
#endif
extern int fieldPWMvalue;
extern int thresholdPWMvalue;
extern bool usingEXTAmps;

extern tCPS chargingParms;

extern float systemVoltMult;
extern float systemAmpMult;
extern uint8_t cpIndex;

extern int8_t SDMCounter;
extern bool sendDebugString;

void stator_IRQ(void);
void calculate_RPMs(void);
void calculate_ALT_targets(void);
void set_charging_mode(tModes settingMode);
void set_ALT_PWM(int PWM);
void manage_ALT(void);
bool initialize_alternator(void);

#endif // _ALTERNATOR_H_

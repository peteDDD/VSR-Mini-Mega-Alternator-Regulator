//      Sensors.h
//
//      Copyright (c) 2016, 2017 by William A. Thomason.      http://arduinoalternatorregulator.blogspot.com/
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


#ifndef _SENSORS_H_
#define _SENSORS_H_

typedef struct { // Calibration Structure - holds board specific values, set at time of manufacturing.
      bool      LOCKED;            // Calibration already done at Factory, and hence should NEVER be overridden (even master reset)
      float     VBAT_GAIN_ERROR;   // Error of VBat ADC + resister dividers 
      float     AMP_GAIN_ERROR;
      int16_t   VBAT_OFFSET;
      int16_t   AMP_OFFSET;        // Offset error of shunt circuit measured @ 0A 
      uint8_t   CALPLACEHOLDER[16];  // Room for future expansion 
      } tCAL;

void WriteOLEDTitlePage(void);
void WriteOLEDBatteryType(void);
void WriteOLEDDIPSettings(void);
void WriteOLEDFactoryReset(void);
void WriteOLEDResetting(void);
void WriteOLEDNonResetFault(void);
void WriteOLEDFault(void);
void WriteOLEDFaultString(void);
void WriteOLEDDataScreenStaticData(void);
void WriteOLEDDynamicData(void);


//extern int inChargingStateCount; // seconds left in warmup
//extern unsigned long inChargingStateTime;  // count up milliseconds in current state

extern bool    updatingBatVAs;
extern bool    updatingAltVAs;
extern bool    shuntAltAmpsMeasured; 

extern float   measuredAltVolts;
extern float   measuredAltAmps;
extern int     measuredAltWatts;
extern float   measuredBatVolts;
extern float   measuredBatAmps;
extern float   simCAN_Ext_amps;

extern int     measuredFETTemp;
extern int     measuredFieldAmps;
extern int     measuredAltTemp;
extern int     measuredBatTemp;   

extern int32_t   accumulatedASecs;
extern int32_t   accumulatedWSecs;
extern uint32_t  generatorLrRunTime;

extern tCAL  ADCCal;

// other variables
bool initialize_sensors(void);
bool read_sensors(void);
bool sample_ALT_and_BAT_VoltAmps(void);
bool read_ALT_and_BAT_VoltAmps(void);
void update_run_summary(void);
void reset_run_summary(void);

#endif  /*  _SENSORS_H_ */

/*  TidelibElkhornSlough.h 
  A library for calculating the current tide height at 
   Elkhorn Slough railroad bridge, Monterey Bay, California 
  Luke Miller, June 2012
  https://github.com/millerlp/Tide_calculator
*/ 
 
#ifndef  TidelibElkhornSlough_h 
#define  TidelibElkhornSlough_h 
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include "RTClib.h"

class TideCalc {
 public:
	 TideCalc();
    float currentTide(DateTime now);
    char* returnStationID(void);
};
#endif
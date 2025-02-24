/*  TidelibLegeCapFerret.h 
  A library for calculating the current tide height at 
   Lege cap Ferret, Gironde France, ID number  
  Valentin Spanu, 2017-05-22
  Compiled under Arduino 1.6.4
  D'apres le site de Luke Miller
  https://github.com/millerlp/Tide_calculator
 Released under the GPL version 3 license.
 The harmonic constituents used here were originally derived from 
 the Center for Operational Oceanic Products and Services (CO-OPS),
 National Ocean Service (NOS), National Oceanic and Atmospheric 
 Administration, U.S.A.
 The data were originally processed by David Flater for use with XTide,
 available at http://www.flaterco.com/xtide/files.html
 The predictions from this program should not be used for navigation
 and no accuracy or warranty is given or implied for these tide predictions.
 It is highly recommended that you verify the output of these predictions
 against the relevant NOAA tide predictions online.
*/ 
 
#ifndef  TidelibLegeCapFerret_h 
#define  TidelibLegeCapFerret_h 
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include "RTClib.h" // https://github.com/millerlp/RTClib

class TideCalc {
 public:
	 TideCalc();
    float currentTide(DateTime now); // returns predicted tide for
    // the supplied date and time. The time should always be given in
    // the local standard time for the site, not daylight savings time
    // output units = feet
    char* returnStationID(void); // NOAA station name
    long returnStationIDnumber(void); // NOAA station ID number
};
#endif
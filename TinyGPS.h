/*
TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
Based on work by and "distance_to" and "course_to" courtesy of Maarten Lamers.
Suggestion to add satellites(), course_to(), and cardinal(), by Matt Monson.
Precision improvements suggested by Wayne Holder.
Copyright (C) 2008-2013 Mikal Hart
All rights reserved.

  Satellite Count Mod - by Brett Hagman
  http://www.roguerobotics.com/

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef TinyGPS_h
#define TinyGPS_h

#ifdef ARDUINO
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#else /* ARDUINO */
#include <inttypes.h>
typedef unsigned char byte;
#endif /* ARDUINO */


#include <stdlib.h>

#define _GPS_VERSION 13 // software version of this library
#define _GPS_MPH_PER_KNOT 1.15077945
#define _GPS_MPS_PER_KNOT 0.51444444
#define _GPS_KMPH_PER_KNOT 1.852
#define _GPS_MILES_PER_METER 0.00062137112
#define _GPS_KM_PER_METER 0.001
#define _GPS_NO_STATS

class TinyGPS
{
public:
  enum {
    GPS_INVALID_AGE        = 0xFFFFFFFF,
    GPS_INVALID_ANGLE      = 999999999, 
    GPS_INVALID_ALTITUDE   = 999999999,
    GPS_INVALID_DATE       = 0,
    GPS_INVALID_TIME       = 0xFFFFFFFF,
    GPS_INVALID_SPEED      = 999999999, 
    GPS_INVALID_FIX_TIME   = 0xFFFFFFFF,
    GPS_INVALID_SATELLITES = 0xFF,
    GPS_INVALID_HDOP       = 0xFFFFFFFF,
    GPS_INVALID_PDOP       = 0xFFFF,
    GPS_INVALID_FIXTYPE    = 0xFF
  };
    
  enum {
      GPS_FIX_NO_FIX = 1,
      GPS_FIX_2D     = 2,
      GPS_FIX_3D     = 3
  };

  static const float GPS_INVALID_F_ANGLE, GPS_INVALID_F_ALTITUDE, GPS_INVALID_F_SPEED;

  TinyGPS();
  bool encode(char c); // process one character received from GPS
  TinyGPS &operator << (char c) {encode(c); return *this;}

  // lat/long in MILLIONTHs of a degree and age of fix in milliseconds
  // (note: versions 12 and earlier gave lat/long in 100,000ths of a degree.
  void get_position(long *latitude, long *longitude, unsigned long *fix_age = 0);

  // date as ddmmyy, time as hhmmsscc, and age in milliseconds
  void get_datetime(unsigned long *date, unsigned long *time, unsigned long *age = 0);

  // signed altitude in centimeters (from GPGGA sentence)
  inline long altitude() {
      long a = _altitude;
      _altitude = 0;            // reset value after single use
      return a;
  }

#ifndef REDUCE_RAM_USE
  // course in last full GPRMC sentence in 100th of a degree
  inline unsigned long course() { return _course; }

  // speed in last full GPRMC sentence in 100ths of a knot
  inline unsigned long speed() { return _speed; }

  // horizontal dilution of precision in 100ths
  inline unsigned long hdop() {
      unsigned long a = _hdop;
      _hdop = 0;            // reset value after single use
      return a;
  }
#endif /* REDUCE_RAM_USE */

  // position dilution of precision in 100ths
  inline unsigned short pdop() {
      return _pdop;
  }

  // number of satellites in view (GPGSV sentence)
  inline unsigned char satsinview() {
      unsigned char a = _satsinview;
      _satsinview = 0;            // reset value after single use
      return a;
  }

  // number of satellites used for fix (GPGSA sentence)
  inline unsigned char satsused() {
      unsigned char a = _satsused;
      _satsused = 0;            // reset value after single use
      return a;
  }
  
  // get the fix type
  inline unsigned char fixtype() {
      unsigned char a = _fixtype;
      _fixtype = 0;            // reset value after single use
      return a;
  }

  void f_get_position(float *latitude, float *longitude, unsigned long *fix_age = 0);
  void crack_datetime(int *year, byte *month, byte *day, 
  byte *hour, byte *minute, byte *second, byte *hundredths = 0, unsigned long *fix_age = 0);
  inline void  resetGPSstatusVars(void) {
    _satsused   = GPS_INVALID_SATELLITES;
    _satsinview = GPS_INVALID_SATELLITES;
    _fixtype    = GPS_INVALID_FIXTYPE;
#ifndef REDUCE_RAM_USE
    _hdop       = GPS_INVALID_HDOP;
#endif /* REDUCE_RAM_USE */
    _pdop       = GPS_INVALID_PDOP;
    _altitude   = GPS_INVALID_ALTITUDE;
  }
  float f_altitude();
  float f_course();
  float f_speed_knots();
  float f_speed_mph();
  float f_speed_mps();
  float f_speed_kmph();

  static int library_version() { return _GPS_VERSION; }

  static float distance_between (float lat1, float long1, float lat2, float long2);
  static float course_to (float lat1, float long1, float lat2, float long2);
  static const char *cardinal(float course);

#ifndef _GPS_NO_STATS
  void stats(unsigned long *chars, unsigned short *good_sentences, unsigned short *failed_cs);
#endif

private:
  enum {
      _GPS_SENTENCE_GPGGA,
      _GPS_SENTENCE_GPRMC,
      _GPS_SENTENCE_GPGSV,
      _GPS_SENTENCE_GPGSA,
      _GPS_SENTENCE_OTHER
  };

  // properties
  unsigned long _time, _new_time;
  unsigned long _date, _new_date;
#ifndef REDUCE_RAM_USE
  long _latitude, _new_latitude;
  long _longitude, _new_longitude;
  unsigned long  _speed, _new_speed;
  unsigned long  _course, _new_course;
  unsigned long  _hdop, _new_hdop;
#endif /* REDUCE_RAM_USE */
  long _altitude, _new_altitude;
  unsigned short _pdop, _new_pdop;  // 100 * PDOP (position dilution of precision)
  unsigned char  _satsinview, _new_satsinview;
  unsigned char  _satsused, _new_satsused;
  unsigned char  _fixtype, _new_fixtype;

  unsigned long _last_time_fix, _new_time_fix;
  unsigned long _last_position_fix, _new_position_fix;
  unsigned long _last_character_received_time;

  // parsing state variables
  byte _parity;
  bool _is_checksum_term;
  char _term[15];
  byte _sentence_type;
  byte _term_number;
  byte _term_offset;
  bool _gps_data_good;

#ifndef _GPS_NO_STATS
  // statistics
  unsigned long _encoded_characters;
  unsigned short _good_sentences;
  unsigned short _failed_checksum;
  unsigned short _passed_checksum;
#endif

  // internal utilities
  int from_hex(char a);
  unsigned long parse_decimal();
  unsigned long parse_degrees();
  bool term_complete();
  bool gpsisdigit(char c) { return c >= '0' && c <= '9'; }
  long gpsatol(const char *str);
  int gpsstrcmp(const char *str1, const char *str2);
};

#if !defined(ARDUINO) 
// Arduino 0012 workaround
#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round 
#endif

#endif

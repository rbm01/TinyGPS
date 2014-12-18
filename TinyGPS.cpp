/*
TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
Based on work by and "distance_to" and "course_to" courtesy of Maarten Lamers.
Suggestion to add satellites(), course_to(), and cardinal(), by Matt Monson.
Precision improvements suggested by Wayne Holder.
Copyright (C) 2008-2013 Mikal Hart
All rights reserved.

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

/*
 * Additional notes by RBM 2014-03-12
 * **********************************
 *
 * Time accuracy depends on the offset between the UTC edge (usually the PPS)
 * and the transmission of the NMEA messages. If the PPS is not available,
 * which is often the case, then this offset value is unknown, but can
 * typically be a couple of hundred milliseconds, which is further
 * exacerbated by a significant jitter.
 *
 * The first NMEA sentence after the UTC edge is typically $GPRMC, which
 * contains both date and time, so I have modified the code in TinyGPS.cpp
 * to record the millis() when a '$' character is received as the GPS time
 * fix reference, rather than when the time is decoded.
 *
 * In addition, I have modified the code in TinyGPS.cpp to accept time
 * from the module even when the $GPRMC status flag is not 'A' (valid),
 * provided the object is instantiated with the new 'allowRTCtime' flag.
 * This means the module will return time from the GPS module on-board
 * RTC (real time clock) even when it does not have a GPS lock.
 */

#ifndef ARDUINO
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include "arduino_util.h"
#endif /* ARDUINO */

#include "TinyGPS.h"

#define _GPRMC_TERM   "GPRMC"
#define _GPGGA_TERM   "GPGGA"
#define _GPGSV_TERM   "GPGSV"
#define _GPGSA_TERM   "GPGSA"

TinyGPS::TinyGPS()
  :  _time(GPS_INVALID_TIME)
  ,  _date(GPS_INVALID_DATE)
  ,  _latitude(GPS_INVALID_ANGLE)
  ,  _longitude(GPS_INVALID_ANGLE)
  ,  _altitude(GPS_INVALID_ALTITUDE)
  ,  _speed(GPS_INVALID_SPEED)
  ,  _course(GPS_INVALID_ANGLE)
  ,  _hdop(GPS_INVALID_HDOP)
  ,  _satsinview(GPS_INVALID_SATELLITES)
  ,  _satsused(GPS_INVALID_SATELLITES)
  ,  _fixtype(GPS_INVALID_FIXTYPE)
  ,  _last_time_fix(GPS_INVALID_FIX_TIME)
  ,  _last_position_fix(GPS_INVALID_FIX_TIME)
  ,  _last_character_received_time(0)
  ,  _parity(0)
  ,  _is_checksum_term(false)
  ,  _sentence_type(_GPS_SENTENCE_OTHER)
  ,  _term_number(0)
  ,  _term_offset(0)
#ifndef _GPS_NO_STATS
  ,  _encoded_characters(0)
  ,  _good_sentences(0)
  ,  _failed_checksum(0)
#endif
{
  _term[0] = '\0';
}

#ifndef ARDUINO
/*
 * Resolve some undefined functions and #defines when this code is being
 * used in a non-Arduino environment.
 */

#define TWO_PI M_PI * 2

inline float radians(float degrees) { return M_PI * (degrees / 180); }
inline float degrees(float radians) { return (radians * 180) / M_PI; }
inline float sq(float f)            { return f * f; }
#endif /* ARDUINO */

//
// public methods
//

bool TinyGPS::encode(char c)
{
  bool valid_sentence = false;
  unsigned long charReadTime = millis();

#ifndef _GPS_NO_STATS
  ++_encoded_characters;
#endif
  switch(c)
  {
  case ',': // term terminators
    _parity ^= c;
  case '\r':
  case '\n':
  case '*':
    if (_term_offset < sizeof(_term))
    {
      _term[_term_offset] = 0;
      valid_sentence = term_complete();
    }
    ++_term_number;
    _term_offset = 0;
    _is_checksum_term = c == '*';
    //printf("TinyGPS::encode(): About to return %s\n", valid_sentence ? "true" : "false");
    break;

  case '$': // sentence begin
    _term_number = _term_offset = 0;
    _parity = 0;
    _sentence_type = _GPS_SENTENCE_OTHER;
    _is_checksum_term = false;
    //_new_time_fix = millis();   // synch time fix age at start of sentence

    // Detect the start of a new "paragraph" of NMEA sentences by looking
    // for a long gap between characters.
    //
    // A continuous sequence of characters at 4800 baud will be spaced approx
    // 2mS apart (10/4800 S). At 2400 baud this becomes 4mS, and at 9600
    // baud, 1mS. Here we use an arbitary value of 100mS to indicate
    // the minimum time between one reporting cycle and the next.
    // Because this character is a '$' we know it's the start of a sentence.
    if ((int32_t)(charReadTime - _last_character_received_time) > 100)
    {
      _new_time_fix = charReadTime; // synch time fix age at start of sentence
    }

    break;

  default:
    // ordinary characters
    if (_term_offset < sizeof(_term) - 1)
      _term[_term_offset++] = c;
    if (!_is_checksum_term)
      _parity ^= c;
    break;
  }

  _last_character_received_time = charReadTime;

  return valid_sentence;
}

#ifndef _GPS_NO_STATS
void TinyGPS::stats(unsigned long *chars, unsigned short *sentences, unsigned short *failed_cs)
{
  if (chars) *chars = _encoded_characters;
  if (sentences) *sentences = _good_sentences;
  if (failed_cs) *failed_cs = _failed_checksum;
}
#endif

//
// internal utilities
//
int TinyGPS::from_hex(char a)
{
  if (a >= 'A' && a <= 'F')
    return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f')
    return a - 'a' + 10;
  else
    return a - '0';
}

unsigned long TinyGPS::parse_decimal()
{
  char *p = _term;
  bool isneg = *p == '-';
  if (isneg) ++p;
  unsigned long ret = 100UL * gpsatol(p);
  while (gpsisdigit(*p)) ++p;
  if (*p == '.')
  {
    if (gpsisdigit(p[1]))
    {
      ret += 10 * (p[1] - '0');
      if (gpsisdigit(p[2]))
        ret += p[2] - '0';
    }
  }
  return isneg ? -ret : ret;
}

// Parse a string in the form ddmm.mmmmmmm...
unsigned long TinyGPS::parse_degrees()
{
  char *p;
  unsigned long left_of_decimal = gpsatol(_term);
  unsigned long hundred1000ths_of_minute = (left_of_decimal % 100UL) * 100000UL;
  for (p=_term; gpsisdigit(*p); ++p);
  if (*p == '.')
  {
    unsigned long mult = 10000;
    while (gpsisdigit(*++p))
    {
      hundred1000ths_of_minute += mult * (*p - '0');
      mult /= 10;
    }
  }
  return (left_of_decimal / 100) * 1000000 + (hundred1000ths_of_minute + 3) / 6;
}

#define COMBINE(sentence_type, term_number) (((unsigned)(sentence_type) << 5) | term_number)

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool TinyGPS::term_complete()
{
  if (_is_checksum_term)
  {
    byte checksum = 16 * from_hex(_term[0]) + from_hex(_term[1]);
    //printf("TinyGPS::term_complete(): inside \"if (_is_checksum_term).  checksum=0x%02x  _parity=0x%02x\"\n", checksum, _parity);
    if (checksum == _parity)
    {
#ifndef _GPS_NO_STATS
      ++_good_sentences;
#endif

      switch(_sentence_type)
      {
      case _GPS_SENTENCE_GPRMC:
        _time      = _new_time;
        _date      = _new_date;
        _last_time_fix = _new_time_fix;
        if (_gps_data_good)
        {
            _last_position_fix = _new_position_fix;
        }
        _latitude  = _new_latitude;
        _longitude = _new_longitude;
        _speed     = _new_speed;
        _course    = _new_course;
        // printf("TinyGPS::term_complete(): inside \"case _GPS_SENTENCE_GPRMC\".\n");
        break;
      case _GPS_SENTENCE_GPGGA:
        if (_gps_data_good)
        {
            _last_position_fix = _new_position_fix;
        }
        _altitude  = _new_altitude;
        _time      = _new_time;
        _latitude  = _new_latitude;
        _longitude = _new_longitude;
        _hdop      = _new_hdop;
        _satsused  = _new_satsused;
        break;
      case _GPS_SENTENCE_GPGSV:
        _satsinview = _new_satsinview;
        break;
      case _GPS_SENTENCE_GPGSA:
        _fixtype = _new_fixtype;
        break;
      }

      return true;
    }

#ifndef _GPS_NO_STATS
    else
      ++_failed_checksum;
#endif
    return false;
  }

  // the first term determines the sentence type
  if (_term_number == 0)
  {
    if (!gpsstrcmp(_term, _GPRMC_TERM))
      _sentence_type = _GPS_SENTENCE_GPRMC;
    else if (!gpsstrcmp(_term, _GPGGA_TERM))
      _sentence_type = _GPS_SENTENCE_GPGGA;
    else if (!gpsstrcmp(_term, _GPGSV_TERM))
      _sentence_type = _GPS_SENTENCE_GPGSV;
    else if (!gpsstrcmp(_term, _GPGSA_TERM))
      _sentence_type = _GPS_SENTENCE_GPGSA;
    else
      _sentence_type = _GPS_SENTENCE_OTHER;
    return false;
  }

  // Finish here if the term is empty or it's a sentence type that we ignore
  if (_term[0] == '\0' || _sentence_type == _GPS_SENTENCE_OTHER)
    return false;

  switch(COMBINE(_sentence_type, _term_number))
  {
  case COMBINE(_GPS_SENTENCE_GPRMC, 1): // Time
    _new_time = parse_decimal();
    // _new_time_fix = millis();        // _new_time_fix set when '$' received
    break;
  case COMBINE(_GPS_SENTENCE_GPGGA, 1): // Time
    // Ignore this time because it's already skewed by > 100mS
    break;
  case COMBINE(_GPS_SENTENCE_GPRMC, 2): // GPRMC validity
    _gps_data_good = _term[0] == 'A';
    break;
  case COMBINE(_GPS_SENTENCE_GPGSA, 2): // Fix type
    _new_fixtype = (unsigned char) gpsatol(_term);
    break;
  case COMBINE(_GPS_SENTENCE_GPRMC, 3): // Latitude
  case COMBINE(_GPS_SENTENCE_GPGGA, 2):
    _new_latitude = parse_degrees();
    _new_position_fix = millis();
    break;
  case COMBINE(_GPS_SENTENCE_GPGSV, 3): // Satellites in view
    // we've got our number of sats
    // NOTE: we will more than likely hit this a few times in a row, because
    // there are usually multiple GPGSV sentences to describe all of the
    // satelites, but that's OK because the each contain the total number
    // of satellites in view.
    _new_satsinview = (unsigned char) gpsatol(_term);
    break;
  case COMBINE(_GPS_SENTENCE_GPRMC, 4): // N/S
  case COMBINE(_GPS_SENTENCE_GPGGA, 3):
    if (_term[0] == 'S')
      _new_latitude = -_new_latitude;
    break;
  case COMBINE(_GPS_SENTENCE_GPRMC, 5): // Longitude
  case COMBINE(_GPS_SENTENCE_GPGGA, 4):
    _new_longitude = parse_degrees();
    break;
  case COMBINE(_GPS_SENTENCE_GPRMC, 6): // E/W
  case COMBINE(_GPS_SENTENCE_GPGGA, 5):
    if (_term[0] == 'W')
      _new_longitude = -_new_longitude;
    break;
  case COMBINE(_GPS_SENTENCE_GPRMC, 7): // Speed (GPRMC)
    _new_speed = parse_decimal();
    break;
  case COMBINE(_GPS_SENTENCE_GPRMC, 8): // Course (GPRMC)
    _new_course = parse_decimal();
    break;
  case COMBINE(_GPS_SENTENCE_GPRMC, 9): // Date (GPRMC)
    _new_date = gpsatol(_term);
    break;
  case COMBINE(_GPS_SENTENCE_GPGGA, 6): // Fix data (GPGGA)
    _gps_data_good = _term[0] > '0';
    break;
  case COMBINE(_GPS_SENTENCE_GPGGA, 7): // Satellites used
    _new_satsused = (unsigned char) gpsatol(_term);
    break;
  case COMBINE(_GPS_SENTENCE_GPGGA, 8): // HDOP
    _new_hdop = parse_decimal();
    break;
  case COMBINE(_GPS_SENTENCE_GPGGA, 9): // Altitude (GPGGA)
    _new_altitude = parse_decimal();
    break;
  }
  return false;
}

long TinyGPS::gpsatol(const char *str)
{
  long ret = 0;
  while (gpsisdigit(*str))
    ret = 10 * ret + *str++ - '0';
  return ret;
}

int TinyGPS::gpsstrcmp(const char *str1, const char *str2)
{
  while (*str1 && *str1 == *str2)
    ++str1, ++str2;
  return *str1;
}

/* static */
float TinyGPS::distance_between (float lat1, float long1, float lat2, float long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  float delta = radians(long1-long2);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

float TinyGPS::course_to (float lat1, float long1, float lat2, float long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  float dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float a1 = sin(dlon) * cos(lat2);
  float a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}

const char *TinyGPS::cardinal (float course)
{
  static const char* directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};

  int direction = (int)((course + 11.25f) / 22.5f);
  return directions[direction % 16];
}

// lat/long in MILLIONTHs of a degree and age of fix in milliseconds
// (note: versions 12 and earlier gave this value in 100,000ths of a degree.
void TinyGPS::get_position(long *latitude, long *longitude, unsigned long *fix_age)
{
  if (latitude) *latitude = _latitude;
  if (longitude) *longitude = _longitude;

  if (fix_age)
  {
#ifdef ARDUINO
    *fix_age = _last_position_fix == GPS_INVALID_FIX_TIME
                 ? GPS_INVALID_AGE : (millis() - _last_position_fix);
#else /* ARDUINO */
    *fix_age = 0;
#endif /* ARDUINO */
  }
}

// date as ddmmyy, time as hhmmsscc, and age in milliseconds
#ifdef ARDUINO
void TinyGPS::get_datetime(unsigned long *date, unsigned long *time, unsigned long *age)
{
  if (date) *date = _date;
  if (time) *time = _time;
  if (age) *age = _last_time_fix == GPS_INVALID_FIX_TIME ?
   GPS_INVALID_AGE : millis() - _last_time_fix;
}
#else /* ARDUINO */
void TinyGPS::get_datetime(unsigned long *date, unsigned long *timeval, unsigned long *age)
{
    struct timespec ts;
    struct tm tt;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    ts.tv_sec += monotonic_clkGpsOffset;

    gmtime_r(&ts.tv_sec, &tt);      // thread safe version of localtime()

    if (date)
    {
        if (tt.tm_year >= 100)
            tt.tm_year -= 100;
        *date = tt.tm_year + (tt.tm_mon + 1) * 100 + tt.tm_mday * 10000;
    }
    if (timeval)
        *timeval = tt.tm_hour * 1000000 + tt.tm_min * 10000 + tt.tm_sec * 100 + ts.tv_nsec / 10000000;
    if (age)
    {
        *age = 130;     // just an arbitary value
        //*age = 0;     // just an arbitary value
    }

#ifdef DEBUG_GPSTIME
    char buf[100];
    snprintf_P(buf, sizeof(buf),
               PSTR("TinyGPS::get_datetime(): date=%06lu, timeval=%08lu, age=%lu"),
               date ? *date : 0, timeval ? *timeval : 0, age ? *age : 0);
    Serial.println(buf);
#endif /* DEBUG_GPSTIME */
}
#endif /* ARDUINO */

void TinyGPS::f_get_position(float *latitude, float *longitude, unsigned long *fix_age)
{
  long lat, lon;
  get_position(&lat, &lon, fix_age);
  *latitude = lat == GPS_INVALID_ANGLE ? GPS_INVALID_F_ANGLE : (lat / 1000000.0);
  *longitude = lat == GPS_INVALID_ANGLE ? GPS_INVALID_F_ANGLE : (lon / 1000000.0);
}

void TinyGPS::crack_datetime(int *year, byte *month, byte *day,
  byte *hour, byte *minute, byte *second, byte *hundredths, unsigned long *age)
{
  unsigned long date, time;
  uint16_t t1, t2;              // Used to hold temporary values during calculation
  get_datetime(&date, &time, age);

  // decompose date "ddmmyy"
  t1 = date / 100;            // t1 contains ddmm
  if (year)
  {
      t2 = date - (t1 * 100); // t2 contains yy
      *year = t2;
      *year += *year > 80 ? 1900 : 2000;
  }

  t2 = t1 / 100;              // t2 contains dd
  if (month) *month = t1 - (t2 * 100);
  if (day) *day = t2;

  // Decompose time "hhmmsscc"
  // Reuse the "date variable because we need a long here
  date = time / 100;          // date contains hhmmss
  if (hundredths) *hundredths = time - (date * 100);

  t1 = date  / 100;           // t1 contains hhmm
  if (second) *second = date - (t1 * 100);

  t2 = t1 / 100;              // t2 contains hh
  if (hour) *hour = t2;
  if (minute) *minute = t1 - (t2 * 100);
}

/*
 * Reset some GPS status variables. This should be called when GPS fix
 * is lost.
 */
void TinyGPS::resetGPSstatusVars(void)
{
    _satsused   = GPS_INVALID_SATELLITES;
    _satsinview = GPS_INVALID_SATELLITES;
    _fixtype    = GPS_INVALID_FIXTYPE;
    _hdop       = GPS_INVALID_HDOP;
}

float TinyGPS::f_altitude()
{
  return _altitude == GPS_INVALID_ALTITUDE ? GPS_INVALID_F_ALTITUDE : _altitude / 100.0;
}

float TinyGPS::f_course()
{
  return _course == GPS_INVALID_ANGLE ? GPS_INVALID_F_ANGLE : _course / 100.0;
}

float TinyGPS::f_speed_knots()
{
  return _speed == GPS_INVALID_SPEED ? GPS_INVALID_F_SPEED : _speed / 100.0;
}

float TinyGPS::f_speed_mph()
{
  float sk = f_speed_knots();
  return sk == GPS_INVALID_F_SPEED ? GPS_INVALID_F_SPEED : _GPS_MPH_PER_KNOT * sk;
}

float TinyGPS::f_speed_mps()
{
  float sk = f_speed_knots();
  return sk == GPS_INVALID_F_SPEED ? GPS_INVALID_F_SPEED : _GPS_MPS_PER_KNOT * sk;
}

float TinyGPS::f_speed_kmph()
{
  float sk = f_speed_knots();
  return sk == GPS_INVALID_F_SPEED ? GPS_INVALID_F_SPEED : _GPS_KMPH_PER_KNOT * sk;
}

const float TinyGPS::GPS_INVALID_F_ANGLE = 1000.0;
const float TinyGPS::GPS_INVALID_F_ALTITUDE = 1000000.0;
const float TinyGPS::GPS_INVALID_F_SPEED = -1.0;

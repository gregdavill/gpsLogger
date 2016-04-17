#pragma once

#ifndef GPS_UTIL_H__
#define GPS_UTIL_H__

#include <stdint.h>


uint8_t gps_util_valid(uint8_t c);
uint8_t* gps_util_get_last_valid_line(void);

uint8_t gps_util_is_RMC(uint8_t* s);
uint8_t gps_util_fix_valid(uint8_t* s);


uint8_t gps_util_extract_lat_degrees(uint8_t* in, uint8_t* out);
uint8_t gps_util_extract_lat_minutes(uint8_t* in, uint8_t* out);
uint8_t gps_util_extract_north(uint8_t* in);
uint8_t gps_util_extract_long_degrees(uint8_t* in, uint8_t* out);
uint8_t gps_util_extract_long_minutes(uint8_t* in, uint8_t* out);
uint8_t gps_util_extract_west(uint8_t* in);

uint8_t gps_util_extract_time(uint8_t* in, uint8_t* out);
uint8_t gps_util_extract_elevation(uint8_t* in, uint8_t* out);
void gps_util_update_timezone (char* file_date, char* file_time);


#endif

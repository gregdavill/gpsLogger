
#include <stdio.h>
#include <string.h>
#include "gps_util.h"



typedef enum {
	VALID_INIT = 0,
	VALID_CAPTURE,
	VALID_CRC_CHECK_1,
	VALID_CRC_CHECK_2,
} validState_t;

uint8_t buff[128];


validState_t valid_state = VALID_INIT;
uint8_t valid_line_crc;
uint8_t* valid_line_ptr;
uint8_t valid_line_length;


/*
 * Take charaters at a time, return 1 if valid string found
 */
uint8_t gps_util_valid(uint8_t c)
{

	uint8_t return_value = 0;

	switch (valid_state)
	{
		case VALID_INIT:
		{
			if (c == '$')
			{
				valid_state = VALID_CAPTURE;
				valid_line_crc = 0;
				valid_line_ptr = buff;
				valid_line_length = 0;
			}
		}
		break;

		case VALID_CAPTURE:
		{
			switch (c)
			{
				case '\r':
				case '\n':
				case '$':
				{
					valid_state = VALID_INIT;
					break;
				}

				case '*':
				{
					/* next we campare crc */
					valid_state = VALID_CRC_CHECK_1;
					break;
				}

				default:
				{
					if (valid_line_length++ > 120) /* line too long? */
					{
						valid_state = VALID_INIT;
						break;
					}
					*valid_line_ptr++ = c;
					valid_line_crc ^= c;
				}
			}
		}
		break;

		case VALID_CRC_CHECK_1:
		{
			if (c >= 'A' | c >= 'a') /* Hex to dec */
			{
				c -= ('a' + 10);
			}
			else if (c >= '0' && c <= '9')
			{
				c -= '0';
			}
			else
			{
				valid_state = VALID_INIT; /* crc format error */
				break;
			}

			if ((c & 0xF) != ((valid_line_crc >> 4) & 0xF))
			{
				valid_state = VALID_INIT; /* crc MSB not good */
			}
			else
			{
				valid_state = VALID_CRC_CHECK_2; /* crc MSB not good */
			}
		}
		break;

		case VALID_CRC_CHECK_2:
		{
			if (c >= 'A' | c >= 'a') /* Hex to dec */
			{
				c -= ('a' + 10);
			}
			else if (c >= '0' && c <= '9')
			{
				c -= '0';
			}
			else
			{
				valid_state = VALID_INIT; /* crc format error */
				break;
			}

			if ((c & 0xF) == ((valid_line_crc) & 0xF)) /* crc good! */
			{
				*valid_line_ptr++ = '0';
				return_value = 1;
			}

			valid_state = VALID_INIT; /* line done return */
		}
		break;

	}

	return return_value;
}


uint8_t* gps_util_get_last_valid_line(void)
{
	return buff;
}

uint8_t gps_util_is_RMC(uint8_t* s)
{
	if (strncmp((const char*)s, "GPRMC", 5) == 0)
		return 1;
	return 0;
}

/* only checking RMC strings */
/* $GPRMC,220516,[A],5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70 */
uint8_t gps_util_fix_valid(uint8_t* s)
{
	uint8_t* s_ptr = s;
	/* skip type, time*/
	for (uint8_t skip = 0; skip < 2; skip++)
	{
		while (*s_ptr != ',' && *s_ptr != 0)
			s_ptr++;
		s_ptr++;
	}
	
	if (*s_ptr == 'A')
		return 1;

	return 0;

}


uint8_t gps_util_extract_lat_degrees(uint8_t* in, uint8_t* out)
{
	uint8_t* s_ptr = in;

	/* skip type, time, fix*/
	for (uint8_t skip = 0; skip < 3; skip++)
	{
		while (*s_ptr != ',' && *s_ptr != 0)
			s_ptr++;
		s_ptr++;
	}

	/* determine where decimal point is located, this determines if degrees is 2 or 3 digits. */
	uint8_t maxlen = 0;
	uint8_t* temp = s_ptr;
	while (*temp != '.' && *temp != 0) {
		temp++;
		maxlen++;
	}
	maxlen -= 2;


	uint8_t len = 0;
	do
	{
		if (*s_ptr == ',' || *s_ptr == 0)
			break;

		*out++ = *s_ptr++;
	} while (++len < maxlen);

	*out = 0;

	return 1;
}

uint8_t gps_util_extract_lat_minutes(uint8_t* in, uint8_t* out)
{
	uint8_t* s_ptr = in;

	/* skip type, time, fix*/
	for (uint8_t skip = 0; skip < 3; skip++)
	{
		while (*s_ptr != ',' && *s_ptr != 0)
			s_ptr++;
		s_ptr++;
	}

	/* determine where decimal point is located, then step back two digits */
	while (*s_ptr != '.' && *s_ptr != 0)
		s_ptr++;
	s_ptr -= 2;

	uint8_t len = 0;
	do
	{
		if (*s_ptr == ',' || *s_ptr == 0)
			break;

		*out++ = *s_ptr++;
	} while (++len <= 12); /* XX.YYY */
	
	*out = 0;

	return 1;
}

uint8_t gps_util_extract_north(uint8_t* in)
{
	uint8_t* s_ptr = in;

	/* skip type, time, fix, lat*/
	for (uint8_t skip = 0; skip < 4; skip++)
	{
		while (*s_ptr != ',' && *s_ptr != 0)
			s_ptr++;
		s_ptr++;
	}

	if (*s_ptr == 'N')
		return 0;
	return 1;
}



uint8_t gps_util_extract_long_degrees(uint8_t* in, uint8_t* out)
{
	uint8_t* s_ptr = in;

	/* skip type, time, fix, lat, N, */
	for (uint8_t skip = 0; skip < 5; skip++)
	{
		while (*s_ptr != ',' && *s_ptr != 0)
			s_ptr++;
		s_ptr++;
	}

	/* determine where decimal point is located, this determines if degrees is 2 or 3 digits. */
	uint8_t maxlen = 0;
	uint8_t* temp = s_ptr;
	while (*temp != '.' && *temp != 0) {
		temp++;
		maxlen++;
	}
	maxlen -= 2;


	uint8_t len = 0;
	do
	{
		if (*s_ptr == ',' || *s_ptr == 0)
			break;

		*out++ = *s_ptr++;
	} while (++len < maxlen);

	*out = 0;

	return 1;
}

uint8_t gps_util_extract_long_minutes(uint8_t* in, uint8_t* out)
{
	uint8_t* s_ptr = in;

	/* skip type, time, fix, lat, N*/
	for (uint8_t skip = 0; skip < 5; skip++)
	{
		while (*s_ptr != ',' && *s_ptr != 0)
			s_ptr++;
		s_ptr++;
	}

	/* determine where decimal point is located, then step back two digits */
	while (*s_ptr != '.' && *s_ptr != 0)
		s_ptr++;
	s_ptr -= 2;

	uint8_t len = 0;
	do
	{
		if (*s_ptr == ',' || *s_ptr == 0)
			break;

		*out++ = *s_ptr++;
	} while (++len <= 12); /* XX.YYYYYY */

	*out = 0;

	return 1;
}

uint8_t gps_util_extract_west(uint8_t* in)
{
	uint8_t* s_ptr = in;

	/* skip type, time, fix, lat, N, Long */
	for (uint8_t skip = 0; skip < 6; skip++)
	{
		while (*s_ptr != ',' && *s_ptr != 0)
			s_ptr++;
		s_ptr++;
	}

	if (*s_ptr == 'W')
		return 1;
	return 0;
}

uint8_t gps_util_extract_time(uint8_t* in, uint8_t* out)
{
	uint8_t* s_ptr = in;

	/* skip type, */
	for (uint8_t skip = 0; skip < 1; skip++)
	{
		while (*s_ptr != ',' && *s_ptr != 0)
			s_ptr++;
		s_ptr++;
	}

	uint8_t len = 0;
	do
	{
		if (*s_ptr == ',' || *s_ptr == '.' || *s_ptr == 0)
			break;

		*out++ = *s_ptr++;
	} while (++len < 6); /* XX.YYYYYY */

	*out = 0;

	return 1;
}


uint8_t gps_util_extract_date(uint8_t* in, uint8_t* out)
{
	uint8_t* s_ptr = in;

	/* skip type, $GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70*/
	for (uint8_t skip = 0; skip < 9; skip++)
	{
		while (*s_ptr != ',' && *s_ptr != 0)
			s_ptr++;
		s_ptr++;
	}

	uint8_t len = 0;
	do
	{
		if (*s_ptr == ',' || *s_ptr == 0)
			break;

		*out++ = *s_ptr++;
	} while (++len < 6); /* XX.YYYYYY */

	*out = 0;

	return 1;
}

uint8_t gps_util_time_is_larger(uint8_t* a, uint8_t* b)
{
	for (uint8_t counter = 0; counter < 6; counter++)
	{
		if (*a > *b)
			return 1;
		else if (*a < *b)
			return 0;

		a++, b++;
	}
	return 0;
}


uint16_t gps_year;
uint8_t gps_day,gps_month;
uint8_t gps_hour,gps_minute,gps_second;

const char days_of_month[] = {0,31,29,31,30,31,30,31,31,30,31,30,31};
uint8_t isLeap(uint16_t y)
{
	if( y % 4 == 0 && y % 100 != 0 && y % 400 == 0)
		return 1;
	return 0;
}

void gps_util_update_timezone (char* file_date, char* file_time)
{

    uint8_t day = ((file_date[0] - '0') * 10) + (file_date[1] - '0');
	uint8_t month = ((file_date[2] - '0') * 10) + (file_date[3] - '0');
    uint16_t year = ((file_date[4] - '0') * 10) + (file_date[5] - '0') + 2000;

    uint8_t hour = ((file_time[0] - '0') * 10) + (file_time[1] - '0');
    uint8_t minute = ((file_time[2] - '0') * 10) + (file_time[3] - '0');
    uint8_t second = ((file_time[4] - '0') * 10) + (file_time[5] - '0');


    /* apply simple timezone formating */
    gps_minute += 30;
    if( gps_minute >= 60)
    {
    	gps_minute -= 60;
    	gps_hour++;
    }

    gps_hour += 9;
    if(gps_hour >= 24)
    {
    	gps_hour -= 24;
    	gps_day++;
    	if((gps_day > days_of_month[gps_month]) || (!isLeap(gps_year) && gps_month == 2 && gps_day == 29))
    	{
    		gps_day = 1;
    		if(++gps_month > 12)
    		{
    			gps_month = 1;
    			gps_year++;
    		}
    	}
    }
}

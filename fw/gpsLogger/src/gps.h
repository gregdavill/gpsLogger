
#ifndef GPS_H
#define GPS_H
#include <stdint.h>


void GPS_init(void);
uint8_t gps_check(void);
void gps_do(void);
void gps_start(void);
void gps_stop(void);
void USCI_A0_ISR(void);

void inc_time();
uint32_t millis();


void check_new_firmware();
void rename_firmware();

#endif

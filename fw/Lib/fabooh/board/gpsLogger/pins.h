/*
  * pins.h - gpsLogger GPIO port and pins
 *
 * Created: Nov 20, 2012
 *  Author: rick@kimballsoftware.com
 *    Date: 03-04-2012
 * Version: 1.0.3
 *
 * Note: this file is a work in progress don't expect it to be final ...
 *
 * =========================================================================
 *  Copyright © 2016 Greg Davill
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef GPSLOGGER_NOBOARD_H_
#define GPSLOGGER_NOBOARD_H_

/*
 * P1 port and P1.X shorthand typedefs
 */
#ifdef __MSP430_HAS_PORT1_R__
 typedef GPIO_PORT<
         1
         ,PAIN_L
         ,PAOUT_L
         ,PADIR_L
		 ,PAIFG_L
		 ,PAIES_L
		 ,PAIE_L
         ,PASEL_L
         ,PAREN_L
         > P1;

 typedef GPIO_PIN<BIT0,P1> P1_0;
 typedef GPIO_PIN<BIT1,P1> P1_1;
 typedef GPIO_PIN<BIT2,P1> P1_2;
 typedef GPIO_PIN<BIT3,P1> P1_3;
 typedef GPIO_PIN<BIT4,P1> P1_4;
 typedef GPIO_PIN<BIT5,P1> P1_5;
 typedef GPIO_PIN<BIT6,P1> P1_6;
 typedef GPIO_PIN<BIT7,P1> P1_7;

#endif

 /*
 * P1 port and P1.X shorthand typedefs
 */
 #ifdef __MSP430_HAS_PORT2_R__
 typedef GPIO_PORT<
 	  2
 	  ,PAIN_H
 	  ,PAOUT_H
 	  ,PADIR_H
 	  ,PAIFG_H
 	  ,PAIES_H
 	  ,PAIE_H
 	  ,PASEL_H
 	  ,PAREN_H
 	  > P2;

 typedef GPIO_PIN<BIT0,P2> P2_0;
 typedef GPIO_PIN<BIT1,P2> P2_1;
 typedef GPIO_PIN<BIT2,P2> P2_2;
 typedef GPIO_PIN<BIT3,P2> P2_3;
 typedef GPIO_PIN<BIT4,P2> P2_4;
 typedef GPIO_PIN<BIT5,P2> P2_5;
 typedef GPIO_PIN<BIT6,P2> P2_6;
 typedef GPIO_PIN<BIT7,P2> P2_7;

 #endif


 /*
 * P1 port and P1.X shorthand typedefs
 */
 #ifdef __MSP430_HAS_PORT3_R__
 typedef GPIO_PORT_BASE<
 	  3
 	  ,PBIN_L
 	  ,PBOUT_L
 	  ,PBDIR_L
 	  ,PBSEL_L
 	  ,PBREN_L
 	  > P3;

 typedef GPIO_PIN<BIT0,P3> P3_0;
 typedef GPIO_PIN<BIT1,P3> P3_1;
 typedef GPIO_PIN<BIT2,P3> P3_2;
 typedef GPIO_PIN<BIT3,P3> P3_3;
 typedef GPIO_PIN<BIT4,P3> P3_4;
 typedef GPIO_PIN<BIT5,P3> P3_5;
 typedef GPIO_PIN<BIT6,P3> P3_6;
 typedef GPIO_PIN<BIT7,P3> P3_7;

 #endif


 /*
 * P1 port and P1.X shorthand typedefs
 */
 #ifdef __MSP430_HAS_PORT4_R__
 typedef GPIO_PORT_BASE<
 	  4
 	  ,PBIN_H
 	  ,PBOUT_H
 	  ,PBDIR_H
 	  ,PBSEL_H
 	  ,PBREN_H
 	  > P4;

 typedef GPIO_PIN<BIT0,P4> P4_0;
 typedef GPIO_PIN<BIT1,P4> P4_1;
 typedef GPIO_PIN<BIT2,P4> P4_2;
 typedef GPIO_PIN<BIT3,P4> P4_3;
 typedef GPIO_PIN<BIT4,P4> P4_4;
 typedef GPIO_PIN<BIT5,P4> P4_5;
 typedef GPIO_PIN<BIT6,P4> P4_6;
 typedef GPIO_PIN<BIT7,P4> P4_7;

 #endif


 /*
  * P5 port and P5.X shorthand typedefs
  */
 #ifdef __MSP430_HAS_PORT5_R__
  typedef GPIO_PORT_BASE<
          5
          ,PCIN_L
          ,PCOUT_L
          ,PCDIR_L
          ,PCSEL_L
          ,PCREN_L
          > P5;

  typedef GPIO_PIN<BIT0,P5> P5_0;
  typedef GPIO_PIN<BIT1,P5> P5_1;
  typedef GPIO_PIN<BIT2,P5> P5_2;
  typedef GPIO_PIN<BIT3,P5> P5_3;
  typedef GPIO_PIN<BIT4,P5> P5_4;
  typedef GPIO_PIN<BIT5,P5> P5_5;
  typedef GPIO_PIN<BIT6,P5> P5_6;
  typedef GPIO_PIN<BIT7,P5> P5_7;

 #endif


/*
 * P6 port and P6.X shorthand typedefs
 */
#ifdef __MSP430_HAS_PORT6_R__
 typedef GPIO_PORT_BASE<
         6
         ,PCIN_H
         ,PCOUT_H
         ,PCDIR_H
         ,PCSEL_H
         ,PCREN_H
         > P6;

 typedef GPIO_PIN<BIT0,P6> P6_0;
 typedef GPIO_PIN<BIT1,P6> P6_1;
 typedef GPIO_PIN<BIT2,P6> P6_2;
 typedef GPIO_PIN<BIT3,P6> P6_3;
 typedef GPIO_PIN<BIT4,P6> P6_4;
 typedef GPIO_PIN<BIT5,P6> P6_5;
 typedef GPIO_PIN<BIT6,P6> P6_6;
 typedef GPIO_PIN<BIT7,P6> P6_7;

#endif


 enum led_value {
   LED_ON=0x00
   ,LED_OFF=0x01
 };

 typedef P6_0 V_SENSE;
 typedef P6_1 V_SENSE_ENABLE;

 typedef P1_0 POWER_BUTTON;
 typedef P1_1 BATTERY_CHARGE_STATUS;

 typedef P2_5 GPS_BACKUP_POWER;
 typedef P2_7 GPS_PPS;
 typedef P3_0 GPS_POWER_ENABLE;
 typedef P3_1 GPS_POWER_FAULT;
 typedef P3_3 GPS_RX;
 typedef P3_4 GPS_TX;

 typedef P4_0 MMC_CLK;
 typedef P4_1 MMC_POWER_ENABLE;
 typedef P4_2 MMC_POWER_FAULT;
 typedef P4_3 MMC_CS;
 typedef P4_4 MMC_MOSI;
 typedef P4_5 MMC_MISO;

 typedef P6_3 LED_A_RED;
 typedef P6_5 LED_A_GREEN;
 typedef P6_6 LED_A_BLUE;
 typedef P6_7 LED_B_RED;
 typedef P5_0 LED_B_GREEN;
 typedef P5_1 LED_B_BLUE;



#endif /* MSP430FR5739_NOBOARD_H_ */

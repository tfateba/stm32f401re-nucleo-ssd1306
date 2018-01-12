/**
 *
 * @file    ds1307.h
 *
 * @brief   ds1307 driver header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    21 June 2015
 *
 */

#ifndef DS1317_H
#define DS1317_H

/*===========================================================================*/
/* Include Files.                                                            */
/*===========================================================================*/
#include "ch.h"
#include "hal.h"
#include <stdlib.h>
#include "chprintf.h"

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   DS1307 data structure and type.
 */
typedef struct {
	uint8_t     seconds;
	uint8_t     minutes;
	uint8_t     hours;
	uint8_t     day;
	uint8_t     date;
	uint8_t     month;
	uint16_t    year;
}ds1307_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*
 * @note RTC Reception and Transmission Buffer Size
 */
#define DS1307_RX_DEPTH     7
#define DS1307_TX_DEPTH     8

/*
 * @note RTC I2C and Rgeister Address
 */
#define DS1307_ADDRESS      0x68
#define DS1307_SECONDS_REG  0x00

/*
 * @note RTC Days definition
 */
#define SUNDAY              1
#define MONDAY              2
#define TUESDAY             3
#define WEDNESDAY           4
#define THURSDAY            5
#define FRIDAY              6
#define SATURDAY            7

/*
 * @note Months definition
 */
#define JANUARY             1
#define FEBRUARY            2
#define MARCH               3
#define APRIL               4
#define MAY                 5
#define JUNE                6
#define JULY                7
#define AUGUST              8
#define SEPTEMBER           9
#define OCTOBER             10
#define NOVEMBER            11
#define DECEMBER            12

/*===========================================================================*/
/* Functions.                                                                */
/*===========================================================================*/

uint8_t bcd2Dec(uint8_t val);
uint8_t dec2Bcd(uint8_t val);
void ds1307InterfaceInit(void);
msg_t ds1307SetDate(ds1307_t *dsData);
void ds1307Print(ds1307_t *dsData);
msg_t ds1307GetDate(ds1307_t *dsData);

#endif /* DS1307_H */

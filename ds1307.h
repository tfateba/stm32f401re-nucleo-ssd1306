/**
 *
 * @file    ds1307.h
 *
 * @brief   ds1307 interface library header.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @version 1.0
 *
 * @date    21 June 2015
 *
 * @update  26 July 2016
 *
 */

#ifndef _DSLIB_H_
#define _DSLIB_H_

/*===========================================================================*/
/* Include Files                                                             */
/*===========================================================================*/
#include "ch.h"
#include "hal.h"
#include <stdlib.h>
#include "chprintf.h"

/*===========================================================================*/
/* Driver data structures and types                                          */
/*===========================================================================*/

/**
 * @note DS1307 data structure and type
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

/**
 * @fn    bcd2Dec
 * @brief Convert BCD to Decimal
 *
 * @param[in]	val Value to convert from BCD to Decimal
 * @return		    Converted Decimal value
 */
uint8_t bcd2Dec(uint8_t val);

/**
 * @fn    dec2Bcd
 * @brief Convert Decimal to BCD
 *
 * @param[in]	val Value to convert from Decimal to BCD
 * @return		    Converted BCD value
 */
uint8_t dec2Bcd(uint8_t val);

/**
 * @fn		ds1307InterfaceInit
 * @brief	Configure the I2C Interface 1 and start the Interface
 */
void ds1307InterfaceInit(void);

/**
 * @fn    ds1307SetDate
 * @brief Configuration of the RTC
 *
 * @param[in]	dsData  Pointer of data structure use to set the RTC
 * @return    msg     The result of the operation
 */
msg_t ds1307SetDate(ds1307_t *dsData);

/**
 * @fn    ds1307Print
 * @brief Print the clock and date read from the DS1307 RTC
 *
 * @param[in]	calendar Pointer of the structure contening the data to print
 */
void ds1307Print(ds1307_t *dsData);

/**
 * @fn    ds1307GetDate
 * @brief Read data from RTC
 *
 * @param[out]  dsData  Pointer of data structure for the RTC reading
 * @return      msg     Result of the reading operation
 */
msg_t ds1307GetDate(ds1307_t *dsData);

#endif // _DS1307_H_

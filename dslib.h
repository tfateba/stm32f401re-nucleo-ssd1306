/*
 * @file     dslib.h
 * @brief    ds1307 library
 * @author   Theodore ATEBA
 * @version  1.0
 * @date     21 June 2015
 *
 */

#ifndef _DSLIB_H_
#define _DSLIB_H_

//=============================================================================
// Include Files
//=============================================================================
  #include "ch.h"
  #include "hal.h"
  #include <stdlib.h>
  #include "chprintf.h"
	

//=============================================================================
// Global variables, I2C TX and RX buffers, I2C and Serial Configurations
//=============================================================================
    #define DS1307_RX_DEPTH 7
    #define DS1307_TX_DEPTH 8

    #define DS1307_ADDRESS      0x68
    #define DS1307_SECONDS_REG	0x00

    // DS1307 data structure and type
    typedef struct ds1307_t
    {
        uint8_t     seconds;
        uint8_t     minutes;
        uint8_t     hours;
        uint8_t     day;
        uint8_t     date;
        uint8_t     month;
        uint16_t    year;
    }ds1307;


//=============================================================================
// Functions
//=============================================================================

/*
 * @fn          uint8_t bcd2Dec (uint8_t val)
 * @brief       Convert BCD to Decimal
 * @param[in]   val value to convert from BCD to Decimal
 * @return      res Converted value
 */
uint8_t bcd2Dec (uint8_t val);

/*
 * @fn           uint8_t dec2Bcd (uint8_t val)
 * @brief        Convert Decimal to BCD
 * @param[in]    val value to convert from Decimal to BCD
 * @return       res Converted value
 */
uint8_t dec2Bcd (uint8_t val);
 
 /*
 * @fn       void ds1307InterfaceInit (void)
 * @brief   Configure the I2C Interface 1 and start the Interface
 */
void ds1307InterfaceInit (void);

/*
 * @fn          void ds1307SetDate (struct ds1307_t dsData);
 * @brief       Set the clock and the calendar of the RTC
 * @param[in]   dsData data used to set Clock and Calendar
 */
void ds1307SetDate (struct ds1307_t dsData);

/*
 * @fn           void ds1307Print (struct ds1307_t dsData)
 * @brief        print the clock and date read from the DS1307 RTC
 * @param[in]    calendar structure contening the data to print
 */
void ds1307Print (struct ds1307_t dsData);

/*
 * @fn           struct ds1307_t ds1307GetDate (void)
 * @brief        Get Clock and Calendar
 */
struct ds1307_t ds1307GetDate (void);

#endif // _DSLIB_H_

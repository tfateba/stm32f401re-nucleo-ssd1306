/*
 * @file     dsLib.c
 * @brief    ds1307 library
 * @author   Theodore ATEBA
 * @version  1.0
 * @date     21 June 2015
 *
 */

//=============================================================================
// Include Files
//=============================================================================
    #include "dslib.h"

//=============================================================================
// Global variables, I2C TX and RX buffers, I2C and Serial Configurations
//=============================================================================

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

  extern BaseSequentialStream* chp;

  static uint8_t rxbuf[DS1307_RX_DEPTH];
  static uint8_t txbuf[DS1307_TX_DEPTH];
  static i2cflags_t errors = 0;

  // I2C interface #1
  static const I2CConfig i2cConf = {
    OPMODE_I2C,
    400000,
    FAST_DUTY_CYCLE_2,
  };



//=============================================================================
// Functions
//=============================================================================

/*
 * @fn          static uint8_t bcd2Dec (uint8_t val)
 * @brief       Convert BCD to Decimal
 * @param[in]   val value to convert from BCD to Decimal
 * @return      res Converted value
 */
uint8_t bcd2Dec (uint8_t val) 
{
    uint8_t res = ((val/16*10) + (val % 16));
    return res;
}

/*
 * @fn           static uint8_t dec2Bcd (uint8_t val)
 * @brief        Convert Decimal to BCD
 * @param[in]    val value to convert from Decimal to BCD
 * @return       res Converted value
 */
uint8_t dec2Bcd (uint8_t val)
{
    uint8_t res = ((val/10*16) + (val%10));
    return res;
}

/*
 * @fn       void ds1307InterfaceInit (void)
 * @brief    Configure the I2C Interface 1 and start the Interface
 */
void ds1307InterfaceInit (void)
{
    i2cStart (&I2CD1, &i2cConf);
    palSetPadMode (GPIOB, 8, PAL_MODE_ALTERNATE(4) |
    PAL_STM32_OTYPE_OPENDRAIN ); // SCL
    palSetPadMode (GPIOB, 9, PAL_MODE_ALTERNATE(4) | 
    PAL_STM32_OTYPE_OPENDRAIN ); // SDA
}

/*
 * @fn          void ds1307SetDate (struct ds1307_t dsData)
 * @brief       Set the clock and the calendar of the RTC
 * @param[in]   dsData data used to set Clock and Calendar
 */
void ds1307SetDate (struct ds1307_t dsData)
{
    msg_t status;

    txbuf[0] = DS1307_SECONDS_REG;
    txbuf[1] = dec2Bcd (dsData.seconds);
    txbuf[2] = dec2Bcd (dsData.minutes);
    txbuf[3] = dec2Bcd (dsData.hours);
    txbuf[4] = dec2Bcd (dsData.day);
    txbuf[5] = dec2Bcd (dsData.date);
    txbuf[6] = dec2Bcd (dsData.month);
    txbuf[7] = dec2Bcd (dsData.year - 2000);

    i2cAcquireBus (&I2CD1);
    status = i2cMasterTransmitTimeout (&I2CD1, DS1307_ADDRESS, txbuf, 
                DS1307_TX_DEPTH, NULL, 0, MS2ST(10));
    i2cReleaseBus (&I2CD1);

    if (status != MSG_OK)
        chprintf(chp, "\n\r Error when setting the DS1307 date over the I2C bus.");
    else
        chprintf(chp, "\n\r DS1307 was setting succefuly.");
}

/*
 * @fn           void ds1307Print (struct ds1307_t dsData)
 * @brief        print the clock and date read from the DS1307 RTC
 * @param[in]    calendar structure contening the data to print
 */
void ds1307Print (struct ds1307_t dsData)
{
  chprintf(chp, "\n\r RTC: Date is, %d / %d / %d %d:%d:%d", dsData.date, 
dsData.month, dsData.year, dsData.hours, dsData.minutes, dsData.seconds);
}

/*
 * @fn		struct ds1307_t ds1307GetDate (void)
 * @brief	Get Clock and Calendar
 */
struct ds1307_t ds1307GetDate (void)
{
    msg_t status;
    struct ds1307_t dsData;

    txbuf[0] = DS1307_SECONDS_REG; // Register address of the Seconds
    i2cAcquireBus(&I2CD1);
    status = i2cMasterTransmitTimeout (&I2CD1, DS1307_ADDRESS, txbuf, 1, 
                rxbuf, 7, MS2ST(10));
    i2cReleaseBus (&I2CD1);

    if (status != MSG_OK)
    {
        errors = i2cGetErrors (&I2CD1);
       chprintf(chp, "\n\r RTC: Get data error!, NOK");
    }
    else
    {    
	    dsData.seconds  = bcd2Dec (rxbuf[0] & 0x7F);
	    dsData.minutes  = bcd2Dec (rxbuf[1]);
	    dsData.hours    = bcd2Dec (rxbuf[2] & 0x3F);
	    dsData.day      = bcd2Dec (rxbuf[3]);
	    dsData.date     = bcd2Dec (rxbuf[4]);
	    dsData.month    = bcd2Dec (rxbuf[5]);
	    dsData.year     = bcd2Dec (rxbuf[6]) + 2000;
    }
    return dsData;
}

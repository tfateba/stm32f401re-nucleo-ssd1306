/**
 *
 * @file    tmlib.h
 *
 * @brief   TM1637 driver header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    04 July 2015
 *
 */

#ifndef TM1637_H
#define TM1637_H

/*===========================================================================*/
/* Include Files.                                                            */
/*===========================================================================*/
#include <stdlib.h>
#include "ch.h"
#include "hal.h"
#include "ds1307.h"

/*===========================================================================*/
/* Driver data structure and type.                                           */
/*===========================================================================*/

/*===========================================================================*/
/* Driver Macros.                                                            */
/*===========================================================================*/
#define DIO_PIN           GPIOC_PIN8  /**< DIO pin for i2c communication.    */ 
#define CLK_PIN           GPIOC_PIN9  /**< CLK pin for i2c communication.    */
#define TM1637_I2C_COMM1  0x40        /**< TM1637 first i2c address.         */
#define TM1637_I2C_COMM2  0xC0        /**< TM1637 second i2c address.        */
#define TM1637_I2C_COMM3  0x80        /**< TM1637 third i2c address.         */
#define POINT_ON          TRUE        /**< Turn ON the display clock points. */
#define POINT_OFF         FALSE       /**< Turn OFF the display clock points.*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/*===========================================================================*/
/* Functions prototypes.                                                     */
/*===========================================================================*/

void bitDelay(void);
void start(void);
void stop(void);
uint8_t encodeDigit(uint8_t digit);
void setBrightness(uint8_t  bright);
bool writeByte(uint8_t byte);
void setSegments(const uint8_t data[], uint8_t length, uint8_t pos);
void showNumberDec(uint16_t num, bool leading_zero, uint8_t length, uint8_t pos);
void showTime (ds1307_t *clock, bool dp, uint8_t msg);

#endif /* TM1637_H */

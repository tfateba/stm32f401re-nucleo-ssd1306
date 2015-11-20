/*
* @file     tmlib.h
* @brief    TM1637 C library
* @author   Theodore ATEBA
* @version  1.0
* @date 04  Jully 2015
*/

#ifndef _TM1637_H_
#define _TM1637_H_

//=============================================================================
// Include Files
//=============================================================================
    #include <stdlib.h>
    #include "ch.h"
    #include "hal.h"
    #include "dslib.h"

//=============================================================================
// Global variables, I2C TX and RX buffers, I2C and Serial Configurations
//=============================================================================

    // Declare DIO Pin
    #define DIO_PIN GPIOC_PIN8
    #define CLK_PIN GPIOC_PIN9

    // TM1637 Address
    #define TM1637_I2C_COMM1    0x40
    #define TM1637_I2C_COMM2    0xC0
    #define TM1637_I2C_COMM3    0x80

    // Clock Point Definition
    #define POINT_ON    TRUE
    #define POINT_OFF   FALSE

//=============================================================================
// Functions
//=============================================================================

/*
 * @fn      void bitDelay (void)
 * @brief   Width of a bit on DIO bus
 */
void bitDelay (void);

/*
 * @fn      void start (void)
 * @brief   Generation of the start condition on the DIO bus.
 */
void start (void);

/*
 * @fnn     void stop (void)
 * @brief   Generation of the stop condition on the DIO bus.
 */
void stop (void);

/*
 * @fn          uint8_t encodeDigit (uint8_t digit)
 * @brief       Encode the data to print on the 4-digit Display
 * @param[in]   digit the data to convert for the Display
 */
uint8_t encodeDigit(uint8_t digit);

/*
 * @fn          void digitBrightness (uint8_t brightness)
 * @brief       set the brightness of the 4-Digit Display.
 * @note        0 is the dimmest and 255 is the brightest
 * @note        set before calling four_digit_display
 * @param[in]   brightness The level of the brightness we want to set
 */
void setBrightness (uint8_t  bright);
    
/*
 * @fn          bool writeByte (uint8_t byte)
 * @brief       write a byte to the GPIO pin used as DIO
 * @param[in]   byte Data to write on the DIO pin
 */
bool writeByte(uint8_t byte);

//void four_digit_clear (void)

/*
 * @fn          void setSegments (const uint8_t segments[], uint8_t length, uint8_t pos)
 * @brief       print the data on the segments
 * @param[in]   segments array of data to print on the 4-digit Display
 * @param[in]   length Size of the array of data to print
 * @param[in]   pos Position of the first digit to update
 */
void setSegments (const uint8_t segments[], uint8_t length, uint8_t pos);

/*
 * @fn          void showNumberDec (uint16_t num, bool leading_zero, uint8_t length, uint8_t pos)
 * @brief       Print Decimal number on the 4-Digit Display
 * @param[in]   num number to print
 * @param[in]   leading_zero 
 * @param[in]   length
 * @param[in]   pos position of the digit
 */
void showNumberDec(uint16_t num, bool leading_zero, uint8_t length, uint8_t pos);

/*
 * @fn          void showTime (struc ds1307_t clock, bool dp, uint8_t msg)
 * @brief       Convert and print data on 4Digit-Display
 * @param[in]   clock data date structure.
 * @param[in]   dp flag of double point for clock
 */
void showTime (struct ds1307_t clock, bool dp, uint8_t msg);

#endif // _TM1637_H_

/**
 *
 * @file     tmlib.c
 *
 * @brief    TM1637 driver source file.
 *
 * @author   Theodore Ateba, tf.ateba@gmail.com
 *
 * @date      04 July 2015
 *
 */

/*===========================================================================*/
/* Include Files.                                                            */
/*===========================================================================*/
#include "tmlib.h"

/*===========================================================================*/
/* Local varaibles and types.                                                */
/*===========================================================================*/
uint8_t brightness; /**< 4-digits display brightness                         */

const uint8_t digitToSegments[] = {
0x3F, // 0
0x06, // 1
0x5B, // 2
0x4F, // 3
0x66, // 4
0x6D, // 5
0x7D, // 6
0x07, // 7
0x7F, // 8
0x6F, // 9
0x77, // A
0x7C, // b
0x39, // C
0x5E, // d
0x79, // E
0x71, // F
};

/*===========================================================================*/
/* Functions                                                                 */
/*===========================================================================*/

/**
 * @brief   Width of a bit on DIO bus.
 */
void bitDelay(void) {

  osalThreadSleepMicroseconds(50);
}

/**
 * @brief   Generation of the start condition on the DIO bus.
 */
void start(void) {

  palSetPadMode(GPIOC, DIO_PIN, PAL_MODE_OUTPUT_PUSHPULL);
  bitDelay();
}

/**
 * @brief   Generation of the stop condition on the DIO bus.
 */
void stop(void) {

  palSetPadMode(GPIOC, DIO_PIN, PAL_MODE_OUTPUT_PUSHPULL);
  bitDelay();
  palSetPadMode(GPIOC, CLK_PIN, PAL_MODE_INPUT_PULLUP);
  bitDelay();
  palSetPadMode(GPIOC, DIO_PIN, PAL_MODE_INPUT_PULLUP);
  bitDelay();
}

/**
 * @brief   Encode the data to print on the 4-digit Display.
 *
 * @param[in] digit   the data to convert for the Display
 */
uint8_t encodeDigit(uint8_t digit) {

  return digitToSegments [digit & 0x0F];
}

/**
 * @brief   set the brightness of the 4-Digit Display.
 * @note    0 is the dimmest and 255 is the brightest
 * @note    set before calling four_digit_display
 *
 * @param[in] brightness  the level of the brightness we want to set
 */
void setBrightness(uint8_t  bright) {

  brightness = bright;
}

/**
 * @brief write a byte to the GPIO pin used as DIO.
 *
 * @param[in] byte  data to write on the DIO pin
 */
bool writeByte(uint8_t byte) {

    uint8_t data = byte;
    uint8_t i;

    for (i = 0; i < 8; i++) {
        palSetPadMode(GPIOC, CLK_PIN, PAL_MODE_OUTPUT_PUSHPULL);
        bitDelay ();

        if (data & 0x01)
            palSetPadMode(GPIOC, DIO_PIN, PAL_MODE_INPUT_PULLUP);
        else
            palSetPadMode(GPIOC, DIO_PIN, PAL_MODE_OUTPUT_PUSHPULL);

        bitDelay ();

        palSetPadMode(GPIOC, CLK_PIN, PAL_MODE_INPUT_PULLUP);
        bitDelay ();
        data = data >> 1;
    }

    palSetPadMode(GPIOC, CLK_PIN, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOC, DIO_PIN, PAL_MODE_INPUT_PULLUP);
    bitDelay ();

    palSetPadMode(GPIOC, CLK_PIN, PAL_MODE_INPUT_PULLUP);
    bitDelay ();
    uint8_t ack = palReadPad(GPIOC, DIO_PIN);
    if (ack == 0)
        palSetPadMode(GPIOC, DIO_PIN, PAL_MODE_OUTPUT_PUSHPULL);

    bitDelay ();
    palSetPadMode(GPIOC, CLK_PIN, PAL_MODE_OUTPUT_PUSHPULL);
    bitDelay();

    return ack;
}

/**
 * @brief   Print the data on the segments.
 *
 * @param[in] data    array of data to print on the 4-digits display
 * @param[in] length  size of the array of data to print
 * @param[in] pos     position of the first digit to update
 */
void setSegments(const uint8_t data[], uint8_t length, uint8_t pos) {

  uint8_t i;

  /* Write COMM1. */
  start ();
  writeByte (TM1637_I2C_COMM1);
  stop ();

  /* Wirte COMM2 + first digit address. */
  start ();
  writeByte (TM1637_I2C_COMM2 + (pos & 0x03));

  /* Write the data byte. */
  for (i = 0; i < length; i++)
    writeByte (data[i]);
  stop ();

  /* Write COMM3 + Brightness. */
  start ();
  writeByte (TM1637_I2C_COMM3 + (brightness & 0x0F));
  stop ();
}

/**
 * @brief   Print Decimal number on the 4-digits display.
 *
 * @param[in] num           number to print
 * @param[in] leading_zero  TODO: comment
 * @param[in] length        size of array of data to print
 * @param[in] pos           position of the digit
 */
void showNumberDec(uint16_t num, bool leading_zero, uint8_t length, uint8_t pos) {

  uint8_t digits[4], i;
  const uint16_t divisors[] = {1, 10, 100, 1000};
  bool leading = true;

  for (i = 0; i < 4; i++) {
    uint8_t divisor = divisors[3 - i];
    uint8_t d = num / divisor;

    if (d == 0){
      if (leading_zero || !leading || (i == 3))
        digits[i] = encodeDigit(d);
      else
        digits[i] = 0;
    }
    else {
      digits[i] = encodeDigit(d);
      num -= d * divisor;
      leading = false;
    }
  }
  setSegments (digits + (4 - length), length, pos);
}

/**
 * @brief   Convert and print data on 4Digit-Display.
 *
 * @param[in] clock   data date structure.
 * @param[in] dp      flag of double point for clock
 * @param[in] msg     code of data to print on the 4-digits display
 */
void showTime (ds1307_t *clock, bool dp, uint8_t msg) {

  uint8_t d1, d2, d3, d4, data[4];
  uint16_t year, divisor[4] = {1000, 100, 10, 1};

  switch (msg) {
    case 0:
      d1 = clock->seconds%10;
      d2 = (clock->seconds - d1)/10;
      d3 = clock->minutes%10;
      d4 = (clock->minutes - d3)/10;
      data[0] = encodeDigit (d4);
      if (dp)
        data[1] = 0x80 | encodeDigit (d3);
      else
        data[1] = encodeDigit (d3);
      data[2] = encodeDigit (d2);
      data[3] = encodeDigit (d1);
      setSegments(data, 4, 0);
      break;

    case 1:
      d1 = clock->minutes%10;
      d2 = (clock->minutes - d1)/10;
      d3 = clock->hours%10;
      d4 = (clock->hours - d3)/10;
      data[0] = encodeDigit (d4);
      if (dp)
        data[1] = 0x80 | encodeDigit (d3);
      else
        data[1] = encodeDigit(d3);
      data[2] = encodeDigit(d2);
      data[3] = encodeDigit(d1);
      setSegments(data, 4, 0);
      break;
    case 2:
      showNumberDec (clock->day, false, 4, 0);
      break;

    case 3:
      showNumberDec (clock->date, false, 4, 0);
      break;

    case 4:
      showNumberDec (clock->month, false, 4, 0);
      break;

    case 5:
      year = clock->year;

      for (d1 = 0; d1 < 4; d1++) {
        data[d1] = year/divisor[d1];

        if (data[d1])
          year -= (data[d1] * divisor[d1]);
        else
          data[d1] = 0;
        showNumberDec (data[d1], true, 1, d1);
      }
      break;

    default:
      break;
  }
}


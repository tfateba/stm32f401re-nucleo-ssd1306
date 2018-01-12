/**
 *
 * @file    ssd1306.c
 *
 * @brief   SSD1306 controler driver for Oled LCD.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    21 June 2015
 *
 */

/*===========================================================================*/
/* Include Files.                                                            */
/*===========================================================================*/
#include "ssd1306.h"
#include "font.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
extern BaseSequentialStream* chp;

#define SCREEN_HEIGHT		64
#define SCREEN_WIDTH		128
//#define INITIAL_CONTRAST	0xFF

/* @brief   I2C interface. */
static const I2CConfig i2cConf = {
  OPMODE_I2C,         /**< I2C Mode.                */
  400000,             /**< I2C interface speed.     */
  FAST_DUTY_CYCLE_2,  /**< TODO: comment this part. */
};

/* The memory buffer for the display. */
uint8_t gdisp_buffer[SCREEN_HEIGHT * SCREEN_WIDTH / 8];
int pBuff         = 0;

/* Parametres de communication i2c. */
uint8_t ssd_addr  = 0x3C;
uint8_t control   = 0x00;
uint8_t data      = 0x40;

/* Current position of the cursor. */
uint8_t m_col     = 0;
uint8_t m_row     = 0;

/* Box pattern. */
uint8_t pattern1[] = {
  0x00,0x7E,0x42,0x42,0x42,0x42,0x7E,0x00,
  0x00,0x7E,0x42,0x42,0x42,0x42,0x7E,0x00
};

/* Cross weave pattern. */
uint8_t pattern2[] = {
  0x81,0x42,0x24,0x18,0x18,0x24,0x42,0x81,
  0x81,0x42,0x24,0x18,0x18,0x24,0x42,0x81
};

/* Box pattern. */
uint8_t pattern3[] = {
  0x00, 0x00, 0x1E, 0x30, 0x3E, 0x33, 0x6E, 0x00,
  0x0C, 0x1E, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x00
};

/*===========================================================================*/
/* Functions.                                                                */
/*===========================================================================*/

/**
 * @brief   Write a command to the i2c bus for the lcd.
 *
 * @param[in] cmd   command to write
 * @return    msg   the result of the writing operation
 */
msg_t writeCmd(uint8_t cmd) {

  uint8_t command[2];
  uint8_t txLength = sizeof(command)/sizeof(command[0]);
  uint8_t rxLength = 0;

  command[0] = 0x00; /* Co = 0, D/C = 0. */
  command[1] = cmd;

  i2cAcquireBus(&I2CD1);
  msg_t msg = i2cMasterTransmitTimeout(&I2CD1, ssd_addr, command, txLength,
      NULL, rxLength, MS2ST(4));
  i2cReleaseBus(&I2CD1);

  return msg;
}

/**
 * @brief   Write the data to the i2C bus for the lcd.
 *
 * @param[in] data    data to send to the lcd
 * @param[in] length  size of the data to send
 * @return    msg     the result of the writing operation
 */
msg_t writeData(uint8_t* data, uint16_t length) {

  uint8_t command[length+1];
  uint8_t txLength = length+1;
  uint8_t rxLength = 0;

  command[0] = 0x40; 		/* Co = 0, D/C = 1. */
  memmove(&command[1], data, length);

  i2cAcquireBus (&I2CD1);
  msg_t msg = i2cMasterTransmitTimeout(&I2CD1, ssd_addr, command, txLength,
      NULL, rxLength, MS2ST(10));
  i2cReleaseBus (&I2CD1);

  return msg;
}

/**
 * @brief Reset of the LCD.
 */
void ssdReset(void) {

  /* Reset sequence. */
  palClearPad(RESET_PORT, SSD_RESET_PIN);
  chThdSleepMilliseconds(1);
  palSetPad(RESET_PORT, SSD_RESET_PIN);
  chThdSleepMilliseconds(1);
  writeCmd(SSD1306_DISPLAY_ON);
  /* Wait the SEG/COM signal to be ON after 100ms. */
  chThdSleepMilliseconds(100);
}

/**
 * @brief   Set Multiplexer ration TODO.
 *
 * @param[in] ratio   ration to set
 */
void ssdSetMuxRatio(uint8_t ratio) {

  writeCmd(SSD1306_SET_MULTIPLEX);
  writeCmd(ratio & 0x3F);
}

/**
 * @brief   Use to set the LCD display offset.
 *
 * @param[in] offset  the offset of the LCD
 */
void ssdSetDisplayOffset(uint8_t offset) {

  writeCmd(SSD1306_SET_DISPLAY_OFFSET);
  writeCmd(offset & 0x3F);
}

/**
 * @brief   Use to set the line number.
 *
 * @param[in] line    the line to set
 */
void ssdSetDisplayStartLine(uint8_t line) {

  writeCmd(SSD1306_SET_START_LINE | line);
}

/**
 * @brief   Set the segment remap, TODO.
 *
 * @param[in]	value   segment remap value
 */
void ssdSetSegmentRemap(uint8_t value) {

	writeCmd(value ? SSD1306_SEGREMAP+1 : SSD1306_SEGREMAP);
}

/**
 * @brief   Set start adress parameter.
 *
 * @param[in] address   the start adress param
 */
void ssdSetStartAddressPam(uint8_t address) {

  writeCmd(address & 0x0F);
  writeCmd((address << 4) & 0x0F);
}

/**
 * @brief   Set the memomry addressing mode.
 *
 * @param[in] mode    memory addressing mode
 */
void ssdSetMemoryAddressingMode(uint8_t mode) {

  writeCmd(SSD1306_MEMORY_MODE);
  writeCmd(mode & 0x3);
}

/**
 * @brief   Set com output direction.
 *
 * @param[in] direction   direction of the com output
 */
void ssdSetComOutputScanDirection(uint8_t direction) {

  writeCmd(direction ? SSD1306_COM_SCANDEC : SSD1306_COM_SCANINC);
}

/**
 * @brief   Set com pins hardware configuration.
 */
void ssdSetComPinsHardwareConfiguration(void) {

  writeCmd(SSD1306_SET_COM_PINS);
  writeCmd(0x02);
}

/**
 * @brief   Set the LCD contrast.
 *
 * @param[in] contrast  contrast to set on the LCD
 */
void ssdSetContrastControl(uint8_t contrast) {

  writeCmd(SSD1306_SET_CONTRAST);
  writeCmd(contrast);
}

/**
 * @brief   Disable the entire display.
 */
void ssdDisableEntireDisplayOn(void) {

  writeCmd(SSD1306_DISPLAY_ALL_ON_RESUME);
}

/**
 * @brief   Set the normal display.
 */
void ssdSetNormalDisplay(void) {

  writeCmd(SSD1306_NORMAL_DISPLAY);
}

/**
 * @brief   Set the LCD frequency.
 */
void ssdSetOscFrequency(void) {

  writeCmd(SSD1306_SET_DISPLAY_CLOCK_DIV);
  writeCmd(0x80);
}

/**
 * @brief   Enable the charge pum regulator of the LCD.
 *
 * @param[in] enable    use to enable or disable the pum regulator
 */
void ssdEnableChargePumpRegulator(bool enable) {

  writeCmd(SSD1306_ENABLE_CHARGE_PUMP);
  writeCmd(enable ? 0x14: 0x10);
}

/**
 * @brief   Used to invert the display.
 *
 * @param[in] state   invert the display if state is true
 */
void ssdInvertedDisplay(bool state) {

  writeCmd(state ? SSD1306_INVERT_DISPLAY : SSD1306_NORMAL_DISPLAY);
}

/**
 * @brief   Turn off the display.
 */
void ssdDisplayOff(void) {

  writeCmd(SSD1306_DISPLAY_OFF);
}

/**
 * @brief   Turn on the display.
 */
void ssdDisplayOn(void) {

  writeCmd(SSD1306_DISPLAY_ON);
}

/**
 * @brief   Set All the LCD Pixel ON TODO: Verify this comment :).
 */
void ssdDisplayAllOn(void) {

  writeCmd(SSD1306_DISPLAY_ALL_ON);
}

/**
 * @brief   Set the culumn range.
 *
 * @param[in]	culumn  the culumn range to configure
 */
void ssdSetColumnRange(uint8_t culumn) {

  writeCmd(SSD1306_HV_COLUMN_ADDRESS);
  writeCmd(culumn);
}

/**
 * @brief   Set the page Range.
 *
 * @param[in]	page  the page range to configure
 */
void ssdSetPageRange(uint8_t page) {

	writeCmd(0x22);
	writeCmd(page);
}

/**
 * @brief   Display lines on the LCD.
 */
void ssdDisplayLines(void) {

  uint8_t i;
  uint8_t GDDRAM[129] = {0x00};

  GDDRAM[0] = 0x40;

  for (i = 0; i< 17; i++)
    GDDRAM[i+1] = 0x81;

  i2cAcquireBus(&I2CD1);
  i2cMasterTransmitTimeout(&I2CD1, ssd_addr, GDDRAM, 17, NULL, 0, MS2ST(4));
	i2cReleaseBus(&I2CD1);
}

/**
 * @brief   Display boxes on the LCD.
 */
void ssdDisplayBox(void) {

  uint8_t i;
  uint8_t GDDRAM[17] = {0x00};

  GDDRAM[0] = 0x40;

  for (i = 0; i < 16; i++)
    GDDRAM[i+1] = pattern1[i];

  i2cAcquireBus(&I2CD1);
  i2cMasterTransmitTimeout(&I2CD1, ssd_addr, GDDRAM, 17, NULL, 0, MS2ST(10));
  i2cReleaseBus(&I2CD1);
}

/**
 * @brief   Display cross on the LCD.
 */
void ssdDisplayCross(void) {

  uint8_t i;
  uint8_t GDDRAM[17] = {0x00};

  GDDRAM[0] = 0x40;

  for (i = 0; i < 16; i++)
    GDDRAM[i+1] = pattern2[i];

  i2cAcquireBus(&I2CD1);
  i2cMasterTransmitTimeout (&I2CD1, ssd_addr, GDDRAM, 17, NULL, 0, MS2ST(4));
  i2cReleaseBus(&I2CD1);
}

/**
 * @brief   Print Zero on all the display.
 */
void ssdDisplayZero(void) {

  uint8_t i;
  uint8_t GDDRAM[17] = {0x00};

  GDDRAM[0] = 0x40;

  for (i = 0; i < 16; i++)
    GDDRAM[i+1] = pattern3[i];

  i2cAcquireBus (&I2CD1);
  i2cMasterTransmitTimeout (&I2CD1, ssd_addr, GDDRAM, 17, NULL, 0, MS2ST(4));
  i2cReleaseBus (&I2CD1);
}

/**
 * @brief   Display the param argument on the LCD.
 *
 * @param[in] data  the data to print on the LCD screen
 * @return    msg   the result of the display operation
 */
msg_t ssdDisplay(uint8_t *data) {

  uint8_t i;
  uint8_t GDDRAM[17] = {0x00};

  GDDRAM[0] = 0x40;

  for (i = 0; i < 16; i++)
    GDDRAM[i+1] = *(data+i);

  i2cAcquireBus(&I2CD1);
  msg_t msg = i2cMasterTransmitTimeout(&I2CD1, ssd_addr, GDDRAM, 17, NULL, 0,
      MS2ST(10));
  i2cReleaseBus(&I2CD1);

  return msg;
}

/**
 * @brief   Set the LCD cursor.
 *
 * @param[in] culumn  the cursor vertical position on the LCD screen
 * @param[in] line    the cursor horizontal position on the LCD screen
 */
void ssdSetCursor(uint8_t column, uint8_t line) {

  m_col = column;
  m_row = line;
  writeCmd(0xB0 + m_row);         /* set page address.          */
  writeCmd(m_col & 0xf);          /* set lower column address.  */
  writeCmd(0x10 | (m_col >> 4));  /* set higher column address. */
}

/**
 * @brief   Update the LCD display state.
 */
void ssdDisplayUpdate(void) {

  writeCmd(0xA4);
}

/**
 * @brief   Initialization of the LCD.
 */
void oledInit(void) {

  /* Tell the SSD1306 that a command stream is incoming. */
  writeCmd(SSD1306_CONTROL_BYTE_CMD_STREAM);

  /* Follow instructions on pg.64 of the dataSheet for software */
  /* configuration of the SSD1306 Turn the Display OFF.         */
  writeCmd(SSD1306_CMD_DISPLAY_OFF);

  /* Set mux ration tp select max number of rows - 64. */
  writeCmd(SSD1306_CMD_SET_MUX_RATIO);
  writeCmd(0x3F);

  /* Set the display offset to 0. */
  writeCmd(SSD1306_CMD_SET_DISPLAY_OFFSET);
  writeCmd(0x00);

  /* Display start line to 0. */
  writeCmd(SSD1306_CMD_SET_DISPLAY_START_LINE);

  /* Mirror the x-axis. In case you set it up such that the pins are north. */
  /* Wire.write(0xA0); - in case pins are south - default                   */
  writeCmd(SSD1306_CMD_SET_SEGMENT_REMAP);

  /* Mirror the y-axis. In case you set it up such that the pins are north. */
  /* Wire.write(0xC0); - in case pins are south - default                   */
  writeCmd(SSD1306_CMD_SET_COM_SCAN_MODE);

	/* Default - alternate COM pin map. */
  writeCmd(SSD1306_CMD_SET_COM_PIN_MAP);
  writeCmd(0x12);

  /* set contrast. */
  writeCmd(SSD1306_CMD_SET_CONTRAST);
  writeCmd(0x7F);

  /* Set display to enable rendering from GDDRAM (Graphic Display Data RAM). */
  writeCmd(SSD1306_CMD_DISPLAY_RAM);

  /* Normal mode!. */
  writeCmd(SSD1306_CMD_DISPLAY_NORMAL);

  /* Default oscillator clock. */
  writeCmd(SSD1306_CMD_SET_DISPLAY_CLK_DIV);
  writeCmd(0x00);

  /* Enable the charge pump. */
  writeCmd(SSD1306_CMD_SET_CHARGE_PUMP);
  writeCmd(0x14);

  /* Set precharge cycles to high cap type. */
  writeCmd(SSD1306_CMD_SET_PRECHARGE);
  writeCmd(0x22);

  /* Set the V_COMH deselect volatage to max. */
  writeCmd(SSD1306_CMD_SET_VCOMH_DESELCT);
  writeCmd(0x30);

  /* Horizonatal addressing mode - same as the KS108 GLCD. */
  writeCmd(SSD1306_CMD_SET_MEMORY_ADDR_MODE);
  writeCmd(0x00);

  /* Turn the Display ON. */
  writeCmd(SSD1306_CMD_DISPLAY_ON);
}

/**
 * @brief   Set column address.
 *
 * @param[in]	start   the starting column
 * @param[in]	end     the ending column
 */
void ssdSetColumnAddressHvam(uint8_t start, uint8_t end) {

  writeCmd(SSD1306_HV_COLUMN_ADDRESS);
  writeCmd(start & 0x7F);
  writeCmd(end & 0x7F);
}

/**
 * @brief   Set page.
 *
 * @param[in]	start   the starting page
 * @param[in]	end     the ending page
 */
void ssdSetPageAddressHvam(uint8_t start, uint8_t end) {

	writeCmd(SSD1306_HV_PAGE_ADDRESS);
	writeCmd(start & 0x07);
	writeCmd(end & 0x07);
}

/**
 * @brief   Set page start param.
 *
 * @param[in]	address   the starting parameter
 */
void ssdSetPageStartPam(uint8_t address) {

  writeCmd(SSD1306_PAM_PAGE_START | (address & 0x07));
}

/**
 * @brief   Set the display clock ration and frequency.
 *
 * @param[in]	ratio       the ratio of the display
 * @param[in]	frequency   the frequency of the display
 */
void ssdSetDisplayClockRatioAndFrequency(uint8_t ratio, uint8_t frequency) {

  writeCmd(SSD1306_SET_DISPLAY_CLOCK_DIV);
  writeCmd((ratio & 0x0F) | ((frequency & 0x0F) << 4));
}

/**
 * @brief   Set precharge period.
 *
 * @param[in]	phase1  TODO: comment
 * @param[in]	phase2  TODO: comment
 */
void ssdSetPrechargePeriod(uint8_t phase1, uint8_t phase2)  {

  writeCmd(SSD1306_SET_PRECHARGE);
  writeCmd((phase1 & 0x0F) | ((phase2 & 0x0F ) << 4));
}

/**
 * @brief   Set the detection level.
 *
 * @param[in] level   detection level
 */
void ssdSetVcomhDeselectLevel(uint8_t level) {

  writeCmd(SSD1306_SET_VCOMDETECT);
  writeCmd((level & 0x03) << 4);
}

/**
 * @brief   Use to enable or disable the charge pump.
 *
 * @param[in]	enable    enable the charge pum
 */
void ssdSetChargePump(uint8_t enable) {

  writeCmd(SSD1306_ENABLE_CHARGE_PUMP);
  writeCmd(enable ? 0x14 : 0x10);
}

/**
 * @brief   Draw a pixel on the LCD.
 *
 * @param[in]	x       horizontal position
 * @param[in]	y       vertical position
 * @param[in]	color   pixel color, black or white
 */
void ssdDrawPixel(uint16_t x, uint16_t y, uint8_t color) {

  if (color == SSD1306_WHITE)
    gdisp_buffer[x+ (y/8)*SCREEN_WIDTH] |=  (1<<y%8);
  else
    gdisp_buffer[x+ (y/8)*SCREEN_WIDTH] &= ~(1<<y%8);
}

/**
 * @brief   Display the buffer content on the LCD.
 */
void ssdDisplayBuff(void) {

  int i, j;
  uint8_t command[SCREEN_WIDTH/2];
  uint8_t cmdLength = sizeof(command)/sizeof(command[0]);
  uint8_t parts = SCREEN_WIDTH/cmdLength;

  ssdSetDisplayStartLine(0);

  for (i=0; i<SCREEN_HEIGHT/8; i++) {
    for (j = 0; j<parts; j++) {
      memmove(command, &gdisp_buffer[i*SCREEN_WIDTH + j*cmdLength], cmdLength);
      writeData(command, cmdLength);
    }
  }
}

/**
 * @brief   Function use to print a character to the LCD.
 *
 * @param[in] ch    character to print on the LCD
 * @param[in] font  character font used.
 */
void ssdCharFont(char ch, uint8_t font) {

  uint8_t i;
  uint8_t c = ch - 32;

  if(font == 2) {
    for (i = 0; i < 6; i++) {
      gdisp_buffer[i] = (ssd1306_6x8Font[c][i]);
    }
    writeData(gdisp_buffer, 6);
  }
  if(font == 1) {
    for (i = 0; i < 5; i++) {
      gdisp_buffer[i] = (ssd1306_5x8Font[c][i]);
    }
    writeData(gdisp_buffer, 5);
  }
}

/**
 * @brief   Function use to print string on the LCD.
 *
 * @param[in] string  data to print on the LCD
 * @param[in] font    define font size to use whie printing data on the LCD
 */
void ssdString(char *string, uint8_t font) {

  while (*string) {
    ssdCharFont(*string++, font);
  }
}

/**
 * @brief   Clean the current line.
 *
 * @param[in] line  the index of the line to be erase
 */
static void clearLine(uint8_t line) {

  uint8_t i;

  ssdSetCursor(0, line);

  for (i = 0; i < 26; i++)
    ssdString(" ", 1);
}

/**
 * @brief   Clean the data on the entire display.
 */
static void cleanEntireDisplay(void) {

  uint8_t line;

  for (line = 0; line < 8; line++) {
    ssdSetCursor(0, line);
    clearLine(line);
  }
}

/**
 * @brief   Shows the demo on the LCD.
 */
void oledDemo(void) {

  int i;

  cleanEntireDisplay();
  for (i = 0; i < 1024; i++)
    ssdDisplayLines();
  chThdSleepMilliseconds(2000);

  cleanEntireDisplay();
  for (i = 0; i < 1024; i++)
    ssdDisplayBox();
  chThdSleepMilliseconds(2000);

  cleanEntireDisplay();
  for (i = 0; i < 1024; i++)
    ssdDisplayCross();
  chThdSleepMilliseconds(2000);

  cleanEntireDisplay();

  clearLine(1);
  ssdSetCursor(0, 1);
  ssdString("ChibiOS/RT", 2);

  clearLine(3);
  ssdSetCursor(0, 3);
  ssdString("Date: 07 / 11 / 2015", 1);

  clearLine(5);
  ssdSetCursor(0, 5);
  ssdString("Learn by doing", 1);

  clearLine(7);
  ssdSetCursor(0, 7);
  ssdString("Implemented by tfateba", 1);
  chThdSleepMilliseconds(2000);
}

/**
 * @brief Shows the demo on the LCD.
 *
 * @param[in] calendar  structure to print on the LCD
 */
void oledPrintDate(ds1307_t *calendar) {

  char asciiDay[2];
  char asciiMonth[2];
  char asciiYear[4];
  char asciiSeconds[2];
  char asciiMinutes[2];
  char asciiHours[2];

  itoa(calendar->date, asciiDay, 10);
  itoa(calendar->month, asciiMonth, 10);
  itoa(calendar->year, asciiYear, 10);
  itoa(calendar->seconds, asciiSeconds, 10);
  itoa(calendar->minutes, asciiMinutes, 10);
  itoa(calendar->hours, asciiHours, 10);

  clearLine(3);
  ssdSetCursor(0, 3);
  ssdString("Date: ", 1);
  ssdString(asciiDay, 1);
  ssdString(" / ", 1);
  ssdString(asciiMonth, 1);
  ssdString(" / ", 1);
  ssdString(asciiYear, 1);

  clearLine(5);
  ssdSetCursor(0, 5);
  ssdString("Time: ", 1);
  ssdString(asciiHours, 1);
  ssdString(" : ", 1);
  ssdString(asciiMinutes, 1);
  ssdString(" : ", 1);
  ssdString(asciiSeconds, 1);
}


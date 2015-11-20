/*
 * @file     ssd1306.h
 * @brief    Oled LCD library
 * @author   Theodore ATEBA
 * @version  1.0
 * @date     8 November 2015
 *
 */
#ifndef _SSD1306_H_
#define _SSD1306_H_

//=============================================================================
// Include Files
//=============================================================================
#include "ch.h"
#include "hal.h"
#include "dslib.h"
//#include "font.h"
#include <stdlib.h>
#include "chprintf.h"
#include "string.h"

// Macro for Pin RESET of the LCD.
#define RESET_PORT		GPIOA
#define SSD_RESET_PIN	GPIOA_LED_GREEN

// Color of the Pixel
#define SSD1306_BLACK 0
#define SSD1306_WHITE 1

// SSD1306 Command Instruction set
#define SSD1306_SET_CONTRAST							0x81
#define SSD1306_DISPLAY_ALL_ON_RESUME 					0xA4
#define SSD1306_DISPLAY_ALL_ON 							0xA5
#define SSD1306_NORMAL_DISPLAY 							0xA6
#define SSD1306_INVERT_DISPLAY 							0xA7
#define SSD1306_DISPLAY_OFF 							0xAE
#define SSD1306_DISPLAY_ON 								0xAF
#define SSD1306_SET_DISPLAY_OFFSET 						0xD3
#define SSD1306_SET_COM_PINS 							0xDA
#define SSD1306_SET_VCOMDETECT 							0xDB // TODO find in the documentation
#define SSD1306_SET_DISPLAY_CLOCK_DIV 					0xD5
#define SSD1306_SET_PRECHARGE 							0xD9
#define SSD1306_ENABLE_CHARGE_PUMP						0x8D
#define SSD1306_SET_MULTIPLEX 							0xA8
#define SSD1306_SET_START_LINE 							0x40
#define SSD1306_MEMORY_MODE 							0x20
#define SSD1306_HV_COLUMN_ADDRESS						0x21
#define SSD1306_HV_PAGE_ADDRESS							0x22
#define SSD1306_PAM_PAGE_START							0xB0
#define SSD1306_COM_SCANINC 							0xC0 // TODO
#define SSD1306_COM_SCANDEC 							0xC8 // TODO
#define SSD1306_SEGREMAP 								0xA0 // TODO
#define SSD1306_CHARGEPUMP 								0x8D
#define SSD1306_EXTERNAL_VCC 							0x1
#define SSD1306_SWITCH_CAPVCC 							0x2 // TODO
// Scrolling #defines
#define SSD1306_SCROLL_ACTIVATE 						0x2F
#define SSD1306_SCROLL_DEACTIVATE 						0x2E
#define SSD1306_SCROLL_SET_VERTICAL_SCROLL_AREA 		0xA3
#define SSD1306_SCROLL_HORIZONTAL_RIGHT 				0x26
#define SSD1306_SCROLL_HORIZONTAL_LEFT 					0x27
#define SSD1306_SCROLL_VERTICAL_AND_HORIZONTAL_RIGHT 	0x29
#define SSD1306_SCROLL_VERTICAL_AND_HORIZONTAL_LEFT		0x2A

// I2C address of the OLED LCD
#define OLED_I2C_ADDRESS   								0x3C
// Type of command to send to the Oled LCD
#define SSD1306_CONTROL_BYTE_CMD_SINGLE					0x80
#define SSD1306_CONTROL_BYTE_CMD_STREAM					0x00
#define SSD1306_CONTROL_BYTE_DATA_STREAM				0x40
// Fundamental commands (pg.28)
#define SSD1306_CMD_SET_CONTRAST						0x81
#define SSD1306_CMD_DISPLAY_RAM							0xA4
#define SSD1306_CMD_DISPLAY_ALLON						0xA5
#define SSD1306_CMD_DISPLAY_NORMAL						0xA6
#define SSD1306_CMD_DISPLAY_INVERTED					0xA7
#define SSD1306_CMD_DISPLAY_OFF							0xAE
#define SSD1306_CMD_DISPLAY_ON							0xAF
// Addressing Command Table (pg.30)
#define SSD1306_CMD_SET_MEMORY_ADDR_MODE				0x20
#define SSD1306_CMD_SET_COLUMN_RANGE					0x21
#define SSD1306_CMD_SET_PAGE_RANGE						0x22
// Hardware Config (pg.31)
#define SSD1306_CMD_SET_DISPLAY_START_LINE				0x40
#define SSD1306_CMD_SET_SEGMENT_REMAP					0xA1
#define SSD1306_CMD_SET_MUX_RATIO						0xA8
#define SSD1306_CMD_SET_COM_SCAN_MODE					0xC8
#define SSD1306_CMD_SET_DISPLAY_OFFSET					0xD3
#define SSD1306_CMD_SET_COM_PIN_MAP						0xDA
// Timing and Driving Scheme (pg.32)
#define SSD1306_CMD_SET_DISPLAY_CLK_DIV					0xD5
#define SSD1306_CMD_SET_PRECHARGE						0xD9
#define SSD1306_CMD_SET_VCOMH_DESELCT					0xDB
// Charge Pump (pg.62)
#define SSD1306_CMD_SET_CHARGE_PUMP 					0x8D
// NOP
#define SSD1306_CMD_NOP 								0xE3



#define USINT2DECASCII_MAX_DIGITS 5

//=============================================================================
// Functions
//=============================================================================
void ssdReset(void);
void ssdSetMuxRatio(uint8_t ratio);
void ssdSetDisplayOffset(uint8_t offset);
void ssdSetDisplayStartLine(uint8_t line);
void ssdSetSegmentRemap(uint8_t value);
void ssdSetComOutputScanDirection(uint8_t direction);
void ssdSetComPinsHardwareConfiguration(void);
void ssdSetContrastControl(uint8_t contrast);
void ssdDisableEntireDisplayOn(void);
void ssdSetNormalDisplay(void);
void ssdSetOscFrequency(void);
void ssdEnableChargePumpRegulator(bool enable);
void ssdInvertedDisplay(bool state);
void ssdDisplayOff(void);
void ssdDisplayAllOn(void);
void ssdDisplayOn(void);
void ssdSetStartAddressPam(uint8_t address);
void ssdSetMemoryAddressingMode(uint8_t mode);
void ssdSetColumnAddressHvam(uint8_t start, uint8_t end);
void ssdSetPageAddressHvam(uint8_t start, uint8_t end);
void ssdSetPageStartPam(uint8_t address);
void ssdSetDisplayClockRatioAndFrequency(uint8_t ratio, uint8_t frequency);
void ssdSetPrechargePeriod(uint8_t phase1, uint8_t phase2);
void ssdSetVcomhDeselectLevel(uint8_t level);
void ssdSetChargePump(uint8_t enable);
void gdisp_lld_draw_pixel(uint16_t x, uint16_t y, uint8_t color);
void gdisp_lld_display(void);
void display(void);
void setColumnRange(uint8_t culumn);
void setPageRange(uint8_t page);
void ssdDisplayUpdate(void);
void oled_print_date(struct ds1307_t calendar);
void oled_demo(void);
void oledInit(void);
void displayBox(void);
void displayCross(void);
void display0(void);
void ssd1306_char_font(char ch, uint8_t font);
void ssd1306_string(char *s, uint8_t font);
void ssd1306SetCursor(uint8_t column, uint8_t line);

/**
 * @brief   Draws a pixel on the display.
 *
 * @param[in] x        X location of the pixel
 * @param[in] y        Y location of the pixel
 * @param[in] color    The color of the pixel
 *
 * @notapi
 */
void gdisp_lld_draw_pixel(uint16_t x, uint16_t y, uint8_t color);


/**
 * @brief   Take exclusive control of the bus
 * @notapi
 */
void acquireBus(void);

/**
 * @brief   Release exclusive control of the bus
 * @notapi
 */
void releaseBus(void);

/**
 * @brief   Send data to the display.
 * @param[in] data	The data to send
 * @notapi
 */
void writeData(uint8_t* data, uint16_t length);


/**
 * @brief   Send command to the display.
 * @param[in] cmd	The command to send *
 * @notapi
 */
void writeCmd(uint8_t cmd);

#endif /* _SSD1306_H */


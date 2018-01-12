/*
 * @file    main.c
 *
 * @brief   Control the oled LCD to print RTC data such as hour, day, time.
 *
 * @author	Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    15 October 2015
 *
 * TODO:
 *      	- Add a LDR to control the 4-Digit Display intensity
 *      	- Gestion des rebonds sur le BP pour eviter les pb d'affichage
 *		    Gestion du LCD
 *        - Add shell in order to read data from SD1 and update the RTC clock.
 */

/*===========================================================================*/
/* Include Files                                                             */
/*===========================================================================*/
#include <stdlib.h>
#include "ch.h"
#include "hal.h"
#include "ds1307.h"
#include "tmlib.h"
#include "chprintf.h"
#include "ssd1306.h"

BaseSequentialStream* chp = (BaseSequentialStream*) &SD2;

/*===========================================================================*/
/* Global variables, I2C TX and RX buffers, I2C and Serial Configurations.   */
/*===========================================================================*/

#define CONFIG_RTC false

ds1307_t calendar;      /* We will use the time for 4-digit Display.  */
bool dp_flag = false;   /* 4-Digit Double Point State.                */

/**
 * @brief Data to print on the 4-Digit Display
 *
 *        0 ==> minutes:secondes
 *        1 ==> hours:minutes
 *        2 ==> date
 *        3 ==> day
 *        4 ==> month
 *        5 ==> year
 */
uint8_t data2Print = 0;
uint16_t heartbeat = 0;

/*==========================================================================*/
/* Functions.                                                               */
/*==========================================================================*/

/* @brief   RTC reader thread. */
static THD_WORKING_AREA(waRtcReadThread, 128);
static THD_FUNCTION(RtcReadThread, arg) {

  (void)arg;

  chRegSetThreadName("RTC-Reader");
  while (TRUE) {
    ds1307GetDate(&calendar);
    chThdSleepMilliseconds(1000);
  }
}

/* @biref   RTC printer thread. */
static THD_WORKING_AREA(waRtcPrintThread, 128);
static THD_FUNCTION(RtcPrintThread, arg) {

  (void)arg;

  chRegSetThreadName("RTC-Printer");
  while (TRUE) {
    ds1307Print(&calendar);
    showTime(&calendar, dp_flag ^=1 , data2Print);
    oledPrintDate(&calendar);
    chThdSleepMilliseconds (1000);
  }
}

/* @brief    Alife Thread, Blink the LED. */
static THD_WORKING_AREA(waLedGreenThread, 128);
static THD_FUNCTION(BlinkThread, arg) {

  (void)arg;

  chRegSetThreadName("Led-Green-Binker");
  while (TRUE) {
    palTogglePad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(50);
  }

}
/* TODO: Faire une gestion de rebond sur le BP. */
/* Trigger the User Button.                     */
static void extcb(EXTDriver *extp, expchannel_t channel) {

  (void)extp;
  (void)channel;

  data2Print++;
  if (data2Print == 6)
    data2Print = 0;
}

/* Set the external interrupt to the push button pin. */
static const EXTConfig extConfig = {
  {
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, extcb},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
  }
};

/*
* @brief    Application entry point.
*/
int main(void){
	halInit();
	chSysInit();

	palSetPad(RESET_PORT, SSD_RESET_PIN);
	ds1307InterfaceInit();
	sdStart(&SD2, NULL);
	chprintf(chp, "\n\n\r ChibiOS: HAL initialisation.");
	chprintf(chp, "\n\r ChibiOS: System initialisation.");
	/* Set the GPIOs pin used. */
	palSetPadMode(GPIOC, DIO_PIN, PAL_MODE_INPUT_PULLUP);
	palClearPad(GPIOC, DIO_PIN);
	palSetPadMode(GPIOC, CLK_PIN, PAL_MODE_INPUT_PULLUP);
	palClearPad(GPIOC, CLK_PIN);
	chprintf(chp, "\n\r ChibiOS: I2C initialisation.");

	/* Activates the Button Interruption. */
	extStart(&EXTD1, &extConfig);
	chprintf(chp, "\n\r ChibiOS: Switch interruption routine handler attach.");
	chprintf(chp, "\n\r ChibiOS: Start LCD initialisation.");
	chprintf(chp, "\n\r ChibiOS: Reset and init the oLCD.");
	ssdReset();
	oledInit();

	chprintf(chp, "\n\r ChibiOS: LCD initialisation finished.");
	chprintf(chp, "\n\r ChibiOS: Real Time Clock calendar with:");
	chprintf(chp, "\n\r ChibiOS: --> Nucleo STM32F401RE Board");
	chprintf(chp, "\n\r ChibiOS: --> DS1307 RTC");
	chprintf(chp, "\n\r ChibiOS: --> 4-Digits Display");
	chprintf(chp, "\n\r ChibiOS: --> Oled I2C LCD 128*64.\n\r");

	#if defined CONFIG_RTC && CONFIG_RTC == true
		/* Used when you whant to set the calendar and clock. */
		calendar.seconds    = 0;
		calendar.minutes    = 48;
		calendar.hours      = 1;
		calendar.day        = SATURDAY;
		calendar.date       = 21;
		calendar.month      = NOVEMBER;
		calendar.year       = 2015;
		ds1307SetDate(calendar);
	#endif

  /**
   * Set the 4Digit display intensity before sending data to print.
   */
	setBrightness(0x0F);
	chprintf(chp, "\n\r ChibiOS: Set 4-digits display Brightness");

  /* @brief   Run the demo on the LCD. */
	oledDemo();

	chprintf(chp, "\n\r ChibiOS: Date reader and printer Threads cration.");
	chprintf(chp, "\n\r ChibiOS: DEBUT DU PROGRAMME SANS FIN.");
	chThdSleepMilliseconds(1000);

	/**
   * Create the thread used to read the RTC DS1307
   */
	chThdCreateStatic(waRtcReadThread, sizeof(waRtcReadThread), NORMALPRIO+1,
		RtcReadThread, NULL);

  /**
   * Create the thread used to print the RTC data every seconds
   */
	chThdCreateStatic(waRtcPrintThread, sizeof(waRtcPrintThread), NORMALPRIO,
		RtcPrintThread, NULL);

  /**
   * Create the thread for the on board LED
   */
  chThdCreateStatic(waLedGreenThread, sizeof(waLedGreenThread), LOWPRIO,
  BlinkThread, NULL);

  while (true) {
    chprintf(chp, "\n\r ChibiOS: heartbeat = %d", heartbeat++);
    chThdSleepMilliseconds(5000);
  }
  return 0;
}


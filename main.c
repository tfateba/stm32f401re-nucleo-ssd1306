/*
 * @file	main.c 
 * @brief	Control the oled LCD to print RTC data such as hour, day, time.
 * @author	Theodore ATEBA
 * @version 	1.0
 * @date 	15  October 2015
 * TODO:
 *      	add a LDR to control the 4-Digit Display intensity
 *      	Gestion des rebonds sur le BP pour eviter les pb d'affichage
 *		Gestion du LCD
 */


//=============================================================================
// Include Files
//=============================================================================
  #include <stdlib.h>
  #include "ch.h"
  #include "hal.h"
  #include "dslib.h"	// RTC Library
  #include "tmlib.h"	// 4 Digit Display Library
  #include "chprintf.h"
  #include "ssd1306.h"
  #include "num2str.h"

BaseSequentialStream* chp = (BaseSequentialStream*) &SD2;

//=============================================================================
// Global variables, I2C TX and RX buffers, I2C and Serial Configurations
//=============================================================================

    struct ds1307_t calendar; // We will use the time for 4-digit Display

    bool dp_flag = false; // 4-Digit Double Point State.

    // Data to print on the 4-Digit Display
    // 0 ==> minutes:secondes
    // 1 ==> hours:minutes
    // 2 ==> date
    // 3 ==> day
    // 4 ==> month
    // 5 ==> year
    uint8_t data2Print = 0;

uint16_t heartbeat = 0;


//=============================================================================
// Functions
//=============================================================================

// RTC reader thread
static THD_WORKING_AREA(waRtcReadThread, 128);
static THD_FUNCTION(RtcReadThread, arg)
{
    (void)arg;
    chRegSetThreadName("RTC-Reader");
    while(TRUE)
    {
        calendar = ds1307GetDate();
        chThdSleepMilliseconds(1000);
    }
}

// RTC printer thread
static THD_WORKING_AREA(waRtcPrintThread, 128);
static THD_FUNCTION(RtcPrintThread, arg)
{	
    (void)arg;
    chRegSetThreadName("RTC-Printer");
    while(TRUE)
    {
        ds1307Print(calendar); // Print RTC data on the console.
        //dp_flag ^=1;
        showTime(calendar, dp_flag ^=1 , data2Print); // Pirnt RTC data on the 4-Digit display
		oled_print_date(calendar);
	//	chprintf(chp, "\n\rcalendar.day = %d", calendar.date);
	//	chprintf(chp, "\n\rcalendar.month = %d", calendar.month);
	//	chprintf(chp, "\n\rcalendar.year = %d", calendar.year);
        chThdSleepMilliseconds (1000);
    }
}

// TODO: Faire une gestion de rebond sur le BP
// Trigger the User Button
static void extcb(EXTDriver *extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

    data2Print++;
    if(data2Print == 6)
        data2Print = 0;
}

// Set the external interrupt to the push button pin
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
* @fn       int main(void)
* @brief    Application entry point.
*/
int main(void){

  halInit(); // HAL Init
  chSysInit(); // Chibios System init
  
  palSetPad(RESET_PORT, SSD_RESET_PIN);
  ds1307InterfaceInit();
  sdStart(&SD2, NULL);
  chprintf(chp, "\n\n\r ChibiOS: HAL initialisation.");
  chprintf(chp, "\n\r ChibiOS: System initialisation.");
  
  // Set the GPIOs pin used
  palSetPadMode(GPIOC, DIO_PIN, PAL_MODE_INPUT_PULLUP);
  palClearPad(GPIOC, DIO_PIN);
  palSetPadMode(GPIOC, CLK_PIN, PAL_MODE_INPUT_PULLUP);
  palClearPad(GPIOC, CLK_PIN);
  chprintf(chp, "\n\r ChibOSs: I2C initialisation.");
  
  // Activates the Button Interruption
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
  
  // Used when you whant to set the calendar and clock
  calendar.seconds    = 0;
  calendar.minutes    = 40;
  calendar.hours      = 1; 
  calendar.day        = 2;
  calendar.date       = 14;
  calendar.month      = 10;
  calendar.year       = 2015;
  //ds1307SetDate(calendar);
  
  // Set the 4Digit display intensity before sending data to print.
  setBrightness(0x0F);
  chprintf(chp, "\n\r ChibiOS: Set 4-digits display Brightness");

  oled_demo();
  
  chprintf(chp, "\n\r ChibiOS: Date reader and printer Threads cration.");
  chprintf(chp, "\n\r ChibiOS: DEBUT DU PROGRAMME SANS FIN.");
  chThdSleepMilliseconds(1000);
  
  // Create the thread used to read the RTC DS1307
  chThdCreateStatic(waRtcReadThread, sizeof(waRtcReadThread), NORMALPRIO+1, 
      RtcReadThread, NULL);
  
  // Create the thread used to print the RTC data every seconds
  chThdCreateStatic(waRtcPrintThread, sizeof(waRtcPrintThread), NORMALPRIO, 
    RtcPrintThread, NULL);
  
  while (true){
    chprintf(chp, "\n\r ChibiOS: heartbeat = %d", heartbeat++);
    chThdSleepMilliseconds(5000);
  }
  return 0;
}

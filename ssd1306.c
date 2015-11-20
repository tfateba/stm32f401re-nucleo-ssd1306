/*
 * @file     ssd1306.c
 * @brief    Olcd library for the SSD1306 controler
 * @author   Theodore ATEBA
 * @version  1.0
 * @date     21 June 2015
 *
 */

//=============================================================================
// Include Files
//=============================================================================
    #include "ssd1306.h"
    #include "font.h"

//=============================================================================
// Global variables, I2C TX and RX buffers, I2C and Serial Configurations
//=============================================================================

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

    extern BaseSequentialStream* chp;

    #define SCREEN_HEIGHT		64
    #define SCREEN_WIDTH		128
    //#define INITIAL_CONTRAST	0xFF

    // I2C interface #1chibios arduino uno port
    static const I2CConfig i2cConf = {
        OPMODE_I2C,
        400000,
        FAST_DUTY_CYCLE_2,
    };

    // The memory buffer for the display
    uint8_t gdisp_buffer[SCREEN_HEIGHT * SCREEN_WIDTH / 8];
    int pBuff = 0;
    
    // Parametres de communication I2c
    uint8_t ssd_addr = 0x3C;
    uint8_t control = 0x00;
    uint8_t data = 0x40;
    
    // Box pattern
    uint8_t pattern1[] = {
        0x00,0x7E,0x42,0x42,0x42,0x42,0x7E,0x00,
        0x00,0x7E,0x42,0x42,0x42,0x42,0x7E,0x00
    };

    // Cross weave pattern
    uint8_t pattern2[] = {
        0x81,0x42,0x24,0x18,0x18,0x24,0x42,0x81,
        0x81,0x42,0x24,0x18,0x18,0x24,0x42,0x81
    };

    // Box pattern
    uint8_t pattern3[] = {
        0x00, 0x00, 0x1E, 0x30, 0x3E, 0x33, 0x6E, 0x00,
        0x0C, 0x1E, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x00
    };

    // Current position of the cursor
    uint8_t m_col = 0;
    uint8_t m_row = 0;

//=============================================================================
// Functions
//=============================================================================

/*
 * @fn      ssdReset
 * @brief   reset of the LCD
 */
void ssdReset(void){
	// Reset sequence
	palClearPad(RESET_PORT, SSD_RESET_PIN);
	chThdSleepMilliseconds(1);
	palSetPad(RESET_PORT, SSD_RESET_PIN);
    chThdSleepMilliseconds(1);
    writeCmd(SSD1306_DISPLAY_ON);
  
    // Wait the SEG/COM signal to be ON after 100ms
    chThdSleepMilliseconds(100);
}

/*
 * @fn          ssdSetMuxRatio
 * @brief       set Multiplexer ration TODO
 * @param[in]   ratio ration to set
 */
void ssdSetMuxRatio(uint8_t ratio){
	writeCmd(SSD1306_SET_MULTIPLEX);
	writeCmd(ratio & 0x3F);
}

/*
 * @fn          ssdSetDisplayOffset
 * @brief       Use to set the LCD display offset
 * @param[in]   offset the offset of the LCD
 */
void ssdSetDisplayOffset(uint8_t offset){
	writeCmd(SSD1306_SET_DISPLAY_OFFSET);
	writeCmd(offset & 0x3F);
}

/*
 * @fn          ssdSetDisplayStartLine
 * @brief       Use to set the line numbre
 * @param[in]   line the line to set
 */
void ssdSetDisplayStartLine(uint8_t line){
	writeCmd(SSD1306_SET_START_LINE | line);
}

/*
 * @fn          ssdSetSegmentRemap
 * @brief       set the segment remap, TODO
 * @param[in]   value segment remap value
 */
void ssdSetSegmentRemap(uint8_t value){
	writeCmd(value ? SSD1306_SEGREMAP+1 : SSD1306_SEGREMAP);
}

/*
 * @fn          ssdSetStartAdressParam
 * @brief       set start adress parameter
 * @param[in]   address address param
 */
void ssdSetStartAddressPam(uint8_t address){
	writeCmd(address & 0x0F);
	writeCmd((address << 4) & 0x0F);
}

/*
 * @fn          ssdSetMemoryAddressingMode
 * @brief       Set the memomry addressing mode
 * @param[in]   mode TODO, reconment
 */
void ssdSetMemoryAddressingMode(uint8_t mode){
	writeCmd(SSD1306_MEMORY_MODE);
	writeCmd(mode & 0x3);
}

/*
 * @fn          ssdSetComOutputScanDirection
 * @brief       set com output direction
 * @param[in]   direction direction of the com output
 */
void ssdSetComOutputScanDirection(uint8_t direction){
	writeCmd(direction ? SSD1306_COM_SCANDEC : SSD1306_COM_SCANINC);   
}

/*
 * @fn      ssdSetComPinsHardwareCinfiguration
 * @brief   Set com pins hardware configuration
 */
void ssdSetComPinsHardwareConfiguration(void){
	writeCmd(SSD1306_SET_COM_PINS);
	writeCmd(0x02);
}

/*
 * @fn          ssdSetContrastControl
 * @brief       set the LCD contrast
 * @param[in]   contrast contrast to set on the LCD
 */
void ssdSetContrastControl(uint8_t contrast){
	writeCmd(SSD1306_SET_CONTRAST);
	writeCmd(contrast);
}

/*
 * @fn      ssdDisableEntireDisplayOn
 * @brief   disable the entire display
 */
void ssdDisableEntireDisplayOn(void){
	writeCmd(0xA4);// TODO remplacer la valeur par un define
}

/*
 * @fn      ssdSetNormalDisplay
 * @brief   Set the normal display
 */
void ssdSetNormalDisplay(void){
	writeCmd(0xA6);// TODO Remplacer la valeur pa un define
}

/*
 * @fn      ssdSetOscFrequency
 * @brief   set the LCD frequecy
 */
void ssdSetOscFrequency(void){
	writeCmd(0xD5);// TODO
	writeCmd(0x80);// TODO Remplacer la valeur par un define
}

/*
 * @fn          ssdEnableChargePumRegulator
 * @brief       Enable the charge pum regulator of the LCD
 * @param[in]   enable use to enable or disable the pum regulator
 */
void ssdEnableChargePumpRegulator(bool enable){
	writeCmd(SSD1306_ENABLE_CHARGE_PUMP);
	writeCmd(enable ? 0x14: 0x10);
}

/*
 * @fn          ssdInvertDisplay
 * @brief       Used to invert the display
 * @param[in]   state invert the display if state is true.
 */
void ssdInvertedDisplay(bool state){
	writeCmd(state ? SSD1306_INVERT_DISPLAY : SSD1306_NORMAL_DISPLAY);
}

/*
 * @fn      ssdDisplayOff
 * @brief   turn off the display
 */
void ssdDisplayOff(void){
	writeCmd(SSD1306_DISPLAY_OFF);
}

/*
 * @fn      ssdDisplayOn
 * @brief   turn on the display
 */
void ssdDisplayOn(void){
	writeCmd(SSD1306_DISPLAY_ON); 
}

/*
 * @fn      ssdDisplayAllOn
 * @brief   set All the LCD Pixel ON TODO: Verify this comment :)
 */
void ssdDisplayAllOn(void){
	writeCmd(SSD1306_DISPLAY_ALL_ON);
}

/*
 * @fn          ssdSetColumnRange
 * @brief       set the culumn range
 * @param[in]   culumn range
 */
void ssdSetColumnRange(uint8_t culumn){
	writeCmd(SSD1306_HV_COLUMN_ADDRESS);
	writeCmd(culumn);
}

/*
 * @fn          ssdSetPageRange
 * @brief       Set the page Range
 * @param[in]   page the page to set
 */
void ssdSetPageRange(uint8_t page){
	writeCmd(0x22);
	writeCmd(page);
}

/*
 * @fn      display
 * @brief   Display the buffer on the LCD
 */
void display(void){
	uint8_t i;
	uint8_t GDDRAM[129]= {0x00};

	GDDRAM[0] = 0x40;

	for(i = 0; i< 17; i++){
		GDDRAM[i+1] = 0x81;
	}
	i2cAcquireBus (&I2CD1);
	i2cMasterTransmitTimeout (&I2CD1, ssd_addr, GDDRAM, 17, 
			NULL, 0, MS2ST(10));
	i2cReleaseBus (&I2CD1);
}

/*
 * @fn      ssdDisplayBox
 * @brief   Display boxes on the LCD
 */
void ssdDisplayBox(void){
	uint8_t i;
	uint8_t GDDRAM[17] = {0x00};

	GDDRAM[0] = 0x40;

	for(i = 0; i < 16; i++){
		GDDRAM[i+1] = pattern1[i];
	}
	i2cAcquireBus (&I2CD1);
	i2cMasterTransmitTimeout (&I2CD1, ssd_addr, GDDRAM, 17, 
	NULL, 0, MS2ST(10));
	i2cReleaseBus (&I2CD1);
}

/*
 * @fn      ssdDisplayCross
 * @brief   Display cross on the LCD
 */
void ssdDisplayCross(void){
	uint8_t i;
	uint8_t GDDRAM[17] = {0x00};

	GDDRAM[0] = 0x40;

	for(i = 0; i < 16; i++){
		GDDRAM[i+1] = pattern2[i];
	}
	i2cAcquireBus (&I2CD1);
	i2cMasterTransmitTimeout (&I2CD1, ssd_addr, GDDRAM, 17, 
	NULL, 0, MS2ST(10));
	i2cReleaseBus (&I2CD1);
}

/*
 * @fn      ssdDisplayZerochibios arduino uno port
 * @brief   Display zero on 
 */
void ssdDisplayZero(void){
	uint8_t i;
	uint8_t GDDRAM[17] = {0x00};

	GDDRAM[0] = 0x40;

	for(i = 0; i < 16; i++){
		GDDRAM[i+1] = pattern3[i];
	}
	i2cAcquireBus (&I2CD1);
	i2cMasterTransmitTimeout (&I2CD1, ssd_addr, GDDRAM, 17, 
	NULL, 0, MS2ST(10));
	i2cReleaseBus (&I2CD1);
}

/*
 * @fn          ssdSetCursor
 * @brief       set tjej LCD cursor
 * @param[in]   culumn cursor vertical position
 * @param[in]	line cursor horizontal position
 */
void ssdSetCursor(uint8_t column, uint8_t line){
    m_col = column;
    m_row = line;
    writeCmd(0xB0 + m_row);//set page address
    writeCmd(m_col & 0xf);//set lower column address
    writeCmd(0x10 | (m_col >> 4));//set higher column address
}

/*
 * @fn      ssdDisplayUpdate
 * @brief   Update the LCD display state
 */
void ssdDisplayUpdate(void){
	writeCmd(0xA4);
}

/*
 * @fn      olsdInit
 * @brief   Init the LCD
 */
void oledInit(void){
	
	// Tell the SSD1306 that a command stream is incoming
	writeCmd(SSD1306_CONTROL_BYTE_CMD_STREAM);

	// Follow instructions on pg.64 of the dataSheet for software configuration of the SSD1306
	// Turn the Display OFF
    writeCmd(SSD1306_CMD_DISPLAY_OFF);

	// Set mux ration tp select max number of rows - 64
    writeCmd(SSD1306_CMD_SET_MUX_RATIO);
	writeCmd(0x3F);

	// Set the display offset to 0
    writeCmd(SSD1306_CMD_SET_DISPLAY_OFFSET);
	writeCmd(0x00);

	// Display start line to 0
    writeCmd(SSD1306_CMD_SET_DISPLAY_START_LINE);	

	// Mirror the x-axis. In case you set it up such that the pins are north.
	// Wire.write(0xA0); - in case pins are south - default
    writeCmd(SSD1306_CMD_SET_SEGMENT_REMAP);

	// Mirror the y-axis. In case you set it up such that the pins are north.
	// Wire.write(0xC0); - in case pins are south - default
    writeCmd(SSD1306_CMD_SET_COM_SCAN_MODE);

	// Default - alternate COM pin map
    writeCmd(SSD1306_CMD_SET_COM_PIN_MAP);
    writeCmd(0x12);

	// set contrast
    writeCmd(SSD1306_CMD_SET_CONTRAST);
    writeCmd(0x7F);

	// Set display to enable rendering from GDDRAM (Graphic Display Data RAM)
    writeCmd(SSD1306_CMD_DISPLAY_RAM);

	// Normal mode!
    writeCmd(SSD1306_CMD_DISPLAY_NORMAL);

	// Default oscillator clock
    writeCmd(SSD1306_CMD_SET_DISPLAY_CLK_DIV);
    writeCmd(0x00);

	// Enable the charge pump
    writeCmd(SSD1306_CMD_SET_CHARGE_PUMP);
    writeCmd(0x14);

	// Set precharge cycles to high cap type
    writeCmd(SSD1306_CMD_SET_PRECHARGE);
	writeCmd(0x22);

	// Set the V_COMH deselect volatage to max
    writeCmd(SSD1306_CMD_SET_VCOMH_DESELCT);
	writeCmd(0x30);

	// Horizonatal addressing mode - same as the KS108 GLCD
    writeCmd(SSD1306_CMD_SET_MEMORY_ADDR_MODE);
	writeCmd(0x00);

	// Turn the Display ON
    writeCmd(SSD1306_CMD_DISPLAY_ON);
}

/*
 * @fn          ssdSetColumnAddressHvam
 * @brief       set column address
 * @param[in]   start 
 * @param[in]	end
 */
void ssdSetColumnAddressHvam(uint8_t start, uint8_t end){
	writeCmd(SSD1306_HV_COLUMN_ADDRESS);
	writeCmd(start & 0x7F);
	writeCmd(end & 0x7F);
}

/*
 * @fn          ssdSetAddressHvam
 * @brief       set page
 * @param[in]   start
 * @param[in]	end
 */
void ssdSetPageAddressHvam(uint8_t start, uint8_t end){
	writeCmd(SSD1306_HV_PAGE_ADDRESS);
	writeCmd(start & 0x07);
	writeCmd(end & 0x07);
}

/*
 * @fn          ssdSetPageStartParam
 * @brief       set page start param
 * @param[in]   address 
 */
void ssdSetPageStartPam(uint8_t address){
	writeCmd(SSD1306_PAM_PAGE_START | (address & 0x07));
}

/*
 * @fn          ssdSetDisplayClockRatioAndFrequency
 * @brief       set the display clock ration and frequency
 * @param[in]   ratio
 * @param[in]	frequency
 */
void ssdSetDisplayClockRatioAndFrequency(uint8_t ratio, uint8_t frequency){
	writeCmd(SSD1306_SET_DISPLAY_CLOCK_DIV);
	writeCmd((ratio & 0x0F) | ((frequency & 0x0F) << 4));
}

/*
 * @fn          ssdSetPrechargePeriod
 * @brief       set precharge period
 * @param[in]   phase1
 * @param[in]	phase2
 */
void ssdSetPrechargePeriod(uint8_t phase1, uint8_t phase2){
	writeCmd(SSD1306_SET_PRECHARGE);
	writeCmd((phase1 & 0x0F) | ((phase2 & 0x0F ) << 4));
}

/*
 * @fn          ssdSetVcomhDeselectLevel
 * @brief       set the detection level
 * @param[in]   level detection level
 */
void ssdSetVcomhDeselectLevel(uint8_t level){
	writeCmd(SSD1306_SET_VCOMDETECT);
	writeCmd((level & 0x03) << 4);
}

/*
 * @fn          ssdSetChargePump
 * @brief       use to enable or disable the charge pump
 * @param[in]   enable enable the charge pum
 */
void ssdSetChargePump(uint8_t enable){
	writeCmd(SSD1306_ENABLE_CHARGE_PUMP);
	writeCmd(enable ? 0x14 : 0x10);
}

/*
 * @fn          ssdDrawPixel
 * @brief       Draw a pixel on the LCD
 * @param[in]   x horizontal position
 * @param[in]	y vertical position
 * @param[in]   color pixel color, black or white
 */
void ssdDrawPixel(uint16_t x, uint16_t y, uint8_t color){
	if (color == SSD1306_WHITE)
		gdisp_buffer[x+ (y/8)*SCREEN_WIDTH] |=  (1<<y%8);
	else
		gdisp_buffer[x+ (y/8)*SCREEN_WIDTH] &= ~(1<<y%8);	
}

/*
 * @fn      ssdDisplay
 * @brief   Display the buffer content on the LCD
 */
void ssdDisplay(void){
	int i, j;
	ssdSetDisplayStartLine(0);

	// We're sending half a line in one X-mission.
	uint8_t command[SCREEN_WIDTH/2], 
			cmdLength = sizeof(command)/sizeof(command[0]),
			parts = SCREEN_WIDTH/cmdLength;

	for(i=0; i<SCREEN_HEIGHT/8; i++){
		for(j = 0; j<parts; j++){
			memmove(command, &gdisp_buffer[i*SCREEN_WIDTH + j*cmdLength], cmdLength);
			writeData(command, cmdLength);
		}
	}
}

/*
 * @fn      acquireBus
 * @brief   acquire teh i2c bus
 */
void acquireBus(void){
	i2cAcquireBus(&I2CD1);
}

/*
 * @fn      releaseBus
 * @brief   Release the i2c bus
 */
void releaseBus(void){
	i2cReleaseBus(&I2CD1);
}

/*
 * @fn          writeCmd
 * @brief       write a command to the i2c bus for the lcd
 * @param[in]   cmd command to 
 */
void writeCmd(uint8_t cmd){
	uint8_t command[2];
	uint8_t txLength = sizeof(command)/sizeof(command[0]);
	uint8_t rxLength = 0;

	command[0] = 0x00; // Co = 0, D/C = 0
	command[1] = cmd;
	i2cAcquireBus (&I2CD1);
	i2cMasterTransmitTimeout(&I2CD1, ssd_addr, command, txLength, 
			NULL, rxLength, MS2ST(10));
	i2cReleaseBus (&I2CD1);
}

/*
 * @fn          writeData
 * @brief       write the data to the i2C bus for the lcd
 * @param[in]   data data to send to the lcd
 * @param[in]	length size of the data to send.
 */
void writeData(uint8_t* data, uint16_t length){
	uint8_t command[length+1],txLength = length+1,rxLength = 0;
	command[0] = 0x40; 		// Co = 0, D/C = 1
	memmove(&command[1], data, length);

	i2cAcquireBus (&I2CD1);
	i2cMasterTransmitTimeout(&I2CD1, ssd_addr, command, txLength, 
			NULL, rxLength, MS2ST(10));
	i2cReleaseBus (&I2CD1);
}


/*
 * @fn			ssdCharFont
 * @brief		Function use to print a character to the LCD
 * @param[in]   ch character to print on the LCD
 * @param[in]   font Definchibios arduino uno porte font size to use whie printing data on the LCD
 */
void ssdCharFont(char ch, uint8_t font){
	uint8_t i;
	uint8_t c = ch - 32;	
	
	if(font == 2){
		for (i = 0; i < 6; i++){
			gdisp_buffer[i] = (ssd1306_6x8Font[c][i]);
		}
		writeData(gdisp_buffer, 6);
	}
	
	if(font == 1){
		for (i = 0; i < 5; i++){
			gdisp_buffer[i] = (ssd1306_5x8Font[c][i]);
		}
		writeData(gdisp_buffer, 5);
	}
}

/*
 * @fn			ssdString
 * @brief		Function use to print string on the LCD
 * @param[in]   string Data to print on the LCD
 * @param[in]	font Define font size to use whie printing data on the LCD
 */
void ssdString(char *string, uint8_t font){
	while (*string) {
        ssdCharFont(*string++, font);
	}
}

/*
 * @fn		oled_demo
 * @brief   Shows the demo on the LCD.
 */
void oled_demo(void){
	int i;

	ssdDrawPixel(0, 0, SSD1306_WHITE); // SSD1306_WHITE or SSD1306_BLACK
	ssdDisplay();
	chThdSleepMilliseconds(2000);

	for(i = 0; i <1024; i++)
    display();

	chThdSleepMilliseconds(2000);

	for(i =0; i <1024; i++)
		ssdDisplayBox();
  
	chThdSleepMilliseconds(2000);

	for(i =0; i <1024; i++)
		ssdDisplayCross();
  
	chThdSleepMilliseconds(2000);
	ssdDrawPixel(0, 0, SSD1306_WHITE); // SSD1306_WHITE or SSD1306_BLACK
	ssdDisplay();

	ssdSetCursor(0, 1);
	ssdString("BETA-WATCH", 2);

	ssdSetCursor(0, 3);
	ssdString("Date: 07 / 11 / 2015", 1);

	ssdSetCursor(0, 5);
	ssdString("Learn by doing", 1);

	ssdSetCursor(0, 7);
	ssdString("Made by Theo", 1);

	chThdSleepMilliseconds(2000);
}

/*
 * @fn		oled_demo
 * @brief   Shows the demo on the LCD.
 */
void oled_print_date(struct ds1307_t calendar){
 	int i;
	char asciiDay[2];
	char asciiMonth[2];
	char asciiYear[4];
	char asciiSeconds[2];
	char asciiMinutes[2];
	char asciiHours[2];

	itoa(calendar.date, asciiDay, 10);
	itoa(calendar.month, asciiMonth, 10);
	itoa(calendar.year, asciiYear, 10);
	itoa(calendar.seconds, asciiSeconds, 10);
	itoa(calendar.minutes, asciiMinutes, 10);
	itoa(calendar.hours, asciiHours, 10);

	ssdSetCursor(0, 3);
    for(i = 0; i < 26; i++)
 		ssdString(" ", 1);

	ssdSetCursor(0, 3);
	ssdString("Date: ", 1);
	ssdString(asciiDay, 1);
	ssdString(" / ", 1);
	ssdString(asciiMonth, 1);
	ssdString(" / ", 1);
	ssdString(asciiYear, 1);

	ssdSetCursor(0, 5);
    for(i = 0; i < 26; i++)
 		ssdString(" ", 1);

	ssdSetCursor(0, 5);
	ssdString("Time: ", 1);
	ssdString(asciiHours, 1);
	ssdString(" : ", 1);
	ssdString(asciiMinutes, 1);
	ssdString(" : ", 1);
	ssdString(asciiSeconds, 1);
}

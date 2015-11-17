//!*****************************************************************************
//! INCLUSIONS
//!*****************************************************************************

// These are included by the system and/or Anil.
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

// Lee added these
#include <inttypes.h>
#include <stdlib.h>
#include "buttons.h"
#include "stdio.h"
#include "string.h"
#include "uart0stdio.h"		// Console
#include "uart1stdio.h"		// GSM
#include "driverlib/adc.h"
#include "driverlib/eeprom.h"
#include "driverlib/i2c.h"
#include "driverlib/ssi.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "inc/hw_gpio.h"
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "inc/hw_ssi.h"

//!*****************************************************************************
//! DEFINITIONS
//!*****************************************************************************

// RGB LED pin definitions
#define RD_LED GPIO_PIN_1
#define BL_LED GPIO_PIN_2
#define GN_LED GPIO_PIN_3
#define LED_MAP RD_LED | BL_LED | GN_LED

// For driving buttons
#define APP_SYSTICKS_PER_SEC		32
#define APP_BUTTON_POLL_DIVIDER		8
#define APP_HIB_BUTTON_DEBOUNCE		(APP_SYSTICKS_PER_SEC * 3)

// EEPROM start address
#define E2PROM_TEST_ADRES 0x0000

// For touchpad
#define K1			8
#define K2			4
#define K3			3
#define K4			9
#define K5			5
#define K6			2
#define K7			10
#define K8			6
#define K9			1
#define K0			7
#define KS			11
#define KP			0
#define TOU_THRESH 	0x0F	// touch threshold
#define REL_THRESH 	0x09	// release threshold

// For LCD screen
#define XPIXEL			102
#define YPIXEL			64
#define XMAX			XPIXEL-1
#define YMAX			YPIXEL-1
#define LCD_CMD			0
#define LCD_DATA		1
#define NORMAL			1
#define INVERSE			2

// Interrupt handlers
void UARTIntHandler0(void);
void UARTIntHandler1(void);
void MPR121IntHandler(void);
void KeyIdleTimer1IntHandler(void);
void KeyPressTimer0IntHandler(void);
void SysTickIntHandler(void);

// Functions
void blinkLED (int blinkCount, int blinkSpeed, uint8_t LEDcolor);
/// -- Relay handling
void relayOn(int relayNum);
void relayOff(int relayNum);
void relayToggle(int relayNum);
/// -- GSM module
bool GSMcheckPower (void);
void GSMtogglePower( void );
int GSMgetResponse(void);
void GSMcheckTime(void);
void GSMprocessMessage( int activeMsg );
void GSMsendSMS(char *destNbr, char *msgBody);
/// -- Keypad
void initI2C(void);
uint32_t I2Creceive(uint32_t slave_addr, uint8_t reg);
void I2Csend(uint8_t slave_addr, uint8_t num_of_args, ...);
void initMPR121(void);
void MPR121toggleLock(void);
/// -- LCD display
void initSSI3(void);
void SSI3sendByte(uint32_t theData);
void initLCD(void);
void LCDsend(uint32_t theData, uint8_t cmdByte);
void LCDsetAddress(uint32_t page, uint32_t column);
void LCDclear(uint8_t xs, uint8_t ys, uint8_t xe, uint8_t ye);
void LCDfill(uint8_t xs, uint8_t ys, uint8_t xe, uint8_t ye);
void LCDchar(uint8_t row, uint8_t col, uint16_t f, uint8_t style);
void LCDstring(uint8_t row, uint8_t col, char *word, uint8_t style);
void LCDline(uint8_t xs, uint8_t ys, uint8_t xe, uint8_t ye);
void LCDrect(uint8_t xs, uint8_t ys, uint8_t xe, uint8_t ye);

//!*****************************************************************************
//! GLOBAL VARIABLES
//!*****************************************************************************

// TESTING VARIABLES: use these to disable code segments during testing
bool testGSM =		0;				// Turn on the GSM during boot
bool testEEPROM =	0;				// Store/retrieve ontime from EEPROM 
									// (requires testGSM)
bool testRelay =	0;				// Cycle the relays on/off (exclude 1 and 4)
bool testDelete =	0;				// Delete messages during processing
bool testNotify =	0;				// Text the controller when coming on-line
bool testI2C =		1;				// I2C testing area
bool testADC =		1;				// Analog to Digital converter

// Identifying constants
char ctrlID[11] = "3158078555";		// Phone number of the controller
char boardID[5] = "0001";			// ID of this board

// Status holders across functions
bool GSMoff =		true;			// Flag to see if the GSM module is on/off
bool talkMode =		true;			// Allow user to interface directly with GSM
int buttonLEDhold;					// Holds status of RGB LED
bool relayStatus[4];				// store the status of the relays
int msgCount =		0;				// Hold the int value w/count of new messages
char pressedKey;					// Keypad: store which key was last pressed
bool keysUnlocked =	false;			// For locking the keypad
uint16_t touchedMap;				// Map of key status

// Used by UART interrupt handlers
unsigned char var;					// Incoming UART character
unsigned char ptr[10000];			// Array for storing incoming UART characters
unsigned long i;					// UART character pointer.
unsigned long ulStatus0,ulStatus1;	// To hold the interrupt status

// Data from most recent incoming message stored here
char responseLine[10][75];			// Use to store UART inputs
// TO DO: do this without pointers, or make sure they're used correctly
char *msgContent =	NULL;			// Message content holder
char *msgSender =	NULL;			// Message sender
char *msgDate =		NULL;			// Message date
char *msgTime =		NULL;			// Message time

// GSMcheckTime function writes to these variables for main program access
int hh,mm,ss,DD,MM,YY,zz;			// Hour, minute, second, day, month, year, 
									// time zone (offset from GMT in 1/4 hours)
char fullTimestamp[23] = "\0";		// Complete timestamp

// For EEPROM
// TO DO: Generalize these variables for EEPROM use
uint32_t esize,eblock;				// EEPROM block size and block count
struct eprom_timestamp				// Struct for writing timestamps to EEPROM
{
	int v1;
	int v2;
	int v3;
	int v4;
	int v5;
	int v6;
	int v7;
}; 
struct eprom_timestamp eprom_readtime =  {0,0,0,0,0,0,0}; /* Read struct */

// For keypad
int curCol=XPIXEL;						// Current column in LCD

// For LCD: Font lookup table
static const uint8_t FONT6x8[] = {
    /* 6x8 font, each line is a character each byte is a one pixel wide column
     * of that character. MSB is the top pixel of the column, LSB is the bottom
     * pixel of the column. 0 = pixel off. 1 = pixel on. */

		0,  0,  0,  0,  0,  0,
		0,  0, 95,  0,  0,  0,
		0,  7,  0,  7,  0,  0,
	   20,127, 20,127, 20,  0,
	   36, 42,127, 42, 18,  0,
	   35, 19,  8,100, 98,  0,
	   54, 73, 86, 32, 80,  0,
		0,  8,  7,  3,  0,  0,
		0, 28, 34, 65,  0,  0,
		0, 65, 34, 28,  0,  0,
	   42, 28,127, 28, 42,  0,
		8,  8, 62,  8,  8,  0,
		0,128,112, 48,  0,  0,
		8,  8,  8,  8,  8,  0,
		0,  0, 96, 96,  0,  0,
	   32, 16,  8,  4,  2,  0,
	   62, 81, 73, 69, 62,  0,
		0, 66,127, 64,  0,  0,
	   66, 97, 81, 73, 70,  0,
	   33, 65, 73, 77, 51,  0,
	   24, 20, 18,127, 16,  0,
	   39, 69, 69, 69, 57,  0,
	   60, 74, 73, 73, 48,  0,
	   65, 33, 17,  9,  7,  0,
	   54, 73, 73, 73, 54,  0,
		6, 73, 73, 41, 30,  0,
		0,  0, 20,  0,  0,  0,
		0, 64, 52,  0,  0,  0,
		0,  8, 20, 34, 65,  0,
	   20, 20, 20, 20, 20,  0,
		0, 65, 34, 20,  8,  0,
		2,  1, 81,  9,  6,  0,
	   62, 65, 93, 89, 78,  0,
	  124, 18, 17, 18,124,  0,
	  127, 73, 73, 73, 54,  0,
	   62, 65, 65, 65, 34,  0,
	  127, 65, 65, 65, 62,  0,
	  127, 73, 73, 73, 65,  0,
	  127,  9,  9,  9,  1,  0,
	   62, 65, 73, 73,122,  0,
	  127,  8,  8,  8,127,  0,
		0, 65,127, 65,  0,  0,
	   32, 64, 65, 63,  1,  0,
	  127,  8, 20, 34, 65,  0,
	  127, 64, 64, 64, 64,  0,
	  127,  2, 28,  2,127,  0,
	  127,  4,  8, 16,127,  0,
	   62, 65, 65, 65, 62,  0,
	  127,  9,  9,  9,  6,  0,
	   62, 65, 81, 33, 94,  0,
	  127,  9, 25, 41, 70,  0,
	   38, 73, 73, 73, 50,  0,
		1,  1,127,  1,  1,  0,
	   63, 64, 64, 64, 63,  0,
	   31, 32, 64, 32, 31,  0,
	   63, 64, 56, 64, 63,  0,
	   99, 20,  8, 20, 99,  0,
		3,  4,120,  4,  3,  0,
	   97, 81, 73, 69, 67,  0,
		0,127, 65, 65, 65,  0,
		2,  4,  8, 16, 32,  0,
		0, 65, 65, 65,127,  0,
		4,  2,  1,  2,  4,  0,
	   64, 64, 64, 64, 64,  0,
		0,  3,  7,  8,  0,  0,
	   32, 84, 84, 84,120,  0,
	  127, 40, 68, 68, 56,  0,
	   56, 68, 68, 68, 40,  0,
	   56, 68, 68, 40,127,  0,
	   56, 84, 84, 84, 24,  0,
		0,  8,126,  9,  2,  0,
	   24,164,164,164,124,  0,
	  127,  8,  4,  4,120,  0,
		0, 68,125, 64,  0,  0,
	   32, 64, 64, 61,  0,  0,
	  127, 16, 40, 68,  0,  0,
		0, 65,127, 64,  0,  0,
	  124,  4,120,  4,120,  0,
	  124,  8,  4,  4,120,  0,
	   56, 68, 68, 68, 56,  0,
	  252, 24, 36, 36, 24,  0,
	   24, 36, 36, 24,252,  0,
	  124,  8,  4,  4,  8,  0,
	   72, 84, 84, 84, 36,  0,
		4,  4, 63, 68, 36,  0,
	   60, 64, 64, 32,124,  0,
	   28, 32, 64, 32, 28,  0,
	   60, 64, 48, 64, 60,  0,
	   68, 40, 16, 40, 68,  0,
	   76,144,144,144,124,  0,
	   68,100, 84, 76, 68,  0,
		0,  8, 54, 65,  0,  0,
		0,  0,119,  0,  0,  0,
		0, 65, 54,  8,  0,  0,
		2,  1,  2,  4,  2,  0,
	   60, 38, 35, 38, 60,  0,
	   30,161,161, 97, 18,  0,
	   58, 64, 64, 32,122,  0
};

//!*****************************************************************************
//! ERROR HANDLER
//!*****************************************************************************

// REQUIRED (included by Anil - not sure what it's for or if it's needed)
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

//!*****************************************************************************
//! INTERRUPT HANDLERS
//!*****************************************************************************

//*****************************************************************************
//
// UART0 - CONSOLE
//
//*****************************************************************************
void
UARTIntHandler0(void)
{
	// Get the interrupt status.
	ulStatus0 = ROM_UARTIntStatus(UART0_BASE, true);
	// Clear the asserted interrupts
	ROM_UARTIntClear(UART0_BASE, ulStatus0);
	// Loop while there are characters in the receive FIFO.
	while(ROM_UARTCharsAvail(UART0_BASE))
	{
		// Grab a character
		var = (unsigned char)ROM_UARTCharGetNonBlocking(UART0_BASE);
		// Hold it
		ptr[i] = var;
		// Mirror it to GSM
		ROM_UARTCharPutNonBlocking(UART1_BASE, ptr[i]);
		// Proceed to next character
		i++;
	}
}

//*****************************************************************************
//
// UART1 - GSM
//
//*****************************************************************************
void
UARTIntHandler1(void)
{
	char *msgCountStr;					// Number of new messages
	static char g_cInput[128];			// String input to a UART
	
	// Get the interrupt status.
	ulStatus1 = ROM_UARTIntStatus(UART1_BASE, true);
	// Clear the asserted interrupts.
	ROM_UARTIntClear(UART1_BASE, ulStatus1);
	
	if ( GSMoff ) { GSMoff = false; }	// Interrupt trigger means GSM is on.
	// Loop while there are characters in the receive FIFO.
	while(ROM_UARTCharsAvail(UART1_BASE)){
		// Grab a character
		var = (unsigned char)ROM_UARTCharGetNonBlocking(UART1_BASE);
		// Hold it
		ptr[i] = var;
		// Mirror to console in talk mode...
		if (talkMode){ROM_UARTCharPutNonBlocking(UART0_BASE, ptr[i]);}
		// ...or else see if it's a message notification, like +CMTI: "SM",12
		else {
			if(ptr[i-3] == 'C' && ptr[i-2] == 'M' && ptr[i-1] == 'T'&& ptr[i] == 'I'){
				UART1gets(g_cInput,sizeof(g_cInput));		// Grab everything...
				msgCountStr = strtok(g_cInput,"\n");		// ...stop after newline.
				strncpy(msgCountStr,msgCountStr+7,3);		// Parse for count...
				msgCountStr[3]='\0';
				sscanf(msgCountStr, "%d", &msgCount);		// ... convert to integer
				
				// Tell the user
				UART0printf("\n\r>>> %u NEW MESSAGES",msgCount);
			}
		}
		// Proceed to next character
		i++;
	}
}

//*****************************************************************************
//
// MPR121 TOUCHPAD INTERRUPT HANDLER 
// Interrupt is active low on PC7
//
//*****************************************************************************
void
MPR121IntHandler(void)
{
	int touchNumber = 0;
	int j;
	uint32_t touchedLSB, touchedMSB;
	
	// Get the status of the electrodes
	touchedLSB = I2Creceive(0x5A,0x00);
	touchedMSB = I2Creceive(0x5A,0x01);
	touchedMap = ((touchedMSB << 8) | touchedLSB);

	// Check how many electrodes were pressed
	for (j=0; j<12; j++) { if ((touchedMap & (1<<j))) { touchNumber++; } }
	// If one electrode was pressed, register it
	if (touchNumber == 1) {
		if (touchedMap & (1<<K1)){ 
			pressedKey = '1';
			if (keysUnlocked ) { relayToggle(1); }	// Toggle relay 1
		}
		else if (touchedMap & (1<<K2)){ 
			pressedKey = '2';
			if (keysUnlocked ) { relayToggle(2); }	// Toggle relay 2
		}
		else if (touchedMap & (1<<K3)){ 
			pressedKey = '3'; 
			if (keysUnlocked ) { relayToggle(3); }	// Toggle relay 3
		}
		else if (touchedMap & (1<<K4)){ 
			pressedKey = '4'; 
			if (keysUnlocked ) { relayToggle(4); }	// Toggle relay 4
		}
		else if (touchedMap & (1<<K5)) { pressedKey = '5'; }
		else if (touchedMap & (1<<K6)) { pressedKey = '6'; }
		else if (touchedMap & (1<<K7)) { pressedKey = '7'; }
		else if (touchedMap & (1<<K8)) { pressedKey = '8'; }
		else if (touchedMap & (1<<K9)) { pressedKey = '9'; }
		else if (touchedMap & (1<<K0)) { pressedKey = '0'; }
		else if (touchedMap & (1<<KS)) { pressedKey = '*'; }
		else if (touchedMap & (1<<KP)) { 
			pressedKey = '#';
			// Start timer 0A when user touches "#," to see if they're
			// trying to lock or unlock
			ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet()* 2);
			ROM_TimerEnable(TIMER0_BASE, TIMER_A);
		}
		if (keysUnlocked) { 
			// If the keys are unlocked, print the touched key to console...
			UART0printf("\n\r> %c",pressedKey); 
			curCol+=6;
			if (curCol>XMAX) { curCol = 0; }
			LCDchar(4,curCol,pressedKey, NORMAL);
			LCDchar(4,curCol+6,' ', NORMAL);
			// ... and start timer 1A, the idle keypad timer.
			ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, ROM_SysCtlClockGet()* 15);
			ROM_TimerEnable(TIMER1_BASE, TIMER_A);
		}
	}
	// One electrode released - do nothing
	else if (touchNumber == 0) {}
	// More than one button is pressed - do nothing
	else {}
	
	// Clear the asserted interrupts
	GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_7);
}

//*****************************************************************************
//
// KEYPAD IDLE TIMER INTERRUPT HANDLER
//
//*****************************************************************************
void
KeyIdleTimer1IntHandler(void)
{
    // Clear the timer interrupt.
	ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	
	// Disable the timer
	ROM_TimerDisable(TIMER1_BASE, TIMER_A);

	// The timer is up! Lock the keypad
	if (keysUnlocked){ MPR121toggleLock(); }
}

//*****************************************************************************
//
// KEYPRESS TIMER INTERRUPT HANDLER
//
//*****************************************************************************
void
KeyPressTimer0IntHandler(void)
{
    // Clear the timer interrupt.
	ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	
	// Disable the timer
	ROM_TimerDisable(TIMER0_BASE, TIMER_A);

	// The timer is up! If # is still being pressed, toggle keylock.
	if (touchedMap & (1<<KP)) { 
		if (keysUnlocked){ MPR121toggleLock(); }
		else { 
			MPR121toggleLock();
			// Also, restart the idle timer
			ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, ROM_SysCtlClockGet()* 15);
			ROM_TimerEnable(TIMER1_BASE, TIMER_A);
		}
	}
}

//*****************************************************************************
//
// BUTTON INTERRUPT HANDLER
// This only handles debug situations for now.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
	static uint32_t ui32TickCounter;
	uint32_t ui32Buttons;
	static volatile uint32_t ui32HibMMdeEntryCount;
	
	// Get the button state
	ui32Buttons = ButtonsPoll(0,0);
    ui32TickCounter++;

    switch(ui32Buttons & ALL_BUTTONS)
    {
    case LEFT_BUTTON:	// TOGGLE "TALK TO GSM" MODE
        // Check if the button has been held int32_t enough to act
        if((ui32TickCounter % APP_BUTTON_POLL_DIVIDER) == 0)
        {
			// Switch modes depending on current state
			if (talkMode == false){
				// Get LED status and store. Turn on blue LED.
				buttonLEDhold = GPIOPinRead(GPIO_PORTF_BASE, LED_MAP);
				GPIOPinWrite(GPIO_PORTF_BASE, LED_MAP, 0);
				GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, BL_LED);
				UART0printf("\n\r> [Left button] Entering talk to GSM mode. Press left button to end.");
				talkMode = true;
			}
			else{
				UART0printf("\n\r> [Left button] Returning to main program.");
				// Restore LED status
				GPIOPinWrite(GPIO_PORTF_BASE, LED_MAP, buttonLEDhold);
				talkMode = false;
			}
        }
        break;

    case RIGHT_BUTTON:	// TOGGLE POWER TO GSM MODULE
        // Check if the button has been held int32_t enough to act
        if((ui32TickCounter % APP_BUTTON_POLL_DIVIDER) == 0)
        {
			blinkLED(3,2,RD_LED);
			UART0printf("\n\r> [Right button] Cycling power to GSM.");
			GSMtogglePower();
        }
        break;

    case ALL_BUTTONS:	// DO NOTHING
        // Both buttons for longer than debounce time
        if(ui32HibMMdeEntryCount < APP_HIB_BUTTON_DEBOUNCE)
        {
			UART0printf("\n\r> [Both buttons] No action defined.");
        }
        break;

    default:
        break;
    }
}

//!*****************************************************************************
//! FUNCTIONS
//!*****************************************************************************

//*****************************************************************************
//
// INITIATE ADC
// PE0 (external D0)
//
//*****************************************************************************
void
ADCinit(void)
{
	// Set PE0 (External D0) as the ADC pin
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);
	
	// Enable sample sequence 3 with a processor signal trigger.  Sequence 3
	// will do a single sample when the processor sends a signal to start the
	// conversion.  Each ADC module has 4 programmable sequences, sequence 0
	// to sequence 3.  This example is arbitrarily using sequence 3.
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
	
	// Configure step 0 on sequence 3.  Sample channel 3 (ADC_CTL_CH3) in
	// single-ended mode (default) and configure the interrupt flag
	// (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
	// that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
	// 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
	// sequence 0 has 8 programmable steps.  Since we are only doing a single
	// conversion using sequence 3 we will only configure step 0.
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
	
	// Enable sample sequence 3
	ADCSequenceEnable(ADC0_BASE, 3);
	
	// Clear interrupt flag before starting
	ADCIntClear(ADC0_BASE, 3);
}

//*****************************************************************************
//
// BLINK LED
// blinkSpeed in 1/10 of a second (roughly)
//
//*****************************************************************************
void 
blinkLED (int blinkCount, int blinkSpeed, uint8_t LEDcolor)
{
	int m;
	int blinkHold;
	volatile uint32_t blinkLoop;
	
	// Store current LED status, then turn them all off
	blinkHold = GPIOPinRead(GPIO_PORTF_BASE, LED_MAP);
	GPIOPinWrite(GPIO_PORTF_BASE, LED_MAP, 0);
	blinkSpeed = blinkSpeed*100000;

	for( m=blinkCount; m>=0; m--)
	{
		// Turn on the LED.
		GPIOPinWrite(GPIO_PORTF_BASE, LEDcolor, LEDcolor);
		
		// Delay for a bit.
		for(blinkLoop = 0; blinkLoop < blinkSpeed; blinkLoop++){}
		
		// Turn off the LED.
		GPIOPinWrite(GPIO_PORTF_BASE, LED_MAP, 0);

        // Delay for a bit.
        for(blinkLoop = 0; blinkLoop < blinkSpeed; blinkLoop++){}
    }
	
	// Restore prior LED status
	GPIOPinWrite(GPIO_PORTF_BASE, LED_MAP, blinkHold);
}

//*****************************************************************************
//
// TURN ON RELAY(S)
// 0 turns on all
//
//*****************************************************************************
void 
relayOn(int relayNum)
{
    if (relayNum == 1 || relayNum == 0){
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);	// Rel1
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0);			// Rel1N
		relayStatus[0] = 1;
		UART0printf("\n\r> RELAY 1 ON");
		LCDstring(0,0,"RELAY 1 ON ", NORMAL);
	}
	if (relayNum == 2 || relayNum == 0){
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_PIN_3);	// Rel2
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0);			// Rel2N
		relayStatus[1] = 1;
		UART0printf("\n\r> RELAY 2 ON");
		LCDstring(1,0,"RELAY 2 ON ", NORMAL);
	}
	if (relayNum == 3 || relayNum == 0){
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);	// Rel3
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0);			// Rel3N
		relayStatus[2] = 1;
		UART0printf("\n\r> RELAY 3 ON");
		LCDstring(2,0,"RELAY 3 ON ", NORMAL);
	}
	if (relayNum == 4 || relayNum == 0){
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);	// Rel4
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);			// Rel4N
		relayStatus[3] = 1;
		UART0printf("\n\r> RELAY 4 ON");
		LCDstring(3,0,"RELAY 4 ON ", NORMAL);
	}
}

//*****************************************************************************
//
// TURN OFF RELAY(S)
// 0 turns off all
//
//*****************************************************************************
void 
relayOff(int relayNum)
{
    if (relayNum == 1 || relayNum == 0){
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);			// Rel1
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);	// Rel1N
		relayStatus[0] = 0;
		UART0printf("\n\r> RELAY 1 OFF");
		LCDstring(0,0,"RELAY 1 OFF", INVERSE);
	}
	if (relayNum == 2 || relayNum == 0){
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 0);			// Rel2
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5);	// Rel2N
		relayStatus[1] = 0;
		UART0printf("\n\r> RELAY 2 OFF");
		LCDstring(1,0,"RELAY 2 OFF", INVERSE);
	}
	if (relayNum == 3 || relayNum == 0){
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);			// Rel3
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);	// Rel3N
		relayStatus[2] = 0;
		UART0printf("\n\r> RELAY 3 OFF");
		LCDstring(2,0,"RELAY 3 OFF", INVERSE);
	}
	if (relayNum == 4 || relayNum == 0){
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);			// Rel4
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);	// Rel4N
		relayStatus[3] = 0;
		UART0printf("\n\r> RELAY 4 OFF");
		LCDstring(3,0,"RELAY 4 OFF", INVERSE);
	}
}

//*****************************************************************************
//
// TOGGLE RELAY
// 0 toggles all
//
//*****************************************************************************
void 
relayToggle(int relayNum)
{
	if (relayNum == 0) {
		for ( relayNum=0; relayNum<5; relayNum++ ){
			if (relayStatus[relayNum-1]) { relayOff(relayNum); }
			else { relayOn(relayNum); }
		}
	}
	else {
		if (relayStatus[relayNum-1]) { relayOff(relayNum); }
		else { relayOn(relayNum); }
	}
}

//*****************************************************************************
//
// GSM - CHECK POWER
//
//*****************************************************************************
bool 
GSMcheckPower (void)
{
	volatile uint32_t ui32Loop;			// for time delay
	
	// Delay for a bit, then make sure the flag is ON (something seems to 
	// trigger the interrupt when I'm setting it up)
	for(ui32Loop = 0; ui32Loop < 990000; ui32Loop++){}
	GSMoff = true;
	
	// Then, send an AT to the GSM module - if it triggers the int, GSM is on
	UART1printf("AT\r");
	// Delay for a bit.
	for(ui32Loop = 0; ui32Loop < 900000; ui32Loop++){}
	
	// Return status
	if ( GSMoff ) { return false; }
	return true;
}

//*****************************************************************************
//
// GSM - TOGGLE POWER
//
//*****************************************************************************
void 
GSMtogglePower( void )
{     
	volatile uint32_t ui32Loop;			// for time delay
	
	//GSM_RESET is active low. Keep this on all the time
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0);
	for(ui32Loop = 0; ui32Loop < 900000; ui32Loop++){}
	UART0printf("\n\r--- (step 1 of 2) GSM reset asserted...");

	//PWRKEY should be 1 for at least 1 second to power on/off the SIM900 module
	//(This pulls down actual SIM900 PWRKEY)
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7);
	for(ui32Loop = 0; ui32Loop < 10000000; ui32Loop++){}	//3 second delay
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0);			//PWRKEY de-asserted
	UART0printf("\n\r--- (step 2 of 2) GSM power key toggled...");
	
	// This part doesn't work. Only ever says off.
	for(ui32Loop = 0; ui32Loop < 1000000; ui32Loop++){}
	if (GSMcheckPower()){ UART0printf("\n\r--- GSM power ON!"); }
	else { UART0printf("\n\r--- GSM power OFF!"); }
}

//*****************************************************************************
//
// GET DATE/TIME FROM GSM MODULE 
// Used to fail when run right after GSM power up, but it seems to be working now.
//
//*****************************************************************************
void 
GSMcheckTime(void)
{
	int k;						// Count of lines from GSM response
	int j = 1;					// Keep track of which line we're checking
	
	UART1printf("AT+CCLK?\r");	// Ask GSM module for the time
	k = GSMgetResponse();		// Store to responseLine[] array, get line count
		
	while ( j < k-1 ){
		// Find the time in the lines we get back
		if ( strstr(responseLine[j],"+CCLK: \"") != '\0' ){
			strncpy(fullTimestamp,responseLine[j]+8,20);
			// Split to individual integers
			if (strstr(fullTimestamp,"+")){			// Case for positive offset from GMT
				sscanf(fullTimestamp,"%d/%d/%d,%d:%d:%d+%d", &YY, &MM, &DD, &hh, &mm, &ss, &zz);
			}
			else{									// Case for negative offset from GMT
				sscanf(fullTimestamp,"%d/%d/%d,%d:%d:%d-%d", &YY, &MM, &DD, &hh, &mm, &ss, &zz);
				zz = zz*-1;
			}
			YY = YY + 2000;
		}
		j++;
	}
}

//*****************************************************************************
//
// STORE A GSM RESPONSE TO ARRAY responseLine[]
//
//*****************************************************************************
int 
GSMgetResponse(void)
{
	bool readResponse = true;		// Keeps the loop open while getting message
	int readLine = 1;				// Counts the lines of the message
	char *GSMresponse = NULL;		// Use to grab input
	static char g_cInput[128];		// String input to a UART
	
	while (readResponse){
		// Grab a line
		UART1gets(g_cInput,sizeof(g_cInput));
		// Stop after newline
		GSMresponse = strtok(g_cInput,"\n");
		strcpy(responseLine[readLine], GSMresponse);
		// If this line says OK we've got the whole message
		if ( strncmp(responseLine[readLine],"OK",2) == 0 ){readResponse = false;}
		else { readLine++; }
	}
	return(readLine);
}

//*****************************************************************************
//
// PROCESS SMS FOR ENVELOPE AND CONTENT 
// TO DO: 
// 1. Figure out if it's OK to do this with pointers
//
//*****************************************************************************
void 
GSMprocessMessage( int activeMsg )
{
	bool msgPresent = true;			// Flag to ignore deleted messages
	char *msgEnvelope = NULL;		// Message envelope holder
	const char commaCh[] = ",";		// Comma character
	volatile uint32_t ui32Loop;		// For time delay
	int j = 1;						// Keep track of the line being processed
	int k;							// Hold the number of lines
	
	msgContent = NULL;
	
	/// FIRST: Request the message and get the lines of the response (includes 
	/// envelope, nulls, SIM responses)
	UART0printf("\n\r>>> PROCESSING MESSAGE %u",activeMsg);
	UART1printf("AT+CMGR=%u\r\n",activeMsg);
	k = GSMgetResponse();
	
	// Delay for a bit, needed when processing multiple messages
	for(ui32Loop = 0; ui32Loop < 100000; ui32Loop++){}
	
	/// SECOND: Process message lines for envelope information, and to make the 
	/// message into a string
	msgContent = NULL;
	// Now we've got the whole message, so let's parse it
	while ( j < k+1 ){
		// If this line's the envelope, like: 
		// +CMGR: "REC READ","+13158078555","","15/10/08,13:18:40-20"
		if ( strstr(responseLine[j],"+CMGR:") != '\0' ){
			// Parse the line for status, ph number, date, and time.
			msgEnvelope = responseLine[j];
			msgSender = strtok(msgEnvelope,",");	// Skip status
			msgSender = strtok(NULL,commaCh);
			msgDate = strtok(NULL,commaCh);			// Skip phonebook entry
			msgDate = strtok(NULL,commaCh);
			msgTime = strtok(NULL,commaCh);
			strncpy(msgSender,msgSender+2,11);		// Store the number
			msgSender[11] = '\0';
			strncpy(msgDate,msgDate+1,8);			// Store the date
			msgDate[8] = '\0';
			strncpy(msgTime,msgTime,8);				// Store the time
			msgTime[8] = '\0';
			msgPresent = true;						// Envelope means a message
		}
		// Case for message content
		// If we already found the envelope, and the line's not blank...
		else if ( msgEnvelope != NULL && responseLine[j] != NULL ){
			// ... and we haven't found any content, this is the first line.
			if (msgContent == NULL) { msgContent = responseLine[j]; }
			// ... otherwise, add a space and append this line.
			else if ( j + 2 <= k ) {
				strcat(msgContent, " ");
				strcat(msgContent, responseLine[j]);
			}
		}
		// If it's not the envelope, and envelope is blank, no message
		else { msgPresent = false; }
		j++;
	}
	
	// Show the user what we found
	if ( msgPresent ) {
		UART0printf("\n\r> msgENV: from: %s on: %s at: %s",msgSender,msgDate,msgTime);
		UART0printf("\n\r> msgTXT: %s",msgContent);
	}
	else { UART0printf("... doesn't exist"); }
	
	/// THIRD: delete the message
	if ( testDelete && msgPresent ){
		UART1printf("AT+CMGD=%u\r\n",activeMsg);
		k = GSMgetResponse();
		UART0printf ( "\n\r> MSG %u deleted",activeMsg );
	}
}

//*****************************************************************************
//
// GSM - SEND AN SMS
//
//*****************************************************************************
void 
GSMsendSMS(char *destNbr, char *msgBody)
{
	volatile uint32_t ui32Loop;		// For time delay
	bool msgSent = false;			// Flag for confirming message was sent
	static char g_cInput[128];		// String input to a UART
	
	UART1printf( "AT+CMGS=\"%s\"\r",destNbr);			// Initialize with ph#
	for(ui32Loop = 0; ui32Loop < 900000; ui32Loop++){}	// Wait
	UART1printf( "%s\r", msgBody );						// Enter message body
	for(ui32Loop = 0; ui32Loop < 9000; ui32Loop++){}	// Wait
	UART1printf( "\x1A" );								// Ctrl-Z to send

	// Loop to wait for message sent confirmation
	while (msgSent == false){
		UART1gets(g_cInput,sizeof(g_cInput));
		if ( strstr(g_cInput,"OK") != '\0' ){ 
			UART0printf ( "\n\r> Message sent to %s: %s",destNbr,msgBody );
			msgSent = true; 
		}
	}
}

//*****************************************************************************
//
// INITIALIZE I2C MODULE 0
// Slightly modified version of TI's example code - from:
// https://eewiki.net/display/microcontroller/I2C+Communication+with+the+TI+Tiva+TM4C123GXL
//
//*****************************************************************************
void 
initI2C(void)
{
	///reset module
	ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

	/// Configure the pin muxing for I2C0 functions on port B2 and B3.
	ROM_GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	ROM_GPIOPinConfigure(GPIO_PB3_I2C0SDA);

	/// Select the I2C function for these pins.
	ROM_GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
	ROM_GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

	/// Enable and initialize the I2C0 master module.  Use the system clock for
	/// the I2C0 module.  The last parameter sets the I2C data transfer rate.
	/// If false the data rate is set to 100kbps and if true the data rate will
	/// be set to 400kbps.
	ROM_I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

	///clear I2C FIFOs
	HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
	
	/// Enable the I2C interrupt:
	// Set pin C7 as input
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_7);
	// Weak pull up
	ROM_GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	// Disable interrupt for PC7
	GPIOIntDisable(GPIO_PORTC_BASE, GPIO_PIN_7);
	// Clear pending interrupts for PC7
	GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_7);
	// Register port C interrupt handler
	GPIOIntRegister(GPIO_PORTC_BASE, MPR121IntHandler);
	// Configure PC7 for falling edge
	GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_LOW_LEVEL);
	// Enable interrupt
	GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_7);
}

//*****************************************************************************
//
// READ SPECIFIED REGISTER ON I2C SLAVE
// From:
// https://eewiki.net/display/microcontroller/I2C+Communication+with+the+TI+Tiva+TM4C123GXL
//
//*****************************************************************************
uint32_t 
I2Creceive(uint32_t slave_addr, uint8_t reg)
{
	// Specify that we are writing (a register address) to the slave address
	I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);

	// Specify register to be read
	I2CMasterDataPut(I2C0_BASE, reg);

	// Send control byte and register address byte to slave device
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	 
	// Wait for MCU to finish transaction
	while(I2CMasterBusy(I2C0_BASE));
	 
	// Specify that we are going to read from slave device
	I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);
	 
	// Send control byte and read from the register we specified
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
	 
	// Wait for MCU to finish transaction
	while(I2CMasterBusy(I2C0_BASE));
	 
	// Return data pulled from the specified register
	return I2CMasterDataGet(I2C0_BASE);
}

//*****************************************************************************
//
// WRITE TO SPECIFIED REGISTER ON I2C SLAVE
// From:
// https://eewiki.net/display/microcontroller/I2C+Communication+with+the+TI+Tiva+TM4C123GXL
//
//*****************************************************************************
void 
I2Csend(uint8_t slave_addr, uint8_t num_of_args, ...)
{
	// Tell the master module what address it will place on the bus when
	// communicating with the slave.
	I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
	 
	// Stores list of variable number of arguments
	va_list vargs;
	 
	// Specifies the va_list to "open" and the last fixed argument
	// so vargs knows where to start looking
	va_start(vargs, num_of_args);
	 
	// Put data to be sent into FIFO
	I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));
	 
	// If there is only one argument, we only need to use the
	// single send I2C function
	if(num_of_args == 1)
	{
		// Initiate send of data from the MCU
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
		 
		// Wait until MCU is done transferring.
		while(I2CMasterBusy(I2C0_BASE));
		 
		// "Close" variable argument list
		va_end(vargs);
	}
	 
	// Otherwise, we start transmission of multiple bytes on the
	// I2C bus
	else
	{
		// Initiate send of data from the MCU
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
		 
		// Wait until MCU is done transferring.
		while(I2CMasterBusy(I2C0_BASE));
		 
		// Send num_of_args-2 pieces of data, using the
		// BURST_SEND_CONT command of the I2C module
		for(uint8_t i = 1; i < (num_of_args - 1); i++)
		{
			// Put next piece of data into I2C FIFO
			I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));
			// Send next data that was just placed into FIFO
			I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
	 
			// Wait until MCU is done transferring.
			while(I2CMasterBusy(I2C0_BASE));
		}
	 
		// Put last piece of data into I2C FIFO
		I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));
		// Send next data that was just placed into FIFO
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
		// Wait until MCU is done transferring.
		while(I2CMasterBusy(I2C0_BASE));
		 
		// "Close" variable args list
		va_end(vargs);
	}
}

//*****************************************************************************
//
// INITIALIZE MPR121
//
//*****************************************************************************
void 
initMPR121(void)
{
	I2Csend(0x5A,2,0x80, 0x63);			// Soft reset
	
	// Set thresholds and other control values
	I2Csend(0x5A,2,0x2B, 0x01);			// MHD_R
	I2Csend(0x5A,2,0x2C, 0x01);			// NHD_R
	I2Csend(0x5A,2,0x2D, 0x00);			// NCL_R
	I2Csend(0x5A,2,0x2E, 0x00);			// FDL_R
	I2Csend(0x5A,2,0x2F, 0x01);			// MHD_F
	I2Csend(0x5A,2,0x30, 0x01);			// NHD_F
	I2Csend(0x5A,2,0x31, 0x7F);			// NCL_F
	I2Csend(0x5A,2,0x32, 0x09);			// FDL_F
	I2Csend(0x5A,2,0x41, TOU_THRESH);	// ELE0_T
	I2Csend(0x5A,2,0X42, REL_THRESH);	// ELE0_R
	I2Csend(0x5A,2,0x43, TOU_THRESH);	// ELE1_T
	I2Csend(0x5A,2,0X44, REL_THRESH);	// ELE1_R
	I2Csend(0x5A,2,0x45, TOU_THRESH);	// ELE2_T
	I2Csend(0x5A,2,0X46, REL_THRESH);	// ELE2_R
	I2Csend(0x5A,2,0x47, TOU_THRESH);	// ELE3_T
	I2Csend(0x5A,2,0X48, REL_THRESH);	// ELE3_R
	I2Csend(0x5A,2,0x49, TOU_THRESH);	// ELE4_T
	I2Csend(0x5A,2,0X4A, REL_THRESH);	// ELE4_R
	I2Csend(0x5A,2,0x4B, TOU_THRESH);	// ELE5_T
	I2Csend(0x5A,2,0X4C, REL_THRESH);	// ELE5_R
	I2Csend(0x5A,2,0x4D, TOU_THRESH);	// ELE6_T
	I2Csend(0x5A,2,0X4E, REL_THRESH);	// ELE6_R
	I2Csend(0x5A,2,0x4F, TOU_THRESH);	// ELE7_T
	I2Csend(0x5A,2,0X50, REL_THRESH);	// ELE7_R
	I2Csend(0x5A,2,0x51, TOU_THRESH);	// ELE8_T
	I2Csend(0x5A,2,0X52, REL_THRESH);	// ELE8_R
	I2Csend(0x5A,2,0x53, TOU_THRESH);	// ELE9_T
	I2Csend(0x5A,2,0X54, REL_THRESH);	// ELE9_R
	I2Csend(0x5A,2,0x55, TOU_THRESH);	// ELE10_T
	I2Csend(0x5A,2,0X56, REL_THRESH);	// ELE10_R
	I2Csend(0x5A,2,0x57, TOU_THRESH);	// ELE11_T
	I2Csend(0x5A,2,0X58, REL_THRESH);	// ELE11_R
	I2Csend(0x5A,2,0x5D, 0x04);			// FIL_CFG
	
	// Turn on all 12 electrodes and enter run mode
	I2Csend(0x5A,2,0x5E, 0x0C);			// ELE_CFG
}

//*****************************************************************************
//
// TOGGLE KEYPAD LOCK
//
//*****************************************************************************
void 
MPR121toggleLock(void)
{
	if (keysUnlocked) {
		keysUnlocked = false;
		GPIOPinWrite(GPIO_PORTF_BASE, LED_MAP, 0);
		GPIOPinWrite(GPIO_PORTF_BASE, RD_LED, RD_LED);
		UART0printf("\n\r> KEYPAD LOCKED");
		LCDstring(7,0," KEYPAD LOCKED ", INVERSE);
	}
	else{
		keysUnlocked = true;
		GPIOPinWrite(GPIO_PORTF_BASE, LED_MAP, 0);
		GPIOPinWrite(GPIO_PORTF_BASE, GN_LED, GN_LED);
		LCDstring(7,0,"KEYPAD UNLOCKED", NORMAL);
	}
	
}

//*****************************************************************************
//
// INITIALIZE SSI/SPI FOR LCD
// PD0 - SSI3CLK	clock
// PD1 - SSI3FSS	chip select
// PD2 - SSI3RX		MISO		// not present on LCD, cofigure anyway...
// PD3 - SSI3TX		MOSI
// PD6 - 			command		// 0 for command, 1 for data
// PD7 - 			reset		// hold high to activate LCD
//
//*****************************************************************************
void
initSSI3(void)
{
	uint32_t trashBin[1] = {0};
	
	// Configure GPIO Pins for SSI3 mode.
	ROM_GPIOPinConfigure(GPIO_PD0_SSI3CLK);
	ROM_GPIOPinConfigure(GPIO_PD1_SSI3FSS);
	//ROM_GPIOPinConfigure(GPIO_PD2_SSI3RX);
	ROM_GPIOPinConfigure(GPIO_PD3_SSI3TX);
	ROM_GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);

	// Configure the SSI port for SPI master mode. Moto modes:
	// Polarity Phase       Mode
	//   0       0   SSI_FRF_MOTO_MODE_0
	//   0       1   SSI_FRF_MOTO_MODE_1
	//   1       0   SSI_FRF_MOTO_MODE_2
	//   1       1   SSI_FRF_MOTO_MODE_3 -> select for EA DOGS102W6
	SSIConfigSetExpClk(SSI3_BASE, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_3, SSI_MODE_MASTER, 1000000, 8);

	// Enable
	SSIEnable(SSI3_BASE);

	// Clear SSI0 RX Buffer
	while (ROM_SSIDataGetNonBlocking(SSI3_BASE, &trashBin[0])) {}
}

//*****************************************************************************
//
// SEND BYTE ON SSI3
//
//*****************************************************************************
void
SSI3sendByte(uint32_t theData)
{
	uint32_t trashBin[1] = {0};
	
	SSIDataPut(SSI3_BASE, theData);			// Send the data
	while(SSIBusy(SSI3_BASE)) {}			// Wait for transmission to finish
	
	// Clear SSI0 RX Buffer (not sure if this is needed)
	while (ROM_SSIDataGetNonBlocking(SSI3_BASE, &trashBin[0])) {}
}

//*****************************************************************************
//
// INITIALIZE LCD
// PD0 - SSI3CLK	clock		// stays high - why?
// PD1 - SSI3FSS	chip select
// PD2 - SSI3RX		MISO		// not present on LCD
// PD3 - SSI3TX		MOSI
// PD6 - 			command		// 0 for command, 1 for data
// PD7 - 			reset		// hold high to activate LCD
//
//*****************************************************************************
void
initLCD(void)
{
	// Configure D6 as the LCD command, set high
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);
	
	// Set up PD7 as reset (disable NMI) NOTE: PD7 is NMI by default, so unlike 
	// other pins, this procedure must be followed in order to make the pin 
	// usable as GPIO.
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;	// Unlock the port
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= GPIO_PIN_7;		// Unlock the pin
	HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~GPIO_PIN_7;  
	HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= GPIO_PIN_7;
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;				// Lock the port
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_7);	// Output configure
	
	// Set display reset to high -> LCD is running now
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7);
	
	// Initialize bottom view 3.3V (booster on) 8Bit SPI
	LCDsend(0x40,LCD_CMD);	// Startline 0
	LCDsend(0xA1,LCD_CMD);	// SEG reverse
	LCDsend(0xC0,LCD_CMD);	// Set COM direction (COM0-COM63)
	LCDsend(0xA4,LCD_CMD);	// Set all Pixel to on
	LCDsend(0xA6,LCD_CMD);	// Display inverse off
	LCDsend(0xA2,LCD_CMD);	// Set bias 1/9
	LCDsend(0x2F,LCD_CMD);	// Booster, regulator, follower on
	LCDsend(0x27,LCD_CMD);	// Set contrast
	LCDsend(0x81,LCD_CMD);	// Set contrast
	LCDsend(0x10,LCD_CMD);	// Set contrast
	LCDsend(0xFA,LCD_CMD);	// Temperature compensation
	LCDsend(0x90,LCD_CMD);	// Temperature compensation
	LCDsend(0xAF,LCD_CMD);	// Display on

	// Clear display
	LCDclear(0,0,XMAX,YMAX);
}

//*****************************************************************************
//
// SEND BYTE ON LCD
//
//*****************************************************************************
void
LCDsend(uint32_t theData, uint8_t cmdByte)
{
	if (cmdByte == LCD_CMD){ GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0); }
	else { GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); }
	SSI3sendByte(theData);
}

//*****************************************************************************
//
// SET ADDRESS FOR LCD DATA
//
//*****************************************************************************
void 
LCDsetAddress(uint32_t page, uint32_t column)
{
	LCDsend(0xB0+page,LCD_CMD);			// Set page address
	LCDsend(0x0F&column,LCD_CMD);		// Set LSB of column address
	LCDsend(0x10+(column>>4),LCD_CMD);	// Set MSB of column address
}

//*****************************************************************************
//
// CLEAR LCD
//
//*****************************************************************************
void
LCDclear(uint8_t xPxStart, uint8_t yPxStart, uint8_t xPxEnd, uint8_t yPxEnd)
{
	unsigned char LCDbuffer[XPIXEL][YPIXEL/8];
	uint8_t iCol=0,iPg=0,pgStart=0,pgEnd=0,skipPx=0;
	uint8_t maskPx = 0xFF;	// Mask when first and last pixel are on start page
	
	// Calculate start and end pages
	pgStart=yPxStart/8;
	pgEnd=yPxEnd/8;

	/// CLEAR FIRST PAGE from FIRST COLUMN
	LCDsetAddress(pgStart, xPxStart);	// Set address

	// Find start within first page
	skipPx=yPxStart%8;
	// Find stop within first page (if first and last pixel are on the same page)
	if (pgEnd == pgStart) { maskPx=yPxEnd%8-0xFF; }
	
	// Loop from first to last column
	for(iCol=xPxStart; iCol<=xPxEnd; iCol++)
	{
		LCDbuffer[iCol][pgStart]&=maskPx>>(8-skipPx);	// Clear buffer
		LCDsend(LCDbuffer[iCol][pgStart],LCD_DATA);		// Send buffer to display
	}

	/// CLEAR LAST PAGE to LAST COLUMN
	LCDsetAddress(pgEnd, xPxStart);	// Set address
	
	// Find stop within last page
	skipPx=yPxEnd%8;

	// Loop from first to last column
	for(iCol=xPxStart; iCol<=xPxEnd; iCol++)
	{
		LCDbuffer[iCol][pgEnd]&=(0xFF<<(skipPx+1));	// Clear buffer
		LCDsend(LCDbuffer[iCol][pgEnd],LCD_DATA);	// Send buffer to display
	}

	/// CLEAR MIDDLE PAGES and ALL COLUMNS
	// Loop from second page to last page less one
	for(iPg=pgStart+1; iPg<pgEnd; iPg++)
	{
		LCDsetAddress(iPg, xPxStart);	// Set address

		// Loop from first to last column (no need for buffer as we're clearing all)
		for(iCol=xPxStart; iCol<=xPxEnd; iCol++) { LCDsend(0x00,LCD_DATA); }
	}
}

//*****************************************************************************
//
// FILL LCD
//
//*****************************************************************************
void
LCDfill(uint8_t xPxStart, uint8_t yPxStart, uint8_t xPxEnd, uint8_t yPxEnd)
{
	unsigned char LCDbuffer[XPIXEL][YPIXEL/8];
	uint8_t iCol=0,iPg=0,pgStart=0,pgEnd=0,skipPx=0;
	uint8_t maskPx = 0xFF;	// Mask when first and last pixel are on start page
	
	// Calculate start and end pages
	pgStart=yPxStart/8;
	pgEnd=yPxEnd/8;

	/// FILL FIRST PAGE from FIRST COLUMN
	LCDsetAddress(pgStart, xPxStart);	// Set address

	// Find start within first page
	skipPx=yPxStart%8;
	// Find stop within first page (if first and last pixel are on the same page)
	if (pgEnd == pgStart) { maskPx=yPxEnd%8-0xFF; }
	
	// Loop from first to last column
	for(iCol=xPxStart; iCol<=xPxEnd; iCol++)
	{
		LCDbuffer[iCol][pgStart]|=maskPx<<skipPx;	// Fill buffer
		LCDsend(LCDbuffer[iCol][pgStart],LCD_DATA);	// Send buffer to display
	}

	/// FILL LAST PAGE to LAST COLUMN
	LCDsetAddress(pgEnd, xPxStart);	// Set address
	
	// Find stop within last page
	skipPx=yPxEnd%8;

	// Loop from first to last column
	for(iCol=xPxStart; iCol<=xPxEnd; iCol++)
	{
		LCDbuffer[iCol][pgEnd]|=(0xFF>>(8-skipPx-1));	// Fill  buffer
		LCDsend(LCDbuffer[iCol][pgEnd],LCD_DATA);		// Send buffer to display
	}

	/// FILL MIDDLE PAGES and ALL COLUMNS
	// Loop from second page to last page less one
	for(iPg=pgStart+1; iPg<pgEnd; iPg++)
	{
		LCDsetAddress(iPg, xPxStart);	// Set address

		// Loop from first to last column (no need for buffer as we're filling all)
		for(iCol=xPxStart; iCol<=xPxEnd; iCol++) { LCDsend(0xFF,LCD_DATA); }
	}
}

//*****************************************************************************
//
// SEND CHARACTER TO LCD
//
//*****************************************************************************
void
LCDchar(uint8_t row, uint8_t col, uint16_t f, uint8_t style)
{
	// Each Character consists of 6 Columns on 1 Page
	// Each Page presents 8 pixels vertically (top = MSB)
	uint8_t b;
	uint16_t h;

	// Row boundary check
	if (row > 7) { row = 7; }
	// Column boundary check
	if (col > 101) { col = 101; }

	// Handle characters not in our table, replace with '.'
	if (f < 32 || f > 129) { f = '.'; }

	// Subtract 32 because FONT6x8[0] is "space" which is ascii 32,
	// Multiply by 6 because each character is columns wide
	h = (f - 32) * 6;

	// Set address
	LCDsetAddress(row,col);
	
	// Send character
	if (style == INVERSE){ 
		for (b = 0; b < 6; b++) { LCDsend(FONT6x8[h + b] ^ 0xFF,LCD_DATA); }
	}
	else { 
		for (b = 0; b < 6; b++) { LCDsend(FONT6x8[h + b],LCD_DATA); }
	}
}

//*****************************************************************************
//
// PRINT STRING TO LCD
//
//*****************************************************************************
void
LCDstring(uint8_t row, uint8_t col, char *word, uint8_t style)
{
	// Each Character consists of 6 Columns on 1 Page
	// Each Page presents 8 pixels vertically (top = MSB)
	uint8_t a = 0;

	// Row boundary check
	if (row > 7) { row = 7; }
	// Column boundary check
	if (col > 101) { col = 101; }

	while (word[a] != 0)
	{
		// check for line feed '/n'
		if (word[a] != 0x0A)
		{
			//check for carriage return '/r' (ignore if found)
			if (word[a] != 0x0D)
			{
				//Draw a character
				LCDchar(row, col, word[a], style);

				//Update location
				col += 6;

				//Text wrapping
				if (col >= 102)
				{
					col = 0;
					if (row < 7) { row++; }
					else { row = 0; }
				}
			}
		}
		// handle line feed character
		else
		{
			if (row < 7) { row++; }
			else { row = 0; }
			col = 0;
		}
		a++;
	}
}

//*****************************************************************************
//
// PRINT LINE TO LCD
//
//*****************************************************************************
void
LCDline(uint8_t xPxStart, uint8_t yPxStart, uint8_t xPxEnd, uint8_t yPxEnd)
{
	unsigned char LCDbuffer[XPIXEL][YPIXEL/8];
	uint8_t iCol=0,iPg=0,pgStart=0,pgEnd=0,skipPx=0;
	uint8_t maskPx = 0xFF;	// Mask when first and last pixel are on start page
	
	// Calculate start and end pages
	pgStart=yPxStart/8;
	pgEnd=yPxEnd/8;

	// Set address of start page
	LCDsetAddress(pgStart,xPxStart);	// Set address

	// Find start within first page
	skipPx=yPxStart%8;

	/// HORIZONTAL LINE
	if ( yPxStart == yPxEnd ){
		// Loop from first to last column
		for(iCol=xPxStart; iCol<=xPxEnd; iCol++)
		{
			LCDbuffer[iCol][pgStart]|=0x01<<skipPx;	// Fill buffer
			LCDsend(LCDbuffer[iCol][pgStart],1);	// Send buffer to display
		}
	}
	/// VERTICAL LINE
	else {
		// START PAGE
		// Find stop within first page (if first and last pixel are on same page)
		if (pgEnd == pgStart) { maskPx=yPxEnd%8-0xFF; }
		
		// Fill appropriate parts
		LCDbuffer[xPxStart][pgStart]|=maskPx<<skipPx;
		LCDsend(LCDbuffer[xPxStart][pgStart],1);
		
		// If line is only one page, exit function here
		if (pgEnd == pgStart) { return; }
		
		// LAST PAGE
		LCDsetAddress(pgEnd,xPxStart);	// Set address
		
		// Find stop within last page
		skipPx=yPxEnd%8;
		
		// Fill appropriate parts
		LCDbuffer[xPxStart][pgEnd]|=(0xFF>>(8-skipPx-1));
		LCDsend(LCDbuffer[xPxStart][pgEnd],1);
		
		// MIDDLE PAGES
		for(iPg=pgStart+1; iPg<pgEnd; iPg++)
		{
			LCDsetAddress(iPg,xPxStart);	// Set address

			// Fill appropriate parts (no need for buffer as we're filling all)
			LCDsend(0xFF,1);
		}
	}
}

//*****************************************************************************
//
// PRINT RECTANGLE TO LCD
//
//*****************************************************************************
void 
LCDrect(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
	// (x1,y1)-------------(x2,y1)
	//    |                   |
	//    |                   |
	//    |                   |
	// (x1,y2)-------------(x2,y2)
	
	LCDline(x1, y1, x2, y1);	// Top
	LCDline(x1, y2, x2, y2);	// Bottom
	LCDline(x1, y1, x1, y2);	// Left
	LCDline(x2, y1, x2, y2);	// Right
}

//!*****************************************************************************
//! THE PROGRAM
//!*****************************************************************************
int
main(void)
{
	char aString[2][128];				// Generic string
	int msgOpen = 0;					// Message being processed
	int ctr1;							// Generic counter
	volatile uint32_t ui32Loop;			// for time delay
	uint32_t pui32ADC0Value[1];			// ADC0 data value
	uint32_t ui32D0v;					// mV value on external input D0
	char D0v[100];						// D0v stored as a string
	
	/// Initial settings - from Anil
	ROM_FPUEnable();					// Enable floating point unit
	ROM_FPULazyStackingEnable();		// Enable lazy stacking of FPU
	ROM_IntMasterEnable();				// Enable processor interrupts
	
	/// Enable device clocking
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	
	/// Enable peripherals
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);		// ADC1
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);	// EEPROM (2048 bytes in 32 blocks)
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);	// Pins: UART0 
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);	// Pins: UART1, GSM, Rel3N, I2C0SCL & SDA
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);	// Pins: Neopixel, keypad INT2
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);	// Pins: LCD screen
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);	// Pins: Rel1N, Rel2, Rel2N, Rel3, Rel4
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);	// Pins: RGB LED, Rel1, Rel4N
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);		// I2C for MPR121 touchpad controller
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);		// SSI3 for EA DOGS102W6 LCD display
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);	// Timer for keylock
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);	// Timer for keypad timeout
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);	// Console UART
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);	// GSM UART
    
	/// Configure GPIO outputs (disable neopixel, relays one and four in current hardware version)
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);		//Rel3N
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6);		//GSM PWRKEY
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);		//GSM RESET
	//ROM_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);	//Neopixel
	//ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);	//Rel4
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2);		//Rel3
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_3);		//Rel2
	//ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);	//Rel1N
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_5);		//Rel2N
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);	//RGB LED (PF1=Rel1)
	//ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);	//Rel4N/USR SW1
	
	// Turn on an LED to show that we're working
	GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, BL_LED);
	
	// Start the LCD display
	initSSI3();			// Start SSI3/SPI for sending data to LCD
	initLCD();			// Start the LCD screen
	
	// Turn the relays off initially
	relayOff(0);

	// Console UART0: Set PA0 and PA1 as UART0, configure for 115200, 
	// 8-N-1 operation, enable interrupts
	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UART0StdioConfig(0, 115200, 16000000);
	ROM_IntEnable(INT_UART0);
	ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);  

	// GSM UART1: Set PB0 and PB1 as UART1, configure for 115200, 
	// 8-N-1 operation, enable interrupts
	ROM_GPIOPinConfigure(GPIO_PB0_U1RX);
	ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
	ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UART1StdioConfig(1, 115200, 16000000);
	ROM_IntEnable(INT_UART1);
	ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);     

	// Notify the user what's going on
	UART0printf("\n\n\n\r>>> INITIALIZING");
	LCDstring(5,0,"INITIALIZING...",NORMAL);
	
	// Notify the user what testing functions are active
	UART0printf("\n\r> ----------Testing function status:----------");
	if (testGSM) { UART0printf("\n\r> ENABLED : GSM power at boot"); }
	else {UART0printf("\n\r> DISABLED: GSM power at boot");}
	if (testEEPROM) { UART0printf("\n\r> ENABLED : Store/retrieve ontime from EEPROM"); }
	else {UART0printf("\n\r> DISABLED: Store/retrieve ontime from EEPROM");}
	if (testRelay) { UART0printf("\n\r> ENABLED : Cycle relays on/off"); }
	else {UART0printf("\n\r> DISABLED: Cycle relays on/off");}
	if (testDelete) { UART0printf("\n\r> ENABLED : Delete messages during processing"); }
	else {UART0printf("\n\r> DISABLED: Delete messages during processing");}
	if (testNotify) { UART0printf("\n\r> ENABLED : Message controller at boot"); }
	else {UART0printf("\n\r> DISABLED: Message controller at boot");}
	if (testI2C) { UART0printf("\n\r> ENABLED : Activate touchpad"); }
	else {UART0printf("\n\r> DISABLED: Activate touchpad");}
	UART0printf("\n\r> --------------------------------------------");
	
	/// GSM TEST AREA: Make sure GSM is on and get the time.
	if (testGSM){
		UART0printf("\n\r> Checking to see if GSM is on...");
		// Find out if the GSM module is on - check three times
		for (ctr1=1; ctr1<4; ctr1++){
			UART0printf( "\n\r> Check %u of 3...\n\r", ctr1 );
			if ( GSMcheckPower() )
			{
				blinkLED(5,3,BL_LED);
				UART0printf("\n\r> GSM is ON.");
				break;
			}
			else
			{
				UART0printf("\n\r> GSM is OFF, turning on...");
				GSMtogglePower();
				// Delay for a bit, check again. Keep this delay long 
				// enough for GSM to get the time.
				for(ui32Loop = 0; ui32Loop < 3000000; ui32Loop++){}
				UART1printf("AT\r");
				if (GSMoff){ UART0printf("\n\r> Power on failed!"); }
			}
		}
		if ( ctr1 == 4 ){
			GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, 0);
			blinkLED(10,3,RD_LED);
			GPIOPinWrite(GPIO_PORTF_BASE, RD_LED, RD_LED);
			UART0printf("\n\r> Cannot power on the GSM! \n\r>>> ENDING PROGRAM");
			return 0;
		}

		// Get the time from the GSM module and print to user
		UART0printf("\n\r> Getting date/time from the GSM module");
		GSMcheckTime();
		UART0printf("\n\r> Timestamp: %s",fullTimestamp);
		LCDstring(6,0,fullTimestamp,NORMAL);
	}

	/// EEPROM TEST AREA: Store on-time, retrieve last on-time. 
	// Don't run this each time 'cause EEPROM wears out.
	if (testEEPROM){
		EEPROMInit();
		struct eprom_timestamp eprom_writetime = {YY,MM,DD,hh,mm,ss,zz};
		//Read from struct at EEPROM start from 0x0000
		EEPROMRead((uint32_t *)&eprom_readtime, E2PROM_TEST_ADRES, sizeof(eprom_readtime));
		UART0printf("\n\r> Last on-time: %u/%u/%u, %u:%u:%u, %d (EEPROM address: %u)", eprom_readtime.v1, eprom_readtime.v2, eprom_readtime.v3, eprom_readtime.v4, eprom_readtime.v5, eprom_readtime.v6, eprom_readtime.v7, E2PROM_TEST_ADRES);
		//Write struct to EEPROM start from 0x0000
		EEPROMProgram((uint32_t *)&eprom_writetime, E2PROM_TEST_ADRES, sizeof(eprom_writetime));
	
		// Some EEPROM functions
		/*esize = EEPROMSizeGet(); // Get EEPROM Size 
		UART0printf("E2> EEPROM Size %d bytes\n", e2size);
		eblock = EEPROMBlockCountGet(); // Get EEPROM Block Count
		UART0printf("E2> EEPROM Blok Count: %d\n", e2block);*/
	}
	
	/// RELAY TEST AREA: Cycle on/off, flash green when turning on, red for off.
	if (testRelay) {
		UART0printf("\n\r> RELAY TESTING:");
		// Delay for a bit (so you can grab your multimeter)
		for(ui32Loop = 0; ui32Loop < 1000000; ui32Loop++){}
		for (ctr1=1; ctr1<5; ctr1++){
			UART0printf("\n\r> RELAY %u: ON...",ctr1);
			relayOn(ctr1);
			blinkLED(7,3,GN_LED);
			UART0printf("OFF");
			relayOff(ctr1);
			blinkLED(7,3,RD_LED);
		}
	}
		
	/// TOUCHPAD TEST AREA
	if (testI2C){
		// Start I2C module
		initI2C();
		// Start the MPR121 and set thresholds
		initMPR121();
		
		// Set up the timers used to lock/unlock the keypad
		ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);
		ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT);
		ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, ROM_SysCtlClockGet()* 15);
		// Setup the interrupts for the timer timeouts
		ROM_IntEnable(INT_TIMER0A);
		ROM_IntEnable(INT_TIMER1A);
		ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
		ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	}
	
	/// ADC TEST AREA
	if (testADC){ ADCinit(); }	
	
	// Initialize the SysTick interrupt to process buttons
	ButtonsInit();
	SysTickPeriodSet(SysCtlClockGet() / APP_SYSTICKS_PER_SEC);
	SysTickEnable();
	SysTickIntEnable();
	
	// Setup complete!
	UART0printf("\n\r> Setup complete! \n\r>>> RUNNING MAIN PROGRAM");
	LCDstring(5,0,"COMPLETE!       ",NORMAL);
	GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, RD_LED, RD_LED);
	blinkLED(5,5,GN_LED);
	// We were in talk mode during set-up to let SIM messages 
	// through, but now we don't want that.
	talkMode = false;
	// Tell the user about buttons
	UART0printf("\n\r> Press left button to enter \"talk to GSM\" mode (blue LED)");
	UART0printf("\n\r> Press right button to toggle power to GSM module (red LED)");
	
	// Start keypad timer timeout
	if (testI2C) {ROM_TimerEnable(TIMER1_BASE, TIMER_A);}
	
	// CONTROLLER NOTIFY
	if (testNotify){
		strcpy (aString[1],"BOARD #");
		strcat (aString[1], boardID);
		strcat (aString[1], " ON: ");
		strcat (aString[1], fullTimestamp );
		GSMsendSMS( ctrlID, aString[1] );
	}
	
	LCDstring(5,0,"                ",NORMAL);
	LCDstring(7,0," KEYPAD LOCKED ", INVERSE);
	
	// MAIN LOOP - wait for new message notification and process
	while(1){
		// Process new messages.
		if (msgCount > 0){
			// Start working on the oldest message
			msgOpen = msgCount;
			msgCount--;
			// Delay to let each message get printed to UART before going to the next
			for(ui32Loop = 0; ui32Loop < 10000; ui32Loop++){}
			// Process message for envelope and content
			GSMprocessMessage(msgOpen);
			// Turn on/off the appropriate relays
			if (strstr(msgSender,ctrlID) != NULL && strlen(msgContent) == 4) {
				for (ctr1=0;ctr1<4;ctr1++){
					if (msgContent[ctr1] == '0') { relayOff(ctr1+1); }
					else if (msgContent[ctr1] == '1') { relayOn(ctr1+1); } 
				}
			}
		}
		
		// Update the ADC
		if (testADC){
			// Trigger the ADC conversion.
			ADCProcessorTrigger(ADC0_BASE, 3);
			
			// Wait for conversion to be completed.
			while(!ADCIntStatus(ADC0_BASE, 3, false)){}

			// Clear the ADC interrupt flag.
			ADCIntClear(ADC0_BASE, 3);

			// Read ADC Value.
			ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);
			
			// Convert to millivolts
			ui32D0v = pui32ADC0Value[0] * (3300.0/4095);
			
			// Convert to a string
			snprintf (D0v,100,"D0 = %d.%03d V", ui32D0v / 1000, ui32D0v % 1000);

			// Display the AIN0 (PE0) digital value on the console.
			LCDstring(6,0,D0v,NORMAL);

			// Wait a bit
			for(ui32Loop = 0; ui32Loop < 1000000; ui32Loop++){}
		}
		
	}
	//return(0);
}

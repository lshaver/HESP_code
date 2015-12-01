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
#include <ctype.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include "buttons.h"
#include "stdio.h"
#include "string.h"
#include "uart0stdio.h"		// Console
#include "uart1stdio.h"		// GSM
#include "driverlib/adc.h"
#include "driverlib/eeprom.h"
#include "driverlib/flash.h"
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
#define RD_LED	GPIO_PIN_1
#define BL_LED	GPIO_PIN_2
#define GN_LED	GPIO_PIN_3
#define LED_MAP	RD_LED | BL_LED | GN_LED

// For driving buttons
#define APP_SYSTICKS_PER_SEC		32
#define APP_BUTTON_POLL_DIVIDER		8
#define APP_HIB_BUTTON_DEBOUNCE		(APP_SYSTICKS_PER_SEC * 3)

// EEPROM addresses
#define E2A_START			0x0000
#define E2A_RELAY_STATUS	0x0000
#define E2A_CTRL_ID			0x0008
#define E2A_MASTER_ID		0x0008
#define E2A_SLAVE_ID1		0x0008
#define E2A_SLAVE_ID2		0x0008

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
#define XPIXEL		102
#define YPIXEL		64
#define XMAX		XPIXEL-1
#define YMAX		YPIXEL-1
#define LCD_CMD		0
#define LCD_DATA	1
#define NORMAL		1
#define INVERSE		2

//!*****************************************************************************
//! FUNCTION DEFINITIONS
//!*****************************************************************************

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
void relayLatch(int relayNum);
void relayToggle(int relayNum);
void relaySet(uint32_t relayCmd);
void relayStatusE2(void);
/// -- GSM module
bool GSMcheckPower (int checkCount);
void GSMtogglePower(void);
int GSMgetResponse(void);
void GSMcheckTime(void);
void GSMprocessMessage(int activeMsg );
bool GSMparseMessage(int lineCount );
bool GSMsendSMS(char *destNbr, char *msgBody);
int GSMcheckSignal(void);
void GSMcheckBalance(void);
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
void LCDchar(uint8_t row, uint8_t col, uint16_t f, uint8_t style);
void LCDstring(uint8_t row, uint8_t col, char *word, uint8_t style);

// FUNCTIONS TO BUILD:
// - Use keypad to change values

//!*****************************************************************************
//! GLOBAL VARIABLES
//!*****************************************************************************

// TESTING VARIABLES: use these to disable code segments during testing
bool testGSM =		1;				// Turn on the GSM during boot
bool testEEPROM =	1;				// Store/retrieve ontime from EEPROM 
									// (requires testGSM)
bool testDelete =	0;				// Delete messages during processing
bool testNotify =	0;				// Text the controller when coming on-line
bool testADC =		1;				// Analog to Digital converter

// Identifying constants
char ctrlID[] = "3158078555";		// Phone number of the controller
uint32_t boardID1;					// IDpt1 of MCU
uint32_t boardID2;					// IDpt2 of MCU
char IMEI[] = "000000000000000";	// SN of SIM module (equates to board ID)
char SIMID[] = "NO SIM CARD";		// Phone number of SIM card
int hwRev = 1;						// Hardware rev of this board

// Status holders across functions
bool GSMoff = true;					// Flag to see if the GSM module is on/off
bool talkMode = true;				// Allow user to interface directly with GSM
bool SIMpresent = false;			// Flag to see if SIM is present - assume NO
bool keysUnlocked =	false;			// For locking the keypad
uint32_t relayStatus=0;				// Stores the status of the relays
uint32_t E2relayStatus=0;			// For reading/writing relay status to EEPROM
int buttonLEDhold;					// Holds status of RGB LED
int msgCount = 0;					// Hold the int value w/count of new messages
char pressedKey;					// Keypad: store which key was last pressed
uint16_t touchedMap;				// Map of key status
char balance[] = 	"000.00";		// Remaining SIM balance
int curCol=XPIXEL;					// Current column in LCD

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
char fullOnTime[23] = "\0";			// Complete ontime
char lastOnTime[23] = "\0";			// Previous ontime (from EEPROM)

// For EEPROM
uint32_t E2size,E2block;			// EEPROM block size and block count
struct E2S_TIME						// Struct for writing timestamps to EEPROM
{
	int E2YY;
	int E2MM;
	int E2DD;
	int E2hh;
	int E2mm;
	int E2ss;
	int E2zz;
}; 
struct E2S_TIME E2readTime =  {0,0,0,0,0,0,0};		// Read time

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
	
	// Interrupt trigger means GSM is on
	if ( GSMoff ) { GSMoff = false; }
	
	// Loop while there are characters in the receive FIFO.
	while(ROM_UARTCharsAvail(UART1_BASE))
	{
		// Grab a character
		var = (unsigned char)ROM_UARTCharGetNonBlocking(UART1_BASE);
		
		// Hold it
		ptr[i] = var;
		
		// In talk mode mirror to console...
		if (talkMode) { ROM_UARTCharPutNonBlocking(UART0_BASE, ptr[i]); }
		
		// ...or else see if it's a message notification (like +CMTI: "SM",12):
		else 
		{
			if(ptr[i-3] == 'C' && ptr[i-2] == 'M' && ptr[i-1] == 'T'&& ptr[i] == 'I')
			{
				// Grab everything
				UART1gets(g_cInput,sizeof(g_cInput));
				
				// Stop after newline character
				msgCountStr = strtok(g_cInput,"\n");
				
				// Parse out the message count (terminate with null to store)
				strncpy(msgCountStr,msgCountStr+7,3);
				msgCountStr[3]='\0';
				
				// Convert to integer
				sscanf(msgCountStr, "%d", &msgCount);
				
				// Tell the user
				UART0printf("\n\r>>> %u NEW MESSAGE(S)",msgCount);
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
	int touchNumber = 0;				// How many keys were touched
	int relayNum;						// For direct relay control
	uint32_t touchedLSB, touchedMSB;	// Map of touched keys
	
	// Get the status of the electrodes
	touchedLSB = I2Creceive(0x5A,0x00);
	touchedMSB = I2Creceive(0x5A,0x01);
	touchedMap = ((touchedMSB << 8) | touchedLSB);

	// Check how many electrodes were pressed
	for ( int j=0; j<12; j++ ) { if ((touchedMap & (1<<j))) { touchNumber++; } }
	
	// If one electrode was pressed, register it
	if (touchNumber == 1) {
		if (touchedMap & (1<<K1)) { pressedKey = '1'; }
		else if (touchedMap & (1<<K2)) { pressedKey = '2'; }
		else if (touchedMap & (1<<K3)) { pressedKey = '3'; }
		else if (touchedMap & (1<<K4)) { pressedKey = '4'; }
		else if (touchedMap & (1<<K5)) { pressedKey = '5'; }
		else if (touchedMap & (1<<K6)) { pressedKey = '6'; }
		else if (touchedMap & (1<<K7)) { pressedKey = '7'; }
		else if (touchedMap & (1<<K8)) { pressedKey = '8'; }
		else if (touchedMap & (1<<K9)) { pressedKey = '9'; }
		else if (touchedMap & (1<<K0)) { pressedKey = '0'; }
		else if (touchedMap & (1<<KS)) { pressedKey = '*'; }
		else if (touchedMap & (1<<KP)) { pressedKey = '#';
			
			// Start timer 0A when user touches "#," to see if they're
			// trying to lock or unlock
			ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet()* 2);
			ROM_TimerEnable(TIMER0_BASE, TIMER_A);
		}
		if (keysUnlocked) 
		{ 
			// If the keys are unlocked, toggle relay (if applicable):
			relayNum = pressedKey - '0';
			if ( relayNum > 0 && relayNum <= 4 ){ relayToggle(relayNum-1); }
			
			// Print the touched key to console and LCD screen:
			UART0printf("\n\r> %c",pressedKey); 
			curCol+=6;
			if ( curCol>XMAX ) { curCol = 0; }
			LCDchar(6,curCol,pressedKey, NORMAL);
			LCDchar(6,curCol+6,' ', NORMAL);
			
			// And start timer 1A, the idle keypad timer:
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

	// The timer is up! If # is still being pressed, toggle keylock. (Note that 
	// if the user is just rapidly pressing # this could still work - but that's
	// not a big concern.)
	if (touchedMap & (1<<KP)) 
	{ 
		// Lock if keys are unlocked
		if (keysUnlocked){ MPR121toggleLock(); }
		
		// Otherwise, unlock and restart the timer
		else { 
			MPR121toggleLock();
			
			// Restart the idle timer
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
				if (talkMode == false)
				{
					// Get LED status and store. Turn on blue LED.
					buttonLEDhold = GPIOPinRead(GPIO_PORTF_BASE, LED_MAP);
					GPIOPinWrite(GPIO_PORTF_BASE, LED_MAP, 0);
					GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, BL_LED);
					UART0printf("\n\r> [Left button] Entering talk to GSM mode. Press left button to end.");
					talkMode = true;
				}
				else
				{
					UART0printf("\n\r> [Left button] Returning to main program.");
					
					// Restore LED status
					GPIOPinWrite(GPIO_PORTF_BASE, LED_MAP, buttonLEDhold);
					talkMode = false;
					
					// Get the GSM signal strength and print to LCD
					GSMcheckSignal();
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
	// Rev 1: Set PE0 (External D0) as the ADC pin
	// Rev 2: Set PE1 (unnassigned) as the ADC pin
	if (hwRev == 1) { GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); }
	else if (hwRev == 2) { GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1); }
	
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
	if (hwRev == 1) { ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END); }
	else if (hwRev == 2) { ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END); }
	
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
blinkLED (int blinkCount, int blinkLength, uint8_t LEDcolor)
{
	int blinkHold;		// Hold LED status
	
	// Store current LED status, then turn them all off
	blinkHold = GPIOPinRead(GPIO_PORTF_BASE, LED_MAP);
	GPIOPinWrite(GPIO_PORTF_BASE, LED_MAP, 0);
	
	for( int j=blinkCount; j>=0; j--)
	{
		// Turn on the LED.
		GPIOPinWrite(GPIO_PORTF_BASE, LEDcolor, LEDcolor);
		
		// Delay for a bit.
		ROM_SysCtlDelay(ROM_SysCtlClockGet()/100*blinkLength);
		
		// Turn off the LED.
		GPIOPinWrite(GPIO_PORTF_BASE, LED_MAP, 0);

        // Delay for a bit.
        ROM_SysCtlDelay(ROM_SysCtlClockGet()/10*blinkLength);
    }
	
	// Restore prior LED status
	GPIOPinWrite(GPIO_PORTF_BASE, LED_MAP, blinkHold);
}

//*****************************************************************************
//
// TURN ON RELAY
//
//*****************************************************************************
void 
relayOn(int relayNum)
{
	if ( relayNum == 0 )
	{
		// Rel1
		if (hwRev == 1) { GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1); }
		else if (hwRev == 2) { GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0); }
	}
	else if ( relayNum == 1 )
	{
		// Rel2
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_PIN_3);
	}
	else if ( relayNum == 2 )
	{
		// Rel3
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);
	}
	else if ( relayNum == 3 )
	{
		// Rel4
		if (hwRev == 1) { GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4); }
		else if (hwRev == 2) { GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4); }
	}
}

//*****************************************************************************
//
// TURN OFF RELAY
//
//*****************************************************************************
void 
relayOff(int relayNum)
{
    if ( relayNum == 0 )
	{
		// Rel1N
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);
	}
	else if ( relayNum == 1 )
	{
		// Rel2N
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5);
	}
	else if ( relayNum == 2 )
	{
		// Rel3N
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);
	}
	else if ( relayNum == 3 )
	{
		// Rel4N
		if (hwRev == 1) { GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1); }
		else if (hwRev == 2) { GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0); }
	}
}

//*****************************************************************************
//
// LATCH RELAY
//
//*****************************************************************************
void 
relayLatch(int relayNum)
{
    if ( relayNum == 0 )
	{
		// Rel1
		if (hwRev == 1) { GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0); }
		else if (hwRev == 2) { GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0); }
		// Rel1N
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0);
	}
	else if ( relayNum == 1 )
	{
		// Rel2
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 0);
		// Rel2N
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0);
	}
	else if ( relayNum == 2 )
	{
		// Rel3
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);
		// Rel3N
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0);
	}
	else if ( relayNum == 3 )
	{
		// Rel4
		if (hwRev == 1) { GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0); }
		else if (hwRev == 2) { GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0); }
		// Rel4N
		if (hwRev == 1) { GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0); }
		else if (hwRev == 2) { GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0); }
	}
}

//*****************************************************************************
//
// TOGGLE RELAY
//
//*****************************************************************************
void 
relayToggle(int relayNum)
{
	// Hold the relay status for passing to relay set function
	uint32_t h_status = relayStatus;
	
	// Set/clear the appropriate relay
	relaySet(h_status ^= 1 << relayNum);
}

//*****************************************************************************
//
// SET RELAY STATES
//
//*****************************************************************************
void 
relaySet(uint32_t relayCmd)
{
	char printStatus[7];		// For updating status on LCD screen
		
	// Set the relays on/off as required. Update display.
	for ( int r=0; r<4; r++ )
	{
		if ( relayCmd & (1 << r) ^ relayStatus & (1 << r) ) 
		{ 
			if ( relayCmd & (1 << r) ) 
			{ 
				relayOn(r);
				snprintf (printStatus,7,"R%u ON ", r+1 );
				LCDstring(r,0,printStatus,NORMAL);
			}
			else 
			{ 
				relayOff(r); 
				snprintf (printStatus,7,"R%u OFF", r+1 );
				LCDstring(r,0,printStatus,INVERSE);
			}
		}
	}
	
	// Wait 10ms
	ROM_SysCtlDelay(ROM_SysCtlClockGet()/100);
	
	// Set appropriate pins back to ground
	for ( int r=0; r<4; r++ )
	{
		if ( relayCmd & (1 << r) ^ relayStatus & (1 << r) ) { relayLatch(r); }
	}
	
	// Update global relay status
	relayStatus = relayCmd;
}

//*****************************************************************************
//
// STORE RELAY STATUS TO EEPROM
//
//*****************************************************************************
void
relayStatusE2(void)
{
	// Make sure value to write is different from current value
	if ( E2relayStatus != relayStatus )
	{
		// Grab the relay status
		E2relayStatus = relayStatus;
		
		// Notify user
		UART0printf("\n\r> Storing relay status to EEPROM (%X): ",relayStatus);
		for ( int r=0; r<4; r++ ) { UART0printf("%u",(relayStatus >> r) & 1); }
		
		// Write to EEPROM
		EEPROMProgramNonBlocking(E2relayStatus, E2A_RELAY_STATUS);
	}
}

//*****************************************************************************
//
// GSM - CHECK POWER
//
//*****************************************************************************
bool 
GSMcheckPower (int checkCount)
{
	for ( int ctr1=0; ctr1<checkCount; ctr1++)
	{
		UART0printf( "\n\r> GSM power check %u of %u...", ctr1+1,checkCount );
		
		// Set the flag OFF initially
		GSMoff = true;
		
		// Send an AT: if it triggers the int, GSM is on.
		UART1printf("AT\r");
		
		// Delay for a bit
		ROM_SysCtlDelay(ROM_SysCtlClockGet()/10);
		
		// Check the flag
		if ( GSMoff ) { 
			UART0printf( "\n\r> GSM off, starting...");
			GSMtogglePower(); 
		}
		else { break; }
		
		// Delay for a bit
		ROM_SysCtlDelay(ROM_SysCtlClockGet()/4);
	}

	// Report if we couldn't power on the GSM
	if ( GSMoff ) 
	{
		UART0printf("\n\r> Could not power the GSM in %u tries!",checkCount);
		LCDclear(0,0,XMAX,YMAX);
		LCDstring(0,0,"No GSM power!",NORMAL);
		return false;
	}
	else 
	{
		UART0printf("\n\r> GSM power confirmed!");
		return true;
	}
}

//*****************************************************************************
//
// GSM - TOGGLE POWER
// GSM power key is active low. Pull the pin high for one second to toggle power
//
//*****************************************************************************
void 
GSMtogglePower( void )
{     
	// Set the flag to off
	GSMoff = true;
	
	// Set GSM reset low (must stay low to operate)
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0);
	ROM_SysCtlDelay(ROM_SysCtlClockGet()/20);
	UART0printf("\n\r--- 1/3 GSM RESET ASSERTED... ");
	
	// Set GSM power high for at least one second. This pulls down the actual 
	// SIM900 power key.
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7);
	
	// 3 second delay
	ROM_SysCtlDelay(ROM_SysCtlClockGet()*2);
	
	// PWRKEY de-asserted
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0);
	UART0printf("\n\r--- 2/3 GSM POWER KEY TOGGLED: ");
	
	// Wait a bit, then confirm status
	ROM_SysCtlDelay(ROM_SysCtlClockGet()/16);
	if (!GSMoff){ UART0printf("\n\r--- 3/3 GSM POWER ON!"); }
	else { UART0printf("\n\r--- 3/3 GSM POWER OFF!"); }
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
	
	// Ask GSM module for the time
	UART1printf("AT+CCLK?\r");
	
	// Store to responseLine[] array, get line count
	k = GSMgetResponse();
		
	while ( j < k-1 ){
		// Find the time in the lines we get back
		if ( strstr(responseLine[j],"+CCLK: \"") != '\0' ){
			strncpy(fullOnTime,responseLine[j]+8,20);
			
			// Split to individual integers
			if (strstr(fullOnTime,"+")){			// Case for positive offset from GMT
				sscanf(fullOnTime,"%d/%d/%d,%d:%d:%d+%d", &YY, &MM, &DD, &hh, &mm, &ss, &zz);
			}
			else{									// Case for negative offset from GMT
				sscanf(fullOnTime,"%d/%d/%d,%d:%d:%d-%d", &YY, &MM, &DD, &hh, &mm, &ss, &zz);
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
	
	while ( readResponse )
	{
		// Grab a line
		UART1gets(g_cInput,sizeof(g_cInput));
		
		// Stop after newline
		GSMresponse = strtok(g_cInput,"\n");
		strcpy(responseLine[readLine], GSMresponse);
		
		// If this line says OK we've got the whole message
		if ( strncmp(responseLine[readLine],"OK",2) == 0 ){readResponse = false;}
		
		// If this line says "DST: " it means we just caught the network,
		// and should signal (via returning 0) that the request should be
		// sent again.
		else if ( strncmp(responseLine[readLine],"DST: ",2) == 0 )
		{ 
			readLine = 0;
			readResponse = false;
		}
		else { readLine++; }
	}
	
	return readLine;
}

//*****************************************************************************
//
// SEE IF SIM CARD IS PRESENT
//
//*****************************************************************************
void
GSMgetSIMstatus(void)
{
	int ctr1;
	
	// Request SIM card status
	UART1printf("AT+CSMINS?\r");
	ctr1 = GSMgetResponse();
	
	// Parse the response
	for ( int ctr2 = 0; ctr2<= ctr1; ctr2++ )
	{
		if ( strstr(responseLine[ctr2],"+CSMINS: 0,1") != '\0' ) { 
			SIMpresent = 1;
			UART0printf("\n\r> SIM present!");
		}
		else if ( strstr(responseLine[ctr2],"+CSMINS: 0,0") != '\0' ) { 
			UART0printf("\n\r> SIM card not present!");
		}
	}
}

//*****************************************************************************
//
// GET IMEI FROM SIM900
//
//*****************************************************************************
void
GSMgetIMEI(void)
{
	int ctr1;
	
	// Request IMEI
	UART1printf("AT+GSN\r");
	ctr1 = GSMgetResponse();
	
	// Parse the response
	for ( int ctr2 = 0; ctr2<= ctr1; ctr2++ )
	{
		// GSN response is just the 15-digit IMEI number
		if ( isdigit(responseLine[ctr2][0]) ) { strncpy(IMEI,responseLine[ctr2],15); }
	}
	UART0printf("\n\r> IMEI: %s",IMEI );
}

//*****************************************************************************
//
// GET SIM PHONE NUMBER
//
//*****************************************************************************
void
GSMgetNum (void)
{
	int ctr1;
	
	// Request phone number
	UART1printf("AT+CNUM\r");
	ctr1 = GSMgetResponse();
	
	// Break if we got the network response
	if ( ctr1 == 0 ) 
	{
		UART0printf("\n\r> ERRROR GETTING NUMBER!");
		return;
	}
	
	// Parse the response
	for ( int ctr2 = 0; ctr2<= ctr1; ctr2++ )
	{
		// Response looks like:
		// +CNUM: "","16086920935",129,7,4
		if ( strstr(responseLine[ctr2],"+CNUM: ") != '\0' )
		{
			// Start parsing
			msgContent = responseLine[ctr2];
			
			// Go to first comma, skipping status
			msgSender = strtok(msgContent,",");
			
			// Grab the number
			msgSender = strtok(NULL,",");
			
			// Store the number (with null terminator)
			strncpy(SIMID,msgSender+1,11);
			SIMID[11] = '\0';
		}
	}
	UART0printf("\n\r> Number: %s",SIMID);
}

//*****************************************************************************
//
// PROCESS SMS FOR ENVELOPE AND CONTENT 
// TO DO: 
// 1. Figure out if it's OK to do this with pointers
//
//*****************************************************************************
void 
GSMprocessMessage( int msgNum )
{
	bool msgPresent[4] = {0000};	// Flag to ignore deleted messages
	bool msgVerify = false;			// Flag for message error checking
	char msgErrorCheck[4][225];		// Holder for message error checking
	int lineCount;					// Hold the number of lines
	int oLoop;						// Counter for outside error checking loop
	int iLoop;						// Counter for inside error checking loop
	
	// Start message retrieval/parsing/error checking (runs simultaneously to
	// reduce calls to the SIM module).
	for ( oLoop=0; oLoop<3; oLoop++ )
	{
		// Request the message and get the lines of the response (includes 
		// envelope, nulls, SIM responses)
		UART1printf("AT+CMGR=%u\r\n",msgNum);
		lineCount = GSMgetResponse();
	
		// Delay for a bit, needed when processing multiple messages (maybe?)
		ROM_SysCtlDelay(ROM_SysCtlClockGet()/160);
	
		// Make sure there's message content, process for envelope and content
		msgPresent[oLoop] = GSMparseMessage( lineCount );
		
		// If there is a message, store it, see if it matches previous retrieval loop
		if (msgPresent[oLoop])
		{
			// Store the message to one big string
			strcpy(msgErrorCheck[oLoop], msgSender);
			strcat(msgErrorCheck[oLoop], msgDate);
			strcat(msgErrorCheck[oLoop], msgTime);
			strcat(msgErrorCheck[oLoop], msgContent);
			
			// Check that string against previous copies from outer loop
			for ( iLoop = 0; iLoop < oLoop; iLoop++ ) 
			{
				if (strstr(msgErrorCheck[oLoop],msgErrorCheck[iLoop]) != NULL){
					
					// Set a flag to use for exiting outer loop
					msgVerify = true;
					
					// Exit inner loop
					break;
				}
			}
		}
		
		// If there's no message, exit retrieval loop
		else { break; }
		
		// If we verified the message, exit retrieval loop
		if (msgVerify) { break; }
	}
	
	// Show the user what we found
	UART0printf("\n\r>>> MESSAGE %u:",msgNum);
	if ( msgPresent[oLoop] && msgVerify ) {
		UART0printf("\n\r> FROM: %s ON: %s AT: %s",msgSender,msgDate,msgTime);
		UART0printf("\n\r> TEXT: %s",msgContent);
	}
	else if ( !msgPresent[oLoop] ) { UART0printf("\n\r> NOT PRESENT!"); }
	else { UART0printf("\n\r> COULD NOT VERIFY!"); }
	
	// Delete the message
	if ( testDelete && msgPresent ){
		UART1printf("AT+CMGD=%u\r\n",msgNum);
		GSMgetResponse();
		UART0printf ( "\n\r>>> MESSAGE %u DELETED",msgNum );
	}
}

//*****************************************************************************
//
// PARSE GSM MESSAGE FOR ENVELOPE AND MESSAGE CONTENT
// Stores message envelope and constant to global variables, OR returns true
// for message present, false for no message
//
//*****************************************************************************
bool
GSMparseMessage( int lineCount )
{
	int activeLine = 1;				// Counter for line being processed
	char *msgEnvelope = NULL;		// Message envelope holder
	const char commaCh[] = ",";		// Comma character
	
	// Clear out the old message
	msgContent = NULL;
	
	// Parse the new message
	while ( activeLine < lineCount+1 )
	{
		// CASE FOR ENVELOPE (which will look like:)
		// +CMGR: "REC READ","+13158078555","","15/10/08,13:18:40-20"
		if ( strstr(responseLine[activeLine],"+CMGR:") != '\0' )
		{
			// Start parsing
			msgEnvelope = responseLine[activeLine];
			
			// Go to first comma, skipping status
			msgSender = strtok(msgEnvelope,",");
			
			// Grab the number
			msgSender = strtok(NULL,commaCh);
			
			// Go to next comma, skipping phonebook entry
			msgDate = strtok(NULL,commaCh);
			
			// Grab the date
			msgDate = strtok(NULL,commaCh);
			
			// Grab the time
			msgTime = strtok(NULL,commaCh);
			
			// Store the number (with null terminator)
			strncpy(msgSender,msgSender+2,11);
			msgSender[11] = '\0';
			
			// Store the date (with null terminator)
			strncpy(msgDate,msgDate+1,8);
			msgDate[8] = '\0';
			
			// Store the time (with null terminator)
			strncpy(msgTime,msgTime,8);
			msgTime[8] = '\0';
		}
		
		// CASE FOR MESSAGE CONTENT
		// If we already found the envelope, and the line's not blank...
		else if ( msgEnvelope != NULL && responseLine[activeLine] != NULL )
		{
			// ... and we haven't found any content, this is the first line.
			if (msgContent == NULL) { msgContent = responseLine[activeLine]; }
			
			// ... otherwise, add a space and append this line.
			else if ( activeLine + 2 <= lineCount ) {
				strcat(msgContent, " ");
				strcat(msgContent, responseLine[activeLine]);
			}
		}
		
		// Proceed to next line
		activeLine++;
	}
	// If we didn't find an envelope, there's no message
	if (msgEnvelope == NULL) { return false; }
	
	// Otherwise, return true.
	else { return true; }
}

//*****************************************************************************
//
// GSM - SEND AN SMS
//
//*****************************************************************************
bool 
GSMsendSMS(char *destNbr, char *msgBody)
{
	bool msgSent = false;			// Flag for confirming message was sent
	static char g_cInput[128];		// String input to a UART
	
	// Initialize with phone number
	UART1printf( "AT+CMGS=\"%s\"\r",destNbr);
	
	// Wait a bit
	ROM_SysCtlDelay(ROM_SysCtlClockGet()/20);
	
	// Enter message body
	UART1printf( "%s\r", msgBody );
	
	// Wait a bit
	ROM_SysCtlDelay(ROM_SysCtlClockGet()/2000);
	
	// Enter ctrl+Z to send
	UART1printf( "\x1A" );

	// Loop to wait for message sent confirmation
	while (msgSent == false){
		UART1gets(g_cInput,sizeof(g_cInput));
		if ( strstr(g_cInput,"OK") != '\0' ){ 
			UART0printf ( "\n\r> Message sent to %s: %s",destNbr,msgBody );
			msgSent = true; 
		}
	}
	
	// Return status
	if (msgSent) { return true; }
	else { return false; }
}

//*****************************************************************************
//
// GSM - CHECK SIGNAL STRENGTH
//
//*****************************************************************************
int 
GSMcheckSignal(void)
{
	int lineCount;					// Count of lines from GSM response
	int lineActive = 1;				// Keep track of which line we're checking
	char ch_sigStrength[20] = "\0";	// char value of signal strength
	int i_sigStrength;				// int value of signal strength
	
	// Ask GSM module for the signal strength
	UART1printf("AT+CSQ\r");
	
	// Store to responseLine[] array, get line count
	lineCount = GSMgetResponse();
		
	while ( lineActive < lineCount-1 )
	{
		// Find the strength in the lines we get back
		if ( strstr(responseLine[lineActive],"+CSQ: ") != '\0' ){
			strncpy(ch_sigStrength,responseLine[lineActive]+6,4);
			
			// Parse for signal strength
			sscanf(ch_sigStrength,"%d,", &i_sigStrength);
		}
		lineActive++;
	}
	
	// Strength ranges from 0-31: normalize to 0-100
	i_sigStrength = i_sigStrength *100 /31;
	
	// Update to LCD
	snprintf (ch_sigStrength,20,"%d%%", i_sigStrength);
	LCDstring(0,14*6,ch_sigStrength,NORMAL);
	
	// Return value
	return i_sigStrength;
}

//*****************************************************************************
//
// GSM - CHECK BALANCE
//
//*****************************************************************************
void 
GSMcheckBalance(void)
{
	bool readResponse = true;		// Keeps the loop open while getting message
	char *GSMresponse = NULL;		// Use to grab input
	static char g_cInput[128];		// String input to a UART
	char expiry[] = "00/00/00";		// Expiry date
	char currency[] = "USD";		// Balance currency
	char *ptr;						// Pointer for parsing balance message 
	int ctr=0;						// For stepping through tokens
	
	// Ask GSM module for balance
	UART1printf("ATD*777#\r");
	
	// Retrieve balance and expiry date from GSM:
	// The GSM module treats this as a message, not a response, so we have 
	// to do this a bit differently than the GSMgetResponse function.
	// +CUSD: 0,"Your account balance is 49.10 USD and will expire on 04/03/16.",64
	while (readResponse && ctr < 20)
	{
		// Grab a line
		UART1gets(g_cInput,sizeof(g_cInput));
		
		// Stop after newline
		GSMresponse = strtok(g_cInput,"\n");
		
		// If this line says "expire on" it contains the balance
		if ( strstr(GSMresponse,"expire on") != '\0' ){readResponse = false;}
		
		// Use this counter to break out of loop if necessary
		ctr++;
	}

	// Parse for data of interest
	ctr = 0;
	ptr = strtok (GSMresponse," ");
	while (ptr != NULL){
		ptr = strtok (NULL," ");
		if (ctr == 4) { strcpy(balance,ptr); }
		else if (ctr == 5) { strcpy(currency,ptr); }
		else if (ctr == 10) { strncpy(expiry,ptr,8); }
		ctr++;
	}
	
	// Notify user
	UART0printf("\n\r> GSM BALANCE: %s %s expires on %s",balance,currency,expiry);
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
	//reset module
	ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

	// Configure the pin muxing for I2C0 functions on port B2 and B3.
	ROM_GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	ROM_GPIOPinConfigure(GPIO_PB3_I2C0SDA);

	// Select the I2C function for these pins.
	ROM_GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
	ROM_GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

	// Enable and initialize the I2C0 master module.  Use the system clock for
	// the I2C0 module.  The last parameter sets the I2C data transfer rate.
	// If false the data rate is set to 100kbps and if true the data rate will
	// be set to 400kbps.
	ROM_I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

	//clear I2C FIFOs
	HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
	
	// Enable the I2C interrupt:
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
	if (keysUnlocked) 
	{
		// Lock if unlocked
		keysUnlocked = false;
		
		// Turn on the red LED
		GPIOPinWrite(GPIO_PORTF_BASE, LED_MAP, 0);
		GPIOPinWrite(GPIO_PORTF_BASE, RD_LED, RD_LED);
		
		// Clear the line of input keys and set column to beginning
		LCDstring(6,0,"                 ", NORMAL);
		curCol = XPIXEL;
		
		// Update status on LCD
		LCDstring(7,0,"KEYPAD LOCKED   #", INVERSE);
		
		// Start timer 1A, the idle keypad timer:
		ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, ROM_SysCtlClockGet()* 15);
		ROM_TimerEnable(TIMER1_BASE, TIMER_A);
		
		// Store relay status to EEPROM (only done at lock to save EEPROM)
		if (testEEPROM) { relayStatusE2(); }
	}
	else 
	{
		// Unlock if locked
		keysUnlocked = true;
		
		// Turn on the green LED
		GPIOPinWrite(GPIO_PORTF_BASE, LED_MAP, 0);
		GPIOPinWrite(GPIO_PORTF_BASE, GN_LED, GN_LED);
		
		// Update status on LCD
		LCDstring(7,0,"KEYPAD UNLOCKED #", NORMAL);
		
		// Start timer 1A, the idle keypad timer:
		ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, ROM_SysCtlClockGet()* 15);
		ROM_TimerEnable(TIMER1_BASE, TIMER_A);
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
	uint32_t trashBin[1] = {0};		// For clearing the buffer
	
	// Configure GPIO Pins for SSI3 mode.
	ROM_GPIOPinConfigure(GPIO_PD0_SSI3CLK);
	ROM_GPIOPinConfigure(GPIO_PD1_SSI3FSS);
	// ROM_GPIOPinConfigure(GPIO_PD2_SSI3RX); // RX pin not present on LCD
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
	uint32_t trashBin[1] = {0};		// For clearing the buffer
	
	// Send the data
	SSIDataPut(SSI3_BASE, theData);
	
	// Wait for transmission to finish
	while(SSIBusy(SSI3_BASE)) {}
	
	// Clear SSI0 RX Buffer (not sure if this is needed)
	while (ROM_SSIDataGetNonBlocking(SSI3_BASE, &trashBin[0])) {}
}

//*****************************************************************************
//
// INITIALIZE LCD
// PD0 - SSI3CLK	clock
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
	
	// Set up PD7 as reset 
	// NOTE: PD7 is NMI by default, so unlike  other pins, this procedure must 
	// be followed in order to make the pin usable as GPIO.
	// Unlock the port: 
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	
	// Unlock the pin
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= GPIO_PIN_7;
	HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~GPIO_PIN_7;  
	HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= GPIO_PIN_7;
	
	// Re-lock the port
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
	
	// Configure the pin for output
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_7);
	
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
	// Set command pin for data/command
	if (cmdByte == LCD_CMD){ GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0); }
	else { GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); }
	
	// Send the data
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
	// Set page address
	LCDsend(0xB0+page,LCD_CMD);
	
	// Set LSB of column address
	LCDsend(0x0F&column,LCD_CMD);
	
	// Set MSB of column address
	LCDsend(0x10+(column>>4),LCD_CMD);
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

//!*****************************************************************************
//! THE PROGRAM
//!*****************************************************************************
int
main(void)
{
	char aString[2][128];				// Generic string
	int anInt;						// Generic int
	int msgOpen = 0;					// Message being processed
	int ctr1;							// Generic counter
	uint32_t pui32ADC0Value[1];			// ADC0 data value
	uint32_t ui32D0v;					// mV value on external input D0
	
	// Initial settings - from Anil
	ROM_FPUEnable();					// Enable floating point unit
	ROM_FPULazyStackingEnable();		// Enable lazy stacking of FPU
	ROM_IntMasterEnable();				// Enable processor interrupts
	
	// Enable device clocking
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	
	// Enable peripherals
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);		// ADC1
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);	// EEPROM (2048 bytes in 32 blocks)
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);	// Pins: UART0 
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);	// Pins: UART1, GSM, Relays, I2C0SCL & SDA
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);	// Pins: Neopixel, keypad INT2
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);	// Pins: LCD screen
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);	// Pins: Relays
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);	// Pins: RGB LED, Relays
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);		// I2C for MPR121 touchpad controller
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);		// SSI3 for EA DOGS102W6 LCD display
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);	// Timer for keylock
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);	// Timer for keypad timeout
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);	// Console UART
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);	// GSM UART
    
	// Configure GPIO outputs
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);		// Rel3N
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6);		// GSM PWRKEY
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);		// GSM RESET
	//ROM_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);	// Neopixel
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2);		// Rel3
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_3);		// Rel2
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_5);		// Rel2N
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);	// RGB LED
	if (hwRev == 1) {
		// ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);	// Rel4
		// ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);	// Rel1N
		// ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);	// Rel1  (conflict with red LED)
		// ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);	// Rel4N (conflict with USR SW1)
	}
	else if (hwRev == 2) {
		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0);		// Rel4N
		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);		// Rel1N 
		
		// Disable NMI on PF0
		HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;		// Unlock the port
		HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_0;			// Unlock the pin
		HWREG(GPIO_PORTF_BASE + GPIO_O_AFSEL) &= ~GPIO_PIN_0;  
		HWREG(GPIO_PORTF_BASE + GPIO_O_DEN) |= GPIO_PIN_0;
		HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;					// Lock the port
		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);		// Rel1 (conflict with USR SW2)
		
		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);		// Rel4 (conflict with USR SW1)
	}
	
	// Turn on an LED to show that we're working
	GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, BL_LED);
	
	// Start I2C module (for keypad)
	initI2C();
	
	// Start the MPR121 (keypad controller) and set thresholds (do this early 
	// since it takes a moment to calibrate)
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

	// Start SSI3 and the LCD display
	initSSI3();
	initLCD();
	
	// Console UART0: Set PA0 and PA1 as UART0, configure for 115200, 
	// 8-N-1 operation, enable interrupts
	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UART0StdioConfig(0, 115200, 16000000);
	ROM_IntEnable(INT_UART0);
	ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);  

	// Notify the user what's going on
	UART0printf("\n\n\n\r>>> INITIALIZING");

	// Get MCU ID
	FlashUserGet(&boardID1,&boardID2);
	
	// Print to LCD (along with header)
	LCDstring(0,0,"MCU/IMEI/ON@/NUM:",INVERSE);
	snprintf (aString[1],18,"%X-%X",boardID1,boardID2);
	LCDstring(1,0,aString[1],NORMAL);
	LCDstring(2,0,"...",NORMAL);

	// GSM UART1: Set PB0 and PB1 as UART1, configure for 115200, 
	// 8-N-1 operation, enable interrupts
	ROM_GPIOPinConfigure(GPIO_PB0_U1RX);
	ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
	ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UART1StdioConfig(1, 115200, 16000000);
	ROM_IntEnable(INT_UART1);
	ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);  
	
	/// GSM TEST AREA
	if (testGSM)
	{
		// See if the GSM module is on: try three times to power up
		GSMcheckPower(3);
		
		// Request and print the IMEI (s/n) of the SIM module (used as board s/n)
		GSMgetIMEI();
		LCDstring(2,0,IMEI,NORMAL);
		LCDstring(3,0,"...",NORMAL);
		
		// Request SIM card status
		GSMgetSIMstatus();
		
		// Get the time. Use a loop here since the time doesn't always come 
		// through on the first try after powering up. Make ctr1 attempts.
		ctr1 = 10;
		YY = 2000;
		while (YY == 2000 && ctr1 > 0)
		{
			// Turn off the LED to indicate what's going on
			GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, 0);
			
			// Check the time, decrement counter
			GSMcheckTime();
			ctr1--;
			
			// Wait a bit, turn the LED back on
			ROM_SysCtlDelay(ROM_SysCtlClockGet()/3);
			GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, BL_LED);
		}
		
		// Print the on-time
		UART0printf("\n\r> On-time from GSM: %s",fullOnTime);
		LCDstring(3,0,fullOnTime,NORMAL);
		LCDstring(4,0,"...",NORMAL);
		
		// If SIM card is present, get the phone number and balance
		if ( SIMpresent ) 
		{ 
			// Get number
			GSMgetNum(); 
			
			// Get balance
			GSMcheckBalance();
		}
		
		// Print phone number / SIM status to LCD
		LCDstring(4,0,SIMID,NORMAL);
	}
	
	/// ADC TEST AREA - start the ADC
	if (testADC){ ADCinit(); }
	
	// Notify the user what testing functions are active
	UART0printf("\n\r> ----------Testing function status:----------");
	if (testGSM) { UART0printf("\n\r> ENABLED : GSM power at boot"); }
	else {UART0printf("\n\r> DISABLED: GSM power at boot");}
	if (testEEPROM) { UART0printf("\n\r> ENABLED : Store/retrieve ontime from EEPROM"); }
	else {UART0printf("\n\r> DISABLED: Store/retrieve ontime from EEPROM");}
	if (testDelete) { UART0printf("\n\r> ENABLED : Delete messages during processing"); }
	else {UART0printf("\n\r> DISABLED: Delete messages during processing");}
	if (testNotify) { UART0printf("\n\r> ENABLED : Message controller at boot"); }
	else {UART0printf("\n\r> DISABLED: Message controller at boot");}
	if (testADC) { UART0printf("\n\r> ENABLED : Test ADC"); }
	else {UART0printf("\n\r> DISABLED: Test ADC");}
	UART0printf("\n\r> --------------------------------------------");
	
	// Initialize the SysTick interrupt to process buttons
	ButtonsInit();
	SysTickPeriodSet(SysCtlClockGet() / APP_SYSTICKS_PER_SEC);
	SysTickEnable();
	SysTickIntEnable();
	
	// Notify the user about buttons
	UART0printf("\n\r> LEFT BUTTON:  Enter \"talk to GSM\" mode (blue LED). Updates signal strength.");
	UART0printf("\n\r> RIGHT BUTTON: Toggle power to GSM module (red LED).");
	
	/// EEPROM TEST AREA: Store on-time, retrieve last on-time. 
	// Don't run this each time 'cause EEPROM wears out.
	if (testEEPROM) 
	{
		EEPROMInit();

		struct E2S_TIME E2writeTime = {YY,MM,DD,hh,mm,ss,zz};
		
		//Read from struct at EEPROM start from 0x0000
		EEPROMRead((uint32_t *)&E2readTime, E2A_ONTIME, sizeof(E2readTime));
		UART0printf("\n\r> Last on-time: %u/%u/%u, %u:%u:%u, %d", E2readTime.E2YY, E2readTime.E2MM, E2readTime.E2DD, E2readTime.E2hh, E2readTime.E2mm, E2readTime.E2ss, E2readTime.E2zz, E2A_ONTIME);
		
		//Write struct to EEPROM start from 0x0000
		EEPROMProgram((uint32_t *)&E2writeTime, E2A_ONTIME, sizeof(E2writeTime));
	
		// Some EEPROM functions
		/*esize = EEPROMSizeGet(); // Get EEPROM Size 
		UART0printf("E2> EEPROM Size %d bytes\n", e2size);
		eblock = EEPROMBlockCountGet(); // Get EEPROM Block Count
		UART0printf("E2> EEPROM Blok Count: %d\n", e2block);*/
		
	}
	
	// Wait for user to proceed
	LCDstring(6,0,"Touch keypad to  continue",NORMAL);
	UART0printf("\n\r> Touch keypad to continue");
	while ( GPIOIntStatus(GPIO_PORTC_BASE,GPIO_PIN_7) == 0 ) {}
	
	// Clear the LCD and set up for normal use:
	LCDclear(0,0,XMAX,YMAX);

	// Print relay status:
	if ( !testEEPROM ) { relaySet(0); }
	else 
	{
		// Read the last relay status from EEPROM
		EEPROMRead(&E2relayStatus, E2A_RELAY_STATUS, sizeof(E2relayStatus));
		
		// Notify user
		UART0printf("\n\r> Relay status from EEPROM (%X): ",E2relayStatus);
		for ( int r=0; r<4; r++ ) { UART0printf("%u",(E2relayStatus >> r) & 1); }
		
		// Get only the LSB of E2relayStatus
		E2relayStatus &= 0x000F;
		
		// Toggle each bit in relayStatus in order to properly update the display
		relayStatus = 15-E2relayStatus;
		UART0printf("\n\r> 15-E2relayStatus = %u",relayStatus);
		
		// Apply the previous relay states
		relaySet(E2relayStatus);
	}
	
	// Get the GSM signal strength and print to LCD (along with balance)
	if (testGSM) 
	{ 
		GSMcheckSignal(); 
		
		// Print balance to LCD
		LCDstring(1,(18-sizeof(balance))*6,"$",NORMAL);
		LCDstring(1,(19-sizeof(balance))*6,balance,NORMAL);
	}
	
	/// CONTROLLER NOTIFY
	if (testNotify && SIMpresent){ 
		snprintf(aString[1],83,"MCU %X-%X IMEI %s OT %s BAL %s",boardID1,boardID2,IMEI,fullOnTime,balance);
		GSMsendSMS( ctrlID, aString[1] ); 
	}
	
	// Disable talk mode (was letting GSM notifs in during setup)
	talkMode = false;
	
	/// SETUP COMPLETE!
	UART0printf("\n\r> Setup complete! \n\r>>> RUNNING MAIN PROGRAM");
	GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, 0);
	
	// Lock keypad
	MPR121toggleLock();
	LCDstring(7,0,"SETUP COMPLETE!  ", NORMAL);
	
	/// MAIN LOOP - 
	// 1. Wait for new message notification and process. 
	// 2. Update ADC.
	while(1){
		// Process new messages.
		if (msgCount > 0)
		{
			// Start working on the oldest message
			msgOpen = msgCount;
			msgCount--;
			
			// Process message for envelope and content
			GSMprocessMessage(msgOpen);
			
			// If message content is good, act on message
			if (strstr(msgSender,ctrlID) != NULL && strlen(msgContent) == 4) {
				for ( ctr1=0;ctr1<4;ctr1++ ){
					if ( msgContent[ctr1] == '1' ) { anInt |= 1 << ctr1; }
					else if (msgContent[ctr1] == '0' ) { anInt &= ~(1 << ctr1); } 
				}
				relaySet(anInt);
			}
			
			// After the last new message, update the balance and EEPROM
			if ( msgCount == 0 ) 
			{ 
				GSMcheckBalance();
				relayStatusE2();
			}
		}

		// Run the ADC
		if ( testADC && msgCount == 0 ) {
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
			
			// Convert to a string (in volts, three decimal places)
			snprintf (aString[1],7,"%d.%03dV", ui32D0v / 1000, ui32D0v % 1000);

			// Display the AIN0 (PE0) digital value on the console.
			LCDstring(2,11*6,aString[1],NORMAL);

			// Wait a bit
			ROM_SysCtlDelay(ROM_SysCtlClockGet()/4);
		}
	}
	//return(0);
}

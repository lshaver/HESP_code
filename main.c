// These are included by the system and/or Anil.
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

// Lee added these
#include "uart0stdio.h"		// Console
#include "uart1stdio.h"		// GSM
#include "string.h"
#include "stdio.h"
#include <stdlib.h>
#include "buttons.h"
#include "driverlib/systick.h"
#include "eeprom.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#define RD_LED GPIO_PIN_1
#define BL_LED GPIO_PIN_2
#define GN_LED GPIO_PIN_3

// Debug/testing functions:
bool testEEPROM = false;	// Store the on-time and retrieve previous ontime from EEPROM
bool testRelay = false;		// Cycle the relays on/off to test
bool msgDelete = false;		// Delete messages during processing
bool ctrlNotify = false;	// Text the controller when coming on-line

// For driving buttons
uint32_t ui32Buttons;
uint32_t ui32Data;
#define APP_SYSTICKS_PER_SEC            32
#define APP_BUTTON_POLL_DIVIDER          8
#define APP_HIB_BUTTON_DEBOUNCE          (APP_SYSTICKS_PER_SEC * 3)
static volatile uint32_t ui32HibModeEntryCount;

// Global variables
bool GSM_off = true;				// Flag to see if the GSM module is on/off
bool talkMode = true;				// Mode for user to interface directly with GSM module
int LEDhold;						// Holds status of RGB LED so it can be restored after blinking
int LEDmap;							// Map of the RGB LED bits (works with LEDhold, ie store LEDhold to LEDmap)
static char g_cInput[128];			// String input to a UART
char *GSMresponse = NULL;			// Use to parse UART inputs
volatile uint32_t ui32Loop;			// for time delays
char ctrlID[11] = "3158078555";		// Phone number of the controller
char boardID[5] = "0001";			// ID of this board

// Variables for processing messages
const char newline[] = "\n";	// Newline character
const char comma[] = ",";		// Comma character
int msgOpen = 0;				// Keep track of which message we're processing
char *msgSender = NULL;			// Message sender
char *msgDate = NULL;			// Message date
char *msgTime = NULL;			// Message time


// Variables used by GSM and console interrupts
unsigned char var;					// Incoming UART character
unsigned char ptr[10000];			// Array for storing incoming UART characters
unsigned long i;					// UART character pointer. There was a j here but I don't think it's used.
unsigned long ulStatus0,ulStatus1;	// To hold the interrupt status
char *msgCountStr;					// Hold the string with the number of new messages
int msgCount = 0;					// Hold the integer value with number of new messages

// For EEPROM
#define E2PROM_TEST_ADRES 0x0000 
uint32_t esize,eblock;

struct eprom_timestamp
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

// Generic variables
int j,k;				// Generic counter
char aString[2][128];	// Generic string

// Checktime function writes to these variables for main program access
int hh,mm,ss,dd,mo,yy,zz;
char fullTimestamp[23] = "\0";

// REQUIRED.
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

// User authorization checker. For now, just confirms if user is on a local area code.
bool aCodeauth(char *incoming_number){
    return(incoming_number[1] == '6' && incoming_number[2] == '0' && incoming_number[3] == '8') ? true : false;
}

// Send an SMS
void sendsms(char *recipient, char *body)
{
	bool msgSent = false;		// Flag for confirming message was sent
	
	UART1printf( "AT+CMGS=\"%s\"\r",recipient);
	for(ui32Loop = 0; ui32Loop < 900000; ui32Loop++){}
	UART1printf( "%s\r", body );
	for(ui32Loop = 0; ui32Loop < 900000; ui32Loop++){}
	UART1printf( "\x1A" );
	while (msgSent == false){
		UART1gets(g_cInput,sizeof(g_cInput));
		if ( strstr(g_cInput,"OK") != '\0' ){ 
			UART0printf ( "\n\r> Message sent" );
			msgSent = true; 
		}
	}
}

// Turn on the relay(s)
void on(int relay)
{
    if (relay == 1 || relay == 0){
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);	// Rel1
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0);			// Rel1N
	}
	if (relay == 2 || relay == 0){
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_PIN_3);	// Rel2
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0);			// Rel2N
	}
	if (relay == 3 || relay == 0){
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);	// Rel3
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0);			// Rel3N
	}
	if (relay == 4 || relay == 0){
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);	// Rel4
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);			// Rel4N
	}
}

// Turn off the relay(s)
void off(int relay)
{
    if (relay == 1 || relay == 0){
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);			// Rel1
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);	// Rel1N
	}
	if (relay == 2 || relay == 0){
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 0);			// Rel2
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5);	// Rel2N
	}
	if (relay == 3 || relay == 0){
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);			// Rel3
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);	// Rel3N
	}
	if (relay == 4 || relay == 0){
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);			// Rel4
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);	// Rel4N
	}
}

// Clear received buffer (I don't know what this is for, but it looks important)
/*void clear(void){
    for(j=0; j<i; j++)
        ptr[j]=0;
}*/

// CONSOLE UART interrupt handler.
void
UARTIntHandler0(void)
{
    // Get the interrupt status.
    ulStatus0 = ROM_UARTIntStatus(UART0_BASE, true); // CONSOLE
    
    // Clear the asserted interrupts.
    ROM_UARTIntClear(UART0_BASE, ulStatus0);

    // Loop while there are characters in the receive FIFO.
    while(ROM_UARTCharsAvail(UART0_BASE))
    {
        var = (unsigned char)ROM_UARTCharGetNonBlocking(UART0_BASE);
        ptr[i] = var;    
        ROM_UARTCharPutNonBlocking(UART1_BASE, ptr[i]);
        i++;
	}
}

// GSM UART interrupt handler.
void
UARTIntHandler1(void)
{
	// Get the interrupt status.
	ulStatus1 = ROM_UARTIntStatus(UART1_BASE, true); // GSM
	
	// Clear the asserted interrupts.
	ROM_UARTIntClear(UART1_BASE, ulStatus1);
	
	// When the interrupt is triggered, this means the GSM module is on.
	if ( GSM_off ) { GSM_off = false; }

	while(ROM_UARTCharsAvail(UART1_BASE)){
		var = (unsigned char)ROM_UARTCharGetNonBlocking(UART1_BASE);
		ptr[i] = var;	
		if (talkMode){ROM_UARTCharPutNonBlocking(UART0_BASE, ptr[i]);}
		else {
			// See if this is a new message flag - notifier looks like: +CMTI: "SM",12
			// where 12 is the number of unread messages
			if(ptr[i-3] == 'C' && ptr[i-2] == 'M' && ptr[i-1] == 'T'&& ptr[i] == 'I'){
				// Grab everything after the notification, stop at newline
				UART1gets(g_cInput,sizeof(g_cInput));
				msgCountStr = strtok(g_cInput,newline);
				// Parse out the message counter
				strncpy(msgCountStr,msgCountStr+7,3);
				msgCountStr[3]='\0';
				sscanf(msgCountStr, "%d", &msgCount);
				UART0printf("\n\r>>> %u NEW MESSAGES",msgCount);
			}
		}
		i++;
	}
	
	// If we're in "talk to GSM" mode, we don't want this interrupt to continue
	// (I don't think we need this, going to test without it for a while.)
	// if( talkMode ){return;}
}

// Find out if the GSM module is on
bool GSMpower (void)
{
	// Delay for a bit, then make sure the flag is ON (something seems to trigger the interrupt when I'm setting it up)
	for(ui32Loop = 0; ui32Loop < 990000; ui32Loop++){}
	GSM_off = true;
	// Then, send an AT to the GSM module - if it triggers the interrupt, we know the GSM is on
	UART1printf("AT\r");
	// Delay for a bit.
	for(ui32Loop = 0; ui32Loop < 900000; ui32Loop++){}
	if ( GSM_off ) { return false; }
	return true;
}

// Toggle power to the GSM module
void GSMpowerToggle( void )
{     
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
	if (GSMpower()){ UART0printf("\n\r--- GSM power ON!"); }
	else { UART0printf("\n\r--- GSM power OFF!"); }
}

// Blink an LED: 1=red, 2=blue, 3=green
void blink (int blinkcount, int blinkspeed, int LEDcolor)
{
	int m;
	volatile uint32_t blinkLoop;
	
	// Store current LED status, then turn them all off
	LEDhold = GPIOPinRead(GPIO_PORTF_BASE, LEDmap);
	GPIOPinWrite(GPIO_PORTF_BASE, LEDmap, 0);

	for( m=blinkcount; m>=0; m--)
    {
		// Turn on the LED.
	    if ( LEDcolor == 1 )      {GPIOPinWrite(GPIO_PORTF_BASE, RD_LED, RD_LED);}
		else if ( LEDcolor == 2 ) {GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, BL_LED);}
		else if ( LEDcolor == 3 ) {GPIOPinWrite(GPIO_PORTF_BASE, GN_LED, GN_LED);}
		
		// Delay for a bit.
		for(blinkLoop = 0; blinkLoop < blinkspeed; blinkLoop++){}
		
		// Turn off the LED.
		GPIOPinWrite(GPIO_PORTF_BASE, LEDmap, 0);

        // Delay for a bit.
        for(blinkLoop = 0; blinkLoop < blinkspeed; blinkLoop++){}
    }
	
	// Restore prior LED status
	GPIOPinWrite(GPIO_PORTF_BASE, LEDmap, LEDhold);
}

// Handles button press events
void
SysTickIntHandler(void)
{
	// Get the button state
	ui32Buttons = ButtonsPoll(0,0);
	static uint32_t ui32TickCounter;
    ui32TickCounter++;

    switch(ui32Buttons & ALL_BUTTONS)
    {
    case LEFT_BUTTON:
        // Check if the button has been held int32_t enough to act
        if((ui32TickCounter % APP_BUTTON_POLL_DIVIDER) == 0)
        {
			// Switch modes depending on current state
			if (talkMode == false){
				// Get LED status and store for when we switch back. Turn on blue LED only.
				LEDhold = GPIOPinRead(GPIO_PORTF_BASE, LEDmap);
				GPIOPinWrite(GPIO_PORTF_BASE, LEDmap, 0);
				GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, BL_LED);
				UART0printf("\n\r> [Left button] Entering talk to GSM mode. Press left button to end.");
				talkMode = true;
			}
			else{
				UART0printf("\n\r> [Left button] Returning to main program.");
				GPIOPinWrite(GPIO_PORTF_BASE, LEDmap, LEDhold);		//Restore LED status
				talkMode = false;
			}
        }
        break;

    case RIGHT_BUTTON:
        // Check if the button has been held int32_t enough to act
        if((ui32TickCounter % APP_BUTTON_POLL_DIVIDER) == 0)
        {
			blink(3,200000,1);
			UART0printf("\n\r> [Right button] Cycling power to GSM.");
			GSMpowerToggle();
        }
        break;

    case ALL_BUTTONS:
        // Both buttons for longer than debounce time
        if(ui32HibModeEntryCount < APP_HIB_BUTTON_DEBOUNCE)
        {
			UART0printf("\n\r> [Both buttons] No action defined.");
        }
        break;

    default:
        break;
    }
}

// Get the date/time from the GSM module. This breaks down right after power on. Need to improve.
void checktime(void)
{
	bool timeunknown = true;
	char msgLine[10][75];
	
	k = 1;
	UART1printf("AT+CCLK?\r");
	while (timeunknown){
		// Grab a line, stop after new-line
		UART1gets(g_cInput,sizeof(g_cInput));
		GSMresponse = strtok(g_cInput,newline);
		strcpy(msgLine[k], GSMresponse);
		// If this line says OK we've got the whole message
		if ( strncmp(msgLine[k],"OK",2) == 0 ){timeunknown = false;}
		else { k++; }
	}
	
	j = 1;
	while ( j < k-1 ){
		// Find the time in the lines we get back
		if ( strstr(msgLine[j],"+CCLK: \"") != '\0' ){
			strncpy(fullTimestamp,msgLine[j]+8,20);
			// Split to individual integers
			if (strstr(fullTimestamp,"+")){
				sscanf(fullTimestamp,"%d/%d/%d,%d:%d:%d+%d", &yy, &mo, &dd, &hh, &mm, &ss, &zz);
			}
			else{
				sscanf(fullTimestamp,"%d/%d/%d,%d:%d:%d-%d", &yy, &mo, &dd, &hh, &mm, &ss, &zz);
				zz = zz*-1;
			}
			yy = yy + 2000;
		}
		j++;
	}
}

// Process a message for envelope and content info
void msgProcess( int activeMsg )
{
	bool msgReading = true;		// This will keep message retrieval loop open
	bool msgPresent = true;		// Flag to ignore deleted messages
	char *msgEnvelope = NULL;	// Message envelope
	char *msgContent = NULL;	// Message content
	char msgLine[10][75];		// One line of the message
	
	// FIRST: Request the message and get the lines of the response (includes envelope, nulls, SIM responses)
	UART0printf("\n\r>>> PROCESSING MESSAGE %u",activeMsg);
	k = 1;		// Counts up while getting the lines of the message
	// Request the message
	UART1printf("AT+CMGR=%u\r\n",activeMsg);
	while (msgReading){
		// Grab a line, stop after new-line
		UART1gets(g_cInput,sizeof(g_cInput));
		msgContent = strtok(g_cInput,newline);
		strcpy(msgLine[k], msgContent);
		// If this line says OK we've got the whole message
		if ( strncmp(msgLine[k],"OK",2) == 0 ){msgReading = false;}
		else { k++; }
	}
	// Delay for a bit, needed when processing multiple messages
	for(ui32Loop = 0; ui32Loop < 100000; ui32Loop++){}
	
	// See if there's actually a message
	if ( k < 5 ){
		msgPresent = false;
		UART0printf("... doesn't exist");
	}
	
	// SECOND: Process message lines for envelope information, and to make the message into a string
	msgContent = NULL;
	// Now we've got the whole message, so let's parse it
	if (msgPresent){
		j = 1;
		while ( j < k-1 ){
			// Case for the envelope
			if ( strstr(msgLine[j],"+CMGR:") != '\0' ){
				// Parse the line for status, ph number, date, and time.
				msgEnvelope = msgLine[j];
				msgSender = strtok(msgEnvelope,comma);		// This way we skip status
				msgSender = strtok(NULL,comma);
				msgDate = strtok(NULL,comma);				// This way we skip phonebook entry
				msgDate = strtok(NULL,comma);
				msgTime = strtok(NULL,comma);
				strncpy(msgSender,msgSender+2,11);			// Store the number
				msgSender[11] = '\0';
				strncpy(msgDate,msgDate+1,8);				// Store the date
				msgDate[8] = '\0';
				strncpy(msgTime,msgTime,8);					// Store the time
				msgTime[8] = '\0';
			}
			// Case for message content
			// If we already found the envelope, and the line's not blank...
			else if ( msgEnvelope != NULL && msgLine[j] != NULL ){
				// ... and we haven't found any content, this is the first message line.
				if (msgContent == NULL) { msgContent = msgLine[j]; }
				// ... otherwise, add a space and append this line.
				else {
					strcat(msgContent, " ");
					strcat(msgContent, msgLine[j]);
				}
			}
			j++;
		}
		// Show the user what we found
		UART0printf("\n\r> msgENV: from: %s on: %s at: %s",msgSender,msgDate,msgTime);
		UART0printf("\n\r> msgTXT: %s",msgContent);
		
		// THIRD: delete the message, confirm it's gone
		if ( msgDelete ){
			UART1printf("AT+CMGD=%u\r\n",activeMsg);
			while (msgPresent){
				// Grab a line, check for OK
				UART1gets(g_cInput,sizeof(g_cInput));
				if ( strstr(g_cInput,"OK") != '\0' ){ 
					UART0printf ( "\n\r> MSG %u deleted",activeMsg );
					msgPresent = false; 
				}
			}
		}
	}
}
	
// Program guts.
int
main(void)
{
	// Initial settings
	ROM_FPUEnable();
	ROM_FPULazyStackingEnable();
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |    SYSCTL_XTAL_16MHZ);
	ROM_IntMasterEnable();                                      // Enable processor interrupts.
    
	// Enable peripherals
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);            //UART 0 pins
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);            //GSM: UART1: RX(0), TX(1), PWRKEY(7), GSM_RESET(6)
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);            //Neopixel
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);            //Relay 1 negative (4)
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);            //PORTF: Output LED's and relay 1
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);            //Turn on UART0 module on chip for debugging (PuTTY)
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);            //For GSM module
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);			//enable EEPROM: 2048bytes in 32 blocks
    
	// Set pins as GPIO outputs (disable relays one and four in current hardware version)
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);        //Rel3N
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6);        //GSM PWRKEY
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);        //GSM RESET
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);        //Neopixel
	//ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);        //Rel4
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2);        //Rel3
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_3);        //Rel2
	//ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);        //Rel1N
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_5);        //Rel2N
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);	//RGB LED (pin 1 also Rel1?)
	//ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);        //Rel4N
	LEDmap = RD_LED+BL_LED+GN_LED;													//Defines an RGB LED mapping

	// Turn on an LED to show we're working
	GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, BL_LED);

	//Turn the relays off initially
	off(0);

	// Set GPIO A0 and A1 as UART0 pins
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Set GPIO B0 and B1 as UART1 pins
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure the UART0 and UART1 for 115200, 8-N-1 operation
	// UART0 (console)
	UART0StdioConfig(0, 115200, 16000000);
	// UART1 (GSM)
	UART1StdioConfig(1, 115200, 16000000);
    
	// Enable the UART0 console interrupt for passing messages to the GSM
	ROM_IntEnable(INT_UART0);
	ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);   

	// Enable the UART1 GSM interrupt for passing messages to the console
	ROM_IntEnable(INT_UART1);
	ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);     

	// Notify the user what's going on
	UART0printf("\n\n\n\r>>> INITIALIZING");
	UART0printf("\n\r> Checking to see if GSM is on...");
	
	// Find out if the GSM module is on - check three times
	for (k=1; k<4; k++){
		UART0printf( "\n\r> Check %u of 3...\n\r", k );
		if ( GSMpower() )
		{
			blink(5,300000,2);
			UART0printf("\n\r> GSM is ON.");
			break;
		}
		else
		{
			UART0printf("\n\r> GSM is OFF, turning on...");
			GSMpowerToggle();
			
			// Delay for a bit, check again. Keep this delay long enough for GSM to get the time.
			for(ui32Loop = 0; ui32Loop < 3000000; ui32Loop++){}
			UART1printf("AT\r");
			if (GSM_off){ UART0printf("\n\r> Power on failed!"); }
		}
	}

	if ( k == 4 ){
		GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, 0);
		blink(10,300000,1);
		GPIOPinWrite(GPIO_PORTF_BASE, RD_LED, RD_LED);
		UART0printf("\n\r> Cannot power on the GSM!");
		UART0printf("\n\r>>> ENDING PROGRAM");
		return 0;
	}

	// Get the time from the GSM module and print to user (sometimes still hangs right after powering GSM)
	UART0printf("\n\r> Getting date/time from the GSM module");
	checktime();
	UART0printf("\n\r> Timestamp: %s",fullTimestamp);

	// Store on-time, retrieve last on-time. Don't run this each time 'cause EEPROM wears out.
	if (testEEPROM){
		EEPROMInit();
		struct eprom_timestamp eprom_writetime = {yy,mo,dd,hh,mm,ss,zz};
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
	
	// Relay control testing - flash green when turning on, red for off. Go through relays 1 - 4.
	if (testRelay) {
		UART0printf("\n\r> RELAY TESTING:");
		for(ui32Loop = 0; ui32Loop < 990000; ui32Loop++){}		// Delay for a bit
		k = 1;
		while (k <= 4){
			UART0printf("\n\r> RELAY %u: ON...",k);
			on(k);
			blink(7,300000,3);
			UART0printf("OFF");
			off(k);
			blink(7,300000,1);
			k++;
		}
	}
		
	// Initialize the SysTick interrupt to process buttons
	ButtonsInit();
	SysTickPeriodSet(SysCtlClockGet() / APP_SYSTICKS_PER_SEC);
	SysTickEnable();
	SysTickIntEnable();
	IntMasterEnable();
	
	UART0printf("\n\r> Setup complete! \n\r>>> RUNNING MAIN PROGRAM");
	GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GN_LED, GN_LED);
	
	// Tell the user about buttons
	UART0printf("\n\r> Press left button to enter \"talk to GSM\" mode (blue LED)");
	UART0printf("\n\r> Press right button to toggle power to GSM module (red LED)");
	talkMode = false;	// We were in talk mode during set-up to let SIM messages through
	
	// Notify controller that we're online
	if (ctrlNotify){
		strcpy (aString[1],"HESP");
		strcat (aString[1], boardID);
		strcat (aString[1], " ON: ");
		strcat (aString[1], fullTimestamp );
		sendsms( ctrlID, aString[1] );
	}
	
	while(1){
		// Process new messages. Need to turn this into a function once it's working so we can process any messages that came in while power was off.
		if (msgCount > 0){
			msgOpen = msgCount;		// Keep track of the message we're working on, in case the count updates
			msgCount--;
			for(ui32Loop = 0; ui32Loop < 10000; ui32Loop++){}
			msgProcess(msgOpen);
		}
	}
}

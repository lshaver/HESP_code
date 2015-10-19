//!*****************************************************************************
//! DEFINITIONS
//!*****************************************************************************

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
#include "uart0stdio.h"				// Console
#include "uart1stdio.h"				// GSM
#include "string.h"
#include "stdio.h"
#include <stdlib.h>
#include "buttons.h"
#include "driverlib/systick.h"
#include "eeprom.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "inc/hw_i2c.h"
//#include <stdarg.h>				// Not sure if needed

// RGB LED pin definitions
#define RD_LED GPIO_PIN_1
#define BL_LED GPIO_PIN_2
#define GN_LED GPIO_PIN_3

//!*****************************************************************************
//! VARIABLES
//!*****************************************************************************

// Debug/testing functions:
bool testEEPROM = false;			// Store the on-time and retrieve previous ontime from EEPROM
bool testRelay = false;				// Cycle the relays on/off to test
bool msgDelete = false;				// Delete messages during processing
bool ctrlNotify = false;			// Text the controller when coming on-line
bool testI2C = true;				// I2C testing area

// For driving buttons
uint32_t ui32Buttons;
uint32_t ui32Data;
#define APP_SYSTICKS_PER_SEC		32
#define APP_BUTTON_POLL_DIVIDER		8
#define APP_HIB_BUTTON_DEBOUNCE		(APP_SYSTICKS_PER_SEC * 3)
static volatile uint32_t ui32HibMMdeEntryCount;

// Global variables
bool GSMoff = true;					// Flag to see if the GSM module is on/off
bool talkMode = true;				// MMde for user to interface directly with GSM module
int LEDhold;						// Holds status of RGB LED so it can be restored after running functions
int LEDmap = RD_LED+BL_LED+GN_LED;	// Map of the RGB LED bits (works with LEDhold, ie store LEDhold to LEDmap)
static char g_cInput[128];			// String input to a UART
char *GSMresponse = NULL;			// Use to grab UART inputs
char responseLine[10][75];			// Use to store UART inputs
volatile uint32_t ui32Loop;			// for time delays
char ctrlID[11] = "3158078555";		// Phone number of the controller
char boardID[5] = "0001";			// ID of this board

// Variables for processing messages/ storing message info
const char newlineCh[] = "\n";		// newlineCh character
const char commaCh[] = ",";			// commaCh character
int msgOpen = 0;					// Keep track of which message we're processing
// TO DO: do this without pointers, or make sure they're used correctly
char *msgSender = NULL;				// Message sender
char *msgDate = NULL;				// Message date
char *msgTime = NULL;				// Message time

// Variables used by UART (GSM and console) interrupts
unsigned char var;					// Incoming UART character
unsigned char ptr[10000];			// Array for storing incoming UART characters
unsigned long i;					// UART character pointer. There was a j here but I don't think it's used.
unsigned long ulStatus0,ulStatus1;	// To hold the interrupt status
int msgCount = 0;					// Hold the integer value with number of new messages

// Generic variables
int j,k;							// Generic counter
char aString[2][128];				// Generic string

// checkTime function writes to these variables for main program access
int hh,mm,ss,DD,MM,YY,zz;
char fullTimestamp[23] = "\0";

// For EEPROM
// TO DO: Generalize these variables for EEPROM use
#define E2PROM_TEST_ADRES 0x0000	// EEPROM start address
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

// REQUIRED (included by Anil - not sure what it's for or if it's needed)
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

//!*****************************************************************************
//! GENERAL FUNCTIONS
//!*****************************************************************************

//*****************************************************************************
//
// CHECK USER AUTHENTICATION 
// For now, just confirms if user is on a local area code.
//
//*****************************************************************************
bool aCodeauth(char *incoming_number){
    return(incoming_number[1] == '6' && incoming_number[2] == '0' && incoming_number[3] == '8') ? true : false;
}

//*****************************************************************************
//
// SEND AN SMS
//
//*****************************************************************************
void sendSMS(char *destNbr, char *msgBody)
{
	bool msgSent = false;		// Flag for confirming message was sent
	
	UART1printf( "AT+CMGS=\"%s\"\r",destNbr);				// Initialize with dest. address
	for(ui32Loop = 0; ui32Loop < 900000; ui32Loop++){}		// wait
	UART1printf( "%s\r", msgBody );							// Enter message body
	for(ui32Loop = 0; ui32Loop < 9000; ui32Loop++){}		// wait
	UART1printf( "\x1A" );									// Ctrl-Z to send
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
// TURN ON RELAY(S)
// 0 turns on all
//
//*****************************************************************************
void turnOn(int relayNum)
{
    if (relayNum == 1 || relayNum == 0){
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);	// Rel1
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0);			// Rel1N
	}
	if (relayNum == 2 || relayNum == 0){
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_PIN_3);	// Rel2
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0);			// Rel2N
	}
	if (relayNum == 3 || relayNum == 0){
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);	// Rel3
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0);			// Rel3N
	}
	if (relayNum == 4 || relayNum == 0){
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);	// Rel4
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);			// Rel4N
	}
}

//*****************************************************************************
//
// TURN OFF RELAY(S)
// 0 turns off all
//
//*****************************************************************************
void turnOff(int relayNum)
{
    if (relayNum == 1 || relayNum == 0){
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);			// Rel1
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);	// Rel1N
	}
	if (relayNum == 2 || relayNum == 0){
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 0);			// Rel2
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5);	// Rel2N
	}
	if (relayNum == 3 || relayNum == 0){
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);			// Rel3
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);	// Rel3N
	}
	if (relayNum == 4 || relayNum == 0){
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);			// Rel4
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);	// Rel4N
	}
}

//*****************************************************************************
//
// CHECK POWER TO GSM
//
//*****************************************************************************
bool checkGSMpwr (void)
{
	// Delay for a bit, then make sure the flag is ON (something seems to trigger the interrupt when I'm setting it up)
	for(ui32Loop = 0; ui32Loop < 990000; ui32Loop++){}
	GSMoff = true;
	// Then, send an AT to the GSM module - if it triggers the interrupt, we know the GSM is on
	UART1printf("AT\r");
	// Delay for a bit.
	for(ui32Loop = 0; ui32Loop < 900000; ui32Loop++){}
	if ( GSMoff ) { return false; }
	return true;
}

//*****************************************************************************
//
// TOGGLE GSM POWER
//
//*****************************************************************************
void toggleGSMpwr( void )
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
	if (checkGSMpwr()){ UART0printf("\n\r--- GSM power ON!"); }
	else { UART0printf("\n\r--- GSM power OFF!"); }
}

//*****************************************************************************
//
// BLINK LED
// blinkSpeed in 1/10 of a second (roughly)
//
//*****************************************************************************
void blinkLED (int blinkCount, int blinkSpeed, char *LEDcolor)
{
	int m;
	int blinkHold;
	volatile uint32_t blinkLoop;
	
	// Store current LED status, then turn them all off
	blinkHold = GPIOPinRead(GPIO_PORTF_BASE, LEDmap);
	GPIOPinWrite(GPIO_PORTF_BASE, LEDmap, 0);
	blinkSpeed = blinkSpeed*100000;

	for( m=blinkCount; m>=0; m--)
    {
		// Turn on the LED.
	    if ( strstr(LEDcolor,"red") != NULL ) {GPIOPinWrite(GPIO_PORTF_BASE, RD_LED, RD_LED);}
		else if ( strstr(LEDcolor,"blue") != NULL ) {GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, BL_LED);}
		else if ( strstr(LEDcolor,"green") != NULL ) {GPIOPinWrite(GPIO_PORTF_BASE, GN_LED, GN_LED);}
		
		// Delay for a bit.
		for(blinkLoop = 0; blinkLoop < blinkSpeed; blinkLoop++){}
		
		// Turn off the LED.
		GPIOPinWrite(GPIO_PORTF_BASE, LEDmap, 0);

        // Delay for a bit.
        for(blinkLoop = 0; blinkLoop < blinkSpeed; blinkLoop++){}
    }
	
	// Restore prior LED status
	GPIOPinWrite(GPIO_PORTF_BASE, LEDmap, blinkHold);
}


//*****************************************************************************
//
// STORE A GSM RESPONSE TO ARRAY responseLine[]
// Testing to see if this will stop ignoring an SMS of just OK
//
//*****************************************************************************
int getResponse(void)
{
	bool readResponse = true;		// Keeps the loop open while getting a message
	int readLine = 1;				// Counts the lines of the message
	
	while (readResponse){
		// Grab a line, stop after new-line
		UART1gets(g_cInput,sizeof(g_cInput));
		GSMresponse = strtok(g_cInput,newlineCh);
		strcpy(responseLine[readLine], GSMresponse);
		// If this line says OK we've got the whole message
		if ( strncmp(responseLine[readLine],"OK",2) == 0 && strncmp(responseLine[readLine-1],newlineCh,4) == 0 ){readResponse = false;}
		else { readLine++; }
	}
	return(readLine);
}
	
//*****************************************************************************
//
// GET DATE/TIME FROM GSM MODULE 
// Used to fail when run right after GSM power up, but it seems to be working now.
//
//*****************************************************************************
void checkTime(void)
{
	UART1printf("AT+CCLK?\r");
	k = getResponse();
	
	j = 1;
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
// PROCESS SMS FOR ENVELOPE AND CONTENT 
// TO DO: 
// 1. Figure out if it's OK to do this with pointers
// 2. Stop ignoring messages of just "OK"
//
//*****************************************************************************
void processMsg( int activeMsg )
{
	bool msgPresent = true;		// Flag to ignore deleted messages
	char *msgEnvelope = NULL;	// Message envelope holder
	char *msgContent = NULL;	// Message content holder
	
	// FIRST: Request the message and get the lines of the response (includes envelope, nulls, SIM responses)
	UART0printf("\n\r>>> PROCESSING MESSAGE %u",activeMsg);
	UART1printf("AT+CMGR=%u\r\n",activeMsg);
	k = getResponse();
	
	// Delay for a bit, needed when processing multiple messages
	for(ui32Loop = 0; ui32Loop < 100000; ui32Loop++){}
	
	// See if there's actually a message (there will be at least five lines - FIX THIS! IGNORES AN SMS OF "OK"
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
			// Case for the envelope, like +CMGR: "REC READ","+13158078555","","15/10/08,13:18:40-20"
			if ( strstr(responseLine[j],"+CMGR:") != '\0' ){
				// Parse the line for status, ph number, date, and time.
				msgEnvelope = responseLine[j];
				msgSender = strtok(msgEnvelope,commaCh);	// This way we skip status
				msgSender = strtok(NULL,commaCh);
				msgDate = strtok(NULL,commaCh);				// This way we skip phonebook entry
				msgDate = strtok(NULL,commaCh);
				msgTime = strtok(NULL,commaCh);
				strncpy(msgSender,msgSender+2,11);			// Store the number
				msgSender[11] = '\0';
				strncpy(msgDate,msgDate+1,8);				// Store the date
				msgDate[8] = '\0';
				strncpy(msgTime,msgTime,8);					// Store the time
				msgTime[8] = '\0';
			}
			// Case for message content
			// If we already found the envelope, and the line's not blank...
			else if ( msgEnvelope != NULL && responseLine[j] != NULL ){
				// ... and we haven't found any content, this is the first message line.
				if (msgContent == NULL) { msgContent = responseLine[j]; }
				// ... otherwise, add a space and append this line.
				else {
					strcat(msgContent, " ");
					strcat(msgContent, responseLine[j]);
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

//*****************************************************************************
//
// INITIALIZE I2C MODULE 0
// Slightly modified version of TI's example code - from:
// https://eewiki.net/display/microcontroller/I2C+Communication+with+the+TI+Tiva+TM4C123GXL
//
//*****************************************************************************
void initI2C(void)
{
	//reset module
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

	// Configure the pin muxing for I2C0 functions on port B2 and B3.
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	GPIOPinConfigure(GPIO_PB3_I2C0SDA);

	// Select the I2C function for these pins.
	ROM_GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
	ROM_GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

	// Enable and initialize the I2C0 master module.  Use the system clock for
	// the I2C0 module.  The last parameter sets the I2C data transfer rate.
	// If false the data rate is set to 100kbps and if true the data rate will
	// be set to 400kbps.
	I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

	//clear I2C FIFOs
	HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

//*****************************************************************************
//
// READ SPECIFIED REGISTER ON I2C SLAVE
// Not yet tested. Not my code. From:
// https://eewiki.net/display/microcontroller/I2C+Communication+with+the+TI+Tiva+TM4C123GXL
//
//*****************************************************************************
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg)
{
    //specify that we are writing (a register address) to the
    //slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
 
    //specify register to be read
    I2CMasterDataPut(I2C0_BASE, reg);
 
    //send control byte and register address byte to slave device
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
     
    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));
     
    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);
     
    //send control byte and read from the register we
    //specified
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
     
    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));
     
    //return data pulled from the specified register
    return I2CMasterDataGet(I2C0_BASE);
}

//*****************************************************************************
//
// WRITE TO SPECIFIED REGISTER ON I2C SLAVE
// Not yet tested. Not my code. From:
// https://eewiki.net/display/microcontroller/I2C+Communication+with+the+TI+Tiva+TM4C123GXL
//
//*****************************************************************************
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...)
{
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
     
    //stores list of variable number of arguments
    va_list vargs;
     
    //specifies the va_list to "open" and the last fixed argument
    //so vargs knows where to start looking
    va_start(vargs, num_of_args);
     
    //put data to be sent into FIFO
    I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));
     
    //if there is only one argument, we only need to use the
    //single send I2C function
    if(num_of_args == 1)
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
         
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));
         
        //"close" variable argument list
        va_end(vargs);
    }
     
    //otherwise, we start transmission of multiple bytes on the
    //I2C bus
    else
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
         
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));
         
        //send num_of_args-2 pieces of data, using the
        //BURST_SEND_CONT command of the I2C module
        for(uint8_t i = 1; i < (num_of_args - 1); i++)
        {
            //put next piece of data into I2C FIFO
            I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));
            //send next data that was just placed into FIFO
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
     
            // Wait until MCU is done transferring.
            while(I2CMasterBusy(I2C0_BASE));
        }
     
        //put last piece of data into I2C FIFO
        I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));
        //send next data that was just placed into FIFO
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));
         
        //"close" variable args list
        va_end(vargs);
    }
}

//!*****************************************************************************
//! INTERRUPT FUNCTIONS
//!*****************************************************************************

//*****************************************************************************
//
// UART0 - CONSOLE
//
//*****************************************************************************
void
UARTIntHandler0(void)
{
	ulStatus0 = ROM_UARTIntStatus(UART0_BASE, true);					// Get the interrupt status.
	ROM_UARTIntClear(UART0_BASE, ulStatus0);							// Clear the asserted interrupts.
	// Loop while there are characters in the receive FIFO.
	while(ROM_UARTCharsAvail(UART0_BASE))
	{
		var = (unsigned char)ROM_UARTCharGetNonBlocking(UART0_BASE);	// Grab a character
		ptr[i] = var;													// Hold it
		ROM_UARTCharPutNonBlocking(UART1_BASE, ptr[i]);					// Mirror it to GSM
		i++;															// Proceed to next character
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
	char *msgCountStr;													// String with the number of new messages
	
	ulStatus1 = ROM_UARTIntStatus(UART1_BASE, true);					// Get the interrupt status.
	ROM_UARTIntClear(UART1_BASE, ulStatus1);							// Clear the asserted interrupts.
	if ( GSMoff ) { GSMoff = false; }									// Interrupt trigger means the GSM module is on.
	// Loop while there are characters in the receive FIFO.
	while(ROM_UARTCharsAvail(UART1_BASE)){
		var = (unsigned char)ROM_UARTCharGetNonBlocking(UART1_BASE);	// Grab a character
		ptr[i] = var;													// Hold it
		if (talkMode){ROM_UARTCharPutNonBlocking(UART0_BASE, ptr[i]);}	// in "talk to GSM MMde," mirror to console
		// Otherwise, see if it's a new message notification, like +CMTI: "SM",12
		else {
			if(ptr[i-3] == 'C' && ptr[i-2] == 'M' && ptr[i-1] == 'T'&& ptr[i] == 'I'){
				UART1gets(g_cInput,sizeof(g_cInput));					// Grab everything...
				msgCountStr = strtok(g_cInput,newlineCh);				// ...stop after newline
				strncpy(msgCountStr,msgCountStr+7,3);					// Parse out the message counter
				msgCountStr[3]='\0';
				sscanf(msgCountStr, "%d", &msgCount);					// Convert to integer
				UART0printf("\n\r>>> %u NEW MESSAGES",msgCount);		// Tell the user
			}
		}
		i++;															// Go get the next character
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
			blinkLED(3,2,"red");
			UART0printf("\n\r> [Right button] Cycling power to GSM.");
			toggleGSMpwr();
        }
        break;

    case ALL_BUTTONS:
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
//! THE PROGRAM
//!*****************************************************************************
int
main(void)
{
	// Initial settings - from Anil, I'm not sure what all of these are for
	ROM_FPUEnable();											// Enable floating point unit
	ROM_FPULazyStackingEnable();								// Enable lazy stacking of FPU
	// Enable device clocking
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	ROM_IntMasterEnable();										// Enable processor interrupts
    
	// Enable peripherals (disable neopixel, relays one and four in current hardware version)
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);			// UART0
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);			// UART1, GSM pwr, GSM reset, Rel3N, I2C0SCL, I2C0SDA
	//ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);			// Neopixel, keypad interrupt (INT2)
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);			// Rel1N, Rel2, Rel2N, Rel3, Rel4
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);			// RGB LED, Rel1, Rel4N
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);			// Console UART
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);			// GSM UART
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);			// Enable EEPROM: 2048bytes in 32 blocks
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);				// Enable I2C
    
	// Set pins as GPIO outputs (disable neopixel, relays one and four in current hardware version)
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);							//Rel3N
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6);							//GSM PWRKEY
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);							//GSM RESET
	//ROM_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);						//Neopixel
	//ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);						//Rel4
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2);							//Rel3
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_3);							//Rel2
	//ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);						//Rel1N
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_5);							//Rel2N
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);	//RGB LED (pin 1 also Rel1?)
	//ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);						//Rel4N
	
	turnOff(0);											// Turn the relays off initially
	GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, BL_LED);		// Turn on an LED to show that we're working

	// Console UART0: Set GPIO A0 and A1 as UART0 pins, configure for 115200, 8-N-1 operation, enable interrupts
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UART0StdioConfig(0, 115200, 16000000);
	ROM_IntEnable(INT_UART0);
	ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);  

	// GSM UART1: Set GPIO B0 and B1 as UART1 pins, configure for 115200, 8-N-1 operation, enable interrupts
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UART1StdioConfig(1, 115200, 16000000);
	ROM_IntEnable(INT_UART1);
	ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);     

	// Notify the user what's going on
	UART0printf("\n\n\n\r>>> INITIALIZING ");
	UART0printf("\n\r> Checking to see if GSM is on...");
	
	// Find out if the GSM module is on - check three times
	for (k=1; k<4; k++){
		UART0printf( "\n\r> Check %u of 3...\n\r", k );
		if ( checkGSMpwr() )
		{
			blinkLED(5,3,"blue");
			UART0printf("\n\r> GSM is ON.");
			break;
		}
		else
		{
			UART0printf("\n\r> GSM is OFF, turning on...");
			toggleGSMpwr();
			// Delay for a bit, check again. Keep this delay long enough for GSM to get the time.
			for(ui32Loop = 0; ui32Loop < 3000000; ui32Loop++){}
			UART1printf("AT\r");
			if (GSMoff){ UART0printf("\n\r> Power on failed!"); }
		}
	}
	if ( k == 4 ){
		GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, 0);
		blinkLED(10,3,"red");
		GPIOPinWrite(GPIO_PORTF_BASE, RD_LED, RD_LED);
		UART0printf("\n\r> Cannot power on the GSM! \n\r>>> ENDING PROGRAM");
		return 0;
	}

	// Get the time from the GSM module and print to user
	UART0printf("\n\r> Getting date/time from the GSM module");
	checkTime();
	UART0printf("\n\r> Timestamp: %s",fullTimestamp);

	// EEPROM TESTING AREA: Store on-time, retrieve last on-time. Don't run this each time 'cause EEPROM wears out.
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
	
	// RELAY TESTING AREA: Cycle on/off, flash green when turning on, red for off.
	if (testRelay) {
		UART0printf("\n\r> RELAY TESTING:");
		for(ui32Loop = 0; ui32Loop < 1000000; ui32Loop++){}		// Delay for a bit (so you can grab your multimeter)
		for (k=1; k<5; k++){
			UART0printf("\n\r> RELAY %u: ON...",k);
			turnOn(k);
			blinkLED(7,3,"green");
			UART0printf("OFF");
			turnOff(k);
			blinkLED(7,3,"red");
		}
	}

	// Initialize the SysTick interrupt to process buttons
	ButtonsInit();
	SysTickPeriodSet(SysCtlClockGet() / APP_SYSTICKS_PER_SEC);
	SysTickEnable();
	SysTickIntEnable();
	IntMasterEnable();
	
	// Setup complete!
	UART0printf("\n\r> Setup complete! \n\r>>> RUNNING MAIN PROGRAM");
	GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GN_LED, GN_LED);
	
	// Tell the user about buttons
	UART0printf("\n\r> Press left button to enter \"talk to GSM\" mode (blue LED)");
	UART0printf("\n\r> Press right button to toggle power to GSM module (red LED)");
	talkMode = false;	// We were in talk mode during set-up to let SIM messages through, but now we don't want that.
	
	// Notify controller that we're online
	if (ctrlNotify){
		strcpy (aString[1],"BOARD #");
		strcat (aString[1], boardID);
		strcat (aString[1], " ON: ");
		strcat (aString[1], fullTimestamp );
		sendSMS( ctrlID, aString[1] );
	}
	
	// I2C TESTING AREA
	if (testI2C){
		initI2C();
	}
	
	while(1){
		// Process new messages.
		if (msgCount > 0){
			msgOpen = msgCount;									// Keep track of the message we're working on, in case the count updates
			msgCount--;
			for(ui32Loop = 0; ui32Loop < 10000; ui32Loop++){}	// Delay to let each message get printed to UART before going to the next
			processMsg(msgOpen);
		}
	}
}

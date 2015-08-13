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
#include "uart0stdio.h"
#include "uart1stdio.h"
#include "string.h"
#include "stdio.h"
#include "buttons.h"
#include "driverlib/systick.h"
#define RD_LED GPIO_PIN_1
#define BL_LED GPIO_PIN_2
#define GN_LED GPIO_PIN_3

// For driving buttons
uint32_t ui32Buttons;
uint32_t ui32Data;
#define APP_SYSTICKS_PER_SEC            32
#define APP_BUTTON_POLL_DIVIDER          8
#define APP_HIB_BUTTON_DEBOUNCE          (APP_SYSTICKS_PER_SEC * 3)
static volatile uint32_t ui32HibModeEntryCount;

// Global variables.
bool GSM_off = true;
bool talk_mode = false;
int LEDhold;
int LEDmap;
unsigned char var;
unsigned char ptr[10000];
unsigned long i,j;
unsigned long ulStatus0,ulStatus1;
char hold[2][128];
static char g_cInput[128];
int k;							// Generic counter
char *response = NULL;
char printint[10];				// Generic variable to use when printing an integer
volatile uint32_t ui32Loop;		// for time delays

// Checktime function writes to these variables for main program access
char timestamp[20];
char zonestamp[20];
char datestamp[20] = "20";
char datetimezone[30] = "20";

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
void sendsms(char * recipient, char * body)
{
//    strcpy (hold[1],"AT+CMGS=\"");
//    strcat (hold[1],recipient);
//    strcat (hold[1],"\"\r");
//    strcat (body,"\r");
//    UARTSend1( (const unsigned char *) hold[1],strlen(hold[1]));
//    SysCtlDelay(SysCtlClockGet() *0.5);
//    UARTSend1( (const unsigned char *) body,strlen(body));
//    SysCtlDelay(SysCtlClockGet() *0.5);
//    UARTSend1("\x1A",1);
//    SysCtlDelay(SysCtlClockGet() *0.5);
}

// Turn on the relay(s)
void on(int relay)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0);
}

// Turn off the relay(s)
void off(int relay)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);
}

// Clear received buffer (I don't know what this is for, but it looks important)
void clear(void){
    for(j=0; j<i; j++)
        ptr[j]=0;
}

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
	if ( GSM_off == true ) { GSM_off = false; }

	while(ROM_UARTCharsAvail(UART1_BASE)){
		var = (unsigned char)ROM_UARTCharGetNonBlocking(UART1_BASE);
		ptr[i] = var;	
		ROM_UARTCharPutNonBlocking(UART0_BASE, ptr[i]);
		i++;
	}
	
	// If we're in "talk to GSM" mode, we don't want this interrupt to continue
	if( talk_mode == true ){return;}
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
			if (talk_mode == false){
				// Get LED status and store for when we switch back. Turn on blue LED only.
				LEDhold = GPIOPinRead(GPIO_PORTF_BASE, LEDmap);
				GPIOPinWrite(GPIO_PORTF_BASE, LEDmap, 0);
				GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, BL_LED);
				UART0printf("> Entering talk to GSM mode. Press left button to end.\n\r");
				talk_mode = true;
			}
			else{
				UART0printf("> Returning to main program.\n\r");
				GPIOPinWrite(GPIO_PORTF_BASE, LEDmap, LEDhold);		//Restore LED status
				talk_mode = false;
			}
        }
        break;

    case RIGHT_BUTTON:
        // Check if the button has been held int32_t enough to act
        if((ui32TickCounter % APP_BUTTON_POLL_DIVIDER) == 0)
        {
			UART0printf("right button press\n\r");
        }
        break;

    case ALL_BUTTONS:
        // Both buttons for longer than debounce time
        if(ui32HibModeEntryCount < APP_HIB_BUTTON_DEBOUNCE)
        {
			UART0printf("both button press\n\r");
        }
        break;

    default:
        break;
    }
}

// Get the date/time from the GSM module
void checktime(void)
{
 	bool timeunknown = true;
	char *rawStamp = NULL;
	char rawDate[20];

	// Disable the GSM interrupt momentarily
	//ROM_IntDisable(INT_UART1);
	//ROM_UARTIntDisable(UART1_BASE, UART_INT_RX | UART_INT_RT); 
	
	k = 3;
	while ( timeunknown == true && k != 0 )
	{
		// Poll GSM for the time
		UART1printf("AT+CCLK?\r");
		
		// Wait for response
		UART1gets(g_cInput,sizeof(g_cInput));
				
		// Stop checking after new-line
		response = strtok(g_cInput,"\n");
		
		// Make sure it's the time
		if ( strncmp(response,"+CCLK: \"",8) == 0 )
		{
			// Parse for time. Given format: yy/MM/dd,hh:mm:ss±zz (zz is difference from GMT in quarter hours)
			rawStamp = strtok(response,"\"");
			strncpy(rawStamp,rawStamp+8,20);
			strncpy(rawDate,rawStamp,8);
			strcat(datestamp,rawDate);
			strcat(datetimezone,rawStamp);
			datetimezone[22] = '\0';
			strncpy(timestamp,rawStamp+9,8);
			strncpy(zonestamp,rawStamp+17,3);
		}
		else if ( strncmp(response,"*PSNWID: \"",11) == 0 )
		{
			for(ui32Loop = 0; ui32Loop < 900000; ui32Loop++){}
			sprintf (printint, "%d", k);
			UART0printf( "> Found PSNWID " );
			UART0printf( (const char *) printint);
			UART0printf( " \r\n" );			
		}
	}
	// Re-enable the interrupts
	//ROM_IntEnable(INT_UART1);
	//ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
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

// Find out if the GSM module is on
bool checkpower (void)
{
	// Delay for a bit, then make sure the flag is ON (something seems to trigger the interrupt when I'm setting it up)
	for(ui32Loop = 0; ui32Loop < 900000; ui32Loop++){}
	GSM_off = true;
	// Then, send an AT to the GSM module - if it triggers the interrupt, we know the GSM is on
	UART1printf("AT\r");
	// Delay for a bit.
	for(ui32Loop = 0; ui32Loop < 900000; ui32Loop++){}
	if (GSM_off == true)
    {
		// Commands to turn on the GSM module.    
		UART0printf("> GSM is OFF, turning on...\n\r");
        
		//GSM_RESET is active low. Keep this on all the time
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0);
		for(ui32Loop = 0; ui32Loop < 900000; ui32Loop++){}
		UART0printf("> Reset asserted...\n\r");

		//PWRKEY should be 1 for at least 1 second to power on/off the SIM900 module
		//(This pulls down actual SIM900 PWRKEY)
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7);
		for(ui32Loop = 0; ui32Loop < 10000000; ui32Loop++){}                      //3 second delay
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0);            //PWRKEY de-asserted
		UART0printf("\n\r> Power key cycled...\n\r");
		
		// Delay for a bit, check again...
		for(ui32Loop = 0; ui32Loop < 1000000; ui32Loop++){}
		UART1printf("AT\r");
		if (GSM_off == true){return false;}
	}
	else {return true;}
	return true;
}

// Program guts.
int
main(void)
{
	// Initial settings
	ROM_FPUEnable();
	ROM_FPULazyStackingEnable();
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |    SYSCTL_XTAL_16MHZ);
	ROM_IntMasterEnable();                                        // Enable processor interrupts.
    
	// Enable peripherals
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);            //UART 0 pins
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);            //GSM: UART1: RX(0), TX(1), PWRKEY(7), GSM_RESET(6)
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);            //Relay 1 negative (4)
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);            //PORTF: Output LED's and relay 1
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);            //Turn on UART0 module on chip for debugging (PuTTY)
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);            //For GSM module
    
	// Set pins as GPIO outputs
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6);        //GSM PWRKEY
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);        //GSM RESET
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);        //Rel1n
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);	//RGB LED
	LEDmap = RD_LED+BL_LED+GN_LED;													//Defines an RGB LED mapping

	// Turn on an LED to show we're working
	GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, BL_LED);

	//Turn the relays off initially
	off(1);

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
    
	// Enable the UART0 console interrupt for passing messages to GSM module
	ROM_IntEnable(INT_UART0);
	ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);   

	// Enable the UART1 GSM interrupt for passing messages to the console
	ROM_IntEnable(INT_UART1);
	ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);     

	// Notify the user what's going on
	UART0printf("\n\r\r>>> INITIALIZING\n\r");
	UART0printf("> Checking to see if GSM is on...\n\r");
	
	// Find out if the GSM module is on - check three times
	for (k=1; k<4; k++){
		sprintf (printint, "%d", k);
		UART0printf( "> Check " );
		UART0printf( (const char *) printint);
		UART0printf( " of 3...\r\n" );
		if ( checkpower() == true )
		{
			blink(5,300000,2);
			UART0printf("> GSM is ON.\n\r");
			break;
		}
		else
		{
			UART0printf("> Failed! Trying again...\n\r");
		}
	}

	if ( k == 4 ){
		GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, 0);
		blink(10,300000,1);
		GPIOPinWrite(GPIO_PORTF_BASE, RD_LED, RD_LED);
		UART0printf("> Cannot power on the GSM!\n\r");
		UART0printf(">>> ENDING PROGRAM\n\r");
		return 0;
	}

	// Get the time from the GSM module and print to user
	// IMPROVE!!! wait for the OK before proceeding.
	UART0printf("> Getting date/time from the GSM module\n\r");
	checktime();
	blink(10,300000,2);
	UART0printf("> The current date is: ");
	UART0printf( datestamp);
	UART0printf("\n\r> The current time is: ");
	UART0printf( timestamp);
	UART0printf("\n\r> The zone is: ");
	UART0printf( zonestamp);
	UART0printf("\n\r> All together now: ");
	UART0printf( datetimezone);
	UART0printf("\n\r");
		
	// Initialize the SysTick interrupt to process buttons
	ButtonsInit();
	SysTickPeriodSet(SysCtlClockGet() / APP_SYSTICKS_PER_SEC);
	SysTickEnable();
	SysTickIntEnable();
	IntMasterEnable();
	
	//UARTSend1("AT+CMGD=1,4\r",12);  // delete stored msgs
	
	UART0printf("> Setup complete! \n\r>>> RUNNING MAIN PROGRAM\n\r");
	GPIOPinWrite(GPIO_PORTF_BASE, BL_LED, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GN_LED, GN_LED);
	
	// Tell the user about modes
	UART0printf("> Press left button at any time to enter \"talk to GSM\"\n\r> mode, indicated by blue LED\n\r");
	
	// Notify HoH that program is running
	// sendsms("6084735467","HESP is running!");

	// Enter a never-ending while loop. Interrupts trigger everything else.
	while(1){}
}

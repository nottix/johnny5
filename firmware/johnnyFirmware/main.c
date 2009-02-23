/********************************************************************

*/
#ifndef USBMOUSE_C
#define USBMOUSE_C

/** INCLUDES **/
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "./USB/usb_device.h"
#include "./USB/usb.h"
#include <timers.h>
#include <pwm.h>
#include <i2c.h>
#include <delays.h>
#include <usart.h>

/*void DelayTXBitUART(void )	{	
	Delay100TCYx(11);
	Delay10TCYx(7);
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
}

void DelayRXBitUART(void) {		
	Delay100TCYx(11);
	Delay10TCYx(7);
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();

}

void DelayRXHalfBitUART(void) {
	Delay100TCYx(5);
	Delay10TCYx(8);
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
}*/

void DelayTXBitUART(void )	{	
	Delay10TCYx(1);
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
}

void DelayRXBitUART(void) {		
	Delay10TCYx(1);
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
}

void DelayRXHalfBitUART(void) {
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
}


#include <sw_uart.h>

#include "HardwareProfile.h"

#include "./USB/usb_function_hid.h"

/** CONFIGURATION **/
#if defined(PICDEM_FS_USB)      // Configuration bits for PICDEM FS USB Demo Board (based on PIC18F4550)
        #pragma config PLLDIV   = 5         // (20 MHz crystal on PICDEM FS USB board)
        #pragma config CPUDIV   = OSC1_PLL2   
        #pragma config USBDIV   = 2         // Clock source from 96MHz PLL/2
        #pragma config FOSC     = HSPLL_HS
        #pragma config FCMEN    = OFF
        #pragma config IESO     = OFF
        #pragma config PWRT     = OFF
        #pragma config BOR      = ON
        #pragma config BORV     = 3
        #pragma config VREGEN   = ON      //USB Voltage Regulator
        #pragma config WDT      = OFF
        #pragma config WDTPS    = 32768
        #pragma config MCLRE    = ON
        #pragma config LPT1OSC  = OFF
        #pragma config PBADEN   = OFF
//      #pragma config CCP2MX   = ON
        #pragma config STVREN   = ON
        #pragma config LVP      = OFF
//      #pragma config ICPRT    = OFF       // Dedicated In-Circuit Debug/Programming
        #pragma config XINST    = OFF       // Extended Instruction Set
        #pragma config CP0      = OFF
        #pragma config CP1      = OFF
//      #pragma config CP2      = OFF
//      #pragma config CP3      = OFF
        #pragma config CPB      = OFF
//      #pragma config CPD      = OFF
        #pragma config WRT0     = OFF
        #pragma config WRT1     = OFF
//      #pragma config WRT2     = OFF
//      #pragma config WRT3     = OFF
        #pragma config WRTB     = OFF       // Boot Block Write Protection
        #pragma config WRTC     = OFF
//      #pragma config WRTD     = OFF
        #pragma config EBTR0    = OFF
        #pragma config EBTR1    = OFF
//      #pragma config EBTR2    = OFF
//      #pragma config EBTR3    = OFF
        #pragma config EBTRB    = OFF
#endif

/** CONSTANTS **/
#define KP 6
#define KI 1
#define DIR1	TRISDbits.TRISD4
#define DIR2	TRISDbits.TRISD5
#define DIR1L	LATDbits.LATD4
#define DIR2L	LATDbits.LATD5
#define DIR1_HIGH	LATDbits.LATD4 = 1;
#define DIR1_LOW	LATDbits.LATD4 = 0;
#define DIR2_HIGH	LATDbits.LATD5 = 1;
#define DIR2_LOW	LATDbits.LATD5 = 0;

#define USB_BUS_TYPE	0x00
#define USART_BUS_TYPE	0x01


/** VARIABLES **/
#pragma udata
BYTE old_sw2,old_sw3;
BOOL emulate_mode;
BYTE movement_length;
BYTE vector = 0;
unsigned char bus_type = USB_BUS_TYPE;
int usb_con_delay = 0;
char inputstr[20], inch;
int i;

// PWM
unsigned int duty[2] = {0,0};
unsigned int dutyMax = 200;
unsigned int period = 70;
int err1[2], ref1, ref2;
int err[2];
unsigned int count;
int tolerance = 0;

#pragma udata USB_VARIABLES=0x500

unsigned char ReceivedDataBuffer[64];
unsigned char ToSendDataBuffer[64];

#pragma udata

USB_HANDLE USBOutHandle = 0;
USB_HANDLE USBInHandle = 0;
BOOL blinkStatusValid = TRUE;



/** PRIVATE PROTOTYPES **/
void BlinkUSBStatus(void);
BOOL Switch2IsPressed(void);
BOOL Switch3IsPressed(void);
void Emulate_Mouse(void);
static void InitializeSystem(void);
void ProcessIO(void);
void UserInit(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
WORD_VAL ReadPOT(void);

/** VECTOR REMAPPING **/
	//On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
	//the reset, high priority interrupt, and low priority interrupt
	//vectors.  However, the current Microchip USB bootloader 
	//examples are intended to occupy addresses 0x00-0x7FF or
	//0x00-0xFFF depending on which bootloader is used.  Therefore,
	//the bootloader code remaps these vectors to new locations
	//as indicated below.  This remapping is only necessary if you
	//wish to program the hex file generated from this project with
	//the USB bootloader.  If no bootloader is used, edit the
	//usb_config.h file and comment out the following defines:
	//#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER
	//#define PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER
	

	#define REMAPPED_RESET_VECTOR_ADDRESS			0x1000
	#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
	#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018
	
	extern void _startup (void);        // See c018i.c in your C18 compiler dir
	#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
	void _reset (void)
	{
	    _asm goto _startup _endasm
	}
	
	#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
	void Remapped_High_ISR (void)
	{
	     _asm goto YourHighPriorityISRCode _endasm
	}
	#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
	void Remapped_Low_ISR (void)
	{
	     _asm goto YourLowPriorityISRCode _endasm
	}
	
	//Note: If this project is built while one of the bootloaders has
	//been defined, but then the output hex file is not programmed with
	//the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
	//As a result, if an actual interrupt was enabled and occured, the PC would jump
	//to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
	//executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
	//(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
	//would effective reset the application.
	
	//To fix this situation, we should always deliberately place a 
	//"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
	//"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
	//hex file of this project is programmed with the bootloader, these sections do not
	//get bootloaded (as they overlap the bootloader space).  If the output hex file is not
	//programmed using the bootloader, then the below goto instructions do get programmed,
	//and the hex file still works like normal.  The below section is only required to fix this
	//scenario.
	#pragma code HIGH_INTERRUPT_VECTOR = 0x08
	void High_ISR (void)
	{
	     _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
	}
	#pragma code LOW_INTERRUPT_VECTOR = 0x18
	void Low_ISR (void)
	{
	     _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
	}

	#pragma code
	
	
	//These are your actual interrupt handling routines.
	#pragma interrupt YourHighPriorityISRCode
	void YourHighPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
	
	}	//This return will be a "retfie fast", since this is in a #pragma interrupt section 
	#pragma interruptlow YourLowPriorityISRCode
	void YourLowPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
	
	}	//This return will be a "retfie", since this is in a #pragma interruptlow section 

/** DECLARATIONS **/
#pragma code

/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
void main(void)
{
    InitializeSystem();
	mLED_1_Off();
	mLED_1_Off();
    while(1)
    {


		//WriteUART(50);
		if(bus_type==USB_BUS_TYPE) {
			// Check bus status and service USB interrupts.
        	USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
        					  // this function periodically.  This function will take care
        					  // of processing and responding to SETUP transactions 
        					  // (such as during the enumeration process when you first
        					  // plug in).  USB hosts require that USB devices should accept
	        				  // and process SETUP packets in a timely fashion.  Therefore,
    	    				  // when using polling, this function should be called 
        					  // frequently (such as once about every 100 microseconds) at any
        					  // time that a SETUP packet might reasonably be expected to
        					  // be sent by the host to your device.  In most cases, the
        					  // USBDeviceTasks() function does not take very long to
        					  // execute (~50 instruction cycles) before it returns.			  
			if((USBDeviceState < DEFAULT_STATE) && ((usb_con_delay++)>32000)) {
				bus_type = USART_BUS_TYPE;
			}
			// Application-specific tasks.
			// Application related code may be added here, or in the ProcessIO() function.	
        	ProcessIO();
		}
		else {
			mLED_2_On();
			mLED_1_On();

		/*	inch=ReadUART();
			if(inch=='a') {
				WriteUART(inch);
			}
			else if(inch==0x86) {
				WriteUART('z');
			}
			else {
				Delay10KTCYx(255);

				WriteUART('b');
			}*/
//			if(!BusyUSART()) {
//				WriteUSART('a');
//			}
			while(!DataRdyUSART());
		/*	Delay10KTCYx(255);
				Delay10KTCYx(255);
				Delay10KTCYx(255);
				Delay10KTCYx(255);
				Delay10KTCYx(255);
				mLED_2_Off();
				mLED_1_Off();
				Delay10KTCYx(255);
				Delay10KTCYx(255);
				Delay10KTCYx(255);
				Delay10KTCYx(255);
				Delay10KTCYx(255);*/
		/*	memset(inputstr,0,14);
			getsUSART( inputstr, 14 );
			for(i=0; i<14; i++) {
				if(inputstr[i]==0x02) {
					mLED_2_Off();
					mLED_1_Off();
					//printf("char: 0x%x\n", inputstr[i]);
				}
			}*/
			inch = ReadUSART();
			if(inch=='a') {
				while(BusyUSART());
				WriteUSART('a');
			}
			else {
				while(BusyUSART());
				WriteUSART('b');
			}
			//	printf("OOOOOOOOOOOOOOOOOK");
		/*	if(!BusyUSART()) {
				WriteUSART('z');
				//putrsUSART("Hello World");
				mLED_1_Off();
				Delay10KTCYx(255);
				Delay10KTCYx(255);
				Delay10KTCYx(255);
				Delay10KTCYx(255);
				Delay10KTCYx(255);
				WriteUSART('a');
				mLED_1_On();
				Delay10KTCYx(255);
				Delay10KTCYx(255);
				Delay10KTCYx(255);
				Delay10KTCYx(255);
				Delay10KTCYx(255);
				
			}*/
			
		}
    }//end while
}//end main

/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{
	ADCON1 |= 0x0F;                 // Default all pins to digital

//	The USB specifications require that USB peripheral devices must never source
//	current onto the Vbus pin.  Additionally, USB peripherals should not source
//	current on D+ or D- when the host/hub is not actively powering the Vbus line.
//	When designing a self powered (as opposed to bus powered) USB peripheral
//	device, the firmware should make sure not to turn on the USB module and D+
//	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
//	firmware needs some means to detect when Vbus is being powered by the host.
//	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
// 	can be used to detect when Vbus is high (host actively powering), or low
//	(host is shut down or otherwise not supplying power).  The USB firmware
// 	can then periodically poll this I/O pin to know when it is okay to turn on
//	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
//	peripheral device, it is not possible to source current on D+ or D- when the
//	host is not actively providing power on Vbus. Therefore, implementing this
//	bus sense feature is optional.  This firmware can be made to use this bus
//	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
//	HardwareProfile.h file.    
    #if defined(USE_USB_BUS_SENSE_IO)
//    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
    #endif
    
//	If the host PC sends a GetStatus (device) request, the firmware must respond
//	and let the host know if the USB peripheral device is currently bus powered
//	or self powered.  See chapter 9 in the official USB specifications for details
//	regarding this request.  If the peripheral device is capable of being both
//	self and bus powered, it should not return a hard coded value for this request.
//	Instead, firmware should check if it is currently self or bus powered, and
//	respond accordingly.  If the hardware has been configured like demonstrated
//	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
//	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2" 
//	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
//	has been defined in HardwareProfile.h, and that an appropriate I/O pin has been mapped
//	to it in HardwareProfile.h.
    #if defined(USE_SELF_POWER_SENSE_IO)
/    tris_self_power = INPUT_PIN;	// See HardwareProfile.h
    #endif
    
    USBDeviceInit();	//usb_device.c.  Initializes USB module SFRs and firmware
    					//variables to known states.
    UserInit();
}//end InitializeSystem

void InitializeUSART(char sp)
{
   TRISCbits.TRISC7=1; // RX
   TRISCbits.TRISC6=0; // TX
   SPBRG = sp;
   SPBRGH = 0x00;      // 0x0271 for 48MHz -> 19200 baud
   TXSTA = 0x24;       // TX enable BRGH=1
   RCSTA = 0x90;       // continuous RX
   BAUDCON = 0x08;     // BRG16 = 1
}//end InitializeUSART


/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:            
 *
 *****************************************************************************/
void UserInit(void)
{
    //Initialize all of the LED pins
    mInitAllLEDs();
    
    //Initialize all of the push buttons
    mInitAllSwitches();
    old_sw2 = sw2;
    old_sw3 = sw3;
    
    //initialize the variable holding the handle for the last
    // transmission
    USBOutHandle = 0;
    USBInHandle = 0;

    blinkStatusValid = TRUE;

	//Init timer0 for enc1
//	OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_EXT & T0_PS_1_1);
//	WriteTimer0(0);
	
//	OpenTimer1(TIMER_INT_ON & T1_16BIT_RW & T1_SOURCE_EXT & T1_PS_1_1);
//	WriteTimer1(0);

//	OpenTimer2(TIMER_INT_ON & T0_PS_1_1);

	//Init timer2 for PWM1/2
/*	T1CONbits.T1OSCEN=0; 
	TRISCbits.TRISC2 = 0;
	TRISCbits.TRISC1 = 0;
	LATCbits.LATC2 = 0;
	LATCbits.LATC1 = 0;*/

//	OpenPWM2(70);
//	SetDCPWM2(1000);
	
	//Direction
//	DIR1 = 0;
//	DIR2 = 0;
//	DIR1_HIGH
//	DIR2_HIGH
	
	err[0] = 0;
	err[1] = 0;
	err1[0] = 0;
	err1[1] = 0;

//	OpenI2C(MASTER, SLEW_ON);// Initialize I2C module
//	SSPADD = 0x18;

	  // configure USART
	InitializeUSART(0x65);
	//baudUSART(BAUD_IDLE_RX_PIN_STATE_HIGH & BAUD_16_BIT_RATE & BAUD_AUTO_OFF);
	//OpenUSART( USART_TX_INT_OFF  &	USART_RX_INT_OFF &	USART_ASYNCH_MODE &	USART_EIGHT_BIT   & USART_CONT_RX &	USART_BRGH_HIGH, 42);
	//baudUSART(BAUD_IDLE_RX_PIN_STATE_HIGH & BAUD_8_BIT_RATE & BAUD_AUTO_OFF & BAUD_WAKEUP_ON);
	//OpenUART();

}//end UserInit

/********************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user
 *                  routines. It is a mixture of both USB and
 *                  non-USB tasks.
 *
 * Note:            None
 *******************************************************************/
void ProcessIO(void)
{   
    //Blink the LEDs according to the USB device status
    if(blinkStatusValid)
    {
        BlinkUSBStatus();
    }

    // User Application USB tasks
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;
    
    if(!HIDRxHandleBusy(USBOutHandle))				//Check if data was received from the host.
    {   
        switch(ReceivedDataBuffer[0])				//Look at the data the host sent, to see what kind of application specific command it sent.
        {
			case 0x04:
				while(!DataRdyUSART());
				ToSendDataBuffer[0]=ReadUSART();
				while(BusyUSART());
				WriteUSART(ToSendDataBuffer[0]);
/*				if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],64);
                }*/
				break;
			case 0x05:
				while(BusyUSART());
				printf("Hello World");
				break;
			case 0x06:
				while(BusyUSART());
				putrsUSART("Hello World");
				break;
			case 0x07:
				InitializeUSART(ReceivedDataBuffer[1]);
				break;
			case 0x08:
				while(BusyUSART());
				WriteUSART('a');
				break;
			case 0x09:
				SetDCPWM1(0);
				OpenPWM1(70);
				break;
			case 0x10: //Enable PWM1
				
				OpenPWM1(ReceivedDataBuffer[1]);
				duty[0] = ReceivedDataBuffer[2];		//LSB 8 bit
				duty[0] += (ReceivedDataBuffer[3]<<8);	//MSB 2 bit
				SetDCPWM1(duty[0]);
				break;
			case 0x11: //Disable PWM1
				ClosePWM1();
				break;
			case 0x12: //Enable PWM2
				OpenPWM2(ReceivedDataBuffer[1]);
				duty[1] = ReceivedDataBuffer[2];		//LSB 8 bit
				duty[1] += (ReceivedDataBuffer[3]<<8);	//MSB 2 bit
				SetDCPWM2(duty[1]);
				break;
			case 0x13: //Disable PWM2
				ClosePWM2();
				break;
			case 0x14:	//DIR1 LOW
				DIR1_HIGH
				break;
			case 0x15:	//DIR1 LOW
				DIR1_LOW
				break;
			case 0x16:	//DIR2 HIGH
				DIR2_HIGH
				break;
			case 0x17:	//DIR2_LOW
				DIR2_LOW
				break;
			case 0x20: //Enable I2C
				OpenI2C(MASTER, SLEW_ON);// Initialize I2C module
				SSPADD = 9;
				break;
			case 0x21:	//Disable I2C
				CloseI2C();
				break;
			case 0x22:
				OpenI2C(MASTER, SLEW_ON);// Initialize I2C module
				SSPADD = 9;
				//ToSendDataBuffer[0]=EEAckPolling(0x3A);
				ToSendDataBuffer[0]=EERandomRead(0x3A, 0x0D);
				ToSendDataBuffer[1]=EERandomRead(0x1D, 0x0D);
				ToSendDataBuffer[2]=EERandomRead(0x3B, 0x0D);
				CloseI2C();
				if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
			case 0x23:
				ToSendDataBuffer[0]=EERandomRead(0x3A, ReceivedDataBuffer[1]);
				//ToSendDataBuffer[1]=EERandomRead(0x1D, ReceivedDataBuffer[1]);
				//ToSendDataBuffer[2]=EERandomRead(0x3B, ReceivedDataBuffer[1]);
				if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
			case 0x24:
				ToSendDataBuffer[0]=EEByteWrite(0x3A, ReceivedDataBuffer[1], ReceivedDataBuffer[2]);
				ToSendDataBuffer[1]=ToSendDataBuffer[0];
				if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
			case 0x30:	//Receive byte I2C
				ToSendDataBuffer[0]=EERandomRead(ReceivedDataBuffer[1], ReceivedDataBuffer[2]);
				if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
			case 0x31:	//Send byte I2C
				ToSendDataBuffer[0]=EEByteWrite(ReceivedDataBuffer[1], ReceivedDataBuffer[2], ReceivedDataBuffer[3]);
				ToSendDataBuffer[1]=ToSendDataBuffer[0];
				if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
            case 0x80:  //Toggle LEDs command
		        blinkStatusValid = FALSE;			//Stop blinking the LEDs automatically, going to manually control them now.
                if(mGetLED_1() == mGetLED_2())
                {
                    mLED_1_Toggle();
                    mLED_2_Toggle();
                }
                else
                {
                    if(mGetLED_1())
                    {
                        mLED_2_On();
                    }
                    else
                    {
                        mLED_2_Off();
                    }
                }
                break;
			case 0x82:
				count = ReadTimer1();
				
				ToSendDataBuffer[0] = (unsigned char)(count & 0x00FF);
				ToSendDataBuffer[1] = (unsigned char)((count & 0xFF00)>>8);
				if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
			case 0x83:
				count = ReadTimer0();
				
				ToSendDataBuffer[0] = (unsigned char)(count & 0x00FF);
				ToSendDataBuffer[1] = (unsigned char)((count & 0xFF00)>>8);
				if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;

			case 0x90:
				tolerance = ReceivedDataBuffer[1];
				period = ReceivedDataBuffer[2];
				dutyMax = 4*ReceivedDataBuffer[3];
				break;
			case 0x84:
				ToSendDataBuffer[0] = 0x00;
				if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],64);
                }
				ref1 = ReceivedDataBuffer[1];
				ref2 = ReceivedDataBuffer[2];
				SetDCPWM1(0);
				SetDCPWM2(0); 
				OpenPWM1(period);
				WriteTimer0(0);
				err[0] = ref1 - ReadTimer0();
 				OpenPWM2(period);
				WriteTimer1(0);
				err[1] = ref2 - ReadTimer1();
				while(err[0]>tolerance || err[1]>tolerance /*|| err[0]<-tolerance || err[1]<-tolerance*/) {
					duty[0] = duty[0] + KP * err[0] + KI*(err[0]-err1[0]);
					duty[1] = duty[1] + KP * err[1] + KI*(err[1]-err1[1]);
					
					if(duty[0]>dutyMax)
						duty[0]=dutyMax;
					if(duty[1]>dutyMax)
						duty[1]=dutyMax;

					err1[0] = err[0];
					err[0] = ref1 - ReadTimer0();
					err1[1] = err[1];
					err[1] = ref2 - ReadTimer1();
					/*if(err[0]<-tolerance) {
						ref1 += err[0]*(-2);
						DIR1L = !DIR1L;
					}
					if(err[1]<-tolerance) {
						ref2 += err[1]*(-2);
						DIR2L = !DIR2L;
					}*/
					if(err[0]<=tolerance && err[1]<=tolerance /*&& err[0]>=-tolerance && err[1]>=-tolerance*/) {
						ClosePWM1();
						ClosePWM2();
						continue;
					}
					SetDCPWM1(duty[0]);
					SetDCPWM2(duty[1]); 
				}
				/*if(err[0]<0)
					ToSendDataBuffer[0] = -err[0];
				else
					ToSendDataBuffer[0] = err[0];
				if(err[1]<0)
					ToSendDataBuffer[1] = -err[1];
				else
					ToSendDataBuffer[1] = err[1];
				if(err1[0]<0)
					ToSendDataBuffer[2] = -err1[0];
				else
					ToSendDataBuffer[2] = err1[0];
				if(err1[1]<0)
					ToSendDataBuffer[3] = -err1[1];
				else
					ToSendDataBuffer[3] = err1[1];*/
				break;
            case 0x81:  //Get push button state
                ToSendDataBuffer[0] = 0x81;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Pushbutton State command.
				if(sw2 == 1)							//pushbutton not pressed, pull up resistor on circuit board is pulling the PORT pin high
				{
					ToSendDataBuffer[1] = 0x01;			
				}
				else									//sw2 must be == 0, pushbutton is pressed and overpowering the pull up resistor
				{
					ToSendDataBuffer[1] = 0x00;
				}
                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;

            case 0x37:	//Read POT command.  Uses ADC to measure an analog voltage on one of the ANxx I/O pins, and returns the result to the host
                {
                    WORD_VAL w;

	                if(!HIDTxHandleBusy(USBInHandle))
	                {
	                    mInitPOT();
	                    w = ReadPOT();					//Use ADC to read the I/O pin voltage.  See the relevant HardwareProfile - xxxxx.h file for the I/O pin that it will measure.
														//Some demo boards, like the PIC18F87J50 FS USB Plug-In Module board, do not have a potentiometer (when used stand alone).
														//This function call will still measure the analog voltage on the I/O pin however.  To make the demo more interesting, it
														//is suggested that an external adjustable analog voltage should be applied to this pin.
						ToSendDataBuffer[0] = 0x37;  	//Echo back to the host the command we are fulfilling in the first byte.  In this case, the Read POT (analog voltage) command.
						ToSendDataBuffer[1] = w.v[0];  	//Measured analog voltage LSB
						ToSendDataBuffer[2] = w.v[1];  	//Measured analog voltage MSB

	                    USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],64);
	                }					
                }
                break;
        }
        //Re-arm the OUT endpoint for the next packet
        USBOutHandle = HIDRxPacket(HID_EP,(BYTE*)&ReceivedDataBuffer,64);
    }

    
}//end ProcessIO


/******************************************************************************
 * Function:        BOOL Switch2IsPressed(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          TRUE - pressed, FALSE - not pressed
 *
 * Side Effects:    None
 *
 * Overview:        Indicates if the switch is pressed.  
 *
 * Note:            
 *
 *****************************************************************************/
BOOL Switch2IsPressed(void)
{
    if(sw2 != old_sw2)
    {
        old_sw2 = sw2;                  // Save new value
        if(sw2 == 0)                    // If pressed
            return TRUE;                // Was pressed
    }//end if
    return FALSE;                       // Was not pressed
}//end Switch2IsPressed

/******************************************************************************
 * Function:        BOOL Switch3IsPressed(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          TRUE - pressed, FALSE - not pressed
 *
 * Side Effects:    None
 *
 * Overview:        Indicates if the switch is pressed.  
 *
 * Note:            
 *
 *****************************************************************************/
BOOL Switch3IsPressed(void)
{
    if(sw3 != old_sw3)
    {
        old_sw3 = sw3;                  // Save new value
        if(sw3 == 0)                    // If pressed
            return TRUE;                // Was pressed
    }//end if
    return FALSE;                       // Was not pressed
}//end Switch3IsPressed

/******************************************************************************
 * Function:        WORD_VAL ReadPOT(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          WORD_VAL - the 10-bit right justified POT value
 *
 * Side Effects:    ADC buffer value updated
 *
 * Overview:        This function reads the POT and leaves the value in the 
 *                  ADC buffer register
 *
 * Note:            None
 *****************************************************************************/
WORD_VAL ReadPOT(void)
{
    WORD_VAL w;

    mInitPOT();

    ADCON0bits.GO = 1;              // Start AD conversion
    while(ADCON0bits.NOT_DONE);     // Wait for conversion

    w.v[0] = ADRESL;
    w.v[1] = ADRESH;

    //w.Val = ADC1BUF0;

    return w;
}//end ReadPOT

/********************************************************************
 * Function:        void BlinkUSBStatus(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        BlinkUSBStatus turns on and off LEDs 
 *                  corresponding to the USB device state.
 *
 * Note:            mLED macros can be found in HardwareProfile.h
 *                  USBDeviceState is declared and updated in
 *                  usb_device.c.
 *******************************************************************/
void BlinkUSBStatus(void)
{
    static WORD led_count=0;
    
    if(led_count == 0)led_count = 10000U;
    led_count--;

    #define mLED_Both_Off()         {mLED_1_Off();mLED_2_Off();}
    #define mLED_Both_On()          {mLED_1_On();mLED_2_On();}
    #define mLED_Only_1_On()        {mLED_1_On();mLED_2_Off();}
    #define mLED_Only_2_On()        {mLED_1_Off();mLED_2_On();}

    if(USBSuspendControl == 1)
    {
        if(led_count==0)
        {
            mLED_1_Toggle();
            if(mGetLED_1())
            {
                mLED_2_On();
            }
            else
            {
                mLED_2_Off();
            }
        }//end if
    }
    else
    {
        if(USBDeviceState == DETACHED_STATE)
        {
            mLED_Both_Off();
        }
        else if(USBDeviceState == ATTACHED_STATE)
        {
            mLED_Both_On();
        }
        else if(USBDeviceState == POWERED_STATE)
        {
            mLED_Only_1_On();
        }
        else if(USBDeviceState == DEFAULT_STATE)
        {
            mLED_Only_2_On();
        }
        else if(USBDeviceState == ADDRESS_STATE)
        {
            if(led_count == 0)
            {
                mLED_1_Toggle();
                mLED_2_Off();
            }//end if
        }
        else if(USBDeviceState == CONFIGURED_STATE)
        {
            if(led_count==0)
            {
                mLED_1_Toggle();
                if(mGetLED_1())
                {
                    mLED_2_Off();
                }
                else
                {
                    mLED_2_On();
                }
            }//end if
        }//end if(...)
    }//end if(UCONbits.SUSPND...)

}//end BlinkUSBStatus




// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void)
{
	//Example power saving code.  Insert appropriate code here for the desired
	//application behavior.  If the microcontroller will be put to sleep, a
	//process similar to that shown below may be used:
	
	//ConfigureIOPinsForLowPower();
	//SaveStateOfAllInterruptEnableBits();
	//DisableAllInterruptEnableBits();
	//EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
	//Sleep();
	//RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
	//RestoreIOPinsToNormal();									//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

	//IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is 
	//cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause 
	//things to not work as intended.	
	

    #if defined(__C30__)
    #if 0
        U1EIR = 0xFFFF;
        U1IR = 0xFFFF;
        U1OTGIR = 0xFFFF;
        IFS5bits.USB1IF = 0;
        IEC5bits.USB1IE = 1;
        U1OTGIEbits.ACTVIE = 1;
        U1OTGIRbits.ACTVIF = 1;
        TRISA &= 0xFF3F;
        LATAbits.LATA6 = 1;
        Sleep();
        LATAbits.LATA6 = 0;
    #endif
    #endif
}


/******************************************************************************
 * Function:        void _USB1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the USB interrupt bit is set
 *					In this example the interrupt is only used when the device
 *					goes to sleep when it receives a USB suspend command
 *
 * Note:            None
 *****************************************************************************/
#if 0
void __attribute__ ((interrupt)) _USB1Interrupt(void)
{
    #if !defined(self_powered)
        if(U1OTGIRbits.ACTVIF)
        {
            IEC5bits.USB1IE = 0;
            U1OTGIEbits.ACTVIE = 0;
            IFS5bits.USB1IF = 0;
        
            //USBClearInterruptFlag(USBActivityIFReg,USBActivityIFBitNum);
            USBClearInterruptFlag(USBIdleIFReg,USBIdleIFBitNum);
            //USBSuspendControl = 0;
        }
    #endif
}
#endif

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *					suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *					mode, the host may wake the device back up by sending non-
 *					idle state signalling.
 *					
 *					This call back is invoked when a wakeup from USB suspend 
 *					is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
	// If clock switching or other power savings measures were taken when
	// executing the USBCBSuspend() function, now would be a good time to
	// switch back to normal full power run mode conditions.  The host allows
	// a few milliseconds of wakeup time, after which the device must be 
	// fully back to normal, and capable of receiving and processing USB
	// packets.  In order to do this, the USB module must receive proper
	// clocking (IE: 48MHz clock must be available to SIE for full speed USB
	// operation).
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

	// Typically, user firmware does not need to do anything special
	// if a USB error occurs.  For example, if the host sends an OUT
	// packet to your device, but the packet gets corrupted (ex:
	// because of a bad connection, or the user unplugs the
	// USB cable during the transmission) this will typically set
	// one or more USB error interrupt flags.  Nothing specific
	// needs to be done however, since the SIE will automatically
	// send a "NAK" packet to the host.  In response to this, the
	// host will normally retry to send the packet again, and no
	// data loss occurs.  The system will typically recover
	// automatically, without the need for application firmware
	// intervention.
	
	// Nevertheless, this callback function is provided, such as
	// for debugging purposes.
}


/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 * 					firmware must process the request and respond
 *					appropriately to fulfill the request.  Some of
 *					the SETUP packets will be for standard
 *					USB "chapter 9" (as in, fulfilling chapter 9 of
 *					the official USB specifications) requests, while
 *					others may be specific to the USB device class
 *					that is being implemented.  For example, a HID
 *					class device needs to be able to respond to
 *					"GET REPORT" type of requests.  This
 *					is not a standard USB chapter 9 request, and 
 *					therefore not handled by usb_device.c.  Instead
 *					this request should be handled by class specific 
 *					firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq(void)
{
    USBCheckHIDRequest();
}//end


/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This 
 *					callback function should initialize the endpoints 
 *					for the device's usage according to the current 
 *					configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP(void)
{
    //enable the HID endpoint
    USBEnableEndpoint(HID_EP,USB_IN_ENABLED|USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    //Re-arm the OUT endpoint for the next packet
    USBOutHandle = HIDRxPacket(HID_EP,(BYTE*)&ReceivedDataBuffer,64);
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 * 					peripheral devices to wake up a host PC (such
 *					as if it is in a low power suspend to RAM state).
 *					This can be a very useful feature in some
 *					USB applications, such as an Infrared remote
 *					control	receiver.  If a user presses the "power"
 *					button on a remote control, it is nice that the
 *					IR receiver can detect this signalling, and then
 *					send a USB "command" to the PC to wake up.
 *					
 *					The USBCBSendResume() "callback" function is used
 *					to send this special USB signalling which wakes 
 *					up the PC.  This function may be called by
 *					application firmware to wake up the PC.  This
 *					function should only be called when:
 *					
 *					1.  The USB driver used on the host PC supports
 *						the remote wakeup capability.
 *					2.  The USB configuration descriptor indicates
 *						the device is remote wakeup capable in the
 *						bmAttributes field.
 *					3.  The USB host PC is currently sleeping,
 *						and has previously sent your device a SET 
 *						FEATURE setup packet which "armed" the
 *						remote wakeup capability.   
 *
 *					This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            Interrupt vs. Polling
 *                  -Primary clock
 *                  -Secondary clock ***** MAKE NOTES ABOUT THIS *******
 *                   > Can switch to primary first by calling USBCBWakeFromSuspend()
 
 *                  The modifiable section in this routine should be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of 1-13 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at lest 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
    static WORD delay_count;
    
    USBResumeControl = 1;                // Start RESUME signaling
    
    delay_count = 1800U;                // Set RESUME line for 1-13 ms
    do
    {
        delay_count--;
    }while(delay_count);
    USBResumeControl = 0;
}


/** EOF main.c *************************************************/
#endif

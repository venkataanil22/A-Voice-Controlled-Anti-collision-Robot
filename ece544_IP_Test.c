
/**
*
* @file ece544periph_test.c
*
* @author Roy Kravitz (roy.kravitz@pdx.edu)
* @modified by Shrivatsa Yogendra (srivatsa@pdx.edu)
* @modified by Chetan Bornarkar (bchetan@pdx.edu)
* @modified by Randon Stasney (rstasney@pdx.edu) Aditya Pawar (adipawar@pdx.edu)
*	Venkata Anil Viswanadha (v22@pdx.edu)Brandon Biodrowski(bran5@pdx.edu)
* @copyright Portland State University, 2014-2015, 2016-2017
*
* This file implements the control over a remote vehicle equipped with
* a Nexy4 DDR Board.  The two means of communication are through the gpio
* PMOD pins and through a Bluetooth (BT) PMODBT2 device from Digilent.  The 
* program also utilizes the OLED PMOD display and the HB3 PMOD motor drivers.
*
* The new program allows Cyclic Executive complete control over the Rover
*	o initialize the Nexys4IO driver and all PMOD devices
*	o Set the SSEG (seven segment display) to display the speed of the
*		vehicle using the onboard accelerometer.
*	o Safety splash screen and turning indicators on the OLED 
*	o (SW) switch 0 gates control to activate motors
*	o The RGB LED flashes green to indicate safe collision detection in 
*		front of the vehicle.  Flashes red when object found in path
*	o The system is set on priority with lowest being voice commands from 
*		the Kinect through serial wixel interface to the attached arduino
*		which decodes the string into a binary value transmitted over the 
*		gpio pins allowing control over the motors and the light servo.
*	o The next priority is collision detection with a MAXSONAR PMOD connected 
*		to the on board ADC.  This safety range is held in a variable and could be 
*		changed through a command as well
*	o The highest priority is the Bluetooth (BT).  This will allow control 
*		the vehicle even is a collsion obstacle has been detected.  This is
*		interfaced through the Nexys4 board through the Uart interface and 
* 		drven through a computer serial port or over the phone using the 
* 		arduino bluetooth app.  
*	o The solar light is mounted on a servo which can be halted with the stop
*		command
*	o The motors themselves are driven through the PMOD HB3 and the created IP
*		from ECE 544 Project 2.  There remains software and hardware protection
* 		from changing direction while the motor is turning
*
* MODIFICATION HISTORY:
*
* Ver   Who  Date     Changes
* ----- ---- -------- -----------------------------------------------
* 1.00a	rhk	12/22/14	First release of test program.  Builds on the nx4io_test program.
* 2.00  sy	08/22/16	Modified the code and implemented the functionality of color wheel implementation
* 						and detection of the generated PWM for the RGB LED's
* 3.00	rs	02/01/17	Modified to detect pulse width modulation in SW and display HW results
* 4.00	rs	03/21/17	Modified to run Lunar Rover remotely
*
* @note
* The minimal hardware configuration for this test is a Microblaze-based system with at least 32KB of memory,
* an instance of Nexys4IO, an instance of the PMod544IOR2,  and an instance of the Xilinx
* UARTLite (used for xil_printf() console output)
*
******************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include "platform.h"
#include "xparameters.h"
#include "xstatus.h"
#include "nexys4IO.h"
#include "xuartlite.h"
#include "xsysmon.h"
#include "IPhb3.h"
#include "xgpio.h"
#include "xintc.h"
#include "xtmrctr.h"
#include "PmodOLEDrgb.h"
#include "bitmap.h"
#include "PmodBT2.h"

/************************** Constant Definitions ****************************/
// Clock frequencies
#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ

// AXI timer parameters
#define AXI_TIMER_DEVICE_ID		XPAR_AXI_TIMER_0_DEVICE_ID
#define AXI_TIMER_BASEADDR		XPAR_AXI_TIMER_0_BASEADDR
#define AXI_TIMER_HIGHADDR		XPAR_AXI_TIMER_0_HIGHADDR
#define TmrCtrNumber			0

// Definitions for peripheral NEXYS4IO
#define NX4IO_DEVICE_ID		XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR		XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR		XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

// Definitions definitions for peripheral AXI_UARTLITE
#define UARTLITE_1_DEVICE_ID  	XPAR_UARTLITE_1_DEVICE_ID 
#define UARTLITE_1_BASEADDR		XPAR_UARTLITE_1_BASEADDR 


// Definitions for peripheral PMODOLEDRGB
#define RGBDSPLY_DEVICE_ID		XPAR_PMODOLEDRGB_0_DEVICE_ID
#define RGBDSPLY_GPIO_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR
#define RGBDSPLY_GPIO_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_HIGHADD
#define RGBDSPLY_SPI_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR
#define RGBDSPLY_SPI_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_HIGHADDR

// Definitions for peripheral PMODBT2
#define PMODBT2_DEVICE_ID 					XPAR_PMODBT2_0_DEVICE_ID
#define PMODBT2_UART_BASEADDR				XPAR_PMODBT2_0_AXI_LITE_UART_BASEADDR 
#define PMODBT2_UART_HIGHADDR				XPAR_PMODBT2_0_AXI_LITE_UART_HIGHADDR 
#define PMODBT2_GPIO_BASEADDR				XPAR_PMODBT2_0_AXI_LITE_GPIO_BASEADDR 
#define PMODBT2_GPIO_HIGHADDR				XPAR_PMODBT2_0_AXI_LITE_GPIO_HIGHADDR 


// Definitions for peripheral IPHB3_0 
#define IPHB3_DEVICE_ID			XPAR_IPHB3_0_DEVICE_ID 
#define IPHB3_BASEADDR 			XPAR_IPHB3_0_S00_AXI_BASEADDR
#define IPHB3_HIGHADDR 			XPAR_IPHB3_0_S00_AXI_HIGHADDR 
// Definitions for peripheral IPHB3_1
#define IPHB3_DEVICE_ID			XPAR_IPHB3_1_DEVICE_ID
#define IPHB3_BASEADDR 			XPAR_IPHB3_1_S00_AXI_BASEADDR
#define IPHB3_HIGHADDR 			XPAR_IPHB3_1_S00_AXI_HIGHADDR

// Fixed Interval timer - 100 MHz input clock, 5KHz output clock
#define FIT_IN_CLOCK_FREQ_HZ	CPU_CLOCK_FREQ_HZ
#define FIT_CLOCK_FREQ_HZ		5000
#define FIT_COUNT				(FIT_IN_CLOCK_FREQ_HZ / FIT_CLOCK_FREQ_HZ)
#define FIT_COUNT_1MSEC			5

// GPIO parameters
#define GPIO_0_DEVICE_ID			XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_0_INPUT_0_CHANNEL		1
#define GPIO_0_ACCIN_0_CHANNEL		2

// ADC
#define SYSMON_DEVICE_ID 	XPAR_SYSMON_0_DEVICE_ID

// Interrupt Controller parameters
#define INTC_DEVICE_ID			XPAR_INTC_0_DEVICE_ID
#define FIT_INTERRUPT_ID		XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR


// Mask for HB3
#define IPHB3_MOT_MASK				0x00000100


// Mask for switch[0]
#define SW_0 						0x00000001

//#define
/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/


/************************** Function Prototypes *****************************/
void usleep(u32 usecs);

void PMDIO_itoa(int32_t value, char *string, int32_t radix);
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num);
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix);

void FIT_Handler(void);										
int do_init_nx4io(u32 BaseAddress);
int do_init_pmdio(u32 BaseAddress);
int AXI_Timer_initialize(void);
int do_init();
static int SysMonFractionToInt(float FloatNum);


PmodOLEDrgb	pmodOLEDrgb_inst;			// PmodOLED instance ref
PmodBT2		myDevice;					// Pmod Bluetooth
XGpio		GPIOInst0;					// GPIO instance
XIntc 		IntrptCtlrInst;				// Interrupt Controller instance
XTmrCtr		AXITimerInst;				// PWM timer instance
static XSysMon SysMonInst;  			// ADC instance



// global variables 

// ADC variables
XSysMon_Config 		*ConfigPtr;			// configuration pointer
u32 				aux3Raw;			// raw input from analog line in
float 				aux3Data;			// converted to voltage
XSysMon *SysMonInstPtr = &SysMonInst;

volatile u32			gpio_in;		// GPIO input port "arduino voice"


volatile u16			accel_x;		// raw accelerometer from board
float					g_xaxis;		// conversion to speed

u16 					sw = 0;			// storage to read in the switches

volatile u32			MOT_CONR = 0;	// hold value to write right motor
volatile u32			MOT_CONL = 0;	// hold value to write left motor

volatile uint8_t		DirSafe = 0;	// sw protection flag to avoid hbridge short
volatile uint8_t		sensor = 0;		// collision detector flag

/************************** MAIN PROGRAM ************************************/
int main()				
{
	int sts;
    init_platform();

    sts = do_init();		// initialize the peripherals
    if (XST_SUCCESS != sts)
    {
    	exit(1);
    }
	xil_printf("ECE 544 Lunar Rover\r\n");
	xil_printf("By Randon Stasney, Aditya Pawar \r\n");
	xil_printf("Venkata Anil Viswanadha, Brandon Biodrowski 21-March 2017\r\n\n");

	while (1) {
	// Detect PWM duty cycle and display
	LunarRover();

	// blank the display digits and turn off the decimal points
	NX410_SSEG_setAllDigits(SSEGLO, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);

	// Clear all the display digits and the OLED display at the end of the program
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	IPHB3_mWriteReg(IPHB3_BASEADDR, IPHB3_S00_AXI_SLV_REG0_OFFSET, 0x00000000);
	}
	cleanup_platform();
    exit(0);
}


/**
 * Function Name: do_init()
 *
 * Return: XST_FAILURE or XST_SUCCESS
 *
 * Description: Initialize the AXI timer, gpio, interrupt, FIT timer, Encoder,
 * 				OLED display
 */
int do_init()
{
	int status;

	// initialize the Nexys4 driver and other devices
	status = (uint32_t) NX4IO_initialize(NX4IO_BASEADDR);
	if (status == XST_FAILURE)
	{
		exit(1);
	}
	// Initiaize Bluetooth and set baud rate
	BT2_begin(&myDevice, XPAR_PMODBT2_0_AXI_LITE_GPIO_BASEADDR, XPAR_PMODBT2_0_AXI_LITE_UART_BASEADDR);
	BT2_changeBaud(&myDevice, 9600);


	// Initialize the AXI Timer 
	status = AXI_Timer_initialize();
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}


	// write ECE 544 on SSEG
	NX4IO_SSEG_setSSEG_DATA(SSEGHI, 0x0058E30E);
	NX4IO_SSEG_setSSEG_DATA(SSEGLO, 0x00144116);
	
	// Initialize the OLED display 
	OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);
	
	// initialize the GPIO instances
	status = XGpio_Initialize(&GPIOInst0, GPIO_0_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL, 0xFFFFFFFF); // arduino input
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_ACCIN_0_CHANNEL, 0x1FF);		// accelerometer input

	// initialize the interrupt controller
	status = XIntc_Initialize(&IntrptCtlrInst, INTC_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
	   return XST_FAILURE;
	}

	// connect the fixed interval timer (FIT) handler to the interrupt
	status = XIntc_Connect(&IntrptCtlrInst, FIT_INTERRUPT_ID,
						   (XInterruptHandler)FIT_Handler,
						   (void *)0);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;

	}

	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts.
	status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// enable the FIT interrupt
	XIntc_Enable(&IntrptCtlrInst, FIT_INTERRUPT_ID);
	
	/*
	 * Initialize the SysMon driver.
	 */
	ConfigPtr = XSysMon_LookupConfig(SYSMON_DEVICE_ID);
	if (ConfigPtr == NULL) {
		return XST_FAILURE;
	}
	XSysMon_CfgInitialize(SysMonInstPtr, ConfigPtr,
				ConfigPtr->BaseAddress);
	/*
	 * Self Test the System Monitor/ADC device
	 */
	status = XSysMon_SelfTest(SysMonInstPtr);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	/*
	 * Set the ADCCLK frequency equal to 1/32 of System clock for the System
	 * Monitor/ADC in the Configuration Register 2.
	 */
	XSysMon_SetAdcClkDivisor(SysMonInstPtr, 32);
	/*
	 * Wait till the End of Sequence occurs
	 */
	XSysMon_GetStatus(SysMonInstPtr); /* Clear the old status */

	return XST_SUCCESS;
}
/*
 * AXI timer initializes it to generate out a 4Khz signal, Which is given to the Nexys4IO module as clock input.
 * DO NOT MODIFY
 */
int AXI_Timer_initialize(void){

	uint32_t status;				// status from Xilinx Lib calls
	u32		ctlsts;		// control/status register or mask

	status = XTmrCtr_Initialize(&AXITimerInst,AXI_TIMER_DEVICE_ID);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	status = XTmrCtr_SelfTest(&AXITimerInst, TmrCtrNumber);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	ctlsts = XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_EXT_GENERATE_MASK | XTC_CSR_LOAD_MASK |XTC_CSR_DOWN_COUNT_MASK ;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber,ctlsts);

	//Set the value that is loaded into the timer counter and cause it to be loaded into the timer counter
	XTmrCtr_SetLoadReg(AXI_TIMER_BASEADDR, TmrCtrNumber, 24998);
	XTmrCtr_LoadTimerCounterReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts &= (~XTC_CSR_LOAD_MASK);
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts |= XTC_CSR_ENABLE_TMR_MASK;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	XTmrCtr_Enable(AXI_TIMER_BASEADDR, TmrCtrNumber);
	return XST_SUCCESS;

}



/****************************************************************************/
/**
* LunarRover - Test to detect PWM and test board interface through MicroBlaze
*
*Controls the remote operation of the Lunar Rover
*			o 	Slide switch 0 on for run mode 
* 			o   BTNC for soft reset back to program
* 			o 	Priority BT - collision sensor - voice commands
*			o 	RGB LED signals collision detection
*			o 	SSEG display velocity by accelerometer
*			o 	OLED used as turn indicators
*
*
* @param	*NONE*
*
* @return	*NONE*
*
*****************************************************************************/
void LunarRover(void)
{	
	
	u32 hard_soft;						 // hold value for sw 0 

	// OLED values for blinkers
	uint8_t r = 255;					// red on
	uint8_t g = 0;						// green off
	uint8_t b = 0;						// blue off

	uint8_t Blink_flag = 0;				// timer for signal blink
	uint8_t DirFlag = 0;				// forward 0 reverse 1

	int len;							// BT variable for aquisition

	char output = "\n";					// BT input char


	xil_printf("Starting LunarRover\n");
	xil_printf("Switch 0 enters operational mode\n");
	xil_printf("BTNC soft reset back to splash screen\n");


	// turn off LEDS
	NX4IO_setLEDs(0x00000000);
	NX4IO_RGBLED_setChnlEn(RGB1, true, true, true);
	NX4IO_RGBLED_setChnlEn(RGB2, true, true, true);	
	NX4IO_RGBLED_setDutyCycle(RGB1, 0, 0, 0);
	NX4IO_RGBLED_setDutyCycle(RGB2, 0, 0, 0);
	
	// raw set motors off
	IPHB3_mWriteReg(XPAR_IPHB3_0_S00_AXI_BASEADDR, IPHB3_S00_AXI_SLV_REG0_OFFSET, 0x00000000);
	IPHB3_mWriteReg(XPAR_IPHB3_1_S00_AXI_BASEADDR, IPHB3_S00_AXI_SLV_REG0_OFFSET, 0x00000000);
	
	// display Amelie splash screen for intro before detection
	OLEDrgb_DrawBitmap(&pmodOLEDrgb_inst,0,0,95,63, (u8*)tommy);
	usleep(0100000);//Wait 1 seconds
	while(1)
	{
		sw = NX4IO_getSwitches();							// get switch info
		hard_soft = (sw & SW_0);							// mask to just get sw 0
		
		if (hard_soft){										// exit to lunar operation
			break;
		}
	}


	// clear splash screen
	OLEDrgb_Clear(&pmodOLEDrgb_inst);


	while(1)
	{
		// check if the BTNC is pressed
		// exit to soft reset if it is pressed.

		if (NX4IO_isPressed(BTNC))
		{
			break;
		}
	//	otherwise run lunar module
 
	// wait for ADC value to be ready
	while ((XSysMon_GetStatus(SysMonInstPtr) & XSM_SR_EOS_MASK) !=
			XSM_SR_EOS_MASK);				
	// get ADC value
	aux3Raw = XSysMon_GetAdcData(SysMonInstPtr, XSM_CH_AUX_MIN+3);
	// convert to voltage
	aux3Data = XSysMon_RawToVoltage(aux3Raw);
	// threshhold for .18 volts is right about 8 inches
	if (aux3Data < .18){
		sensor = 1;  // trigger voice stop
	}
	else {
		sensor = 0;	// allow regular voice operation
	}
	
	// Inputs to Control Via priority case statements
	if((len=BT2_getData(&myDevice, 600))){ // if new BT data 
	output = myDevice.recv[0];}	// update BT case variable
		switch (output){ // BT
			case '0'	:									// Allow voice to run
				if (sensor){ 								// top priority in voice object detect
					MOT_CONR = 0;							// turn off right motor
					MOT_CONL = 0;							// turn off left motor
					DirSafe = 1;							// Motors off allow reverse
					OLEDrgb_Clear(&pmodOLEDrgb_inst);		// clear possible blinkers
					}
				else{										// sensor not detect use voice
				switch (gpio_in){							// use voice data 
					case 0 :  
						MOT_CONR = 0;						// turn off right motor
						MOT_CONL = 0;						// turn off left motor
						DirSafe = 1;						// Motors off allow reverse
						OLEDrgb_Clear(&pmodOLEDrgb_inst);	// clear possible blinkers
					break;
					case 1 :	// turn right 	
						MOT_CONR = 0;						// turn off right motor
						MOT_CONL = 0x00000132;				// left on super low to turn right
						DirSafe = 0;						// Motors running do not allow reverse
					break;
					case 2 :	// turn left 	
						MOT_CONR = 0x00000132;				// right on super low to turn left
						MOT_CONL = 0;						// turn off left motor
						DirSafe = 0;						// Motors running do not allow reverse
					break;
					case 3 :	// slow 				
						MOT_CONR = 0x00000164;				// medium speed
						MOT_CONL = 0x00000164;				// medium speed
						DirSafe = 0;						// Motors running do not allow reverse
					break;
					case 4 :	// fast	
						MOT_CONR = 0x000001F0;				// fast
						MOT_CONL = 0x000001F0;				// fast
						DirSafe = 0;						// Motors running do not allow reverse
					break;
					case 5 :	// stop	
						NX4IO_RGBLED_setDutyCycle(RGB1, 0, 0, 0); 	// turn off light servo
						MOT_CONR = 0;							// turn off right motor
						MOT_CONL = 0;							// turn off left motor
						DirSafe = 1;							// Motors off allow reverse
						OLEDrgb_Clear(&pmodOLEDrgb_inst);		// clear possible blinkers
					break;
					case 6 :	// reverse	
						if (DirSafe){							// if safe put in reverse
							// write the motor directuin via axi
							IPHB3_mWriteReg(XPAR_IPHB3_0_S00_AXI_BASEADDR, IPHB3_S00_AXI_SLV_REG3_OFFSET, 0x00000001);
							IPHB3_mWriteReg(XPAR_IPHB3_1_S00_AXI_BASEADDR, IPHB3_S00_AXI_SLV_REG3_OFFSET, 0x00000001);					
							MOT_CONR = 0;						// turn off right motor
							MOT_CONL = 0;						// turn off left motor
						}
						else {
							MOT_CONR = MOT_CONR;				// motor remains
							MOT_CONL = MOT_CONL;				// motor remains
						}
					break;
					case 7:		// search light
						NX4IO_RGBLED_setDutyCycle(RGB1, 0, 255, 0); // set pwm for servo output
						MOT_CONR = MOT_CONR;					// motor remains
						MOT_CONL = MOT_CONL;					// motor remains
					break;
					default : // unknown turn off motors
						MOT_CONR = 0;							// turn off right motor
						MOT_CONL = 0;							// turn off left motor
						DirSafe = 1;							// Motors off allow reverse
					break;

				}	// end voice control case '0'
				break;
					case '1' :	// turn right 	
						MOT_CONR = 0;						// turn off right motor
						MOT_CONL = 0x00000132;				// left on super low to turn right
						DirSafe = 0;						// Motors running do not allow reverse
					break;
					case '2' :	// turn left 	
						MOT_CONR = 0x00000132;				// right on super low to turn left
						MOT_CONL = 0;						// turn off left motor
						DirSafe = 0;						// Motors running do not allow reverse
					break;
					case '3' :	// slow 	
						MOT_CONR = 0x00000164;				// medium speed
						MOT_CONL = 0x00000164;				// medium speed
						DirSafe = 0;						// Motors running do not allow reverse
					break;
					case '4' :	// fast	
						MOT_CONR = 0x000001F0;				// fast
						MOT_CONL = 0x000001F0;				// fast
						DirSafe = 0;						// Motors running do not allow reverse
					break;
					case '5' :	// stop	
						NX4IO_RGBLED_setDutyCycle(RGB1, 0, 0, 0); 	// turn off light servo
						MOT_CONR = 0;						// turn off right motor
						MOT_CONL = 0;						// turn off left motor
						DirSafe = 1;						// Motors off allow reverse
						OLEDrgb_Clear(&pmodOLEDrgb_inst);	// clear possible blinkers
					break;
					case '6' :	// reverse	
					if (DirSafe){							// if motor off
						if (DirFlag == 0){					// flag of direction state
							// write the motor direction reverse
							IPHB3_mWriteReg(XPAR_IPHB3_0_S00_AXI_BASEADDR, IPHB3_S00_AXI_SLV_REG3_OFFSET, 0x00000001);
							IPHB3_mWriteReg(XPAR_IPHB3_1_S00_AXI_BASEADDR, IPHB3_S00_AXI_SLV_REG3_OFFSET, 0x00000001);					
							MOT_CONR = 0;					// turn off right motor
							MOT_CONL = 0;					// turn off left motor
							output = '5';					// exit direction to safe stop state
						}
						else {	
							// write the motor direction forward
							IPHB3_mWriteReg(XPAR_IPHB3_0_S00_AXI_BASEADDR, IPHB3_S00_AXI_SLV_REG3_OFFSET, 0x00000000);
							IPHB3_mWriteReg(XPAR_IPHB3_1_S00_AXI_BASEADDR, IPHB3_S00_AXI_SLV_REG3_OFFSET, 0x00000000);	
							DirFlag = 0;					// allow toggle of direction by flag
							output = '5';					// exit direction to safe stop state
						}
						MOT_CONR = 0;						// turn off right motor
						MOT_CONL = 0;						// turn off left motor
						DirSafe = 1;						// Motors off allow reverse
					}
					else {
						MOT_CONR = MOT_CONR;				// motor remains
						MOT_CONL = MOT_CONL;				// motor remains				
					}
					break;
					case '7' : // search light
						NX4IO_RGBLED_setDutyCycle(RGB1, 0, 255, 0); // set pwm for servo output
						MOT_CONR = MOT_CONR;					// motor remains
						MOT_CONL = MOT_CONL;					// motor remains;
					break;
					default :	// continue if newline etc.
						MOT_CONR = MOT_CONR;				// motor remains
						MOT_CONL = MOT_CONL;				// motor remains
		}
	}


	// Axi write commands to write determined motor values
	IPHB3_mWriteReg(XPAR_IPHB3_0_S00_AXI_BASEADDR, IPHB3_S00_AXI_SLV_REG0_OFFSET, MOT_CONR);
	IPHB3_mWriteReg(XPAR_IPHB3_1_S00_AXI_BASEADDR, IPHB3_S00_AXI_SLV_REG0_OFFSET, MOT_CONL);
				
	// capture arduino values from voice commands
	gpio_in = XGpio_DiscreteRead(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL);
						
	// timer for turn signals on OLED 			
	Blink_flag = Blink_flag + 1;	// increment counter
	if (Blink_flag == 1){			// start of count clear screen (turn off lights)
		OLEDrgb_Clear(&pmodOLEDrgb_inst);
	}
			
	if (Blink_flag == 15){			// turn on for half period
		if (output == '0') {		// if run by voice
			if (gpio_in == 1){		// turn right light on
				OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst, 5, 30, 45, 60, OLEDrgb_BuildRGB(r, g, b), 1,OLEDrgb_BuildRGB(r, g, b));
			}
			if (gpio_in == 2){		// turn on left light
				OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst, 50, 30, 90, 60, OLEDrgb_BuildRGB(r, g, b), 1,OLEDrgb_BuildRGB(r, g, b));	
			}
		}
		if (output == '1'){			// BT right light on
			OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst, 5, 30, 45, 60, OLEDrgb_BuildRGB(r, g, b), 1,OLEDrgb_BuildRGB(r, g, b));
		}
		if (output == '2'){			// BT left light on
			OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst, 50, 30, 90, 60, OLEDrgb_BuildRGB(r, g, b), 1,OLEDrgb_BuildRGB(r, g, b));	
		}
	}
	if (Blink_flag >= 30){			// reset flag
		Blink_flag = 0;
	}

	// Accelerometer read and display 
	// get raw data
	accel_x = XGpio_DiscreteRead(&GPIOInst0, GPIO_0_ACCIN_0_CHANNEL); 
	// convert to speed
	g_xaxis = 0.00391*accel_x - 1;
	// display as int on SSEG
	NX4IO_SSEG_putu32Dec(SysMonFractionToInt(g_xaxis), false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT3, true);	

	// collision detection blinking light			
	if (sensor){
		// flash red if obstacle causing halt
		NX4IO_RGBLED_setDutyCycle(RGB2, 255, 0, 0);
	}	
	else {
		// flash light green if no obstacle
		NX4IO_RGBLED_setDutyCycle(RGB2, 0, 120, 0);
	}

	// output text and number to top of OLED currently written in black
	// Thought was to use for speed before we went to SSEG
	// can be converted to show actual distance from sensor
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst,OLEDrgb_BuildRGB(0, 0,0));
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 0);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,output);
	PMDIO_putnum(&pmodOLEDrgb_inst, Blink_flag, 10);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"  ");

	} 	// rotary button has been pressed - exit the loop

	// this allows for a soft reset that just calls the function again
	// turn off RGB leds
	NX4IO_RGBLED_setDutyCycle(RGB1, 0, 0, 0);        
	NX4IO_RGBLED_setDutyCycle(RGB2, 0, 0, 0);
	// shut off motors
	MOT_CONR = 0x0;
	MOT_CONL = 0x0;	
	IPHB3_mWriteReg(XPAR_IPHB3_0_S00_AXI_BASEADDR, IPHB3_S00_AXI_SLV_REG0_OFFSET, MOT_CONR);
	IPHB3_mWriteReg(XPAR_IPHB3_1_S00_AXI_BASEADDR, IPHB3_S00_AXI_SLV_REG0_OFFSET, MOT_CONL);
	xil_printf("Lunar module halting\n");

	return;
}




/*********************** HELPER FUNCTIONS ***********************************/

/****************************************************************************/
/**
* insert delay (in microseconds) between instructions.
*
* This function should be in libc but it seems to be missing.  This emulation implements
* a delay loop with (really) approximate timing; not perfect but it gets the job done.
*
* @param	usec is the requested delay in microseconds
*
* @return	*NONE*
*
* @note
* This emulation assumes that the microblaze is running @ 100MHz and takes 15 clocks
* per iteration - this is probably totally bogus but it's a start.
*
*****************************************************************************/

static const u32	DELAY_1US_CONSTANT	= 15;	// constant for 1 microsecond delay

void usleep(u32 usec)
{
	volatile u32 i, j;

	for (i = 0; i < usec; i++)
	{
		for (j = 0; j < DELAY_1US_CONSTANT; j++);
	}
	return;
}


/****************************************************************************/
/**
* initialize the Nexys4 LEDs and seven segment display digits
*
* Initializes the NX4IO driver, turns off all of the LEDs and blanks the seven segment display
*
* @param	BaseAddress is the memory mapped address of the start of the Nexys4 registers
*
* @return	XST_SUCCESS if initialization succeeds.  XST_FAILURE otherwise
*
* @note
* The NX4IO_initialize() function calls the NX4IO self-test.  This could
* cause the program to hang if the hardware was not configured properly
*
*****************************************************************************/
int do_init_nx4io(u32 BaseAddress)
{
	int sts;

	// initialize the NX4IO driver
	sts = NX4IO_initialize(BaseAddress);
	if (sts == XST_FAILURE)
		return XST_FAILURE;

	// turn all of the LEDs off using the "raw" set functions
	// functions should mask out the unused bits..something to check w/
	// the debugger when we bring the drivers up for the first time
	NX4IO_setLEDs(0xFFF0000);
	NX4IO_RGBLED_setRGB_DATA(RGB1, 0xFF000000);
	NX4IO_RGBLED_setRGB_DATA(RGB2, 0xFF000000);
	NX4IO_RGBLED_setRGB_CNTRL(RGB1, 0xFFFFFFF0);
	NX4IO_RGBLED_setRGB_CNTRL(RGB2, 0xFFFFFFFC);

	// set all of the display digits to blanks and turn off
	// the decimal points using the "raw" set functions.
	// These registers are formatted according to the spec
	// and should remain unchanged when written to Nexys4IO...
	// something else to check w/ the debugger when we bring the
	// drivers up for the first time
	NX4IO_SSEG_setSSEG_DATA(SSEGHI, 0x0058E30E);
	NX4IO_SSEG_setSSEG_DATA(SSEGLO, 0x00144116);

	return XST_SUCCESS;

}


/*********************** DISPLAY-RELATED FUNCTIONS ***********************************/

/****************************************************************************/
/**
* Converts an integer to ASCII characters
*
* algorithm borrowed from ReactOS system libraries
*
* Converts an integer to ASCII in the specified base.  Assumes string[] is
* long enough to hold the result plus the terminating null
*
* @param 	value is the integer to convert
* @param 	*string is a pointer to a buffer large enough to hold the converted number plus
*  			the terminating null
* @param	radix is the base to use in conversion,
*
* @return  *NONE*
*
* @note
* No size check is done on the return string size.  Make sure you leave room
* for the full string plus the terminating null in string
*****************************************************************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix)
{
	char tmp[33];
	char *tp = tmp;
	int32_t i;
	uint32_t v;
	int32_t  sign;
	char *sp;

	if (radix > 36 || radix <= 1)
	{
		return;
	}

	sign = ((10 == radix) && (value < 0));
	if (sign)
	{
		v = -value;
	}
	else
	{
		v = (uint32_t) value;
	}

  	while (v || tp == tmp)
  	{
		i = v % radix;
		v = v / radix;
		if (i < 10)
		{
			*tp++ = i+'0';
		}
		else
		{
			*tp++ = i + 'a' - 10;
		}
	}
	sp = string;

	if (sign)
		*sp++ = '-';

	while (tp > tmp)
		*sp++ = *--tp;
	*sp = 0;

  	return;
}


/****************************************************************************/
/**
* Write a 32-bit unsigned hex number to PmodOLEDrgb in Hex
*
* Writes  32-bit unsigned number to the pmodOLEDrgb display starting at the current
* cursor position.
*
* @param num is the number to display as a hex value
*
* @return  *NONE*
*
* @note
* No size checking is done to make sure the string will fit into a single line,
* or the entire display, for that matter.  Watch your string sizes.
*****************************************************************************/
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num)
{
  char  buf[9];
  int32_t   cnt;
  char  *ptr;
  int32_t  digit;

  ptr = buf;
  for (cnt = 7; cnt >= 0; cnt--) {
    digit = (num >> (cnt * 4)) & 0xF;

    if (digit <= 9)
	{
      *ptr++ = (char) ('0' + digit);
	}
    else
	{
      *ptr++ = (char) ('a' - 10 + digit);
	}
  }

  *ptr = (char) 0;
  OLEDrgb_PutString(InstancePtr,buf);

  return;
}


/****************************************************************************/
/**
* Write a 32-bit number in Radix "radix" to LCD display
*
* Writes a 32-bit number to the LCD display starting at the current
* cursor position. "radix" is the base to output the number in.
*
* @param num is the number to display
*
* @param radix is the radix to display number in
*
* @return *NONE*
*
* @note
* No size checking is done to make sure the string will fit into a single line,
* or the entire display, for that matter.  Watch your string sizes.
*****************************************************************************/

void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix)
{
  char  buf[16];

  PMDIO_itoa(num, buf, radix);
  OLEDrgb_PutString(InstancePtr,buf);

  return;
}

/****************************************************************************/
/**
Convert a float to an int 
* @param FloatNum is the float to be converted
*
* @return result as coversion integer
* @note
* used in ADC for display
*****************************************************************************/

int SysMonFractionToInt(float FloatNum)
{
	float Temp;

	Temp = FloatNum;
	if (FloatNum < 0) {
		Temp = -(FloatNum);
	}

	return( ((int)((Temp -(float)((int)Temp)) * (1000.0f))));
}

/**************************** INTERRUPT HANDLERS ******************************/

/****************************************************************************/
/**
* Fixed interval timer interrupt handler
*
* Reads the GPIO port which reads back the hardware generated PWM wave for the RGB Leds
* Steps through edge detection to find a high and low count to determine the duty cycle
*
* @note
* software solution for pulse width detection 
 *****************************************************************************/

void FIT_Handler(void)
{
// not used atm
}



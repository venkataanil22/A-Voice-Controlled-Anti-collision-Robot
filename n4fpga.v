`timescale 1ns / 1ps

// n4fpga.v - Top level module for the ECE 544 Final Project 
//
// Copyright Chetan Bornarkar, Portland State University, 2016
// 
// Created By:	Roy Kravitz
// Modified By: Chetan Bornarkar, Randon Stasney, Aditya Pawar, 
//				Venkata Anil Viswanadha ,Brandon Biodrowski
// Date:		3/21/2017
// Version:		4.0
//
// Description:
// ------------
// This module provides the top level for 544 Final Project 
// The module assumes that a PmodOLED is plugged into the JB 
// expansion ports and that a BT2Pmod is plugged into the JA 
// expansion port.  The MAXSONAR Pmod is plugged into the JXADC
// [0].  The two HB3 Pmod motor drivers are plugged into the
// bottom or JC and JD. The binary input from the serial arduino
// conversion is on pins JC[3:0].  The servo pwm pin for the 
// light is JD[0]
// port (bottom row).  
//////////////////////////////////////////////////////////////////////
module n4fpga(
    input				clk,			// 100Mhz clock input
    input				btnC,			// center pushbutton
    input				btnU,			// UP (North) pusbhbutton
    input				btnL,			// LEFT (West) pushbutton
    input				btnD,			// DOWN (South) pushbutton  
    input				btnR,			// RIGHT (East) pushbutton
	input				btnCpuReset,	// CPU reset pushbutton
    input	[15:0]		sw,				// slide switches on Nexys 4
    output	[15:0] 		led,			// LEDs on Nexys 4   
    output              RGB1_Blue,      // RGB1 LED (LD16) 
    output              RGB1_Green,
    output              RGB1_Red,
    output              RGB2_Blue,      // RGB2 LED (LD17)
    output              RGB2_Green,
    output              RGB2_Red,
    output [7:0]        an,             // Seven Segment display
    output [6:0]        seg,
    output              dp,             // decimal point display on the seven segment 
    
    input				uart_rtl_rxd,	// USB UART Rx and Tx on Nexys 4
    output				uart_rtl_txd,	
    
	inout   [7:0]       JA,             // JA BT2 Pmod connector 
	                                    // both rows are used 
    inout	[7:0] 		JB,				// JB OLED Pmod connector 
                                        // both rows are used 
    inout	[7:0] 		JC,				// JC Pmod connector - top arduino
										// bottom HB3 Pmod right motor
	inout	[7:0]		JD,				// JD Pmod connector - top servo
										// bottom HB3 Pmod left motor
	inout 				XA_N, 			// ground for ADC reference
	
	inout				XA_P,			// Analog input from MAXSONAR
	
	// SPI interface for Accelerometer
	output         sclk,   
    output         mosi,      
    input          miso,    
    output         ss   
);

// internal variables

// Clock and Reset 
wire				sysclk;              					// 100 MHz, 10 MHz
wire				sysreset_n, sysreset;					// active low reset, active high reset

// HB3 motor driver connection wires 
wire				HB3_FBAR, HB3_FBAL, DIRR, ENAR, DIRL, ENAL;
wire 	[3:0]		Arduino_Enc;							// decoded voice commands in binary

// GPIO pins 
wire	[31:0]	    gpio_in;								// embsys GPIO input port


// OLED pins 
wire 				pmodoledrgb_out_pin1_i, pmodoledrgb_out_pin1_io, pmodoledrgb_out_pin1_o, pmodoledrgb_out_pin1_t; 
wire 				pmodoledrgb_out_pin2_i, pmodoledrgb_out_pin2_io, pmodoledrgb_out_pin2_o, pmodoledrgb_out_pin2_t; 
wire 				pmodoledrgb_out_pin3_i, pmodoledrgb_out_pin3_io, pmodoledrgb_out_pin3_o, pmodoledrgb_out_pin3_t; 
wire 				pmodoledrgb_out_pin4_i, pmodoledrgb_out_pin4_io, pmodoledrgb_out_pin4_o, pmodoledrgb_out_pin4_t; 
wire 				pmodoledrgb_out_pin7_i, pmodoledrgb_out_pin7_io, pmodoledrgb_out_pin7_o, pmodoledrgb_out_pin7_t; 
wire 				pmodoledrgb_out_pin8_i, pmodoledrgb_out_pin8_io, pmodoledrgb_out_pin8_o, pmodoledrgb_out_pin8_t; 
wire 				pmodoledrgb_out_pin9_i, pmodoledrgb_out_pin9_io, pmodoledrgb_out_pin9_o, pmodoledrgb_out_pin9_t; 
wire 				pmodoledrgb_out_pin10_i, pmodoledrgb_out_pin10_io, pmodoledrgb_out_pin10_o, pmodoledrgb_out_pin10_t;

// Bluetooth BT2 pins
wire 				pmodbt_out_pin1_i, pmodbt_out_pin1_io, pmodbt_out_pin1_o, pmodbt_out_pin1_t; 
wire 				pmodbt_out_pin2_i, pmodbt_out_pin2_io, pmodbt_out_pin2_o, pmodbt_out_pin2_t; 
wire 				pmodbt_out_pin3_i, pmodbt_out_pin3_io, pmodbt_out_pin3_o, pmodbt_out_pin3_t; 
wire 				pmodbt_out_pin4_i, pmodbt_out_pin4_io, pmodbt_out_pin4_o, pmodbt_out_pin4_t; 
wire 				pmodbt_out_pin7_i, pmodbt_out_pin7_io, pmodbt_out_pin7_o, pmodbt_out_pin7_t; 
wire 				pmodbt_out_pin8_i, pmodbt_out_pin8_io, pmodbt_out_pin8_o, pmodbt_out_pin8_t; 
wire 				pmodbt_out_pin9_i, pmodbt_out_pin9_io, pmodbt_out_pin9_o, pmodbt_out_pin9_t; 
wire 				pmodbt_out_pin10_i, pmodbt_out_pin10_io, pmodbt_out_pin10_o, pmodbt_out_pin10_t;

wire    [15:0]      led_int;                // Nexys4IO drives these outputs

assign sysclk = clk;						// define block clock 

// connect driving wires back through gpio for voice control	
assign gpio_in = {28'b0, Arduino_Enc};

// Drive the leds from the signal generated by the microblaze 
assign led = led_int;                   	// LEDs are driven by led

// make the connections
// system-wide signals
assign sysreset_n = btnCpuReset;				// The CPU reset pushbutton is asserted low.  The other pushbuttons are asserted high
												// but the microblaze for Nexys 4 expects reset to be asserted low
assign sysreset = ~sysreset_n;					// Generate a reset signal that is asserted high for any logic blocks expecting it.

// Pmod OLED connections 
assign JB[0] = pmodoledrgb_out_pin1_io;
assign JB[1] = pmodoledrgb_out_pin2_io;
assign JB[2] = pmodoledrgb_out_pin3_io;
assign JB[3] = pmodoledrgb_out_pin4_io;
assign JB[4] = pmodoledrgb_out_pin7_io;
assign JB[5] = pmodoledrgb_out_pin8_io;
assign JB[6] = pmodoledrgb_out_pin9_io;
assign JB[7] = pmodoledrgb_out_pin10_io;

// Pmod BT2 connections
assign JA[0] = pmodbt_out_pin1_io;
assign JA[1] = pmodbt_out_pin2_io;
assign JA[2] = pmodbt_out_pin3_io;
assign JA[3] = pmodbt_out_pin4_io;
assign JA[4] = pmodbt_out_pin7_io;
assign JA[5] = pmodbt_out_pin8_io;
assign JA[6] = pmodbt_out_pin9_io;
assign JA[7] = pmodbt_out_pin10_io;

assign JD[0] = RGB1_Green;				// servo pwm pin
// Pmod HB3 lower pins
assign JD[4] = DIRL;    
assign JD[5] = ENAL;
assign HB3_FBAR = JD[6];
assign JD[7] = 1'b0;

assign Arduino_Enc = JC [3:0];			// binary instruction from arduino
// Pmod HB3 lower pins
assign JC[4] = DIRR;
assign JC[5] = ENAR;
assign HB3_FBAR = JC[6];
assign  JC[7] = 1'b0;

design_1 Design_1
       (// PMOD OLED pins 
        .PmodOLEDrgb_out_pin10_i(pmodoledrgb_out_pin10_i),
	    .PmodOLEDrgb_out_pin10_o(pmodoledrgb_out_pin10_o),
	    .PmodOLEDrgb_out_pin10_t(pmodoledrgb_out_pin10_t),
	    .PmodOLEDrgb_out_pin1_i(pmodoledrgb_out_pin1_i),
	    .PmodOLEDrgb_out_pin1_o(pmodoledrgb_out_pin1_o),
	    .PmodOLEDrgb_out_pin1_t(pmodoledrgb_out_pin1_t),
	    .PmodOLEDrgb_out_pin2_i(pmodoledrgb_out_pin2_i),
	    .PmodOLEDrgb_out_pin2_o(pmodoledrgb_out_pin2_o),
	    .PmodOLEDrgb_out_pin2_t(pmodoledrgb_out_pin2_t),
	    .PmodOLEDrgb_out_pin3_i(pmodoledrgb_out_pin3_i),
	    .PmodOLEDrgb_out_pin3_o(pmodoledrgb_out_pin3_o),
	    .PmodOLEDrgb_out_pin3_t(pmodoledrgb_out_pin3_t),
	    .PmodOLEDrgb_out_pin4_i(pmodoledrgb_out_pin4_i),
	    .PmodOLEDrgb_out_pin4_o(pmodoledrgb_out_pin4_o),
	    .PmodOLEDrgb_out_pin4_t(pmodoledrgb_out_pin4_t),
	    .PmodOLEDrgb_out_pin7_i(pmodoledrgb_out_pin7_i),
	    .PmodOLEDrgb_out_pin7_o(pmodoledrgb_out_pin7_o),
	    .PmodOLEDrgb_out_pin7_t(pmodoledrgb_out_pin7_t),
	    .PmodOLEDrgb_out_pin8_i(pmodoledrgb_out_pin8_i),
	    .PmodOLEDrgb_out_pin8_o(pmodoledrgb_out_pin8_o),
	    .PmodOLEDrgb_out_pin8_t(pmodoledrgb_out_pin8_t),
	    .PmodOLEDrgb_out_pin9_i(pmodoledrgb_out_pin9_i),
	    .PmodOLEDrgb_out_pin9_o(pmodoledrgb_out_pin9_o),
	    .PmodOLEDrgb_out_pin9_t(pmodoledrgb_out_pin9_t),
		// PMOD BT2 pins
        .Pmod_out_pin10_i(pmodbt_out_pin10_i),
	    .Pmod_out_pin10_o(pmodbt_out_pin10_o),
	    .Pmod_out_pin10_t(pmodbt_out_pin10_t),
	    .Pmod_out_pin1_i(pmodbt_out_pin1_i),
	    .Pmod_out_pin1_o(pmodbt_out_pin1_o),
	    .Pmod_out_pin1_t(pmodbt_out_pin1_t),
	    .Pmod_out_pin2_i(pmodbt_out_pin2_i),
	    .Pmod_out_pin2_o(pmodbt_out_pin2_o),
	    .Pmod_out_pin2_t(pmodbt_out_pin2_t),
	    .Pmod_out_pin3_i(pmodbt_out_pin3_i),
	    .Pmod_out_pin3_o(pmodbt_out_pin3_o),
	    .Pmod_out_pin3_t(pmodbt_out_pin3_t),
	    .Pmod_out_pin4_i(pmodbt_out_pin4_i),
	    .Pmod_out_pin4_o(pmodbt_out_pin4_o),
	    .Pmod_out_pin4_t(pmodbt_out_pin4_t),
	    .Pmod_out_pin7_i(pmodbt_out_pin7_i),
	    .Pmod_out_pin7_o(pmodbt_out_pin7_o),
	    .Pmod_out_pin7_t(pmodbt_out_pin7_t),
	    .Pmod_out_pin8_i(pmodbt_out_pin8_i),
	    .Pmod_out_pin8_o(pmodbt_out_pin8_o),
	    .Pmod_out_pin8_t(pmodbt_out_pin8_t),
	    .Pmod_out_pin9_i(pmodbt_out_pin9_i),
	    .Pmod_out_pin9_o(pmodbt_out_pin9_o),
	    .Pmod_out_pin9_t(pmodbt_out_pin9_t),

	    // GPIO pins 
        .gpio_0_GPIO_tri_i(gpio_in),

        // RGB1/2 Led's 
        .RGB1_Blue(RGB1_Blue),
        .RGB1_Green(RGB1_Green),
        .RGB1_Red(RGB1_Red),
        .RGB2_Blue(RGB2_Blue),
        .RGB2_Green(RGB2_Green),
        .RGB2_Red(RGB2_Red),
        // Seven Segment Display anode control  
        .an(an),
        .dp(dp),
        .led(led_int),
        .seg(seg),
        // Push buttons and switches  
        .btnC(btnC),
        .btnD(btnD),
        .btnL(btnL),
        .btnR(btnR),
        .btnU(btnU),
        .sw(sw),
        // reset and clock 
        .sysreset_n(sysreset_n),
        .sysclk(sysclk),
        // ADC pins
		.Vaux3_v_n(XA_N),
		.Vaux3_v_p(XA_P),
		// Accelerometer SPI pins
		.miso(miso),
        .mosi(mosi),
        .sclk(sclk),
        .ss(ss),

		// Stdin / out pins
        .uart_rtl_rxd(uart_rtl_rxd),
        .uart_rtl_txd(uart_rtl_txd),
		
		// HB3
		.ENABLE(ENAL),	
		.DIRECTION(DIRR),
		.SA(HB3_FBAR),
		.SB(1'b1),
		.ENABLE_1(ENAR),
		.DIRECTION_1(DIRL),
		.SA_1(HB3_FBAL),
		.SB_1(1'b1)
);
     
// Tristate buffers for the BT2 pins
// generated by PMOD bridge component.  Many
// of these signals are not tri-state.
IOBUF pmodbt_out_pin1_iobuf
(
    .I(pmodbt_out_pin1_o),
    .IO(pmodbt_out_pin1_io),
    .O(pmodbt_out_pin1_i),
    .T(pmodbt_out_pin1_t)
);

IOBUF pmodbt_out_pin2_iobuf
(
    .I(pmodbt_out_pin2_o),
    .IO(pmodbt_out_pin2_io),
    .O(pmodbt_out_pin2_i),
    .T(pmodbt_out_pin2_t)
);

IOBUF pmodbt_out_pin3_iobuf
(
    .I(pmodbt_out_pin3_o),
    .IO(pmodbt_out_pin3_io),
    .O(pmodbt_out_pin3_i),
    .T(pmodbt_out_pin3_t)
);

IOBUF pmodbt_out_pin4_iobuf
(
    .I(pmodbt_out_pin4_o),
    .IO(pmodbt_out_pin4_io),
    .O(pmodbt_out_pin4_i),
    .T(pmodbt_out_pin4_t)
);

IOBUF pmodbt_out_pin7_iobuf
(
    .I(pmodbt_out_pin7_o),
    .IO(pmodbt_out_pin7_io),
    .O(pmodbt_out_pin7_i),
    .T(pmodbt_out_pin7_t)
);

IOBUF pmodbt_out_pin8_iobuf
(
    .I(pmodbt_out_pin8_o),
    .IO(pmodbt_out_pin8_io),
    .O(pmodbt_out_pin8_i),
    .T(pmodbt_out_pin8_t)
);

IOBUF pmodbt_out_pin9_iobuf
(
    .I(pmodbt_out_pin9_o),
    .IO(pmodbt_out_pin9_io),
    .O(pmodbt_out_pin9_i),
    .T(pmodbt_out_pin9_t)
);

IOBUF pmodbt_out_pin10_iobuf
(
    .I(pmodbt_out_pin10_o),
    .IO(pmodbt_out_pin10_io),
    .O(pmodbt_out_pin10_i),
    .T(pmodbt_out_pin10_t)
);

// Tristate buffers for the pmodOLEDrgb pins
// generated by PMOD bridge component.  Many
// of these signals are not tri-state.
IOBUF pmodoledrgb_out_pin1_iobuf
(
    .I(pmodoledrgb_out_pin1_o),
    .IO(pmodoledrgb_out_pin1_io),
    .O(pmodoledrgb_out_pin1_i),
    .T(pmodoledrgb_out_pin1_t)
);

IOBUF pmodoledrgb_out_pin2_iobuf
(
    .I(pmodoledrgb_out_pin2_o),
    .IO(pmodoledrgb_out_pin2_io),
    .O(pmodoledrgb_out_pin2_i),
    .T(pmodoledrgb_out_pin2_t)
);

IOBUF pmodoledrgb_out_pin3_iobuf
(
    .I(pmodoledrgb_out_pin3_o),
    .IO(pmodoledrgb_out_pin3_io),
    .O(pmodoledrgb_out_pin3_i),
    .T(pmodoledrgb_out_pin3_t)
);

IOBUF pmodoledrgb_out_pin4_iobuf
(
    .I(pmodoledrgb_out_pin4_o),
    .IO(pmodoledrgb_out_pin4_io),
    .O(pmodoledrgb_out_pin4_i),
    .T(pmodoledrgb_out_pin4_t)
);

IOBUF pmodoledrgb_out_pin7_iobuf
(
    .I(pmodoledrgb_out_pin7_o),
    .IO(pmodoledrgb_out_pin7_io),
    .O(pmodoledrgb_out_pin7_i),
    .T(pmodoledrgb_out_pin7_t)
);

IOBUF pmodoledrgb_out_pin8_iobuf
(
    .I(pmodoledrgb_out_pin8_o),
    .IO(pmodoledrgb_out_pin8_io),
    .O(pmodoledrgb_out_pin8_i),
    .T(pmodoledrgb_out_pin8_t)
);

IOBUF pmodoledrgb_out_pin9_iobuf
(
    .I(pmodoledrgb_out_pin9_o),
    .IO(pmodoledrgb_out_pin9_io),
    .O(pmodoledrgb_out_pin9_i),
    .T(pmodoledrgb_out_pin9_t)
);

IOBUF pmodoledrgb_out_pin10_iobuf
(
    .I(pmodoledrgb_out_pin10_o),
    .IO(pmodoledrgb_out_pin10_io),
    .O(pmodoledrgb_out_pin10_i),
    .T(pmodoledrgb_out_pin10_t)
);



endmodule
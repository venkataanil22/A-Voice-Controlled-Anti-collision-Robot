`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: PSU
// Engineer: Randon Stasney, Aditya Pawar
// 
// Create Date: 02/15/2017 05:47:06 PM
// Design Name: IP for HP3
// Module Name: HB3IP
// Project Name: Project 2 motor control
// Target Devices: Nexys 4 fpga with hb3 pmod 
// Tool Versions: 16.2
// Description:    This converts the feedback to a frequency for control
// 
// Dependencies: 
// 
// Revision: 1.0
// Revision 0.01 - File Created
// Revision 1.0 - Added the debounce directly to the module
// Additional Comments:
// Sample count to convert rising edge detection directly to a mapped
// count to match the 0-255 range of pwm input.
//////////////////////////////////////////////////////////////////////////////////


module HB3IP (
    input clock,
    input reset,
    input SA,
    input SB,
    output [31:0]SA_rpm
    );
    
  reg SA_Old = 0;								// store old value to detect rising edge
  reg SB_Old = 0;
  reg [15:0] SA_Count = 0;						// keep count for known period
  reg [15:0] SB_Count = 0;
  reg [15:0] SA_Out = 0;						// location to store frequency count for output
  reg [15:0] SB_Out = 0;
  reg [31:0] counter = 32'd170454; 				// value to map to pwm for approx .25 sec period
  reg [7:0] debounce =	7'd99;					// debounce signal to take 1 of 100 samples from 100 MHZ clk

always @(posedge clock) begin
      if (!reset) begin							// reset to known state
        SA_Count <= 16'd0;
        SB_Count <= 16'd0;
        SA_Out <= 16'd0;
        SB_Out <= 16'd0;
        SA_Old <= 1'b0;
        SB_Old <= 1'b0;
        counter <= 32'd170454;
		debounce <=	7'd99;
      end  // peripheral reset 
      else begin								// default values to prevent latch inference
      SA_Out <= SA_Out;
      SB_Out <= SB_Out;
      SA_Count <= SA_Count;
      SB_Count <= SB_Count;
	  SA_Old <= SA_Old;
	  SB_Old <= SB_Old;
	  counter <= counter;
		if (debounce == 0) begin				// every 1 MHz sample the signal
			// detect rising edge and count how many in period
			if ((SA == 1'b1) & (SA_Old == 1'b0)) begin
				SA_Count <= SA_Count + 1;
				end
			else begin  
				SA_Count <= SA_Count;
				end  
			if ((SB == 1'b1) & (SB_Old == 1'b0)) begin
				SB_Count <= SB_Count + 1;
				end
			else begin  
				SB_Count <= SB_Count;
				end
			// reload counter and update output 
			if (counter == 0) begin
				SA_Out <= SA_Count;
				SB_Out <= SB_Count;
				SA_Count <= 0;
				SB_Count <= 0;
				counter <= 32'd170454;
			end
			// 
			else begin
			counter <= counter - 1;				// decrement count
			end 
			SA_Old <= SA;
			SB_Old <= SB;
			debounce <=	7'd99;					// reset count for debounce
		end
		else begin
			debounce <= debounce - 1;			// decrement debounce
		end
	end
end
	// output the detected frequency count to be written to axi memory
    assign SA_rpm = {SB_Out[15:0],SA_Out[15:0]}; 
endmodule

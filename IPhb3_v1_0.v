
`timescale 1 ns / 1 ps
//////////////////////////////////////////////////////////////////////////////////
// Company: PSU
// Engineer: Randon Stasney, Aditya Pawar
// 
// Create Date: 02/15/2017 05:47:06 PM
// Design Name: IP for HP3
// Module Name: IPhb3_v1_0
// Project Name: Project 2 motor control
// Target Devices: Nexys 4 fpga with hb3 pmod 
// Tool Versions: 16.2
// Description:    This interfaces pmod with axi bus
// 
// Dependencies: 
// 
// Revision: 1.0
// Revision 0.01 - File Created
// Additional Comments:
// Wrapper for pmod hb3 to interface with axi bus.  Instantiates all parts of the IP
// Added hardware protection for direction control
//////////////////////////////////////////////////////////////////////////////////

	module IPhb3_v1_0 #
	(
		// Parameters of Axi Slave Bus Interface S00_AXI
		parameter integer C_S00_AXI_DATA_WIDTH	= 32,
		parameter integer C_S00_AXI_ADDR_WIDTH	= 4
	)
	(	
		// user ports
        input wire SA,									// Feedback from Hall sensor
        input wire SB,									// to be used later
        output wire ENABLE,								// pwm signal used to drive motor
        output wire DIRECTION,							// direction control of motor
		// Ports of Axi Slave Bus Interface S00_AXI
		input wire  s00_axi_aclk,
		input wire  s00_axi_aresetn,
		input wire [C_S00_AXI_ADDR_WIDTH-1 : 0] s00_axi_awaddr,
		input wire [2 : 0] s00_axi_awprot,
		input wire  s00_axi_awvalid,
		output wire  s00_axi_awready,
		input wire [C_S00_AXI_DATA_WIDTH-1 : 0] s00_axi_wdata,
		input wire [(C_S00_AXI_DATA_WIDTH/8)-1 : 0] s00_axi_wstrb,
		input wire  s00_axi_wvalid,
		output wire  s00_axi_wready,
		output wire [1 : 0] s00_axi_bresp,
		output wire  s00_axi_bvalid,
		input wire  s00_axi_bready,
		input wire [C_S00_AXI_ADDR_WIDTH-1 : 0] s00_axi_araddr,
		input wire [2 : 0] s00_axi_arprot,
		input wire  s00_axi_arvalid,
		output wire  s00_axi_arready,
		output wire [C_S00_AXI_DATA_WIDTH-1 : 0] s00_axi_rdata,
		output wire [1 : 0] s00_axi_rresp,
		output wire  s00_axi_rvalid,
		input wire  s00_axi_rready
	);
	
	// declare the interconnect wires

    wire [31:0] SA_RPM; 					// Holds count from HB3IP to be written to memory

    wire [7:0] Motor_DC;					// PWM signal to drive motor
    wire Motor_enable;						// wire to motor driven by pwm
    wire Motor_direction;					// direction of motor rotation
    reg Safe_dir = 1'b0;					// value to hold prev dir to prevent short throuh h bridge
    

// Instantiation of Axi Bus Interface S00_AXI
	IPhb3_v1_0_S00_AXI # ( 
		.C_S_AXI_DATA_WIDTH(C_S00_AXI_DATA_WIDTH),
		.C_S_AXI_ADDR_WIDTH(C_S00_AXI_ADDR_WIDTH)
	) IPhb3_v1_0_S00_AXI_inst (
	    .SA_RPM(SA_RPM),
        .SA_W({SA,SA_D}),
        .Motor_DC(Motor_DC),
        .Motor_enable(Motor_enable),
        .Motor_direction(Motor_direction),
		.S_AXI_ACLK(s00_axi_aclk),
		.S_AXI_ARESETN(s00_axi_aresetn),
		.S_AXI_AWADDR(s00_axi_awaddr),
		.S_AXI_AWPROT(s00_axi_awprot),
		.S_AXI_AWVALID(s00_axi_awvalid),
		.S_AXI_AWREADY(s00_axi_awready),
		.S_AXI_WDATA(s00_axi_wdata),
		.S_AXI_WSTRB(s00_axi_wstrb),
		.S_AXI_WVALID(s00_axi_wvalid),
		.S_AXI_WREADY(s00_axi_wready),
		.S_AXI_BRESP(s00_axi_bresp),
		.S_AXI_BVALID(s00_axi_bvalid),
		.S_AXI_BREADY(s00_axi_bready),
		.S_AXI_ARADDR(s00_axi_araddr),
		.S_AXI_ARPROT(s00_axi_arprot),
		.S_AXI_ARVALID(s00_axi_arvalid),
		.S_AXI_ARREADY(s00_axi_arready),
		.S_AXI_RDATA(s00_axi_rdata),
		.S_AXI_RRESP(s00_axi_rresp),
		.S_AXI_RVALID(s00_axi_rvalid),
		.S_AXI_RREADY(s00_axi_rready)
	);
	
	// instantiate the pmod interface to nexys board for pwm
	Mpwm HB3pwm (
        .CLK(s00_axi_aclk),
        .RESET(s00_axi_aresetn),
        .MotorDC(Motor_DC),
        .PWM_CHANNEL_EN(Motor_enable),
        .enable(ENABLE)
      );
        

	// instantiate the freq detection for feedback from hall
	HB3IP IPdut (
        .clock(s00_axi_aclk),
        .reset(s00_axi_aresetn),
        .SA(SA),
        .SB(SB),
        .SA_rpm(SA_RPM)
        );

	// Protect the direction from switching by making sure enable is off
	// on the motor and that the frequency has no rotations
	// If both conditions pass allow direction to be changed
assign DIRECTION = Safe_dir;
always @ (posedge s00_axi_aclk) begin
    if ((Motor_enable == 0) & (SA_RPM == 0)) begin
        Safe_dir <= Motor_direction;
    end   
    else begin
        Safe_dir <= Safe_dir;
    end
end


	endmodule

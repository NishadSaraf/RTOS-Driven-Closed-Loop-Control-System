/**
* @author : Nishad Saraf, Chaitanya Deshpande
* This module is the top level for the pmodHB3 peripheral.

* Dependencies: 

* This module is dependent on Edge_detection.v (H bridge hardware) and PWMGeneration.v
* and makes use of signals produced in the AXI interface module for the periphal


*/

`timescale 1 ns / 1 ps

	module PmodHB3_v1_0 #
	(
		// Users to add parameters here

		// User parameters ends
		// Do not modify the parameters beyond this line

		// Parameters of Axi Slave Bus Interface S00_AXI
		parameter integer C_S00_AXI_DATA_WIDTH	= 32,
		parameter integer C_S00_AXI_ADDR_WIDTH	= 4
	)
	(
		// Users to add ports here
		
		// Output of hall sensor
        input wire PmodHB3_SA,
        
		// Direction of motor
        output wire PmodHB3_DIR,
		
		// PWM wave for driving motor
        output wire PmodHB3_EN, 
		
		// User ports ends
		// Do not modify the ports beyond this line

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
	
	// Duty cycle of PWM Wave
	wire [7:0] Duty_Cycle;
	
	// Edges of Hall sensor output (Frequency of hall sensor output wave)
	wire [31:0] SA_HighC;
	
// Instantiation of Axi Bus Interface S00_AXI
	PmodHB3_v1_0_S00_AXI # ( 
		.C_S_AXI_DATA_WIDTH(C_S00_AXI_DATA_WIDTH),
		.C_S_AXI_ADDR_WIDTH(C_S00_AXI_ADDR_WIDTH)
	) PmodHB3_v1_0_S00_AXI_inst (
	     // User Defined Signals
	    .SA_HighC_In(SA_HighC),
	   
	    .DIR_Out(PmodHB3_DIR),
        .Duty_Cycle_Out(Duty_Cycle),
	     // User defination ends
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

	// Add user logic here
	
	// Instantiation of PWMGeneration module
    PWMGeneration PmodHB3GEN
    (
        .CLK(s00_axi_aclk),                         // system clock
        .RESET(s00_axi_aresetn),                        // system reset
        .PWM_DC(Duty_Cycle),                        // duty cycle for PWM
        .PWM_OUT(PmodHB3_EN)                        // PWM outputs for each channel
    );
    
	// Instantiation of PWMDetection module module
    edge_detect PmodHB3DET
    (
	   
        .Highcount(SA_HighC),
        .clk(s00_axi_aclk),
        .rst(s00_axi_aresetn),
		.signal(PmodHB3_SA)
        
     );
	// User logic ends

	endmodule

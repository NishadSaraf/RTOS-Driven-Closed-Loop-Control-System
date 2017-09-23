// PWMGeneration.v - Parameterized PWM generator for Driving motor 
//
// @ Autor :- Nishad Saraf, Chaitanya Deshpande

// Description:
// This is a module used for PWM signal which is then used for driving motor
// CLK :  10 MHZ clock signal
// PWM_DC : Setting Duty cycle of PWM Wave
// PWM_OUT : Out PWM wave for Driving motor
//
//
//////////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps										
module PWMGeneration
#(
	//parameter declarations
	parameter 			RESET_POLARITY_LOW = 1,				// Reset is active-low?  (default is yes)
	parameter 			PWM_DC_WIDTH = 8					// Number of duty cycle bits for each channel (Default = 8-bit or 1/256 resolution)
)
(
	// port declarations
	input 						CLK, 						// system clock
	input						RESET,						// system reset
	input	[PWM_DC_WIDTH-1:0]	PWM_DC,						// duty cycle for PWM
	output	PWM_OUT						// PWM outputs for each channel
);

// local parameters
localparam PWM_CNTR_WIDTH = PWM_DC_WIDTH;               // make the PWM counter one bit wider than the duty cycle input to keep
                                                           
// use the RESET_POLARITY_LOW parameter to set the RESET_Int slevel
wire RESET_Int = RESET_POLARITY_LOW ? ~RESET : RESET;
								
// counter array.  Each PWM channel gets its own counter
reg			[PWM_CNTR_WIDTH-1:0]	pwm_cntr;

// output signals
reg			pwm_out_reg;

	
// PWM channel counters
// counters overflow to restart PWM period
always @(posedge CLK) begin
		if (RESET_Int) begin
			pwm_cntr <= {PWM_CNTR_WIDTH{1'b0}};
		end
		else begin
			pwm_cntr <= pwm_cntr + 1'b1;	
	end // for loop
end // pwm counters

// PWM output generation
// Block can be combinational because the counters are synchronized to the clock

always @* begin
    // control the  PWM channel
    if (pwm_cntr < PWM_DC) 
        pwm_out_reg = 1'b1;
    else
        pwm_out_reg = 1'b0;
        
               
end // PWM output generation

// assign the outputs
assign PWM_OUT = pwm_out_reg;

// initialize the counters and signal outputs.  Synthesis tool uses initial block for this
initial begin
		pwm_cntr = {PWM_CNTR_WIDTH{1'b0}};
		pwm_out_reg = 1'b0;
end  // initialization block

endmodule
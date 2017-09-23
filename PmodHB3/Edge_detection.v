/** file Edge_detection.v
* @Author: Nishad saraf, Chaitanya Deshpande
* This module calculates the frequency of the hall sensor output
* by calculating number of positive edges of hall sensor passing
* through 1 second.
*/
 
// module start
module edge_detect(TotalCount,clk,reset,signal);

// counter for storing highcount of Signal
output reg[31:0] TotalCount;  

//input signals
input clk,reset,signal;
                 
reg signal_neg;
wire pwm;
reg [31:0] sampleCounter = 32'b0;
reg [31:0] tempCount = 32'b0;


always@(posedge clk)
begin
    if(reset)
      begin 
	   sampleCounter <= 32'b0;
       tempCount <= 32'b0;
      end 
    else
    begin
        signal_neg <= signal;
        //Calculating positive edge count in 1 sec
        if(sampleCounter != 32'b00001011100110101100101000000000)        
        begin
            sampleCounter <= sampleCounter + 32'b1;
            if(pwm == 1'b1)
            begin
               //Increment highcount counter for positive edge
                tempCount <= tempCount + 1;
            end
        end 
        else
        begin
		
		// Assigning high count to the output port
            TotalCount <= tempCount;
           
		
		// Assigning value of high  count as well as clock counter to zero
            sampleCounter <= 32'b0;
            tempCount <= 32'b0;
           
        end
    end
end
//Edge detction of signal
assign pwm = signal & ~signal_neg;

endmodule

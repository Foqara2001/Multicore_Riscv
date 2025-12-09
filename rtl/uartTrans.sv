`timescale 1ns / 1ps

module uartTrans(
	input  logic i_wen,
	input  logic ready,
	input  logic [63:0] i_data,
	input  logic [63:0] address,
	output logic [7:0] uart_out_data,
	output logic valid,
	output logic o_wen,
	output logic stall
);
	
	always_comb begin : behavior
		// Default values - ALL outputs get default values
		stall         = 1'b0;
		o_wen         = 1'b0;
		valid         = 1'b0;
		uart_out_data = 8'h00;  // Explicit default value
		
		if(address == 64'h10000000 && i_wen)begin
			if(!ready)begin
				stall = 1'b1;
				valid = 1'b0;
				o_wen = 1'b0;
				uart_out_data = 8'h00;  // Added for completeness
			end
			else begin
				stall = 1'b0;
				valid = 1'b1;
				o_wen = 1'b0;
				uart_out_data = i_data[7:0];  // Only assign here
			end
		end
		// else case is covered by defaults
	end
endmodule
module core_rf
(
	input  logic i_rf_clk,
	input  logic i_rf_we3,
	input  logic [4:0] i_rf_a1,
	input  logic [4:0] i_rf_a2,
	input  logic [4:0] i_rf_a3,
	input  logic [63:0] i_rf_wd3,
	output logic [63:0] o_rf_rd1,
	output logic [63:0] o_rf_rd2
);

	logic [63:0] rf[31:0];

	

	always_ff @(negedge i_rf_clk) begin
		if (i_rf_we3 && (!(i_rf_a3 == 5'b0))) begin
			rf[i_rf_a3] <= i_rf_wd3;
		end
	end

	assign o_rf_rd1 = (i_rf_a1 == 5'b0) ? 64'b0 : rf[i_rf_a1];
	assign o_rf_rd2 = (i_rf_a2 == 5'b0) ? 64'b0 : rf[i_rf_a2];

endmodule
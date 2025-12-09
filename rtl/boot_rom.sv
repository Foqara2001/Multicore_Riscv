`timescale 1ns / 1ps

module boot_rom #(
    parameter CORE_ID = 0,
    parameter ROM_DEPTH = 512,
    parameter DATA_WIDTH = 32
)(
    input  logic i_clk,
    input  logic i_rst_n,
    input  logic [11:0] i_addr,
    input  logic i_req,
    output logic [DATA_WIDTH-1:0] o_data,
    output logic o_valid
);
    
    logic [DATA_WIDTH-1:0] rom [0:ROM_DEPTH-1];
    logic [DATA_WIDTH-1:0] data_reg;
    logic valid_reg;
    logic [8:0] word_addr;
    
    assign word_addr = i_addr[11:2];
    
    // Initialize with boot code
    initial begin
        // Default: all zeros
        for (int i = 0; i < ROM_DEPTH; i++) begin
            rom[i] = 32'h00000013;  // nop
        end
        
        if (CORE_ID == 0) begin
            // Core 0: Primary boot core
            rom[0] = 32'h00000093;  // li x1, 0
            rom[1] = 32'h00000113;  // li x2, 0
            rom[2] = 32'h00000193;  // li x3, 0
            rom[3] = 32'h00000213;  // li x4, 0
            rom[4] = 32'h00000293;  // li x5, 0
            rom[5] = 32'h00000313;  // li x6, 0
            rom[6] = 32'h00000393;  // li x7, 0
            rom[7] = 32'h00000413;  // li x8, 0
            rom[8] = 32'h10010113;  // addi sp, sp, 256
            rom[9] = 32'h0100006f;  // jal x0, main (offset 0x40)
        end else begin
            // Secondary cores: wait for wakeup
            rom[0] = 32'h10500073;  // wfi (wait for interrupt)
            rom[1] = 32'hffdff06f;  // j 0x0 (loop back)
        end
    end
    
    // Registered output for timing
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            data_reg <= 0;
            valid_reg <= 1'b0;
        end else begin
            if (i_req) begin
                data_reg <= rom[word_addr];
                valid_reg <= 1'b1;
            end else begin
                valid_reg <= 1'b0;
            end
        end
    end
    
    assign o_data = data_reg;
    assign o_valid = valid_reg;
    
endmodule

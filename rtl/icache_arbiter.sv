`timescale 1ns / 1ps

module icache_arbiter #(
    parameter N_CORES = 4,
    parameter ADDR_WIDTH = 32,  // Changed from 64
    parameter BLOCK_WIDTH = 256,
    parameter INSTR_WIDTH = 32
)(
    input  logic i_clk,
    input  logic i_rst_n,
    
    // Core interfaces
    input  logic [N_CORES-1:0] i_core_req,
    input  logic [N_CORES-1:0][ADDR_WIDTH-1:0] i_core_addr,
    output logic [N_CORES-1:0][BLOCK_WIDTH-1:0] o_core_rdata,
    output logic [N_CORES-1:0] o_core_done,
    
    // Shared L2 interface
    output logic o_l2_req,
    output logic [ADDR_WIDTH-1:0] o_l2_addr,
    input  logic [BLOCK_WIDTH-1:0] i_l2_rdata,
    input  logic i_l2_done,
    
    // Boot ROM interface
    input  logic [ADDR_WIDTH-1:0] i_boot_base,
    input  logic [ADDR_WIDTH-1:0] i_boot_size,
    output logic [N_CORES-1:0][11:0] o_boot_addr,
    output logic [N_CORES-1:0] o_boot_req,
    input  logic [N_CORES-1:0][INSTR_WIDTH-1:0] i_boot_data,
    input  logic [N_CORES-1:0] i_boot_valid
);

    // Check if address is in boot ROM range
    function automatic logic is_boot_rom_addr(
        input logic [ADDR_WIDTH-1:0] addr
    );
        return (addr >= i_boot_base) && (addr < (i_boot_base + i_boot_size));
    endfunction

    typedef enum logic [2:0] {
        IDLE,
        CHECK_REQ,
        BOOT_ROM_ACCESS,
        L2_REQUEST,
        L2_WAIT_RESPONSE,
        L2_RESPONSE,
        DELIVER_DATA
    } arb_state_t;
    
    arb_state_t arb_state;
    
    // Arbitration signals
    logic [N_CORES-1:0] req_pending;
    logic [N_CORES-1:0] req_served;
    logic [N_CORES-1:0] set_req_served;
    logic [$clog2(N_CORES)-1:0] selected_core;
    logic selected_core_valid;
    
    // Round-robin arbitration
    logic [$clog2(N_CORES)-1:0] rr_pointer;
    
    // Registered request info
    logic [ADDR_WIDTH-1:0] selected_addr;
    logic is_boot_rom;
    logic [INSTR_WIDTH-1:0] boot_data_latched;
    
    // Boot ROM address calculation
    logic [N_CORES-1:0][11:0] boot_offset;
    
    generate
        for (genvar i = 0; i < N_CORES; i++) begin : boot_offset_calc
            assign boot_offset[i] = selected_addr[11:0] - i_boot_base[11:0];
        end
    endgenerate
    
    // Core request tracking
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            req_pending <= '0;
        end else begin
            for (int i = 0; i < N_CORES; i++) begin
                if (i_core_req[i] && !req_served[i]) begin
                    req_pending[i] <= 1'b1;
                end
                if (set_req_served[i]) begin
                    req_pending[i] <= 1'b0;
                end
            end
        end
    end
    
    // req_served register
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            req_served <= '0;
        end else begin
            req_served <= set_req_served;
        end
    end
    
    logic found;
    // Round-robin arbiter for pending requests
    always_comb begin
        found = 1'b0;
        selected_core = 0;
        selected_core_valid = 1'b0;
        
        for (int i = 0; i < N_CORES; i++) begin
            automatic int idx = (rr_pointer + i) % N_CORES;
            
            if (!found && req_pending[idx] && !req_served[idx]) begin
                selected_core = idx;
                selected_core_valid = 1'b1;
                found = 1'b1;
            end
        end
    end
    
    // Main FSM
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            arb_state <= IDLE;
            rr_pointer <= 0;
            o_l2_req <= 1'b0;
            o_core_done <= '0;
            o_core_rdata <= '0;
            o_boot_req <= '0;
            o_boot_addr <= '0;
            selected_addr <= '0;
            is_boot_rom <= 1'b0;
            boot_data_latched <= '0;
            set_req_served <= '0;
        end else begin
            // Default outputs
            o_core_done <= '0;
            o_boot_req <= '0;
            set_req_served <= '0;
            
            case (arb_state)
                IDLE: begin
                    if (selected_core_valid) begin
                        arb_state <= CHECK_REQ;
                        selected_addr <= i_core_addr[selected_core];
                        is_boot_rom <= is_boot_rom_addr(i_core_addr[selected_core]);
                    end
                end
                
                CHECK_REQ: begin
                    if (is_boot_rom) begin
                        arb_state <= BOOT_ROM_ACCESS;
                        o_boot_req[selected_core] <= 1'b1;
                        o_boot_addr[selected_core] <= boot_offset[selected_core];
                    end else begin
                        arb_state <= L2_REQUEST;
                        o_l2_req <= 1'b1;
                        o_l2_addr <= selected_addr;
                    end
                end
                
                BOOT_ROM_ACCESS: begin
                    if (i_boot_valid[selected_core]) begin
                        boot_data_latched <= i_boot_data[selected_core];
                        arb_state <= DELIVER_DATA;
                        o_boot_req[selected_core] <= 1'b0;
                    end
                end
                
                L2_REQUEST: begin
                    if (i_l2_done) begin
                        arb_state <= L2_RESPONSE;
                        o_l2_req <= 1'b0;
                    end else begin
                        arb_state <= L2_WAIT_RESPONSE;
                    end
                end
                
                L2_WAIT_RESPONSE: begin
                    if (i_l2_done) begin
                        arb_state <= L2_RESPONSE;
                        o_l2_req <= 1'b0;
                    end
                end
                
                L2_RESPONSE: begin
                    arb_state <= DELIVER_DATA;
                end
                
                DELIVER_DATA: begin
                    if (is_boot_rom) begin
                        for (int i = 0; i < BLOCK_WIDTH/INSTR_WIDTH; i++) begin
                            o_core_rdata[selected_core][(i+1)*INSTR_WIDTH-1 -: INSTR_WIDTH] <= 
                                boot_data_latched;
                        end
                    end else begin
                        o_core_rdata[selected_core] <= i_l2_rdata;
                    end
                    
                    o_core_done[selected_core] <= 1'b1;
                    set_req_served[selected_core] <= 1'b1;
                    rr_pointer <= (selected_core + 1) % N_CORES;
                    arb_state <= IDLE;
                end
                
                default: begin
                    arb_state <= IDLE;
                end
            endcase
        end
    end
    
endmodule

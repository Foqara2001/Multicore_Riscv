`timescale 1ns / 1ps

module mesi_coherence_controller #(
    parameter N_CORES = 4,
    parameter ADDR_WIDTH = 32,  // Changed from 64
    parameter DATA_WIDTH = 256
)(
    input  logic clk,
    input  logic rst_n,
    
    // Core interfaces - FIXED to match core_top
    input  logic [N_CORES-1:0] i_cache_req,
    input  logic [N_CORES-1:0][ADDR_WIDTH-1:0] i_cache_addr,
    input  logic [N_CORES-1:0][DATA_WIDTH-1:0] i_cache_wdata,
    input  logic [N_CORES-1:0][ADDR_WIDTH-1:0] i_cache_waddr,
    input  logic [N_CORES-1:0] i_cache_wvalid,
    input  logic [N_CORES-1:0][7:0] i_cache_wstrb,
    output logic [N_CORES-1:0][DATA_WIDTH-1:0] o_cache_rdata,
    output logic [N_CORES-1:0] o_cache_rdone,
    output logic [N_CORES-1:0] o_cache_wdone,
    
    // L2 cache interface - FIXED to match shared_l2_cache
    output logic o_l2_req,
    output logic [ADDR_WIDTH-1:0] o_l2_addr,
    output logic [DATA_WIDTH-1:0] o_l2_wdata,
    output logic [7:0] o_l2_wstrb,
    output logic o_l2_op,
    input  logic [DATA_WIDTH-1:0] i_l2_rdata,
    input  logic i_l2_done,
    
    // Coherence states
    output logic [N_CORES-1:0][1:0] o_coherence_state
);

    // MESI states
    typedef enum logic [1:0] {
        M_MODIFIED  = 2'b00,
        E_EXCLUSIVE = 2'b01,
        S_SHARED    = 2'b10,
        I_INVALID   = 2'b11
    } mesi_state_t;
    
    // Tag and index for cache lines
    localparam TAG_WIDTH = ADDR_WIDTH - 6;  // 64-byte cache lines
    localparam INDEX_WIDTH = 8;
    
    // Directory entry
    typedef struct packed {
        mesi_state_t [N_CORES-1:0] core_state;
        logic [TAG_WIDTH-1:0] tag;
        logic valid;
        logic dirty;
    } directory_entry_t;
    
    directory_entry_t directory [2**INDEX_WIDTH-1:0];
    
    // Current transaction
    logic [N_CORES-1:0] pending_core;
    logic [ADDR_WIDTH-1:0] pending_addr;
    logic [DATA_WIDTH-1:0] pending_wdata;
    logic [7:0] pending_wstrb;
    logic pending_op;
    logic pending_we;
    
    // State machine
    typedef enum logic [2:0] {
        IDLE,
        CHECK_DIRECTORY,
        SNOOP_REQUEST,
        WAIT_SNOOP_RESPONSE,
        L2_ACCESS,
        UPDATE_DIRECTORY,
        SEND_RESPONSE
    } state_t;
    
    state_t state, next_state;
    
    // Address decomposition
    logic [TAG_WIDTH-1:0] addr_tag;
    logic [INDEX_WIDTH-1:0] addr_index;
    logic [5:0] addr_offset;
    
    assign {addr_tag, addr_index, addr_offset} = pending_addr;
    
    // Snoop signals
    logic [N_CORES-1:0] snoop_req;
    logic [N_CORES-1:0] snoop_type;
    logic [N_CORES-1:0] snoop_ack;
    logic [N_CORES-1:0][DATA_WIDTH-1:0] snoop_data;
    
    // Main state machine
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            pending_core <= 0;
            pending_addr <= 0;
            pending_wdata <= 0;
            pending_wstrb <= 0;
            pending_op <= 0;
            pending_we <= 0;
            
            for (int i = 0; i < 2**INDEX_WIDTH; i++) begin
                directory[i].valid <= 1'b0;
                directory[i].dirty <= 1'b0;
                directory[i].core_state <= '{default: I_INVALID};
            end
        end else begin
            state <= next_state;
            
            if (state == IDLE) begin
                for (int i = 0; i < N_CORES; i++) begin
                    pending_core[i] <= 1'b0;
                end
            end
            
            case (state)
                IDLE: begin
                    logic found;
                    found = 1'b0;
                    
                    for (int i = 0; i < N_CORES; i++) begin
                        if (!found && i_cache_req[i]) begin
                            pending_core[i] <= 1'b1;
                            pending_addr <= i_cache_addr[i];
                            pending_wdata <= i_cache_wdata[i];
                            pending_wstrb <= i_cache_wstrb[i];
                            pending_op <= i_cache_wvalid[i];
                            pending_we <= i_cache_wvalid[i];
                            found = 1'b1;
                        end
                    end
                end
                
                UPDATE_DIRECTORY: begin
                    if (pending_op) begin
                        directory[addr_index].core_state[pending_core] <= M_MODIFIED;
                        for (int i = 0; i < N_CORES; i++) begin
                            if (i != pending_core && 
                                directory[addr_index].core_state[i] != I_INVALID) begin
                                directory[addr_index].core_state[i] <= I_INVALID;
                            end
                        end
                        directory[addr_index].dirty <= 1'b1;
                    end else begin
                        if (directory[addr_index].core_state[pending_core] == I_INVALID) begin
                            directory[addr_index].core_state[pending_core] <= E_EXCLUSIVE;
                        end else begin
                            directory[addr_index].core_state[pending_core] <= S_SHARED;
                        end
                    end
                    directory[addr_index].tag <= addr_tag;
                    directory[addr_index].valid <= 1'b1;
                end
            endcase
        end
    end
    
    // Next state logic
    always_comb begin
        next_state = state;
        
        case (state)
            IDLE: begin
                if (|i_cache_req) begin
                    next_state = CHECK_DIRECTORY;
                end
            end
            
            CHECK_DIRECTORY: begin
                if (directory[addr_index].valid && 
                    directory[addr_index].tag == addr_tag) begin
                    if (pending_op && |directory[addr_index].core_state) begin
                        next_state = SNOOP_REQUEST;
                    end else begin
                        next_state = UPDATE_DIRECTORY;
                    end
                end else begin
                    next_state = L2_ACCESS;
                end
            end
            
            SNOOP_REQUEST: begin
                next_state = WAIT_SNOOP_RESPONSE;
            end
            
            WAIT_SNOOP_RESPONSE: begin
                if (&snoop_ack) begin
                    next_state = UPDATE_DIRECTORY;
                end
            end
            
            L2_ACCESS: begin
                if (i_l2_done) begin
                    next_state = UPDATE_DIRECTORY;
                end
            end
            
            UPDATE_DIRECTORY: begin
                next_state = SEND_RESPONSE;
            end
            
            SEND_RESPONSE: begin
                next_state = IDLE;
            end
        endcase
    end
    
    // Snoop logic
    generate
        for (genvar i = 0; i < N_CORES; i++) begin : snoop_gen
            assign snoop_req[i] = (state == SNOOP_REQUEST) && 
                                 (directory[addr_index].core_state[i] != I_INVALID);
            assign snoop_type[i] = pending_op;
        end
    endgenerate
    
    // Response logic
    always_ff @(posedge clk) begin
        if (state == SEND_RESPONSE) begin
            for (int i = 0; i < N_CORES; i++) begin
                if (pending_core[i]) begin
                    o_cache_rdone[i] <= 1'b1;
                    o_cache_wdone[i] <= pending_we;
                    if (!pending_we) begin
                        o_cache_rdata[i] <= i_l2_rdata;
                    end
                end
            end
        end else begin
            o_cache_rdone <= 0;
            o_cache_wdone <= 0;
        end
    end
    
    // L2 interface
    assign o_l2_req = (state == L2_ACCESS);
    assign o_l2_addr = pending_addr;
    assign o_l2_wdata = pending_wdata;
    assign o_l2_wstrb = pending_wstrb;
    assign o_l2_op = pending_op;
    
    // Output coherence states
    generate
        for (genvar i = 0; i < N_CORES; i++) begin : state_out
            assign o_coherence_state[i] = directory[addr_index].core_state[i];
        end
    endgenerate
    
endmodule

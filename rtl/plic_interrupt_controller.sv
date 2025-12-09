`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ASF
// Engineer: Ahmad Foqara
// 
// Create Date: 12/01/2025 05:12:50 PM
// Design Name: 
// Module Name: plic_interrupt_controller
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module plic_interrupt_controller #(
    parameter N_CORES = 4,
    parameter N_INTERRUPTS = 64
)(
    input  logic clk,
    input  logic rst_n,
    
    // Interrupt sources
    input  logic [N_INTERRUPTS-1:0] i_interrupt_sources,
    
    // Core interrupts
    output logic [N_CORES-1:0] o_core_interrupt_m,
    output logic [N_CORES-1:0] o_core_interrupt_s,
    
    // Configuration interface (memory-mapped)
    input  logic [31:0] i_config_addr,
    input  logic [31:0] i_config_wdata,
    input  logic i_config_we,
    output logic [31:0] o_config_rdata
);

    // ============================================
    // INTERNAL REGISTERS
    // ============================================
    
    // Priority registers (per interrupt, 3 bits each)
    logic [N_INTERRUPTS-1:0][2:0] interrupt_priority;
    
    // Pending bits
    logic [N_INTERRUPTS-1:0] interrupt_pending;
    
    // Enable bits (per core, per interrupt)
    logic [N_CORES-1:0][N_INTERRUPTS-1:0] interrupt_enable;
    
    // Threshold (per core)
    logic [N_CORES-1:0][2:0] threshold;
    
    // Claim/Complete registers
    logic [N_CORES-1:0][31:0] claim_complete;
    
    // Current highest priority interrupt for each core
    logic [N_CORES-1:0][$clog2(N_INTERRUPTS)-1:0] highest_irq;
    logic [N_CORES-1:0][2:0] highest_priority;
    logic [N_CORES-1:0] has_pending_irq;
    
    // Address decoding
    logic [3:0] word_index;
    logic [1:0] hart_index;
    logic is_priority;
    logic is_pending;
    logic is_enable;
    logic is_threshold;
    logic is_claim;
    
    // ============================================
    // ADDRESS DECODING
    // ============================================
    
    assign word_index = i_config_addr[3:0];
    assign hart_index = i_config_addr[11:10];  // Simple hart index for 4 cores
    
    assign is_priority = (i_config_addr[19:12] == 8'h00);      // 0x0000-0x0FFF
    assign is_pending = (i_config_addr[19:12] == 8'h10);       // 0x1000-0x1FFF
    assign is_enable = (i_config_addr[19:12] == 8'h20);        // 0x2000-0x2FFF
    assign is_threshold = (i_config_addr[19:8] == 12'h200);    // 0x200000-0x200FFF
    assign is_claim = (i_config_addr[19:8] == 12'h200 && i_config_addr[3:0] == 4'h4); // 0x200004
    
    // ============================================
    // UPDATE PENDING BITS
    // ============================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            interrupt_pending <= 0;
        end else begin
            // Set pending bits on rising edge of interrupt sources
            for (int i = 0; i < N_INTERRUPTS; i++) begin
                if (i_interrupt_sources[i] && !interrupt_pending[i]) begin
                    interrupt_pending[i] <= 1'b1;
                end
            end
            
            // Clear pending bits on claim
            for (int c = 0; c < N_CORES; c++) begin
                if (claim_complete[c] != 0 && claim_complete[c] < N_INTERRUPTS) begin
                    interrupt_pending[claim_complete[c]] <= 1'b0;
                end
            end
        end
    end
    
    // ============================================
    // FIND HIGHEST PRIORITY INTERRUPT FOR EACH CORE
    // ============================================
    
    generate
        for (genvar c = 0; c < N_CORES; c++) begin : core_priority
            always_comb begin
                highest_irq[c] = 0;
                highest_priority[c] = 0;
                has_pending_irq[c] = 1'b0;
                
                for (int i = 0; i < N_INTERRUPTS; i++) begin
                    if (interrupt_pending[i] && 
                        interrupt_enable[c][i] && 
                        interrupt_priority[i] > threshold[c] &&
                        interrupt_priority[i] > highest_priority[c]) begin
                        highest_irq[c] = i;
                        highest_priority[c] = interrupt_priority[i];
                        has_pending_irq[c] = 1'b1;
                    end
                end
            end
        end
    endgenerate
    
    // ============================================
    // GENERATE INTERRUPTS TO CORES
    // ============================================
    
    always_comb begin
        for (int c = 0; c < N_CORES; c++) begin
            o_core_interrupt_m[c] = has_pending_irq[c];
            o_core_interrupt_s[c] = 1'b0;  // Supervisor mode not used
        end
    end
    
    // ============================================
    // CONFIGURATION WRITE INTERFACE
    // ============================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Initialize with defaults
            interrupt_priority <= '{default: 0};
            interrupt_enable <= '{default: 0};
            threshold <= '{default: 0};
            claim_complete <= '{default: 0};
        end else if (i_config_we) begin
            // Priority registers (0x0000 - 0x0FFC)
            if (is_priority) begin
                // Each interrupt has a 32-bit priority register
                // We only use lower 3 bits
                for (int i = 0; i < N_INTERRUPTS; i++) begin
                    if (i_config_addr[11:2] == i[9:0]) begin  // Word addressing
                        interrupt_priority[i] <= i_config_wdata[2:0];
                    end
                end
            end
            
            // Enable bits (0x2000 - 0x3FFF)
            else if (is_enable) begin
                // Each hart has N_INTERRUPTS enable bits
                // We need to handle 32-bit words (32 interrupts per word)
                integer hart;
                integer interrupt_base;
                
                hart = i_config_addr[9:8];  // 0-3 for 4 cores
                interrupt_base = (i_config_addr[7:2] * 32);  // 32 interrupts per word
                
                if (hart < N_CORES) begin
                    for (int i = 0; i < 32; i++) begin
                        if (interrupt_base + i < N_INTERRUPTS) begin
                            interrupt_enable[hart][interrupt_base + i] <= i_config_wdata[i];
                        end
                    end
                end
            end
            
            // Threshold (0x200000 + hart*0x1000)
            else if (is_threshold && word_index == 4'h0) begin
                automatic integer hart = i_config_addr[11:10];
                if (hart < N_CORES) begin
                    threshold[hart] <= i_config_wdata[2:0];
                end
            end
            
            // Claim/Complete (0x200004 + hart*0x1000)
            else if (is_claim) begin
                automatic integer hart = i_config_addr[11:10];
                if (hart < N_CORES) begin
                    if (i_config_wdata == 0) begin
                        // Complete
                        claim_complete[hart] <= 0;
                    end else begin
                        // Claim
                        claim_complete[hart] <= highest_irq[hart];
                    end
                end
            end
        end
    end
    
    // ============================================
    // CONFIGURATION READ INTERFACE (FIXED)
    // ============================================
    
    always_comb begin
        o_config_rdata = 32'h0;
        
        // Priority registers
        if (is_priority) begin
            for (int i = 0; i < N_INTERRUPTS; i++) begin
                if (i_config_addr[11:2] == i[9:0]) begin
                    o_config_rdata[2:0] = interrupt_priority[i];
                end
            end
        end
        
        // Pending bits - FIXED: Use 32-bit output only
        else if (is_pending) begin
            integer pending_base;
            pending_base = (i_config_addr[11:2] * 32);  // 32 bits per word
            
            for (int i = 0; i < 32; i++) begin
                if (pending_base + i < N_INTERRUPTS) begin
                    o_config_rdata[i] = interrupt_pending[pending_base + i];
                end
            end
        end
        
        // Enable bits - FIXED: Use 32-bit output only
        else if (is_enable) begin
            integer hart;
            integer interrupt_base;
            
            hart = i_config_addr[9:8];
            interrupt_base = (i_config_addr[7:2] * 32);
            
            if (hart < N_CORES) begin
                for (int i = 0; i < 32; i++) begin
                    if (interrupt_base + i < N_INTERRUPTS) begin
                        o_config_rdata[i] = interrupt_enable[hart][interrupt_base + i];
                    end
                end
            end
        end
        
        // Threshold
        else if (is_threshold && word_index == 4'h0) begin
            automatic integer hart = i_config_addr[11:10];
            if (hart < N_CORES) begin
                o_config_rdata[2:0] = threshold[hart];
            end
        end
        
        // Claim/Complete
        else if (is_claim) begin
            automatic integer hart = i_config_addr[11:10];
            if (hart < N_CORES) begin
                o_config_rdata = claim_complete[hart];
            end
        end
    end
    
endmodule

`timescale 1ns / 1ps

module mailbox_system #(
    parameter N_CORES = 4,
    parameter MAILBOX_DEPTH = 8,
    parameter DATA_WIDTH = 64
)(
    input  logic i_clk,
    input  logic i_rst_n,
    
    // Write interface
    input  logic [N_CORES-1:0] i_wr_en,
    input  logic [N_CORES-1:0][$clog2(N_CORES)-1:0] i_wr_dest,
    input  logic [N_CORES-1:0][DATA_WIDTH-1:0] i_wr_data,
    output logic [N_CORES-1:0] o_wr_full,
    
    // Read interface
    input  logic [N_CORES-1:0] i_rd_en,
    output logic [N_CORES-1:0][DATA_WIDTH-1:0] o_rd_data,
    output logic [N_CORES-1:0] o_rd_empty,
    
    // Interrupt
    output logic [N_CORES-1:0] o_mailbox_irq
);

    // ============================================
    // MAILBOX STRUCTURE
    // ============================================
    
    // Memory for each mailbox
    logic [DATA_WIDTH-1:0] mailbox_mem [0:N_CORES-1][0:MAILBOX_DEPTH-1];
    logic [0:N_CORES-1][0:MAILBOX_DEPTH-1] mailbox_valid;
    
    // Pointers and counters
    logic [0:N_CORES-1][$clog2(MAILBOX_DEPTH)-1:0] wr_ptr;
    logic [0:N_CORES-1][$clog2(MAILBOX_DEPTH)-1:0] rd_ptr;
    logic [0:N_CORES-1][$clog2(MAILBOX_DEPTH):0] count;
    
    // Write arbitration signals
    logic [0:N_CORES-1] write_pending;
    logic [0:N_CORES-1][$clog2(N_CORES)-1:0] write_source;
    
    // ============================================
    // WRITE ARBITRATION (Fixed priority)
    // ============================================
    
    always_comb begin
        // Initialize all outputs
        write_pending = '0;
        write_source = '0;
        
        // Arbitration for each destination
        for (int dst = 0; dst < N_CORES; dst++) begin
            // Fixed priority arbitration
            if (i_wr_en[0] && (i_wr_dest[0] == dst) && !o_wr_full[dst]) begin
                write_pending[dst] = 1'b1;
                write_source[dst] = 0;
            end else if (i_wr_en[1] && (i_wr_dest[1] == dst) && !o_wr_full[dst]) begin
                write_pending[dst] = 1'b1;
                write_source[dst] = 1;
            end else if (i_wr_en[2] && (i_wr_dest[2] == dst) && !o_wr_full[dst]) begin
                write_pending[dst] = 1'b1;
                write_source[dst] = 2;
            end else if (i_wr_en[3] && (i_wr_dest[3] == dst) && !o_wr_full[dst]) begin
                write_pending[dst] = 1'b1;
                write_source[dst] = 3;
            end
        end
    end
    
    // ============================================
    // MAILBOX OPERATIONS (Sequential)
    // ============================================
    
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            // Reset everything
            for (int dst = 0; dst < N_CORES; dst++) begin
                for (int i = 0; i < MAILBOX_DEPTH; i++) begin
                    mailbox_valid[dst][i] <= 1'b0;
                    mailbox_mem[dst][i] <= 0;
                end
                wr_ptr[dst] <= 0;
                rd_ptr[dst] <= 0;
                count[dst] <= 0;
            end
        end else begin
            // Handle each mailbox
            for (int dst = 0; dst < N_CORES; dst++) begin
                // Write operation
                if (write_pending[dst] && (count[dst] < MAILBOX_DEPTH)) begin
                    mailbox_mem[dst][wr_ptr[dst]] <= i_wr_data[write_source[dst]];
                    mailbox_valid[dst][wr_ptr[dst]] <= 1'b1;
                    wr_ptr[dst] <= (wr_ptr[dst] + 1) % MAILBOX_DEPTH;
                    count[dst] <= count[dst] + 1;
                end
                
                // Read operation
                if (i_rd_en[dst] && (count[dst] > 0)) begin
                    mailbox_valid[dst][rd_ptr[dst]] <= 1'b0;
                    rd_ptr[dst] <= (rd_ptr[dst] + 1) % MAILBOX_DEPTH;
                    count[dst] <= count[dst] - 1;
                end
            end
        end
    end
    
    // ============================================
    // OUTPUT LOGIC (Combinational)
    // ============================================
    
    always_comb begin
        for (int dst = 0; dst < N_CORES; dst++) begin
            // Read from head of queue
            o_rd_data[dst] = mailbox_mem[dst][rd_ptr[dst]];
            
            // Status flags
            o_rd_empty[dst] = (count[dst] == 0);
            o_wr_full[dst] = (count[dst] == MAILBOX_DEPTH);
            
            // Interrupt
            o_mailbox_irq[dst] = (count[dst] > 0);
        end
    end
    
endmodule

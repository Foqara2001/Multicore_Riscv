`timescale 1ns / 1ps

module hw_semaphore #(
    parameter N_SEMAPHORES = 16,
    parameter N_CORES = 4
)(
    input  logic i_clk,
    input  logic i_rst_n,
    
    // Request interface (per-core)
    input  logic [N_CORES-1:0] i_sem_req,
    input  logic [N_CORES-1:0][$clog2(N_SEMAPHORES)-1:0] i_sem_id,
    input  logic [N_CORES-1:0] i_sem_op,  // 0: acquire, 1: release
    output logic [N_CORES-1:0] o_sem_grant,
    output logic [N_CORES-1:0] o_sem_wait
);

    // Semaphore structure
    typedef struct packed {
        logic [$clog2(N_CORES)-1:0] owner;  // Which core owns it (if locked)
        logic locked;                       // 1 if semaphore is locked
        logic [N_CORES-1:0] waiters;        // Which cores are waiting
        logic [15:0] count;                 // Counting semaphore value
    } semaphore_t;
    
    semaphore_t semaphores [N_SEMAPHORES];
    
    // Request handling FSM
    typedef enum logic [1:0] {
        IDLE,
        PROCESS,
        UPDATE
    } sem_state_t;
    
    sem_state_t state;
    
    // Registered request info
    logic [$clog2(N_CORES)-1:0] req_core;
    logic [$clog2(N_SEMAPHORES)-1:0] req_sem_id;
    logic req_op;
    logic req_valid;
    
    // Round-robin for multiple concurrent requests
    logic [$clog2(N_CORES)-1:0] arb_pointer;
	
	logic found ; 
	// Find next pending request 
	always_comb begin
		req_valid = 1'b0;
		req_core = 0;
	    found = 1'b0;  // Flag to track if we've found a request
		
		for (int i = 0; i < N_CORES; i++) begin
			automatic int idx = (arb_pointer + i) % N_CORES;
			if (!found && i_sem_req[idx]) begin
				req_core = idx;
				req_valid = 1'b1;
				found = 1'b1;  // Set flag to prevent further updates
			end
		end
	end
    
    // Main FSM
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state <= IDLE;
            o_sem_grant <= 0;
            o_sem_wait <= 0;
            arb_pointer <= 0;
            
            // Initialize semaphores
            for (int i = 0; i < N_SEMAPHORES; i++) begin
                semaphores[i].owner <= 0;
                semaphores[i].locked <= 1'b0;
                semaphores[i].waiters <= 0;
                semaphores[i].count <= 16'h0001; // Initial value = 1 (available)
            end
        end else begin
            // Default outputs
            o_sem_grant <= 0;
            o_sem_wait <= 0;
            
            case (state)
                IDLE: begin
                    if (req_valid) begin
                        state <= PROCESS;
                        req_sem_id <= i_sem_id[req_core];
                        req_op <= i_sem_op[req_core];
                    end
                end
                
                PROCESS: begin
                    if (req_op == 0) begin // ACQUIRE
                        if (semaphores[req_sem_id].count > 0) begin
                            // Semaphore available
                            semaphores[req_sem_id].count <= semaphores[req_sem_id].count - 1;
                            o_sem_grant[req_core] <= 1'b1;
                            
                            // Mark as locked if count becomes 0
                            if (semaphores[req_sem_id].count == 1) begin
                                semaphores[req_sem_id].locked <= 1'b1;
                                semaphores[req_sem_id].owner <= req_core;
                            end
                        end else begin
                            // Semaphore not available, add to waiters
                            semaphores[req_sem_id].waiters[req_core] <= 1'b1;
                            o_sem_wait[req_core] <= 1'b1;
                        end
                    end else begin // RELEASE
                        if (semaphores[req_sem_id].locked && 
                            semaphores[req_sem_id].owner == req_core) begin
                            // Release semaphore
                            semaphores[req_sem_id].count <= semaphores[req_sem_id].count + 1;
                            semaphores[req_sem_id].locked <= 1'b0;
                            o_sem_grant[req_core] <= 1'b1;
                            
                            // Check if any waiters
                            if (|semaphores[req_sem_id].waiters) begin
                                // Find highest priority waiter
                                logic found;
                                found = 0;
                                for (int i = 0; i < N_CORES; i++) begin
                                    if (!found && semaphores[req_sem_id].waiters[i]) begin
                                        // Grant to this waiter
                                        semaphores[req_sem_id].waiters[i] <= 1'b0;
                                        semaphores[req_sem_id].count <= semaphores[req_sem_id].count - 1;
                                        semaphores[req_sem_id].locked <= 1'b1;
                                        semaphores[req_sem_id].owner <= i;
                                        found = 1;
                                    end
                                end
                            end
                        end else begin
                            // Invalid release attempt
                            o_sem_wait[req_core] <= 1'b1;
                        end
                    end
                    
                    state <= UPDATE;
                end
                
                UPDATE: begin
                    // Update arbitration pointer
                    arb_pointer <= (req_core + 1) % N_CORES;
                    state <= IDLE;
                end
                
                default: begin
                    state <= IDLE;
                end
            endcase
        end
    end
    
   
    
endmodule

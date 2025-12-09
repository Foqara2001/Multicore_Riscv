`timescale 1ns / 1ps

module multicore_system #(parameter N_CORES = 4) (
	// Global inputs
	input  logic               i_sys_clk,
	input  logic               i_sys_rst_n,
	
	// External interrupts
	input  logic [N_CORES-1:0] i_external_interrupt_m,
	input  logic [N_CORES-1:0] i_external_interrupt_s,
	output logic [N_CORES-1:0] o_core_ack,
	
	// Memory interface (shared) - FIXED: 32-bit addresses
	output logic [31:0]        o_mem_read_address,
	output logic [63:0]        o_mem_write_data,
	output logic [31:0]        o_mem_write_address,
	output logic               o_mem_read_req,
	output logic               o_mem_write_valid,
	input  logic               i_mem_read_done,
	input  logic               i_mem_write_done,
	input  logic [255:0]       i_block_from_axi,
	output logic [7:0]         o_mem_write_strobe,
	
	// Instruction fetch interface
	output logic [31:0]        o_inst_addr,
	output logic               o_inst_req,
	input  logic               i_inst_done,
	input  logic [255:0]       i_inst_block,
	
	// UART interfaces
	input  logic               uart_ready,
	output logic [7:0]         uart_out_data [0:N_CORES-1],
	output logic               uart_valid    [0:N_CORES-1],
	
	// DFT/Scan
	input  logic               SI,
	input  logic               SE,
	input  logic               scan_clk,
	input  logic               scan_rst,
	input  logic               test_mode,
	output logic [N_CORES-1:0] SO
);

	// ============================================
	// SIGNAL DECLARATIONS
	// ============================================
	
	// Core data cache interfaces - FIXED: 32-bit addresses
	logic [N_CORES-1:0][31:0] core_dcache_raddr;
	logic [N_CORES-1:0][63:0] core_dcache_wdata;
	logic [N_CORES-1:0][31:0] core_dcache_waddr;
	logic [N_CORES-1:0] core_dcache_rreq;
	logic [N_CORES-1:0] core_dcache_wvalid;
	logic [N_CORES-1:0] core_dcache_rdone;
	logic [N_CORES-1:0] core_dcache_wdone;
	logic [N_CORES-1:0][63:0] core_dcache_rdata;
	logic [N_CORES-1:0][7:0] core_dcache_wstrb;
	
	// Core instruction cache interfaces
	logic [N_CORES-1:0][31:0] core_icache_addr;
	logic [N_CORES-1:0] core_icache_req;
	logic [N_CORES-1:0] core_icache_done;
	logic [N_CORES-1:0][255:0] core_icache_rdata;
	
	// Coherence controller interface
	logic l2_req;
	logic [31:0] l2_addr;
	logic [255:0] l2_wdata;
	logic [7:0] l2_wstrb;
	logic l2_op;
	logic [255:0] l2_rdata;
	logic l2_done;
	
	// PLIC interface signals
	logic [63:0] interrupt_sources;
	logic [N_CORES-1:0] plic_interrupt_m;
	logic [N_CORES-1:0] plic_interrupt_s;
	logic [N_CORES-1:0][19:0] plic_config_addr;
	logic [N_CORES-1:0][31:0] plic_config_wdata;
	logic [N_CORES-1:0] plic_config_we;
	logic [N_CORES-1:0][31:0] plic_config_rdata;
	
	// Mailbox system interface
	logic [N_CORES-1:0] core_mailbox_wren;
	logic [N_CORES-1:0][$clog2(N_CORES)-1:0] core_mailbox_wrdest;
	logic [N_CORES-1:0][63:0] core_mailbox_wrdata;
	logic [N_CORES-1:0] core_mailbox_wrfull;
	logic [N_CORES-1:0] core_mailbox_rden;
	logic [N_CORES-1:0][63:0] core_mailbox_rddata;
	logic [N_CORES-1:0] core_mailbox_rdempty;
	logic [N_CORES-1:0] core_mailbox_irq;
	
	// Hardware semaphore interface
	logic [N_CORES-1:0] core_sem_req;
	logic [N_CORES-1:0][3:0] core_sem_id;
	logic [N_CORES-1:0] core_sem_op;
	logic [N_CORES-1:0] core_sem_grant;
	logic [N_CORES-1:0] core_sem_wait;
	
	// Coherence states
	logic [N_CORES-1:0][1:0] coherence_state;
	
	// Boot ROM interface
	logic [N_CORES-1:0][11:0] boot_addr;
	logic [N_CORES-1:0] boot_req;
	logic [N_CORES-1:0][31:0] boot_data;
	logic [N_CORES-1:0] boot_valid;
	
	// Core ID and boot address
	logic [N_CORES-1:0][31:0] i_boot_address;  // Changed to 32-bit
	
	// Hart ID from cores
	logic [N_CORES-1:0][63:0] core_hartid;
	
	// Atomic operation interfaces
	logic [N_CORES-1:0] core_atomic_req;
	logic [N_CORES-1:0][31:0] core_atomic_addr;  // Changed to 32-bit
	logic [N_CORES-1:0][63:0] core_atomic_data;
	logic [N_CORES-1:0][3:0] core_atomic_op;
	logic [N_CORES-1:0] core_atomic_ack;
	logic [N_CORES-1:0][63:0] core_atomic_rdata;
	
	// Boot ROM parameters - FIXED: 32-bit addresses
	localparam logic [31:0] BOOT_ROM_BASE = 32'h8000_0000;
	localparam logic [31:0] BOOT_ROM_SIZE = 32'h0000_0800;
	
	// Core reset synchronization
	logic [N_CORES-1:0] core_rst_n;
	logic [N_CORES-1:0] core_enable;
	
	// Coherence controller read data conversion
	logic [N_CORES-1:0][255:0] coherence_rdata;
	
	// UART internal signals
	logic [N_CORES-1:0][7:0] uart_out_data_int;
	logic [N_CORES-1:0] uart_valid_int;
	
	// ============================================
	// INITIALIZATION FSM
	// ============================================
	
	typedef enum logic [2:0] {
		SYS_RESET,
		CORE0_BOOT,
		CORE1_BOOT,
		CORE2_BOOT,
		CORE3_BOOT,
		ALL_RUNNING
	} boot_state_t;
	
	boot_state_t boot_state;
	logic [31:0] boot_counter;
	
	always_ff @(posedge i_sys_clk or negedge i_sys_rst_n) begin
		if (!i_sys_rst_n) begin
			boot_state <= SYS_RESET;
			boot_counter <= 0;
			core_enable <= 4'b0000;
		end else begin
			case (boot_state)
				SYS_RESET: begin
					boot_counter <= boot_counter + 1;
					if (boot_counter == 100) begin
						boot_state <= CORE0_BOOT;
						boot_counter <= 0;
					end
				end
				
				CORE0_BOOT: begin
					core_enable[0] <= 1'b1;
					boot_counter <= boot_counter + 1;
					if (boot_counter == 10) begin
						boot_state <= CORE1_BOOT;
						boot_counter <= 0;
					end
				end
				
				CORE1_BOOT: begin
					core_enable[1] <= 1'b1;
					boot_counter <= boot_counter + 1;
					if (boot_counter == 10) begin
						boot_state <= CORE2_BOOT;
						boot_counter <= 0;
					end
				end
				
				CORE2_BOOT: begin
					core_enable[2] <= 1'b1;
					boot_counter <= boot_counter + 1;
					if (boot_counter == 10) begin
						boot_state <= CORE3_BOOT;
						boot_counter <= 0;
					end
				end
				
				CORE3_BOOT: begin
					core_enable[3] <= 1'b1;
					boot_counter <= boot_counter + 1;
					if (boot_counter == 10) begin
						boot_state <= ALL_RUNNING;
					end
				end
				
				ALL_RUNNING: begin
					// All cores running
				end
				
				default: begin
					boot_state <= SYS_RESET;
				end
			endcase
		end
	end
	
	// Initialize boot addresses
	generate
		for (genvar i = 0; i < N_CORES; i++) begin : boot_addr_gen
			assign i_boot_address[i] = BOOT_ROM_BASE;
		end
	endgenerate
	
	// ============================================
	// CORE INSTANTIATION
	// ============================================
	
	generate
		for (genvar i = 0; i < N_CORES; i++) begin : core_gen
			assign core_rst_n[i] = i_sys_rst_n && core_enable[i];
			
			core_top u_core (
				// Global inputs
				.i_core_clk                 (i_sys_clk),
				.i_core_rst_n               (core_rst_n[i]),
				.i_core_external_interrupt_m(i_external_interrupt_m[i]),
				.i_core_external_interrupt_s(i_external_interrupt_s[i]),
				.o_core_ack                 (o_core_ack[i]),
				
				// Data cache interface - FIXED to match
				.mem_read_address           (core_dcache_raddr[i]),
				.o_mem_write_data           (core_dcache_wdata[i]),
				.o_mem_write_address        (core_dcache_waddr[i]),
				.mem_read_req               (core_dcache_rreq[i]),
				.o_mem_write_valid          (core_dcache_wvalid[i]),
				.mem_read_done              (core_dcache_rdone[i]),
				.i_mem_write_done           (core_dcache_wdone[i]),
				.i_block_from_axi_data_cache(core_dcache_rdata[i]),  // 64-bit per access
				.o_mem_write_strobe         (core_dcache_wstrb[i]),
				
				// Instruction cache interface
				.o_addr_from_control_to_axi (core_icache_addr[i]),
				.o_mem_req                  (core_icache_req[i]),
				.i_mem_done                 (core_icache_done[i]),
				.i_block_from_axi_i_cache   (core_icache_rdata[i]),
				
				// DFT
				.SI                         (SI),
				.SE                         (SE),
				.scan_clk                   (scan_clk),
				.scan_rst                   (scan_rst),
				.test_mode                  (test_mode),
				.SO                         (SO[i]),
				
				// UART
				.uart_ready                 (uart_ready),
				.uart_out_data              (uart_out_data_int[i]),
				.uart_valid                 (uart_valid_int[i]),
				
				// Core ID and boot address - FIXED: 32-bit
				.i_core_id                  (32'(i)),
				.i_boot_address             (i_boot_address[i]),
				.o_core_hartid              (core_hartid[i]),
				
				// Mailbox interface
				.o_mailbox_wr_en            (core_mailbox_wren[i]),
				.o_mailbox_wr_dest          (core_mailbox_wrdest[i]),
				.o_mailbox_wr_data          (core_mailbox_wrdata[i]),
				.i_mailbox_wr_full          (core_mailbox_wrfull[i]),
				.o_mailbox_rd_en            (core_mailbox_rden[i]),
				.i_mailbox_rd_data          (core_mailbox_rddata[i]),
				.i_mailbox_rd_empty         (core_mailbox_rdempty[i]),
				.i_mailbox_irq              (core_mailbox_irq[i]),
				
				// Semaphore interface
				.o_sem_req                  (core_sem_req[i]),
				.o_sem_id                   (core_sem_id[i]),
				.o_sem_op                   (core_sem_op[i]),
				.i_sem_grant                (core_sem_grant[i]),
				.i_sem_wait                 (core_sem_wait[i]),
				
				// PLIC interface
				.i_plic_interrupt_m         (plic_interrupt_m[i]),
				.i_plic_interrupt_s         (plic_interrupt_s[i]),
				.o_plic_config_addr         (plic_config_addr[i]),
				.o_plic_config_wdata        (plic_config_wdata[i]),
				.o_plic_config_we           (plic_config_we[i]),
				.i_plic_config_rdata        (plic_config_rdata[i]),
				
				// Atomic operations
				.o_atomic_req               (core_atomic_req[i]),
				.o_atomic_addr              (core_atomic_addr[i]),
				.o_atomic_data              (core_atomic_data[i]),
				.o_atomic_op                (core_atomic_op[i]),
				.i_atomic_ack               (core_atomic_ack[i]),
				.i_atomic_rdata             (core_atomic_rdata[i])
			);
			
			// Instantiate boot ROM for each core
			boot_rom #(.CORE_ID(i)) u_boot_rom (
				.i_clk  (i_sys_clk),
				.i_rst_n(i_sys_rst_n),
				.i_req  (boot_req[i]),
				.i_addr (boot_addr[i]),
				.o_data (boot_data[i]),
				.o_valid(boot_valid[i])
			);
		end
	endgenerate
	
	// Assign UART outputs
	generate
		for (genvar i = 0; i < N_CORES; i++) begin : uart_output_assign
			assign uart_out_data[i] = uart_out_data_int[i];
			assign uart_valid[i] = uart_valid_int[i];
		end
	endgenerate
	
	// ============================================
	// COHERENCE CONTROLLER
	// ============================================
	
	mesi_coherence_controller #(
		.N_CORES(N_CORES),
		.ADDR_WIDTH(32),
		.DATA_WIDTH(256)
	) u_coherence (
		.clk              (i_sys_clk),
		.rst_n            (i_sys_rst_n),
		
		// Core interfaces - FIXED: Pass 32-bit addresses directly
		.i_cache_req      (core_dcache_rreq),
		.i_cache_addr     (core_dcache_raddr),      // 32-bit addresses
		.i_cache_wdata    (core_dcache_wdata),      // 64-bit write data
		.i_cache_waddr    (core_dcache_waddr),      // 32-bit write addresses
		.i_cache_wvalid   (core_dcache_wvalid),
		.i_cache_wstrb    (core_dcache_wstrb),
		.o_cache_rdata    (coherence_rdata),        // 256-bit read data
		.o_cache_rdone    (core_dcache_rdone),
		.o_cache_wdone    (core_dcache_wdone),
		
		// L2 cache interface
		.o_l2_req         (l2_req),
		.o_l2_addr        (l2_addr),
		.o_l2_wdata       (l2_wdata),
		.o_l2_wstrb       (l2_wstrb),
		.o_l2_op          (l2_op),
		.i_l2_rdata       (l2_rdata),
		.i_l2_done        (l2_done),
		
		// Coherence states
		.o_coherence_state(coherence_state)
	);
	
	// Data width conversion: 256-bit block to 64-bit per core
	generate
		for (genvar i = 0; i < N_CORES; i++) begin : data_conversion
			always_comb begin
				case (core_dcache_raddr[i][4:3])
					2'b00: core_dcache_rdata[i] = coherence_rdata[i][63:0];
					2'b01: core_dcache_rdata[i] = coherence_rdata[i][127:64];
					2'b10: core_dcache_rdata[i] = coherence_rdata[i][191:128];
					2'b11: core_dcache_rdata[i] = coherence_rdata[i][255:192];
					default: core_dcache_rdata[i] = 64'b0;
				endcase
			end
		end
	endgenerate
	
	// ============================================
	// SHARED L2 CACHE
	// ============================================
	
	shared_l2_cache #(
		.N_CORES(N_CORES),
		.ADDR_WIDTH(32),
		.DATA_WIDTH(256),
		.CACHE_SIZE(64*1024)
	) u_l2_cache (
		.i_clk       (i_sys_clk),
		.i_rst_n     (i_sys_rst_n),
		
		// Coherence controller interface
		.i_req       (l2_req),
		.i_addr      (l2_addr),
		.i_wdata     (l2_wdata),
		.i_wstrb     (l2_wstrb),
		.i_op        (l2_op),
		.o_rdata     (l2_rdata),
		.o_done      (l2_done),
		
		// Main memory interface - FIXED to match top-level
		.o_mem_addr  (o_mem_read_address),
		.o_mem_wdata (o_mem_write_data),
		.o_mem_waddr (o_mem_write_address),
		.o_mem_rreq  (o_mem_read_req),
		.o_mem_wvalid(o_mem_write_valid),
		.i_mem_rdone (i_mem_read_done),
		.i_mem_wdone (i_mem_write_done),
		.i_mem_rdata (i_block_from_axi),
		.o_mem_wstrb (o_mem_write_strobe)
	);
	
	// ============================================
	// PLIC INTERRUPT CONTROLLER
	// ============================================
	
	logic [19:0] plic_config_addr_shared;
	logic [31:0] plic_config_wdata_shared;
	logic plic_config_we_shared;
	logic [31:0] plic_config_rdata_shared;
	
	assign plic_config_addr_shared = plic_config_addr[0];
	assign plic_config_wdata_shared = plic_config_wdata[0];
	assign plic_config_we_shared = plic_config_we[0];
	
	generate
		for (genvar i = 0; i < N_CORES; i++) begin : plic_rdata_gen
			assign plic_config_rdata[i] = plic_config_rdata_shared;
		end
	endgenerate
	
	assign interrupt_sources[63:10] = 54'b0;
	assign interrupt_sources[9:6] = {uart_valid_int[3], uart_valid_int[2], uart_valid_int[1], uart_valid_int[0]};
	assign interrupt_sources[5:2] = {core_mailbox_irq[3], core_mailbox_irq[2], core_mailbox_irq[1], core_mailbox_irq[0]};
	assign interrupt_sources[1:0] = 2'b0;
	
	plic_interrupt_controller #(.N_CORES(N_CORES), .N_INTERRUPTS(64)) u_plic (
		.clk                (i_sys_clk),
		.rst_n              (i_sys_rst_n),
		
		// Interrupt sources
		.i_interrupt_sources(interrupt_sources),
		
		// Core interrupts
		.o_core_interrupt_m (plic_interrupt_m),
		.o_core_interrupt_s (plic_interrupt_s),
		
		// Configuration interface
		.i_config_addr      (plic_config_addr_shared),
		.i_config_wdata     (plic_config_wdata_shared),
		.i_config_we        (plic_config_we_shared),
		.o_config_rdata     (plic_config_rdata_shared)
	);
	
	// ============================================
	// INSTRUCTION CACHE ARBITER
	// ============================================
	
	icache_arbiter #(
		.N_CORES(N_CORES),
		.ADDR_WIDTH(32),
		.BLOCK_WIDTH(256)
	) u_icache_arbiter (
		.i_clk       (i_sys_clk),
		.i_rst_n     (i_sys_rst_n),
		
		// Core interfaces
		.i_core_addr (core_icache_addr),
		.i_core_req  (core_icache_req),
		.o_core_rdata(core_icache_rdata),
		.o_core_done (core_icache_done),
		
		// Shared L2 interface
		.o_l2_addr   (o_inst_addr),
		.o_l2_req    (o_inst_req),
		.i_l2_rdata  (i_inst_block),
		.i_l2_done   (i_inst_done),
		
		// Boot ROM interface
		.i_boot_base (BOOT_ROM_BASE),
		.i_boot_size (BOOT_ROM_SIZE),
		.o_boot_addr (boot_addr),
		.o_boot_req  (boot_req),
		.i_boot_data (boot_data),
		.i_boot_valid(boot_valid)
	);
	
	// ============================================
	// MAILBOX SYSTEM
	// ============================================
	
	mailbox_system #(
		.N_CORES(N_CORES),
		.MAILBOX_DEPTH(8),
		.DATA_WIDTH(64)
	) u_mailbox (
		.i_clk        (i_sys_clk),
		.i_rst_n      (i_sys_rst_n),
		
		// Write interface
		.i_wr_en      (core_mailbox_wren),
		.i_wr_dest    (core_mailbox_wrdest),
		.i_wr_data    (core_mailbox_wrdata),
		.o_wr_full    (core_mailbox_wrfull),
		
		// Read interface
		.i_rd_en      (core_mailbox_rden),
		.o_rd_data    (core_mailbox_rddata),
		.o_rd_empty   (core_mailbox_rdempty),
		
		// Interrupt
		.o_mailbox_irq(core_mailbox_irq)
	);
	
	// ============================================
	// HARDWARE SEMAPHORE
	// ============================================
	
	hw_semaphore #(
		.N_SEMAPHORES(16),
		.N_CORES(N_CORES)
	) u_semaphore (
		.i_clk      (i_sys_clk),
		.i_rst_n    (i_sys_rst_n),
		
		// Request interface
		.i_sem_req  (core_sem_req),
		.i_sem_id   (core_sem_id),
		.i_sem_op   (core_sem_op),
		.o_sem_grant(core_sem_grant),
		.o_sem_wait (core_sem_wait)
	);
	
	// ============================================
	// ATOMIC OPERATIONS HANDLING
	// ============================================
	
	generate
		for (genvar i = 0; i < N_CORES; i++) begin : atomic_tieoff
			assign core_atomic_ack[i] = 1'b0;
			assign core_atomic_rdata[i] = 64'b0;
		end
	endgenerate

endmodule

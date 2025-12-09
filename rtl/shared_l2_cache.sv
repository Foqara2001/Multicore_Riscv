`timescale 1ns / 1ps

module shared_l2_cache #(
	parameter N_CORES = 4,
	parameter ADDR_WIDTH = 32,  // Changed from 64 to match multicore_system
	parameter DATA_WIDTH = 256,
	parameter CACHE_SIZE = 64*1024,
	parameter NUM_WAYS = 4
)(
	input  logic i_clk,
	input  logic i_rst_n,
	
	// Coherence controller interface - FIXED to match mesi_coherence_controller
	input  logic i_req,
	input  logic [ADDR_WIDTH-1:0] i_addr,
	input  logic [DATA_WIDTH-1:0] i_wdata,
	input  logic [7:0] i_wstrb,
	input  logic i_op,  // 0: read, 1: write
	output logic [DATA_WIDTH-1:0] o_rdata,
	output logic o_done,
	
	// Main memory interface - FIXED to match multicore_system
	output logic [ADDR_WIDTH-1:0] o_mem_addr,
	output logic [63:0] o_mem_wdata,
	output logic [ADDR_WIDTH-1:0] o_mem_waddr,
	output logic o_mem_rreq,
	output logic o_mem_wvalid,
	input  logic i_mem_rdone,
	input  logic i_mem_wdone,
	input  logic [DATA_WIDTH-1:0] i_mem_rdata,
	output logic [7:0] o_mem_wstrb
);

	// ============================================
	// CACHE PARAMETERS
	// ============================================
	
	// SRAM Spec: SRAM2RW16x4 = 16 entries × 4 bits, dual port
	localparam SRAM_DEPTH = 16;
	localparam SRAM_WIDTH = 4;
	localparam SRAM_TOTAL_WIDTH = 8;
	
	// Cache organization
	localparam BLOCK_SIZE_BYTES = DATA_WIDTH / 8;
	localparam BLOCK_OFFSET_BITS = $clog2(BLOCK_SIZE_BYTES);
	localparam BLOCK_BITS = DATA_WIDTH;
	localparam SRAMS_PER_BLOCK = BLOCK_BITS / SRAM_TOTAL_WIDTH;
	
	// Cache size calculations
	localparam TOTAL_BLOCKS = CACHE_SIZE / BLOCK_SIZE_BYTES;
	localparam BLOCKS_PER_WAY = TOTAL_BLOCKS / NUM_WAYS;
	localparam SET_INDEX_BITS = $clog2(BLOCKS_PER_WAY);
	
	// Since SRAM depth is only 16, we need multiple banks
	localparam BANKS_PER_WAY = BLOCKS_PER_WAY / SRAM_DEPTH;
	localparam BANK_SEL_BITS = $clog2(BANKS_PER_WAY);
	
	// Tag width (32-bit addr - 9-bit index - 5-bit offset) = 18 bits
	localparam TAG_WIDTH = ADDR_WIDTH - SET_INDEX_BITS - BLOCK_OFFSET_BITS;
	
	// Tag needs to be split into 8-bit chunks
	localparam TAG_SRAMS = (TAG_WIDTH + SRAM_TOTAL_WIDTH - 1) / SRAM_TOTAL_WIDTH;
	
	// ============================================
	// SRAM DECLARATIONS
	// ============================================
	
	// Power management
	logic memory_sleep;
	logic [TAG_SRAMS+SRAMS_PER_BLOCK-1:0] power_ack;
	
	// Tag SRAMs
	logic [3:0] tag_addr [0:NUM_WAYS-1];
	logic [SRAM_TOTAL_WIDTH-1:0] tag_wdata [0:NUM_WAYS-1][0:TAG_SRAMS-1];
	logic [SRAM_TOTAL_WIDTH-1:0] tag_rdata [0:NUM_WAYS-1][0:TAG_SRAMS-1];
	logic tag_wen [0:NUM_WAYS-1];
	
	// Data SRAMs
	logic [3:0] data_addr [0:NUM_WAYS-1];
	logic [SRAM_TOTAL_WIDTH-1:0] data_wdata [0:NUM_WAYS-1][0:SRAMS_PER_BLOCK-1];
	logic [SRAM_TOTAL_WIDTH-1:0] data_rdata [0:NUM_WAYS-1][0:SRAMS_PER_BLOCK-1];
	logic data_wen [0:NUM_WAYS-1];
	
	// Control signals
	logic tag_cen [0:NUM_WAYS-1];
	logic data_cen [0:NUM_WAYS-1];
	
	// ============================================
	// SRAM INSTANTIATIONS
	// ============================================
	
	assign memory_sleep = 1'b0;
	
	generate
		for (genvar way = 0; way < NUM_WAYS; way++) begin : way_gen
			// Tag SRAMs
			for (genvar tag_sram = 0; tag_sram < TAG_SRAMS; tag_sram++) begin : tag_sram_gen
				// @SuppressProblem -type unresolved_hierarchy -count 1 -length 
				SRAM2RW16x4 u_tag_sram (
					.SLEEPIN (memory_sleep),
					.ACK     (power_ack[tag_sram]),
					.A1      (tag_addr[way]),
					.A2      (tag_addr[way]),
					.CE1     (tag_cen[way]),
					.CE2     (tag_cen[way]),
					.WEB1    (!tag_wen[way]),
					.WEB2    (!tag_wen[way]),
					.OEB1    (1'b0),
					.OEB2    (1'b0),
					.CSB1    (1'b0),
					.CSB2    (1'b0),
					.I1      (tag_wdata[way][tag_sram][3:0]),
					.I2      (tag_wdata[way][tag_sram][7:4]),
					.O1      (tag_rdata[way][tag_sram][3:0]),
					.O2      (tag_rdata[way][tag_sram][7:4])
				);
			end
			
			// Data SRAMs
			for (genvar data_sram = 0; data_sram < SRAMS_PER_BLOCK; data_sram++) begin : data_sram_gen
				// @SuppressProblem -type unresolved_hierarchy -count 1 -length 1
				SRAM2RW16x4 u_data_sram (
					.SLEEPIN (memory_sleep),
					.ACK     (power_ack[TAG_SRAMS + data_sram]),
					.A1      (data_addr[way]),
					.A2      (data_addr[way]),
					.CE1     (data_cen[way]),
					.CE2     (data_cen[way]),
					.WEB1    (!data_wen[way]),
					.WEB2    (!data_wen[way]),
					.OEB1    (1'b0),
					.OEB2    (1'b0),
					.CSB1    (1'b0),
					.CSB2    (1'b0),
					.I1      (data_wdata[way][data_sram][3:0]),
					.I2      (data_wdata[way][data_sram][7:4]),
					.O1      (data_rdata[way][data_sram][3:0]),
					.O2      (data_rdata[way][data_sram][7:4])
				);
			end
		end
	endgenerate
	
	// ============================================
	// ADDRESS DECOMPOSITION
	// ============================================
	
	logic [TAG_WIDTH-1:0] req_tag;
	logic [SET_INDEX_BITS-1:0] req_set_index;
	logic [3:0] req_intra_bank_addr;
	
	always_comb begin
		req_tag = i_addr[ADDR_WIDTH-1:SET_INDEX_BITS+BLOCK_OFFSET_BITS];
		req_set_index = i_addr[SET_INDEX_BITS+BLOCK_OFFSET_BITS-1:BLOCK_OFFSET_BITS];
		req_intra_bank_addr = req_set_index[3:0];
	end
	
	// ============================================
	// TAG MANAGEMENT
	// ============================================
	
	logic [TAG_WIDTH-1:0] current_tag [0:NUM_WAYS-1];
	logic [NUM_WAYS-1:0] tag_valid;
	logic [NUM_WAYS-1:0] tag_dirty;
	
	generate
		for (genvar way = 0; way < NUM_WAYS; way++) begin : tag_reconstruct
			always_comb begin
				current_tag[way] = 0;
				for (int j = 0; j < TAG_SRAMS; j++) begin
					if (j < TAG_SRAMS-1) begin
						current_tag[way][(j*8)+:8] = tag_rdata[way][j];
					end else begin
						// Last SRAM
						logic [TAG_WIDTH*8-1:0] shifted_tag;

						shifted_tag = tag_rdata[way][j] >> (j*8);

						current_tag[way][j] = shifted_tag[TAG_WIDTH*8-1 -: 8];

					end
				end
			end
		end
	endgenerate
	
	logic hit;
	logic [NUM_WAYS-1:0] way_hit;
	logic [$clog2(NUM_WAYS)-1:0] hit_way;
	logic [$clog2(NUM_WAYS)-1:0] replace_way;
	
	always_comb begin
		way_hit = 0;
		hit = 1'b0;
		hit_way = 0;
		
		for (int i = 0; i < NUM_WAYS; i++) begin
			if (tag_valid[i] && (current_tag[i] == req_tag)) begin
				way_hit[i] = 1'b1;
				hit = 1'b1;
				hit_way = i[$clog2(NUM_WAYS)-1:0];
			end
		end
	end
	
	// ============================================
	// DATA MANAGEMENT
	// ============================================
	
	logic [DATA_WIDTH-1:0] current_data [0:NUM_WAYS-1];
	
	generate
		for (genvar way = 0; way < NUM_WAYS; way++) begin : data_reconstruct
			always_comb begin
				current_data[way] = 0;
				for (int j = 0; j < SRAMS_PER_BLOCK; j++) begin
					current_data[way][(j*8)+:8] = data_rdata[way][j];
				end
			end
		end
	endgenerate
	
	assign o_rdata = hit ? current_data[hit_way] : '0;
	
	// ============================================
	// REPLACEMENT POLICY
	// ============================================
	
	logic [31:0] lfsr;
	logic [$clog2(NUM_WAYS)-1:0] random_way;
	
	always_ff @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			lfsr <= 32'hABCD1234;
		end else if (i_req && !hit) begin
			lfsr <= {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};
		end
	end
	
	assign random_way = lfsr[1:0];
	
	always_comb begin
		replace_way = random_way;

		for (int i = 0; i < NUM_WAYS; i++) begin
			if (!tag_valid[i] && replace_way == random_way) begin
				replace_way = i[$clog2(NUM_WAYS)-1:0];
			end
		end
	end
	
	// ============================================
	// STATE MACHINE
	// ============================================
	
	typedef enum logic [2:0] {
		STATE_IDLE,
		STATE_ACCESS_SRAM,
		STATE_WAIT_SRAM,
		STATE_READ_MEM,
		STATE_WRITE_MEM,
		STATE_UPDATE_SRAM,
		STATE_COMPLETE
	} cache_state_t;
	
	cache_state_t current_state, next_state;
	
	logic [ADDR_WIDTH-1:0] req_addr_reg;
	logic [DATA_WIDTH-1:0] req_wdata_reg;
	logic [7:0] req_wstrb_reg;
	logic req_op_reg;
	logic [$clog2(NUM_WAYS)-1:0] selected_way_reg;
	logic hit_reg;
	logic [DATA_WIDTH-1:0] mem_data_reg;
	logic [DATA_WIDTH-1:0] write_mask;
	
	// Write mask generation
	always_comb begin
		write_mask = 0;
		for (int i = 0; i < 8; i++) begin
			if (req_wstrb_reg[i]) begin
				write_mask[(i*8)+:8] = 8'hFF;
			end
		end
	end
	
	// Prepare tag write data
	always_comb begin
		for (int way = 0; way < NUM_WAYS; way++) begin
			for (int ts = 0; ts < TAG_SRAMS; ts++) begin
				tag_wdata[way][ts] = 0;
				if (way == selected_way_reg && current_state == STATE_UPDATE_SRAM && !hit_reg) begin
					if (ts < TAG_SRAMS-1) begin
						tag_wdata[way][ts] = req_tag[(ts*8)+:8];
					end else begin
						logic [TAG_WIDTH*8-1:0] shifted_tag;

						shifted_tag = req_tag >> (ts*8);

						tag_wdata[way][ts] = shifted_tag[TAG_WIDTH*8-1 -: 8];
					end

				end
			end
		end
	end
	logic [7:0] existing ;
	logic [7:0] new_data ;
	logic [7:0] mask ;
	// Prepare data write data
	always_comb begin
		for (int way = 0; way < NUM_WAYS; way++) begin
			for (int ds = 0; ds < SRAMS_PER_BLOCK; ds++) begin
				data_wdata[way][ds] = 0;
				if (way == selected_way_reg && current_state == STATE_UPDATE_SRAM) begin
					if (hit_reg) begin
						// Write hit - merge
						existing = data_rdata[way][ds];
						new_data = req_wdata_reg[(ds*8)+:8];
						mask = write_mask[(ds*8)+:8];
						data_wdata[way][ds] = (new_data & mask) | (existing & ~mask);
					end else begin
						// Cache fill
						if (req_op_reg) begin
							data_wdata[way][ds] = req_wdata_reg[(ds*8)+:8];
						end else begin
							data_wdata[way][ds] = mem_data_reg[(ds*8)+:8];
						end
					end
				end
			end
		end
	end
	
	// Main FSM
	always_ff @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			current_state <= STATE_IDLE;
			req_addr_reg <= 0;
			req_wdata_reg <= 0;
			req_wstrb_reg <= 0;
			req_op_reg <= 0;
			selected_way_reg <= 0;
			hit_reg <= 0;
			mem_data_reg <= 0;
			
			for (int i = 0; i < NUM_WAYS; i++) begin
				tag_valid[i] <= 1'b0;
				tag_dirty[i] <= 1'b0;
				tag_cen[i] <= 1'b0;
				data_cen[i] <= 1'b0;
				tag_wen[i] <= 1'b0;
				data_wen[i] <= 1'b0;
				tag_addr[i] <= 0;
				data_addr[i] <= 0;
			end
			
		end else begin
			// Default SRAM controls
			for (int i = 0; i < NUM_WAYS; i++) begin
				tag_cen[i] <= 1'b0;
				data_cen[i] <= 1'b0;
				tag_wen[i] <= 1'b0;
				data_wen[i] <= 1'b0;
			end
			
			next_state = current_state;
			
			case (current_state)
				STATE_IDLE: begin
					if (i_req) begin
						req_addr_reg <= i_addr;
						req_wdata_reg <= i_wdata;
						req_wstrb_reg <= i_wstrb;
						req_op_reg <= i_op;
						next_state = STATE_ACCESS_SRAM;
					end
				end
				
				STATE_ACCESS_SRAM: begin
					for (int i = 0; i < NUM_WAYS; i++) begin
						tag_cen[i] <= 1'b1;
						data_cen[i] <= 1'b1;
						tag_addr[i] <= req_intra_bank_addr;
						data_addr[i] <= req_intra_bank_addr;
					end
					next_state = STATE_WAIT_SRAM;
				end
				
				STATE_WAIT_SRAM: begin
					hit_reg <= hit;
					selected_way_reg <= hit ? hit_way : replace_way;
					
					if (hit) begin
						if (req_op_reg) begin
							next_state = STATE_UPDATE_SRAM;
						end else begin
							next_state = STATE_COMPLETE;
						end
					end else begin
						if (tag_dirty[replace_way]) begin
							next_state = STATE_WRITE_MEM;
						end else begin
							next_state = STATE_READ_MEM;
						end
					end
				end
				
				STATE_READ_MEM: begin
					if (i_mem_rdone) begin
						mem_data_reg <= i_mem_rdata;
						next_state = STATE_UPDATE_SRAM;
					end
				end
				
				STATE_WRITE_MEM: begin
					if (i_mem_wdone) begin
						next_state = STATE_READ_MEM;
					end
				end
				
				STATE_UPDATE_SRAM: begin
					tag_cen[selected_way_reg] <= 1'b1;
					data_cen[selected_way_reg] <= 1'b1;
					
					if (hit_reg) begin
						if (req_op_reg) begin
							data_wen[selected_way_reg] <= 1'b1;
							tag_dirty[selected_way_reg] <= 1'b1;
						end
					end else begin
						tag_wen[selected_way_reg] <= 1'b1;
						data_wen[selected_way_reg] <= 1'b1;
						tag_valid[selected_way_reg] <= 1'b1;
						tag_dirty[selected_way_reg] <= req_op_reg;
					end
					next_state = STATE_COMPLETE;
				end
				
				STATE_COMPLETE: begin
					next_state = STATE_IDLE;
				end
				
				default: begin
					next_state = STATE_IDLE;
				end
			endcase
			
			current_state <= next_state;
		end
	end
	
	// ============================================
	// OUTPUT ASSIGNMENTS
	// ============================================
	
	assign o_done = (current_state == STATE_COMPLETE);
	
	assign o_mem_addr = {req_addr_reg[ADDR_WIDTH-1:BLOCK_OFFSET_BITS], 
						{BLOCK_OFFSET_BITS{1'b0}}};
	assign o_mem_waddr = o_mem_addr;
	assign o_mem_wdata = current_data[replace_way][63:0];
	
	assign o_mem_rreq = (current_state == STATE_READ_MEM);
	assign o_mem_wvalid = (current_state == STATE_WRITE_MEM);
	assign o_mem_wstrb = 8'hFF;
	
endmodule

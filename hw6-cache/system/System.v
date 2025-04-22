`default_nettype none
module MyClockGen (
	input_clk_25MHz,
	clk_proc,
	locked
);
	input input_clk_25MHz;
	output wire clk_proc;
	output wire locked;
	wire clkfb;
	(* FREQUENCY_PIN_CLKI = "25" *) (* FREQUENCY_PIN_CLKOP = "17.8571" *) (* ICP_CURRENT = "12" *) (* LPF_RESISTOR = "8" *) (* MFG_ENABLE_FILTEROPAMP = "1" *) (* MFG_GMCREF_SEL = "2" *) EHXPLLL #(
		.PLLRST_ENA("DISABLED"),
		.INTFB_WAKE("DISABLED"),
		.STDBY_ENABLE("DISABLED"),
		.DPHASE_SOURCE("DISABLED"),
		.OUTDIVIDER_MUXA("DIVA"),
		.OUTDIVIDER_MUXB("DIVB"),
		.OUTDIVIDER_MUXC("DIVC"),
		.OUTDIVIDER_MUXD("DIVD"),
		.CLKI_DIV(7),
		.CLKOP_ENABLE("ENABLED"),
		.CLKOP_DIV(34),
		.CLKOP_CPHASE(16),
		.CLKOP_FPHASE(0),
		.FEEDBK_PATH("INT_OP"),
		.CLKFB_DIV(5)
	) pll_i(
		.RST(1'b0),
		.STDBY(1'b0),
		.CLKI(input_clk_25MHz),
		.CLKOP(clk_proc),
		.CLKFB(clkfb),
		.CLKINTFB(clkfb),
		.PHASESEL0(1'b0),
		.PHASESEL1(1'b0),
		.PHASEDIR(1'b1),
		.PHASESTEP(1'b1),
		.PHASELOADREG(1'b1),
		.PLLWAKESYNC(1'b0),
		.ENCLKOP(1'b0),
		.LOCK(locked)
	);
endmodule
`default_nettype none
module SystemResourceCheck (
	external_clk_25MHz,
	btn,
	led
);
	input wire external_clk_25MHz;
	input wire [6:0] btn;
	output wire [7:0] led;
	wire clk;
	wire clk_locked;
	MyClockGen clock_gen(
		.input_clk_25MHz(external_clk_25MHz),
		.clk_proc(clk),
		.locked(clk_locked)
	);
	wire rst = !clk_locked;
	generate
		if (1) begin : axi_data_cache
			localparam signed [31:0] ADDR_WIDTH = 32;
			localparam signed [31:0] DATA_WIDTH = 32;
			reg ARREADY;
			wire ARVALID;
			wire [31:0] ARADDR;
			wire [2:0] ARPROT;
			wire RREADY;
			reg RVALID;
			reg [31:0] RDATA;
			wire [1:0] RRESP;
			reg AWREADY;
			wire AWVALID;
			wire [31:0] AWADDR;
			wire [2:0] AWPROT;
			reg WREADY;
			wire WVALID;
			wire [31:0] WDATA;
			wire [3:0] WSTRB;
			wire BREADY;
			reg BVALID;
			wire [1:0] BRESP;
		end
		if (1) begin : axi_mem_ro
			localparam signed [31:0] ADDR_WIDTH = 32;
			localparam signed [31:0] DATA_WIDTH = 32;
			reg ARREADY;
			wire ARVALID;
			wire [31:0] ARADDR;
			wire [2:0] ARPROT;
			wire RREADY;
			reg RVALID;
			reg [31:0] RDATA;
			wire [1:0] RRESP;
			reg AWREADY;
			wire AWVALID;
			wire [31:0] AWADDR;
			wire [2:0] AWPROT;
			reg WREADY;
			wire WVALID;
			wire [31:0] WDATA;
			wire [3:0] WSTRB;
			wire BREADY;
			wire BVALID;
			wire [1:0] BRESP;
		end
		if (1) begin : axi_mem_rw
			localparam signed [31:0] ADDR_WIDTH = 32;
			localparam signed [31:0] DATA_WIDTH = 32;
			reg ARREADY;
			reg ARVALID;
			reg [31:0] ARADDR;
			wire [2:0] ARPROT;
			reg RREADY;
			reg RVALID;
			reg [31:0] RDATA;
			wire [1:0] RRESP;
			reg AWREADY;
			reg AWVALID;
			reg [31:0] AWADDR;
			wire [2:0] AWPROT;
			reg WREADY;
			reg WVALID;
			reg [31:0] WDATA;
			reg [3:0] WSTRB;
			reg BREADY;
			reg BVALID;
			wire [1:0] BRESP;
		end
	endgenerate
	localparam _param_957C3_NUM_WORDS = 8192;
	generate
		if (1) begin : memory
			localparam signed [31:0] NUM_WORDS = _param_957C3_NUM_WORDS;
			wire ACLK;
			wire ARESETn;
			localparam [0:0] True = 1'b1;
			localparam [0:0] False = 1'b0;
			localparam signed [31:0] AddrLsb = 2;
			localparam signed [31:0] AddrMsb = 14;
			reg [31:0] mem_array [0:8191];
			reg [31:0] ro_araddr;
			reg ro_araddr_valid;
			initial $readmemh("mem_initial_contents.hex", mem_array);
			assign SystemResourceCheck.axi_mem_ro.RRESP = 2'b00;
			assign SystemResourceCheck.axi_mem_ro.BRESP = 2'b00;
			assign SystemResourceCheck.axi_mem_rw.RRESP = 2'b00;
			assign SystemResourceCheck.axi_mem_rw.BRESP = 2'b00;
			always @(posedge ACLK)
				if (!ARESETn) begin
					ro_araddr <= 0;
					ro_araddr_valid <= False;
					SystemResourceCheck.axi_mem_ro.ARREADY <= True;
					SystemResourceCheck.axi_mem_ro.AWREADY <= False;
					SystemResourceCheck.axi_mem_ro.WREADY <= False;
					SystemResourceCheck.axi_mem_ro.RVALID <= False;
					SystemResourceCheck.axi_mem_ro.RDATA <= 0;
					SystemResourceCheck.axi_mem_rw.ARREADY <= True;
					SystemResourceCheck.axi_mem_rw.AWREADY <= True;
					SystemResourceCheck.axi_mem_rw.WREADY <= True;
					SystemResourceCheck.axi_mem_rw.RVALID <= False;
					SystemResourceCheck.axi_mem_rw.RDATA <= 0;
				end
				else begin
					if (ro_araddr_valid) begin
						if (SystemResourceCheck.axi_mem_ro.RREADY) begin
							SystemResourceCheck.axi_mem_ro.RVALID <= True;
							SystemResourceCheck.axi_mem_ro.RDATA <= mem_array[ro_araddr[AddrMsb:AddrLsb]];
							ro_araddr <= 0;
							ro_araddr_valid <= False;
							SystemResourceCheck.axi_mem_ro.ARREADY <= True;
						end
					end
					else if (SystemResourceCheck.axi_mem_ro.ARVALID && SystemResourceCheck.axi_mem_ro.ARREADY) begin
						if (SystemResourceCheck.axi_mem_ro.RVALID && !SystemResourceCheck.axi_mem_ro.RREADY) begin
							ro_araddr <= SystemResourceCheck.axi_mem_ro.ARADDR;
							ro_araddr_valid <= True;
							SystemResourceCheck.axi_mem_ro.ARREADY <= False;
						end
						else begin
							SystemResourceCheck.axi_mem_ro.RVALID <= True;
							SystemResourceCheck.axi_mem_ro.RDATA <= mem_array[SystemResourceCheck.axi_mem_ro.ARADDR[AddrMsb:AddrLsb]];
						end
					end
					else if (SystemResourceCheck.axi_mem_ro.RVALID && SystemResourceCheck.axi_mem_ro.RREADY) begin
						SystemResourceCheck.axi_mem_ro.RVALID <= False;
						SystemResourceCheck.axi_mem_ro.RDATA <= 0;
						SystemResourceCheck.axi_mem_ro.ARREADY <= True;
					end
					if (SystemResourceCheck.axi_mem_rw.ARVALID && SystemResourceCheck.axi_mem_rw.ARREADY) begin
						SystemResourceCheck.axi_mem_rw.RVALID <= True;
						SystemResourceCheck.axi_mem_rw.RDATA <= mem_array[SystemResourceCheck.axi_mem_rw.ARADDR[AddrMsb:AddrLsb]];
					end
					else if (SystemResourceCheck.axi_mem_rw.RVALID) begin
						SystemResourceCheck.axi_mem_rw.RVALID <= False;
						SystemResourceCheck.axi_mem_rw.RDATA <= 0;
					end
					if (((SystemResourceCheck.axi_mem_rw.AWVALID && SystemResourceCheck.axi_mem_rw.AWREADY) && SystemResourceCheck.axi_mem_rw.WVALID) && SystemResourceCheck.axi_mem_rw.WREADY) begin
						if (SystemResourceCheck.axi_mem_rw.WSTRB[0])
							mem_array[SystemResourceCheck.axi_mem_rw.AWADDR[AddrMsb:AddrLsb]][7:0] <= SystemResourceCheck.axi_mem_rw.WDATA[7:0];
						if (SystemResourceCheck.axi_mem_rw.WSTRB[1])
							mem_array[SystemResourceCheck.axi_mem_rw.AWADDR[AddrMsb:AddrLsb]][15:8] <= SystemResourceCheck.axi_mem_rw.WDATA[15:8];
						if (SystemResourceCheck.axi_mem_rw.WSTRB[2])
							mem_array[SystemResourceCheck.axi_mem_rw.AWADDR[AddrMsb:AddrLsb]][23:16] <= SystemResourceCheck.axi_mem_rw.WDATA[23:16];
						if (SystemResourceCheck.axi_mem_rw.WSTRB[3])
							mem_array[SystemResourceCheck.axi_mem_rw.AWADDR[AddrMsb:AddrLsb]][31:24] <= SystemResourceCheck.axi_mem_rw.WDATA[31:24];
						SystemResourceCheck.axi_mem_rw.BVALID <= True;
					end
					else if (SystemResourceCheck.axi_mem_rw.BVALID)
						SystemResourceCheck.axi_mem_rw.BVALID <= False;
				end
		end
	endgenerate
	assign memory.ACLK = clk;
	assign memory.ARESETn = ~rst;
	localparam _param_7E3C6_BLOCK_SIZE_BITS = 32;
	localparam _param_7E3C6_NUM_SETS = 16;
	generate
		if (1) begin : dcache
			reg _sv2v_0;
			localparam signed [31:0] BLOCK_SIZE_BITS = _param_7E3C6_BLOCK_SIZE_BITS;
			localparam signed [31:0] NUM_SETS = _param_7E3C6_NUM_SETS;
			wire ACLK;
			wire ARESETn;
			localparam signed [31:0] BlockOffsetBits = 2;
			localparam signed [31:0] IndexBits = 4;
			localparam signed [31:0] TagBits = 26;
			reg [31:0] current_state;
			reg [31:0] data [0:15];
			reg [25:0] tag [0:15];
			reg [0:0] valid [0:15];
			reg [0:0] dirty [0:15];
			localparam [0:0] True = 1'b1;
			localparam [0:0] False = 1'b0;
			genvar _gv_seti_1;
			for (_gv_seti_1 = 0; _gv_seti_1 < NUM_SETS; _gv_seti_1 = _gv_seti_1 + 1) begin : gen_cache_init
				localparam seti = _gv_seti_1;
				initial begin
					valid[seti] = 1'sb0;
					dirty[seti] = 1'sb0;
					data[seti] = 0;
					tag[seti] = 0;
				end
			end
			always @(*)
				if (_sv2v_0)
					;
			assign SystemResourceCheck.axi_data_cache.RRESP = 2'b00;
			assign SystemResourceCheck.axi_data_cache.BRESP = 2'b00;
			reg [31:0] n_current_state;
			reg n_proc_arready;
			reg n_proc_awready;
			reg n_proc_bvalid;
			reg [31:0] n_proc_rdata;
			reg n_proc_rvalid;
			reg n_proc_wready;
			always @(posedge ACLK)
				if (!ARESETn) begin
					current_state <= 32'd0;
					SystemResourceCheck.axi_data_cache.ARREADY <= True;
					SystemResourceCheck.axi_data_cache.RVALID <= False;
					SystemResourceCheck.axi_data_cache.RDATA <= 0;
					SystemResourceCheck.axi_data_cache.AWREADY <= True;
					SystemResourceCheck.axi_data_cache.WREADY <= True;
					SystemResourceCheck.axi_data_cache.BVALID <= False;
				end
				else begin
					current_state <= n_current_state;
					SystemResourceCheck.axi_data_cache.ARREADY <= n_proc_arready;
					SystemResourceCheck.axi_data_cache.RVALID <= n_proc_rvalid;
					SystemResourceCheck.axi_data_cache.RDATA <= n_proc_rdata;
					SystemResourceCheck.axi_data_cache.AWREADY <= n_proc_awready;
					SystemResourceCheck.axi_data_cache.WREADY <= n_proc_wready;
					SystemResourceCheck.axi_data_cache.BVALID <= n_proc_bvalid;
				end
			wire [3:0] imm_index_read;
			wire [3:0] imm_index_write;
			wire [3:0] process_index;
			wire [25:0] imm_tag_in_read;
			wire [25:0] imm_tag_in_write;
			wire [25:0] process_tag_in;
			wire [31:0] process_write_data;
			assign imm_index_read = SystemResourceCheck.axi_data_cache.ARADDR[(IndexBits + BlockOffsetBits) - 1:BlockOffsetBits];
			assign imm_tag_in_read = SystemResourceCheck.axi_data_cache.ARADDR[((TagBits + IndexBits) + BlockOffsetBits) - 1:IndexBits + BlockOffsetBits];
			assign imm_index_write = SystemResourceCheck.axi_data_cache.AWADDR[(IndexBits + BlockOffsetBits) - 1:BlockOffsetBits];
			assign imm_tag_in_write = SystemResourceCheck.axi_data_cache.AWADDR[((TagBits + IndexBits) + BlockOffsetBits) - 1:IndexBits + BlockOffsetBits];
			reg [3:0] s_index_buffer;
			reg [3:0] n_index_buffer;
			reg [3:0] f_index_buffer;
			reg [3:0] nf_index_buffer;
			reg [25:0] s_tag_buffer;
			reg [25:0] n_tag_buffer;
			reg [25:0] f_tag_buffer;
			reg [25:0] nf_tag_buffer;
			reg [31:0] s_wdata_buffer;
			reg [31:0] n_wdata_buffer;
			reg [31:0] f_wdata_buffer;
			reg [31:0] nf_wdata_buffer;
			reg [3:0] s_wstrb_buffer;
			reg [3:0] n_wstrb_buffer;
			reg [3:0] f_wstrb_buffer;
			reg [3:0] nf_wstrb_buffer;
			reg s_write;
			reg n_write;
			reg f_write;
			reg nf_write;
			reg we;
			reg wf_mem;
			reg dirty_out;
			reg [31:0] data_out;
			reg [3:0] index_out;
			reg [3:0] wstrb_out;
			reg [25:0] tag_out;
			always @(posedge ACLK)
				if (we) begin
					case (wstrb_out)
						4'b0001: data[index_out][7:0] <= data_out[7:0];
						4'b0010: data[index_out][15:8] <= data_out[15:8];
						4'b0100: data[index_out][23:16] <= data_out[23:16];
						4'b1000: data[index_out][31:24] <= data_out[31:24];
						4'b0011: data[index_out][15:0] <= data_out[15:0];
						4'b1100: data[index_out][31:16] <= data_out[31:16];
						4'b1111: data[index_out] <= data_out;
						default:
							;
					endcase
					if (wf_mem) begin
						tag[index_out] <= tag_out;
						valid[index_out] <= True;
					end
					dirty[index_out] <= dirty_out;
				end
			always @(posedge ACLK)
				if (!ARESETn) begin
					s_index_buffer <= 0;
					s_tag_buffer <= 0;
					s_wdata_buffer <= 0;
					s_wstrb_buffer <= 0;
					s_write <= 0;
					f_index_buffer <= 0;
					f_tag_buffer <= 0;
					f_wdata_buffer <= 0;
					f_wstrb_buffer <= 0;
					f_write <= 0;
				end
				else begin
					s_index_buffer <= n_index_buffer;
					s_tag_buffer <= n_tag_buffer;
					s_wdata_buffer <= n_wdata_buffer;
					s_wstrb_buffer <= n_wstrb_buffer;
					s_write <= n_write;
					f_index_buffer <= nf_index_buffer;
					f_tag_buffer <= nf_tag_buffer;
					f_wdata_buffer <= nf_wdata_buffer;
					f_wstrb_buffer <= nf_wstrb_buffer;
					f_write <= nf_write;
				end
			always @(*) begin
				if (_sv2v_0)
					;
				n_current_state = current_state;
				n_proc_arready = SystemResourceCheck.axi_data_cache.ARREADY;
				n_proc_rvalid = SystemResourceCheck.axi_data_cache.RVALID;
				n_proc_rdata = SystemResourceCheck.axi_data_cache.RDATA;
				n_proc_awready = SystemResourceCheck.axi_data_cache.AWREADY;
				n_proc_wready = SystemResourceCheck.axi_data_cache.WREADY;
				n_proc_bvalid = SystemResourceCheck.axi_data_cache.BVALID;
				SystemResourceCheck.axi_mem_rw.ARVALID = False;
				SystemResourceCheck.axi_mem_rw.ARADDR = 0;
				SystemResourceCheck.axi_mem_rw.RREADY = False;
				SystemResourceCheck.axi_mem_rw.AWVALID = False;
				SystemResourceCheck.axi_mem_rw.AWADDR = 0;
				SystemResourceCheck.axi_mem_rw.WVALID = False;
				SystemResourceCheck.axi_mem_rw.WDATA = 0;
				SystemResourceCheck.axi_mem_rw.WSTRB = 0;
				SystemResourceCheck.axi_mem_rw.BREADY = False;
				we = 0;
				wf_mem = 0;
				data_out = 0;
				index_out = 0;
				wstrb_out = 0;
				tag_out = 0;
				case (current_state)
					32'd0: begin
						n_proc_arready = True;
						n_proc_awready = True;
						n_proc_wready = True;
						if (SystemResourceCheck.axi_data_cache.ARVALID && SystemResourceCheck.axi_data_cache.ARREADY) begin
							if (valid[imm_index_read] && (tag[imm_index_read] == imm_tag_in_read)) begin
								n_proc_rvalid = True;
								n_proc_rdata = data[imm_index_read];
								n_current_state = 32'd3;
							end
							else begin
								nf_index_buffer = imm_index_read;
								nf_tag_buffer = imm_tag_in_read;
								nf_write = 0;
								if (dirty[imm_index_read]) begin
									SystemResourceCheck.axi_mem_rw.AWVALID = True;
									SystemResourceCheck.axi_mem_rw.AWADDR = {tag[imm_index_read], imm_index_read, 2'b00};
									SystemResourceCheck.axi_mem_rw.WVALID = True;
									SystemResourceCheck.axi_mem_rw.WDATA = data[imm_index_read];
									SystemResourceCheck.axi_mem_rw.WSTRB = 4'b1111;
									SystemResourceCheck.axi_mem_rw.BREADY = True;
									SystemResourceCheck.axi_mem_rw.RREADY = False;
									n_current_state = 32'd2;
								end
								else begin
									SystemResourceCheck.axi_mem_rw.ARVALID = True;
									SystemResourceCheck.axi_mem_rw.ARADDR = {imm_tag_in_read, imm_index_read, 2'b00};
									SystemResourceCheck.axi_mem_rw.RREADY = True;
									n_current_state = 32'd1;
								end
							end
						end
						else if ((SystemResourceCheck.axi_data_cache.AWVALID && SystemResourceCheck.axi_data_cache.WVALID) && SystemResourceCheck.axi_data_cache.ARREADY) begin
							if (valid[imm_index_write] && (tag[imm_index_write] == imm_tag_in_write)) begin
								we = 1;
								data_out = SystemResourceCheck.axi_data_cache.WDATA;
								index_out = imm_index_write;
								wstrb_out = SystemResourceCheck.axi_data_cache.WSTRB;
								dirty_out = SystemResourceCheck.axi_data_cache.WSTRB != 0;
								n_proc_bvalid = True;
								n_current_state = 32'd3;
							end
							else begin
								nf_index_buffer = imm_index_write;
								nf_tag_buffer = imm_tag_in_write;
								nf_wdata_buffer = SystemResourceCheck.axi_data_cache.WDATA;
								nf_wstrb_buffer = SystemResourceCheck.axi_data_cache.WSTRB;
								nf_write = 1;
								if (dirty[imm_index_write]) begin
									SystemResourceCheck.axi_mem_rw.AWVALID = True;
									SystemResourceCheck.axi_mem_rw.AWADDR = {tag[imm_index_write], imm_index_write, 2'b00};
									SystemResourceCheck.axi_mem_rw.WVALID = True;
									SystemResourceCheck.axi_mem_rw.WDATA = data[imm_index_write];
									SystemResourceCheck.axi_mem_rw.WSTRB = 4'b1111;
									SystemResourceCheck.axi_mem_rw.BREADY = True;
									SystemResourceCheck.axi_mem_rw.RREADY = False;
									n_current_state = 32'd2;
								end
								else begin
									SystemResourceCheck.axi_mem_rw.ARVALID = True;
									SystemResourceCheck.axi_mem_rw.ARADDR = {imm_tag_in_write, imm_index_write, 2'b00};
									SystemResourceCheck.axi_mem_rw.RREADY = True;
									n_current_state = 32'd1;
								end
							end
						end
						else
							n_current_state = 32'd0;
					end
					32'd1: begin
						SystemResourceCheck.axi_mem_rw.RREADY = True;
						if (SystemResourceCheck.axi_mem_rw.RVALID && SystemResourceCheck.axi_mem_rw.RREADY) begin
							if (f_write) begin
								case (f_wstrb_buffer)
									4'b0001: data_out = {SystemResourceCheck.axi_mem_rw.RDATA[31:8], f_wdata_buffer[7:0]};
									4'b0010: data_out = {SystemResourceCheck.axi_mem_rw.RDATA[31:16], f_wdata_buffer[15:8], SystemResourceCheck.axi_mem_rw.RDATA[7:0]};
									4'b0100: data_out = {SystemResourceCheck.axi_mem_rw.RDATA[31:24], f_wdata_buffer[23:16], SystemResourceCheck.axi_mem_rw.RDATA[15:0]};
									4'b1000: data_out = {f_wdata_buffer[31:24], SystemResourceCheck.axi_mem_rw.RDATA[23:0]};
									4'b0011: data_out = {SystemResourceCheck.axi_mem_rw.RDATA[31:16], f_wdata_buffer[15:0]};
									4'b1100: data_out = {f_wdata_buffer[31:16], SystemResourceCheck.axi_mem_rw.RDATA[15:0]};
									4'b1111: data_out = f_wdata_buffer;
									default: data_out = {SystemResourceCheck.axi_mem_rw.RDATA[31:8], f_wdata_buffer[7:0]};
								endcase
								n_proc_bvalid = True;
								dirty_out = f_wstrb_buffer != 0;
							end
							else begin
								data_out = SystemResourceCheck.axi_mem_rw.RDATA;
								n_proc_rvalid = True;
								n_proc_rdata = SystemResourceCheck.axi_mem_rw.RDATA;
								dirty_out = 0;
							end
							index_out = f_index_buffer;
							tag_out = f_tag_buffer;
							wstrb_out = 4'b1111;
							we = 1;
							wf_mem = 1;
							n_current_state = 32'd3;
						end
						else
							n_current_state = 32'd1;
						if (SystemResourceCheck.axi_data_cache.ARVALID && SystemResourceCheck.axi_data_cache.ARREADY) begin
							n_index_buffer = imm_index_read;
							n_tag_buffer = imm_tag_in_read;
							n_wdata_buffer = 0;
							n_wstrb_buffer = 0;
							n_write = 0;
							n_proc_arready = False;
							n_proc_awready = False;
							n_proc_wready = False;
						end
						else if ((SystemResourceCheck.axi_data_cache.AWVALID && SystemResourceCheck.axi_data_cache.WVALID) && SystemResourceCheck.axi_data_cache.ARREADY) begin
							n_index_buffer = imm_index_write;
							n_tag_buffer = imm_tag_in_write;
							n_wdata_buffer = SystemResourceCheck.axi_data_cache.WDATA;
							n_wstrb_buffer = SystemResourceCheck.axi_data_cache.WSTRB;
							n_write = 1;
							n_proc_arready = False;
							n_proc_awready = False;
							n_proc_wready = False;
						end
					end
					32'd2: begin
						SystemResourceCheck.axi_mem_rw.BREADY = True;
						if (SystemResourceCheck.axi_mem_rw.BVALID && SystemResourceCheck.axi_mem_rw.BREADY) begin
							n_current_state = 32'd1;
							SystemResourceCheck.axi_mem_rw.ARVALID = True;
							SystemResourceCheck.axi_mem_rw.ARADDR = {f_tag_buffer, f_index_buffer, 2'b00};
						end
						else
							n_current_state = 32'd2;
						if (SystemResourceCheck.axi_data_cache.ARVALID && SystemResourceCheck.axi_data_cache.ARREADY) begin
							n_index_buffer = imm_index_read;
							n_tag_buffer = imm_tag_in_read;
							n_wdata_buffer = 0;
							n_wstrb_buffer = 0;
							n_write = 0;
							n_proc_arready = False;
							n_proc_awready = False;
							n_proc_wready = False;
						end
						else if ((SystemResourceCheck.axi_data_cache.AWVALID && SystemResourceCheck.axi_data_cache.WVALID) && SystemResourceCheck.axi_data_cache.ARREADY) begin
							n_index_buffer = imm_index_write;
							n_tag_buffer = imm_tag_in_write;
							n_wdata_buffer = SystemResourceCheck.axi_data_cache.WDATA;
							n_wstrb_buffer = SystemResourceCheck.axi_data_cache.WSTRB;
							n_write = 1;
							n_proc_arready = False;
							n_proc_awready = False;
							n_proc_wready = False;
						end
					end
					32'd3: begin
						if (SystemResourceCheck.axi_data_cache.RVALID && SystemResourceCheck.axi_data_cache.RREADY) begin
							n_proc_rvalid = 0;
							n_proc_rdata = 0;
						end
						else if (SystemResourceCheck.axi_data_cache.BVALID && SystemResourceCheck.axi_data_cache.BREADY)
							n_proc_bvalid = 0;
						else
							n_current_state = 32'd3;
						if (SystemResourceCheck.axi_data_cache.ARVALID && SystemResourceCheck.axi_data_cache.ARREADY) begin
							if ((SystemResourceCheck.axi_data_cache.RVALID && !SystemResourceCheck.axi_data_cache.RREADY) || (SystemResourceCheck.axi_data_cache.BVALID && !SystemResourceCheck.axi_data_cache.BREADY)) begin
								n_index_buffer = imm_index_read;
								n_tag_buffer = imm_tag_in_read;
								n_wdata_buffer = 0;
								n_wstrb_buffer = 0;
								n_write = 0;
								n_proc_arready = False;
								n_proc_awready = False;
								n_proc_wready = False;
								n_current_state = 32'd3;
							end
							else if (valid[imm_index_read] && (tag[imm_index_read] == imm_tag_in_read)) begin
								n_proc_rvalid = True;
								n_proc_rdata = data[imm_index_read];
								n_current_state = 32'd3;
							end
							else begin
								nf_index_buffer = imm_index_read;
								nf_tag_buffer = imm_tag_in_read;
								nf_write = 0;
								if (dirty[imm_index_read]) begin
									SystemResourceCheck.axi_mem_rw.AWVALID = True;
									SystemResourceCheck.axi_mem_rw.AWADDR = {tag[imm_index_read], imm_index_read, 2'b00};
									SystemResourceCheck.axi_mem_rw.WVALID = True;
									SystemResourceCheck.axi_mem_rw.WDATA = data[imm_index_read];
									SystemResourceCheck.axi_mem_rw.WSTRB = 4'b1111;
									SystemResourceCheck.axi_mem_rw.BREADY = True;
									SystemResourceCheck.axi_mem_rw.RREADY = False;
									n_current_state = 32'd2;
								end
								else begin
									SystemResourceCheck.axi_mem_rw.ARVALID = True;
									SystemResourceCheck.axi_mem_rw.ARADDR = {imm_tag_in_read, imm_index_read, 2'b00};
									SystemResourceCheck.axi_mem_rw.RREADY = True;
									n_current_state = 32'd1;
								end
							end
						end
						else if (((SystemResourceCheck.axi_data_cache.AWVALID && SystemResourceCheck.axi_data_cache.AWREADY) && SystemResourceCheck.axi_data_cache.WVALID) && SystemResourceCheck.axi_data_cache.WREADY) begin
							if ((SystemResourceCheck.axi_data_cache.RVALID && !SystemResourceCheck.axi_data_cache.RREADY) || (SystemResourceCheck.axi_data_cache.BVALID && !SystemResourceCheck.axi_data_cache.BREADY)) begin
								n_index_buffer = imm_index_write;
								n_tag_buffer = imm_tag_in_write;
								n_wdata_buffer = SystemResourceCheck.axi_data_cache.WDATA;
								n_wstrb_buffer = SystemResourceCheck.axi_data_cache.WSTRB;
								n_write = 1;
								n_proc_arready = False;
								n_proc_awready = False;
								n_proc_wready = False;
								n_current_state = 32'd3;
							end
							else if (valid[imm_index_write] && (tag[imm_index_write] == imm_tag_in_write)) begin
								we = 1;
								data_out = SystemResourceCheck.axi_data_cache.WDATA;
								index_out = imm_index_write;
								wstrb_out = SystemResourceCheck.axi_data_cache.WSTRB;
								n_proc_bvalid = True;
								n_current_state = 32'd3;
							end
							else begin
								nf_index_buffer = imm_index_write;
								nf_tag_buffer = imm_tag_in_write;
								nf_wdata_buffer = SystemResourceCheck.axi_data_cache.WDATA;
								nf_wstrb_buffer = SystemResourceCheck.axi_data_cache.WSTRB;
								nf_write = 1;
								if (dirty[imm_index_write]) begin
									SystemResourceCheck.axi_mem_rw.AWVALID = True;
									SystemResourceCheck.axi_mem_rw.ARADDR = {tag[imm_index_write], imm_index_write, 2'b00};
									SystemResourceCheck.axi_mem_rw.WVALID = True;
									SystemResourceCheck.axi_mem_rw.WDATA = data[imm_index_write];
									SystemResourceCheck.axi_mem_rw.WSTRB = 4'b1111;
									n_current_state = 32'd2;
								end
								else begin
									SystemResourceCheck.axi_mem_rw.ARVALID = True;
									SystemResourceCheck.axi_mem_rw.ARADDR = {imm_tag_in_write, imm_index_write, 2'b00};
									n_current_state = 32'd1;
								end
							end
						end
						else if ((!SystemResourceCheck.axi_data_cache.ARREADY && !SystemResourceCheck.axi_data_cache.AWREADY) && !SystemResourceCheck.axi_data_cache.WREADY) begin
							if ((SystemResourceCheck.axi_data_cache.RVALID && !SystemResourceCheck.axi_data_cache.RREADY) || (SystemResourceCheck.axi_data_cache.BVALID && !SystemResourceCheck.axi_data_cache.BREADY)) begin
								n_index_buffer = s_index_buffer;
								n_tag_buffer = s_tag_buffer;
								n_wdata_buffer = s_wdata_buffer;
								n_wstrb_buffer = s_wstrb_buffer;
								n_write = s_write;
								n_proc_arready = False;
								n_proc_awready = False;
								n_proc_wready = False;
								n_current_state = 32'd3;
							end
							else begin
								if (s_write) begin
									if (valid[s_index_buffer] && (tag[s_index_buffer] == s_tag_buffer)) begin
										we = 1;
										data_out = s_wdata_buffer;
										index_out = s_index_buffer;
										wstrb_out = s_wstrb_buffer;
										dirty_out = s_wstrb_buffer != 0;
										n_proc_bvalid = True;
										n_current_state = 32'd3;
									end
									else begin
										nf_index_buffer = s_index_buffer;
										nf_tag_buffer = s_tag_buffer;
										nf_wdata_buffer = s_wdata_buffer;
										nf_wstrb_buffer = s_wstrb_buffer;
										nf_write = 1;
										if (dirty[s_index_buffer]) begin
											SystemResourceCheck.axi_mem_rw.AWVALID = True;
											SystemResourceCheck.axi_mem_rw.AWADDR = {tag[s_index_buffer], s_index_buffer, 2'b00};
											SystemResourceCheck.axi_mem_rw.WVALID = True;
											SystemResourceCheck.axi_mem_rw.WDATA = data[s_index_buffer];
											SystemResourceCheck.axi_mem_rw.WSTRB = 4'b1111;
											SystemResourceCheck.axi_mem_rw.RREADY = False;
											n_current_state = 32'd2;
										end
										else begin
											SystemResourceCheck.axi_mem_rw.ARVALID = True;
											SystemResourceCheck.axi_mem_rw.ARADDR = {s_tag_buffer, s_index_buffer, 2'b00};
											n_current_state = 32'd1;
										end
									end
								end
								else if (valid[s_index_buffer] && (tag[s_index_buffer] == s_tag_buffer)) begin
									n_proc_rvalid = True;
									n_proc_rdata = data[s_index_buffer];
									n_current_state = 32'd3;
								end
								else begin
									nf_index_buffer = s_index_buffer;
									nf_tag_buffer = s_tag_buffer;
									nf_write = 0;
									if (dirty[s_index_buffer]) begin
										SystemResourceCheck.axi_mem_rw.AWVALID = True;
										SystemResourceCheck.axi_mem_rw.AWADDR = {tag[s_index_buffer], s_index_buffer, 2'b00};
										SystemResourceCheck.axi_mem_rw.WVALID = True;
										SystemResourceCheck.axi_mem_rw.WDATA = data[s_index_buffer];
										SystemResourceCheck.axi_mem_rw.WSTRB = 4'b1111;
										n_current_state = 32'd2;
									end
									else begin
										SystemResourceCheck.axi_mem_rw.ARVALID = True;
										SystemResourceCheck.axi_mem_rw.ARADDR = {s_tag_buffer, s_index_buffer, 2'b00};
										n_current_state = 32'd1;
									end
								end
								n_index_buffer = 0;
								n_tag_buffer = 0;
								n_wdata_buffer = 0;
								n_wstrb_buffer = 0;
								n_write = 0;
								n_proc_arready = True;
								n_proc_awready = True;
								n_proc_wready = True;
							end
						end
						else if ((SystemResourceCheck.axi_data_cache.RVALID && SystemResourceCheck.axi_data_cache.RREADY) || (SystemResourceCheck.axi_data_cache.BVALID && SystemResourceCheck.axi_data_cache.BREADY))
							n_current_state = 32'd0;
					end
					default:
						;
				endcase
			end
			initial _sv2v_0 = 0;
		end
	endgenerate
	assign dcache.ACLK = clk;
	assign dcache.ARESETn = ~rst;
	generate
		if (1) begin : datapath
			wire clk;
			wire rst;
			wire halt;
			wire [31:0] trace_writeback_pc;
			wire [31:0] trace_writeback_insn;
			wire [31:0] trace_writeback_cycle_status;
			localparam [0:0] True = 1'b1;
			localparam [0:0] False = 1'b0;
			reg [31:0] cycles_current;
			always @(posedge clk)
				if (rst)
					cycles_current <= 0;
				else
					cycles_current <= cycles_current + 1;
		end
	endgenerate
	assign datapath.clk = clk;
	assign datapath.rst = rst;
	assign led[0] = datapath.halt;
endmodule
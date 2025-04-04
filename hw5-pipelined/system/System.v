module MyClockGen (
	input_clk_25MHz,
	clk_proc,
	locked
);
	input input_clk_25MHz;
	output wire clk_proc;
	output wire locked;
	wire clkfb;
	(* FREQUENCY_PIN_CLKI = "25" *) (* FREQUENCY_PIN_CLKOP = "30" *) (* ICP_CURRENT = "12" *) (* LPF_RESISTOR = "8" *) (* MFG_ENABLE_FILTEROPAMP = "1" *) (* MFG_GMCREF_SEL = "2" *) EHXPLLL #(
		.PLLRST_ENA("DISABLED"),
		.INTFB_WAKE("DISABLED"),
		.STDBY_ENABLE("DISABLED"),
		.DPHASE_SOURCE("DISABLED"),
		.OUTDIVIDER_MUXA("DIVA"),
		.OUTDIVIDER_MUXB("DIVB"),
		.OUTDIVIDER_MUXC("DIVC"),
		.OUTDIVIDER_MUXD("DIVD"),
		.CLKI_DIV(5),
		.CLKOP_ENABLE("ENABLED"),
		.CLKOP_DIV(20),
		.CLKOP_CPHASE(9),
		.CLKOP_FPHASE(0),
		.FEEDBK_PATH("INT_OP"),
		.CLKFB_DIV(6)
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
module gp1 (
	a,
	b,
	g,
	p
);
	input wire a;
	input wire b;
	output wire g;
	output wire p;
	assign g = a & b;
	assign p = a | b;
endmodule
module gp4 (
	gin,
	pin,
	cin,
	gout,
	pout,
	cout
);
	input wire [3:0] gin;
	input wire [3:0] pin;
	input wire cin;
	output wire gout;
	output wire pout;
	output wire [2:0] cout;
	assign pout = &pin;
	assign gout = ((gin[3] | (gin[2] & pin[3])) | (gin[1] & (&pin[3:2]))) | (gin[0] & (&pin[3:1]));
	assign cout[0] = gin[0] | (cin & pin[0]);
	assign cout[1] = (gin[1] | (gin[0] & pin[1])) | (cin & (&pin[1:0]));
	assign cout[2] = ((gin[2] | (gin[1] & pin[2])) | (gin[0] & (&pin[2:1]))) | (cin & (&pin[2:0]));
endmodule
module gp8 (
	gin,
	pin,
	cin,
	gout,
	pout,
	cout
);
	input wire [7:0] gin;
	input wire [7:0] pin;
	input wire cin;
	output wire gout;
	output wire pout;
	output wire [6:0] cout;
	wire p0;
	wire g0;
	wire p1;
	wire g1;
	assign pout = &pin;
	assign gout = g1 | (g0 & p1);
	assign cout[3] = (cin & p0) | g0;
	gp4 m1(
		.gin(gin[3:0]),
		.pin(pin[3:0]),
		.cin(cin),
		.gout(g0),
		.pout(p0),
		.cout(cout[2:0])
	);
	gp4 m2(
		.gin(gin[7:4]),
		.pin(pin[7:4]),
		.cin(cout[3]),
		.gout(g1),
		.pout(p1),
		.cout(cout[6:4])
	);
endmodule
module cla (
	a,
	b,
	cin,
	sum
);
	input wire [31:0] a;
	input wire [31:0] b;
	input wire cin;
	output wire [31:0] sum;
	wire [31:0] g;
	wire [31:0] p;
	wire [6:0] c1 [0:3];
	wire [3:0] c4;
	wire [3:0] p0;
	wire [3:0] g0;
	wire [31:0] carry;
	wire [31:0] cnull;
	assign c4[0] = cin;
	genvar _gv_i_1;
	generate
		for (_gv_i_1 = 0; _gv_i_1 < 32; _gv_i_1 = _gv_i_1 + 1) begin : genblk1
			localparam i = _gv_i_1;
			gp1 gp(
				.a(a[i]),
				.b(b[i]),
				.g(g[i]),
				.p(p[i])
			);
		end
	endgenerate
	genvar _gv_k_1;
	generate
		for (_gv_k_1 = 0; _gv_k_1 < 4; _gv_k_1 = _gv_k_1 + 1) begin : genblk2
			localparam k = _gv_k_1;
			gp8 m1(
				.gin(g[7 + (8 * k):0 + (8 * k)]),
				.pin(p[7 + (8 * k):0 + (8 * k)]),
				.cin(c4[k]),
				.gout(g0[k]),
				.pout(p0[k]),
				.cout(c1[k])
			);
		end
	endgenerate
	wire gout;
	wire pout;
	gp4 m(
		.gin(g0),
		.pin(p0),
		.cin(cin),
		.gout(gout),
		.pout(pout),
		.cout(c4[3:1])
	);
	assign carry = {c1[3][6:0], c4[3], c1[2][6:0], c4[2], c1[1][6:0], c4[1], c1[0][6:0], c4[0]};
	genvar _gv_k2_1;
	generate
		for (_gv_k2_1 = 0; _gv_k2_1 < 32; _gv_k2_1 = _gv_k2_1 + 1) begin : genblk3
			localparam k2 = _gv_k2_1;
			fulladder fa(
				.a(a[k2]),
				.b(b[k2]),
				.s(sum[k2]),
				.cin(carry[k2]),
				.cout(cnull[k2])
			);
		end
	endgenerate
endmodule
module halfadder (
	a,
	b,
	s,
	cout
);
	input wire a;
	input wire b;
	output wire s;
	output wire cout;
	assign s = a ^ b;
	assign cout = a & b;
endmodule
module fulladder (
	cin,
	a,
	b,
	s,
	cout
);
	input wire cin;
	input wire a;
	input wire b;
	output wire s;
	output wire cout;
	wire s_tmp;
	wire cout_tmp1;
	wire cout_tmp2;
	halfadder h0(
		.a(a),
		.b(b),
		.s(s_tmp),
		.cout(cout_tmp1)
	);
	halfadder h1(
		.a(s_tmp),
		.b(cin),
		.s(s),
		.cout(cout_tmp2)
	);
	assign cout = cout_tmp1 | cout_tmp2;
endmodule
module fulladder2 (
	cin,
	a,
	b,
	s,
	cout
);
	input wire cin;
	input wire [1:0] a;
	input wire [1:0] b;
	output wire [1:0] s;
	output wire cout;
	wire cout_tmp;
	fulladder a0(
		.cin(cin),
		.a(a[0]),
		.b(b[0]),
		.s(s[0]),
		.cout(cout_tmp)
	);
	fulladder a1(
		.cin(cout_tmp),
		.a(a[1]),
		.b(b[1]),
		.s(s[1]),
		.cout(cout)
	);
endmodule
module rca4 (
	cin,
	a,
	b,
	sum,
	cout
);
	input wire cin;
	input wire [3:0] a;
	input wire [3:0] b;
	output wire [3:0] sum;
	output wire cout;
	wire cout0;
	wire carry_out;
	fulladder2 a0(
		.cin(cin),
		.a(a[1:0]),
		.b(b[1:0]),
		.s(sum[1:0]),
		.cout(cout0)
	);
	fulladder2 a3(
		.cin(cout0),
		.a(a[3:2]),
		.b(b[3:2]),
		.s(sum[3:2]),
		.cout(cout)
	);
endmodule
module rca8 (
	cin,
	a,
	b,
	sum,
	cout
);
	input wire cin;
	input wire [7:0] a;
	input wire [7:0] b;
	output wire [7:0] sum;
	output wire cout;
	wire cout0;
	rca4 a0(
		.cin(cin),
		.a(a[3:0]),
		.b(b[3:0]),
		.sum(sum[3:0]),
		.cout(cout0)
	);
	rca4 a7(
		.cin(cout0),
		.a(a[7:4]),
		.b(b[7:4]),
		.sum(sum[7:4]),
		.cout(cout)
	);
endmodule
module rca32 (
	cin,
	a,
	b,
	sum32,
	carry_out
);
	input wire cin;
	input wire [31:0] a;
	input wire [31:0] b;
	output wire [31:0] sum32;
	output wire carry_out;
	wire cout0;
	wire cout1;
	wire cout2;
	rca8 a0(
		.cin(cin),
		.a(a[7:0]),
		.b(b[7:0]),
		.sum(sum32[7:0]),
		.cout(cout0)
	);
	rca8 a8(
		.cin(cout0),
		.a(a[15:8]),
		.b(b[15:8]),
		.sum(sum32[15:8]),
		.cout(cout1)
	);
	rca8 a16(
		.cin(cout1),
		.a(a[23:16]),
		.b(b[23:16]),
		.sum(sum32[23:16]),
		.cout(cout2)
	);
	rca8 a24(
		.cin(cout2),
		.a(a[31:24]),
		.b(b[31:24]),
		.sum(sum32[31:24]),
		.cout(carry_out)
	);
endmodule
module DividerUnsignedPipelined (
	clk,
	rst,
	stall,
	i_dividend,
	i_divisor,
	o_remainder,
	o_quotient
);
	input wire clk;
	input wire rst;
	input wire stall;
	input wire [31:0] i_dividend;
	input wire [31:0] i_divisor;
	output wire [31:0] o_remainder;
	output wire [31:0] o_quotient;
	wire [31:0] div_temp [0:32];
	wire [31:0] rem_temp [0:32];
	wire [31:0] q_temp [0:32];
	wire [31:0] divisor_temp [0:32];
	reg [127:0] registers [0:6];
	assign div_temp[0] = i_dividend;
	assign rem_temp[0] = 32'b00000000000000000000000000000000;
	assign q_temp[0] = 32'b00000000000000000000000000000000;
	always @(posedge clk)
		if (rst) begin : sv2v_autoblock_1
			integer j0;
			for (j0 = 0; j0 < 7; j0 = j0 + 1)
				registers[j0] <= 128'd0;
		end
		else begin : sv2v_autoblock_2
			integer j;
			for (j = 0; j < 7; j = j + 1)
				begin
					registers[j][31:0] <= div_temp[4 * (j + 1)];
					if (j == 0)
						registers[0][63:32] <= i_divisor;
					else
						registers[j][63:32] <= registers[j - 1][63:32];
					registers[j][95:64] <= rem_temp[4 * (j + 1)];
					registers[j][127:96] <= q_temp[4 * (j + 1)];
				end
		end
	genvar _gv_j3_1;
	generate
		for (_gv_j3_1 = 1; _gv_j3_1 < 8; _gv_j3_1 = _gv_j3_1 + 1) begin : genblk1
			localparam j3 = _gv_j3_1;
			assign div_temp[j3 * 4] = registers[j3 - 1][31:0];
			assign divisor_temp[j3 * 4] = registers[j3 - 1][63:32];
			assign rem_temp[j3 * 4] = registers[j3 - 1][95:64];
			assign q_temp[j3 * 4] = registers[j3 - 1][127:96];
		end
	endgenerate
	genvar _gv_i_2;
	generate
		for (_gv_i_2 = 0; _gv_i_2 < 32; _gv_i_2 = _gv_i_2 + 1) begin : genblk2
			localparam i = _gv_i_2;
			if (i < 4) begin : gen_cycle_1
				divu_1iter comparator(
					.i_dividend(div_temp[i]),
					.i_divisor(i_divisor),
					.i_remainder(rem_temp[i]),
					.i_quotient(q_temp[i]),
					.o_dividend(div_temp[i + 1]),
					.o_remainder(rem_temp[i + 1]),
					.o_quotient(q_temp[i + 1])
				);
			end
			else if ((i > 3) && (i < 8)) begin : gen_cycle_2
				divu_1iter comparator(
					.i_dividend(div_temp[i]),
					.i_divisor(registers[0][63:32]),
					.i_remainder(rem_temp[i]),
					.i_quotient(q_temp[i]),
					.o_dividend(div_temp[i + 1]),
					.o_remainder(rem_temp[i + 1]),
					.o_quotient(q_temp[i + 1])
				);
			end
			else if ((7 < i) && (i < 12)) begin : gen_cycle_3
				divu_1iter comparator(
					.i_dividend(div_temp[i]),
					.i_divisor(registers[1][63:32]),
					.i_remainder(rem_temp[i]),
					.i_quotient(q_temp[i]),
					.o_dividend(div_temp[i + 1]),
					.o_remainder(rem_temp[i + 1]),
					.o_quotient(q_temp[i + 1])
				);
			end
			else if ((11 < i) && (i < 16)) begin : gen_cycle_4
				divu_1iter comparator(
					.i_dividend(div_temp[i]),
					.i_divisor(registers[2][63:32]),
					.i_remainder(rem_temp[i]),
					.i_quotient(q_temp[i]),
					.o_dividend(div_temp[i + 1]),
					.o_remainder(rem_temp[i + 1]),
					.o_quotient(q_temp[i + 1])
				);
			end
			else if ((15 < i) && (i < 20)) begin : gen_cycle_5
				divu_1iter comparator(
					.i_dividend(div_temp[i]),
					.i_divisor(registers[3][63:32]),
					.i_remainder(rem_temp[i]),
					.i_quotient(q_temp[i]),
					.o_dividend(div_temp[i + 1]),
					.o_remainder(rem_temp[i + 1]),
					.o_quotient(q_temp[i + 1])
				);
			end
			else if ((19 < i) && (i < 24)) begin : gen_cycle_6
				divu_1iter comparator(
					.i_dividend(div_temp[i]),
					.i_divisor(registers[4][63:32]),
					.i_remainder(rem_temp[i]),
					.i_quotient(q_temp[i]),
					.o_dividend(div_temp[i + 1]),
					.o_remainder(rem_temp[i + 1]),
					.o_quotient(q_temp[i + 1])
				);
			end
			else if ((23 < i) && (i < 28)) begin : gen_cycle_7
				divu_1iter comparator(
					.i_dividend(div_temp[i]),
					.i_divisor(registers[5][63:32]),
					.i_remainder(rem_temp[i]),
					.i_quotient(q_temp[i]),
					.o_dividend(div_temp[i + 1]),
					.o_remainder(rem_temp[i + 1]),
					.o_quotient(q_temp[i + 1])
				);
			end
			else if (27 < i) begin : gen_cycle_8
				divu_1iter comparator(
					.i_dividend(div_temp[i]),
					.i_divisor(registers[6][63:32]),
					.i_remainder(rem_temp[i]),
					.i_quotient(q_temp[i]),
					.o_dividend(div_temp[i + 1]),
					.o_remainder(rem_temp[i + 1]),
					.o_quotient(q_temp[i + 1])
				);
			end
		end
	endgenerate
	assign o_remainder = rem_temp[32];
	assign o_quotient = q_temp[32];
endmodule
module divu_1iter (
	i_dividend,
	i_divisor,
	i_remainder,
	i_quotient,
	o_dividend,
	o_remainder,
	o_quotient
);
	reg _sv2v_0;
	input wire [31:0] i_dividend;
	input wire [31:0] i_divisor;
	input wire [31:0] i_remainder;
	input wire [31:0] i_quotient;
	output wire [31:0] o_dividend;
	output wire [31:0] o_remainder;
	output wire [31:0] o_quotient;
	wire [31:0] dividend_temp;
	assign dividend_temp[31:0] = {i_remainder[30:0], i_dividend[31]};
	wire [31:0] n_divisor;
	assign n_divisor[31:0] = ~i_divisor[31:0];
	wire [31:0] sum;
	wire c_out;
	rca32 a32(
		.cin(1'b1),
		.a(dividend_temp[31:0]),
		.b(n_divisor[31:0]),
		.sum32(sum[31:0]),
		.carry_out(c_out)
	);
	always @(*) begin : pos_diff_mux
		if (_sv2v_0)
			;
		if (c_out)
			o_remainder[31:0] = sum[31:0];
		else
			o_remainder[31:0] = dividend_temp[31:0];
	end
	assign o_quotient[31:1] = i_quotient[30:0];
	assign o_quotient[0] = c_out;
	assign o_dividend[31:0] = {i_dividend[30:0], 1'b0};
	initial _sv2v_0 = 0;
endmodule
module Disasm (
	insn,
	disasm
);
	parameter signed [7:0] PREFIX = "D";
	input wire [31:0] insn;
	output wire [255:0] disasm;
endmodule
module RegFile (
	rd,
	rd_data,
	rs1,
	rs1_data,
	rs2,
	rs2_data,
	clk,
	we,
	rst
);
	input wire [4:0] rd;
	input wire [31:0] rd_data;
	input wire [4:0] rs1;
	output wire [31:0] rs1_data;
	input wire [4:0] rs2;
	output wire [31:0] rs2_data;
	input wire clk;
	input wire we;
	input wire rst;
	localparam signed [31:0] NumRegs = 32;
	reg [31:0] regs [0:31];
	assign rs1_data = regs[rs1];
	assign rs2_data = regs[rs2];
	wire [32:1] sv2v_tmp_A2C07;
	assign sv2v_tmp_A2C07 = 32'b00000000000000000000000000000000;
	always @(*) regs[0] = sv2v_tmp_A2C07;
	always @(posedge clk)
		if (rst) begin : sv2v_autoblock_1
			integer i;
			for (i = 1; i < 32; i = i + 1)
				regs[i] <= 32'd0;
		end
		else if (we) begin
			if (rd != 0)
				regs[rd] <= rd_data;
		end
endmodule
module DatapathPipelined (
	clk,
	rst,
	pc_to_imem,
	insn_from_imem,
	addr_to_dmem,
	load_data_from_dmem,
	store_data_to_dmem,
	store_we_to_dmem,
	halt,
	trace_writeback_pc,
	trace_writeback_insn,
	trace_writeback_cycle_status
);
	reg _sv2v_0;
	input wire clk;
	input wire rst;
	output wire [31:0] pc_to_imem;
	input wire [31:0] insn_from_imem;
	output wire [31:0] addr_to_dmem;
	input wire [31:0] load_data_from_dmem;
	output reg [31:0] store_data_to_dmem;
	output reg [3:0] store_we_to_dmem;
	output wire halt;
	output wire [31:0] trace_writeback_pc;
	output wire [31:0] trace_writeback_insn;
	output wire [31:0] trace_writeback_cycle_status;
	reg [31:0] cycles_current;
	always @(posedge clk)
		if (rst)
			cycles_current <= 0;
		else
			cycles_current <= cycles_current + 1;
	reg [31:0] data_to_reg;
	wire [4:0] rd_to_reg;
	reg we_to_reg;
	wire [31:0] x_rs1_data;
	wire [31:0] x_rs2_data;
	reg [315:0] x_state;
	RegFile rf(
		.clk(clk),
		.rst(rst),
		.we(we_to_reg),
		.rd(rd_to_reg),
		.rd_data(data_to_reg),
		.rs1(x_state[271:267]),
		.rs2(x_state[276:272]),
		.rs1_data(x_rs1_data),
		.rs2_data(x_rs2_data)
	);
	reg [31:0] f_pc_current;
	reg [31:0] f_pc_next;
	wire [31:0] f_insn;
	reg [31:0] f_cycle_status;
	reg x_branch;
	always @(*) begin
		if (_sv2v_0)
			;
		if (rst)
			f_cycle_status = 32'd2;
		else if (x_branch)
			f_cycle_status = 32'd4;
		else
			f_cycle_status = 32'd2;
	end
	wire f_div_stall_next;
	reg f_div_stall_curr;
	reg f_load_stall_next;
	reg f_load_stall_curr;
	reg f_fence;
	always @(posedge clk)
		if (rst) begin
			f_pc_current <= 32'd0;
			f_div_stall_curr <= 1'b0;
			f_load_stall_curr <= 1'b0;
		end
		else if ((f_fence || f_div_stall_next) || f_load_stall_next) begin
			f_pc_current <= f_pc_current;
			f_div_stall_curr <= f_div_stall_next;
			f_load_stall_curr <= f_load_stall_next;
		end
		else begin
			f_pc_current <= f_pc_next;
			f_div_stall_curr <= f_div_stall_next;
			f_load_stall_curr <= f_load_stall_next;
		end
	assign pc_to_imem = f_pc_current;
	assign f_insn = (x_branch ? 32'd0 : insn_from_imem);
	wire [255:0] f_disasm;
	Disasm #(.PREFIX("F")) disasm_0fetch(
		.insn(f_insn),
		.disasm(f_disasm)
	);
	reg [95:0] decode_state;
	always @(posedge clk)
		if (rst)
			decode_state <= 96'h000000000000000000000001;
		else if (x_branch)
			decode_state <= {32'd0, f_insn, 32'd4};
		else if ((f_div_stall_next || f_load_stall_next) || f_fence)
			decode_state <= decode_state;
		else
			decode_state <= {f_pc_current, f_insn, f_cycle_status};
	wire [255:0] d_disasm;
	Disasm #(.PREFIX("D")) disasm_1decode(
		.insn(decode_state[63-:32]),
		.disasm(d_disasm)
	);
	wire [6:0] d_insn_funct7;
	wire [4:0] d_insn_rs2;
	wire [4:0] d_insn_rs1;
	wire [2:0] d_insn_funct3;
	wire [4:0] d_insn_rd;
	wire [6:0] d_insn_opcode;
	assign {d_insn_funct7, d_insn_rs2, d_insn_rs1, d_insn_funct3, d_insn_rd, d_insn_opcode} = decode_state[63-:32];
	wire [19:0] d_imm_u;
	assign d_imm_u = decode_state[63:44];
	wire [11:0] d_imm_i;
	assign d_imm_i = decode_state[63:52];
	wire [4:0] d_imm_shamt = decode_state[56:52];
	wire [11:0] d_imm_s;
	assign d_imm_s[11:5] = d_insn_funct7;
	assign d_imm_s[4:0] = d_insn_rd;
	wire [12:0] d_imm_b;
	assign {d_imm_b[12], d_imm_b[10:5]} = d_insn_funct7;
	assign {d_imm_b[4:1], d_imm_b[11]} = d_insn_rd;
	assign d_imm_b[0] = 1'b0;
	wire [20:0] d_imm_j;
	assign {d_imm_j[20], d_imm_j[10:1], d_imm_j[11], d_imm_j[19:12], d_imm_j[0]} = {decode_state[63:44], 1'b0};
	wire [31:0] d_imm_i_sext = {{20 {d_imm_i[11]}}, d_imm_i[11:0]};
	wire [31:0] d_imm_s_sext = {{20 {d_imm_s[11]}}, d_imm_s[11:0]};
	wire [31:0] d_imm_b_sext = {{19 {d_imm_b[12]}}, d_imm_b[12:0]};
	wire [31:0] d_imm_j_sext = {{11 {d_imm_j[20]}}, d_imm_j[20:0]};
	localparam [6:0] OpLoad = 7'b0000011;
	localparam [6:0] OpStore = 7'b0100011;
	localparam [6:0] OpBranch = 7'b1100011;
	localparam [6:0] OpJalr = 7'b1100111;
	localparam [6:0] OpMiscMem = 7'b0001111;
	localparam [6:0] OpJal = 7'b1101111;
	localparam [6:0] OpRegImm = 7'b0010011;
	localparam [6:0] OpRegReg = 7'b0110011;
	localparam [6:0] OpEnviron = 7'b1110011;
	localparam [6:0] OpAuipc = 7'b0010111;
	localparam [6:0] OpLui = 7'b0110111;
	wire d_insn_lui = d_insn_opcode == OpLui;
	wire d_insn_auipc = d_insn_opcode == OpAuipc;
	wire d_insn_jal = d_insn_opcode == OpJal;
	wire d_insn_jalr = d_insn_opcode == OpJalr;
	wire d_insn_beq = (d_insn_opcode == OpBranch) && (decode_state[46:44] == 3'b000);
	wire d_insn_bne = (d_insn_opcode == OpBranch) && (decode_state[46:44] == 3'b001);
	wire d_insn_blt = (d_insn_opcode == OpBranch) && (decode_state[46:44] == 3'b100);
	wire d_insn_bge = (d_insn_opcode == OpBranch) && (decode_state[46:44] == 3'b101);
	wire d_insn_bltu = (d_insn_opcode == OpBranch) && (decode_state[46:44] == 3'b110);
	wire d_insn_bgeu = (d_insn_opcode == OpBranch) && (decode_state[46:44] == 3'b111);
	wire d_insn_lb = (d_insn_opcode == OpLoad) && (decode_state[46:44] == 3'b000);
	wire d_insn_lh = (d_insn_opcode == OpLoad) && (decode_state[46:44] == 3'b001);
	wire d_insn_lw = (d_insn_opcode == OpLoad) && (decode_state[46:44] == 3'b010);
	wire d_insn_lbu = (d_insn_opcode == OpLoad) && (decode_state[46:44] == 3'b100);
	wire d_insn_lhu = (d_insn_opcode == OpLoad) && (decode_state[46:44] == 3'b101);
	wire d_insn_sh = (d_insn_opcode == OpStore) && (decode_state[46:44] == 3'b001);
	wire d_insn_sb = (d_insn_opcode == OpStore) && (decode_state[46:44] == 3'b000);
	wire d_insn_sw = (d_insn_opcode == OpStore) && (decode_state[46:44] == 3'b010);
	wire d_insn_addi = (d_insn_opcode == OpRegImm) && (decode_state[46:44] == 3'b000);
	wire d_insn_slti = (d_insn_opcode == OpRegImm) && (decode_state[46:44] == 3'b010);
	wire d_insn_sltiu = (d_insn_opcode == OpRegImm) && (decode_state[46:44] == 3'b011);
	wire d_insn_xori = (d_insn_opcode == OpRegImm) && (decode_state[46:44] == 3'b100);
	wire d_insn_ori = (d_insn_opcode == OpRegImm) && (decode_state[46:44] == 3'b110);
	wire d_insn_andi = (d_insn_opcode == OpRegImm) && (decode_state[46:44] == 3'b111);
	wire d_insn_slli = ((d_insn_opcode == OpRegImm) && (decode_state[46:44] == 3'b001)) && (decode_state[63:57] == 7'd0);
	wire d_insn_srli = ((d_insn_opcode == OpRegImm) && (decode_state[46:44] == 3'b101)) && (decode_state[63:57] == 7'd0);
	wire d_insn_srai = ((d_insn_opcode == OpRegImm) && (decode_state[46:44] == 3'b101)) && (decode_state[63:57] == 7'b0100000);
	wire d_insn_add = ((d_insn_opcode == OpRegReg) && (decode_state[46:44] == 3'b000)) && (decode_state[63:57] == 7'd0);
	wire d_insn_sub = ((d_insn_opcode == OpRegReg) && (decode_state[46:44] == 3'b000)) && (decode_state[63:57] == 7'b0100000);
	wire d_insn_sll = ((d_insn_opcode == OpRegReg) && (decode_state[46:44] == 3'b001)) && (decode_state[63:57] == 7'd0);
	wire d_insn_slt = ((d_insn_opcode == OpRegReg) && (decode_state[46:44] == 3'b010)) && (decode_state[63:57] == 7'd0);
	wire d_insn_sltu = ((d_insn_opcode == OpRegReg) && (decode_state[46:44] == 3'b011)) && (decode_state[63:57] == 7'd0);
	wire d_insn_xor = ((d_insn_opcode == OpRegReg) && (decode_state[46:44] == 3'b100)) && (decode_state[63:57] == 7'd0);
	wire d_insn_srl = ((d_insn_opcode == OpRegReg) && (decode_state[46:44] == 3'b101)) && (decode_state[63:57] == 7'd0);
	wire d_insn_sra = ((d_insn_opcode == OpRegReg) && (decode_state[46:44] == 3'b101)) && (decode_state[63:57] == 7'b0100000);
	wire d_insn_or = ((d_insn_opcode == OpRegReg) && (decode_state[46:44] == 3'b110)) && (decode_state[63:57] == 7'd0);
	wire d_insn_and = ((d_insn_opcode == OpRegReg) && (decode_state[46:44] == 3'b111)) && (decode_state[63:57] == 7'd0);
	wire d_insn_mul = ((d_insn_opcode == OpRegReg) && (decode_state[63:57] == 7'd1)) && (decode_state[46:44] == 3'b000);
	wire d_insn_mulh = ((d_insn_opcode == OpRegReg) && (decode_state[63:57] == 7'd1)) && (decode_state[46:44] == 3'b001);
	wire d_insn_mulhsu = ((d_insn_opcode == OpRegReg) && (decode_state[63:57] == 7'd1)) && (decode_state[46:44] == 3'b010);
	wire d_insn_mulhu = ((d_insn_opcode == OpRegReg) && (decode_state[63:57] == 7'd1)) && (decode_state[46:44] == 3'b011);
	wire d_insn_div = ((d_insn_opcode == OpRegReg) && (decode_state[63:57] == 7'd1)) && (decode_state[46:44] == 3'b100);
	wire d_insn_divu = ((d_insn_opcode == OpRegReg) && (decode_state[63:57] == 7'd1)) && (decode_state[46:44] == 3'b101);
	wire d_insn_rem = ((d_insn_opcode == OpRegReg) && (decode_state[63:57] == 7'd1)) && (decode_state[46:44] == 3'b110);
	wire d_insn_remu = ((d_insn_opcode == OpRegReg) && (decode_state[63:57] == 7'd1)) && (decode_state[46:44] == 3'b111);
	wire d_insn_ecall = (d_insn_opcode == OpEnviron) && (decode_state[63:39] == 25'd0);
	wire d_insn_fence = d_insn_opcode == OpMiscMem;
	reg [7:0] d_ic;
	localparam [7:0] IClui = 8'd0;
	localparam [7:0] ICauipc = 8'd1;
	localparam [7:0] ICjal = 8'd2;
	localparam [7:0] ICjalr = 8'd3;
	localparam [7:0] ICbeq = 8'd4;
	localparam [7:0] ICbne = 8'd5;
	localparam [7:0] ICblt = 8'd6;
	localparam [7:0] ICbge = 8'd7;
	localparam [7:0] ICbltu = 8'd8;
	localparam [7:0] ICbgeu = 8'd9;
	localparam [7:0] IClb = 8'd10;
	localparam [7:0] IClh = 8'd11;
	localparam [7:0] IClw = 8'd12;
	localparam [7:0] IClbu = 8'd13;
	localparam [7:0] IClhu = 8'd14;
	localparam [7:0] ICsb = 8'd15;
	localparam [7:0] ICsh = 8'd16;
	localparam [7:0] ICsw = 8'd17;
	localparam [7:0] ICaddi = 8'd18;
	localparam [7:0] ICslti = 8'd19;
	localparam [7:0] ICsltiu = 8'd20;
	localparam [7:0] ICxori = 8'd21;
	localparam [7:0] ICori = 8'd22;
	localparam [7:0] ICandi = 8'd23;
	localparam [7:0] ICslli = 8'd24;
	localparam [7:0] ICsrli = 8'd25;
	localparam [7:0] ICsrai = 8'd26;
	localparam [7:0] ICadd = 8'd27;
	localparam [7:0] ICsub = 8'd28;
	localparam [7:0] ICsll = 8'd29;
	localparam [7:0] ICslt = 8'd30;
	localparam [7:0] ICsltu = 8'd31;
	localparam [7:0] ICxor = 8'd32;
	localparam [7:0] ICsrl = 8'd33;
	localparam [7:0] ICsra = 8'd34;
	localparam [7:0] ICor = 8'd35;
	localparam [7:0] ICand = 8'd36;
	localparam [7:0] ICmul = 8'd37;
	localparam [7:0] ICmulh = 8'd38;
	localparam [7:0] ICmulhsu = 8'd39;
	localparam [7:0] ICmulhu = 8'd40;
	localparam [7:0] ICdiv = 8'd41;
	localparam [7:0] ICdivu = 8'd42;
	localparam [7:0] ICrem = 8'd43;
	localparam [7:0] ICremu = 8'd44;
	localparam [7:0] ICecall = 8'd45;
	localparam [7:0] ICfence = 8'd46;
	localparam [7:0] ICIllegal = 8'd100;
	always @(*) begin
		if (_sv2v_0)
			;
		if (d_insn_lui)
			d_ic = IClui;
		else if (d_insn_auipc)
			d_ic = ICauipc;
		else if (d_insn_jal)
			d_ic = ICjal;
		else if (d_insn_jalr)
			d_ic = ICjalr;
		else if (d_insn_beq)
			d_ic = ICbeq;
		else if (d_insn_bne)
			d_ic = ICbne;
		else if (d_insn_blt)
			d_ic = ICblt;
		else if (d_insn_bge)
			d_ic = ICbge;
		else if (d_insn_bltu)
			d_ic = ICbltu;
		else if (d_insn_bgeu)
			d_ic = ICbgeu;
		else if (d_insn_lb)
			d_ic = IClb;
		else if (d_insn_lh)
			d_ic = IClh;
		else if (d_insn_lw)
			d_ic = IClw;
		else if (d_insn_lbu)
			d_ic = IClbu;
		else if (d_insn_lhu)
			d_ic = IClhu;
		else if (d_insn_sb)
			d_ic = ICsb;
		else if (d_insn_sh)
			d_ic = ICsh;
		else if (d_insn_sw)
			d_ic = ICsw;
		else if (d_insn_addi)
			d_ic = ICaddi;
		else if (d_insn_slti)
			d_ic = ICslti;
		else if (d_insn_sltiu)
			d_ic = ICsltiu;
		else if (d_insn_xori)
			d_ic = ICxori;
		else if (d_insn_ori)
			d_ic = ICori;
		else if (d_insn_andi)
			d_ic = ICandi;
		else if (d_insn_slli)
			d_ic = ICslli;
		else if (d_insn_srli)
			d_ic = ICsrli;
		else if (d_insn_srai)
			d_ic = ICsrai;
		else if (d_insn_add)
			d_ic = ICadd;
		else if (d_insn_sub)
			d_ic = ICsub;
		else if (d_insn_sll)
			d_ic = ICsll;
		else if (d_insn_slt)
			d_ic = ICslt;
		else if (d_insn_sltu)
			d_ic = ICsltu;
		else if (d_insn_xor)
			d_ic = ICxor;
		else if (d_insn_srl)
			d_ic = ICsrl;
		else if (d_insn_sra)
			d_ic = ICsra;
		else if (d_insn_or)
			d_ic = ICor;
		else if (d_insn_and)
			d_ic = ICand;
		else if (d_insn_mul)
			d_ic = ICmul;
		else if (d_insn_mulh)
			d_ic = ICmulh;
		else if (d_insn_mulhsu)
			d_ic = ICmulhsu;
		else if (d_insn_mulhu)
			d_ic = ICmulhu;
		else if (d_insn_div)
			d_ic = ICdiv;
		else if (d_insn_divu)
			d_ic = ICdivu;
		else if (d_insn_rem)
			d_ic = ICrem;
		else if (d_insn_remu)
			d_ic = ICremu;
		else if (d_insn_ecall)
			d_ic = ICecall;
		else if (d_insn_fence)
			d_ic = ICfence;
		else
			d_ic = ICIllegal;
	end
	reg m_store;
	reg x_store;
	always @(*) begin
		if (_sv2v_0)
			;
		if ((d_ic == ICfence) && (x_store || m_store))
			f_fence = 1;
		else
			f_fence = 0;
	end
	reg [128:0] w_state;
	wire wd_bypass_rs1;
	wire wd_bypass_rs2;
	function automatic [31:0] sv2v_cast_32;
		input reg [31:0] inp;
		sv2v_cast_32 = inp;
	endfunction
	function automatic [19:0] sv2v_cast_20;
		input reg [19:0] inp;
		sv2v_cast_20 = inp;
	endfunction
	function automatic [7:0] sv2v_cast_8;
		input reg [7:0] inp;
		sv2v_cast_8 = inp;
	endfunction
	always @(posedge clk)
		if (rst || f_fence)
			x_state <= {64'h0000000000000000, (rst ? 32'd1 : 32'd64), 220'h0000000000000000000000000000000000000000000000000000000};
		else if (f_div_stall_next || f_load_stall_next)
			x_state <= x_state;
		else
			x_state <= {sv2v_cast_32((x_branch ? 0 : decode_state[95-:32])), sv2v_cast_32((x_branch ? 0 : decode_state[63-:32])), sv2v_cast_32((x_branch ? 32'd4 : decode_state[31-:32])), sv2v_cast_20((x_branch ? 0 : d_imm_u)), sv2v_cast_32((x_branch ? 0 : d_imm_i_sext)), sv2v_cast_32((x_branch ? 0 : d_imm_s_sext)), sv2v_cast_32((x_branch ? 0 : d_imm_b_sext)), sv2v_cast_32((x_branch ? 0 : d_imm_j_sext)), sv2v_cast_32((wd_bypass_rs1 ? w_state[31-:32] : 0)), sv2v_cast_32((wd_bypass_rs2 ? w_state[31-:32] : 0)), sv2v_cast_8((x_branch ? 0 : d_ic))};
	wire [255:0] x_disasm;
	Disasm #(.PREFIX("X")) disasm_2execute(
		.insn(x_state[283-:32]),
		.disasm(x_disasm)
	);
	wire x_con_insn_div;
	wire d_con_insn_div;
	reg [2:0] x_cycle_count;
	assign x_con_insn_div = (((x_state[7-:8] == ICdiv) | (x_state[7-:8] == ICdivu)) | (x_state[7-:8] == ICrem)) | (x_state[7-:8] == ICremu);
	assign d_con_insn_div = (((d_ic == ICdiv) | (d_ic == ICdivu)) | (d_ic == ICrem)) | (d_ic == ICremu);
	wire [4:0] x_rd;
	assign f_div_stall_next = (((x_con_insn_div && d_con_insn_div) && (d_insn_rs1 != x_rd)) && (d_insn_rs2 != x_rd) ? 0 : x_con_insn_div && (x_cycle_count != 3'd7));
	always @(posedge clk)
		if (rst)
			x_cycle_count <= 3'd0;
		else if (f_div_stall_next) begin
			if (x_cycle_count == 3'd7)
				x_cycle_count <= 3'd0;
			else
				x_cycle_count <= x_cycle_count + 3'd1;
		end
		else
			x_cycle_count <= 3'd0;
	reg [31:0] dividend;
	reg [31:0] divisor;
	wire [31:0] quo;
	wire [31:0] rem;
	DividerUnsignedPipelined divider(
		.clk(clk),
		.rst(rst),
		.stall(1'b0),
		.i_dividend(dividend),
		.i_divisor(divisor),
		.o_remainder(rem),
		.o_quotient(quo)
	);
	reg [63:0] m1;
	reg [63:0] m2;
	reg [63:0] m3;
	reg [31:0] a;
	reg [31:0] b;
	reg cin;
	wire [31:0] sum;
	cla cla_mod(
		.a(a),
		.b(b),
		.cin(cin),
		.sum(sum)
	);
	reg [31:0] x_memory_address;
	reg [31:0] x_data_dmem;
	reg [31:0] x_rd_data;
	reg [31:0] x_d_1;
	reg [31:0] x_d_2;
	wire [11:0] x_imm_i;
	assign x_imm_i = x_state[283:272];
	reg [199:0] m_state;
	wire mx_bypass_rs1;
	wire mx_bypass_rs2;
	wire wx_bypass_rs1;
	wire wx_bypass_rs2;
	always @(*) begin
		if (_sv2v_0)
			;
		x_d_1 = (x_state[71-:32] == 0 ? x_rs1_data : x_state[71-:32]);
		x_d_2 = (x_state[39-:32] == 0 ? x_rs2_data : x_state[39-:32]);
		if (wx_bypass_rs1)
			x_d_1 = w_state[31-:32];
		if (wx_bypass_rs2)
			x_d_2 = w_state[31-:32];
		if (mx_bypass_rs1)
			x_d_1 = m_state[103-:32];
		if (mx_bypass_rs2)
			x_d_2 = m_state[103-:32];
	end
	always @(*) begin
		if (_sv2v_0)
			;
		if (((x_state[7-:8] == ICsw) || (x_state[7-:8] == ICsb)) || (x_state[7-:8] == ICsh))
			x_store = 1;
		else
			x_store = 0;
	end
	always @(*) begin
		if (_sv2v_0)
			;
		if (((m_state[7-:8] == ICsw) || (m_state[7-:8] == ICsb)) || (m_state[7-:8] == ICsh))
			m_store = 1;
		else
			m_store = 0;
	end
	reg m_load;
	always @(*) begin
		if (_sv2v_0)
			;
		if (((((m_state[7-:8] == IClw) || (m_state[7-:8] == IClb)) || (m_state[7-:8] == IClh)) || (m_state[7-:8] == IClhu)) || (m_state[7-:8] == IClbu))
			m_load = 1;
		else
			m_load = 0;
	end
	wire [4:0] m_rd;
	wire x_req_rs1;
	wire x_req_rs2;
	wire [4:0] x_rs1;
	wire [4:0] x_rs2;
	always @(*) begin
		if (_sv2v_0)
			;
		if (m_load) begin
			if (x_store) begin
				if ((m_rd == x_rs1) && x_req_rs1)
					f_load_stall_next = (f_load_stall_curr == 1'b0 ? 1'b1 : 1'b0);
				else
					f_load_stall_next = 1'b0;
			end
			else if ((m_rd == x_rs1) && x_req_rs1)
				f_load_stall_next = (f_load_stall_curr == 1'b0 ? 1'b1 : 1'b0);
			else if ((m_rd == x_rs2) && x_req_rs2)
				f_load_stall_next = (f_load_stall_curr == 1'b0 ? 1'b1 : 1'b0);
			else
				f_load_stall_next = 1'b0;
		end
		else
			f_load_stall_next = 1'b0;
		f_pc_next = f_pc_current + 4;
		x_rd_data = 32'd0;
		x_branch = 1'b0;
		a = 32'b00000000000000000000000000000000;
		b = 32'b00000000000000000000000000000000;
		cin = 0;
		x_memory_address = 0;
		x_data_dmem = 0;
		m1 = 64'd0;
		m2 = 64'd0;
		m3 = 64'd0;
		dividend = 32'd0;
		divisor = 32'd0;
		case (x_state[7-:8])
			IClui: x_rd_data = {x_state[219-:20], 12'd0};
			ICauipc: x_rd_data = x_state[315-:32] + {x_state[219-:20], 12'd0};
			ICaddi: begin
				a = x_d_1;
				b = x_state[199-:32];
				x_rd_data = sum;
			end
			ICslti: x_rd_data = ($signed(x_d_1) < $signed(x_state[199-:32]) ? 32'h00000001 : 32'h00000000);
			ICsltiu: x_rd_data = ($unsigned(x_d_1) < $unsigned(x_state[199-:32]) ? 32'h00000001 : 32'h00000000);
			ICxori: x_rd_data = x_d_1 ^ x_state[199-:32];
			ICandi: x_rd_data = x_d_1 & x_state[199-:32];
			ICori: x_rd_data = x_d_1 | x_state[199-:32];
			ICslli: x_rd_data = x_d_1 << x_state[172:168];
			ICsrli: x_rd_data = x_d_1 >> x_state[172:168];
			ICsrai: x_rd_data = $signed(x_d_1) >>> x_state[172:168];
			ICadd: begin
				a = x_d_1;
				b = x_d_2;
				x_rd_data = sum;
			end
			ICsub: begin
				a = x_d_1;
				b = ~x_d_2;
				x_rd_data = sum;
				cin = 1'b1;
			end
			ICsll: x_rd_data = x_d_1 << x_d_2[4:0];
			ICslt: x_rd_data = ($signed(x_d_1) < $signed(x_d_2) ? 32'h00000001 : 32'h00000000);
			ICsltu: x_rd_data = ($unsigned(x_d_1) < $unsigned(x_d_2) ? 32'h00000001 : 32'h00000000);
			ICxor: x_rd_data = x_d_1 ^ x_d_2;
			ICsrl: x_rd_data = x_d_1 >> x_d_2[4:0];
			ICsra: x_rd_data = $signed(x_d_1) >>> x_d_2[4:0];
			ICor: x_rd_data = x_d_1 | x_d_2;
			ICand: x_rd_data = x_d_1 & x_d_2;
			ICbeq:
				if (x_d_1 == x_d_2) begin
					f_pc_next = x_state[315-:32] + x_state[135-:32];
					x_branch = 1;
				end
			ICbne:
				if (x_d_1 != x_d_2) begin
					f_pc_next = x_state[315-:32] + x_state[135-:32];
					x_branch = 1;
				end
			ICblt:
				if ($signed(x_d_1) < $signed(x_d_2)) begin
					f_pc_next = x_state[315-:32] + x_state[135-:32];
					x_branch = 1;
				end
			ICbge:
				if ($signed(x_d_1) >= $signed(x_d_2)) begin
					f_pc_next = x_state[315-:32] + x_state[135-:32];
					x_branch = 1;
				end
			ICbltu:
				if (x_d_1 < $unsigned(x_d_2)) begin
					f_pc_next = x_state[315-:32] + x_state[135-:32];
					x_branch = 1;
				end
			ICbgeu:
				if (x_d_1 >= $unsigned(x_d_2)) begin
					f_pc_next = x_state[315-:32] + x_state[135-:32];
					x_branch = 1;
				end
			ICjal: begin
				x_rd_data = x_state[315-:32] + 32'd4;
				f_pc_next = x_state[315-:32] + x_state[103-:32];
				x_branch = 1;
			end
			ICjalr: begin
				x_rd_data = x_state[315-:32] + 32'd4;
				f_pc_next = (x_d_1 + x_state[199-:32]) & ~1;
				x_branch = 1;
			end
			ICmul: x_rd_data = x_d_1 * x_d_2;
			ICmulh: begin
				m1 = {{32 {x_d_1[31]}}, x_d_1} * {{32 {x_d_2[31]}}, x_d_2};
				x_rd_data = m1[63:32];
			end
			ICmulhsu: begin
				m2 = {{32 {x_d_1[31]}}, x_d_1} * {32'b00000000000000000000000000000000, x_d_2};
				x_rd_data = m2[63:32];
			end
			ICmulhu: begin
				m3 = $unsigned(x_d_1) * $unsigned(x_d_2);
				x_rd_data = m3[63:32];
			end
			IClb: begin
				a = x_d_1;
				b = x_state[199-:32];
				x_memory_address = sum;
			end
			IClh: begin
				a = x_d_1;
				b = x_state[199-:32];
				x_memory_address = sum;
			end
			IClw: begin
				a = x_d_1;
				b = x_state[199-:32];
				x_memory_address = sum;
			end
			IClbu: begin
				a = x_d_1;
				b = x_state[199-:32];
				x_memory_address = sum;
			end
			IClhu: begin
				a = x_d_1;
				b = x_state[199-:32];
				x_memory_address = sum;
			end
			ICsw: begin
				a = x_d_1;
				b = x_state[167-:32];
				x_memory_address = sum;
				x_data_dmem = x_d_2;
			end
			ICsh: begin
				a = x_d_1;
				b = x_state[167-:32];
				x_memory_address = sum;
				x_data_dmem = x_d_2;
			end
			ICsb: begin
				a = x_d_1;
				b = x_state[167-:32];
				x_memory_address = sum;
				x_data_dmem = x_d_2;
			end
			ICdiv: begin
				if (x_d_1[31])
					dividend = ~x_d_1 + 1;
				else
					dividend = x_d_1;
				if (x_d_2[31])
					divisor = ~x_d_2 + 1;
				else
					divisor = x_d_2;
				if ((x_d_1[31] ~^ x_d_2[31]) || (x_d_2 == 32'd0))
					x_rd_data = quo;
				else
					x_rd_data = ~quo + 'd1;
			end
			ICrem: begin
				if (x_d_1[31])
					dividend = ~x_d_1 + 1;
				else
					dividend = x_d_1;
				if (x_d_2[31])
					divisor = ~x_d_2 + 1;
				else
					divisor = x_d_2;
				if (x_d_1[31])
					x_rd_data = ~rem + 1;
				else
					x_rd_data = rem;
			end
			ICdivu: begin
				dividend = x_d_1;
				divisor = $unsigned(x_d_2);
				x_rd_data = quo;
			end
			ICremu: begin
				divisor = $unsigned(x_d_2);
				dividend = x_d_1;
				x_rd_data = rem;
			end
			default:
				;
		endcase
	end
	always @(posedge clk)
		if (rst)
			m_state <= 200'h00000000000000000000000100000000000000000000000000;
		else if ((f_div_stall_curr || f_div_stall_next) || f_load_stall_next)
			m_state <= {64'h0000000000000000, (f_load_stall_next ? 32'd16 : 32'd32), 104'h00000000000000000000000000};
		else
			m_state <= {sv2v_cast_32(x_state[315-:32]), sv2v_cast_32(x_state[283-:32]), sv2v_cast_32(x_state[251-:32]), x_rd_data, x_memory_address, x_data_dmem, sv2v_cast_8(x_state[7-:8])};
	wire [255:0] m_disasm;
	Disasm #(.PREFIX("M")) disasm_2memory(
		.insn(m_state[167-:32]),
		.disasm(m_disasm)
	);
	wire m_r_mem = (((m_state[142:136] == OpLui) || (m_state[142:136] == OpRegReg)) || (m_state[142:136] == OpRegImm)) || (m_state[142:136] == OpJal);
	reg [31:0] m_rd_data;
	reg [31:0] m_data_to_dmem;
	reg [3:0] m_we_to_dmem;
	reg m_illegal_insn;
	wire wm_bypass_data;
	always @(*) begin
		if (_sv2v_0)
			;
		if (wm_bypass_data)
			m_data_to_dmem = w_state[31-:32];
		else
			m_data_to_dmem = m_state[39-:32];
	end
	assign addr_to_dmem = {m_state[71:42], 2'b00};
	always @(*) begin
		if (_sv2v_0)
			;
		m_we_to_dmem = 0;
		m_illegal_insn = 0;
		store_data_to_dmem = 32'd0;
		store_we_to_dmem = 4'b0000;
		case (m_state[7-:8])
			IClw: m_rd_data = load_data_from_dmem;
			IClh:
				case (m_state[41])
					default: m_illegal_insn = 1;
					1'b0: m_rd_data = {{16 {load_data_from_dmem[15]}}, load_data_from_dmem[15:0]};
					1'b1: m_rd_data = {{16 {load_data_from_dmem[31]}}, load_data_from_dmem[31:16]};
				endcase
			IClb:
				case (m_state[41:40])
					default: m_illegal_insn = 1;
					2'b00: m_rd_data = {{24 {load_data_from_dmem[7]}}, load_data_from_dmem[7:0]};
					2'b01: m_rd_data = {{24 {load_data_from_dmem[15]}}, load_data_from_dmem[15:8]};
					2'b10: m_rd_data = {{24 {load_data_from_dmem[23]}}, load_data_from_dmem[23:16]};
					2'b11: m_rd_data = {{24 {load_data_from_dmem[31]}}, load_data_from_dmem[31:24]};
				endcase
			IClhu:
				case (m_state[41])
					default: m_illegal_insn = 1;
					1'b0: m_rd_data = {16'b0000000000000000, load_data_from_dmem[15:0]};
					1'b1: m_rd_data = {16'b0000000000000000, load_data_from_dmem[31:16]};
				endcase
			IClbu:
				case (m_state[41:40])
					default: m_illegal_insn = 1;
					2'b00: m_rd_data = {24'b000000000000000000000000, load_data_from_dmem[7:0]};
					2'b01: m_rd_data = {24'b000000000000000000000000, load_data_from_dmem[15:8]};
					2'b10: m_rd_data = {24'b000000000000000000000000, load_data_from_dmem[23:16]};
					2'b11: m_rd_data = {24'b000000000000000000000000, load_data_from_dmem[31:24]};
				endcase
			ICsw: begin
				store_data_to_dmem = m_data_to_dmem[31:0];
				store_we_to_dmem = 4'b1111;
			end
			ICsh:
				case (m_state[41])
					default: m_illegal_insn = 1;
					1'b0: begin
						store_we_to_dmem = 4'b0011;
						store_data_to_dmem[15:0] = m_data_to_dmem[15:0];
					end
					1'b1: begin
						store_we_to_dmem = 4'b1100;
						store_data_to_dmem[31:16] = m_data_to_dmem[15:0];
					end
				endcase
			ICsb:
				case (m_state[41:40])
					default: m_illegal_insn = 1;
					2'b00: begin
						store_we_to_dmem = 4'b0001;
						store_data_to_dmem[7:0] = m_data_to_dmem[7:0];
					end
					2'b01: begin
						store_we_to_dmem = 4'b0010;
						store_data_to_dmem[15:8] = m_data_to_dmem[7:0];
					end
					2'b10: begin
						store_we_to_dmem = 4'b0100;
						store_data_to_dmem[23:16] = m_data_to_dmem[7:0];
					end
					2'b11: begin
						store_we_to_dmem = 4'b1000;
						store_data_to_dmem[31:24] = m_data_to_dmem[7:0];
					end
				endcase
			default: begin
				m_rd_data = m_state[103-:32];
				m_illegal_insn = 0;
				m_we_to_dmem = 0;
			end
		endcase
	end
	always @(posedge clk)
		if (rst)
			w_state <= 129'h000000000000000000000000200000000;
		else if (f_div_stall_curr)
			w_state <= {sv2v_cast_32(x_state[315-:32]), sv2v_cast_32(x_state[283-:32]), sv2v_cast_32(x_state[251-:32]), x_state[7-:8] == ICIllegal, x_rd_data};
		else
			w_state <= {sv2v_cast_32(m_state[199-:32]), sv2v_cast_32(m_state[167-:32]), sv2v_cast_32(m_state[135-:32]), m_illegal_insn, m_rd_data};
	wire [255:0] w_disasm;
	Disasm #(.PREFIX("W")) disasm_2writeback(
		.insn(w_state[96-:32]),
		.disasm(w_disasm)
	);
	assign rd_to_reg = w_state[76:72];
	wire regfile_we;
	assign regfile_we = (((((((w_state[96-:32] == 32'h00000000) || (w_state[71:65] == 7'h63)) || (w_state[64-:32] == 32'd4)) || (w_state[71:65] == OpStore)) || (w_state[71:65] == OpBranch)) || (w_state[71:65] == OpMiscMem)) || (w_state[71:65] == OpEnviron) ? 1'b0 : 1'b1);
	always @(*) begin
		if (_sv2v_0)
			;
		if (regfile_we == 1) begin
			data_to_reg = w_state[31-:32];
			we_to_reg = 1;
		end
		else begin
			data_to_reg = 32'd0;
			we_to_reg = 0;
		end
	end
	assign halt = (m_state[142:136] == 7'h73) & (m_state[167:143] == 'b0);
	wire m_req_rd;
	wire m_req_rs2;
	wire w_req_rd;
	wire d_req_rs1;
	wire d_req_rs2;
	assign m_req_rd = ((((((m_state[142:136] == OpLui) || (m_state[142:136] == OpAuipc)) || (m_state[142:136] == OpRegImm)) || (m_state[142:136] == OpRegReg)) || (m_state[142:136] == OpLoad)) || (m_state[142:136] == OpJal)) || (m_state[142:136] == OpJalr);
	assign m_req_rs2 = m_state[142:136] == OpStore;
	assign w_req_rd = ((((((w_state[71:65] == OpLui) || (w_state[71:65] == OpAuipc)) || (w_state[71:65] == OpRegImm)) || (w_state[71:65] == OpRegReg)) || (w_state[71:65] == OpLoad)) || (w_state[71:65] == OpJal)) || (w_state[71:65] == OpJalr);
	assign x_req_rs1 = (((((x_state[258:252] == OpRegReg) || (x_state[258:252] == OpRegImm)) || (x_state[258:252] == OpBranch)) || (x_state[258:252] == OpStore)) || (x_state[258:252] == OpLoad)) || (x_state[258:252] == OpJalr);
	assign x_req_rs2 = ((x_state[258:252] == OpRegReg) || (x_state[258:252] == OpStore)) || (x_state[258:252] == OpBranch);
	assign d_req_rs1 = (((((d_insn_opcode == OpRegImm) || (d_insn_opcode == OpRegReg)) || (d_insn_opcode == OpBranch)) || (d_insn_opcode == OpLoad)) || (d_insn_opcode == OpStore)) || (d_insn_opcode == OpJalr);
	assign d_req_rs2 = ((d_insn_opcode == OpRegReg) || (d_insn_opcode == OpStore)) || (d_insn_opcode == OpBranch);
	reg illegal_insn = ((((x_state[7-:8] == ICIllegal) || (d_ic == ICIllegal)) || (m_state[7-:8] == ICIllegal)) || m_illegal_insn) || w_state[32];
	wire [4:0] m_rs2;
	assign m_rd = m_state[147:143];
	assign m_rs2 = m_state[160:156];
	assign x_rs1 = x_state[271:267];
	assign x_rs2 = x_state[276:272];
	assign x_rd = x_state[263:259];
	assign mx_bypass_rs1 = ((((x_rs1 == m_rd) && (illegal_insn == 1'b0)) && (m_rd != 5'd0)) && m_req_rd) && x_req_rs1;
	assign mx_bypass_rs2 = ((((x_rs2 == m_rd) && (illegal_insn == 1'b0)) && (m_rd != 5'd0)) && m_req_rd) && x_req_rs2;
	assign wx_bypass_rs1 = ((((x_rs1 == rd_to_reg) && (illegal_insn == 1'b0)) && (rd_to_reg != 5'd0)) && w_req_rd) && x_req_rs1;
	assign wx_bypass_rs2 = ((((x_rs2 == rd_to_reg) && (illegal_insn == 1'b0)) && (rd_to_reg != 5'd0)) && w_req_rd) && x_req_rs2;
	assign wd_bypass_rs2 = (((d_insn_rs2 == rd_to_reg) && (rd_to_reg != 5'd0)) && w_req_rd) && d_req_rs2;
	assign wd_bypass_rs1 = (((d_insn_rs1 == rd_to_reg) && (rd_to_reg != 5'd0)) && w_req_rd) && d_req_rs1;
	assign wm_bypass_data = ((rd_to_reg == m_rs2) && w_req_rd) && m_req_rs2;
	assign trace_writeback_pc = w_state[128-:32];
	assign trace_writeback_insn = w_state[96-:32];
	assign trace_writeback_cycle_status = w_state[64-:32];
	initial _sv2v_0 = 0;
endmodule
module MemorySingleCycle (
	rst,
	clk,
	pc_to_imem,
	insn_from_imem,
	addr_to_dmem,
	load_data_from_dmem,
	store_data_to_dmem,
	store_we_to_dmem
);
	reg _sv2v_0;
	parameter signed [31:0] NUM_WORDS = 512;
	input wire rst;
	input wire clk;
	input wire [31:0] pc_to_imem;
	output reg [31:0] insn_from_imem;
	input wire [31:0] addr_to_dmem;
	output reg [31:0] load_data_from_dmem;
	input wire [31:0] store_data_to_dmem;
	input wire [3:0] store_we_to_dmem;
	reg [31:0] mem_array [0:NUM_WORDS - 1];
	initial $readmemh("mem_initial_contents.hex", mem_array);
	always @(*)
		if (_sv2v_0)
			;
	localparam signed [31:0] AddrMsb = $clog2(NUM_WORDS) + 1;
	localparam signed [31:0] AddrLsb = 2;
	always @(negedge clk)
		if (rst)
			;
		else
			insn_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
	always @(negedge clk)
		if (rst)
			;
		else begin
			if (store_we_to_dmem[0])
				mem_array[addr_to_dmem[AddrMsb:AddrLsb]][7:0] <= store_data_to_dmem[7:0];
			if (store_we_to_dmem[1])
				mem_array[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <= store_data_to_dmem[15:8];
			if (store_we_to_dmem[2])
				mem_array[addr_to_dmem[AddrMsb:AddrLsb]][23:16] <= store_data_to_dmem[23:16];
			if (store_we_to_dmem[3])
				mem_array[addr_to_dmem[AddrMsb:AddrLsb]][31:24] <= store_data_to_dmem[31:24];
			load_data_from_dmem <= mem_array[{addr_to_dmem[AddrMsb:AddrLsb]}];
		end
	initial _sv2v_0 = 0;
endmodule
module SystemResourceCheck (
	external_clk_25MHz,
	btn,
	led
);
	input wire external_clk_25MHz;
	input wire [6:0] btn;
	output wire [7:0] led;
	wire clk_proc;
	wire clk_locked;
	MyClockGen clock_gen(
		.input_clk_25MHz(external_clk_25MHz),
		.clk_proc(clk_proc),
		.locked(clk_locked)
	);
	wire [31:0] pc_to_imem;
	wire [31:0] insn_from_imem;
	wire [31:0] mem_data_addr;
	wire [31:0] mem_data_loaded_value;
	wire [31:0] mem_data_to_write;
	wire [3:0] mem_data_we;
	wire [31:0] trace_writeback_pc;
	wire [31:0] trace_writeback_insn;
	wire [31:0] trace_writeback_cycle_status;
	MemorySingleCycle #(.NUM_WORDS(128)) memory(
		.rst(!clk_locked),
		.clk(clk_proc),
		.pc_to_imem(pc_to_imem),
		.insn_from_imem(insn_from_imem),
		.addr_to_dmem(mem_data_addr),
		.load_data_from_dmem(mem_data_loaded_value),
		.store_data_to_dmem(mem_data_to_write),
		.store_we_to_dmem(mem_data_we)
	);
	DatapathPipelined datapath(
		.clk(clk_proc),
		.rst(!clk_locked),
		.pc_to_imem(pc_to_imem),
		.insn_from_imem(insn_from_imem),
		.addr_to_dmem(mem_data_addr),
		.store_data_to_dmem(mem_data_to_write),
		.store_we_to_dmem(mem_data_we),
		.load_data_from_dmem(mem_data_loaded_value),
		.halt(led[0]),
		.trace_writeback_pc(trace_writeback_pc),
		.trace_writeback_insn(trace_writeback_insn),
		.trace_writeback_cycle_status(trace_writeback_cycle_status)
	);
endmodule
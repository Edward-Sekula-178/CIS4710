module MyClockGen (
	input_clk_25MHz,
	clk_proc,
	clk_mem,
	locked
);
	input input_clk_25MHz;
	output wire clk_proc;
	output wire clk_mem;
	output wire locked;
	wire clkfb;
	(* FREQUENCY_PIN_CLKI = "25" *) (* FREQUENCY_PIN_CLKOP = "3.57143" *) (* FREQUENCY_PIN_CLKOS = "3.80952" *) (* ICP_CURRENT = "12" *) (* LPF_RESISTOR = "8" *) (* MFG_ENABLE_FILTEROPAMP = "1" *) (* MFG_GMCREF_SEL = "2" *) EHXPLLL #(
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
		.CLKOP_DIV(128),
		.CLKOP_CPHASE(63),
		.CLKOP_FPHASE(0),
		.CLKOS_ENABLE("ENABLED"),
		.CLKOS_DIV(120),
		.CLKOS_CPHASE(93),
		.CLKOS_FPHASE(0),
		.FEEDBK_PATH("INT_OP"),
		.CLKFB_DIV(1)
	) pll_i(
		.RST(1'b0),
		.STDBY(1'b0),
		.CLKI(input_clk_25MHz),
		.CLKOP(clk_proc),
		.CLKOS(clk_mem),
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
module DatapathMultiCycle (
	clk,
	rst,
	halt,
	pc_to_imem,
	insn_from_imem,
	addr_to_dmem,
	load_data_from_dmem,
	store_data_to_dmem,
	store_we_to_dmem
);
	reg _sv2v_0;
	input wire clk;
	input wire rst;
	output reg halt;
	output wire [31:0] pc_to_imem;
	input wire [31:0] insn_from_imem;
	output reg [31:0] addr_to_dmem;
	input wire [31:0] load_data_from_dmem;
	output reg [31:0] store_data_to_dmem;
	output reg [3:0] store_we_to_dmem;
	wire [6:0] insn_funct7;
	wire [4:0] insn_rs2;
	wire [4:0] insn_rs1;
	wire [2:0] insn_funct3;
	wire [4:0] insn_rd;
	wire [6:0] insn_opcode;
	assign {insn_funct7, insn_rs2, insn_rs1, insn_funct3, insn_rd, insn_opcode} = insn_from_imem;
	wire [19:0] imm_u;
	assign imm_u = insn_from_imem[31:12];
	wire [11:0] imm_i;
	assign imm_i = insn_from_imem[31:20];
	wire [4:0] imm_shamt = insn_from_imem[24:20];
	wire [11:0] imm_s;
	assign imm_s[11:5] = insn_funct7;
	assign imm_s[4:0] = insn_rd;
	wire [12:0] imm_b;
	assign {imm_b[12], imm_b[10:5]} = insn_funct7;
	assign {imm_b[4:1], imm_b[11]} = insn_rd;
	assign imm_b[0] = 1'b0;
	wire [20:0] imm_j;
	assign {imm_j[20], imm_j[10:1], imm_j[11], imm_j[19:12], imm_j[0]} = {insn_from_imem[31:12], 1'b0};
	wire [31:0] imm_i_sext = {{20 {imm_i[11]}}, imm_i[11:0]};
	wire [31:0] imm_s_sext = {{20 {imm_s[11]}}, imm_s[11:0]};
	wire [31:0] imm_b_sext = {{19 {imm_b[12]}}, imm_b[12:0]};
	wire [31:0] imm_j_sext = {{11 {imm_j[20]}}, imm_j[20:0]};
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
	wire insn_lui = insn_opcode == OpLui;
	wire insn_auipc = insn_opcode == OpAuipc;
	wire insn_jal = insn_opcode == OpJal;
	wire insn_jalr = insn_opcode == OpJalr;
	wire insn_beq = (insn_opcode == OpBranch) && (insn_from_imem[14:12] == 3'b000);
	wire insn_bne = (insn_opcode == OpBranch) && (insn_from_imem[14:12] == 3'b001);
	wire insn_blt = (insn_opcode == OpBranch) && (insn_from_imem[14:12] == 3'b100);
	wire insn_bge = (insn_opcode == OpBranch) && (insn_from_imem[14:12] == 3'b101);
	wire insn_bltu = (insn_opcode == OpBranch) && (insn_from_imem[14:12] == 3'b110);
	wire insn_bgeu = (insn_opcode == OpBranch) && (insn_from_imem[14:12] == 3'b111);
	wire insn_lb = (insn_opcode == OpLoad) && (insn_from_imem[14:12] == 3'b000);
	wire insn_lh = (insn_opcode == OpLoad) && (insn_from_imem[14:12] == 3'b001);
	wire insn_lw = (insn_opcode == OpLoad) && (insn_from_imem[14:12] == 3'b010);
	wire insn_lbu = (insn_opcode == OpLoad) && (insn_from_imem[14:12] == 3'b100);
	wire insn_lhu = (insn_opcode == OpLoad) && (insn_from_imem[14:12] == 3'b101);
	wire insn_sb = (insn_opcode == OpStore) && (insn_from_imem[14:12] == 3'b000);
	wire insn_sh = (insn_opcode == OpStore) && (insn_from_imem[14:12] == 3'b001);
	wire insn_sw = (insn_opcode == OpStore) && (insn_from_imem[14:12] == 3'b010);
	wire insn_addi = (insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b000);
	wire insn_slti = (insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b010);
	wire insn_sltiu = (insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b011);
	wire insn_xori = (insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b100);
	wire insn_ori = (insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b110);
	wire insn_andi = (insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b111);
	wire insn_slli = ((insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b001)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_srli = ((insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b101)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_srai = ((insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b101)) && (insn_from_imem[31:25] == 7'b0100000);
	wire insn_add = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b000)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_sub = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b000)) && (insn_from_imem[31:25] == 7'b0100000);
	wire insn_sll = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b001)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_slt = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b010)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_sltu = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b011)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_xor = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b100)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_srl = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b101)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_sra = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b101)) && (insn_from_imem[31:25] == 7'b0100000);
	wire insn_or = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b110)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_and = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b111)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_mul = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b000);
	wire insn_mulh = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b001);
	wire insn_mulhsu = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b010);
	wire insn_mulhu = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b011);
	wire insn_div = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b100);
	wire insn_divu = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b101);
	wire insn_rem = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b110);
	wire insn_remu = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b111);
	wire insn_ecall = (insn_opcode == OpEnviron) && (insn_from_imem[31:7] == 25'd0);
	wire insn_fence = insn_opcode == OpMiscMem;
	reg [31:0] pcNext;
	reg [31:0] pcCurrent;
	wire insn_stall;
	wire div_stall;
	reg [2:0] cycle_count;
	assign insn_stall = ((insn_div | insn_divu) | insn_rem) | insn_remu;
	assign div_stall = insn_stall && (cycle_count != 3'd7);
	always @(posedge clk)
		if (rst)
			cycle_count <= 3'd0;
		else if (insn_stall) begin
			if (cycle_count == 3'd7)
				cycle_count <= 3'd0;
			else
				cycle_count <= cycle_count + 3'd1;
		end
		else
			cycle_count <= 3'd0;
	always @(posedge clk)
		if (rst)
			pcCurrent <= 32'b00000000000000000000000000000000;
		else if (!div_stall)
			pcCurrent <= pcNext;
	assign pc_to_imem = pcCurrent;
	reg [31:0] cycles_current;
	reg [31:0] num_insns_current;
	always @(posedge clk)
		if (rst) begin
			cycles_current <= 0;
			num_insns_current <= 0;
		end
		else begin
			cycles_current <= cycles_current + 1;
			if (!rst)
				num_insns_current <= num_insns_current + 1;
		end
	wire [31:0] rs1_data;
	wire [31:0] rs2_data;
	reg [31:0] rd_data;
	reg we;
	RegFile rf(
		.clk(clk),
		.rst(rst),
		.we(we),
		.rd(insn_rd),
		.rd_data(rd_data),
		.rs1(insn_rs1),
		.rs2(insn_rs2),
		.rs1_data(rs1_data),
		.rs2_data(rs2_data)
	);
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
	reg [31:0] address;
	reg illegal_insn;
	reg [63:0] m1;
	reg [63:0] m2;
	reg [63:0] m3;
	always @(*) begin
		if (_sv2v_0)
			;
		pcNext = pcCurrent + 32'd4;
		rd_data = 32'd0;
		we = 1'b0;
		illegal_insn = 1'b0;
		halt = 1'b0;
		a = 32'b00000000000000000000000000000000;
		b = 32'b00000000000000000000000000000000;
		cin = 1'b0;
		dividend = 32'd0;
		divisor = 32'd0;
		address = 32'd0;
		store_data_to_dmem = 32'd0;
		store_we_to_dmem = 4'b0000;
		addr_to_dmem = 32'd0;
		m1 = 64'd0;
		m2 = 64'd0;
		m3 = 64'd0;
		case (insn_opcode)
			OpLui: begin
				rd_data = {imm_u[19:0], 12'h000};
				we = 1'b1;
			end
			OpAuipc: begin
				we = 1'b1;
				rd_data = pcCurrent + {imm_u[19:0], 12'h000};
			end
			OpRegImm: begin
				we = 1'b1;
				if (insn_addi) begin
					a = rs1_data;
					b = imm_i_sext;
					rd_data = sum;
				end
				if (insn_slti)
					rd_data = ($signed(rs1_data) < $signed(imm_i_sext) ? 32'd1 : 32'd0);
				if (insn_sltiu)
					rd_data = ($unsigned(rs1_data) < $unsigned(imm_i_sext) ? 32'd1 : 32'd0);
				if (insn_xori)
					rd_data = rs1_data ^ imm_i_sext;
				if (insn_ori)
					rd_data = rs1_data | imm_i_sext;
				if (insn_andi)
					rd_data = rs1_data & imm_i_sext;
				if (insn_slli)
					rd_data = rs1_data << imm_i[4:0];
				if (insn_srli)
					rd_data = rs1_data >> imm_i[4:0];
				if (insn_srai)
					rd_data = $signed(rs1_data) >>> imm_i[4:0];
			end
			OpRegReg: begin
				we = 1'b1;
				if (insn_add) begin
					a = rs1_data;
					b = rs2_data;
					rd_data = sum;
				end
				if (insn_sub) begin
					a = rs1_data;
					b = ~rs2_data;
					cin = 1'b1;
					rd_data = sum;
				end
				if (insn_sll)
					rd_data = rs1_data << rs2_data[4:0];
				if (insn_slt)
					rd_data = ($signed(rs1_data) < $signed(rs2_data) ? 32'b00000000000000000000000000000001 : 32'b00000000000000000000000000000000);
				if (insn_sltu)
					rd_data = ($unsigned(rs1_data) < $unsigned(rs2_data) ? 32'b00000000000000000000000000000001 : 32'b00000000000000000000000000000000);
				if (insn_xor)
					rd_data = rs1_data ^ rs2_data;
				if (insn_sra)
					rd_data = $signed(rs1_data) >>> rs2_data[4:0];
				if (insn_srl)
					rd_data = rs1_data >> rs2_data[4:0];
				if (insn_or)
					rd_data = rs1_data | rs2_data;
				if (insn_and)
					rd_data = rs1_data & rs2_data;
				if (insn_mul)
					rd_data = rs1_data * rs2_data;
				if (insn_mulh) begin
					m1 = {{32 {rs1_data[31]}}, rs1_data} * {{32 {rs2_data[31]}}, rs2_data};
					rd_data = m1[63:32];
				end
				if (insn_mulhsu) begin
					m2 = {{32 {rs1_data[31]}}, rs1_data} * {32'b00000000000000000000000000000000, rs2_data};
					rd_data = m2[63:32];
				end
				if (insn_mulhu) begin
					m3 = $unsigned(rs1_data) * $unsigned(rs2_data);
					rd_data = m3[63:32];
				end
				if (insn_div) begin
					if (rs1_data[31])
						dividend = ~rs1_data + 1;
					else
						dividend = rs1_data;
					if (rs2_data[31])
						divisor = ~rs2_data + 1;
					else
						divisor = rs2_data;
					if ((rs1_data[31] ~^ rs2_data[31]) || (rs2_data == 32'd0))
						rd_data = quo;
					else
						rd_data = ~quo + 'd1;
				end
				if (insn_rem) begin
					if (rs1_data[31])
						dividend = ~rs1_data + 1;
					else
						dividend = rs1_data;
					if (rs2_data[31])
						divisor = ~rs2_data + 1;
					else
						divisor = rs2_data;
					if (rs1_data[31])
						rd_data = ~rem + 1;
					else
						rd_data = rem;
				end
				if (insn_divu) begin
					dividend = rs1_data;
					divisor = $unsigned(rs2_data);
					rd_data = quo;
				end
				if (insn_remu) begin
					divisor = $unsigned(rs2_data);
					dividend = rs1_data;
					rd_data = rem;
				end
			end
			OpBranch: begin
				if (insn_beq) begin
					if (rs1_data == rs2_data)
						pcNext = pcCurrent + imm_b_sext;
				end
				if (insn_bne) begin
					if (rs1_data != rs2_data)
						pcNext = pcCurrent + imm_b_sext;
				end
				if (insn_blt) begin
					if ($signed(rs1_data) < $signed(rs2_data))
						pcNext = pcCurrent + imm_b_sext;
				end
				if (insn_bge) begin
					if ($signed(rs1_data) >= $signed(rs2_data))
						pcNext = pcCurrent + imm_b_sext;
				end
				if (insn_bltu) begin
					if (rs1_data < $unsigned(rs2_data))
						pcNext = pcCurrent + imm_b_sext;
				end
				if (insn_bgeu) begin
					if (rs1_data >= $unsigned(rs2_data))
						pcNext = pcCurrent + imm_b_sext;
				end
			end
			OpStore: begin
				address = rs1_data + imm_s_sext;
				addr_to_dmem = {address[31:2], 2'b00};
				if (insn_sb)
					case (address[1:0])
						default: illegal_insn = 1;
						2'b00: begin
							store_we_to_dmem = 4'b0001;
							store_data_to_dmem[7:0] = rs2_data[7:0];
						end
						2'b01: begin
							store_we_to_dmem = 4'b0010;
							store_data_to_dmem[15:8] = rs2_data[7:0];
						end
						2'b10: begin
							store_we_to_dmem = 4'b0100;
							store_data_to_dmem[23:16] = rs2_data[7:0];
						end
						2'b11: begin
							store_we_to_dmem = 4'b1000;
							store_data_to_dmem[31:24] = rs2_data[7:0];
						end
					endcase
				else if (insn_sh)
					case (address[1])
						default: illegal_insn = 1;
						1'b0: begin
							store_we_to_dmem = 4'b0011;
							store_data_to_dmem[15:0] = rs2_data[15:0];
						end
						1'b1: begin
							store_we_to_dmem = 4'b1100;
							store_data_to_dmem[31:16] = rs2_data[15:0];
						end
					endcase
				else if (insn_sw) begin
					store_data_to_dmem = rs2_data;
					store_we_to_dmem = 4'b1111;
				end
			end
			OpLoad: begin
				address = rs1_data + imm_i_sext;
				addr_to_dmem = {address[31:2], 2'b00};
				we = 1'b1;
				if (insn_lb)
					case (address[1:0])
						default: illegal_insn = 1;
						2'b00: rd_data = {{24 {load_data_from_dmem[7]}}, load_data_from_dmem[7:0]};
						2'b01: rd_data = {{24 {load_data_from_dmem[15]}}, load_data_from_dmem[15:8]};
						2'b10: rd_data = {{24 {load_data_from_dmem[23]}}, load_data_from_dmem[23:16]};
						2'b11: rd_data = {{24 {load_data_from_dmem[31]}}, load_data_from_dmem[31:24]};
					endcase
				else if (insn_lh)
					case (address[1])
						default: illegal_insn = 1;
						1'b0: rd_data = {{16 {load_data_from_dmem[15]}}, load_data_from_dmem[15:0]};
						1'b1: rd_data = {{16 {load_data_from_dmem[31]}}, load_data_from_dmem[31:16]};
					endcase
				else if (insn_lw)
					rd_data = load_data_from_dmem;
				else if (insn_lbu)
					case (address[1:0])
						default: illegal_insn = 1;
						2'b00: rd_data = {24'b000000000000000000000000, load_data_from_dmem[7:0]};
						2'b01: rd_data = {24'b000000000000000000000000, load_data_from_dmem[15:8]};
						2'b10: rd_data = {24'b000000000000000000000000, load_data_from_dmem[23:16]};
						2'b11: rd_data = {24'b000000000000000000000000, load_data_from_dmem[31:24]};
					endcase
				else if (insn_lhu)
					case (address[1])
						default: illegal_insn = 1;
						1'b0: rd_data = {16'b0000000000000000, load_data_from_dmem[15:0]};
						1'b1: rd_data = {16'b0000000000000000, load_data_from_dmem[31:16]};
					endcase
			end
			OpEnviron:
				if (insn_ecall)
					halt = 1'b1;
			OpJal:
				if (insn_jal) begin
					rd_data = pcCurrent + 32'd4;
					pcNext = pcCurrent + imm_j_sext;
					we = 1'b1;
				end
			OpJalr:
				if (insn_jalr) begin
					rd_data = pcCurrent + 32'd4;
					pcNext = (rs1_data + imm_i_sext) & ~1;
					we = 1'b1;
				end
			default: illegal_insn = 1'b1;
		endcase
	end
	initial _sv2v_0 = 0;
endmodule
module MemorySingleCycle (
	rst,
	clock_mem,
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
	input wire clock_mem;
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
	always @(posedge clock_mem)
		if (rst)
			;
		else
			insn_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
	always @(negedge clock_mem)
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
	wire clk_mem;
	wire clk_locked;
	MyClockGen clock_gen(
		.input_clk_25MHz(external_clk_25MHz),
		.clk_proc(clk_proc),
		.clk_mem(clk_mem),
		.locked(clk_locked)
	);
	wire [31:0] pc_to_imem;
	wire [31:0] insn_from_imem;
	wire [31:0] mem_data_addr;
	wire [31:0] mem_data_loaded_value;
	wire [31:0] mem_data_to_write;
	wire [3:0] mem_data_we;
	MemorySingleCycle #(.NUM_WORDS(128)) memory(
		.rst(!clk_locked),
		.clock_mem(clk_mem),
		.pc_to_imem(pc_to_imem),
		.insn_from_imem(insn_from_imem),
		.addr_to_dmem(mem_data_addr),
		.load_data_from_dmem(mem_data_loaded_value),
		.store_data_to_dmem(mem_data_to_write),
		.store_we_to_dmem(mem_data_we)
	);
	DatapathMultiCycle datapath(
		.clk(clk_proc),
		.rst(!clk_locked),
		.pc_to_imem(pc_to_imem),
		.insn_from_imem(insn_from_imem),
		.addr_to_dmem(mem_data_addr),
		.store_data_to_dmem(mem_data_to_write),
		.store_we_to_dmem(mem_data_we),
		.load_data_from_dmem(mem_data_loaded_value),
		.halt(led[0])
	);
endmodule
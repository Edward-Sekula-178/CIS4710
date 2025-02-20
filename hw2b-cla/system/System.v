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
	genvar _gv_i_2;
	generate
		for (_gv_i_2 = 0; _gv_i_2 < 32; _gv_i_2 = _gv_i_2 + 1) begin : genblk1
			localparam i = _gv_i_2;
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
module SystemDemo (
	btn,
	led
);
	input wire [6:0] btn;
	output wire [7:0] led;
	wire [31:0] sum;
	cla cla_inst(
		.a(32'd26),
		.b({27'b000000000000000000000000000, btn[1], btn[2], btn[5], btn[4], btn[6]}),
		.cin(1'b0),
		.sum(sum)
	);
	assign led = sum[7:0];
endmodule
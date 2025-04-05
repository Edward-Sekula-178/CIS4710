/* EDWARD SEKULA 84768505 */
`timescale 1ns / 1ns

// quotient = dividend / divisor
/* A half-adder that adds two 1-bit numbers and produces a 2-bit result (as sum
 * and carry-out) */
module halfadder(input wire  a,
                 input wire  b,
                 output wire s,
                 output wire cout);
   assign s = a ^ b;
   assign cout = a & b;
endmodule

/* A full adder adds three 1-bit numbers (a, b, carry-in) and produces a 2-bit
 * result (as sum and carry-out) */
module fulladder(input wire  cin,
                 input wire  a,
                 input wire  b,
                 output wire s,
                 output wire cout);
   wire s_tmp, cout_tmp1, cout_tmp2;
   halfadder h0(.a(a), .b(b), .s(s_tmp), .cout(cout_tmp1));
   halfadder h1(.a(s_tmp), .b(cin), .s(s), .cout(cout_tmp2));
   assign cout = cout_tmp1 | cout_tmp2;
endmodule

/* A full adder that adds 2-bit numbers. Builds upon the 1-bit full adder. */
module fulladder2(input wire        cin,
                  input wire  [1:0] a,
                  input wire  [1:0] b,
                  output wire [1:0] s,
                  output wire       cout);
   wire cout_tmp;
   fulladder a0(.cin(cin), .a(a[0]), .b(b[0]), .s(s[0]), .cout(cout_tmp));
   fulladder a1(.cin(cout_tmp), .a(a[1]), .b(b[1]), .s(s[1]), .cout(cout));
endmodule

/* 4-bit ripple-carry adder that adds two 4-bit numbers */
module rca4(input wire        cin,
            input wire [3:0]  a,
            input wire [3:0]  b,
            output wire [3:0] sum,
            output wire       cout);
   wire cout0,carry_out;
   fulladder2 a0(.cin(cin), .a(a[1:0]), .b(b[1:0]), .s(sum[1:0]), .cout(cout0));
   fulladder2 a3(.cin(cout0), .a(a[3:2]), .b(b[3:2]), .s(sum[3:2]), .cout(cout));
endmodule

module rca8(input wire        cin,
            input wire [7:0]  a,
            input wire [7:0]  b,
            output wire [7:0] sum,
            output wire       cout);
   wire cout0;
   rca4 a0(.cin(cin), .a(a[3:0]), .b(b[3:0]), .sum(sum[3:0]), .cout(cout0));
   rca4 a7(.cin(cout0), .a(a[7:4]), .b(b[7:4]), .sum(sum[7:4]), .cout(cout));
endmodule

module rca32(input wire cin,
            input wire [31:0]  a,
            input wire [31:0]  b,
            output wire [31:0] sum32,
            output wire       carry_out);
   wire cout0;
   wire cout1;
   wire cout2;

   rca8 a0(.cin(cin), .a(a[7:0]), .b(b[7:0]), .sum(sum32[7:0]), .cout(cout0));
   rca8 a8(.cin(cout0), .a(a[15:8]), .b(b[15:8]), .sum(sum32[15:8]), .cout(cout1));
   rca8 a16(.cin(cout1), .a(a[23:16]), .b(b[23:16]), .sum(sum32[23:16]), .cout(cout2));
   rca8 a24(.cin(cout2), .a(a[31:24]), .b(b[31:24]), .sum(sum32[31:24]), .cout(carry_out));
endmodule

typedef struct packed {
    logic [31:0] pc;
    logic [31:0] insn;
    logic [7:0]  insn_ic;
    // div stuff
    logic [31:0] dividend;
    logic [31:0] remainder;
    logic [31:0] quotient;
    logic [31:0] divisor;
} div_register_t;


module DividerUnsignedPipelined (
    input wire clk, rst, stall,
    input wire  [31:0] i_dividend,
    input wire  [31:0] i_divisor,
    // HW5 adding controll information
    input wire [31:0] i_pc, i_insn,
    input wire [7:0] i_insn_ic,
    output logic [31:0] o_remainder,
    output logic [31:0] o_quotient,

    output logic [31:0] o_pc,o_insn,
    output logic [7:0] o_insn_ic
);
/**/
    div_register_t div_registers[7];

    wire [31:0] stage_dividend[8];
    wire [31:0] stage_remainder[8];
    wire [31:0] stage_quotient[8];

    stage stage1(
        .i_dividend(i_dividend),
        .i_divisor(i_divisor),
        .i_remainder(32'b0),
        .i_quotient(32'b0),
        .o_dividend(stage_dividend[0]),
        .o_remainder(stage_remainder[0]),
        .o_quotient(stage_quotient[0])
    );
    always_ff @(posedge clk) begin
        if(rst) begin
          div_registers[0] <= '{
                dividend: 32'b0,
                remainder: 32'b0,
                quotient: 32'b0,
                divisor: 32'b0,

                pc: 32'b0,
                insn: 32'b0,
                insn_ic: 8'b0
            };
        end else if (!stall) begin
            div_registers[0] <= '{
                dividend: stage_dividend[0],
                remainder: stage_remainder[0],
                quotient: stage_quotient[0],
                divisor: i_divisor,

                pc: i_pc,
                insn: i_insn,
                insn_ic: i_insn_ic
            };
        end else begin
            div_registers[0] <= div_registers[0];
        end
    end

    genvar i;
    for (i=1; i < 7; i = i + 1) begin : gen_stages
        always_ff @(posedge clk) begin
            if (rst) begin
                div_registers[i] <= '{
                    dividend: 32'b0,
                    remainder: 32'b0,
                    quotient: 32'b0,
                    divisor: 32'b0,

                    pc: 32'b0,
                    insn: 32'b0,
                    insn_ic: 8'b0
                };
            end else if (!stall) begin
                div_registers[i] <= '{
                    dividend: stage_dividend[i],
                    remainder: stage_remainder[i],
                    quotient: stage_quotient[i],
                    divisor: div_registers[i-1].divisor, // keep the same divisor

                    // for HW5B and later
                    pc: div_registers[i-1].pc,
                    insn: div_registers[i-1].insn,
                    insn_ic: div_registers[i-1].insn_ic
                };
            end else begin
                // stall: keep the previous stage's values
                div_registers[i] <= div_registers[i];
            end
        end
    end
    assign o_pc = div_registers[6].pc;
    assign o_insn = div_registers[6].insn;
    assign o_insn_ic = div_registers[6].insn_ic;

    // generate stages
    genvar j;
    for (j=0; j < 7; j = j + 1) begin : gen_stage
        stage stage_inst (
            .i_dividend(div_registers[j].dividend),
            .i_divisor(div_registers[j].divisor),
            .i_remainder(div_registers[j].remainder),
            .i_quotient(div_registers[j].quotient),
            .o_dividend(stage_dividend[j + 1]),
            .o_remainder(stage_remainder[j + 1]),
            .o_quotient(stage_quotient[j + 1])
        );
    end

    wire [31:0] stagef_dividend;
    stage stagef(
        .i_dividend(div_registers[6].dividend),
        .i_divisor(div_registers[6].divisor),
        .i_remainder(div_registers[6].remainder),
        .i_quotient(div_registers[6].quotient),
        .o_dividend(stagef_dividend),
        .o_remainder(o_remainder),
        .o_quotient(o_quotient)
    );
endmodule

module divu_1iter (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    input  wire [31:0] i_remainder,
    input  wire [31:0] i_quotient,
    output wire [31:0] o_dividend,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);
    wire [31:0] dividend_temp;

    assign dividend_temp[31:0] = {i_remainder[30:0],i_dividend[31]};

    wire [31:0] n_divisor;
    assign n_divisor[31:0] = ~i_divisor[31:0];

    wire [31:0] sum;
    wire c_out;

    rca32 a32(.cin(1'b1),.a(dividend_temp[31:0]), 
    .b(n_divisor[31:0]), .sum32(sum[31:0]), .carry_out(c_out));

    always_comb begin : pos_diff_mux
        if (c_out) begin
            o_remainder[31:0] = sum[31:0];
        end else begin
            o_remainder[31:0] = dividend_temp[31:0];
        end
    end

    assign o_quotient[31:1] = i_quotient[30:0];
    assign o_quotient[0] = c_out;
    assign o_dividend[31:0] = {i_dividend[30:0],1'b0};
endmodule

module stage (
    input wire [31:0] i_dividend,
    input wire [31:0] i_divisor,
    input wire [31:0] i_remainder,
    input wire [31:0] i_quotient,
    output wire [31:0] o_dividend,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);

    wire [31:0] dividends[5];
    wire [31:0] remainders[5];
    wire [31:0] quotients[5];

    assign dividends[0] = i_dividend;
    assign remainders[0] = i_remainder;
    assign quotients[0] = i_quotient;

    genvar i;
    for(i = 1; i < 5; i = i + 1) begin
        divu_1iter iter(
            .i_dividend(dividends[i - 1]),
            .i_divisor(i_divisor),
            .i_remainder(remainders[i - 1]),
            .i_quotient(quotients[i - 1]),
            .o_dividend(dividends[i]),
            .o_remainder(remainders[i]),
            .o_quotient(quotients[i])
        );
    end

    assign o_dividend = dividends[4];
    assign o_remainder = remainders[4];
    assign o_quotient = quotients[4];
endmodule



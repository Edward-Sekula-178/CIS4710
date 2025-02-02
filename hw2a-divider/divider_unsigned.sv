/* INSERT NAME AND PENNKEY HERE */

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
module rca4(input wire [3:0]  a,
            input wire [3:0]  b,
            output wire [3:0] sum,
            output wire       cout);
   wire cout0,carry_out;
   fulladder2 a0(.cin(1'b0), .a(a[1:0]), .b(b[1:0]), .s(sum[1:0]), .cout(cout0));
   fulladder2 a3(.cin(cout0), .a(a[3:2]), .b(b[3:2]), .s(sum[3:2]), .cout(cout));
endmodule

module rca8(input wire [7:0]  a,
            input wire [7:0]  b,
            output wire [7:0] sum,
            output wire       cout);
   wire cout0;
   rca4 a0(.cin(1'b0), .a(a[3:0]), .b(b[3:0]), .sum(sum[3:0]), .cout(cout0));
   rca4 a7(.cin(cout0), .a(a[7:4]), .b(b[7:4]), .sum(sum[3:2]), .cout(cout));
endmodule

module rca32(input wire [31:0]  a,
            input wire [31:0]  b,
            output wire [31:0] sum32,
            output wire       carry_out);
   wire cout0;
   wire cout1;
   wire cout2;

   rca8 a0(.cin(1'b0), .a(a[7:0]), .b(b[7:0]), .sum(sum32[7:0]), .cout(cout0));
   rca8 a8(.cin(cout0), .a(a[15:8]), .b(b[15:8]), .sum(sum32[15:8]), .cout(cout1));
   rca8 a16(.cin(cout1), .a(a[23:16]), .b(b[23:16]), .sum(sum32[23:16]), .cout(cout2));
   rca8 a24(.cin(cout2), .a(a[31:24]), .b(b[31:24]), .sum(sum32[31:24]), .cout(carry_out));
endmodule



module divider_unsigned (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
); 
    // genvar i;
    // for(i=0;i < 32; i = i + 1) begin
    //     divu_1iter()
    
    // end


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
  /*
    for (int i = 0; i < 32; i++) {
        remainder = (remainder << 1) | ((dividend >> 31) & 0x1);
        if (remainder < divisor) {
            quotient = (quotient << 1);
        } else {
            quotient = (quotient << 1) | 0x1;
            remainder = remainder - divisor;
        }
        dividend = dividend << 1;
    }
    */

    // TODO: your code here
    wire dividend_temp[31:0];

    assign dividend_temp[31:1] = i_remainder[30:0];
    assign dividend_temp[0] = i_dividend[31];
    
    wire c_out;

    wire n_divisor[31:0];
    assign n_divisor[31:0] = ~i_divisor[31:0];

    wire sum32[31:0];

    rac32 a32(.cin(1'b0), .a(dividend_temp[31:0]), .b(n_divisor[31:0]), .sum32(sum32[31:0]), .cout(c_out));
    
    always_comb begin : pos_diff_mux
        if (cout == 1'b1) begin
            assign o_remainder = sum;
        end else begin
            assign o_remainder = remainder;
        end
    end

    assign o_quotient[31:1] = i_quotient[30:0];
    assign o_quotient[0] = cout;
    assign o_dividend[31:1] = i_dividend[30:0]; 


endmodule

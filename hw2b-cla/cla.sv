`timescale 1ns / 1ps
`include "../hw2a-divider/divider_unsigned.sv"

/**
 * @param a first 1-bit input
 * @param b second 1-bit input
 * @param g whether a and b generate a carry
 * @param p whether a and b would propagate an incoming carry
 */
module gp1(input wire a, b,
           output wire g, p);
   assign g = a & b;
   assign p = a | b;
endmodule

/**
 * Computes aggregate generate/propagate signals over a 4-bit window.
 * @param gin incoming generate signals
 * @param pin incoming propagate signals
 * @param cin the incoming carry
 * @param gout whether these 4 bits internally would generate a carry-out (independent of cin)
 * @param pout whether these 4 bits internally would propagate an incoming carry from cin
 * @param cout the carry outs for the low-order 3 bits
 */
module gp4(input wire [3:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [2:0] cout);
   assign pout = (&pin);
   assign gout = (gin[3])|(gin[2]&pin[3])|(gin[1]&(&pin[3:2]))|(gin[0]&(&pin[3:1]));

   assign cout[0] = gin[0]|(cin&pin[0]);//c1
   assign cout[1] = gin[1]|(gin[0]&pin[1])|(cin&(&pin[1:0]));//c2
   assign cout[2] = gin[2]|(gin[1]&pin[2])|(gin[0]&(&pin[2:1]))|(cin&(&pin[2:0]));//c3
endmodule

/** Same as gp4 but for an 8-bit window instead */
module gp8(input wire [7:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [6:0] cout);
   wire p0,g0,p1,g1;
   
   assign pout = (&pin);
   assign gout = g1 | (g0&p1);
   
   assign cout[3] = (cin&p0) | g0;

   gp4 m1(.gin(gin[3:0]),.pin(pin[3:0]),.cin(cin),.gout(g0),.pout(p0),.cout(cout[2:0]));
   gp4 m2(.gin(gin[7:4]),.pin(pin[7:4]),.cin(cout[3]),.gout(g1),.pout(p1),.cout(cout[6:4]));

   
endmodule

module cla
  (input wire [31:0]  a, b,
   input wire         cin,
   output wire [31:0] sum);
   /**calculate gp*/
   wire [31:0] g,p;
   wire [6:0] c1[4];

   wire [3:0] c4;
   wire [3:0] p0;
   wire [3:0] g0;

   wire [31:0] carry;
   wire [31:0] cnull;

   assign c4[0] =  cin;

   genvar i;
   for (i=0;i<32;i=i+1) begin
      gp1 gp(.a(a[i]),.b(b[i]),.g(g[i]),.p(p[i]));
   end

   
   
   genvar k;
   for (k=0;k<4;k=k+1) begin
      gp8 m1(.gin(g[(7+(8*k)):(0+(8*k))]),.pin(p[(7+(8*k)):(0+(8*k))]),
      .cin(c4[k]),.gout(g0[k]),.pout(p0[k]),.cout(c1[k]));
   end
   wire gout,pout;
   gp4 m(.gin(g0), .pin(p0), .cin(cin),.gout(gout),.pout(pout),.cout(c4[3:1]));
   
   assign carry = {c1[3][6:0],c4[3],
                  c1[2][6:0],c4[2],
                  c1[1][6:0],c4[1],
                  c1[0][6:0],c4[0]};
   

   genvar k2;
   for (k2=0; k2<32;k2=k2+1) begin
      fulladder fa(.a(a[k2]),.b(b[k2]),.s(sum[k2]),.cin(carry[k2]),.cout(cnull[k2]));
   end

endmodule

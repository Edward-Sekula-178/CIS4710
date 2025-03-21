`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31:0

// insns are 32 bits in RV32IM
`define INSN_SIZE 31:0

// RV opcodes are 7 bits
`define OPCODE_SIZE 6:0

`ifndef DIVIDER_STAGES
`define DIVIDER_STAGES 8
`endif

`ifndef SYNTHESIS
`include "../hw3-singlecycle/RvDisassembler.sv"
`endif
`include "../hw2b-cla/cla.sv"
`include "../hw4-multicycle/DividerUnsignedPipelined.sv"
`include "cycle_status.sv"

module Disasm #(
    byte PREFIX = "D"
) (
    input wire [31:0] insn,
    output wire [(8*32)-1:0] disasm
);
`ifndef SYNTHESIS
  // this code is only for simulation, not synthesis
  string disasm_string;
  always_comb begin
    disasm_string = rv_disasm(insn);
  end
  // HACK: get disasm_string to appear in GtkWave, which can apparently show only wire/logic. Also,
  // string needs to be reversed to render correctly.
  genvar i;
  for (i = 3; i < 32; i = i + 1) begin : gen_disasm
    assign disasm[((i+1-3)*8)-1-:8] = disasm_string[31-i];
  end
  assign disasm[255-:8] = PREFIX;
  assign disasm[247-:8] = ":";
  assign disasm[239-:8] = " ";
`endif
endmodule

module RegFile (
    input logic [4:0] rd,
    input logic [`REG_SIZE] rd_data,
    input logic [4:0] rs1,
    output logic [`REG_SIZE] rs1_data,
    input logic [4:0] rs2,
    output logic [`REG_SIZE] rs2_data,

    input logic clk,
    input logic we,
    input logic rst
);
  localparam int NumRegs = 32;
  logic [`REG_SIZE] regs[NumRegs];

  assign rs1_data = regs[rs1]; // 1st read port
  assign rs2_data = regs[rs2]; // 2nd read port

  assign regs[0] = 32'b0; // x0 is always zero


    always_ff @(posedge clk) begin
      if (rst) begin
        integer i;
        for (i=1; i<32; i=i+1) begin
          regs[i] <= 32'd0;
        end
      end else begin
        if (we) begin
          if (rd!=0) regs[rd] <= rd_data;
        end
      end
    end
endmodule

/** state at the start of Decode stage */
typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;
} stage_decode_t;

typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;

  logic [4:0] rs1;
  logic [4:0] rs2;
  logic [4:0] rd;

  /*not sure what parts are needed*/

  logic [19:0] imm_u;
  logic [11:0] imm_s;
  logic [12:0] imm_b;
  logic [20:0] imm_j;
  logic [11:0] imm_i;

  logic [`REG_SIZE] imm_i_sext;
  logic [`REG_SIZE] imm_s_sext;
  logic [`REG_SIZE] imm_b_sext;
  logic [`REG_SIZE] imm_j_sext;

  logic [7:0] insn_ic;
} stage_execute_t;

typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;

  logic [4:0] rs1,rs2,rd;

  logic [`REG_SIZE] rd_data;
  logic we, halt;

  logic [7:0] insn_ic;
} stage_memory_t;

typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;

  logic [4:0] rd;
  logic [`REG_SIZE] rd_data;

  logic we, halt;
} stage_writeback_t;


module DatapathPipelined (
    input wire clk,
    input wire rst,
    output logic [`REG_SIZE] pc_to_imem,
    input wire [`INSN_SIZE] insn_from_imem,
    // dmem is read/write
    output logic [`REG_SIZE] addr_to_dmem,
    input wire [`REG_SIZE] load_data_from_dmem,
    output logic [`REG_SIZE] store_data_to_dmem,
    output logic [3:0] store_we_to_dmem,

    output logic halt,

    // The PC of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`REG_SIZE] trace_writeback_pc,
    // The bits of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`INSN_SIZE] trace_writeback_insn,
    // The status of the insn (or stall) currently in Writeback. See the cycle_status.sv file for valid values.
    output cycle_status_e trace_writeback_cycle_status
);


  // cycle counter, not really part of any stage but useful for orienting within GtkWave
  // do not rename this as the testbench uses this value
  logic [`REG_SIZE] cycles_current;
  always_ff @(posedge clk) begin
    if (rst) begin
      cycles_current <= 0;
    end else begin
      cycles_current <= cycles_current + 1;
    end
  end

  RegFile rf (
    .clk(clk),
    .rst(rst),
    .we(w_we),
    .rd(w_state.rd),
    .rd_data(w_rd_data),
    .rs1(x_state.rs1),
    .rs2(x_state.rs2),
    .rs1_data(x_rs1_data),
    .rs2_data(x_rs2_data)
  );

  /***************/
  /* FETCH STAGE */
  /***************/

  logic [`REG_SIZE] f_pc_current;
  wire [`REG_SIZE] f_insn;
  cycle_status_e f_cycle_status;

  // program counter
  always_ff @(posedge clk) begin
    if (rst) begin
      f_pc_current <= 32'd0;
      // NB: use CYCLE_NO_STALL since this is the value that will persist after the last reset cycle
      f_cycle_status <= CYCLE_NO_STALL;
    end else if (x_branch) begin
      f_pc_current <= x_b_pc;
      f_cycle_status <= CYCLE_NO_STALL;
    end else begin
      f_cycle_status <= CYCLE_NO_STALL;
      f_pc_current <= f_pc_current + 4;
    end
  end
  // send PC to imem
  assign pc_to_imem = f_pc_current;
  assign f_insn = insn_from_imem;

  // Here's how to disassemble an insn into a string you can view in GtkWave.
  // Use PREFIX to provide a 1-character tag to identify which stage the insn comes from.
  wire [255:0] f_disasm;
  Disasm #(
      .PREFIX("F")
  ) disasm_0fetch (
      .insn  (f_insn),
      .disasm(f_disasm)
  );

  /****************/
  /* DECODE STAGE */
  /****************/

  // this shows how to package up state in a `struct packed`, and how to pass it between stages
  stage_decode_t decode_state;
  always_ff @(posedge clk) begin
    if (rst) begin
      decode_state <= '{
        pc: 0,
        insn: 0,
        cycle_status: CYCLE_RESET
      };
    end else if (x_branch) begin
      decode_state <= '{
        pc:0,
        insn: 0,
        cycle_status:CYCLE_TAKEN_BRANCH
      };
    end else begin
        decode_state <= '{
          pc: f_pc_current,
          insn: f_insn,
          cycle_status: f_cycle_status
        };
    end
  end
  wire [255:0] d_disasm;

  Disasm #(
      .PREFIX("D")
  ) disasm_1decode (
      .insn  (decode_state.insn),
      .disasm(d_disasm)
  );
  /*we first decode the instruction*/

  // components of the instruction
  wire [6:0] d_insn_funct7;
  wire [4:0] d_insn_rs2;
  wire [4:0] d_insn_rs1;
  wire [2:0] d_insn_funct3;
  wire [4:0] d_insn_rd;
  wire [`OPCODE_SIZE] d_insn_opcode;

  // split R-type instruction - see section 2.2 of RiscV spec
  assign {d_insn_funct7, d_insn_rs2, d_insn_rs1, d_insn_funct3, d_insn_rd, d_insn_opcode} = decode_state.insn;

  // setup for I, S, B & J type instructions
  // U - Type
  wire [19:0] d_imm_u;
  assign d_imm_u = decode_state.insn[31:12];

  // I - short immediates and loads
  wire [11:0] d_imm_i;
  assign d_imm_i = decode_state.insn[31:20];
  wire [ 4:0] d_imm_shamt = decode_state.insn[24:20];

  // S - stores
  wire [11:0] d_imm_s;
  assign d_imm_s[11:5] = d_insn_funct7, d_imm_s[4:0] = d_insn_rd;

  // B - conditionals
  wire [12:0] d_imm_b;
  assign {d_imm_b[12], d_imm_b[10:5]} = d_insn_funct7, {d_imm_b[4:1], d_imm_b[11]} = d_insn_rd, d_imm_b[0] = 1'b0;

  // J - unconditional jumps
  wire [20:0] d_imm_j;
  assign {d_imm_j[20], d_imm_j[10:1], d_imm_j[11], d_imm_j[19:12], d_imm_j[0]}
  = {decode_state.insn[31:12], 1'b0};

  wire [`REG_SIZE] d_imm_i_sext = {{20{d_imm_i[11]}}, d_imm_i[11:0]};
  wire [`REG_SIZE] d_imm_s_sext = {{20{d_imm_s[11]}}, d_imm_s[11:0]};
  wire [`REG_SIZE] d_imm_b_sext = {{19{d_imm_b[12]}}, d_imm_b[12:0]};
  wire [`REG_SIZE] d_imm_j_sext = {{11{d_imm_j[20]}}, d_imm_j[20:0]};

  // opcodes - see section 19 of RiscV spec
  localparam bit [`OPCODE_SIZE] OpLoad = 7'b00_000_11;
  localparam bit [`OPCODE_SIZE] OpStore = 7'b01_000_11;
  localparam bit [`OPCODE_SIZE] OpBranch = 7'b11_000_11;
  localparam bit [`OPCODE_SIZE] OpJalr = 7'b11_001_11;
  localparam bit [`OPCODE_SIZE] OpMiscMem = 7'b00_011_11;
  localparam bit [`OPCODE_SIZE] OpJal = 7'b11_011_11;

  localparam bit [`OPCODE_SIZE] OpRegImm = 7'b00_100_11;
  localparam bit [`OPCODE_SIZE] OpRegReg = 7'b01_100_11;
  localparam bit [`OPCODE_SIZE] OpEnviron = 7'b11_100_11;

  localparam bit [`OPCODE_SIZE] OpAuipc = 7'b00_101_11;
  localparam bit [`OPCODE_SIZE] OpLui = 7'b01_101_11;

  wire d_insn_lui   = d_insn_opcode == OpLui;
  wire d_insn_auipc = d_insn_opcode == OpAuipc;
  wire d_insn_jal   = d_insn_opcode == OpJal;
  wire d_insn_jalr  = d_insn_opcode == OpJalr;

  wire d_insn_beq  = d_insn_opcode == OpBranch && decode_state.insn[14:12] == 3'b000;
  wire d_insn_bne  = d_insn_opcode == OpBranch && decode_state.insn[14:12] == 3'b001;
  wire d_insn_blt  = d_insn_opcode == OpBranch && decode_state.insn[14:12] == 3'b100;
  wire d_insn_bge  = d_insn_opcode == OpBranch && decode_state.insn[14:12] == 3'b101;
  wire d_insn_bltu = d_insn_opcode == OpBranch && decode_state.insn[14:12] == 3'b110;
  wire d_insn_bgeu = d_insn_opcode == OpBranch && decode_state.insn[14:12] == 3'b111;

  wire d_insn_lb  = d_insn_opcode == OpLoad && decode_state.insn[14:12] == 3'b000;
  wire d_insn_lh  = d_insn_opcode == OpLoad && decode_state.insn[14:12] == 3'b001;
  wire d_insn_lw  = d_insn_opcode == OpLoad && decode_state.insn[14:12] == 3'b010;
  wire d_insn_lbu = d_insn_opcode == OpLoad && decode_state.insn[14:12] == 3'b100;
  wire d_insn_lhu = d_insn_opcode == OpLoad && decode_state.insn[14:12] == 3'b101;

  wire d_insn_sh = d_insn_opcode == OpStore && decode_state.insn[14:12] == 3'b001;
  wire d_insn_sb = d_insn_opcode == OpStore && decode_state.insn[14:12] == 3'b000;
  wire d_insn_sw = d_insn_opcode == OpStore && decode_state.insn[14:12] == 3'b010;

  wire d_insn_addi  = d_insn_opcode == OpRegImm && decode_state.insn[14:12] == 3'b000;
  wire d_insn_slti  = d_insn_opcode == OpRegImm && decode_state.insn[14:12] == 3'b010;
  wire d_insn_sltiu = d_insn_opcode == OpRegImm && decode_state.insn[14:12] == 3'b011;
  wire d_insn_xori  = d_insn_opcode == OpRegImm && decode_state.insn[14:12] == 3'b100;
  wire d_insn_ori   = d_insn_opcode == OpRegImm && decode_state.insn[14:12] == 3'b110;
  wire d_insn_andi  = d_insn_opcode == OpRegImm && decode_state.insn[14:12] == 3'b111;

  wire d_insn_slli = d_insn_opcode == OpRegImm && decode_state.insn[14:12] == 3'b001 && decode_state.insn[31:25] == 7'd0;
  wire d_insn_srli = d_insn_opcode == OpRegImm && decode_state.insn[14:12] == 3'b101 && decode_state.insn[31:25] == 7'd0;
  wire d_insn_srai = d_insn_opcode == OpRegImm && decode_state.insn[14:12] == 3'b101 && decode_state.insn[31:25] == 7'b0100000;

  wire d_insn_add  = d_insn_opcode == OpRegReg && decode_state.insn[14:12] == 3'b000 && decode_state.insn[31:25] == 7'd0;
  wire d_insn_sub  = d_insn_opcode == OpRegReg && decode_state.insn[14:12] == 3'b000 && decode_state.insn[31:25] == 7'b0100000;
  wire d_insn_sll  = d_insn_opcode == OpRegReg && decode_state.insn[14:12] == 3'b001 && decode_state.insn[31:25] == 7'd0;
  wire d_insn_slt  = d_insn_opcode == OpRegReg && decode_state.insn[14:12] == 3'b010 && decode_state.insn[31:25] == 7'd0;
  wire d_insn_sltu = d_insn_opcode == OpRegReg && decode_state.insn[14:12] == 3'b011 && decode_state.insn[31:25] == 7'd0;
  wire d_insn_xor  = d_insn_opcode == OpRegReg && decode_state.insn[14:12] == 3'b100 && decode_state.insn[31:25] == 7'd0;
  wire d_insn_srl  = d_insn_opcode == OpRegReg && decode_state.insn[14:12] == 3'b101 && decode_state.insn[31:25] == 7'd0;
  wire d_insn_sra  = d_insn_opcode == OpRegReg && decode_state.insn[14:12] == 3'b101 && decode_state.insn[31:25] == 7'b0100000;
  wire d_insn_or   = d_insn_opcode == OpRegReg && decode_state.insn[14:12] == 3'b110 && decode_state.insn[31:25] == 7'd0;
  wire d_insn_and  = d_insn_opcode == OpRegReg && decode_state.insn[14:12] == 3'b111 && decode_state.insn[31:25] == 7'd0;

  wire d_insn_mul    = d_insn_opcode == OpRegReg && decode_state.insn[31:25] == 7'd1 && decode_state.insn[14:12] == 3'b000;
  wire d_insn_mulh   = d_insn_opcode == OpRegReg && decode_state.insn[31:25] == 7'd1 && decode_state.insn[14:12] == 3'b001;
  wire d_insn_mulhsu = d_insn_opcode == OpRegReg && decode_state.insn[31:25] == 7'd1 && decode_state.insn[14:12] == 3'b010;
  wire d_insn_mulhu  = d_insn_opcode == OpRegReg && decode_state.insn[31:25] == 7'd1 && decode_state.insn[14:12] == 3'b011;
  wire d_insn_div    = d_insn_opcode == OpRegReg && decode_state.insn[31:25] == 7'd1 && decode_state.insn[14:12] == 3'b100;
  wire d_insn_divu   = d_insn_opcode == OpRegReg && decode_state.insn[31:25] == 7'd1 && decode_state.insn[14:12] == 3'b101;
  wire d_insn_rem    = d_insn_opcode == OpRegReg && decode_state.insn[31:25] == 7'd1 && decode_state.insn[14:12] == 3'b110;
  wire d_insn_remu   = d_insn_opcode == OpRegReg && decode_state.insn[31:25] == 7'd1 && decode_state.insn[14:12] == 3'b111;

  wire d_insn_ecall = d_insn_opcode == OpEnviron && decode_state.insn[31:7] == 25'd0;
  wire d_insn_fence = d_insn_opcode == OpMiscMem;

  logic [7:0] d_ic;
  localparam bit [7:0]
  IClui   = 8'd0,
  ICauipc = 8'd1,
  ICjal   = 8'd2,
  ICjalr  = 8'd3,
  ICbeq   = 8'd4,
  ICbne   = 8'd5,
  ICblt   = 8'd6,
  ICbge   = 8'd7,
  ICbltu  = 8'd8,
  ICbgeu  = 8'd9,
  IClb    = 8'd10,
  IClh    = 8'd11,
  IClw    = 8'd12,
  IClbu   = 8'd13,
  IClhu   = 8'd14,
  ICsb    = 8'd15,
  ICsh    = 8'd16,
  ICsw    = 8'd17,
  ICaddi  = 8'd18,
  ICslti  = 8'd19,
  ICsltiu = 8'd20,
  ICxori  = 8'd21,
  ICori   = 8'd22,
  ICandi  = 8'd23,
  ICslli  = 8'd24,
  ICsrli  = 8'd25,
  ICsrai  = 8'd26,
  ICadd   = 8'd27,
  ICsub   = 8'd28,
  ICsll   = 8'd29,
  ICslt   = 8'd30,
  ICsltu  = 8'd31,
  ICxor   = 8'd32,
  ICsrl   = 8'd33,
  ICsra   = 8'd34,
  ICor    = 8'd35,
  ICand   = 8'd36,
  ICmul   = 8'd37,
  ICmulh  = 8'd38,
  ICmulhsu= 8'd39,
  ICmulhu = 8'd40,
  ICdiv   = 8'd41,
  ICdivu  = 8'd42,
  ICrem   = 8'd43,
  ICremu  = 8'd44,
  ICecall = 8'd45,
  ICfence = 8'd46,
  ICIllegal = 8'd100;


always_comb begin
  if      (d_insn_lui) begin d_ic =  IClui; end
  else if (d_insn_auipc) begin d_ic =  ICauipc; end
  else if (d_insn_jal) begin d_ic =  ICjal; end
  else if (d_insn_jalr) begin d_ic =  ICjalr; end
  else if (d_insn_beq) begin d_ic =  ICbeq; end
  else if (d_insn_bne) begin d_ic =  ICbne; end
  else if (d_insn_blt) begin d_ic =  ICblt; end
  else if (d_insn_bge) begin d_ic =  ICbge; end
  else if (d_insn_bltu) begin d_ic =  ICbltu; end
  else if (d_insn_bgeu) begin d_ic =  ICbgeu; end
  else if (d_insn_lb) begin d_ic =  IClb; end
  else if (d_insn_lh) begin d_ic =  IClh; end
  else if (d_insn_lw) begin d_ic =  IClw; end
  else if (d_insn_lbu) begin d_ic =  IClbu; end
  else if (d_insn_lhu) begin d_ic =  IClhu; end
  else if (d_insn_sb) begin d_ic =  ICsb; end
  else if (d_insn_sh) begin d_ic =  ICsh; end
  else if (d_insn_sw) begin d_ic =  ICsw; end
  else if (d_insn_addi) begin d_ic =  ICaddi; end
  else if (d_insn_slti) begin d_ic =  ICslti; end
  else if (d_insn_sltiu) begin d_ic =  ICsltiu; end
  else if (d_insn_xori) begin d_ic =  ICxori; end
  else if (d_insn_ori) begin d_ic =  ICori; end
  else if (d_insn_andi) begin d_ic =  ICandi; end
  else if (d_insn_slli) begin d_ic =  ICslli; end
  else if (d_insn_srli) begin d_ic =  ICsrli; end
  else if (d_insn_srai) begin d_ic =  ICsrai; end
  else if (d_insn_add) begin d_ic =  ICadd; end
  else if (d_insn_sub) begin d_ic =  ICsub; end
  else if (d_insn_sll) begin d_ic =  ICsll; end
  else if (d_insn_slt) begin d_ic =  ICslt; end
  else if (d_insn_sltu) begin d_ic =  ICsltu; end
  else if (d_insn_xor) begin d_ic =  ICxor; end
  else if (d_insn_srl) begin d_ic =  ICsrl; end
  else if (d_insn_sra) begin d_ic =  ICsra; end
  else if (d_insn_or) begin d_ic =  ICor; end
  else if (d_insn_and) begin d_ic =  ICand; end
  else if (d_insn_mul) begin d_ic =  ICmul; end
  else if (d_insn_mulh) begin d_ic =  ICmulh; end
  else if (d_insn_mulhsu) begin d_ic =  ICmulhsu; end
  else if (d_insn_mulhu) begin d_ic =  ICmulhu; end
  else if (d_insn_div) begin d_ic =  ICdiv; end
  else if (d_insn_divu) begin d_ic =  ICdivu; end
  else if (d_insn_rem) begin d_ic =  ICrem; end
  else if (d_insn_remu) begin d_ic =  ICremu; end
  else if (d_insn_ecall) begin d_ic =  ICecall; end
  else if (d_insn_fence) begin d_ic =  ICfence; end
  else begin d_ic = ICIllegal; end
end

  /*****************/
  /* EXECUTE STAGE */
  /*****************/

  stage_execute_t x_state;
  always_ff @(posedge clk) begin
    if (rst) begin
      x_state <= '{
        pc: 0,
        insn: 0,
        cycle_status: CYCLE_RESET,
        rs1:0,
        rs2:0,
        rd:0,
        imm_u:0,
        imm_s:0,
        imm_b:0,
        imm_j:0,
        imm_i:0,
        imm_i_sext:0,
        imm_s_sext:0,
        imm_b_sext:0,
        imm_j_sext:0,
        insn_ic:0
      };
    end else if (x_branch) begin
      x_state <= '{
        pc: 0,
        insn: 0,
        cycle_status: CYCLE_TAKEN_BRANCH,
        rs1:0,
        rs2:0,
        rd:0,
        imm_u:0,
        imm_s:0,
        imm_b:0,
        imm_j:0,
        imm_i:0,
        imm_i_sext:0,
        imm_s_sext:0,
        imm_b_sext:0,
        imm_j_sext:0,
        insn_ic:0
      };
    end
    else begin
      begin
        x_state <= '{
          pc: decode_state.pc,
          insn: decode_state.insn,
          cycle_status: decode_state.cycle_status,
          rs1:d_insn_rs1,
          rs2:d_insn_rs2,
          rd:d_insn_rd,

          imm_u:d_imm_u,
          imm_s:d_imm_s,
          imm_b:d_imm_b,
          imm_j:d_imm_j,
          imm_i:d_imm_i,

          imm_i_sext:d_imm_i_sext,
          imm_s_sext:d_imm_s_sext,
          imm_b_sext:d_imm_b_sext,
          imm_j_sext:d_imm_j_sext,

          insn_ic:d_ic
        };
      end
    end
  end
  wire [255:0] x_disasm;
  Disasm #(
      .PREFIX("X")
  ) disasm_2execute (
      .insn  (x_state.insn),
      .disasm(x_disasm)
  );
  logic x_we, x_illegal_insn, x_branch,x_halt;
  logic [31:0] x_b_pc;

  logic [4:0] x_rs1, x_rs2, x_rd;

  logic [31:0] x_rd_data, x_data_1, x_data_2;

  wire [`REG_SIZE]  x_rs1_data;
  wire [`REG_SIZE]  x_rs2_data;

  logic [31:0] dividend,divisor, rem,quo;
  logic [63:0] m1,m2,m3;
  logic cin;

  logic [11:0] x_imm_i;
  assign x_imm_i = x_state.insn[31:20];
  wire x_r_rs1 =  (x_state.insn[6:0]==OpRegReg) ||
                  (x_state.insn[6:0]==OpRegImm) ||
                  (x_state.insn[6:0]==OpBranch)  ||
                  (x_state.insn[6:0]==OpStore);

  wire x_r_rs2 =  (x_state.insn[6:0]==OpBranch) ||
                  (x_state.insn[6:0]==OpStore)  ||
                  (x_state.insn[6:0]==OpRegReg);


  always_comb begin
    x_rd = 0;
    x_rd_data = 32'd00;
    x_we = 1'b0;

    x_illegal_insn = 1'b0;
    x_branch = 1'b0;
    x_halt = 1'b0;
    x_b_pc = 32'd0;

    dividend = 32'd0;
    divisor = 32'd0;

    m1 = 64'd0;
    m2 = 64'd0;
    m3 = 64'd0;

  // BYPASS
  x_data_1 = x_rs1_data;
  x_data_2 = x_rs2_data;
  if ((w_state.rd == x_state.rs1) && x_state.rs1!=0 && w_state.rd !=0 && x_r_rs1 && w_r_mem) begin x_data_1 = w_state.rd_data; end
  if ((w_state.rd == x_state.rs2) && x_state.rs2!=0 && w_state.rd !=0 && x_r_rs2 && w_r_mem) begin x_data_2 = w_state.rd_data; end
  if ((m_state.rd == x_state.rs1) && x_state.rs1!=0 && m_state.rd !=0 && x_r_rs1 && m_r_mem) begin x_data_1 = m_state.rd_data; end
  if ((m_state.rd == x_state.rs2) && x_state.rs2!=0 && m_state.rd !=0 && x_r_rs2 && m_r_mem) begin x_data_2 = m_state.rd_data; end
  /*writeback still required*/
    case (x_state.insn_ic)
      IClui: begin
        x_rd = x_state.rd;
        x_rd_data = {x_state.imm_u, 12'b0};
        x_we = 1'b1;
      end
      ICaddi: begin
        x_rd = x_state.rd;
        x_rd_data = x_data_1 + x_state.imm_i_sext;
        x_we = 1'b1;
      end
      ICslti: begin
        x_rd = x_state.rd;
        x_rd_data = $signed(x_data_1) < $signed(x_state.imm_i_sext) ? 32'h00000001 : 32'h00000000;
        x_we = 1'b1;
      end
      ICsltiu: begin
        x_rd = x_state.rd;
        x_rd_data = $unsigned(x_data_1) < $unsigned(x_state.imm_i_sext) ? 32'h00000001 : 32'h00000000;
        x_we = 1'b1;
      end
      ICxori: begin
        x_rd = x_state.rd;
        x_rd_data = x_data_1 ^ x_state.imm_i_sext;
        x_we = 1'b1;
      end
      ICandi: begin
        x_rd=x_state.rd;
        x_rd_data = x_data_1 & x_state.imm_i_sext;
        x_we = 1'b1;
      end
      ICori: begin
        x_rd = x_state.rd;
        x_rd_data = x_data_1 | x_state.imm_i_sext;
        x_we = 1'b1;
      end
      ICslli: begin
        x_rd = x_state.rd;
        x_rd_data = x_data_1 << x_state.imm_i[4:0];
        x_we = 1;
      end
      ICsrli: begin
        x_rd = x_state.rd;
        x_rd_data = x_data_1 >> x_state.imm_i[4:0];
        x_we = 1;
      end
      ICsrai: begin
        x_rd = x_state.rd;
        x_rd_data = $signed(x_data_1) >>> x_state.imm_i[4:0];
        x_we = 1;
      end
      ICadd: begin
        x_rd = x_state.rd;
        x_rd_data = x_data_1+x_data_2;
        x_we = 1;
      end
      ICsub: begin
        x_rd = x_state.rd;
        x_rd_data = x_data_1 - x_data_2;
        x_we = 1;
      end
      ICsll: begin 
        x_rd = x_state.rd;
        x_rd_data  = x_data_1 << x_data_2[4:0];
        x_we = 1;
      end
      ICslt: begin
        x_rd = x_state.rd;
        x_rd_data = $signed(x_data_1) < $signed(x_data_2) ? 32'h00000001 : 32'h00000000;
        x_we = 1;
      end
      ICsltu: begin
        x_rd = x_state.rd;
        x_rd_data = $unsigned(x_data_1) < $unsigned(x_data_2) ? 32'h00000001 : 32'h00000000;
        x_we = 1;
      end
      ICxor: begin
        x_rd = x_state.rd;
        x_rd_data = x_data_1 ^ x_data_2;
        x_we = 1;
      end
      ICsrl: begin
        x_rd = x_state.rd;
        x_rd_data  = x_data_1  >> x_data_2[4:0];
      end
      ICsra: begin
        x_rd = x_state.rd;
        x_rd_data = $signed(x_data_1) >>> x_data_2[4:0];
      end
      ICor: begin
        x_rd = x_state.rd;
        x_rd_data = x_data_1 | x_data_2;
        x_we = 1;
      end
      ICand: begin
        x_rd = x_state.rd;
        x_rd_data = x_data_1 & x_data_2;
        x_we = 1;
      end

      /*branching*/
      ICbeq: begin
        if (x_data_1 == x_data_2)begin
          x_b_pc = x_state.pc + x_state.imm_b_sext;
          x_branch = 1;
        end
      end
      ICbne: begin
        if (x_data_1 != x_data_2)begin
          x_b_pc = x_state.pc + x_state.imm_b_sext;
          x_branch = 1;
        end
      end
      ICblt: begin
        if ($signed(x_data_1) < $signed(x_data_2))begin
          x_b_pc = x_state.pc + x_state.imm_b_sext;
          x_branch = 1;
        end
      end
      ICbge: begin
        if ($signed(x_data_1) >= $signed(x_data_2))begin
          x_b_pc = x_state.pc + x_state.imm_b_sext;
          x_branch = 1;
        end
      end
      ICbltu: begin
        if (x_data_1 < $unsigned(x_data_2))begin
          x_b_pc = x_state.pc + x_state.imm_b_sext;
          x_branch = 1;
        end
      end
      ICbgeu: begin
        if (x_data_1 >= $unsigned(x_data_2))begin
          x_b_pc = x_state.pc + x_state.imm_b_sext;
          x_branch = 1;
        end
      end
      default: begin end


      InsnDiv : begin
          
          x_rd = execute_state.rd;

          if (d1[31])
            dividend = ~d1 + 1;
          else
            dividend = d1;
          if (d2[31])
            divisor = ~d2 + 1;
          else
            divisor = d2;

          if ((d1[31] ~^ d2[31]) || (d2 == 'd0))
            x_rd_data = quotient;          
          else
            x_rd_data = ~quotient + 'd1;
            x_we = 1'b1;

        end

      InsnDivu : begin
          x_rd = execute_state.rd;

          dividend = d1;
          divisor = $unsigned(d2);
          x_rd_data = quotient;
          x_we = 1'b1;
        end

      InsnRem :  begin
          x_rd = execute_state.rd;

          if (d1[31])
            dividend = ~d1 + 1;
          else
            dividend = d1;
          if (d2[31])
            divisor = ~d2 + 1;
          else
            divisor = d2;
          if (d1[31])
            x_rd_data = ~remainder + 'd1;
          else
            x_rd_data = remainder;
          x_we = 1'b1;

        end

      InsnRemu: begin
          x_rd = execute_state.rd;

          dividend = d1;
          divisor = $unsigned(d2);
          x_rd_data = remainder;
          x_we = 1'b1;
        end


      ICmul: begin
          x_rd = x_state.rd;
          x_rd_data = (x_data_1 * x_data_2);
          x_we = 1'b1;
      end
      ICmulh: begin
          x_rd = x_state.rd;
          m1 = {{32{x_data_1[31]}}, x_data_1} * {{32{x_data_2[31]}}, x_data_2};
          x_rd_data = m1[63:32];
          x_we = 1'b1;
      end
      ICmulhsu: begin
          x_rd = x_state.rd;
          m2 = {{32{x_data_1[31]}}, x_data_1} * {32'b0, x_data_2};
          x_rd_data = m2[63:32];
          x_we = 1'b1;
        end

      ICmulhu: begin
          x_rd = x_state.rd;
          m3 = ($unsigned(x_data_1) * $unsigned(x_data_2));
          x_rd_data = m3[63:32];
          x_we = 1'b1;
        end


        ICecall: begin
          x_halt = 1;
        end
        ICfence: begin
          x_we = 1'b0;
        end
    endcase

  // then we process all of the instructions
  end

  /****************/
  /* MEMORY STAGE */
  /****************/
  stage_memory_t m_state;
  always_ff @(posedge clk) begin
    if (rst) begin
      m_state <= '{
        pc: 0,
        insn: 0,
        cycle_status: CYCLE_RESET,
        rs1:0,
        rs2:0,
        rd:0,
        rd_data:0,
        we: 0,
        halt:0,
        insn_ic:0
      };
    end else begin
      begin
        m_state <= '{
          pc: x_state.pc,
          insn: x_state.insn,
          cycle_status: x_state.cycle_status,

          rs1:x_state.rs1,
          rs2:x_state.rs2,
          rd:x_state.rd,

          rd_data: x_rd_data,
          we:x_we,
          halt:x_halt,

          insn_ic:x_state.insn_ic
        };
      end
    end
  end
  wire [255:0] m_disasm;
  Disasm #(
      .PREFIX("M")
  ) disasm_2memory (
      .insn  (m_state.insn),
      .disasm(m_disasm)
  );

    wire m_r_mem =  (m_state.insn[6:0]==OpLui)    ||
                    (m_state.insn[6:0]==OpRegReg) ||
                    (m_state.insn[6:0]==OpRegImm) ||
                    (m_state.insn[6:0]==OpJal);

  /****************/
  /* WRTBCK STAGE */
  /****************/

   stage_writeback_t w_state;
  always_ff @(posedge clk) begin
    if (rst) begin
      w_state <= '{
        pc: 0,
        insn: 0,
        cycle_status: CYCLE_RESET,
        rd:0,
        rd_data:0,
        we: 0,
        halt:0
      };
    end else begin
        w_state <= '{
          pc: m_state.pc,
          insn: m_state.insn,
          cycle_status: m_state.cycle_status,

          rd:m_state.rd,
          rd_data:m_state.rd_data,

          we: m_state.we,
          halt: m_state.halt
        };
    end
  end
  wire [255:0] w_disasm;
  Disasm #(
      .PREFIX("W")
  ) disasm_2writeback (
      .insn  (w_state.insn),
      .disasm(w_disasm)
  );
  logic w_we;
  logic [`REG_SIZE] w_rd_data;

  always_comb begin
    if (w_state.we == 1) begin
      w_rd_data = w_state.rd_data;
      w_we = w_state.we;
    end else begin
      w_rd_data = 32'd0;
      w_we = 0;
    end
  end

  wire w_r_mem = (w_state.insn[6:0]==OpRegImm) ||
                  (w_state.insn[6:0]==OpRegReg) ||
                  (w_state.insn[6:0]==OpLui)    ||
                  (w_state.insn[6:0]==OpJal);

  assign halt = (m_state.insn[6:0] == 7'h73) & (m_state.insn[31:7] == 'b0);

  assign trace_writeback_pc = w_state.pc;
  assign trace_writeback_insn = w_state.insn;
  assign trace_writeback_cycle_status = w_state.cycle_status;
endmodule

module MemorySingleCycle #(
    parameter int NUM_WORDS = 512
) (
    // rst for both imem and dmem
    input wire rst,

    // clock for both imem and dmem. The memory reads/writes on @(negedge clk)
    input wire clk,

    // must always be aligned to a 4B boundary
    input wire [`REG_SIZE] pc_to_imem,

    // the value at memory location pc_to_imem
    output logic [`REG_SIZE] insn_from_imem,

    // must always be aligned to a 4B boundary
    input wire [`REG_SIZE] addr_to_dmem,

    // the value at memory location addr_to_dmem
    output logic [`REG_SIZE] load_data_from_dmem,

    // the value to be written to addr_to_dmem, controlled by store_we_to_dmem
    input wire [`REG_SIZE] store_data_to_dmem,

    // Each bit determines whether to write the corresponding byte of store_data_to_dmem to memory location addr_to_dmem.
    // E.g., 4'b1111 will write 4 bytes. 4'b0001 will write only the least-significant byte.
    input wire [3:0] store_we_to_dmem
);

  // memory is arranged as an array of 4B words
  logic [`REG_SIZE] mem_array[NUM_WORDS];

`ifdef SYNTHESIS
  initial begin
    $readmemh("mem_initial_contents.hex", mem_array);
  end
`endif

  always_comb begin
    // memory addresses should always be 4B-aligned
    assert (pc_to_imem[1:0] == 2'b00);
    assert (addr_to_dmem[1:0] == 2'b00);
  end

  localparam int AddrMsb = $clog2(NUM_WORDS) + 1;
  localparam int AddrLsb = 2;

  always @(negedge clk) begin
    if (rst) begin
    end else begin
      insn_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
    end
  end

  always @(negedge clk) begin
    if (rst) begin
    end else begin
      if (store_we_to_dmem[0]) begin
        mem_array[addr_to_dmem[AddrMsb:AddrLsb]][7:0] <= store_data_to_dmem[7:0];
      end
      if (store_we_to_dmem[1]) begin
        mem_array[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <= store_data_to_dmem[15:8];
      end
      if (store_we_to_dmem[2]) begin
        mem_array[addr_to_dmem[AddrMsb:AddrLsb]][23:16] <= store_data_to_dmem[23:16];
      end
      if (store_we_to_dmem[3]) begin
        mem_array[addr_to_dmem[AddrMsb:AddrLsb]][31:24] <= store_data_to_dmem[31:24];
      end
      // dmem is "read-first": read returns value before the write
      load_data_from_dmem <= mem_array[{addr_to_dmem[AddrMsb:AddrLsb]}];
    end
  end
endmodule

/* This design has just one clock for both processor and memory. */
module Processor (
    input  wire  clk,
    input  wire  rst,
    output logic halt,
    output wire [`REG_SIZE] trace_writeback_pc,
    output wire [`INSN_SIZE] trace_writeback_insn,
    output cycle_status_e trace_writeback_cycle_status
);

  wire [`INSN_SIZE] insn_from_imem;
  wire [`REG_SIZE] pc_to_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
  wire [3:0] mem_data_we;

  // This wire is set by cocotb to the name of the currently-running test, to make it easier
  // to see what is going on in the waveforms.
  wire [(8*32)-1:0] test_case;

  MemorySingleCycle #(
      .NUM_WORDS(8192)
  ) memory (
      .rst                (rst),
      .clk                (clk),
      // imem is read-only
      .pc_to_imem         (pc_to_imem),
      .insn_from_imem     (insn_from_imem),
      // dmem is read-write
      .addr_to_dmem       (mem_data_addr),
      .load_data_from_dmem(mem_data_loaded_value),
      .store_data_to_dmem (mem_data_to_write),
      .store_we_to_dmem   (mem_data_we)
  );

  DatapathPipelined datapath (
      .clk(clk),
      .rst(rst),
      .pc_to_imem(pc_to_imem),
      .insn_from_imem(insn_from_imem),
      .addr_to_dmem(mem_data_addr),
      .store_data_to_dmem(mem_data_to_write),
      .store_we_to_dmem(mem_data_we),
      .load_data_from_dmem(mem_data_loaded_value),
      .halt(halt),
      .trace_writeback_pc(trace_writeback_pc),
      .trace_writeback_insn(trace_writeback_insn),
      .trace_writeback_cycle_status(trace_writeback_cycle_status)
  );

endmodule

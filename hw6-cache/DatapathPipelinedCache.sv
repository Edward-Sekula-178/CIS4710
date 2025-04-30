`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31:0

// insns are 32 bits in RV32IM
`define INSN_SIZE 31:0

// RV opcodes are 7 bits
`define OPCODE_SIZE 6:0

`define ADDR_WIDTH 32
`define DATA_WIDTH 32

`ifndef DIVIDER_STAGES
`define DIVIDER_STAGES 8
`endif

`ifndef SYNTHESIS
  `include "../hw3-singlecycle/RvDisassembler.sv"
`endif
`include "../hw2b-cla/cla.sv"
`include "../hw4-multicycle/DividerUnsignedPipelined.sv"
`include "../hw5-pipelined/cycle_status.sv"
`include "AxilCache.sv"

module Disasm #(
    PREFIX = "D"
) (
    input wire [31:0] insn,
    output wire [(8*32)-1:0] disasm
);
`ifndef RISCV_FORMAL
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
  logic [19:0] imm_u;
  logic [`REG_SIZE] imm_i_sext;
  logic [`REG_SIZE] imm_s_sext;
  logic [`REG_SIZE] imm_b_sext;
  logic [`REG_SIZE] imm_j_sext;
  logic [`REG_SIZE] data_1;
  logic [`REG_SIZE] data_2;

  logic [7:0] insn_ic;
} stage_execute_t;

typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;
  logic [`REG_SIZE] rd_data;
  logic [31:0] memory_address,data_dmem;
  logic [7:0] insn_ic;
} stage_memory_t;

typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;
  logic illegal_insn;
  logic [`REG_SIZE] rd_data;
  logic [7:0] insn_ic;
  logic [`REG_SIZE] memory_address;
} stage_writeback_t;


module DatapathPipelinedCache (
    input wire clk,
    input wire rst,

    // AXIL interface to insn memory
    axi_if.manager icache,
    // AXIL interface to data memory/cache
    axi_if.manager dcache,

    output logic halt,

    // The PC of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`REG_SIZE] trace_writeback_pc,
    // The bits of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`INSN_SIZE] trace_writeback_insn,
    // The status of the insn (or stall) currently in Writeback. See the cycle_status.sv file for valid values.
    output cycle_status_e trace_writeback_cycle_status
);

  localparam bit True = 1'b1;
  localparam bit False = 1'b0;

  // cycle counter
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
    .we(we_to_reg),
    .rd(rd_to_reg),
    .rd_data(data_to_reg),
    .rs1(x_state.insn[19:15]),
    .rs2(x_state.insn[24:20]),
    .rs1_data(x_rs1_data),
    .rs2_data(x_rs2_data)
  );

  /***************/
  /* FETCH STAGE */
  /***************/

  logic [`REG_SIZE] f_pc_current, f_pc_next;
  cycle_status_e f_cycle_status;

  // program counter
  always_comb begin
    if (rst) begin
      f_cycle_status = CYCLE_NO_STALL;
    end else if (x_branch) begin
      f_cycle_status = CYCLE_TAKEN_BRANCH;
    end else begin
      f_cycle_status = CYCLE_NO_STALL;
    end
  end

  logic f_div_stall_next, f_div_stall_curr;
  logic f_load_stall_next, f_load_stall_curr;

  logic f_fence;

  always_ff @(posedge clk) begin
    if (rst) begin
      f_pc_current <= 32'd0;
      f_div_stall_curr <= 1'b0;
      f_load_stall_curr <= 1'b0;
      m_grab_div_curr <= 1'b0;
    end else if (f_fence || f_div_stall_next || f_load_stall_next||writeback_waiting_for_cache) begin
      f_pc_current <= f_pc_current;
      f_div_stall_curr <= f_div_stall_next;
      f_load_stall_curr <= f_load_stall_next;
      m_grab_div_curr <= m_grab_div_next;
    end else begin
      f_pc_current <= f_pc_next;
      f_div_stall_curr <= f_div_stall_next;
      f_load_stall_curr <= f_load_stall_next;
      m_grab_div_curr <= m_grab_div_next;
    end
  end
  // send PC to imem
  always_comb begin
    if (f_load_stall_curr) begin //this may need to be f_loadstall_next
      icache.ARADDR = 0;
      icache.ARVALID = False;
      icache.RREADY = False;
    end else begin
      icache.ARADDR = f_pc_current;
      icache.ARVALID = True;
      icache.RREADY = True;
    end
  end

  // Here's how to disassemble an insn into a string you can view in GtkWave.
  // Use PREFIX to provide a 1-character tag to identify which stage the insn comes from.
  wire [255:0] f_disasm;
  Disasm #(
      .PREFIX("F")
  ) disasm_0fetch (
      .insn  (0),
      .disasm(f_disasm)
  );

  /****************/
  /* DECODE STAGE */
  /****************/

  // this shows how to package up state in a `struct packed`, and how to pass it between stages
  stage_decode_t decode_state;
  logic[`INSN_SIZE] insn_from_imem;
  assign insn_from_imem = icache.RDATA;

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
    end else if (f_div_stall_next || f_load_stall_next || f_fence || writeback_waiting_for_cache) begin
      decode_state <= decode_state;
    end else begin
        decode_state <= '{
          pc: f_pc_current,
          insn: 0,
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
  assign {d_insn_funct7, d_insn_rs2, d_insn_rs1, d_insn_funct3, d_insn_rd, d_insn_opcode} = insn_from_imem;

  // setup for I, S, B & J type instructions
  // U - Type
  wire [19:0] d_imm_u;
  assign d_imm_u = insn_from_imem[31:12];

  // I - short immediates and loads
  wire [11:0] d_imm_i;
  assign d_imm_i = insn_from_imem[31:20];
  wire [ 4:0] d_imm_shamt = insn_from_imem[24:20];

  // S - stores
  wire [11:0] d_imm_s;
  assign d_imm_s[11:5] = d_insn_funct7, d_imm_s[4:0] = d_insn_rd;

  // B - conditionals
  wire [12:0] d_imm_b;
  assign {d_imm_b[12], d_imm_b[10:5]} = d_insn_funct7, {d_imm_b[4:1], d_imm_b[11]} = d_insn_rd, d_imm_b[0] = 1'b0;

  // J - unconditional jumps
  wire [20:0] d_imm_j;
  assign {d_imm_j[20], d_imm_j[10:1], d_imm_j[11], d_imm_j[19:12], d_imm_j[0]}
  = {insn_from_imem[31:12], 1'b0};

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

  wire d_insn_beq  = d_insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b000;
  wire d_insn_bne  = d_insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b001;
  wire d_insn_blt  = d_insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b100;
  wire d_insn_bge  = d_insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b101;
  wire d_insn_bltu = d_insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b110;
  wire d_insn_bgeu = d_insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b111;

  wire d_insn_lb  = d_insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b000;
  wire d_insn_lh  = d_insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b001;
  wire d_insn_lw  = d_insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b010;
  wire d_insn_lbu = d_insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b100;
  wire d_insn_lhu = d_insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b101;

  wire d_insn_sh = d_insn_opcode == OpStore && insn_from_imem[14:12] == 3'b001;
  wire d_insn_sb = d_insn_opcode == OpStore && insn_from_imem[14:12] == 3'b000;
  wire d_insn_sw = d_insn_opcode == OpStore && insn_from_imem[14:12] == 3'b010;

  wire d_insn_addi  = d_insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b000;
  wire d_insn_slti  = d_insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b010;
  wire d_insn_sltiu = d_insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b011;
  wire d_insn_xori  = d_insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b100;
  wire d_insn_ori   = d_insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b110;
  wire d_insn_andi  = d_insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b111;

  wire d_insn_slli = d_insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b001 && insn_from_imem[31:25] == 7'd0;
  wire d_insn_srli = d_insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'd0;
  wire d_insn_srai = d_insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'b0100000;

  wire d_insn_add  = d_insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b000 && insn_from_imem[31:25] == 7'd0;
  wire d_insn_sub  = d_insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b000 && insn_from_imem[31:25] == 7'b0100000;
  wire d_insn_sll  = d_insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b001 && insn_from_imem[31:25] == 7'd0;
  wire d_insn_slt  = d_insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b010 && insn_from_imem[31:25] == 7'd0;
  wire d_insn_sltu = d_insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b011 && insn_from_imem[31:25] == 7'd0;
  wire d_insn_xor  = d_insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b100 && insn_from_imem[31:25] == 7'd0;
  wire d_insn_srl  = d_insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'd0;
  wire d_insn_sra  = d_insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'b0100000;
  wire d_insn_or   = d_insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b110 && insn_from_imem[31:25] == 7'd0;
  wire d_insn_and  = d_insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b111 && insn_from_imem[31:25] == 7'd0;

  wire d_insn_mul    = d_insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b000;
  wire d_insn_mulh   = d_insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b001;
  wire d_insn_mulhsu = d_insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b010;
  wire d_insn_mulhu  = d_insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b011;
  wire d_insn_div    = d_insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b100;
  wire d_insn_divu   = d_insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b101;
  wire d_insn_rem    = d_insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b110;
  wire d_insn_remu   = d_insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b111;

  wire d_insn_ecall = d_insn_opcode == OpEnviron && insn_from_imem[31:7] == 25'd0;
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

  always_comb begin
    if((d_ic == ICfence)&&(x_store || m_store)) begin f_fence = 1; end
    else begin f_fence = 0; end
  end

  /*****************/
  /* EXECUTE STAGE */
  /*****************/

  stage_execute_t x_state;
  always_ff @(posedge clk) begin
    if (rst || f_fence) begin
      x_state <= '{
        pc: 0,
        insn: 0,
        cycle_status: (rst) ? CYCLE_RESET : CYCLE_FENCEI,
        imm_u:0,
        imm_i_sext:0,
        imm_s_sext:0,
        imm_b_sext:0,
        imm_j_sext:0,
        data_1:0,
        data_2:0,
        insn_ic:0
      };
    end else if (f_div_stall_next || f_load_stall_next||writeback_waiting_for_cache)  begin
      x_state <= x_state;
    end else begin
        x_state <= '{
          pc: (x_branch) ? 0 :  decode_state.pc,
          insn: (x_branch) ? 0 :  insn_from_imem,
          cycle_status: (x_branch) ? CYCLE_TAKEN_BRANCH :  decode_state.cycle_status,
          imm_u: (x_branch) ? 0 : d_imm_u,
          imm_i_sext: (x_branch) ? 0 : d_imm_i_sext,
          imm_s_sext: (x_branch) ? 0 : d_imm_s_sext,
          imm_b_sext: (x_branch) ? 0 : d_imm_b_sext,
          imm_j_sext: (x_branch) ? 0 : d_imm_j_sext,
          data_1: (wd_bypass_rs1) ? w_state.rd_data : 0,
          data_2: (wd_bypass_rs2) ? w_state.rd_data : 0,
          insn_ic: (x_branch) ? 0 : d_ic
        };
    end
  end
  wire [255:0] x_disasm;
  Disasm #(
      .PREFIX("X")
  ) disasm_2execute (
      .insn  (x_state.insn),
      .disasm(x_disasm)
  );

  // control variables
  logic x_branch;
  logic x_con_insn_div;
  logic d_con_insn_div;
  logic m_grab_div_curr, m_grab_div_next, m_bubble_curr, m_bubble_next;
  logic [2:0] x_cycle_count, m_grab_div_count, m_bubble_count;

  // If the current X instruction is div
  assign x_con_insn_div = (x_state.insn_ic == ICdiv) | (x_state.insn_ic == ICdivu) |
                      (x_state.insn_ic == ICrem) | (x_state.insn_ic == ICremu);
  // If the current D instruction is div
  assign d_con_insn_div = (d_ic == ICdiv) | (d_ic == ICdivu) |
                      (d_ic == ICrem) | (d_ic == ICremu);
  // If we need to stall the pipeline i.e. if we don't have independent divs
  assign f_div_stall_next = (x_con_insn_div && d_con_insn_div && (d_insn_rs1 != x_rd) && (d_insn_rs2 != x_rd)) ?
                            0 : (x_con_insn_div && x_cycle_count!=3'd7);
  // We define grab div logic
  assign m_grab_div_next = d_con_insn_div | (m_grab_div_curr && m_grab_div_count != 7);

  always @(posedge clk) begin
    if (rst) begin
      x_cycle_count <= 3'd0;
    end else if (f_div_stall_next) begin
      if (x_cycle_count == 3'd7)
        x_cycle_count <= 3'd0;
      else
        x_cycle_count <= x_cycle_count + 3'd1;
    end else begin
      x_cycle_count <= 3'd0;
    end

    if(rst) begin m_grab_div_count <= 0; end
    else if (d_con_insn_div) begin m_grab_div_count <= 0; end
    else if (m_grab_div_next) begin
      if (m_grab_div_count == 7) begin m_grab_div_count <= 0; end
      else if (writeback_waiting_for_cache) begin m_grab_div_count <= m_grab_div_count; end
      else begin m_grab_div_count <= m_grab_div_count + 1; end
    end else begin m_grab_div_count <=0; end
  end

  // modules
  DividerUnsignedPipelined divider(.clk(clk),.rst(rst),.stall(writeback_waiting_for_cache),
    .i_dividend(dividend),.i_divisor(divisor),.o_remainder(rem),.o_quotient(quo),

    .i_pc(i_div_pc), .i_insn(i_div_insn),
    .i_insn_ic(i_div_ic),

    .o_pc(div_pc),.o_insn(div_insn),
    .o_insn_ic(div_insn_ic));


  logic [31:0] dividend,divisor, rem,quo, div_pc, div_insn, i_div_insn,i_div_pc;
  logic [7:0] div_insn_ic, i_div_ic;

  logic [63:0] m1,m2,m3;

  cla cla_mod(.a(a),.b(b),.cin(cin),.sum(sum));
  logic [31:0] a,b,sum;
  logic cin;
  // memory
  logic [31:0] x_memory_address,x_data_dmem;

  wire [`REG_SIZE]  x_rs1_data;
  wire [`REG_SIZE]  x_rs2_data;

  logic [31:0] x_rd_data, x_d_1, x_d_2;

  logic [11:0] x_imm_i;
  assign x_imm_i = x_state.insn[31:20];

  always_comb begin
    x_d_1 = (x_state.data_1 == 0) ? x_rs1_data : x_state.data_1;
    x_d_2 = (x_state.data_2 == 0) ? x_rs2_data : x_state.data_2;
    if (wx_bypass_rs1) begin x_d_1 = w_state.rd_data; end
    if (wx_bypass_rs2) begin x_d_2 = w_state.rd_data; end
    if (mx_bypass_rs1) begin x_d_1 = m_state.rd_data; end
    if (mx_bypass_rs2) begin x_d_2 = m_state.rd_data; end
  end

  logic x_store;
  always_comb begin
    if(x_state.insn_ic == ICsw || x_state.insn_ic == ICsb || x_state.insn_ic == ICsh) begin x_store = 1; end
    else begin x_store = 0; end
  end
  logic m_store;
  always_comb begin
    if(m_state.insn_ic == ICsw || m_state.insn_ic == ICsb || m_state.insn_ic == ICsh) begin m_store = 1; end
    else begin m_store = 0; end
  end
  logic m_load;
  always_comb begin
    if(m_state.insn_ic == IClw ||  m_state.insn_ic == IClb || m_state.insn_ic == IClh
              || m_state.insn_ic == IClhu|| m_state.insn_ic == IClbu) begin m_load = 1; end
    else begin m_load = 0; end
  end

  always_comb begin
    // load stall control
    if (m_load) begin
      if(x_store) begin
        if(m_rd == x_rs1 && x_req_rs1) begin f_load_stall_next = (f_load_stall_curr == 1'b0) ? 1'b1 : 1'b0; end
        else begin f_load_stall_next = 1'b0; end
      end else begin
        if(m_rd == x_rs1 && x_req_rs1) begin f_load_stall_next = (f_load_stall_curr == 1'b0) ? 1'b1 : 1'b0; end
        else if (m_rd == x_rs2 && x_req_rs2) begin f_load_stall_next = (f_load_stall_curr == 1'b0) ? 1'b1 : 1'b0; end
        else begin f_load_stall_next = 1'b0; end
      end
    end else begin f_load_stall_next = 1'b0; end

    // control vars
    f_pc_next = f_pc_current + 4;

    x_rd_data = 32'd00;

    x_branch = 1'b0;

    a = 32'b0;
    b = 32'b0;
    cin = 0;

    x_memory_address = 0;
    x_data_dmem = 0;

    m1 = 64'd0;
    m2 = 64'd0;
    m3 = 64'd0;

    dividend = 32'd0;
    divisor= 32'd0;
    i_div_ic = 8'd0;
    i_div_pc = 32'd0;
    i_div_insn = 32'd0;

    dcache.ARADDR = 0;
    dcache.ARVALID = False;
    dcache.RREADY = False;

    case (x_state.insn_ic)
      // U-type
      IClui: begin
        x_rd_data = {x_state.imm_u, 12'd0};
      end ICauipc: begin
        x_rd_data = x_state.pc + {x_state.insn[31:12],12'd0};
      end

      // I-type
      ICaddi: begin
        a = x_d_1;
        b = x_state.imm_i_sext;
        x_rd_data = sum;
      end
      ICslti: begin
        x_rd_data = $signed(x_d_1) < $signed(x_state.imm_i_sext) ? 32'h00000001 : 32'h00000000;
      end ICsltiu: begin
        x_rd_data = $unsigned(x_d_1) < $unsigned(x_state.imm_i_sext) ? 32'h00000001 : 32'h00000000;
      end ICxori: begin
        x_rd_data = x_d_1 ^ x_state.imm_i_sext;
      end ICandi: begin
        x_rd_data = x_d_1 & x_state.imm_i_sext;
      end ICori: begin
        x_rd_data = x_d_1 | x_state.imm_i_sext;
      end ICslli: begin
        x_rd_data = x_d_1 << x_state.imm_i_sext[4:0];
      end ICsrli: begin
        x_rd_data = x_d_1 >> x_state.imm_i_sext[4:0];
      end ICsrai: begin
        x_rd_data = $signed(x_d_1) >>> x_state.imm_i_sext[4:0];
      end


      // REG REG
      ICadd: begin
        a = x_d_1;
        b = x_d_2;
        x_rd_data = sum;
      end ICsub: begin
        a = x_d_1;
        b = ~x_d_2;
        x_rd_data = sum;
        cin = 1'b1;
      end ICsll: begin
        x_rd_data  = x_d_1 << x_d_2[4:0];
      end ICslt: begin
        x_rd_data = $signed(x_d_1) < $signed(x_d_2) ? 32'h00000001 : 32'h00000000;
      end ICsltu: begin
        x_rd_data = $unsigned(x_d_1) < $unsigned(x_d_2) ? 32'h00000001 : 32'h00000000;
      end ICxor: begin
        x_rd_data = x_d_1 ^ x_d_2;
      end ICsrl: begin
        x_rd_data  = x_d_1  >> x_d_2[4:0];
      end ICsra: begin
        x_rd_data = $signed(x_d_1) >>> x_d_2[4:0];
      end ICor: begin
        x_rd_data = x_d_1 | x_d_2;
      end ICand: begin
        x_rd_data = x_d_1 & x_d_2;
      end


      /*branching*/
      ICbeq: begin
        if (x_d_1 == x_d_2)begin
          f_pc_next = x_state.pc + x_state.imm_b_sext;
          x_branch = 1;
        end
      end ICbne: begin
        if (x_d_1 != x_d_2)begin
          f_pc_next = x_state.pc + x_state.imm_b_sext;
          x_branch = 1;
        end
      end ICblt: begin
        if ($signed(x_d_1) < $signed(x_d_2))begin
          f_pc_next = x_state.pc + x_state.imm_b_sext;
          x_branch = 1;
        end
      end ICbge: begin
        if ($signed(x_d_1) >= $signed(x_d_2))begin
          f_pc_next = x_state.pc + x_state.imm_b_sext;
          x_branch = 1;
        end
      end ICbltu: begin
        if (x_d_1 < $unsigned(x_d_2))begin
          f_pc_next = x_state.pc + x_state.imm_b_sext;
          x_branch = 1;
        end
      end ICbgeu: begin
        if (x_d_1 >= $unsigned(x_d_2))begin
          f_pc_next = x_state.pc + x_state.imm_b_sext;
          x_branch = 1;
        end
      end

      // JAL JALR
      ICjal: begin
        x_rd_data = x_state.pc + 32'd4;
        f_pc_next = x_state.pc + x_state.imm_j_sext;
        x_branch = 1;
      end ICjalr: begin
        x_rd_data = x_state.pc + 32'd4;
        f_pc_next = (x_d_1 + x_state.imm_i_sext) & ~1;
        x_branch = 1;
      end

      // Multiplication
      ICmul: begin
          x_rd_data = (x_d_1 * x_d_2);
      end ICmulh: begin

          m1 = {{32{x_d_1[31]}}, x_d_1} * {{32{x_d_2[31]}}, x_d_2};
          x_rd_data = m1[63:32];
      end ICmulhsu: begin

          m2 = {{32{x_d_1[31]}}, x_d_1} * {32'b0, x_d_2};
          x_rd_data = m2[63:32];
      end ICmulhu: begin

          m3 = ($unsigned(x_d_1) * $unsigned(x_d_2));
          x_rd_data = m3[63:32];
      end


      // LOAD AND STORE INSTRUCTIONS cin assumed 0
      IClb: begin
        a = x_d_1;
        b = x_state.imm_i_sext;
        x_memory_address = sum;
      end IClh: begin
        a = x_d_1;
        b = x_state.imm_i_sext;
        x_memory_address = sum;
      end IClw: begin
        a = x_d_1;
        b = x_state.imm_i_sext;
        x_memory_address = sum;
      end IClbu: begin
        a = x_d_1;
        b = x_state.imm_i_sext;
        x_memory_address = sum;
      end IClhu: begin
        a = x_d_1;
        b = x_state.imm_i_sext;
        x_memory_address = sum;
      end ICsw: begin
        a = x_d_1;
        b = x_state.imm_s_sext;
        x_memory_address = sum;
        x_data_dmem = x_d_2;
      end ICsh: begin
        x_memory_address = (x_d_1 + x_state.imm_s_sext);
        cin = 1'b0;
        x_data_dmem = x_d_2;
      end ICsb: begin
        a = x_d_1;
        b = x_state.imm_s_sext;
        x_memory_address = sum;
        cin = 0;
        x_data_dmem = x_d_2;
      end

      ICdiv: begin
          i_div_ic = x_state.insn_ic;
          i_div_insn = x_state.insn;
          i_div_pc = x_state.pc;

          if (x_d_1[31]) begin
            dividend = ~x_d_1 + 1;
          end else begin
            dividend = x_d_1;
          end if (x_d_2[31]) begin
            divisor = ~x_d_2 + 1;
          end else begin
            divisor = x_d_2;
          end

          if ((x_d_1[31] ~^ x_d_2[31]) || (x_d_2 == 32'd0)) begin
            x_rd_data = quo;
          end else begin
            x_rd_data = ~quo + 'd1;
          end
      end
      ICrem: begin
        i_div_ic = x_state.insn_ic;
        i_div_insn = x_state.insn;
        i_div_pc = x_state.pc;
        if (x_d_1[31]) begin
          dividend = ~x_d_1 + 1;
        end else begin
          dividend = x_d_1;
        end if (x_d_2[31]) begin
          divisor = ~x_d_2 + 1;
        end else begin
          divisor = x_d_2;
        end

        if (x_d_1[31]) begin
          x_rd_data = ~rem + 1;
        end else begin
          x_rd_data = rem;
        end
      end

      ICdivu: begin
        i_div_ic = x_state.insn_ic;
        i_div_insn = x_state.insn;
        i_div_pc = x_state.pc;
        dividend = x_d_1;
        divisor = $unsigned(x_d_2);
        x_rd_data = quo;
      end

      ICremu: begin
        i_div_ic = x_state.insn_ic;
        i_div_insn = x_state.insn;
        i_div_pc = x_state.pc;
        divisor = $unsigned(x_d_2);
        dividend = x_d_1;
        x_rd_data = rem;
      end

      // covers ecall fence and illegal
      default: begin end
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
        rd_data:0,
        memory_address:0,
        data_dmem:0,
        insn_ic:0
      };
     end else if (m_grab_div_curr) begin
      m_state <= '{
        pc: div_pc,
        insn: div_insn,
        cycle_status: CYCLE_DIV2USE,
        rd_data: x_rd_data,

        memory_address:0,
        data_dmem: 0,

        insn_ic: div_insn_ic
      };end else if (f_div_stall_next || f_load_stall_next || writeback_waiting_for_cache) begin
      m_state <= '{
        pc: 0,
        insn: 0,
        cycle_status: f_load_stall_next ? CYCLE_LOAD2USE : CYCLE_DIV2USE,
        rd_data:0,
        memory_address:0,
        data_dmem:0,
        insn_ic:0
      };
    end else begin
      begin
        m_state <= '{
          pc: x_state.pc,
          insn: x_state.insn,
          cycle_status: x_state.cycle_status,

          rd_data: x_rd_data,

          memory_address: x_memory_address,
          data_dmem: x_data_dmem,

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

  logic [31:0] m_rd_data, m_data_to_dmem;
  logic [3:0] m_we_to_dmem;
  logic m_illegal_insn;

  always_comb begin
    if (wm_bypass_data) begin m_data_to_dmem = w_state.rd_data; end
    else m_data_to_dmem = m_state.data_dmem;
  end

  wire [31:0] addr_to_dmem;
  assign addr_to_dmem = {m_state.memory_address[31:2],2'b00};

  logic [31:0] load_data_from_dmem, store_data_to_dmem;
  logic [3:0] store_we_to_dmem;

  logic m_read, m_write;

  always_comb begin
    dcache.ARVALID = False;
    dcache.ARADDR = 0;
    dcache.AWVALID = False;
    dcache.AWADDR = 0;
    dcache.WDATA = 0;
    dcache.WSTRB = 0;
    dcache.WVALID = False;
    if (m_read) begin
      dcache.ARVALID = True;
      dcache.ARADDR = addr_to_dmem;
    end else if (m_write) begin
      dcache.AWVALID = True;
      dcache.AWADDR = addr_to_dmem;
      dcache.WDATA = store_data_to_dmem;
      dcache.WSTRB = store_we_to_dmem;
      dcache.WVALID = True;
    end
  end

  always_comb begin
    store_we_to_dmem = 0;
    m_illegal_insn = 0;

    store_data_to_dmem = 32'd0;
    store_we_to_dmem = 4'b0000;

    m_read = 0;
    m_write = 0;

    case (m_state.insn_ic)
      IClw, IClh, IClb, IClhu, IClbu: begin
        m_read = 1;
      end
      ICsw: begin
        m_write = 1;
        store_data_to_dmem = m_data_to_dmem[31:0];
        store_we_to_dmem = 4'b1111;
      end
      ICsh: begin
        m_write = 1;
        case (m_state.memory_address[1])
          default: begin m_illegal_insn = 1; end
          1'b0:begin
            store_we_to_dmem = 4'b0011;
            store_data_to_dmem[15:0] = m_data_to_dmem[15:0];
          end
          1'b1:begin
            store_we_to_dmem = 4'b1100;
            store_data_to_dmem[31:16] = m_data_to_dmem[15:0];
          end
        endcase
      end
      ICsb: begin
        m_write = 1;
        case (m_state.memory_address[1:0])
            default: begin m_illegal_insn = 1; end
            2'b00: begin
              store_we_to_dmem = 4'b0001;
              store_data_to_dmem[7:0] = m_data_to_dmem[7:0];
            end
            2'b01: begin
              store_we_to_dmem = 4'b0010;
              store_data_to_dmem[15:8] = m_data_to_dmem[7:0];  end
            2'b10: begin
              store_we_to_dmem = 4'b0100;
              store_data_to_dmem[23:16] = m_data_to_dmem[7:0];  end
            2'b11: begin
              store_we_to_dmem = 4'b1000;
              store_data_to_dmem[31:24] = m_data_to_dmem[7:0];
              end
            endcase
       end
      default: begin
        m_rd_data = m_state.rd_data;
        m_illegal_insn = 0;
        m_we_to_dmem = 0;
      end
    endcase
  end


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
        rd_data:0,
        illegal_insn:0,
        insn_ic:0,
        memory_address:0
      };
    end else begin
        w_state <= '{
          pc: m_state.pc,
          insn: m_state.insn,
          cycle_status: m_state.cycle_status,
          illegal_insn:m_illegal_insn,
          insn_ic: m_state.insn_ic,
          rd_data:m_rd_data,
          memory_address:m_state.memory_address
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


  logic we_to_reg, w_illegal_insn;
  logic [4:0] rd_to_reg;
  logic [`REG_SIZE] data_to_reg;

  assign rd_to_reg = w_state.insn[11:7];

  wire regfile_we;
  assign regfile_we = (w_state.insn == 32'h0 ||
                      w_state.insn[6:0] == 7'h63 ||
                      w_state.cycle_status == CYCLE_TAKEN_BRANCH ||
                      w_state.insn[6:0] == OpStore ||
                      w_state.insn[6:0] == OpBranch ||
                      w_state.insn[6:0] == OpMiscMem ||
                      w_state.insn[6:0] == OpEnviron) ? 1'b0 : 1'b1;

  // we need to stall the pipline if we are waiting for cache to get back with a load data
  logic writeback_waiting_for_cache;
  always_comb begin
    dcache.RREADY = False;
    dcache.BREADY = False;

    if (w_state.insn_ic == IClw || w_state.insn_ic == IClh || w_state.insn_ic == IClb
                || w_state.insn_ic == IClhu || w_state.insn_ic == IClbu) begin
      writeback_waiting_for_cache = !dcache.RVALID;
      if (dcache.RVALID) begin
        dcache.RREADY = True;
      end
    end else if (w_state.insn_ic == ICsw || w_state.insn_ic == ICsh || w_state.insn_ic == ICsb) begin
      // if we have written to cache, we expect a bvalid repsonse
      writeback_waiting_for_cache = !dcache.BVALID;
      if (dcache.BVALID) begin
        dcache.BREADY = True;
      end
    end else begin
      writeback_waiting_for_cache = 1'b0;
    end
  end

  always_comb begin
    w_illegal_insn = 0;
    if (regfile_we == 1) begin
      if (w_state.insn_ic == IClw || w_state.insn_ic == IClh || w_state.insn_ic == IClb
                || w_state.insn_ic == IClhu || w_state.insn_ic == IClbu) begin
        if (!writeback_waiting_for_cache && regfile_we) begin
          case(w_state.insn_ic)
            IClw: begin
              data_to_reg = dcache.RDATA;
            end
            IClh: begin
              case (w_state.memory_address[1])
                default: w_illegal_insn = 1;
                1'b0:  data_to_reg = {{16{dcache.RDATA[15]}}, dcache.RDATA[15:0]};
                1'b1:  data_to_reg = {{16{dcache.RDATA[31]}}, dcache.RDATA[31:16]};
              endcase
            end
            IClb: begin
              case (w_state.memory_address[1:0])
                default: w_illegal_insn = 1;
                2'b00:  data_to_reg = {{24{dcache.RDATA[ 7]}}, dcache.RDATA[7:0]};
                2'b01:  data_to_reg = {{24{dcache.RDATA[15]}}, dcache.RDATA[15:8]};
                2'b10:  data_to_reg = {{24{dcache.RDATA[23]}}, dcache.RDATA[23:16]};
                2'b11:  data_to_reg = {{24{dcache.RDATA[31]}}, dcache.RDATA[31:24]};
              endcase
            end
            IClhu: begin
              case (w_state.memory_address[1])
                default: w_illegal_insn = 1;
                1'b0:  data_to_reg = {16'b0, dcache.RDATA[15:0]};
                1'b1:  data_to_reg = {16'b0, dcache.RDATA[31:16]};
              endcase
            end
            IClbu: begin
              case (w_state.memory_address[1:0])
                default: w_illegal_insn = 1;
                2'b00: data_to_reg = {24'b0, dcache.RDATA[7:0]};
                2'b01: data_to_reg = {24'b0, dcache.RDATA[15:8]};
                2'b10: data_to_reg = {24'b0, dcache.RDATA[23:16]};
                2'b11: data_to_reg = {24'b0, dcache.RDATA[31:24]};
              endcase
            end
            default: begin
              data_to_reg = 32'd0;
              we_to_reg = 0;
            end
          endcase
        end
      end else begin
        data_to_reg = w_state.rd_data;
        we_to_reg = 1;
      end
    end else begin
      data_to_reg = 32'd0;
      we_to_reg = 0;
    end
  end

  assign halt = (m_state.insn[6:0] == 7'h73) & (m_state.insn[31:7] == 'b0);

    /*****************/
    /* BYPASS HANDLE */
    /*****************/
    wire mx_bypass_rs1;
    wire mx_bypass_rs2;
    wire wx_bypass_rs1;
    wire wx_bypass_rs2;
    wire wd_bypass_rs1;
    wire wd_bypass_rs2;
    wire wm_bypass_data;


    wire m_req_rd;
    wire m_req_rs2;
    wire w_req_rd;
    wire x_req_rs1;
    wire x_req_rs2;
    wire d_req_rs1;
    wire d_req_rs2;
    assign m_req_rd = m_state.insn[6:0] == OpLui    ||
                    m_state.insn[6:0] == OpAuipc  ||
                    m_state.insn[6:0] == OpRegImm ||
                    m_state.insn[6:0] == OpRegReg ||
                    m_state.insn[6:0] == OpLoad   ||
                    m_state.insn[6:0] == OpJal    ||
                    m_state.insn[6:0] == OpJalr;

    assign m_req_rs2 = m_state.insn[6:0] == OpStore;

    assign w_req_rd = w_state.insn[6:0] == OpLui    ||
                    w_state.insn[6:0] == OpAuipc  ||
                    w_state.insn[6:0] == OpRegImm ||
                    w_state.insn[6:0] == OpRegReg ||
                    w_state.insn[6:0] == OpLoad   ||
                    w_state.insn[6:0] == OpJal    ||
                    w_state.insn[6:0] == OpJalr;

    assign x_req_rs1 =  x_state.insn[6:0]==OpRegReg ||
                    (x_state.insn[6:0]==OpRegImm) ||
                    (x_state.insn[6:0]==OpBranch) ||
                    (x_state.insn[6:0]==OpStore)  ||
                    (x_state.insn[6:0]==OpLoad)   ||
                    (x_state.insn[6:0]==OpJalr);

    assign x_req_rs2 =  x_state.insn[6:0] == OpRegReg ||
                      x_state.insn[6:0] == OpStore  ||
                      x_state.insn[6:0] == OpBranch;

    assign d_req_rs1 =  d_insn_opcode == OpRegImm ||
                      d_insn_opcode == OpRegReg ||
                      d_insn_opcode == OpBranch ||
                      d_insn_opcode == OpLoad   ||
                      d_insn_opcode == OpStore  ||
                      d_insn_opcode == OpJalr;

    assign d_req_rs2 =  d_insn_opcode == OpRegReg ||
                      d_insn_opcode == OpStore  ||
                      d_insn_opcode == OpBranch;

   logic illegal_insn = x_state.insn_ic == ICIllegal || d_ic == ICIllegal ||
                        m_state.insn_ic == ICIllegal || m_illegal_insn || w_state.illegal_insn;

    logic [4:0] m_rd,m_rs2,x_rs1,x_rs2,x_rd;
    assign m_rd = m_state.insn[11:7];
    assign m_rs2 = m_state.insn[24:20];

    assign x_rs1 = x_state.insn[19:15];
    assign x_rs2 = x_state.insn[24:20];
    assign x_rd = x_state.insn[11:7];


    assign mx_bypass_rs1 = (x_rs1 == m_rd && illegal_insn == 1'b0 && m_rd != 5'd0 && m_req_rd && x_req_rs1);
    assign mx_bypass_rs2 = (x_rs2 == m_rd && illegal_insn == 1'b0 && m_rd != 5'd0 && m_req_rd && x_req_rs2);

    assign wx_bypass_rs1 = (x_rs1 == rd_to_reg && illegal_insn == 1'b0 && rd_to_reg != 5'd0 && w_req_rd && x_req_rs1);
    assign wx_bypass_rs2 = (x_rs2 == rd_to_reg && illegal_insn == 1'b0 && rd_to_reg != 5'd0 && w_req_rd && x_req_rs2);

    assign wd_bypass_rs2 = (d_insn_rs2 == rd_to_reg && rd_to_reg != 5'd0 && w_req_rd && d_req_rs2);
    assign wd_bypass_rs1 = (d_insn_rs1 == rd_to_reg && rd_to_reg != 5'd0 && w_req_rd && d_req_rs1);

    assign wm_bypass_data = (rd_to_reg == m_rs2 && w_req_rd && m_req_rs2);

  assign trace_writeback_pc = w_state.pc;
  assign trace_writeback_insn = w_state.insn;
  assign trace_writeback_cycle_status = w_state.cycle_status;

endmodule // DatapathPipelinedCache

module Processor (
    input wire                       clk,
    input wire                       rst,
    output logic                     halt,
    output wire [`REG_SIZE]          trace_writeback_pc,
    output wire [`INSN_SIZE]         trace_writeback_insn,
    output                           cycle_status_e trace_writeback_cycle_status
);

  // This wire is set by cocotb to the name of the currently-running test, to make it easier
  // to see what is going on in the waveforms.
  wire [(8*32)-1:0] test_case;

  axi_if axi_data_cache ();
  axi_if axi_insn_cache ();
  // memory is dual-ported, to connect to both I$ and D$
  axi_if axi_mem_ro ();
  axi_if axi_mem_rw ();

AxilMemory #(.NUM_WORDS(8192)) memory (
  .ACLK(clk),
  .ARESETn(~rst),
  .port_ro(axi_mem_ro.subord),
  .port_rw(axi_mem_rw.subord)
);

`ifdef ENABLE_INSN_CACHE
  AxilCache #(
    .BLOCK_SIZE_BITS(32),
    .NUM_SETS(16)) icache (
    .ACLK(clk),
    .ARESETn(~rst),
    .proc(axi_insn_cache.subord),
    .mem(axi_mem_ro.manager)
  );
`endif
`ifdef ENABLE_DATA_CACHE
  AxilCache #(
    .BLOCK_SIZE_BITS(32),
    .NUM_SETS(16)) dcache (
    .ACLK(clk),
    .ARESETn(~rst),
    .proc(axi_data_cache.subord),
    .mem(axi_mem_rw.manager)
  );
`endif

  DatapathPipelinedCache datapath (
      .clk(clk),
      .rst(rst),
`ifdef ENABLE_INSN_CACHE
      .icache(axi_insn_cache.manager),
`else
      .icache(axi_mem_ro.manager),
`endif
`ifdef ENABLE_DATA_CACHE
      .dcache(axi_data_cache.manager),
`else
      .dcache(axi_mem_rw.manager),
`endif
      .halt(halt),
      .trace_writeback_pc(trace_writeback_pc),
      .trace_writeback_insn(trace_writeback_insn),
      .trace_writeback_cycle_status(trace_writeback_cycle_status)
  );

endmodule

`timescale 1ns / 1ns

`define ADDR_WIDTH 32
`define DATA_WIDTH 32

interface axi_if #(
      parameter int ADDR_WIDTH = 32
    , parameter int DATA_WIDTH = 32
);
  logic                      ARREADY;
  logic                      ARVALID;
  logic [    ADDR_WIDTH-1:0] ARADDR;
  logic [               2:0] ARPROT;

  logic                      RREADY;
  logic                      RVALID;
  logic [    DATA_WIDTH-1:0] RDATA;
  logic [               1:0] RRESP;

  logic                      AWREADY;
  logic                      AWVALID;
  logic [    ADDR_WIDTH-1:0] AWADDR;
  logic [               2:0] AWPROT;

  logic                      WREADY;
  logic                      WVALID;
  logic [    DATA_WIDTH-1:0] WDATA;
  logic [(DATA_WIDTH/8)-1:0] WSTRB;

  logic                      BREADY;
  logic                      BVALID;
  logic [               1:0] BRESP;

  modport manager(
      input ARREADY, RVALID, RDATA, RRESP, AWREADY, WREADY, BVALID, BRESP,
      output ARVALID, ARADDR, ARPROT, RREADY, AWVALID, AWADDR, AWPROT, WVALID, WDATA, WSTRB, BREADY
  );
  modport subord(
      input ARVALID, ARADDR, ARPROT, RREADY, AWVALID, AWADDR, AWPROT, WVALID, WDATA, WSTRB, BREADY,
      output ARREADY, RVALID, RDATA, RRESP, AWREADY, WREADY, BVALID, BRESP
  );
endinterface

// [BR]RESP codes, from Section A 3.4.4 of AXI4 spec
`define RESP_OK 2'b00
`define RESP_SUBORDINATE_ERROR 2'b10
`define RESP_DECODE_ERROR 2'b11

/** This is a simple memory that uses the AXI-Lite interface. */
module AxilMemory #(
    parameter int NUM_WORDS = 1024
) (
    input wire ACLK,
    input wire ARESETn,
    axi_if.subord port_ro,
    axi_if.subord port_rw
);
  localparam bit True = 1'b1;
  localparam bit False = 1'b0;
  localparam int AddrLsb = 2;  // since memory elements are 4B
  localparam int AddrMsb = $clog2(NUM_WORDS) + AddrLsb - 1;

  logic [31:0] mem_array[NUM_WORDS];
  logic [31:0] ro_araddr;
  logic ro_araddr_valid;

  initial begin
`ifdef SYNTHESIS
    $readmemh("mem_initial_contents.hex", mem_array);
`endif
  end

  assign port_ro.RRESP = `RESP_OK;
  assign port_ro.BRESP = `RESP_OK;
  assign port_rw.RRESP = `RESP_OK;
  assign port_rw.BRESP = `RESP_OK;

  always_ff @(posedge ACLK) begin
    if (!ARESETn) begin
      ro_araddr <= 0;
      ro_araddr_valid <= False;

      port_ro.ARREADY <= True;
      port_ro.AWREADY <= False;
      port_ro.WREADY  <= False;
      port_ro.RVALID <= False;
      port_ro.RDATA <= 0;

      port_rw.ARREADY <= True;
      port_rw.AWREADY <= True;
      port_rw.WREADY  <= True;
      port_rw.RVALID <= False;
      port_rw.RDATA <= 0;
    end else begin

      // port_ro is read-only

      if (ro_araddr_valid) begin
        // there is a buffered read request
        if (port_ro.RREADY) begin
          // manager accepted our response, we generate next response
          port_ro.RVALID <= True;
          port_ro.RDATA  <= mem_array[ro_araddr[AddrMsb:AddrLsb]];
          ro_araddr <= 0;
          ro_araddr_valid <= False;
          port_ro.ARREADY <= True;
        end
      end else if (port_ro.ARVALID && port_ro.ARREADY) begin
        // we have accepted a read request
        if (port_ro.RVALID && !port_ro.RREADY) begin
          // We have sent a response but manager has not accepted it. Buffer the new read request.
          ro_araddr <= port_ro.ARADDR;
          ro_araddr_valid <= True;
          port_ro.ARREADY <= False;
        end else begin
          // We have sent a response and manager has accepted it. Or, we were not already sending a response.
          // Either way, send a response to the request we just accepted.
          port_ro.RVALID <= True;
          port_ro.RDATA  <= mem_array[port_ro.ARADDR[AddrMsb:AddrLsb]];
        end
      end else if (port_ro.RVALID && port_ro.RREADY) begin
        // No incoming request. We have sent a response and manager has accepted it
        port_ro.RVALID <= False;
        port_ro.RDATA  <= 0;
        port_ro.ARREADY <= True;
      end

      // port_rw is read-write

      // NB: we take a shortcut on port_rw because the manager will always be RREADY/BREADY
      // as 1) the datapath never stalls in the W stage and 2) the cache is always ready
      if (port_rw.ARVALID && port_rw.ARREADY) begin
        port_rw.RVALID <= True;
        port_rw.RDATA  <= mem_array[port_rw.ARADDR[AddrMsb:AddrLsb]];
      end else if (port_rw.RVALID) begin
        port_rw.RVALID <= False;
        port_rw.RDATA  <= 0;
      end

      if (port_rw.AWVALID && port_rw.AWREADY && port_rw.WVALID && port_rw.WREADY) begin
        if (port_rw.WSTRB[0]) begin
          mem_array[port_rw.AWADDR[AddrMsb:AddrLsb]][7:0] <= port_rw.WDATA[7:0];
        end
        if (port_rw.WSTRB[1]) begin
          mem_array[port_rw.AWADDR[AddrMsb:AddrLsb]][15:8] <= port_rw.WDATA[15:8];
        end
        if (port_rw.WSTRB[2]) begin
          mem_array[port_rw.AWADDR[AddrMsb:AddrLsb]][23:16] <= port_rw.WDATA[23:16];
        end
        if (port_rw.WSTRB[3]) begin
          mem_array[port_rw.AWADDR[AddrMsb:AddrLsb]][31:24] <= port_rw.WDATA[31:24];
        end
        port_rw.BVALID <= True;
      end else if (port_rw.BVALID) begin
        port_rw.BVALID <= False;
      end
    end
  end

endmodule

// States for cache state machine. You can change these if you want.
typedef enum {
  // cache can respond to an incoming request
  CACHE_AVAILABLE = 0,
  // cache miss, waiting for fill from memory
  CACHE_AWAIT_FILL_RESPONSE = 1,
  // cache miss, waiting for writeback to memory
  CACHE_AWAIT_WRITEBACK_RESPONSE = 2,
  // cache waiting for manager to accept response
  CACHE_AWAIT_MANAGER_READY = 3
} cache_state_t;

module AxilCache #(
    /** size of each cache block, in bits */
    parameter int BLOCK_SIZE_BITS = 32,
    /** number of blocks in each way of the cache */
    parameter int NUM_SETS = 4
) (
    input wire ACLK,
    input wire ARESETn,
    axi_if.subord  proc,
    axi_if.manager mem
);

  // TODO: calculate these
  localparam int BlockOffsetBits = 2;
  localparam int IndexBits = $clog2(NUM_SETS);
  localparam int TagBits = `ADDR_WIDTH - IndexBits - BlockOffsetBits;

  // cache state
  cache_state_t current_state;
  // main cache structures: do not rename as tests reference these names
  logic [BLOCK_SIZE_BITS-1:0] data[NUM_SETS];
  logic [TagBits-1:0] tag[NUM_SETS];
  logic [0:0] valid[NUM_SETS];
  logic [0:0] dirty[NUM_SETS];

  localparam bit True = 1'b1;
  localparam bit False = 1'b0;

  // initialize cache state to all zeroes
  genvar seti;
  for (seti = 0; seti < NUM_SETS; seti = seti + 1) begin : gen_cache_init
    initial begin
      valid[seti] = '0;
      dirty[seti] = '0;
      data[seti] = 0;
      tag[seti] = 0;
    end
  end

  always_comb begin
    // addresses should always be 4B-aligned
    assert (!proc.ARVALID || proc.ARADDR[1:0] == 2'b00);
    assert (proc.ARPROT == 3'd0);
    assert (!proc.AWVALID || proc.AWADDR[1:0] == 2'b00);
    assert (proc.AWPROT == 3'd0);
    // cache is single-ported
    assert (!(proc.ARVALID && (proc.AWVALID || proc.WVALID)));
  end
  // the cache never raises any errors
  assign proc.RRESP = `RESP_OK;
  assign proc.BRESP = `RESP_OK;
  // ------------------------
  // AXI-Lite Signal Summary (with driver annotation)
  // ------------------------

  // Global Signals
  // ACLK    | Global | Shared clock for all transactions
  // ARESETn | Global | Active-low reset

  // ------------------------------
  // Read Address (AR) Channel
  // ------------------------------
  // ARVALID | proc  | Asserts when proc issues a read request
  // ARREADY | cch   | Asserted by cache when ready to accept proc request
  // ARADDR  | proc  | Address to read from
  // ARPROT  | proc  | Protection bits (unused)

  // mem.ARVALID | cch  | Cache issues read to memory
  // mem.ARREADY | mem  | Memory ready to accept read
  // mem.ARADDR  | cch  | Read address sent to memory
  // mem.ARPROT  | cch  | Protection bits (unused)

  // ------------------------------
  // Read Data (R) Channel
  // ------------------------------
  // RVALID  | cch   | Cache has data to return to proc
  // RREADY  | proc  | Proc has accepted read data
  // RDATA   | cch   | Data returned to proc
  // RRESP   | cch   | Read response (OKAY, etc.)

  // mem.RVALID | mem  | Memory returns data
  // mem.RREADY | cch  | Cache ready to accept data
  // mem.RDATA  | mem  | Data from memory
  // mem.RRESP  | mem  | Response code

  // ------------------------------
  // Write Address (AW) Channel
  // ------------------------------
  // AWVALID | proc  | Proc issues write address
  // AWREADY | cch   | Cache ready to accept write address
  // AWADDR  | proc  | Address to write to
  // AWPROT  | proc  | Protection bits (unused)

  // mem.AWVALID | cch  | Cache sends write address to memory
  // mem.AWREADY | mem  | Memory ready to accept address
  // mem.AWADDR  | cch  | Write address to memory
  // mem.AWPROT  | cch  | Protection bits (unused)

  // ------------------------------
  // Write Data (W) Channel
  // ------------------------------
  // WVALID  | proc  | Proc issues write data
  // WREADY  | cch   | Cache ready to accept write data
  // WDATA   | proc  | Write data
  // WSTRB   | proc  | Byte write mask

  // mem.WVALID | cch  | Cache sends write data to memory
  // mem.WREADY | mem  | Memory ready to accept data
  // mem.WDATA  | cch  | Write data
  // mem.WSTRB  | cch  | Write mask

  // ------------------------------
  // Write Response (B) Channel
  // ------------------------------
  // BVALID  | cch   | Cache returns write response to proc
  // BREADY  | proc  | Proc ready to accept write response
  // BRESP   | cch   | Write response code

  // mem.BVALID | mem  | Memory returns write response
  // mem.BREADY | cch  | Cache ready to accept it
  // mem.BRESP  | mem  | Write response code

  // Legend:
  // proc = datapath/processor (AXI manager)
  // cch  = AxilCache (AXI subordinate to proc, manager to mem)
  // mem  = AxilMemory (AXI subordinate)

  // ------------------- //
  // Cache state machine //
  // ------------------- //

  // we define an always_ff block to maintain state
  // we use an always_comb block to calculate the next state and handle requests


  always_ff @(posedge ACLK) begin
    if (!ARESETn) begin
      current_state <= CACHE_AVAILABLE;

      proc.ARREADY <= True;

      proc_rvalid <= False;
      proc_rdata <= 0;

      proc.WREADY <= True;
      proc.BVALID <= False;
    end else begin
      current_state <= n_current_state;

      proc_rvalid <= n_proc_rvalid;
      proc_rdata <= n_proc_rdata;

      proc.ARREADY  <= n_proc_arready;
      proc.AWREADY  <= n_proc_awready;
      proc.WREADY   <= n_proc_wready;
      proc.BVALID   <= n_proc_bvalid;
    end
  end // always_ff

  // define next_state variables
  cache_state_t n_current_state;
  logic [BLOCK_SIZE_BITS-1:0] proc_rdata, n_proc_rdata;
  logic n_proc_arready, proc_rvalid, n_proc_rvalid, n_proc_awready, n_proc_wready, n_proc_bvalid;

  // define internal signals


  // Select correct address bits, depending on type of instruction
  wire [IndexBits-1:0] imm_index_read, imm_index_write, process_index;
  wire [TagBits-1:0] imm_tag_in_read, imm_tag_in_write, process_tag_in;
  wire [BLOCK_SIZE_BITS - 1:0] process_write_data;

  assign imm_index_read  = proc.ARADDR[IndexBits+BlockOffsetBits-1:BlockOffsetBits];
  assign imm_tag_in_read = proc.ARADDR[TagBits+IndexBits+BlockOffsetBits-1:IndexBits+BlockOffsetBits];

  assign imm_index_write  = proc.AWADDR[IndexBits+BlockOffsetBits-1:BlockOffsetBits];
  assign imm_tag_in_write = proc.AWADDR[TagBits+IndexBits+BlockOffsetBits-1:IndexBits+BlockOffsetBits];

  logic [IndexBits -1:0]      s_index_buffer, n_index_buffer, f_index_buffer, nf_index_buffer;
  logic [TagBits-1:0]         s_tag_buffer,   n_tag_buffer,   f_tag_buffer,   nf_tag_buffer;
  logic [BLOCK_SIZE_BITS-1:0] s_wdata_buffer, n_wdata_buffer, f_wdata_buffer, nf_wdata_buffer;
  logic [3:0]                 s_wstrb_buffer, n_wstrb_buffer, f_wstrb_buffer, nf_wstrb_buffer;
  logic                       s_write,        n_write,        f_write,        nf_write;

  logic we, wf_mem, dirty_out;
  logic [BLOCK_SIZE_BITS-1:0] data_out;
  logic [IndexBits-1:0] index_out;
  logic [3:0] wstrb_out;
  logic [TagBits-1:0] tag_out;

  // write to registers
  always_ff @(posedge ACLK) begin
    if (we) begin
      case (wstrb_out)
        4'b0001: data[index_out][7:0] <= data_out[7:0];
        4'b0010: data[index_out][15:8] <= data_out[15:8];
        4'b0100: data[index_out][23:16] <= data_out[23:16];
        4'b1000: data[index_out][31:24] <= data_out[31:24];
        4'b0011: data[index_out][15:0] <= data_out[15:0];
        4'b1100: data[index_out][31:16] <= data_out[31:16];
        4'b1111: data[index_out] <= data_out;
        default: begin end
      endcase
      if (wf_mem) begin
        //block is from memory
        tag[index_out] <= tag_out;
        valid[index_out] <= True;
      end
      dirty[index_out] <= dirty_out;
    end
  end

  // update buffers
  always_ff @(posedge ACLK) begin
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
    end else begin
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
  end

  always_comb begin
    // default values for all outputs
    n_current_state = current_state;
    n_proc_arready = proc.ARREADY;
    n_proc_rvalid = proc_rvalid;
    n_proc_rdata = proc_rdata;
    n_proc_awready = proc.AWREADY;
    n_proc_wready = proc.WREADY;
    n_proc_bvalid = proc.BVALID;
    proc.RVALID = proc_rvalid;
    proc.RDATA = proc_rdata;

    // default values for ram signals
    mem.ARVALID = False;
    mem.ARADDR = 0;
    mem.RREADY = False;

    mem.AWVALID = False;
    mem.AWADDR = 0;
    mem.WVALID = False;
    mem.WDATA = 0;
    mem.WSTRB = 0;
    mem.BREADY = False;

    we = 0;
    wf_mem = 0;
    data_out = 0;
    index_out = 0;
    wstrb_out = 0;
    tag_out = 0;
    // case on current state
    case (current_state)
      CACHE_AVAILABLE: begin
        n_proc_arready = True;
        n_proc_awready = True;
        n_proc_wready = True;
        // in this state we have no buffered requests - we respond to any incoming request
        if (proc.ARVALID && proc.ARREADY) begin
          if (valid[imm_index_read] && tag[imm_index_read] == imm_tag_in_read) begin
            // cache hit
            n_proc_rvalid = True;
            n_proc_rdata = data[imm_index_read];
            n_current_state = CACHE_AWAIT_MANAGER_READY;
          end else begin
            //Cache miss - we need to fetch the dirty block from memory
            //fill buffer
            nf_index_buffer = imm_index_read;
            nf_tag_buffer = imm_tag_in_read;
            nf_write = 0;

            if (dirty[imm_index_read]) begin
              //Write this block back to memory
              mem.AWVALID = True;
              mem.AWADDR = {tag[imm_index_read], imm_index_read, 2'b00};
              mem.WVALID = True;
              mem.WDATA = data[imm_index_read];
              mem.WSTRB = 4'b1111;
              mem.BREADY = True;

              mem.RREADY = False;
              n_current_state = CACHE_AWAIT_WRITEBACK_RESPONSE;
            end else begin
              mem.ARVALID = True;
              mem.ARADDR = {imm_tag_in_read, imm_index_read, 2'b00};
              mem.RREADY = True;
              n_current_state = CACHE_AWAIT_FILL_RESPONSE;
            end
          end
        end else if (proc.AWVALID && proc.WVALID && proc.ARREADY) begin
          // cache hit
          if (valid[imm_index_write] && tag[imm_index_write] == imm_tag_in_write) begin
            // cache hit
            we = 1;
            data_out = proc.WDATA;
            index_out = imm_index_write;
            wstrb_out = proc.WSTRB;
            dirty_out = proc.WSTRB != 0;

            n_proc_bvalid = True;
            n_current_state = CACHE_AWAIT_MANAGER_READY;
          end else begin
            // cache miss
            nf_index_buffer = imm_index_write;
            nf_tag_buffer = imm_tag_in_write;
            nf_wdata_buffer = proc.WDATA;
            nf_wstrb_buffer = proc.WSTRB;
            nf_write = 1;
            if (dirty[imm_index_write]) begin
              //Write this block back to memory
              mem.AWVALID = True;
              mem.AWADDR = {tag[imm_index_write], imm_index_write, 2'b00};
              mem.WVALID = True;
              mem.WDATA = data[imm_index_write];
              mem.WSTRB = 4'b1111;
              mem.BREADY = True;

              mem.RREADY = False;
              n_current_state = CACHE_AWAIT_WRITEBACK_RESPONSE;
            end else begin
              mem.ARVALID = True;
              mem.ARADDR = {imm_tag_in_write, imm_index_write, 2'b00};
              mem.RREADY = True;
              n_current_state = CACHE_AWAIT_FILL_RESPONSE;
            end
          end
        end else begin
          n_current_state = CACHE_AVAILABLE;
        end
      end

      CACHE_AWAIT_FILL_RESPONSE: begin
        mem.RREADY = True;
        if (mem.RVALID && mem.RREADY) begin
          // process incoming response
          if (f_write) begin
            case (f_wstrb_buffer)
              4'b0001: data_out = {mem.RDATA[31:8], f_wdata_buffer[7:0]};
              4'b0010: data_out = {mem.RDATA[31:16], f_wdata_buffer[15:8], mem.RDATA[7:0]};
              4'b0100: data_out = {mem.RDATA[31:24], f_wdata_buffer[23:16], mem.RDATA[15:0]};
              4'b1000: data_out = {f_wdata_buffer[31:24], mem.RDATA[23:0]};
              4'b0011: data_out = {mem.RDATA[31:16], f_wdata_buffer[15:0]};
              4'b1100: data_out = {f_wdata_buffer[31:16], mem.RDATA[15:0]};
              4'b1111: data_out = f_wdata_buffer;
              default: begin data_out = {mem.RDATA[31:8], f_wdata_buffer[7:0]}; end
            endcase
            n_proc_bvalid = True;
            dirty_out = f_wstrb_buffer != 0;
          end else begin
            data_out = mem.RDATA;

            if (!proc.RREADY) begin
              n_proc_rvalid = True;
              n_proc_rdata = mem.RDATA;
            end

            proc.RVALID = True;
            proc.RDATA = mem.RDATA;

            dirty_out = 0;
          end
          index_out = f_index_buffer;
          tag_out = f_tag_buffer;
          wstrb_out = 4'b1111;
          we = 1;
          wf_mem = 1;

          n_current_state = CACHE_AWAIT_MANAGER_READY;
        end else begin
          n_current_state = CACHE_AWAIT_FILL_RESPONSE;
        end

        // buffer any instructions recieved this cycle
        if (proc.ARVALID && proc.ARREADY) begin
          n_index_buffer = imm_index_read;
          n_tag_buffer = imm_tag_in_read;
          n_wdata_buffer = 0;
          n_wstrb_buffer = 0;
          n_write = 0;
          // we are now unable to accept any new requsts
          n_proc_arready = False;
          n_proc_awready = False;
          n_proc_wready = False;
        end else if (proc.AWVALID && proc.WVALID && proc.ARREADY) begin
          // process incoming response
          n_index_buffer = imm_index_write;
          n_tag_buffer = imm_tag_in_write;
          n_wdata_buffer = proc.WDATA;
          n_wstrb_buffer = proc.WSTRB;
          n_write = 1;
          // we are now unable to accept any new requsts
          n_proc_arready = False;
          n_proc_awready = False;
          n_proc_wready = False;
        end
      end

      CACHE_AWAIT_WRITEBACK_RESPONSE: begin
        mem.BREADY = True;

        if (mem.BVALID && mem.BREADY) begin
          n_current_state = CACHE_AWAIT_FILL_RESPONSE;

          mem.ARVALID = True;
          mem.ARADDR = {f_tag_buffer, f_index_buffer, 2'b00};
        end else begin
          n_current_state = CACHE_AWAIT_WRITEBACK_RESPONSE;
        end

        // buffer any instructions recieved this cycle
        if (proc.ARVALID && proc.ARREADY) begin
          n_index_buffer = imm_index_read;
          n_tag_buffer = imm_tag_in_read;
          n_wdata_buffer = 0;
          n_wstrb_buffer = 0;
          n_write = 0;
          // we are now unable to accept any new requsts
          n_proc_arready = False;
          n_proc_awready = False;
          n_proc_wready = False;
        end else if (proc.AWVALID && proc.WVALID && proc.ARREADY) begin
          // process incoming response
          n_index_buffer = imm_index_write;
          n_tag_buffer = imm_tag_in_write;
          n_wdata_buffer = proc.WDATA;
          n_wstrb_buffer = proc.WSTRB;
          n_write = 1;
          // we are now unable to accept any new requsts
          n_proc_arready = False;
          n_proc_awready = False;
          n_proc_wready = False;
        end
      end

      CACHE_AWAIT_MANAGER_READY: begin
        if (proc.RVALID && proc.RREADY) begin
          n_proc_rvalid = 0;
          n_proc_rdata = 0;
        end else if (proc.BVALID && proc.BREADY) begin
          n_proc_bvalid = 0;
        end else begin
          n_current_state = CACHE_AWAIT_MANAGER_READY;
        end

        if (proc.ARVALID && proc.ARREADY) begin
          if ((proc.RVALID && !proc.RREADY) || (proc.BVALID && !proc.BREADY)) begin
              // We have sent a response but manager has not accepted it. Buffer the new read request.
              n_index_buffer = imm_index_read;
              n_tag_buffer = imm_tag_in_read;
              n_wdata_buffer = 0;
              n_wstrb_buffer = 0;
              n_write = 0;
              // we are now unable to accept any new requsts
              n_proc_arready = False;
              n_proc_awready = False;
              n_proc_wready = False;
              n_current_state = CACHE_AWAIT_MANAGER_READY;
          end else begin
             if (valid[imm_index_read] && tag[imm_index_read] == imm_tag_in_read) begin
              // cache hit
              n_proc_rvalid = True;
              n_proc_rdata = data[imm_index_read];
              n_current_state = CACHE_AWAIT_MANAGER_READY;
            end else begin
              //Cache miss - we need to fetch the dirty block from memory
              //fill buffer
              nf_index_buffer = imm_index_read;
              nf_tag_buffer = imm_tag_in_read;
              nf_write = 0;

              if (dirty[imm_index_read]) begin
                //Write this block back to memory
                mem.AWVALID = True;
                mem.AWADDR = {tag[imm_index_read], imm_index_read, 2'b00};
                mem.WVALID = True;
                mem.WDATA = data[imm_index_read];
                mem.WSTRB = 4'b1111;
                mem.BREADY = True;

                mem.RREADY = False;
                n_current_state = CACHE_AWAIT_WRITEBACK_RESPONSE;
              end else begin
                mem.ARVALID = True;
                mem.ARADDR = {imm_tag_in_read, imm_index_read, 2'b00};
                mem.RREADY = True;
                n_current_state = CACHE_AWAIT_FILL_RESPONSE;
              end
            end
          end
        end else if (proc.AWVALID && proc.AWREADY && proc.WVALID && proc.WREADY) begin
          // process incoming response
          if ((proc.RVALID && !proc.RREADY) || (proc.BVALID && !proc.BREADY)) begin
            // We have sent a response but manager has not accepted it. Buffer the new write request.
            n_index_buffer = imm_index_write;
            n_tag_buffer = imm_tag_in_write;
            n_wdata_buffer = proc.WDATA;
            n_wstrb_buffer = proc.WSTRB;
            n_write = 1;
            // we are now unable to accept any new requsts
            n_proc_arready = False;
            n_proc_awready = False;
            n_proc_wready = False;

            n_current_state = CACHE_AWAIT_MANAGER_READY;
          end else begin
            // cache hit
            if (valid[imm_index_write] && tag[imm_index_write] == imm_tag_in_write) begin
              // cache hit
              we = 1;
              data_out = proc.WDATA;
              index_out = imm_index_write;
              wstrb_out = proc.WSTRB;

              n_proc_bvalid = True;
              n_current_state = CACHE_AWAIT_MANAGER_READY;
            end else begin
              // cache miss
              nf_index_buffer = imm_index_write;
              nf_tag_buffer = imm_tag_in_write;
              nf_wdata_buffer = proc.WDATA;
              nf_wstrb_buffer = proc.WSTRB;
              nf_write = 1;
              if (dirty[imm_index_write]) begin
                //Write this block back to memory
                mem.AWVALID = True;
                mem.ARADDR = {tag[imm_index_write], imm_index_write, 2'b00};
                mem.WVALID = True;
                mem.WDATA = data[imm_index_write];
                mem.WSTRB = 4'b1111;

                n_current_state = CACHE_AWAIT_WRITEBACK_RESPONSE;
              end else begin
                mem.ARVALID = True;
                mem.ARADDR = {imm_tag_in_write, imm_index_write, 2'b00};
                n_current_state = CACHE_AWAIT_FILL_RESPONSE;
              end
            end
          end
        end else if (!proc.ARREADY && !proc.AWREADY && !proc.WREADY) begin
          // we are unable to accept new responses => we have a buffered request
          if ((proc.RVALID && !proc.RREADY) || (proc.BVALID && !proc.BREADY)) begin
            //still waiting for manager - maintain the buffers
            n_index_buffer = s_index_buffer;
            n_tag_buffer = s_tag_buffer;
            n_wdata_buffer = s_wdata_buffer;
            n_wstrb_buffer = s_wstrb_buffer;
            n_write = s_write;

            n_proc_arready = False;
            n_proc_awready = False;
            n_proc_wready = False;

            n_current_state = CACHE_AWAIT_MANAGER_READY;
          end else begin
            //manager accepted response - process buffered request
            if (s_write) begin
              // write
              if (valid[s_index_buffer] && tag[s_index_buffer] == s_tag_buffer) begin
                // cache hit
                we = 1;
                data_out = s_wdata_buffer;
                index_out = s_index_buffer;
                wstrb_out = s_wstrb_buffer;
                dirty_out = s_wstrb_buffer != 0;

                n_proc_bvalid = True;
                n_current_state = CACHE_AWAIT_MANAGER_READY;
              end else begin
                // cache miss
                nf_index_buffer = s_index_buffer;
                nf_tag_buffer = s_tag_buffer;
                nf_wdata_buffer = s_wdata_buffer;
                nf_wstrb_buffer = s_wstrb_buffer;
                nf_write = 1;
                if (dirty[s_index_buffer]) begin
                  //Write this block back to memory
                  mem.AWVALID = True;
                  mem.AWADDR = {tag[s_index_buffer], s_index_buffer, 2'b00};
                  mem.WVALID = True;
                  mem.WDATA = data[s_index_buffer];
                  mem.WSTRB = 4'b1111;

                  mem.RREADY = False;
                  n_current_state = CACHE_AWAIT_WRITEBACK_RESPONSE;
                end else begin
                  mem.ARVALID = True;
                  mem.ARADDR = {s_tag_buffer, s_index_buffer, 2'b00};
                  n_current_state = CACHE_AWAIT_FILL_RESPONSE;
                end
              end
            end else begin
              // read
              if (valid[s_index_buffer] && tag[s_index_buffer] == s_tag_buffer) begin
                // cache hit
                n_proc_rvalid = True;
                n_proc_rdata = data[s_index_buffer];
                n_current_state = CACHE_AWAIT_MANAGER_READY;
              end else begin
                //Cache miss - we need to fetch the dirty block from memory
                //fill buffer
                nf_index_buffer = s_index_buffer;
                nf_tag_buffer = s_tag_buffer;
                nf_write = 0;

                if (dirty[s_index_buffer]) begin
                  //Write this block back to memory
                  mem.AWVALID = True;
                  mem.AWADDR = {tag[s_index_buffer], s_index_buffer, 2'b00};
                  mem.WVALID = True;
                  mem.WDATA = data[s_index_buffer];
                  mem.WSTRB = 4'b1111;

                  n_current_state = CACHE_AWAIT_WRITEBACK_RESPONSE;
                end else begin
                  mem.ARVALID = True;
                  mem.ARADDR = {s_tag_buffer, s_index_buffer, 2'b00};

                  n_current_state = CACHE_AWAIT_FILL_RESPONSE;
                end
              end

            end
            //zero the buffers
            n_index_buffer = 0;
            n_tag_buffer = 0;
            n_wdata_buffer = 0;
            n_wstrb_buffer = 0;
            n_write = 0;

            n_proc_arready = True;
            n_proc_awready = True;
            n_proc_wready = True;
          end
        end else begin
          // no buffered request, no incoming request - if manager is ready, we are now cache available
          if ((proc.RVALID && proc.RREADY) || (proc.BVALID && proc.BREADY)) begin
            n_current_state = CACHE_AVAILABLE;
          end
        end
      end
      default: begin end
    endcase
  end
endmodule // AxilCache

`ifndef SYNTHESIS
/** This is used for testing AxilCache in simulation. Since Verilator doesn't allow
SV interfaces in a top-level module, we wrap the interfaces with plain wires. */
module AxilCacheTester #(
    // these parameters are for the AXIL interface
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 32,
    // these parameters are for the cache
    parameter int BLOCK_SIZE_BITS = 32,
    parameter int NUM_SETS = 4
) (
    input wire ACLK,
    input wire ARESETn,

    input  wire                       CACHE_ARVALID,
    output logic                      CACHE_ARREADY,
    input  wire  [    ADDR_WIDTH-1:0] CACHE_ARADDR,
    input  wire  [               2:0] CACHE_ARPROT,
    output logic                      CACHE_RVALID,
    input  wire                       CACHE_RREADY,
    output logic [    ADDR_WIDTH-1:0] CACHE_RDATA,
    output logic [               1:0] CACHE_RRESP,
    input  wire                       CACHE_AWVALID,
    output logic                      CACHE_AWREADY,
    input  wire  [    ADDR_WIDTH-1:0] CACHE_AWADDR,
    input  wire  [               2:0] CACHE_AWPROT,
    input  wire                       CACHE_WVALID,
    output logic                      CACHE_WREADY,
    input  wire  [    DATA_WIDTH-1:0] CACHE_WDATA,
    input  wire  [(DATA_WIDTH/8)-1:0] CACHE_WSTRB,
    output logic                      CACHE_BVALID,
    input  wire                       CACHE_BREADY,
    output logic [               1:0] CACHE_BRESP,

    output wire                       MEM_ARVALID,
    input  logic                      MEM_ARREADY,
    output wire  [    ADDR_WIDTH-1:0] MEM_ARADDR,
    output wire  [               2:0] MEM_ARPROT,
    input  logic                      MEM_RVALID,
    output wire                       MEM_RREADY,
    input  logic [    ADDR_WIDTH-1:0] MEM_RDATA,
    input  logic [               1:0] MEM_RRESP,
    output wire                       MEM_AWVALID,
    input  logic                      MEM_AWREADY,
    output wire  [    ADDR_WIDTH-1:0] MEM_AWADDR,
    output wire  [               2:0] MEM_AWPROT,
    output wire                       MEM_WVALID,
    input  logic                      MEM_WREADY,
    output wire  [    DATA_WIDTH-1:0] MEM_WDATA,
    output wire  [(DATA_WIDTH/8)-1:0] MEM_WSTRB,
    input  logic                      MEM_BVALID,
    output wire                       MEM_BREADY,
    input  logic [               1:0] MEM_BRESP
);

  axi_if #(
      .ADDR_WIDTH(ADDR_WIDTH),
      .DATA_WIDTH(DATA_WIDTH)
  ) cache_axi ();
  assign cache_axi.manager.ARVALID = CACHE_ARVALID;
  assign CACHE_ARREADY = cache_axi.manager.ARREADY;
  assign cache_axi.manager.ARADDR = CACHE_ARADDR;
  assign cache_axi.manager.ARPROT = CACHE_ARPROT;
  assign CACHE_RVALID = cache_axi.manager.RVALID;
  assign cache_axi.manager.RREADY = CACHE_RREADY;
  assign CACHE_RRESP = cache_axi.manager.RRESP;
  assign CACHE_RDATA = cache_axi.manager.RDATA;
  assign cache_axi.manager.AWVALID = CACHE_AWVALID;
  assign CACHE_AWREADY = cache_axi.manager.AWREADY;
  assign cache_axi.manager.AWADDR = CACHE_AWADDR;
  assign cache_axi.manager.AWPROT = CACHE_AWPROT;
  assign cache_axi.manager.WVALID = CACHE_WVALID;
  assign CACHE_WREADY = cache_axi.manager.WREADY;
  assign cache_axi.manager.WDATA = CACHE_WDATA;
  assign cache_axi.manager.WSTRB = CACHE_WSTRB;
  assign CACHE_BVALID = cache_axi.manager.BVALID;
  assign cache_axi.manager.BREADY = CACHE_BREADY;
  assign CACHE_BRESP = cache_axi.manager.BRESP;

  axi_if #(
      .ADDR_WIDTH(ADDR_WIDTH),
      .DATA_WIDTH(DATA_WIDTH)
  ) mem_axi ();
   assign MEM_ARVALID = mem_axi.subord.ARVALID;
   assign mem_axi.subord.ARREADY = MEM_ARREADY;
   assign MEM_ARADDR = mem_axi.subord.ARADDR;
   assign MEM_ARPROT = mem_axi.subord.ARPROT;
   assign mem_axi.subord.RVALID = MEM_RVALID;
   assign MEM_RREADY = mem_axi.subord.RREADY;
   assign mem_axi.subord.RRESP = MEM_RRESP;
   assign mem_axi.subord.RDATA = MEM_RDATA;
   assign MEM_AWVALID = mem_axi.subord.AWVALID;
   assign mem_axi.subord.AWREADY = MEM_AWREADY;
   assign MEM_AWADDR = mem_axi.subord.AWADDR;
   assign MEM_AWPROT = mem_axi.subord.AWPROT;
   assign MEM_WVALID = mem_axi.subord.WVALID;
   assign mem_axi.subord.WREADY = MEM_WREADY;
   assign MEM_WDATA = mem_axi.subord.WDATA;
   assign MEM_WSTRB = mem_axi.subord.WSTRB;
   assign mem_axi.subord.BVALID = MEM_BVALID;
   assign MEM_BREADY = mem_axi.subord.BREADY;
   assign mem_axi.subord.BRESP = MEM_BRESP;

  AxilCache #(
    .BLOCK_SIZE_BITS(BLOCK_SIZE_BITS),
    .NUM_SETS(NUM_SETS)
  ) cache (
      .ACLK(ACLK),
      .ARESETn(ARESETn),
      .proc(cache_axi.subord),
      .mem(mem_axi.manager)
  );
endmodule // AxilCacheTester
`endif

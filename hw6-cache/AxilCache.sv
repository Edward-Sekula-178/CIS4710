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

  // Select correct address bits, depending on type of instruction
  wire [IndexBits-1:0] imm_index_read, imm_index_write, process_index;
  wire [TagBits-1:0] imm_tag_in_read, imm_tag_in_write, process_tag_in;
  wire [BLOCK_SIZE_BITS - 1:0] process_write_data;

  assign imm_index_read  = proc.ARADDR[IndexBits+BlockOffsetBits-1:BlockOffsetBits];
  assign imm_tag_in_read = proc.ARADDR[TagBits+IndexBits+BlockOffsetBits-1:IndexBits+BlockOffsetBits];

  assign imm_index_write  = proc.AWADDR[IndexBits+BlockOffsetBits-1:BlockOffsetBits];
  assign imm_tag_in_write = proc.AWADDR[TagBits+IndexBits+BlockOffsetBits-1:IndexBits+BlockOffsetBits];

  logic [IndexBits -1:0] curr_index, fill_index;
  logic [TagBits-1:0] curr_tag_in, fill_tag_in;
  logic [BLOCK_SIZE_BITS-1:0] curr_write_data, fill_write_data;
  logic is_write_operation, fill_write_operation;
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

  always_ff @(posedge ACLK) begin
    if (!ARESETn) begin // NB: reset when ARESETn == 0
      current_state <= CACHE_AVAILABLE;
      // Initialize the AXI interface signals
      proc.ARREADY <= 1'b1;     // Ready to accept read addresses
      proc.AWREADY <= 1'b1;     // Ready to accept write addresses
      proc.WREADY <= 1'b1;      // Ready to accept write data
      proc.RVALID <= 1'b0;      // No read response ready yet
      proc.RDATA <= 32'b0;      // No read data ready yet
      proc.BVALID <= 1'b0;      // No write response ready yet

      // Initialize memory interface signals
      mem.ARVALID <= 1'b0;      // No read request to memory yet
      mem.AWVALID <= 1'b0;      // No write request to memory yet
      mem.WVALID <= 1'b0;       // No write data to memory yet
      mem.RREADY <= 1'b1;       // Ready to accept read responses from memory
      mem.BREADY <= 1'b1;       // Ready to accept write responses from memory
    end else begin
      case (current_state)
        CACHE_AVAILABLE: begin
          if (proc.ARVALID && proc.ARREADY) begin
            // check if cache-hit
            if (valid[imm_index_read] && tag[imm_index_read]== imm_tag_in_read) begin
              proc.RVALID <= 1;
              proc.RDATA <= data[imm_index_read];
              current_state <= CACHE_AWAIT_MANAGER_READY; // wait for manager to accept response
            end else begin
              // We have a chache-miss so fill from mem
              current_state <= CACHE_AWAIT_FILL_RESPONSE;

              fill_index <= imm_index_read;
              fill_tag_in <= imm_tag_in_read;
              fill_write_data <= 0;
              fill_write_operation <= 0;

              if (dirty[imm_index_read]) begin
                // we need to write the dirty block back to memory
                mem.AWVALID <= 1;
                mem.AWADDR <= {tag[imm_index_read],imm_index_read, 2'b00};
                mem.WVALID <= 1;
                mem.WDATA <= data[imm_index_read];
                mem.WSTRB <= 4'b1111;
                mem.BREADY <= 1;
                current_state <= CACHE_AWAIT_WRITEBACK_RESPONSE; // wait for writeback response
              end else begin
                // request block from memory
                mem.ARVALID <= 1;
                mem.ARADDR <= {imm_tag_in_read, imm_index_read, 2'b00};
                mem.RREADY <= 1;
                current_state <= CACHE_AWAIT_FILL_RESPONSE;//wait for memory response
                end
            end
          end else if (proc.AWVALID && proc.AWREADY && proc.WVALID && proc.WREADY) begin //N.B. data passed at same time as address
            // write request from processor
            // check cache-hit
            if (valid[imm_index_write] && tag[imm_index_write]== imm_tag_in_write) begin
              data[imm_index_write] <= proc.WDATA;
              dirty[imm_index_write] <= 1;
              proc.BVALID <= 1;
              current_state <= CACHE_AWAIT_MANAGER_READY;
            end else begin
              // we have a cache-miss - if the block is dirty we must wright back to mem.
              // If it is clean, we only need to adjust the record of what memory is in the cache - because we are overwrighting anyway
              if (dirty[imm_index_write]) begin
                // we need to write the dirty block back to memory
                mem.AWVALID <= 1;
                mem.AWADDR <= {tag[imm_index_write], imm_index_write, 2'b00}; // address to write back
                mem.WVALID <= 1; // write data to memory
                mem.WDATA <= data[imm_index_write]; // data to write back
                mem.WSTRB <= 4'b1111; // write all bytes
                mem.BREADY <= 1;

                fill_index <= imm_index_write;
                fill_tag_in <= imm_tag_in_write;
                fill_write_data <= proc.WDATA;
                fill_write_operation <= 1;

                current_state <= CACHE_AWAIT_WRITEBACK_RESPONSE; // wait for writeback response
              end else begin
                // we adjust the record of what memory is in the cache and write to cache

                data[imm_index_write] <= proc.WDATA;
                tag[imm_index_write] <= imm_tag_in_write; // set the tag for the block
                valid[imm_index_write] <= 1; // mark the tag as valid
                dirty[imm_index_write] <= 1; // mark the block as dirty

                data[imm_index_write] <= proc.WDATA;
                dirty[imm_index_write] <= 1;
                proc.BVALID <= 1; // write response to processor
                current_state <= CACHE_AWAIT_MANAGER_READY; // wait for manager to accept response
              end
            end
          end
          proc.ARREADY <= 1; // ready to accept another request
          proc.AWREADY <= 1;
          proc.WREADY <= 1;
        end

        CACHE_AWAIT_FILL_RESPONSE: begin
          // we are now awaiting a fill response from memory. This is only triggered by an r instruction instruction

          if (mem.RVALID && mem.RREADY) begin
            // Memory has sent back valid data and we are ready for it
            mem.RREADY <= 0;
            data[fill_index] <= mem.RDATA;
            tag[fill_index] <= fill_tag_in; // set the tag for the block
            valid[fill_index] <= 1; // mark the tag as valid
            dirty[fill_index] <= 0; // mark the block as dirty if it was a write operation

            // we will only be waiting for a fill response if it is a read
            // so we can savely process only a read
            proc.RVALID <= 1;
            proc.RDATA <= mem.RDATA;
            current_state <= CACHE_AWAIT_MANAGER_READY; // wait for manager to accept response
          end else begin
            // we are still waiting on memory response
            // ensure mem-rready is 1
            mem.RREADY <= 1;
            current_state <= CACHE_AWAIT_FILL_RESPONSE;
          end

          // we need to buffer any new requests we get in the meantime
          if (proc.ARREADY && proc.AWREADY && proc.WREADY) begin
            if (proc.ARVALID && proc.ARREADY) begin
              curr_index <= imm_index_read;
              curr_tag_in <= imm_tag_in_read;
              is_write_operation <= 0; // this is a read request

              proc.ARREADY <= 0;
              proc.AWREADY <= 0;
              proc.WREADY <= 0;
            end else if (proc.AWVALID && proc.AWREADY) begin
              curr_index <= imm_index_write;
              curr_tag_in <= imm_tag_in_write;
              curr_write_data <= proc.WDATA;
              is_write_operation <= 1; // this is a write request

              proc.ARREADY <= 0;
              proc.AWREADY <= 0;
              proc.WREADY <= 0;
            end
          end
        end
        CACHE_AWAIT_WRITEBACK_RESPONSE: begin
          // if we are in this state, we are ready for writeback
          // handshake
          if (mem.BVALID && mem.BREADY) begin
            // Memory accepted our writeback
            mem.BREADY <= 0;
            dirty[fill_index] <= 0; // mark the block as clean

            // we now schedule read in the new block if its a read instruction. Finish processing write o.w.
            if (fill_write_operation) begin
              data[fill_index] <= fill_write_data;
              tag[fill_index] <= fill_tag_in;
              valid[fill_index] <= 1;
              dirty[fill_index] <= 1;
              proc.BVALID <= 1;
              current_state <= CACHE_AWAIT_MANAGER_READY; // wait for manager to accept response
            end else begin
              mem.ARVALID <= 1;
              mem.ARADDR <= {fill_tag_in, fill_index, 2'b00};
              mem.RREADY <= 1;
              current_state <= CACHE_AWAIT_FILL_RESPONSE;
            end
          end else begin
            // we are still waiting on memory response
            current_state <= CACHE_AWAIT_WRITEBACK_RESPONSE;
          end

          // we need to buffer any new requests we get in the meantime
          if (proc.ARREADY && proc.AWREADY && proc.WREADY) begin
            if (proc.ARVALID && proc.ARREADY) begin
              curr_index <= imm_index_read;
              curr_tag_in <= imm_tag_in_read;
              is_write_operation <= 0; // this is a read request

              proc.ARREADY <= 0;
              proc.AWREADY <= 0;
              proc.WREADY <= 0;
            end else if (proc.AWVALID && proc.AWREADY) begin
              curr_index <= imm_index_write;
              curr_tag_in <= imm_tag_in_write;
              curr_write_data <= proc.WDATA;
              is_write_operation <= 1; // this is a write request

              proc.ARREADY <= 0;
              proc.AWREADY <= 0;
              proc.WREADY <= 0;
            end
          end
          // N.B. we only hit this state if we have a missed w/r request where cache block is dirty
          // thus at this point we are already processing an instruction so we require no processing here
        end

        CACHE_AWAIT_MANAGER_READY: begin
          // Read response handshake
          if (proc.RVALID && proc.RREADY) begin
              // Manager accepted our read response
              proc.RVALID <= 0;
              proc.RDATA <= 0;
          end else if (proc.BVALID && proc.BREADY) begin
            if (proc.BREADY) begin
              proc.BVALID <= 0;
            end
          end else begin
              // we are still waiting on the manager
              current_state <= CACHE_AWAIT_MANAGER_READY;
          end

          // if there is a new request and we are ready for it, then either buffer or process it
          if (proc.ARVALID && proc.ARREADY) begin
            if ((proc.RVALID && !proc.RREADY) || (proc.BVALID && !proc.BREADY)) begin
              // We have sent a response but manager has not accepted it. Buffer the new read request.
              curr_index <= imm_index_read;
              curr_tag_in <= imm_tag_in_read;
              current_state <= CACHE_AWAIT_MANAGER_READY; // wait for manager to accept response
              // we are now unable to accept any new requsts
              proc.ARREADY <= 0;
              proc.AWREADY <= 0;
              proc.WREADY <= 0;
            end else begin
              if (valid[imm_index_read] && tag[imm_index_read]== imm_tag_in_read) begin
                proc.RVALID <= 1;
                proc.RDATA <= data[imm_index_read];
                current_state <= CACHE_AWAIT_MANAGER_READY; // wait for manager to accept response
              end else begin
                // We have a chache-miss so fill from mem
                current_state <= CACHE_AWAIT_FILL_RESPONSE;
                // we need to use the fill-buffer
                fill_index <= imm_index_read;
                fill_tag_in <= imm_tag_in_read;
                fill_write_data <= 0;
                fill_write_operation <= 0;

                if (dirty[imm_index_read]) begin
                  mem.AWVALID <= 1;
                  mem.AWADDR <= {tag[imm_index_read],imm_index_read, 2'b00};
                  mem.WVALID <= 1; // write data to memory
                  mem.WDATA <= data[imm_index_read]; // data to write back
                  mem.WSTRB <= 4'b1111; // write all bytes
                  mem.BREADY <= 1;
                  current_state <= CACHE_AWAIT_WRITEBACK_RESPONSE; // wait for writeback response
                end else begin current_state <= CACHE_AWAIT_FILL_RESPONSE; end
              end
            end
          end else if (proc.AWVALID && proc.AWREADY) begin
            if ((proc.RVALID && !proc.RREADY) || (proc.BVALID && !proc.BREADY)) begin
              // We have sent a response but manager has not accepted it. Buffer the new read request.
              curr_index <= imm_index_write;
              curr_tag_in <= imm_tag_in_write;
              curr_write_data <= proc.WDATA;
              current_state <= CACHE_AWAIT_MANAGER_READY; // wait for manager to accept response
              // we are now unable to accept any new requsts
              proc.ARREADY <= 0;
              proc.AWREADY <= 0;
              proc.WREADY <= 0;
            end else begin
              // write request from processor
              // check cache-hit
              if (valid[imm_index_write] && tag[imm_index_write]== imm_tag_in_write) begin
                data[imm_index_write] <= proc.WDATA;
                dirty[imm_index_write] <= 1;
                proc.BVALID <= 1;
                current_state <= CACHE_AWAIT_MANAGER_READY;
              end else begin
                // we have a cache-miss - if the block is dirty we must wright back to mem.
                // If it is clean, we only need to adjust the record of what memory is in the cache - because we are overwrighting anyway
                if (dirty[imm_index_write]) begin
                  // we need to write the dirty block back to memory
                  mem.AWVALID <= 1;
                  mem.AWADDR <= {tag[imm_index_write], imm_index_write, 2'b00}; // address to write back
                  mem.WVALID <= 1; // write data to memory
                  mem.WDATA <= data[imm_index_write]; // data to write back
                  mem.WSTRB <= 4'b1111; // write all bytes
                  mem.BREADY <= 1;

                  fill_index <= imm_index_write;
                  fill_tag_in <= imm_tag_in_write;
                  fill_write_data <= proc.WDATA;
                  fill_write_operation <= 1;

                  current_state <= CACHE_AWAIT_WRITEBACK_RESPONSE; // wait for writeback response
                end else begin
                  // we adjust the record of what memory is in the cache and write to cache
                  data[imm_index_write] <= proc.WDATA;
                  tag[imm_index_write] <= imm_tag_in_write; // set the tag for the block
                  valid[imm_index_write] <= 1; // mark the tag as valid
                  dirty[imm_index_write] <= 1; // mark the block as dirty

                  data[imm_index_write] <= proc.WDATA;
                  dirty[imm_index_write] <= 1;
                  proc.BVALID <= 1; // write response to processor
                  current_state <= CACHE_AWAIT_MANAGER_READY; // wait for manager to accept response
                end
              end
            end
          end else if (!proc.ARREADY && !proc.AWREADY && !proc.WREADY) begin
            // We are not ready to accept any new requests - if manager responded we process buffered request
            if ((proc.RVALID && proc.RREADY) || (proc.BVALID && proc.BREADY)) begin
              // Manager accepted our response, we process buffered instruction
              if (is_write_operation) begin
                // write request from processor
                // check cache-hit
                if (valid[curr_index] && tag[curr_index]== curr_tag_in) begin
                  data[curr_index] <= curr_write_data;
                  dirty[curr_index] <= 1;
                  proc.BVALID <= 1;
                  current_state <= CACHE_AWAIT_MANAGER_READY;
                end else begin
                  // we have a cache-miss - if the block is dirty we must wright back to mem.
                  // If it is clean, we only need to adjust the record of what memory is in the cache - because we are overwrighting anyway
                  if (dirty[curr_index]) begin
                    // we need to write the dirty block back to memory
                    mem.AWVALID <= 1;
                    mem.AWADDR <= {tag[curr_index], curr_index, 2'b00}; // address to write back
                    mem.WVALID <= 1; // write data to memory
                    mem.WDATA <= data[curr_index]; // data to write back
                    mem.WSTRB <= 4'b1111; // write all bytes
                    mem.BREADY <= 1;

                    fill_index <= curr_index;
                    fill_tag_in <= curr_tag_in;
                    fill_write_data <= proc.WDATA;
                    fill_write_operation <= 1;

                    current_state <= CACHE_AWAIT_WRITEBACK_RESPONSE; // wait for writeback response
                  end else begin
                    // we adjust the record of what memory is in the cache and write to cache
                    data[curr_index] <= proc.WDATA;
                    tag[curr_index] <= curr_tag_in; // set the tag for the block
                    valid[curr_index] <= 1; // mark the tag as valid
                    dirty[curr_index] <= 1; // mark the block as dirty

                    data[curr_index] <= proc.WDATA;
                    dirty[curr_index] <= 1;
                    proc.BVALID <= 1; // write response to processor
                    current_state <= CACHE_AWAIT_MANAGER_READY; // wait for manager to accept response
                  end
                end
              end else begin
                // cache hit assumed, return data
                proc.RVALID <= 1;
                proc.RDATA <= data[curr_index];
              end
              // reset the buffer
              curr_index <= 0;
              curr_tag_in <= 0;
              curr_write_data <= 0;

              proc.ARREADY <= 1; // ready to accept another request
              proc.AWREADY <= 1;
              proc.WREADY <= 1;
              // go back to waiting for manager to accept response
              current_state <= CACHE_AWAIT_MANAGER_READY;
            end else begin
              current_state <= CACHE_AWAIT_MANAGER_READY; // wait for manager to accept response
            end
          end else begin
            // No new request and we had no buffered requests we go back to
            // normal state iff proc accepted return
            if ((proc.RVALID && proc.RREADY)|| (proc.BVALID && proc.BREADY)) begin
              current_state <= CACHE_AVAILABLE;
            end
          end
        end

        default: begin
          current_state <= CACHE_AVAILABLE; // reset to available on any unknown state.
        end

      endcase // case (current_state)
    end // else: !if(!ARESETn)
  end // always_ff

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

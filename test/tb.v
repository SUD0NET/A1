`default_nettype none
`timescale 1ns / 1ps

/*
 * Testbench wrapper for tt_um_a1 (A1 MPU), driven by cocotb (test.py).
 * Models external hardware of A1 system:
 *   - 2x 74xx573 transparent latches (ALE -> A[7:0], ALEH -> A[15:8])
 *   - 8 KB byte-wide SRAM, loaded from nuc.hex (SwapForth nucleus)
 *   - optional READY wait-state injector
 * and watches the chip from outside:
 *   - bus protocol checker (violation counter, readable from cocotb)
 *   - UART TX monitor (decoded bytes -> txline / txn)
 * Only DUT ports are used, so this also works for gate-level simulation.
 */
module tb ();

  // Dump signals to FST file. You can view it with gtkwave or surfer.
  initial begin
    $dumpfile("tb.fst");
    $dumpvars(0, tb);
    #1;
  end

  // DUT hookup
  reg        clk   = 1'b0;
  reg        rst_n = 1'b0;
  reg        ena   = 1'b1;
  reg        rxd   = 1'b1; // UART RX into the chip (ui_in[1])
  reg  [5:0] gpi   = 6'b000000; // general input (ui_in[7:2])
  wire       ready_eff; // READY (ui_in[0])

  wire [7:0] ui_in = {gpi, rxd, ready_eff};
  wire [7:0] uo_out;
  wire [7:0] uio_in; // driven by the SRAM model below
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

`ifdef GL_TEST
  wire VPWR = 1'b1;
  wire VGND = 1'b0;
`endif

  tt_um_a1 user_project (
`ifdef GL_TEST
      .VPWR(VPWR),
      .VGND(VGND),
`endif
      .ui_in  (ui_in),
      .uo_out (uo_out),
      .uio_in (uio_in),
      .uio_out(uio_out),
      .uio_oe (uio_oe),
      .ena    (ena),
      .clk    (clk),
      .rst_n  (rst_n)
  );

  wire ale  = uo_out[0];
  wire aleh = uo_out[1];
  wire rd_n = uo_out[2];
  wire wr_n = uo_out[3];
  wire txd  = uo_out[4];

  // external latches & SRAM
  reg [7:0] lat_lo = 8'h00;
  reg [7:0] lat_hi = 8'h00;
  always @* if (ale  === 1'b1) lat_lo = uio_out; // transparent while HIGH
  always @* if (aleh === 1'b1) lat_hi = uio_out;

  wire [12:0] ext_addr = {lat_hi[4:0], lat_lo};

  reg [7:0]  sram [0:8191];
  reg [15:0] img  [0:4095];

  // Floating-bus junk: chip must only sample AD during RD# LOW
  reg [7:0] junk = 8'h5A;
  always @(posedge clk) junk <= {junk[6:0], junk[7] ^ junk[5] ^ junk[4] ^ junk[3]};

  assign uio_in = (rd_n === 1'b0) ? sram[ext_addr] : junk;

  always @(posedge wr_n) if (rst_n === 1'b1) sram[ext_addr] <= uio_out;

  // cocotb sets peek_addr, reads peek_data (look into SRAM)
  reg  [12:0] peek_addr = 13'd0;
  wire [7:0]  peek_data = sram[peek_addr];

  // First 8 bytes the chip reads, for fetch-order test
  reg [103:0] rdtrace = 104'd0;
  integer     rdcnt   = 0;
  always @(negedge rd_n) if (rst_n === 1'b1) begin
    if (rdcnt < 8) rdtrace[rdcnt*13 +: 13] = ext_addr;
    rdcnt = rdcnt + 1;
  end

  // (re)load nucleus image into SRAM: at time 0 and on every posedge reload
  reg reload = 1'b0;
  integer li;
  reg [15:0] lw;
  task load_image;
    begin
      for (li = 0; li < 4096; li = li + 1) begin
        lw = img[li];
        sram[2*li]   = lw[7:0]; // little-endian: byte 2w = low byte
        sram[2*li+1] = lw[15:8];
      end
    end
  endtask

  integer ii;
  initial begin
    for (ii = 0; ii < 4096; ii = ii + 1) img[ii] = 16'h0000;
    $readmemh("nuc.hex", img);
    load_image;
  end
  always @(posedge reload) begin
    load_image;
    rdcnt = 0;
    rdtrace = 0;
  end

  // READY wait states
  reg        wait_en  = 1'b0; // cocotb: enable
  reg  [2:0] wait_max = 3'd0; // cocotb: random stall of 0..wait_max
  reg        ready_r  = 1'b1;
  reg  [2:0] wait_cnt = 3'd0;
  integer    waits_inserted = 0;
  reg [15:0] lfsr = 16'hACE1;
  always @(posedge clk) lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};

  assign ready_eff = ready_r;
  always @(posedge clk) begin
    if (rst_n !== 1'b1 || !wait_en) begin
      ready_r  <= 1'b1;
      wait_cnt <= 3'd0;
    end else if (ale === 1'b1) begin // pull READY LOW right after ALE
      ready_r  <= 1'b0;
      wait_cnt <= lfsr[2:0] % ({1'b0, wait_max} + 4'd1);
      waits_inserted = waits_inserted + 1;
    end else if (!ready_r) begin
      if (wait_cnt == 3'd0) ready_r <= 1'b1;
      else                  wait_cnt <= wait_cnt - 3'd1;
    end
  end

  // bus protocol checker
  // viol_kind bit: 0 RD# & WR# both LOW        1 strobe during address phase
  //                2 ALE&ALEH both HIGH        3 chip drives AD while RD# LOW
  //                4 AD changed when ALE fell  5 AD changed when ALEH fell
  //                6 X/Z on a control output
  integer    viol      = 0;
  reg [7:0]  viol_kind = 8'h00;
  reg        ale_q = 1'b0, aleh_q = 1'b0;
  reg [7:0]  ad_q  = 8'h00;

  task flag(input integer k);
    begin
      viol = viol + 1;
      viol_kind[k] = 1'b1;
    end
  endtask

  always @(posedge clk) begin
    if (rst_n !== 1'b1) begin
      ale_q  <= 1'b0;
      aleh_q <= 1'b0;
      ad_q   <= 8'h00;
    end else begin
      if (rd_n === 1'b0 && wr_n === 1'b0) flag(0);
      if ((ale === 1'b1 || aleh === 1'b1) && (rd_n === 1'b0 || wr_n === 1'b0)) flag(1);
      if (ale === 1'b1 && aleh === 1'b1) flag(2);
      if (rd_n === 1'b0 && uio_oe !== 8'h00) flag(3);
      if (ale_q  && ale  === 1'b0 && uio_out !== ad_q) flag(4);
      if (aleh_q && aleh === 1'b0 && uio_out !== ad_q) flag(5);
      if ((^uo_out[3:0]) === 1'bx) flag(6);
      ale_q  <= (ale  === 1'b1);
      aleh_q <= (aleh === 1'b1);
      ad_q   <= uio_out;
    end
  end

  // UART TX monitor
  // Decodes chip's TXD at 115200 baud into txline (newest byte in
  // LOW bits, 256 bytes) and txn (running count). Frame errors are counted.
  localparam real BIT_NS = 1.0e9 / 115200.0;

  reg [2047:0] txline    = 2048'd0;
  integer      txn       = 0;
  integer      frame_err = 0;
  reg [7:0]    mb;
  integer      mk;
  reg          mabort;

  always @(negedge txd) if (rst_n === 1'b1) begin
    mabort = 1'b0;
    #(BIT_NS * 1.5);
    for (mk = 0; mk < 8; mk = mk + 1) begin
      mb[mk] = txd;
      if (rst_n !== 1'b1) mabort = 1'b1;
      #(BIT_NS);
    end
    if (!mabort) begin
      if (txd !== 1'b1) frame_err = frame_err + 1; // no stop bit
      txline = {txline[2039:0], mb};
      txn    = txn + 1;
    end
  end

endmodule

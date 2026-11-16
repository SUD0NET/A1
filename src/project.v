/*
 * A1 - a SwapForth J1a microprocessor with multiplexed address/data bus.
 *
 * Copyright (c) 2026 Adnan Saad
 * SPDX-License-Identifier: Apache-2.0
 *
 * a1_core, a1_stack2 and UART modules are derived from the j1a CPU and
 * peripherals in James Bowman's SwapForth project and remain under its
 * BSD-3-Clause license.
 */

`default_nettype none

// a1_stack2 - shift-register stack ripped from j1a stack2.v
// "head" is top of stack, "tail" holds rest. delta: 01 push, 11 pop.

module a1_stack2 #(
  parameter DEPTH = 18
) (
  input  wire        clk,
  input  wire        rst_n,
  output wire [15:0] rd,
  input  wire        we,
  input  wire [1:0]  delta,
  input  wire [15:0] wd
);
  localparam BITS = (16 * DEPTH) - 1;

  wire move = delta[0];

  reg  [15:0]   head;
  reg  [BITS:0] tail;
  wire [15:0]   headN;
  wire [BITS:0] tailN;

  assign headN = we ? wd : tail[15:0];
  assign tailN = delta[1] ? {16'h55aa, tail[BITS:16]} : {tail[BITS-16:0], head};

  always @(posedge clk) begin
    if (!rst_n) begin
      head <= 16'd0;
      tail <= {(BITS+1){1'b0}};
    end else begin
      if (we | move)
        head <= headN;
      if (move)
        tail <= tailN;
    end
  end
  assign rd = head;
endmodule

// a1_core - j1a CPU.
// There is no data-read port: "@" is implemented in software by returning
// into data word with pc[12] set, which fetches it as a 16-bit literal.
// only memory read is instruction fetch (code_addr -> insn).

module a1_core (
  input  wire        clk,
  input  wire        rst_n,
  input  wire        ce,

  output wire        io_rd,
  output wire        io_wr,
  output wire [15:0] mem_addr,
  output wire        mem_wr,
  output wire [15:0] dout,
  input  wire [15:0] io_din,

  output wire [12:0] code_addr,
  input  wire [15:0] insn
);
  reg  [3:0]  dsp, dspN; // data stack pointer (status only)
  reg  [15:0] st0, st0N; // top of data stack
  reg         dstkW; // data stack write

  reg  [12:0] pc, pcN; // program counter; pc[12] = data fetch
  wire [12:0] pc_plus_1 = pc + 13'd1;
  reg         rstkW; // return stack write
  wire [15:0] rstkD; // return stack write value
  reg         reboot;

  assign mem_addr  = st0;
  assign code_addr = pcN;

  // d and r stacks
  wire [15:0] st1, rst0;
  reg  [1:0]  dspI, rspI;
  a1_stack2 #(.DEPTH(15)) dstack (.clk(clk), .rst_n(rst_n), .rd(st1),  .we(dstkW & ce),
                                  .wd(st0),   .delta(ce ? dspI : 2'b00));
  a1_stack2 #(.DEPTH(17)) rstack (.clk(clk), .rst_n(rst_n), .rd(rst0), .we(rstkW & ce),
                                  .wd(rstkD), .delta(ce ? rspI : 2'b00));
								  
  wire [16:0] minus = {1'b1, ~st0} + st1 + 17'd1;
  wire signedless = st0[15] ^ st1[15] ? st1[15] : minus[16];

  always @* begin
    casez ({pc[12], insn[15:8]})
      9'b1_???_?????: st0N = insn; // data word as literal
      9'b0_1??_?????: st0N = {1'b0, insn[14:0]}; // literal
      9'b0_000_?????: st0N = st0; // jump
      9'b0_010_?????: st0N = st0; // call
      9'b0_001_?????: st0N = st1; // conditional jump
      9'b0_011_?0000: st0N = st0; // ALU operations
      9'b0_011_?0001: st0N = st1;
      9'b0_011_?0010: st0N = st0 + st1;
      9'b0_011_?0011: st0N = st0 & st1;
      9'b0_011_?0100: st0N = st0 | st1;
      9'b0_011_?0101: st0N = st0 ^ st1;
      9'b0_011_?0110: st0N = ~st0;
      9'b0_011_?0111: st0N = {16{(minus == 17'd0)}}; // =
      9'b0_011_?1000: st0N = {16{signedless}}; // <
      9'b0_011_?1001: st0N = {st0[15], st0[15:1]}; // 2/
      9'b0_011_?1010: st0N = {st0[14:0], 1'b0}; // 2*
      9'b0_011_?1011: st0N = rst0;
      9'b0_011_?1100: st0N = minus[15:0]; // -
      9'b0_011_?1101: st0N = io_din;
      9'b0_011_?1110: st0N = {12'd0, dsp};
      9'b0_011_?1111: st0N = {16{minus[16]}}; // u<
      default:        st0N = st0;
    endcase
  end

  wire func_T_N   = (insn[6:4] == 3'd1);
  wire func_T_R   = (insn[6:4] == 3'd2);
  wire func_write = (insn[6:4] == 3'd3);
  wire func_iow   = (insn[6:4] == 3'd4);
  wire func_ior   = (insn[6:4] == 3'd5);

  wire is_alu = !pc[12] & (insn[15:13] == 3'b011);
  assign mem_wr = !reboot & is_alu & func_write;
  assign dout   = st1;
  assign io_wr  = !reboot & is_alu & func_iow;
  assign io_rd  = !reboot & is_alu & func_ior;

  assign rstkD = (insn[13] == 1'b0) ? {2'b00, pc_plus_1, 1'b0} : st0;

  always @* begin
    casez ({pc[12], insn[15:13]})
      4'b1_???,
      4'b0_1??:  {dstkW, dspI} = {1'b1,     2'b01};
      4'b0_001:  {dstkW, dspI} = {1'b0,     2'b11};
      4'b0_011:  {dstkW, dspI} = {func_T_N, insn[1:0]};
      default:   {dstkW, dspI} = {1'b0,     2'b00};
    endcase
    dspN = dsp + {dspI[1], dspI[1], dspI};

    casez ({pc[12], insn[15:13]})
      4'b1_???:  {rstkW, rspI} = {1'b0,     2'b11};
      4'b0_010:  {rstkW, rspI} = {1'b1,     2'b01};
      4'b0_011:  {rstkW, rspI} = {func_T_R, insn[3:2]};
      default:   {rstkW, rspI} = {1'b0,     2'b00};
    endcase

    casez ({reboot, pc[12], insn[15:13], insn[7], |st0})
      7'b1_0_???_?_?: pcN = 13'd0;
      7'b0_0_000_?_?,
      7'b0_0_010_?_?,
      7'b0_0_001_?_0: pcN = insn[12:0];
      7'b0_1_???_?_?,
      7'b0_0_011_1_?: pcN = rst0[13:1];
      default:        pcN = pc_plus_1;
    endcase
  end

  always @(posedge clk) begin
    if (!rst_n) begin
      reboot <= 1'b1;
      pc  <= 13'd0;
      dsp <= 4'd0;
      st0 <= 16'd0;
    end else if (ce) begin
      reboot <= 1'b0;
      pc  <= pcN;
      dsp <= dspN;
      st0 <= st0N;
    end
  end
endmodule


// UART, ripped from j1a "uart.v"

module a1_baudgen #(parameter CLK_HZ = 25000000, parameter BAUD = 115200) (
  input  wire clk,
  input  wire resetq,
  output wire ser_clk
);
  localparam lim = (CLK_HZ / BAUD) - 1;
  localparam w = $clog2(lim + 1);
  wire [w-1:0] limit = lim;
  reg  [w-1:0] counter;
  assign ser_clk = (counter == limit);

  always @(posedge clk)
    if (!resetq) counter <= 0;
    else         counter <= ser_clk ? 0 : (counter + 1'b1);
endmodule

// receiver baud generator: 2x baud rate, restartable
module a1_baudgen2 #(parameter CLK_HZ = 25000000, parameter BAUD = 115200) (
  input  wire clk,
  input  wire resetq,
  input  wire restart,
  output wire ser_clk
);
  localparam lim = (CLK_HZ / (2 * BAUD)) - 1;
  localparam w = $clog2(lim + 1);
  wire [w-1:0] limit = lim;
  reg  [w-1:0] counter;
  assign ser_clk = (counter == limit);

  always @(posedge clk)
    if (!resetq || restart) counter <= 0;
    else                    counter <= ser_clk ? 0 : (counter + 1'b1);
endmodule

module a1_txuart #(parameter CLK_HZ = 25000000, parameter BAUD = 115200) (
  input  wire       clk,
  input  wire       resetq,
  output wire       uart_busy,  // HIGH while txing
  output reg        uart_tx,
  input  wire       uart_wr_i,  // raise to tx byte
  input  wire [7:0] uart_dat_i
);
  reg [3:0] bitcount; // 0 = idle, otherwise 1-based
  reg [8:0] shifter;

  assign uart_busy = |bitcount;
  wire sending = |bitcount;

  wire ser_clk;
  a1_baudgen #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) _baudgen (
    .clk(clk), .resetq(resetq), .ser_clk(ser_clk));

  always @(posedge clk) begin
    if (!resetq) begin
      uart_tx  <= 1'b1;
      bitcount <= 4'd0;
      shifter  <= 9'd0;
    end else begin
      if (uart_wr_i) begin
        { shifter, uart_tx } <= { uart_dat_i[7:0], 1'b0, 1'b1 };
        bitcount <= 4'd10; // 1 start, 8 data, 1 stop
      end else if (ser_clk & sending) begin
        { shifter, uart_tx } <= { 1'b1, shifter };
        bitcount <= bitcount - 4'd1;
      end
    end
  end
endmodule

module a1_rxuart #(parameter CLK_HZ = 25000000, parameter BAUD = 115200) (
  input  wire       clk,
  input  wire       resetq,
  input  wire       uart_rx,
  input  wire       rd, // read strobe (clears valid)
  output wire       valid,
  output wire [7:0] data
);
  reg [4:0] bitcount; // 11111: idle, 0-17: sampling, 18: received
  reg [7:0] shifter;

  wire idle = &bitcount;
  assign valid = (bitcount == 5'd18);

  wire sample;
  reg  [2:0] hh;
  wire [2:0] hhN = {hh[1:0], uart_rx};
  wire startbit = idle & (hhN[2:1] == 2'b10);
  wire [7:0] shifterN = sample ? {hh[1], shifter[7:1]} : shifter;

  wire ser_clk;
  a1_baudgen2 #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) _baudgen (
    .clk(clk), .resetq(resetq), .restart(startbit), .ser_clk(ser_clk));

  reg [4:0] bitcountN;
  always @* begin
    if (startbit)
      bitcountN = 5'd0;
    else if (!idle & !valid & ser_clk)
      bitcountN = bitcount + 5'd1;
    else if (valid & rd)
      bitcountN = 5'b11111;
    else
      bitcountN = bitcount;
  end

  assign sample = ~idle & (|bitcount[4:1]) & bitcount[0] & ser_clk;
  assign data = shifter;

  always @(posedge clk) begin
    if (!resetq) begin
      hh       <= 3'b111;
      bitcount <= 5'b11111;
      shifter  <= 8'd0;
    end else begin
      hh       <= hhN;
      bitcount <= bitcountN;
      shifter  <= shifterN;
    end
  end
endmodule

module a1_buart #(parameter CLK_HZ = 25000000, parameter BAUD = 115200) (
  input  wire       clk,
  input  wire       resetq,
  input  wire       rx,
  output wire       tx,
  input  wire       rd,
  input  wire       wr,
  output wire       valid,
  output wire       busy,
  input  wire [7:0] tx_data,
  output wire [7:0] rx_data
);
  a1_rxuart #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) _rx (
    .clk(clk), .resetq(resetq), .uart_rx(rx), .rd(rd),
    .valid(valid), .data(rx_data));
  a1_txuart #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) _tx (
    .clk(clk), .resetq(resetq), .uart_busy(busy), .uart_tx(tx),
    .uart_wr_i(wr), .uart_dat_i(tx_data));
endmodule

// tt_um_a1 - the A1 microprocessor: j1a core + bus controller + UART + GPIO.
//
// Pins
//   uio[7:0]    AD[7:0]  multiplexed address / data
//   uo_out[0]   ALE      latch AD[7:0] (A[7:0]) on its falling edge
//   uo_out[1]   ALEH     latch AD[7:0] (A[15:8]; only A[12:8] are non-zero)
//   uo_out[2]   RD#      active low read strobe
//   uo_out[3]   WR#      active low write strobe
//   uo_out[4]   TXD      UART transmit
//   uo_out[7:5] GPO[2:0] general outputs (SwapForth "LEDs", I/O port 4)
//   ui_in[0]    READY    low during T5 stalls bus (tie high for no waits;
//                        single flop drives synchronously to clk)
//   ui_in[1]    RXD      UART receive
//   ui_in[7:2]  GPI[5:0] general inputs (I/O port 0x2000, bits 7:2)
//
// Memory: 8 KB (13-bit byte address), unified code + data, 16-bit
// little-endian cells (code word w at byte 2w), exactly like j1a
//
// I/O map (j1a compatible, selected by one-hot address bits):
//   0x0004  r/w  GPO[2:0]
//   0x1000  r/w  UART data (write = transmit, read = receive)
//   0x2000  r    {GPI[5:0], rx_valid, !tx_busy}
//
// One byte transfer, 6 clocks (4 for second byte of word):
//   T1  ALE=1   AD = A[7:0]
//   T2  ALE=0   AD = A[7:0]          (address hold)
//   T3  ALEH=1  AD = A[15:8]         (skipped for second byte)
//   T4  ALEH=0  AD = A[15:8]         (address hold; skipped likewise)
//   T5  RD#=0 (AD floating) or WR#=0 (AD = data); waits while READY=0;
//   read data is sampled on edge that ends T5
//   T6  strobes high, data held
//
// Per instruction: fetch 2 bytes then, if instruction is "!", write
// 2 bytes, then one clock where core advances. Reads of data words are
// instruction fetches (see a1_core), so no separate read cycle exists.

module tt_um_a1 #(
    parameter CLK_HZ = 25000000,   // clk frequency for UART divider
    parameter BAUD   = 115200
) (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

  // core
  localparam [1:0] PH_FETCH = 2'd0,
                   PH_WRITE = 2'd1,
                   PH_TICK  = 2'd2;

  reg  [1:0]  ph; // macro-cycle phase
  reg         bi; // byte index within 16-bit transfer
  reg  [5:0]  ts; // one-hot bus state T1..T6 (0 during TICK)
  reg  [15:0] insn, new_insn;

  wire [15:0] mem_addr, dout, io_din;
  wire [12:0] code_addr;
  wire        mem_wr, io_wr, io_rd;
  wire        ce = (ph == PH_TICK);

  a1_core core (
    .clk(clk), .rst_n(rst_n), .ce(ce),
    .io_rd(io_rd), .io_wr(io_wr), .mem_addr(mem_addr), .mem_wr(mem_wr),
    .dout(dout), .io_din(io_din), .code_addr(code_addr), .insn(insn));

  // bus sequencer
  reg ready_q;
  always @(posedge clk) begin
    if (!rst_n) ready_q <= 1'b1;
    else        ready_q <= ui_in[0];
  end

  reg [1:0] ph_n;
  reg       bi_n;
  reg [5:0] ts_n;
  always @* begin
    ph_n = ph;
    bi_n = bi;
    ts_n = ts;
    if (ph == PH_TICK) begin
      ph_n = PH_FETCH;  bi_n = 1'b0;  ts_n = 6'b000001;
    end else if (ts[0]) begin
      ts_n = 6'b000010;
    end else if (ts[1]) begin
      ts_n = bi ? 6'b010000 : 6'b000100; // 2nd byte skips ALEH phase
    end else if (ts[2]) begin
      ts_n = 6'b001000;
    end else if (ts[3]) begin
      ts_n = 6'b010000;
    end else if (ts[4]) begin
      if (ready_q) ts_n = 6'b100000;
    end else begin // T6: end of one byte transfer
      ts_n = 6'b000001;
      if (!bi) begin
        bi_n = 1'b1;
      end else begin
        bi_n = 1'b0;
        if (ph == PH_FETCH && mem_wr) ph_n = PH_WRITE;
        else begin ph_n = PH_TICK;  ts_n = 6'b000000; end
      end
    end
  end

  // strobes registered from NEXT state so they are glitch-free
  wire is_rd_n = (ph_n == PH_FETCH);
  wire is_wr_n = (ph_n == PH_WRITE);

  reg ale_r, aleh_r, rdn_r, wrn_r, oe_r;

  always @(posedge clk) begin
    if (!rst_n) begin
      ph <= PH_TICK;  bi <= 1'b0;  ts <= 6'b000000;
      ale_r <= 1'b0;  aleh_r <= 1'b0;  rdn_r <= 1'b1;  wrn_r <= 1'b1;
      oe_r  <= 1'b0;
      insn  <= 16'h0000;  new_insn <= 16'h0000; // 0000 = "jump 0" (nop)
    end else begin
      ph <= ph_n;  bi <= bi_n;  ts <= ts_n;
      ale_r  <= ts_n[0];
      aleh_r <= ts_n[2];
      rdn_r  <= ~(ts_n[4] & is_rd_n);
      wrn_r  <= ~(ts_n[4] & is_wr_n);
      oe_r   <= ts_n[0] | ts_n[1] | ts_n[2] | ts_n[3]
              | (is_wr_n & (ts_n[4] | ts_n[5]));

      // sample fetch data on edge that ends T5
      if (ts[4] && ready_q && ph == PH_FETCH)
        new_insn <= {uio_in, new_insn[15:8]};

      // core advances on this edge; hand it fetched instruction
      if (ce)
        insn <= new_insn;
    end
  end

  // address & data
  wire [12:0] a_full = (ph == PH_WRITE) ? {mem_addr[12:1], bi}
                                        : {code_addr[11:0], bi};
  wire [7:0]  wdata  = bi ? dout[15:8] : dout[7:0];

  assign uio_out = (ts[0] | ts[1]) ? a_full[7:0] :
                   (ts[2] | ts[3]) ? {3'b000, a_full[12:8]} : wdata;
  assign uio_oe  = {8{oe_r}};

  // registered one-clock I/O strobes (core is frozen for many clocks
  // per instruction, so strobe once on clock after core advances).
  reg        io_wr_, io_rd_;
  reg [15:0] dout_, io_addr_;
  always @(posedge clk) begin
    if (!rst_n) begin
      io_wr_ <= 1'b0;  io_rd_ <= 1'b0;
      dout_  <= 16'h0000;  io_addr_ <= 16'h0000;
    end else begin
      io_wr_ <= ce & io_wr;
      io_rd_ <= ce & io_rd;
      if (ce) begin
        dout_ <= dout;
        if (io_rd | io_wr) io_addr_ <= mem_addr;
      end
    end
  end

  reg [2:0] gpo;
  always @(posedge clk)
    if (!rst_n)                       gpo <= 3'b000;
    else if (io_wr_ & io_addr_[2])    gpo <= dout_[2:0];

  reg rx_q1, rx_q2;
  always @(posedge clk) begin
    if (!rst_n) begin
      rx_q1 <= 1'b1;
      rx_q2 <= 1'b1;
    end else begin
      rx_q1 <= ui_in[1];
      rx_q2 <= rx_q1;
    end
  end

  reg [5:0] gpi_q1, gpi_q2;
  always @(posedge clk) begin
    if (!rst_n) begin
      gpi_q1 <= 6'd0;
      gpi_q2 <= 6'd0;
    end else begin
      gpi_q1 <= ui_in[7:2];
      gpi_q2 <= gpi_q1;
    end
  end

  wire uart_tx, uart_valid, uart_busy;
  wire [7:0] uart_data;
  a1_buart #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) uart0 (
    .clk(clk), .resetq(rst_n),
    .rx(rx_q2), .tx(uart_tx),
    .rd(io_rd_ & io_addr_[12]), .wr(io_wr_ & io_addr_[12]),
    .valid(uart_valid), .busy(uart_busy),
    .tx_data(dout_[7:0]), .rx_data(uart_data));

  assign io_din =
    ((mem_addr == 16'h0004) ? {13'd0, gpo}: 16'd0) |
    ((mem_addr == 16'h1000) ? {8'd0, uart_data}: 16'd0) |
    ((mem_addr == 16'h2000) ? {8'd0, gpi_q2, uart_valid, ~uart_busy}: 16'd0);
  assign uo_out = {gpo, uart_tx, wrn_r, rdn_r, aleh_r, ale_r};
  
  // list all unused inputs to prevent warnings
  wire _unused = &{ena, code_addr[12], 1'b0};
  
endmodule

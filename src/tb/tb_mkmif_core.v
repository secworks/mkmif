//======================================================================
//
// tb_mkmif_core.v
// ---------------
// Testbench for the mkmif core module.
//
//
// Copyright (c) 2016, Assured AB
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or
// without modification, are permitted provided that the following
// conditions are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//======================================================================

//------------------------------------------------------------------
// Compiler directives.
//------------------------------------------------------------------
`timescale 1ns/100ps

module tb_mkmif_core();

  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  parameter DEBUG = 1;

  parameter CLK_HALF_PERIOD = 2;
  parameter CLK_PERIOD      = 2 * CLK_HALF_PERIOD;


  //----------------------------------------------------------------
  // Register and Wire declarations.
  //----------------------------------------------------------------
  reg [31 : 0] cycle_ctr;
  reg [31 : 0] test_ctr;
  reg [31 : 0] error_ctr;

  reg           tb_clk;
  reg           tb_reset_n;
  wire          tb_spi_sclk;
  wire          tb_spi_cs_n;
  reg           tb_spi_do;
  wire          tb_spi_di;
  reg           tb_write_op;
  reg           tb_read_op;
  wire          tb_ready;
  wire          tb_valid;
  reg  [15 : 0] tb_sclk_div;
  reg  [15 : 0] tb_addr;
  reg  [31 : 0] tb_write_data;
  wire [31 : 0] tb_read_data;

  reg           tb_dump_ports;
  reg [31 : 0]  read_data;


  //----------------------------------------------------------------
  // mkmif device under test.
  //----------------------------------------------------------------
  mkmif_core dut(
                 .clk(tb_clk),
                 .reset_n(tb_reset_n),

                 .spi_sclk(tb_spi_sclk),
                 .spi_cs_n(tb_cs_n),
                 .spi_do(tb_spi_do),
                 .spi_di(tb_spi_di),

                 .write_op(tb_write_op),
                 .read_op(tb_read_op),
                 .ready(tb_ready),
                 .valid(tb_valid),
                 .sclk_div(tb_sclk_div),
                 .addr(tb_addr),
                 .write_data(tb_write_data),
                 .read_data(tb_read_data)
                );


  //----------------------------------------------------------------
  // clk_gen
  // Clock generator process.
  //----------------------------------------------------------------
  always
    begin : clk_gen
      #CLK_HALF_PERIOD tb_clk = !tb_clk;
    end // clk_gen


  //--------------------------------------------------------------------
  // dut_monitor
  // Monitor for observing the inputs and outputs to the dut.
  // Includes the cycle counter.
  //--------------------------------------------------------------------
  always @ (posedge tb_clk)
    begin : dut_monitor
      cycle_ctr = cycle_ctr + 1;

      if (DEBUG)
        $display("cycle = %8x:", cycle_ctr);
        dump_state();

      if (tb_dump_ports)
        dump_ports();
    end // dut_monitor


  //----------------------------------------------------------------
  // inc_test_ctr
  //----------------------------------------------------------------
  task inc_test_ctr;
    begin
      test_ctr = test_ctr +1;
    end
  endtask // inc_test_ctr


  //----------------------------------------------------------------
  // inc_error_ctr
  //----------------------------------------------------------------
  task inc_error_ctr;
    begin
      error_ctr = error_ctr +1;
    end
  endtask // inc_error_ctr


  //----------------------------------------------------------------
  // dump_ports
  // Dump the status of the dut ports.
  //----------------------------------------------------------------
  task dump_ports;
    begin
      $display("API ports:");
      $display("");

      $display("SPI ports:");
      $display("clock: 0x%01x, en: 0x%01x, di: 0x%01x, do: 0x%01x:",
               tb_spi_sclk, tb_spi_cs_n, tb_spi_di, tb_spi_do);
      $display("");
    end
  endtask // dump_ports


  //----------------------------------------------------------------
  // dump_state
  // Dump the internal MKMIF state to std out.
  //----------------------------------------------------------------
  task dump_state;
    begin
      $display("Core state:");
      $display("mkmif_ctrl_reg: 0x%02x", dut.mkmif_ctrl_reg);
      $display("spi_write_data: 0x%014x", dut.spi_write_data);
      $display("spi_length: 0x%02x, spi_set: 0x%01x, spi_start: 0x%01x, spi_ready: 0x%01x",
               dut.spi_length, dut.spi_set, dut.spi_start, dut.spi_ready);
      $display("");
    end
  endtask // dump_state


  //----------------------------------------------------------------
  // tb_init
  // Initialize varibles, dut inputs at start.
  //----------------------------------------------------------------
  task tb_init;
    begin
      test_ctr      = 0;
      error_ctr     = 0;
      cycle_ctr     = 0;

      tb_clk        = 0;
      tb_reset_n    = 1;
      tb_spi_do     = 0;
      tb_write_op   = 0;
      tb_read_op    = 0;
      tb_sclk_div   = 16'h0002;
      tb_addr       = 16'h0010;
      tb_write_data = 32'haa55aa55;

      tb_dump_ports = 0;
    end
  endtask // tb_init


  //----------------------------------------------------------------
  // toggle_reset
  // Toggle the reset.
  //----------------------------------------------------------------
  task toggle_reset;
    begin
      $display("  -- Toggling reset.");
      dump_state();
      #(2 * CLK_PERIOD);
      tb_reset_n = 0;
      #(10 * CLK_PERIOD);
      @(negedge tb_clk)
      tb_reset_n = 1;
      dump_state();
      $display("  -- Toggling of reset done.");
    end
  endtask // toggle_reset


  //----------------------------------------------------------------
  // mkmif_core_test
  // The main test functionality.
  //----------------------------------------------------------------
  initial
    begin : mkmif__core_test
      $display("   -- Test of mkmif core started --");

      tb_init();
      toggle_reset();
      #(1000 * CLK_PERIOD);

      $display("");
      $display("   -- Test of mkmif core completed --");
      $display("Tests executed: %04d", test_ctr);
      $display("Tests failed:   %04d", error_ctr);
      $finish;
    end // mkmif_core_test

endmodule // tb_mkmif_core

//======================================================================
// EOF tb_mkmif_core.v
//======================================================================
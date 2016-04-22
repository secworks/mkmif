//======================================================================
//
// mkmif_spi.v
// -----------
// SPI interface for the master key memory.
//
//
// Author: Joachim Strombergson
// Copyright (c) 2016, Assured AB.
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

module mkmif_spi(
                 input wire           clk,
                 input wire           reset_n,

                 output wire          spi_sclk,
                 output wire          spi_cs_n,
                 input wire           spi_do,
                 output wire          spi_di,

                 input wire           enable,
                 input wire           set,
                 input wire           start,
                 input wire           write,
                 input wire [2 : 0]   length,
                 input wire [15 : 0]  divisor,
                 output wire          ready,
                 input wire [55 : 0]  wr_data,
                 output wire [31 : 0] rd_data
                );


  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg sclk_reg;
  reg sclk_new;
  reg sclk_we;

  reg cs_n_reg;
  reg cs_n_new;
  reg cs_n_we;

  reg [55 : 0] di_reg;
  reg [55 : 0] di_new;
  reg          di_we;

  reg do_sample0_reg;
  reg do_sample1_reg;

  reg [31 : 0] rd_data_reg;
  reg          rd_data_we;

  reg ready_reg;
  reg ready_new;
  reg ready_we;


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign spi_sclk = sclk_reg;
  assign spi_cs_n = cs_n_reg;
  assign spi_di   = di_reg[55];
  assign ready    = ready_reg;
  assign rd_data  = rd_data_reg;


  //----------------------------------------------------------------
  // reg_update
  // Update functionality for all registers in the core.
  // All registers are positive edge triggered with asynchronous
  // active low reset.
  //----------------------------------------------------------------
  always @ (posedge clk or negedge reset_n)
    begin
      if (!reset_n)
        begin
          sclk_reg       <= 1'b0;
          cs_n_reg       <= 1'b1;
          di_reg         <= 56'h0;
          do_sample0_reg <= 0;
          do_sample1_reg <= 0;
          ready_reg      <= 0;
          rd_data_reg    <= 32'h0;

        end
      else
        begin
          do_sample0_reg <= spi_do;
          do_sample1_reg <= do_sample0_reg;

          if (sclk_we)
            sclk_reg <= sclk_new;

          if (cs_n_we)
            cs_n_reg <= cs_n_new;

          if (di_we)
            di_reg <= di_new;

          if (ready_we)
            ready_reg <= ready_new;

          if (rd_data_we)
            rd_data_reg <= {rd_data_reg[30 : 0], do_sample1_reg};

        end
    end // reg_update


  //----------------------------------------------------------------
  // Dummy logic. Used just to allow us to build the design.
  //----------------------------------------------------------------
  always @*
    begin
      sclk_new   = 0;
      sclk_we    = 0;
      cs_n_new   = 1;
      cs_n_we    = 0;
      di_new     = 56'h0;
      di_we      = 0;
      rd_data_we = 0;
      ready_new  = 0;
      ready_we   = 0;

      if (set)
        begin
          sclk_new   = 1;
          sclk_we    = 1;
          cs_n_new   = 0;
          cs_n_we    = 1;
          di_new     = 56'h1;
          di_we      = 1;
          rd_data_we = 1;
          ready_new  = 1;
          ready_we   = 1;
        end
    end


//
//
//
//  //----------------------------------------------------------------
//  // spi_di_gen
//  //
//  // Generate the bitstream to be written as data into the
//  // external SPI connected memory. The generator also counts
//  // bits shifted out and signals when 8 bits has been
//  // shifted.
//  //----------------------------------------------------------------
//  always @*
//    begin : spi_di_gen
//      spi_di_data_new    = 8'h00;
//      spi_di_data_we     = 0;
//      spi_di_data_done   = 0;
//      spi_di_bit_ctr_new = 3'h0;
//      spi_di_bit_ctr_we  = 0;
//
//      if (spi_di_bit_ctr_reg == 3'h7)
//        spi_di_data_done = 1;
//
//      if (spi_di_data_set)
//        begin
//          spi_di_data_new    = spi_di_data;
//          spi_di_data_we     = 1;
//          spi_di_bit_ctr_new = 3'h0;
//          spi_di_bit_ctr_we  = 1;
//        end
//
//      if (spi_di_data_nxt)
//        begin
//          spi_di_data_new = {spi_di_data_reg[6 : 0], 1'b0};
//          spi_di_data_we  = 1;
//
//          spi_di_bit_ctr_new  = spi_di_bit_ctr_reg + 1'b1;
//          spi_di_bit_ctr_we   = 1;
//        end
//
//    end // spi_di_gen
//
//  //----------------------------------------------------------------
//  // spi_write_data_gen
//  //
//  // Generates the data to be written.
//  //----------------------------------------------------------------
//  always @*
//    begin : spi_write_data_gen
//      spi_write_data_new = 32'h00;
//      spi_write_data_we  = 0;
//
//      if (spi_write_data_set)
//        begin
//          spi_write_data_new = write_data;
//          spi_write_data_we  = 1;
//        end
//
//      if (spi_write_data_rst)
//        begin
//          spi_write_data_new = 32'h00;
//          spi_write_data_we  = 1;
//        end
//    end // spi_write_data_gen
//
//
//  //----------------------------------------------------------------
//  // spi_read_data_gen
//  //
//  // Generates the data to be written.
//  //----------------------------------------------------------------
//  always @*
//    begin : spi_read_data_gen
//      spi_read_data_new = 32'h0;
//      spi_read_data_we  = 0;
//
//      if (spi_read_data_rst)
//        begin
//          spi_read_data_new = 32'h0;
//          spi_read_data_we  = 1;
//        end
//
//      if (spi_read_data_nxt)
//        begin
//          spi_read_data_new = {spi_read_data_reg[30 : 0], spi_do_reg};
//          spi_read_data_we  = 1;
//        end
//    end // spi_read_data_gen
//
//
//  //----------------------------------------------------------------
//  // spi_sclk_gen
//  //
//  // Generator of the spi_sclk clock. The generator includes
//  // a detector for midpoint of a flank.
//  //----------------------------------------------------------------
//  always @*
//    begin : siphash_sclk_gen
//      spi_sclk_ctr_new = 16'h00;
//      spi_sclk_ctr_we  = 0;
//      spi_sclk_we      = 0;
//      spi_sclk_new     = ~spi_sclk_reg;
//      spi_sclk_ctr_mid = 0;
//
//      if (spi_sclk_ctr_reg == {1'b0, spi_sclk_div_reg[15 : 1]})
//        spi_sclk_ctr_mid = 1;
//
//      if (spi_sclk_en)
//        begin
//          if (spi_sclk_ctr_reg == spi_sclk_div_reg)
//            begin
//              spi_sclk_ctr_new = 16'h00;
//              spi_sclk_we      = 1;
//            end
//          else
//            spi_sclk_ctr_new = spi_sclk_ctr_new + 1'b1;
//        end
//    end // siphash_sclk_gen
//
//
//  //----------------------------------------------------------------
//  // spi_byte_ctr
//  //
//  // Byte counter used by the FSM to keep track of the bytes
//  // being read or written.
//  //----------------------------------------------------------------
//  always @*
//    begin : spi_byte_ctr
//      spi_byte_ctr_new = 12'h0;
//      spi_byte_ctr_we  = 1'b0;
//
//      if (spi_byte_ctr_rst)
//        begin
//          spi_byte_ctr_new = 12'h0;
//          spi_byte_ctr_we  = 1'b1;
//        end
//
//      if (spi_byte_ctr_inc)
//        begin
//          spi_byte_ctr_new = spi_byte_ctr_reg + 1'b1;
//          spi_byte_ctr_we  = 1'b1;
//        end
//    end // spi_byte_ctr
//

//  //----------------------------------------------------------------
//  // mkmif_spi_ctrl
//  //----------------------------------------------------------------
//  always @*
//    begin : mkmif_spi_ctrl
//
//    end // mkmif_spi_ctrl

endmodule // mkmif_spi

//======================================================================
// EOF mkmif_spi.v
//======================================================================

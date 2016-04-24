//======================================================================
//
// mkmif_spi.v
// -----------
// SPI interface for the master key memory. When enabled the
// interface waits for command to transmit and receive a given
// number of bytes. Data is transmitted onto the spi_di port
// from the MSB of the spi_data register. Simultaneously,
// data captured on the spi_do port is inserted at LSB in the
// spi_data register. The spi clock is generated when data is to be
// sent or recived.
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

                 input wire           set,
                 input wire           start,
                 input wire [2 : 0]   length,
                 input wire [15 : 0]  divisor,
                 output wire          ready,
                 input wire [55 : 0]  wr_data,
                 output wire [31 : 0] rd_data
                );


  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  localparam CTRL_IDLE = 0;


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg          sclk_reg;
  reg          sclk_new;
  reg          sclk_we;

  reg          cs_n_reg;
  reg          cs_n_new;
  reg          cs_n_we;

  reg          do_sample0_reg;
  reg          do_sample1_reg;

  reg [55 : 0] data_reg;
  reg [55 : 0] data_new;
  reg          data_set;
  reg          data_nxt;
  reg          data_we;

  reg          ready_reg;
  reg          ready_new;
  reg          ready_we;

  reg  [5 : 0] bit_ctr_reg;
  reg  [5 : 0] bit_ctr_new;
  reg          bit_ctr_rst;
  reg          bit_ctr_inc;
  reg          bit_ctr_we;

  reg  [2 : 0] spi_ctrl_reg;
  reg  [2 : 0] spi_ctrl_new;
  reg          spi_ctrl_we;


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg en_clk_gen;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign spi_sclk = sclk_reg;
  assign spi_cs_n = cs_n_reg;
  assign spi_di   = data_reg[55];
  assign rd_data  = data_reg[31 : 0];
  assign ready    = ready_reg;


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
          sclk_reg       <= 1'h0;
          cs_n_reg       <= 1'h1;
          do_sample0_reg <= 1'h0;
          do_sample1_reg <= 1'h0;
          data_reg       <= 56'h0;
          ready_reg      <= 1'h0;
          bit_ctr_reg    <= 6'h0;
          spi_ctrl_reg   <= CTRL_IDLE;
        end
      else
        begin
          do_sample0_reg <= spi_do;
          do_sample1_reg <= do_sample0_reg;

          if (sclk_we)
            sclk_reg <= sclk_new;

          if (cs_n_we)
            cs_n_reg <= cs_n_new;

          if (ready_we)
            ready_reg <= ready_new;

          if (data_we)
            data_reg <= data_new;

          if (bit_ctr_we)
            bit_ctr_reg <= bit_ctr_new;

          if (spi_ctrl_we)
            spi_ctrl_reg <= spi_ctrl_new;

        end
    end // reg_update


  //----------------------------------------------------------------
  // data_gen
  //
  // Generate the data bitstream to be written out to the external
  // SPI connected memory. Basically a shift register.
  // Note that we also shift in data received from the external
  // memory.
  //----------------------------------------------------------------
  always @*
    begin : data_gen
      data_new = 56'h0;
      data_we  = 0;

      if (data_set)
        begin
          data_new = wr_data;
          data_we  = 1;
        end

      if (data_nxt)
        begin
          data_new = {data_reg[54 : 0], do_sample1_reg};
          data_we  = 1;
        end
    end // data_gen


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

  //----------------------------------------------------------------
  // bit_ctr
  //
  // Bit counter used by the FSM to keep track of the number bits
  // being read from or written to the memory.
  //----------------------------------------------------------------
  always @*
    begin : bit_ctr
      bit_ctr_new = 5'h0;
      bit_ctr_we  = 1'b0;

      if (bit_ctr_rst)
        begin
          bit_ctr_new = 5'h0;
          bit_ctr_we  = 1'b1;
        end

      if (bit_ctr_inc)
        begin
          bit_ctr_new = bit_ctr_reg + 1'b1;
          bit_ctr_we  = 1'b1;
        end
    end // bit_ctr


  //----------------------------------------------------------------
  // spi_ctrl
  //
  // Control FSM for the SPI interface.
  //----------------------------------------------------------------
  always @*
    begin : spi_ctrl
      sclk_new     = 0;
      sclk_we      = 0;
      cs_n_new     = 1;
      cs_n_we      = 0;
      data_set     = 0;
      data_nxt     = 0;
      ready_new    = 0;
      ready_we     = 0;
      en_clk_gen   = 0;
      bit_ctr_rst  = 0;
      bit_ctr_inc  = 0;
      spi_ctrl_new = CTRL_IDLE;
      spi_ctrl_we  = 0;

      case (spi_ctrl_reg)
        CTRL_IDLE:
          begin
          end

        default:
          begin
          end
      endcase // case (spi_ctrl_reg)
    end // spi_ctrl

endmodule // mkmif_spi

//======================================================================
// EOF mkmif_spi.v
//======================================================================

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
                 input wire [13 : 0]  length,
                 input wire [15 : 0]  divisor,
                 output wire          ready,
                 input wire [31 : 0]  wr_data,
                 output wire [31 : 0] rd_data
                );


  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------


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

        end
      else
        begin

        end
    end // reg_update


  //----------------------------------------------------------------
  // mkmif_spi_ctrl
  //----------------------------------------------------------------
  always @*
    begin : mkmif_spi_ctrl

    end // mkmif_spi_ctrl

endmodule // mkmif_spi

//======================================================================
// EOF mkmif_spi.v
//======================================================================

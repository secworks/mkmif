//======================================================================
//
// mkmif_core.v
// ------------
// The actual core module for the Master Key Memory (MKM) interface.
// The interface is implemented to use the Microchip 23K640 serial
// sram as external storage. The core acts as a SPI Master for the
// external memory including SPI clock generation.
//
// The current version of the core does not provide any functionality
// to protect against remanence.
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

module mkmif_core(
                  input wire           clk,
                  input wire           reset_n,

                  output wire          spi_sclk,
                  output wire          spi_cs_n,
                  input wire           spi_do,
                  output wire          spi_di,

                  input wire           write_op,
                  input wire           read_op,
                  output wire          ready,
                  output wire          valid,
                  input wire [15 : 0]  sclk_div,
                  input wire [10 : 0]  spi_addr,
                  input wire [31 : 0]  spi_write_data,
                  output wire [31 : 0] spi_read_data
                 );


  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  localparam SPI_READ_DATA_CMD    = 8'h03;
  localparam SPI_WRITE_DATA_CMD   = 8'h02;
  localparam SPI_READ_STATUS_CMD  = 8'h05;
  localparam SPI_WRITE_STATUS_CMD = 8'h01;

  localparam STATUS_SEQ_MODE_NO_HOLD = 8'b01000001;

  localparam CTRL_IDLE      = 0;
  localparam CTRL_SRAM_INIT = 1;
  localparam CTRL_READY     = 2;
  localparam CTRL_READ      = 3;
  localparam CTRL_WRITE     = 4;


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg         ready_reg;
  reg         ready_new;
  reg         ready_we;
  reg         valid_reg;
  reg         valid_new;
  reg         valid_we;

  reg [4 : 0] mkmif_ctrl_reg;
  reg [4 : 0] mkmif_ctrl_new;
  reg         mkmif_ctrl_we;


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
          ready_reg      <= 0;
          valid_reg      <= 0;
          mkmif_ctrl_reg <= CTRL_IDLE;
        end
      else
        begin
          if (ready_we)
            ready_reg <= ready_new;

          if (valid_we)
            valid_reg <= valid_new;

          if (mkmif_ctrl_we)
            mkmif_ctrl_reg <= mkmif_ctrl_new;
        end
    end // reg_update


  //----------------------------------------------------------------
  // mkmif_ctrl
  // Main control FSM.
  //----------------------------------------------------------------
  always @*
    begin : mkmif_ctrl
      ready_new          = 0;
      ready_we           = 0;
      valid_new          = 0;
      valid_we           = 0;
      mkmif_ctrl_new     = CTRL_IDLE;
      mkmif_ctrl_we      = 0;

      case (mkmif_ctrl_reg)
        CTRL_IDLE:
          begin
          end

        default:
          begin
          end
      endcase // case (mkmif_ctrl_reg)
    end // mkmif_ctrl
endmodule // mkmif

//======================================================================
// EOF mkmif.v
//======================================================================

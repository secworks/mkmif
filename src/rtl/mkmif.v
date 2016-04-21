//======================================================================
//
// mkmif.v
// -------
// Master Key Memory (MKM) interface. The interface is implemented
// to use the Microchip 23K640 serial sram as external storage.
// The core acts as a SPI Master for the external memory including
// SPI clock generation.
//
// The current version of the core does not provide any functionality
// to protect against remanence. The core however will clear
// any data stored in the core after being written to or read from
// the external memory.
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

module mkmif(
             input wire           clk,
             input wire           reset_n,

             output wire          spi_sclk,
             output wire          spi_cs_n,
             input wire           spi_s0,
             output wire          spi_si,

             input wire           cs,
             input wire           we,
             input wire  [7 : 0]  address,
             input wire  [31 : 0] write_data,
             output wire [31 : 0] read_data
            );


  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  localparam ADDR_NAME0       = 8'h00;
  localparam ADDR_NAME1       = 8'h01;
  localparam ADDR_VERSION     = 8'h02;
  localparam ADDR_CTRL        = 8'h08;
  localparam CTRL_READ_BIT    = 0;
  localparam CTRL_WRITE_BIT   = 1;
  localparam ADDR_STATUS      = 8'h09;
  localparam STATUS_READY_BIT = 0;
  localparam STATUS_VALID_BIT = 0;
  localparam ADDR_SCLK_DIV    = 8'h0a;
  localparam ADDR_EMEM_ADDR   = 8'h10;
  localparam ADDR_EMEM_DATA   = 8'h20;

  localparam DEFAULT_SCLK_RATE = 32'h1000;

  localparam CORE_NAME0          = 32'h6d6b6d69; // "mkmi"
  localparam CORE_NAME1          = 32'h66202020; // "f   "
  localparam CORE_VERSION        = 32'h302e3130; // "0.10"


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg read_op_reg
  reg read_op_new
  reg write_op_reg
  reg write_op_new


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg [31 : 0]   tmp_read_data;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign read_data = tmp_read_data;


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
          read_op_reg  <= 0;
          write_op_reg <= 0;
        end
      else
        begin
          read_op_reg  <= read_op_new;
          write_op_reg <= write_op_new;
        end
    end // reg_update


  //----------------------------------------------------------------
  // api
  //----------------------------------------------------------------
  always @*
    begin : api
      read_op_new   = 0;
      write_op_new  = 0;
      tmp_read_data = 32'h00000000;

      if (cs)
        begin
          if (we)
            begin
              case (address)
                ADDR_CTRL:
                  begin
                    read_op_new  = write_data[CTRL_READ_BIT];
                    write_op_new = write_data[CTRL_WRITE_BIT];
                  end

                default:
                  begin
                  end
              endcase // case (address)
            end // if (we)

          else
            begin
              case (address)
                ADDR_NAME0:
                  tmp_read_data = CORE_NAME0;

                ADDR_NAME1:
                  tmp_read_data = CORE_NAME1;

                ADDR_VERSION:
                  tmp_read_data = CORE_VERSION;

                ADDR_STATUS:
                    tmp_read_data = {30'h0, {valid_reg, ready_reg}};

                default:
              endcase // case (address)
            end
        end
    end // api
endmodule // mkmif

//======================================================================
// EOF mkmif.v
//======================================================================

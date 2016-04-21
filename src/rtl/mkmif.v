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

             input wire           alarm,

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
  reg read_op_reg;
  reg read_op_new;
  reg write_op_reg;
  reg write_op_new;

  reg ready_reg;
  reg ready_new;
  reg ready_we;

  reg valid_reg;
  reg valid_new;
  reg valid_we;

  reg spi_sclk_reg;
  reg spi_sclk_new;
  reg spi_sclk_we;
  reg spi_di_reg;
  reg spi_di_new;
  reg spi_di_we;
  reg spi_do_sample0_reg;
  reg spi_do_sample1_reg;
  reg spi_do_reg;
  reg spi_do_we;

  reg [15 : 0] spi_sclk_ctr_reg;
  reg [15 : 0] spi_sclk_ctr_new;
  reg          spi_sclk_ctr_inc;
  reg          spi_sclk_ctr_rst;
  reg          spi_sclk_ctr_we;
  reg          spi_sclk_ctr_en;

  reg [15 : 0] spi_sclk_middle_ctr_reg;
  reg [15 : 0] spi_sclk_middle_ctr_new;
  reg          spi_sclk_middle_ctr_inc;
  reg          spi_sclk_middle_ctr_rst;
  reg          spi_sclk_middle_ctr_we;
  reg          spi_sclk_middle_ctr_en;

  reg alarm_sample0_reg;
  reg alarm_sample1_reg;
  reg alarm_flank0_reg;
  reg alarm_flank1_reg;
  reg alarm_reg;
  reg alarm_new;
  reg alarm_we;
  reg alarm_event_reg;
  reg alarm_event_new;
  reg alarm_event_set;
  reg alarm_event_rst;
  reg alarm_event_we;

  reg [10 : 0] addr_reg;
  reg          addr_we;

  reg [31 : 0] spi_read_data_reg;
  reg [31 : 0] spi_read_data_new;
  reg          spi_read_data_nxt;
  reg          spi_read_data_rst;
  reg          spi_read_data_we;

  reg [31 : 0] spi_write_data_reg;
  reg [31 : 0] spi_write_data_new;
  reg          spi_write_data_set;
  reg          spi_write_data_rst;
  reg          spi_write_data_we;


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
          ready_reg          <= 0;
          valid_reg          <= 0;
          alarm_reg          <= 0;
          read_op_reg        <= 0;
          write_op_reg       <= 0;
          alarm_sample0_reg  <= 0;
          alarm_sample1_reg  <= 0;
          alarm_event_reg    <= 0;
          spi_sclk_reg       <= 0;
          spi_di_reg         <= 0;
          spi_do_sample0_reg <= 0;
          spi_do_sample1_reg <= 0;
          spi_do_reg         <= 0;
          addr_reg           <= 11'h0;
          spi_read_data_reg  <= 32'h0;
          spi_write_data_reg <= 32'h0;
        end
      else
        begin
          read_op_reg  <= read_op_new;
          write_op_reg <= write_op_new;

          if (ready_we)
            ready_reg <= ready_new;

          if (valid_we)
            valid_reg <= valid_new;

          if (addr_we)
            addr_reg <= write_data[10 : 0];

          if (spi_read_data_we)
            spi_read_data_reg <= spi_read_data_new;

          if (spi_write_data_we)
            spi_write_data_reg <= spi_write_data_new;
        end
    end // reg_update


  //----------------------------------------------------------------
  // api
  //----------------------------------------------------------------
  always @*
    begin : api
      spi_write_data_set = 0;
      read_op_new        = 0;
      write_op_new       = 0;
      addr_we            = 0;
      tmp_read_data      = 32'h00000000;

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

                ADDR_EMEM_ADDR:
                  addr_we = 1;

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
                    tmp_read_data = {29'h0, {alarm_reg, valid_reg, ready_reg}};

                default:
                  begin
                  end
              endcase // case (address)
            end
        end
    end // api
endmodule // mkmif

//======================================================================
// EOF mkmif.v
//======================================================================
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
             input wire           spi_do,
             output wire          spi_di,

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

  localparam DEFAULT_SCLK_DIV = 16'h1000;

  localparam CORE_NAME0   = 32'h6d6b6d69; // "mkmi"
  localparam CORE_NAME1   = 32'h66202020; // "f   "
  localparam CORE_VERSION = 32'h302e3130; // "0.10"

  localparam SPI_READ_DATA_CMD    = 8'h03;
  localparam SPI_WRITE_DATA_CMD   = 8'h02;
  localparam SPI_READ_STATUS_CMD  = 8'h05;
  localparam SPI_WRITE_STATUS_CMD = 8'h01;

  localparam CTRL_IDLE        = 0;
  localparam CTRL_READ_START  = 1;
  localparam CTRL_READ_END    = 2;
  localparam CTRL_WRITE_START = 3;
  localparam CTRL_WRITE_END   = 4;
  localparam CTRL_ALARM_START = 5;
  localparam CTRL_ALARM_END   = 6;


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg          read_op_reg;
  reg          read_op_new;
  reg          write_op_reg;
  reg          write_op_new;

  reg          ready_reg;
  reg          ready_new;
  reg          ready_we;

  reg          valid_reg;
  reg          valid_new;
  reg          valid_we;

  reg          spi_cs_n_reg;
  reg          spi_cs_n_new;
  reg          spi_cs_n_we;

  reg          spi_sclk_reg;
  reg          spi_sclk_new;
  reg          spi_sclk_we;
  reg          spi_sclk_en;
  reg          spi_do_sample0_reg;
  reg          spi_do_sample1_reg;
  reg          spi_do_reg;
  reg          spi_do_we;

  reg  [7 : 0] spi_di_data_reg;
  reg  [7 : 0] spi_di_data_new;
  reg          spi_di_data_we;
  reg          spi_di_data_set;
  reg          spi_di_data_nxt;
  reg          spi_di_data_done;
  reg  [7 : 0] spi_di_data;

  reg  [2 : 0] spi_di_bit_ctr_reg;
  reg  [2 : 0] spi_di_bit_ctr_new;
  reg          spi_di_bit_ctr_we;

  reg  [2 : 0] spi_byte_ctr_reg;
  reg  [2 : 0] spi_byte_ctr_new;
  reg          spi_byte_ctr_inc;
  reg          spi_byte_ctr_rst;
  reg          spi_byte_ctr_we;

  reg [15 : 0] spi_sclk_div_reg;
  reg          spi_sclk_div_we;

  reg [15 : 0] spi_sclk_ctr_reg;
  reg [15 : 0] spi_sclk_ctr_new;
  reg          spi_sclk_ctr_mid;
  reg          spi_sclk_ctr_inc;
  reg          spi_sclk_ctr_rst;
  reg          spi_sclk_ctr_we;
  reg          spi_sclk_ctr_en;

  reg          alarm_sample0_reg;
  reg          alarm_sample1_reg;
  reg          alarm_flank0_reg;
  reg          alarm_flank1_reg;
  reg          alarm_event_reg;
  reg          alarm_event_new;
  reg          alarm_event_we;
  reg          alarm_reg;
  reg          alarm_new;
  reg          alarm_we;

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
  reg          spi_write_data_nxt;
  reg          spi_write_data_rst;
  reg          spi_write_data_we;

  reg [3 : 0]  mkmif_ctrl_reg;
  reg [3 : 0]  mkmif_ctrl_new;
  reg          mkmif_ctrl_we;


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg [31 : 0]   tmp_read_data;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign read_data = tmp_read_data;
  assign spi_sclk  = spi_sclk_reg;
  assign spi_cs_n  = spi_cs_n_reg;
  assign spi_di    = spi_di_data_reg[7];


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
          ready_reg           <= 0;
          valid_reg           <= 0;
          read_op_reg         <= 0;
          write_op_reg        <= 0;
          addr_reg            <= 11'h0;
          alarm_reg           <= 0;
          alarm_sample0_reg   <= 0;
          alarm_sample1_reg   <= 0;
          alarm_flank0_reg    <= 0;
          alarm_flank1_reg    <= 0;
          alarm_event_reg     <= 0;
          alarm_event_reg     <= 0;
          spi_sclk_div_reg    <= DEFAULT_SCLK_DIV;
          spi_sclk_reg        <= 0;
          spi_sclk_ctr_reg    <= 16'h0;
          spi_di_data_reg     <= 8'h0;
          spi_di_bit_ctr_reg  <= 3'h0;
          spi_do_sample0_reg  <= 0;
          spi_do_sample1_reg  <= 0;
          spi_do_reg          <= 0;
          spi_cs_n_reg        <= 1;
          spi_byte_ctr_reg    <= 12'h0;
          spi_read_data_reg   <= 32'h0;
          spi_write_data_reg  <= 32'h0;
          mkmif_ctrl_reg      <= CTRL_IDLE;
        end
      else
        begin
          read_op_reg  <= read_op_new;
          write_op_reg <= write_op_new;

          spi_do_sample0_reg <= spi_do;
          spi_do_sample1_reg <= spi_do_sample0_reg;

          alarm_sample0_reg <= alarm;
          alarm_sample1_reg <= alarm_sample0_reg;
          alarm_flank0_reg  <= alarm_sample1_reg;
          alarm_flank1_reg  <= alarm_flank0_reg;

          if (ready_we)
            ready_reg <= ready_new;

          if (valid_we)
            valid_reg <= valid_new;

          if (addr_we)
            addr_reg <= write_data[10 : 0];

          if (spi_cs_n_we)
            spi_cs_n_reg <= spi_cs_n_new;

          if (spi_sclk_we)
            spi_sclk_reg <= spi_sclk_new;

          if (spi_sclk_ctr_we)
            spi_sclk_ctr_reg <= spi_sclk_ctr_new;

          if (spi_sclk_div_we)
            spi_sclk_div_reg <= write_data[15 : 0];

          if (spi_di_data_we)
            spi_di_data_reg <= spi_di_data_new;

          if (spi_di_bit_ctr_we)
            spi_di_bit_ctr_reg <= spi_di_bit_ctr_new;

          if (spi_byte_ctr_we)
            spi_byte_ctr_reg <= spi_byte_ctr_new;

          if (spi_do_we)
            spi_do_reg <= spi_do_sample1_reg;

          if (spi_read_data_we)
            spi_read_data_reg <= spi_read_data_new;

          if (spi_write_data_we)
            spi_write_data_reg <= spi_write_data_new;

          if (mkmif_ctrl_we)
            mkmif_ctrl_reg <= mkmif_ctrl_new;

        end
    end // reg_update


  //----------------------------------------------------------------
  // api
  //----------------------------------------------------------------
  always @*
    begin : api
      spi_read_data_rst  = 0;
      spi_write_data_set = 0;
      spi_sclk_div_we    = 0;
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

                ADDR_SCLK_DIV:
                  spi_sclk_div_we = 1;

                ADDR_EMEM_ADDR:
                  addr_we = 1;

                ADDR_EMEM_DATA:
                  spi_write_data_set = 1;

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
                ADDR_SCLK_DIV:
                  tmp_read_data = {16'h0, spi_sclk_div_reg};

                ADDR_EMEM_ADDR:
                  tmp_read_data = {21'h0, addr_reg};

                ADDR_EMEM_DATA:
                  begin
                    tmp_read_data = spi_read_data_reg;
                    spi_read_data_rst = 1;
                  end

                default:
                  begin
                  end
              endcase // case (address)
            end
        end
    end // api


  //----------------------------------------------------------------
  // alarm_detect
  //
  // Detect an alarm signal and when that happens sets the
  // alarm_event register.
  //----------------------------------------------------------------
  always @*
    begin : alarm_detect
      alarm_event_new = 0;
      alarm_event_we  = 0;

      if ((alarm_flank0_reg) && (!alarm_flank1_reg))
        begin
          alarm_event_new = 1;
          alarm_event_we  = 1;
        end
    end // alarm_detect


  //----------------------------------------------------------------
  // spi_di_gen
  //
  // Generate the bitstream to be written as data into the
  // external SPI connected memory. The generator also counts
  // bits shifted out and signals when 8 bits has been
  // shifted.
  //----------------------------------------------------------------
  always @*
    begin : spi_di_gen
      spi_di_data_new    = 8'h00;
      spi_di_data_we     = 0;
      spi_di_data_done   = 0;
      spi_di_bit_ctr_new = 3'h0;
      spi_di_bit_ctr_we  = 0;

      if (spi_di_bit_ctr_reg == 3'h7)
        spi_di_data_done = 1;

      if (spi_di_data_set)
        begin
          spi_di_data_new    = spi_di_data;
          spi_di_data_we     = 1;
          spi_di_bit_ctr_new = 3'h0;
          spi_di_bit_ctr_we  = 1;
        end

      if (spi_di_data_nxt)
        begin
          spi_di_data_new = {spi_di_data_reg[6 : 0], 1'b0};
          spi_di_data_we  = 1;

          spi_di_bit_ctr_new  = spi_di_bit_ctr_reg + 1'b1;
          spi_di_bit_ctr_we   = 1;
        end

    end // spi_di_gen

  //----------------------------------------------------------------
  // spi_write_data_gen
  //
  // Generates the data to be written.
  //----------------------------------------------------------------
  always @*
    begin : spi_write_data_gen
      spi_write_data_new = 32'h00;
      spi_write_data_we  = 0;

      if (spi_write_data_set)
        begin
          spi_write_data_new = write_data;
          spi_write_data_we  = 1;
        end

      if (spi_write_data_rst)
        begin
          spi_write_data_new = 32'h00;
          spi_write_data_we  = 1;
        end
    end // spi_write_data_gen


  //----------------------------------------------------------------
  // spi_sclk_gen
  //
  // Generator of the spi_sclk clock. The generator includes
  // a detector for midpoint of a flank.
  //----------------------------------------------------------------
  always @*
    begin : siphash_sclk_gen
      spi_sclk_ctr_new = 16'h00;
      spi_sclk_ctr_we  = 0;
      spi_sclk_we      = 0;
      spi_sclk_new     = ~spi_sclk_reg;
      spi_sclk_ctr_mid = 0;

      if (spi_sclk_ctr_reg == {1'b0, spi_sclk_div_reg[15 : 1]})
        spi_sclk_ctr_mid = 1;

      if (spi_sclk_en)
        begin
          if (spi_sclk_ctr_reg == spi_sclk_div_reg)
            begin
              spi_sclk_ctr_new = 16'h00;
              spi_sclk_we      = 1;
            end
          else
            spi_sclk_ctr_new = spi_sclk_ctr_new + 1'b1;
        end
    end // siphash_sclk_gen


  //----------------------------------------------------------------
  // spi_byte_ctr
  //
  // Byte counter used by the FSM to keep track of the bytes
  // being read or written.
  //----------------------------------------------------------------
  always @*
    begin : spi_byte_ctr
      spi_byte_ctr_new = 12'h0;
      spi_byte_ctr_we  = 1'b0;

      if (spi_byte_ctr_rst)
        begin
          spi_byte_ctr_new = 12'h0;
          spi_byte_ctr_we  = 1'b1;
        end

      if (spi_byte_ctr_inc)
        begin
          spi_byte_ctr_new = spi_byte_ctr_reg + 1'b1;
          spi_byte_ctr_we  = 1'b1;
        end
    end // spi_byte_ctr


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
      spi_sclk_en        = 0;
      spi_di_data_set    = 0;
      spi_di_data_nxt    = 0;
      spi_di_data        = 8'h00;
      spi_do_we          = 0;
      spi_cs_n_new       = 0;
      spi_cs_n_we        = 0;
      spi_byte_ctr_inc   = 0;
      spi_byte_ctr_rst   = 0;
      spi_write_data_rst = 0;
      mkmif_ctrl_new     = CTRL_IDLE;
      mkmif_ctrl_we      = 0;

      case (mkmif_ctrl_reg)
        CTRL_IDLE:
          begin
            if (read_op_reg)
              begin
                mkmif_ctrl_new = CTRL_READ_START;
                mkmif_ctrl_we  = 1;
              end

            if (write_op_reg)
              begin
                mkmif_ctrl_new = CTRL_WRITE_START;
                mkmif_ctrl_we  = 1;
              end

            if (alarm_event_reg)
              begin
                mkmif_ctrl_new = CTRL_ALARM_START;
                mkmif_ctrl_we  = 1;
              end
          end

        CTRL_READ_START:
          begin
            mkmif_ctrl_new = CTRL_IDLE;
            mkmif_ctrl_we  = 1;
          end

        CTRL_WRITE_START:
          begin
            mkmif_ctrl_new = CTRL_IDLE;
            mkmif_ctrl_we  = 1;
          end

        CTRL_ALARM_START:
          begin
            if (spi_di_data_done)
              begin
                mkmif_ctrl_new = CTRL_IDLE;
                mkmif_ctrl_we  = 1;
              end
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

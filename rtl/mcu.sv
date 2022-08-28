//============================================================================
//  Irem M72 for MiSTer FPGA - 8051 protection and sample playback MCU
//
//  Copyright (C) 2022 Martin Donlon
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================

module mcu(
    input CLK_32M,
    input ce_8m,
    input reset,

    // shared ram
    output reg [11:0] ext_ram_addr,
    input [7:0] ext_ram_din,
    output reg [7:0] ext_ram_dout,
    output reg ext_ram_cs,
    output reg ext_ram_we,
    input ext_ram_int,

    // z80 latch
    input [7:0] z80_din,
    input z80_latch_en,

    // sample output, 8-bit unsigned
    output [7:0] sample_data,

    // ioctl
    input clk_bram,
    input bram_wr,
    input [7:0] bram_data,
    input [19:0] bram_addr,
    input bram_prom_cs,
    input bram_samples_cs,

    output [15:0] dbg_rom_addr,
    output reg valid_rom,
    input SND2,
    input [7:0] IO_IN
);

initial valid_rom = 0;

wire [6:0] ram_addr;
wire [7:0] ram_din, ram_dout;
wire ram_we, ram_cs;

wire [7:0] sample_port;
reg [7:0] bypass_sample_data;
reg valid_samples = 0;
always @(posedge clk_bram) if (bram_samples_cs & bram_wr) valid_samples <= 1;
always @(posedge clk_bram) if (bram_prom_cs & bram_wr) valid_rom <= 1;

reg [2:0] delayed_ce_count = 0;
wire delayed_ce = ce_8m & ~|delayed_ce_count;

assign sample_data = valid_samples ? (valid_rom ? sample_port : bypass_sample_data ): 8'h80;

reg [17:0] bypass_sample_addr = 18'b0;
wire [17:0] sample_start[28] = '{
		18'h00000, 18'h00020, 18'h01800, 18'h02da0, 18'h03be0, 18'h05ae0, 18'h06100, 18'h06de0,
		18'h07260, 18'h07a60, 18'h08720, 18'h0a5c0, 18'h0c3c0, 18'h0c7a0, 18'h0e140, 18'h0fb00,
		18'h10fa0, 18'h10fc0, 18'h10fe0, 18'h11f40, 18'h12b20, 18'h130a0, 18'h13c60, 18'h14740,
		18'h153c0, 18'h197e0, 18'h1af40, 18'h1c080
		};

always @(posedge CLK_32M) begin
reg [9:0] div = 0;
	if (SND2) begin
		bypass_sample_addr = sample_start[IO_IN];
		div = 10'd0;
	end
	if (ce_8m && div == 0) begin
			bypass_sample_data <= sample_data_dout;
			bypass_sample_addr <= bypass_sample_addr + 18'd1;
	end
	if (ce_8m && bypass_sample_data != 0)
		div <= div + 10'd1;
end

dpramv_cen #(.widthad_a(7)) internal_ram
(
    .clock_a(CLK_32M),
    .address_a(ram_addr),
    .q_a(ram_din),
    .wren_a(ram_we),
    .data_a(ram_dout),
    .cen_a(delayed_ce),

    .clock_b(CLK_32M),
    .address_b(0),
    .data_b(),
    .wren_b(1'd0),
    .q_b(),
    .cen_b(1'b0)
);

dpramv_cen #(.widthad_a(13)) prom
(
    .clock_a(CLK_32M),
    .address_a(prom_addr[12:0]),
    .q_a(prom_data),
    .wren_a(1'b0),
    .data_a(),
    .cen_a(delayed_ce),

    .clock_b(clk_bram),
    .address_b(bram_addr[12:0]),
    .data_b(bram_data),
    .wren_b(bram_prom_cs),
    .q_b(),
    .cen_b(1'b1)
);



/// SAMPLE ROM
dpramv #(.widthad_a(18)) sample_rom
(
    .clock_a(CLK_32M),
    .address_a(valid_rom ? sample_addr : bypass_sample_addr),
    .q_a(sample_data_dout),
    .wren_a(1'b0),
    .data_a(),

    .clock_b(clk_bram),
    .address_b(bram_addr[17:0]),
    .data_b(bram_data),
    .wren_b(bram_samples_cs),
    .q_b()
);

wire [7:0] sample_data_dout;
reg [7:0] sample_data_latch;
reg [17:0] sample_addr;

reg [7:0] z80_latch;
reg z80_latch_int = 0;

wire [7:0] ext_dout;
wire [15:0] ext_addr;
wire ext_cs, ext_we;

enum { SAMPLE, Z80, RAM } ext_src = SAMPLE;

always @(posedge CLK_32M) begin
    if (z80_latch_en) begin
        z80_latch <= z80_din;
        z80_latch_int <= 1;
    end
     
     if (ce_8m & |delayed_ce_count) delayed_ce_count <= delayed_ce_count - 3'd1;

    if (delayed_ce) begin
        dbg_rom_addr <= prom_addr;
        
        ext_ram_cs <= 0;
        ext_ram_we <= 0;
        if (ext_cs) begin
            casex (ext_addr)
            16'h0000: if (ext_we) begin
                sample_addr[12:0] <= { ext_dout, 5'd0 };
            end else begin
                ext_src <= SAMPLE;
                sample_addr <= sample_addr + 18'd1;
            end
            
            16'h0001: if (ext_we) begin
                sample_addr[17:13] <= ext_dout[4:0];
            end

            16'h0002: if (ext_we) begin
                z80_latch_int <= 0;
            end else begin
                ext_src <= Z80;
            end

            16'hcxxx: begin
                if (ext_we) delayed_ce_count <= 7;
                ext_ram_addr <= ext_addr[11:0];
                ext_ram_dout <= ext_dout;
                ext_ram_cs <= ext_cs;
                ext_ram_we <= ext_cs & ext_we;
                if (~ext_we) ext_src <= RAM;
                end
            endcase
        end
    end
end

wire [7:0] ext_din = ext_src == SAMPLE ? sample_data_dout : ext_src == Z80 ? z80_latch : ext_ram_din;

wire [15:0] prom_addr;
wire [7:0] prom_data;

mc8051_core mc8051(
    .clk(CLK_32M),
    .cen(delayed_ce),
    .reset(reset | ~valid_rom),

    // prom
    .rom_data_i(prom_data),
    .rom_adr_o(prom_addr),

    // internal ram
    .ram_data_i(ram_din),
    .ram_data_o(ram_dout),
    .ram_adr_o(ram_addr),
    .ram_wr_o(ram_we),
    .ram_en_o(ram_cs),

    // interrupt lines
    .int0_i(~ext_ram_int),
    .int1_i(~z80_latch_int),

    // sample dac
    .p1_o(sample_port),

    // external ram
    .datax_i(ext_din),
    .datax_o(ext_dout),
    .adrx_o(ext_addr),
    .memx_o(ext_cs),
    .wrx_o(ext_we)
);

endmodule
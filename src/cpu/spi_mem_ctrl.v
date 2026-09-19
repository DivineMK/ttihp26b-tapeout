/*
 * Copyright (c) 2026 LKhanh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Single-SPI memory controller for the RP2040 emulator using spi-ram-emu:
 * - 23LC512 protocol: 0x03 read / 0x02 write, 16-bit address, byte bursts
 * - SPI mode 0 (CPOL=0, CPHA=0), MSB first
 * - CS-high cooldown between transactions for RP2040 PIO re-arm
 * - same byte-level handshake as qspi_controller for mem_ctrl swap
 * - split style: always_comb drives _d values, always_ff flops them to _q
 */

`default_nettype none

module spi_mem_ctrl #(
    // CLK_DIV=8 at 64 MHz gives ~8 MHz SCK, within spi-ram-emu 12-15 MHz limit
    parameter CLK_DIV = 8,
    // CS-high hold between transactions for RP2040 PIO/DMA cleanup
    parameter COOLDOWN_CYCLES = 48
) (
    input  wire        clk,
    input  wire        rstn,

    // Pin lanes used: out[0] for MOSI, in[1] for MISO, rest tied off
    // - PMOD mapping: uio[1]=MOSI, uio[2]=MISO, uio[3]=SCK, uio[0]=CS
    input  wire [3:0]  spi_data_in,   // only bit [1] (MISO) is sampled
    output reg  [3:0]  spi_data_out,  // only bit [0] (MOSI) is driven
    output reg  [3:0]  spi_data_oe,   // 0001 = driving MOSI, 0000 = listening
    output reg         spi_clk_out,   // SCK, idle low (SPI mode 0)
    output reg         spi_cs,        // chip select, idle high

    // Byte port to tinyqv_mem_ctrl, same shape as qspi_controller:
    // - addr + byte in/out, start/stop/stall control
    input  wire [24:0] addr_in,
    input  wire [7:0]  data_in,
    input  wire        start_read,
    input  wire        start_write,
    input  wire        stall_txn,     // hold the clock after a byte (reads)
    input  wire        stop_txn,      // mem_ctrl wants this burst to end

    output reg  [7:0]  data_out,      // fresh byte from the RAM
    output reg         data_req,      // pulsed when we need the next TX byte
    output reg         data_ready,    // pulsed for one cycle per RX byte
    output wire        busy           // high whenever we own the bus
);

    localparam HALF_DIV = CLK_DIV / 2;

    // Transaction phases: command, address, data bytes
    // - ST_STALL: SCK parked for full instruction buffer
    // - ST_COOLDOWN: CS-high gap for RP2040 re-arm
    localparam [2:0]
        ST_IDLE     = 3'd0,
        ST_CMD      = 3'd1,
        ST_ADDR     = 3'd2,
        ST_DATA     = 3'd3,
        ST_STALL    = 3'd4,
        ST_COOLDOWN = 3'd5;

    // Flop outputs (_q), updated in always_ff, driven by _d in always_comb
    // - ports (spi_*, data_*) are the _q for their lanes, paired with *_d below
    reg [2:0]  state_q;
    reg [4:0]  div_cnt_q;      // core clocks within one SCK half period
    reg [4:0]  bit_cnt_q;      // bits left in phase (8 cmd, 16 addr, 8 data)
    reg [7:0]  tx_shift_q;     // remaining MOSI bits
    reg [15:0] addr_shift_q;   // address bits, MSB first
    reg [7:0]  rx_shift_q;     // collected MISO bits
    reg        is_writing_q;
    reg [5:0]  cooldown_cnt_q; // CS-high gap countdown
    reg        stop_latched_q; // latched stop_txn, short pulse may land mid-bit

    // Comb drivers (_d), flopped into _q by always_ff
    reg [2:0]  state_d;
    reg [4:0]  div_cnt_d;
    reg [4:0]  bit_cnt_d;
    reg [7:0]  tx_shift_d;
    reg [15:0] addr_shift_d;
    reg [7:0]  rx_shift_d;
    reg        is_writing_d;
    reg [5:0]  cooldown_cnt_d;
    reg        stop_latched_d;
    reg [3:0]  spi_data_out_d;
    reg [3:0]  spi_data_oe_d;
    reg        spi_clk_out_d;
    reg        spi_cs_d;
    reg [7:0]  data_out_d;
    reg        data_req_d;
    reg        data_ready_d;

    // Busy during burst or cooldown, blocks next start from mem_ctrl
    assign busy = (state_q != ST_IDLE) || (cooldown_cnt_q != 6'd0);

    // 25-bit to 16-bit map using spi-ram-emu:
    // - addr_in[24]=0 (code) to 0x0000-0x7FFF, addr_in[24]=1 (data) to 0x8000-0xFFFF
    // - split avoids gp (0x01000400) aliasing onto 0x0400 past 1 KB programs
    wire [15:0] mapped_addr = {addr_in[24], addr_in[14:0]};

    wire should_stop = stop_txn || stop_latched_q;

    // SCK edge helpers on the registered divider
    // - at_half: mid-bit point, SCK rises here
    // - at_end: end of bit, SCK falls here
    wire at_half  = (div_cnt_q == HALF_DIV - 1);
    wire at_end   = (div_cnt_q == CLK_DIV - 1);
    wire last_bit = (bit_cnt_q == 5'd1);

    always @(*) begin
        // Hold defaults, pulses default low for one-cycle data_req/data_ready
        state_d        = state_q;
        div_cnt_d      = div_cnt_q;
        bit_cnt_d      = bit_cnt_q;
        tx_shift_d     = tx_shift_q;
        addr_shift_d   = addr_shift_q;
        rx_shift_d     = rx_shift_q;
        is_writing_d   = is_writing_q;
        spi_data_out_d = spi_data_out;
        spi_data_oe_d  = spi_data_oe;
        spi_clk_out_d  = spi_clk_out;
        spi_cs_d       = spi_cs;
        data_out_d     = data_out;
        data_req_d     = 1'b0;
        data_ready_d   = 1'b0;

        // Cooldown countdown, branch loads below override on abort/stop
        cooldown_cnt_d = (cooldown_cnt_q > 6'd0) ? cooldown_cnt_q - 6'd1 : 6'd0;

        // Latch stop_txn, clear on park, stop_txn wins on conflict
        stop_latched_d = stop_latched_q;
        if (stop_txn)
            stop_latched_d = 1'b1;
        else if (state_q == ST_COOLDOWN || state_q == ST_IDLE)
            stop_latched_d = 1'b0;

        case (state_q)
            ST_IDLE: begin
                // Parked: CS high, SCK low, MOSI released, divider reset
                spi_cs_d      = 1'b1;
                spi_clk_out_d = 1'b0;
                spi_data_oe_d = 4'b0000;
                div_cnt_d     = 5'd0;

                // Start gated on cooldown, busy holds mem_ctrl off meanwhile
                if ((start_read || start_write) && (cooldown_cnt_q == 6'd0)) begin
                    is_writing_d   = start_write;
                    spi_cs_d       = 1'b0;
                    spi_data_oe_d  = 4'b0001;
                    addr_shift_d   = mapped_addr;
                    state_d        = ST_CMD;
                    bit_cnt_d      = 5'd8;
                    div_cnt_d      = 5'd0;

                    // First MOSI bit out now, rest from tx_shift on falling edges
                    // - constants are 0x02/0x03 with MSB already consumed
                    if (start_write) begin
                        spi_data_out_d = 4'b0000; // MSB of 0x02 = 0
                        tx_shift_d     = 8'b0000_0100;
                    end else begin
                        spi_data_out_d = 4'b0000; // MSB of 0x03 = 0
                        tx_shift_d     = 8'b0000_0110;
                    end
                end
            end

            ST_CMD: begin
                if (should_stop) begin
                    // Early abort on fetch restart race, take cooldown
                    spi_cs_d       = 1'b1;
                    spi_clk_out_d  = 1'b0;
                    spi_data_oe_d  = 4'b0000;
                    cooldown_cnt_d = COOLDOWN_CYCLES[5:0];
                    state_d        = ST_COOLDOWN;
                end else if (at_half) begin
                    // Mid-bit: raise SCK, emulator samples MOSI
                    spi_clk_out_d = 1'b1;
                    div_cnt_d     = div_cnt_q + 5'd1;
                end else if (at_end) begin
                    // End of bit: drop SCK, shift next MOSI bit
                    spi_clk_out_d = 1'b0;
                    div_cnt_d     = 5'd0;

                    if (last_bit) begin
                        // Command done, enter address phase
                        state_d              = ST_ADDR;
                        bit_cnt_d            = 5'd16;
                        spi_data_out_d[0]    = addr_shift_q[15];
                        addr_shift_d         = {addr_shift_q[14:0], 1'b0};
                    end else begin
                        bit_cnt_d            = bit_cnt_q - 5'd1;
                        spi_data_out_d[0]    = tx_shift_q[7];
                        tx_shift_d           = {tx_shift_q[6:0], 1'b0};
                    end
                end else begin
                    div_cnt_d = div_cnt_q + 5'd1;
                end
            end

            ST_ADDR: begin
                if (should_stop) begin
                    // Early abort, take cooldown
                    spi_cs_d       = 1'b1;
                    spi_clk_out_d  = 1'b0;
                    spi_data_oe_d  = 4'b0000;
                    cooldown_cnt_d = COOLDOWN_CYCLES[5:0];
                    state_d        = ST_COOLDOWN;
                end else if (at_half) begin
                    // Mid-bit: raise SCK
                    spi_clk_out_d = 1'b1;
                    div_cnt_d     = div_cnt_q + 5'd1;
                end else if (at_end) begin
                    // End of bit: drop SCK
                    spi_clk_out_d = 1'b0;
                    div_cnt_d     = 5'd0;

                    if (last_bit) begin
                        // Address done, enter data phase
                        // - writes: keep MOSI driven with first data byte
                        // - reads: release MOSI for MISO return
                        state_d  = ST_DATA;
                        bit_cnt_d = 5'd8;
                        if (is_writing_q) begin
                            spi_data_oe_d      = 4'b0001;
                            spi_data_out_d[0]  = data_in[7];
                            tx_shift_d         = {data_in[6:0], 1'b0};
                        end else begin
                            spi_data_oe_d      = 4'b0000;
                            spi_data_out_d[0]  = 1'b0;
                        end
                    end else begin
                        bit_cnt_d            = bit_cnt_q - 5'd1;
                        spi_data_out_d[0]    = addr_shift_q[15];
                        addr_shift_d         = {addr_shift_q[14:0], 1'b0};
                    end
                end else begin
                    div_cnt_d = div_cnt_q + 5'd1;
                end
            end

            ST_DATA: begin
                if (at_half) begin
                    // Rising edge: bit transfer point
                    spi_clk_out_d = 1'b1;
                    div_cnt_d     = div_cnt_q + 5'd1;

                    if (!is_writing_q) begin
                        // Read: shift MISO in, publish byte + pulse on last bit
                        rx_shift_d = {rx_shift_q[6:0], spi_data_in[1]};
                        if (last_bit) begin
                            data_out_d   = {rx_shift_q[6:0], spi_data_in[1]};
                            data_ready_d = 1'b1;
                        end
                    end else begin
                        // Write, 8th SCK rising edge: current byte fully on wire
                        // - pulse data_req so mem_ctrl muxes next byte before next fall
                        // - skip when stopping: burst ends, no next byte exists
                        if (last_bit && !should_stop) begin
                            data_req_d = 1'b1;
                        end
                    end
                end else if (at_end) begin
                    // Falling edge: byte done handling, stop/stall/continue
                    spi_clk_out_d = 1'b0;
                    div_cnt_d     = 5'd0;

                    if (last_bit) begin
                        if (should_stop) begin
                            // Burst end: raise CS, release MOSI, take cooldown
                            spi_cs_d       = 1'b1;
                            spi_data_oe_d  = 4'b0000;
                            cooldown_cnt_d = COOLDOWN_CYCLES[5:0];
                            state_d        = ST_COOLDOWN;
                        end else if (stall_txn && !is_writing_q) begin
                            // Full instruction buffer: freeze SCK, hold CS for resume
                            state_d   = ST_STALL;
                            bit_cnt_d = 5'd8;
                        end else begin
                            // Burst continue: reload bit counter and next TX byte
                            bit_cnt_d = 5'd8;
                            if (is_writing_q) begin
                                spi_data_out_d[0] = data_in[7];
                                tx_shift_d        = {data_in[6:0], 1'b0};
                            end
                        end
                    end else begin
                        bit_cnt_d = bit_cnt_q - 5'd1;
                        if (is_writing_q) begin
                            spi_data_out_d[0] = tx_shift_q[7];
                            tx_shift_d        = {tx_shift_q[6:0], 1'b0};
                        end
                    end
                end else begin
                    div_cnt_d = div_cnt_q + 5'd1;
                end
            end

            ST_STALL: begin
                // CS held low, SCK low, resume on !stall_txn, stop wins
                spi_clk_out_d = 1'b0;
                div_cnt_d     = 5'd0;

                if (should_stop) begin
                    spi_cs_d       = 1'b1;
                    spi_data_oe_d  = 4'b0000;
                    cooldown_cnt_d = COOLDOWN_CYCLES[5:0];
                    state_d        = ST_COOLDOWN;
                end else if (!stall_txn) begin
                    state_d = ST_DATA;
                end
            end

            ST_COOLDOWN: begin
                // CS high, SCK low, wait counter for RP2040 PIO reset
                spi_cs_d      = 1'b1;
                spi_clk_out_d = 1'b0;
                spi_data_oe_d = 4'b0000;
                div_cnt_d     = 5'd0;

                if (cooldown_cnt_q == 6'd0) begin
                    state_d = ST_IDLE;
                end
            end

            default: state_d = ST_IDLE;
        endcase
    end

    always @(posedge clk) begin
        if (!rstn) begin
            state_q        <= ST_IDLE;
            div_cnt_q      <= 5'd0;
            bit_cnt_q      <= 5'd0;
            spi_clk_out    <= 1'b0;
            spi_cs         <= 1'b1;
            spi_data_out   <= 4'b0000;
            spi_data_oe    <= 4'b0000;
            tx_shift_q     <= 8'h00;
            addr_shift_q   <= 16'h0000;
            rx_shift_q     <= 8'h00;
            is_writing_q   <= 1'b0;
            data_out       <= 8'h00;
            data_req       <= 1'b0;
            data_ready     <= 1'b0;
            cooldown_cnt_q <= 6'd0;
            stop_latched_q <= 1'b0;
        end else begin
            state_q        <= state_d;
            div_cnt_q      <= div_cnt_d;
            bit_cnt_q      <= bit_cnt_d;
            spi_clk_out    <= spi_clk_out_d;
            spi_cs         <= spi_cs_d;
            spi_data_out   <= spi_data_out_d;
            spi_data_oe    <= spi_data_oe_d;
            tx_shift_q     <= tx_shift_d;
            addr_shift_q   <= addr_shift_d;
            rx_shift_q     <= rx_shift_d;
            is_writing_q   <= is_writing_d;
            data_out       <= data_out_d;
            data_req       <= data_req_d;
            data_ready     <= data_ready_d;
            cooldown_cnt_q <= cooldown_cnt_d;
            stop_latched_q <= stop_latched_d;
        end
    end

endmodule

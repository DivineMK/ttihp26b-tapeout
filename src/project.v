/*
 * Copyright (c) 2026 LKhanh / Antigravity
 * SPDX-License-Identifier: Apache-2.0
 *
 * Top-level for Tiny Tapeout: TinyQV RISC-V SoC with Dual Memory Backend:
 *   - strap = 0 (default): Single-SPI master for RP2040 spi-ram-emu (23LC512 protocol)
 *   - strap = 1: Quad-SPI master for QSPI PMOD (W25Q128 Flash + APS6404 PSRAM)
 * Strap is sampled from ui_in[0] on reset release.
 */

`default_nettype none

module tt_um_lkhanh_cordic (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // Always 1 when powered
    input  wire       clk,      // System clock
    input  wire       rst_n     // Active-low reset
);

    // Strap sampling on ui_in[0] during reset, held after
    // - 0 for Single-SPI (spi-ram-emu), 1 for QSPI PMOD (Flash + PSRAM)
    // - must settle before rst_n rises, no mid-run switching
    reg strap_mode;
    always @(posedge clk) begin
        if (!rst_n)
            strap_mode <= ui_in[0];
    end

    // Peripheral bus: 0x8000000 and up, not via SPI
    // - 0x8000000 for GPIO, 0x8000010 for UART
    wire [27:0] peri_addr;
    wire [1:0]  peri_write_n;
    wire [1:0]  peri_read_n;
    wire        peri_read_complete;
    wire [31:0] peri_data_out;
    wire        peri_data_ready;
    reg  [31:0] peri_data_in;

    // Interrupts on ui_in[6:3], timer on ui_in[7]
    // - ui_in[0] excluded, strap pin never fires edge interrupt
    wire [3:0]  interrupt_req = ui_in[6:3];
    wire        timer_interrupt = ui_in[7];

    // SPI Interface Signals from TinyQV
    wire [3:0]  spi_data_in;
    wire [3:0]  spi_data_out;
    wire [3:0]  spi_data_oe;
    wire        spi_clk_out;
    wire        spi_flash_select;
    wire        spi_ram_a_select;
    wire        spi_ram_b_select;

    // Debug signals
    wire        debug_instr_complete;
    wire        debug_instr_ready;
    wire        debug_instr_valid;
    wire        debug_fetch_restart;
    wire        debug_data_ready;
    wire        debug_interrupt_pending;
    wire        debug_branch;
    wire        debug_early_branch;
    wire        debug_ret;
    wire        debug_reg_wen;
    wire        debug_counter_0;
    wire        debug_data_continue;
    wire        debug_stall_txn;
    wire        debug_stop_txn;
    wire [3:0]  debug_rd;

    // Instantiate TinyQV Core + Dual Memory Controller
    tinyQV tinyqv_inst (
        .clk                    (clk),
        .rstn                   (rst_n),

        .data_addr              (peri_addr),
        .data_write_n           (peri_write_n),
        .data_read_n            (peri_read_n),
        .data_read_complete     (peri_read_complete),
        .data_out               (peri_data_out),
        .data_ready             (peri_data_ready),
        .data_in                (peri_data_in),

        .interrupt_req          (interrupt_req),
        .timer_interrupt        (timer_interrupt),

        .spi_data_in            (spi_data_in),
        .spi_data_out           (spi_data_out),
        .spi_data_oe            (spi_data_oe),
        .spi_clk_out            (spi_clk_out),
        .spi_flash_select       (spi_flash_select),
        .spi_ram_a_select       (spi_ram_a_select),
        .spi_ram_b_select       (spi_ram_b_select),
        .strap_mode             (strap_mode),

        .debug_instr_complete   (debug_instr_complete),
        .debug_instr_ready      (debug_instr_ready),
        .debug_instr_valid      (debug_instr_valid),
        .debug_fetch_restart    (debug_fetch_restart),
        .debug_data_ready       (debug_data_ready),
        .debug_interrupt_pending(debug_interrupt_pending),
        .debug_branch           (debug_branch),
        .debug_early_branch     (debug_early_branch),
        .debug_ret              (debug_ret),
        .debug_reg_wen          (debug_reg_wen),
        .debug_counter_0        (debug_counter_0),
        .debug_data_continue    (debug_data_continue),
        .debug_stall_txn        (debug_stall_txn),
        .debug_stop_txn         (debug_stop_txn),
        .debug_rd               (debug_rd)
    );

    // =========================================================================
    // PMOD Pin Mapping on uio[7:0]
    //   uio[0] - CS0 (Flash in QSPI, primary CS in Single-SPI)
    //   uio[1] - SD0 / MOSI
    //   uio[2] - SD1 / MISO
    //   uio[3] - SCK
    //   uio[4] - SD2
    //   uio[5] - SD3
    //   uio[6] - CS1 (RAMA)
    //   uio[7] - CS2 (RAMB)
    // =========================================================================

    // Outputs: CS and SCK always driven, data lines follow spi_data_oe
    // - single-SPI mode: SD2/SD3 released
    assign uio_out[0] = spi_flash_select;
    assign uio_out[1] = spi_data_out[0];
    assign uio_out[2] = spi_data_out[1];
    assign uio_out[3] = spi_clk_out;
    assign uio_out[4] = spi_data_out[2];
    assign uio_out[5] = spi_data_out[3];
    assign uio_out[6] = spi_ram_a_select;
    assign uio_out[7] = spi_ram_b_select;

    // Output Enables
    assign uio_oe[0] = 1'b1;
    assign uio_oe[1] = spi_data_oe[0];
    assign uio_oe[2] = spi_data_oe[1];
    assign uio_oe[3] = 1'b1;
    assign uio_oe[4] = spi_data_oe[2];
    assign uio_oe[5] = spi_data_oe[3];
    assign uio_oe[6] = 1'b1;
    assign uio_oe[7] = 1'b1;

    // Inputs: mem_ctrl lane numbers, not uio pin numbers
    // - MOSI on uio[1] to lane 0, MISO on uio[2] to lane 1
    assign spi_data_in[0] = uio_in[1];
    assign spi_data_in[1] = uio_in[2];
    assign spi_data_in[2] = uio_in[4];
    assign spi_data_in[3] = uio_in[5];

    // =========================================================================
    // Peripherals (Memory-mapped above 0x8000000 via tp)
    // 0x8000000: GPIO Output Register
    // 0x8000010: UART TX Data Register
    // =========================================================================
    reg [7:0] gpio_out;
    wire is_gpio = (peri_addr[27:4] == 24'h800000);
    wire is_uart = (peri_addr[27:4] == 24'h800001);

    wire uart_txd;
    wire uart_tx_busy;
    wire uart_tx_start = (peri_write_n != 2'b11) && is_uart;

    always @(posedge clk) begin
        if (!rst_n) begin
            gpio_out <= 8'h00;
        end else if (peri_write_n != 2'b11 && is_gpio) begin
            gpio_out <= peri_data_out[7:0];
        end
    end

    // Peripheral read, combinational for same-cycle loads
    // - unmapped addresses read zero, no CPU hang
    always @(*) begin
        if (is_gpio)
            peri_data_in = {24'h0, gpio_out};
        else if (is_uart)
            peri_data_in = {31'h0, uart_tx_busy};
        else
            peri_data_in = 32'h0;
    end

    assign peri_data_ready = 1'b1; // All peripheral accesses complete immediately

    // UART transmitter (115200 baud at 64 MHz), write-only from CPU
    // - read returns busy flag in bit 0
    uart_tx #(
        .CLK_HZ   (64_000_000),
        .BIT_RATE (115_200)
    ) i_uart_tx (
        .clk          (clk),
        .resetn       (rst_n),
        .uart_txd     (uart_txd),
        .uart_tx_busy (uart_tx_busy),
        .uart_tx_en   (uart_tx_start),
        .uart_tx_data (peri_data_out[7:0])
    );

    // Dedicated outputs:
    // - uo_out[0]: UART TX
    // - uo_out[1]: strap_mode indicator for boot mode check
    // - uo_out[7:2]: GPIO output bits [7:2]
    assign uo_out[0]   = uart_txd;
    assign uo_out[1]   = strap_mode;
    assign uo_out[7:2] = gpio_out[7:2];

    wire _unused = &{ena, ui_in[2:1], peri_addr[3:0], peri_read_complete, debug_instr_complete,
                     debug_instr_ready, debug_instr_valid, debug_fetch_restart, debug_data_ready,
                     debug_interrupt_pending, debug_branch, debug_early_branch, debug_ret,
                     debug_reg_wen, debug_counter_0, debug_data_continue, debug_stall_txn,
                     debug_stop_txn, debug_rd, 1'b0};

endmodule

`default_nettype none
`timescale 1ns / 1ps

/* SDF-annotated gate-level testbench (LOCAL ONLY - never used by CI).
   Identical to tb.v plus $sdf_annotate for true timed simulation.
   Select with: GATES=yes SDF=1 make
   Corner baked in below; edit SDF_FILE to switch corners.
*/
module tb ();

  // Dump the signals to a FST file. You can view it with gtkwave or surfer.
  initial begin
    $dumpfile("tb.fst");
    $dumpvars(0, tb);
    #1;
  end

  // Wire up the inputs and outputs:
  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  reg [7:0] uio_in;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

  // Replace tt_um_example with your module name:
  tt_um_lkhanh_cordic user_project (
      .ui_in  (ui_in),    // Dedicated inputs
      .uo_out (uo_out),   // Dedicated outputs
      .uio_in (uio_in),   // IOs: Input path
      .uio_out(uio_out),  // IOs: Output path
      .uio_oe (uio_oe),   // IOs: Enable path (active high: 0=input, 1=output)
      .ena    (ena),      // enable - goes high when design is selected
      .clk    (clk),      // clock
      .rst_n  (rst_n)     // not reset
  );

  // SDF back-annotation: slow corner (setup-worst case @ 64MHz).
  // NOTE: icarus $sdf_annotate only honors the first two arguments;
  // the SDF already holds corner-specific single values, so no MTM flag needed.
  // Requires -gspecify at compile time (see Makefile SDF branch).
  initial begin
    $sdf_annotate("/mnt/data/projects/tinytapeout/ttihp26b-tapeout/runs/wokwi/final/sdf/nom_slow_1p08V_125C/tt_um_lkhanh_cordic__nom_slow_1p08V_125C.sdf", user_project);
  end

endmodule

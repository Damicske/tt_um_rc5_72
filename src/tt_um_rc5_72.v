// ============================================================================
// tt_um_rc5_72.v
//
// Tiny Tapeout top-level wrapper for the ASIC-track RC5-72 compute stack
// (rc5_batch_controller_spi, wrapping rc5_batch_controller +
// rc5_iterative_core + spi_slave - all independently verified in
// simulation before this wrapper was written; see that history for what
// each piece has already been checked against).
//
// Port list is the fixed Tiny Tapeout user-module interface - not
// something chosen here, this exact shape is required by the shuttle's
// own shared, multiplexed I/O architecture (confirmed against several
// current, real TT project repos before writing this, not assumed from
// memory).
//
// PIN MAPPING - deliberately minimal. This design's real I/O footprint
// is just the 5-signal SPI slave interface (sclk/mosi/cs_n in,
// miso/ready out), comfortably inside ui_in[2:0]/uo_out[1:0] with the
// rest of the 24-pin budget entirely unused - genuinely not a tight fit,
// unlike some TT projects that need every available pin.
//   ui_in[0]  = sclk
//   ui_in[1]  = mosi
//   ui_in[2]  = cs_n
//   ui_in[7:3] = unused (inputs - safe to leave unconnected externally)
//   uo_out[0] = miso
//   uo_out[1] = ready
//   uo_out[7:2] = unused, forced 0
//   uio[7:0]  = entirely unused - uio_oe all 0 (all-input, the safe
//               default), uio_out all 0
//
// ena HANDLING - two-layered, deliberately redundant:
//   1. Internal reset is `rst_n & ena`, not rst_n alone - the whole
//      design is held in reset whenever NOT selected, so it genuinely
//      cannot do anything (not merely "gated from acting") while some
//      OTHER project sharing this die's multiplexed bus is the one
//      actually connected to the physical pins.
//   2. uo_out/uio_out/uio_oe are ALSO explicitly forced to 0 whenever
//      !ena, regardless of what the internal (already-reset) logic
//      would produce on its own. Confirmed this is not covering for an
//      unsafe internal default - spi_slave.v's own miso register resets
//      cleanly to 0 - this is deliberate defense-in-depth, not a
//      workaround: it makes the output-safety guarantee visible at the
//      pin boundary itself, without requiring a reviewer to trace
//      through the full submodule hierarchy to confirm it.
// Matches the pattern found in real, working TT example designs (ena
// checked live/continuously, not sampled once), rather than assuming a
// specific timing guarantee on how often ena can change that isn't
// explicitly documented.
// ============================================================================

`default_nettype none

module tt_um_rc5_72 (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,   // unused
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire        ena,
    input  wire        clk,
    input  wire        rst_n
);

    wire internal_rst_n = rst_n & ena;

    wire core_miso, core_ready;

    rc5_batch_controller_spi u_core (
        .clk(clk), .rst_n(internal_rst_n),
        .sclk(ui_in[0]), .mosi(ui_in[1]), .cs_n(ui_in[2]),
        .miso(core_miso), .ready(core_ready)
        // debug_busy/debug_match/debug_receiving/debug_transmitting left
        // unconnected - no spare pins allocated to them, not needed for
        // correct operation
    );

    assign uo_out = ena ? {6'b0, core_ready, core_miso} : 8'b0;
    assign uio_out = 8'b0;
    assign uio_oe  = 8'b0; // all-input - this design never drives uio at all

    // silence unused-input lint warnings without affecting anything -
    // uio_in and ui_in[7:3] are genuinely, deliberately unused
    wire _unused = &{uio_in, ui_in[7:3], 1'b0};

endmodule

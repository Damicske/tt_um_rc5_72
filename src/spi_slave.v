// ============================================================================
// spi_slave.v
//
// Drop-in transport replacement for uart_rx.v + uart_tx.v combined -
// presents the EXACT SAME byte-level interface (rx_valid/rx_data,
// tx_start/tx_data/tx_busy) that rc5_work_unit_controller.v already
// drives, so the controller's own state machine - every hard-won fix in
// it - needs ZERO changes to use this instead. Only the transport
// underneath changes; the 31-byte-request/status-coded-response
// protocol riding on top is untouched.
//
// SPI mode 0 (CPOL=0, CPHA=0), MSB-first - standard, unambiguous
// defaults. sclk/mosi/cs_n are genuinely asynchronous to this core's own
// clock (each board on the eventual controller/core pair runs its own
// independent oscillator) - all three get the same 2-stage synchronizer
// pattern uart_rx.v already uses for its own async rxd input, not a new
// idiom invented for this file.
//
// PROTOCOL SHAPE: our existing request/response protocol is inherently
// half-duplex (a full request one way, then, much later, a full
// response the other way) - rather than force SPI's simultaneous
// bidirectional shifting into that shape, request and response are two
// SEPARATE SPI transactions, each its own CS assertion:
//   1. REQUEST transaction: master asserts cs_n, clocks out 31 bytes on
//      mosi (miso is don't-care here), deasserts cs_n.
//   2. RESPONSE transaction: master waits for `ready` to go high (this
//      core has NO way to make the master wait mid-transaction the way
//      a UART's async transmit doesn't need to - `ready` is the
//      substitute), then asserts cs_n again and clocks to read back
//      whatever this core shifts out on miso (mosi is don't-care here).
//
// `ready` requires ONE extra wire beyond the standard 4 (sclk/mosi/
// miso/cs_n) between the two boards - deliberate: SPI itself has no
// built-in "not ready yet" signaling (unlike e.g. I2C clock stretching),
// and a dedicated line is simpler and more robust than layering a
// software polling protocol on top of SPI to fake one.
// ============================================================================

module spi_slave (
    input  wire       clk,
    input  wire       rst_n,

    // raw, ASYNCHRONOUS SPI signals from/to the master - never used
    // directly, always through the synchronizers below first
    input  wire       sclk,
    input  wire       mosi,
    input  wire       cs_n,
    output reg        miso,
    output wire       ready,   // extra GPIO - see file header

    // byte-level interface - EXACTLY matches uart_rx.v's valid/data and
    // uart_tx.v's start/data/busy, so rc5_work_unit_controller.v's own
    // instantiation of this module can swap in for uart_rx+uart_tx with
    // no other change anywhere in the controller's own logic
    output reg        rx_valid,   // one-cycle pulse when a byte is ready
    output reg  [7:0] rx_data,
    input  wire       tx_start,
    input  wire [7:0] tx_data,
    output reg        tx_busy
);

    // ---- synchronizers: 2-stage, same pattern as uart_rx.v's own
    // rxd_sync0/rxd_sync1 for its async rxd input - not a new idiom ----
    reg sclk_s0, sclk_s1, sclk_s2; // s2 = one cycle further back, for
                                    // edge detection (rising: s1 && !s2;
                                    // falling: !s1 && s2)
    reg mosi_s0, mosi_s1;
    reg cs_n_s0, cs_n_s1; // 2-stage synchronizer - s1 gates whether this
                          // core is currently selected
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sclk_s0 <= 1'b0; sclk_s1 <= 1'b0; sclk_s2 <= 1'b0;
            mosi_s0 <= 1'b0; mosi_s1 <= 1'b0;
            cs_n_s0 <= 1'b1; cs_n_s1 <= 1'b1; // idle high
        end else begin
            sclk_s0 <= sclk;   sclk_s1 <= sclk_s0;   sclk_s2 <= sclk_s1;
            mosi_s0 <= mosi;   mosi_s1 <= mosi_s0;
            cs_n_s0 <= cs_n;   cs_n_s1 <= cs_n_s0;
        end
    end

    wire sclk_rising  = sclk_s1 && !sclk_s2;
    wire sclk_falling = !sclk_s1 && sclk_s2;

    // ---- RX side: shift mosi in on each rising edge, MSB-first ----
    reg [7:0] rx_shreg;
    reg [2:0] rx_bit_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_valid   <= 1'b0;
            rx_bit_cnt <= 3'd0;
        end else begin
            rx_valid <= 1'b0; // default: single-cycle pulse, same as uart_rx.v

            if (cs_n_s1) begin
                // not selected - bit counter re-arms for the next
                // transaction, whatever it turns out to be (request or
                // response read-back both start fresh at bit 0)
                rx_bit_cnt <= 3'd0;
            end else if (sclk_rising) begin
                rx_shreg   <= {rx_shreg[6:0], mosi_s1};
                rx_bit_cnt <= rx_bit_cnt + 1'b1;
                if (rx_bit_cnt == 3'd7) begin
                    // 8th bit just arrived on mosi_s1 this same edge -
                    // rx_shreg[6:0] holds the PREVIOUS 7 bits, so the
                    // complete byte is {rx_shreg[6:0], mosi_s1}, not
                    // the not-yet-updated rx_shreg alone
                    rx_data  <= {rx_shreg[6:0], mosi_s1};
                    rx_valid <= 1'b1;
                end
            end
        end
    end

    // ---- TX side ----
    reg [7:0] tx_shreg;
    reg [2:0] tx_bit_cnt;
    reg       tx_active; // true from tx_start until this byte is fully
                          // shifted out - distinct from tx_busy only in
                          // naming clarity, same lifetime

    // FIX: ready was originally a separately-latched signal, asserted on
    // tx_start and dropped on the NEXT cs_n falling edge. That breaks
    // for a master that holds cs_n continuously low through an entire
    // multi-byte response (a perfectly valid, arguably more natural way
    // to implement the master side) - cs_n never re-falls between
    // bytes, so ready would never correctly reflect byte 2 onward.
    // tx_busy's own lifetime - from tx_start until THIS byte is fully
    // shifted out - is already exactly the window the master needs to
    // know about, for every byte, not just the first one of a
    // transaction - so ready is just tx_busy directly, combinational,
    // not a separate latched signal at all.
    assign ready = tx_busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_busy   <= 1'b0;
            tx_active <= 1'b0;
            miso      <= 1'b0;
            tx_bit_cnt <= 3'd0;
        end else begin
            if (tx_start && !tx_busy) begin
                // controller wants to send a byte - load it and go
                // busy/ready. Mirrors uart_tx.v's own "start pulse ->
                // busy goes high immediately" contract exactly, just
                // with an unbounded (not fixed-10-period) busy duration
                // - the controller's own TXS_WAIT state already just
                // waits unconditionally for tx_busy to clear, with no
                // timing assumption baked in, so this is compatible
                // with no changes needed on that side.
                tx_shreg   <= tx_data;
                tx_bit_cnt <= 3'd0;
                tx_busy    <= 1'b1;
                tx_active  <= 1'b1;
                miso       <= tx_data[7]; // MSB pre-loaded, valid the
                    // instant cs_n falls for this transaction, before
                    // any sclk edge has occurred yet
            end

            if (tx_active && !cs_n_s1 && sclk_falling) begin
                // CPHA=0: shift/change data on the FALLING edge, so
                // it's stable and settled before the master's NEXT
                // rising-edge sample - shifting on the rising edge
                // itself (the same edge the master samples on) would
                // race the master's own read of this same bit
                if (tx_bit_cnt == 3'd7) begin
                    tx_busy   <= 1'b0;
                    tx_active <= 1'b0;
                end else begin
                    tx_shreg   <= {tx_shreg[6:0], 1'b0};
                    miso       <= tx_shreg[6];
                    tx_bit_cnt <= tx_bit_cnt + 1'b1;
                end
            end
        end
    end

endmodule

// ============================================================================
// rc5_batch_controller_spi.v
//
// SPI-facing wrapper around rc5_batch_controller (the ASIC-targeted,
// iterative RC5-72 engine) - same outer protocol as
// rc5_work_unit_controller_spi.v throughout (31-byte request, status-coded
// response, same field layout, same SYNC bytes), reusing that file's own
// proven byte-reception and response-transmission logic as directly as
// possible, including its hard-won fixes (the report_start edge-pulse fix,
// the report_len latching fix - see that file's own history for what each
// one addressed and why).
//
// GENUINE SIMPLIFICATION vs rc5_work_unit_controller_spi.v: no generation-
// tag/two-slot pt-target scheme here at all. That scheme exists ONLY
// because the deeply pipelined FPGA core could have keys from a PREVIOUS
// work unit still draining while a NEW request's bytes were already
// arriving. rc5_batch_controller has no such pipelining - it's fully
// sequential and genuinely idle between work units - so a new request can
// only ever be accepted once the previous one's response has completely
// finished sending, at which point rc5_batch_controller is provably back
// in its own idle state too. One set of base_key/count/pt_a/pt_b/
// target_ct_a/target_ct_b registers is sufficient; there is no "two work
// units in flight at once" case to protect against here.
//
// SAME SIMPLIFICATION APPLIES to the response side: rc5_batch_controller's
// own status/result_key/result_ct_a/result_ct_b/result_cmc_count are only
// ever written once, during ITS OWN internal report state, and won't be
// touched again until the next start pulse - which this wrapper only
// issues once it's back in ST_IDLE, long after the previous response
// finished transmitting. So unlike the original design (whose
// unconditional pipe_out_valid check kept accumulating results even
// during transmission, forcing everything to be explicitly latched at
// report_start to avoid a response changing mid-stream), rc5_batch_
// controller's outputs are already stable for the whole transmission
// window by construction - referenced directly below, no separate latch
// needed.
// ============================================================================

module rc5_batch_controller_spi (
    input  wire clk,
    input  wire rst_n,

    input  wire        sclk,
    input  wire        mosi,
    input  wire        cs_n,
    output wire        miso,
    output wire        ready,

    output wire debug_busy,
    output wire debug_match,
    output wire debug_receiving,
    output wire debug_transmitting
);

    localparam SYNC_RX = 8'hAA;
    localparam SYNC_TX = 8'h55;
    localparam PKT_LEN = 31;

    // ---- SPI slave instance - identical instantiation pattern to
    // rc5_work_unit_controller_spi.v ----
    wire       rx_valid;
    wire [7:0] rx_data;
    reg        tx_start;
    reg  [7:0] tx_data;
    wire       tx_busy;

    spi_slave u_spi (
        .clk(clk), .rst_n(rst_n),
        .sclk(sclk), .mosi(mosi), .cs_n(cs_n), .miso(miso), .ready(ready),
        .rx_valid(rx_valid), .rx_data(rx_data),
        .tx_start(tx_start), .tx_data(tx_data), .tx_busy(tx_busy)
    );

    // ---- the compute engine this wraps - already verified, standalone,
    // against this project's own trusted vectors before this file was
    // written ----
    reg         core_start;
    reg  [71:0] core_base_key;
    reg  [31:0] core_count;
    reg  [31:0] core_pt_a, core_pt_b, core_target_a, core_target_b;
    wire        core_busy, core_done;
    wire [7:0]  core_status;
    wire [71:0] core_result_key;
    wire [31:0] core_result_ct_a, core_result_ct_b, core_result_cmc_count;

    rc5_batch_controller u_batch (
        .clk(clk), .rst_n(rst_n),
        .start(core_start), .base_key(core_base_key), .count(core_count),
        .pt_a(core_pt_a), .pt_b(core_pt_b),
        .target_ct_a(core_target_a), .target_ct_b(core_target_b),
        .busy(core_busy), .done(core_done), .status(core_status),
        .result_key(core_result_key),
        .result_ct_a(core_result_ct_a), .result_ct_b(core_result_ct_b),
        .result_cmc_count(core_result_cmc_count)
    );

    // ---- work-unit registers, loaded from an incoming packet - ONE set,
    // no generation tags, see file header ----
    reg [71:0] base_key;
    reg [31:0] key_count;
    reg [31:0] pt_a, pt_b, target_a, target_b;

    // ---- byte-accumulation (only active while IDLE) - same field byte
    // layout as rc5_work_unit_controller_spi.v ----
    reg [5:0] byte_idx;
    reg       receiving;
    reg       packet_ready;

    localparam ST_IDLE   = 2'd0,
               ST_WAIT   = 2'd1,
               ST_REPORT = 2'd2;
    reg [1:0] state;
    reg       report_done;
    reg       report_start;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            receiving    <= 1'b0;
            byte_idx     <= 0;
            packet_ready <= 1'b0;
        end else begin
            packet_ready <= 1'b0; // default: single-cycle pulse
            if (state == ST_IDLE && rx_valid) begin
                if (!receiving) begin
                    if (rx_data == SYNC_RX) begin
                        receiving <= 1'b1;
                        byte_idx  <= 1;
                    end
                end else begin
                    case (byte_idx)
                        2:  base_key[0*8  +: 8] <= rx_data;
                        3:  base_key[1*8  +: 8] <= rx_data;
                        4:  base_key[2*8  +: 8] <= rx_data;
                        5:  base_key[3*8  +: 8] <= rx_data;
                        6:  base_key[4*8  +: 8] <= rx_data;
                        7:  base_key[5*8  +: 8] <= rx_data;
                        8:  base_key[6*8  +: 8] <= rx_data;
                        9:  base_key[7*8  +: 8] <= rx_data;
                        10: base_key[8*8  +: 8] <= rx_data;
                        11: key_count[0  +: 8] <= rx_data;
                        12: key_count[8  +: 8] <= rx_data;
                        13: key_count[16 +: 8] <= rx_data;
                        14: key_count[24 +: 8] <= rx_data;
                        15: pt_a[0  +: 8] <= rx_data;
                        16: pt_a[8  +: 8] <= rx_data;
                        17: pt_a[16 +: 8] <= rx_data;
                        18: pt_a[24 +: 8] <= rx_data;
                        19: pt_b[0  +: 8] <= rx_data;
                        20: pt_b[8  +: 8] <= rx_data;
                        21: pt_b[16 +: 8] <= rx_data;
                        22: pt_b[24 +: 8] <= rx_data;
                        23: target_a[0  +: 8] <= rx_data;
                        24: target_a[8  +: 8] <= rx_data;
                        25: target_a[16 +: 8] <= rx_data;
                        26: target_a[24 +: 8] <= rx_data;
                        27: target_b[0  +: 8] <= rx_data;
                        28: target_b[8  +: 8] <= rx_data;
                        29: target_b[16 +: 8] <= rx_data;
                        30: target_b[24 +: 8] <= rx_data;
                        default: ; // byte 1 (cmd) - not stored
                    endcase

                    if (byte_idx == PKT_LEN-1) begin
                        receiving    <= 1'b0;
                        byte_idx     <= 0;
                        packet_ready <= 1'b1;
                    end else begin
                        byte_idx <= byte_idx + 1'b1;
                    end
                end
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            core_start   <= 1'b0;
            report_start <= 1'b0;
        end else begin
            core_start   <= 1'b0; // default: single-cycle pulse
            report_start <= 1'b0; // default: single-cycle pulse

            case (state)
                ST_IDLE: begin
                    if (packet_ready) begin
                        core_base_key <= base_key;
                        core_count    <= key_count;
                        core_pt_a     <= pt_a;
                        core_pt_b     <= pt_b;
                        core_target_a <= target_a;
                        core_target_b <= target_b;
                        core_start    <= 1'b1;
                        state         <= ST_WAIT;
                    end
                end

                ST_WAIT: begin
                    if (core_done) begin
                        state        <= ST_REPORT;
                        report_start <= 1'b1;
                    end
                end

                ST_REPORT: begin
                    // handled by the byte-sending block below; report_done
                    // signals completion instead of that block touching
                    // `state` directly
                    if (report_done)
                        state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

    // ---- result packet transmission - same proven pattern as
    // rc5_work_unit_controller_spi.v, referencing rc5_batch_controller's
    // own outputs directly (stable throughout, see file header) rather
    // than a separate latch ----
    reg [5:0] tx_idx;

    function [7:0] result_byte;
        input [5:0] idx;
        begin
            case (idx)
                0: result_byte = SYNC_TX;
                1: result_byte = core_status;
                2:  result_byte = core_result_key[0*8  +: 8];
                3:  result_byte = core_result_key[1*8  +: 8];
                4:  result_byte = core_result_key[2*8  +: 8];
                5:  result_byte = core_result_key[3*8  +: 8];
                6:  result_byte = core_result_key[4*8  +: 8];
                7:  result_byte = core_result_key[5*8  +: 8];
                8:  result_byte = core_result_key[6*8  +: 8];
                9:  result_byte = core_result_key[7*8  +: 8];
                10: result_byte = core_result_key[8*8  +: 8];
                // bytes 11-14: CMC count (if status==2) or result_ct_a (if status==1)
                11: result_byte = (core_status == 8'h02) ? core_result_cmc_count[0  +: 8] : core_result_ct_a[0  +: 8];
                12: result_byte = (core_status == 8'h02) ? core_result_cmc_count[8  +: 8] : core_result_ct_a[8  +: 8];
                13: result_byte = (core_status == 8'h02) ? core_result_cmc_count[16 +: 8] : core_result_ct_a[16 +: 8];
                14: result_byte = (core_status == 8'h02) ? core_result_cmc_count[24 +: 8] : core_result_ct_a[24 +: 8];
                // bytes 15-18: only meaningful for status==1 - status==2's response ends at byte 14
                15: result_byte = core_result_ct_b[0  +: 8];
                16: result_byte = core_result_ct_b[8  +: 8];
                17: result_byte = core_result_ct_b[16 +: 8];
                18: result_byte = core_result_ct_b[24 +: 8];
                default: result_byte = 8'h00;
            endcase
        end
    endfunction

    localparam TXS_IDLE = 2'd0, TXS_SEND = 2'd1, TXS_WAIT = 2'd2, TXS_NEXT = 2'd3;
    reg [1:0] txs_state;
    localparam integer RESULT_LEN_MATCH   = 19;
    localparam integer RESULT_LEN_NOMATCH = 2;
    localparam integer RESULT_LEN_CMC     = 15;

    reg [5:0] report_len; // latched once, at the true start of ST_REPORT -
        // report_start is a dedicated, edge-true-only-once pulse, same
        // fix as rc5_work_unit_controller_spi.v's own (see that file's
        // history for the exact duplicate-transmission bug this pattern
        // fixes)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            txs_state   <= TXS_IDLE;
            tx_start    <= 1'b0;
            report_done <= 1'b0;
        end else begin
            tx_start    <= 1'b0; // default: single-cycle pulse
            report_done <= 1'b0; // default: single-cycle pulse

            case (txs_state)
                TXS_IDLE: begin
                    if (report_start) begin
                        tx_idx <= 0;
                        case (core_status)
                            8'h01:   report_len <= RESULT_LEN_MATCH[5:0];
                            8'h02:   report_len <= RESULT_LEN_CMC[5:0];
                            default: report_len <= RESULT_LEN_NOMATCH[5:0];
                        endcase
                        txs_state <= TXS_SEND;
                    end
                end
                TXS_SEND: begin
                    tx_data   <= result_byte(tx_idx);
                    tx_start  <= 1'b1;
                    txs_state <= TXS_WAIT;
                end
                TXS_WAIT: begin
                    if (tx_busy)
                        txs_state <= TXS_NEXT;
                end
                TXS_NEXT: begin
                    if (!tx_busy) begin
                        if (tx_idx == report_len - 1) begin
                            txs_state   <= TXS_IDLE;
                            report_done <= 1'b1;
                        end else begin
                            tx_idx    <= tx_idx + 1'b1;
                            txs_state <= TXS_SEND;
                        end
                    end
                end
                default: txs_state <= TXS_IDLE;
            endcase
        end
    end

    assign debug_busy         = (state != ST_IDLE);
    assign debug_match        = (core_status == 8'h01);
    assign debug_receiving    = receiving;
    assign debug_transmitting = (txs_state != TXS_IDLE);

endmodule

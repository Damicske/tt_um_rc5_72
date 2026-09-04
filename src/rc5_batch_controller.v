// ============================================================================
// rc5_batch_controller.v
//
// Wraps rc5_iterative_core with batch dispatch: given a work-unit's worth
// of parameters (base_key, count, pt_a/pt_b, target_ct_a/target_ct_b - same
// field semantics as this project's own established work-unit protocol
// throughout, from rc5_work_unit_controller.v onward), tests `count` keys
// starting at base_key, incrementing by 1 each time, driving the compute
// core one key at a time.
//
// Deliberately given a SIMPLE, direct, parallel-input interface here, not
// SPI - same "small piece, verified standalone, then integrate" discipline
// as rc5_iterative_core.v and spi_slave.v before it. An SPI-facing shell
// wiring this to the real protocol bytes is a separate, later step.
//
// DESIGN DECISION - stops EARLY the moment a full match is found, rather
// than always testing the whole batch like the FPGA pipeline does. That
// always-test-the-whole-batch behavior there is a consequence of being
// deeply pipelined (no clean way to cancel keys already committed
// mid-flight), not a deliberate protocol requirement - and the established
// protocol's own 0x01 (full-match) response never includes cmc_count at
// all, so stopping early can't omit or corrupt anything the response
// format actually reports. Every cycle matters more here (this core tests
// ~93 cycles/key, not 1) than it did on the FPGA, so this is a deliberate,
// real behavioral difference, not an oversight.
//
// status/result semantics match the established protocol exactly:
//   status=8'h00: no match, no CMC this batch
//   status=8'h01: full match - result_key/result_ct_a/result_ct_b valid
//   status=8'h02: CMC (partial match) only - result_key = most RECENT CMC
//                 candidate's key (matching this project's own "highest
//                 index/most recent wins" convention), cmc_count = total
//                 CMCs found this batch
// Full match always takes priority over CMC when both occur in the same
// batch - same priority as the established protocol.
// ============================================================================

module rc5_batch_controller (
    input  wire        clk,
    input  wire        rst_n,

    input  wire         start,
    input  wire [71:0]  base_key,
    input  wire [31:0]  count,
    input  wire [31:0]  pt_a,
    input  wire [31:0]  pt_b,
    input  wire [31:0]  target_ct_a,
    input  wire [31:0]  target_ct_b,

    output reg          busy,
    output reg          done,          // pulses for 1 cycle when the batch completes
    output reg  [7:0]   status,
    output reg  [71:0]  result_key,
    output reg  [31:0]  result_ct_a,
    output reg  [31:0]  result_ct_b,
    output reg  [31:0]  result_cmc_count
);

    // ---- the compute engine this wraps - verified standalone against
    // this project's own trusted vectors before this file was written ----
    reg         core_start;
    reg  [71:0] core_key;
    wire        core_busy, core_done;
    wire [31:0] core_ct_a, core_ct_b;

    rc5_iterative_core u_core (
        .clk(clk), .rst_n(rst_n),
        .start(core_start), .key(core_key), .pt_a(pt_a), .pt_b(pt_b),
        .busy(core_busy), .done(core_done),
        .ct_a(core_ct_a), .ct_b(core_ct_b)
    );

    wire core_full_match = (core_ct_a == target_ct_a) && (core_ct_b == target_ct_b);

    localparam ST_IDLE     = 3'd0,
               ST_DISPATCH = 3'd1,
               ST_WAIT     = 3'd2,
               ST_REPORT   = 3'd3;

    reg [2:0]  state;
    reg [71:0] key_counter;
    reg [31:0] keys_tested;
    reg        found_match;
    reg [71:0] match_key;
    reg [31:0] match_ct_a, match_ct_b;
    reg [31:0] cmc_count;
    reg [71:0] cmc_key;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            busy       <= 1'b0;
            done       <= 1'b0;
            core_start <= 1'b0;
        end else begin
            done       <= 1'b0; // single-cycle pulse, default low
            core_start <= 1'b0; // single-cycle pulse, default low

            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        key_counter <= base_key;
                        keys_tested <= 32'd0;
                        found_match <= 1'b0;
                        cmc_count   <= 32'd0;
                        busy        <= 1'b1;
                        // defensive: an empty batch (count==0) reports
                        // immediately rather than hanging - not something
                        // the established protocol is expected to send,
                        // but cheap to handle correctly
                        state <= (count == 32'd0) ? ST_REPORT : ST_DISPATCH;
                    end
                end

                ST_DISPATCH: begin
                    core_key   <= key_counter;
                    core_start <= 1'b1;
                    state      <= ST_WAIT;
                end

                ST_WAIT: begin
                    if (core_done) begin
                        if (core_full_match) begin
                            // full match - takes priority, see file header
                            found_match <= 1'b1;
                            match_key   <= key_counter;
                            match_ct_a  <= core_ct_a;
                            match_ct_b  <= core_ct_b;
                        end else if (core_ct_a == target_ct_a) begin
                            // CMC only - lo half matches, hi doesn't;
                            // most-recent candidate wins, per this
                            // project's own established convention
                            cmc_count <= cmc_count + 1'b1;
                            cmc_key   <= key_counter;
                        end

                        keys_tested <= keys_tested + 1'b1;

                        if (core_full_match) begin
                            // early exit - see file header design note
                            state <= ST_REPORT;
                        end else if (keys_tested + 1'b1 == count) begin
                            state <= ST_REPORT;
                        end else begin
                            key_counter <= key_counter + 1'b1;
                            state       <= ST_DISPATCH;
                        end
                    end
                end

                ST_REPORT: begin
                    if (found_match) begin
                        status           <= 8'h01;
                        result_key       <= match_key;
                        result_ct_a      <= match_ct_a;
                        result_ct_b      <= match_ct_b;
                        // result_cmc_count intentionally left unset here -
                        // the established protocol's own 0x01 response
                        // never reads it, same as the original FPGA
                        // controller's own response formatting
                    end else if (cmc_count != 32'd0) begin
                        status           <= 8'h02;
                        result_key       <= cmc_key;
                        result_cmc_count <= cmc_count;
                        // result_ct_a/result_ct_b intentionally left
                        // unset here - the established protocol's own
                        // 0x02 response never reads them
                    end else begin
                        status <= 8'h00;
                        // nothing else read by the established protocol
                        // on a 0x00 response
                    end
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule

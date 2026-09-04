// ============================================================================
// rc5_iterative_core.v
//
// Area-optimized RC5-72 compute core for ASIC targets (Tiny Tapeout/sky130).
// Tests ONE key per ~93 cycles (78 keysched mixing + 1 init + 12 rounds +
// 2 overhead), versus 1 cycle/key on the FPGA pipeline this project spent
// all night on - a deliberate trade of throughput for area, since a small
// shuttle die has no room for 12 rounds' + 78 mixing steps' worth of
// registers existing simultaneously. Reuses ONE shared 2-rotator datapath
// across both the keysched-mixing phase and the encryption-round phase
// (muxed by phase, not duplicated per-phase), since only one phase is ever
// active at a time.
//
// Simple, standalone start/done/key-in/pt-in/ct-out interface, deliberately
// scoped to ONLY the compute engine - batch iteration (testing multiple
// keys, key increment, comparison against a target, CMC/match reporting)
// belongs in a separate wrapper controller, not built yet. Same "small
// piece, verified standalone, then integrate" discipline as spi_slave.v
// earlier in this project.
//
// Algorithm (standard RC5 spec, w=32, r=12, b=9-byte/72-bit key,
// c=ceil(9/4)=3 key words, t=2*(r+1)=26 S[] words) - verified against a
// bit-exact Python model of this EXACT cycle-by-cycle structure (shared
// rotator muxing, i_idx/j_idx wraparound, S[]/L[] indexing), which was
// then checked against all three of this project's own trusted
// selftest() vectors (no-match, CMC-only, full-match) and produced
// byte-for-byte correct results on all three before this file was
// written - not verified after the fact.
//
// S[] does NOT compute its own P/Q-derived initial values at runtime -
// those are fixed constants (don't depend on the key at all), so each of
// the 26 S[] registers just resets directly to its own correct constant.
// Costs nothing extra in area (ordinary reset-value flip-flops) and
// removes an entire init-computation phase that would otherwise cost 26
// more cycles per key.
//
// KNOWN OPEN ITEM: this file has NEVER been through a real simulator -
// same honest caveat as this whole project's other from-scratch modules
// before their own first simulation run. The Python verification above is
// real and meaningful (same discipline that caught two genuine bugs in
// spi_master.v/spi_relay_controller.v earlier in this project, just
// applied via a reference model instead of RTL simulation, since this
// environment has no Verilog simulator - see that history for why this
// distinction matters), but it is not a substitute for an actual
// testbench once one is buildable.
// ============================================================================

module rc5_iterative_core (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        start,          // pulse: begin testing `key` against `pt_a`/`pt_b`
    input  wire [71:0] key,            // K[0]=LSB, matching this project's convention throughout
    input  wire [31:0] pt_a,
    input  wire [31:0] pt_b,

    output reg          busy,
    output reg          done,          // pulses for 1 cycle when ct_a/ct_b are valid
    output reg  [31:0]  ct_a,
    output reg  [31:0]  ct_b
);

    // ---- S[] initial values: P32=0xB7E15163, Q32=0x9E3779B9,
    // S_init[0]=P32, S_init[i]=S_init[i-1]+Q32 - precomputed, not
    // synthesized as runtime adds. Values derived and verified in Python
    // (see above) before being transcribed here. ----
    localparam [31:0] S_INIT_00 = 32'hb7e15163, S_INIT_01 = 32'h5618cb1c,
                       S_INIT_02 = 32'hf45044d5, S_INIT_03 = 32'h9287be8e,
                       S_INIT_04 = 32'h30bf3847, S_INIT_05 = 32'hcef6b200,
                       S_INIT_06 = 32'h6d2e2bb9, S_INIT_07 = 32'h0b65a572,
                       S_INIT_08 = 32'ha99d1f2b, S_INIT_09 = 32'h47d498e4,
                       S_INIT_10 = 32'he60c129d, S_INIT_11 = 32'h84438c56,
                       S_INIT_12 = 32'h227b060f, S_INIT_13 = 32'hc0b27fc8,
                       S_INIT_14 = 32'h5ee9f981, S_INIT_15 = 32'hfd21733a,
                       S_INIT_16 = 32'h9b58ecf3, S_INIT_17 = 32'h399066ac,
                       S_INIT_18 = 32'hd7c7e065, S_INIT_19 = 32'h75ff5a1e,
                       S_INIT_20 = 32'h1436d3d7, S_INIT_21 = 32'hb26e4d90,
                       S_INIT_22 = 32'h50a5c749, S_INIT_23 = 32'heedd4102,
                       S_INIT_24 = 32'h8d14babb, S_INIT_25 = 32'h2b4c3474;

    // ---- rotate-left, shared by both instantiations below - amt=0 is
    // safe without special-casing: `data >> 32` on a 32-bit value is 0
    // per Verilog's own shift semantics, so rotl32(data,0) = data|0 =
    // data, correct rotate-by-0 behavior, matching the Python model's
    // own explicit (but equivalent) n=0 guard ----
    function [31:0] rotl32;
        input [31:0] data;
        input [4:0]  amt;
        begin
            rotl32 = (data << amt) | (data >> (6'd32 - amt));
        end
    endfunction

    localparam ST_IDLE      = 3'd0,
               ST_MIX       = 3'd1,
               ST_ENC_INIT  = 3'd2,
               ST_ENC_ROUND = 3'd3,
               ST_DONE      = 3'd4;

    reg [2:0] state;

    reg [31:0] S [0:25];
    reg [31:0] L [0:2];
    reg [31:0] A, B;
    reg [4:0]  i_idx;      // 0-25
    reg [1:0]  j_idx;      // 0-2
    reg [6:0]  mix_cnt;    // 0-77
    reg [3:0]  round_cnt;  // 0-11

    wire mix_phase = (state == ST_MIX);

    // ---- shared 2-rotator datapath - see file header. MIX: add-then-
    // rotate, no post-rotate add (matches S[i]=ROTL(S[i]+A+B,3) directly).
    // ENC: xor-then-rotate-then-add (matches
    // A=ROTL(A^B,B)+S[2i] - the add happens AFTER the rotate here, unlike
    // MIX, hence the extra muxed adder stage on a_new/b_new below). ----
    wire [31:0] rot1_data   = mix_phase ? (S[i_idx] + A + B) : (A ^ B);
    wire [4:0]  rot1_amount = mix_phase ? 5'd3 : B[4:0];
    wire [31:0] rot1_result = rotl32(rot1_data, rot1_amount);
    wire [31:0] a_new = mix_phase ? rot1_result
                                   : (rot1_result + S[{round_cnt, 1'b0} + 5'd2]);

    wire [31:0] rot2_data   = mix_phase ? (L[j_idx] + a_new + B) : (B ^ a_new);
    // FIX: (a_new + B)[4:0] directly is not legal Verilog-2001 - a
    // bit-select can only apply to a net/reg/identifier, not an
    // arbitrary expression (that's a SystemVerilog extension Icarus's
    // default parser doesn't accept). Named intermediate wire instead.
    wire [31:0] mix_sum_ab  = a_new + B;
    wire [4:0]  rot2_amount = mix_phase ? mix_sum_ab[4:0] : a_new[4:0];
    wire [31:0] rot2_result = rotl32(rot2_data, rot2_amount);
    wire [31:0] b_new = mix_phase ? rot2_result
                                   : (rot2_result + S[{round_cnt, 1'b0} + 5'd3]);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            busy  <= 1'b0;
            done  <= 1'b0;
            S[0]  <= S_INIT_00;  S[1]  <= S_INIT_01;  S[2]  <= S_INIT_02;
            S[3]  <= S_INIT_03;  S[4]  <= S_INIT_04;  S[5]  <= S_INIT_05;
            S[6]  <= S_INIT_06;  S[7]  <= S_INIT_07;  S[8]  <= S_INIT_08;
            S[9]  <= S_INIT_09;  S[10] <= S_INIT_10;  S[11] <= S_INIT_11;
            S[12] <= S_INIT_12;  S[13] <= S_INIT_13;  S[14] <= S_INIT_14;
            S[15] <= S_INIT_15;  S[16] <= S_INIT_16;  S[17] <= S_INIT_17;
            S[18] <= S_INIT_18;  S[19] <= S_INIT_19;  S[20] <= S_INIT_20;
            S[21] <= S_INIT_21;  S[22] <= S_INIT_22;  S[23] <= S_INIT_23;
            S[24] <= S_INIT_24;  S[25] <= S_INIT_25;
        end else begin
            done <= 1'b0; // single-cycle pulse, default low

            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        // load L[] from the key, reset A/B/indices, S[]
                        // is ALREADY at its correct init constants
                        // (reset above / restored below after each use -
                        // see ST_DONE) - one cycle, then straight into
                        // MIX
                        L[0] <= key[31:0];
                        L[1] <= key[63:32];
                        L[2] <= {24'b0, key[71:64]};
                        A <= 32'd0;
                        B <= 32'd0;
                        i_idx <= 5'd0;
                        j_idx <= 2'd0;
                        mix_cnt <= 7'd0;
                        busy  <= 1'b1;
                        state <= ST_MIX;
                    end
                end

                ST_MIX: begin
                    S[i_idx] <= a_new;
                    L[j_idx] <= b_new;
                    A <= a_new;
                    B <= b_new;
                    i_idx <= (i_idx == 5'd25) ? 5'd0 : i_idx + 1'b1;
                    j_idx <= (j_idx == 2'd2)  ? 2'd0 : j_idx + 1'b1;
                    if (mix_cnt == 7'd77) begin
                        state <= ST_ENC_INIT;
                    end else begin
                        mix_cnt <= mix_cnt + 1'b1;
                    end
                end

                ST_ENC_INIT: begin
                    A <= pt_a + S[0];
                    B <= pt_b + S[1];
                    round_cnt <= 4'd0;
                    state <= ST_ENC_ROUND;
                end

                ST_ENC_ROUND: begin
                    A <= a_new;
                    B <= b_new;
                    if (round_cnt == 4'd11) begin
                        state <= ST_DONE;
                    end else begin
                        round_cnt <= round_cnt + 1'b1;
                    end
                end

                ST_DONE: begin
                    ct_a  <= A;
                    ct_b  <= B;
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    // restore S[] to its fixed init constants, ready for
                    // the NEXT key's mixing phase - mixing overwrote all
                    // 26 entries above, and S[] must start from the same
                    // P/Q baseline every time, not the previous key's
                    // already-mixed schedule
                    S[0]  <= S_INIT_00;  S[1]  <= S_INIT_01;  S[2]  <= S_INIT_02;
                    S[3]  <= S_INIT_03;  S[4]  <= S_INIT_04;  S[5]  <= S_INIT_05;
                    S[6]  <= S_INIT_06;  S[7]  <= S_INIT_07;  S[8]  <= S_INIT_08;
                    S[9]  <= S_INIT_09;  S[10] <= S_INIT_10;  S[11] <= S_INIT_11;
                    S[12] <= S_INIT_12;  S[13] <= S_INIT_13;  S[14] <= S_INIT_14;
                    S[15] <= S_INIT_15;  S[16] <= S_INIT_16;  S[17] <= S_INIT_17;
                    S[18] <= S_INIT_18;  S[19] <= S_INIT_19;  S[20] <= S_INIT_20;
                    S[21] <= S_INIT_21;  S[22] <= S_INIT_22;  S[23] <= S_INIT_23;
                    S[24] <= S_INIT_24;  S[25] <= S_INIT_25;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule

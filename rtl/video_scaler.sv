//
// video_scaler.sv — TI-89 LCD raster scaler for MiSTer video path
// TI-89 MiSTer Core
//
// lcd_ctrl generates a 160x100 active LCD raster where one LCD pixel
// lasts 48 master clock cycles. pix_ce is a ONE-CYCLE strobe on the first
// clock of every active pixel (it must never stay high between pixels —
// the capture logic counts each pix_ce cycle as a new pixel); blanking
// intervals arrive as gaps between strobes. One LCD row therefore lasts
// 200*48 = 9600 master clocks and one LCD frame 110 rows = 1,056,000
// clocks (~60.6 Hz).
//
// This scaler re-emits every LCD pixel as SCALE x SCALE output pixels,
// phase-locked to the LCD stream and at the SAME ~60.6 Hz frame rate in
// every mode (MiSTer's ascal needs a steady 50-75 Hz vertical input to
// lock; a slower raster makes the HDMI picture roll):
//
//   scale_sel | scale | output raster | pixel clock   | frame rate
//   ----------+-------+---------------+---------------+-----------
//       0     |  4x   |   800 x 440   | ~21.33 MHz    | 60.6 Hz
//       1     |  3x   |   600 x 330   | ~12.07 MHz    | 60.6 Hz
//       2     |  2x   |   400 x 220   |   5.33 MHz    | 60.6 Hz
//       3     |  1x   |   200 x 110   |   1.33 MHz    | 60.6 Hz
//
// One output frame is exactly 110*scale rows of 200*scale pixels, which
// spans exactly one LCD frame (1,056,000 clocks) for every scale, so the
// output refresh rate equals the LCD refresh rate. A full LCD frame is
// emitted while exactly one LCD frame is received: each LCD row period
// (9600 clocks) carries `scale` output rows of 200*scale pixels each
// (9600 clocks total; for 3x the pixel divider alternates 5/6 clocks via
// a small accumulator since 9600/1800 is not integral).
//
// Because the output pixel rate is higher than the incoming LCD pixel
// rate, one row of LCD pixels is line-buffered (double buffered) and
// expanded on readout: while LCD row N is being captured into one bank,
// the output emits row N-1 from the other bank. The output raster is
// phase-locked to the LCD stream once at start; both periods are exact
// integer clock counts, so the lock never drifts.
//
// If scale_sel changes while running (OSD scale option), the output raster
// period changes and the phase lock is invalid. The scaler then halts the
// output (blank) and re-locks at the next LCD frame start, so a rescaled
// picture re-acquires lock within one frame instead of tearing forever.
//
// VGA-style output: positive-sync HS/VS, DE = active drawing area.
// The MiSTer framework (ascal) scales this raster to the selected HDMI
// resolution.
//
// The LCD raster constants below must stay in sync with lcd_ctrl.sv.
//

module video_scaler (
    input             clk,        // Master clock (60 MHz)
    input             reset,

    // From lcd_ctrl
    input             pix_ce,     // LCD pixel clock enable (active pixels)
    input             pixel,      // LCD pixel: 1 = dark, 0 = background

    // OSD options
    input       [1:0] scale_sel,  // 0 = 4x, 1 = 3x, 2 = 2x, 3 = 1x
    input       [1:0] color_sel,  // 0 = green, 1 = blue, 2 = amber, 3 = B&W

    // Boot-chain diagnostic (from TI89.sv). While the OS has not fully
    // booted, the entire active area is painted with a solid color so
    // the load/boot state can be read off the TV:
    //   0 = blue   - no OS image loaded yet
    //   1 = orange - OS image download / fill in progress
    //   2 = red    - download finished but the image was not recognized
    //   3 = violet - image accepted, boot copy / CPU start in progress
    //   4 = normal palette raster (boot_done)
    input       [2:0] boot_status,

    // Real-time live debug hex overlay inputs
    input             dbg_en,
    input      [23:0] dbg_pc,
    input      [23:0] dbg_addr,
    input      [15:0] dbg_data,
    input             dbg_rw,
    input       [2:0] dbg_ipl,
    input      [15:0] dbg_int_cnt,
    input      [15:0] dbg_flw_cnt,
    input             dbg_lcd_on,
    input             dbg_protect,
    input             dbg_stopped,
    input             dbg_ai7,

    // VGA output (directly to the emu module outputs)
    output reg        ce_pix,     // Output pixel strobe (based on clk)
    output reg  [7:0] R,
    output reg  [7:0] G,
    output reg  [7:0] B,
    output reg        HSync,      // Positive sync
    output reg        VSync,      // Positive sync
    output reg        DE          // = ~(HBlank | VBlank)
);

    // =========================================================================
    // LCD raster constants (must match lcd_ctrl.sv)
    // =========================================================================
    localparam [8:0] H_ACTIVE = 9'd160;
    localparam [8:0] H_TOTAL  = 9'd200;
    localparam [7:0] V_ACTIVE = 8'd100;
    localparam [7:0] V_TOTAL  = 8'd110;

    // Row period in master clocks (200 LCD pixels x 48 clocks)
    localparam [13:0] ROW_CLKS = 14'd9600;

    // Sync positions within the LCD raster (in LCD pixel units)
    localparam [8:0] HS_START = 9'd164;
    localparam [8:0] HS_END   = 9'd167;   // exclusive
    localparam [7:0] VS_START = 8'd102;
    localparam [7:0] VS_END   = 8'd104;   // exclusive

    // =========================================================================
    // Scale factor decode
    // =========================================================================
    reg [2:0] scale;        // 1..4
    reg [5:0] div_base;     // master clocks per output pixel (base)
    reg [9:0] div_rem;      // Bresenham remainder per output pixel
    reg [15:0] row_px;      // output pixels per LCD row = 200*scale^2

    always @(*) begin
        case (scale_sel)
            // div_base + div_rem spread over row_px pixels sums to exactly
            // ROW_CLKS per LCD row in every mode.
            2'd0:    begin scale = 3'd4; div_base = 6'd3;  div_rem = 10'd0;   row_px = 16'd3200; end
            2'd1:    begin scale = 3'd3; div_base = 6'd5;  div_rem = 10'd600; row_px = 16'd1800; end
            2'd2:    begin scale = 3'd2; div_base = 6'd12; div_rem = 10'd0;   row_px = 16'd800;  end
            default: begin scale = 3'd1; div_base = 6'd48; div_rem = 10'd0;   row_px = 16'd200;  end
        endcase
    end

    // Scaled raster limits
    wire [9:0] h_total_m1  = (H_TOTAL  * scale) - 10'd1;
    wire [9:0] h_active_m1 = (H_ACTIVE * scale) - 10'd1;
    wire [9:0] v_total_m1  = (V_TOTAL  * scale) - 10'd1;
    wire [9:0] v_active_m1 = (V_ACTIVE * scale) - 10'd1;
    wire [9:0] hs_start    = HS_START * scale;
    wire [9:0] hs_end      = HS_END   * scale;
    wire [9:0] vs_start    = VS_START * scale;
    wire [9:0] vs_end      = VS_END   * scale;
    
    // Scale-change detection: the output raster periods derive from
    // scale_sel, so any change breaks the phase lock to the LCD stream;
    // the lock is then re-acquired at the next LCD frame start (see the
    // capture block below).
    reg  [1:0] scale_d;
    wire       scale_chg = (scale_d != scale_sel);

    always @(posedge clk) begin
        if (reset)
            scale_d <= 2'd0;
        else
            scale_d <= scale_sel;
    end

    // =========================================================================
    // LCD palette (background / foreground)
    // =========================================================================
    reg [23:0] bg_rgb, fg_rgb;

    always @(*) begin
        case (color_sel)
            2'd0:    begin bg_rgb = 24'h9CB868; fg_rgb = 24'h1E2412; end // green
            2'd1:    begin bg_rgb = 24'h84A0C4; fg_rgb = 24'h0E1622; end // blue
            2'd2:    begin bg_rgb = 24'hD2A45A; fg_rgb = 24'h34200A; end // amber
            default: begin bg_rgb = 24'hECECE4; fg_rgb = 24'h141414; end // B&W
        endcase
    end

    // =========================================================================
    // Pixel strobe synchronization
    // =========================================================================
    // pix_ce is produced by lcd_ctrl; its display outputs (pixel, blanking)
    // settle two master clocks after pix_ce. Delay the strobe accordingly so
    // everything is sampled cleanly.

    reg [2:0] ce_d;
    wire      lcd_ce = ce_d[2];

    always @(posedge clk) begin
        if (reset)
            ce_d <= 3'd0;
        else
            ce_d <= {ce_d[1:0], pix_ce};
    end

    // =========================================================================
    // Capture side: line-buffer the incoming LCD row stream
    // =========================================================================
    // pix_ce strobes only for active pixels (160 per row), so blanking
    // shows up as gaps: >= 1920 clocks between rows, >= 97920 across
    // vertical blank. A gap marks the first pixel of the next row (row 0
    // after the long frame gap).
    //
    // Double buffer: LCD row N is written into bank N[0] while the output
    // side reads row N-1 from bank ~N[0].

    reg [15:0] lbuf [0:1][0:9];   // 2 banks x 160 pixels (10 words each)
    reg  [6:0] lcd_row;          // LCD row being captured (0..109)
    reg  [7:0] lcd_px;           // active pixel index within row (0..159)
    reg [16:0] idle_cnt;         // clocks since last strobe (saturating)

    wire        row_gap   = (idle_cnt >= 17'd1000);
    wire        frame_gap = (idle_cnt >  17'd40000);
    wire  [6:0] nxt_row   = row_gap ? (frame_gap ? 7'd0 : lcd_row + 7'd1)
                                    : lcd_row;
    wire  [7:0] nxt_px    = row_gap ? 8'd0 : lcd_px + 8'd1;

    // =========================================================================
    // Output raster state
    // =========================================================================
    reg        started;    // output raster is phase-locked to the LCD stream
    reg        relock;     // scale changed: re-acquire lock at next frame start
    reg [9:0]  oh;         // output horizontal position (0..h_total_m1)
    reg [9:0]  ov;         // output vertical position   (0..v_total_m1)
    reg [5:0]  out_cnt;    // output pixel divider
    reg        out_tick;
    reg  [11:0] div_acc;   // Bresenham accumulator for fractional dividers

    // Lock: the first LCD strobe is row 0, pixel 0. Output group G (scale
    // rows showing LCD row G) is emitted during LCD row period G+1, so at
    // the first strobe the output must sit at the start of group 109
    // (emitted during LCD row 0, all blanking anyway).
    wire [9:0] lock_ov = v_total_m1 - ({7'd0, scale} - 10'd1);

    always @(posedge clk) begin
        if (reset) begin
            started <= 1'b0;
            relock  <= 1'b0;
            lcd_row <= 7'd0;
            lcd_px  <= 8'd0;
            idle_cnt<= 17'd0;
            // NOTE: oh/ov are driven ONLY by the raster block below
            // (Quartus error 10028 otherwise); its reset branch zeroes them.
            lbuf[0][0] <= 16'd0; lbuf[0][1] <= 16'd0; lbuf[0][2] <= 16'd0;
            lbuf[0][3] <= 16'd0; lbuf[0][4] <= 16'd0; lbuf[0][5] <= 16'd0;
            lbuf[0][6] <= 16'd0; lbuf[0][7] <= 16'd0; lbuf[0][8] <= 16'd0;
            lbuf[0][9] <= 16'd0;
            lbuf[1][0] <= 16'd0; lbuf[1][1] <= 16'd0; lbuf[1][2] <= 16'd0;
            lbuf[1][3] <= 16'd0; lbuf[1][4] <= 16'd0; lbuf[1][5] <= 16'd0;
            lbuf[1][6] <= 16'd0; lbuf[1][7] <= 16'd0; lbuf[1][8] <= 16'd0;
            lbuf[1][9] <= 16'd0;
        end else begin
            if (idle_cnt != 17'd131071)
                idle_cnt <= idle_cnt + 17'd1;

            if (lcd_ce) begin
                idle_cnt <= 17'd0;
                lcd_row  <= nxt_row;
                lcd_px   <= nxt_px;
                // Store MSB-first, matching lcd_ctrl's pixel ordering
                lbuf[nxt_row[0]][nxt_px[7:4]][15 - nxt_px[3:0]] <= pixel;
                if (relock) begin
                    // Re-lock at the first strobe of a new LCD frame — the
                    // only phase where the fixed lock formula (group 109 at
                    // row 0, pixel 0) is correct. While the re-lock is
                    // pending the raster block's (lcd_ce && !started) branch
                    // holds oh=0 / ov=lock_ov on every strobe, so the
                    // re-acquisition lands phase-locked with no tearing.
                    if (frame_gap) begin
                        relock  <= 1'b0;
                        started <= 1'b1;
                    end
                end else if (!started)
                    started <= 1'b1;
            end

            // AFTER the lcd_ce block: a scale change in the same cycle as a
            // strobe must still halt the output (last write to `started`
            // wins), arming the re-lock above for the next frame start.
            if (scale_chg) begin
                // The raster periods no longer match the LCD stream phase:
                // halt the output until the lock can be re-acquired.
                started <= 1'b0;
                relock  <= 1'b1;
            end
        end
    end

    // =========================================================================
    // Output pixel timebase
    // =========================================================================
    // Free-running divider producing out_tick every div_base (or
    // div_base+1) master clocks. For the 3x mode the divider alternates
    // 5/6 clocks so each LCD row spans exactly ROW_CLKS. Both the output
    // frame and the LCD frame are exactly 1,056,000 clocks, so the raster
    // locked above never drifts.

    wire [11:0] acc_nxt = div_acc + div_rem;
    wire       acc_carry = (acc_nxt >= row_px);

    always @(posedge clk) begin
        if (reset) begin
            out_cnt  <= 6'd0;
            out_tick <= 1'b0;
            div_acc  <= 12'd0;
        end else if (!started) begin
            out_cnt  <= 6'd0;
            out_tick <= 1'b0;
            div_acc  <= 12'd0;
        end else begin
            out_tick <= 1'b0;
            if (out_cnt >= div_base + {4'd0, acc_carry} - 6'd1) begin
                out_cnt  <= 6'd0;
                out_tick <= 1'b1;
                div_acc  <= acc_carry ? (acc_nxt - row_px[11:0]) : acc_nxt;
            end else
                out_cnt <= out_cnt + 6'd1;
        end
    end

    // =========================================================================
    // Raster advance
    // =========================================================================
    // Loaded once at lock (first LCD strobe = row 0 pixel 0 -> output sits
    // at the start of group 109); afterwards the counters free-run with
    // exact-integer periods that match the LCD stream, so the lock holds.
    //
    // oh_next/ov_next are shared with the h_wrap/v_wrap conditions used by
    // the source-index tracking below, which advances its own registers on
    // the same tick using the same "about to wrap" comparisons on the
    // CURRENT oh/ov (see the emission block for why that keeps zero net
    // delay without re-deriving oh_next/ov_next as data there).
    wire [9:0] oh_next = (oh >= h_total_m1) ? 10'd0 : oh + 10'd1;
    wire [9:0] ov_next = (oh >= h_total_m1) ? ((ov >= v_total_m1) ? 10'd0 : ov + 10'd1) : ov;

    always @(posedge clk) begin
        if (reset) begin
            oh <= 10'd0;
            ov <= 10'd0;
        end else if (lcd_ce && !started) begin
            oh <= 10'd0;
            ov <= lock_ov;
        end else if (out_tick && started) begin
            oh <= oh_next;
            ov <= ov_next;
        end
    end

    // =========================================================================
    // Output pixel emission — pipelined, division-free source-index tracking
    // =========================================================================
    // px_idx/grp must hold floor(oh/scale) / floor(ov/scale) (the source LCD
    // pixel column / row-group currently being displayed), registered from
    // the NEXT raster position for the same zero-net-delay reason as the
    // oh/ov advance above (a naive register from the CURRENT counters would
    // slip the data one pixel behind DE and corrupt every row).
    //
    // Recomputing floor(x/scale) from scratch every tick (needed only for
    // the non-power-of-2 3x mode; 4x/2x/1x are plain shifts) put a 10-bit
    // constant-divide — synthesized as a DSP multiply-by-reciprocal plus
    // shift — combinationally in front of these registers every tick.
    // Quartus's Timing Closure Recommendations flagged exactly that: "DSP
    // register packing" (the multiplier is used purely combinationally,
    // with no pipeline register of its own) and "long combinational path",
    // both on the status[4] (scale_sel[0]) -> grp[0] path, for all 20
    // px_idx/grp register bits.
    //
    // Fix: track floor(oh/scale) incrementally instead of recomputing it,
    // using the same "accumulate one step at a time, avoid the divider"
    // idiom already used above for the 3x output-pixel timebase
    // (div_acc/out_cnt). oh/ov only ever move by exactly one step per tick,
    // so floor(oh/scale) only needs a small repeat counter: h_rep/v_rep
    // count 0..scale-1 sub-steps per source pixel/row, and px_idx/grp
    // advance by one only when that counter wraps. This is exact (no
    // rounding, no drift) and the combinational depth in front of the
    // registers is now a 2-bit compare against `scale`, not a 10-bit
    // divide — eliminating the DSP block from this path entirely.
    //
    // h_rep/px_idx track oh 1:1 (they step every tick). v_rep/grp track ov,
    // which itself only steps when oh wraps, so they are gated on the same
    // condition as the ov advance above.
    reg [1:0] h_rep, v_rep;
    reg [9:0] px_idx;
    reg [9:0] grp;

    // scale is 1..4, so scale-1 (0..3) fits in 2 bits; for scale==4 (3'b100)
    // the low bits are 2'b00 and subtracting 1 wraps to 2'b11 = 3, which is
    // still the correct scale-1 -- the mod-4 wraparound coincides exactly
    // with the one case (scale==4) that relies on it.
    wire [1:0] scale_m1   = scale[1:0] - 2'd1;
    wire       h_rep_max  = (h_rep >= scale_m1);
    wire       v_rep_max  = (v_rep >= scale_m1);
    wire       h_wrap     = (oh >= h_total_m1);  // last column of the row
    wire       v_wrap     = (ov >= v_total_m1);  // last row of the frame

    always @(posedge clk) begin
        if (reset) begin
            h_rep  <= 2'd0;
            v_rep  <= 2'd0;
            px_idx <= 10'd0;
            grp    <= 10'd0;
        end else if (lcd_ce && !started) begin
            // Mirrors the oh<=0 / ov<=lock_ov jump above: lock_ov is always
            // an exact multiple of scale (== scale*(V_TOTAL-1)), so the
            // matching source indices are always px_idx=0 / grp=V_TOTAL-1
            // with both repeat counters at 0, regardless of scale.
            h_rep  <= 2'd0;
            v_rep  <= 2'd0;
            px_idx <= 10'd0;
            grp    <= {2'd0, V_TOTAL - 8'd1};
        end else if (out_tick && started) begin
            // Horizontal: steps every tick, in lock-step with oh.
            if (h_wrap) begin
                h_rep  <= 2'd0;
                px_idx <= 10'd0;
            end else if (h_rep_max) begin
                h_rep  <= 2'd0;
                px_idx <= px_idx + 10'd1;
            end else begin
                h_rep <= h_rep + 2'd1;
            end

            // Vertical: steps only when the horizontal raster wraps — the
            // same condition under which ov itself advances above.
            if (h_wrap) begin
                if (v_wrap) begin
                    v_rep <= 2'd0;
                    grp   <= 10'd0;
                end else if (v_rep_max) begin
                    v_rep <= 2'd0;
                    grp   <= grp + 10'd1;
                end else begin
                    v_rep <= v_rep + 2'd1;
                end
            end
        end
    end

    // RAM read using the registered indices.
    wire        rsel  = grp[0];
    wire [15:0] rword = lbuf[rsel][px_idx[7:4]];
    wire        rpix  = rword[15 - px_idx[3:0]];

    // Colour selection (combinational — same as the original design).
    wire [23:0] pix_rgb = rpix ? fg_rgb : bg_rgb;

    // Region flags (combinational from counters — short paths)
    wire de_area = (oh <= h_active_m1) && (ov <= v_active_m1);
    wire hs_area = (oh >= hs_start) && (oh < hs_end);
    wire vs_area = (ov >= vs_start) && (ov < vs_end);

    // Boot-status diagnostic fill
    reg [23:0] status_rgb;

    always @(*) begin
        case (boot_status)
            3'd0:    status_rgb = 24'h2060D0; // blue   - no image loaded
            3'd1:    status_rgb = 24'hF08000; // orange - loading image
            3'd2:    status_rgb = 24'hD02020; // red    - image not recognized
            3'd3:    status_rgb = 24'h9030D0; // violet - booting
            default: status_rgb = 24'h000000; // unused (4+ = normal raster)
        endcase
    end

    // =========================================================================
    // Real-Time On-Screen Visual Debug Hex Overlay HUD
    // =========================================================================
    // Displays a 2-line HUD on LCD rows 0..11 across columns 0..159 (40 chars/line):
    // Line 0: "PC:XXXXXX A:XXXXXX D:XXXX RD IPL:X"
    // Line 1: "INT:XXXX FLW:XXXX L:1 P:1 S:0 7:0 ST:4"

    wire       hud_line = (grp >= 10'd6);
    wire [2:0] hud_y    = (grp < 10'd6) ? grp[2:0] : (grp[2:0] - 3'd6);
    wire [5:0] hud_col  = px_idx[7:2]; // 0..39
    wire [1:0] hud_x    = px_idx[1:0]; // 0..3

    reg [5:0] char_code;

    always @(*) begin
        if (!hud_line) begin
            case (hud_col)
                6'd0:  char_code = 6'd18; // 'P'
                6'd1:  char_code = 6'd19; // 'C'
                6'd2:  char_code = 6'd17; // ':'
                6'd3:  char_code = {2'b00, dbg_pc[23:20]};
                6'd4:  char_code = {2'b00, dbg_pc[19:16]};
                6'd5:  char_code = {2'b00, dbg_pc[15:12]};
                6'd6:  char_code = {2'b00, dbg_pc[11:8]};
                6'd7:  char_code = {2'b00, dbg_pc[7:4]};
                6'd8:  char_code = {2'b00, dbg_pc[3:0]};
                6'd9:  char_code = 6'd16; // ' '
                6'd10: char_code = 6'd20; // 'A'
                6'd11: char_code = 6'd17; // ':'
                6'd12: char_code = {2'b00, dbg_addr[23:20]};
                6'd13: char_code = {2'b00, dbg_addr[19:16]};
                6'd14: char_code = {2'b00, dbg_addr[15:12]};
                6'd15: char_code = {2'b00, dbg_addr[11:8]};
                6'd16: char_code = {2'b00, dbg_addr[7:4]};
                6'd17: char_code = {2'b00, dbg_addr[3:0]};
                6'd18: char_code = 6'd16; // ' '
                6'd19: char_code = 6'd21; // 'D'
                6'd20: char_code = 6'd17; // ':'
                6'd21: char_code = {2'b00, dbg_data[15:12]};
                6'd22: char_code = {2'b00, dbg_data[11:8]};
                6'd23: char_code = {2'b00, dbg_data[7:4]};
                6'd24: char_code = {2'b00, dbg_data[3:0]};
                6'd25: char_code = 6'd16; // ' '
                6'd26: char_code = dbg_rw ? 6'd22 : 6'd23; // 'R' or 'W'
                6'd27: char_code = dbg_rw ? 6'd21 : 6'd22; // 'D' or 'R' (RD or WR)
                6'd28: char_code = 6'd16; // ' '
                6'd29: char_code = 6'd24; // 'I'
                6'd30: char_code = 6'd18; // 'P'
                6'd31: char_code = 6'd28; // 'L'
                6'd32: char_code = 6'd17; // ':'
                6'd33: char_code = {3'b000, dbg_ipl};
                default: char_code = 6'd16; // ' '
            endcase
        end else begin
            case (hud_col)
                6'd0:  char_code = 6'd24; // 'I'
                6'd1:  char_code = 6'd25; // 'N'
                6'd2:  char_code = 6'd26; // 'T'
                6'd3:  char_code = 6'd17; // ':'
                6'd4:  char_code = {2'b00, dbg_int_cnt[15:12]};
                6'd5:  char_code = {2'b00, dbg_int_cnt[11:8]};
                6'd6:  char_code = {2'b00, dbg_int_cnt[7:4]};
                6'd7:  char_code = {2'b00, dbg_int_cnt[3:0]};
                6'd8:  char_code = 6'd16; // ' '
                6'd9:  char_code = 6'd27; // 'F'
                6'd10: char_code = 6'd28; // 'L'
                6'd11: char_code = 6'd23; // 'W'
                6'd12: char_code = 6'd17; // ':'
                6'd13: char_code = {2'b00, dbg_flw_cnt[15:12]};
                6'd14: char_code = {2'b00, dbg_flw_cnt[11:8]};
                6'd15: char_code = {2'b00, dbg_flw_cnt[7:4]};
                6'd16: char_code = {2'b00, dbg_flw_cnt[3:0]};
                6'd17: char_code = 6'd16; // ' '
                6'd18: char_code = 6'd28; // 'L'
                6'd19: char_code = 6'd17; // ':'
                6'd20: char_code = dbg_lcd_on ? 6'd1 : 6'd0;
                6'd21: char_code = 6'd16; // ' '
                6'd22: char_code = 6'd18; // 'P'
                6'd23: char_code = 6'd17; // ':'
                6'd24: char_code = dbg_protect ? 6'd1 : 6'd0;
                6'd25: char_code = 6'd16; // ' '
                6'd26: char_code = 6'd29; // 'S'
                6'd27: char_code = 6'd17; // ':'
                6'd28: char_code = dbg_stopped ? 6'd1 : 6'd0;
                6'd29: char_code = 6'd16; // ' '
                6'd30: char_code = 6'd7;  // '7'
                6'd31: char_code = 6'd17; // ':'
                6'd32: char_code = dbg_ai7 ? 6'd1 : 6'd0;
                6'd33: char_code = 6'd16; // ' '
                6'd34: char_code = 6'd29; // 'S'
                6'd35: char_code = 6'd26; // 'T'
                6'd36: char_code = 6'd17; // ':'
                6'd37: char_code = {3'b000, boot_status};
                default: char_code = 6'd16; // ' '
            endcase
        end
    end

    reg [2:0] font_bits;
    always @(*) begin
        if (hud_y >= 3'd5) begin
            font_bits = 3'b000;
        end else begin
            case (char_code)
                6'd0:  case (hud_y) 3'd0: font_bits = 3'b111; 3'd1: font_bits = 3'b101; 3'd2: font_bits = 3'b101; 3'd3: font_bits = 3'b101; default: font_bits = 3'b111; endcase
                6'd1:  case (hud_y) 3'd0: font_bits = 3'b010; 3'd1: font_bits = 3'b110; 3'd2: font_bits = 3'b010; 3'd3: font_bits = 3'b010; default: font_bits = 3'b111; endcase
                6'd2:  case (hud_y) 3'd0: font_bits = 3'b111; 3'd1: font_bits = 3'b001; 3'd2: font_bits = 3'b111; 3'd3: font_bits = 3'b100; default: font_bits = 3'b111; endcase
                6'd3:  case (hud_y) 3'd0: font_bits = 3'b111; 3'd1: font_bits = 3'b001; 3'd2: font_bits = 3'b111; 3'd3: font_bits = 3'b001; default: font_bits = 3'b111; endcase
                6'd4:  case (hud_y) 3'd0: font_bits = 3'b101; 3'd1: font_bits = 3'b101; 3'd2: font_bits = 3'b111; 3'd3: font_bits = 3'b001; default: font_bits = 3'b001; endcase
                6'd5:  case (hud_y) 3'd0: font_bits = 3'b111; 3'd1: font_bits = 3'b100; 3'd2: font_bits = 3'b111; 3'd3: font_bits = 3'b001; default: font_bits = 3'b111; endcase
                6'd6:  case (hud_y) 3'd0: font_bits = 3'b111; 3'd1: font_bits = 3'b100; 3'd2: font_bits = 3'b111; 3'd3: font_bits = 3'b101; default: font_bits = 3'b111; endcase
                6'd7:  case (hud_y) 3'd0: font_bits = 3'b111; 3'd1: font_bits = 3'b001; 3'd2: font_bits = 3'b010; 3'd3: font_bits = 3'b010; default: font_bits = 3'b010; endcase
                6'd8:  case (hud_y) 3'd0: font_bits = 3'b111; 3'd1: font_bits = 3'b101; 3'd2: font_bits = 3'b111; 3'd3: font_bits = 3'b101; default: font_bits = 3'b111; endcase
                6'd9:  case (hud_y) 3'd0: font_bits = 3'b111; 3'd1: font_bits = 3'b101; 3'd2: font_bits = 3'b111; 3'd3: font_bits = 3'b001; default: font_bits = 3'b111; endcase
                6'd10: case (hud_y) 3'd0: font_bits = 3'b111; 3'd1: font_bits = 3'b101; 3'd2: font_bits = 3'b111; 3'd3: font_bits = 3'b101; default: font_bits = 3'b101; endcase // 'A'
                6'd11: case (hud_y) 3'd0: font_bits = 3'b110; 3'd1: font_bits = 3'b101; 3'd2: font_bits = 3'b110; 3'd3: font_bits = 3'b101; default: font_bits = 3'b110; endcase // 'B'
                6'd12: case (hud_y) 3'd0: font_bits = 3'b111; 3'd1: font_bits = 3'b100; 3'd2: font_bits = 3'b100; 3'd3: font_bits = 3'b100; default: font_bits = 3'b111; endcase // 'C'
                6'd13: case (hud_y) 3'd0: font_bits = 3'b110; 3'd1: font_bits = 3'b101; 3'd2: font_bits = 3'b101; 3'd3: font_bits = 3'b101; default: font_bits = 3'b110; endcase // 'D'
                6'd14: case (hud_y) 3'd0: font_bits = 3'b111; 3'd1: font_bits = 3'b100; 3'd2: font_bits = 3'b111; 3'd3: font_bits = 3'b100; default: font_bits = 3'b111; endcase // 'E'
                6'd15: case (hud_y) 3'd0: font_bits = 3'b111; 3'd1: font_bits = 3'b100; 3'd2: font_bits = 3'b110; 3'd3: font_bits = 3'b100; default: font_bits = 3'b100; endcase // 'F'
                6'd16: font_bits = 3'b000; // ' '
                6'd17: case (hud_y) 3'd1: font_bits = 3'b010; 3'd3: font_bits = 3'b010; default: font_bits = 3'b000; endcase // ':'
                6'd18: case (hud_y) 3'd0: font_bits = 3'b111; 3'd1: font_bits = 3'b101; 3'd2: font_bits = 3'b111; 3'd3: font_bits = 3'b100; default: font_bits = 3'b100; endcase // 'P'
                6'd19: case (hud_y) 3'd0: font_bits = 3'b111; 3'd1: font_bits = 3'b100; 3'd2: font_bits = 3'b100; 3'd3: font_bits = 3'b100; default: font_bits = 3'b111; endcase // 'C'
                6'd20: case (hud_y) 3'd0: font_bits = 3'b111; 3'd1: font_bits = 3'b101; 3'd2: font_bits = 3'b111; 3'd3: font_bits = 3'b101; default: font_bits = 3'b101; endcase // 'A'
                6'd21: case (hud_y) 3'd0: font_bits = 3'b110; 3'd1: font_bits = 3'b101; 3'd2: font_bits = 3'b101; 3'd3: font_bits = 3'b101; default: font_bits = 3'b110; endcase // 'D'
                6'd22: case (hud_y) 3'd0: font_bits = 3'b111; 3'd1: font_bits = 3'b101; 3'd2: font_bits = 3'b110; 3'd3: font_bits = 3'b101; default: font_bits = 3'b101; endcase // 'R'
                6'd23: case (hud_y) 3'd0: font_bits = 3'b101; 3'd1: font_bits = 3'b101; 3'd2: font_bits = 3'b101; 3'd3: font_bits = 3'b111; default: font_bits = 3'b101; endcase // 'W'
                6'd24: case (hud_y) 3'd0: font_bits = 3'b111; 3'd1: font_bits = 3'b010; 3'd2: font_bits = 3'b010; 3'd3: font_bits = 3'b010; default: font_bits = 3'b111; endcase // 'I'
                6'd25: case (hud_y) 3'd0: font_bits = 3'b111; 3'd1: font_bits = 3'b101; 3'd2: font_bits = 3'b101; 3'd3: font_bits = 3'b101; default: font_bits = 3'b101; endcase // 'N'
                6'd26: case (hud_y) 3'd0: font_bits = 3'b111; 3'd1: font_bits = 3'b010; 3'd2: font_bits = 3'b010; 3'd3: font_bits = 3'b010; default: font_bits = 3'b010; endcase // 'T'
                6'd27: case (hud_y) 3'd0: font_bits = 3'b111; 3'd1: font_bits = 3'b100; 3'd2: font_bits = 3'b110; 3'd3: font_bits = 3'b100; default: font_bits = 3'b100; endcase // 'F'
                6'd28: case (hud_y) 3'd0: font_bits = 3'b100; 3'd1: font_bits = 3'b100; 3'd2: font_bits = 3'b100; 3'd3: font_bits = 3'b100; default: font_bits = 3'b111; endcase // 'L'
                6'd29: case (hud_y) 3'd0: font_bits = 3'b011; 3'd1: font_bits = 3'b100; 3'd2: font_bits = 3'b010; 3'd3: font_bits = 3'b001; default: font_bits = 3'b110; endcase // 'S'
                default: font_bits = 3'b000;
            endcase
        end
    end

    wire hud_pixel  = (hud_x < 2'd3) && font_bits[2 - hud_x];
    wire hud_active = dbg_en && (grp < 10'd13);
    wire [23:0] hud_rgb = (grp == 10'd12) ? 24'h306090 : (hud_pixel ? 24'hFFE020 : 24'h081018);

    wire [23:0] active_rgb = hud_active ? hud_rgb : pix_rgb;
    wire [23:0] draw_rgb   = (boot_status != 3'd4 && !hud_active) ? status_rgb : active_rgb;

    // Output registers
    always @(posedge clk) begin
        if (reset) begin
            ce_pix <= 1'b0;
            R      <= 8'd0;
            G      <= 8'd0;
            B      <= 8'd0;
            HSync  <= 1'b0;
            VSync  <= 1'b0;
            DE     <= 1'b0;
        end else begin
            ce_pix <= out_tick && started;
            if (out_tick && started) begin
                R     <= de_area ? draw_rgb[23:16] : 8'd0;
                G     <= de_area ? draw_rgb[15:8]  : 8'd0;
                B     <= de_area ? draw_rgb[7:0]   : 8'd0;
                HSync <= hs_area;
                VSync <= vs_area;
                DE    <= de_area;
            end else if (!started) begin
                R      <= 8'd0;
                G      <= 8'd0;
                B      <= 8'd0;
                HSync  <= 1'b0;
                VSync  <= 1'b0;
                DE     <= 1'b0;
            end
        end
    end

endmodule

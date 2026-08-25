//
// video_scaler.sv — TI-89 LCD raster scaler for MiSTer video path
// TI-89 MiSTer Core
//
// lcd_ctrl generates a 160x100 active LCD raster where one LCD pixel
// lasts 48 master clock cycles (pixel strobe pix_ce = clk/48, asserted
// for active pixels only; blanking intervals arrive as gaps between
// strobes). One LCD row therefore lasts 200*48 = 9600 master clocks and
// one LCD frame 110 rows = 1,056,000 clocks (~60.6 Hz).
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
                if (!started)
                    started <= 1'b1;
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
    always @(posedge clk) begin
        if (reset) begin
            oh <= 10'd0;
            ov <= 10'd0;
        end else if (lcd_ce && !started) begin
            oh <= 10'd0;
            ov <= lock_ov;
        end else if (out_tick && started) begin
            if (oh >= h_total_m1) begin
                oh <= 10'd0;
                if (ov >= v_total_m1)
                    ov <= 10'd0;
                else
                    ov <= ov + 10'd1;
            end else
                oh <= oh + 10'd1;
        end
    end

    // =========================================================================
    // Output pixel emission
    // =========================================================================
    // Output row ov shows LCD row ov/scale, captured one LCD row earlier
    // into bank (ov/scale)[0].
    reg [9:0] px_idx;    // LCD pixel x being displayed
    reg [9:0] grp;       // output group (LCD row shown) = ov / scale
    always @(*) begin
        case (scale_sel)
            2'd0:    begin px_idx = {2'd0, oh[9:2]}; grp = {2'd0, ov[9:2]}; end
            2'd1:    begin px_idx = oh / 10'd3;      grp = ov / 10'd3;      end
            2'd2:    begin px_idx = {1'd0, oh[9:1]}; grp = {1'd0, ov[9:1]}; end
            default: begin px_idx = oh;              grp = ov;              end
        endcase
    end

    wire        rsel  = grp[0];
    wire [15:0] rword = lbuf[rsel][px_idx[7:4]];
    wire        rpix  = rword[15 - px_idx[3:0]];

    wire de_area = (oh <= h_active_m1) && (ov <= v_active_m1);
    wire hs_area = (oh >= hs_start) && (oh < hs_end);
    wire vs_area = (ov >= vs_start) && (ov < vs_end);

    wire [23:0] pix_rgb = rpix ? fg_rgb : bg_rgb;

    // =========================================================================
    // Boot-status diagnostic fill
    // =========================================================================
    // Paint the whole active area with a saturated color per boot_status.
    // This can never hide real LCD content: lcd_on (and therefore any real
    // pixel stream) can only become active after boot_done, at which point
    // boot_status is 4 and the normal palette raster is shown. Sync, DE and
    // ce_pix timing are untouched, so ascal keeps locking onto the raster.

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

    wire [23:0] draw_rgb = (boot_status != 3'd4) ? status_rgb : pix_rgb;

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

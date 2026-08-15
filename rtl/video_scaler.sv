//
// video_scaler.sv — TI-89 LCD raster scaler for MiSTer video path
// TI-89 MiSTer Core
//
// lcd_ctrl generates a 200x110 LCD raster (160x100 active) where one
// LCD pixel lasts 48 master clock cycles (pixel strobe pix_ce = clk/48).
//
// This scaler re-emits every LCD pixel as SCALE x SCALE output pixels:
//
//   scale_sel | scale | out divider | output raster | pixel clock
//   ----------+-------+-------------+---------------+------------
//       0     |  4x   |  48/4 = 12  |   800 x 440   |  5.33 MHz
//       1     |  3x   |  48/3 = 16  |   600 x 330   |  4.00 MHz
//       2     |  2x   |  48/2 = 24  |   400 x 220   |  2.67 MHz
//       3     |  1x   |  48/1 = 48  |   200 x 110   |  1.33 MHz
//
// The output frame rate is ~60 Hz in all modes. The MiSTer framework
// (ascal) scales this raster to the selected HDMI resolution, so the
// scaler only has to produce a clean, steady VGA-style signal:
// positive-sync HS/VS and DE = active drawing area.
//
// The LCD raster constants below must stay in sync with lcd_ctrl.sv.
//

module video_scaler (
    input             clk,        // Master clock (64 MHz)
    input             reset,

    // From lcd_ctrl
    input             pix_ce,     // LCD pixel clock enable (clk/48)
    input             pixel,      // LCD pixel: 1 = dark, 0 = background

    // OSD options
    input       [1:0] scale_sel,  // 0 = 4x, 1 = 3x, 2 = 2x, 3 = 1x
    input       [1:0] color_sel,  // 0 = green, 1 = blue, 2 = amber, 3 = B&W

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

    // Sync positions within the LCD raster (in LCD pixel units)
    localparam [8:0] HS_START = 9'd164;
    localparam [8:0] HS_END   = 9'd167;   // exclusive
    localparam [7:0] VS_START = 8'd102;
    localparam [7:0] VS_END   = 8'd104;   // exclusive

    // =========================================================================
    // Scale factor decode
    // =========================================================================
    reg [2:0] scale;      // 1..4
    reg [5:0] out_div;    // master clocks per output pixel = 48/scale

    always @(*) begin
        case (scale_sel)
            2'd0:    begin scale = 3'd4; out_div = 6'd12; end
            2'd1:    begin scale = 3'd3; out_div = 6'd16; end
            2'd2:    begin scale = 3'd2; out_div = 6'd24; end
            default: begin scale = 3'd1; out_div = 6'd48; end
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
    // everything is sampled cleanly, then phase-lock the output raster to it.
    //
    // Both pix_ce (period 48) and the output pixel grid (period 48/scale)
    // start together when 'started' is set, and 48 is an integer multiple of
    // every possible output period (12/16/24/48), so the two grids stay
    // aligned forever: each LCD pixel maps to exactly 'scale' output pixels.

    reg [2:0] ce_d;
    wire      lcd_ce = ce_d[2];

    always @(posedge clk) begin
        if (reset)
            ce_d <= 3'd0;
        else
            ce_d <= {ce_d[1:0], pix_ce};
    end

    // =========================================================================
    // Raster state
    // =========================================================================
    reg        started;    // output raster is phase-locked to the LCD stream
    reg [9:0]  oh;         // output horizontal position (0..h_total_m1)
    reg [9:0]  ov;         // output vertical position   (0..v_total_m1)
    reg        pix_state;  // current LCD pixel, held for 'scale' output pixels
    reg [5:0]  out_cnt;    // output pixel divider (from the timebase below)
    reg        out_tick;

    always @(posedge clk) begin
        if (reset) begin
            started   <= 1'b0;
            oh        <= 10'd0;
            ov        <= 10'd0;
            pix_state <= 1'b0;
        end else if (lcd_ce) begin
            // New LCD pixel arrives: capture it and (re-)align the raster.
            pix_state <= pixel;
            if (!started) begin
                started <= 1'b1;
                oh      <= 10'd0;
                ov      <= 10'd0;
            end
        end else if (out_tick && started) begin
            if (oh >= h_total_m1) begin
                oh <= 10'd0;
                if (ov >= v_total_m1)
                    ov <= 10'd0;
                else
                    ov <= ov + 10'd1;
            end else begin
                oh <= oh + 10'd1;
            end
        end
    end

    // =========================================================================
    // Output pixel timebase
    // =========================================================================
    // Every lcd_ce (one per LCD pixel, period 48) re-seeds the divider so
    // the output spans stay locked to the LCD pixels; between LCD pixels the
    // counter free-runs. Because 48 is an integer multiple of every possible
    // out_div (12/16/24/48), the free-running count would land on the same
    // value at each re-seed, so the re-seed only corrects drift, never
    // disturbs the grid. The seed value of 1 makes the first output span
    // last exactly out_div cycles (the first span of a plain 0..div-1 count
    // would be off by one).

    always @(posedge clk) begin
        if (reset) begin
            out_cnt  <= 6'd0;
            out_tick <= 1'b0;
        end else if (lcd_ce) begin
            out_cnt  <= 6'd1;   // phase-align output spans with the LCD pixel
            out_tick <= 1'b0;
        end else if (started) begin
            out_tick <= 1'b0;
            if (out_cnt >= out_div - 6'd1) begin
                out_cnt  <= 6'd0;
                out_tick <= 1'b1;
            end else begin
                out_cnt <= out_cnt + 6'd1;
            end
        end else begin
            out_cnt  <= 6'd0;
            out_tick <= 1'b0;
        end
    end

    // =========================================================================
    // Output pixel emission
    // =========================================================================
    wire de_area = (oh <= h_active_m1) && (ov <= v_active_m1);
    wire hs_area = (oh >= hs_start) && (oh < hs_end);
    wire vs_area = (ov >= vs_start) && (ov < vs_end);

    wire [23:0] pix_rgb = pix_state ? fg_rgb : bg_rgb;

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
                R     <= de_area ? pix_rgb[23:16] : 8'd0;
                G     <= de_area ? pix_rgb[15:8]  : 8'd0;
                B     <= de_area ? pix_rgb[7:0]   : 8'd0;
                HSync <= hs_area;
                VSync <= vs_area;
                DE    <= de_area;
            end else if (!started) begin
                R     <= 8'd0;
                G     <= 8'd0;
                B     <= 8'd0;
                HSync <= 1'b0;
                VSync <= 1'b0;
                DE    <= 1'b0;
            end
        end
    end

endmodule

//
// lcd_ctrl.sv — TI-89 LCD DMA Controller
// TI-89 MiSTer Core
//
// The TI-89 LCD is 160×100 pixels, monochrome (1 bit per pixel).
// LCD memory lives in the calculator's RAM. On HW2+ (Titanium) the base
// byte address is computed by the I/O port logic:
//     base = $4C00 + $1000 * (io2[$17] & 3)
// and fed to this controller in lcd_base_addr (already a byte address).
//
// Each row of the display is (log_w / 8) bytes = 20 bytes = 10 words.
// Total LCD memory = 20 bytes × 100 rows = 2000 bytes.
//
// This controller DMA-reads one row at a time from RAM into a line
// buffer during the previous line's horizontal blank (and line 0 during
// the last line of vertical blank), then shifts pixels out serially at
// the LCD pixel clock (clk / 48 ≈ 1.33 MHz → ~60 Hz frame rate).
//
// Pixels are ordered MSB-first within each 16-bit word (68k big-endian).
//

module lcd_ctrl (
    input         clk,
    input         reset,

    // LCD configuration from I/O ports
    input  [15:0] lcd_base_addr, // LCD base BYTE address (from io_ports)
    input   [7:0] lcd_log_w,     // Logical width register ($600012)
    input   [7:0] lcd_log_h,     // Logical height register ($600013)
    input   [3:0] lcd_contrast,  // Contrast level
    input         lcd_on,        // LCD enabled

    // RAM DMA interface (reads port B of the dual-port RAM)
    // ram_addr is held stable for one cycle before ram_data is valid
    // (synchronous RAM read: address this cycle, data next cycle).
    output reg [17:0] ram_addr,  // Byte address within 256KB RAM (even)
    input      [15:0] ram_data,  // 16-bit data from RAM

    // Video output interface (directly to scaler)
    output reg        pixel_out,  // 1 = pixel on, 0 = pixel off
    output reg        pixel_valid,// Pixel data is valid
    output reg        hsync,      // Horizontal sync (active high, 1 cycle)
    output reg        vsync,      // Vertical sync (active high, 1 cycle)
    output reg        hblank,     // Horizontal blanking
    output reg        vblank,     // Vertical blanking
    output reg  [7:0] pixel_x,   // Current pixel X coordinate (0-159)
    output reg  [6:0] pixel_y    // Current pixel Y coordinate (0-99)
);

    // =========================================================================
    // LCD timing parameters
    // =========================================================================
    // Pixel clock: 160 pixels + blanking ≈ 200 clocks per line
    //              100 lines + blanking ≈ 110 lines per frame
    //              200 * 110 = 22000 pixel clocks per frame
    //              22000 * 60 Hz = 1.32 MHz pixel clock
    //              64 MHz master / 48 ≈ 1.333 MHz  ✓

    localparam [8:0] H_ACTIVE  = 9'd160;
    localparam [8:0] H_TOTAL   = 9'd200;

    localparam [7:0] V_ACTIVE  = 8'd100;
    localparam [7:0] V_TOTAL   = 8'd110;

    localparam [5:0] PIX_DIV   = 6'd48; // Master clock divider for pixel clock

    // =========================================================================
    // LCD base byte address
    // =========================================================================
    // lcd_base_addr is already the byte address (io_ports computes
    // $4C00 + $1000*bank for HW2+). Only the low 15 bits can matter
    // (RAM is 256KB); keep 18 bits for address arithmetic safety.
    wire [17:0] lcd_mem_base = {2'd0, lcd_base_addr};

    // =========================================================================
    // Pixel clock divider
    // =========================================================================

    reg [5:0] pix_counter;
    reg       pix_tick;

    always @(posedge clk) begin
        if (reset) begin
            pix_counter <= 6'd0;
            pix_tick    <= 1'b0;
        end else begin
            pix_tick <= 1'b0;
            if (pix_counter >= PIX_DIV - 6'd1) begin
                pix_counter <= 6'd0;
                pix_tick    <= 1'b1;
            end else begin
                pix_counter <= pix_counter + 6'd1;
            end
        end
    end

    // =========================================================================
    // Scan position counters
    // =========================================================================
    // Reset into the last pixel of the last vertical-blank line so the very
    // first action after reset is the DMA trigger for line 0; by the time
    // the raster reaches line 0 it is already in the line buffer.

    reg [8:0] h_count; // 0 to H_TOTAL-1
    reg [7:0] v_count; // 0 to V_TOTAL-1

    // Line buffer: stores one row of pixels (160 bits = 10 words)
    reg [15:0] line_buf [0:9];

    // =========================================================================
    // DMA: Fetch one line from RAM into line buffer
    // =========================================================================
    // Row N is fetched during the horizontal blanking of row N-1.
    // Row 0 is fetched during the last line of vertical blanking.
    // Each fetch = 10 word reads × 3 cycles = 30 master clocks, far less
    // than one horizontal blank period (40 pixels × 48 = 1920 clocks).

    localparam [1:0] DMA_IDLE  = 2'd0;
    localparam [1:0] DMA_FETCH = 2'd1; // present address to RAM
    localparam [1:0] DMA_WAIT  = 2'd2; // RAM read latency (1 cycle)
    localparam [1:0] DMA_STORE = 2'd3; // latch ram_data into line_buf

    reg [1:0]  dma_state;
    reg [3:0]  dma_word;
    reg [17:0] dma_addr;

    // Row to fetch when the trigger fires: during line N's blank we fetch
    // line N+1; during all vertical-blank lines we keep refreshing line 0.
    wire [6:0] fetch_row = (v_count < V_ACTIVE - 8'd1) ? (v_count[6:0] + 7'd1)
                                                       : 7'd0;
    // Row byte offset = row * 20 = row*16 + row*4 (max 99*20 = 1980)
    wire [17:0] row_offset = ({11'd0, fetch_row} << 4) + ({11'd0, fetch_row} << 2);

    wire dma_trigger = pix_tick && (h_count == H_ACTIVE);

    always @(posedge clk) begin
        if (reset) begin
            dma_state <= DMA_IDLE;
            dma_word  <= 4'd0;
            dma_addr  <= 18'd0;
            ram_addr  <= 18'd0;
        end else begin
            case (dma_state)
                DMA_IDLE: begin
                    if (dma_trigger) begin
                        dma_state <= DMA_FETCH;
                        dma_word  <= 4'd0;
                        dma_addr  <= lcd_mem_base + row_offset;
                    end
                end

                DMA_FETCH: begin
                    // Present word address; RAM data available next cycle
                    ram_addr  <= dma_addr + {13'd0, dma_word, 1'b0};
                    dma_state <= DMA_WAIT;
                end

                DMA_WAIT: begin
                    dma_state <= DMA_STORE;
                end

                DMA_STORE: begin
                    line_buf[dma_word] <= ram_data;
                    if (dma_word >= 4'd9) begin
                        dma_state <= DMA_IDLE;
                    end else begin
                        dma_word  <= dma_word + 4'd1;
                        dma_state <= DMA_FETCH;
                    end
                end
            endcase
        end
    end

    // =========================================================================
    // Scan output — pixel generation
    // =========================================================================

    always @(posedge clk) begin
        if (reset) begin
            h_count     <= H_ACTIVE;        // first hblank tick of the last
            v_count     <= V_TOTAL - 8'd1;  // vblank line -> fetch line 0 first
            pixel_out   <= 1'b0;
            pixel_valid <= 1'b0;
            hsync       <= 1'b0;
            vsync       <= 1'b0;
            hblank      <= 1'b1;
            vblank      <= 1'b1;
            pixel_x     <= 8'd0;
            pixel_y     <= 7'd0;
        end else if (pix_tick) begin
            // Default outputs
            pixel_valid <= 1'b0;
            hsync       <= 1'b0;
            vsync       <= 1'b0;

            // Horizontal counter
            if (h_count >= H_TOTAL - 9'd1) begin
                h_count <= 9'd0;
                // Vertical counter
                if (v_count >= V_TOTAL - 8'd1)
                    v_count <= 8'd0;
                else
                    v_count <= v_count + 8'd1;
            end else begin
                h_count <= h_count + 9'd1;
            end

            // Blanking signals (values for the pixel period starting now)
            hblank <= (h_count >= H_ACTIVE);
            vblank <= (v_count >= V_ACTIVE);

            // Sync pulses
            if (h_count == H_ACTIVE + 9'd4)
                hsync <= 1'b1;
            if (v_count == V_ACTIVE + 8'd2 && h_count == H_ACTIVE + 9'd4)
                vsync <= 1'b1;

            // Active pixel output
            if (h_count < H_ACTIVE && v_count < V_ACTIVE && lcd_on) begin
                // Extract pixel from line buffer, MSB-first per word
                pixel_out   <= line_buf[h_count[7:4]][4'd15 - h_count[3:0]];
                pixel_valid <= 1'b1;
                pixel_x     <= h_count[7:0];
                pixel_y     <= v_count[6:0];
            end else if (h_count < H_ACTIVE && v_count < V_ACTIVE && !lcd_on) begin
                pixel_out   <= 1'b0; // LCD off = blank (background)
                pixel_valid <= 1'b1;
                pixel_x     <= h_count[7:0];
                pixel_y     <= v_count[6:0];
            end
        end
    end

endmodule

//
// io_ports.sv — TI-89 Titanium memory-mapped I/O register controller
// TI-89 MiSTer Core
//
// Implements the I/O register spaces (from TiEmu ports.c, HW2+/HW3 paths):
//   $600000-$6FFFFF : I/O Bank 1 — 32 bytes, mirrored over the whole MB
//   $700000-$70003F : I/O Bank 2 — 64 bytes
//   $710000-$7100FF : I/O Bank 3 — 256 bytes
//
// The bus is 16 bits wide; byte lanes follow 68000 conventions:
//   - word access (uds_n=0, lds_n=0) at even address A:
//       upper byte <- register A, lower byte <- register A+1
//   - byte access at address X (even: UDS, odd: LDS):
//       the register byte is driven on both lanes, the CPU picks its lane
//
// rdata is combinational; mem_ctrl captures it one cycle after asserting
// the rd strobe.
//

module io_ports (
    input         clk,
    input         reset,

    // Bus interface from memory controller
    input   [7:0] addr,         // Byte address within the bank
    input  [15:0] wdata,        // Write data
    output [15:0] rdata,        // Read data (combinational)
    input         rd,           // One-cycle read strobe
    input         wr,           // One-cycle write strobe
    input   [1:0] bank,         // 0=IO1, 1=IO2, 2=IO3
    input         uds_n,        // Upper byte strobe (valid with rd/wr)
    input         lds_n,        // Lower byte strobe (valid with rd/wr)
    input         protect,      // Flash protection armed (mem_ctrl hwprot)

    // Keyboard controller interface
    output  [9:0] kbd_row_mask, // Row selection mask (10 bits: $600018-$600019)
    input   [7:0] kbd_col_data, // Column data from keyboard matrix
    input         on_key,       // ON key state (1 = pressed)

    // Timer interface
    output  [7:0] timer_ctrl,   // Timer control register ($600015)
    output  [7:0] timer_init,   // Timer init value ($600017)
    input   [7:0] timer_value,  // Current timer value

    // LCD interface
    output [15:0] lcd_addr,     // LCD base byte address (HW2+ decode)
    output  [7:0] lcd_log_w,    // LCD logical width register ($600012)
    output  [7:0] lcd_log_h,    // LCD logical height register ($600013)
    output  [3:0] lcd_contrast, // LCD contrast ($60001D)
    output        lcd_on,       // LCD enabled
    input         lcd_vsync,    // One-frame pulse (toggles $70001D bit 7)

    // CPU control
    output        cpu_stop,     // Write to $600005 (one-cycle pulse)
    output  [4:0] stop_mask,    // Wake-up interrupt mask ($600005 bits 4:0)

    // Interrupt acknowledgement (one-cycle pulses)
    output        ack_ai2,      // Write to $60001B acknowledges AI2
    output        ack_ai6,      // Write to $60001A acknowledges AI6

    // Programmable timer reload strobe (one-cycle pulse, one cycle after
    // the bus write so io1[$17] already carries the new value)
    output        timer_load,   // Write to $600017 resets the timer value

    // AI7 arm bit: $600001 bit 2 (io_bit_tst(0x01, 2) in mem.c). While
    // set, any CPU write below $000120 raises level-7 autovector
    // ("vector table write protection & stack overflow").
    output        prot_arm
);

    // =========================================================================
    // Register files (byte-addressed)
    // =========================================================================
    reg [7:0] io1 [0:31];
    reg [7:0] io2 [0:63];
    reg [7:0] io3 [0:255];

    // Pulse outputs
    reg cpu_stop_pulse;
    reg ack_ai2_pulse;
    reg ack_ai6_pulse;
    reg timer_load_pulse;

    assign cpu_stop   = cpu_stop_pulse;
    assign ack_ai2    = ack_ai2_pulse;
    assign ack_ai6    = ack_ai6_pulse;
    assign timer_load = timer_load_pulse;
    assign prot_arm   = io1[5'h01][2];

    // =========================================================================
    // Write addresses and byte lanes (68000 bus conventions)
    // =========================================================================
    // mem_ctrl supplies an even-aligned address (A0 is dropped there; for
    // RAM/flash the byte lanes already encode odd/even). On the I/O bus
    // the lanes must be decoded back into register offsets:
    //   word access  (UDS+LDS): upper lane -> addr, lower lane -> addr+1
    //   even byte    (UDS only):            -> addr
    //   odd byte     (LDS only):            -> addr+1   (A0 was 1!)
    // Getting the odd-byte case wrong silently shifts every odd register
    // ($600005 STOP, $600015.17 timer, $60001D contrast, $70001D screen
    // enable) by one — which kills the timer interrupt and the STOP
    // low-power state the OS idles in.
    wire        wr_hi   = wr && !uds_n;
    wire        wr_lo   = wr && !lds_n;
    wire [8:0]  addr_hi = {addr, 1'b0};
    wire [8:0]  addr_lo = {addr, 1'b0} + 9'd1;

    // =========================================================================
    // I/O Bank 1 — $600000 (32 bytes, mirrored, addr & 31)
    // =========================================================================

    wire [4:0] a1 = addr_hi[4:0];
    wire [4:0] a1_lo = addr_lo[4:0];

    task io1_write(input [4:0] a, input [7:0] d);
    begin
        io1[a] <= d;
        case (a)
            5'h05: cpu_stop_pulse <= 1'b1;  // Stop OSC1 (CPU), wake per mask
            5'h0C: if (d[6] && d[5]) io1[5'h0D] <= 8'h40; // link reset
            5'h0F: io1[5'h0D][0] <= 1'b0;   // STX=0: tx register full
            5'h17: timer_load_pulse <= 1'b1; // Writing $600017 resets the timer (ports.c)
            5'h1A: ack_ai6_pulse <= 1'b1;   // Acknowledge AI6 (ON key)
            5'h1B: ack_ai2_pulse <= 1'b1;   // Acknowledge AI2 (keyboard)
            default: ;
        endcase
    end
    endtask

    // Bank 1 byte read (ports.c io_get_byte)
    reg [7:0] b1;
    always @(*) begin
        b1 = io1[a1];
        case (a1)
            5'h00: b1 = (io1[5'h00] & 8'h3B) | 8'h04; // battery good, 7:6 clear
            5'h06, 5'h07, 5'h08, 5'h09, 5'h0A, 5'h0B:
                   b1 = 8'h14;                        // unmapped
            5'h0D: b1 = (io1[5'h0C][1]) ? 8'h50 : 8'h00; // link status (v12 compute_link_status: no cable -> STX empty only when TX-empty signalling enabled; never busy)
            5'h0F: b1 = 8'h00;                        // link rx (no cable)
            5'h10, 5'h11, 5'h12, 5'h13:
                   b1 = 8'h14;                        // write-only
            5'h17: b1 = timer_value;                  // live counter
            5'h1A: b1 = (io1[5'h1A] & 8'hFD) | ({7'd0, ~on_key} << 1);
            5'h1B: b1 = kbd_col_data;
            5'h0E: b1 = 8'h14;                        // falls to default (0x14) in ref
            5'h1E, 5'h1F:
                   b1 = 8'h14;                        // unmapped
            // n-89 parity: every other bank-1 register (0x01-0x05, 0x0C,
            // 0x0F, 0x14-0x19, 0x1C, 0x1D) has an explicit case that returns
            // the stored value, so the default here must NOT force 0x14.
            default: ;
        endcase
    end

    // Same decode for the addr+1 byte of a word access
    reg [7:0] b1_lo;
    always @(*) begin
        b1_lo = io1[a1_lo];
        case (a1_lo)
            5'h00: b1_lo = (io1[5'h00] & 8'h3B) | 8'h04;
            5'h06, 5'h07, 5'h08, 5'h09, 5'h0A, 5'h0B:
                   b1_lo = 8'h14;
            5'h0D: b1_lo = (io1[5'h0C][1]) ? 8'h50 : 8'h00; // link status (v12)
            5'h0F: b1_lo = 8'h00;
            5'h10, 5'h11, 5'h12, 5'h13:
                   b1_lo = 8'h14;
            5'h17: b1_lo = timer_value;
            5'h1A: b1_lo = (io1[5'h1A] & 8'hFD) | ({7'd0, ~on_key} << 1);
            5'h1B: b1_lo = kbd_col_data;
            5'h0E: b1_lo = 8'h14;                     // ref default
            5'h1E, 5'h1F:
                   b1_lo = 8'h14;
            default: ;
        endcase
    end

    // =========================================================================
    // I/O Bank 2 — $700000 (64 bytes, addr & 63)
    // =========================================================================

    wire [5:0] a2 = addr_hi[5:0];
    wire [5:0] a2_lo = addr_lo[5:0];

    // $70001D bit 7 toggles every LCD frame (free-running status bit)
    reg [19:0] fs_div;
    reg frame_bit;

    task io2_write(input [5:0] a, input [7:0] d);
    begin
        // TiEmu ports.c io2_put_byte: while flash protection is armed,
        // writes to $700000-$70000F (RAM-execute map), $700012 and $70001F
        // are ignored ("if(tihw.protect) return;").
        if (protect && (a <= 6'h0F || a == 6'h12 || a == 6'h1F)) begin
            // dropped
        end else begin
            case (a)
                6'h12: io2[a] <= d & 8'h3F;
                default: io2[a] <= d;
            endcase
        end
    end
    endtask

    reg [7:0] b2, b2_lo;
    always @(*) begin
        b2 = io2[a2];
        if (a2 == 6'h1D)
            b2 = {frame_bit, io2[6'h1D][6:0]};
        b2_lo = io2[a2_lo];
        if (a2_lo == 6'h1D)
            b2_lo = {frame_bit, io2[6'h1D][6:0]};
    end

    always @(posedge clk) begin
        if (reset) begin
            fs_div <= 20'd0;
            frame_bit <= 1'b0;
        end else begin
            if (fs_div == 20'd749951) begin
                fs_div <= 20'd0;
                frame_bit <= ~frame_bit;
            end else begin
                fs_div <= fs_div + 20'd1;
            end
        end
    end

    wire [7:0] a3 = addr_hi[7:0];
    wire [7:0] a3_lo = addr_lo[7:0];

    // =========================================================================
    // I/O Bank 3 — $710000 (256 bytes, addr & 255) — RTC (minimal)
    // =========================================================================
    // Seconds since 1997-01-01 in $710046-$710049 (read-only counter),
    // sixteenths of a second in $710045. $71005F bit 0 enables the clock,
    // bit 1 = 0 (with bit 0 = 1) reloads the counter from $710040-$710044.

    reg [31:0] rtc_seconds;
    reg  [3:0] rtc_sixteenths;
    reg [21:0] rtc_div;     // 60 MHz / 3,750,000 = 16 Hz sixteenth tick

    wire rtc_enabled = io3[8'h5F][0];

    // Reload strobe: write of $01 to $71005F (bit0=1, bit1=0). The loading
    // registers ($710040-$710044) were written by earlier bus cycles, so
    // sampling them one cycle after the strobe is safe.
    reg rtc_load_strobe;
    always @(posedge clk) begin
        if (reset)
            rtc_load_strobe <= 1'b0;
        else begin
            rtc_load_strobe <= 1'b0;
            if (wr && bank == 2'd2 &&
                ((wr_hi && a3 == 8'h5F && (wdata[15:8] & 8'h03) == 8'h01) ||
                 (wr_lo && a3_lo == 8'h5F && (wdata[7:0] & 8'h03) == 8'h01)))
                rtc_load_strobe <= 1'b1;
        end
    end

    // Single writer of rtc_seconds / rtc_sixteenths: either load the values
    // staged in the loading registers, or free-run at 16 Hz while enabled.
    always @(posedge clk) begin
        if (reset) begin
            rtc_div        <= 22'd0;
            rtc_seconds    <= 32'd0;
            rtc_sixteenths <= 4'd0;
        end else if (rtc_load_strobe) begin
            rtc_seconds    <= {io3[8'h40], io3[8'h41],
                               io3[8'h42], io3[8'h43]};
            rtc_sixteenths <= io3[8'h44][3:0];
            rtc_div        <= 22'd0;
        end else if (rtc_enabled) begin
            if (rtc_div == 22'd3749999) begin // 60 MHz / 16 Hz
                rtc_div <= 22'd0;
                if (rtc_sixteenths == 4'd15) begin
                    rtc_sixteenths <= 4'd0;
                    rtc_seconds    <= rtc_seconds + 32'd1;
                end else begin
                    rtc_sixteenths <= rtc_sixteenths + 4'd1;
                end
            end else begin
                rtc_div <= rtc_div + 22'd1;
            end
        end
    end

    task io3_write(input [7:0] a, input [7:0] d);
    begin
        case (a)
            8'h44: io3[a] <= {4'd0, d[3:0]};
            8'h45, 8'h46, 8'h47, 8'h48, 8'h49:
                ; // read-only counting registers (live counter is read back)
            8'h5F: begin
                io3[a] <= (d & 8'h03) | 8'h80;
                if (!d[0]) begin
                    // RTC disabled: clear the loading registers
                    io3[8'h40] <= 8'd0; io3[8'h41] <= 8'd0;
                    io3[8'h42] <= 8'd0; io3[8'h43] <= 8'd0;
                end
            end
            default: io3[a] <= d;
        endcase
    end
    endtask

    reg [7:0] b3, b3_lo;
    always @(*) begin
        b3 = io3[a3];
        case (a3)
            8'h44: b3 = {4'd0, io3[8'h44][3:0]};
            8'h45: b3 = {4'd0, rtc_sixteenths};
            8'h46: b3 = rtc_seconds[31:24];
            8'h47: b3 = rtc_seconds[23:16];
            8'h48: b3 = rtc_seconds[15:8];
            8'h49: b3 = rtc_seconds[7:0];
            default: ;
        endcase

        b3_lo = io3[a3_lo];
        case (a3_lo)
            8'h44: b3_lo = {4'd0, io3[8'h44][3:0]};
            8'h45: b3_lo = {4'd0, rtc_sixteenths};
            8'h46: b3_lo = rtc_seconds[31:24];
            8'h47: b3_lo = rtc_seconds[23:16];
            8'h48: b3_lo = rtc_seconds[15:8];
            8'h49: b3_lo = rtc_seconds[7:0];
            default: ;
        endcase
    end

    // =========================================================================
    // Register write logic
    // =========================================================================

    always @(posedge clk) begin
        if (reset) begin
            integer i;
            for (i = 0; i < 32; i = i + 1)
                io1[i] <= 8'h00;
            for (i = 0; i < 64; i = i + 1)
                io2[i] <= 8'h00;
            for (i = 0; i < 256; i = i + 1)
                io3[i] <= 8'h00;

            // TiEmu hw_io_init(): HW2+ defaults
            io2[8'h1D] <= 8'h02;      // LCD screen enable
            // TiEmu hw_hwp_init(): HW2+ flash protection page limit. The
            // OS reads this back; leaving it 0 makes the OS believe the
            // protection hardware has failed.
            io2[8'h13] <= 8'h18;

            cpu_stop_pulse   <= 1'b0;
            ack_ai2_pulse    <= 1'b0;
            ack_ai6_pulse    <= 1'b0;
            timer_load_pulse <= 1'b0;
        end else begin
            cpu_stop_pulse   <= 1'b0;
            ack_ai2_pulse    <= 1'b0;
            ack_ai6_pulse    <= 1'b0;
            timer_load_pulse <= 1'b0;

            if (wr) begin
                case (bank)
                    2'd0: begin
                        if (wr_hi) io1_write(a1, wdata[15:8]);
                        if (wr_lo) io1_write(a1_lo, wdata[7:0]);
                    end
                    2'd1: begin
                        if (wr_hi) io2_write(a2, wdata[15:8]);
                        if (wr_lo) io2_write(a2_lo, wdata[7:0]);
                    end
                    2'd2: begin
                        if (wr_hi) io3_write(a3, wdata[15:8]);
                        if (wr_lo) io3_write(a3_lo, wdata[7:0]);
                    end
                    default: ;
                endcase
            end
        end
    end

    // =========================================================================
    // Read mux (combinational)
    // =========================================================================
    // Word access: {byte(addr), byte(addr+1)}.
    // Even byte access (UDS only): byte(addr) driven on both lanes.
    // Odd byte access (LDS only): the CPU samples the LOWER lane, which
    // must carry byte(addr+1) — the register at the true odd address.

    wire [7:0] rd_hi = (bank == 2'd0) ? b1    :
                       (bank == 2'd1) ? b2    : b3;
    wire [7:0] rd_lo = (bank == 2'd0) ? b1_lo :
                       (bank == 2'd1) ? b2_lo : b3_lo;

    wire word_access = !uds_n && !lds_n;

    assign rdata = word_access ? {rd_hi, rd_lo} :
                   !uds_n      ? {rd_hi, rd_hi} :   // even byte
                                 {rd_lo, rd_lo};    // odd byte

    // =========================================================================
    // Output signal assignments
    // =========================================================================

    assign kbd_row_mask = {io1[8'h18][1:0], io1[8'h19]};

    assign timer_ctrl = io1[8'h15];
    assign timer_init = io1[8'h17];
    assign stop_mask  = io1[8'h05][4:0];

    // HW2+ LCD base: $4C00 + $1000 * (io2[$17] & 3)  (ports.c case 0x17)
    assign lcd_addr  = 16'h4C00 + {io2[6'h17][1:0], 12'd0};
    assign lcd_log_w = io1[8'h12];
    assign lcd_log_h = io1[8'h13];
    // Contrast is $60001D (TiEmu ports.c); $60001C is the row-sync (RS)
    // register whose [5:2] field is used for lcd_on above.
    assign lcd_contrast = io1[8'h1D][3:0];

    // LCD active: DMA enable ($600015.0), screen enable ($70001D.1) and
    // row-sync not switched off ($60001C [5:2] != 4'b1111)
    wire lcd_rs_on = (io1[8'h1C][5:2] != 4'b1111);
    assign lcd_on = io1[8'h15][0] && io2[8'h1D][1] && lcd_rs_on;

endmodule

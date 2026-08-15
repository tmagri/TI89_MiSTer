//
// rom_loader.sv — TI-89 Titanium OS (.89u) image loader
// TI-89 MiSTer Core
//
// Loads a .89u FLASH upgrade file streamed from hps_io (WIDE=1 mode)
// into the 4MB SDRAM that backs the calculator's flash window
// ($800000-$BFFFFF).
//
// .89u format handling follows the reference simulator (v12.js
// handle_newromready):
//
//   * The file must start with the "**TIFL**" signature.
//   * The stream is scanned for the ASCII marker "basecode"; the OS
//     payload starts marker_pos + 0x3D bytes into the file.
//   * The payload is stored at SDRAM byte offset 0x12000 (word address
//     0x9000) — where the hardware expects the OS base code — as
//     big-endian words: word = {file[x], file[x+1]}.
//   * Words below 0x12000 are filled with 0x1400 and words above the
//     payload with 0xFFFF, exactly like the reference model, so reads
//     outside the image return the correct values.
//
// WIDE-mode byte order: hps_io presents the byte at the (even) address
// on ioctl_dout[7:0] and the following byte on ioctl_dout[15:8].
//
// Writes to the SDRAM are fire-and-forget strobes; the SDRAM controller
// buffers them and returns sdram_wait (wired to hps_io's ioctl_wait)
// when its input buffer nears full. The 0x1400/0xFFFF fill runs after
// the download ends, self-paced at one word every 8 clocks.
//
// rom_loaded only asserts if the marker was actually found, so a wrong
// file never starts the CPU.
//

module rom_loader (
    input         clk,
    input         reset,

    // ioctl interface from hps_io (WIDE = 1)
    input         ioctl_download,
    input  [15:0] ioctl_index,
    input         ioctl_wr,
    input  [26:0] ioctl_addr,
    input  [15:0] ioctl_dout,

    // SDRAM write interface (word addressed). Shared with flash_ctrl;
    // TI89.sv selects this module while 'loading' is high.
    output reg        sdram_wr,
    output reg [20:0] sdram_addr,
    output reg [15:0] sdram_dout,

    // Status
    output reg        rom_loaded,  // Image valid; boot may start
    output reg        loading      // Download or fill in progress
);

    // =========================================================================
    // Constants
    // =========================================================================

    localparam [63:0] MARK_TIFL  = 64'h2A2A5449464C2A2A; // "**TIFL**"
    localparam [63:0] MARK_BASE  = 64'h62617365636F6465; // "basecode"

    localparam [20:0] PAYLOAD_START = 21'h009000; // word address of byte 0x12000
    localparam [20:0] HEAD_LAST     = 21'h008FFF; // last word of the 0x1400 fill
    localparam [20:0] FLASH_LAST    = 21'h1FFFFF; // last word of the 4MB flash

    localparam [5:0]  SKIP_BYTES = 6'd53; // 0x3D - 8 ("basecode" already consumed)

    localparam [2:0] S_IDLE  = 3'd0;
    localparam [2:0] S_SCAN  = 3'd1;
    localparam [2:0] S_FILL1 = 3'd2; // 0x1400 into [0 .. 0x8FFF]
    localparam [2:0] S_FILL2 = 3'd3; // 0xFFFF into [payload_end .. 0x1FFFFF]
    localparam [2:0] S_DONE  = 3'd4;

    reg [2:0] state;

    // =========================================================================
    // Download start detection
    // =========================================================================

    reg dl_prev;
    wire dl_start = ioctl_download && !dl_prev &&
                    (ioctl_index[7:0] == 8'd0);

    always @(posedge clk) begin
        if (reset) dl_prev <= 1'b0;
        else       dl_prev <= ioctl_download;
    end

    // =========================================================================
    // Stream parsing state
    // =========================================================================

    reg [63:0] sh;        // Last 8 bytes, newest in [7:0]
    reg  [3:0] ncnt;      // Bytes consumed (saturates at 8)
    reg        tfl_ok;    // File starts with "**TIFL**"
    reg        mf;        // "basecode" marker found
    reg  [5:0] skip;      // Remaining skip bytes after the marker
    reg        pay;       // Payload phase active
    reg  [7:0] pend;      // First byte of the next payload word
    reg        hp;        // 'pend' is valid
    reg [20:0] waddr;     // Next payload word address in SDRAM

    // Fill FSM
    reg [20:0] fill_addr;   // Current fill word address
    reg [20:0] fill_start;  // First word of the 0xFFFF tail fill
    reg        found;       // Marker found in the finished download
    reg  [2:0] fdiv;        // Fill pacing divider
    wire       fill_tick = (fdiv == 3'd7);

    // =========================================================================
    // Main FSM
    // =========================================================================

    // Combinational byte-processing temporaries (blocking inside the
    // clocked block; committed to registers at the end of the cycle).
    reg [63:0] t_sh;
    reg  [3:0] t_ncnt;
    reg        t_tfl;
    reg        t_mf;
    reg  [5:0] t_skip;
    reg        t_pay;
    reg  [7:0] t_pend;
    reg        t_hp;
    reg [20:0] t_waddr;
    reg        t_wr;
    reg [15:0] t_din;
    reg [20:0] t_addr;

    integer bi;
    reg [7:0] bb;

    always @(posedge clk) begin
        if (reset) begin
            state      <= S_IDLE;
            sdram_wr   <= 1'b0;
            sdram_addr <= 21'd0;
            sdram_dout <= 16'd0;
            rom_loaded <= 1'b0;
            loading    <= 1'b0;
            sh         <= 64'd0;
            ncnt       <= 4'd0;
            tfl_ok     <= 1'b0;
            mf         <= 1'b0;
            skip       <= 6'd0;
            pay        <= 1'b0;
            pend       <= 8'd0;
            hp         <= 1'b0;
            waddr      <= PAYLOAD_START;
            fill_addr  <= 21'd0;
            fill_start <= 21'd0;
            found      <= 1'b0;
            fdiv       <= 3'd0;
        end else begin
            sdram_wr <= 1'b0;

            if (dl_start) begin
                // (Re)start: a new OS image download begins
                state      <= S_SCAN;
                loading    <= 1'b1;
                rom_loaded <= 1'b0;
                sh         <= 64'd0;
                ncnt       <= 4'd0;
                tfl_ok     <= 1'b0;
                mf         <= 1'b0;
                skip       <= 6'd0;
                pay        <= 1'b0;
                hp         <= 1'b0;
                waddr      <= PAYLOAD_START;
                found      <= 1'b0;
            end else begin
                case (state)
                    // -----------------------------------------------------
                    S_IDLE: ; // waiting for a download

                    // -----------------------------------------------------
                    S_SCAN: begin
                        // Defaults: no word processed, keep current state
                        t_sh    = sh;
                        t_ncnt  = ncnt;
                        t_tfl   = tfl_ok;
                        t_mf    = mf;
                        t_skip  = skip;
                        t_pay   = pay;
                        t_pend  = pend;
                        t_hp    = hp;
                        t_waddr = waddr;
                        t_wr    = 1'b0;
                        t_din   = 16'd0;
                        t_addr  = waddr;

                        if (ioctl_wr) begin
                            // Two file bytes arrive per strobe:
                            //   byte 0 = ioctl_dout[7:0]  (even address)
                            //   byte 1 = ioctl_dout[15:8] (odd address)
                            for (bi = 0; bi < 2; bi = bi + 1) begin
                                bb = (bi == 0) ? ioctl_dout[7:0]
                                               : ioctl_dout[15:8];

                                if (t_pay) begin
                                    if (t_hp) begin
                                        // Payload word complete
                                        t_wr    = 1'b1;
                                        t_din   = {t_pend, bb};
                                        t_addr  = t_waddr;
                                        t_waddr = t_waddr + 21'd1;
                                        t_hp    = 1'b0;
                                    end else begin
                                        t_pend = bb;
                                        t_hp   = 1'b1;
                                    end
                                end else if (t_mf) begin
                                    if (t_skip == 6'd0) begin
                                        // First payload byte
                                        t_pay  = 1'b1;
                                        t_pend = bb;
                                        t_hp   = 1'b1;
                                    end else begin
                                        t_skip = t_skip - 6'd1;
                                    end
                                end else begin
                                    // Signature / marker scan
                                    t_sh = {t_sh[55:0], bb};
                                    if ((t_ncnt == 4'd7) && (t_sh == MARK_TIFL))
                                        t_tfl = 1'b1;
                                    if (t_tfl && (t_sh == MARK_BASE)) begin
                                        t_mf   = 1'b1;
                                        t_skip = SKIP_BYTES;
                                    end
                                    if (t_ncnt < 4'd8)
                                        t_ncnt = t_ncnt + 4'd1;
                                end
                            end

                            sh    <= t_sh;
                            ncnt  <= t_ncnt;
                            tfl_ok <= t_tfl;
                            mf    <= t_mf;
                            skip  <= t_skip;
                            pay   <= t_pay;
                            pend  <= t_pend;
                            hp    <= t_hp;
                            waddr <= t_waddr;

                            if (t_wr) begin
                                sdram_wr   <= 1'b1;
                                sdram_addr <= t_addr;
                                sdram_dout <= t_din;
                            end
                        end

                        if (!ioctl_download) begin
                            // Download finished: flush a dangling byte,
                            // remember the tail fill boundary, then fill.
                            found <= t_mf;
                            if (t_hp) begin
                                sdram_wr   <= 1'b1;
                                sdram_addr <= t_waddr;
                                sdram_dout <= {t_pend, 8'hFF};
                                fill_start <= t_waddr + 21'd1;
                            end else begin
                                fill_start <= t_waddr;
                            end
                            fill_addr <= 21'd0;
                            state     <= S_FILL1;
                        end
                    end

                    // -----------------------------------------------------
                    // Fill [0 .. 0x8FFF] with 0x1400 (unmapped pattern)
                    S_FILL1: begin
                        fdiv <= fdiv + 3'd1;
                        if (fill_tick) begin
                            sdram_wr   <= 1'b1;
                            sdram_addr <= fill_addr;
                            sdram_dout <= 16'h1400;
                            if (fill_addr == HEAD_LAST) begin
                                fill_addr <= fill_start;
                                if (fill_start > FLASH_LAST) begin
                                    state      <= S_DONE;
                                    loading    <= 1'b0;
                                    rom_loaded <= found;
                                end else begin
                                    state <= S_FILL2;
                                end
                            end else begin
                                fill_addr <= fill_addr + 21'd1;
                            end
                        end
                    end

                    // -----------------------------------------------------
                    // Fill [payload_end .. end of flash] with 0xFFFF
                    S_FILL2: begin
                        fdiv <= fdiv + 3'd1;
                        if (fill_tick) begin
                            sdram_wr   <= 1'b1;
                            sdram_addr <= fill_addr;
                            sdram_dout <= 16'hFFFF;
                            if (fill_addr == FLASH_LAST) begin
                                state      <= S_DONE;
                                loading    <= 1'b0;
                                rom_loaded <= found;
                            end else begin
                                fill_addr <= fill_addr + 21'd1;
                            end
                        end
                    end

                    // -----------------------------------------------------
                    S_DONE: ; // image ready; wait for a new download

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule

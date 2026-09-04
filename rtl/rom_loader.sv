//
// rom_loader.sv — TI-89 Titanium OS (.89u) image loader
// TI-89 MiSTer Core
//
// Loads a .89u FLASH upgrade file streamed from hps_io (WIDE=1 mode)
// into the 4MB SDRAM that backs the calculator's flash window
// ($800000-$BFFFFF), building the same flash image the n-89/TiEmu
// converter builds from an upgrade file (references/n-89,
// n-89/src/convert.rs):
//
//   [0x000..0x0FF]   mirror of the first 256 bytes of the boot block
//                    (payload bytes 0x88..0x187), as in a real dump
//   [0x100..0x103]   0xFEEDBABE
//   [0x104..0x107]   HWPB pointer 0x00800108
//   [0x108..0x121]   hardware parameter block: len=24, hardware ID=9
//                    (TI-89 Titanium), revision=2, boot 1.1.1,
//                    gate array=3 (HW3)
//   [0x122..0x11FFF] 0xFFFF (erased flash)
//   [0x12000..]      the OS payload, then 0xFFFF to the end of flash
//
// .89u parsing follows the reference simulator (v12.js
// handle_newromready):
//
//   * The file must start with the "**TIFL**" signature.
//   * The stream is scanned for the ASCII marker "basecode"; the OS
//     payload starts marker_pos + 0x3D bytes into the file (for a
//     standard .89u this is file offset 0x4E, right after the TIFL
//     header, matching n-89).
//
// The mem_ctrl boot FSM then copies 128 words from flash byte 0x12088
// to RAM $000000; the first two longs there are the initial SSP and PC
// (v12.js reset_calculator).
//
// WIDE-mode byte order: hps_io presents the byte at the (even) address
// on ioctl_dout[7:0] and the following byte on ioctl_dout[15:8].
//
// Payload writes are pushed into a 4-word skid buffer the moment each
// word forms; a drain process retires them into the SDRAM port-B FIFO
// whenever b_wait is low. (The SDRAM controller silently drops a b_wr
// strobe that coincides with b_wait, so strobing directly from the
// byte stream could — and did — lose a word whenever hps_io's
// ioctl_wait backpressure lost the race.) The fill passes pace
// themselves at one word every 8 clocks and additionally stall on
// sdram_wait so no fill write is ever dropped (the SDRAM controller
// needs ~9 clocks per write).
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

    // SDRAM write-FIFO backpressure (b_wait from the SDRAM controller).
    input             sdram_wait,

    // Status
    output reg        rom_loaded,  // Image valid; boot may start
    output reg        load_failed, // Download ended without the marker
    output reg        loading      // Download or fill in progress
);

    // =========================================================================
    // Constants
    // =========================================================================

    localparam [63:0] MARK_TIFL  = 64'h2A2A5449464C2A2A; // "**TIFL**"
    localparam [63:0] MARK_BASE  = 64'h62617365636F6465; // "basecode"

    localparam [20:0] PAYLOAD_START = 21'h009000; // word address of byte 0x12000
    localparam [20:0] BOOT_FIRST    = 21'h009044; // payload word holding byte 0x88
    localparam [20:0] BOOT_LAST     = 21'h0090C3; // payload word holding byte 0x187
    localparam [20:0] BOOT_TOP      = 21'h00007F; // last mirrored boot-block word
    localparam [20:0] HDR_LAST      = 21'h000090; // last synthesized header word
    localparam [20:0] HEAD_LAST     = 21'h008FFF; // last word of the head fill
    localparam [20:0] FLASH_LAST    = 21'h1FFFFF; // last word of the 4MB flash

    localparam [5:0]  SKIP_BYTES = 6'd53; // 0x3D - 8 ("basecode" already consumed)

    localparam [2:0] S_IDLE  = 3'd0;
    localparam [2:0] S_SCAN  = 3'd1;
    localparam [2:0] S_FLUSH = 3'd2; // flush a dangling byte (sdram_wait aware)
    localparam [2:0] S_FILL1 = 3'd3; // header + 0xFFFF into [0 .. 0x8FFF]
    localparam [2:0] S_FILL2 = 3'd4; // 0xFFFF into [payload_end .. 0x1FFFFF]
    localparam [2:0] S_DONE  = 3'd5;

    reg [2:0] state;

    // =========================================================================
    // Payload-write skid buffer (see header). Depth 4 + the controller's
    // 8-deep FIFO outrun any hps_io burst; S_SCAN cannot leave for S_FLUSH
    // until the skid is empty, so the later fill passes never collide with
    // a pending payload write.
    // =========================================================================

    reg [20:0] sk_addr [0:3];
    reg [15:0] sk_data [0:3];
    reg [2:0]  sk_wp, sk_rp;
    wire [2:0] sk_cnt   = sk_wp - sk_rp;
    wire       sk_empty = (sk_cnt == 3'd0);

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
    // Boot block buffer
    // =========================================================================
    // The 128 payload words holding boot-block bytes 0x88..0x187 are
    // captured here during the download and later mirrored into SDRAM
    // words 0x000..0x07F (flash bytes 0x000..0x0FF), like a real
    // TI-89 Titanium flash dump.

    reg [15:0] boot_buf [0:127];

    // =========================================================================
    // Synthesized flash header (SDRAM words 0x80..0x90)
    // =========================================================================
    // The TI-89 Titanium OS expects the factory header an .89u upgrade
    // does not contain; n-89 (convert.rs) and v12.js both synthesize
    // it: 0xFEEDBABE at 0x100, the HWPB pointer at 0x104 and the HWPB
    // itself at 0x108.

    reg [15:0] hdr_word;

    always @(*) begin
        case (fill_addr[6:0]) // fill_addr - 0x80 for 0x80..0x90
            7'd0:    hdr_word = 16'hFEED; // 0x100: FEEDBABE
            7'd1:    hdr_word = 16'hBABE;
            7'd2:    hdr_word = 16'h0080; // 0x104: HWPB pointer -> $800108
            7'd3:    hdr_word = 16'h0108;
            7'd4:    hdr_word = 16'h0018; // 0x108: HWPB len = 24
            7'd5:    hdr_word = 16'h0000; // 0x10A: hardware ID = 9
            7'd6:    hdr_word = 16'h0009; //      (TI-89 Titanium)
            7'd7:    hdr_word = 16'h0000; // 0x10E: hardware revision = 2
            7'd8:    hdr_word = 16'h0002;
            7'd9:    hdr_word = 16'h0000; // 0x112: boot major = 1
            7'd10:   hdr_word = 16'h0001;
            7'd11:   hdr_word = 16'h0000; // 0x116: boot revision = 1
            7'd12:   hdr_word = 16'h0001;
            7'd13:   hdr_word = 16'h0000; // 0x11A: boot build = 1
            7'd14:   hdr_word = 16'h0001;
            7'd15:   hdr_word = 16'h0000; // 0x11E: gate array = 3 (HW3)
            default: hdr_word = 16'h0003; //      (fill_addr == 0x90)
        endcase
    end

    // Head-fill data mux: mirrored boot block, then synthesized header,
    // then erased flash.
    reg [15:0] fill_data;

    always @(*) begin
        if (fill_addr <= BOOT_TOP)
            fill_data = boot_buf[fill_addr[6:0]];
        else if (fill_addr <= HDR_LAST)
            fill_data = hdr_word;
        else
            fill_data = 16'hFFFF;
    end

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
            state       <= S_IDLE;
            sdram_wr    <= 1'b0;
            sdram_addr  <= 21'd0;
            sdram_dout  <= 16'd0;
            rom_loaded  <= 1'b0;
            load_failed <= 1'b0;
            loading     <= 1'b0;
            sh          <= 64'd0;
            ncnt        <= 4'd0;
            tfl_ok      <= 1'b0;
            mf          <= 1'b0;
            skip        <= 6'd0;
            pay         <= 1'b0;
            pend        <= 8'd0;
            hp          <= 1'b0;
            waddr       <= PAYLOAD_START;
            fill_addr   <= 21'd0;
            fill_start  <= 21'd0;
            found       <= 1'b0;
            fdiv        <= 3'd0;
            sk_wp       <= 3'd0;
            sk_rp       <= 3'd0;
        end else begin
            sdram_wr <= 1'b0;

            // Retire payload words into the SDRAM port-B FIFO whenever it
            // has room. The b_wr/b_wait race that silently dropped words
            // is gone: a word leaves the skid only when b_wait is low.
            if ((state == S_SCAN) && !sk_empty && !sdram_wait) begin
                sdram_wr   <= 1'b1;
                sdram_addr <= sk_addr[sk_rp[1:0]];
                sdram_dout <= sk_data[sk_rp[1:0]];
                sk_rp      <= sk_rp + 3'd1;
            end

            if (dl_start) begin
                // (Re)start: a new OS image download begins
                state       <= S_SCAN;
                loading     <= 1'b1;
                rom_loaded  <= 1'b0;
                load_failed <= 1'b0;
                sh          <= 64'd0;
                ncnt        <= 4'd0;
                tfl_ok      <= 1'b0;
                mf          <= 1'b0;
                skip        <= 6'd0;
                pay         <= 1'b0;
                hp          <= 1'b0;
                waddr       <= PAYLOAD_START;
                found       <= 1'b0;
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
                                // Completed payload word -> skid buffer;
                                // the drain process above writes it to the
                                // SDRAM FIFO when b_wait allows.
                                sk_addr[sk_wp[1:0]] <= t_addr;
                                sk_data[sk_wp[1:0]] <= t_din;
                                sk_wp               <= sk_wp + 3'd1;
                                // Capture the boot-block words for the
                                // 0x000 mirror
                                if ((t_addr >= BOOT_FIRST) &&
                                    (t_addr <= BOOT_LAST))
                                    boot_buf[t_addr[6:0] - 7'h44] <= t_din;
                            end
                        end

                        if (!ioctl_download && sk_empty) begin
                            // Download finished AND every payload word has
                            // been retired into the SDRAM FIFO: defer the
                            // dangling byte flush to S_FLUSH so it respects
                            // sdram_wait.
                            found       <= t_mf;
                            load_failed <= ~t_mf;
                            state       <= S_FLUSH;
                        end
                    end

                    // -----------------------------------------------------
                    // Flush a dangling odd byte, then start the head fill
                    S_FLUSH: begin
                        if (!sdram_wait) begin
                            if (hp) begin
                                sdram_wr   <= 1'b1;
                                sdram_addr <= waddr;
                                sdram_dout <= {pend, 8'hFF};
                                if ((waddr >= BOOT_FIRST) &&
                                    (waddr <= BOOT_LAST))
                                    boot_buf[waddr[6:0] - 7'h44] <= {pend, 8'hFF};
                                fill_start <= waddr + 21'd1;
                            end else begin
                                fill_start <= waddr;
                            end
                            fill_addr <= 21'd0;
                            state     <= S_FILL1;
                        end
                    end

                    // -----------------------------------------------------
                    // Fill [0 .. 0x8FFF]: mirrored boot block, synthesized
                    // header, then 0xFFFF (erased flash). The SDRAM
                    // controller needs ~9 clocks per write while the
                    // pacing tick fires every 8, so stall on sdram_wait
                    // or writes would be silently dropped.
                    S_FILL1: begin
                        if (!sdram_wait) fdiv <= fdiv + 3'd1;
                        if (fill_tick && !sdram_wait) begin
                            sdram_wr   <= 1'b1;
                            sdram_addr <= fill_addr;
                            sdram_dout <= fill_data;
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
                        if (!sdram_wait) fdiv <= fdiv + 3'd1;
                        if (fill_tick && !sdram_wait) begin
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

//
// savestate.sv — TI-89 MiSTer multi-slot save-state controller
// TI-89 MiSTer Core
//
// Full-system save states (NOT the old "Backup RAM" battery-RAM feature):
// captures the complete architectural state needed to resume execution at
// the exact instruction boundary it was saved at --
//   * fx68k CPU core (register file, PC, SR/CCR, and all internal
//     pipeline/sequencer/bus-control latches -- 1411 bits)
//   * io_ports (I/O banks 1 & 2: LCD config, timer control, keyboard
//     row mask, plus the LCD frame-toggle divider -- 793 bits)
//   * timer_int (free-running timer/counters and pending-interrupt
//     flags -- 50 bits)
//   * the full 256KB calculator RAM (512 x 512-byte sectors)
//
// NOT captured (documented, deliberate scope limits):
//   * io_ports' io3 RTC bank / rtc_* registers: the RTC free-runs from
//     the MiSTer HPS wall-clock timestamp and resynchronizes itself
//     within at most one HPS timestamp tick after a restore -- not
//     worth ~2100 extra state bits.
//   * keyboard.sv's internal PS/2 FIFO: entirely transient, re-derived
//     from live key input within milliseconds regardless.
//   * lcd_ctrl.sv: has no persistent state of its own -- the framebuffer
//     content lives in the 256KB RAM, which IS captured.
//
// Unlike the old Backup RAM feature, there is no auto-restore-on-boot:
// save states are always a deliberate, OSD-triggered action, in either
// direction, while the CPU is already running. This sidesteps the whole
// class of boot-FSM-interaction bugs the Backup RAM feature had.
//
// CPU pause semantics: a save/load freezes the CPU with the SAME
// mechanism as the $600005 STOP register (cpu_wrapper's `halt` input,
// gated by cpu_as_n so the bus is always idle when frozen) -- NOT with
// extReset/pwrUp. Those inputs *reset* fx68k's internal state; freezing
// via `halt` leaves every bit exactly where it was, which is the whole
// point of a save state.
//
// SD card layout (hps_io virtual block device, 512-byte sectors):
//   slot N (N = 0..NUM_SLOTS-1), sector base = N * SECTORS_PER_SLOT
//     sectors [base .. base+511]   : RAM, 256 words each (word-addressed
//                                    exactly like the old Backup RAM
//                                    sector layout)
//     sector  base+512             : CPU+IO+Timer state blob (2256 bits
//                                    = 282 bytes = 141 words, packed
//                                    into the first 141 of 256 word
//                                    slots in the sector; the rest is
//                                    unused padding)
//

module savestate (
    input             clk,
    input             reset,

    // ---- MiSTer block device interface (from / to hps_io) ----
    input             img_mounted,
    input             img_readonly,
    input      [63:0] img_size,
    output reg [31:0] sd_lba,
    output      [5:0] sd_blk_cnt,
    output reg        sd_rd,
    output reg        sd_wr,
    input             sd_ack,
    input       [7:0] sd_buff_addr,
    input      [15:0] sd_buff_dout,
    output reg [15:0] sd_buff_din,
    input             sd_buff_wr,

    // ---- OSD controls ----
    input             save_trig,     // status: "Save State" pulse
    input             load_trig,     // status: "Load State" pulse
    input       [1:0] slot_sel,      // status: active slot (0..NUM_SLOTS-1)

    // ---- CPU pause handshake (see header comment) ----
    output            want_halt,     // request cpu_halt (TI89.sv ANDs w/ cpu_as_n)
    input             is_halted,     // cpu_halt && want_halt, fed back once safe

    // ---- RAM access (mem_ctrl's lowest-priority RAM-read client, for
    // SAVE; SDRAM port B direct write, for LOAD -- identical convention
    // to the old ram_save.sv) ----
    output reg        save_req,
    output     [16:0] save_addr,
    input      [15:0] save_rdata,
    input             save_ack,

    output reg [23:0] b_addr,
    output reg [15:0] b_wdata,
    output reg        b_wr,
    input             b_wait,

    // ---- CPU + IO + Timer state bus (fx68k/cpu_wrapper + io_ports +
    // timer_int, concatenated: {cpu(1411), io(793), timer(50), 2'b00}) ----
    output reg        ssWr,
    output     [2255:0] ssDin,
    input      [2255:0] ssDout,

    // ---- status LED-style busy flag (OSD visibility only) ----
    output            busy,

    // ---- SDRAM port B mux select: high exactly while this module is
    // writing restored RAM sectors (so TI89.sv can mux the shared port B
    // between rom_loader's OS-image writes and ours) ----
    output            restoring
);

    localparam [9:0]  LAST_RAM_SECTOR    = 10'd511;
    localparam [23:0] RAM_SDRAM_WORD_BASE = 24'h200000; // matches mem_ctrl's RAM_BASE/2 (4MB point, in words)
    localparam [9:0]  SECTORS_PER_SLOT    = 10'd513;    // 512 RAM + 1 state blob
    localparam [9:0]  STATE_SECTOR        = 10'd512;     // sector index within a slot
    localparam [7:0]  STATE_WORDS         = 8'd141;     // 2256 bits / 16

    assign sd_blk_cnt = 6'd0; // 1 sector (512 bytes) per transfer

    // 256-word sector buffer, reused for both RAM sectors and the state blob
    reg [15:0] sec_buf [0:255];

    always @(posedge clk) begin
        sd_buff_din <= sec_buf[sd_buff_addr];
    end

    // =========================================================================
    // Mount tracking
    // =========================================================================
    reg ss_ena;
    reg old_ack;

    always @(posedge clk) begin
        if (reset) begin
            ss_ena <= 1'b0;
        end else begin
            if (img_mounted && !img_readonly)
                ss_ena <= 1'b1;
        end
    end

    wire load_trigger = (load_trig && ss_ena && |img_size);
    wire save_trigger = (save_trig && ss_ena);

    reg pend_load, pend_save;
    reg [1:0] active_slot;

    // =========================================================================
    // FSM
    // =========================================================================
    localparam [3:0] S_IDLE         = 4'd0;
    localparam [3:0] S_WAIT_HALT    = 4'd1; // waiting for is_halted before starting
    localparam [3:0] S_SAVE_FETCH   = 4'd2; // RAM word read -> sec_buf
    localparam [3:0] S_SAVE_WAIT    = 4'd3; // HPS reads sec_buf via sd_wr
    localparam [3:0] S_REST_REQ     = 4'd4; // issue sd_rd to HPS
    localparam [3:0] S_REST_WAIT    = 4'd5; // HPS writes into sec_buf via sd_buff_wr
    localparam [3:0] S_REST_DRAIN   = 4'd6; // sec_buf -> SDRAM port B (RAM sectors only)
    localparam [3:0] S_BLOB_PACK    = 4'd7; // ssDout -> sec_buf (save, state sector)
    localparam [3:0] S_BLOB_UNPACK  = 4'd8; // sec_buf -> ssDin + ssWr pulse (load, state sector)
    localparam [3:0] S_DONE         = 4'd9; // release want_halt

    reg [3:0] state;
    reg       loading;       // 1 = restore, 0 = save
    reg [9:0] sector_cnt;    // 0..512 (512 = state blob sector)
    reg [7:0] word_idx;      // 0..255 inside sector
    reg       fetch_busy;
    reg       d_pending;

    reg [2255:0] ssDin_r;
    assign ssDin = ssDin_r;

    assign save_addr = {sector_cnt[8:0], word_idx}; // only read while sector_cnt is a valid RAM sector (0..511)
    assign want_halt = (state != S_IDLE);
    assign busy       = (state != S_IDLE);
    assign restoring  = loading && (state != S_IDLE);

    // Absolute sector number on the SD card for the current sector_cnt
    wire [31:0] abs_sector = {22'd0, active_slot} * {22'd0, SECTORS_PER_SLOT} + {22'd0, sector_cnt};

    always @(posedge clk) begin
        if (reset) begin
            state       <= S_IDLE;
            loading     <= 1'b0;
            sector_cnt  <= 10'd0;
            word_idx    <= 8'd0;
            fetch_busy  <= 1'b0;
            d_pending   <= 1'b0;
            save_req    <= 1'b0;
            sd_rd       <= 1'b0;
            sd_wr       <= 1'b0;
            sd_lba      <= 32'd0;
            b_wr        <= 1'b0;
            b_addr      <= 24'd0;
            b_wdata     <= 16'd0;
            old_ack     <= 1'b0;
            pend_load   <= 1'b0;
            pend_save   <= 1'b0;
            ssWr        <= 1'b0;
            active_slot <= 2'd0;
            ssDin_r     <= '0;
        end else begin
            old_ack <= sd_ack;
            b_wr    <= 1'b0;
            ssWr    <= 1'b0;

            if (load_trigger) begin
                pend_load   <= 1'b1;
                active_slot <= slot_sel;
            end
            if (save_trigger) begin
                pend_save   <= 1'b1;
                active_slot <= slot_sel;
            end

            if (!old_ack && sd_ack) begin
                sd_rd <= 1'b0;
                sd_wr <= 1'b0;
            end

            case (state)
                S_IDLE: begin
                    fetch_busy <= 1'b0;
                    save_req   <= 1'b0;

                    if (pend_load) begin
                        state      <= S_WAIT_HALT;
                        loading    <= 1'b1;
                        sector_cnt <= 10'd0;
                        word_idx   <= 8'd0;
                        pend_load  <= 1'b0;
                    end else if (pend_save) begin
                        state      <= S_WAIT_HALT;
                        loading    <= 1'b0;
                        sector_cnt <= 10'd0;
                        word_idx   <= 8'd0;
                        fetch_busy <= 1'b0;
                        pend_save  <= 1'b0;
                    end
                end

                S_WAIT_HALT: begin
                    // want_halt is already asserted (state != S_IDLE); wait
                    // for the CPU to actually park at a bus-idle boundary.
                    if (is_halted) begin
                        if (loading) begin
                            state <= S_REST_REQ;
                        end else begin
                            // Latch the live CPU/IO/Timer state once, right
                            // as we freeze -- stable for the whole transfer.
                            ssDin_r <= ssDout; // reuse ssDin_r as scratch capture reg
                            state   <= S_SAVE_FETCH;
                        end
                    end
                end

                // -------------------------------------------------------------
                // SAVE PATH: SDRAM -> sec_buf -> HPS (RAM sectors), then the
                // captured state blob -> sec_buf -> HPS (one extra sector)
                // -------------------------------------------------------------
                S_SAVE_FETCH: begin
                    if (sector_cnt == STATE_SECTOR) begin
                        state <= S_BLOB_PACK;
                    end
                    else if (!fetch_busy) begin
                        save_req   <= 1'b1;
                        fetch_busy <= 1'b1;
                    end else if (save_ack) begin
                        save_req          <= 1'b0;
                        fetch_busy        <= 1'b0;
                        sec_buf[word_idx] <= save_rdata;
                        if (word_idx == 8'd255) begin
                            word_idx <= 8'd0;
                            sd_lba   <= abs_sector;
                            sd_wr    <= 1'b1;
                            state    <= S_SAVE_WAIT;
                        end else begin
                            word_idx <= word_idx + 8'd1;
                        end
                    end
                end

                S_BLOB_PACK: begin
                    // Unpack the captured 2256-bit snapshot into the first
                    // 141 words of sec_buf (word 0 = MSB chunk); the rest
                    // of the sector is left as whatever sec_buf last held
                    // (harmless padding, never read back meaningfully).
                    // Verilog-2001 style loop var (not SV inline `for (int
                    // i...)`) -- Quartus 17.0's parser rejects the latter.
                    integer pi;
                    for (pi = 0; pi < STATE_WORDS; pi = pi + 1) begin
                        sec_buf[pi] <= ssDin_r[ (2255 - pi*16) -: 16];
                    end
                    sd_lba <= abs_sector;
                    sd_wr  <= 1'b1;
                    state  <= S_SAVE_WAIT;
                end

                S_SAVE_WAIT: begin
                    if (old_ack && !sd_ack) begin
                        if (sector_cnt == STATE_SECTOR) begin
                            state <= S_DONE; // state blob was the last thing written
                        end else if (sector_cnt == LAST_RAM_SECTOR) begin
                            sector_cnt <= STATE_SECTOR;
                            state      <= S_SAVE_FETCH; // routes to S_BLOB_PACK
                        end else begin
                            sector_cnt <= sector_cnt + 10'd1;
                            word_idx   <= 8'd0;
                            state      <= S_SAVE_FETCH;
                        end
                    end
                end

                // -------------------------------------------------------------
                // RESTORE PATH: HPS -> sec_buf -> SDRAM port B (RAM sectors),
                // then HPS -> sec_buf -> ssDin_r + ssWr pulse (state blob)
                // -------------------------------------------------------------
                S_REST_REQ: begin
                    sd_lba <= abs_sector;
                    sd_rd  <= 1'b1;
                    state  <= S_REST_WAIT;
                end

                S_REST_WAIT: begin
                    if (sd_buff_wr && sd_ack) begin
                        sec_buf[sd_buff_addr] <= sd_buff_dout;
                    end
                    if (old_ack && !sd_ack) begin
                        word_idx <= 8'd0;
                        state    <= (sector_cnt == STATE_SECTOR) ? S_BLOB_UNPACK : S_REST_DRAIN;
                    end
                end

                S_REST_DRAIN: begin
                    // Identical acceptance handshake to the old ram_save.sv:
                    // a word is only consumed when b_wr && !b_wait landed in
                    // the same cycle, one cycle after being issued.
                    if (!d_pending) begin
                        if (!b_wait) begin
                            b_addr    <= RAM_SDRAM_WORD_BASE + {7'd0, sector_cnt[8:0], word_idx};
                            b_wdata   <= sec_buf[word_idx];
                            b_wr      <= 1'b1;
                            d_pending <= 1'b1;
                        end
                    end else begin
                        d_pending <= 1'b0;
                        if (!b_wait) begin
                            if (word_idx == 8'd255) begin
                                if (sector_cnt == LAST_RAM_SECTOR) begin
                                    sector_cnt <= STATE_SECTOR;
                                    state      <= S_REST_REQ;
                                end else begin
                                    sector_cnt <= sector_cnt + 10'd1;
                                    state      <= S_REST_REQ;
                                end
                            end else begin
                                word_idx <= word_idx + 8'd1;
                            end
                        end
                        // else: rejected -- loop, same word_idx re-issued
                    end
                end

                S_BLOB_UNPACK: begin
                    // Pack the first 141 words of sec_buf back into a flat
                    // 2256-bit vector, matching S_BLOB_PACK's layout exactly.
                    integer ui;
                    for (ui = 0; ui < STATE_WORDS; ui = ui + 1) begin
                        ssDin_r[ (2255 - ui*16) -: 16] <= sec_buf[ui];
                    end
                    ssWr  <= 1'b1;   // one-cycle load pulse, applies ssDin_r
                    state <= S_DONE;
                end

                S_DONE: begin
                    loading <= 1'b0;
                    state   <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

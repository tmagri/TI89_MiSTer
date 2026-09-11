//
// ram_save.sv — TI-89 MiSTer standard Backup RAM (SRM/SAV) controller
// TI-89 MiSTer Core
//
// Implements MiSTer standard battery-backed RAM save/restore (.sav file)
// using hps_io's virtual block device (sd_* interface) as seen in MegaDrive,
// SNES, and other MiSTer console cores.
//
// When an OS image (.89u) is loaded via CONF_STR "FS0,89u...", MiSTer Main:
//   1. Automatically locates or creates /media/fat/saves/TI89/<name>.sav (256 KB).
//   2. Mounts it on the secondary SD/block interface and asserts img_mounted.
//   3. On boot completion, ram_save automatically restores the 256 KB calculator
//      RAM from the .sav file through SDRAM port B while holding the CPU in reset.
//   4. When the CPU writes to RAM, ram_write_pulse sets sav_pending.
//   5. When the user opens the OSD menu (with Autosave On) or selects "Save Backup RAM",
//      ram_save reads the 256 KB RAM through mem_ctrl and writes all 512 sectors
//      back to the .sav file on the SD card.
//

module ram_save (
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

    // ---- Status / OSD controls ----
    input             bk_load,          // status[16] (Load Backup RAM)
    input             bk_save,          // status[17] (Save Backup RAM)
    input             autosave_en,      // status[13] (Autosave: On)
    input             osd_status,       // OSD menu open (OSD_STATUS)
    input             downloading,      // OS loader busy (rom_loader loading)
    input             ram_write_pulse,  // 1-cycle strobe on CPU write to RAM
    output reg        bk_ena,           // Save image is mounted and valid

    // ---- SDRAM Port B write interface (for restore: SD -> RAM) ----
    output reg [23:0] b_addr,
    output reg [15:0] b_wdata,
    output reg        b_wr,
    input             b_wait,

    // ---- mem_ctrl save read interface (for save: RAM -> SD) ----
    output reg        save_req,
    output     [16:0] save_addr,
    input      [15:0] save_rdata,
    input             save_ack,

    // ---- CPU reset control ----
    output reg        rst_req           // Hold CPU in reset while restoring RAM
);

    assign sd_blk_cnt = 6'd0; // 1 sector (512 bytes) per read/write

    // Total 256 KB RAM = 131,072 words = 512 sectors of 256 words each
    localparam [8:0] LAST_SECTOR = 9'd511;
    localparam [23:0] RAM_SDRAM_WORD_BASE = 24'h200000; // 4MB offset in 16-bit words

    // 256-word sector buffer (dual-port BRAM)
    reg [15:0] sec_buf [0:255];

    // Asynchronous read for HPS during sd_wr
    always @(posedge clk) begin
        sd_buff_din <= sec_buf[sd_buff_addr];
    end

    // =========================================================================
    // Mount and dirty tracking (matching MegaDrive)
    // =========================================================================
    reg sav_pending;
    reg old_downloading;
    reg old_osd;
    reg old_load, old_save;
    reg old_ack;

    always @(posedge clk) begin
        if (reset) begin
            bk_ena          <= 1'b0;
            sav_pending     <= 1'b0;
            old_downloading <= 1'b0;
            old_osd         <= 1'b0;
        end else begin
            old_downloading <= downloading;
            old_osd         <= osd_status;

            // Clear bk_ena when downloading starts; set when save image mounted.
            // No img_size check (same as MegaDrive): main creates a fresh
            // .sav as a 0-byte file, so requiring |img_size would deadlock
            // first-run saving. The size check belongs only on the
            // auto-restore trigger (nothing to restore from an empty file).
            if (!old_downloading && downloading)
                bk_ena <= 1'b0;
            if (img_mounted && !img_readonly)
                bk_ena <= 1'b1;

            // Mark RAM dirty when CPU writes while OSD is closed
            if (ram_write_pulse && !osd_status)
                sav_pending <= 1'b1;
            else if (state != S_IDLE && !bk_loading)
                sav_pending <= 1'b0; // Cleared once save begins
        end
    end

    // Save/Load trigger detection. Triggers are level-latched into pend_*
    // flags so a request that arrives while the FSM is busy (e.g. clicking
    // "Load Backup RAM" while the OSD-open autosave is still streaming)
    // is serviced when the FSM returns to idle instead of being lost.
    wire load_trigger = (bk_load && !old_load && bk_ena);
    wire save_trigger = (bk_save && !old_save && bk_ena) ||
                        (sav_pending && osd_status && !old_osd && autosave_en && bk_ena);

    // Automatic restore upon ROM download finish if a save file exists
    wire auto_restore_trigger = (old_downloading && !downloading && bk_ena && |img_size);

    reg pend_load, pend_save;

    // =========================================================================
    // FSM States
    // =========================================================================
    localparam S_IDLE        = 3'd0;
    localparam S_SAVE_FETCH  = 3'd1; // Read 256 words from SDRAM -> sec_buf
    localparam S_SAVE_WAIT   = 3'd2; // Wait for HPS to read sec_buf via sd_wr
    localparam S_REST_REQ    = 3'd3; // Issue sd_rd to HPS
    localparam S_REST_WAIT   = 3'd4; // HPS writes into sec_buf via sd_buff_wr
    localparam S_REST_DRAIN  = 3'd5; // Write 256 words from sec_buf -> SDRAM port B

    reg [2:0] state;
    reg       bk_loading;    // 1 = restore, 0 = save
    reg [8:0] sector_cnt;    // 0..511
    reg [7:0] word_idx;      // 0..255 inside sector
    reg       fetch_busy;    // waiting for save_ack
    reg       d_pending;     // drain word in flight, awaiting acceptance

    assign save_addr = {sector_cnt, word_idx};

    always @(posedge clk) begin
        if (reset) begin
            state       <= S_IDLE;
            bk_loading  <= 1'b0;
            sector_cnt  <= 9'd0;
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
            rst_req     <= 1'b0;
            old_load    <= 1'b0;
            old_save    <= 1'b0;
            old_ack     <= 1'b0;
            pend_load   <= 1'b0;
            pend_save   <= 1'b0;
        end else begin
            old_load <= bk_load;
            old_save <= bk_save;
            old_ack  <= sd_ack;
            b_wr     <= 1'b0;

            if (load_trigger) pend_load <= 1'b1;
            if (save_trigger) pend_save <= 1'b1;

            // Clear rd/wr pulses when HPS acknowledges
            if (!old_ack && sd_ack) begin
                sd_rd <= 1'b0;
                sd_wr <= 1'b0;
            end

            case (state)
                S_IDLE: begin
                    rst_req    <= 1'b0;
                    fetch_busy <= 1'b0;
                    save_req   <= 1'b0;

                    if (auto_restore_trigger)
                        pend_load <= 1'b1;

                    if (pend_load) begin
                        state      <= S_REST_REQ;
                        bk_loading <= 1'b1;
                        sector_cnt <= 9'd0;
                        word_idx   <= 8'd0;   // stale 255 after a prior restore
                        rst_req    <= 1'b1;   // hold CPU during restore
                        pend_load  <= 1'b0;
                    end else if (pend_save) begin
                        state      <= S_SAVE_FETCH;
                        bk_loading <= 1'b0;
                        sector_cnt <= 9'd0;
                        word_idx   <= 8'd0;
                        fetch_busy <= 1'b0;
                        pend_save  <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // SAVE PATH: SDRAM -> sec_buf -> HPS
                // -------------------------------------------------------------
                S_SAVE_FETCH: begin
                    if (!fetch_busy) begin
                        save_req   <= 1'b1;
                        fetch_busy <= 1'b1;
                    end else if (save_ack) begin
                        save_req               <= 1'b0;
                        fetch_busy             <= 1'b0;
                        sec_buf[word_idx]      <= save_rdata;
                        if (word_idx == 8'd255) begin
                            // Entire sector loaded into buffer, request HPS write
                            word_idx <= 8'd0;
                            sd_lba   <= {23'd0, sector_cnt};
                            sd_wr    <= 1'b1;
                            state    <= S_SAVE_WAIT;
                        end else begin
                            word_idx <= word_idx + 8'd1;
                        end
                    end
                end

                S_SAVE_WAIT: begin
                    // Wait for HPS to finish transferring the sector
                    if (old_ack && !sd_ack) begin
                        if (sector_cnt == LAST_SECTOR) begin
                            state <= S_IDLE; // All 256 KB saved!
                        end else begin
                            sector_cnt <= sector_cnt + 9'd1;
                            word_idx   <= 8'd0;
                            state      <= S_SAVE_FETCH;
                        end
                    end
                end

                // -------------------------------------------------------------
                // RESTORE PATH: HPS -> sec_buf -> SDRAM port B
                // -------------------------------------------------------------
                S_REST_REQ: begin
                    sd_lba <= {23'd0, sector_cnt};
                    sd_rd  <= 1'b1;
                    state  <= S_REST_WAIT;
                end

                S_REST_WAIT: begin
                    // Latch incoming words into sec_buf as HPS streams them
                    if (sd_buff_wr && sd_ack) begin
                        sec_buf[sd_buff_addr] <= sd_buff_dout;
                    end

                    // Sector transfer complete from HPS
                    if (old_ack && !sd_ack) begin
                        word_idx <= 8'd0;
                        state    <= S_REST_DRAIN;
                    end
                end

                S_REST_DRAIN: begin
                    // Push sec_buf into the SDRAM port-B FIFO with an exact
                    // acceptance handshake. The controller latches a word
                    // iff b_wr && !b_wait in the SAME cycle, one cycle after
                    // ram_save issues it — so sample b_wait on that follow-up
                    // cycle: low = accepted (advance), high = the FIFO filled
                    // and the word was rejected (re-issue the same word).
                    // Issuing back-to-back without this check silently drops
                    // every word that lands on a FIFO-full boundary.
                    if (!d_pending) begin
                        if (!b_wait) begin
                            b_addr    <= RAM_SDRAM_WORD_BASE + {7'd0, sector_cnt, word_idx};
                            b_wdata   <= sec_buf[word_idx];
                            b_wr      <= 1'b1;
                            d_pending <= 1'b1;
                        end
                    end else begin
                        d_pending <= 1'b0;
                        if (!b_wait) begin
                            if (word_idx == 8'd255) begin
                                if (sector_cnt == LAST_SECTOR) begin
                                    state      <= S_IDLE;
                                    bk_loading <= 1'b0;
                                    rst_req    <= 1'b0; // Release CPU!
                                end else begin
                                    sector_cnt <= sector_cnt + 9'd1;
                                    state      <= S_REST_REQ;
                                end
                            end else begin
                                word_idx <= word_idx + 8'd1;
                            end
                        end
                        // else: rejected — loop, d_pending cleared, same
                        // word_idx re-issued next cycle
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

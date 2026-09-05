//
// flash_ctrl.sv — TI-89 Titanium flash memory controller (Sharp WSM)
// TI-89 MiSTer Core
//
// Sits between mem_ctrl and the SDRAM controller. The 4MB flash window
// ($800000-$BFFFFF) is stored in SDRAM; this module layers the Sharp
// LH28F320BF Write State Machine on top, following the reference
// simulator's word-based model (v12.js ww_flashspecial):
//
//   Word writes:
//     while write_ready:  rom[addr] &= data; write_ready--; busy status
//     0x5050: clear status register (phase = 0x50)
//     0x9090: read identifier codes mode
//     0x1010: write setup (phase 0x50 -> write_ready = 1)
//     0x2020: block erase setup (phase 0x50 -> phase 0x20)
//     0xD0D0: block erase confirm (phase 0x20): the 64KB block
//             containing addr is filled with 0xFFFF by a background FSM
//     0xFFFF: read array / reset (phase 0x50 or 0x90): clears busy
//     anything else: ignored
//
//   Word reads:
//     ID mode:     addr&0xFFFE == 0 -> 0x00B0 (manufacturer, Sharp)
//                  addr&0xFFFE == 2 -> 0x00B5 (device, LH28F320BF)
//                  anything else    -> 0xFFFF
//     status mode: DQ7 (bit 7) = WSM busy/ready. Reports BUSY ($0000,
//                  DQ7=0) while an erase fill is in flight and READY
//                  ($0080) once it completes — like real silicon. Our fill
//                  is deferred (not instantaneous like the reference
//                  emulators), so reporting ready early let the OS read and
//                  program blocks mid-fill: corrupted decompressed data
//                  (doubled banner glyphs) and the post-decompression
//                  derail. Chip-global (not bank-scoped): the OS polls via
//                  a command/status pointer that may sit in the other
//                  2 MB half than the data pointer.
//     otherwise:   the SDRAM word
//
// Byte writes to flash are ignored (the reference model does not
// implement them; the OS programs flash with word writes). Byte reads
// work: the full word is returned and the 68000 selects its lane.
//
// Programming is a SDRAM read-modify-write (word &= data). While an
// erase fill is running, reads are answered with 0xFFFF immediately
// (no SDRAM access) and new writes are held until the fill finishes.
//

module flash_ctrl (
    input         clk,
    input         reset,

    // Interface from mem_ctrl
    input  [21:0] flash_addr,    // Byte address within the 4MB window
    input  [15:0] flash_wdata,
    input         flash_rd,      // One-cycle read strobe
    input         flash_wr,      // One-cycle write strobe
    input         flash_uds_n,
    input         flash_lds_n,
    output reg [15:0] flash_rdata,
    output reg    flash_ready,   // One-cycle completion pulse

    // SDRAM port A (request/acknowledge handshake)
    output reg [21:0] sd_addr,   // Byte address
    output reg [15:0] sd_wdata,
    output reg        sd_rd,
    output reg        sd_wr,
    output reg        sd_uds_n,
    output reg        sd_lds_n,
    input      [15:0] sd_rdata,
    input             sd_ready
);

    // =========================================================================
    // WSM state
    // =========================================================================

    reg [7:0] phase;       // Write phase (0x50 idle, 0x20 erase setup,
                           //                0xD0 erasing, 0x90 ID mode)
    reg       wready;      // Next word write programs flash
    reg       ret_or;      // 1: reads return WSM status (chip-global)

    // =========================================================================
    // Erase fill state
    // =========================================================================

    reg        erase_busy;
    reg [20:0] erase_waddr;   // Current word address (block base + count)
    reg [15:0] erase_left;    // Words remaining (0x8000 for 64KB)

    // =========================================================================
    // Main FSM
    // =========================================================================

    localparam [2:0] F_IDLE     = 3'd0;
    localparam [2:0] F_RWAIT    = 3'd1; // normal read: wait for SDRAM
    localparam [2:0] F_PRWAIT   = 3'd2; // program: read word, wait
    localparam [2:0] F_PWWAIT   = 3'd3; // program: write word, wait
    localparam [2:0] F_ERASE    = 3'd4; // erase fill: issue write
    localparam [2:0] F_EWAIT    = 3'd5; // erase fill: wait for SDRAM

    reg [2:0]  state;
    reg        req_valid;
    reg        req_rw;         // 1 = write
    reg [21:0] req_addr;
    reg [15:0] req_wdata;
    reg        req_word;       // full word access (both lanes)

    always @(posedge clk) begin
        if (reset) begin
            state       <= F_IDLE;
            phase       <= 8'h50;
            wready      <= 1'b0;
            ret_or      <= 1'b0;
            flash_rdata <= 16'd0;
            flash_ready <= 1'b0;
            sd_addr     <= 22'd0;
            sd_wdata    <= 16'd0;
            sd_rd       <= 1'b0;
            sd_wr       <= 1'b0;
            sd_uds_n    <= 1'b1;
            sd_lds_n    <= 1'b1;
            erase_busy  <= 1'b0;
            erase_waddr <= 21'd0;
            erase_left  <= 16'd0;
            req_valid   <= 1'b0;
            req_rw      <= 1'b0;
            req_addr    <= 22'd0;
            req_wdata   <= 16'd0;
            req_word    <= 1'b0;
        end else begin
            flash_ready <= 1'b0;
            sd_rd       <= 1'b0;
            sd_wr       <= 1'b0;

            case (state)
                // -----------------------------------------------------
                F_IDLE: begin
                    // Reads go first: while an erase fill is running
                    // ret_or is set, so they report ready status (0x0080)
                    // immediately and never stall behind the fill.
                    if (req_valid && !req_rw) begin
                        // ---------------- Read ----------------
                        if (phase == 8'h90) begin
                            // ID mode
                            case (req_addr[15:1])
                                15'd0:    flash_rdata <= 16'h00B0;
                                15'd1:    flash_rdata <= 16'h00B5;
                                default:  flash_rdata <= 16'hFFFF;
                            endcase
                            flash_ready <= 1'b1;
                            req_valid   <= 1'b0;
                        end else if (ret_or) begin
                            // Status register. DQ7 (bit 7) = WSM ready:
                            // 0 while an erase fill is in flight, 1 when
                            // done. REAL silicon reports busy for the whole
                            // erase time (0.7–3 s); our fill is ~0.5 ms but
                            // it is NOT instantaneous — reporting ready
                            // early let the OS read/program the block while
                            // the fill was still overwriting it (the
                            // corrupted-font garble + derail). The OS's
                            // poll loop expects to wait here, exactly like
                            // real hardware.
                            flash_rdata <= erase_busy ? 16'h0000 : 16'h0080;
                            flash_ready <= 1'b1;
                            req_valid   <= 1'b0;
                        end else begin
                            sd_addr  <= req_addr;
                            sd_rd    <= 1'b1;
                            sd_uds_n <= 1'b0;
                            sd_lds_n <= 1'b0;
                            state    <= F_RWAIT;
                        end
                    end else if (erase_busy) begin
                        // Continue the erase fill (writes stay pending)
                        state <= F_ERASE;
                    end else if (req_valid && req_rw) begin
                        // ---------------- Write ----------------
                        if (wready) begin
                            // Program: read-modify-write
                            sd_addr  <= req_addr;
                            sd_rd    <= 1'b1;
                            sd_uds_n <= 1'b0;
                            sd_lds_n <= 1'b0;
                            state    <= F_PRWAIT;
                        end else begin
                            // Command word
                            flash_ready <= 1'b1;
                            req_valid   <= 1'b0;
                            case (req_word ? req_wdata[7:0] : (req_addr[0] ? req_wdata[7:0] : req_wdata[15:8]))
                                8'h50: begin
                                              phase  <= 8'h50;
                                          end
                                8'h70: begin
                                              phase  <= 8'h70;
                                              ret_or <= 1'b1;
                                          end
                                8'h90: begin
                                              phase  <= 8'h90;
                                              ret_or <= 1'b0;
                                          end
                                8'h10, 8'h40: begin
                                              wready <= 1'b1;
                                              phase  <= req_word ? req_wdata[7:0] : (req_addr[0] ? req_wdata[7:0] : req_wdata[15:8]);
                                          end
                                8'h20: begin
                                              phase <= 8'h20;
                                          end
                                8'hD0: if (phase == 8'h20) begin
                                              phase       <= 8'hD0;
                                              ret_or      <= 1'b1;
                                              erase_busy  <= 1'b1;
                                              erase_waddr <= {req_addr[21:16], 15'd0};
                                              erase_left  <= 16'h8000;
                                          end
                                8'hFF: begin
                                              phase  <= 8'h50;
                                              wready <= 1'b0;
                                              ret_or <= 1'b0;
                                          end
                                default: ;
                            endcase
                        end
                    end
                end

                // -----------------------------------------------------
                // Normal read: wait for SDRAM data
                F_RWAIT: begin
                    if (sd_ready) begin
                        flash_rdata <= sd_rdata;
                        flash_ready <= 1'b1;
                        req_valid   <= 1'b0;
                        state       <= F_IDLE;
                    end
                end

                // -----------------------------------------------------
                // Program: word read returned; write back (word & data)
                F_PRWAIT: begin
                    if (sd_ready) begin
                        sd_addr  <= req_addr;
                        sd_wdata <= sd_rdata & (req_word ? req_wdata : (req_addr[0] ? {8'hFF, req_wdata[7:0]} : {req_wdata[15:8], 8'hFF}));
                        sd_wr    <= 1'b1;
                        sd_uds_n <= 1'b0;
                        sd_lds_n <= 1'b0;
                        state    <= F_PWWAIT;
                    end
                end

                // -----------------------------------------------------
                F_PWWAIT: begin
                    if (sd_ready) begin
                        wready      <= 1'b0;
                        ret_or      <= 1'b1;
                        flash_ready <= 1'b1;
                        req_valid   <= 1'b0;
                        state       <= F_IDLE;
                    end
                end

                // -----------------------------------------------------
                // Erase fill: one 0xFFFF word at a time
                F_ERASE: begin
                    if (erase_left == 16'd0) begin
                        erase_busy <= 1'b0;
                        state      <= F_IDLE;
                    end else begin
                        sd_addr  <= {erase_waddr, 1'b0};
                        sd_wdata <= 16'hFFFF;
                        sd_wr    <= 1'b1;
                        sd_uds_n <= 1'b0;
                        sd_lds_n <= 1'b0;
                        state    <= F_EWAIT;
                    end
                end

                // -----------------------------------------------------
                F_EWAIT: begin
                    if (sd_ready) begin
                        erase_waddr <= erase_waddr + 21'd1;
                        erase_left  <= erase_left - 16'd1;
                        state       <= F_ERASE;
                    end
                end

                default: state <= F_IDLE;
            endcase

            // Capture new requests last so a strobe arriving on a
            // completion cycle is not dropped. (mem_ctrl only strobes
            // after receiving flash_ready, so this cannot collide with
            // an outstanding request.)
            if (flash_rd || flash_wr) begin
                req_valid <= 1'b1;
                req_rw    <= flash_wr;
                req_addr  <= flash_addr;
                req_wdata <= flash_wdata;
                req_word  <= !flash_uds_n && !flash_lds_n;
            end
        end
    end

endmodule

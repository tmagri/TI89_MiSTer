//
// dbg_uart.sv — Live UART diagnostic reporter for TI-89 MiSTer Core
//
// Transmits live CPU execution status, PC, address, data, interrupt count,
// and hardware flags over the MiSTer UART interface (HPS UART /dev/ttyS1)
// at 115,200 baud (8N1) at ~4 Hz (every 250 ms).
//
// Access from MiSTer Linux over SSH:
//   stty -F /dev/ttyS1 115200 raw -echo && cat /dev/ttyS1
//
// 89u file location on SD card:
//   /media/fat/games/TI89/TI89Titanium_OS.89u
//

module dbg_uart (
    input             clk,          // 60 MHz master clock
    input             reset,

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
    input       [2:0] boot_status,

    // ---- SDRAM image dump (pre-boot read-back verify from mem_ctrl) ----
    // Streams "$P<n>\r\n" per pass, "@<6-hex word addr>\r\n" every 4096
    // words, then two raw big-endian bytes per word. dump_rdy is high
    // whenever the producer is idle; mem_ctrl pulses dump_stb/dump_word
    // only then. dump_active suppresses the periodic status lines.
    // Command dumps (host-initiated, post-boot) reuse the same framing but
    // emit "$D\r\n" as the marker line (dump_cmd_mode set by mem_ctrl).
    input             dump_active,
    input             dump_stb,
    input      [15:0] dump_word,
    input             dump_pass_stb,
    input             dump_cmd_mode,   // pass marker is "$D" not "$P<n>"
    output            dump_rdy,

    // ---- Host command interface (UART RX, bounded by construction) ----
    //   D <F|R> <6-hex start> <6-hex len>   bounded memory dump
    //     F: flash window (chip byte offset in the 4 MB SDRAM image)
    //     R: calculator RAM (byte offset in the 256 KB calc RAM)
    //     len is clamped to 2 MB and rounded down to even.
    //   T                                    emit trace rings now
    input             rxd,
    input             boot_done,
    output reg        cmd_req,          // 1-clk pulse into mem_ctrl
    output reg        cmd_mem,          // 0 = flash image, 1 = calc RAM
    output reg [23:0] cmd_start,        // byte offset
    output reg [23:0] cmd_len,          // byte length (even, clamped)
    output reg        trace_req,        // 1-clk pulse: emit trace rings

    // ---- CPU bus trace (fault diagnosis) ----
    // Records the last 16 completed bus cycles in a shift ring and
    // freezes it on the first fault after boot_done:
    //   * ai7_hit strobe (protected low-RAM write / NMI source), or
    //   * a PROGRAM fetch from the vector-table RAM ($000000-$0001FF),
    //     or from unmapped space (the two observed crash signatures).
    // When frozen, one block of 16 trace lines is emitted after the
    // next status line. Entry 0 = most recent completed cycle.
    input      [23:0] tr_addr,     // even byte address {cpu_addr,1'b0}
    input      [15:0] tr_data,     // read: cpu_din (valid at cycle end), write: cpu_dout
    input             tr_rw,       // 1 = read
    input       [2:0] tr_fc,
    input             tr_as_n,
    input             tr_ai7,      // ai7_hit one-cycle strobe
    input             tr_boot_done,

    output reg        txd
);

    // 115200 baud @ 60 MHz: 60,000,000 / 115200 = 521 clocks/bit
    localparam [9:0]  BIT_PERIOD    = 10'd520;
    localparam [23:0] REPEAT_PERIOD = 24'd15_000_000; // ~250 ms (4 lines/sec)
    localparam [6:0]  MSG_LEN       = 7'd87;   // incl. " C=xy" parser diag
    // Trace line layout (17 chars, indices 0..16):
    //   0    : line kind — F fault header, E bus-ring entry, L flash-watch
    //   1..2 : fault# (F) / ring entry (E: 00=most recent) / fl entry (L)
    //   3..8 : address, 9..12: data, 13: R/W, 14: FC (F/E) or 'F' (L),
    //   15: CR, 16: LF
    localparam [5:0]  TR_LEN        = 6'd16;
    // Dump = 1 header + 32 bus-ring lines + 64 flash-watch lines
    localparam [6:0]  TR_LINES      = 7'd97;

    localparam [1:0] S_IDLE = 2'd0;
    localparam [1:0] S_LOAD = 2'd1;
    localparam [1:0] S_SEND = 2'd2;

    reg  [1:0] state;
    reg [23:0] period_cnt;
    reg  [9:0] baud_cnt;
    reg  [3:0] bit_idx;
    reg  [6:0] char_idx;
    reg  [9:0] tx_shift;

    // Trace mode: transmitter runs the same S_LOAD/S_SEND engine, but
    // tx_byte comes from the trace mux and lines are TR_LEN+1 chars.
    // Line char 0 = fault dump index (re-armed dumps: 1..F), char 1 =
    // ring entry (0 = most recent completed cycle).
    reg        tr_active;
    reg        tr_sent;
    reg  [6:0] tr_line;
    reg  [3:0] fault_count;
    reg        tr_dump_done; // 1-clk pulse: dump finished -> ring block re-arms
    reg        tr_pend;      // sticky: host 'T' command requests a trace dump
    reg        tr_forced;    // current trace dump was host-forced (T command)

    // Faulting-cycle latch: WHAT tripped the dump and the bus cycle that
    // did it (the ring below only holds cycles that COMPLETED before the
    // fault — the faulting access itself never lands in it).
    reg [23:0] f_addr;
    reg [15:0] f_data;
    reg        f_rw;
    reg  [2:0] f_fc;
    reg        f_ai7;       // 1 = AI7 trigger, 0 = bad program fetch

    // Snapshot registers (latched when a new line begins)
    reg [23:0] snap_pc;
    reg [23:0] snap_addr;
    reg [15:0] snap_data;
    reg        snap_rw;
    reg  [2:0] snap_ipl;
    reg [15:0] snap_int_cnt;
    reg [15:0] snap_flw_cnt;
    reg        snap_lcd_on;
    reg        snap_protect;
    reg        snap_stopped;
    reg        snap_ai7;
    reg  [2:0] snap_boot_status;
    reg  [7:0] snap_cpdiag;   // parser end-state at last CR ("C=" field)

    function [7:0] hex2ascii;
        input [3:0] nib;
        begin
            hex2ascii = (nib < 4'd10) ? (8'h30 + {4'd0, nib}) : (8'h41 + {4'd0, nib - 4'd10});
        end
    endfunction

    // =========================================================================
    // Bus-cycle trace ring
    // =========================================================================

    function mapped;
        input [23:0] a;
        begin
            mapped = (a[23:18] == 6'b000000) ||   // RAM $000000-$03FFFF
                     (a[23:18] == 6'b001000) ||   // RAM mirror $200000
                     (a[23:18] == 6'b010000) ||   // RAM mirror $400000
                     (a[23:20] == 4'h6)       ||   // I/O bank 1 $6xxxxx
                     (a[23:16] == 8'h70)      ||   // I/O bank 2 $70xxxx
                     (a[23:16] == 8'h71)      ||   // I/O bank 3 $71xxxx
                     (a[23:22] == 2'b10);          // FLASH $800000-$BFFFFF
        end
    endfunction

    reg [23:0] ring_addr [0:31];
    reg [15:0] ring_data [0:31];
    reg        ring_rw  [0:31];
    reg  [2:0] ring_fc  [0:31];

    // Snapshot taken when a dump starts so the live ring can keep
    // recording the post-fault death sequence during the ~22 ms UART dump.
    reg [23:0] snap_addr_t [0:31];
    reg [15:0] snap_data_t [0:31];
    reg        snap_rw_t  [0:31];
    reg  [2:0] snap_fc_t  [0:31];

    // Flash watch ring: every COMPLETED CPU bus cycle whose address is
    // inside the flash window ($800000-$BFFFFF) — commands, programs,
    // status polls and array read-backs through the WSM. This is where
    // the OS/flash handshake divergences become visible (e.g. read-backs
    // returning WSM status instead of array data after programming).
    reg [21:0] flr_addr [0:63];
    reg [15:0] flr_data [0:63];
    reg        flr_rw   [0:63];
    reg [21:0] snap_fl_addr [0:63];
    reg [15:0] snap_fl_data [0:63];
    reg        snap_fl_rw   [0:63];

    reg        fault_latched;
    reg        record_stop;   // one clk after fault_latched: stops recording
    reg        prev_as;
    reg        tr_prog_fetch;
    reg        tr_bad_fetch;
    reg  [7:0] rec_count;     // completed cycles seen (arms the detector)
    integer k;
    always @(posedge clk) begin
        if (reset) begin
            fault_latched <= 1'b0;
            record_stop   <= 1'b0;
            prev_as       <= 1'b1;
            tr_prog_fetch <= 1'b0;
            tr_bad_fetch  <= 1'b0;
            rec_count     <= 8'd0;
            f_addr        <= 24'd0;
            f_data        <= 16'd0;
            f_rw          <= 1'b1;
            f_fc          <= 3'd0;
            f_ai7         <= 1'b0;
            for (k = 0; k < 32; k = k + 1) begin
                ring_addr[k] <= 24'd0;
                ring_data[k] <= 16'd0;
                ring_rw[k]   <= 1'b1;
                ring_fc[k]   <= 3'd0;
            end
            for (k = 0; k < 64; k = k + 1) begin
                flr_addr[k] <= 22'd0;
                flr_data[k] <= 16'd0;
                flr_rw[k]   <= 1'b1;
            end
        end else begin
            prev_as <= tr_as_n;
            record_stop <= fault_latched && !tr_active;

            // Fault conditions (evaluated while AS is low; addr/fc valid).
            // The fetch triggers arm only after 64 completed cycles: the
            // fx68k reset sequence fetches the initial PC vector from
            // RAM $000004 as a PROGRAM-space read, which must not trip
            // the vector-RAM trigger.
            tr_prog_fetch <= !tr_as_n && (tr_fc == 3'b010 || tr_fc == 3'b110);
            tr_bad_fetch  <= tr_prog_fetch &&
                             (tr_addr < 24'h000200 || !mapped(tr_addr));

            if (tr_dump_done) begin
                // Dump finished: re-arm the detector for the next fault
                fault_latched <= 1'b0;
                rec_count     <= 8'd0;
            end else if (tr_boot_done && !fault_latched &&
                (((tr_bad_fetch && !tr_as_n) && (rec_count >= 8'd64)) ||
                 tr_ai7)) begin
                // Latch WHAT tripped and the bus cycle that did it; the
                // ring below only ever shows cycles that completed
                // BEFORE the fault.
                fault_latched <= 1'b1;
                f_addr        <= tr_addr;
                f_data        <= tr_data;
                f_rw          <= tr_rw;
                f_fc          <= tr_fc;
                f_ai7         <= tr_ai7;
            end

            // Record COMPLETED cycles (AS rising: read data valid, write
            // data/address stable). The one-clk record_stop delay lets the
            // faulting cycle itself land in ring[0] if it completes; a hung
            // cycle simply never records.
            if (tr_as_n && !prev_as && !record_stop) begin
                for (k = 31; k > 0; k = k - 1) begin
                    ring_addr[k] <= ring_addr[k-1];
                    ring_data[k] <= ring_data[k-1];
                    ring_rw[k]   <= ring_rw[k-1];
                    ring_fc[k]   <= ring_fc[k-1];
                end
                ring_addr[0] <= tr_addr;
                ring_data[0] <= tr_data;
                ring_rw[0]   <= tr_rw;
                ring_fc[0]   <= tr_fc;
                if (rec_count != 8'hFF)
                    rec_count <= rec_count + 8'd1;
                // Flash watch (address logged word-aligned like the bus
                // ring: {cpu_addr,1'b0})
                if (tr_addr[23:22] == 2'b10) begin
                    for (k = 63; k > 0; k = k - 1) begin
                        flr_addr[k] <= flr_addr[k-1];
                        flr_data[k] <= flr_data[k-1];
                        flr_rw[k]   <= flr_rw[k-1];
                    end
                    flr_addr[0] <= tr_addr[21:0];
                    flr_data[0] <= tr_data;
                    flr_rw[0]   <= tr_rw;
                end
            end
        end
    end

    reg [7:0] tx_byte;

    // =========================================================================
    // UART RX — 8N1 sampler + host command parser
    // =========================================================================
    // Commands (ASCII, CR or LF terminated, case-insensitive):
    //   D <F|R> <6-hex start> <6-hex len>
    //   T
    // Issued only when the mem_ctrl boot/dump FSM is idle (B_DONE) and the
    // CPU is released (boot_done); mem_ctrl re-checks on its side.

    reg rx_ff0, rx_ff1;
    always @(posedge clk) begin
        if (reset) begin
            rx_ff0 <= 1'b1;
            rx_ff1 <= 1'b1;
        end else begin
            rx_ff0 <= rxd;
            rx_ff1 <= rx_ff0;
        end
    end

    localparam [1:0] RX_IDLE  = 2'd0;
    localparam [1:0] RX_START = 2'd1;
    localparam [1:0] RX_DATA  = 2'd2;
    localparam [1:0] RX_STOP  = 2'd3;

    reg [1:0]  rx_state;
    reg [9:0]  rx_baud;
    reg [3:0]  rx_bits;
    reg [7:0]  rx_shift;
    reg        rx_we;      // 1-clk: rx_d valid
    reg [7:0]  rx_d;

    always @(posedge clk) begin
        if (reset) begin
            rx_state <= RX_IDLE;
            rx_baud  <= 10'd0;
            rx_bits  <= 4'd0;
            rx_shift <= 8'hFF;
            rx_we    <= 1'b0;
            rx_d     <= 8'h00;
        end else begin
            rx_we <= 1'b0;
            case (rx_state)
                RX_IDLE:
                    if (!rx_ff1) begin
                        rx_state <= RX_START;
                        rx_baud  <= 10'd0;
                    end
                RX_START: begin
                    rx_baud <= rx_baud + 10'd1;
                    if (rx_baud == BIT_PERIOD[9:1]) begin
                        // mid start bit: verify still low, center the rest
                        rx_baud  <= 10'd0;
                        rx_bits  <= 4'd0;
                        rx_state <= rx_ff1 ? RX_IDLE : RX_DATA;
                    end
                end
                RX_DATA: begin
                    rx_baud <= rx_baud + 10'd1;
                    if (rx_baud == BIT_PERIOD) begin
                        rx_baud  <= 10'd0;
                        rx_shift <= {rx_ff1, rx_shift[7:1]};
                        rx_bits  <= rx_bits + 4'd1;
                        if (rx_bits == 4'd7)
                            rx_state <= RX_STOP;
                    end
                end
                RX_STOP: begin
                    rx_baud <= rx_baud + 10'd1;
                    if (rx_baud == BIT_PERIOD) begin
                        rx_d     <= rx_shift;
                        rx_we    <= 1'b1;
                        rx_state <= RX_IDLE;
                    end
                end
                default: rx_state <= RX_IDLE;
            endcase
        end
    end

    // ---- streaming command parser ----
    // Fixed-width fields keep the FSM tiny: idx 0 = 'D'/'T', 1 = ' ', 2 =
    // 'F'/'R', 3 = ' ', 4..9 = start hex, 10 = ' ', 11..16 = len hex, 17 = CR/LF.
    localparam [23:0] CMD_LEN_MAX = 24'h200000;   // 2 MB hard cap
    // Command parser state: which field we are in + accumulated values.
    // Variable-width hex (a superset of the fixed-width format): fields end
    // at ' ' / CR, so the host may send "D F 4C00 1000" or "D F 004C00 001000".
    localparam [2:0] CP_CMD = 3'd0;   // expecting 'D' or 'T'
    localparam [2:0] CP_SP1 = 3'd1;   // space after cmd
    localparam [2:0] CP_MEM = 3'd2;   // 'F' or 'R'
    localparam [2:0] CP_SP2 = 3'd3;   // space after F/R
    localparam [2:0] CP_A   = 3'd4;   // start hex digits
    localparam [2:0] CP_L   = 3'd6;   // length hex digits
    reg  [2:0]  cp_st;
    reg  [23:0] cp_start;
    reg  [23:0] cp_len;
    reg         cp_mem;     // 'R' seen
    reg         cp_bad;     // malformed line: swallow until CR/LF
    reg  [7:0]  cp_diag;    // parser end-state at last CR (status "C=" field)

    wire [7:0] rx_lc = rx_d | 8'h20;              // lowercase for letters
    wire       rx_hex_ok = ((rx_d >= "0") && (rx_d <= "9")) ||
                           ((rx_lc >= "a") && (rx_lc <= "f"));
    wire [3:0] rx_nib = (rx_d >= "0" && rx_d <= "9") ? rx_d[3:0]
                                                     : rx_lc[3:0] + 4'd9;

    task cp_issue;
        reg [23:0] n;
        reg [23:0] limit;
        begin
            limit = cp_mem ? 24'h040000 : 24'h400000;   // RAM 256K / flash 4M
            n = cp_len;
            if (n > CMD_LEN_MAX)
                n = CMD_LEN_MAX;
            if (n > limit - cp_start)                    // clamp to region end
                n = limit - cp_start;
            if (n[0])
                n = {n[23:1], 1'b0};              // even byte count
            cmd_mem    <= cp_mem;
            cmd_start  <= cp_start;
            cmd_len    <= n;
            // pulse suppressed (mem_ctrl would ignore it anyway; the host
            // simply gets no "$D" response) when out of range/zero length/
            // boot not done/a boot dump is in flight.
            cmd_req    <= !dump_active && boot_done &&
                          (n != 24'd0) && (cp_start < limit);
        end
    endtask

    always @(posedge clk) begin
        if (reset) begin
            cp_st     <= CP_CMD;
            cp_start  <= 24'd0;
            cp_len    <= 24'd0;
            cp_mem    <= 1'b0;
            cp_bad    <= 1'b0;
            cp_diag   <= 8'h00;
            cmd_req   <= 1'b0;
            cmd_mem   <= 1'b0;
            cmd_start <= 24'd0;
            cmd_len   <= 24'd0;
            trace_req <= 1'b0;
        end else begin
            cmd_req   <= 1'b0;
            trace_req <= 1'b0;
            if (rx_we) begin
                if (rx_d == 8'h0D || rx_d == 8'h0A) begin
                    // Line end: "T" (CP_SP1 with cmd 't') -> trace;
                    // complete "D ..." (CP_L, not bad) -> issue.
                    cp_diag <= {cp_bad, 1'b0, cp_st};
                    if (!cp_bad && cp_st == CP_L)
                        cp_issue;
                    else if (!cp_bad && cp_st == CP_SP1)
                        trace_req <= 1'b1;      // "T" (latched by TX FSM)
                    cp_st   <= CP_CMD;
                    cp_bad  <= 1'b0;
                end else if (!cp_bad) begin
                    case (cp_st)
                        CP_CMD: begin
                            if (rx_lc == "d" || rx_lc == "t")
                                cp_st <= CP_SP1;
                            else
                                cp_bad <= 1'b1;
                        end
                        CP_SP1: begin
                            if (rx_d == " ")
                                cp_st <= CP_MEM;
                            else
                                cp_bad <= 1'b1;
                        end
                        CP_MEM: begin
                            if (rx_lc == "f") begin
                                cp_mem <= 1'b0;
                                cp_st  <= CP_SP2;
                            end else if (rx_lc == "r") begin
                                cp_mem <= 1'b1;
                                cp_st  <= CP_SP2;
                            end else
                                cp_bad <= 1'b1;
                        end
                        CP_SP2: begin
                            if (rx_d == " ")
                                cp_st <= CP_A;
                            else
                                cp_bad <= 1'b1;
                        end
                        CP_A: begin
                            if (rx_hex_ok)
                                cp_start <= {cp_start[19:0], rx_nib};
                            else if (rx_d == " ")
                                cp_st <= CP_L;   // start field done
                            else
                                cp_bad <= 1'b1;
                        end
                        CP_L: begin
                            if (rx_hex_ok)
                                cp_len <= {cp_len[19:0], rx_nib};
                            else
                                cp_bad <= 1'b1;
                        end
                        default: cp_bad <= 1'b1;
                    endcase
                end
            end
        end
    end


    // Trace dump line mux. tr_line 0 = header (faulting cycle + trigger),
    // tr_line 1..32 = bus-ring entries 0..31 (0 = most recent completed),
    // tr_line 33..96 = flash-watch entries 0..63 (0 = most recent
    // completed CPU bus cycle in the flash window $800000-$BFFFFF).
    wire [4:0] tr_ent = tr_line[4:0] - 5'd1;   // E entry 0..31 (mod-32 wrap)
    wire [5:0] fl_ent = tr_line[5:0] - 6'd33;  // L entry 0..63 (mod-64 wrap)
    reg [23:0] ent_addr;
    reg [15:0] ent_data;
    reg        ent_rw;
    reg  [2:0] ent_fc;
    always @(*) begin
        if (tr_line == 7'd0) begin
            ent_addr = f_addr;
            ent_data = f_data;
            ent_rw   = f_rw;
            ent_fc   = f_fc;
        end else if (tr_line <= 7'd32) begin
            ent_addr = snap_addr_t[tr_ent];
            ent_data = snap_data_t[tr_ent];
            ent_rw   = snap_rw_t[tr_ent];
            ent_fc   = snap_fc_t[tr_ent];
        end else begin
            ent_addr = {2'b00, snap_fl_addr[fl_ent]};
            ent_data = snap_fl_data[fl_ent];
            ent_rw   = snap_fl_rw[fl_ent];
            ent_fc   = 3'd5;
        end
    end

    // =========================================================================
    // SDRAM image dump producer (see port comment)
    // =========================================================================
    // Feeds one byte at a time to the TX engine below: du_byte is valid
    // whenever du_state != DU_IDLE; the engine pulses du_load_pulse in
    // S_LOAD when it actually shifts the byte out, which advances the
    // producer. mem_ctrl waits on dump_rdy between words, so a word is
    // never overwritten mid-shift.

    localparam [2:0] DU_IDLE = 3'd0;
    localparam [2:0] DU_PASS = 3'd1;
    localparam [2:0] DU_SYNC = 3'd2;
    localparam [2:0] DU_HI   = 3'd3;
    localparam [2:0] DU_LO   = 3'd4;

    reg  [2:0]  du_state;
    reg  [3:0]  du_pos;        // byte index within the PASS/SYNC strings
    reg [15:0]  du_word;       // word being shifted out
    reg  [20:0] du_wc;         // words accepted this pass
    reg  [20:0] du_saddr;      // word index of the current 4096-word block
    reg  [1:0]  du_markn;      // pass number to print next
    reg  [1:0]  du_passcnt;    // pass markers seen so far
    reg         du_pass_pend;  // "$P<n>"/"$D" line queued
    reg         du_cmd;        // current pass marker is "$D" (host command)
    reg         du_load_pulse; // 1-clk: TX engine consumed du_byte
                               // (driven solely by the TX engine below)
    reg         du_send;       // TX engine is shifting a dump byte

    assign dump_rdy = (du_state == DU_IDLE);

    reg [7:0] du_byte;
    always @(*) begin
        case (du_state)
            DU_PASS: begin
                case (du_pos)
                    4'd0:    du_byte = "$";
                    4'd1:    du_byte = du_cmd ? "D" : "P";
                    4'd2:    du_byte = du_cmd ? 8'h0D :
                                         hex2ascii({2'b00, du_markn});
                    4'd3:    du_byte = du_cmd ? 8'h0A : 8'h0D;
                    default: du_byte = du_cmd ? 8'h20 : 8'h0A;  // cmd: pos==4 unused
                endcase
            end
            DU_SYNC: begin
                case (du_pos)
                    4'd0:    du_byte = "@";
                    4'd1:    du_byte = hex2ascii({3'b000, du_saddr[20]});
                    4'd2:    du_byte = hex2ascii(du_saddr[19:16]);
                    4'd3:    du_byte = hex2ascii(du_saddr[15:12]);
                    4'd4:    du_byte = hex2ascii(du_saddr[11:8]);
                    4'd5:    du_byte = hex2ascii(du_saddr[7:4]);
                    4'd6:    du_byte = hex2ascii(du_saddr[3:0]);
                    4'd7:    du_byte = 8'h0D;
                    default: du_byte = 8'h0A;   // du_pos == 8
                endcase
            end
            DU_HI:   du_byte = du_word[15:8];
            DU_LO:   du_byte = du_word[7:0];
            default: du_byte = 8'h00;
        endcase
    end

    always @(posedge clk) begin
        if (reset) begin
            du_state      <= DU_IDLE;
            du_pos        <= 4'd0;
            du_word       <= 16'd0;
            du_wc         <= 21'd0;
            du_saddr      <= 21'd0;
            du_markn      <= 2'd0;
            du_passcnt    <= 2'd0;
            du_pass_pend  <= 1'b0;
            du_cmd        <= 1'b0;
        end else begin
            if (dump_pass_stb) begin
                du_markn     <= du_passcnt;
                du_passcnt   <= du_passcnt + 2'd1;
                du_pass_pend <= 1'b1;
                du_cmd       <= dump_cmd_mode;
                du_wc        <= 21'd0;    // block syncs align per stream
                if (dump_cmd_mode)
                    du_passcnt <= du_passcnt;   // command dumps don't bump $P<n>
            end

            if (dump_stb && (du_state == DU_IDLE)) begin
                // A dump word arrived (dump_stb is combinational on
                // dump_rdy, so du_state is IDLE in the strobe cycle).
                du_word <= dump_word;
                du_wc   <= du_wc + 21'd1;
                du_pos  <= 4'd0;      // string byte index starts fresh
                if (du_wc[11:0] == 12'd0) begin
                    // Block boundary: resync line before the data bytes
                    du_saddr <= du_wc;
                    du_state <= DU_SYNC;
                end else begin
                    du_state <= DU_HI;
                end
            end else begin
                case (du_state)
                    DU_IDLE: begin
                        if (du_pass_pend) begin
                            du_pass_pend <= 1'b0;
                            du_pos       <= 4'd0;
                            du_state     <= DU_PASS;
                        end
                    end
                    DU_PASS: begin
                        if (du_load_pulse) begin
                            if (du_pos == (du_cmd ? 4'd3 : 4'd4))
                                du_state <= DU_IDLE;
                            else
                                du_pos <= du_pos + 4'd1;
                        end
                    end
                    DU_SYNC: begin
                        if (du_load_pulse) begin
                            if (du_pos == 4'd8)
                                du_state <= DU_HI;
                            else
                                du_pos <= du_pos + 4'd1;
                        end
                    end
                    DU_HI: if (du_load_pulse) du_state <= DU_LO;
                    DU_LO: if (du_load_pulse) du_state <= DU_IDLE;
                    default: du_state <= DU_IDLE;
                endcase
            end
        end
    end

    // Byte the TX engine actually loads this cycle: a dump byte while
    // du_send is set, otherwise the status/trace mux above.
    wire [7:0] tx_byte_sel = du_send ? du_byte : tx_byte;

    always @(*) begin
        if (tr_active) begin
            case (char_idx[5:0])
                6'd0:    tx_byte = (tr_line == 7'd0) ? "F" :
                                    (tr_line <= 7'd32) ? "E" : "L";
                6'd1:    tx_byte = (tr_line == 7'd0) ?
                             hex2ascii(fault_count[3:0]) :
                             (tr_line <= 7'd32) ?
                                 hex2ascii({3'b0, tr_ent[4]}) :
                                 hex2ascii({2'b0, fl_ent[5]});
                6'd2:    tx_byte = (tr_line == 7'd0) ?
                             (f_ai7 ? "a" : "f") :
                             (tr_line <= 7'd32) ?
                                 hex2ascii(tr_ent[3:0]) :
                                 hex2ascii(fl_ent[4:0]);
                6'd3:    tx_byte = hex2ascii(ent_addr[23:20]);
                6'd4:    tx_byte = hex2ascii(ent_addr[19:16]);
                6'd5:    tx_byte = hex2ascii(ent_addr[15:12]);
                6'd6:    tx_byte = hex2ascii(ent_addr[11:8]);
                6'd7:    tx_byte = hex2ascii(ent_addr[7:4]);
                6'd8:    tx_byte = hex2ascii(ent_addr[3:0]);
                6'd9:    tx_byte = hex2ascii(ent_data[15:12]);
                6'd10:   tx_byte = hex2ascii(ent_data[11:8]);
                6'd11:   tx_byte = hex2ascii(ent_data[7:4]);
                6'd12:   tx_byte = hex2ascii(ent_data[3:0]);
                6'd13:   tx_byte = ent_rw ? "R" : "W";
                6'd14:   tx_byte = (tr_line > 7'd32) ? "F" :
                                    hex2ascii({1'b0, ent_fc[2:0]});
                6'd15:   tx_byte = 8'h0D;
                default: tx_byte = 8'h0A;
            endcase
        end else begin
        case (char_idx)
            7'd0:  tx_byte = "[";
            7'd1:  tx_byte = "T";
            7'd2:  tx_byte = "I";
            7'd3:  tx_byte = "8";
            7'd4:  tx_byte = "9";
            7'd5:  tx_byte = "]";
            7'd6:  tx_byte = " ";
            7'd7:  tx_byte = "P";
            7'd8:  tx_byte = "C";
            7'd9:  tx_byte = "=";
            7'd10: tx_byte = hex2ascii(snap_pc[23:20]);
            7'd11: tx_byte = hex2ascii(snap_pc[19:16]);
            7'd12: tx_byte = hex2ascii(snap_pc[15:12]);
            7'd13: tx_byte = hex2ascii(snap_pc[11:8]);
            7'd14: tx_byte = hex2ascii(snap_pc[7:4]);
            7'd15: tx_byte = hex2ascii(snap_pc[3:0]);
            7'd16: tx_byte = " ";
            7'd17: tx_byte = "A";
            7'd18: tx_byte = "=";
            7'd19: tx_byte = hex2ascii(snap_addr[23:20]);
            7'd20: tx_byte = hex2ascii(snap_addr[19:16]);
            7'd21: tx_byte = hex2ascii(snap_addr[15:12]);
            7'd22: tx_byte = hex2ascii(snap_addr[11:8]);
            7'd23: tx_byte = hex2ascii(snap_addr[7:4]);
            7'd24: tx_byte = hex2ascii(snap_addr[3:0]);
            7'd25: tx_byte = " ";
            7'd26: tx_byte = "D";
            7'd27: tx_byte = "=";
            7'd28: tx_byte = hex2ascii(snap_data[15:12]);
            7'd29: tx_byte = hex2ascii(snap_data[11:8]);
            7'd30: tx_byte = hex2ascii(snap_data[7:4]);
            7'd31: tx_byte = hex2ascii(snap_data[3:0]);
            7'd32: tx_byte = " ";
            7'd33: tx_byte = snap_rw ? "R" : "W";
            7'd34: tx_byte = snap_rw ? "D" : "R";
            7'd35: tx_byte = " ";
            7'd36: tx_byte = "I";
            7'd37: tx_byte = "P";
            7'd38: tx_byte = "L";
            7'd39: tx_byte = "=";
            7'd40: tx_byte = hex2ascii({1'b0, snap_ipl});
            7'd41: tx_byte = " ";
            7'd42: tx_byte = "I";
            7'd43: tx_byte = "N";
            7'd44: tx_byte = "T";
            7'd45: tx_byte = "=";
            7'd46: tx_byte = hex2ascii(snap_int_cnt[15:12]);
            7'd47: tx_byte = hex2ascii(snap_int_cnt[11:8]);
            7'd48: tx_byte = hex2ascii(snap_int_cnt[7:4]);
            7'd49: tx_byte = hex2ascii(snap_int_cnt[3:0]);
            7'd50: tx_byte = " ";
            7'd51: tx_byte = "F";
            7'd52: tx_byte = "L";
            7'd53: tx_byte = "W";
            7'd54: tx_byte = "=";
            7'd55: tx_byte = hex2ascii(snap_flw_cnt[15:12]);
            7'd56: tx_byte = hex2ascii(snap_flw_cnt[11:8]);
            7'd57: tx_byte = hex2ascii(snap_flw_cnt[7:4]);
            7'd58: tx_byte = hex2ascii(snap_flw_cnt[3:0]);
            7'd59: tx_byte = " ";
            7'd60: tx_byte = "L";
            7'd61: tx_byte = "=";
            7'd62: tx_byte = snap_lcd_on ? "1" : "0";
            7'd63: tx_byte = " ";
            7'd64: tx_byte = "P";
            7'd65: tx_byte = "=";
            7'd66: tx_byte = snap_protect ? "1" : "0";
            7'd67: tx_byte = " ";
            7'd68: tx_byte = "S";
            7'd69: tx_byte = "=";
            7'd70: tx_byte = snap_stopped ? "1" : "0";
            7'd71: tx_byte = " ";
            7'd72: tx_byte = "7";
            7'd73: tx_byte = "=";
            7'd74: tx_byte = snap_ai7 ? "1" : "0";
            7'd75: tx_byte = " ";
            7'd76: tx_byte = "S";
            7'd77: tx_byte = "T";
            7'd78: tx_byte = "=";
            7'd79: tx_byte = hex2ascii({1'b0, snap_boot_status});
            7'd80: tx_byte = " ";
            7'd81: tx_byte = "C";
            7'd82: tx_byte = "=";
            7'd83: tx_byte = hex2ascii(snap_cpdiag[7:4]);
            7'd84: tx_byte = hex2ascii(snap_cpdiag[3:0]);
            7'd85: tx_byte = 8'h0D; // '\r'
            7'd86: tx_byte = 8'h0A; // '\n'
            default: tx_byte = 8'h20;
        endcase
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            state            <= S_IDLE;
            period_cnt       <= 24'd0;
            baud_cnt         <= 10'd0;
            bit_idx          <= 4'd0;
            char_idx         <= 7'd0;
            tx_shift         <= 10'h3FF;
            txd              <= 1'b1;
            tr_active        <= 1'b0;
            tr_sent          <= 1'b0;
            tr_pend          <= 1'b0;
            tr_forced        <= 1'b0;
            tr_dump_done     <= 1'b0;
            tr_line          <= 7'd0;
            snap_pc          <= 24'd0;
            snap_addr        <= 24'd0;
            snap_data        <= 16'd0;
            snap_rw          <= 1'b1;
            snap_ipl         <= 3'd0;
            snap_int_cnt     <= 16'd0;
            snap_flw_cnt     <= 16'd0;
            snap_lcd_on      <= 1'b0;
            snap_protect     <= 1'b0;
            snap_stopped     <= 1'b0;
            snap_ai7         <= 1'b0;
            snap_boot_status <= 3'd0;
            snap_cpdiag      <= 8'h00;
            du_send          <= 1'b0;
            du_load_pulse    <= 1'b0;
        end else begin
            tr_dump_done <= 1'b0;
            du_load_pulse <= 1'b0;
            // sticky latch of the host 'T' pulse (single driver for tr_pend);
            // ignored while a trace dump is already active.
            if (trace_req && !tr_active && !tr_pend)
                tr_pend <= 1'b1;
            case (state)
                S_IDLE: begin
                    txd <= 1'b1;
                    if (tr_pend || (fault_latched && !tr_sent)) begin
                        // Trace dump: header + snapshot of the frozen
                        // ring, then let the live ring resume recording.
                        // A host 'T' command forces the same emission on
                        // demand (tr_forced: no fault-count side effects).
                        for (k = 0; k < 32; k = k + 1) begin
                            snap_addr_t[k] <= ring_addr[k];
                            snap_data_t[k] <= ring_data[k];
                            snap_rw_t[k]   <= ring_rw[k];
                            snap_fc_t[k]   <= ring_fc[k];
                        end
                        for (k = 0; k < 64; k = k + 1) begin
                            snap_fl_addr[k] <= flr_addr[k];
                            snap_fl_data[k] <= flr_data[k];
                            snap_fl_rw[k]   <= flr_rw[k];
                        end
                        tr_forced <= tr_pend && !(fault_latched && !tr_sent);
                        tr_pend   <= 1'b0;
                        // NB: tr_sent is NOT touched here. Clearing it would
                        // let a permanently-faulting CPU (derailed fetches
                        // re-latch fault_latched instantly) restart the
                        // trace flood forever and starve the du engine.
                        tr_active <= 1'b1;
                        tr_line   <= 7'd0;   // line 0 = header
                        char_idx  <= 7'd0;
                        state     <= S_LOAD;
                    end else if (du_state != DU_IDLE) begin
                        // Dump byte waiting: send it. (Fault traces keep
                        // priority above; they cannot fire while the CPU
                        // is held in reset during the dump.)
                        du_send <= 1'b1;
                        state   <= S_LOAD;
                    end else if (!dump_active && period_cnt >= REPEAT_PERIOD) begin
                        period_cnt       <= 24'd0;
                        tr_active        <= 1'b0;
                        snap_pc          <= dbg_pc;
                        snap_addr        <= dbg_addr;
                        snap_data        <= dbg_data;
                        snap_rw          <= dbg_rw;
                        snap_ipl         <= dbg_ipl;
                        snap_int_cnt     <= dbg_int_cnt;
                        snap_flw_cnt     <= dbg_flw_cnt;
                        snap_lcd_on      <= dbg_lcd_on;
                        snap_protect     <= dbg_protect;
                        snap_stopped     <= dbg_stopped;
                        snap_ai7         <= dbg_ai7;
                        snap_boot_status <= boot_status;
                        snap_cpdiag      <= cp_diag;
                        char_idx         <= 7'd0;
                        state            <= S_LOAD;
                    end else begin
                        period_cnt <= period_cnt + 24'd1;
                    end
                end

                S_LOAD: begin
                    // Load byte into shift register with start bit (0) and stop bit (1)
                    if (du_send)
                        du_load_pulse <= 1'b1;
                    tx_shift <= {1'b1, tx_byte_sel, 1'b0};
                    bit_idx  <= 4'd0;
                    baud_cnt <= 10'd0;
                    state    <= S_SEND;
                end

                S_SEND: begin
                    txd <= tx_shift[0];
                    if (baud_cnt >= BIT_PERIOD) begin
                        baud_cnt <= 10'd0;
                        tx_shift <= {1'b1, tx_shift[9:1]};
                        if (bit_idx == 4'd9) begin
                            // Byte completed
                            if (du_send) begin
                                du_send <= 1'b0;
                                state   <= S_IDLE;
                            end else if (tr_active) begin
                                if (char_idx[5:0] == TR_LEN) begin
                                    if (tr_line == TR_LINES) begin
                                        tr_active <= 1'b0;
                                        // Re-arm: allow the NEXT fault (the
                                        // AI7 aftermath / NMI redirect) to
                                        // dump again once some new cycles
                                        // have been recorded. Host-forced
                                        // dumps leave fault bookkeeping alone.
                                        if (!tr_forced) begin
                                            if (fault_count != 4'hF)
                                                fault_count <= fault_count + 4'd1;
                                            if (fault_count == 4'hE)
                                                tr_sent <= 1'b1; // cap: 15 dumps
                                        end
                                        tr_forced     <= 1'b0;
                                        tr_dump_done  <= 1'b1;   // ring block re-arms
                                        state         <= S_IDLE;
                                    end else begin
                                        tr_line   <= tr_line + 7'd1;
                                        char_idx  <= 7'd0;
                                        state     <= S_LOAD;
                                    end
                                end else begin
                                    char_idx <= char_idx + 7'd1;
                                    state    <= S_LOAD;
                                end
                            end else if (char_idx == MSG_LEN - 7'd1) begin
                                state <= S_IDLE;
                            end else begin
                                char_idx <= char_idx + 7'd1;
                                state    <= S_LOAD;
                            end
                        end else begin
                            bit_idx <= bit_idx + 4'd1;
                        end
                    end else begin
                        baud_cnt <= baud_cnt + 10'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

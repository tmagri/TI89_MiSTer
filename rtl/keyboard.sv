//
// keyboard.sv — TI-89 Keyboard Matrix Controller
// TI-89 MiSTer Core
//
// Implements the 10x8 keyboard scan matrix from tiemu kbd.c (keyRow89)
// and maps PS/2 scancodes from MiSTer hps_io to TI key positions.
//
// Matrix layout (from tiemu/src/core/ti_hw/kbd.c):
//   Row 0: ALPHA,   DIAMOND, SHIFT,   2ND,     RIGHT,   DOWN,    LEFT,    UP
//   Row 1: F5,      CLEAR,   POWER,   DIVIDE,  MULTIPLY,MINUS,   PLUS,    ENTER
//   Row 2: F4,      BACKSP,  T,       COMMA,   9,       6,       3,       NEGATE
//   Row 3: F3,      CATALOG, Z,       PARIGHT, 8,       5,       2,       PERIOD
//   Row 4: F2,      MODE,    Y,       PALEFT,  7,       4,       1,       0
//   Row 5: F1,      HOME,    X,       EQUALS,  PIPE,    EE,      STORE,   APPS
//   Row 6: (void except col 7 = ESC)
//   Rows 7-9: void
//
// Two keyboard modes are supported:
//
//   Legacy (Mapped) Mode (native_mode = 0):
//     Direct physical mapping. PC Shift -> TI SHIFT, PC Ctrl -> TI DIAMOND.
//     Letters use ALPHA auto-assert with a reference counter.
//
//   Natural Mode (native_mode = 1):
//     True Logical Translation (Reverse Mapping). PC Shift is decoupled
//     from TI SHIFT and tracked internally in pc_shift_down. A comprehensive
//     scancode decoder uses {ps2_ext, pc_shift_down, ps2_code} to determine
//     the target row/col and a 4-bit required modifier mask.
//     Keys are routed through a 16-deep event FIFO and a non-blocking
//     sequencer that injects the required modifier bit (if any) and waits
//     for it to settle before the key itself is pressed/released.
//
//     IMPORTANT (verified against tiemu's chars_to_keys()/tichars.c, the
//     reference this core follows, and confirmed on real hardware): for the
//     shared letter keys A-W, ALPHA and SHIFT are ALTERNATIVES, never held
//     together. Unshifted = ALPHA + key (lowercase). Shift+letter = SHIFT +
//     key, with ALPHA *not* asserted at all (uppercase) -- see tichars.c's
//     ALPHA() macro: `if (!shift) emit ALPHA；` the shift flag, when set,
//     suppresses ALPHA entirely for that keystroke. Asserting ALPHA+SHIFT
//     together (the original design here) silently produces lowercase on
//     real hardware, since the OS has no "capital letter while in alpha
//     mode" state at all -- SHIFT alone is what selects the capital.
//     T/X/Y/Z need only SHIFT when capitalized because they're dedicated
//     keys that never need ALPHA in the first place, consistent with this.
//
// Manual Override Keys (both modes, bypass FIFO/sequencer entirely):
//   PC Left/Right Ctrl  -> TI DIAMOND (row 0, col 1)
//   PC Left/Right Alt   -> TI 2ND     (row 0, col 3)
//   PC Windows/GUI      -> TI SHIFT   (row 0, col 2)
//   PC Caps Lock        -> TI ALPHA   (row 0, col 0)
//
// Numpad (Num Lock on): 0-9 . + - * / Enter map to their TI-89 equivalents.
// With Num Lock off the numpad sends E0-prefixed codes and acts as the
// cursor/navigation keys in the extended table above. The np_digits input
// (OSD "Numpad Mode: Digits") re-decodes those E0 codes as keypad digits.
//

module keyboard (
    input         clk,
    input         reset,

    // PS/2 keyboard from hps_io
    input  [10:0] ps2_key,      // {toggle, pressed, extended, scancode[7:0]}

    // Matrix scan interface (directly called by I/O ports)
    input   [9:0] row_mask,     // Row selection mask (from $600018-$600019)
    output  [7:0] col_data,     // Column data (active low -- returned at $60001B)

    // ON key output (directly triggers AI6)
    output reg    on_key,       // ON key state (active high)
    output reg    on_key_press, // ON key edge (1 cycle pulse on press)

    // Keyboard interrupt output
    output reg    kbd_int,      // Key state change (triggers AI2)

    // Native Keyboard Mode (from OSD status bit)
    input         native_mode,  // 0 = Legacy, 1 = Native (sequenced modifiers)

    // Numpad Digits Mode (from OSD status bit). MiSTer never sets the Num Lock
    // LED on USB keyboards, so many numpads are stuck in cursor mode and send
    // E0-prefixed navigation codes instead of digits. When asserted, those E0
    // codes are decoded as keypad digits 1-9 (see extended table).
    input         np_digits
);

    // =========================================================================
    // Key state storage
    // =========================================================================
    reg [7:0] key_matrix [0:9]; // 10 rows x 8 columns

    // =========================================================================
    // Key Tracker (Fixes Shift-Release Desync Latching)
    // =========================================================================
    reg [10:0] key_tracker [0:511]; // Stores {dec_req[3:0], dec_row[3:0], dec_col[2:0]}

    // =========================================================================
    // Native Mode: per-scancode "last requested state" (Fixes dropped
    // press/release races). The old duplicate/auto-repeat filter compared
    // against key_matrix, which only reflects what the sequencer has
    // actually COMMITTED -- but a single key's press+release round trip
    // through the sequencer takes many clock cycles (up to several
    // TIMER_MAX waits for multi-modifier keys). Under normal overlapping
    // typing (NKRO) a key's release event can arrive before its own press
    // has been committed to key_matrix; comparing against key_matrix then
    // makes the release look like a no-op "duplicate" and silently drops
    // it, leaving that key (and any modifier it needed) stuck asserted
    // forever -- which then corrupts every subsequent keystroke. This
    // array instead remembers the last press/release state WE decided to
    // act on for each scancode, independent of commit latency.
    // =========================================================================
    reg key_native_pending [0:511];

    // =========================================================================
    // PC Shift tracking (Natural Mode: decoupled from TI SHIFT matrix key)
    // =========================================================================
    reg pc_lshift;
    reg pc_rshift;
    wire pc_shift_down = pc_lshift | pc_rshift;

    reg [4:0] alpha_count;   // Legacy Mode alpha reference counter

    // =========================================================================
    // Natural Mode: Event FIFO (16-deep circular buffer for N-key rollover)
    // Entry format (14 bits):
    //   [13]    press       (1 = key press, 0 = key release)
    //   [12:9]  row         (4-bit TI matrix row)
    //   [8:6]   col         (3-bit TI matrix column)
    //   [5:2]   req_mask    {req_alpha, req_shift, req_2nd, req_diamond}
    // =========================================================================

    localparam FIFO_DEPTH = 16;
    localparam [3:0] FIFO_AW = 4'd4;
    localparam [4:0] FIFO_CW = 5'd5;

    reg [13:0] fifo_mem [0:FIFO_DEPTH-1];
    reg [FIFO_AW-1:0] fifo_wr_ptr;
    reg [FIFO_AW-1:0] fifo_rd_ptr;
    reg [FIFO_CW-1:0] fifo_count;

    wire [FIFO_CW-1:0] fifo_full  = (fifo_count == FIFO_DEPTH[FIFO_CW-1:0]);
    wire               fifo_empty = (fifo_count == 5'd0);

    wire [13:0] fifo_data  = fifo_mem[fifo_rd_ptr];
    wire        fifo_press = fifo_data[13];
    wire [3:0]  fifo_row   = fifo_data[12:9];
    wire [2:0]  fifo_col   = fifo_data[8:6];
    wire [3:0]  fifo_req   = fifo_data[5:2];

    // =========================================================================
    // Natural Mode: Non-blocking sequencer state machine
    //
    // Modifiers required by a keystroke (ALPHA/SHIFT/2ND/DIAMOND) are
    // asserted and released ONE BIT AT A TIME (never atomically together),
    // each with its own settle wait, in fixed priority order
    // ALPHA -> SHIFT -> 2ND -> DIAMOND on press, and the reverse on release.
    // This mirrors how a key would naturally be pressed by hand and avoids
    // relying on the OS scan loop observing two simultaneous matrix changes
    // in the same scan cycle (which is what made Shift+<letter> silently
    // fail for every letter except T/X/Y/Z, since those are the only
    // letters that ever need just a single modifier bit).
    // =========================================================================
    localparam [2:0] ST_IDLE        = 3'd0;
    localparam [2:0] ST_MOD_ASSERT  = 3'd1; // assert next pending modifier bit
    localparam [2:0] ST_MOD_WAIT    = 3'd2; // settle wait after an assert
    localparam [2:0] ST_KEY_ASSERT  = 3'd3;
    localparam [2:0] ST_KEY_RELEASE = 3'd4;
    localparam [2:0] ST_MOD_RELEASE = 3'd5; // release next pending modifier bit
    localparam [2:0] ST_REL_WAIT    = 3'd6; // settle wait after a release

    reg [2:0]  nm_state;
    reg [17:0] nm_timer;
    reg [3:0]  nm_row;
    reg [2:0]  nm_col;
    reg [3:0]  nm_req;
    reg [3:0]  nm_pending; // modifier bits {ALPHA,SHIFT,2ND,DIAMOND} still to
                            // be asserted/released one at a time for the
                            // key currently in flight

    reg [4:0]  nm_alpha_count;
    reg [4:0]  nm_shift_count;
    reg [4:0]  nm_2nd_count;
    reg [4:0]  nm_diamond_count;

    localparam [17:0] TIMER_MAX = 18'd160000; // ~2.67 ms @ 60 MHz (clk_sys)

    // =========================================================================
    // PS/2 signal extraction
    // =========================================================================
    wire       ps2_strobe  = ps2_key[10];
    wire       ps2_pressed = ps2_key[9];
    wire       ps2_ext     = ps2_key[8];
    wire [7:0] ps2_code    = ps2_key[7:0];

    reg ps2_strobe_prev;
    reg win_prefix;

    always @(posedge clk) begin
        if (reset) begin
            integer i;
            for (i = 0; i < 10; i = i + 1) key_matrix[i] <= 8'h00;
            for (i = 0; i < 512; i = i + 1) key_tracker[i] <= 11'd0;
            for (i = 0; i < 512; i = i + 1) key_native_pending[i] <= 1'b0;
            on_key           <= 1'b0;
            on_key_press     <= 1'b0;
            kbd_int          <= 1'b0;
            ps2_strobe_prev  <= 1'b0;
            alpha_count      <= 5'd0;
            pc_lshift        <= 1'b0;
            pc_rshift        <= 1'b0;
            win_prefix       <= 1'b0;
            nm_state         <= ST_IDLE;
            nm_timer         <= 18'd0;
            nm_row           <= 4'd0;
            nm_col           <= 3'd0;
            nm_req           <= 4'd0;
            nm_pending       <= 4'd0;
            nm_alpha_count   <= 5'd0;
            nm_shift_count   <= 5'd0;
            nm_2nd_count     <= 5'd0;
            nm_diamond_count <= 5'd0;
            fifo_wr_ptr      <= '0;
            fifo_rd_ptr      <= '0;
            fifo_count       <= '0;
        end else begin
            on_key_press <= 1'b0;
            kbd_int      <= 1'b0;
            ps2_strobe_prev <= ps2_strobe;

            if (ps2_strobe != ps2_strobe_prev) begin
                reg [3:0] dec_row;
                reg [2:0] dec_col;
                reg [3:0] dec_req; // {alpha(3), shift(2), 2nd(1), diamond(0)}
                reg       dec_valid;
                reg       dec_is_on;
                reg       dec_is_mod_override;
                reg       legacy_is_alpha; // Legacy mode auto-alpha flag

                dec_row             = 4'd0;
                dec_col             = 3'd0;
                dec_req             = 4'd0;
                dec_valid           = 1'b0;
                dec_is_on           = 1'b0;
                dec_is_mod_override = 1'b0;
                legacy_is_alpha     = 1'b0;

                // ==============================================================
                // MANUAL OVERRIDE KEYS & SHIFT TRACKING
                // ==============================================================
                case ({ps2_ext, ps2_code})
                    9'h012: begin // Left Shift
                        if (native_mode) pc_lshift <= ps2_pressed;
                        else begin key_matrix[0][5] <= ps2_pressed; kbd_int <= 1'b1; end // SHIFT = row0 col2 -> bit(7-2)=5
                        dec_valid = 1'b1; dec_is_mod_override = 1'b1;
                    end
                    9'h059: begin // Right Shift
                        if (native_mode) pc_rshift <= ps2_pressed;
                        else begin key_matrix[0][5] <= ps2_pressed; kbd_int <= 1'b1; end // SHIFT = row0 col2 -> bit(7-2)=5
                        dec_valid = 1'b1; dec_is_mod_override = 1'b1;
                    end
                    9'h014, 9'h114: begin // L/R Ctrl -> TI DIAMOND (row0 col1 -> bit6), both modes
                        key_matrix[0][6] <= ps2_pressed; kbd_int <= 1'b1;
                        dec_valid = 1'b1; dec_is_mod_override = 1'b1;
                    end
                    9'h011, 9'h111: begin // L/R Alt -> TI 2ND (row0 col3 -> bit4), both modes
                        key_matrix[0][4] <= ps2_pressed; kbd_int <= 1'b1;
                        dec_valid = 1'b1; dec_is_mod_override = 1'b1;
                    end
                    9'h11F, 9'h127, 9'h15B, 9'h15C: begin // Win/GUI -> TI SHIFT (row0 col2 -> bit5), both modes
                        key_matrix[0][5] <= ps2_pressed; kbd_int <= 1'b1;
                        dec_valid = 1'b1; dec_is_mod_override = 1'b1;
                    end
                    9'h058: begin // Caps Lock -> TI ALPHA (row0 col0 -> bit7), both modes
                        key_matrix[0][7] <= ps2_pressed; kbd_int <= 1'b1;
                        dec_valid = 1'b1; dec_is_mod_override = 1'b1;
                    end
                    default: ;
                endcase

                // ==============================================================
                // TRUE LOGICAL TRANSLATION (Non-Extended Scancodes)
                // ==============================================================
                if (!ps2_ext && !dec_is_mod_override) begin
                    case (ps2_code)
                        // Letters T, X, Y, Z (No secondary mapping)
                        8'h2C: begin dec_row=4'd2; dec_col=3'd2; dec_valid=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b0000; end // T
                        8'h22: begin dec_row=4'd5; dec_col=3'd2; dec_valid=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b0000; end // X
                        8'h35: begin dec_row=4'd4; dec_col=3'd2; dec_valid=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b0000; end // Y
                        8'h1A: begin dec_row=4'd3; dec_col=3'd2; dec_valid=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b0000; end // Z

                        // Standard Letters A-W
                        8'h1C: begin dec_row=4'd5; dec_col=3'd3; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // A (EQ)
                        8'h32: begin dec_row=4'd4; dec_col=3'd3; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // B (PALEFT)
                        8'h21: begin dec_row=4'd3; dec_col=3'd3; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // C (PARIGHT)
                        8'h23: begin dec_row=4'd2; dec_col=3'd3; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // D (COMMA)
                        8'h24: begin dec_row=4'd1; dec_col=3'd3; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // E (DIVIDE)
                        8'h2B: begin dec_row=4'd5; dec_col=3'd4; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // F (PIPE)
                        8'h34: begin dec_row=4'd4; dec_col=3'd4; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // G (7)
                        8'h33: begin dec_row=4'd3; dec_col=3'd4; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // H (8)
                        8'h43: begin dec_row=4'd2; dec_col=3'd4; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // I (9)
                        8'h3B: begin dec_row=4'd1; dec_col=3'd4; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // J (MULT)
                        8'h42: begin dec_row=4'd5; dec_col=3'd5; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // K (EE)
                        8'h4B: begin dec_row=4'd4; dec_col=3'd5; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // L (4)
                        8'h3A: begin dec_row=4'd3; dec_col=3'd5; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // M (5)
                        8'h31: begin dec_row=4'd2; dec_col=3'd5; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // N (6)
                        8'h44: begin dec_row=4'd1; dec_col=3'd5; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // O (MINUS)
                        8'h4D: begin dec_row=4'd5; dec_col=3'd6; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // P (STORE)
                        8'h15: begin dec_row=4'd4; dec_col=3'd6; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // Q (1)
                        8'h2D: begin dec_row=4'd3; dec_col=3'd6; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // R (2)
                        8'h1B: begin dec_row=4'd2; dec_col=3'd6; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // S (3)
                        8'h3C: begin dec_row=4'd1; dec_col=3'd6; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // U (PLUS)
                        8'h2A: begin dec_row=4'd4; dec_col=3'd7; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // V (0)
                        8'h1D: begin dec_row=4'd3; dec_col=3'd7; dec_valid=1'b1; legacy_is_alpha=1'b1; dec_req = pc_shift_down ? 4'b0100 : 4'b1000; end // W (PERIOD)

                        // Numbers Row (0-9) & Shifted Number Symbols
                        8'h16: begin dec_valid=1'b1; if (pc_shift_down && native_mode) begin dec_row=4'd3; dec_col=3'd7; dec_req=4'b0010; end else begin dec_row=4'd4; dec_col=3'd6; dec_req=4'd0; end end // 1 / !
                        8'h1E: begin dec_valid=1'b1; dec_row=4'd3; dec_col=3'd6; dec_req=4'd0; end // 2
                        8'h26: begin dec_valid=1'b1; dec_row=4'd2; dec_col=3'd6; dec_req=4'd0; end // 3
                        8'h25: begin dec_valid=1'b1; dec_row=4'd4; dec_col=3'd5; dec_req=4'd0; end // 4
                        8'h2E: begin dec_valid=1'b1; dec_row=4'd3; dec_col=3'd5; dec_req=4'd0; end // 5
                        8'h36: begin dec_valid=1'b1; if (pc_shift_down && native_mode) begin dec_row=4'd1; dec_col=3'd2; dec_req=4'b0000; end else begin dec_row=4'd2; dec_col=3'd5; dec_req=4'd0; end end // 6 / ^
                        8'h3D: begin dec_valid=1'b1; dec_row=4'd4; dec_col=3'd4; dec_req=4'd0; end // 7
                        8'h3E: begin dec_valid=1'b1; if (pc_shift_down && native_mode) begin dec_row=4'd1; dec_col=3'd4; dec_req=4'b0000; end else begin dec_row=4'd3; dec_col=3'd4; dec_req=4'd0; end end // 8 / *
                        8'h46: begin dec_valid=1'b1; if (pc_shift_down && native_mode) begin dec_row=4'd4; dec_col=3'd3; dec_req=4'b0000; end else begin dec_row=4'd2; dec_col=3'd4; dec_req=4'd0; end end // 9 / (
                        8'h45: begin dec_valid=1'b1; if (pc_shift_down && native_mode) begin dec_row=4'd3; dec_col=3'd3; dec_req=4'b0000; end else begin dec_row=4'd4; dec_col=3'd7; dec_req=4'd0; end end // 0 / )

                        // ==============================================================
                        // CUSTOM LOGICAL OVERRIDES - Fix mapping discrepancies
                        // ==============================================================
                        
                        // Spacebar: Unshifted -> Space (Alpha + Negate). Shifted -> '-' (Negate)
                        8'h29: begin 
                            dec_valid = 1'b1; 
                            if (native_mode) begin
                                if (pc_shift_down) begin dec_row=4'd2; dec_col=3'd7; dec_req=4'b0000; end // NEGATE
                                else               begin dec_row=4'd2; dec_col=3'd7; dec_req=4'b1000; end // ALPHA + NEGATE
                            end else begin dec_row=4'd2; dec_col=3'd7; end
                        end

                        // Comma (,) / Less Than (<)
                        8'h41: begin 
                            dec_valid = 1'b1; 
                            if (native_mode) begin
                                if (pc_shift_down) begin dec_row=4'd4; dec_col=3'd7; dec_req=4'b0010; end // 2nd + 0 = <
                                else               begin dec_row=4'd2; dec_col=3'd3; dec_req=4'b0000; end // COMMA
                            end else begin dec_row=4'd2; dec_col=3'd3; end 
                        end
                        
                        // Period (.) / Greater Than (>)
                        8'h49: begin 
                            dec_valid = 1'b1; 
                            if (native_mode) begin
                                if (pc_shift_down) begin dec_row=4'd3; dec_col=3'd7; dec_req=4'b0010; end // 2nd + . = >
                                else               begin dec_row=4'd3; dec_col=3'd7; dec_req=4'b0000; end // PERIOD
                            end else begin dec_row=4'd3; dec_col=3'd7; end 
                        end

                        // LBracket ([) / Angle (∠)
                        8'h54: begin 
                            dec_valid = 1'b1; 
                            if (native_mode) begin
                                if (pc_shift_down) begin dec_row=4'd5; dec_col=3'd5; dec_req=4'b0010; end // 2nd + EE = ∠
                                else               begin dec_row=4'd2; dec_col=3'd3; dec_req=4'b0010; end // 2nd + COMMA = [
                            end else begin dec_row=4'd4; dec_col=3'd3; end 
                        end

                        // RBracket (]) / Pi (π)
                        8'h5B: begin 
                            dec_valid = 1'b1; 
                            if (native_mode) begin
                                if (pc_shift_down) begin dec_row=4'd1; dec_col=3'd2; dec_req=4'b0010; end // 2nd + POWER = π
                                else               begin dec_row=4'd1; dec_col=3'd3; dec_req=4'b0010; end // 2nd + DIVIDE = ]
                            end else begin dec_row=4'd3; dec_col=3'd3; end 
                        end

                        // Other Symbols & Punctuation
                        // PC Minus (-): Unshifted -> NEGATE '(-)', Shifted -> MINUS operator '-'
                        8'h4E: begin 
                            dec_valid = 1'b1; 
                            if (native_mode) begin
                                if (pc_shift_down) begin dec_row=4'd1; dec_col=3'd5; dec_req=4'b0000; end // MINUS Operator
                                else               begin dec_row=4'd2; dec_col=3'd7; dec_req=4'b0000; end // NEGATE
                            end else begin dec_row=4'd1; dec_col=3'd5; end 
                        end
                        8'h55: begin dec_valid=1'b1; if (pc_shift_down && native_mode) begin dec_row=4'd1; dec_col=3'd6; dec_req=4'b0000; end else begin dec_row=4'd5; dec_col=3'd3; dec_req=4'd0; end end // = / +
                        8'h5D: begin dec_valid=1'b1; dec_row=4'd5; dec_col=3'd4; dec_req = 4'b0000; end // \ / |
                        8'h4C: begin dec_valid=1'b1; dec_row=4'd2; dec_col=3'd4; dec_req = (native_mode ? 4'b0010 : 4'd0); end // ; / :
                        8'h52: begin dec_valid=1'b1; dec_row=4'd1; dec_col=3'd6; dec_req = (native_mode ? 4'b0010 : 4'd0); end // ' / "
                        8'h4A: begin dec_valid=1'b1; dec_row=4'd1; dec_col=3'd3; dec_req = 4'b0000; end // / / ?

                        // Special Keys
                        8'h76: begin dec_row = 4'd6; dec_col = 3'd7; dec_valid = 1'b1; end // ESC -> ESCAPE
                        8'h66: begin dec_row = 4'd2; dec_col = 3'd1; dec_valid = 1'b1; end // Backspace
                        8'h5A: begin dec_row = 4'd1; dec_col = 3'd7; dec_valid = 1'b1; end // Enter
                        8'h0D: begin dec_row = 4'd5; dec_col = 3'd6; dec_valid = 1'b1; end // Tab -> STORE
                        8'h0E: begin dec_row = 4'd1; dec_col = 3'd2; dec_valid = 1'b1; end // ` backtick -> POWER

                        // Numpad (non-extended scancodes = Num Lock on; with Num
                        // Lock off these arrive E0-prefixed and land in the
                        // extended table as cursor/navigation keys instead)
                        8'h70: begin dec_row=4'd4; dec_col=3'd7; dec_valid=1'b1; dec_req=4'd0; end // NP 0
                        8'h69: begin dec_row=4'd4; dec_col=3'd6; dec_valid=1'b1; dec_req=4'd0; end // NP 1
                        8'h72: begin dec_row=4'd3; dec_col=3'd6; dec_valid=1'b1; dec_req=4'd0; end // NP 2
                        8'h7A: begin dec_row=4'd2; dec_col=3'd6; dec_valid=1'b1; dec_req=4'd0; end // NP 3
                        8'h6B: begin dec_row=4'd4; dec_col=3'd5; dec_valid=1'b1; dec_req=4'd0; end // NP 4
                        8'h73: begin dec_row=4'd3; dec_col=3'd5; dec_valid=1'b1; dec_req=4'd0; end // NP 5
                        8'h74: begin dec_row=4'd2; dec_col=3'd5; dec_valid=1'b1; dec_req=4'd0; end // NP 6
                        8'h6C: begin dec_row=4'd4; dec_col=3'd4; dec_valid=1'b1; dec_req=4'd0; end // NP 7
                        8'h75: begin dec_row=4'd3; dec_col=3'd4; dec_valid=1'b1; dec_req=4'd0; end // NP 8
                        8'h7D: begin dec_row=4'd2; dec_col=3'd4; dec_valid=1'b1; dec_req=4'd0; end // NP 9
                        8'h71: begin dec_row=4'd3; dec_col=3'd7; dec_valid=1'b1; dec_req=4'd0; end // NP . -> PERIOD
                        8'h79: begin dec_row=4'd1; dec_col=3'd6; dec_valid=1'b1; dec_req=4'd0; end // NP + -> PLUS
                        8'h7B: begin dec_row=4'd1; dec_col=3'd5; dec_valid=1'b1; dec_req=4'd0; end // NP - -> MINUS
                        8'h7C: begin dec_row=4'd1; dec_col=3'd4; dec_valid=1'b1; dec_req=4'd0; end // NP * -> MULTIPLY

                        // Function Keys
                        8'h05: begin dec_row = 4'd5; dec_col = 3'd0; dec_valid = 1'b1; end // F1
                        8'h06: begin dec_row = 4'd4; dec_col = 3'd0; dec_valid = 1'b1; end // F2
                        8'h04: begin dec_row = 4'd3; dec_col = 3'd0; dec_valid = 1'b1; end // F3
                        8'h0C: begin dec_row = 4'd2; dec_col = 3'd0; dec_valid = 1'b1; end // F4
                        8'h03: begin dec_row = 4'd1; dec_col = 3'd0; dec_valid = 1'b1; end // F5
                        8'h0B: begin dec_row = 4'd3; dec_col = 3'd1; dec_valid = 1'b1; end // F6 -> CATALOG
                        8'h83: begin dec_row = 4'd5; dec_col = 3'd1; dec_valid = 1'b1; end // F7 -> HOME
                        8'h0A: begin dec_row = 4'd4; dec_col = 3'd1; dec_valid = 1'b1; end // F8 -> MODE
                        default: dec_valid = 1'b0;
                    endcase
                end else if (ps2_ext && np_digits && !dec_is_mod_override) begin
                    // ==========================================================
                    // EXTENDED SCANCODES - NUMPAD DIGITS MODE
                    // With Num Lock off the numpad emits the cursor/navigation
                    // codes below. Decode them as keypad digits 1-9 instead.
                    // 0x70 (Insert/NP 0) stays the ON key and 0x71 (Delete/NP .)
                    // stays CLEAR - they have no duplicate elsewhere, while
                    // digits do (top number row).
                    // ==========================================================
                    case (ps2_code)
                        8'h69: begin dec_row = 4'd4; dec_col = 3'd6; dec_valid = 1'b1; dec_req = 4'd0; end // NP 1
                        8'h72: begin dec_row = 4'd3; dec_col = 3'd6; dec_valid = 1'b1; dec_req = 4'd0; end // NP 2
                        8'h7A: begin dec_row = 4'd2; dec_col = 3'd6; dec_valid = 1'b1; dec_req = 4'd0; end // NP 3
                        8'h6B: begin dec_row = 4'd4; dec_col = 3'd5; dec_valid = 1'b1; dec_req = 4'd0; end // NP 4
                        8'h73: begin dec_row = 4'd3; dec_col = 3'd5; dec_valid = 1'b1; dec_req = 4'd0; end // NP 5
                        8'h74: begin dec_row = 4'd2; dec_col = 3'd5; dec_valid = 1'b1; dec_req = 4'd0; end // NP 6
                        8'h6C: begin dec_row = 4'd4; dec_col = 3'd4; dec_valid = 1'b1; dec_req = 4'd0; end // NP 7
                        8'h75: begin dec_row = 4'd3; dec_col = 3'd4; dec_valid = 1'b1; dec_req = 4'd0; end // NP 8
                        8'h7D: begin dec_row = 4'd2; dec_col = 3'd4; dec_valid = 1'b1; dec_req = 4'd0; end // NP 9
                        8'h70: dec_is_on = 1'b1;                                                          // ON key
                        8'h71: begin dec_row = 4'd1; dec_col = 3'd1; dec_valid = 1'b1; end                // CLEAR
                        8'h4A: begin dec_row = 4'd1; dec_col = 3'd3; dec_valid = 1'b1; end                // NP / -> DIVIDE
                        8'h5A: begin dec_row = 4'd1; dec_col = 3'd7; dec_valid = 1'b1; end                // NP Enter -> ENTER
                        default: dec_valid = 1'b0;
                    endcase
                end else if (ps2_ext && !dec_is_mod_override) begin
                    // ==============================================================
                    // EXTENDED SCANCODES (ps2_ext = 1)
                    // ==============================================================
                    case (ps2_code)
                        8'h6B: begin dec_row = 4'd0; dec_col = 3'd6; dec_valid = 1'b1; end // Left
                        8'h72: begin dec_row = 4'd0; dec_col = 3'd5; dec_valid = 1'b1; end // Down
                        8'h74: begin dec_row = 4'd0; dec_col = 3'd4; dec_valid = 1'b1; end // Right
                        8'h75: begin dec_row = 4'd0; dec_col = 3'd7; dec_valid = 1'b1; end // Up
                        8'h71: begin dec_row = 4'd1; dec_col = 3'd1; dec_valid = 1'b1; end // Delete -> CLEAR
                        8'h70: dec_is_on = 1'b1;                                           // Insert -> ON key
                        8'h6C: begin dec_row = 4'd5; dec_col = 3'd1; dec_valid = 1'b1; end // Home -> HOME
                        8'h7D: begin dec_row = 4'd5; dec_col = 3'd7; dec_valid = 1'b1; end // Page Up -> APPS
                        8'h7A: begin dec_row = 4'd5; dec_col = 3'd5; dec_valid = 1'b1; end // Page Down -> EE
                        8'h69: begin dec_row = 4'd3; dec_col = 3'd1; dec_valid = 1'b1; end // End -> CATALOG
                        8'h4A: begin dec_row = 4'd1; dec_col = 3'd3; dec_valid = 1'b1; end // Numpad / -> DIVIDE
                        8'h5A: begin dec_row = 4'd1; dec_col = 3'd7; dec_valid = 1'b1; end // Numpad Enter -> ENTER
                        default: dec_valid = 1'b0;
                    endcase
                end

                // ==============================================================
                // KEY TRACKER (Fixes Shift-Release Desync)
                // ==============================================================
                if (dec_valid && !dec_is_mod_override) begin
                    if (ps2_pressed) begin
                        key_tracker[{ps2_ext, ps2_code}] <= {dec_req, dec_row, dec_col};
                    end else begin
                        dec_req = key_tracker[{ps2_ext, ps2_code}][10:7];
                        dec_row = key_tracker[{ps2_ext, ps2_code}][6:3];
                        dec_col = key_tracker[{ps2_ext, ps2_code}][2:0];
                    end
                end

                // ==============================================================
                // MATRIX UPDATE & FIFO DISPATCH (Auto-Repeat Filtered)
                // ==============================================================
                if (native_mode) begin
                    // Native Mode: filter duplicates/auto-repeat against the
                    // last state WE decided to act on for this scancode
                    // (key_native_pending), NOT against key_matrix -- the
                    // sequencer can take many clock cycles to actually
                    // commit a press, so comparing against key_matrix would
                    // wrongly drop a fast release (or an overlapping NKRO
                    // key) as a "duplicate", leaving it stuck asserted.
                    if (dec_valid && !dec_is_mod_override &&
                        (key_native_pending[{ps2_ext, ps2_code}] != ps2_pressed)) begin
                        key_native_pending[{ps2_ext, ps2_code}] <= ps2_pressed;
                        // All non-override keys in Native Mode go through FIFO to guarantee sequence
                        if (!fifo_full) begin
                            fifo_mem[fifo_wr_ptr] <= {ps2_pressed, dec_row, dec_col, dec_req, 2'd0}; // Padded to 14 bits
                            fifo_wr_ptr <= fifo_wr_ptr + 1'b1;
                            fifo_count  <= fifo_count + 1'b1;
                        end
                    end
                end else begin
                    // Legacy Mode: direct, same-cycle update, so key_matrix
                    // is always current -- no commit-latency race here.
                    if (dec_valid && (key_matrix[dec_row][3'd7 - dec_col] != ps2_pressed) && !dec_is_mod_override) begin
                        key_matrix[dec_row][3'd7 - dec_col] <= ps2_pressed;
                        if (ps2_pressed) kbd_int <= 1'b1;

                        if (legacy_is_alpha) begin
                            if (ps2_pressed) begin
                                key_matrix[0][7] <= 1'b1;
                                alpha_count      <= alpha_count + 1;
                                kbd_int          <= 1'b1;
                            end else begin
                                if (alpha_count != 5'd0) alpha_count <= alpha_count - 1;
                                if (alpha_count <= 5'd1) key_matrix[0][7] <= 1'b0;
                            end
                        end
                    end
                end

                if (dec_is_on) begin
                    on_key <= ps2_pressed;
                    if (ps2_pressed) on_key_press <= 1'b1;
                end
            end

            // =================================================================
            // Native Mode: Sequencer State Machine (Handles Multi-Modifiers)
            // =================================================================
            case (nm_state)
                ST_IDLE: begin
                    if (!fifo_empty) begin
                        fifo_rd_ptr <= fifo_rd_ptr + 1'b1;
                        fifo_count  <= fifo_count - 1'b1;
                        nm_row      <= fifo_row;
                        nm_col      <= fifo_col;
                        nm_req      <= fifo_req;

                        if (fifo_press) begin
                            reg [4:0] next_alpha, next_shift, next_2nd, next_diamond;
                            reg [3:0] newly_needed;

                            next_alpha      = nm_alpha_count;
                            next_shift      = nm_shift_count;
                            next_2nd        = nm_2nd_count;
                            next_diamond    = nm_diamond_count;

                            // Increment counters for all requested modifiers
                            if (fifo_req[3]) next_alpha   = next_alpha + 1'b1;
                            if (fifo_req[2]) next_shift   = next_shift + 1'b1;
                            if (fifo_req[1]) next_2nd     = next_2nd + 1'b1;
                            if (fifo_req[0]) next_diamond = next_diamond + 1'b1;

                            nm_alpha_count   <= next_alpha;
                            nm_shift_count   <= next_shift;
                            nm_2nd_count     <= next_2nd;
                            nm_diamond_count <= next_diamond;

                            // Modifiers that are newly pressed (0->1) for this key
                            newly_needed = { (fifo_req[3] && nm_alpha_count   == 5'd0),
                                             (fifo_req[2] && nm_shift_count   == 5'd0),
                                             (fifo_req[1] && nm_2nd_count     == 5'd0),
                                             (fifo_req[0] && nm_diamond_count == 5'd0) };

                            if (newly_needed != 4'b0000) begin
                                // Assert them ONE AT A TIME (never atomically together)
                                nm_pending <= newly_needed;
                                nm_state   <= ST_MOD_ASSERT;
                            end else begin
                                nm_state <= ST_KEY_ASSERT;
                            end
                        end else begin
                            // Decrement required modifiers safely
                            if (fifo_req[3] && nm_alpha_count != 5'd0)   nm_alpha_count   <= nm_alpha_count - 1'b1;
                            if (fifo_req[2] && nm_shift_count != 5'd0)   nm_shift_count   <= nm_shift_count - 1'b1;
                            if (fifo_req[1] && nm_2nd_count != 5'd0)     nm_2nd_count     <= nm_2nd_count - 1'b1;
                            if (fifo_req[0] && nm_diamond_count != 5'd0) nm_diamond_count <= nm_diamond_count - 1'b1;
                            nm_state <= ST_KEY_RELEASE;
                        end
                    end
                end

                // Assert the single highest-priority pending modifier bit
                // (ALPHA -> SHIFT -> 2ND -> DIAMOND), then settle-wait before
                // either asserting the next pending bit or pressing the key.
                ST_MOD_ASSERT: begin
                    if (nm_pending[3])      begin key_matrix[0][7] <= 1'b1; nm_pending[3] <= 1'b0; end // ALPHA
                    else if (nm_pending[2]) begin key_matrix[0][5] <= 1'b1; nm_pending[2] <= 1'b0; end // SHIFT
                    else if (nm_pending[1]) begin key_matrix[0][4] <= 1'b1; nm_pending[1] <= 1'b0; end // 2ND
                    else if (nm_pending[0]) begin key_matrix[0][6] <= 1'b1; nm_pending[0] <= 1'b0; end // DIAMOND
                    kbd_int  <= 1'b1;
                    nm_timer <= 18'd0;
                    nm_state <= ST_MOD_WAIT;
                end

                ST_MOD_WAIT: begin
                    if (nm_timer == TIMER_MAX) begin
                        if (nm_pending != 4'b0000) nm_state <= ST_MOD_ASSERT; // more bits to assert
                        else nm_state <= ST_KEY_ASSERT;
                    end else nm_timer <= nm_timer + 1'b1;
                end

                ST_KEY_ASSERT: begin
                    key_matrix[nm_row][3'd7 - nm_col] <= 1'b1;
                    kbd_int  <= 1'b1;
                    nm_state <= ST_IDLE;
                end

                ST_KEY_RELEASE: begin
                    reg [3:0] release_pending;

                    key_matrix[nm_row][3'd7 - nm_col] <= 1'b0;
                    kbd_int <= 1'b1;

                    // Modifiers this key used whose ref count just hit zero
                    release_pending = { (nm_req[3] && nm_alpha_count   == 5'd0),
                                        (nm_req[2] && nm_shift_count   == 5'd0),
                                        (nm_req[1] && nm_2nd_count     == 5'd0),
                                        (nm_req[0] && nm_diamond_count == 5'd0) };

                    if (release_pending != 4'b0000) begin
                        nm_pending <= release_pending;
                        nm_timer   <= 18'd0;
                        nm_state   <= ST_REL_WAIT;
                    end else begin
                        nm_state <= ST_IDLE;
                    end
                end

                ST_REL_WAIT: begin
                    if (nm_timer == TIMER_MAX) nm_state <= ST_MOD_RELEASE;
                    else nm_timer <= nm_timer + 1'b1;
                end

                // Release the single lowest-priority pending modifier bit
                // (DIAMOND -> 2ND -> SHIFT -> ALPHA, the reverse of assert
                // order), then settle-wait before releasing the next pending
                // bit, or return to idle once none remain.
                ST_MOD_RELEASE: begin
                    reg [3:0] remaining;
                    remaining = nm_pending;

                    if (remaining[0])      begin key_matrix[0][6] <= 1'b0; remaining[0] = 1'b0; end // DIAMOND
                    else if (remaining[1]) begin key_matrix[0][4] <= 1'b0; remaining[1] = 1'b0; end // 2ND
                    else if (remaining[2]) begin key_matrix[0][5] <= 1'b0; remaining[2] = 1'b0; end // SHIFT
                    else if (remaining[3]) begin key_matrix[0][7] <= 1'b0; remaining[3] = 1'b0; end // ALPHA

                    nm_pending <= remaining;
                    kbd_int    <= 1'b1;

                    if (remaining != 4'b0000) begin
                        nm_timer <= 18'd0;
                        nm_state <= ST_REL_WAIT;
                    end else begin
                        nm_state <= ST_IDLE;
                    end
                end

                default: nm_state <= ST_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Column readback logic (Active Low Output)
    // =========================================================================
    reg [7:0] col_or;
    integer r;
    always @(*) begin
        col_or = 8'h00;
        for (r = 0; r < 10; r = r + 1) begin
            if (!row_mask[r]) col_or = col_or | key_matrix[r];
        end
    end
    assign col_data = ~col_or;

endmodule